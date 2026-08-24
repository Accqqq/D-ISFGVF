#include "bspline_race/integration/phase_offset_matched_adapter.h"
#include "bspline_race/integration/phase_offset_tube_epoch_diagnostics.h"
#include "bspline_race/integration/phase_offset_tube_markers.h"
#include <bspline_race/continuous_phase_normal_frame.h>
#include <bspline_race/integration/phase_offset_executed_reference_query.h>
#include <bspline_race/integration/phase_offset_recovery_continuation_provider.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <thread>
#include <utility>

namespace FLAG_Race {

struct PathTubePairPinRegistry {
  std::mutex mutex;
  std::uint64_t next_lease_id = 0U;
  std::uint64_t active_lease_id = 0U;
  std::shared_ptr<const PathTubePair> active_pair;
  std::uint64_t active_generation = 0U;
  std::uint64_t active_session = 0U;
};

namespace {
// A replan callback may produce two adjacent adapter updates before a
// subscriber drains its queue.  Keep the diagnostic and both tube-marker
// streams together so a recorder/RViz never receives only the later DELETE
// for an already published certificate frame.  This is transport buffering
// only; it does not change geometry, certification, or control selection.
constexpr std::uint32_t kManualEvidenceQueueSize = 10U;
constexpr double kRecoveryNonzeroDeltaTolerance = 1e-3;

bool IsFinite(double value) { return std::isfinite(value); }
bool IsFinite(const Eigen::Vector3d& value) { return value.allFinite(); }
double FiniteOrZero(double value) { return IsFinite(value) ? value : 0.0; }

bool HasNonzeroRawBuildCapacity(
    const phase_offset_navigation::TubeProfile& profile) {
  constexpr double kCapacityTolerance = 1e-10;
  for (const phase_offset_navigation::TubeRawSample& sample :
       profile.raw_build_samples) {
    if (!IsFinite(sample.raw_lower) || !IsFinite(sample.raw_upper)) continue;
    if (sample.raw_lower < -kCapacityTolerance ||
        sample.raw_upper > kCapacityTolerance) {
      return true;
    }
  }
  return false;
}

void LogZeroBaselineAttribution(
    const phase_offset_navigation::TubeEpochUpdateResult& result) {
  if (!result.status.candidate_complete ||
      result.status.candidate_classification !=
          phase_offset_navigation::TubeProfileClassification::
              ZERO_ONLY_PLANNER_BASELINE ||
      !HasNonzeroRawBuildCapacity(result.candidate_profile)) {
    return;
  }
  const phase_offset_navigation::TubeStopReason first_stop_reason =
      static_cast<phase_offset_navigation::TubeStopReason>(
          result.candidate_profile.diagnostics.first_stop_reason);
  ROS_WARN(
      "[PHASE_OFFSET][TUBE][ZERO_BASELINE] candidate_seq=%llu "
      "map_seq=%llu obstacle_certified=%d first_stop_reason=%s "
      "first_stop_w=%.9f invalid_reason=\"%s\"",
      static_cast<unsigned long long>(result.status.candidate_sequence),
      static_cast<unsigned long long>(
          result.status.candidate_map_observation_sequence),
      result.candidate_profile.obstacle_certified ? 1 : 0,
      phase_offset_navigation::tubeStopReasonName(first_stop_reason),
      FiniteOrZero(result.candidate_profile.diagnostics.first_invalid_w),
      result.candidate_profile.diagnostics.invalid_reason.c_str());
}

bool IsOffsetCertifiedProfile(
    const std::shared_ptr<const phase_offset_navigation::TubeProfile>& profile) {
  return profile && profile->classification ==
      phase_offset_navigation::TubeProfileClassification::OFFSET_CERTIFIED;
}

bool FrameMatchesRevision(
    const std::shared_ptr<const ContinuousPhaseNormalFrame>& frame,
    const std::uint64_t revision) {
  return frame && revision != 0U && frame->pathRevision() == revision &&
      frame->frameRevision() != 0U;
}

constexpr double kPreparedCoverageTolerance = 1e-10;
// Must remain aligned with TubeBuilder's frozen current-anchor contract.  It
// is used only to prove that this adapter-side partition contains exactly one
// owner-evaluated representation of the original current phase; it does not
// change any Tube acceptance tolerance.
constexpr double kTubeCurrentPhaseAnchorTolerance = 1e-9;

// This spelling is intentionally adapter-local.  The manager owns the
// ROS-facing bootstrap result log; stagePathTubePair only carries its exact
// temporary enum to that caller and never publishes a second diagnostic.
const char* PathTubePairStageFailureName(
    const PathTubePairStageFailure failure) {
  switch (failure) {
    case PathTubePairStageFailure::NONE:
      return "none";
    case PathTubePairStageFailure::INPUT_PRECONDITION:
      return "input_precondition";
    case PathTubePairStageFailure::TRANSACTION_PRECONDITION:
      return "transaction_precondition";
    case PathTubePairStageFailure::PAIR_SESSION_RUNTIME_SNAPSHOT:
      return "pair_session_runtime_snapshot";
    case PathTubePairStageFailure::OWNER_EVALUATE:
      return "owner_evaluate";
    case PathTubePairStageFailure::TUBE_BUILD_PRECONDITION:
      return "tube_build_precondition";
    case PathTubePairStageFailure::TUBE_RAW_BUILD:
      return "tube_raw_build";
    case PathTubePairStageFailure::TUBE_FILTER:
      return "tube_filter";
    case PathTubePairStageFailure::TUBE_SURFACE_VALIDATOR:
      return "tube_surface_validator";
    case PathTubePairStageFailure::TUBE_PROFILE_COVERAGE:
      return "tube_profile_coverage";
    case PathTubePairStageFailure::TUBE_PROFILE_OWNER_MATCH:
      return "tube_profile_owner_match";
    case PathTubePairStageFailure::STAGING_DRY_RUN:
      return "staging_dry_run";
  }
  return "unknown";
}

bool NearlyEqual(const double first, const double second,
                 const double tolerance = kPreparedCoverageTolerance) {
  return IsFinite(first) && IsFinite(second) &&
      std::abs(first - second) <= tolerance;
}

bool StateFiniteAndValid(const phase_offset_core::PathDifferentialState& state) {
  return state.valid && IsFinite(state.w) && IsFinite(state.p) &&
      IsFinite(state.p_w) && IsFinite(state.p_ww);
}

bool ExactDoubleBits(const double first, const double second) {
  return std::memcmp(&first, &second, sizeof(first)) == 0;
}

bool PathStatesEquivalent(
    const phase_offset_core::PathDifferentialState& first,
    const phase_offset_core::PathDifferentialState& second) {
  return StateFiniteAndValid(first) && StateFiniteAndValid(second) &&
      ExactDoubleBits(first.w, second.w) &&
      (first.p - second.p).norm() <= kPreparedCoverageTolerance &&
      (first.p_w - second.p_w).norm() <= kPreparedCoverageTolerance &&
      (first.p_ww - second.p_ww).norm() <= kPreparedCoverageTolerance;
}

bool PathStateMatchesOwner(
    const phase_offset_core::PathDifferentialState& state,
    const std::shared_ptr<const ContinuousPhasePath>& owner,
    const std::shared_ptr<const ContinuousPhaseNormalFrame>& shared_frame =
        std::shared_ptr<const ContinuousPhaseNormalFrame>()) {
  if (!StateFiniteAndValid(state) || !owner || owner->empty()) return false;
  ContinuousPhasePathState evaluated;
  // A missing shared frame is accepted only for legacy synthetic fixtures;
  // production owner/profile checks pass the handoff frame explicitly.
  const bool evaluated_ok = shared_frame
      ? shared_frame->evaluatePathState(state.w, evaluated)
      : (owner->pathRevision() == 0U
          ? owner->evaluate(state.w, evaluated, false)
          : ContinuousPhaseNormalFrame(
                owner, owner->pathRevision(), owner->pathRevision())
                .evaluatePathState(state.w, evaluated));
  return evaluated_ok && evaluated.valid &&
      IsFinite(evaluated.p) && IsFinite(evaluated.dp_dw) &&
      IsFinite(evaluated.d2p_dw2) &&
      (evaluated.p - state.p).norm() <= kPreparedCoverageTolerance &&
      (evaluated.dp_dw - state.p_w).norm() <= kPreparedCoverageTolerance &&
      (evaluated.d2p_dw2 - state.p_ww).norm() <= kPreparedCoverageTolerance;
}

bool EvaluateOwnerState(
    const std::shared_ptr<const ContinuousPhasePath>& owner,
    const double w,
    phase_offset_core::PathDifferentialState& state,
    const std::shared_ptr<const ContinuousPhaseNormalFrame>& shared_frame =
        std::shared_ptr<const ContinuousPhaseNormalFrame>()) {
  if (!owner || owner->empty() || !IsFinite(w)) return false;
  ContinuousPhasePathState evaluated;
  if (shared_frame) {
    if (!shared_frame->evaluatePathState(w, evaluated)) return false;
  } else if (owner->pathRevision() == 0U) {
    if (!owner->evaluate(w, evaluated, false)) return false;
  } else {
    const ContinuousPhaseNormalFrame frame(
        owner, owner->pathRevision(), owner->pathRevision());
    if (!frame.evaluatePathState(w, evaluated)) return false;
  }
  state = ConvertContinuousPhasePathStateForActive(evaluated, w);
  return StateFiniteAndValid(state);
}

bool CaptureMatchesPair(const PathTubePairPinCapture& capture,
                        const std::shared_ptr<const PathTubePair>& pair) {
  return pair && capture.valid() && capture.pair == pair &&
      capture.source_revision == pair->source_revision &&
      capture.generation == pair->generation &&
      capture.authority_session == pair->authority_session &&
      capture.map_observation_sequence == pair->map_observation_sequence &&
      capture.map_observation_is_snapshot == pair->map_observation_is_snapshot &&
      capture.path_owner == pair->path_owner &&
      capture.frame_owner == pair->frame_owner &&
      capture.frozen_cloud_occupancy_snapshot ==
          pair->frozen_cloud_occupancy_snapshot &&
      capture.full_path_samples == pair->full_path_samples &&
      capture.active_profile == pair->active_profile &&
      capture.epoch_snapshot == pair->epoch_snapshot;
}

bool ActiveLeaseMatchesCapture(
    const std::shared_ptr<PathTubePairPinRegistry>& registry,
    const PathTubePairPinCapture& capture, const std::uint64_t lease_id) {
  if (!registry || lease_id == 0U || !capture.valid()) return false;
  std::lock_guard<std::mutex> lock(registry->mutex);
  return registry->active_lease_id == lease_id &&
      registry->active_pair == capture.pair &&
      registry->active_generation == capture.generation &&
      registry->active_session == capture.authority_session;
}

phase_offset_navigation::RuntimeFutureStepContract
MakeFutureStepContract(
    const std::shared_ptr<const ContinuousPhasePath>& owner,
    const std::shared_ptr<const MatchedAdapterPathSamples>& samples,
    const std::shared_ptr<const phase_offset_navigation::TubeProfile>& profile,
    const guidance::IsfGains& gains,
    const double regularity_margin,
    const double minimum_reference_speed,
    const double tube_update_period,
    const double min_certified_forward_w) {
  phase_offset_navigation::RuntimeFutureStepContract contract;
  if ((!owner || owner->empty()) && (!samples || samples->size() < 2U)) {
    return contract;
  }
  if (!profile || !profile->complete ||
      profile->samples.size() < 2U || !std::isfinite(tube_update_period) ||
      tube_update_period <= 0.0 || !std::isfinite(min_certified_forward_w) ||
      min_certified_forward_w < 0.0 || !std::isfinite(regularity_margin) ||
      regularity_margin <= 0.0 || regularity_margin >= 1.0 ||
      !std::isfinite(minimum_reference_speed) ||
      minimum_reference_speed <= 0.0) {
    return contract;
  }
  contract.tube_update_period = tube_update_period;
  contract.min_certified_forward_w = min_certified_forward_w;
  const double certified_start = profile->certified_segment_start_w;
  const double certified_end = profile->certified_segment_end_w;
  if (!std::isfinite(certified_start) || !std::isfinite(certified_end) ||
      certified_end < certified_start ||
      certified_start < profile->samples.front().w -
          kPreparedCoverageTolerance ||
      certified_end > profile->samples.back().w +
          kPreparedCoverageTolerance ||
      certified_end - certified_start <= kPreparedCoverageTolerance) {
    return phase_offset_navigation::RuntimeFutureStepContract();
  }
  contract.profile_domain_end_w = certified_end;
  // Capture only immutable owner/profile/configuration values.  Each rollout
  // step reevaluates its own owner state and uses the unchanged live ISF
  // kernel/gains; no frozen current geometry or base command is reused.
  phase_offset_navigation::PathStateQuery state_query;
  if (owner && !owner->empty()) {
    state_query = [owner](
        const double w,
        phase_offset_core::PathDifferentialState& state) {
      return EvaluateOwnerState(owner, w, state);
    };
  } else {
    state_query = [samples](
        const double w,
        phase_offset_core::PathDifferentialState& state) {
      if (!samples || samples->empty() || !std::isfinite(w) ||
          w < samples->front().w - kPreparedCoverageTolerance ||
          w > samples->back().w + kPreparedCoverageTolerance) {
        return false;
      }
      for (std::size_t index = 1U; index < samples->size(); ++index) {
        const auto& left = (*samples)[index - 1U];
        const auto& right = (*samples)[index];
        if (w > right.w + kPreparedCoverageTolerance) continue;
        const double span = right.w - left.w;
        if (!StateFiniteAndValid(left) || !StateFiniteAndValid(right) ||
            span <= kPreparedCoverageTolerance) {
          return false;
        }
        const double alpha = std::max(
            0.0, std::min(1.0, (w - left.w) / span));
        state.w = w;
        state.p = left.p + alpha * (right.p - left.p);
        state.p_w = left.p_w + alpha * (right.p_w - left.p_w);
        state.p_ww = left.p_ww + alpha * (right.p_ww - left.p_ww);
        state.valid = left.valid && right.valid;
        return StateFiniteAndValid(state);
      }
      state = samples->back();
      state.w = w;
      return StateFiniteAndValid(state);
    };
  }
  contract.evaluate = [state_query, gains, regularity_margin,
                       minimum_reference_speed](
      const phase_offset_navigation::RuntimeFutureStepInput& input,
      phase_offset_navigation::RuntimeFutureStepResult& output) {
    output = phase_offset_navigation::RuntimeFutureStepResult();
    if (!std::isfinite(input.phase) || !std::isfinite(input.delta) ||
        !std::isfinite(input.dt) || input.dt <= 0.0 ||
        !input.matched_position.allFinite() ||
        !input.previous_matched_reference.allFinite() ||
        !std::isfinite(input.previous_final_port.u_w) ||
        !std::isfinite(input.previous_final_port.u_delta)) {
      output.invalid_reason = "future matched input is invalid";
      return false;
    }
    if (!state_query || !state_query(input.phase, output.path) ||
        !StateFiniteAndValid(output.path)) {
      output.invalid_reason = "immutable future owner cannot evaluate phase";
      return false;
    }
    phase_offset_core::GeometryParams geometry_params;
    // Runtime independently rechecks this exact reference against its own
    // evaluator, so this only binds the owner/guidance side of the contract.
    geometry_params.regularity_margin = regularity_margin;
    geometry_params.minimum_reference_speed = minimum_reference_speed;
    phase_offset_core::GeometryEvaluator evaluator(geometry_params);
    phase_offset_core::PhaseOffsetGeometryState geometry;
    if (!evaluator.evaluate(output.path, input.matched_position, input.delta,
                            geometry) || !geometry.valid) {
      output.invalid_reason = geometry.invalid_reason;
      return false;
    }
    guidance::ReferenceGeometry reference;
    reference.point = geometry.r;
    reference.tangent = geometry.T;
    reference.derivative_norm = geometry.r_w.norm();
    reference.valid = geometry.valid;
    guidance::IsfGuidance base;
    if (!guidance::IsfReferenceKernel::evaluate(input.matched_position,
                                                 reference, gains, base) ||
        !base.valid) {
      output.invalid_reason = base.invalid_reason;
      return false;
    }
    output.matched_reference = reference.point;
    output.matched_tangent = reference.tangent;
    output.matched_derivative_norm = reference.derivative_norm;
    output.base_v_cmd = base.v_cmd;
    output.base_w_dot = base.w_dot;
    output.base_guidance_valid = true;
    output.valid = true;
    return true;
  };
  return contract;
}

bool PreparedSamplesOrderedAndFinite(const MatchedAdapterPathSamples& samples) {
  if (samples.size() < 2U) return false;
  for (std::size_t index = 0U; index < samples.size(); ++index) {
    if (!StateFiniteAndValid(samples[index])) return false;
    if (index > 0U &&
        samples[index].w <= samples[index - 1U].w + kPreparedCoverageTolerance) {
      return false;
    }
  }
  return true;
}

bool PreparedSamplesMatchOwner(
    const MatchedAdapterPathSamples& samples,
    const std::shared_ptr<const ContinuousPhasePath>& owner,
    const std::shared_ptr<const ContinuousPhaseNormalFrame>& shared_frame =
        std::shared_ptr<const ContinuousPhaseNormalFrame>()) {
  if (!owner || owner->empty() || !PreparedSamplesOrderedAndFinite(samples)) {
    return false;
  }
  for (const phase_offset_core::PathDifferentialState& sample : samples) {
    if (!PathStateMatchesOwner(sample, owner, shared_frame)) {
      return false;
    }
  }
  return true;
}

bool PreparedProfileSamplesOrderedAndComplete(
    const phase_offset_navigation::TubeProfile& profile) {
  if (!profile.complete || profile.samples.size() < 2U) return false;
  for (std::size_t index = 0U; index < profile.samples.size(); ++index) {
    const phase_offset_navigation::TubeRawSample& sample = profile.samples[index];
    if (!sample.complete || !IsFinite(sample.w) || !IsFinite(sample.p) ||
        !IsFinite(sample.N)) {
      return false;
    }
    if (index > 0U &&
        sample.w <= profile.samples[index - 1U].w + kPreparedCoverageTolerance) {
      return false;
    }
  }
  return true;
}

// This is structural/provenance coverage only.  The retained port must be
// inside the exact *current* interval (TubeEpochManager and Runtime enforce
// that independently), but requiring that unchanged scalar to appear at a
// future seam/horizon rejects an otherwise reachable continuous port witness.
// Runtime dry-run is the sole normal-execution proof across those future PWL
// sections, using the same immutable owner/profile/snapshot as the candidate.
bool ProfileStructurallyCoversPreparedRange(
    const phase_offset_navigation::TubeProfile& profile,
    const double captured_w0,
    const double future_seam_w,
    const double existing_future_horizon_end_w) {
  if (!PreparedProfileSamplesOrderedAndComplete(profile) ||
      !IsFinite(captured_w0) || !IsFinite(future_seam_w) ||
      !IsFinite(existing_future_horizon_end_w) ||
      future_seam_w <= captured_w0 + kPreparedCoverageTolerance ||
      existing_future_horizon_end_w + kPreparedCoverageTolerance < future_seam_w ||
      profile.preview_start_w > captured_w0 + kPreparedCoverageTolerance ||
      profile.preview_end_w + kPreparedCoverageTolerance <
          existing_future_horizon_end_w ||
      profile.certified_segment_start_w > captured_w0 +
          kPreparedCoverageTolerance ||
      profile.certified_segment_end_w + kPreparedCoverageTolerance <
          existing_future_horizon_end_w ||
      profile.samples.front().w > captured_w0 + kPreparedCoverageTolerance ||
      profile.samples.back().w + kPreparedCoverageTolerance <
          existing_future_horizon_end_w) {
    return false;
  }
  phase_offset_navigation::TubeBounds seam_bounds;
  return phase_offset_navigation::TubeFilter::query(profile, captured_w0,
                                                      seam_bounds) &&
      seam_bounds.valid &&
      phase_offset_navigation::TubeFilter::query(profile, future_seam_w,
                                                  seam_bounds) &&
      seam_bounds.valid &&
      phase_offset_navigation::TubeFilter::query(
          profile, existing_future_horizon_end_w, seam_bounds) &&
      seam_bounds.valid;
}

bool ProfileSamplesMatchOwner(
    const phase_offset_navigation::TubeProfile& profile,
    const std::shared_ptr<const ContinuousPhasePath>& owner,
    const double regularity_margin,
    const double minimum_reference_speed,
    const std::shared_ptr<const ContinuousPhaseNormalFrame>& shared_frame =
        std::shared_ptr<const ContinuousPhaseNormalFrame>()) {
  if (!PreparedProfileSamplesOrderedAndComplete(profile) || !owner ||
      owner->empty() || !IsFinite(regularity_margin) ||
      regularity_margin <= 0.0 || regularity_margin >= 1.0 ||
      !IsFinite(minimum_reference_speed) || minimum_reference_speed <= 0.0) {
    return false;
  }
  phase_offset_core::GeometryParams params;
  params.regularity_margin = regularity_margin;
  params.minimum_reference_speed = minimum_reference_speed;
  phase_offset_core::GeometryEvaluator evaluator(params);
  for (const phase_offset_navigation::TubeRawSample& sample : profile.samples) {
    phase_offset_core::PathDifferentialState owner_state;
    phase_offset_core::PhaseOffsetGeometryState geometry;
    const double probe_delta = sample.filtered_lower <= 0.0 &&
            sample.filtered_upper >= 0.0
        ? 0.0 : sample.filtered_lower;
    if (!EvaluateOwnerState(owner, sample.w, owner_state, shared_frame) ||
        !IsFinite(probe_delta) ||
        !evaluator.evaluate(owner_state, owner_state.p, probe_delta, geometry) ||
        !geometry.valid || (geometry.p - sample.p).norm() >
            kPreparedCoverageTolerance ||
        (geometry.N - sample.N).norm() > kPreparedCoverageTolerance) {
      return false;
    }
  }
  return true;
}

bool BitsEqual(const double first, const double second) {
  return std::memcmp(&first, &second, sizeof(double)) == 0;
}

bool BitsEqual(const phase_offset_core::PortCommand& first,
               const phase_offset_core::PortCommand& second) {
  return BitsEqual(first.u_w, second.u_w) &&
      BitsEqual(first.u_delta, second.u_delta);
}

bool LatestCategoricalUnsafe(
    const PhaseOffsetMatchedAdapterConfig& config,
    const CloudOccupancyQueryConfig& query_config,
    const std::shared_ptr<const plan_env::CloudOccupancySnapshot>& snapshot,
    const phase_offset_core::PathDifferentialState& current_path,
    const Eigen::Vector3d& position,
    const double retained_delta) {
  if (config.tube_source != phase_offset_navigation::TubeSource::ESDF) {
    return false;
  }
  phase_offset_core::GeometryParams geometry_params;
  geometry_params.regularity_margin = config.tube.cross_section.regularity_margin;
  geometry_params.minimum_reference_speed =
      config.tube.cross_section.minimum_reference_speed;
  phase_offset_core::GeometryEvaluator evaluator(geometry_params);
  phase_offset_core::PhaseOffsetGeometryState geometry;
  if (!evaluator.evaluate(current_path, position, retained_delta, geometry) ||
      !geometry.valid) {
    return false;
  }
  const phase_offset_navigation::RawOccupancyQuery query =
      makeCloudOccupancyQuery(snapshot, query_config);
  using phase_offset_navigation::DistanceStatus;
  const DistanceStatus reference = query(geometry.r);
  const DistanceStatus actual = query(position);
  return reference == DistanceStatus::OCCUPIED ||
      reference == DistanceStatus::OUT_OF_MAP ||
      actual == DistanceStatus::OCCUPIED ||
      actual == DistanceStatus::OUT_OF_MAP;
}

bool HasUsablePathInput(const phase_offset_core::PathDifferentialState& path) {
  return path.valid && IsFinite(path.w) && IsFinite(path.p) &&
      IsFinite(path.p_w) && IsFinite(path.p_ww);
}

bool HasUsableLegacyInput(const LegacyGuidanceSnapshot& legacy) {
  return legacy.valid && IsFinite(legacy.v_cmd) && IsFinite(legacy.w_dot) &&
      IsFinite(legacy.e_parallel) && IsFinite(legacy.e_perp) &&
      IsFinite(legacy.ref_pt) && IsFinite(legacy.tangent);
}
phase_offset_navigation::TubeSource ParseTubeSource(const std::string& value, bool& valid) {
  valid = true;
  if (value == "none") return phase_offset_navigation::TubeSource::NONE;
  if (value == "fixed") return phase_offset_navigation::TubeSource::FIXED;
  if (value == "esdf") return phase_offset_navigation::TubeSource::ESDF;
  valid = false;
  return phase_offset_navigation::TubeSource::NONE;
}

phase_offset_navigation::PhaseOffsetRuntimeConfig MakeRuntimeConfig(const PhaseOffsetMatchedAdapterConfig& config) {
  phase_offset_navigation::PhaseOffsetRuntimeConfig runtime;
  auto& manual = runtime.manual;
  manual.amplitude = config.amplitude; manual.profile_period = config.profile_period;
  manual.delta_tracking_gain = config.delta_tracking_gain; manual.u_w_amplitude = config.u_w_amplitude;
  manual.u_w_abs_max = config.u_w_abs_max; manual.u_delta_abs_max = config.u_delta_abs_max;
  manual.u_w_rate_max = config.u_w_rate_max; manual.u_delta_rate_max = config.u_delta_rate_max;
  manual.phase_dot_min = config.phase_dot_min; manual.tangent_speed_min = config.tangent_speed_min;
  manual.preflight_sample_step_w = config.preflight_sample_step_w;
  runtime.tube.source = config.tube_source; runtime.tube.invariant_gain = config.tube.invariant_gain;
  runtime.tube.interior_margin = config.tube.interior_margin;
  const bool categorical_environment = config.tube_source ==
      phase_offset_navigation::TubeSource::ESDF;
  runtime.tube.tracking_error_bound = categorical_environment
      ? config.tube.cross_section.margins.tracking_error_bound
      : config.tube.erosion.tracking_error_bound;
  runtime.tube.regularity_margin = categorical_environment
      ? config.tube.cross_section.regularity_margin
      : config.tube.regularity_margin;
  runtime.tube.minimum_reference_speed =
      config.tube.cross_section.minimum_reference_speed;
  return runtime;
}

phase_offset_navigation::TubeEpochManagerConfig MakeEpochConfig(const PhaseOffsetMatchedAdapterConfig& config) {
  phase_offset_navigation::TubeEpochManagerConfig epoch;
  epoch.builder = config.tube;
  epoch.filter = config.filter;
  return epoch;
}
geometry_msgs::Point ToPoint(const Eigen::Vector3d& value) {
  geometry_msgs::Point point;
  point.x = value.x(); point.y = value.y(); point.z = value.z();
  return point;
}
visualization_msgs::Marker MakeLine(const ros::Time& stamp, const std::string& frame,
                                    const std::string& ns, int id, float r, float g, float b) {
  visualization_msgs::Marker marker;
  marker.header.stamp = stamp; marker.header.frame_id = frame; marker.ns = ns;
  marker.id = id; marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD; marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.04; marker.color.r = r; marker.color.g = g; marker.color.b = b;
  marker.color.a = 1.0;
  return marker;
}
visualization_msgs::Marker MakeDelete(const ros::Time& stamp, const std::string& frame,
                                      const std::string& ns, int id) {
  visualization_msgs::Marker marker;
  marker.header.stamp = stamp; marker.header.frame_id = frame; marker.ns = ns;
  marker.id = id; marker.action = visualization_msgs::Marker::DELETE;
  return marker;
}
void SetGuidance(const guidance::IsfGuidance& base,
                 const phase_offset_core::MatchedPortOutput& matched,
                 guidance::IsfGuidance& output) {
  output = base; output.v_cmd = matched.v_cmd; output.w_dot = matched.w_dot;
  output.valid = matched.valid; output.invalid_reason = matched.invalid_reason;
}
int CountAsInt(std::uint64_t value) {
  return static_cast<int>(std::min<std::uint64_t>(
      value, static_cast<std::uint64_t>(std::numeric_limits<int>::max())));
}

phase_offset_navigation::PathStateQuery MakeTimerPathStateQuery(
    const std::shared_ptr<const ContinuousPhasePath>& semantic_path_owner,
    const std::shared_ptr<const MatchedAdapterPathSamples>& samples,
    const std::shared_ptr<const ContinuousPhaseNormalFrame>& shared_frame =
        std::shared_ptr<const ContinuousPhaseNormalFrame>()) {
  if (semantic_path_owner) {
    // Production requests carry shared_frame; this construction is only the
    // legacy synthetic-owner fallback.
    const std::shared_ptr<const ContinuousPhaseNormalFrame> immutable_frame =
        shared_frame ? shared_frame :
        (semantic_path_owner->pathRevision() == 0U
        ? std::shared_ptr<const ContinuousPhaseNormalFrame>()
        : std::shared_ptr<const ContinuousPhaseNormalFrame>(
              new ContinuousPhaseNormalFrame(
                  semantic_path_owner, semantic_path_owner->pathRevision(),
                  semantic_path_owner->pathRevision())));
    return [semantic_path_owner, immutable_frame](const double w,
                                 phase_offset_core::PathDifferentialState& state) {
      ContinuousPhasePathState continuous;
      if (immutable_frame) {
        if (!immutable_frame->evaluatePathState(w, continuous)) return false;
      } else if (!semantic_path_owner->evaluate(w, continuous, false)) {
        return false;
      }
      state = ConvertContinuousPhasePathStateForActive(continuous, w);
      return state.valid;
    };
  }
  if (!samples || samples->empty()) {
    return phase_offset_navigation::PathStateQuery();
  }
  // Synthetic/unit-test requests may carry complete immutable samples rather
  // than a semantic owner.  This timer-local interpolation never crosses a
  // callback boundary as a PathStateQuery.
  return [samples](const double w,
                   phase_offset_core::PathDifferentialState& state) {
    if (!std::isfinite(w) || samples->empty()) return false;
    const auto& values = *samples;
    if (w < values.front().w - 1e-12 || w > values.back().w + 1e-12) {
      return false;
    }
    for (std::size_t index = 1U; index < values.size(); ++index) {
      const auto& left = values[index - 1U];
      const auto& right = values[index];
      if (w > right.w + 1e-12) continue;
      const double span = right.w - left.w;
      if (!std::isfinite(span) || span <= 1e-12) return false;
      const double alpha = std::max(0.0, std::min(1.0, (w - left.w) / span));
      state = left;
      state.w = w;
      state.p = left.p + alpha * (right.p - left.p);
      state.p_w = left.p_w + alpha * (right.p_w - left.p_w);
      state.p_ww = left.p_ww + alpha * (right.p_ww - left.p_ww);
      state.valid = left.valid && right.valid;
      return state.valid;
    }
    state = values.back();
    state.w = w;
    return state.valid;
  };
}

phase_offset_navigation::PathCellBoundQuery MakeTimerPathCellBoundQuery(
    const std::shared_ptr<const ContinuousPhasePath>& semantic_path_owner,
    const std::shared_ptr<const ContinuousPhaseNormalFrame>& shared_frame =
        std::shared_ptr<const ContinuousPhaseNormalFrame>()) {
  if (!semantic_path_owner || semantic_path_owner->empty()) {
    return phase_offset_navigation::PathCellBoundQuery();
  }
  // Both pure queries capture the same immutable owner.  A query crossing
  // source segments, or touching an unsupported/low-speed evaluator, returns
  // false and the navigation layer retains the fixed inset.
  // Production requests carry shared_frame; the construction below is only a
  // legacy synthetic-owner fallback.
  const std::shared_ptr<const ContinuousPhaseNormalFrame> immutable_frame =
      shared_frame ? shared_frame :
      (semantic_path_owner->pathRevision() == 0U
      ? std::shared_ptr<const ContinuousPhaseNormalFrame>()
      : std::shared_ptr<const ContinuousPhaseNormalFrame>(
            new ContinuousPhaseNormalFrame(
                semantic_path_owner, semantic_path_owner->pathRevision(),
                semantic_path_owner->pathRevision())));
  return [semantic_path_owner, immutable_frame](
      const double w0, const double w1,
      phase_offset_core::PathCellGeometryCertificate& certificate) {
    if (!semantic_path_owner->cellBounds(w0, w1, certificate)) return false;
    if (immutable_frame) {
      phase_offset_core::NormalFrameCellProof frame_proof;
      if (!immutable_frame->certifyCell(w0, w1, frame_proof) ||
          !phase_offset_core::normalFrameCellProofIsComplete(frame_proof)) {
        return false;
      }
      certificate.path_revision = immutable_frame->pathRevision();
      certificate.frame_revision = immutable_frame->frameRevision();
      certificate.sup_N_w_norm = frame_proof.sup_normal_derivative;
      certificate.normal_variation_bound = frame_proof.normal_variation_bound;
      certificate.curvature_variation_bound =
          std::max(certificate.curvature_variation_bound,
                   frame_proof.tangent_variation_bound);
      certificate.normal_frame_proof_complete = true;
      // The frame certifies path-speed/normal variation only.  The combined
      // ||p_w + N_w*delta|| bound is deliberately left unset until Builder
      // has the actual admissible delta interval for this cell.
      certificate.combined_regularity_proof_complete = false;
      certificate.provenance =
          "ContinuousPhaseNormalFrame/ProjectedHermiteTransport/"
          "CertifiedProjectedRawNormLowerBound";
    }
    return phase_offset_core::pathCellGeometryCertificateIsComplete(certificate);
  };
}

// Build the immutable preview partition used by TubeBuilder.  A semantic
// owner may contain several C2 pieces (copied prefix, quintic connector and
// mapped tail), while the planner sample list is intentionally independent
// of those structural boundaries.  TubeBuilder's cell certificate is valid
// only within one owner segment, so include every segment endpoint in the
// transaction range before sorting/deduplicating the preview.  This is a
// partitioning/provenance operation; it does not alter the path, margins or
// any runtime acceptance predicate.
bool BuildOwnerAlignedTubePreview(
    const std::shared_ptr<const ContinuousPhasePath>& owner,
    const std::shared_ptr<const ContinuousPhaseNormalFrame>& shared_frame,
    const MatchedAdapterPathSamples& supplied_samples,
    const double start_w,
    const double end_w,
    const double required_current_w,
    MatchedAdapterPathSamples& output,
    const CanonicalOwnerStateReuse* const canonical_owner_state = nullptr) {
  output.clear();
  if (!owner || owner->empty() || !IsFinite(start_w) || !IsFinite(end_w) ||
      !IsFinite(required_current_w) ||
      end_w <= start_w + kPreparedCoverageTolerance ||
      required_current_w < start_w - kPreparedCoverageTolerance ||
      required_current_w > end_w + kPreparedCoverageTolerance) {
    return false;
  }
  // Validate the requested anchor before it participates in any partition
  // equivalence class.  In particular, the boundary roundoff allowance above
  // never authorizes replacing this original global phase with start_w or
  // end_w: TubeBuilder's current-state policy is about this exact double.
  phase_offset_core::PathDifferentialState required_current_state;
  if (canonical_owner_state && canonical_owner_state->valid &&
      canonical_owner_state->owner == owner &&
      ExactDoubleBits(canonical_owner_state->state.w, required_current_w)) {
    required_current_state = canonical_owner_state->state;
  } else if (!EvaluateOwnerState(owner, required_current_w,
                                 required_current_state, shared_frame)) {
    return false;
  }
  std::vector<double> knots;
  knots.reserve(supplied_samples.size() + 2U * owner->segments().size() + 3U);
  // Keep structural seam values separate from ordinary partition hints.  The
  // owner is the only authority for these boundaries: cellBounds() proves one
  // stored owner segment, not a numerical neighbourhood which happens to
  // straddle two segments.
  std::vector<double> structural_seams;
  structural_seams.reserve(2U * owner->segments().size());
  const auto append_if_in_range = [&knots, start_w, end_w](double w) {
    if (IsFinite(w) && w >= start_w - kPreparedCoverageTolerance &&
        w <= end_w + kPreparedCoverageTolerance) {
      if (std::abs(w - start_w) <= kPreparedCoverageTolerance) {
        knots.push_back(start_w);
      } else if (std::abs(w - end_w) <= kPreparedCoverageTolerance) {
        knots.push_back(end_w);
      } else {
        knots.push_back(w);
      }
    }
  };
  append_if_in_range(start_w);
  append_if_in_range(end_w);
  // The Tube current-state policy is anchored to this exact global phase.  It
  // is neither a sampling hint nor a point that may be snapped to a nearby
  // planner knot: the owner below evaluates this original double directly.
  // Do not route it through append_if_in_range(): that helper intentionally
  // canonicalizes ordinary range-neighbouring samples to a range endpoint,
  // whereas the required current phase must own its equivalence class even
  // when it is within kPreparedCoverageTolerance of start_w or end_w.
  knots.push_back(required_current_w);
  for (const auto& sample : supplied_samples) append_if_in_range(sample.w);
  const std::vector<ContinuousPhasePath::Segment>& owner_segments =
      owner->segments();
  for (std::size_t index = 1U; index < owner_segments.size(); ++index) {
    const ContinuousPhasePath::Segment& left = owner_segments[index - 1U];
    const ContinuousPhasePath::Segment& right = owner_segments[index];
    // Do not pass an owner seam through append_if_in_range(): that helper
    // intentionally canonicalizes ordinary range-neighbour knots to a
    // preview endpoint, whereas a structural seam must retain its exact owner
    // double whenever it lies in the preview range.
    if (IsFinite(left.w1) && left.w1 >= start_w && left.w1 <= end_w) {
      structural_seams.push_back(left.w1);
      knots.push_back(left.w1);
    }
    if (IsFinite(right.w0) && right.w0 >= start_w && right.w0 <= end_w) {
      structural_seams.push_back(right.w0);
      knots.push_back(right.w0);
    }
  }
  std::sort(structural_seams.begin(), structural_seams.end());
  structural_seams.erase(std::unique(structural_seams.begin(),
                                     structural_seams.end()),
                         structural_seams.end());

  // Canonical precedence is structural seam > exact required current >
  // ordinary supplied/range-neighbour knot.  TubeBuilder already freezes a
  // nearest-current match within kTubeCurrentPhaseAnchorTolerance; when that
  // equivalence class contains an immutable seam, use the exact seam as its
  // sole representative.  Retaining both values could manufacture a cell
  // smaller than the Builder's minimum cell epsilon.
  double current_match_representative = required_current_w;
  double nearest_seam_error = std::numeric_limits<double>::infinity();
  for (const double seam : structural_seams) {
    const double error = std::abs(seam - required_current_w);
    if (error <= kTubeCurrentPhaseAnchorTolerance &&
        error < nearest_seam_error) {
      current_match_representative = seam;
      nearest_seam_error = error;
    }
  }
  const auto is_structural_seam = [&structural_seams](const double w) {
    return std::binary_search(structural_seams.begin(),
                              structural_seams.end(), w);
  };
  std::sort(knots.begin(), knots.end());
  // Preserve the representative of the one direct current-match
  // neighbourhood.  Do not form a transitive chain over adjacent values
  // here: structural knots always survive, while ordinary planner/range knots
  // outside that direct neighbourhood retain the old non-transitive
  // partitioning.
  std::vector<double> canonical_knots;
  canonical_knots.reserve(knots.size());
  for (const double knot : knots) {
    if (is_structural_seam(knot)) {
      canonical_knots.push_back(knot);
      continue;
    }
    if (std::abs(knot - current_match_representative) <=
        kTubeCurrentPhaseAnchorTolerance) {
      continue;
    }
    // A nearby ordinary knot must not win the later tolerance deduplication
    // merely because it sorts before an exact structural seam.
    bool near_structural_seam = false;
    for (const double seam : structural_seams) {
      if (std::abs(knot - seam) <= kPreparedCoverageTolerance) {
        near_structural_seam = true;
        break;
      }
    }
    if (!near_structural_seam) {
      canonical_knots.push_back(knot);
    }
  }
  canonical_knots.erase(std::unique(canonical_knots.begin(),
                                    canonical_knots.end()),
                        canonical_knots.end());
  std::vector<double> deduplicated_knots;
  deduplicated_knots.reserve(canonical_knots.size() + 1U);
  for (const double knot : canonical_knots) {
    if (deduplicated_knots.empty() ||
        knot - deduplicated_knots.back() > kPreparedCoverageTolerance) {
      deduplicated_knots.push_back(knot);
      continue;
    }
    // Two distinct structural seams inside the established partition epsilon
    // cannot be represented without a degenerate cell.  Fail closed rather
    // than swallowing a seam and creating a cross-owner certificate request.
    if (is_structural_seam(knot) &&
        is_structural_seam(deduplicated_knots.back())) {
      output.clear();
      return false;
    }
  }
  canonical_knots.swap(deduplicated_knots);
  canonical_knots.insert(std::lower_bound(canonical_knots.begin(),
                                          canonical_knots.end(),
                                          current_match_representative),
                         current_match_representative);
  canonical_knots.erase(std::unique(canonical_knots.begin(),
                                    canonical_knots.end()),
                        canonical_knots.end());
  knots.swap(canonical_knots);
  // The current-match representative is permitted to be the canonical value
  // of a range-endpoint equivalence class.  It is normally the exact current;
  // only an immutable structural seam may replace it.
  const bool start_preserved =
      std::abs(knots.front() - start_w) <= kPreparedCoverageTolerance ||
      std::abs(knots.front() - current_match_representative) <=
          kTubeCurrentPhaseAnchorTolerance;
  const bool end_preserved =
      std::abs(knots.back() - end_w) <= kPreparedCoverageTolerance ||
      std::abs(knots.back() - current_match_representative) <=
          kTubeCurrentPhaseAnchorTolerance;
  if (knots.size() < 2U || !start_preserved || !end_preserved) {
    output.clear();
    return false;
  }
  output.reserve(knots.size());
  for (const double w : knots) {
    if (canonical_owner_state && canonical_owner_state->valid &&
        canonical_owner_state->owner == owner &&
        canonical_owner_state->verified_samples == &supplied_samples) {
      bool reused = false;
      for (const auto& supplied : supplied_samples) {
        if (ExactDoubleBits(supplied.w, w)) {
          if (!shared_frame ||
              (supplied.path_revision == shared_frame->pathRevision() &&
               supplied.frame_revision == shared_frame->frameRevision())) {
            output.push_back(supplied);
            reused = true;
          }
          break;
        }
      }
      if (reused) continue;
    }
    if (canonical_owner_state && canonical_owner_state->valid &&
        canonical_owner_state->owner == owner &&
        ExactDoubleBits(canonical_owner_state->state.w, w)) {
      output.push_back(canonical_owner_state->state);
      continue;
    }
    phase_offset_core::PathDifferentialState owner_state;
    if (!EvaluateOwnerState(owner, w, owner_state, shared_frame) ||
        !owner_state.valid) {
      output.clear();
      return false;
    }
    output.push_back(owner_state);
  }
  std::size_t current_anchor_count = 0U;
  for (std::size_t index = 0U; index < output.size(); ++index) {
    if (!StateFiniteAndValid(output[index]) ||
        (index > 0U && output[index].w <= output[index - 1U].w +
            kPreparedCoverageTolerance)) {
      output.clear();
      return false;
    }
    if (std::abs(output[index].w - required_current_w) <=
        kTubeCurrentPhaseAnchorTolerance) {
      ++current_anchor_count;
    }
  }
  if (current_anchor_count != 1U) {
    output.clear();
    return false;
  }
  return output.size() >= 2U;
}
}  // namespace

PathTubePairPin::PathTubePairPin(
    const std::shared_ptr<PathTubePairPinRegistry>& registry,
    const PathTubePairPinCapture& capture, const std::uint64_t lease_id)
    : registry_(registry), capture_(capture), lease_id_(lease_id) {}

PathTubePairPin::~PathTubePairPin() { release(); }

PathTubePairPin::PathTubePairPin(PathTubePairPin&& other) noexcept
    : registry_(std::move(other.registry_)), capture_(std::move(other.capture_)),
      lease_id_(other.lease_id_) {
  other.lease_id_ = 0U;
  other.capture_ = PathTubePairPinCapture();
}

PathTubePairPin& PathTubePairPin::operator=(PathTubePairPin&& other) noexcept {
  if (this == &other) return *this;
  release();
  registry_ = std::move(other.registry_);
  capture_ = std::move(other.capture_);
  lease_id_ = other.lease_id_;
  other.lease_id_ = 0U;
  other.capture_ = PathTubePairPinCapture();
  return *this;
}

void PathTubePairPin::release() {
  if (!registry_ || lease_id_ == 0U) {
    registry_.reset();
    capture_ = PathTubePairPinCapture();
    lease_id_ = 0U;
    return;
  }
  {
    std::lock_guard<std::mutex> lock(registry_->mutex);
    // ABA protection: a late guard can clear only the exact lease and exact
    // authority identity which it acquired.  A newer lease survives intact.
    if (registry_->active_lease_id == lease_id_ &&
        registry_->active_pair == capture_.pair &&
        registry_->active_generation == capture_.generation &&
        registry_->active_session == capture_.authority_session) {
      registry_->active_lease_id = 0U;
      registry_->active_pair.reset();
      registry_->active_generation = 0U;
      registry_->active_session = 0U;
    }
  }
  registry_.reset();
  capture_ = PathTubePairPinCapture();
  lease_id_ = 0U;
}

PhaseOffsetMatchedAdapterConfig PhaseOffsetMatchedAdapter::loadConfig(
    ros::NodeHandle& nh, PhaseOffsetMatchedMode mode) {
  PhaseOffsetMatchedAdapterConfig config;
  config.mode = mode;
  nh.param("phase_offset/active_equivalence_tolerance", config.equivalence_tolerance, 1e-10);
  nh.param("phase_offset/active_required_consecutive_cycles", config.warmup_cycles, 100);
  nh.param("phase_offset/measurement/enable", config.measurement_enabled, false);
  nh.param<std::string>("phase_offset/measurement/tube_due_csv_path",
                        config.measurement_tube_due_csv_path,
                        std::string());
  if (mode == PhaseOffsetMatchedMode::ACTIVE) return config;
  nh.param("phase_offset/manual/amplitude", config.amplitude, 0.10);
  nh.param("phase_offset/manual/observe_only", config.observe_only, true);
  nh.param("phase_offset/manual/profile_period", config.profile_period, 10.0);
  nh.param("phase_offset/manual/warmup_cycles", config.warmup_cycles, 100); nh.param("phase_offset/manual/delta_tracking_gain", config.delta_tracking_gain, 3.0);
  nh.param("phase_offset/manual/u_w_amplitude", config.u_w_amplitude, 0.05); nh.param("phase_offset/manual/u_w_abs_max", config.u_w_abs_max, 0.12);
  nh.param("phase_offset/manual/u_delta_abs_max", config.u_delta_abs_max, 0.25); nh.param("phase_offset/manual/u_w_rate_max", config.u_w_rate_max, 0.60);
  nh.param("phase_offset/manual/u_delta_rate_max", config.u_delta_rate_max, 1.20); nh.param("phase_offset/manual/phase_dot_min", config.phase_dot_min, 0.02);
  nh.param("phase_offset/manual/tangent_speed_min", config.tangent_speed_min, 0.02); nh.param("phase_offset/manual/preflight_sample_step_w", config.preflight_sample_step_w, 0.10);
  std::string source = "none"; nh.param<std::string>("phase_offset/manual/tube_source", source, source);
  bool source_valid = false; config.tube_source = ParseTubeSource(source, source_valid);
  nh.param("phase_offset/tube/fixed_delta_max", config.tube.fixed_delta_max, 0.06); nh.param("phase_offset/tube/sample_step_w", config.tube.sample_step_w, 0.10);
  nh.param("phase_offset/tube/lookahead_w", config.tube.lookahead_w, 2.0); nh.param("phase_offset/tube/back_w", config.tube.back_w, 0.20);
  nh.param("phase_offset/tube/min_certified_forward_w", config.tube.min_certified_forward_w, 0.40);
  nh.param("phase_offset/tube/boundary_slope_max", config.filter.boundary_slope_max, 0.80);
  nh.param("phase_offset/tube/invariant_gain", config.tube.invariant_gain, 1.0); nh.param("phase_offset/tube/interior_margin", config.tube.interior_margin, 0.0);
  nh.param("phase_offset/tube/update_period", config.tube_update_period, 0.10);
  nh.param("phase_offset/tube/environment_search_extent", config.tube.cross_section.search_extent, 3.0);
  nh.param("phase_offset/tube/raw_ray_step", config.tube.cross_section.ray_step, 0.05);
  nh.param("phase_offset/tube/boundary_tolerance", config.tube.cross_section.boundary_tolerance, 0.01);
  nh.param("phase_offset/tube/raw_regularity_margin", config.tube.cross_section.regularity_margin, 0.10);
  nh.param("phase_offset/tube/curvature_epsilon", config.tube.cross_section.curvature_epsilon, 1e-9);
  nh.param("planning/safe_distance",
           config.tube.cross_section.planner_safe_distance, 0.4);
  // The raw robust-margin parameters below are retained only for legacy
  // diagnostics.  Production Tube geometry uses planning/safe_distance.
  nh.param("phase_offset/tube/raw_uav_radius", config.tube.cross_section.margins.uav_radius, 0.25);
  nh.param("phase_offset/tube/raw_map_uncertainty", config.tube.cross_section.margins.map_uncertainty, 0.10);
  nh.param("phase_offset/tube/raw_localization_uncertainty", config.tube.cross_section.margins.localization_uncertainty, 0.05);
  nh.param("phase_offset/tube/raw_tracking_error_bound", config.tube.cross_section.margins.tracking_error_bound, 0.15);
  nh.param("phase_offset/tube/cloud_obstacle_set_complete",
           config.cloud_obstacle_set_complete, false);
  nh.param("phase_offset/tube/preincluded_map_uncertainty",
           config.tube.cross_section.margins.preincluded_map_uncertainty, 0.0);
  nh.param<std::string>("phase_offset/frame_id", config.frame_id, "world");
  if (!source_valid) config.tube.fixed_delta_max = -1.0;
  return config;
}
PhaseOffsetMatchedAdapter::PhaseOffsetMatchedAdapter(const PhaseOffsetMatchedAdapterConfig& config)
    : config_(config), path_tube_pin_registry_(
          std::make_shared<PathTubePairPinRegistry>()) {
  // Unadvertised adapter instances are deterministic unit/observe fixtures.
  // They may exercise the legacy Runtime path, but this exception is removed
  // at advertise() so production can never substitute Runtime for the sole
  // PhaseOffsetAllocator owner.
  execution_authority_.setTestOnlyRuntimeOwnerAllowed(true);
  measurement_tube_due_enabled_ = config_.measurement_enabled &&
      !config_.measurement_tube_due_csv_path.empty();
  if (measurement_tube_due_enabled_) {
    measurement_tube_due_samples_.reserve(4096U);
  }
  cloud_occupancy_query_config_.obstacle_set_complete =
      config_.cloud_obstacle_set_complete;
  cloud_occupancy_query_config_.required_preincluded_map_uncertainty =
      config_.tube.cross_section.margins.preincluded_map_uncertainty;
  configuration_valid_ = (config_.mode == PhaseOffsetMatchedMode::ACTIVE ||
      config_.mode == PhaseOffsetMatchedMode::MANUAL) &&
      IsFinite(config_.equivalence_tolerance) && config_.equivalence_tolerance >= 0.0 &&
      config_.warmup_cycles >= 100;
  if (config_.frame_id.empty()) config_.frame_id = "world";
  if (config_.mode == PhaseOffsetMatchedMode::MANUAL) {
    runtime_.reset(new phase_offset_navigation::PhaseOffsetRuntime(MakeRuntimeConfig(config_)));
    tube_epoch_manager_.reset(new phase_offset_navigation::TubeEpochManager(MakeEpochConfig(config_)));
    configuration_valid_ = configuration_valid_ && runtime_->configurationValid() &&
        tube_epoch_manager_->configurationValid() && IsFinite(config_.tube_update_period) &&
        config_.tube_update_period >= 0.05 && config_.tube_update_period <= 0.10 &&
        cloudOccupancyQueryConfigurationValid(cloud_occupancy_query_config_);
  }
}
PhaseOffsetMatchedAdapter::~PhaseOffsetMatchedAdapter() {
  shutdown();
  flushTubeDueTiming();
}
void PhaseOffsetMatchedAdapter::recordTubeDueTiming(
    const std::uint64_t steady_duration_ns,
    const std::uint64_t ros_stamp_ns,
    const bool source_current_finalized,
    const bool raw_cloud_publish_attempted) {
  if (!measurement_tube_due_enabled_) return;
  std::lock_guard<std::mutex> lock(measurement_tube_due_mutex_);
  TubeDueTimingSample sample;
  sample.sequence = measurement_tube_due_sequence_++;
  sample.steady_duration_ns = steady_duration_ns;
  sample.ros_stamp_ns = ros_stamp_ns;
  sample.source_current_finalized = source_current_finalized;
  sample.raw_cloud_publish_attempted = raw_cloud_publish_attempted;
  measurement_tube_due_samples_.push_back(sample);
}
void PhaseOffsetMatchedAdapter::flushTubeDueTiming() {
  if (!measurement_tube_due_enabled_) return;
  std::vector<TubeDueTimingSample> samples;
  {
    std::lock_guard<std::mutex> lock(measurement_tube_due_mutex_);
    samples.swap(measurement_tube_due_samples_);
  }
  if (samples.empty()) return;
  std::ofstream stream(config_.measurement_tube_due_csv_path.c_str(),
                       std::ios::out | std::ios::trunc);
  if (!stream.is_open()) return;
  stream << "kind,sequence,steady_duration_ns,ros_stamp_ns,"
         << "source_current_finalized,raw_cloud_publish_attempted\n";
  for (const TubeDueTimingSample& sample : samples) {
    stream << "tube_due," << sample.sequence << ','
           << sample.steady_duration_ns << ',' << sample.ros_stamp_ns << ','
           << (sample.source_current_finalized ? 1 : 0) << ','
           << (sample.raw_cloud_publish_attempted ? 1 : 0) << '\n';
  }
}
bool PhaseOffsetMatchedAdapter::configurationValid() const { return configuration_valid_; }
bool PhaseOffsetMatchedAdapter::requiresPathSamples() const { return false; }
double PhaseOffsetMatchedAdapter::sampleStepW() const {
  return std::min(0.10,
                  std::min(config_.tube.sample_step_w,
                           config_.preflight_sample_step_w));
}
bool PhaseOffsetMatchedAdapter::requiresTubeTimer() const {
  return configuration_valid_ &&
      config_.mode == PhaseOffsetMatchedMode::MANUAL;
}

bool PhaseOffsetMatchedAdapter::requiresAuthoritativeOffsetHandoff() const {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  return requiresAuthoritativeOffsetHandoffLocked();
}

bool PhaseOffsetMatchedAdapter::requiresAuthoritativeOffsetHandoffLocked() const {
  if (!requiresTubeTimer() || config_.observe_only || !runtime_ ||
      config_.tube_source == phase_offset_navigation::TubeSource::NONE) {
    return false;
  }
  if (runtime_->hasExecutedOffsetAuthority()) return true;
  // A committed RECOVERY tick may have reached neutral while the immutable
  // authority snapshot still names RecoveryOwner.  Keep the pair on the
  // command path until the explicit atomic neutral handoff is published.
  const phase_offset_navigation::ActiveReferenceSnapshot authority =
      execution_authority_.snapshot();
  return authority.valid &&
      authority.owner_mode ==
          phase_offset_navigation::ActiveReferenceOwnerMode::RECOVERY &&
      authority.proposed_next_delta == 0.0;
}

bool PhaseOffsetMatchedAdapter::requiresPathTubePairBootstrap() const {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  return requiresPathTubePairBootstrapLocked();
}

bool PhaseOffsetMatchedAdapter::requiresPathTubePairBootstrapLocked() const {
  if (!requiresTubeTimer() || config_.observe_only || !runtime_ ||
      config_.tube_source == phase_offset_navigation::TubeSource::NONE) {
    return false;
  }
  // Runtime is a deterministic unadvertised fixture owner only.  The same
  // existing authority capability is cleared by advertise() for production;
  // do not let passive Runtime intent bootstrap a PathTubePair that cannot
  // legally execute production NORMAL selected-u.
  if (!execution_authority_.config().allow_test_only_runtime_owner) {
    return false;
  }
  // Configuration alone must never capture the planner frontend.  This is
  // intentionally the post-warm-up activation edge: the manager will build
  // and dry-run a same-owner pair before Runtime is allowed to start.
  // Bootstrap is only the transition from pending neutral intent to the first
  // authoritative nonzero command.  If Runtime has already executed offset
  // authority, losing the Pair must remain fail-closed: a null-expected-pair
  // bootstrap would otherwise attach retained nonzero state to a new planner
  // owner without an old-authority seam or recovery proof.
  const std::shared_ptr<const TubeEpochSnapshot> epoch =
      std::atomic_load(&latest_epoch_snapshot_);
  if (!epoch || !IsOffsetCertifiedProfile(epoch->active_profile)) {
    return false;
  }
  return zero_gate_open_ && !failure_latched_ &&
      !capturePathTubePair() && runtime_->hasPendingOrActiveOffsetIntent() &&
      !runtime_->hasExecutedOffsetAuthority();
}

bool PhaseOffsetMatchedAdapter::hasPendingOffsetActivationPair(
    const std::shared_ptr<const PathTubePair>& pair) const {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  return hasPendingOffsetActivationPairLocked(pair);
}

bool PhaseOffsetMatchedAdapter::requestRecenter() {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  if (!runtime_ || !configuration_valid_ || config_.observe_only ||
      config_.tube_source == phase_offset_navigation::TubeSource::NONE ||
      !runtime_->hasExecutedOffsetAuthority()) {
    return false;
  }
  runtime_->requestRecenter();
  return true;
}

bool PhaseOffsetMatchedAdapter::recenterRequested() const {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  return runtime_ && runtime_->recenterRequested();
}

bool PhaseOffsetMatchedAdapter::hasPendingOffsetActivationPairLocked(
    const std::shared_ptr<const PathTubePair>& pair) const {
  if (!pair || !requiresTubeTimer() || config_.observe_only || !runtime_ ||
      config_.tube_source == phase_offset_navigation::TubeSource::NONE) {
    return false;
  }
  // A Runtime-owned pending activation is valid only for the existing
  // unadvertised test fixture capability.  In production, an already present
  // never-executed Runtime pair must not block neutral planner retirement.
  if (!execution_authority_.config().allow_test_only_runtime_owner) {
    return false;
  }
  return IsOffsetCertifiedProfile(pair->active_profile) &&
      zero_gate_open_ && !failure_latched_ &&
      pair == capturePathTubePair() &&
      runtime_->hasPendingOrActiveOffsetIntent() &&
      !runtime_->hasExecutedOffsetAuthority();
}

double PhaseOffsetMatchedAdapter::bootstrapPreparedHorizonEnd(
    const double current_w, const double future_seam_w,
    const double path_end_w) const {
  if (!IsFinite(current_w) || !IsFinite(future_seam_w) ||
      !IsFinite(path_end_w) || future_seam_w <= current_w ||
      path_end_w < future_seam_w) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::min(path_end_w,
                  std::max(future_seam_w,
                           current_w +
                               config_.tube.min_certified_forward_w));
}

std::shared_ptr<const PathTubePair>
PhaseOffsetMatchedAdapter::capturePathTubePair() const {
  return std::atomic_load(&authoritative_path_tube_pair_);
}

std::unique_ptr<PathTubePairPin>
PhaseOffsetMatchedAdapter::captureAndAcquirePathTubePairPin() {
  if (!runtime_ || !path_tube_pin_registry_ ||
      shutdown_requested_.load(std::memory_order_acquire)) {
    return std::unique_ptr<PathTubePairPin>();
  }
  // Capture pair identity/provenance and the Runtime bits under the one
  // short command boundary.  No seam selection, tube build, or dry-run may
  // start until this acquisition succeeds.
  std::lock_guard<std::mutex> command_lock(runtime_command_mutex_);
  const std::shared_ptr<const PathTubePair> pair =
      std::atomic_load(&authoritative_path_tube_pair_);
  if (!pair || pair->source_revision == 0U || pair->generation == 0U ||
      pair->authority_session !=
          authority_session_.load(std::memory_order_acquire) ||
      !pair->path_owner || !pair->full_path_samples ||
      !pair->active_profile || !pair->epoch_snapshot) {
    return std::unique_ptr<PathTubePairPin>();
  }
  PathTubePairPinCapture capture;
  capture.pair = pair;
  capture.source_revision = pair->source_revision;
  capture.generation = pair->generation;
  capture.authority_session = pair->authority_session;
  capture.map_observation_sequence = pair->map_observation_sequence;
  capture.map_observation_is_snapshot = pair->map_observation_is_snapshot;
  capture.path_owner = pair->path_owner;
  capture.frame_owner = pair->frame_owner;
  capture.frozen_cloud_occupancy_snapshot =
      pair->frozen_cloud_occupancy_snapshot;
  capture.full_path_samples = pair->full_path_samples;
  capture.active_profile = pair->active_profile;
  capture.epoch_snapshot = pair->epoch_snapshot;
  capture.retained_delta = runtime_->retainedDelta();
  capture.previous_final_port = runtime_->previousFinalPort();
  if (!capture.valid()) return std::unique_ptr<PathTubePairPin>();

  std::lock_guard<std::mutex> pin_lock(path_tube_pin_registry_->mutex);
  if (path_tube_pin_registry_->active_lease_id != 0U) {
    return std::unique_ptr<PathTubePairPin>();
  }
  const std::uint64_t lease_id = ++path_tube_pin_registry_->next_lease_id;
  path_tube_pin_registry_->active_lease_id = lease_id;
  path_tube_pin_registry_->active_pair = pair;
  path_tube_pin_registry_->active_generation = capture.generation;
  path_tube_pin_registry_->active_session = capture.authority_session;
  return std::unique_ptr<PathTubePairPin>(
      new PathTubePairPin(path_tube_pin_registry_, capture, lease_id));
}

void PhaseOffsetMatchedAdapter::requestShutdown() {
  shutdown_requested_.store(true, std::memory_order_release);
}

void PhaseOffsetMatchedAdapter::shutdown() {
  requestShutdown();
  while (timer_inflight_.load(std::memory_order_acquire)) {
    std::this_thread::yield();
  }
  std::lock_guard<std::mutex> command_lock(runtime_command_mutex_);
  pending_runtime_commit_ = phase_offset_navigation::RuntimeCommitToken();
  pending_authority_prepared_ =
      phase_offset_navigation::AuthorityPreparedStep();
  pending_authority_session_ = 0U;
  pending_authority_valid_ = false;
  pending_recovery_step_ = phase_offset_navigation::RecoveryPreparedStep();
  pending_recovery_step_valid_ = false;
  pending_handoff_input_ = phase_offset_navigation::HandoffStateInput();
  pending_handoff_decision_ = phase_offset_navigation::HandoffDecision();
  pending_handoff_valid_ = false;
  pending_recovery_source_pair_.reset();
  pending_recovery_execution_pair_.reset();
  pending_recovery_target_pair_.reset();
  recovery_deadline_ = std::numeric_limits<double>::quiet_NaN();
  recovery_deadline_session_ = 0U;
  recovery_deadline_target_revision_ = 0U;
  const std::shared_ptr<const ControlPublishSnapshot> control =
      std::atomic_load(&latest_control_snapshot_);
  if (control && control->active) publishManualDelete(*control);
  if (path_tube_pin_registry_) {
    std::lock_guard<std::mutex> pin_lock(path_tube_pin_registry_->mutex);
    path_tube_pin_registry_->active_lease_id = 0U;
    path_tube_pin_registry_->active_pair.reset();
    path_tube_pin_registry_->active_generation = 0U;
    path_tube_pin_registry_->active_session = 0U;
  }
  std::atomic_store(&latest_build_request_,
                    std::shared_ptr<const TubeBuildRequest>());
  std::atomic_store(&latest_candidate_epoch_snapshot_,
                    std::shared_ptr<const TubeEpochSnapshot>());
  std::atomic_store(&latest_epoch_snapshot_,
                    std::shared_ptr<const TubeEpochSnapshot>());
  std::atomic_store(&latest_control_snapshot_,
                    std::shared_ptr<const ControlPublishSnapshot>());
  std::atomic_store(&authoritative_path_tube_pair_,
                    std::shared_ptr<const PathTubePair>());
  authority_session_.fetch_add(1U, std::memory_order_acq_rel);
}

std::uint64_t PhaseOffsetMatchedAdapter::retirePathTubeAuthority(
    const std::uint64_t authority_session) {
  // This is intentionally only an ownership retirement.  In particular do
  // not reset Runtime's retained delta/previous final port: a stale command
  // must be rejected, and any surviving executed authority remains unpaired
  // and fail-closed rather than being silently rebound by bootstrap.
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  return retirePathTubeAuthorityLocked(authority_session);
}

bool PhaseOffsetMatchedAdapter::resetForNewNavigationTask(
    const std::uint64_t expected_authority_session,
    std::uint64_t& retired_authority_session) {
  retired_authority_session = 0U;
  // This is the task-boundary linearization point.  A timer completion that
  // already owns the publication barrier finishes before reset returns; a
  // later completion observes the new generation and cannot publish old
  // diagnostics, markers, or evidence slots.  Keep this order consistent
  // with finalizeTubeEpoch to avoid a runtime/publication lock inversion.
  std::lock_guard<std::mutex> publication_lock(task_publication_mutex_);
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  const std::uint64_t current =
      authority_session_.load(std::memory_order_acquire);
  if (expected_authority_session != current || !runtime_) return false;

  const std::shared_ptr<const ControlPublishSnapshot> previous_control =
      std::atomic_load(&latest_control_snapshot_);

  // The timer owns its manager/profile/cache and will consume this token on
  // its next tick.  Do not touch timer-owned state from this command thread.
  task_generation_.fetch_add(1U, std::memory_order_acq_rel);

  // Reuse the established retirement operation for the old lease, pair, and
  // timer evidence, then deliberately clear only the new-task additions.
  // Ordinary H2 retirement must keep Runtime history and therefore never
  // reaches this branch.
  retired_authority_session = retirePathTubeAuthorityLocked(current + 1U);
  // The new task now owns the lifecycle boundary.  It may clear the old
  // namespaces once, while stale timer callbacks remain unable to do so.
  if (previous_control && previous_control->active) {
    publishManualDelete(*previous_control);
  }
  std::atomic_store(&latest_control_snapshot_,
                    std::shared_ptr<const ControlPublishSnapshot>());
  zero_gate_consecutive_count_ = 0;
  zero_gate_open_ = false;
  source_identity_ = nullptr;
  source_start_w_ = 0.0;
  source_end_w_ = 0.0;
  // Revisions are deliberately monotonic across tasks.  A new planner may
  // restart its local revision at one; reusing that number could otherwise
  // make an old timer profile look equivalent to a new task's profile.
  last_preflight_revision_ = 0U;
  have_preflight_revision_ = false;
  last_consumed_epoch_build_sequence_ = 0U;
  runtime_->resetForNewNavigationTask();
  pending_runtime_commit_ = phase_offset_navigation::RuntimeCommitToken();
  pending_authority_prepared_ =
      phase_offset_navigation::AuthorityPreparedStep();
  pending_authority_session_ = 0U;
  pending_authority_valid_ = false;
  pending_recovery_step_ = phase_offset_navigation::RecoveryPreparedStep();
  pending_recovery_step_valid_ = false;
  pending_handoff_input_ = phase_offset_navigation::HandoffStateInput();
  pending_handoff_decision_ = phase_offset_navigation::HandoffDecision();
  pending_handoff_valid_ = false;
  pending_recovery_source_pair_.reset();
  pending_recovery_execution_pair_.reset();
  pending_recovery_target_pair_.reset();
  recovery_deadline_ = std::numeric_limits<double>::quiet_NaN();
  recovery_deadline_session_ = 0U;
  recovery_deadline_target_revision_ = 0U;
  execution_authority_.resetForNewTask(retired_authority_session);
  return true;
}

std::uint64_t PhaseOffsetMatchedAdapter::retirePathTubeAuthorityLocked(
    const std::uint64_t authority_session) {
  const std::uint64_t current =
      authority_session_.load(std::memory_order_acquire);
  const std::uint64_t retired_session = authority_session > current
      ? authority_session : current + 1U;
  if (path_tube_pin_registry_) {
    std::lock_guard<std::mutex> pin_lock(path_tube_pin_registry_->mutex);
    path_tube_pin_registry_->active_lease_id = 0U;
    path_tube_pin_registry_->active_pair.reset();
    path_tube_pin_registry_->active_generation = 0U;
    path_tube_pin_registry_->active_session = 0U;
  }
  authority_session_.store(retired_session, std::memory_order_release);
  std::atomic_store(&authoritative_path_tube_pair_,
                    std::shared_ptr<const PathTubePair>());
  std::atomic_store(&latest_build_request_,
                    std::shared_ptr<const TubeBuildRequest>());
  std::atomic_store(&latest_candidate_epoch_snapshot_,
                    std::shared_ptr<const TubeEpochSnapshot>());
  std::atomic_store(&latest_epoch_snapshot_,
                    std::shared_ptr<const TubeEpochSnapshot>());
  have_source_identity_ = false;
  have_preflight_revision_ = false;
  command_active_ = false;
  pending_runtime_commit_ = phase_offset_navigation::RuntimeCommitToken();
  pending_authority_prepared_ =
      phase_offset_navigation::AuthorityPreparedStep();
  pending_authority_session_ = 0U;
  pending_authority_valid_ = false;
  pending_recovery_step_ = phase_offset_navigation::RecoveryPreparedStep();
  pending_recovery_step_valid_ = false;
  pending_handoff_input_ = phase_offset_navigation::HandoffStateInput();
  pending_handoff_decision_ = phase_offset_navigation::HandoffDecision();
  pending_handoff_valid_ = false;
  pending_recovery_source_pair_.reset();
  pending_recovery_execution_pair_.reset();
  pending_recovery_target_pair_.reset();
  recovery_deadline_ = std::numeric_limits<double>::quiet_NaN();
  recovery_deadline_session_ = 0U;
  recovery_deadline_target_revision_ = 0U;
  return retired_session;
}

bool PhaseOffsetMatchedAdapter::retirePathTubeAuthorityIfNeutral(
    const std::uint64_t authority_session,
    std::uint64_t& retired_session) {
  retired_session = 0U;
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  // A just-committed bootstrap Pair has not executed Runtime authority yet,
  // but it is nevertheless the sole certified owner of the next activation
  // command.  Check it under this same lock as the final CAS so a neutral
  // planner publication cannot erase the Pair in the check-to-retire gap.
  const std::shared_ptr<const PathTubePair> pair = capturePathTubePair();
  if (!runtime_ || runtime_->hasExecutedOffsetAuthority() ||
      hasPendingOffsetActivationPairLocked(pair)) {
    return false;
  }
  retired_session = retirePathTubeAuthorityLocked(authority_session);
  execution_authority_.resetForNewTask(retired_session);
  return true;
}

void PhaseOffsetMatchedAdapter::deactivate(const ros::Time& stamp) {
  const std::uint64_t task_generation =
      task_generation_.load(std::memory_order_acquire);
  if (deactivate_test_hook_) deactivate_test_hook_();
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  deactivateLocked(stamp, task_generation);
}

void PhaseOffsetMatchedAdapter::deactivateLocked(
    const ros::Time& stamp, const std::uint64_t expected_task_generation) {
  if (shutdown_requested_.load(std::memory_order_acquire) ||
      expected_task_generation !=
          task_generation_.load(std::memory_order_acquire)) {
    return;
  }
  // Deactivation invalidates any staged PositionCommand transaction.  In
  // particular, a goal override or an explicit planner handoff may occur
  // after the adapter prepared a recovery/runtime step but before the manager
  // reaches publication.  Never let that stale staged step commit against the
  // replacement command; retain only the last already-committed authority.
  pending_runtime_commit_ = phase_offset_navigation::RuntimeCommitToken();
  pending_authority_prepared_ =
      phase_offset_navigation::AuthorityPreparedStep();
  pending_authority_session_ = 0U;
  pending_authority_valid_ = false;
  pending_recovery_step_ = phase_offset_navigation::RecoveryPreparedStep();
  pending_recovery_step_valid_ = false;
  pending_handoff_input_ = phase_offset_navigation::HandoffStateInput();
  pending_handoff_decision_ = phase_offset_navigation::HandoffDecision();
  pending_handoff_valid_ = false;
  pending_recovery_source_pair_.reset();
  pending_recovery_execution_pair_.reset();
  pending_recovery_target_pair_.reset();
  recovery_deadline_ = std::numeric_limits<double>::quiet_NaN();
  recovery_deadline_session_ = 0U;
  recovery_deadline_target_revision_ = 0U;
  if (!command_active_) return;
  command_active_ = false;
  std::shared_ptr<TubeBuildRequest> request(new TubeBuildRequest());
  request->active = false;
  request->task_generation =
      task_generation_.load(std::memory_order_acquire);
  request->control_sequence = ++control_sequence_;
  request->stamp = stamp;
  std::atomic_store(&latest_build_request_,
                    std::shared_ptr<const TubeBuildRequest>(request));
  std::atomic_store(&latest_candidate_epoch_snapshot_,
                    std::shared_ptr<const TubeEpochSnapshot>());
  std::atomic_store(&latest_epoch_snapshot_,
                    std::shared_ptr<const TubeEpochSnapshot>());

  std::shared_ptr<ControlPublishSnapshot> control(new ControlPublishSnapshot());
  control->active = false;
  control->task_generation = request->task_generation;
  control->control_sequence = request->control_sequence;
  control->stamp = stamp;
  std::atomic_store(&latest_control_snapshot_,
                    std::shared_ptr<const ControlPublishSnapshot>(control));
}

bool PhaseOffsetMatchedAdapter::prepareDeactivationLocked(
    const ros::Time& stamp, const std::uint64_t expected_task_generation,
    DeactivationCommitToken& token) const {
  token = DeactivationCommitToken();
  if (shutdown_requested_.load(std::memory_order_acquire) ||
      expected_task_generation !=
          task_generation_.load(std::memory_order_acquire)) {
    return false;
  }
  token.task_generation = expected_task_generation;
  token.next_control_sequence = control_sequence_ + 1U;
  token.command_active_before = command_active_;
  if (command_active_) {
    try {
      std::shared_ptr<TubeBuildRequest> request(new TubeBuildRequest());
      request->active = false;
      request->task_generation = expected_task_generation;
      request->control_sequence = token.next_control_sequence;
      request->stamp = stamp;
      token.inactive_request =
          std::shared_ptr<const TubeBuildRequest>(std::move(request));
      std::shared_ptr<ControlPublishSnapshot> control(
          new ControlPublishSnapshot());
      control->active = false;
      control->task_generation = expected_task_generation;
      control->control_sequence = token.next_control_sequence;
      control->stamp = stamp;
      token.inactive_control =
          std::shared_ptr<const ControlPublishSnapshot>(std::move(control));
    } catch (const std::bad_alloc&) {
      return false;
    }
  }
  token.valid = true;
  return true;
}

void PhaseOffsetMatchedAdapter::commitDeactivationNoFailLocked(
    const DeactivationCommitToken& token) noexcept {
  pending_runtime_commit_ = phase_offset_navigation::RuntimeCommitToken();
  pending_authority_prepared_ =
      phase_offset_navigation::AuthorityPreparedStep();
  pending_authority_session_ = 0U;
  pending_authority_valid_ = false;
  pending_recovery_step_ = phase_offset_navigation::RecoveryPreparedStep();
  pending_recovery_step_valid_ = false;
  pending_handoff_input_ = phase_offset_navigation::HandoffStateInput();
  pending_handoff_decision_ = phase_offset_navigation::HandoffDecision();
  pending_handoff_valid_ = false;
  pending_recovery_source_pair_.reset();
  pending_recovery_execution_pair_.reset();
  pending_recovery_target_pair_.reset();
  recovery_deadline_ = std::numeric_limits<double>::quiet_NaN();
  recovery_deadline_session_ = 0U;
  recovery_deadline_target_revision_ = 0U;
  if (!token.command_active_before) return;
  command_active_ = false;
  control_sequence_ = token.next_control_sequence;
  std::atomic_store(&latest_build_request_, token.inactive_request);
  std::atomic_store(&latest_candidate_epoch_snapshot_,
                    token.inactive_candidate_snapshot);
  std::atomic_store(&latest_epoch_snapshot_, token.inactive_epoch_snapshot);
  std::atomic_store(&latest_control_snapshot_, token.inactive_control);
}
void PhaseOffsetMatchedAdapter::advertise(ros::NodeHandle& nh) {
  if (!configuration_valid_ || advertised_) return;
  execution_authority_.setTestOnlyRuntimeOwnerAllowed(false);
  if (config_.mode == PhaseOffsetMatchedMode::ACTIVE) {
    active_diagnostics_pub_ = nh.advertise<std_msgs::Float64MultiArray>("phase_offset_active/diagnostics", 1);
  } else {
    manual_base_path_pub_ = nh.advertise<visualization_msgs::Marker>("phase_offset_manual/base_path", 1, true);
    manual_active_path_pub_ = nh.advertise<visualization_msgs::Marker>("phase_offset_manual/active_path", 1, true);
    manual_frame_pub_ = nh.advertise<visualization_msgs::MarkerArray>("phase_offset_manual/frame", 1);
    manual_tube_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        "phase_offset_manual/tube", kManualEvidenceQueueSize);
    manual_tube_candidate_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        "phase_offset_manual/tube_candidate", kManualEvidenceQueueSize);
    manual_diagnostics_pub_ = nh.advertise<std_msgs::Float64MultiArray>(
        "phase_offset_manual/diagnostics", kManualEvidenceQueueSize);
    manual_tube_epoch_diagnostics_pub_ = nh.advertise<std_msgs::Float64MultiArray>(
        "phase_offset_manual/tube_epoch_diagnostics", kManualEvidenceQueueSize);
    if (config_.tube_source == phase_offset_navigation::TubeSource::ESDF) {
      manual_raw_candidate_diagnostics_pub_ = nh.advertise<std_msgs::Float64MultiArray>(
          "phase_offset_manual/tube_raw_candidate_diagnostics", kManualEvidenceQueueSize);
      manual_cloud_snapshot_diagnostics_pub_ =
          nh.advertise<std_msgs::Float64MultiArray>(
              "phase_offset_manual/tube_cloud_snapshot_diagnostics",
              kManualEvidenceQueueSize);
    }
  }
  advertised_ = true;
}
bool PhaseOffsetMatchedAdapter::collectSamples(const TubeBuildRequest& request,
                                               PathSamples& full_path) const {
  full_path.clear();
  if (request.supplied_path_samples) {
    full_path = *request.supplied_path_samples;
  }
  if (!full_path.empty()) return true;
  if (!request.semantic_path_owner || request.semantic_path_owner->empty()) {
    return false;
  }
  std::vector<double> w; std::vector<ContinuousPhasePathState> states;
  const double tube_sample_step = std::min(
      0.10, std::min(config_.tube.sample_step_w, config_.preflight_sample_step_w));
  if (!request.semantic_path_owner->sample(tube_sample_step, w, states) ||
      w.size() != states.size()) return false;
  for (std::size_t index = 0U; index < w.size(); ++index) {
    full_path.push_back(ConvertContinuousPhasePathStateForActive(states[index], w[index]));
  }
  return !full_path.empty();
}
bool PhaseOffsetMatchedAdapter::makePreview(const PathSamples& full_path,
                                            const phase_offset_core::PathDifferentialState& current,
                                            PathSamples& preview) const {
  preview.clear();
  const double lower = current.w - config_.tube.back_w;
  const double upper = current.w + config_.tube.lookahead_w;
  for (const auto& state : full_path) {
    if (state.w >= lower - 1e-12 && state.w <= upper + 1e-12) preview.push_back(state);
  }
  preview.push_back(current);
  std::sort(preview.begin(), preview.end(), [](const auto& a, const auto& b) { return a.w < b.w; });
  preview.erase(std::unique(preview.begin(), preview.end(), [](const auto& a, const auto& b) {
    return std::abs(a.w - b.w) <= 1e-12;
  }), preview.end());
  return !preview.empty();
}
std::uint64_t PhaseOffsetMatchedAdapter::sourceRevision(const MatchedAdapterInput& input) {
  const void* identity = input.semantic_path_owner
      ? static_cast<const void*>(input.semantic_path_owner.get())
      : (input.semantic_path ? static_cast<const void*>(input.semantic_path)
                             : input.semantic_path_identity);
  if (!have_source_identity_ || identity != source_identity_ ||
      std::abs(input.semantic_path_start_w - source_start_w_) > 1e-12 ||
      std::abs(input.semantic_path_end_w - source_end_w_) > 1e-12) {
    have_source_identity_ = true; source_identity_ = identity;
    source_start_w_ = input.semantic_path_start_w; source_end_w_ = input.semantic_path_end_w;
    ++source_revision_;
  }
  return source_revision_ == 0U ? ++source_revision_ : source_revision_;
}
bool PhaseOffsetMatchedAdapter::makeAuthorityRequest(
    const double retained_delta,
    phase_offset_navigation::TubeBounds& authority_request) const {
  authority_request = phase_offset_navigation::TubeBounds();
  const double amplitude = std::abs(config_.amplitude);
  const double interior_margin = config_.tube.interior_margin;
  if (!IsFinite(amplitude) || !IsFinite(retained_delta) ||
      !IsFinite(interior_margin) || interior_margin < 0.0) {
    return false;
  }
  const double requested_lower =
      std::min(std::min(-amplitude, retained_delta), 0.0) - interior_margin;
  const double requested_upper =
      std::max(std::max(amplitude, retained_delta), 0.0) + interior_margin;
  if (!IsFinite(requested_lower) || !IsFinite(requested_upper) ||
      requested_lower > requested_upper || requested_lower > 0.0 ||
      requested_upper < 0.0) {
    return false;
  }
  authority_request.lower = requested_lower;
  authority_request.upper = requested_upper;
  authority_request.valid = true;
  return true;
}
std::shared_ptr<const TubeBuildRequest> PhaseOffsetMatchedAdapter::makeBuildRequest(
    const MatchedAdapterInput& input, const std::uint64_t source_revision,
    const double retained_delta) {
  std::shared_ptr<TubeBuildRequest> request(new TubeBuildRequest());
  request->active = true;
  request->task_generation =
      task_generation_.load(std::memory_order_acquire);
  request->control_sequence = ++control_sequence_;
  request->source_revision = source_revision;
  // Ordinary (non-pair) requests still belong to the adapter's current
  // authority session.  Without carrying this private session token, any
  // planner-owned C2 retirement increments authority_session_ while leaving
  // request->authority_session at its default zero, so every timer build is
  // rejected by requestSourceStillCurrent() before Candidate/epoch
  // publication.  Pair requests set the same field explicitly in update().
  request->authority_session = authority_session_.load(
      std::memory_order_acquire);
  request->stamp = input.stamp;
  request->semantic_path_owner = input.semantic_path_owner;
  request->frame_owner = input.frame_owner;
  if (!request->frame_owner && request->semantic_path_owner &&
      source_revision != 0U) {
    const std::shared_ptr<const TubeBuildRequest> latest =
        std::atomic_load(&latest_build_request_);
    if (latest && latest->semantic_path_owner == request->semantic_path_owner &&
        latest->source_revision == source_revision &&
        FrameMatchesRevision(latest->frame_owner, source_revision)) {
      request->frame_owner = latest->frame_owner;
    } else {
      const std::shared_ptr<const PathTubePair> pair =
          std::atomic_load(&authoritative_path_tube_pair_);
      if (pair && pair->path_owner == request->semantic_path_owner &&
          pair->source_revision == source_revision &&
          FrameMatchesRevision(pair->frame_owner, source_revision)) {
        request->frame_owner = pair->frame_owner;
      }
    }
    if (!request->frame_owner) {
      request->frame_owner = std::shared_ptr<const ContinuousPhaseNormalFrame>(
          new ContinuousPhaseNormalFrame(request->semantic_path_owner,
                                         source_revision, source_revision));
    }
  }
  request->semantic_path_start_w = input.semantic_path_start_w;
  request->semantic_path_end_w = input.semantic_path_end_w;
  if (!input.sampled_path.empty()) {
    request->supplied_path_samples =
        std::make_shared<const PathSamples>(input.sampled_path);
  }
  request->current_path = input.path;
  request->position = input.position;
  request->gains = input.gains;
  request->dt = input.dt;
  request->retained_delta = retained_delta;
  if (config_.tube_source == phase_offset_navigation::TubeSource::ESDF) {
    makeAuthorityRequest(retained_delta, request->authority_request);
  }
  request->cloud_snapshot = input.cloud_occupancy_snapshot;
  if (config_.tube_source == phase_offset_navigation::TubeSource::ESDF) {
    request->map_observation_is_snapshot = true;
    request->map_observation_sequence = request->cloud_snapshot
        ? request->cloud_snapshot->observation_sequence : 0U;
  }
  return std::shared_ptr<const TubeBuildRequest>(request);
}

bool PhaseOffsetMatchedAdapter::requestSourceStillCurrent(
    const TubeBuildRequest& request) const {
  const std::shared_ptr<const TubeBuildRequest> latest =
      std::atomic_load(&latest_build_request_);
  if (!latest || !latest->active ||
      latest->task_generation != request.task_generation ||
      latest->source_revision != request.source_revision ||
      latest->authority_session != request.authority_session ||
      task_generation_.load(std::memory_order_acquire) !=
          request.task_generation ||
      authority_session_.load(std::memory_order_acquire) !=
          request.authority_session) {
    return false;
  }
  if (request.base_path_tube_pair) {
    const std::shared_ptr<const PathTubePair> live =
        std::atomic_load(&authoritative_path_tube_pair_);
    if (live != request.base_path_tube_pair ||
        live->generation != request.base_path_tube_pair_generation) {
      return false;
    }
  }
  return true;
}

bool PhaseOffsetMatchedAdapter::requestCloudSnapshotUsable(
    const TubeBuildRequest& request) const {
  return config_.tube_source != phase_offset_navigation::TubeSource::ESDF ||
      inspectCloudOccupancyQuery(request.cloud_snapshot,
                                 cloud_occupancy_query_config_).usable;
}

std::shared_ptr<const TubeEpochSnapshot>
PhaseOffsetMatchedAdapter::makeCandidateOnlyEpoch(
    const TubeEpochSnapshot& epoch) const {
  // A map-lag Candidate remains useful provenance-bound geometry evidence,
  // but it must not resemble a current active certificate.  Keep only the
  // Candidate facts/key and erase every Active/current-execution witness.
  std::shared_ptr<TubeEpochSnapshot> masked(new TubeEpochSnapshot(epoch));
  masked->active_profile.reset();
  phase_offset_navigation::TubeEpochStatus& status = masked->epoch_status;
  status.state = phase_offset_navigation::TubeEpochState::WAITING_FOR_CANDIDATE;
  status.disposition = phase_offset_navigation::TubeInstallDisposition::NONE;
  status.current_safety_status =
      phase_offset_navigation::CurrentSafetyStatus::NOT_EVALUATED;
  status.active_available = false;
  status.active_current_validation_valid = false;
  status.active_tube_epoch = 0U;
  status.active_path_source_revision = 0U;
  status.active_map_observation_sequence = 0U;
  status.current_geometry_valid = false;
  status.current_bounds_valid = false;
  status.current_interval_nonempty = false;
  status.current_interval_contains_zero = false;
  status.current_interval_contains_retained_delta = false;
  status.base_centerline_clearance_sufficient = false;
  status.retained_delta_current_inside = false;
  status.current_state_admissible = false;
  status.reference_clearance_sufficient = false;
  status.actual_clearance_sufficient = false;
  status.tracking_within_bound = false;
  status.forward_horizon_sufficient = false;
  status.certificate_denied = false;
  status.transient_blocked = true;
  status.genuine_fatal_invariant = false;
  status.control_failure_reason =
      phase_offset_navigation::ControlFailureReason::NONE;
  status.retained_delta = 0.0;
  status.tracking_error_norm = 0.0;
  status.tracking_error_bound = 0.0;
  status.reference_signed_distance = 0.0;
  status.actual_signed_distance = 0.0;
  status.required_reference_clearance = 0.0;
  status.required_actual_clearance = 0.0;
  status.certified_forward_w = 0.0;
  status.reason = phase_offset_navigation::TubeEpochReason::NONE;
  status.reason_text = "historical map candidate; active view masked";
  return std::shared_ptr<const TubeEpochSnapshot>(masked);
}

bool PhaseOffsetMatchedAdapter::epochMatchesRequest(
    const TubeEpochSnapshot& epoch, const TubeBuildRequest& request) const {
  if (!epoch.active || epoch.task_generation != request.task_generation ||
      task_generation_.load(std::memory_order_acquire) !=
          request.task_generation ||
      epoch.source_revision != request.source_revision ||
      epoch.request_control_sequence > request.control_sequence) {
    return false;
  }
  const auto& status = epoch.epoch_status;
  if (config_.tube_source != phase_offset_navigation::TubeSource::NONE &&
      status.candidate_path_source_revision != request.source_revision) {
    return false;
  }
  if (status.active_available != static_cast<bool>(epoch.active_profile)) {
    return false;
  }
  if (status.active_available &&
      status.active_path_source_revision != request.source_revision) {
    return false;
  }
  if (config_.tube_source != phase_offset_navigation::TubeSource::ESDF) {
    return true;
  }
  // A build owns one immutable, self-consistent snapshot.  Later observations
  // are not an equality key: S3 hands a completed same-source profile to the
  // command thread by sample-and-hold until a later epoch replaces it.
  if (!requestCloudSnapshotUsable(request) ||
      !epoch.map_observation_is_snapshot ||
      status.candidate_map_observation_sequence !=
          epoch.map_observation_sequence) {
    return false;
  }
  // The manager's Active provenance must agree with the frozen epoch that
  // produced it.  This is not compared with a later command snapshot.
  return !status.active_available ||
      status.active_map_observation_sequence == epoch.map_observation_sequence;
}

std::shared_ptr<const TubeEpochSnapshot>
PhaseOffsetMatchedAdapter::matchingEpochForRequest(
    const std::shared_ptr<const TubeBuildRequest>& request) const {
  const std::shared_ptr<const TubeEpochSnapshot> epoch =
      std::atomic_load(&latest_epoch_snapshot_);
  return request && epoch && epochMatchesRequest(*epoch, *request)
      ? epoch : std::shared_ptr<const TubeEpochSnapshot>();
}

bool PhaseOffsetMatchedAdapter::refreshPairFromTimerEpoch(
    const std::shared_ptr<const TubeBuildRequest>& request,
    const std::shared_ptr<const TubeEpochSnapshot>& epoch) {
  TimerPairRefreshPreparation preparation;
  if (!prepareTimerPairRefresh(request, epoch, preparation)) return false;
  std::shared_ptr<const PathTubePair> refreshed;
  return finalizePreparedTimerPairRefresh(preparation, refreshed);
}

bool PhaseOffsetMatchedAdapter::prepareTimerPairRefresh(
    const std::shared_ptr<const TubeBuildRequest>& request,
    const std::shared_ptr<const TubeEpochSnapshot>& epoch,
    TimerPairRefreshPreparation& preparation) {
  preparation = TimerPairRefreshPreparation();
  const phase_offset_navigation::PhaseOffsetRuntimeConfig runtime_config =
      MakeRuntimeConfig(config_);
  if (!request || !epoch || !request->base_path_tube_pair ||
      !epochMatchesRequest(*epoch, *request) || !epoch->full_path_samples ||
      !IsOffsetCertifiedProfile(epoch->active_profile) || epoch->source_revision !=
          request->base_path_tube_pair->source_revision ||
      epoch->epoch_status.active_path_source_revision !=
          request->base_path_tube_pair->source_revision ||
      !PreparedSamplesMatchOwner(*epoch->full_path_samples,
                                 request->base_path_tube_pair->path_owner,
                                 request->base_path_tube_pair->frame_owner) ||
      !ProfileSamplesMatchOwner(*epoch->active_profile,
                                request->base_path_tube_pair->path_owner,
                                runtime_config.tube.regularity_margin,
                                runtime_config.tube.minimum_reference_speed,
                                request->base_path_tube_pair->frame_owner)) {
    return false;
  }

  // The completed epoch retains the build request's immutable profile/map
  // provenance.  Its installation, however, is evaluated against the newest
  // command that still names this exact same authority; a normal selected
  // command may legitimately advance Runtime during construction.
  const std::shared_ptr<const TubeBuildRequest> latest_request =
      std::atomic_load(&latest_build_request_);
  if (!latest_request || !latest_request->active ||
      latest_request->task_generation != request->task_generation ||
      latest_request->source_revision != request->source_revision ||
      latest_request->authority_session != request->authority_session ||
      latest_request->base_path_tube_pair != request->base_path_tube_pair ||
      latest_request->base_path_tube_pair_generation !=
          request->base_path_tube_pair_generation ||
      latest_request->semantic_path_owner !=
          request->base_path_tube_pair->path_owner ||
      !PathStateMatchesOwner(latest_request->current_path,
                             request->base_path_tube_pair->path_owner) ||
      !IsFinite(latest_request->position) ||
      !std::isfinite(latest_request->dt) || latest_request->dt <= 0.0) {
    return false;
  }

  // A transaction pin deliberately does not block tube construction or
  // evidence publication.  The timer never waits for it: a contended command
  // boundary or active H2 lease simply skips this refresh attempt.
  std::unique_lock<std::mutex> lock(runtime_command_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return false;
  if (path_tube_pin_registry_) {
    std::lock_guard<std::mutex> pin_lock(path_tube_pin_registry_->mutex);
    if (path_tube_pin_registry_->active_lease_id != 0U) return false;
  }
  const std::shared_ptr<const PathTubePair> live =
      std::atomic_load(&authoritative_path_tube_pair_);
  if (live != request->base_path_tube_pair ||
      live->generation != request->base_path_tube_pair_generation ||
      live->authority_session != request->authority_session ||
      authority_session_.load(std::memory_order_acquire) !=
          request->authority_session) {
    return false;
  }
  phase_offset_navigation::PhaseOffsetRuntime runtime_snapshot(*runtime_);
  const double expected_retained_delta = runtime_->retainedDelta();
  const phase_offset_core::PortCommand expected_previous_final_port =
      runtime_->previousFinalPort();
  lock.unlock();

  // This is the existing Runtime exact-PWL witness, applied to a local copy.
  // It neither writes live Runtime nor alters the completed epoch/profile.
  phase_offset_navigation::RuntimeDryRunInput dry_run_input;
  dry_run_input.preflight.path = *epoch->full_path_samples;
  dry_run_input.preflight.position = latest_request->position;
  dry_run_input.preflight.path_source_revision = request->source_revision;
  dry_run_input.prepare.current_path = latest_request->current_path;
  dry_run_input.prepare.position = latest_request->position;
  dry_run_input.prepare.tube_view.active_profile = epoch->active_profile;
  dry_run_input.prepare.tube_view.epoch_status = epoch->epoch_status;
  dry_run_input.prepare.dt = latest_request->dt;
  dry_run_input.prepare.future_step = MakeFutureStepContract(
      request->base_path_tube_pair->path_owner, epoch->full_path_samples,
      epoch->active_profile, latest_request->gains,
      runtime_snapshot.config().tube.regularity_margin,
      runtime_snapshot.config().tube.minimum_reference_speed,
      config_.tube_update_period, config_.tube.min_certified_forward_w);
  // The proof checks existing port viability independently of control arming;
  // the following normal command retains the only selection decision.
  dry_run_input.prepare.zero_gate_open = false;
  phase_offset_core::GeometryParams geometry_params;
  geometry_params.regularity_margin =
      runtime_snapshot.config().tube.regularity_margin;
  geometry_params.minimum_reference_speed =
      runtime_snapshot.config().tube.minimum_reference_speed;
  phase_offset_core::GeometryEvaluator evaluator(geometry_params);
  phase_offset_core::PhaseOffsetGeometryState geometry;
  guidance::ReferenceGeometry reference;
  guidance::IsfGuidance base;
  if (!evaluator.evaluate(latest_request->current_path,
                          latest_request->position,
                          runtime_snapshot.retainedDelta(), geometry) ||
      !geometry.valid) {
    return false;
  }
  reference.point = geometry.r;
  reference.tangent = geometry.T;
  reference.derivative_norm = geometry.r_w.norm();
  reference.valid = geometry.valid;
  if (!guidance::IsfReferenceKernel::evaluate(latest_request->position,
                                               reference,
                                               latest_request->gains,
                                               base) ||
      !base.valid) {
    return false;
  }
  dry_run_input.base_guidance_valid = true;
  dry_run_input.base_v_cmd = base.v_cmd;
  dry_run_input.base_w_dot = base.w_dot;
  phase_offset_navigation::RuntimeDryRunResult dry_run;
  if (!runtime_snapshot.dryRun(dry_run_input, dry_run) || !dry_run.valid) {
    return false;
  }
  // Preserve the H2 install boundary for a newer explicit categorical map
  // fact.  The epoch's frozen snapshot remains its construction provenance;
  // this is only the existing current-reference/actual OCCUPIED/OUT_OF_MAP
  // veto against the latest immutable command observation.
  if (LatestCategoricalUnsafe(config_, cloud_occupancy_query_config_,
                              latest_request->cloud_snapshot,
                              latest_request->current_path,
                              latest_request->position,
                              runtime_snapshot.retainedDelta())) {
    return false;
  }

  preparation.build_request = request;
  preparation.latest_request = latest_request;
  preparation.epoch = epoch;
  preparation.expected_pair = live;
  preparation.authority_session = request->authority_session;
  preparation.expected_retained_delta = expected_retained_delta;
  preparation.expected_previous_final_port = expected_previous_final_port;
  preparation.valid = true;
  return true;
}

bool PhaseOffsetMatchedAdapter::finalizePreparedTimerPairRefresh(
    const TimerPairRefreshPreparation& preparation,
    std::shared_ptr<const PathTubePair>& refreshed_pair) {
  refreshed_pair.reset();
  if (!preparation.valid || !preparation.build_request ||
      !preparation.latest_request || !preparation.epoch ||
      !preparation.expected_pair) {
    return false;
  }
  const std::shared_ptr<const TubeBuildRequest>& request =
      preparation.build_request;
  const std::shared_ptr<const TubeEpochSnapshot>& epoch = preparation.epoch;

  std::unique_lock<std::mutex> lock(runtime_command_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return false;
  if (path_tube_pin_registry_) {
    std::lock_guard<std::mutex> pin_lock(path_tube_pin_registry_->mutex);
    if (path_tube_pin_registry_->active_lease_id != 0U) return false;
  }
  const std::shared_ptr<const PathTubePair> live =
      std::atomic_load(&authoritative_path_tube_pair_);
  const std::shared_ptr<const TubeBuildRequest> latest_request =
      std::atomic_load(&latest_build_request_);
  if (live != preparation.expected_pair ||
      live != request->base_path_tube_pair ||
      live->generation != request->base_path_tube_pair_generation ||
      live->authority_session != preparation.authority_session ||
      authority_session_.load(std::memory_order_acquire) !=
          preparation.authority_session ||
      request->task_generation !=
          task_generation_.load(std::memory_order_acquire) ||
      preparation.latest_request->task_generation != request->task_generation ||
      latest_request != preparation.latest_request ||
      !BitsEqual(runtime_->retainedDelta(),
                 preparation.expected_retained_delta) ||
      !BitsEqual(runtime_->previousFinalPort(),
                 preparation.expected_previous_final_port)) {
    return false;
  }
  std::shared_ptr<PathTubePair> refreshed(new PathTubePair(*live));
  refreshed->generation = ++next_path_tube_pair_generation_;
  refreshed->authority_session = preparation.authority_session;
  refreshed->map_observation_sequence = epoch->map_observation_sequence;
  refreshed->map_observation_is_snapshot = epoch->map_observation_is_snapshot;
  refreshed->frozen_cloud_occupancy_snapshot = request->cloud_snapshot;
  refreshed->full_path_samples = epoch->full_path_samples;
  refreshed->active_profile = epoch->active_profile;
  refreshed->epoch_status = epoch->epoch_status;
  std::shared_ptr<TubeEpochSnapshot> refreshed_epoch(
      new TubeEpochSnapshot(*epoch));
  refreshed->epoch_snapshot =
      std::shared_ptr<const TubeEpochSnapshot>(refreshed_epoch);
  const std::shared_ptr<const PathTubePair> immutable_refreshed(refreshed);
  std::shared_ptr<const PathTubePair> expected = live;
  if (!std::atomic_compare_exchange_strong(&authoritative_path_tube_pair_,
                                           &expected,
                                           immutable_refreshed)) {
    return false;
  }
  refreshed_pair = immutable_refreshed;
  return true;
}

bool PhaseOffsetMatchedAdapter::buildTubeEpoch(
    const std::shared_ptr<const TubeBuildRequest>& request,
    TubeEpochSnapshot& snapshot) {
  snapshot = TubeEpochSnapshot();
  // A request anchored to an installed H2 pair is a same-path refresh.  It
  // must not change the persistent timer manager: a stale completion can be
  // discarded by the pair CAS, but its local construction must also be unable
  // to perturb the next current request's Candidate/Active/counter state.
  const bool pair_refresh = request && request->base_path_tube_pair;
  if (!request || !request->active ||
      request->task_generation != timer_task_generation_ ||
      request->task_generation !=
          task_generation_.load(std::memory_order_acquire) ||
      (request->semantic_path_owner &&
       !FrameMatchesRevision(request->frame_owner,
                             request->source_revision)) ||
      (!pair_refresh && !tube_epoch_manager_)) {
    return false;
  }
  snapshot.active = true;
  snapshot.task_generation = request->task_generation;
  snapshot.request_control_sequence = request->control_sequence;
  snapshot.build_sequence = ++timer_build_sequence_;
  snapshot.source_revision = request->source_revision;
  snapshot.map_observation_sequence = request->map_observation_sequence;
  snapshot.map_observation_is_snapshot = request->map_observation_is_snapshot;
  snapshot.request_stamp = request->stamp;
  snapshot.candidate_build_w = request->current_path.w;

  PathSamples sampled;
  const bool sampled_ok = collectSamples(*request, sampled);
  if (sampled_ok) {
    if (pair_refresh) {
      snapshot.full_path_samples =
          std::make_shared<const PathSamples>(sampled);
    } else {
      cached_full_path_samples_ = sampled;
      cached_path_source_revision_ = request->source_revision;
      have_cached_path_ = true;
      snapshot.full_path_samples =
          std::make_shared<const PathSamples>(cached_full_path_samples_);
    }
  }
  PathSamples preview;
  if (sampled_ok) {
    // Ordinary epochs use the same immutable semantic owner as H2 prepared
    // epochs.  Partition this local preview at every owner segment boundary
    // before adaptive refinement; otherwise a midpoint-adaptive cell can
    // straddle the old-prefix/C2/tail seam and the exact owner cell query
    // correctly fails closed, forcing the historical resolution inset.
    bool owner_aligned_preview = false;
    if (request->semantic_path_owner &&
        !request->semantic_path_owner->empty()) {
      const double preview_start = std::max(
          request->semantic_path_owner->startW(),
          request->current_path.w - config_.tube.back_w);
      const double preview_end = std::min(
          request->semantic_path_owner->endW(),
          request->current_path.w + config_.tube.lookahead_w);
      MatchedAdapterPathSamples aligned_preview;
      if (BuildOwnerAlignedTubePreview(
              request->semantic_path_owner, request->frame_owner, sampled,
              preview_start,
              preview_end, request->current_path.w, aligned_preview)) {
        preview = std::move(aligned_preview);
        owner_aligned_preview = true;
      }
    }
    if (!owner_aligned_preview) {
      makePreview(sampled, request->current_path, preview);
    }
  }

  phase_offset_navigation::TubeEpochUpdateInput epoch_input;
  epoch_input.source = config_.tube_source;
  epoch_input.current_path = request->current_path;
  epoch_input.preview_path = preview;
  epoch_input.actual_position = request->position;
  epoch_input.retained_delta = request->retained_delta;
  epoch_input.authority_request = request->authority_request;
  epoch_input.path_source_revision = request->source_revision;
  epoch_input.path_state_query = MakeTimerPathStateQuery(
      request->semantic_path_owner, snapshot.full_path_samples,
      request->frame_owner);
  epoch_input.path_cell_bound_query = MakeTimerPathCellBoundQuery(
      request->semantic_path_owner, request->frame_owner);

  bool raw_query_injected = false;
  phase_offset_navigation::RawOccupancyQuery categorical_query;
  CloudOccupancyQueryStatus cloud_status;
  if (config_.tube_source == phase_offset_navigation::TubeSource::ESDF) {
    cloud_status = inspectCloudOccupancyQuery(
        request->cloud_snapshot, cloud_occupancy_query_config_);
    snapshot.cloud_status = cloud_status;
    if (!pair_refresh) {
      latest_cloud_occupancy_query_status_ = cloud_status;
    }
    categorical_query = makeCloudOccupancyQuery(
        request->cloud_snapshot, cloud_occupancy_query_config_);
    epoch_input.cloud_clearance_query = makeCloudOccupancyClearanceQuery(
        request->cloud_snapshot, cloud_occupancy_query_config_);
    epoch_input.cloud_snapshot_resolution = request->cloud_snapshot &&
        std::isfinite(request->cloud_snapshot->resolution)
        ? request->cloud_snapshot->resolution : 0.0;
    epoch_input.map_observation_sequence = request->map_observation_sequence;
    epoch_input.map_observation_is_snapshot = true;
    raw_query_injected = true;
  }

  // The stack-local manager is deliberately selected before update rather
  // than copied from the persistent one: copying would retain Candidate /
  // Active history and make a stale base request influence a later refresh.
  phase_offset_navigation::TubeEpochManager local_manager(
      MakeEpochConfig(config_));
  phase_offset_navigation::TubeEpochManager* const manager = pair_refresh
      ? &local_manager : tube_epoch_manager_.get();
  phase_offset_navigation::TubeEpochUpdateResult result;
  const bool update_result = manager->update(epoch_input, result);
  if (config_.tube_source == phase_offset_navigation::TubeSource::ESDF) {
    LogZeroBaselineAttribution(result);
  }
  snapshot.epoch_status = result.status;
  snapshot.candidate_profile =
      std::make_shared<const phase_offset_navigation::TubeProfile>(
          result.candidate_profile);
  if (result.status.active_available) {
    if (pair_refresh) {
      snapshot.active_profile =
          std::make_shared<const phase_offset_navigation::TubeProfile>(
              result.active_profile);
    } else if (timer_active_profile_ &&
               timer_installed_active_epoch_ == result.status.active_tube_epoch) {
      snapshot.active_profile = timer_active_profile_;
    } else {
      timer_active_profile_ =
          std::make_shared<const phase_offset_navigation::TubeProfile>(
              result.active_profile);
      timer_installed_active_epoch_ = result.status.active_tube_epoch;
      snapshot.active_profile = timer_active_profile_;
    }
  } else if (!pair_refresh) {
    timer_active_profile_.reset();
    timer_installed_active_epoch_ = 0U;
  }
  if (config_.tube_source == phase_offset_navigation::TubeSource::ESDF) {
    RawCandidateDiagnosticsInput diagnostics_input;
    diagnostics_input.raw_source_configured = true;
    diagnostics_input.raw_storage_ready = false;
    diagnostics_input.raw_query_injected = raw_query_injected;
    diagnostics_input.tube_update_due = true;
    diagnostics_input.epoch_status = result.status;
    diagnostics_input.candidate_profile = &result.candidate_profile;
    diagnostics_input.current_path = request->current_path;
    diagnostics_input.current_w = request->current_path.w;
    diagnostics_input.actual_position = request->position;
    diagnostics_input.raw_occupancy_query = categorical_query;
    diagnostics_input.raw_ray_step = config_.tube.cross_section.ray_step;
    snapshot.raw_candidate_diagnostics =
        makeRawCandidateDiagnostics(diagnostics_input);
    snapshot.raw_candidate_diagnostics_generated = true;

    CloudSnapshotDiagnosticsInput cloud_diagnostics_input;
    cloud_diagnostics_input.query_status = cloud_status;
    cloud_diagnostics_input.query_config = cloud_occupancy_query_config_;
    cloud_diagnostics_input.margins = config_.tube.cross_section.margins;
    cloud_diagnostics_input.candidate_profile = &result.candidate_profile;
    cloud_diagnostics_input.current_w = request->current_path.w;
    cloud_diagnostics_input.current_path = request->current_path;
    cloud_diagnostics_input.actual_position = request->position;
    cloud_diagnostics_input.categorical_query = categorical_query;
    cloud_diagnostics_input.ray_step = config_.tube.cross_section.ray_step;
    snapshot.cloud_snapshot_diagnostics =
        makeCloudSnapshotDiagnostics(cloud_diagnostics_input);
    snapshot.cloud_snapshot_diagnostics_generated = true;
  }
  snapshot.completion_stamp = ros::Time::isValid()
      ? ros::Time::now() : request->stamp;
  return update_result;
}

bool PhaseOffsetMatchedAdapter::buildPreparedTubeEpoch(
    const TubeBuildRequest& request,
    const MatchedAdapterPathSamples& prepared_path,
    const double prepared_start_w,
    const double prepared_end_w,
    const double future_seam_w,
    const double existing_future_horizon_end_w,
    PreparedTubeBuildResult& result,
    phase_offset_navigation::TubeEpochUpdateResult*
        temporary_epoch_result,
    std::string* const temporary_failure_layer,
    const CanonicalOwnerStateReuse* const canonical_owner_state) const {
  result = PreparedTubeBuildResult();
  if (temporary_failure_layer) temporary_failure_layer->clear();
  if (temporary_epoch_result) {
    *temporary_epoch_result = phase_offset_navigation::TubeEpochUpdateResult();
  }
  const double captured_w0 = request.current_path.w;
  phase_offset_core::PathDifferentialState owner_current_path;
  if (!request.active || !StateFiniteAndValid(request.current_path) ||
      !IsFinite(request.position) || !IsFinite(request.retained_delta) ||
      request.source_revision == 0U || !request.semantic_path_owner ||
      request.semantic_path_owner->empty() ||
      !IsFinite(prepared_start_w) || !IsFinite(prepared_end_w) ||
      !IsFinite(future_seam_w) || !IsFinite(existing_future_horizon_end_w) ||
      !NearlyEqual(prepared_path.front().w, prepared_start_w) ||
      !NearlyEqual(prepared_path.back().w, prepared_end_w) ||
      prepared_end_w < prepared_start_w ||
      captured_w0 < prepared_start_w - kPreparedCoverageTolerance ||
      captured_w0 > prepared_end_w + kPreparedCoverageTolerance ||
      future_seam_w <= captured_w0 + kPreparedCoverageTolerance ||
      future_seam_w < prepared_start_w - kPreparedCoverageTolerance ||
      future_seam_w > prepared_end_w + kPreparedCoverageTolerance ||
      existing_future_horizon_end_w < future_seam_w -
          kPreparedCoverageTolerance ||
      existing_future_horizon_end_w > prepared_end_w +
          kPreparedCoverageTolerance ||
      (request.semantic_path_owner &&
       !FrameMatchesRevision(request.frame_owner, request.source_revision))) {
    if (temporary_failure_layer) {
      *temporary_failure_layer = "tube_build_input_precondition";
    }
    return false;
  }
  const bool canonical_reuse_valid = canonical_owner_state &&
      canonical_owner_state->valid &&
      canonical_owner_state->owner == request.semantic_path_owner &&
      canonical_owner_state->verified_samples == &prepared_path &&
      canonical_owner_state->task_generation == request.task_generation &&
      canonical_owner_state->source_revision == request.source_revision &&
      canonical_owner_state->authority_session == request.authority_session &&
      canonical_owner_state->map_observation_sequence ==
          request.map_observation_sequence &&
      canonical_owner_state->map_observation_is_snapshot ==
          request.map_observation_is_snapshot &&
      ExactDoubleBits(canonical_owner_state->state.w, captured_w0) &&
      PathStatesEquivalent(request.current_path,
                           canonical_owner_state->state);
  if (canonical_owner_state && !canonical_reuse_valid) {
    if (temporary_failure_layer) {
      *temporary_failure_layer = "tube_build_owner_evaluate";
    }
    return false;
  }
  if (canonical_reuse_valid) {
    owner_current_path = canonical_owner_state->state;
  } else if (!EvaluateOwnerState(request.semantic_path_owner, captured_w0,
                                 owner_current_path, request.frame_owner) ||
             !PathStatesEquivalent(request.current_path,
                                   owner_current_path)) {
    if (temporary_failure_layer) {
      *temporary_failure_layer = "tube_build_owner_evaluate";
    }
    return false;
  }
  if (config_.tube_source == phase_offset_navigation::TubeSource::ESDF) {
    const CloudOccupancyQueryStatus cloud = inspectCloudOccupancyQuery(
        request.cloud_snapshot, cloud_occupancy_query_config_);
    if (!request.map_observation_is_snapshot || !cloud.usable ||
        cloud.observation_sequence != request.map_observation_sequence) {
      if (temporary_failure_layer) {
        *temporary_failure_layer = "tube_build_map_precondition";
      }
      return false;
    }
  } else if (request.map_observation_is_snapshot ||
             request.map_observation_sequence != 0U) {
    if (temporary_failure_layer) {
      *temporary_failure_layer = "tube_build_map_precondition";
    }
    return false;
  }

  CanonicalOwnerStateReuse local_owner_reuse;
  const CanonicalOwnerStateReuse* preview_owner_reuse =
      canonical_reuse_valid ? canonical_owner_state : nullptr;
  if (!canonical_reuse_valid) {
    if (!PreparedSamplesMatchOwner(prepared_path,
                                   request.semantic_path_owner,
                                   request.frame_owner)) {
      if (temporary_failure_layer) {
        *temporary_failure_layer = "tube_build_owner_evaluate";
      }
      return false;
    }
    local_owner_reuse.owner = request.semantic_path_owner;
    local_owner_reuse.state = owner_current_path;
    local_owner_reuse.verified_samples = &prepared_path;
    local_owner_reuse.task_generation = request.task_generation;
    local_owner_reuse.source_revision = request.source_revision;
    local_owner_reuse.authority_session = request.authority_session;
    local_owner_reuse.map_observation_sequence =
        request.map_observation_sequence;
    local_owner_reuse.map_observation_is_snapshot =
        request.map_observation_is_snapshot;
    local_owner_reuse.valid = true;
    preview_owner_reuse = &local_owner_reuse;
  }

  // Retain the complete sampled owner in the pair, but build the expensive
  // tube only over this transaction's required executable range.  Requiring
  // the filter to certify the unrelated remainder of a long path would
  // silently replace the existing lookahead semantics with a whole-path one.
  MatchedAdapterPathSamples prepared_preview;
  if (!BuildOwnerAlignedTubePreview(
          request.semantic_path_owner, request.frame_owner, prepared_path,
          captured_w0,
          existing_future_horizon_end_w, captured_w0, prepared_preview,
          preview_owner_reuse)) {
    if (temporary_failure_layer) {
      *temporary_failure_layer = "tube_build_owner_evaluate";
    }
    return false;
  }

  // The prepared range is transaction input, not a configuration override.
  // Each preview sample has been evaluated by the new immutable owner.
  phase_offset_navigation::TubeEpochUpdateInput epoch_input;
  epoch_input.source = config_.tube_source;
  epoch_input.current_path = owner_current_path;
  epoch_input.preview_path = prepared_preview;
  epoch_input.actual_position = request.position;
  epoch_input.retained_delta = request.retained_delta;
  epoch_input.authority_request = request.authority_request;
  epoch_input.path_source_revision = request.source_revision;
  epoch_input.map_observation_sequence = request.map_observation_sequence;
  epoch_input.map_observation_is_snapshot = request.map_observation_is_snapshot;
  const std::shared_ptr<const MatchedAdapterPathSamples> owned_samples(
      new MatchedAdapterPathSamples(prepared_path));
  epoch_input.path_state_query = MakeTimerPathStateQuery(
      request.semantic_path_owner, owned_samples, request.frame_owner);
  epoch_input.path_cell_bound_query = MakeTimerPathCellBoundQuery(
      request.semantic_path_owner, request.frame_owner);
  if (config_.tube_source == phase_offset_navigation::TubeSource::ESDF) {
    epoch_input.cloud_clearance_query = makeCloudOccupancyClearanceQuery(
        request.cloud_snapshot, cloud_occupancy_query_config_);
    epoch_input.cloud_snapshot_resolution = request.cloud_snapshot &&
        std::isfinite(request.cloud_snapshot->resolution)
        ? request.cloud_snapshot->resolution : 0.0;
  }

  // A fresh stack manager is the isolation boundary: it cannot alter the
  // timer-owned Candidate/Active slots or any of their counters.
  phase_offset_navigation::TubeEpochManager staged_manager(MakeEpochConfig(config_));
  phase_offset_navigation::TubeEpochUpdateResult staged;
  const bool staged_update_ok = staged_manager.update(epoch_input, staged);
  if (config_.tube_source == phase_offset_navigation::TubeSource::ESDF) {
    LogZeroBaselineAttribution(staged);
  }
  if (temporary_epoch_result) *temporary_epoch_result = staged;
  const bool update_status_ok =
      staged_update_ok &&
      staged.status.candidate_complete &&
      staged.status.active_available &&
      staged.status.active_current_validation_valid &&
      staged.status.candidate_path_source_revision == request.source_revision &&
      staged.status.active_path_source_revision == request.source_revision &&
      staged.status.candidate_map_observation_sequence ==
          request.map_observation_sequence &&
      staged.status.active_map_observation_sequence ==
          request.map_observation_sequence &&
      staged.status.map_observation_is_snapshot ==
          request.map_observation_is_snapshot &&
      staged.active_profile.classification ==
          phase_offset_navigation::TubeProfileClassification::OFFSET_CERTIFIED &&
      staged.active_profile.source_revision == request.source_revision &&
      staged.active_profile.source == config_.tube_source &&
      ProfileStructurallyCoversPreparedRange(
          staged.active_profile, captured_w0, future_seam_w,
          existing_future_horizon_end_w) &&
      ProfileSamplesMatchOwner(
          staged.active_profile, request.semantic_path_owner,
          MakeRuntimeConfig(config_).tube.regularity_margin,
          MakeRuntimeConfig(config_).tube.minimum_reference_speed,
          request.frame_owner);
  if (!update_status_ok) {
    if (temporary_failure_layer) {
      if (!staged_update_ok || !staged.status.candidate_complete ||
          !staged.status.active_available ||
          !staged.status.active_current_validation_valid ||
          staged.status.candidate_path_source_revision != request.source_revision ||
          staged.status.active_path_source_revision != request.source_revision ||
          staged.status.candidate_map_observation_sequence !=
              request.map_observation_sequence ||
          staged.status.active_map_observation_sequence !=
              request.map_observation_sequence ||
          staged.status.map_observation_is_snapshot !=
              request.map_observation_is_snapshot ||
          staged.active_profile.classification !=
              phase_offset_navigation::TubeProfileClassification::OFFSET_CERTIFIED ||
          staged.active_profile.source_revision != request.source_revision ||
          staged.active_profile.source != config_.tube_source) {
        *temporary_failure_layer = "tube_update_status";
      } else if (!ProfileStructurallyCoversPreparedRange(
                     staged.active_profile, captured_w0, future_seam_w,
                     existing_future_horizon_end_w)) {
        *temporary_failure_layer = "tube_profile_coverage";
      } else {
        *temporary_failure_layer = "tube_profile_owner_match";
      }
    }
    return false;
  }

  result.complete = true;
  result.source_revision = request.source_revision;
  result.map_observation_sequence = request.map_observation_sequence;
  result.map_observation_is_snapshot = request.map_observation_is_snapshot;
  result.semantic_path_owner = request.semantic_path_owner;
  result.frame_owner = request.frame_owner;
  result.frozen_cloud_occupancy_snapshot = request.cloud_snapshot;
  result.prepared_start_w = prepared_path.front().w;
  result.prepared_end_w = prepared_path.back().w;
  result.captured_w0 = captured_w0;
  result.future_seam_w = future_seam_w;
  result.existing_future_horizon_end_w = existing_future_horizon_end_w;
  result.full_path_samples = owned_samples;
  result.active_profile = std::make_shared<const phase_offset_navigation::TubeProfile>(
      staged.active_profile);
  result.epoch_status = staged.status;
  return true;
}

PathTubePairStageFailure
PhaseOffsetMatchedAdapter::classifyTubeUpdateStatusStageFailure(
    const phase_offset_navigation::TubeEpochUpdateResult& staged) {
  // The epoch result is already a frozen local staging value.  The manager's
  // first-false contract classifies *only* the three candidate completion
  // bits emitted by that update, in build-pipeline order.  Profile pointer,
  // provenance, current validation, classification and all other status
  // denials remain coverage/precondition attribution at their existing
  // callers; none may be relabelled as a raw/filter/validator failure.
  if (!staged.status.candidate_raw_complete) {
    return PathTubePairStageFailure::TUBE_RAW_BUILD;
  }
  if (!staged.status.candidate_filtered_complete) {
    return PathTubePairStageFailure::TUBE_FILTER;
  }
  if (!staged.status.candidate_complete) {
    return PathTubePairStageFailure::TUBE_SURFACE_VALIDATOR;
  }
  return PathTubePairStageFailure::TUBE_BUILD_PRECONDITION;
}

bool PhaseOffsetMatchedAdapter::dryRunPreparedRuntime(
    const PreparedTubeBuildResult& prepared_tube,
    const phase_offset_core::PathDifferentialState& current_path,
    const Eigen::Vector3d& position,
    const guidance::IsfGains& gains,
    const double dt,
    phase_offset_navigation::RuntimeDryRunResult& result) const {
  if (!runtime_) return false;
  phase_offset_navigation::PhaseOffsetRuntime runtime_snapshot(
      MakeRuntimeConfig(config_));
  {
    std::lock_guard<std::mutex> lock(runtime_command_mutex_);
    if (!runtime_) return false;
    runtime_snapshot = *runtime_;
  }
  return dryRunPreparedRuntime(runtime_snapshot, prepared_tube, current_path,
                               position, gains, dt, result);
}

bool PhaseOffsetMatchedAdapter::dryRunPreparedRuntime(
    phase_offset_navigation::PhaseOffsetRuntime& runtime_snapshot,
    const PreparedTubeBuildResult& prepared_tube,
    const phase_offset_core::PathDifferentialState& current_path,
    const Eigen::Vector3d& position,
    const guidance::IsfGains& gains,
    const double dt,
    phase_offset_navigation::RuntimeDryRunResult& result) const {
  result = phase_offset_navigation::RuntimeDryRunResult();
  phase_offset_core::PathDifferentialState owner_current_path;
  if (!runtime_snapshot.configurationValid() || !prepared_tube.complete ||
      !prepared_tube.full_path_samples ||
      !prepared_tube.active_profile || !prepared_tube.semantic_path_owner ||
      ((prepared_tube.frame_owner == nullptr) &&
       (prepared_tube.active_profile->frame_revision != 0U)) ||
      (prepared_tube.frame_owner &&
       !FrameMatchesRevision(prepared_tube.frame_owner,
                             prepared_tube.source_revision)) ||
      current_path.w < prepared_tube.prepared_start_w -
          kPreparedCoverageTolerance ||
      current_path.w >= prepared_tube.future_seam_w ||
      !position.allFinite() ||
      !std::isfinite(dt) || dt <= 0.0) {
    return false;
  }
  if (!EvaluateOwnerState(prepared_tube.semantic_path_owner, current_path.w,
                          owner_current_path, prepared_tube.frame_owner) ||
      !PathStatesEquivalent(current_path, owner_current_path)) {
    return false;
  }
  if (prepared_tube.source_revision == 0U ||
      prepared_tube.epoch_status.candidate_path_source_revision !=
          prepared_tube.source_revision ||
      prepared_tube.epoch_status.active_path_source_revision !=
          prepared_tube.source_revision ||
      !prepared_tube.epoch_status.active_available ||
      !prepared_tube.epoch_status.active_current_validation_valid ||
      prepared_tube.active_profile->source_revision !=
          prepared_tube.source_revision ||
      !ProfileStructurallyCoversPreparedRange(
          *prepared_tube.active_profile, prepared_tube.captured_w0,
          prepared_tube.future_seam_w,
          prepared_tube.existing_future_horizon_end_w) ||
      !ProfileSamplesMatchOwner(*prepared_tube.active_profile,
                                prepared_tube.semantic_path_owner,
                                runtime_snapshot.config().tube.regularity_margin,
                                runtime_snapshot.config().tube.minimum_reference_speed,
                                prepared_tube.frame_owner)) {
    return false;
  }
  if (config_.tube_source == phase_offset_navigation::TubeSource::ESDF) {
    const CloudOccupancyQueryStatus cloud = inspectCloudOccupancyQuery(
        prepared_tube.frozen_cloud_occupancy_snapshot,
        cloud_occupancy_query_config_);
    if (!prepared_tube.map_observation_is_snapshot || !cloud.usable ||
        cloud.observation_sequence != prepared_tube.map_observation_sequence ||
        prepared_tube.epoch_status.candidate_map_observation_sequence !=
            prepared_tube.map_observation_sequence ||
        prepared_tube.epoch_status.active_map_observation_sequence !=
            prepared_tube.map_observation_sequence ||
        !prepared_tube.epoch_status.map_observation_is_snapshot) {
      return false;
    }
  } else if (prepared_tube.map_observation_is_snapshot ||
             prepared_tube.map_observation_sequence != 0U ||
             prepared_tube.frozen_cloud_occupancy_snapshot) {
    return false;
  }
  phase_offset_navigation::RuntimeDryRunInput input;
  input.preflight.path = *prepared_tube.full_path_samples;
  input.preflight.position = position;
  input.preflight.path_source_revision = prepared_tube.source_revision;
  input.prepare.current_path = owner_current_path;
  input.prepare.position = position;
  input.prepare.tube_view.active_profile = prepared_tube.active_profile;
  input.prepare.tube_view.epoch_status = prepared_tube.epoch_status;
  input.prepare.dt = dt;
  input.prepare.future_step = MakeFutureStepContract(
      prepared_tube.semantic_path_owner, prepared_tube.full_path_samples,
      prepared_tube.active_profile, gains,
      runtime_snapshot.config().tube.regularity_margin,
      runtime_snapshot.config().tube.minimum_reference_speed,
      config_.tube_update_period, config_.tube.min_certified_forward_w);
  // Staging must evaluate the exact existing port independently of selection;
  // the caller performs the real gate/commit decision later.
  input.prepare.zero_gate_open = false;
  guidance::ReferenceGeometry reference;
  phase_offset_core::PhaseOffsetGeometryState geometry;
  phase_offset_core::GeometryParams geometry_params;
  geometry_params.regularity_margin =
      runtime_snapshot.config().tube.regularity_margin;
  geometry_params.minimum_reference_speed =
      runtime_snapshot.config().tube.minimum_reference_speed;
  phase_offset_core::GeometryEvaluator evaluator(geometry_params);
  if (!evaluator.evaluate(owner_current_path, position,
                          runtime_snapshot.retainedDelta(), geometry)) {
    return false;
  }
  reference.point = geometry.r;
  reference.tangent = geometry.T;
  reference.derivative_norm = geometry.r_w.norm();
  reference.valid = geometry.valid;
  guidance::IsfGuidance base;
  input.base_guidance_valid = guidance::IsfReferenceKernel::evaluate(
      position, reference, gains, base) && base.valid;
  input.base_v_cmd = base.v_cmd;
  input.base_w_dot = base.w_dot;
  return runtime_snapshot.dryRun(input, result);
}

bool PhaseOffsetMatchedAdapter::stagePathTubePair(
    const std::shared_ptr<const PathTubePair>& expected_pair,
    const std::shared_ptr<const ContinuousPhasePath>& new_path_owner,
    const MatchedAdapterPathSamples& new_path_samples,
    const double captured_w0,
    const double future_seam_w,
    const double existing_future_horizon_end_w,
    const Eigen::Vector3d& position,
    const guidance::IsfGains& gains,
    const double dt,
    const std::shared_ptr<const plan_env::CloudOccupancySnapshot>&
        frozen_cloud_occupancy_snapshot,
    PathTubePairTransaction& transaction,
    const std::uint64_t authority_session,
    const PathTubePairPinCapture* const expected_capture,
    const std::uint64_t pin_lease_id,
    PathTubePairStageFailure* const temporary_failure) {
  transaction = PathTubePairTransaction();
  if (temporary_failure) *temporary_failure = PathTubePairStageFailure::NONE;
  const auto stage_first_false =
      [](const char* const,
         const phase_offset_navigation::TubeEpochUpdateResult* const) {
        return false;
      };
  const bool prepared_samples_owner_valid = new_path_owner &&
      !new_path_owner->empty() &&
      PreparedSamplesMatchOwner(new_path_samples, new_path_owner);
  if (!runtime_ || !new_path_owner || new_path_owner->empty() ||
      !prepared_samples_owner_valid ||
      !IsFinite(captured_w0) || !IsFinite(future_seam_w) ||
      !IsFinite(existing_future_horizon_end_w) ||
      future_seam_w <= captured_w0 + kPreparedCoverageTolerance ||
      existing_future_horizon_end_w + kPreparedCoverageTolerance <
          future_seam_w ||
      new_path_samples.front().w > captured_w0 +
          kPreparedCoverageTolerance ||
      new_path_samples.back().w + kPreparedCoverageTolerance <
          existing_future_horizon_end_w) {
    if (temporary_failure) {
      *temporary_failure = PathTubePairStageFailure::INPUT_PRECONDITION;
    }
    return stage_first_false("input_precondition", nullptr);
  }
  // Bootstrap has no old authority and therefore no pin.  Every replacement
  // must bind to the capture and monotonic lease obtained before seam work.
  if (expected_pair) {
    if (!expected_capture || !CaptureMatchesPair(*expected_capture,
                                                 expected_pair) ||
        !ActiveLeaseMatchesCapture(path_tube_pin_registry_,
                                   *expected_capture, pin_lease_id)) {
      if (temporary_failure) {
        *temporary_failure = PathTubePairStageFailure::TRANSACTION_PRECONDITION;
      }
      return stage_first_false("transaction_precondition", nullptr);
    }
  } else if (expected_capture || pin_lease_id != 0U) {
    if (temporary_failure) {
      *temporary_failure = PathTubePairStageFailure::TRANSACTION_PRECONDITION;
    }
    return stage_first_false("transaction_precondition", nullptr);
  }

  // Capture Runtime state once under its command boundary.  The local staged
  // manager below owns all expensive work and never observes/mutates live
  // source bookkeeping or Runtime state.
  phase_offset_core::PortCommand previous_port;
  double retained_delta = 0.0;
  phase_offset_navigation::TubeBounds authority_request;
  std::uint64_t captured_task_generation = 0U;
  std::uint64_t captured_authority_session = 0U;
  std::uint64_t staged_revision = 0U;
  phase_offset_navigation::PhaseOffsetRuntime runtime_snapshot(
      MakeRuntimeConfig(config_));
  // A replacement successor must inherit the exact predecessor authority's
  // executed_N.  The seed is projected in the successor composite's start
  // plane (the captured predecessor state), sign-aligned to that immutable
  // predecessor vector, and only then used to construct one immutable
  // successor frame.  The future seam is not the predecessor chaining point.
  Eigen::Vector3d executed_seed = Eigen::Vector3d::Zero();
  bool have_executed_seed = false;
  std::shared_ptr<const phase_offset_navigation::ActiveReferenceSnapshot>
      successor_seed_authority;
  {
    std::lock_guard<std::mutex> lock(runtime_command_mutex_);
    const std::shared_ptr<const PathTubePair> live =
        std::atomic_load(&authoritative_path_tube_pair_);
    if (live != expected_pair) {
      if (temporary_failure) {
        *temporary_failure =
            PathTubePairStageFailure::PAIR_SESSION_RUNTIME_SNAPSHOT;
      }
      return stage_first_false("pair_session_runtime_snapshot", nullptr);
    }
    if (expected_pair &&
        (!CaptureMatchesPair(*expected_capture, live) ||
         !ActiveLeaseMatchesCapture(path_tube_pin_registry_,
                                    *expected_capture, pin_lease_id))) {
      if (temporary_failure) {
        *temporary_failure =
            PathTubePairStageFailure::PAIR_SESSION_RUNTIME_SNAPSHOT;
      }
      return stage_first_false("pair_session_runtime_snapshot", nullptr);
    }
    if (!expected_pair && runtime_->hasExecutedOffsetAuthority()) {
      if (temporary_failure) {
        *temporary_failure =
            PathTubePairStageFailure::PAIR_SESSION_RUNTIME_SNAPSHOT;
      }
      return stage_first_false("pair_session_runtime_snapshot", nullptr);
    }
    const std::uint64_t live_session =
        authority_session_.load(std::memory_order_acquire);
    if (authority_session != 0U && live_session != authority_session) {
      if (temporary_failure) {
        *temporary_failure =
            PathTubePairStageFailure::PAIR_SESSION_RUNTIME_SNAPSHOT;
      }
      return stage_first_false("pair_session_runtime_snapshot", nullptr);
    }
    captured_task_generation =
        task_generation_.load(std::memory_order_acquire);
    captured_authority_session = authority_session == 0U
        ? live_session : authority_session;
    // The pin protects exact pair ownership, not an obsolete Runtime sample.
    // A normal selected command can legally advance Runtime while C2/tube
    // construction runs.  Build/dry-run from the latest state observed under
    // this mutex; final commit will independently require its own latest bits.
    retained_delta = runtime_->retainedDelta();
    if (config_.tube_source == phase_offset_navigation::TubeSource::ESDF &&
        !makeAuthorityRequest(retained_delta, authority_request)) {
      if (temporary_failure) {
        *temporary_failure = PathTubePairStageFailure::TUBE_BUILD_PRECONDITION;
      }
      return stage_first_false("tube_build_input_precondition", nullptr);
    }
    previous_port = runtime_->previousFinalPort();
    runtime_snapshot = *runtime_;
    if (expected_pair) {
      successor_seed_authority = execution_authority_.snapshotPtr();
      const bool authority_is_active = successor_seed_authority &&
          successor_seed_authority->valid &&
          successor_seed_authority->owner_mode !=
              phase_offset_navigation::ActiveReferenceOwnerMode::NONE &&
          successor_seed_authority->owner_mode !=
              phase_offset_navigation::ActiveReferenceOwnerMode::PLANNER_ONLY;
      // Bootstrap and planner-only replacement have no authenticated
      // predecessor normal.  They intentionally use the deterministic
      // least-parallel-axis seed in ContinuousPhaseNormalFrame.  Only an
      // active immutable authority is subject to the strict predecessor
      // identity checks below.
      if (authority_is_active) {
        if (successor_seed_authority->authority_session !=
                captured_authority_session ||
            successor_seed_authority->sequence == 0U ||
            successor_seed_authority->planner_path_revision !=
                expected_pair->source_revision ||
            successor_seed_authority->executed_path_revision !=
                expected_pair->path_revision ||
            successor_seed_authority->frame_revision !=
                expected_pair->frame_revision ||
            std::abs(successor_seed_authority->w - captured_w0) > 1e-12 ||
            !(std::abs(successor_seed_authority->delta - retained_delta) <=
                  1e-12 ||
              std::abs(successor_seed_authority->proposed_next_delta -
                       retained_delta) <= 1e-12) ||
            !(std::abs(successor_seed_authority->u_prev.u_w -
                       previous_port.u_w) <= 1e-12 ||
              std::abs(successor_seed_authority->proposed_next_u_prev.u_w -
                       previous_port.u_w) <= 1e-12) ||
            !(std::abs(successor_seed_authority->u_prev.u_delta -
                       previous_port.u_delta) <= 1e-12 ||
              std::abs(successor_seed_authority->proposed_next_u_prev.u_delta -
                       previous_port.u_delta) <= 1e-12)) {
          if (temporary_failure) *temporary_failure =
              PathTubePairStageFailure::PAIR_SESSION_RUNTIME_SNAPSHOT;
          return stage_first_false("successor_frame_seed_provenance", nullptr);
        }
        ContinuousPhasePathState predecessor_state;
        phase_offset_core::NormalFrameQuery predecessor_frame;
        if (!expected_pair->path_owner->evaluate(captured_w0,
                                                 predecessor_state, false) ||
            !expected_pair->frame_owner->query(captured_w0, predecessor_frame) ||
            !predecessor_state.valid || !predecessor_frame.valid ||
            (predecessor_state.p + predecessor_frame.N *
                 successor_seed_authority->delta -
             successor_seed_authority->r).norm() > 1e-8 ||
            (predecessor_state.dp_dw + predecessor_frame.N_w *
                 successor_seed_authority->delta -
             successor_seed_authority->r_w).norm() > 1e-8 ||
            !successor_seed_authority->executed_N.allFinite() ||
            successor_seed_authority->executed_N.norm() <= 1e-10 ||
            (predecessor_frame.N - successor_seed_authority->executed_N).norm() >
                1e-7) {
          if (temporary_failure) *temporary_failure =
              PathTubePairStageFailure::PAIR_SESSION_RUNTIME_SNAPSHOT;
          return stage_first_false("successor_frame_predecessor_state", nullptr);
        }
        ContinuousPhasePathState successor_start_state;
        if (!new_path_owner->evaluate(captured_w0,
                                      successor_start_state, false) ||
            !successor_start_state.valid ||
            successor_start_state.dp_dw.norm() <= 1e-10) {
          if (temporary_failure) *temporary_failure =
              PathTubePairStageFailure::PAIR_SESSION_RUNTIME_SNAPSHOT;
          return stage_first_false("successor_frame_seed_start_query", nullptr);
        }
        const Eigen::Vector3d tangent =
            successor_start_state.dp_dw.normalized();
        executed_seed = successor_seed_authority->executed_N -
            tangent * successor_seed_authority->executed_N.dot(tangent);
        // A valid authority can still carry a normal nearly parallel to the
        // successor composite-start tangent.  In that genuinely unusable
        // start-plane case leave the explicit seed absent so the immutable
        // frame constructor uses its deterministic least-parallel fallback.
        if (executed_seed.allFinite() && executed_seed.norm() > 1e-10) {
          executed_seed.normalize();
          if (executed_seed.dot(successor_seed_authority->executed_N) < 0.0) {
            executed_seed = -executed_seed;
          }
          have_executed_seed = true;
        }
      }
    }
    // This is transaction-local provenance only.  It becomes live source
    // bookkeeping exclusively in commitPathTubePair after the same CAS.
    staged_revision = std::max(source_revision_,
        expected_pair ? expected_pair->source_revision : 0U) + 1U;
  }

  const std::shared_ptr<const ContinuousPhaseNormalFrame> new_frame_owner =
      have_executed_seed
      ? std::shared_ptr<const ContinuousPhaseNormalFrame>(
            new ContinuousPhaseNormalFrame(new_path_owner, staged_revision,
                                            staged_revision, executed_seed))
      : std::shared_ptr<const ContinuousPhaseNormalFrame>(
            new ContinuousPhaseNormalFrame(new_path_owner, staged_revision,
                                            staged_revision));
  if (have_executed_seed && successor_seed_authority) {
    phase_offset_core::NormalFrameQuery successor_start_frame;
    if (!new_frame_owner->query(captured_w0, successor_start_frame) ||
        !successor_start_frame.valid ||
        (successor_start_frame.N - successor_seed_authority->executed_N).norm() >
            1e-7) {
      if (temporary_failure) {
        *temporary_failure = PathTubePairStageFailure::PAIR_SESSION_RUNTIME_SNAPSHOT;
      }
      return stage_first_false("successor_frame_start_binding", nullptr);
    }
  }
  phase_offset_core::PathDifferentialState captured_path;
  if (!EvaluateOwnerState(new_path_owner, captured_w0, captured_path,
                          new_frame_owner)) {
    if (temporary_failure) {
      *temporary_failure = PathTubePairStageFailure::OWNER_EVALUATE;
    }
    return stage_first_false("owner_evaluate", nullptr);
  }
  TubeBuildRequest request;
  request.active = true;
  request.task_generation = captured_task_generation;
  request.authority_session = captured_authority_session;
  request.source_revision = staged_revision;
  request.semantic_path_owner = new_path_owner;
  request.frame_owner = new_frame_owner;
  request.semantic_path_start_w = new_path_owner->startW();
  request.semantic_path_end_w = new_path_owner->endW();
  request.current_path = captured_path;
  request.position = position;
  request.retained_delta = retained_delta;
  request.authority_request = authority_request;
  request.cloud_snapshot = frozen_cloud_occupancy_snapshot;
  if (config_.tube_source == phase_offset_navigation::TubeSource::ESDF) {
    request.map_observation_is_snapshot = true;
    request.map_observation_sequence = frozen_cloud_occupancy_snapshot
        ? frozen_cloud_occupancy_snapshot->observation_sequence : 0U;
  }

  CanonicalOwnerStateReuse canonical_owner_state;
  canonical_owner_state.owner = new_path_owner;
  canonical_owner_state.state = captured_path;
  canonical_owner_state.verified_samples = &new_path_samples;
  canonical_owner_state.task_generation = request.task_generation;
  canonical_owner_state.source_revision = request.source_revision;
  canonical_owner_state.authority_session = request.authority_session;
  canonical_owner_state.map_observation_sequence =
      request.map_observation_sequence;
  canonical_owner_state.map_observation_is_snapshot =
      request.map_observation_is_snapshot;
  canonical_owner_state.valid = prepared_samples_owner_valid;

  PreparedTubeBuildResult prepared;
  phase_offset_navigation::TubeEpochUpdateResult prepared_epoch;
  std::string prepared_build_failure_layer;
  if (!buildPreparedTubeEpoch(request, new_path_samples,
                              new_path_samples.front().w,
                              new_path_samples.back().w, future_seam_w,
                              existing_future_horizon_end_w, prepared,
                              &prepared_epoch,
                              &prepared_build_failure_layer,
                              &canonical_owner_state)) {
    PathTubePairStageFailure failure =
        PathTubePairStageFailure::TUBE_BUILD_PRECONDITION;
    const char* layer = "tube_build_precondition";
    if (prepared_build_failure_layer == "tube_build_owner_evaluate") {
      failure = PathTubePairStageFailure::OWNER_EVALUATE;
      layer = "tube_build_owner_evaluate";
    } else if (prepared_build_failure_layer == "tube_profile_coverage" ||
               prepared_build_failure_layer == "tube_profile_owner_match") {
      failure = prepared_build_failure_layer == "tube_profile_coverage"
          ? PathTubePairStageFailure::TUBE_PROFILE_COVERAGE
          : PathTubePairStageFailure::TUBE_PROFILE_OWNER_MATCH;
      layer = prepared_build_failure_layer.c_str();
    } else if (prepared_build_failure_layer == "tube_update_status") {
      failure = classifyTubeUpdateStatusStageFailure(prepared_epoch);
      layer = PathTubePairStageFailureName(failure);
    }
    if (temporary_failure) *temporary_failure = failure;
    return stage_first_false(layer, &prepared_epoch);
  }
  // The staging dry run uses the captured w0 and retained port state through
  // a local Runtime copy; it cannot refresh live preflight or mutate delta.
  phase_offset_navigation::RuntimeDryRunResult dry_run;
  if (!dryRunPreparedRuntime(runtime_snapshot, prepared, captured_path,
                             position, gains, dt, dry_run) || !dry_run.valid) {
    if (temporary_failure) {
      *temporary_failure = PathTubePairStageFailure::STAGING_DRY_RUN;
    }
    const std::string& invalid_reason =
        !dry_run.step.invalid_reason.empty()
        ? dry_run.step.invalid_reason : dry_run.prepared.invalid_reason;
    ROS_WARN(
        "[PHASE_OFFSET][H2][STAGING_DRY_RUN] identity=%s "
        "result_valid=%d step_valid=%d step_selected=%d projection_valid=%d "
        "runtime_mode=%d certificate_denied=%d fatal_control_failure=%d "
        "retained_delta=%.9f previous_final_port=(%.9f,%.9f) "
        "current_w=%.9f seam_w=%.9f owner_end_w=%.9f "
        "invalid_reason=\"%s\"",
        expected_pair ? "replacement" : "bootstrap",
        dry_run.valid ? 1 : 0, dry_run.step.valid ? 1 : 0,
        dry_run.step.selected ? 1 : 0,
        dry_run.step.projection.valid ? 1 : 0,
        static_cast<int>(dry_run.step.execution.mode),
        dry_run.step.execution.certificate_denied ? 1 : 0,
        dry_run.step.execution.fatal_control_failure ? 1 : 0,
        retained_delta, previous_port.u_w, previous_port.u_delta,
        captured_w0, future_seam_w, new_path_owner->endW(),
        invalid_reason.c_str());
    return stage_first_false("staging_dry_run", &prepared_epoch);
  }

  std::shared_ptr<PathTubePair> candidate(new PathTubePair());
  candidate->source_revision = prepared.source_revision;
  candidate->path_revision = prepared.active_profile
      ? prepared.active_profile->path_revision : prepared.source_revision;
  candidate->frame_revision = prepared.active_profile
      ? prepared.active_profile->frame_revision : 0U;
  candidate->authority_session = request.authority_session;
  candidate->map_observation_sequence = prepared.map_observation_sequence;
  candidate->map_observation_is_snapshot =
      prepared.map_observation_is_snapshot;
  candidate->path_owner = prepared.semantic_path_owner;
  candidate->frame_owner = prepared.frame_owner;
  candidate->frozen_cloud_occupancy_snapshot =
      prepared.frozen_cloud_occupancy_snapshot;
  candidate->full_path_samples = prepared.full_path_samples;
  candidate->active_profile = prepared.active_profile;
  candidate->executed_reference_query =
      std::shared_ptr<const phase_offset_navigation::ImmutableExecutedReferenceQuery>(
          new PhaseOffsetExecutedReferenceQuery(
              candidate->path_owner, retained_delta,
              candidate->frame_owner,
              candidate->active_profile
                  ? candidate->active_profile->path_revision
                  : candidate->source_revision,
              candidate->active_profile
                  ? candidate->active_profile->frame_revision : 0U,
              candidate->source_revision, candidate->source_revision));
  candidate->successor_seed_authority = successor_seed_authority;
  candidate->epoch_status = prepared.epoch_status;
  candidate->captured_w0 = captured_w0;
  candidate->future_seam_w = future_seam_w;
  candidate->existing_future_horizon_end_w = existing_future_horizon_end_w;
  candidate->captured_retained_delta = retained_delta;
  candidate->captured_previous_final_port = previous_port;
  std::shared_ptr<TubeEpochSnapshot> epoch(new TubeEpochSnapshot());
  epoch->active = true;
  epoch->task_generation = request.task_generation;
  epoch->source_revision = candidate->source_revision;
  epoch->map_observation_sequence = candidate->map_observation_sequence;
  epoch->map_observation_is_snapshot = candidate->map_observation_is_snapshot;
  epoch->candidate_build_w = request.current_path.w;
  epoch->full_path_samples = candidate->full_path_samples;
  // The pair branch exposes this prepared profile as both Candidate and Active
  // evidence.  Keep its immutable epoch snapshot consistent for the existing
  // command-to-timer publication check; this is the same owner/profile pointer.
  epoch->candidate_profile = candidate->active_profile;
  epoch->active_profile = candidate->active_profile;
  epoch->epoch_status = candidate->epoch_status;
  candidate->epoch_snapshot = std::shared_ptr<const TubeEpochSnapshot>(epoch);
  transaction.expected_pair = expected_pair;
  transaction.candidate_pair = std::shared_ptr<const PathTubePair>(candidate);
  if (expected_capture) transaction.expected_capture = *expected_capture;
  transaction.pin_lease_id = pin_lease_id;
  transaction.authority_session = candidate->authority_session;
  transaction.captured_retained_delta = retained_delta;
  transaction.captured_previous_final_port = previous_port;
  return true;
}

bool PhaseOffsetMatchedAdapter::preparePathTubePairCommit(
    const PathTubePairTransaction& transaction,
    const double current_w,
    const Eigen::Vector3d& position,
    const guidance::IsfGains& gains,
    const double dt,
    const std::shared_ptr<const plan_env::CloudOccupancySnapshot>&
        latest_cloud_occupancy_snapshot,
    PathTubePairCommitPreparation& preparation) {
  preparation = PathTubePairCommitPreparation();
  const std::shared_ptr<const PathTubePair>& candidate =
      transaction.candidate_pair;
  const bool replacement = static_cast<bool>(transaction.expected_pair);
  if (!runtime_ || !candidate || !candidate->path_owner ||
      !candidate->full_path_samples || !candidate->active_profile ||
      candidate->source_revision == 0U ||
      candidate->authority_session != transaction.authority_session ||
      !IsFinite(current_w) ||
      current_w < candidate->captured_w0 -
          kPreparedCoverageTolerance ||
      current_w >= candidate->future_seam_w ||
      !ProfileStructurallyCoversPreparedRange(
          *candidate->active_profile, candidate->captured_w0,
          candidate->future_seam_w,
          candidate->existing_future_horizon_end_w) ||
      !ProfileSamplesMatchOwner(
          *candidate->active_profile, candidate->path_owner,
          MakeRuntimeConfig(config_).tube.regularity_margin,
          MakeRuntimeConfig(config_).tube.minimum_reference_speed,
          candidate->frame_owner)) {
    return false;
  }
  if (replacement &&
      (!CaptureMatchesPair(transaction.expected_capture,
                           transaction.expected_pair) ||
       !ActiveLeaseMatchesCapture(path_tube_pin_registry_,
                                  transaction.expected_capture,
                                  transaction.pin_lease_id))) {
    return false;
  }
  if (!replacement &&
      (transaction.expected_capture.pair || transaction.pin_lease_id != 0U)) {
    return false;
  }

  // Short snapshot: the exact port trial below runs on this local copy, never
  // while Runtime mutation is blocked.
  phase_offset_navigation::PhaseOffsetRuntime runtime_snapshot(
      MakeRuntimeConfig(config_));
  {
    std::lock_guard<std::mutex> lock(runtime_command_mutex_);
    const std::shared_ptr<const PathTubePair> live =
        std::atomic_load(&authoritative_path_tube_pair_);
    if (live != transaction.expected_pair ||
        authority_session_.load(std::memory_order_acquire) !=
            transaction.authority_session ||
        (live && live->authority_session != transaction.authority_session)) {
      return false;
    }
    if (replacement &&
        (!CaptureMatchesPair(transaction.expected_capture, live) ||
         !ActiveLeaseMatchesCapture(path_tube_pin_registry_,
                                    transaction.expected_capture,
                                    transaction.pin_lease_id))) {
      return false;
    }
    if (!replacement && runtime_->hasExecutedOffsetAuthority()) {
      return false;
    }
    runtime_snapshot = *runtime_;
  }
  // The old immutable pair need only remain executable at the live CAS
  // phase.  It is intentionally not required to certify the future geometric
  // seam: that copied-prefix/connector obligation belongs to `candidate`.
  if (replacement) {
    const std::shared_ptr<const PathTubePair>& old_pair =
        transaction.expected_pair;
    phase_offset_core::PathDifferentialState old_current;
    phase_offset_navigation::TubeBounds old_bounds;
    const double retained_delta = runtime_snapshot.retainedDelta();
    if (!old_pair || !old_pair->path_owner || !old_pair->active_profile ||
        !old_pair->epoch_status.active_available ||
        !old_pair->epoch_status.active_current_validation_valid ||
        old_pair->epoch_status.active_path_source_revision !=
            old_pair->source_revision ||
        old_pair->active_profile->source_revision !=
            old_pair->source_revision ||
        current_w < old_pair->active_profile->certified_segment_start_w -
            kPreparedCoverageTolerance ||
        current_w > old_pair->active_profile->certified_segment_end_w +
            kPreparedCoverageTolerance ||
        !EvaluateOwnerState(old_pair->path_owner, current_w, old_current) ||
        !phase_offset_navigation::TubeFilter::query(
            *old_pair->active_profile, current_w, old_bounds) ||
        !old_bounds.valid ||
        retained_delta < old_bounds.lower + config_.tube.interior_margin -
            kPreparedCoverageTolerance ||
        retained_delta > old_bounds.upper - config_.tube.interior_margin +
            kPreparedCoverageTolerance) {
      return false;
    }
  }
  // Current w must be evaluated by the newly built owner, never accepted from
  // a stale old-path state with coincident phase alone.
  phase_offset_core::PathDifferentialState owner_current;
  if (!EvaluateOwnerState(candidate->path_owner, current_w,
                          owner_current, candidate->frame_owner)) {
    return false;
  }
  PreparedTubeBuildResult prepared;
  prepared.complete = true;
  prepared.source_revision = candidate->source_revision;
  prepared.map_observation_sequence = candidate->map_observation_sequence;
  prepared.map_observation_is_snapshot = candidate->map_observation_is_snapshot;
  prepared.semantic_path_owner = candidate->path_owner;
  prepared.frame_owner = candidate->frame_owner;
  prepared.frozen_cloud_occupancy_snapshot =
      candidate->frozen_cloud_occupancy_snapshot;
  prepared.prepared_start_w = candidate->full_path_samples->front().w;
  prepared.prepared_end_w = candidate->full_path_samples->back().w;
  prepared.captured_w0 = candidate->captured_w0;
  prepared.future_seam_w = candidate->future_seam_w;
  prepared.existing_future_horizon_end_w =
      candidate->existing_future_horizon_end_w;
  prepared.full_path_samples = candidate->full_path_samples;
  prepared.active_profile = candidate->active_profile;
  prepared.epoch_status = candidate->epoch_status;
  phase_offset_navigation::RuntimeDryRunResult dry_run;
  if (!dryRunPreparedRuntime(runtime_snapshot, prepared, owner_current,
                             position, gains, dt, dry_run) || !dry_run.valid) {
    return false;
  }

  // The frozen snapshot proves construction provenance.  A newer categorical
  // observation that explicitly says OCCUPIED/OUT_OF_MAP at the current base
  // reference or actual position must still deny an install.
  if (LatestCategoricalUnsafe(config_, cloud_occupancy_query_config_,
                              latest_cloud_occupancy_snapshot, owner_current,
                              position, runtime_snapshot.retainedDelta())) {
    return false;
  }

  preparation.transaction = transaction;
  preparation.current_w = current_w;
  preparation.expected_retained_delta = runtime_snapshot.retainedDelta();
  preparation.expected_previous_final_port =
      runtime_snapshot.previousFinalPort();
  preparation.valid = true;
  return true;
}

bool PhaseOffsetMatchedAdapter::finalizePreparedPathTubePairCommit(
    const PathTubePairCommitPreparation& preparation,
    std::shared_ptr<const PathTubePair>& committed_pair) {
  committed_pair.reset();
  if (!preparation.valid || !preparation.transaction.candidate_pair) {
    return false;
  }
  const PathTubePairTransaction& transaction = preparation.transaction;
  const std::shared_ptr<const PathTubePair>& candidate =
      transaction.candidate_pair;
  const bool replacement = static_cast<bool>(transaction.expected_pair);
  if (replacement &&
      (!CaptureMatchesPair(transaction.expected_capture,
                           transaction.expected_pair) ||
       !ActiveLeaseMatchesCapture(path_tube_pin_registry_,
                                  transaction.expected_capture,
                                  transaction.pin_lease_id))) {
    return false;
  }
  if (!replacement &&
      (transaction.expected_capture.pair || transaction.pin_lease_id != 0U)) {
    return false;
  }

  // Short revalidation/CAS: no tube build, preflight, or port enumeration is
  // allowed in this lock.
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  const std::shared_ptr<const PathTubePair> live =
      std::atomic_load(&authoritative_path_tube_pair_);
  if (live != transaction.expected_pair ||
      authority_session_.load(std::memory_order_acquire) !=
          transaction.authority_session ||
      candidate->authority_session != transaction.authority_session ||
      (live && live->authority_session != transaction.authority_session) ||
      (!replacement && runtime_->hasExecutedOffsetAuthority()) ||
      !BitsEqual(runtime_->retainedDelta(),
                 preparation.expected_retained_delta) ||
      !BitsEqual(runtime_->previousFinalPort(),
                 preparation.expected_previous_final_port)) {
    return false;
  }
  if (replacement &&
      (!CaptureMatchesPair(transaction.expected_capture, live) ||
         !ActiveLeaseMatchesCapture(path_tube_pin_registry_,
                                  transaction.expected_capture,
                                  transaction.pin_lease_id))) {
    return false;
  }
  std::shared_ptr<PathTubePair> installed(new PathTubePair(*candidate));
  installed->generation = ++next_path_tube_pair_generation_;
  installed->authority_session = transaction.authority_session;
  const std::shared_ptr<const PathTubePair> immutable_installed(installed);
  // Atomic shared_ptr CAS is the one authority publication.  Only after it
  // succeeds may legacy source bookkeeping reflect the new owner.
  std::shared_ptr<const PathTubePair> expected = transaction.expected_pair;
  if (!std::atomic_compare_exchange_strong(&authoritative_path_tube_pair_,
                                           &expected,
                                           immutable_installed)) {
    return false;
  }
  source_identity_ = installed->path_owner.get();
  source_start_w_ = installed->path_owner->startW();
  source_end_w_ = installed->path_owner->endW();
  have_source_identity_ = true;
  source_revision_ = installed->source_revision;
  have_preflight_revision_ = false;
  committed_pair = immutable_installed;
  return true;
}

void PhaseOffsetMatchedAdapter::makeControlPublishSnapshot(
    const MatchedAdapterInput& input,
    const std::shared_ptr<const TubeBuildRequest>& request,
    const std::shared_ptr<const TubeEpochSnapshot>& epoch,
    const bool candidate_only,
    const MatchedAdapterOutput& output) {
  std::shared_ptr<ControlPublishSnapshot> control(new ControlPublishSnapshot());
  control->active = request && request->active;
  control->task_generation = request ? request->task_generation : 0U;
  control->control_sequence = request ? request->control_sequence : 0U;
  control->source_revision = request ? request->source_revision : 0U;
  control->map_observation_sequence =
      request ? request->map_observation_sequence : 0U;
  control->map_observation_is_snapshot =
      request && request->map_observation_is_snapshot;
  control->candidate_epoch_source_revision =
      epoch ? epoch->source_revision : 0U;
  control->candidate_epoch_map_observation_sequence =
      epoch ? epoch->map_observation_sequence : 0U;
  control->candidate_epoch_map_observation_is_snapshot =
      epoch && epoch->map_observation_is_snapshot;
  control->candidate_only = candidate_only;
  control->epoch_build_sequence = epoch ? epoch->build_sequence : 0U;
  control->stamp = input.stamp;
  control->current_w = input.path.w;
  control->position = input.position;
  control->epoch_snapshot = epoch;
  control->full_path_samples = epoch ? epoch->full_path_samples
                                     : std::shared_ptr<const PathSamples>();
  control->authority_snapshot = execution_authority_.snapshotPtr();
  control->output = output;
  std::atomic_store(&latest_control_snapshot_,
                    std::shared_ptr<const ControlPublishSnapshot>(control));
}

bool PhaseOffsetMatchedAdapter::finalizeTubeEpoch(
    const std::shared_ptr<const TubeBuildRequest>& request,
    const TubeEpochSnapshot& built) {
  if (!request || !request->active) return false;
  // Publication, including the raw/cloud and marker paths, is serialized with
  // task reset.  The expensive build happened before this function, so this
  // short boundary never holds the barrier across geometry/tube construction.
  // ROS Publisher::publish is non-callback dispatch and reset never publishes
  // while holding this lock, so this lock adds no ROS callback inversion.
  std::lock_guard<std::mutex> publication_lock(task_publication_mutex_);
  // Ordinary same-task stale builds still publish their raw/cloud provenance
  // below.  A task boundary is different: no old-task completion may publish
  // or repopulate an evidence slot for the new owner.
  if (built.task_generation != request->task_generation ||
      task_generation_.load(std::memory_order_acquire) !=
          request->task_generation) {
    return false;
  }
  if (finalize_publication_test_hook_) finalize_publication_test_hook_();
  // Raw/cloud payloads are per-timer-build provenance, including a path that
  // became stale while the build ran.  They never authorize Candidate,
  // Runtime, or Certified exposure, so emit them before the source gate.
  publishBuildDiagnostics(built);
  if (!requestSourceStillCurrent(*request)) {
    // A timer build is only evidence.  If its source is stale, discard it;
    // never reset/clear a newer pair or timer manager that may already own a
    // later Runtime-eligible epoch.
    return false;
  }
  const std::shared_ptr<const TubeEpochSnapshot> epoch(
      new TubeEpochSnapshot(built));
  // Every completed Candidate is retained with the snapshot that built it.
  // A new observation sequence alone does not discard an immutable, complete
  // same-source epoch: the command thread sample-and-holds it until the next
  // completed epoch atomically replaces it.  Path revision and the existing
  // cloud usable contract remain fail-closed in epochMatchesRequest().
  std::atomic_store(&latest_candidate_epoch_snapshot_, epoch);
  const std::shared_ptr<const TubeBuildRequest> latest =
      std::atomic_load(&latest_build_request_);
  if (latest && epochMatchesRequest(*epoch, *latest)) {
    std::atomic_store(&latest_epoch_snapshot_, epoch);
  } else {
    std::atomic_store(&latest_epoch_snapshot_,
                      std::shared_ptr<const TubeEpochSnapshot>());
  }
  // Timer evidence may refresh a same-owner pair, but only by comparing the
  // captured base pair/generation and Runtime bits under the short authority
  // lock.  A stale completion is discarded above and cannot clear/replace a
  // newer pair.
  refreshPairFromTimerEpoch(request, epoch);

  // The 83/50 payload still comes from a command-owned snapshot; raw/cloud
  // build facts above are timer-owned and must not wait for that handshake.
  const std::shared_ptr<const ControlPublishSnapshot> control =
      std::atomic_load(&latest_control_snapshot_);
  if (control && control->active) publishManual(*control);
  return true;
}

void PhaseOffsetMatchedAdapter::consumeTimerTaskGeneration(
    const std::uint64_t task_generation) {
  // This function is called only by timerTick after it owns timer_inflight_.
  // It is the sole writer for persistent timer state; command-side task reset
  // only publishes task_generation_ and clears cross-thread snapshots.
  cached_full_path_samples_.clear();
  cached_path_source_revision_ = 0U;
  have_cached_path_ = false;
  timer_active_profile_.reset();
  timer_installed_active_epoch_ = 0U;
  timer_last_deactivate_sequence_ = 0U;
  timer_last_publish_delete_source_revision_ = 0U;
  timer_last_publish_delete_map_observation_sequence_ = 0U;
  timer_last_published_epoch_build_sequence_ = 0U;
  timer_last_raw_diagnostic_build_sequence_ = 0U;
  timer_last_cloud_diagnostic_build_sequence_ = 0U;
  latest_cloud_occupancy_query_status_ = CloudOccupancyQueryStatus();
  if (config_.mode == PhaseOffsetMatchedMode::MANUAL) {
    tube_epoch_manager_.reset(new phase_offset_navigation::TubeEpochManager(
        MakeEpochConfig(config_)));
  }
  std::atomic_store(&latest_candidate_epoch_snapshot_,
                    std::shared_ptr<const TubeEpochSnapshot>());
  std::atomic_store(&latest_epoch_snapshot_,
                    std::shared_ptr<const TubeEpochSnapshot>());
  timer_task_generation_ = task_generation;
}

bool PhaseOffsetMatchedAdapter::timerTick() {
  if (shutdown_requested_.load(std::memory_order_acquire)) return false;
  bool expected = false;
  if (!timer_inflight_.compare_exchange_strong(expected, true,
                                                std::memory_order_acq_rel)) {
    return false;
  }
  struct TimerExit {
    std::atomic<bool>& inflight;
    ~TimerExit() { inflight.store(false, std::memory_order_release); }
  } exit {timer_inflight_};
  if (shutdown_requested_.load(std::memory_order_acquire)) return false;

  const std::uint64_t task_generation =
      task_generation_.load(std::memory_order_acquire);
  if (timer_task_generation_ != task_generation) {
    consumeTimerTaskGeneration(task_generation);
  }

  std::shared_ptr<const TubeBuildRequest> request =
      std::atomic_load(&latest_build_request_);
  if (request && request->task_generation != timer_task_generation_) {
    const std::uint64_t latest_task_generation =
        task_generation_.load(std::memory_order_acquire);
    if (timer_task_generation_ != latest_task_generation) {
      consumeTimerTaskGeneration(latest_task_generation);
    }
    request = std::atomic_load(&latest_build_request_);
  }
  if (request &&
      (request->task_generation != timer_task_generation_ ||
       request->task_generation !=
           task_generation_.load(std::memory_order_acquire))) {
    return false;
  }
  if (!request) return false;
  if (!request->active) {
    // An inactive request may publish DELETE markers and clear timer evidence.
    // Treat those as task publication, not as an unguarded timer-local
    // cleanup: an old A DELETE must never erase B after reset returns.
    if (inactive_publication_test_hook_) inactive_publication_test_hook_();
    std::lock_guard<std::mutex> publication_lock(task_publication_mutex_);
    if (request->task_generation != timer_task_generation_ ||
        request->task_generation !=
            task_generation_.load(std::memory_order_acquire)) {
      return false;
    }
    if (request->control_sequence != timer_last_deactivate_sequence_) {
      cached_full_path_samples_.clear();
      cached_path_source_revision_ = 0U;
      have_cached_path_ = false;
      timer_active_profile_.reset();
      timer_installed_active_epoch_ = 0U;
      tube_epoch_manager_.reset(new phase_offset_navigation::TubeEpochManager(
          MakeEpochConfig(config_)));
      std::atomic_store(&latest_candidate_epoch_snapshot_,
                        std::shared_ptr<const TubeEpochSnapshot>());
      const std::shared_ptr<const ControlPublishSnapshot> control =
          std::atomic_load(&latest_control_snapshot_);
      if (control && !control->active) publishManualDelete(*control);
      timer_last_deactivate_sequence_ = request->control_sequence;
    }
    return true;
  }

  const bool measure_tube_due = measurement_tube_due_enabled_;
  const auto tube_due_start = measure_tube_due
      ? std::chrono::steady_clock::now()
      : std::chrono::steady_clock::time_point();
  TubeEpochSnapshot built;
  const bool built_ok = buildTubeEpoch(request, built);
  std::uint64_t tube_due_duration_ns = 0U;
  if (measure_tube_due) {
    const auto tube_due_end = std::chrono::steady_clock::now();
    tube_due_duration_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            tube_due_end - tube_due_start).count());
  }

  // An incomplete Candidate is still required evidence.  Finalize it before
  // reporting the manager's build result so WAITING/diagnostic semantics do
  // not disappear merely because installation was correctly rejected.
  const bool finalized = finalizeTubeEpoch(request, built);
  if (measure_tube_due) {
    const bool raw_cloud_publish_attempted =
        config_.tube_source == phase_offset_navigation::TubeSource::ESDF &&
        built.raw_candidate_diagnostics_generated &&
        built.cloud_snapshot_diagnostics_generated;
    recordTubeDueTiming(tube_due_duration_ns, request->stamp.toNSec(),
                        finalized, raw_cloud_publish_attempted);
  }
  return finalized && built_ok;
}

void PhaseOffsetMatchedAdapter::latchFailure(
    const phase_offset_navigation::ControlFailureReason reason) {
  if (!failure_latched_) {
    ++manual_invalid_count_;
    ++manual_fallback_count_;
    control_failure_reason_ = reason;
  }
  failure_latched_ = true;
}

bool PhaseOffsetMatchedAdapter::updateGate(const MatchedAdapterInput& input,
                                           MatchedAdapterOutput& output) {
  const bool path_available = HasUsablePathInput(input.path);
  const bool legacy_available = HasUsableLegacyInput(input.legacy);
  if (!path_available || !legacy_available) {
    const std::string reason = !path_available
        ? "matched adapter path input is unavailable"
        : "matched adapter legacy guidance input is unavailable";
    output.zero_port.invalid_reason = reason;
    output.zero_comparison.invalid_reason = reason;
    output.invalid_reason = reason;
    output.zero_gate_open = zero_gate_open_;
    output.zero_gate_consecutive_count = zero_gate_consecutive_count_;
    output.failure_latched = failure_latched_;
    output.control_failure_reason = control_failure_reason_;
    return false;
  }
  ActiveAdapterInput zero_input;
  zero_input.path = input.path; zero_input.position = input.position; zero_input.gains = input.gains;
  zero_port_adapter_.evaluate(zero_input, output.zero_port);
  PhaseOffsetActiveAdapter::compareWithLegacy(output.zero_port, input.legacy,
      config_.equivalence_tolerance, output.zero_comparison);
  const bool equivalent = output.zero_comparison.valid && output.zero_comparison.equivalent;
  if (output.zero_comparison.valid) {
    if (!failure_latched_ && !zero_gate_open_) {
      zero_gate_consecutive_count_ = equivalent ? zero_gate_consecutive_count_ + 1 : 0;
      zero_gate_open_ = zero_gate_consecutive_count_ >= config_.warmup_cycles;
    } else if (!failure_latched_ && !equivalent) {
      latchFailure(phase_offset_navigation::ControlFailureReason::ZERO_PORT_EQUIVALENCE);
    }
  }
  output.zero_gate_open = zero_gate_open_; output.zero_gate_consecutive_count = zero_gate_consecutive_count_;
  output.failure_latched = failure_latched_; output.geometry = output.zero_port.geometry;
  output.control_failure_reason = control_failure_reason_;
  output.base_guidance = output.zero_port.guidance; output.guidance = output.zero_port.guidance;
  return true;
}

void PhaseOffsetMatchedAdapter::fillLegacyTubeStatus(MatchedAdapterOutput& output) const {
  const auto& epoch = output.tube_epoch_status;
  const auto& execution = output.runtime_execution;
  const auto& profile = output.tube_profile;
  auto& status = output.tube_status;
  status = phase_offset_navigation::TubeRuntimeStatus();
  status.source = config_.tube_source;
  status.readiness_evaluated = status.source != phase_offset_navigation::TubeSource::NONE &&
      epoch.candidate_sequence > 0U;
  status.source_ready = epoch.active_current_validation_valid;
  status.raw_complete = profile.raw_complete; status.filtered_complete = profile.filtered_complete;
  status.profile_complete = profile.complete; status.obstacle_certified = profile.obstacle_certified;
  status.current_inside = execution.retained_delta_current_inside;
  status.next_inside = output.tube_next_bounds.valid && output.projection.valid &&
      output.projection.next_delta >= output.tube_next_bounds.lower + config_.tube.interior_margin - 1e-10 &&
      output.projection.next_delta <= output.tube_next_bounds.upper - config_.tube.interior_margin + 1e-10;
  status.tracking_within_bound = execution.tracking_within_bound;
  status.tube_violation = execution.certificate_denied;
  status.failure_latched = output.failure_latched;
  status.rebuild_count = CountAsInt(epoch.candidate_sequence);
  status.reject_count = CountAsInt(epoch.reject_count);
  status.violation_count = status.tube_violation ? 1 : 0;
  status.certified_forward_w = epoch.certified_forward_w;
  status.reference_signed_distance = epoch.reference_signed_distance;
  status.actual_signed_distance = epoch.actual_signed_distance;
  status.tracking_error_norm = execution.tracking_error_norm;
}

bool PhaseOffsetMatchedAdapter::tubeDisplayCertified(const MatchedAdapterOutput& output) const {
  const auto& epoch = output.tube_epoch_status;
  const auto& runtime = output.runtime_execution;
  const auto* active = output.active_profile.get();
  const bool current_runtime_port_set = runtime.mode ==
          phase_offset_navigation::RuntimeExecutionMode::NORMAL ||
      runtime.mode == phase_offset_navigation::RuntimeExecutionMode::SAFETY_PRIORITY;
  const bool current_exact_nonnegative_port = output.valid && output.projection.valid &&
      output.matched.valid && current_runtime_port_set;
  const bool current_map_safe = config_.tube_source ==
      phase_offset_navigation::TubeSource::FIXED ||
      epoch.current_safety_status ==
          phase_offset_navigation::CurrentSafetyStatus::SAFE;
  return config_.mode == PhaseOffsetMatchedMode::MANUAL && active != nullptr && active->complete &&
      epoch.active_current_validation_valid &&
      runtime.current_geometry_valid && runtime.current_bounds_valid &&
      runtime.retained_delta_current_inside && runtime.tracking_within_bound &&
      current_map_safe && !runtime.certificate_denied &&
      !runtime.fatal_control_failure && !output.failure_latched &&
      current_exact_nonnegative_port &&
      (config_.tube_source == phase_offset_navigation::TubeSource::FIXED ||
       active->obstacle_certified);
}

void PhaseOffsetMatchedAdapter::fillManualDiagnostics(MatchedAdapterOutput& output) const {
  output.diagnostics.fill(0.0);
  const auto& status = output.tube_status; const auto& profile = output.tube_profile;
  const auto& erosion = config_.tube.erosion; auto& d = output.diagnostics;
  const bool raw_occupancy = output.tube_epoch_status.raw_cross_section_path_used;
  const double base_speed = output.geometry.valid && output.base_guidance.valid ?
      output.geometry.T.dot(output.base_guidance.v_cmd) : 0.0;
  const double r_z = output.geometry.valid ? std::abs(output.geometry.r.z() - output.geometry.p.z()) : 0.0;
  const double upper_h = output.tube_current_bounds.valid ? output.tube_current_bounds.upper -
      config_.tube.interior_margin - output.delta : 0.0;
  const double lower_h = output.tube_current_bounds.valid ? output.delta -
      output.tube_current_bounds.lower - config_.tube.interior_margin : 0.0;
  d[kModeManual] = 1.0; d[kZeroGateOpen] = output.zero_gate_open; d[kZeroGateConsecutiveCount] = output.zero_gate_consecutive_count;
  d[kFailureLatched] = output.failure_latched; d[kProfileActive] = output.profile_active; d[kPreflightComplete] = output.preflight.complete;
  d[kConfiguredAmplitude] = output.preflight.configured_amplitude; d[kAcceptedAmplitude] = output.preflight.accepted_amplitude;
  d[kDeltaRef] = output.delta_ref; d[kDelta] = output.delta; d[kDeltaTrackingError] = output.delta_ref - output.delta;
  d[kUWRaw] = output.raw_port.u_w; d[kUDeltaRaw] = output.raw_port.u_delta; d[kUWFinal] = output.projection.final_port.u_w;
  d[kUDeltaFinal] = output.projection.final_port.u_delta; d[kUWLimited] = output.projection.u_w_limited; d[kUDeltaLimited] = output.projection.u_delta_limited;
  d[kBaseWDot] = output.base_guidance.w_dot; d[kFinalWDot] = output.guidance.w_dot; d[kBaseTangentSpeed] = base_speed;
  d[kFinalTangentSpeed] = output.projection.final_tangent_speed; d[kCurrentRegularity] = output.geometry.regularity;
  d[kNextRegularity] = output.projection.next_regularity; d[kMatchedResidualNorm] = output.matched.matched_residual_norm;
  d[kPhysicalPortNorm] = output.matched.physical_port.norm(); d[kRMinusPNorm] = output.geometry.valid ? (output.geometry.r - output.geometry.p).norm() : 0.0;
  d[kRZMinusPZ] = r_z; d[kSelectedManual] = output.selected; d[kManualValid] = output.valid;
  d[kPreflightSampleCount] = output.preflight.sample_count; d[kPreflightInvalidSampleCount] = output.preflight.invalid_sample_count;
  d[kPreflightMinRegularity] = output.preflight.min_regularity; d[kManualInvalidCount] = manual_invalid_count_; d[kManualFallbackCount] = manual_fallback_count_;
  d[kTubeSource] = static_cast<int>(status.source); d[kTubeSourceReady] = status.source_ready;
  d[kTubeRawComplete] = status.raw_complete; d[kTubeFilteredComplete] = status.filtered_complete; d[kTubeProfileComplete] = status.profile_complete;
  d[kTubeObstacleCertified] = status.obstacle_certified; d[kTubeSourceRevision] = profile.source_revision; d[kTubeRevision] = profile.tube_revision;
  d[kTubeSampleCount] = profile.diagnostics.sample_count; d[kTubeInvalidCount] = profile.diagnostics.invalid_count;
  d[kTubeUnavailableCount] = profile.diagnostics.unavailable_count; d[kTubeOutOfMapCount] = profile.diagnostics.out_of_map_count;
  d[kTubeUnknownCount] = profile.diagnostics.unknown_count; d[kTubeOccupiedCount] = profile.diagnostics.occupied_count;
  d[kTubePreviewStart] = profile.preview_start_w; d[kTubePreviewEnd] = profile.preview_end_w; d[kTubeCertifiedForward] = status.certified_forward_w;
  d[kTubeLower] = output.tube_current_bounds.lower; d[kTubeUpper] = output.tube_current_bounds.upper;
  d[kTubeLowerW] = output.tube_current_bounds.lower_w; d[kTubeUpperW] = output.tube_current_bounds.upper_w;
  d[kTubeNextLower] = output.tube_next_bounds.lower; d[kTubeNextUpper] = output.tube_next_bounds.upper;
  d[kTubeCurrentInside] = status.current_inside; d[kTubeNextInside] = status.next_inside; d[kTubeUpperH] = upper_h; d[kTubeLowerH] = lower_h;
  d[kTubeUpperInvariant] = output.projection.upper_invariant_residual; d[kTubeLowerInvariant] = output.projection.lower_invariant_residual;
  d[kTubeRequiredReferenceClearance] = output.tube_epoch_status.required_reference_clearance;
  d[kTubeRequiredActualClearance] = output.tube_epoch_status.required_actual_clearance;
  d[kTubeReferenceDistance] = status.reference_signed_distance; d[kTubeActualDistance] = status.actual_signed_distance;
  d[kTubeTrackingNorm] = status.tracking_error_norm; d[kTubeTrackingBound] = raw_occupancy
      ? config_.tube.cross_section.margins.tracking_error_bound
      : erosion.tracking_error_bound;
  d[kTubeViolation] = status.tube_violation; d[kTubeRebuildCount] = status.rebuild_count; d[kTubeRejectCount] = status.reject_count; d[kTubeViolationCount] = status.violation_count;
  d[kTubeReadinessEvaluated] = status.readiness_evaluated; d[kTubeDisplayCertified] = tubeDisplayCertified(output);
  d[kPreflightFirstInvalidW] = output.preflight.first_invalid_w; d[kPreflightFirstInvalidSide] = output.preflight.first_invalid_side;
  d[kTubeFirstInvalidW] = profile.diagnostics.first_invalid_w; d[kTubeFirstInvalidSide] = profile.diagnostics.first_invalid_side;
  d[kTubeFirstStopReason] = profile.diagnostics.first_stop_reason; d[kTubeInsufficientClearanceCount] = profile.diagnostics.insufficient_clearance_count;
  d[kTubeMinWidth] = profile.diagnostics.min_width; d[kTubeMinSafetyMargin] = profile.diagnostics.min_safety_margin;
  for (double& value : d) value = FiniteOrZero(value);
}

bool PhaseOffsetMatchedAdapter::completeThroughExecutionAuthority(
    const MatchedAdapterInput& input,
    const std::shared_ptr<const PathTubePair>& pair,
    const std::shared_ptr<const TubeBuildRequest>& request,
    const std::shared_ptr<const TubeEpochSnapshot>& epoch,
    const phase_offset_navigation::RuntimePreparedStep& prepared,
    const Eigen::Vector3d& base_v_cmd,
    const double base_w_dot,
    const bool base_guidance_valid,
    phase_offset_navigation::RuntimeStepOutput& output) {
  output = phase_offset_navigation::RuntimeStepOutput();
  if (!runtime_) return false;

  // `prepare` is side-effect free.  Complete the exact port only on a local
  // value copy; the live Runtime is replaced inside the authority publisher
  // callback after selected-u/revision validation succeeds.
  phase_offset_navigation::PhaseOffsetRuntime staged_runtime(*runtime_);
  const phase_offset_core::PortCommand previous_final_port =
      staged_runtime.previousFinalPort();
  if (!staged_runtime.complete(prepared, base_v_cmd, base_w_dot,
                               base_guidance_valid, output)) {
    return false;
  }
  if (!output.selected) {
    // A gate-closed but otherwise valid step has no execution-state mutation.
    return true;
  }
  if (!output.valid || !output.projection.valid || !output.matched.valid) {
    output.selected = false;
    output.valid = false;
    if (output.invalid_reason.empty()) {
      output.invalid_reason = "selected runtime output is invalid";
    }
    return false;
  }

  const std::shared_ptr<const ContinuousPhasePath> path_owner = pair
      ? pair->path_owner
      : (request ? request->semantic_path_owner : input.semantic_path_owner);
  const std::shared_ptr<const ContinuousPhaseNormalFrame> frame_owner = pair
      ? pair->frame_owner
      : (request ? request->frame_owner : input.frame_owner);
  const std::shared_ptr<const phase_offset_navigation::TubeProfile> profile =
      pair ? pair->active_profile : (epoch ? epoch->active_profile
                                           : std::shared_ptr<const phase_offset_navigation::TubeProfile>());
  const std::uint64_t source_revision = request
      ? request->source_revision
      : (pair ? pair->source_revision : prepared.geometry.path_revision);
  if (!IsFinite(source_revision) || source_revision == 0U ||
      !IsFinite(prepared.geometry.w) || !IsFinite(prepared.dt) ||
      prepared.dt <= 0.0) {
    output.selected = false;
    output.valid = false;
    output.invalid_reason = "execution authority input provenance is invalid";
    return false;
  }

  const std::uint64_t path_revision = profile && profile->path_revision != 0U
      ? profile->path_revision
      : (prepared.geometry.path_revision != 0U
          ? prepared.geometry.path_revision : source_revision);
  const std::uint64_t frame_revision = profile && profile->frame_revision != 0U
      ? profile->frame_revision
      : (frame_owner ? frame_owner->frameRevision()
                     : prepared.geometry.frame_revision);
  const std::uint64_t query_revision = profile && profile->profile_revision != 0U
      ? profile->profile_revision : source_revision;

  phase_offset_navigation::ImmutableExecutedReferenceQueryPtr reference_query;
  if (path_owner) {
    reference_query =
        phase_offset_navigation::ImmutableExecutedReferenceQueryPtr(
            new PhaseOffsetExecutedReferenceQuery(
                path_owner, prepared.delta, frame_owner, path_revision,
                frame_revision, source_revision, query_revision));
  } else if (input.path_state_query) {
    // Legacy synthetic adapter callers have no ContinuousPhasePath owner, but
    // still provide the same immutable path-state query used by Runtime.
    // Bind that query into the authority-facing reference view rather than
    // weakening the execution contract or fabricating a path owner.
    const phase_offset_navigation::PathStateQuery path_query =
        input.path_state_query;
    const phase_offset_core::GeometryParams geometry_params = [&]() {
      phase_offset_core::GeometryParams params;
      params.regularity_margin = runtime_->config().tube.regularity_margin;
      params.minimum_reference_speed =
          runtime_->config().tube.minimum_reference_speed;
      return params;
    }();
    const double domain_start = input.semantic_path_start_w;
    const double domain_end = input.semantic_path_end_w;
    reference_query =
        phase_offset_navigation::ImmutableExecutedReferenceQueryPtr(
            new phase_offset_navigation::CallbackExecutedReferenceQuery(
                [path_query, geometry_params,
                 delta = prepared.delta](
                    const double w,
                    phase_offset_navigation::ExecutedReferenceQueryResult& result) {
                  phase_offset_core::PathDifferentialState state;
                  if (!path_query(w, state) || !state.valid) return false;
                  if (state.frame_valid && state.N.allFinite() &&
                      state.N_w.allFinite()) {
                    result.r = state.p + state.N * delta;
                    result.r_w = state.p_w + state.N_w * delta;
                  } else {
                    phase_offset_core::GeometryEvaluator evaluator(
                        geometry_params);
                    phase_offset_core::PhaseOffsetGeometryState geometry;
                    if (!evaluator.evaluate(state, Eigen::Vector3d::Zero(),
                                            delta, geometry) || !geometry.valid) {
                      return false;
                    }
                    result.r = geometry.r;
                    result.r_w = geometry.r_w;
                  }
                  result.r_ww.setZero();
                  result.r_ww_valid = false;
                  result.valid = result.r.allFinite() && result.r_w.allFinite();
                  return result.valid;
                },
                domain_start, domain_end, path_revision, frame_revision,
                source_revision, query_revision,
                "PhaseOffsetMatchedAdapter/runtime-path-query"));
  }
  if (!reference_query) {
    output.selected = false;
    output.valid = false;
    output.invalid_reason =
        "immutable executed-reference query is unavailable";
    return false;
  }

  phase_offset_navigation::ExecutedReferenceQueryResult reference_result;
  if (!reference_query->query(prepared.geometry.w, reference_result) ||
      !reference_result.valid) {
    output.selected = false;
    output.valid = false;
    output.invalid_reason = "immutable executed-reference query failed";
    return false;
  }

  phase_offset_navigation::ActiveReferenceSnapshot candidate;
  const phase_offset_navigation::ActiveReferenceSnapshot current_authority =
      execution_authority_.snapshot();
  candidate.authority_session = current_authority.valid
      ? current_authority.authority_session
      : authority_session_.load(std::memory_order_acquire);
  candidate.planner_path_revision = source_revision;
  candidate.executed_path_revision = reference_query->pathRevision();
  candidate.frame_revision = reference_query->frameRevision();
  candidate.tube_revision = profile ? profile->tube_revision : 0U;
  candidate.profile_revision = profile ? profile->profile_revision : 0U;
  candidate.map_revision = profile ? profile->map_revision : 0U;
  candidate.owner_mode =
      phase_offset_navigation::ActiveReferenceOwnerMode::NORMAL;
  candidate.selected_u_owner = "PhaseOffsetMatchedAdapterRuntime";
  candidate.w = prepared.geometry.w;
  candidate.delta = prepared.delta;
  candidate.dt = prepared.dt;
  candidate.u_prev = previous_final_port;
  candidate.selected_u = output.projection.final_port;
  candidate.proposed_next_w =
      prepared.geometry.w + prepared.dt * output.projection.final_w_dot;
  candidate.proposed_next_delta = output.projection.next_delta;
  candidate.proposed_next_u_prev = output.projection.final_port;
  candidate.r = prepared.geometry.r;
  candidate.r_w = prepared.geometry.r_w;
  candidate.r_ww = prepared.geometry.p_ww;
  candidate.r_ww_valid = false;
  candidate.matched_base_v_cmd = base_v_cmd;
  candidate.matched_base_w_dot = base_w_dot;
  candidate.executed_N = prepared.geometry.N;
  candidate.executed_reference_query = reference_query;
  candidate.reference_query_revision = reference_query->queryRevision();
  candidate.provenance = pair
      ? "PhaseOffsetMatchedAdapter/path-tube-runtime"
      : "PhaseOffsetMatchedAdapter/planner-runtime";
  candidate.safety_status = output.execution.certificate_denied
      ? "CERTIFICATE_DENIED" : "SAFE";
  candidate.handoff_state = pair ? "PATH_TUBE_PAIR" : "PLANNER_ONLY";
  candidate.valid = true;
  candidate.selected_u_w = candidate.selected_u.u_w;
  candidate.selected_u_delta = candidate.selected_u.u_delta;

  phase_offset_navigation::AuthorityPrepareInput authority_input;
  authority_input.candidate = candidate;
  if (current_authority.valid) {
    authority_input.expected_authority_session =
        current_authority.authority_session;
    authority_input.expected_sequence = current_authority.sequence;
  }
  authority_input.matched_output_valid = output.valid && output.matched.valid;
  authority_input.reference_valid = candidate.governorViewValid();
  authority_input.provenance = candidate.provenance;
  phase_offset_navigation::AuthorityPreparedStep authority_prepared;
  if (!execution_authority_.prepare(authority_input, authority_prepared)) {
    output.selected = false;
    output.valid = false;
    output.invalid_reason = authority_prepared.failure_reason.empty()
        ? "execution authority rejected selected runtime step"
        : authority_prepared.failure_reason;
    return false;
  }

  // Production publication is performed by gvf_manager after the governor
  // has emitted the local PositionCommand.  Keep both the exact prepared
  // snapshot and the staged Runtime private until that publication reports
  // success; no authority or live Runtime state is changed here.
  if (advertised_) {
    if (pending_authority_valid_) {
      output.selected = false;
      output.valid = false;
      output.invalid_reason =
          "previous PositionCommand transaction is still pending";
      return false;
    }
    if (!staged_runtime.makeCommitToken(prepared, output,
                                        pending_runtime_commit_)) {
      output.selected = false;
      output.valid = false;
      output.invalid_reason =
          "selected Runtime step cannot form a bounded commit token";
      return false;
    }
    pending_authority_prepared_ = authority_prepared;
    pending_authority_session_ = authority_prepared.candidate.authority_session;
    pending_authority_valid_ = true;
    return true;
  }

  phase_offset_navigation::AuthorityCommitResult authority_result;
  const bool committed = execution_authority_.commit(
      authority_prepared,
      [this, &staged_runtime, &output](
          const phase_offset_navigation::ActiveReferenceSnapshot& snapshot) {
        const double epsilon = execution_authority_.config().comparison_epsilon;
        if (!snapshot.selectedUConsistent(epsilon) ||
            std::abs(snapshot.selected_u.u_w - output.projection.final_port.u_w) >
                epsilon ||
            std::abs(snapshot.selected_u.u_delta -
                     output.projection.final_port.u_delta) > epsilon ||
            std::abs(snapshot.proposed_next_delta -
                     output.projection.next_delta) > epsilon) {
          return false;
        }
        *runtime_ = staged_runtime;
        return true;
      },
      authority_result);
  if (!committed) {
    output.selected = false;
    output.valid = false;
    output.invalid_reason = authority_result.failure_reason.empty()
        ? "execution authority commit failed" : authority_result.failure_reason;
    return false;
  }
  return true;
}

bool PhaseOffsetMatchedAdapter::completeAtomicNeutralHandoff(
    const MatchedAdapterInput& input,
    const std::shared_ptr<const PathTubePair>& pair,
    const phase_offset_navigation::RuntimePreparedStep& prepared,
    const Eigen::Vector3d& base_v_cmd,
    const double base_w_dot,
    const bool base_guidance_valid,
    phase_offset_navigation::RuntimeStepOutput& output) {
  output = phase_offset_navigation::RuntimeStepOutput();
  const phase_offset_navigation::ActiveReferenceSnapshot current =
      execution_authority_.snapshot();
  if (!runtime_ || !pair || !pair->path_owner || !pair->frame_owner ||
      !base_guidance_valid || !prepared.geometry.valid ||
      !current.valid ||
      current.owner_mode !=
          phase_offset_navigation::ActiveReferenceOwnerMode::RECOVERY ||
      prepared.delta != 0.0 ||
      !recovery_owner_.status().exact_terminal_predicate ||
      current.proposed_next_delta != 0.0) {
    return false;
  }
  phase_offset_core::PathDifferentialState neutral_path;
  phase_offset_core::PhaseOffsetGeometryState neutral_geometry;
  if (!EvaluateOwnerState(pair->path_owner, prepared.geometry.w,
                          neutral_path, pair->frame_owner)) {
    return false;
  }
  phase_offset_core::GeometryParams neutral_params;
  neutral_params.regularity_margin = runtime_->config().tube.regularity_margin;
  neutral_params.minimum_reference_speed =
      runtime_->config().tube.minimum_reference_speed;
  phase_offset_core::GeometryEvaluator neutral_evaluator(neutral_params);
  if (!neutral_evaluator.evaluate(neutral_path, input.position, 0.0,
                                  neutral_geometry) ||
      !neutral_geometry.valid) {
    return false;
  }
  const phase_offset_navigation::ImmutableExecutedReferenceQueryPtr query(
      new PhaseOffsetExecutedReferenceQuery(
          pair->path_owner, 0.0, pair->frame_owner, pair->path_revision,
          pair->frame_revision, pair->source_revision, pair->source_revision));
  phase_offset_navigation::ExecutedReferenceQueryResult reference;
  if (!query->query(prepared.geometry.w, reference) || !reference.valid) {
    return false;
  }
  phase_offset_core::MatchedPortInput matched_input;
  matched_input.geometry = neutral_geometry;
  matched_input.base_v_cmd = base_v_cmd;
  matched_input.base_w_dot = base_w_dot;
  matched_input.final_port = phase_offset_core::PortCommand();
  if (!phase_offset_core::MatchedPort::evaluate(matched_input,
                                                 output.matched) ||
      !output.matched.valid) {
    return false;
  }
  output.geometry = neutral_geometry;
  output.raw_port = phase_offset_core::PortCommand();
  output.projection.final_port = phase_offset_core::PortCommand();
  output.projection.final_w_dot = base_w_dot;
  output.projection.next_delta = 0.0;
  output.projection.valid = true;
  output.current_bounds = prepared.current_bounds;
  output.next_bounds = prepared.current_bounds;
  output.execution = prepared.execution;
  output.execution.mode = phase_offset_navigation::RuntimeExecutionMode::NO_TUBE_REQUIRED;
  output.execution.executable = true;
  output.delta = 0.0;
  output.delta_ref = 0.0;
  output.profile_active = false;
  output.selected = true;
  output.valid = true;

  phase_offset_navigation::ActiveReferenceSnapshot candidate;
  candidate.authority_session = current.authority_session;
  candidate.planner_path_revision = pair->source_revision;
  candidate.executed_path_revision = pair->path_revision;
  candidate.frame_revision = pair->frame_revision;
  candidate.tube_revision = pair->active_profile
      ? pair->active_profile->tube_revision : 0U;
  candidate.profile_revision = pair->active_profile
      ? pair->active_profile->profile_revision : 0U;
  candidate.map_revision = pair->active_profile
      ? pair->active_profile->map_revision : 0U;
  candidate.owner_mode = phase_offset_navigation::ActiveReferenceOwnerMode::PLANNER_ONLY;
  candidate.selected_u_owner = "PlannerOwner";
  candidate.w = prepared.geometry.w;
  candidate.delta = 0.0;
  candidate.dt = prepared.dt;
  candidate.u_prev = runtime_->previousFinalPort();
  candidate.selected_u = phase_offset_core::PortCommand();
  candidate.selected_u_w = 0.0;
  candidate.selected_u_delta = 0.0;
  candidate.proposed_next_w = prepared.geometry.w + prepared.dt * base_w_dot;
  candidate.proposed_next_delta = 0.0;
  candidate.proposed_next_u_prev = candidate.selected_u;
  candidate.r = reference.r;
  candidate.r_w = reference.r_w;
  candidate.r_ww = reference.r_ww;
  candidate.r_ww_valid = reference.r_ww_valid;
  candidate.matched_base_v_cmd = base_v_cmd;
  candidate.matched_base_w_dot = base_w_dot;
  phase_offset_core::NormalFrameQuery neutral_frame;
  if (!pair->frame_owner->query(prepared.geometry.w, neutral_frame) ||
      !neutral_frame.valid) {
    output.selected = false;
    output.valid = false;
    output.invalid_reason = "neutral handoff frame query failed";
    return false;
  }
  candidate.executed_N = neutral_frame.N;
  candidate.executed_reference_query = query;
  candidate.reference_query_revision = query->queryRevision();
  candidate.provenance = "PhaseOffsetMatchedAdapter/atomic-neutral-handoff";
  candidate.safety_status = "NEUTRAL_HANDOFF";
  candidate.handoff_state = "PLANNER_ONLY";
  candidate.valid = true;
  phase_offset_navigation::AuthorityPrepareInput authority_input;
  authority_input.candidate = candidate;
  authority_input.expected_authority_session = current.authority_session;
  authority_input.expected_sequence = current.sequence;
  authority_input.matched_output_valid = true;
  authority_input.reference_valid = candidate.governorViewValid();
  authority_input.provenance = candidate.provenance;
  phase_offset_navigation::AuthorityPreparedStep prepared_authority;
  if (!execution_authority_.prepare(authority_input, prepared_authority)) {
    output.selected = false;
    output.valid = false;
    output.invalid_reason = prepared_authority.failure_reason;
    return false;
  }
  phase_offset_navigation::RuntimeCommitToken token;
  token.expected_previous_final_port = runtime_->previousFinalPort();
  token.expected_delta = runtime_->retainedDelta();
  token.next_previous_final_port = phase_offset_core::PortCommand();
  token.next_delta = 0.0;
  token.dt = prepared.dt;
  token.selected = true;
  token.valid = true;
  token.complete_profile = true;
  // The neutral planner-only publication is reached only after the preceding
  // RecoveryOwner step proved an exact selected-ZOH arrival.  Carry that
  // immutable terminal proof into Runtime's no-fail commit token so the
  // profile lifecycle retires together with the authority handoff.
  token.exact_terminal_predicate = true;
  if (advertised_) {
    if (pending_authority_valid_) return false;
    pending_runtime_commit_ = token;
    pending_authority_prepared_ = prepared_authority;
    pending_authority_session_ = prepared_authority.candidate.authority_session;
    pending_authority_valid_ = true;
    pending_recovery_step_valid_ = false;
    pending_handoff_input_ = phase_offset_navigation::HandoffStateInput();
    pending_handoff_input_.owner_mode =
        phase_offset_navigation::ActiveReferenceOwnerMode::RECOVERY;
    pending_handoff_input_.delta = 0.0;
    pending_handoff_input_.neutral_handoff_committed = true;
    pending_handoff_input_.event =
        phase_offset_navigation::HandoffEvent::ATOMIC_NEUTRAL_HANDOFF;
    if (!handoff_state_machine_.evaluate(
            pending_handoff_input_, pending_handoff_decision_)) {
      clearPendingPositionCommandLocked();
      return false;
    }
    pending_handoff_valid_ = true;
    return true;
  }
  phase_offset_navigation::AuthorityCommitResult authority_result;
  if (!execution_authority_.commit(
          prepared_authority,
          [this, token](const phase_offset_navigation::ActiveReferenceSnapshot&) {
            runtime_->commitTokenNoFail(token);
            phase_offset_navigation::HandoffStateInput handoff_input;
            handoff_input.owner_mode =
                phase_offset_navigation::ActiveReferenceOwnerMode::RECOVERY;
            handoff_input.delta = 0.0;
            handoff_input.neutral_handoff_committed = true;
            handoff_input.event =
                phase_offset_navigation::HandoffEvent::ATOMIC_NEUTRAL_HANDOFF;
            phase_offset_navigation::HandoffDecision decision;
            return handoff_state_machine_.transition(handoff_input, decision);
          },
          authority_result)) {
    output.selected = false;
    output.valid = false;
    output.invalid_reason = authority_result.failure_reason;
    return false;
  }
  return true;
}

bool PhaseOffsetMatchedAdapter::completeThroughRecoveryOwner(
    const MatchedAdapterInput& input,
    const std::shared_ptr<const PathTubePair>& pair,
    const std::shared_ptr<const PathTubePair>& successor_pair,
    const phase_offset_navigation::RuntimePreparedStep& prepared,
    const Eigen::Vector3d& base_v_cmd,
    const double base_w_dot,
    const bool base_guidance_valid,
    phase_offset_navigation::RuntimeStepOutput& output) {
  output = phase_offset_navigation::RuntimeStepOutput();
  const auto fail = [&output](const char* reason) {
    output.selected = false;
    output.valid = false;
    output.invalid_reason = reason ? reason : "recovery transaction rejected";
    return false;
  };
  const bool has_staged_successor = successor_pair && successor_pair != pair;
  const std::shared_ptr<const PathTubePair>& target_pair =
      has_staged_successor ? successor_pair : pair;
  if (!runtime_ || !pair || !pair->active_profile || !pair->path_owner ||
      !pair->frame_owner || !target_pair || !target_pair->active_profile ||
      !target_pair->path_owner || !target_pair->frame_owner ||
      !base_guidance_valid ||
      !prepared.geometry.valid || !IsFinite(prepared.delta) ||
      prepared.delta == 0.0) {
    return fail("recovery precondition is invalid");
  }

  // A production RECOVERY step is a continuation of an already committed
  // authority.  It may not use the pair/session itself as a hidden bootstrap
  // seed: doing so would let Runtime (or a freshly installed pair) mint a
  // RECOVERY owner without an authoritative predecessor.  Unadvertised
  // fixtures retain the explicit seed-only setup used by unit tests.
  const phase_offset_navigation::ActiveReferenceSnapshot current_authority =
      execution_authority_.snapshot();
  if (advertised_) {
    const double epsilon = execution_authority_.config().comparison_epsilon;
    const std::uint64_t live_session =
        authority_session_.load(std::memory_order_acquire);
    const bool owner_mode_compatible =
        current_authority.owner_mode ==
            phase_offset_navigation::ActiveReferenceOwnerMode::NORMAL ||
        current_authority.owner_mode ==
            phase_offset_navigation::ActiveReferenceOwnerMode::COORDINATION ||
        current_authority.owner_mode ==
            phase_offset_navigation::ActiveReferenceOwnerMode::RECOVERY;
    const phase_offset_core::PortCommand runtime_previous =
        runtime_->previousFinalPort();
    const bool runtime_state_compatible =
        std::abs(current_authority.proposed_next_delta - prepared.delta) <=
            epsilon &&
        std::abs(current_authority.proposed_next_u_prev.u_w -
                 runtime_previous.u_w) <= epsilon &&
        std::abs(current_authority.proposed_next_u_prev.u_delta -
                 runtime_previous.u_delta) <= epsilon;
    const auto pair_revision_matches =
        [&current_authority, &pair](
            const std::shared_ptr<const PathTubePair>& candidate_pair) {
      return candidate_pair && candidate_pair->active_profile &&
          current_authority.planner_path_revision == pair->source_revision &&
          current_authority.executed_path_revision == candidate_pair->path_revision &&
          current_authority.frame_revision == candidate_pair->frame_revision &&
          current_authority.tube_revision == candidate_pair->active_profile->tube_revision &&
          current_authority.profile_revision ==
              candidate_pair->active_profile->profile_revision &&
          current_authority.map_revision == candidate_pair->active_profile->map_revision;
    };
    // After the first staged-successor tick, the target pair is the actual
    // execution frame even though the old pair remains live until the CAS.
    // Accept either exact immutable pair identity; never accept independent
    // revisions assembled from unrelated slots.
    const bool pair_revision_compatible = pair_revision_matches(pair) ||
        pair_revision_matches(successor_pair);
    if (!current_authority.valid ||
        current_authority.authority_session == 0U ||
        pair->authority_session == 0U ||
        current_authority.authority_session != pair->authority_session ||
        live_session != pair->authority_session || !owner_mode_compatible ||
        !current_authority.governorViewValid() ||
        !pair_revision_compatible || !runtime_state_compatible) {
      return fail("recovery predecessor authority is stale or incompatible");
    }
  }

  // Preview and Handoff are observational/lifecycle inputs only.  They do
  // not choose a command or clear the existing owner.
  phase_offset_navigation::PreviewFeasibilityInput preview_input;
  preview_input.profile = target_pair->active_profile.get();
  preview_input.current_w = prepared.geometry.w;
  preview_input.current_delta = prepared.delta;
  preview_input.target_w = prepared.geometry.w;
  preview_input.target_delta = 0.0;
  preview_input.dt = prepared.dt;
  preview_input.horizon = std::max(prepared.dt, config_.tube_update_period);
  preview_input.max_phase_rate = std::max(0.0, base_w_dot) +
      config_.u_w_abs_max;
  preview_input.max_delta_rate = config_.u_delta_abs_max;
  preview_input.max_delta_slew = config_.u_delta_rate_max;
  preview_input.previous_u = runtime_->previousFinalPort();
  preview_input.path_revision = target_pair->path_revision;
  preview_input.frame_revision = target_pair->frame_revision;
  preview_input.profile_revision = target_pair->active_profile->profile_revision;
  preview_input.expected_path_revision = target_pair->path_revision;
  preview_input.expected_frame_revision = target_pair->frame_revision;
  preview_input.expected_profile_revision =
      target_pair->active_profile->profile_revision;
  phase_offset_navigation::PreviewFeasibilityResult preview;
  const bool preview_evaluated =
      phase_offset_navigation::PreviewFeasibility::evaluate(
          preview_input, preview);
  if (!preview_evaluated &&
      (preview.status == phase_offset_navigation::PreviewStatus::STALE ||
       preview.status == phase_offset_navigation::PreviewStatus::CURRENT_STATE_UNSAFE)) {
    return fail("recovery preview or handoff was vetoed");
  }
  phase_offset_navigation::TubeBounds target_bounds;
  const bool target_contains_current =
      phase_offset_navigation::TubeFilter::query(
          *target_pair->active_profile, prepared.geometry.w, target_bounds) &&
      target_bounds.valid &&
      prepared.delta >= target_bounds.lower + config_.tube.interior_margin -
          1e-10 &&
      prepared.delta <= target_bounds.upper - config_.tube.interior_margin +
          1e-10;
  const bool target_zero_only = target_pair->active_profile->classification ==
      phase_offset_navigation::TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE;
  const bool target_disconnected = has_staged_successor &&
      (!target_pair->path_owner ||
       prepared.geometry.w < target_pair->path_owner->startW() - 1e-10 ||
       prepared.geometry.w > target_pair->path_owner->endW() + 1e-10 ||
       !target_contains_current);
  const bool target_domain_contains_current = target_pair->path_owner &&
      prepared.geometry.w >= target_pair->path_owner->startW() - 1e-10 &&
      prepared.geometry.w <= target_pair->path_owner->endW() + 1e-10;
  // A staged ZERO_ONLY/disconnected successor is evidence for the handoff
  // decision, not the geometry owner of the current recovery tick.  Recenter
  // must stay inside the currently executed owner's connected component until
  // a later tick can install a compatible successor.
  const bool target_usable_for_current_tick = target_domain_contains_current &&
      target_contains_current && !target_zero_only;
  const std::shared_ptr<const PathTubePair> execution_pair =
      target_usable_for_current_tick ? target_pair : pair;
  phase_offset_navigation::HandoffStateInput handoff_input;
  handoff_input.owner_mode =
      phase_offset_navigation::ActiveReferenceOwnerMode::RECOVERY;
  handoff_input.delta = prepared.delta;
  handoff_input.target_delta = 0.0;
  handoff_input.successor_available = target_pair != nullptr;
  handoff_input.successor_zero_only = target_zero_only;
  handoff_input.successor_contains_current_delta = target_contains_current;
  handoff_input.successor_disconnected = target_disconnected;
  handoff_input.preview_feasible = preview_evaluated && preview.valid;
  handoff_input.preview_target_overlap = preview.target_overlap;
  if (target_zero_only) {
    handoff_input.event = phase_offset_navigation::HandoffEvent::SUCCESSOR_ZERO_ONLY;
  } else if (target_disconnected) {
    handoff_input.event = phase_offset_navigation::HandoffEvent::SUCCESSOR_DISCONNECTED;
  } else if (!preview.valid) {
    handoff_input.event = phase_offset_navigation::HandoffEvent::PREVIEW_INFEASIBLE;
  } else if (has_staged_successor) {
    handoff_input.event = phase_offset_navigation::HandoffEvent::SUCCESSOR_COMPATIBLE;
  } else {
    handoff_input.event = phase_offset_navigation::HandoffEvent::RECENTER_PROGRESS;
  }
  phase_offset_navigation::HandoffDecision handoff;
  if (!handoff_state_machine_.evaluate(handoff_input, handoff) ||
      handoff.planner_veto) {
    return fail("recovery handoff decision was vetoed");
  }
  pending_handoff_input_ = handoff_input;
  pending_handoff_decision_ = handoff;
  pending_handoff_valid_ = true;

  // A staged successor is real immutable evidence, not a fabricated
  // availability flag.  When it differs from the current owner, validate the
  // inherited base-path C2 seam and the required first-order executed
  // reference seam through the one protected continuation provider.  The
  // optional higher-order r_ww capability is deliberately not requested.
  phase_offset_navigation::ImmutableExecutedReferenceQueryPtr source_query(
      new PhaseOffsetExecutedReferenceQuery(
          pair->path_owner, prepared.delta, pair->frame_owner,
          pair->path_revision, pair->frame_revision, pair->source_revision,
          pair->source_revision));
  phase_offset_navigation::ImmutableExecutedReferenceQueryPtr target_query(
      new PhaseOffsetExecutedReferenceQuery(
          target_pair->path_owner, prepared.delta, target_pair->frame_owner,
          target_pair->path_revision, target_pair->frame_revision,
          target_pair->source_revision, target_pair->source_revision));
  if (!source_query || !target_query) return fail("recovery reference query unavailable");
  PhaseOffsetRecoveryContinuationInput continuation_input;
  bool continuation_input_valid = false;
  if (has_staged_successor) {
    const double seam_w = target_pair->future_seam_w;
    if (!IsFinite(seam_w) || seam_w < pair->path_owner->startW() ||
        seam_w > pair->path_owner->endW() ||
        seam_w < target_pair->path_owner->startW() ||
        seam_w > target_pair->path_owner->endW()) {
      return fail("recovery seam is outside source/target domains");
    }
    phase_offset_navigation::RecoveryReferenceJet seam_jet;
    if (!phase_offset_navigation::makeRecoveryReferenceJet(
            *source_query, seam_w, prepared.delta, seam_jet)) {
      return fail("recovery C2 seam proof failed");
    }
    continuation_input.source_path = pair->path_owner;
    continuation_input.source_frame = pair->frame_owner;
    continuation_input.target_path = target_pair->path_owner;
    continuation_input.target_frame = target_pair->frame_owner;
    continuation_input.source_jet = seam_jet;
    continuation_input.source_w = seam_w;
    continuation_input.target_w = seam_w;
    continuation_input.delta = prepared.delta;
    continuation_input.finite_domain_start_w =
        target_pair->path_owner->startW();
    continuation_input.finite_domain_end_w = target_pair->path_owner->endW();
    continuation_input.recovery_session = pair->authority_session;
    continuation_input.source_owner_revision = pair->source_revision;
    continuation_input.target_owner_revision = target_pair->source_revision;
    continuation_input.v_s_min = 1e-3;
    continuation_input.min_time_progress = 0.0;
    // The continuation provider validates immutable seam geometry here.  The
    // actual selected-ZOH phase-rate witness is bound below, after
    // RecoveryOwner has selected and PortProjector has reverified the command;
    // a current/source guidance rate is never relabelled as target evidence.
    continuation_input.validated_s_dot = 0.0;
    continuation_input.s_dot_revision = 0U;
    continuation_input.s_dot_provenance.clear();
    continuation_input.actual_s_dot_valid = false;
    continuation_input.require_time_progress = false;
    continuation_input_valid = true;
    PhaseOffsetRecoveryContinuationOutput continuation_output;
    const bool continuation_valid =
        PhaseOffsetRecoveryContinuationProvider::propose(
            continuation_input, continuation_output);
    if (!continuation_valid || !continuation_output.finite_domain) {
      output.recovery_replan_required = true;
      return fail(continuation_output.invalid_reason.empty()
          ? "recovery continuation proof failed; renewed evidence required"
          : continuation_output.invalid_reason.c_str());
    }
  }

  phase_offset_core::PortProjectionInput projection_input;
  projection_input.raw.u_w = 0.0;
  projection_input.raw.u_delta = -prepared.delta / prepared.dt;
  projection_input.previous_final = runtime_->previousFinalPort();
  projection_input.dt = prepared.dt;
  projection_input.delta = prepared.delta;
  projection_input.curvature = prepared.geometry.curvature;
  projection_input.r_w_norm = prepared.geometry.r_w.norm();
  projection_input.phase = prepared.geometry.w;
  projection_input.base_w_dot = base_w_dot;
  projection_input.base_tangent_speed = prepared.geometry.T.dot(base_v_cmd);
  const phase_offset_navigation::TubeBounds recovery_bounds =
      target_usable_for_current_tick && target_bounds.valid
          ? target_bounds : prepared.current_bounds;
  projection_input.offset_constraint.lower = recovery_bounds.lower;
  projection_input.offset_constraint.upper = recovery_bounds.upper;
  projection_input.offset_constraint.lower_w = recovery_bounds.lower_w;
  projection_input.offset_constraint.upper_w = recovery_bounds.upper_w;
  projection_input.offset_constraint.invariant_gain = config_.tube.invariant_gain;
  projection_input.offset_constraint.interior_margin = config_.tube.interior_margin;
  projection_input.offset_constraint.enabled = recovery_bounds.valid;
  projection_input.offset_constraint.valid = recovery_bounds.valid;
  phase_offset_core::PortProjectionLimits limits;
  limits.u_w_abs_max = config_.u_w_abs_max;
  limits.u_delta_abs_max = config_.u_delta_abs_max;
  limits.u_w_rate_max = config_.u_w_rate_max;
  limits.u_delta_rate_max = config_.u_delta_rate_max;
  limits.phase_dot_min = 0.0;
  limits.tangent_speed_min = 0.0;
  limits.regularity_margin = config_.tube.regularity_margin;
  phase_offset_core::PortProjector::AdmissibleSet admissible;
  if (!phase_offset_core::PortProjector::buildAdmissibleSet(
          projection_input, limits, admissible) ||
      !phase_offset_navigation::PhaseOffsetRecoveryOwner::verifyAdmissibleSet(
          admissible)) {
    return fail("recovery admissible set is not closed and bounded");
  }

  const phase_offset_navigation::ImmutableExecutedReferenceQueryPtr
      execution_query = target_usable_for_current_tick ? target_query
                                                       : source_query;
  phase_offset_navigation::ExecutedReferenceQueryResult reference_result;
  if (!execution_query->query(prepared.geometry.w, reference_result) ||
      !reference_result.valid) {
    return fail("recovery reference jet proof failed");
  }
  phase_offset_navigation::RecoveryReferenceJet reference_jet;
  if (!phase_offset_navigation::makeRecoveryReferenceJet(
          *execution_query, prepared.geometry.w,
          prepared.delta, reference_jet)) {
    return fail("recovery reference jet construction failed");
  }

  phase_offset_navigation::RecoveryPrepareInput recovery_input;
  recovery_input.recovery_session = pair->authority_session;
  recovery_input.source_path_revision = pair->path_revision;
  recovery_input.target_path_revision = execution_pair->path_revision;
  recovery_input.source_frame_revision = pair->frame_revision;
  recovery_input.target_frame_revision = execution_pair->frame_revision;
  recovery_input.source_owner_revision = pair->source_revision;
  recovery_input.target_owner_revision = execution_pair->source_revision;
  recovery_input.reference_jet = reference_jet;
  recovery_input.u_prev = runtime_->previousFinalPort();
  recovery_input.current_w = prepared.geometry.w;
  recovery_input.current_delta = prepared.delta;
  // RECOVERY's immutable target is the planner centreline.  Bind the owner
  // measure to this exact closed interval so forged next-measure/progress
  // scalars cannot influence ranking or terminal handoff.
  recovery_input.target_delta_lower = 0.0;
  recovery_input.target_delta_upper = 0.0;
  recovery_input.target_interval_valid = true;
  recovery_input.base_w_dot = base_w_dot;
  recovery_input.base_w_dot_valid = IsFinite(base_w_dot);
  recovery_input.dt = prepared.dt;
  recovery_input.measure = std::abs(prepared.delta);
  recovery_input.now = input.stamp.toSec();
  if (!IsFinite(recovery_input.now)) recovery_input.now = 0.0;
  recovery_input.v_s_min = 1e-3;
  recovery_input.max_slew_rate = config_.u_delta_rate_max;
  recovery_input.phase_rate_upper_bound = std::max(0.0, base_w_dot) +
      config_.u_w_abs_max;
  recovery_input.phase_rate_upper_bound_valid =
      IsFinite(recovery_input.phase_rate_upper_bound) &&
      recovery_input.phase_rate_upper_bound > 0.0;
  // The guaranteed recovery-rate lower bound is a measure-progress fact
  // owned by RecoveryOwner.  u_delta_abs_max is only an actuator capability
  // upper bound and cannot certify a lower bound on |delta| reduction.
  recovery_input.certified_recovery_rate = recovery_owner_.config().v_rec_min;
  recovery_input.certified_recovery_rate_valid =
      IsFinite(recovery_input.certified_recovery_rate) &&
      recovery_input.certified_recovery_rate > 0.0;
  recovery_input.phase_domain_start = prepared.geometry.w;
  recovery_input.phase_domain_end = execution_pair->path_owner
      ? execution_pair->path_owner->endW() : prepared.geometry.w;
  recovery_input.admissible_set_closed_bounded = true;
  recovery_input.finite_domain_valid =
      IsFinite(recovery_input.phase_domain_start) &&
      IsFinite(recovery_input.phase_domain_end) &&
      recovery_input.phase_domain_end > recovery_input.phase_domain_start;
  const double bounded_ramp_time = config_.u_delta_rate_max > 0.0
      ? config_.u_delta_abs_max / config_.u_delta_rate_max : 0.0;
  const double certified_recovery_duration =
      recovery_input.measure / recovery_input.certified_recovery_rate +
      bounded_ramp_time + std::max(prepared.dt, config_.tube_update_period);
  recovery_input.required_domain_duration = certified_recovery_duration;
  recovery_input.require_time_progress = true;
  recovery_input.time_progress_valid = IsFinite(base_w_dot) &&
      recovery_input.phase_rate_upper_bound_valid;
  const std::uint64_t recovery_session = pair->authority_session;
  if (recovery_deadline_session_ == recovery_session &&
      recovery_deadline_target_revision_ != 0U &&
      recovery_deadline_target_revision_ != target_pair->path_revision) {
    return fail("recovery deadline target revision changed; replan required");
  }
  if (recovery_deadline_session_ != recovery_session ||
      !IsFinite(recovery_deadline_)) {
    recovery_deadline_ =
        phase_offset_navigation::PhaseOffsetRecoveryOwner::computeBoundedDeadline(
            recovery_input.now, recovery_input.measure,
            recovery_input.certified_recovery_rate, 0.0,
            bounded_ramp_time,
            prepared.dt, config_.u_delta_abs_max,
            config_.u_delta_rate_max,
            recovery_input.phase_domain_start,
            recovery_input.phase_domain_end);
    recovery_deadline_session_ = recovery_session;
    recovery_deadline_target_revision_ = target_pair->path_revision;
  }
  recovery_input.deadline = recovery_deadline_;
  recovery_input.source_reference_matches = true;
  recovery_input.provenance = "PhaseOffsetMatchedAdapter/Preview-Handoff-Recovery";

  // PortProjector's local 2-D polygon is intentionally tiny.  Keep an
  // explicit bound here so a malformed/custom kernel cannot turn recovery
  // into an unbounded search on the control callback.
  constexpr std::size_t kMaxRecoveryVertices = 64U;
  if (admissible.vertices.size() > kMaxRecoveryVertices) {
    return fail("recovery admissible set exceeds bounded vertex budget");
  }
  recovery_input.candidates.reserve(kMaxRecoveryVertices);
  for (const phase_offset_core::PortCommand& command : admissible.vertices) {
    phase_offset_core::PortProjectionResult checked;
    if (!phase_offset_core::PortProjector::verify(
            projection_input, limits, command, checked) || !checked.valid) {
      continue;
    }
    phase_offset_navigation::RecoveryCandidate candidate;
    candidate.command = command;
    candidate.next_w = prepared.geometry.w + prepared.dt * checked.final_w_dot;
    candidate.next_delta = checked.next_delta;
    candidate.measure = recovery_input.measure;
    candidate.next_measure_upper_bound = std::abs(checked.next_delta);
    candidate.progress = candidate.measure - candidate.next_measure_upper_bound;
    candidate.progress_bound_valid = true;
    candidate.forward_progress = checked.final_w_dot;
    candidate.slew_cost = config_.u_w_rate_max *
        std::pow(command.u_w - recovery_input.u_prev.u_w, 2.0) +
        config_.u_delta_rate_max *
            std::pow(command.u_delta - recovery_input.u_prev.u_delta, 2.0);
    candidate.admissible_u_w_lower = admissible.u_w.lower;
    candidate.admissible_u_w_upper = admissible.u_w.upper;
    candidate.admissible_u_delta_lower = admissible.u_delta.lower;
    candidate.admissible_u_delta_upper = admissible.u_delta.upper;
    candidate.command_membership_valid = admissible.contains(command);
    candidate.closed_bounded_set_valid = admissible.closed && admissible.bounded;
    candidate.phase_domain_start = recovery_input.phase_domain_start;
    candidate.phase_domain_end = recovery_input.phase_domain_end;
    candidate.reference_jet = reference_jet;
    const bool strict_exact_arrival = std::abs(checked.next_delta) <= 1e-14;
    candidate.provenance = strict_exact_arrival
        ? "PortProjector/exact-selected-ZOH-arrival"
        : "PortProjector/closed-bounded-membership";
    candidate.progress_provenance = "RecoveryOwner/P_k=recomputed";
    candidate.s_dot = checked.final_w_dot;
    candidate.v_s_min = recovery_input.v_s_min;
    candidate.minimum_time_progress = recovery_input.v_s_min * prepared.dt;
    candidate.s_dot_revision = recovery_input.target_owner_revision;
    candidate.actual_s_dot_valid = IsFinite(candidate.s_dot);
    candidate.require_time_progress = recovery_input.require_time_progress;
    candidate.s_dot_provenance = recovery_input.provenance;
    candidate.exact_terminal_predicate = strict_exact_arrival;
    candidate.terminal_predicate = candidate.exact_terminal_predicate;
    candidate.time_progress_valid = candidate.s_dot >=
        recovery_input.v_s_min && candidate.s_dot * prepared.dt >=
        candidate.minimum_time_progress;
    candidate.finite_domain_valid = candidate.next_w <=
        recovery_input.phase_domain_end + 1e-10;
    if (candidate.progress >= 0.0 && candidate.time_progress_valid &&
        candidate.finite_domain_valid) {
      recovery_input.candidates.push_back(candidate);
    }
  }
  // When the remaining residual is reachable in one selected ZOH interval,
  // include the exact-arrival command in the same deterministic candidate
  // set.  This is not a tolerance snap: the recurrence is verified with the
  // exact command and only the ranked immutable candidate may become owner.
  if (std::abs(prepared.delta) <= config_.u_delta_abs_max * prepared.dt) {
    for (const phase_offset_core::PortCommand& base_command :
         admissible.vertices) {
      phase_offset_core::PortCommand exact_command = base_command;
      exact_command.u_delta = -prepared.delta / prepared.dt;
      if (!admissible.contains(exact_command)) continue;
      phase_offset_core::PortProjectionResult checked;
      if (!phase_offset_core::PortProjector::verify(
              projection_input, limits, exact_command, checked) ||
          !checked.valid || std::abs(checked.next_delta) > 1e-14) {
        continue;
      }
      phase_offset_navigation::RecoveryCandidate candidate;
      candidate.command = exact_command;
      candidate.next_w = prepared.geometry.w + prepared.dt * checked.final_w_dot;
      candidate.next_delta = 0.0;
      candidate.measure = recovery_input.measure;
      candidate.next_measure_upper_bound = 0.0;
      candidate.progress = candidate.measure;
      candidate.progress_bound_valid = true;
      candidate.forward_progress = checked.final_w_dot;
      candidate.slew_cost = config_.u_w_rate_max *
          std::pow(exact_command.u_w - recovery_input.u_prev.u_w, 2.0) +
          config_.u_delta_rate_max *
          std::pow(exact_command.u_delta - recovery_input.u_prev.u_delta, 2.0);
      candidate.admissible_u_w_lower = admissible.u_w.lower;
      candidate.admissible_u_w_upper = admissible.u_w.upper;
      candidate.admissible_u_delta_lower = admissible.u_delta.lower;
      candidate.admissible_u_delta_upper = admissible.u_delta.upper;
      candidate.command_membership_valid = admissible.contains(exact_command);
      candidate.closed_bounded_set_valid = admissible.closed && admissible.bounded;
      candidate.phase_domain_start = recovery_input.phase_domain_start;
      candidate.phase_domain_end = recovery_input.phase_domain_end;
      candidate.reference_jet = reference_jet;
      candidate.provenance = "PortProjector/exact-selected-ZOH-arrival";
      candidate.progress_provenance = "RecoveryOwner/P_k=recomputed";
      candidate.s_dot = checked.final_w_dot;
      candidate.v_s_min = recovery_input.v_s_min;
      candidate.minimum_time_progress = recovery_input.v_s_min * prepared.dt;
      candidate.s_dot_revision = recovery_input.target_owner_revision;
      candidate.actual_s_dot_valid = IsFinite(candidate.s_dot);
      candidate.require_time_progress = recovery_input.require_time_progress;
      candidate.s_dot_provenance = recovery_input.provenance;
      candidate.exact_terminal_predicate = true;
      candidate.terminal_predicate = true;
      candidate.time_progress_valid = candidate.s_dot >= recovery_input.v_s_min &&
          candidate.s_dot * prepared.dt >= candidate.minimum_time_progress;
      candidate.finite_domain_valid = candidate.next_w <=
          recovery_input.phase_domain_end + 1e-10;
      if (candidate.time_progress_valid && candidate.finite_domain_valid) {
        recovery_input.candidates.push_back(candidate);
      }
      break;
    }
  }
  phase_offset_navigation::RecoveryPreparedStep recovery_step;
  if (!recovery_owner_.prepare(recovery_input, recovery_step)) {
    output.recovery_status = recovery_step.status;
    output.recovery_replan_required =
        recovery_step.status ==
        phase_offset_navigation::RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED;
    return fail(recovery_step.proof.empty()
        ? "recovery owner rejected progress/domain evidence"
        : recovery_step.proof.c_str());
  }
  output.recovery_status = phase_offset_navigation::RecoveryStepStatus::PREPARED;

  phase_offset_core::PortProjectionResult selected_projection;
  if (!phase_offset_core::PortProjector::verify(
          projection_input, limits, recovery_step.selected_u,
          selected_projection) || !selected_projection.valid) {
    return fail("recovery selected command failed exact projection verification");
  }
  if (has_staged_successor && execution_pair == target_pair &&
      continuation_input_valid) {
    // RecoveryOwner has now selected and PortProjector has reverified the
    // exact ZOH command.  Close the time-progress contract with that actual
    // selected s_dot and the target pair revision; the source guidance rate
    // above was never accepted as target progress evidence.
    const double selected_s_dot = selected_projection.final_w_dot;
    PhaseOffsetRecoveryContinuationOutput selected_progress_output;
    if (!PhaseOffsetRecoveryContinuationProvider::validateSelectedTimeProgress(
            continuation_input, selected_s_dot,
            execution_pair->source_revision, recovery_input.provenance,
            selected_progress_output)) {
      output.recovery_replan_required = true;
      return fail(selected_progress_output.invalid_reason.empty()
          ? "selected recovery time-progress proof failed"
          : selected_progress_output.invalid_reason.c_str());
    }
  }
  phase_offset_core::MatchedPortInput matched_input;
  matched_input.geometry = prepared.geometry;
  matched_input.base_v_cmd = base_v_cmd;
  matched_input.base_w_dot = base_w_dot;
  matched_input.final_port = recovery_step.selected_u;
  if (!phase_offset_core::MatchedPort::evaluate(matched_input,
                                                 output.matched) ||
      !output.matched.valid) {
    return fail("recovery matched output is invalid");
  }
  output.geometry = prepared.geometry;
  output.projection = selected_projection;
  output.current_bounds = recovery_bounds;
  output.next_bounds = recovery_bounds;
  output.execution = prepared.execution;
  output.execution.mode = phase_offset_navigation::RuntimeExecutionMode::SAFETY_PRIORITY;
  output.execution.executable = true;
  output.delta = prepared.delta;
  output.delta_ref = 0.0;
  output.profile_active = false;
  output.selected = true;
  output.valid = true;

  phase_offset_navigation::ActiveReferenceSnapshot candidate;
  candidate.authority_session = current_authority.valid
      ? current_authority.authority_session : pair->authority_session;
  candidate.planner_path_revision = pair->source_revision;
  candidate.executed_path_revision = execution_pair->path_revision;
  candidate.frame_revision = execution_pair->frame_revision;
  candidate.tube_revision = execution_pair->active_profile->tube_revision;
  candidate.profile_revision = execution_pair->active_profile->profile_revision;
  candidate.map_revision = execution_pair->active_profile->map_revision;
  candidate.owner_mode =
      phase_offset_navigation::ActiveReferenceOwnerMode::RECOVERY;
  candidate.selected_u_owner = "PhaseOffsetRecoveryOwner";
  candidate.w = prepared.geometry.w;
  candidate.delta = prepared.delta;
  candidate.dt = prepared.dt;
  candidate.u_prev = recovery_input.u_prev;
  candidate.selected_u = recovery_step.selected_u;
  candidate.selected_u_w = candidate.selected_u.u_w;
  candidate.selected_u_delta = candidate.selected_u.u_delta;
  candidate.proposed_next_w = recovery_step.next_w;
  candidate.proposed_next_delta = recovery_step.next_delta;
  candidate.proposed_next_u_prev = candidate.selected_u;
  candidate.r = reference_result.r;
  candidate.r_w = reference_result.r_w;
  candidate.r_ww = reference_result.r_ww;
  candidate.r_ww_valid = reference_result.r_ww_valid;
  candidate.matched_base_v_cmd = base_v_cmd;
  candidate.matched_base_w_dot = base_w_dot;
  phase_offset_core::NormalFrameQuery execution_frame_query;
  if (!execution_pair->frame_owner->query(prepared.geometry.w,
                                           execution_frame_query) ||
      !execution_frame_query.valid ||
      (execution_pair == pair &&
       (execution_frame_query.N - prepared.geometry.N).norm() > 1e-6)) {
    output.selected = false;
    output.valid = false;
    output.invalid_reason =
        "recovery executed frame is not bound to the selected immutable owner";
    return fail("recovery executed frame is not bound to selected owner");
  }
  candidate.executed_N = execution_frame_query.N;
  candidate.executed_reference_query = execution_query;
  candidate.reference_query_revision = execution_query->queryRevision();
  candidate.provenance = recovery_input.provenance;
  candidate.safety_status = "RECOVERY";
  candidate.handoff_state = "RECOVERY_OWNER";
  candidate.valid = true;

  phase_offset_navigation::AuthorityPrepareInput authority_input;
  authority_input.candidate = candidate;
  if (current_authority.valid) {
    authority_input.expected_authority_session =
        current_authority.authority_session;
    authority_input.expected_sequence = current_authority.sequence;
  }
  authority_input.matched_output_valid = output.valid && output.matched.valid;
  authority_input.reference_valid = candidate.governorViewValid();
  authority_input.provenance = recovery_input.provenance;
  phase_offset_navigation::AuthorityPreparedStep authority_prepared;
  if (!execution_authority_.prepare(authority_input, authority_prepared)) {
    output.selected = false;
    output.valid = false;
    output.invalid_reason = authority_prepared.failure_reason;
    return fail("recovery authority preparation failed");
  }
  // The unadvertised fixture path uses the same prepare-before-publication
  // contract as the production pending transaction.  Validate every
  // fallible seam before authority publication; the callback below performs
  // only no-fail state writes.
  if (!recovery_owner_.validateCommit(recovery_step)) {
    return fail("recovery owner commit token validation failed");
  }
  const phase_offset_navigation::HandoffDecision prepared_handoff = handoff;
  phase_offset_navigation::RuntimeCommitToken token;
  token.expected_previous_final_port = recovery_input.u_prev;
  token.expected_delta = prepared.delta;
  token.next_previous_final_port = recovery_step.selected_u;
  token.next_delta = recovery_step.next_delta;
  token.dt = prepared.dt;
  token.selected = true;
  token.valid = true;
  token.safety_priority = true;
  if (advertised_) {
    if (pending_authority_valid_) return fail("previous PositionCommand transaction is pending");
    pending_runtime_commit_ = token;
    pending_recovery_step_ = recovery_step;
    pending_recovery_step_valid_ = true;
    pending_recovery_source_pair_ = pair;
    pending_recovery_execution_pair_ = execution_pair;
    pending_recovery_target_pair_ = target_pair;
    pending_authority_prepared_ = authority_prepared;
    pending_authority_session_ = authority_prepared.candidate.authority_session;
    pending_authority_valid_ = true;
  } else {
    phase_offset_navigation::AuthorityCommitResult authority_result;
    if (!execution_authority_.commit(
            authority_prepared,
            [this, token, recovery_step, prepared_handoff](
                const phase_offset_navigation::ActiveReferenceSnapshot&) {
              recovery_owner_.commitNoFail(recovery_step);
              handoff_state_machine_.commitNoFail(prepared_handoff);
              runtime_->commitTokenNoFail(token);
              return true;
            }, authority_result)) {
      output.selected = false;
      output.valid = false;
      output.invalid_reason = authority_result.failure_reason;
      return fail("recovery authority commit failed");
    }
  }
  return true;
}

bool PhaseOffsetMatchedAdapter::hasPendingPositionCommand() const {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  return pending_authority_valid_ && pending_runtime_commit_.valid;
}

void PhaseOffsetMatchedAdapter::clearPendingPositionCommandLocked() {
  pending_runtime_commit_ = phase_offset_navigation::RuntimeCommitToken();
  pending_recovery_step_ = phase_offset_navigation::RecoveryPreparedStep();
  pending_recovery_step_valid_ = false;
  pending_handoff_input_ = phase_offset_navigation::HandoffStateInput();
  pending_handoff_decision_ = phase_offset_navigation::HandoffDecision();
  pending_handoff_valid_ = false;
  pending_recovery_source_pair_.reset();
  pending_recovery_execution_pair_.reset();
  pending_recovery_target_pair_.reset();
  pending_authority_prepared_ =
      phase_offset_navigation::AuthorityPreparedStep();
  pending_authority_session_ = 0U;
  pending_authority_valid_ = false;
}

bool PhaseOffsetMatchedAdapter::validatePendingPositionCommandLocked(
    std::string* reason) const {
  if (reason) reason->clear();
  if (!pending_authority_valid_ || !pending_runtime_commit_.valid) return true;
  if (!execution_authority_.finalValidate(pending_authority_prepared_, reason)) {
    return false;
  }
  const phase_offset_navigation::ActiveReferenceSnapshot current =
      execution_authority_.snapshot();
  if (!pending_authority_prepared_.committed_snapshot) {
    if (reason) *reason = "pending authority snapshot is unavailable";
    return false;
  }
  const phase_offset_navigation::ActiveReferenceSnapshot& candidate =
      *pending_authority_prepared_.committed_snapshot;
  const double epsilon = execution_authority_.config().comparison_epsilon;
  if (pending_authority_session_ == 0U ||
      pending_authority_session_ !=
          authority_session_.load(std::memory_order_acquire)) {
    if (reason) *reason = "pending authority session is stale";
    return false;
  }
  const std::shared_ptr<const PathTubePair> live_pair =
      std::atomic_load(&authoritative_path_tube_pair_);
  if (candidate.owner_mode ==
          phase_offset_navigation::ActiveReferenceOwnerMode::RECOVERY) {
    const auto pair_matches_candidate =
        [&candidate, this](const std::shared_ptr<const PathTubePair>& pair) {
          return pair && pair->authority_session == pending_authority_session_ &&
              pair->path_revision == candidate.executed_path_revision &&
              pair->frame_revision == candidate.frame_revision &&
              pair->active_profile &&
              pair->active_profile->profile_revision == candidate.profile_revision &&
              pair->active_profile->tube_revision == candidate.tube_revision &&
              pair->active_profile->map_revision == candidate.map_revision;
        };
    // The recovery candidate is bound to the exact immutable pair whose
    // geometry/reference query produced it.  Revisions alone are not an
    // identity: a same-revision clone in the live slot must fail closed.
    if (!pending_recovery_execution_pair_ ||
        !pair_matches_candidate(pending_recovery_execution_pair_) ||
        !pending_recovery_source_pair_ ||
        pending_recovery_source_pair_->authority_session !=
            pending_authority_session_ ||
        pending_recovery_source_pair_->source_revision !=
            candidate.planner_path_revision) {
      if (reason) *reason = "pending recovery execution pair is stale";
      return false;
    }
    // A staged successor may not be live yet, so the exact source predecessor
    // remains an allowed live identity.  No unrelated pair (including a clone
    // with identical revisions) can satisfy this boundary.
    if (live_pair != pending_recovery_execution_pair_ &&
        live_pair != pending_recovery_source_pair_) {
      if (reason) *reason = "pending recovery live pair identity changed";
      return false;
    }
    if (live_pair == pending_recovery_source_pair_ &&
        pending_recovery_execution_pair_ != pending_recovery_source_pair_ &&
        pending_recovery_target_pair_ != pending_recovery_execution_pair_) {
      if (reason) *reason = "pending recovery target identity changed";
      return false;
    }
  }
  if (current.snapshotId() != pending_authority_prepared_.expected_snapshot_id) {
    if (reason) *reason = "pending authority snapshot changed";
    return false;
  }
  if (std::abs(runtime_->retainedDelta() -
               pending_runtime_commit_.expected_delta) > epsilon ||
      std::abs(runtime_->previousFinalPort().u_w -
               pending_runtime_commit_.expected_previous_final_port.u_w) >
          epsilon ||
      std::abs(runtime_->previousFinalPort().u_delta -
               pending_runtime_commit_.expected_previous_final_port.u_delta) >
          epsilon || !candidate.selectedUConsistent(epsilon)) {
    if (reason) *reason = "pending Runtime state or exact selected-u changed";
    return false;
  }
  return true;
}

void PhaseOffsetMatchedAdapter::discardPendingPositionCommand() {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  clearPendingPositionCommandLocked();
}

bool PhaseOffsetMatchedAdapter::validatePendingPositionCommand() const {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  return validatePendingPositionCommandLocked(nullptr);
}

PendingPositionCommandCapture
PhaseOffsetMatchedAdapter::capturePendingPositionCommand() const {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  PendingPositionCommandCapture capture;
  if (!pending_authority_valid_ || !pending_runtime_commit_.valid) {
    capture.valid = true;
    return capture;
  }
  capture.pending = true;
  if (pending_authority_prepared_.committed_snapshot) {
    capture.identity =
        pending_authority_prepared_.committed_snapshot->snapshotId();
    capture.reference_query =
        pending_authority_prepared_.committed_snapshot->executed_reference_query;
  }
  if (capture.identity == 0U) {
    // Preserve the distinction between "no transaction" (identity zero) and
    // a malformed/stale transaction that must suppress ordinary publication.
    capture.identity = std::numeric_limits<std::uint64_t>::max();
  }
  capture.valid = capture.identity != 0U &&
      static_cast<bool>(capture.reference_query) &&
      validatePendingPositionCommandLocked(nullptr);
  return capture;
}

phase_offset_navigation::ImmutableExecutedReferenceQueryPtr
PhaseOffsetMatchedAdapter::pendingExecutedReferenceQuery() const {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  if (!pending_authority_valid_ || !pending_runtime_commit_.valid ||
      !pending_authority_prepared_.committed_snapshot) {
    return phase_offset_navigation::ImmutableExecutedReferenceQueryPtr();
  }
  const std::shared_ptr<const phase_offset_navigation::ActiveReferenceSnapshot>&
      snapshot = pending_authority_prepared_.committed_snapshot;
  return snapshot->executed_reference_query;
}

bool PhaseOffsetMatchedAdapter::publishPendingPositionCommand(
    const std::function<bool()>& local_publish,
    const std::uint64_t expected_identity,
    const bool cancel_pending_for_goal_override) {
  // Reset/new-task and timer publication take task_publication_mutex_ before
  // runtime_command_mutex_.  Holding both through the local publish closes
  // the validate→publish→commit race without adding a TransactionManager.
  std::unique_lock<std::mutex> task_lock(task_publication_mutex_);
  std::unique_lock<std::mutex> runtime_lock(runtime_command_mutex_);
  if (!pending_authority_valid_ || !pending_runtime_commit_.valid) {
    // If a transaction was present before the governor ran, its disappearance
    // is a fail-closed race outcome.  Ordinary publication is allowed only
    // when the capture explicitly observed no pending transaction.
    if (cancel_pending_for_goal_override) {
      DeactivationCommitToken deactivation_token;
      if (!prepareDeactivationLocked(
              ros::Time::now(), task_generation_.load(std::memory_order_acquire),
              deactivation_token)) {
        return false;
      }
      if (!local_publish || !local_publish()) return false;
      commitDeactivationNoFailLocked(deactivation_token);
      return true;
    }
    return expected_identity == 0U && local_publish ? local_publish() : false;
  }
  if ((!cancel_pending_for_goal_override && expected_identity == 0U) ||
      !pending_authority_prepared_.committed_snapshot ||
      (!cancel_pending_for_goal_override &&
       pending_authority_prepared_.committed_snapshot->snapshotId() !=
           expected_identity)) {
    clearPendingPositionCommandLocked();
    return false;
  }
  std::string reason;
  if (!validatePendingPositionCommandLocked(&reason)) {
    clearPendingPositionCommandLocked();
    return false;
  }
  if (pending_recovery_step_valid_ &&
      !recovery_owner_.validateCommit(pending_recovery_step_)) {
    clearPendingPositionCommandLocked();
    return false;
  }
  if (pending_handoff_valid_ &&
      (pending_handoff_decision_.previous_state !=
           handoff_state_machine_.state() ||
       pending_handoff_decision_.transition_sequence !=
           handoff_state_machine_.transitionSequence() + 1U)) {
    clearPendingPositionCommandLocked();
    return false;
  }

  // Prepare the refreshed immutable control handoff before local publication.
  // The post-publication path below performs only no-fail value/shared_ptr
  // stores; it never allocates or copies strings after cmd_pub.publish().
  std::shared_ptr<const ControlPublishSnapshot> refreshed_control;
  const std::shared_ptr<const ControlPublishSnapshot> control =
      std::atomic_load(&latest_control_snapshot_);
  if (control && control->active &&
      control->task_generation == task_generation_.load(
          std::memory_order_acquire)) {
    std::shared_ptr<ControlPublishSnapshot> refreshed(
        new ControlPublishSnapshot(*control));
    refreshed->authority_snapshot =
        pending_authority_prepared_.committed_snapshot;
    refreshed_control = std::shared_ptr<const ControlPublishSnapshot>(
        refreshed);
  }

  const phase_offset_navigation::RuntimeCommitToken token =
      pending_runtime_commit_;
  const phase_offset_navigation::AuthorityPreparedStep prepared =
      pending_authority_prepared_;
  const phase_offset_navigation::RecoveryPreparedStep recovery_step =
      pending_recovery_step_;
  const bool recovery_step_valid = pending_recovery_step_valid_;
  const bool handoff_input_valid = pending_handoff_valid_;
  DeactivationCommitToken deactivation_token;
  if (cancel_pending_for_goal_override) {
    if (!prepareDeactivationLocked(
            ros::Time::now(), task_generation_.load(std::memory_order_acquire),
            deactivation_token)) {
      return false;
    }
  }
  const bool published = local_publish && local_publish();
  if (!published) {
    // A failed goal-override publication must preserve the staged authority
    // and deactivation state for a retry; no state is retired at this seam.
    if (!cancel_pending_for_goal_override) clearPendingPositionCommandLocked();
    return false;
  }
  if (cancel_pending_for_goal_override) {
    commitDeactivationNoFailLocked(deactivation_token);
    return true;
  }
  // All fallible checks have completed before local publication.  The
  // post-publication section is intentionally limited to no-fail value/state
  // writes; it cannot allocate, revalidate, rank, or call a second owner.
  execution_authority_.commitNoFail(prepared);
  if (recovery_step_valid) recovery_owner_.commitNoFail(recovery_step);
  if (handoff_input_valid) {
    handoff_state_machine_.commitNoFail(pending_handoff_decision_);
  }
  runtime_->commitTokenNoFail(token);
  const bool committed = true;
  if (refreshed_control) {
    std::atomic_store(&latest_control_snapshot_, refreshed_control);
  }
  clearPendingPositionCommandLocked();
  return committed;
}

bool PhaseOffsetMatchedAdapter::update(const MatchedAdapterInput& input, MatchedAdapterOutput& output) {
  output = MatchedAdapterOutput();
  const std::uint64_t task_generation =
      task_generation_.load(std::memory_order_acquire);
  if (shutdown_requested_.load(std::memory_order_acquire) || !configuration_valid_) {
    output.invalid_reason = "matched adapter configuration is invalid or shutting down";
    return false;
  }
  std::lock_guard<std::mutex> command_lock(runtime_command_mutex_);
  if (task_generation != task_generation_.load(std::memory_order_acquire)) {
    output.invalid_reason = "matched adapter task retired";
    return false;
  }
  const bool input_available = updateGate(input, output);
  if (!input_available) {
    // Do not let an old complete request keep rebuilding while the command
    // side has no valid path/legacy facts.  This only clears async exposure;
    // updateGate deliberately left Runtime, gate count, and latch unchanged.
    if (requiresTubeTimer()) deactivateLocked(input.stamp, task_generation);
    return false;
  }
  const bool equivalent = output.zero_comparison.valid && output.zero_comparison.equivalent;
  if (config_.mode == PhaseOffsetMatchedMode::ACTIVE) {
    output.selected = zero_gate_open_ && !failure_latched_ && equivalent;
    output.valid = output.zero_port.valid && output.zero_comparison.valid;
    output.invalid_reason = output.selected ? std::string() : output.zero_comparison.invalid_reason;
    if (advertised_) {
      const auto values = PhaseOffsetActiveAdapter::makeDiagnostics(output.zero_port, output.zero_comparison,
          zero_gate_open_, zero_gate_consecutive_count_, failure_latched_, output.selected);
      std_msgs::Float64MultiArray diagnostics; diagnostics.data.assign(values.begin(), values.end()); active_diagnostics_pub_.publish(diagnostics);
    }
    return output.selected;
  }
  // A captured H2 pair is the exclusive Runtime authority.  The async epoch
  // slots below remain display/evidence only and are deliberately not read by
  // this branch.  Pair revision was assigned only by a successful CAS.
  if (input.path_tube_pair) {
    const std::shared_ptr<const PathTubePair>& pair = input.path_tube_pair;
    // Retirement is distinct from malformed pair evidence: reset advances
    // this private ownership session while a command may still hold an old
    // immutable shared_ptr.  Reject it before publishing a timer request or
    // touching Runtime so that the captured authority cannot be revived.
    if (pair->authority_session !=
        authority_session_.load(std::memory_order_acquire)) {
      output.invalid_reason = "captured path-tube pair retired";
      return false;
    }
    if (!pair->path_owner || !pair->full_path_samples ||
        !pair->active_profile || pair->source_revision == 0U ||
        pair->epoch_status.candidate_path_source_revision !=
            pair->source_revision ||
        pair->epoch_status.active_path_source_revision !=
            pair->source_revision ||
        !pair->epoch_status.active_available ||
        !pair->epoch_status.active_current_validation_valid ||
        pair->active_profile->source_revision != pair->source_revision ||
        !PathStateMatchesOwner(input.path, pair->path_owner,
                               pair->frame_owner)) {
      output.invalid_reason = "captured path-tube pair is invalid";
      return false;
    }
    phase_offset_core::PathDifferentialState authoritative_current_path;
    if (!EvaluateOwnerState(pair->path_owner, input.path.w,
                            authoritative_current_path, pair->frame_owner)) {
      output.invalid_reason = "captured immutable pair frame query failed";
      return false;
    }
    // A same-path timer refresh is evidence-only until it atomically replaces
    // this exact pair.  Publish the immutable owner and captured Runtime bits
    // as a lightweight request; the timer never reads/mutates Runtime.
    std::shared_ptr<TubeBuildRequest> request(new TubeBuildRequest());
    request->active = true;
    request->task_generation =
        task_generation_.load(std::memory_order_acquire);
    request->control_sequence = ++control_sequence_;
    request->source_revision = pair->source_revision;
    request->authority_session = pair->authority_session;
    request->stamp = input.stamp;
    request->semantic_path_owner = pair->path_owner;
    request->frame_owner = pair->frame_owner;
    request->base_path_tube_pair = pair;
    request->base_path_tube_pair_generation = pair->generation;
    request->base_retained_delta = runtime_->retainedDelta();
    request->base_previous_final_port = runtime_->previousFinalPort();
    request->semantic_path_start_w = pair->path_owner->startW();
    request->semantic_path_end_w = pair->path_owner->endW();
    request->supplied_path_samples = pair->full_path_samples;
    request->current_path = authoritative_current_path;
    request->position = input.position;
    request->gains = input.gains;
    request->dt = input.dt;
    request->retained_delta = request->base_retained_delta;
    if (config_.tube_source == phase_offset_navigation::TubeSource::ESDF) {
      makeAuthorityRequest(request->retained_delta,
                           request->authority_request);
    }
    request->cloud_snapshot = input.cloud_occupancy_snapshot;
    if (config_.tube_source == phase_offset_navigation::TubeSource::ESDF) {
      request->map_observation_is_snapshot = true;
      request->map_observation_sequence = request->cloud_snapshot
          ? request->cloud_snapshot->observation_sequence : 0U;
    }
    command_active_ = true;
    std::atomic_store(&latest_build_request_,
        std::shared_ptr<const TubeBuildRequest>(request));
    output.candidate_profile = pair->epoch_snapshot &&
        pair->epoch_snapshot->candidate_profile
        ? pair->epoch_snapshot->candidate_profile : pair->active_profile;
    output.active_profile = pair->active_profile;
    output.tube_epoch_status = pair->epoch_status;
    // Per-command categorical evidence is intentionally not persisted into
    // the immutable pair.  It masks only this Runtime input so an explicit
    // latest OCCUPIED/OUT_OF_MAP fact reaches the existing certificate-denied
    // fail-closed path without creating a new state/reason/schema.
    if (LatestCategoricalUnsafe(config_, cloud_occupancy_query_config_,
                                input.cloud_occupancy_snapshot, input.path,
                                input.position, runtime_->retainedDelta())) {
      output.tube_epoch_status.active_current_validation_valid = false;
      output.tube_epoch_status.certificate_denied = true;
      output.tube_epoch_status.current_safety_status =
          phase_offset_navigation::CurrentSafetyStatus::UNSAFE;
    }
    phase_offset_navigation::RuntimePreflightInput preflight;
    preflight.position = input.position;
    preflight.path_source_revision = pair->source_revision;
    preflight.path = *pair->full_path_samples;
    if (!have_preflight_revision_ || last_preflight_revision_ !=
        pair->source_revision) {
      runtime_->refreshPreflight(preflight);
      have_preflight_revision_ = true;
      last_preflight_revision_ = pair->source_revision;
    }
    phase_offset_navigation::RuntimePrepareInput runtime_input;
    runtime_input.current_path = authoritative_current_path;
    runtime_input.position = input.position;
    runtime_input.tube_view.active_profile = pair->active_profile;
    runtime_input.tube_view.epoch_status = output.tube_epoch_status;
    runtime_input.dt = input.dt;
    runtime_input.future_step = MakeFutureStepContract(
        pair->path_owner, pair->full_path_samples, pair->active_profile,
        input.gains,
        runtime_->config().tube.regularity_margin,
        runtime_->config().tube.minimum_reference_speed,
        config_.tube_update_period,
        config_.tube.min_certified_forward_w);
    // A pending manual profile may execute only through a matching immutable
    // pair.  If the pair was retired during a neutral planner handoff, keep
    // the sidecar Tube observable but do not let Runtime start a nonzero
    // profile from an unowned epoch.
    const bool pair_available = static_cast<bool>(capturePathTubePair());
    runtime_input.zero_gate_open = zero_gate_open_ && !failure_latched_ &&
        !config_.observe_only &&
        (!requiresPathTubePairBootstrapLocked() || pair_available);
    runtime_input.fatal_adapter_failure_latched = failure_latched_;
    phase_offset_navigation::RuntimePreparedStep prepared;
    runtime_->prepare(runtime_input, prepared);
    output.geometry = prepared.geometry.valid ? prepared.geometry : output.geometry;
    output.preflight = prepared.preflight;
    output.raw_port = prepared.raw_port;
    output.delta = prepared.delta;
    output.delta_ref = prepared.delta_ref;
    output.profile_active = prepared.profile_active;
    output.tube_current_bounds = prepared.current_bounds;
    output.runtime_execution = prepared.execution;
    phase_offset_navigation::RuntimeStepOutput runtime_output;
    bool completed = false;
    if (prepared.requires_base_guidance) {
      guidance::ReferenceGeometry reference;
      reference.point = prepared.geometry.r;
      reference.tangent = prepared.geometry.T;
      reference.derivative_norm = prepared.geometry.r_w.norm();
      reference.valid = prepared.geometry.valid;
      guidance::IsfGuidance base;
      const bool base_ok = guidance::IsfReferenceKernel::evaluate(
          input.position, reference, input.gains, base);
      const phase_offset_navigation::ActiveReferenceSnapshot current_authority =
          execution_authority_.snapshot();
      bool authority_step_completed = false;
      if (current_authority.valid &&
          current_authority.owner_mode ==
              phase_offset_navigation::ActiveReferenceOwnerMode::RECOVERY &&
          prepared.delta == 0.0 &&
          recovery_owner_.status().exact_terminal_predicate) {
        authority_step_completed = completeAtomicNeutralHandoff(
            input, pair, prepared, base.v_cmd, base.w_dot,
            base_ok && base.valid, runtime_output);
      } else {
        authority_step_completed = completeThroughExecutionAuthority(
            input, pair, request, pair->epoch_snapshot, prepared, base.v_cmd,
            base.w_dot, base_ok && base.valid, runtime_output);
      }
      // A production NORMAL/COORDINATION Runtime candidate is deliberately
      // rejected by ExecutionAuthority while Batch C is unauthorized.  An
      // already-authoritative nonzero pair may instead enter the real
      // Preview -> Handoff -> RecoveryOwner chain; the recovery owner owns
      // the exact selected-u and the same authority transaction below.
      if (!authority_step_completed && advertised_ && pair &&
          prepared.delta != 0.0) {
      authority_step_completed = completeThroughRecoveryOwner(
            input, pair, input.successor_path_tube_pair, prepared,
            base.v_cmd, base.w_dot,
            base_ok && base.valid, runtime_output);
      }
      completed = true;
      output.base_guidance = base;
      output.geometry = runtime_output.geometry;
      output.raw_port = runtime_output.raw_port;
      output.projection = runtime_output.projection;
      output.matched = runtime_output.matched;
      output.tube_current_bounds = runtime_output.current_bounds;
      output.tube_next_bounds = runtime_output.next_bounds;
      output.preflight = runtime_output.preflight;
      output.runtime_execution = runtime_output.execution;
      output.delta = runtime_output.delta;
      output.delta_ref = runtime_output.delta_ref;
      output.profile_active = runtime_output.profile_active;
      output.selected = runtime_output.selected;
      output.valid = runtime_output.valid;
      output.recovery_status = runtime_output.recovery_status;
      output.recovery_replan_required =
          runtime_output.recovery_replan_required;
      output.invalid_reason = runtime_output.invalid_reason;
      if (output.selected) SetGuidance(base, output.matched, output.guidance);
    } else {
      output.valid = prepared.valid;
      output.invalid_reason = prepared.invalid_reason;
    }
    const auto& execution = completed ? runtime_output.execution
                                      : prepared.execution;
    if (execution.fatal_control_failure && execution.genuine_fatal_invariant) {
      latchFailure(execution.failure_reason);
    }
    output.failure_latched = failure_latched_;
    output.control_failure_reason = control_failure_reason_;
    if (failure_latched_) { output.selected = false; output.valid = false; }
    output.tube_profile = *pair->active_profile;
    fillLegacyTubeStatus(output);
    fillManualDiagnostics(output);
    // Pair-owned control has the same command-owned publication contract as
    // the no-pair path.  This snapshots only the final facts already produced
    // above; it neither authorizes nor mutates Runtime/pair state.
    makeControlPublishSnapshot(input, request, pair->epoch_snapshot, false,
                               output);
    return output.selected;
  }

  const std::uint64_t revision = sourceRevision(input);
  if (!have_preflight_revision_ || revision != last_preflight_revision_) {
    phase_offset_navigation::RuntimePreflightInput preflight;
    preflight.position = input.position;
    preflight.path_source_revision = revision;
    preflight.path = input.sampled_path;
    if (preflight.path.empty() && input.semantic_path_owner &&
        !input.semantic_path_owner->empty()) {
      std::vector<double> sample_w;
      std::vector<ContinuousPhasePathState> sample_states;
      if (input.semantic_path_owner->sample(sampleStepW(), sample_w,
                                            sample_states) &&
          sample_w.size() == sample_states.size()) {
        for (std::size_t index = 0U; index < sample_w.size(); ++index) {
          preflight.path.push_back(ConvertContinuousPhasePathStateForActive(
              sample_states[index], sample_w[index]));
        }
      }
    }
    // An empty sample set is intentionally committed once for this revision:
    // Runtime then reports an invalid preflight rather than reusing an older
    // path's certificate or trying a callback-owned sampling function.
    runtime_->refreshPreflight(preflight);
    have_preflight_revision_ = true;
    last_preflight_revision_ = revision;
  }
  const std::shared_ptr<const TubeBuildRequest> previous_request =
      std::atomic_load(&latest_build_request_);
  if (previous_request && previous_request->active &&
      previous_request->source_revision != revision) {
    // A changed path never inherits a visual Candidate from the old path.
    // The timer will reset its manager state if an old build is in flight.
    std::atomic_store(&latest_candidate_epoch_snapshot_,
                      std::shared_ptr<const TubeEpochSnapshot>());
    std::atomic_store(&latest_epoch_snapshot_,
                      std::shared_ptr<const TubeEpochSnapshot>());
  }
  const std::shared_ptr<const TubeBuildRequest> request =
      makeBuildRequest(input, revision, runtime_->retainedDelta());
  command_active_ = true;
  // Required ordering: publish the complete current request before reading a
  // timer result.  The timer coalesces same-source control updates.
  std::atomic_store(&latest_build_request_, request);
  const std::shared_ptr<const TubeEpochSnapshot> latest_candidate =
      std::atomic_load(&latest_candidate_epoch_snapshot_);
  const bool candidate_source_matches = latest_candidate &&
      latest_candidate->active &&
      latest_candidate->task_generation == request->task_generation &&
      latest_candidate->source_revision == request->source_revision &&
      (config_.tube_source == phase_offset_navigation::TubeSource::NONE ||
       latest_candidate->epoch_status.candidate_path_source_revision ==
           request->source_revision);
  const std::shared_ptr<const TubeEpochSnapshot> runtime_epoch =
      matchingEpochForRequest(request);
  std::shared_ptr<const TubeEpochSnapshot> exposure_epoch;
  bool candidate_only = false;
  if (candidate_source_matches) {
    const bool candidate_is_current_runtime_epoch =
        epochMatchesRequest(*latest_candidate, *request) && runtime_epoch &&
        runtime_epoch->build_sequence == latest_candidate->build_sequence;
    if (candidate_is_current_runtime_epoch) {
      exposure_epoch = runtime_epoch;
    } else {
      // A Candidate whose current request fails the existing cloud contract
      // remains observable, but cannot cross the Runtime/Certified boundary.
      exposure_epoch = makeCandidateOnlyEpoch(*latest_candidate);
      candidate_only = true;
    }
  }

  if (exposure_epoch) {
    output.candidate_profile = exposure_epoch->candidate_profile;
    output.tube_epoch_status = exposure_epoch->epoch_status;
    output.tube_update_due_this_cycle =
        exposure_epoch->build_sequence != last_consumed_epoch_build_sequence_;
    if (output.tube_update_due_this_cycle) {
      last_consumed_epoch_build_sequence_ = exposure_epoch->build_sequence;
      output.raw_candidate_diagnostics_generated =
          exposure_epoch->raw_candidate_diagnostics_generated;
      output.raw_candidate_diagnostics = exposure_epoch->raw_candidate_diagnostics;
      output.cloud_snapshot_diagnostics_generated =
          exposure_epoch->cloud_snapshot_diagnostics_generated;
      output.cloud_snapshot_diagnostics =
          exposure_epoch->cloud_snapshot_diagnostics;
    }
  }

  // Runtime receives an empty installed-tube view unless the epoch matches
  // the current path source and its current request satisfies the existing
  // cloud-observation contract.  Map sequence is provenance, not a gate.
  if (runtime_epoch) {
    output.active_profile = runtime_epoch->active_profile;
    output.tube_epoch_status = runtime_epoch->epoch_status;
  }
  {
    phase_offset_navigation::RuntimePrepareInput runtime_input;
    runtime_input.current_path = input.path;
    runtime_input.position = input.position;
    if (runtime_epoch) {
      runtime_input.tube_view.active_profile = runtime_epoch->active_profile;
      runtime_input.tube_view.epoch_status = runtime_epoch->epoch_status;
      runtime_input.future_step = MakeFutureStepContract(
          request->semantic_path_owner, runtime_epoch->full_path_samples,
          runtime_epoch->active_profile, input.gains,
          runtime_->config().tube.regularity_margin,
          runtime_->config().tube.minimum_reference_speed,
          config_.tube_update_period, config_.tube.min_certified_forward_w);
    }
    runtime_input.dt = input.dt;
    // A sidecar epoch is evidence only.  Until the manager has atomically
    // installed a matching pair at the activation edge, Runtime must not
    // consume the open warm-up gate and emit the first nonzero offset port.
    // The legacy NONE source has no Tube authority contract and retains its
    // isolated A4 recurrence behavior.
    runtime_input.zero_gate_open =
        config_.tube_source == phase_offset_navigation::TubeSource::NONE &&
        zero_gate_open_ && !failure_latched_ && !config_.observe_only;
    runtime_input.fatal_adapter_failure_latched = failure_latched_;
    phase_offset_navigation::RuntimePreparedStep prepared;
    runtime_->prepare(runtime_input, prepared);
    output.geometry = prepared.geometry.valid ? prepared.geometry : output.geometry;
    output.preflight = prepared.preflight;
    output.raw_port = prepared.raw_port;
    output.delta = prepared.delta;
    output.delta_ref = prepared.delta_ref;
    output.profile_active = prepared.profile_active;
    output.tube_current_bounds = prepared.current_bounds;
    output.runtime_execution = prepared.execution;
    phase_offset_navigation::RuntimeStepOutput runtime_output;
    bool completed = false;
    if (prepared.requires_base_guidance) {
      guidance::ReferenceGeometry reference;
      reference.point = prepared.geometry.r;
      reference.tangent = prepared.geometry.T;
      reference.derivative_norm = prepared.geometry.r_w.norm();
      reference.valid = prepared.geometry.valid;
      guidance::IsfGuidance base;
      const bool base_ok = guidance::IsfReferenceKernel::evaluate(
          input.position, reference, input.gains, base);
      completeThroughExecutionAuthority(
          input, std::shared_ptr<const PathTubePair>(), request, runtime_epoch,
          prepared, base.v_cmd, base.w_dot, base_ok && base.valid,
          runtime_output);
      completed = true;
      output.base_guidance = base;
      output.geometry = runtime_output.geometry;
      output.raw_port = runtime_output.raw_port;
      output.projection = runtime_output.projection;
      output.matched = runtime_output.matched;
      output.tube_current_bounds = runtime_output.current_bounds;
      output.tube_next_bounds = runtime_output.next_bounds;
      output.preflight = runtime_output.preflight;
      output.runtime_execution = runtime_output.execution;
      output.delta = runtime_output.delta;
      output.delta_ref = runtime_output.delta_ref;
      output.profile_active = runtime_output.profile_active;
      output.selected = runtime_output.selected;
      output.valid = runtime_output.valid;
      output.invalid_reason = runtime_output.invalid_reason;
      if (output.selected) SetGuidance(base, output.matched, output.guidance);
    } else {
      output.valid = prepared.valid;
      output.invalid_reason = prepared.invalid_reason;
    }
    const auto& execution = completed ? runtime_output.execution : prepared.execution;
    if (execution.fatal_control_failure && execution.genuine_fatal_invariant) {
      latchFailure(execution.failure_reason);
    }
  }
  output.failure_latched = failure_latched_;
  output.control_failure_reason = control_failure_reason_;
  if (failure_latched_) { output.selected = false; output.valid = false; }
  output.tube_profile = output.active_profile ? *output.active_profile
      : phase_offset_navigation::TubeProfile();
  fillLegacyTubeStatus(output);
  fillManualDiagnostics(output);
  makeControlPublishSnapshot(input, request, exposure_epoch, candidate_only,
                             output);
  return output.selected;
}

bool PhaseOffsetMatchedAdapter::buildMarkers(const MatchedAdapterInput& input,
                                             const MatchedAdapterOutput& output,
                                             MatchedAdapterMarkerBundle& markers) const {
  // Compatibility/test helper.  Production publication uses the immutable
  // ControlPublishSnapshot overload below from the timer only.
  ControlPublishSnapshot control;
  control.active = true;
  control.stamp = input.stamp;
  control.current_w = input.path.w;
  control.position = input.position;
  control.output = output;
  if (!input.sampled_path.empty()) {
    control.full_path_samples =
        std::make_shared<const PathSamples>(input.sampled_path);
  }
  return buildMarkers(control, markers);
}

bool PhaseOffsetMatchedAdapter::buildMarkers(
    const ControlPublishSnapshot& control,
    MatchedAdapterMarkerBundle& markers) const {
  return buildMarkers(control, control.output.candidate_profile, markers);
}

bool PhaseOffsetMatchedAdapter::buildMarkers(
    const ControlPublishSnapshot& control,
    const std::shared_ptr<const phase_offset_navigation::TubeProfile>&
        candidate_marker_profile,
    MatchedAdapterMarkerBundle& markers) const {
  return buildMarkers(control, candidate_marker_profile, control.current_w,
                      markers);
}

bool PhaseOffsetMatchedAdapter::buildMarkers(
    const ControlPublishSnapshot& control,
    const std::shared_ptr<const phase_offset_navigation::TubeProfile>&
        candidate_marker_profile,
    const double candidate_anchor_w,
    MatchedAdapterMarkerBundle& markers) const {
  markers = MatchedAdapterMarkerBundle();
  const MatchedAdapterOutput& output = control.output;
  markers.base_path = MakeLine(control.stamp, config_.frame_id,
      "phase_offset_manual_base", 0, 0.0F, 0.3F, 1.0F);
  markers.active_path = MakeLine(control.stamp, config_.frame_id,
      "phase_offset_manual_active", 0, 1.0F, 0.0F, 1.0F);
  if (!control.full_path_samples || control.full_path_samples->empty()) {
    markers.base_path = MakeDelete(control.stamp, config_.frame_id,
                                   "phase_offset_manual_base", 0);
    markers.active_path = MakeDelete(control.stamp, config_.frame_id,
                                     "phase_offset_manual_active", 0);
  } else {
    phase_offset_core::GeometryParams geometry_params;
    geometry_params.regularity_margin = config_.tube.regularity_margin;
    geometry_params.minimum_reference_speed =
        config_.tube.cross_section.minimum_reference_speed;
    phase_offset_core::GeometryEvaluator evaluator(geometry_params);
    bool active_complete = true;
    for (const auto& path : *control.full_path_samples) {
      markers.base_path.points.push_back(ToPoint(path.p));
      phase_offset_core::PhaseOffsetGeometryState geometry;
      if (!evaluator.evaluate(path, control.position, output.delta, geometry)) active_complete = false;
      else markers.active_path.points.push_back(ToPoint(geometry.r));
    }
    if (!active_complete || markers.active_path.points.size() != markers.base_path.points.size()) {
      markers.active_path = MakeDelete(control.stamp, config_.frame_id,
                                       "phase_offset_manual_active", 0);
    }
  }
  if (output.geometry.valid) {
    visualization_msgs::Marker frame; frame.header.stamp = control.stamp; frame.header.frame_id = config_.frame_id;
    frame.ns = "phase_offset_manual_frame"; frame.id = 0; frame.type = visualization_msgs::Marker::SPHERE;
    frame.action = visualization_msgs::Marker::ADD; frame.pose.position = ToPoint(output.geometry.r); frame.pose.orientation.w = 1.0;
    frame.scale.x = frame.scale.y = frame.scale.z = 0.16; frame.color.r = 1.0F; frame.color.b = 1.0F; frame.color.a = 1.0F;
    markers.frame.markers.push_back(frame);
  } else {
    for (int id = 0; id < 4; ++id) markers.frame.markers.push_back(
        MakeDelete(control.stamp, config_.frame_id, "phase_offset_manual_frame", id));
  }
  const phase_offset_navigation::TubeProfile empty;
  const auto& candidate = candidate_marker_profile
      ? *candidate_marker_profile : empty;
  const auto& active = output.active_profile ? *output.active_profile : empty;
  markers.tube_candidate = MakeCandidateTubeMarkers(control.stamp, config_.frame_id, candidate,
      config_.mode == PhaseOffsetMatchedMode::MANUAL, candidate_anchor_w);
  markers.tube = MakeCertifiedTubeMarkers(control.stamp, config_.frame_id, active,
      tubeDisplayCertified(output));
  return output.geometry.valid;
}

bool PhaseOffsetMatchedAdapter::buildMarkers(
    const ControlPublishSnapshot& control,
    const std::shared_ptr<const TubeEpochSnapshot>& candidate_epoch,
    MatchedAdapterMarkerBundle& markers) const {
  // Production publication must keep Candidate profile and build anchor
  // under one immutable epoch.  If that provenance is unavailable or not a
  // finite build phase, fail closed to DELETE rather than guessing from the
  // later command-cycle phase.
  if (!candidate_epoch || !candidate_epoch->active ||
      !candidate_epoch->candidate_profile ||
      !std::isfinite(candidate_epoch->candidate_build_w)) {
    return buildMarkers(
        control, std::shared_ptr<const phase_offset_navigation::TubeProfile>(),
        std::numeric_limits<double>::quiet_NaN(), markers);
  }

  return buildMarkers(control, candidate_epoch->candidate_profile,
                      candidate_epoch->candidate_build_w, markers);
}

void PhaseOffsetMatchedAdapter::publishManualDelete(
    const ControlPublishSnapshot& control) {
  if (!advertised_ || config_.mode != PhaseOffsetMatchedMode::MANUAL) return;
  MatchedAdapterMarkerBundle markers;
  markers.base_path = MakeDelete(control.stamp, config_.frame_id,
                                 "phase_offset_manual_base", 0);
  markers.active_path = MakeDelete(control.stamp, config_.frame_id,
                                   "phase_offset_manual_active", 0);
  for (int id = 0; id < 4; ++id) {
    markers.frame.markers.push_back(MakeDelete(
        control.stamp, config_.frame_id, "phase_offset_manual_frame", id));
  }
  const phase_offset_navigation::TubeProfile empty;
  markers.tube = MakeCertifiedTubeMarkers(control.stamp, config_.frame_id,
                                           empty, false);
  markers.tube_candidate = MakeCandidateTubeMarkers(control.stamp,
      config_.frame_id, empty, false);
  manual_base_path_pub_.publish(markers.base_path); manual_active_path_pub_.publish(markers.active_path);
  manual_frame_pub_.publish(markers.frame); manual_tube_pub_.publish(markers.tube);
  manual_tube_candidate_pub_.publish(markers.tube_candidate);
}

bool PhaseOffsetMatchedAdapter::markEpochBuildPublished(
    const std::uint64_t build_sequence) {
  const bool due =
      timer_last_published_epoch_build_sequence_ != build_sequence;
  timer_last_published_epoch_build_sequence_ = build_sequence;
  return due;
}

void PhaseOffsetMatchedAdapter::publishBuildDiagnostics(
    const TubeEpochSnapshot& epoch) {
  // 49/18 are build provenance, not control-state facts.  Publish them from
  // the timer exactly once at completion so a rapidly advancing map cannot
  // erase a completed observation before the next 50 Hz control handshake.
  if (config_.mode != PhaseOffsetMatchedMode::MANUAL ||
      config_.tube_source != phase_offset_navigation::TubeSource::ESDF) {
    return;
  }
  const bool raw_due = epoch.raw_candidate_diagnostics_generated &&
      timer_last_raw_diagnostic_build_sequence_ != epoch.build_sequence;
  const bool cloud_due = epoch.cloud_snapshot_diagnostics_generated &&
      timer_last_cloud_diagnostic_build_sequence_ != epoch.build_sequence;
  if (raw_due) {
    timer_last_raw_diagnostic_build_sequence_ = epoch.build_sequence;
  }
  if (cloud_due) {
    timer_last_cloud_diagnostic_build_sequence_ = epoch.build_sequence;
  }
  // Production advertising precedes the timer.  Retaining the one-shot state
  // even in a non-advertised deterministic test lets the test prove that a
  // source-stale completion has exactly one raw/cloud emission decision.
  if (!advertised_) return;
  if (raw_due) {
    std_msgs::Float64MultiArray raw_candidate_diagnostics;
    raw_candidate_diagnostics.data.assign(
        epoch.raw_candidate_diagnostics.begin(),
        epoch.raw_candidate_diagnostics.end());
    manual_raw_candidate_diagnostics_pub_.publish(raw_candidate_diagnostics);
  }
  if (cloud_due) {
    std_msgs::Float64MultiArray cloud_snapshot_diagnostics;
    cloud_snapshot_diagnostics.data.assign(
        epoch.cloud_snapshot_diagnostics.begin(),
        epoch.cloud_snapshot_diagnostics.end());
    manual_cloud_snapshot_diagnostics_pub_.publish(cloud_snapshot_diagnostics);
  }
}

bool PhaseOffsetMatchedAdapter::exactLivePairPublicationControl(
    const ControlPublishSnapshot& control,
    const TubeBuildRequest& request,
    const std::shared_ptr<const PathTubePair>& live_pair,
    const std::uint64_t current_task_generation,
    const std::uint64_t current_authority_session) {
  if (!request.active || !control.active || control.candidate_only ||
      control.task_generation != request.task_generation ||
      control.control_sequence != request.control_sequence ||
      control.source_revision != request.source_revision ||
      control.map_observation_sequence != request.map_observation_sequence ||
      control.map_observation_is_snapshot !=
          request.map_observation_is_snapshot ||
      !request.base_path_tube_pair || !live_pair ||
      request.base_path_tube_pair != live_pair ||
      request.base_path_tube_pair_generation != live_pair->generation ||
      request.authority_session != live_pair->authority_session ||
      current_authority_session != live_pair->authority_session ||
      request.task_generation != current_task_generation ||
      control.task_generation != current_task_generation ||
      control.source_revision != live_pair->source_revision ||
      request.source_revision != live_pair->source_revision ||
      request.semantic_path_owner != live_pair->path_owner ||
      !live_pair->epoch_snapshot || !live_pair->active_profile ||
      control.epoch_snapshot != live_pair->epoch_snapshot ||
      !live_pair->epoch_snapshot->active ||
      live_pair->epoch_snapshot->task_generation != current_task_generation ||
      live_pair->epoch_snapshot->source_revision != live_pair->source_revision ||
      control.output.active_profile != live_pair->active_profile ||
      live_pair->epoch_snapshot->active_profile != live_pair->active_profile ||
      live_pair->epoch_snapshot->candidate_profile !=
          control.output.candidate_profile) {
    return false;
  }
  const TubeEpochSnapshot& epoch = *live_pair->epoch_snapshot;
  return control.epoch_build_sequence == epoch.build_sequence &&
      control.candidate_epoch_source_revision == epoch.source_revision &&
      control.candidate_epoch_map_observation_sequence ==
          epoch.map_observation_sequence &&
      control.candidate_epoch_map_observation_is_snapshot ==
          epoch.map_observation_is_snapshot &&
      control.output.tube_epoch_status.candidate_sequence ==
          epoch.epoch_status.candidate_sequence &&
      control.output.tube_epoch_status.active_tube_epoch ==
          epoch.epoch_status.active_tube_epoch &&
      control.output.tube_epoch_status.candidate_path_source_revision ==
          epoch.epoch_status.candidate_path_source_revision &&
      control.output.tube_epoch_status.candidate_map_observation_sequence ==
          epoch.epoch_status.candidate_map_observation_sequence &&
      live_pair->epoch_status.candidate_sequence ==
          epoch.epoch_status.candidate_sequence &&
      live_pair->epoch_status.active_tube_epoch ==
          epoch.epoch_status.active_tube_epoch &&
      live_pair->epoch_status.candidate_path_source_revision ==
          epoch.epoch_status.candidate_path_source_revision &&
      live_pair->epoch_status.candidate_map_observation_sequence ==
          epoch.epoch_status.candidate_map_observation_sequence;
}

std::shared_ptr<const TubeEpochSnapshot>
PhaseOffsetMatchedAdapter::selectCandidateEpochForPublication(
    const bool exact_live_pair_control,
    const std::shared_ptr<const TubeEpochSnapshot>& authoritative_candidate,
    const std::shared_ptr<const TubeEpochSnapshot>& control_epoch) {
  // A newer authoritative Candidate wins when exact-live-pair publication has
  // one.  Otherwise the exact live pair epoch remains the Candidate owner;
  // never substitute the later command-cycle phase for either epoch.
  return exact_live_pair_control && authoritative_candidate
      ? authoritative_candidate
      : control_epoch;
}

void PhaseOffsetMatchedAdapter::publishManual(
    const ControlPublishSnapshot& control) {
  if (!advertised_ || config_.mode != PhaseOffsetMatchedMode::MANUAL) return;
  const std::shared_ptr<const TubeBuildRequest> request =
      std::atomic_load(&latest_build_request_);
  // Marker ownership follows the complete command identity.  A timer
  // completion from an older task/source/control/map must not clear or
  // overwrite a newer namespace, even when its source revision happens to
  // compare equal.
  const bool current_control_identity = request && request->active &&
      control.active &&
      control.task_generation == request->task_generation &&
      control.control_sequence == request->control_sequence &&
      control.source_revision == request->source_revision &&
      control.map_observation_sequence == request->map_observation_sequence &&
      control.map_observation_is_snapshot ==
          request->map_observation_is_snapshot &&
      task_generation_.load(std::memory_order_acquire) ==
          request->task_generation;
  if (!current_control_identity) return;

  const std::shared_ptr<const TubeEpochSnapshot> latest_candidate =
      std::atomic_load(&latest_candidate_epoch_snapshot_);
  const std::shared_ptr<const TubeEpochSnapshot> authoritative_candidate =
      latest_candidate && latest_candidate->active &&
      latest_candidate->task_generation == request->task_generation &&
      latest_candidate->source_revision == request->source_revision &&
      (config_.tube_source == phase_offset_navigation::TubeSource::NONE ||
       latest_candidate->epoch_status.candidate_path_source_revision ==
           request->source_revision)
      ? latest_candidate : std::shared_ptr<const TubeEpochSnapshot>();
  const bool candidate_epoch_keys_match = control.epoch_snapshot &&
      control.task_generation == control.epoch_snapshot->task_generation &&
      control.epoch_build_sequence == control.epoch_snapshot->build_sequence &&
      control.candidate_epoch_source_revision ==
          control.epoch_snapshot->source_revision &&
      control.candidate_epoch_map_observation_sequence ==
          control.epoch_snapshot->map_observation_sequence &&
      control.candidate_epoch_map_observation_is_snapshot ==
          control.epoch_snapshot->map_observation_is_snapshot;
  const bool regular_epoch_is_current =
      candidate_epoch_keys_match && !control.candidate_only &&
      epochMatchesRequest(*control.epoch_snapshot, *request);
  // A current request that fails the existing cloud contract may still display
  // a same-source historical Candidate because it has no Active/Certified
  // ownership whatsoever.
  const bool historical_candidate_is_publishable =
      control.candidate_only && candidate_epoch_keys_match &&
      control.candidate_epoch_source_revision == control.source_revision &&
      !control.epoch_snapshot->active_profile &&
      !control.output.active_profile &&
      !control.output.tube_epoch_status.active_available;
  const bool control_keys_match = regular_epoch_is_current ||
      historical_candidate_is_publishable;
  const bool output_matches_epoch = control.epoch_snapshot &&
      control.output.candidate_profile == control.epoch_snapshot->candidate_profile &&
      control.output.active_profile == control.epoch_snapshot->active_profile &&
      control.output.tube_epoch_status.candidate_sequence ==
          control.epoch_snapshot->epoch_status.candidate_sequence &&
      control.output.tube_epoch_status.active_tube_epoch ==
          control.epoch_snapshot->epoch_status.active_tube_epoch &&
      control.output.tube_epoch_status.candidate_path_source_revision ==
          control.epoch_snapshot->epoch_status.candidate_path_source_revision &&
      control.output.tube_epoch_status.candidate_map_observation_sequence ==
          control.epoch_snapshot->epoch_status.candidate_map_observation_sequence;
  const std::shared_ptr<const PathTubePair> live_pair =
      std::atomic_load(&authoritative_path_tube_pair_);
  const bool exact_live_pair_control = current_control_identity &&
      output_matches_epoch &&
      exactLivePairPublicationControl(
          control, *request, live_pair,
          task_generation_.load(std::memory_order_acquire),
          authority_session_.load(std::memory_order_acquire));

  // A current lifecycle with no authoritative epoch owns a one-shot DELETE
  // for both topics.  This is distinct from a stale callback above.
  if (!control.epoch_snapshot && !authoritative_candidate) {
    if (timer_last_publish_delete_source_revision_ != request->source_revision ||
        timer_last_publish_delete_map_observation_sequence_ !=
            request->map_observation_sequence) {
      publishManualDelete(control);
      timer_last_publish_delete_source_revision_ = request->source_revision;
      timer_last_publish_delete_map_observation_sequence_ =
          request->map_observation_sequence;
    }
    return;
  }

  // An epoch/profile/status mismatch is a current-identity integrity failure,
  // not ordinary staleness.  Certified visibility fails closed; candidate
  // geometry may come only from the current authoritative epoch, never from
  // the mismatched callback payload.
  const bool authoritative_epoch_matches_control =
      authoritative_candidate && control.epoch_snapshot == authoritative_candidate;
  const bool regular_control_owner = control_keys_match &&
      output_matches_epoch && authoritative_epoch_matches_control;
  if (!regular_control_owner && !exact_live_pair_control) {
    MatchedAdapterMarkerBundle markers;
    ControlPublishSnapshot authoritative_control = control;
    if (authoritative_candidate) {
      authoritative_control.epoch_snapshot = authoritative_candidate;
      authoritative_control.epoch_build_sequence =
          authoritative_candidate->build_sequence;
      authoritative_control.candidate_epoch_source_revision =
          authoritative_candidate->source_revision;
      authoritative_control.candidate_epoch_map_observation_sequence =
          authoritative_candidate->map_observation_sequence;
      authoritative_control.candidate_epoch_map_observation_is_snapshot =
          authoritative_candidate->map_observation_is_snapshot;
      authoritative_control.output.candidate_profile =
          authoritative_candidate->candidate_profile;
      authoritative_control.output.tube_epoch_status =
          authoritative_candidate->epoch_status;
    } else {
      authoritative_control.output.candidate_profile.reset();
      authoritative_control.output.active_profile.reset();
    }
    buildMarkers(authoritative_control, authoritative_candidate, markers);
    markers.tube = MakeCertifiedTubeMarkers(
        control.stamp, config_.frame_id,
        phase_offset_navigation::TubeProfile(), false);
    manual_base_path_pub_.publish(markers.base_path);
    manual_active_path_pub_.publish(markers.active_path);
    manual_frame_pub_.publish(markers.frame);
    manual_tube_pub_.publish(markers.tube);
    manual_tube_candidate_pub_.publish(markers.tube_candidate);
    return;
  }

  MatchedAdapterMarkerBundle markers;
  const std::shared_ptr<const TubeEpochSnapshot> candidate_epoch =
      selectCandidateEpochForPublication(
          exact_live_pair_control, authoritative_candidate,
          control.epoch_snapshot);
  if (candidate_epoch) {
    buildMarkers(control, candidate_epoch, markers);
  } else {
    // No exact immutable Candidate epoch is available; retain the existing
    // fail-closed DELETE behavior without using ControlPublishSnapshot::current_w.
    buildMarkers(
        control,
        std::shared_ptr<const phase_offset_navigation::TubeProfile>(),
        std::numeric_limits<double>::quiet_NaN(), markers);
  }
  // Publication owns the one-shot 50-field "tube due" fact.  A later 50 Hz
  // command may overwrite its control snapshot before this tick, so command
  // output cannot be the authoritative pending bit.
  MatchedAdapterOutput output = control.output;
  output.tube_update_due_this_cycle =
      markEpochBuildPublished(control.epoch_build_sequence);
  manual_base_path_pub_.publish(markers.base_path); manual_active_path_pub_.publish(markers.active_path);
  manual_frame_pub_.publish(markers.frame); manual_tube_pub_.publish(markers.tube);
  manual_tube_candidate_pub_.publish(markers.tube_candidate);
  std_msgs::Float64MultiArray diagnostics;
  diagnostics.data.assign(output.diagnostics.begin(), output.diagnostics.end());
  manual_diagnostics_pub_.publish(diagnostics);
  TubeEpochDiagnosticsInput epoch_input;
  epoch_input.source = config_.tube_source; epoch_input.epoch = output.tube_epoch_status;
  epoch_input.runtime = output.runtime_execution; epoch_input.candidate_profile = output.candidate_profile.get();
  epoch_input.active_profile = output.active_profile.get(); epoch_input.retained_delta = output.delta;
  epoch_input.active_display_certified = tubeDisplayCertified(output);
  epoch_input.tube_update_due_this_cycle = output.tube_update_due_this_cycle;
  epoch_input.control_selected = output.selected;
  epoch_input.failure_latched = output.failure_latched;
  epoch_input.control_failure_reason = output.control_failure_reason;
  const auto epoch_values = makeTubeEpochDiagnostics(epoch_input);
  std_msgs::Float64MultiArray epoch_diagnostics;
  epoch_diagnostics.data.assign(epoch_values.begin(), epoch_values.end());
  manual_tube_epoch_diagnostics_pub_.publish(epoch_diagnostics);
}

}  // namespace FLAG_Race
