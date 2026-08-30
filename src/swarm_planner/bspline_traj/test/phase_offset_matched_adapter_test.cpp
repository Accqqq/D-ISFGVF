#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#define private public
#include <plan_env/sdf_map.h>
#undef private

#define private public
#include "bspline_race/integration/phase_offset_matched_adapter.h"
#undef private
#include "bspline_race/integration/phase_offset_executed_reference_query.h"
#include "bspline_race/integration/phase_offset_tube_epoch_diagnostics.h"
#include "phase_offset_navigation/certified_tube_builder.h"

namespace FLAG_Race {
namespace {

constexpr double kDt = 0.02;
using phase_offset_navigation::DistanceStatus;
using phase_offset_navigation::TubeEpochState;
using phase_offset_navigation::TubeEpochReason;
using phase_offset_navigation::TubeInstallDisposition;
using phase_offset_navigation::TubeEpochStatus;
using phase_offset_navigation::TubeProfile;
using phase_offset_navigation::TubeRawSample;
using phase_offset_navigation::TubeSource;

guidance::IsfGains MakeGains() {
  guidance::IsfGains gains;
  gains.k1 = 2.0; gains.k2 = -2.2; gains.convergence_bandwidth = 0.1;
  gains.progress_rho0 = 0.5; gains.progress_delta = 0.3; gains.alpha_min = 0.05;
  return gains;
}

struct SyntheticPath {
  phase_offset_navigation::RuntimePathSamples samples;
  phase_offset_core::PathDifferentialState current;
  double start_w = 0.0;
  double end_w = 3.0;
};

phase_offset_core::PathDifferentialState MakeState(double w) {
  phase_offset_core::PathDifferentialState state;
  state.p = Eigen::Vector3d(w, 0.4 * std::sin(w), 1.0 + 0.2 * w);
  state.p_w = Eigen::Vector3d(1.0, 0.4 * std::cos(w), 0.2);
  state.p_ww = Eigen::Vector3d(0.0, -0.4 * std::sin(w), 0.0);
  state.w = w; state.valid = true;
  return state;
}

phase_offset_core::PathDifferentialState MakeStraightState(double w) {
  phase_offset_core::PathDifferentialState state;
  state.p = Eigen::Vector3d(w, 0.0, 1.0);
  state.p_w = Eigen::Vector3d::UnitX();
  state.p_ww.setZero();
  state.w = w;
  state.valid = true;
  return state;
}

SyntheticPath MakePath() {
  SyntheticPath path;
  path.current = MakeState(0.4);
  for (int index = 0; index <= 30; ++index) path.samples.push_back(MakeState(0.1 * index));
  return path;
}

SyntheticPath MakeStraightSyntheticPath() {
  SyntheticPath path;
  path.current = MakeStraightState(0.4);
  for (int index = 0; index <= 30; ++index) {
    path.samples.push_back(MakeStraightState(0.1 * index));
  }
  return path;
}

std::shared_ptr<const ContinuousPhasePath> MakeSyntheticOwner() {
  auto owner = std::make_shared<ContinuousPhasePath>();
  EXPECT_TRUE(owner->appendSegment(
      0.0, 3.0, "prepared-synthetic",
      [](const double w, ContinuousPhasePathState& state) {
        const phase_offset_core::PathDifferentialState source = MakeState(w);
        state.p = source.p;
        state.dp_dw = source.p_w;
        state.d2p_dw2 = source.p_ww;
        state.valid = source.valid;
        return true;
      }));
  return std::shared_ptr<const ContinuousPhasePath>(owner);
}

std::shared_ptr<const ContinuousPhasePath> MakeStraightSyntheticOwner() {
  auto owner = std::make_shared<ContinuousPhasePath>();
  EXPECT_TRUE(owner->appendSegment(
      0.0, 3.0, "prepared-straight-synthetic",
      [](const double w, ContinuousPhasePathState& state) {
        state.p = Eigen::Vector3d(w, 0.0, 1.0);
        state.dp_dw = Eigen::Vector3d::UnitX();
        state.d2p_dw2.setZero();
        state.valid = true;
        return true;
      }));
  return std::shared_ptr<const ContinuousPhasePath>(owner);
}

struct BlockingOwnerEvaluation {
  std::atomic<std::size_t> call_count {0U};
  std::atomic<bool> armed {false};
  std::size_t block_on_call = 0U;
  std::mutex mutex;
  std::condition_variable condition;
  bool paused = false;
  bool release = false;
};

std::shared_ptr<const ContinuousPhasePath> MakeBlockingStraightSyntheticOwner(
    const std::shared_ptr<BlockingOwnerEvaluation>& control) {
  auto owner = std::make_shared<ContinuousPhasePath>();
  EXPECT_TRUE(owner->appendSegment(
      0.0, 3.0, "prepared-blocking-straight-synthetic",
      [control](const double w, ContinuousPhasePathState& state) {
        const std::size_t call =
            control->call_count.fetch_add(1U, std::memory_order_acq_rel) + 1U;
        if (control->armed.load(std::memory_order_acquire) &&
            call == control->block_on_call) {
          std::unique_lock<std::mutex> lock(control->mutex);
          control->paused = true;
          control->condition.notify_all();
          control->condition.wait(lock, [&control]() {
            return control->release;
          });
        }
        state.p = Eigen::Vector3d(w, 0.0, 1.0);
        state.dp_dw = Eigen::Vector3d::UnitX();
        state.d2p_dw2.setZero();
        state.valid = true;
        return true;
      }));
  return std::shared_ptr<const ContinuousPhasePath>(owner);
}

ContinuousPhasePath::Evaluator MakeCertifiedLinePiece(
    const double segment_w0, const double segment_w1) {
  const auto point_evaluator = [=](const double w,
                                   ContinuousPhasePathState& state) {
    state.p = Eigen::Vector3d(w, 0.0, 1.0);
    state.dp_dw = Eigen::Vector3d::UnitX();
    state.d2p_dw2.setZero();
    state.vel = state.dp_dw;
    state.valid = std::isfinite(w) && w >= segment_w0 - 1e-12 &&
        w <= segment_w1 + 1e-12;
    return state.valid;
  };
  const auto cell_bound_evaluator = [=](
      const double w0, const double w1,
      phase_offset_core::PathCellGeometryCertificate& certificate) {
    certificate = phase_offset_core::PathCellGeometryCertificate();
    certificate.w0 = w0;
    certificate.w1 = w1;
    certificate.segment_w0 = segment_w0;
    certificate.segment_w1 = segment_w1;
    certificate.segment_identity = 1U;
    certificate.inf_p_w_norm = 1.0;
    certificate.inf_horizontal_p_w_norm = 1.0;
    certificate.sup_p_w_norm = 1.0;
    certificate.sup_p_ww_norm = 0.0;
    certificate.sup_p_www_norm = 0.0;
    certificate.sup_horizontal_p_ww_norm = 0.0;
    certificate.horizontal_acceleration_bound_complete = true;
    certificate.sup_N_w_norm = 0.0;
    certificate.sup_abs_curvature = 0.0;
    certificate.normal_variation_bound = 0.0;
    certificate.tangent_variation_bound = 0.0;
    certificate.curvature_variation_bound = 0.0;
    certificate.midpoint_position_variation_bound = 0.0;
    certificate.chord_deviation_bound = 0.0;
    certificate.valid = std::isfinite(w0) && std::isfinite(w1) &&
        w1 > w0 && w0 >= segment_w0 - 1e-12 &&
        w1 <= segment_w1 + 1e-12;
    certificate.complete = certificate.valid;
    return certificate.valid;
  };
  return ContinuousPhasePath::Evaluator(point_evaluator,
                                        cell_bound_evaluator);
}

std::shared_ptr<const ContinuousPhasePath> MakeCertifiedThreePieceOwner() {
  auto owner = std::make_shared<ContinuousPhasePath>();
  EXPECT_TRUE(owner->appendSegment(
      0.0, 1.0, "certified_prefix", MakeCertifiedLinePiece(0.0, 1.0)));
  EXPECT_TRUE(owner->appendSegment(
      1.0, 2.0, "certified_connector", MakeCertifiedLinePiece(1.0, 2.0)));
  EXPECT_TRUE(owner->appendSegment(
      2.0, 3.0, "certified_tail", MakeCertifiedLinePiece(2.0, 3.0)));
  return std::shared_ptr<const ContinuousPhasePath>(owner);
}

std::shared_ptr<const ContinuousPhasePath> MakeH2ProductionOldOwner() {
  auto owner = std::make_shared<ContinuousPhasePath>();
  EXPECT_TRUE(owner->appendSegment(
      0.0, 3.0, "h2-production-old",
      [](const double w, ContinuousPhasePathState& state) {
        state.p = Eigen::Vector3d(w, 0.10 * w * w, 1.0);
        state.dp_dw = Eigen::Vector3d(1.0, 0.20 * w, 0.0);
        state.d2p_dw2 = Eigen::Vector3d(0.0, 0.20, 0.0);
        state.vel = state.dp_dw;
        state.valid = true;
        return true;
      }));
  return std::shared_ptr<const ContinuousPhasePath>(owner);
}

std::shared_ptr<const ContinuousPhasePath> MakeH2ProductionNewOwner(
    const std::shared_ptr<const ContinuousPhasePath>& old_owner,
    const double captured_w0, const double future_seam_w,
    const double join_w, const double end_w) {
  if (!old_owner) return std::shared_ptr<const ContinuousPhasePath>();
  ContinuousPhasePathState old_seam;
  if (!old_owner->evaluate(future_seam_w, old_seam, false)) {
    return std::shared_ptr<const ContinuousPhasePath>();
  }

  // This is the production C2 connector implementation used by the frontend.
  // The deliberately different terminal state makes its strict interior
  // distinguishable from the old owner.
  ContinuousPhasePathState mapped_tail_join;
  mapped_tail_join.p = Eigen::Vector3d(join_w, 1.60, 1.0);
  mapped_tail_join.dp_dw = Eigen::Vector3d(1.0, -0.40, 0.0);
  mapped_tail_join.d2p_dw2 = Eigen::Vector3d(0.0, -0.20, 0.0);
  mapped_tail_join.vel = mapped_tail_join.dp_dw;
  mapped_tail_join.valid = true;
  const ContinuousPhasePath::Evaluator connector =
      ContinuousPhasePath::makeQuinticHermite(
          future_seam_w, join_w, old_seam, mapped_tail_join);
  if (!connector) return std::shared_ptr<const ContinuousPhasePath>();

  auto owner = std::make_shared<ContinuousPhasePath>();
  if (!owner->appendSlice(*old_owner, captured_w0, future_seam_w) ||
      !owner->appendSegment(future_seam_w, join_w, "c2_quintic", connector) ||
      !owner->appendSegment(
          join_w, end_w, "mapped_tail",
          [mapped_tail_join, join_w](const double w,
                                     ContinuousPhasePathState& state) {
            const double dw = w - join_w;
            state = mapped_tail_join;
            state.p = mapped_tail_join.p + dw * mapped_tail_join.dp_dw +
                0.5 * dw * dw * mapped_tail_join.d2p_dw2;
            state.dp_dw = mapped_tail_join.dp_dw +
                dw * mapped_tail_join.d2p_dw2;
            state.vel = state.dp_dw;
            state.valid = state.p.allFinite() && state.dp_dw.allFinite() &&
                state.d2p_dw2.allFinite() && state.vel.allFinite();
            return state.valid;
          })) {
    return std::shared_ptr<const ContinuousPhasePath>();
  }
  return std::shared_ptr<const ContinuousPhasePath>(owner);
}

std::shared_ptr<const ContinuousPhasePath> MakeHorizontalDegenerateOwner(
    const double start_w, const double end_w, const double horizontal_speed) {
  if (!std::isfinite(start_w) || !std::isfinite(end_w) ||
      end_w <= start_w || !std::isfinite(horizontal_speed) ||
      horizontal_speed < 0.0) {
    return std::shared_ptr<const ContinuousPhasePath>();
  }
  FLAG_Race::ContinuousPhasePathState start;
  start.p = Eigen::Vector3d(0.0, 0.0, 1.0);
  start.dp_dw = Eigen::Vector3d(horizontal_speed, 0.0, 1.0);
  start.d2p_dw2 = Eigen::Vector3d::Zero();
  start.vel = start.dp_dw;
  start.valid = true;
  FLAG_Race::ContinuousPhasePathState end = start;
  const double span = end_w - start_w;
  end.p = start.p + span * start.dp_dw;
  const auto evaluator = FLAG_Race::ContinuousPhasePath::makeQuinticHermite(
      start_w, end_w, start, end);
  if (!evaluator) return std::shared_ptr<const ContinuousPhasePath>();
  auto owner = std::make_shared<ContinuousPhasePath>();
  if (!owner->appendSegment(start_w, end_w, "horizontal-degenerate",
                            evaluator)) {
    return std::shared_ptr<const ContinuousPhasePath>();
  }
  return std::shared_ptr<const ContinuousPhasePath>(owner);
}

MatchedAdapterPathSamples SampleOwner(
    const std::shared_ptr<const ContinuousPhasePath>& owner,
    const std::vector<double>& sample_w) {
  MatchedAdapterPathSamples samples;
  if (!owner) return samples;
  for (const double w : sample_w) {
    ContinuousPhasePathState state;
    if (!owner->evaluate(w, state, false)) {
      samples.clear();
      return samples;
    }
    samples.push_back(ConvertContinuousPhasePathStateForActive(state, w));
  }
  return samples;
}

MatchedAdapterInput MakeInput(const SyntheticPath& path,
                              const void* identity = nullptr,
                              double stamp = 0.0) {
  MatchedAdapterInput input;
  input.path = path.current; input.sampled_path = path.samples;
  input.path_state_query = [](const double w,
                              phase_offset_core::PathDifferentialState& state) {
    state = MakeState(w);
    return true;
  };
  input.semantic_path_identity = identity == nullptr ? &path : identity;
  input.semantic_path_start_w = path.start_w; input.semantic_path_end_w = path.end_w;
  input.position = Eigen::Vector3d(0.4, 0.1, 1.1); input.gains = MakeGains();
  input.dt = kDt; input.stamp = ros::Time(stamp);
  PhaseOffsetActiveAdapter zero;
  ActiveAdapterInput zero_input;
  zero_input.path = input.path; zero_input.position = input.position; zero_input.gains = input.gains;
  ActiveAdapterOutput zero_output;
  EXPECT_TRUE(zero.evaluate(zero_input, zero_output));
  input.legacy = LegacyGuidanceSnapshot(zero_output.guidance.v_cmd, zero_output.guidance.w_dot,
      zero_output.guidance.e_parallel, zero_output.guidance.e_perp,
      zero_output.guidance.ref_pt, zero_output.guidance.tangent, zero_output.guidance.valid);
  return input;
}

// The authority transaction is a sequential state machine: once a tick has
// committed, the next command's phase must begin at the predecessor's exact
// proposed_next_w.  Rebuild the synthetic legacy guidance alongside the
// rebased path so the gate remains a valid fixture rather than an authority
// bypass.
void RebaseInputToAuthority(PhaseOffsetMatchedAdapter& adapter,
                            MatchedAdapterInput& input) {
  const phase_offset_navigation::ActiveReferenceSnapshot authority =
      adapter.execution_authority_.snapshot();
  if (!authority.valid || !std::isfinite(authority.proposed_next_w)) return;
  input.path = MakeState(authority.proposed_next_w);
  ActiveAdapterInput zero_input;
  zero_input.path = input.path;
  zero_input.position = input.position;
  zero_input.gains = input.gains;
  PhaseOffsetActiveAdapter zero;
  ActiveAdapterOutput zero_output;
  EXPECT_TRUE(zero.evaluate(zero_input, zero_output));
  input.legacy = LegacyGuidanceSnapshot(
      zero_output.guidance.v_cmd, zero_output.guidance.w_dot,
      zero_output.guidance.e_parallel, zero_output.guidance.e_perp,
      zero_output.guidance.ref_pt, zero_output.guidance.tangent,
      zero_output.guidance.valid);
}

PhaseOffsetMatchedAdapterConfig MakeManualConfig(TubeSource source = TubeSource::NONE) {
  PhaseOffsetMatchedAdapterConfig config;
  config.mode = PhaseOffsetMatchedMode::MANUAL; config.tube_source = source;
  config.observe_only = false;
  config.profile_period = 2.0; config.warmup_cycles = 100;
  config.u_w_rate_max = 100.0; config.u_delta_rate_max = 100.0;
  config.u_delta_abs_max = 0.40; config.tube_update_period = 0.10;
  config.normal_preview_policy.preview_horizon_w = 2.0;
  config.normal_preview_policy.sample_spacing_w = 0.10;
  config.normal_preview_policy.lower_nu = 0.02;
  config.normal_preview_policy.upper_nu = 2.0;
  config.normal_preview_policy.b_tight = 0.10;
  config.normal_preview_policy.b_open = 0.90;
  config.normal_preview_policy.policy_revision = 1U;
  config.normal_preview_policy.configuration_identity = 1U;
  config.normal_preview_policy.configuration_id = "test-normal-preview-w";
  config.normal_preview_policy_explicit = true;
  config.tube.fixed_delta_max = 0.04; config.tube.back_w = 0.0;
  config.tube.lookahead_w = 2.0; config.tube.min_certified_forward_w = 0.40;
  config.tube.cross_section.search_extent = 3.0;
  config.tube.cross_section.ray_step = 0.05;
  config.tube.cross_section.boundary_tolerance = 0.01;
  config.tube.cross_section.regularity_margin = 0.10;
  config.tube.cross_section.curvature_epsilon = 1e-9;
  config.tube.cross_section.margins.uav_radius = 0.25;
  config.tube.cross_section.margins.map_uncertainty = 0.10;
  config.tube.cross_section.margins.localization_uncertainty = 0.05;
  config.tube.cross_section.margins.tracking_error_bound = 0.15;
  config.tube.cross_section.margins.preincluded_map_uncertainty = 0.10;
  config.cloud_obstacle_set_complete = true;
  return config;
}

std::shared_ptr<const plan_env::CloudOccupancySnapshot>
MakeAllFreeCloudSnapshot() {
  std::shared_ptr<plan_env::CloudOccupancySnapshot> snapshot(
      new plan_env::CloudOccupancySnapshot());
  snapshot->valid = true;
  snapshot->observation_sequence = 1U;
  snapshot->observation_stamp = ros::Time(1.0);
  snapshot->map_min = Eigen::Vector3d(-5.0, -5.0, -5.0);
  snapshot->map_max = Eigen::Vector3d(5.0, 5.0, 5.0);
  snapshot->observed_min = snapshot->map_min;
  snapshot->observed_max = snapshot->map_max;
  snapshot->grid_origin = snapshot->map_min;
  snapshot->voxel_count = Eigen::Vector3i(10, 10, 10);
  snapshot->resolution = 1.0;
  snapshot->included_map_inflation = 0.10;
  snapshot->occupied.assign(1000U, 0U);
  if (!plan_env::cloudOccupancySnapshotConsistent(*snapshot)) {
    return std::shared_ptr<const plan_env::CloudOccupancySnapshot>();
  }
  return std::shared_ptr<const plan_env::CloudOccupancySnapshot>(snapshot);
}

struct NormalProductionFixture {
  std::shared_ptr<const ContinuousPhasePath> owner;
  std::shared_ptr<const ContinuousPhaseNormalFrame> frame;
  std::shared_ptr<const PathTubePair> pair;
  MatchedAdapterInput input;
};

NormalProductionFixture MakeNormalProductionFixture(
    PhaseOffsetMatchedAdapter& adapter) {
  NormalProductionFixture fixture;
  fixture.owner = MakeStraightSyntheticOwner();
  if (!fixture.owner) return fixture;
  fixture.frame = std::make_shared<const ContinuousPhaseNormalFrame>(
      fixture.owner, 7U, 8U);
  std::shared_ptr<TubeProfile> profile(new TubeProfile());
  profile->source = TubeSource::ESDF;
  profile->source_revision = 7U;
  profile->path_revision = 7U;
  profile->frame_revision = 8U;
  profile->tube_revision = 11U;
  profile->profile_revision = 11U;
  profile->map_revision = 41U;
  profile->obstacle_contract_id =
      "direct-clearance/planner-safe-distance";
  profile->preview_start_w = 0.0;
  profile->preview_end_w = 3.0;
  profile->requested_preview_start_w = 0.0;
  profile->requested_preview_end_w = 3.0;
  profile->certified_segment_start_w = 0.0;
  profile->certified_segment_end_w = 3.0;
  profile->raw_complete = true;
  profile->filtered_complete = true;
  profile->complete = true;
  profile->obstacle_certified = true;
  profile->zero_only = false;
  profile->current_delta = 0.0;
  profile->current_delta_valid = true;
  profile->classification =
      phase_offset_navigation::TubeProfileClassification::OFFSET_CERTIFIED;
  for (const double w : {0.0, 0.5, 1.0, 2.0, 3.0}) {
    TubeRawSample sample;
    sample.w = w;
    sample.path_revision = 7U;
    sample.frame_revision = 8U;
    sample.p = Eigen::Vector3d(w, 0.0, 1.0);
    sample.N = Eigen::Vector3d(0.0, 1.0, 0.0);
    sample.raw_lower = sample.filtered_lower = -0.35;
    sample.raw_upper = sample.filtered_upper = 0.35;
    sample.complete = true;
    profile->samples.push_back(sample);
  }
  profile->raw_build_samples = profile->samples;
  profile->zero_component_contains_zero = true;
  profile->current_component_contains_delta = true;
  std::shared_ptr<TubeEpochSnapshot> epoch(new TubeEpochSnapshot());
  epoch->active = true;
  epoch->task_generation = 1U;
  epoch->build_sequence = 1U;
  epoch->source_revision = 7U;
  epoch->path_revision = 7U;
  epoch->frame_revision = 8U;
  epoch->candidate_profile = std::shared_ptr<const TubeProfile>(profile);
  epoch->active_profile = std::shared_ptr<const TubeProfile>(profile);
  epoch->epoch_status.candidate_path_source_revision = 7U;
  epoch->epoch_status.active_path_source_revision = 7U;
  epoch->epoch_status.active_available = true;
  epoch->epoch_status.active_current_validation_valid = true;
  epoch->epoch_status.candidate_complete = true;
  epoch->epoch_status.candidate_classification =
      phase_offset_navigation::TubeProfileClassification::OFFSET_CERTIFIED;
  epoch->epoch_status.map_observation_is_snapshot = true;
  std::shared_ptr<PathTubePair> pair(new PathTubePair());
  pair->source_revision = 7U;
  pair->path_revision = 7U;
  pair->frame_revision = 8U;
  pair->generation = 1U;
  pair->authority_session = 3U;
  pair->map_observation_sequence = 41U;
  pair->map_observation_is_snapshot = true;
  pair->path_owner = fixture.owner;
  pair->frame_owner = fixture.frame;
  pair->full_path_samples = std::make_shared<const MatchedAdapterPathSamples>(
      SampleOwner(fixture.owner, {0.0, 0.5, 1.0, 2.0, 3.0}));
  pair->active_profile = std::shared_ptr<const TubeProfile>(profile);
  pair->epoch_status = epoch->epoch_status;
  pair->epoch_snapshot = std::shared_ptr<const TubeEpochSnapshot>(epoch);
  fixture.pair = std::shared_ptr<const PathTubePair>(pair);
  adapter.authority_session_.store(3U, std::memory_order_release);
  adapter.advertised_ = true;
  adapter.execution_authority_.setTestOnlyRuntimeOwnerAllowed(false);
  std::atomic_store(&adapter.authoritative_path_tube_pair_, fixture.pair);
  fixture.input = MakeInput(MakeStraightSyntheticPath(), fixture.owner.get());
  fixture.input.path = MakeStraightState(0.5);
  fixture.input.semantic_path_owner = fixture.owner;
  fixture.input.frame_owner = fixture.frame;
  fixture.input.semantic_path_start_w = fixture.owner->startW();
  fixture.input.semantic_path_end_w = fixture.owner->endW();
  fixture.input.path_tube_pair = fixture.pair;
  fixture.input.path_state_query = [](const double w,
      phase_offset_core::PathDifferentialState& state) {
    state = MakeStraightState(w);
    return true;
  };
  ActiveAdapterInput zero_input;
  zero_input.path = fixture.input.path;
  zero_input.position = fixture.input.position;
  zero_input.gains = fixture.input.gains;
  PhaseOffsetActiveAdapter zero;
  ActiveAdapterOutput zero_output;
  if (!zero.evaluate(zero_input, zero_output)) return NormalProductionFixture();
  fixture.input.legacy = LegacyGuidanceSnapshot(
      zero_output.guidance.v_cmd, zero_output.guidance.w_dot,
      zero_output.guidance.e_parallel, zero_output.guidance.e_perp,
      zero_output.guidance.ref_pt, zero_output.guidance.tangent,
      zero_output.guidance.valid);
  fixture.input.dt = 0.02;
  fixture.input.stamp = ros::Time(1.0);
  return fixture;
}

TEST(PhaseOffsetMatchedAdapterC3,
     NormalAllocatorZeroPortPreservesNominalMatchedReference) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
  config.u_w_abs_max = 0.20;
  config.u_delta_abs_max = 0.40;
  PhaseOffsetMatchedAdapter adapter(config);
  NormalProductionFixture fixture = MakeNormalProductionFixture(adapter);
  ASSERT_TRUE(fixture.pair);
  fixture.input.g_des_valid = false;
  MatchedAdapterOutput output;
  ASSERT_TRUE(adapter.update(fixture.input, output)) << output.invalid_reason;
  ASSERT_TRUE(output.selected);
  ASSERT_TRUE(output.allocator_evaluated);
  EXPECT_TRUE(output.g_des_valid);
  EXPECT_DOUBLE_EQ(0.0, output.g_des.x());
  EXPECT_DOUBLE_EQ(0.0, output.g_des.y());
  EXPECT_DOUBLE_EQ(0.0, output.g_des.z());
  EXPECT_EQ(output.allocator.selected_u_owner,
            phase_offset_navigation::PhaseOffsetAllocator::ownerName());
  EXPECT_DOUBLE_EQ(0.0, output.allocator.selected_u.u_w);
  EXPECT_DOUBLE_EQ(0.0, output.allocator.selected_u.u_delta);
  EXPECT_DOUBLE_EQ(0.0, output.matched.physical_port.x());
  EXPECT_DOUBLE_EQ(0.0, output.matched.physical_port.y());
  EXPECT_DOUBLE_EQ(0.0, output.matched.physical_port.z());
  EXPECT_DOUBLE_EQ(output.matched.v_cmd.x(), output.base_guidance.v_cmd.x());
  EXPECT_DOUBLE_EQ(output.matched.v_cmd.y(), output.base_guidance.v_cmd.y());
  EXPECT_DOUBLE_EQ(output.matched.v_cmd.z(), output.base_guidance.v_cmd.z());
  const PendingPositionCommandCapture capture =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(capture.pending);
  ASSERT_TRUE(capture.valid);
  const phase_offset_navigation::ActiveReferenceSnapshot pending =
      *adapter.pending_authority_prepared_.committed_snapshot;
  EXPECT_EQ(pending.selected_u_owner,
            phase_offset_navigation::PhaseOffsetAllocator::ownerName());
  EXPECT_TRUE(pending.selectedUConsistent(0.0));
  EXPECT_DOUBLE_EQ(output.allocator.selected_u.u_w, pending.selected_u.u_w);
  EXPECT_DOUBLE_EQ(output.allocator.selected_u.u_delta,
                   pending.selected_u.u_delta);
  ASSERT_TRUE(adapter.publishPendingPositionCommand(
      []() { return true; }, capture.identity));
  EXPECT_EQ(adapter.execution_authority_.snapshot().selected_u_owner,
            phase_offset_navigation::PhaseOffsetAllocator::ownerName());
}

TEST(PhaseOffsetMatchedAdapterC3,
     NonzeroGDesUsesExactAllocatorSelectionAndMatchedComposition) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
  config.u_w_abs_max = 0.20;
  config.u_delta_abs_max = 0.40;
  PhaseOffsetMatchedAdapter adapter(config);
  NormalProductionFixture fixture = MakeNormalProductionFixture(adapter);
  ASSERT_TRUE(fixture.pair);
  fixture.input.g_des_valid = true;
  fixture.input.g_des = Eigen::Vector3d(0.08, 0.16, 0.0);
  MatchedAdapterOutput output;
  ASSERT_TRUE(adapter.update(fixture.input, output)) << output.invalid_reason;
  ASSERT_TRUE(output.selected);
  ASSERT_TRUE(output.allocator.valid);
  EXPECT_EQ(output.allocator.selected_u_owner,
            phase_offset_navigation::PhaseOffsetAllocator::ownerName());
  EXPECT_TRUE(output.allocator.selectedUConsistent(0.0));
  EXPECT_DOUBLE_EQ(output.allocator.selected_u.u_w,
                   output.projection.final_port.u_w);
  EXPECT_DOUBLE_EQ(output.allocator.selected_u.u_delta,
                   output.projection.final_port.u_delta);
  const Eigen::Vector3d expected_physical = output.geometry.r_w *
      output.allocator.selected_u.u_w + output.geometry.N *
      output.allocator.selected_u.u_delta;
  EXPECT_NEAR((expected_physical - output.matched.physical_port).norm(), 0.0,
              1e-12);
  const PendingPositionCommandCapture capture =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(capture.pending);
  ASSERT_TRUE(capture.valid);
  const phase_offset_navigation::ActiveReferenceSnapshot pending =
      *adapter.pending_authority_prepared_.committed_snapshot;
  EXPECT_DOUBLE_EQ(output.allocator.selected_u_w, pending.selected_u_w);
  EXPECT_DOUBLE_EQ(output.allocator.selected_u_delta, pending.selected_u_delta);
  EXPECT_EQ(pending.selected_u_owner,
            phase_offset_navigation::PhaseOffsetAllocator::ownerName());
  ASSERT_TRUE(adapter.publishPendingPositionCommand(
      []() { return true; }, capture.identity));
}

TEST(PhaseOffsetMatchedAdapterC3,
     PhaseOnlyPortUsesExactProductionMatchedContribution) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
  config.u_w_abs_max = 1.0;
  config.u_delta_abs_max = 1.0;
  config.u_w_rate_max = 100.0;
  config.u_delta_rate_max = 100.0;
  PhaseOffsetMatchedAdapter adapter(config);
  NormalProductionFixture fixture = MakeNormalProductionFixture(adapter);
  ASSERT_TRUE(fixture.pair);
  fixture.input.g_des_valid = true;
  fixture.input.g_des = Eigen::Vector3d(0.08, 0.0, 0.0);

  MatchedAdapterOutput output;
  ASSERT_TRUE(adapter.update(fixture.input, output)) << output.invalid_reason;
  ASSERT_TRUE(output.selected);
  ASSERT_TRUE(output.allocator.valid);
  EXPECT_NEAR(output.allocator.u_delta_nom, 0.0, 1e-12);
  EXPECT_NEAR(output.allocator.selected_u.u_delta, 0.0, 1e-12);
  EXPECT_NEAR(output.allocator.selected_u.u_w, 0.08, 1e-12);
  const Eigen::Vector3d expected = output.geometry.r_w *
      output.allocator.selected_u.u_w + output.geometry.N *
      output.allocator.selected_u.u_delta;
  EXPECT_LE((output.matched.physical_port - expected).norm(), 1e-12);
  EXPECT_EQ(output.allocator.selected_u_owner,
            phase_offset_navigation::PhaseOffsetAllocator::ownerName());
}

TEST(PhaseOffsetMatchedAdapterC3,
     TransverseOnlyPortUsesExactProductionMatchedContribution) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
  config.u_w_abs_max = 1.0;
  config.u_delta_abs_max = 1.0;
  config.u_w_rate_max = 100.0;
  config.u_delta_rate_max = 100.0;
  PhaseOffsetMatchedAdapter adapter(config);
  NormalProductionFixture fixture = MakeNormalProductionFixture(adapter);
  ASSERT_TRUE(fixture.pair);
  fixture.input.g_des_valid = true;
  fixture.input.g_des = Eigen::Vector3d(0.0, 0.16, 0.0);

  MatchedAdapterOutput output;
  ASSERT_TRUE(adapter.update(fixture.input, output)) << output.invalid_reason;
  ASSERT_TRUE(output.selected);
  ASSERT_TRUE(output.allocator.valid);
  EXPECT_NEAR(output.allocator.u_w_nom, 0.0, 1e-12);
  EXPECT_NEAR(output.allocator.selected_u.u_w, 0.0, 1e-12);
  EXPECT_NEAR(output.allocator.selected_u.u_delta, 0.16, 1e-12);
  const Eigen::Vector3d expected = output.geometry.r_w *
      output.allocator.selected_u.u_w + output.geometry.N *
      output.allocator.selected_u.u_delta;
  EXPECT_LE((output.matched.physical_port - expected).norm(), 1e-12);
  EXPECT_EQ(output.allocator.selected_u_owner,
            phase_offset_navigation::PhaseOffsetAllocator::ownerName());
}

TEST(PhaseOffsetMatchedAdapterC3,
     SimultaneousPortUsesExactProductionMatchedBTimesU) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
  config.u_w_abs_max = 1.0;
  config.u_delta_abs_max = 1.0;
  config.u_w_rate_max = 100.0;
  config.u_delta_rate_max = 100.0;
  PhaseOffsetMatchedAdapter adapter(config);
  NormalProductionFixture fixture = MakeNormalProductionFixture(adapter);
  ASSERT_TRUE(fixture.pair);
  fixture.input.g_des_valid = true;
  fixture.input.g_des = Eigen::Vector3d(0.08, 0.16, 0.0);

  MatchedAdapterOutput output;
  ASSERT_TRUE(adapter.update(fixture.input, output)) << output.invalid_reason;
  ASSERT_TRUE(output.selected);
  ASSERT_TRUE(output.allocator.valid);
  EXPECT_NEAR(output.allocator.selected_u.u_w, 0.08, 1e-12);
  EXPECT_NEAR(output.allocator.selected_u.u_delta, 0.16, 1e-12);
  const Eigen::Vector3d expected = output.geometry.r_w *
      output.allocator.selected_u.u_w + output.geometry.N *
      output.allocator.selected_u.u_delta;
  EXPECT_LE((output.matched.physical_port - expected).norm(), 1e-12);
  EXPECT_EQ(output.allocator.selected_u_owner,
            phase_offset_navigation::PhaseOffsetAllocator::ownerName());
  const PendingPositionCommandCapture capture =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(capture.pending);
  ASSERT_TRUE(capture.valid);
  const auto pending = *adapter.pending_authority_prepared_.committed_snapshot;
  EXPECT_EQ(pending.selected_u_owner,
            phase_offset_navigation::PhaseOffsetAllocator::ownerName());
  EXPECT_TRUE(pending.selectedUConsistent(0.0));
  ASSERT_TRUE(adapter.publishPendingPositionCommand(
      []() { return true; }, capture.identity));
}

TEST(PhaseOffsetMatchedAdapterC3,
     NormalPendingPairRefreshRejectsStaleAAndCommitsNextB) {
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  NormalProductionFixture fixture = MakeNormalProductionFixture(adapter);
  ASSERT_TRUE(fixture.pair);

  MatchedAdapterOutput output;
  ASSERT_TRUE(adapter.update(fixture.input, output)) << output.invalid_reason;
  ASSERT_TRUE(output.selected);
  const PendingPositionCommandCapture capture =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(capture.pending);
  ASSERT_TRUE(capture.valid);
  const std::shared_ptr<const PathTubePair> pair_a =
      adapter.capturePathTubePair();
  ASSERT_TRUE(pair_a);
  ASSERT_EQ(adapter.pending_normal_source_pair_.get(), pair_a.get());

  const auto authority_before = adapter.execution_authority_.snapshot();
  const double retained_before = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      adapter.runtime_->previousFinalPort();

  // Deterministically model the timer's A->B replacement winning after the
  // manager's update and before publication.  B is numerically compatible
  // with A but has a distinct immutable shared_ptr identity/generation.
  std::shared_ptr<PathTubePair> replacement(new PathTubePair(*pair_a));
  replacement->generation = pair_a->generation + 1U;
  const std::shared_ptr<const PathTubePair> pair_b(replacement);
  EXPECT_NE(pair_b.get(), pair_a.get());
  EXPECT_EQ(pair_b->source_revision, pair_a->source_revision);
  EXPECT_EQ(pair_b->path_revision, pair_a->path_revision);
  EXPECT_EQ(pair_b->frame_revision, pair_a->frame_revision);
  EXPECT_EQ(pair_b->active_profile->tube_revision,
            pair_a->active_profile->tube_revision);
  EXPECT_EQ(pair_b->active_profile->profile_revision,
            pair_a->active_profile->profile_revision);
  std::atomic_store(&adapter.authoritative_path_tube_pair_, pair_b);
  ASSERT_EQ(adapter.capturePathTubePair().get(), pair_b.get());

  int callback_count = 0;
  EXPECT_FALSE(adapter.publishPendingPositionCommand(
      [&callback_count]() {
        ++callback_count;
        return true;
      }, capture.identity));
  EXPECT_EQ(callback_count, 0);
  EXPECT_FALSE(adapter.execution_authority_.snapshot().valid);
  EXPECT_EQ(adapter.execution_authority_.snapshot().snapshotId(),
            authority_before.snapshotId());
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), retained_before);
  EXPECT_EQ(0, std::memcmp(&previous_before.u_w,
                           &adapter.runtime_->previous_final_port_.u_w,
                           sizeof(previous_before.u_w)));
  EXPECT_EQ(0, std::memcmp(&previous_before.u_delta,
                           &adapter.runtime_->previous_final_port_.u_delta,
                           sizeof(previous_before.u_delta)));
  EXPECT_FALSE(adapter.hasPendingPositionCommand());
  EXPECT_FALSE(adapter.pending_normal_source_pair_);

  // The next command captures B and can publish normally; no second
  // lifecycle/session owner is introduced by the rejected stale A tick.
  fixture.input.path_tube_pair = pair_b;
  fixture.input.stamp = ros::Time(2.0);
  ASSERT_TRUE(adapter.update(fixture.input, output)) << output.invalid_reason;
  const PendingPositionCommandCapture replacement_capture =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(replacement_capture.pending);
  ASSERT_TRUE(replacement_capture.valid);
  EXPECT_EQ(adapter.pending_normal_source_pair_.get(), pair_b.get());
  ASSERT_TRUE(adapter.publishPendingPositionCommand(
      []() { return true; }, replacement_capture.identity));
  const auto authority_after = adapter.execution_authority_.snapshot();
  ASSERT_TRUE(authority_after.valid);
  EXPECT_EQ(authority_after.owner_mode,
            phase_offset_navigation::ActiveReferenceOwnerMode::NORMAL);
  EXPECT_EQ(authority_after.selected_u_owner,
            phase_offset_navigation::PhaseOffsetAllocator::ownerName());
  EXPECT_EQ(adapter.capturePathTubePair().get(), pair_b.get());
}

TEST(PhaseOffsetMatchedAdapterC3,
     AllocatorValueFailureDoesNotCreatePlannerHoldOrAuthorityMutation) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::FIXED);
  PhaseOffsetMatchedAdapter adapter(config);
  NormalProductionFixture fixture = MakeNormalProductionFixture(adapter);
  ASSERT_TRUE(fixture.pair);
  const phase_offset_navigation::ActiveReferenceSnapshot before =
      adapter.execution_authority_.snapshot();
  const double delta_before = adapter.runtime_->retainedDelta();
  fixture.input.g_des_valid = true;
  fixture.input.g_des = Eigen::Vector3d(
      std::numeric_limits<double>::quiet_NaN(), 0.1, 0.0);
  MatchedAdapterOutput output;
  EXPECT_FALSE(adapter.update(fixture.input, output));
  EXPECT_TRUE(output.allocator_evaluated);
  EXPECT_TRUE(output.allocator_value_failure);
  EXPECT_TRUE(output.valid);
  EXPECT_FALSE(output.selected);
  EXPECT_TRUE(output.base_guidance.valid);
  EXPECT_GT(output.base_guidance.v_cmd.norm(), 0.0);
  EXPECT_FALSE(adapter.hasPendingPositionCommand());
  EXPECT_DOUBLE_EQ(delta_before, adapter.runtime_->retainedDelta());
  const phase_offset_navigation::ActiveReferenceSnapshot after =
      adapter.execution_authority_.snapshot();
  EXPECT_EQ(after.snapshotId(), before.snapshotId());
}

TEST(PhaseOffsetMatchedAdapterC3,
     MissingOrInvalidNormalPolicyFailsOnlyTheProductionNormalEvaluation) {
  for (const bool missing_policy : {true, false}) {
    PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
    if (missing_policy) {
      config.normal_preview_policy =
          phase_offset_navigation::NormalPreviewProductionPolicy();
      config.normal_preview_policy_explicit = false;
    } else {
      // Keep the immutable value present but invalid; there is no production
      // clamping or defaulting path for an inconsistent phase-rate envelope.
      config.normal_preview_policy.lower_nu = 0.0;
    }
    PhaseOffsetMatchedAdapter adapter(config);
    EXPECT_TRUE(adapter.configurationValid());
    NormalProductionFixture fixture = MakeNormalProductionFixture(adapter);
    ASSERT_TRUE(fixture.pair);

    const phase_offset_navigation::ActiveReferenceSnapshot before =
        adapter.execution_authority_.snapshot();
    const double delta_before = adapter.runtime_->retainedDelta();
    const phase_offset_core::PortCommand previous_before =
        adapter.runtime_->previousFinalPort();
    const auto policy_before = adapter.config_.normal_preview_policy;
    const auto expect_same_double = [](const double expected,
                                       const double actual) {
      if (std::isnan(expected)) {
        EXPECT_TRUE(std::isnan(actual));
      } else {
        EXPECT_DOUBLE_EQ(expected, actual);
      }
    };

    MatchedAdapterOutput output;
    EXPECT_FALSE(adapter.update(fixture.input, output));
    EXPECT_TRUE(output.allocator_evaluated);
    EXPECT_TRUE(output.allocator_value_failure);
    EXPECT_FALSE(output.selected);
    // With no already-authoritative nonzero NORMAL state, the protected base
    // planner guidance remains available even though NORMAL Preview is not.
    EXPECT_TRUE(output.valid);
    EXPECT_TRUE(output.base_guidance.valid);
    EXPECT_FALSE(adapter.hasPendingPositionCommand());
    EXPECT_DOUBLE_EQ(delta_before, adapter.runtime_->retainedDelta());
    EXPECT_DOUBLE_EQ(previous_before.u_w,
                     adapter.runtime_->previousFinalPort().u_w);
    EXPECT_DOUBLE_EQ(previous_before.u_delta,
                     adapter.runtime_->previousFinalPort().u_delta);
    const phase_offset_navigation::ActiveReferenceSnapshot after =
        adapter.execution_authority_.snapshot();
    EXPECT_EQ(after.snapshotId(), before.snapshotId());
    EXPECT_EQ(after.selected_u_owner, before.selected_u_owner);
    expect_same_double(policy_before.preview_horizon_w,
                       adapter.config_.normal_preview_policy.preview_horizon_w);
    expect_same_double(policy_before.sample_spacing_w,
                       adapter.config_.normal_preview_policy.sample_spacing_w);
    expect_same_double(policy_before.lower_nu,
                       adapter.config_.normal_preview_policy.lower_nu);
    expect_same_double(policy_before.upper_nu,
                       adapter.config_.normal_preview_policy.upper_nu);
    expect_same_double(policy_before.b_tight,
                       adapter.config_.normal_preview_policy.b_tight);
    expect_same_double(policy_before.b_open,
                       adapter.config_.normal_preview_policy.b_open);
    EXPECT_EQ(policy_before.policy_revision,
              adapter.config_.normal_preview_policy.policy_revision);
    EXPECT_EQ(policy_before.configuration_identity,
              adapter.config_.normal_preview_policy.configuration_identity);
    EXPECT_EQ(policy_before.configuration_id,
              adapter.config_.normal_preview_policy.configuration_id);
  }
}

TEST(PhaseOffsetMatchedAdapterC3,
     MissingNormalPolicyDoesNotInvalidateTheNonNormalRuntimeSeam) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::FIXED);
  config.normal_preview_policy =
      phase_offset_navigation::NormalPreviewProductionPolicy();
  config.normal_preview_policy_explicit = false;
  PhaseOffsetMatchedAdapter adapter(config);

  // The adapter/tube seam remains usable for the unadvertised Runtime fixture
  // and neutral planner baseline.  No production NORMAL Preview evaluation is
  // entered on this path, so the absent policy is intentionally irrelevant.
  EXPECT_TRUE(adapter.configurationValid());
  EXPECT_TRUE(adapter.requiresTubeTimer());
  MatchedAdapterOutput output;
  for (int cycle = 0; cycle <= 100; ++cycle) {
    EXPECT_FALSE(adapter.update(MakeInput(path, &path, cycle * kDt), output));
    if (cycle == 0) {
      ASSERT_TRUE(adapter.timerTick());
    }
  }
  EXPECT_TRUE(output.valid) << output.invalid_reason;
  EXPECT_TRUE(output.zero_gate_open);
  EXPECT_FALSE(output.selected);
  EXPECT_TRUE(output.base_guidance.valid);
  EXPECT_TRUE(output.matched.valid);
  EXPECT_TRUE(output.active_profile);
}

TEST(PhaseOffsetMatchedAdapterC3,
     RepeatedNormalPolicyFailureIsDeterministicAndDoesNotMutatePolicy) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
  config.normal_preview_policy =
      phase_offset_navigation::NormalPreviewProductionPolicy();
  config.normal_preview_policy_explicit = false;
  PhaseOffsetMatchedAdapter adapter(config);
  NormalProductionFixture fixture = MakeNormalProductionFixture(adapter);
  ASSERT_TRUE(fixture.pair);
  const auto policy_before = adapter.config_.normal_preview_policy;

  MatchedAdapterOutput first;
  MatchedAdapterOutput second;
  EXPECT_FALSE(adapter.update(fixture.input, first));
  EXPECT_FALSE(adapter.update(fixture.input, second));
  EXPECT_TRUE(first.allocator_evaluated);
  EXPECT_TRUE(second.allocator_evaluated);
  EXPECT_TRUE(first.allocator_value_failure);
  EXPECT_TRUE(second.allocator_value_failure);
  EXPECT_EQ(first.invalid_reason, second.invalid_reason);
  EXPECT_EQ(first.allocator.status, second.allocator.status);
  EXPECT_EQ(first.allocator.reason, second.allocator.reason);
  EXPECT_EQ(first.selected, second.selected);
  EXPECT_EQ(first.valid, second.valid);
  const auto expect_same_double = [](const double expected,
                                     const double actual) {
    if (std::isnan(expected)) {
      EXPECT_TRUE(std::isnan(actual));
    } else {
      EXPECT_DOUBLE_EQ(expected, actual);
    }
  };
  expect_same_double(policy_before.preview_horizon_w,
                     adapter.config_.normal_preview_policy.preview_horizon_w);
  expect_same_double(policy_before.sample_spacing_w,
                     adapter.config_.normal_preview_policy.sample_spacing_w);
  expect_same_double(policy_before.lower_nu,
                     adapter.config_.normal_preview_policy.lower_nu);
  expect_same_double(policy_before.upper_nu,
                     adapter.config_.normal_preview_policy.upper_nu);
  expect_same_double(policy_before.b_tight,
                     adapter.config_.normal_preview_policy.b_tight);
  expect_same_double(policy_before.b_open,
                     adapter.config_.normal_preview_policy.b_open);
  EXPECT_EQ(policy_before.policy_revision,
            adapter.config_.normal_preview_policy.policy_revision);
  EXPECT_EQ(policy_before.configuration_identity,
            adapter.config_.normal_preview_policy.configuration_identity);
  EXPECT_EQ(policy_before.configuration_id,
            adapter.config_.normal_preview_policy.configuration_id);
  EXPECT_FALSE(adapter.hasPendingPositionCommand());
}

TEST(PhaseOffsetMatchedAdapterC3,
     ActiveNonzeroAllocatorFailureRetainsAuthorityWithoutNominalLeak) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
  config.u_w_abs_max = 1.0;
  config.u_delta_abs_max = 1.0;
  config.u_w_rate_max = 100.0;
  config.u_delta_rate_max = 100.0;
  PhaseOffsetMatchedAdapter adapter(config);
  NormalProductionFixture fixture = MakeNormalProductionFixture(adapter);
  ASSERT_TRUE(fixture.pair);
  fixture.input.g_des_valid = true;
  fixture.input.g_des = Eigen::Vector3d(0.0, 0.16, 0.0);

  MatchedAdapterOutput first;
  ASSERT_TRUE(adapter.update(fixture.input, first)) << first.invalid_reason;
  ASSERT_TRUE(first.selected);
  const PendingPositionCommandCapture first_capture =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(first_capture.pending);
  ASSERT_TRUE(first_capture.valid);
  ASSERT_TRUE(adapter.publishPendingPositionCommand(
      []() { return true; }, first_capture.identity));

  const phase_offset_navigation::ActiveReferenceSnapshot before =
      adapter.execution_authority_.snapshot();
  ASSERT_TRUE(before.valid);
  EXPECT_EQ(before.owner_mode,
            phase_offset_navigation::ActiveReferenceOwnerMode::NORMAL);
  EXPECT_EQ(before.selected_u_owner,
            phase_offset_navigation::PhaseOffsetAllocator::ownerName());
  ASSERT_GT(std::abs(before.proposed_next_delta), 1e-6);
  const double delta_before = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      adapter.runtime_->previousFinalPort();

  // Rebase only the immutable current path/guidance to the exact predecessor
  // phase; the pair and its owner remain unchanged for the failed tick.
  fixture.input.path = MakeStraightState(before.proposed_next_w);
  ActiveAdapterInput zero_input;
  zero_input.path = fixture.input.path;
  zero_input.position = fixture.input.position;
  zero_input.gains = fixture.input.gains;
  PhaseOffsetActiveAdapter zero;
  ActiveAdapterOutput zero_output;
  ASSERT_TRUE(zero.evaluate(zero_input, zero_output));
  fixture.input.legacy = LegacyGuidanceSnapshot(
      zero_output.guidance.v_cmd, zero_output.guidance.w_dot,
      zero_output.guidance.e_parallel, zero_output.guidance.e_perp,
      zero_output.guidance.ref_pt, zero_output.guidance.tangent,
      zero_output.guidance.valid);
  fixture.input.stamp = ros::Time(2.0);
  fixture.input.g_des = Eigen::Vector3d(
      std::numeric_limits<double>::quiet_NaN(), 0.16, 0.0);
  fixture.input.g_des_valid = true;

  MatchedAdapterOutput failed;
  EXPECT_FALSE(adapter.update(fixture.input, failed));
  EXPECT_TRUE(failed.allocator_evaluated);
  EXPECT_TRUE(failed.allocator_value_failure);
  EXPECT_FALSE(failed.selected);
  EXPECT_FALSE(failed.valid);
  EXPECT_FALSE(failed.matched.valid);
  EXPECT_TRUE(failed.guidance.v_cmd.isZero());
  EXPECT_FALSE(adapter.hasPendingPositionCommand());
  EXPECT_DOUBLE_EQ(delta_before, adapter.runtime_->retainedDelta());
  EXPECT_DOUBLE_EQ(previous_before.u_w,
                   adapter.runtime_->previousFinalPort().u_w);
  EXPECT_DOUBLE_EQ(previous_before.u_delta,
                   adapter.runtime_->previousFinalPort().u_delta);
  const phase_offset_navigation::ActiveReferenceSnapshot after =
      adapter.execution_authority_.snapshot();
  EXPECT_EQ(after.snapshotId(), before.snapshotId());
  EXPECT_EQ(after.selected_u_owner, before.selected_u_owner);
  EXPECT_DOUBLE_EQ(after.delta, before.delta);
  EXPECT_DOUBLE_EQ(after.proposed_next_delta, before.proposed_next_delta);
}

TEST(PhaseOffsetMatchedAdapterC3,
     AbsentProductionGDesUsesExactSingleUavRecenterFallback) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
  config.delta_tracking_gain = 2.5;
  config.u_w_abs_max = 1.0;
  config.u_delta_abs_max = 1.0;
  config.u_w_rate_max = 100.0;
  config.u_delta_rate_max = 100.0;
  PhaseOffsetMatchedAdapter adapter(config);
  NormalProductionFixture fixture = MakeNormalProductionFixture(adapter);
  ASSERT_TRUE(fixture.pair);
  fixture.input.g_des_valid = true;
  fixture.input.g_des = Eigen::Vector3d(0.0, 0.16, 0.0);

  MatchedAdapterOutput first;
  ASSERT_TRUE(adapter.update(fixture.input, first)) << first.invalid_reason;
  ASSERT_TRUE(first.selected);
  const PendingPositionCommandCapture first_capture =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(first_capture.pending);
  ASSERT_TRUE(first_capture.valid);
  ASSERT_TRUE(adapter.publishPendingPositionCommand(
      []() { return true; }, first_capture.identity));
  const phase_offset_navigation::ActiveReferenceSnapshot before =
      adapter.execution_authority_.snapshot();
  ASSERT_TRUE(before.valid);
  const double delta_before = adapter.runtime_->retainedDelta();
  ASSERT_GT(std::abs(delta_before), 1e-6);

  fixture.input.path = MakeStraightState(before.proposed_next_w);
  ActiveAdapterInput zero_input;
  zero_input.path = fixture.input.path;
  zero_input.position = fixture.input.position;
  zero_input.gains = fixture.input.gains;
  PhaseOffsetActiveAdapter zero;
  ActiveAdapterOutput zero_output;
  ASSERT_TRUE(zero.evaluate(zero_input, zero_output));
  fixture.input.legacy = LegacyGuidanceSnapshot(
      zero_output.guidance.v_cmd, zero_output.guidance.w_dot,
      zero_output.guidance.e_parallel, zero_output.guidance.e_perp,
      zero_output.guidance.ref_pt, zero_output.guidance.tangent,
      zero_output.guidance.valid);
  fixture.input.g_des = Eigen::Vector3d::Zero();
  fixture.input.g_des_valid = false;
  fixture.input.stamp = ros::Time(2.0);

  MatchedAdapterOutput fallback;
  ASSERT_TRUE(adapter.update(fixture.input, fallback)) << fallback.invalid_reason;
  ASSERT_TRUE(fallback.selected);
  ASSERT_TRUE(fallback.allocator.valid);
  ASSERT_TRUE(fallback.g_des_valid);
  const Eigen::Vector3d expected = -config.delta_tracking_gain *
      delta_before * fallback.geometry.N;
  EXPECT_TRUE(fallback.g_des.isApprox(expected, 0.0));
  EXPECT_NEAR(fallback.allocator.u_w_nom, 0.0, 1e-12);
  EXPECT_LE((fallback.matched.physical_port -
             (fallback.geometry.r_w * fallback.allocator.selected_u.u_w +
              fallback.geometry.N * fallback.allocator.selected_u.u_delta)).norm(),
            1e-12);
}

TEST(PhaseOffsetMatchedAdapterC3,
     AdvertisedNoPairRemainsPlannerOwnedUntilPairCas) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::FIXED);
  config.u_w_abs_max = 0.20;
  config.u_delta_abs_max = 0.40;
  PhaseOffsetMatchedAdapter adapter(config);
  ASSERT_TRUE(adapter.runtime_);
  // This exercises the advertised production owner contract without starting
  // a ROS timer worker; timerTick() remains the deterministic build boundary.
  adapter.advertised_ = true;
  adapter.execution_authority_.setTestOnlyRuntimeOwnerAllowed(false);
  adapter.authority_session_.store(1U, std::memory_order_release);
  const std::shared_ptr<const ContinuousPhasePath> owner =
      MakeStraightSyntheticOwner();
  ASSERT_TRUE(owner);
  MatchedAdapterInput input = MakeInput(MakeStraightSyntheticPath(),
                                        owner.get());
  input.semantic_path_owner = owner;
  input.semantic_path_start_w = owner->startW();
  input.semantic_path_end_w = owner->endW();
  MatchedAdapterOutput output;
  for (int cycle = 0; cycle < 101; ++cycle) {
    input.stamp = ros::Time(0.02 * cycle);
    EXPECT_FALSE(adapter.update(input, output));
    adapter.timerTick();
  }
  EXPECT_TRUE(adapter.requiresPathTubePairBootstrap());
  EXPECT_FALSE(adapter.capturePathTubePair());
  EXPECT_FALSE(adapter.execution_authority_.snapshot().valid);
  EXPECT_FALSE(adapter.hasPendingPositionCommand());
  EXPECT_FALSE(output.allocator_evaluated);
  EXPECT_FALSE(output.selected);
}

TEST(PhaseOffsetMatchedAdapterC3,
     AdvertisedPairRunsAllocatorOnlyAfterH2PairIsPresent) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
  config.u_w_abs_max = 0.20;
  config.u_delta_abs_max = 0.40;
  PhaseOffsetMatchedAdapter adapter(config);
  ASSERT_TRUE(adapter.runtime_);
  NormalProductionFixture fixture = MakeNormalProductionFixture(adapter);
  ASSERT_TRUE(fixture.pair);
  adapter.zero_gate_open_ = true;
  std::atomic_store(&adapter.authoritative_path_tube_pair_, fixture.pair);
  MatchedAdapterOutput output;
  fixture.input.g_des = Eigen::Vector3d(0.08, 0.0, 0.0);
  fixture.input.g_des_valid = true;
  ASSERT_TRUE(adapter.update(fixture.input, output)) << output.invalid_reason;
  ASSERT_TRUE(output.selected);
  ASSERT_TRUE(output.allocator_evaluated);
  EXPECT_EQ(output.allocator.selected_u_owner,
            phase_offset_navigation::PhaseOffsetAllocator::ownerName());
  EXPECT_TRUE(output.allocator.selectedUConsistent(0.0));
  EXPECT_TRUE(output.matched.physical_port.allFinite());
  EXPECT_EQ(adapter.capturePathTubePair(), fixture.pair);
  const PendingPositionCommandCapture capture =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(capture.pending);
  ASSERT_TRUE(capture.valid);
  const auto pending = adapter.execution_authority_.snapshot();
  EXPECT_FALSE(pending.valid);
  bool observed_before_commit = false;
  ASSERT_TRUE(adapter.publishPendingPositionCommand(
      [&]() {
        observed_before_commit = !adapter.execution_authority_.snapshot().valid;
        return true;
      }, capture.identity));
  EXPECT_TRUE(observed_before_commit);
  const auto committed = adapter.execution_authority_.snapshot();
  ASSERT_TRUE(committed.valid);
  EXPECT_EQ(committed.selected_u_owner,
            phase_offset_navigation::PhaseOffsetAllocator::ownerName());
  EXPECT_TRUE(committed.selectedUConsistent(0.0));
  EXPECT_DOUBLE_EQ(committed.selected_u_w, output.allocator.selected_u.u_w);
  EXPECT_DOUBLE_EQ(committed.selected_u_delta,
                   output.allocator.selected_u.u_delta);
}

bool StageValidPendingPositionCommand(PhaseOffsetMatchedAdapter& adapter) {
  const std::shared_ptr<const ContinuousPhasePath> owner =
      MakeSyntheticOwner();
  if (!owner) return false;
  const phase_offset_navigation::ImmutableExecutedReferenceQueryPtr query(
      new PhaseOffsetExecutedReferenceQuery(owner, 0.0, 1U, 1U, 1U, 4U));
  phase_offset_navigation::ExecutedReferenceQueryResult reference;
  if (!query->query(0.4, reference) || !reference.valid) return false;

  adapter.advertised_ = true;
  adapter.authority_session_.store(7U, std::memory_order_release);
  phase_offset_navigation::ActiveReferenceSnapshot candidate;
  candidate.authority_session = 7U;
  candidate.sequence = 1U;
  candidate.planner_path_revision = 1U;
  candidate.executed_path_revision = 1U;
  candidate.frame_revision = 1U;
  candidate.tube_revision = 1U;
  candidate.profile_revision = 1U;
  candidate.map_revision = 1U;
  candidate.owner_mode =
      phase_offset_navigation::ActiveReferenceOwnerMode::NORMAL;
  candidate.selected_u_owner = "PhaseOffsetAllocator";
  candidate.w = 0.4;
  candidate.delta = 0.0;
  candidate.dt = 0.1;
  candidate.u_prev = adapter.runtime_->previousFinalPort();
  candidate.selected_u.u_w = 0.01;
  candidate.selected_u.u_delta = -0.02;
  candidate.selected_u_w = candidate.selected_u.u_w;
  candidate.selected_u_delta = candidate.selected_u.u_delta;
  candidate.matched_base_v_cmd = Eigen::Vector3d::UnitX();
  candidate.matched_base_w_dot = 1.0;
  candidate.proposed_next_w = candidate.w + candidate.dt *
      (candidate.matched_base_w_dot + candidate.selected_u.u_w);
  candidate.proposed_next_delta = candidate.delta + candidate.dt *
      candidate.selected_u.u_delta;
  candidate.proposed_next_u_prev = candidate.selected_u;
  candidate.r = reference.r;
  candidate.r_w = reference.r_w;
  candidate.r_ww.setZero();
  candidate.r_ww_valid = false;
  candidate.executed_reference_query = query;
  candidate.reference_query_revision = query->queryRevision();
  candidate.provenance = "test/pending-position-command";
  candidate.valid = true;

  phase_offset_navigation::AuthorityPrepareInput authority_input;
  authority_input.candidate = candidate;
  authority_input.matched_output_valid = true;
  authority_input.reference_valid = candidate.governorViewValid();
  authority_input.provenance = candidate.provenance;
  phase_offset_navigation::AuthorityPreparedStep prepared;
  if (!adapter.execution_authority_.prepare(authority_input, prepared) ||
      !prepared.valid || !prepared.committed_snapshot) {
    return false;
  }
  std::shared_ptr<TubeProfile> source_profile(new TubeProfile());
  source_profile->source = TubeSource::FIXED;
  source_profile->source_revision = candidate.planner_path_revision;
  source_profile->path_revision = candidate.executed_path_revision;
  source_profile->frame_revision = candidate.frame_revision;
  source_profile->tube_revision = candidate.tube_revision;
  source_profile->profile_revision = candidate.profile_revision;
  source_profile->map_revision = candidate.map_revision;
  source_profile->complete = true;
  source_profile->raw_complete = true;
  source_profile->filtered_complete = true;
  source_profile->obstacle_certified = true;
  source_profile->classification =
      phase_offset_navigation::TubeProfileClassification::OFFSET_CERTIFIED;
  std::shared_ptr<PathTubePair> source_pair(new PathTubePair());
  source_pair->source_revision = candidate.planner_path_revision;
  source_pair->path_revision = candidate.executed_path_revision;
  source_pair->frame_revision = candidate.frame_revision;
  source_pair->generation = 1U;
  source_pair->authority_session = candidate.authority_session;
  source_pair->path_owner = owner;
  source_pair->active_profile =
      std::shared_ptr<const TubeProfile>(source_profile);
  source_pair->executed_reference_query = query;
  const std::shared_ptr<const PathTubePair> immutable_source_pair(source_pair);
  std::atomic_store(&adapter.authoritative_path_tube_pair_,
                    immutable_source_pair);
  phase_offset_navigation::RuntimeCommitToken token;
  token.expected_previous_final_port = adapter.runtime_->previousFinalPort();
  token.expected_delta = adapter.runtime_->retainedDelta();
  token.next_previous_final_port = candidate.selected_u;
  token.next_delta = candidate.proposed_next_delta;
  token.dt = candidate.dt;
  token.selected = true;
  token.valid = true;
  adapter.pending_runtime_commit_ = token;
  adapter.pending_authority_prepared_ = prepared;
  adapter.pending_normal_source_pair_ = immutable_source_pair;
  adapter.pending_authority_session_ = prepared.candidate.authority_session;
  adapter.pending_authority_valid_ = true;
  return true;
}

bool PrepareAndFinalizePair(
    PhaseOffsetMatchedAdapter& adapter,
    const PathTubePairTransaction& transaction,
    double current_w,
    const Eigen::Vector3d& position,
    const guidance::IsfGains& gains,
    double dt,
    const std::shared_ptr<const plan_env::CloudOccupancySnapshot>& snapshot,
    std::shared_ptr<const PathTubePair>& committed);

void OpenGate(PhaseOffsetMatchedAdapter& adapter, const SyntheticPath& path,
              MatchedAdapterOutput& output) {
  for (int cycle = 0; cycle < 100; ++cycle) {
    adapter.update(MakeInput(path, &path, cycle * kDt), output);
  }
  // Production installs this at the command-boundary activation bridge.  The
  // adapter unit tests exercise the same existing stage/prepare/finalize CAS
  // directly, so a fixed-Tube manual profile never starts from an unpaired
  // sidecar epoch.
  if (adapter.config_.mode != PhaseOffsetMatchedMode::MANUAL ||
      adapter.config_.tube_source == TubeSource::NONE) {
    return;
  }
  ASSERT_TRUE(adapter.requiresPathTubePairBootstrap());
  const std::shared_ptr<const ContinuousPhasePath> owner =
      MakeSyntheticOwner();
  ASSERT_TRUE(owner);
  MatchedAdapterInput activation_input = MakeInput(path, owner.get(),
                                                   100.0 * kDt);
  activation_input.semantic_path_owner = owner;
  activation_input.semantic_path_start_w = owner->startW();
  activation_input.semantic_path_end_w = owner->endW();
  PathTubePairTransaction transaction;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples,
      path.current.w, 1.0, 2.4, activation_input.position,
      activation_input.gains, activation_input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), transaction));
  std::shared_ptr<const PathTubePair> committed;
  ASSERT_TRUE(PrepareAndFinalizePair(
      adapter, transaction, path.current.w, activation_input.position,
      activation_input.gains, activation_input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), committed));
  ASSERT_TRUE(committed);
  ASSERT_TRUE(committed->active_profile);
  ASSERT_TRUE(committed->epoch_status.active_current_validation_valid);
  ASSERT_TRUE(adapter.hasPendingOffsetActivationPair(committed));
  activation_input.path_tube_pair = committed;
  const bool selected = adapter.update(activation_input, output);
  EXPECT_TRUE(output.runtime_execution.current_geometry_valid);
  EXPECT_TRUE(output.runtime_execution.current_bounds_valid);
  EXPECT_TRUE(output.runtime_execution.retained_delta_current_inside);
  ASSERT_TRUE(selected) << output.invalid_reason;
}

void BuildAndConsume(PhaseOffsetMatchedAdapter& adapter,
                     const MatchedAdapterInput& input,
  MatchedAdapterOutput& output) {
  adapter.update(input, output);
  adapter.timerTick();
  adapter.update(input, output);
}

bool PrepareAndFinalizePair(
    PhaseOffsetMatchedAdapter& adapter,
    const PathTubePairTransaction& transaction,
    const double current_w,
    const Eigen::Vector3d& position,
    const guidance::IsfGains& gains,
    const double dt,
    const std::shared_ptr<const plan_env::CloudOccupancySnapshot>& snapshot,
    std::shared_ptr<const PathTubePair>& committed) {
  PathTubePairCommitPreparation preparation;
  if (!adapter.preparePathTubePairCommit(transaction, current_w, position,
                                         gains, dt, snapshot, preparation)) {
    return false;
  }
  return adapter.finalizePreparedPathTubePairCommit(preparation, committed);
}

void ExpectActions(const visualization_msgs::MarkerArray& markers, int action) {
  ASSERT_EQ(markers.markers.size(), 3U);
  for (std::size_t index = 0U; index < markers.markers.size(); ++index) {
    EXPECT_EQ(markers.markers[index].id, static_cast<int>(index));
    EXPECT_EQ(markers.markers[index].action, action);
  }
}

std::shared_ptr<const TubeProfile> MakeMarkerProfile(double x) {
  auto profile = std::make_shared<TubeProfile>();
  profile->source = TubeSource::FIXED; profile->complete = true;
  for (int index = 0; index < 2; ++index) {
    TubeRawSample sample;
    sample.p = Eigen::Vector3d(x + index, 0.0, 1.0); sample.N = Eigen::Vector3d(0.0, 1.0, 0.0);
    sample.filtered_lower = -0.04; sample.filtered_upper = 0.04;
    profile->samples.push_back(sample);
  }
  return profile;
}

std::shared_ptr<const TubeProfile> MakeDualWidthMarkerProfile(
    const double x, const double certified_half_width,
    const double raw_half_width) {
  auto profile = std::make_shared<TubeProfile>();
  profile->source = TubeSource::FIXED;
  profile->raw_complete = true;
  profile->filtered_complete = true;
  profile->complete = true;
  for (int index = 0; index < 2; ++index) {
    TubeRawSample sample;
    sample.w = 0.4 + 0.1 * index;
    sample.p = Eigen::Vector3d(x + index, 0.0, 1.0);
    sample.N = Eigen::Vector3d::UnitY();
    sample.raw_lower = -certified_half_width;
    sample.raw_upper = certified_half_width;
    sample.filtered_lower = -certified_half_width;
    sample.filtered_upper = certified_half_width;
    sample.complete = true;
    profile->samples.push_back(sample);
    sample.raw_lower = -raw_half_width;
    sample.raw_upper = raw_half_width;
    sample.filtered_lower = -raw_half_width;
    sample.filtered_upper = raw_half_width;
    profile->raw_build_samples.push_back(sample);
  }
  return profile;
}

double TubeMarkerWidth(const visualization_msgs::MarkerArray& markers) {
  if (markers.markers.size() != 3U ||
      markers.markers[0].points.empty() ||
      markers.markers[1].points.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return markers.markers[1].points.front().y -
      markers.markers[0].points.front().y;
}

void ExpectMarkerArraysEqual(const visualization_msgs::MarkerArray& lhs,
                             const visualization_msgs::MarkerArray& rhs) {
  ASSERT_EQ(lhs.markers.size(), rhs.markers.size());
  for (std::size_t marker_index = 0U;
       marker_index < lhs.markers.size(); ++marker_index) {
    const visualization_msgs::Marker& left = lhs.markers[marker_index];
    const visualization_msgs::Marker& right = rhs.markers[marker_index];
    EXPECT_EQ(left.ns, right.ns);
    EXPECT_EQ(left.id, right.id);
    EXPECT_EQ(left.action, right.action);
    ASSERT_EQ(left.points.size(), right.points.size());
    for (std::size_t point_index = 0U;
         point_index < left.points.size(); ++point_index) {
      EXPECT_DOUBLE_EQ(left.points[point_index].x,
                       right.points[point_index].x);
      EXPECT_DOUBLE_EQ(left.points[point_index].y,
                       right.points[point_index].y);
      EXPECT_DOUBLE_EQ(left.points[point_index].z,
                       right.points[point_index].z);
    }
  }
}

MatchedAdapterOutput MakeMarkerOutput();

struct PairPublicationFixture {
  std::shared_ptr<const ContinuousPhasePath> owner;
  std::shared_ptr<const TubeProfile> pair_profile;
  std::shared_ptr<const TubeProfile> timer_candidate_profile;
  std::shared_ptr<const PathTubePair> pair;
  std::shared_ptr<const TubeEpochSnapshot> timer_candidate_epoch;
  TubeBuildRequest request;
  ControlPublishSnapshot control;
  std::uint64_t task_generation = 7U;
  std::uint64_t authority_session = 5U;
};

PairPublicationFixture MakePairPublicationFixture() {
  PairPublicationFixture fixture;
  fixture.owner = MakeSyntheticOwner();
  fixture.pair_profile = MakeDualWidthMarkerProfile(20.0, 0.10, 2.50);
  fixture.timer_candidate_profile =
      MakeDualWidthMarkerProfile(10.0, 0.20, 3.00);
  const std::uint64_t source_revision = 23U;
  const std::uint64_t epoch_map_sequence = 41U;

  MatchedAdapterOutput output = MakeMarkerOutput();
  output.candidate_profile = fixture.pair_profile;
  output.active_profile = fixture.pair_profile;
  output.tube_epoch_status.candidate_sequence = 29U;
  output.tube_epoch_status.active_tube_epoch = 19U;
  output.tube_epoch_status.candidate_path_source_revision = source_revision;
  output.tube_epoch_status.active_path_source_revision = source_revision;
  output.tube_epoch_status.candidate_map_observation_sequence =
      epoch_map_sequence;
  output.tube_epoch_status.active_map_observation_sequence =
      epoch_map_sequence;
  output.diagnostics[kSelectedManual] = 0.75;

  std::shared_ptr<TubeEpochSnapshot> pair_epoch(new TubeEpochSnapshot());
  pair_epoch->active = true;
  pair_epoch->task_generation = fixture.task_generation;
  pair_epoch->build_sequence = 31U;
  pair_epoch->source_revision = source_revision;
  pair_epoch->map_observation_sequence = epoch_map_sequence;
  pair_epoch->map_observation_is_snapshot = true;
  pair_epoch->candidate_build_w = 0.5;
  pair_epoch->full_path_samples =
      std::make_shared<const MatchedAdapterPathSamples>(MakePath().samples);
  pair_epoch->candidate_profile = fixture.pair_profile;
  pair_epoch->active_profile = fixture.pair_profile;
  pair_epoch->epoch_status = output.tube_epoch_status;

  std::shared_ptr<PathTubePair> pair(new PathTubePair());
  pair->source_revision = source_revision;
  pair->generation = 17U;
  pair->authority_session = fixture.authority_session;
  pair->map_observation_sequence = epoch_map_sequence;
  pair->map_observation_is_snapshot = true;
  pair->path_owner = fixture.owner;
  pair->full_path_samples = pair_epoch->full_path_samples;
  pair->active_profile = fixture.pair_profile;
  pair->epoch_status = output.tube_epoch_status;
  pair->epoch_snapshot = pair_epoch;
  fixture.pair = pair;

  fixture.request.active = true;
  fixture.request.task_generation = fixture.task_generation;
  fixture.request.control_sequence = 71U;
  fixture.request.source_revision = source_revision;
  fixture.request.authority_session = fixture.authority_session;
  fixture.request.map_observation_sequence = 99U;
  fixture.request.map_observation_is_snapshot = true;
  fixture.request.semantic_path_owner = fixture.owner;
  fixture.request.base_path_tube_pair = fixture.pair;
  fixture.request.base_path_tube_pair_generation = fixture.pair->generation;

  fixture.control.active = true;
  fixture.control.task_generation = fixture.task_generation;
  fixture.control.control_sequence = fixture.request.control_sequence;
  fixture.control.source_revision = source_revision;
  fixture.control.map_observation_sequence =
      fixture.request.map_observation_sequence;
  fixture.control.map_observation_is_snapshot = true;
  fixture.control.candidate_epoch_source_revision = source_revision;
  fixture.control.candidate_epoch_map_observation_sequence =
      epoch_map_sequence;
  fixture.control.candidate_epoch_map_observation_is_snapshot = true;
  fixture.control.epoch_build_sequence = pair_epoch->build_sequence;
  fixture.control.current_w = 0.4;
  fixture.control.position = Eigen::Vector3d(0.4, 0.0, 1.0);
  fixture.control.epoch_snapshot = pair_epoch;
  fixture.control.full_path_samples = pair_epoch->full_path_samples;
  fixture.control.output = output;

  std::shared_ptr<TubeEpochSnapshot> timer_epoch(new TubeEpochSnapshot());
  timer_epoch->active = true;
  timer_epoch->task_generation = fixture.task_generation;
  timer_epoch->build_sequence = 32U;
  timer_epoch->source_revision = source_revision;
  timer_epoch->map_observation_sequence = 99U;
  timer_epoch->map_observation_is_snapshot = true;
  timer_epoch->candidate_build_w = 0.4;
  timer_epoch->candidate_profile = fixture.timer_candidate_profile;
  timer_epoch->epoch_status = output.tube_epoch_status;
  fixture.timer_candidate_epoch = timer_epoch;
  return fixture;
}

struct R3CertifiedGeometryFixture {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
  std::shared_ptr<const ContinuousPhasePath> owner;
  std::shared_ptr<const ContinuousPhaseNormalFrame> frame;
  std::shared_ptr<const TubeBuildRequest> request;
  std::shared_ptr<const TubeEpochSnapshot> candidate;
};

R3CertifiedGeometryFixture MakeR3CertifiedGeometryFixture() {
  R3CertifiedGeometryFixture fixture;
  auto mutable_owner = std::make_shared<ContinuousPhasePath>();
  EXPECT_TRUE(mutable_owner->appendSegment(
      0.0, 3.0, "r3-certified-straight",
      [](const double w, ContinuousPhasePathState& state) {
        state.p = Eigen::Vector3d(w, 0.0, 1.0);
        state.dp_dw = Eigen::Vector3d::UnitX();
        state.d2p_dw2.setZero();
        state.valid = true;
        return true;
      }));
  fixture.owner = std::shared_ptr<const ContinuousPhasePath>(mutable_owner);
  fixture.frame = std::make_shared<const ContinuousPhaseNormalFrame>(
      fixture.owner, 7U, 8U);
  std::shared_ptr<TubeProfile> profile(new TubeProfile());
  profile->source = TubeSource::ESDF;
  profile->source_revision = 7U;
  profile->path_revision = 7U;
  profile->frame_revision = 8U;
  profile->tube_revision = 11U;
  profile->profile_revision = 11U;
  profile->map_revision = 41U;
  profile->snapshot_sequence = 41U;
  profile->snapshot_provenance_is_immutable = true;
  profile->raw_complete = true;
  profile->filtered_complete = true;
  profile->complete = true;
  profile->obstacle_certified = true;
  profile->classification =
      phase_offset_navigation::TubeProfileClassification::OFFSET_CERTIFIED;
  for (const double w : {0.0, 0.5, 1.0}) {
    TubeRawSample sample;
    sample.w = w;
    sample.path_revision = 7U;
    sample.frame_revision = 8U;
    sample.p = Eigen::Vector3d(w, 0.0, 1.0);
    sample.N = Eigen::Vector3d::UnitY();
    sample.filtered_lower = -0.12;
    sample.filtered_upper = 0.12;
    sample.complete = true;
    profile->samples.push_back(sample);
  }

  TubeEpochStatus status;
  status.candidate_sequence = 11U;
  status.candidate_path_source_revision = 7U;
  status.candidate_map_observation_sequence = 41U;
  status.map_observation_is_snapshot = true;
  status.candidate_raw_complete = true;
  status.candidate_filtered_complete = true;
  status.candidate_complete = true;
  status.candidate_classification =
      phase_offset_navigation::TubeProfileClassification::OFFSET_CERTIFIED;
  status.candidate_zero_only = false;

  std::shared_ptr<TubeEpochSnapshot> candidate(new TubeEpochSnapshot());
  candidate->active = true;
  candidate->task_generation = 3U;
  candidate->request_control_sequence = 5U;
  candidate->build_sequence = 19U;
  candidate->source_revision = 7U;
  candidate->map_observation_sequence = 41U;
  candidate->map_observation_is_snapshot = true;
  candidate->candidate_profile = std::shared_ptr<const TubeProfile>(profile);
  candidate->epoch_status = status;
  fixture.candidate = std::shared_ptr<const TubeEpochSnapshot>(candidate);

  std::shared_ptr<TubeBuildRequest> request(new TubeBuildRequest());
  request->active = true;
  request->task_generation = 3U;
  request->control_sequence = 8U;
  request->source_revision = 7U;
  request->map_observation_sequence = 99U;
  request->map_observation_is_snapshot = true;
  request->semantic_path_owner = fixture.owner;
  request->frame_owner = fixture.frame;
  request->current_path = MakeStraightState(0.5);
  fixture.request = std::shared_ptr<const TubeBuildRequest>(request);
  return fixture;
}

std::shared_ptr<const TubeProfile> MakeOneSidedPreparedProfile(
    const std::shared_ptr<const ContinuousPhasePath>& owner,
    const std::uint64_t source_revision,
    const double lower = 0.125, const double future_lower = 0.18,
    const double upper = 0.30) {
  if (!owner || owner->empty()) return std::shared_ptr<const TubeProfile>();
  std::shared_ptr<TubeProfile> profile(new TubeProfile());
  profile->source = TubeSource::FIXED;
  profile->source_revision = source_revision;
  profile->tube_revision = source_revision;
  profile->preview_start_w = 0.0;
  profile->preview_end_w = 3.0;
  profile->requested_preview_start_w = 0.0;
  profile->requested_preview_end_w = 3.0;
  profile->certified_segment_start_w = 0.0;
  profile->certified_segment_end_w = 3.0;
  profile->raw_complete = true;
  profile->filtered_complete = true;
  profile->complete = true;
  phase_offset_core::GeometryParams geometry_params;
  geometry_params.regularity_margin = 0.10;
  phase_offset_core::GeometryEvaluator evaluator(geometry_params);
  for (const double w : {0.0, 0.4, 1.0, 2.4, 3.0}) {
    ContinuousPhasePathState owner_state;
    phase_offset_core::PhaseOffsetGeometryState geometry;
    if (!owner->evaluate(w, owner_state, false)) {
      return std::shared_ptr<const TubeProfile>();
    }
    const phase_offset_core::PathDifferentialState path =
        ConvertContinuousPhasePathStateForActive(owner_state, w);
    if (!evaluator.evaluate(path, path.p, 0.0, geometry)) {
      return std::shared_ptr<const TubeProfile>();
    }
    TubeRawSample sample;
    sample.w = w;
    sample.p = geometry.p;
    sample.N = geometry.N;
    // The current section accepts the armed positive retained port, but the
    // future seam does not accept that unchanged scalar.  This is the exact
    // case for which a static retained-at-seam proxy is wrong: a Runtime
    // continuous witness may evolve the port before the future boundary.
    const double lower_at_w = w <= 0.4 ? lower :
        lower + (future_lower - lower) * std::min(1.0, (w - 0.4) / 0.6);
    sample.raw_lower = sample.filtered_lower = lower_at_w;
    sample.raw_upper = sample.filtered_upper = upper;
    sample.complete = true;
    profile->samples.push_back(sample);
  }
  return profile;
}

TubeEpochStatus MakeOneSidedEpochStatus(const std::uint64_t source_revision) {
  TubeEpochStatus status;
  status.state = TubeEpochState::ROLLING;
  status.disposition = TubeInstallDisposition::INITIAL_INSTALL;
  status.candidate_path_source_revision = source_revision;
  status.active_path_source_revision = source_revision;
  status.candidate_complete = true;
  status.active_available = true;
  status.active_current_validation_valid = true;
  status.current_bounds_valid = true;
  status.current_state_admissible = true;
  status.forward_horizon_sufficient = true;
  return status;
}

MatchedAdapterOutput MakeMarkerOutput() {
  MatchedAdapterOutput output;
  output.geometry.valid = true; output.candidate_profile = MakeMarkerProfile(10.0);
  output.active_profile = MakeMarkerProfile(20.0);
  output.tube_epoch_status.state = TubeEpochState::ROLLING;
  output.tube_epoch_status.active_tube_epoch = 1U;
  output.tube_epoch_status.active_current_validation_valid = true;
  output.runtime_execution.current_geometry_valid = true;
  output.runtime_execution.current_bounds_valid = true;
  output.runtime_execution.retained_delta_current_inside = true;
  output.runtime_execution.tracking_within_bound = true;
  output.runtime_execution.mode =
      phase_offset_navigation::RuntimeExecutionMode::NORMAL;
  output.projection.valid = true;
  output.matched.valid = true;
  output.valid = true;
  return output;
}

TEST(PhaseOffsetMatchedAdapterAttribution,
     TubeUpdateCompletionBitsMapInPipelineOrderAndNothingElse) {
  struct Case {
    bool raw_complete;
    bool filtered_complete;
    bool complete;
    PathTubePairStageFailure expected;
  };
  const std::array<Case, 4U> cases{{
      {false, false, false, PathTubePairStageFailure::TUBE_RAW_BUILD},
      {true, false, false, PathTubePairStageFailure::TUBE_FILTER},
      {true, true, false,
       PathTubePairStageFailure::TUBE_SURFACE_VALIDATOR},
      {true, true, true,
       PathTubePairStageFailure::TUBE_BUILD_PRECONDITION},
  }};
  for (const Case& test_case : cases) {
    phase_offset_navigation::TubeEpochUpdateResult prepared_epoch;
    prepared_epoch.status.candidate_raw_complete = test_case.raw_complete;
    prepared_epoch.status.candidate_filtered_complete =
        test_case.filtered_complete;
    prepared_epoch.status.candidate_complete = test_case.complete;
    // These intentionally disagree with the status.  `tube_update_status`
    // must describe the prepared epoch's candidate pipeline bits only;
    // profile/provenance/current-validation mismatches stay precondition or
    // coverage failures at their own callers.
    prepared_epoch.candidate_profile.raw_complete = !test_case.raw_complete;
    prepared_epoch.candidate_profile.filtered_complete =
        !test_case.filtered_complete;
    prepared_epoch.candidate_profile.complete = !test_case.complete;
    EXPECT_EQ(test_case.expected,
              PhaseOffsetMatchedAdapter::
                  classifyTubeUpdateStatusStageFailure(prepared_epoch));
  }
}

TEST(PhaseOffsetMatchedAdapterTest,
     GateOpensOnlyAtCycleOneHundredAndDoesNotSelectAnUnpairedEpoch) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  for (int cycle = 0; cycle < 99; ++cycle) {
    adapter.update(MakeInput(path, &path, cycle * kDt), output);
    if (cycle == 0) {
      ASSERT_TRUE(adapter.timerTick());
    }
    EXPECT_FALSE(output.zero_gate_open);
    EXPECT_FALSE(output.selected);
  }
  ASSERT_TRUE(output.active_profile);
  const std::uint64_t epoch_before_gate = output.tube_epoch_status.active_tube_epoch;
  EXPECT_EQ(epoch_before_gate, 1U);
  MatchedAdapterMarkerBundle markers;
  ASSERT_TRUE(adapter.buildMarkers(MakeInput(path, &path), output, markers));
  ExpectActions(markers.tube_candidate, visualization_msgs::Marker::ADD);
  ExpectActions(markers.tube, visualization_msgs::Marker::ADD);
  adapter.update(MakeInput(path, &path, 100.0 * kDt), output);
  EXPECT_TRUE(output.zero_gate_open);
  EXPECT_EQ(output.zero_gate_consecutive_count, 100);
  // The sidecar epoch is still a valid Tube observation, but it cannot issue
  // the first nonzero profile port until the manager has committed a matching
  // same-owner PathTubePair at this activation edge.
  EXPECT_FALSE(output.selected);
  EXPECT_TRUE(output.valid) << output.invalid_reason;
  EXPECT_TRUE(adapter.requiresPathTubePairBootstrap());
  EXPECT_EQ(output.tube_epoch_status.active_tube_epoch, epoch_before_gate);
}

TEST(PhaseOffsetMatchedAdapterTest,
     ObserveOnlyBuildsCandidateButNeverSelectsOrCommits) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::FIXED);
  config.observe_only = true;
  PhaseOffsetMatchedAdapter adapter(config);
  MatchedAdapterOutput output;

  for (int cycle = 0; cycle < 120; ++cycle) {
    EXPECT_FALSE(adapter.update(MakeInput(path, &path, cycle * kDt), output));
    if (cycle == 0) {
      ASSERT_TRUE(adapter.timerTick());
    }
    EXPECT_FALSE(output.selected);
    EXPECT_FALSE(output.failure_latched);
    EXPECT_DOUBLE_EQ(output.delta, 0.0);
    ASSERT_TRUE(adapter.runtime_);
    EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), 0.0);
    const phase_offset_core::PortCommand previous =
        adapter.runtime_->previousFinalPort();
    EXPECT_DOUBLE_EQ(previous.u_w, 0.0);
    EXPECT_DOUBLE_EQ(previous.u_delta, 0.0);
  }

  EXPECT_TRUE(output.zero_gate_open);
  EXPECT_EQ(output.zero_gate_consecutive_count, 100);
  EXPECT_TRUE(output.valid) << output.invalid_reason;
  EXPECT_TRUE(output.projection.valid);
  EXPECT_TRUE(output.matched.valid);
  ASSERT_TRUE(output.candidate_profile);
  EXPECT_TRUE(output.candidate_profile->raw_complete);
  EXPECT_TRUE(output.candidate_profile->filtered_complete);
  EXPECT_TRUE(output.candidate_profile->complete);
  MatchedAdapterMarkerBundle markers;
  ASSERT_TRUE(adapter.buildMarkers(MakeInput(path, &path, 120.0 * kDt),
                                   output, markers));
  ExpectActions(markers.tube_candidate, visualization_msgs::Marker::ADD);
}

TEST(PhaseOffsetMatchedAdapterTest,
     OrdinaryRequestCarriesRetiredAuthoritySessionForTimerPublication) {
  const SyntheticPath path = MakePath();
  int identity = 911;
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));

  // Planner-owned direct C2 installation retires the previous authority even
  // when Runtime remains neutral/observe-only.  A subsequent ordinary
  // request must carry the new session into the timer source-current check.
  const std::uint64_t session = adapter.retirePathTubeAuthority(1U);
  ASSERT_GT(session, 0U);

  MatchedAdapterOutput output;
  MatchedAdapterInput input = MakeInput(path, &identity, 0.0);
  ASSERT_FALSE(adapter.update(input, output));
  const std::shared_ptr<const TubeBuildRequest> request =
      std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(request);
  EXPECT_EQ(request->authority_session, session);
  EXPECT_EQ(request->authority_session,
            adapter.authority_session_.load(std::memory_order_acquire));

  ASSERT_TRUE(adapter.timerTick());
  ASSERT_TRUE(std::atomic_load(&adapter.latest_candidate_epoch_snapshot_));

  // The next command consumes the completed epoch; its candidate marker must
  // be ADD even though the Runtime remains observe-only/nonselected.
  ASSERT_FALSE(adapter.update(input, output));
  ASSERT_TRUE(output.candidate_profile);
  MatchedAdapterMarkerBundle markers;
  ASSERT_TRUE(adapter.buildMarkers(input, output, markers));
  ExpectActions(markers.tube_candidate, visualization_msgs::Marker::ADD);
}

TEST(PhaseOffsetMatchedAdapterTest,
     EsdfAuthorityRequestUsesConfiguredAmplitudeRetainedDeltaAndOneMargin) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
  config.amplitude = 0.10;
  config.tube.fixed_delta_max = 0.04;
  config.tube.cross_section.search_extent = 3.0;
  config.tube.interior_margin = 0.0;
  PhaseOffsetMatchedAdapter adapter(config);
  phase_offset_navigation::TubeBounds request;

  ASSERT_TRUE(adapter.makeAuthorityRequest(0.0, request));
  EXPECT_TRUE(request.valid);
  EXPECT_DOUBLE_EQ(request.lower, -0.10);
  EXPECT_DOUBLE_EQ(request.upper, 0.10);
  EXPECT_DOUBLE_EQ(request.query_w, 0.0);
  EXPECT_DOUBLE_EQ(request.lower_w, 0.0);
  EXPECT_DOUBLE_EQ(request.upper_w, 0.0);
  EXPECT_GT(request.upper, config.tube.fixed_delta_max);
  EXPECT_LT(request.lower, -config.tube.fixed_delta_max);

  ASSERT_TRUE(adapter.makeAuthorityRequest(-0.15, request));
  EXPECT_DOUBLE_EQ(request.lower, -0.15);
  EXPECT_DOUBLE_EQ(request.upper, 0.10);
  ASSERT_TRUE(adapter.makeAuthorityRequest(0.16, request));
  EXPECT_DOUBLE_EQ(request.lower, -0.10);
  EXPECT_DOUBLE_EQ(request.upper, 0.16);

  config.tube.interior_margin = 0.02;
  PhaseOffsetMatchedAdapter with_margin(config);
  phase_offset_navigation::TubeBounds before_preflight;
  ASSERT_TRUE(with_margin.makeAuthorityRequest(0.16, before_preflight));
  EXPECT_DOUBLE_EQ(before_preflight.lower, -0.12);
  EXPECT_DOUBLE_EQ(before_preflight.upper, 0.18);
  ASSERT_TRUE(with_margin.runtime_);
  with_margin.runtime_->preflight_.configured_amplitude = 0.10;
  with_margin.runtime_->preflight_.accepted_amplitude = 0.01;
  with_margin.runtime_->preflight_.amplitude_reduced = true;
  phase_offset_navigation::TubeBounds after_preflight;
  ASSERT_TRUE(with_margin.makeAuthorityRequest(0.16, after_preflight));
  EXPECT_DOUBLE_EQ(after_preflight.lower, before_preflight.lower);
  EXPECT_DOUBLE_EQ(after_preflight.upper, before_preflight.upper);
}

TEST(PhaseOffsetMatchedAdapterTest,
     InvalidEsdfAuthorityFactsFailClosedWithoutAlternativeLimit) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
  PhaseOffsetMatchedAdapter adapter(config);
  phase_offset_navigation::TubeBounds request;
  EXPECT_FALSE(adapter.makeAuthorityRequest(
      std::numeric_limits<double>::infinity(), request));
  EXPECT_FALSE(request.valid);

  config.amplitude = std::numeric_limits<double>::quiet_NaN();
  PhaseOffsetMatchedAdapter invalid_amplitude(config);
  EXPECT_FALSE(invalid_amplitude.makeAuthorityRequest(0.0, request));
  EXPECT_FALSE(request.valid);

  config = MakeManualConfig(TubeSource::ESDF);
  config.tube.interior_margin = -0.01;
  PhaseOffsetMatchedAdapter invalid_margin(config);
  EXPECT_FALSE(invalid_margin.makeAuthorityRequest(0.0, request));
  EXPECT_FALSE(request.valid);
}

TEST(PhaseOffsetMatchedAdapterTest,
     ObservationOnlyAndNeutralCurrentFutureIntentHandoff) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapterConfig observe_config =
      MakeManualConfig(TubeSource::FIXED);
  observe_config.observe_only = true;
  PhaseOffsetMatchedAdapter observer(observe_config);
  EXPECT_TRUE(observer.requiresTubeTimer());
  EXPECT_FALSE(observer.requiresAuthoritativeOffsetHandoff());

  PhaseOffsetMatchedAdapter neutral(MakeManualConfig(TubeSource::FIXED));
  EXPECT_TRUE(neutral.requiresTubeTimer());
  // Before the zero-port warm-up finishes, configured amplitude is only
  // future intent: it neither captures the planner owner nor creates a Pair.
  ASSERT_TRUE(neutral.runtime_);
  EXPECT_DOUBLE_EQ(0.0, neutral.runtime_->retainedDelta());
  EXPECT_FALSE(neutral.requiresAuthoritativeOffsetHandoff());
  EXPECT_FALSE(neutral.requiresPathTubePairBootstrap());

  MatchedAdapterOutput output;
  BuildAndConsume(neutral, MakeInput(path, &path), output);
  EXPECT_FALSE(output.selected);
  ASSERT_TRUE(neutral.runtime_);
  EXPECT_DOUBLE_EQ(0.0, neutral.runtime_->retainedDelta());
  EXPECT_FALSE(neutral.requiresAuthoritativeOffsetHandoff());
  for (int cycle = 1; cycle < 100; ++cycle) {
    neutral.update(MakeInput(path, &path, cycle * kDt), output);
  }
  EXPECT_TRUE(output.zero_gate_open);
  EXPECT_FALSE(output.selected);
  EXPECT_TRUE(neutral.requiresPathTubePairBootstrap());
  neutral.runtime_->delta_ = 0.02;
  EXPECT_TRUE(neutral.requiresAuthoritativeOffsetHandoff());
  EXPECT_FALSE(neutral.requiresPathTubePairBootstrap());
}

TEST(PhaseOffsetMatchedAdapterTest,
     ProductionRuntimeCapabilityDisablesBootstrapAndPendingActivation) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  for (int cycle = 0; cycle < 100; ++cycle) {
    EXPECT_FALSE(adapter.update(MakeInput(path, &path, cycle * kDt), output));
    if (cycle == 0) {
      ASSERT_TRUE(adapter.timerTick());
    }
  }
  ASSERT_TRUE(output.zero_gate_open);
  ASSERT_TRUE(adapter.execution_authority_.config().allow_test_only_runtime_owner);
  ASSERT_TRUE(adapter.requiresPathTubePairBootstrap());

  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  ASSERT_TRUE(owner);
  MatchedAdapterInput activation_input = MakeInput(path, owner.get(), 100.0 * kDt);
  activation_input.semantic_path_owner = owner;
  activation_input.semantic_path_start_w = owner->startW();
  activation_input.semantic_path_end_w = owner->endW();
  PathTubePairTransaction transaction;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples,
      path.current.w, 1.0, 2.4, activation_input.position,
      activation_input.gains, activation_input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), transaction));
  std::shared_ptr<const PathTubePair> pair;
  ASSERT_TRUE(PrepareAndFinalizePair(
      adapter, transaction, path.current.w, activation_input.position,
      activation_input.gains, activation_input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), pair));
  ASSERT_TRUE(pair);
  ASSERT_TRUE(adapter.hasPendingOffsetActivationPair(pair));
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), 0.0);
  EXPECT_FALSE(adapter.execution_authority_.snapshot().valid);

  // This is the same capability fact cleared by advertise().  It is the only
  // permitted distinction between the unadvertised fixture and production.
  adapter.execution_authority_.setTestOnlyRuntimeOwnerAllowed(false);
  EXPECT_FALSE(adapter.requiresPathTubePairBootstrap());
  EXPECT_FALSE(adapter.hasPendingOffsetActivationPair(pair));
  std::uint64_t retired_session = 0U;
  EXPECT_TRUE(adapter.retirePathTubeAuthorityIfNeutral(
      adapter.authority_session_.load(std::memory_order_acquire),
      retired_session));
  EXPECT_FALSE(adapter.capturePathTubePair());
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), 0.0);
}

TEST(PhaseOffsetMatchedAdapterTest,
     EsdfBelowRequiredCentrelineClearanceRemainsZeroOnlyFailClosed) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
  phase_offset_navigation::CertifiedTubeBuilder builder(
      config.tube, config.filter,
      phase_offset_navigation::TubeSurfaceValidatorConfig());
  ASSERT_TRUE(builder.configurationValid());

  phase_offset_navigation::CertifiedTubeBuildInput input;
  input.source = TubeSource::ESDF;
  input.preview_path.push_back(MakeStraightState(0.4));
  input.preview_path.push_back(MakeStraightState(0.8));
  input.preview_path.push_back(MakeStraightState(1.2));
  input.current_w = 0.4;
  input.current_delta = 0.0;
  input.path_source_revision = 11U;
  input.tube_revision = 12U;
  input.map_observation_sequence = 13U;
  input.map_observation_is_snapshot = true;
  input.cloud_snapshot_resolution = 0.10;
  input.path_state_query = [](const double w,
                              phase_offset_core::PathDifferentialState& state) {
    state = MakeStraightState(w);
    return true;
  };
  input.cloud_clearance_query =
      [](const Eigen::Vector3d&, const double) {
        phase_offset_navigation::ClearanceQueryResult result;
        result.status = DistanceStatus::KNOWN_FREE;
        result.clearance = 0.39;
        result.clearance_certified = true;
        return result;
      };

  phase_offset_navigation::CertifiedTubeBuildResult result;
  ASSERT_TRUE(builder.build(input, result));
  ASSERT_TRUE(result.complete);
  ASSERT_TRUE(result.profile.raw_complete);
  ASSERT_TRUE(result.profile.filtered_complete);
  EXPECT_EQ(result.profile.classification,
            phase_offset_navigation::TubeProfileClassification::
                ZERO_ONLY_PLANNER_BASELINE);
  EXPECT_TRUE(result.profile.zero_only);
  ASSERT_FALSE(result.profile.samples.empty());
  EXPECT_EQ(result.profile.samples.front().cross_section_reason,
            phase_offset_navigation::TubeCrossSectionReason::
                EMPTY_AFTER_OBSTACLE_BOUNDS);
  EXPECT_DOUBLE_EQ(result.profile.samples.front().filtered_lower, 0.0);
  EXPECT_DOUBLE_EQ(result.profile.samples.front().filtered_upper, 0.0);

  // The adapter lifecycle remains neutral for a zero-only candidate; the
  // below-clearance evidence cannot manufacture a nonzero owner/bootstrap.
  PhaseOffsetMatchedAdapter adapter(config);
  std::shared_ptr<TubeEpochSnapshot> epoch(new TubeEpochSnapshot());
  epoch->active = true;
  epoch->active_profile = std::make_shared<const TubeProfile>(result.profile);
  epoch->epoch_status.active_available = true;
  epoch->epoch_status.active_classification = result.profile.classification;
  std::atomic_store(&adapter.latest_epoch_snapshot_,
                    std::shared_ptr<const TubeEpochSnapshot>(epoch));
  adapter.zero_gate_open_ = true;
  EXPECT_FALSE(adapter.requiresPathTubePairBootstrap());
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), 0.0);
}

TEST(PhaseOffsetMatchedAdapterTest,
     ZeroOnlyEpochNeverRequestsOffsetBootstrapAuthority) {
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  ASSERT_TRUE(adapter.runtime_);
  adapter.zero_gate_open_ = true;
  std::shared_ptr<TubeEpochSnapshot> epoch(new TubeEpochSnapshot());
  epoch->active = true;
  std::shared_ptr<phase_offset_navigation::TubeProfile> profile(
      new phase_offset_navigation::TubeProfile());
  profile->classification =
      phase_offset_navigation::TubeProfileClassification::
          ZERO_ONLY_PLANNER_BASELINE;
  profile->complete = true;
  epoch->active_profile =
      std::shared_ptr<const phase_offset_navigation::TubeProfile>(profile);
  epoch->epoch_status.active_available = true;
  epoch->epoch_status.active_classification = profile->classification;
  std::atomic_store(&adapter.latest_epoch_snapshot_,
      std::shared_ptr<const TubeEpochSnapshot>(epoch));

  EXPECT_FALSE(adapter.requiresPathTubePairBootstrap());
  EXPECT_FALSE(adapter.hasPendingOffsetActivationPair(
      std::shared_ptr<const PathTubePair>()));
}

TEST(PhaseOffsetMatchedAdapterTest,
     CompletedManualProfileWithZeroDeltaReturnsToNeutralPlannerAuthority) {
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  ASSERT_TRUE(adapter.runtime_);
  // Once the profile has actually started it owns path--tube handoffs even at
  // an instantaneous zero crossing.
  adapter.runtime_->profile_started_ = true;
  adapter.runtime_->profile_completed_ = false;
  adapter.runtime_->returning_to_center_ = false;
  adapter.runtime_->delta_ = 0.0;
  EXPECT_TRUE(adapter.requiresAuthoritativeOffsetHandoff());
  EXPECT_FALSE(adapter.requiresPathTubePairBootstrap());

  // Model the lifecycle state produced by Runtime after its one-shot profile
  // has completed and the matched port has recentered.  The configured
  // amplitude remains nonzero, but it is no longer an authority requirement.
  adapter.runtime_->profile_completed_ = true;
  adapter.runtime_->returning_to_center_ = false;
  adapter.runtime_->delta_ = 0.0;
  EXPECT_FALSE(adapter.requiresAuthoritativeOffsetHandoff());

  // A nonzero retained offset always keeps the joint PathTubePair contract,
  // even if the profile lifecycle is already marked complete.
  adapter.runtime_->delta_ = 0.02;
  EXPECT_TRUE(adapter.requiresAuthoritativeOffsetHandoff());
  EXPECT_FALSE(adapter.requiresPathTubePairBootstrap());
}

TEST(PhaseOffsetMatchedAdapterTest,
     RetiredExecutedOffsetCannotReenterNullPairBootstrap) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  ASSERT_TRUE(adapter.runtime_);
  adapter.zero_gate_open_ = true;
  adapter.runtime_->profile_started_ = true;
  adapter.runtime_->profile_completed_ = false;
  adapter.runtime_->delta_ = 0.02;

  EXPECT_TRUE(adapter.requiresAuthoritativeOffsetHandoff());
  EXPECT_FALSE(adapter.requiresPathTubePairBootstrap());
  const std::uint64_t active_session =
      adapter.authority_session_.load(std::memory_order_acquire);
  std::uint64_t conditionally_retired_session = 0U;
  EXPECT_FALSE(adapter.retirePathTubeAuthorityIfNeutral(
      active_session + 1U, conditionally_retired_session));
  EXPECT_EQ(0U, conditionally_retired_session);
  EXPECT_EQ(active_session,
            adapter.authority_session_.load(std::memory_order_acquire));
  const std::uint64_t retired_session =
      adapter.retirePathTubeAuthority(
          active_session + 1U);
  EXPECT_EQ(retired_session,
            adapter.authority_session_.load(std::memory_order_acquire));
  EXPECT_FALSE(adapter.capturePathTubePair());
  EXPECT_DOUBLE_EQ(0.02, adapter.runtime_->retainedDelta());
  EXPECT_TRUE(adapter.requiresAuthoritativeOffsetHandoff());
  EXPECT_FALSE(adapter.requiresPathTubePairBootstrap());

  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  ASSERT_TRUE(owner);
  MatchedAdapterInput input = MakeInput(path, owner.get(), 4.0);
  input.semantic_path_owner = owner;
  input.semantic_path_start_w = owner->startW();
  input.semantic_path_end_w = owner->endW();
  PathTubePairTransaction transaction;
  EXPECT_FALSE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples,
      path.current.w, 1.0, 2.4, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), transaction,
      retired_session));

  // An active profile's instantaneous zero crossing is still executed offset
  // authority and must be rejected by the same defensive staging boundary.
  adapter.runtime_->delta_ = 0.0;
  EXPECT_TRUE(adapter.requiresAuthoritativeOffsetHandoff());
  EXPECT_FALSE(adapter.requiresPathTubePairBootstrap());
  EXPECT_FALSE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples,
      path.current.w, 1.0, 2.4, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), transaction,
      retired_session));
}

TEST(PhaseOffsetMatchedAdapterTest,
     PendingActivationPairCannotBeNeutralRetiredBeforeFirstCommand) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  ASSERT_FALSE(adapter.update(MakeInput(path, &path, 0.0), output));
  ASSERT_TRUE(adapter.timerTick());
  for (int cycle = 1; cycle < 100; ++cycle) {
    EXPECT_FALSE(adapter.update(MakeInput(path, &path, cycle * kDt), output));
  }
  ASSERT_TRUE(adapter.requiresPathTubePairBootstrap());

  const std::shared_ptr<const ContinuousPhasePath> owner =
      MakeSyntheticOwner();
  ASSERT_TRUE(owner);
  MatchedAdapterInput input = MakeInput(path, owner.get(), 2.0);
  input.semantic_path_owner = owner;
  input.semantic_path_start_w = owner->startW();
  input.semantic_path_end_w = owner->endW();
  PathTubePairTransaction transaction;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples,
      path.current.w, 1.0, 2.4, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), transaction));
  std::shared_ptr<const PathTubePair> pair;
  ASSERT_TRUE(PrepareAndFinalizePair(
      adapter, transaction, path.current.w, input.position, input.gains,
      input.dt, std::shared_ptr<const plan_env::CloudOccupancySnapshot>(),
      pair));
  ASSERT_TRUE(pair);
  ASSERT_TRUE(adapter.hasPendingOffsetActivationPair(pair));
  const std::uint64_t session_before =
      adapter.authority_session_.load(std::memory_order_acquire);

  std::uint64_t retired_session = 0U;
  EXPECT_FALSE(adapter.retirePathTubeAuthorityIfNeutral(
      session_before + 1U, retired_session));
  EXPECT_EQ(0U, retired_session);
  EXPECT_EQ(session_before,
            adapter.authority_session_.load(std::memory_order_acquire));
  EXPECT_EQ(pair, adapter.capturePathTubePair());
  EXPECT_EQ(owner, pair->path_owner);
  EXPECT_TRUE(adapter.hasPendingOffsetActivationPair(pair));

  input.path_tube_pair = pair;
  RebaseInputToAuthority(adapter, input);
  EXPECT_TRUE(adapter.update(input, output)) << output.invalid_reason;
  EXPECT_TRUE(output.selected);
  EXPECT_TRUE(adapter.runtime_->hasExecutedOffsetAuthority());
  EXPECT_TRUE(adapter.requiresAuthoritativeOffsetHandoff());

  // The activation command begins the smooth profile at zero.  Model the
  // later continuous-profile command after elapsed Runtime time; the
  // selected projection must carry a significant post-commit retained delta.
  adapter.runtime_->profile_elapsed_ = 0.20;
  input.stamp = ros::Time(2.02);
  RebaseInputToAuthority(adapter, input);
  ASSERT_TRUE(adapter.update(input, output)) << output.invalid_reason;
  ASSERT_TRUE(output.selected);
  ASSERT_TRUE(output.projection.valid);
  EXPECT_GT(std::abs(output.projection.next_delta), 1e-6);
}

TEST(PhaseOffsetMatchedAdapterTest,
     RequestRecenterKeepsPairAndOwnerUntilFiniteNeutralHandoff) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  // Prime the fixed-Tube sidecar once before the warm-up gate.  The helper's
  // command-boundary bootstrap then consumes this immutable candidate epoch.
  ASSERT_FALSE(adapter.update(MakeInput(path, &path, 0.0), output));
  ASSERT_TRUE(adapter.timerTick());
  OpenGate(adapter, path, output);
  ASSERT_TRUE(output.selected) << output.invalid_reason;
  ASSERT_TRUE(adapter.runtime_);
  ASSERT_TRUE(adapter.runtime_->hasExecutedOffsetAuthority());
  std::shared_ptr<const PathTubePair> pair = adapter.capturePathTubePair();
  ASSERT_TRUE(pair);
  const std::uint64_t session =
      adapter.authority_session_.load(std::memory_order_acquire);
  EXPECT_EQ(pair->authority_session, session);

  // Build the same command-owned input used by the activation edge.  Keep
  // the immutable pair in every tick so the test observes the production
  // authority path rather than a sidecar/profile-only shortcut.
  MatchedAdapterInput input = MakeInput(path, pair->path_owner.get(), 2.02);
  input.semantic_path_owner = pair->path_owner;
  input.frame_owner = pair->frame_owner;
  input.semantic_path_start_w = pair->path_owner->startW();
  input.semantic_path_end_w = pair->path_owner->endW();
  input.path_tube_pair = pair;
  // Bind the command input's immutable frame fields to the exact frame owner
  // carried by the pair.  This is the production frame-bound contract used by
  // the authority query once delta is nonzero.
  ContinuousPhasePathState framed_path;
  ASSERT_TRUE(pair->frame_owner);
  ASSERT_TRUE(pair->frame_owner->evaluatePathState(input.path.w, framed_path));
  input.path.T = framed_path.T;
  input.path.N = framed_path.N;
  input.path.N_w = framed_path.N_w;
  input.path.path_revision = framed_path.path_revision;
  input.path.frame_revision = framed_path.frame_revision;
  input.path.frame_valid = framed_path.frame_valid;
  input.path.frame_provenance = framed_path.frame_provenance;

  // Advance a few exact committed ticks until the smooth profile has a
  // measurable nonzero executed delta.  The pair and authority remain the
  // same throughout this precondition phase.
  // The activation edge itself starts at the profile's zero crossing.  As in
  // the existing production handoff fixture, advance the immutable profile
  // clock to a post-crossing instant before the first nonzero command.
  adapter.runtime_->profile_elapsed_ = 0.20;
  bool nonzero_seen = std::abs(adapter.runtime_->retainedDelta()) > 1e-3;
  for (int tick = 0; tick < 40 && !nonzero_seen; ++tick) {
    input.stamp = ros::Time(2.02 + static_cast<double>(tick) * kDt);
    RebaseInputToAuthority(adapter, input);
    ASSERT_TRUE(adapter.update(input, output)) << output.invalid_reason;
    ASSERT_TRUE(output.selected);
    EXPECT_EQ(adapter.capturePathTubePair(), pair);
    nonzero_seen = std::abs(adapter.runtime_->retainedDelta()) > 1e-3;
  }
  ASSERT_TRUE(nonzero_seen);
  const double nonzero_delta = adapter.runtime_->retainedDelta();
  ASSERT_GT(std::abs(nonzero_delta), 1e-3);
  EXPECT_TRUE(adapter.requiresAuthoritativeOffsetHandoff());

  ASSERT_TRUE(adapter.requestRecenter());
  EXPECT_TRUE(adapter.recenterRequested());
  EXPECT_TRUE(adapter.requiresAuthoritativeOffsetHandoff());
  EXPECT_EQ(adapter.capturePathTubePair(), pair);

  // Recenter is a continuous in-owner operation.  Before the exact accepted
  // neutral step, Runtime and the pair remain authoritative and no planner
  // centerline/path-only handoff is allowed.
  bool reached_neutral = false;
  for (int tick = 0; tick < 80; ++tick) {
    input.stamp = ros::Time(3.0 + static_cast<double>(tick) * kDt);
    RebaseInputToAuthority(adapter, input);
    const double before_delta = adapter.runtime_->retainedDelta();
    ASSERT_TRUE(adapter.update(input, output)) << output.invalid_reason;
    ASSERT_TRUE(output.selected);
    EXPECT_TRUE(output.valid);
    EXPECT_EQ(adapter.capturePathTubePair(), pair);
    if (std::abs(adapter.runtime_->retainedDelta()) > 1e-3) {
      EXPECT_TRUE(adapter.requiresAuthoritativeOffsetHandoff());
      EXPECT_TRUE(adapter.recenterRequested());
      EXPECT_GT(std::abs(before_delta), 1e-3);
    } else {
      reached_neutral = true;
      break;
    }
  }
  ASSERT_TRUE(reached_neutral);
  EXPECT_LE(std::abs(adapter.runtime_->retainedDelta()), 1e-3);
  EXPECT_FALSE(adapter.recenterRequested());
  EXPECT_FALSE(adapter.requiresAuthoritativeOffsetHandoff());
  EXPECT_EQ(adapter.capturePathTubePair(), pair);

  // Only after the exact neutral command may the atomic planner-only
  // retirement edge clear the pair and reset the execution authority.
  std::uint64_t retired_session = 0U;
  ASSERT_TRUE(adapter.retirePathTubeAuthorityIfNeutral(
      session, retired_session));
  EXPECT_EQ(retired_session,
            adapter.authority_session_.load(std::memory_order_acquire));
  EXPECT_FALSE(adapter.capturePathTubePair());
  EXPECT_FALSE(adapter.requiresAuthoritativeOffsetHandoff());
  EXPECT_FALSE(adapter.recenterRequested());
}

TEST(PhaseOffsetMatchedAdapterTest,
     SelectedOutputCommitsRuntimeAndPublishesImmediately) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig());
  MatchedAdapterOutput output;
  OpenGate(adapter, path, output);
  ASSERT_TRUE(output.selected) << output.invalid_reason;
  ASSERT_TRUE(output.valid);
  ASSERT_TRUE(adapter.runtime_);
  EXPECT_DOUBLE_EQ(output.projection.next_delta, adapter.runtime_->retainedDelta());
  const std::shared_ptr<const ControlPublishSnapshot> published =
      std::atomic_load(&adapter.latest_control_snapshot_);
  ASSERT_TRUE(published);
  EXPECT_TRUE(published->output.selected);
  EXPECT_TRUE(published->output.valid);
  ASSERT_TRUE(published->authority_snapshot);
  const phase_offset_navigation::ActiveReferenceSnapshot authority_snapshot =
      adapter.execution_authority_.snapshot();
  ASSERT_TRUE(authority_snapshot.valid);
  EXPECT_EQ(authority_snapshot.snapshotId(),
            published->authority_snapshot->snapshotId());
  EXPECT_DOUBLE_EQ(authority_snapshot.selected_u.u_w,
                   output.projection.final_port.u_w);
  EXPECT_DOUBLE_EQ(authority_snapshot.selected_u.u_delta,
                   output.projection.final_port.u_delta);
  EXPECT_DOUBLE_EQ(authority_snapshot.proposed_next_delta,
                   adapter.runtime_->retainedDelta());
  EXPECT_DOUBLE_EQ(published->output.diagnostics[kSelectedManual], 1.0);
  EXPECT_DOUBLE_EQ(published->output.diagnostics[kManualValid], 1.0);
}

TEST(PhaseOffsetMatchedAdapterTest,
     SelectedFixedSnapshotExposesCertifiedMarker) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  adapter.update(MakeInput(path, &path, 0.0), output);
  ASSERT_TRUE(adapter.timerTick());
  OpenGate(adapter, path, output);
  ASSERT_TRUE(output.selected);
  ASSERT_TRUE(output.valid);
  std::shared_ptr<const ControlPublishSnapshot> published =
      std::atomic_load(&adapter.latest_control_snapshot_);
  ASSERT_TRUE(published);
  EXPECT_TRUE(published->output.selected);
  EXPECT_TRUE(published->output.valid);
  EXPECT_DOUBLE_EQ(published->output.diagnostics[kSelectedManual], 1.0);
  EXPECT_DOUBLE_EQ(published->output.diagnostics[kManualValid], 1.0);
  MatchedAdapterMarkerBundle markers;
  ASSERT_TRUE(adapter.buildMarkers(*published, markers));
  ExpectActions(markers.tube, visualization_msgs::Marker::ADD);
}

TEST(PhaseOffsetMatchedAdapterTest,
     ExecutionAuthorityReferenceFailureLeavesRuntimeAndSnapshotUntouched) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig());
  MatchedAdapterOutput output;
  OpenGate(adapter, path, output);
  ASSERT_TRUE(output.selected) << output.invalid_reason;
  const double retained_before = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      adapter.runtime_->previousFinalPort();
  const phase_offset_navigation::ActiveReferenceSnapshot authority_before =
      adapter.execution_authority_.snapshot();
  ASSERT_TRUE(authority_before.valid);

  MatchedAdapterInput invalid_query = MakeInput(path, &path, 3.0);
  invalid_query.path_state_query =
      [](double, phase_offset_core::PathDifferentialState&) { return false; };
  EXPECT_FALSE(adapter.update(invalid_query, output));
  EXPECT_FALSE(output.selected);
  EXPECT_FALSE(output.valid);
  EXPECT_NE(output.invalid_reason.find("reference"), std::string::npos);
  EXPECT_DOUBLE_EQ(retained_before, adapter.runtime_->retainedDelta());
  EXPECT_DOUBLE_EQ(previous_before.u_w,
                   adapter.runtime_->previousFinalPort().u_w);
  EXPECT_DOUBLE_EQ(previous_before.u_delta,
                   adapter.runtime_->previousFinalPort().u_delta);
  const phase_offset_navigation::ActiveReferenceSnapshot authority_after =
      adapter.execution_authority_.snapshot();
  EXPECT_EQ(authority_before.snapshotId(), authority_after.snapshotId());
  EXPECT_DOUBLE_EQ(authority_before.proposed_next_delta,
                   authority_after.proposed_next_delta);
}

TEST(PhaseOffsetMatchedAdapterTest,
     ContinuousWitnessDoesNotCommitBeforeCycleOneHundred) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  const MatchedAdapterInput input = MakeInput(path, &path);
  const double retained_before = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      adapter.runtime_->previousFinalPort();

  ASSERT_FALSE(adapter.update(input, output));
  ASSERT_TRUE(adapter.timerTick());
  for (int cycle = 1; cycle < 99; ++cycle) {
    MatchedAdapterInput cycle_input = input;
    cycle_input.stamp = ros::Time(cycle * kDt);
    EXPECT_FALSE(adapter.update(cycle_input, output));
    EXPECT_FALSE(output.selected);
    EXPECT_TRUE(output.valid) << output.invalid_reason;
    const double retained = adapter.runtime_->retainedDelta();
    const phase_offset_core::PortCommand previous =
        adapter.runtime_->previousFinalPort();
    EXPECT_EQ(0, std::memcmp(&retained_before, &retained, sizeof(retained)));
    EXPECT_EQ(0, std::memcmp(&previous_before.u_w, &previous.u_w,
                             sizeof(previous.u_w)));
    EXPECT_EQ(0, std::memcmp(&previous_before.u_delta, &previous.u_delta,
                             sizeof(previous.u_delta)));
  }

  MatchedAdapterInput armed_input = input;
  armed_input.stamp = ros::Time(99.0 * kDt);
  EXPECT_FALSE(adapter.update(armed_input, output));
  EXPECT_FALSE(output.selected);
  EXPECT_TRUE(adapter.requiresPathTubePairBootstrap());
  EXPECT_TRUE(output.valid);
  EXPECT_TRUE(output.projection.valid);
  EXPECT_TRUE(output.matched.valid);
}

TEST(PhaseOffsetMatchedAdapterTest,
     UnavailablePathDoesNotAdvanceGateOrLatchAndRecovers) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig());
  MatchedAdapterOutput output;
  for (int cycle = 0; cycle < 99; ++cycle) {
    EXPECT_FALSE(adapter.update(MakeInput(path, &path, cycle * kDt), output));
  }
  ASSERT_FALSE(output.zero_gate_open);
  ASSERT_EQ(output.zero_gate_consecutive_count, 99);

  MatchedAdapterInput unavailable = MakeInput(path, &path, 99.0 * kDt);
  unavailable.path.valid = false;
  EXPECT_FALSE(adapter.update(unavailable, output));
  EXPECT_FALSE(output.zero_gate_open);
  EXPECT_EQ(output.zero_gate_consecutive_count, 99);
  EXPECT_FALSE(output.failure_latched);

  EXPECT_TRUE(adapter.update(MakeInput(path, &path, 100.0 * kDt), output));
  EXPECT_TRUE(output.zero_gate_open);
  EXPECT_EQ(output.zero_gate_consecutive_count, 100);
  EXPECT_FALSE(output.failure_latched);
}

TEST(PhaseOffsetMatchedAdapterTest,
     UnavailableLegacyDoesNotAdvanceGateOrLatchAndRecovers) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig());
  MatchedAdapterOutput output;
  for (int cycle = 0; cycle < 99; ++cycle) {
    EXPECT_FALSE(adapter.update(MakeInput(path, &path, cycle * kDt), output));
  }
  ASSERT_FALSE(output.zero_gate_open);
  ASSERT_EQ(output.zero_gate_consecutive_count, 99);

  MatchedAdapterInput unavailable = MakeInput(path, &path, 99.0 * kDt);
  unavailable.legacy.valid = false;
  EXPECT_FALSE(adapter.update(unavailable, output));
  EXPECT_FALSE(output.zero_gate_open);
  EXPECT_EQ(output.zero_gate_consecutive_count, 99);
  EXPECT_FALSE(output.failure_latched);

  EXPECT_TRUE(adapter.update(MakeInput(path, &path, 100.0 * kDt), output));
  EXPECT_TRUE(output.zero_gate_open);
  EXPECT_EQ(output.zero_gate_consecutive_count, 100);
  EXPECT_FALSE(output.failure_latched);
}

TEST(PhaseOffsetMatchedAdapterTest, NoneRetainsA4ManualRecurrenceAndNoTube) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig());
  MatchedAdapterOutput output;
  OpenGate(adapter, path, output);
  ASSERT_TRUE(output.selected);
  double expected_delta = output.delta;
  phase_offset_core::PortCommand previous = output.projection.final_port;
  for (int cycle = 0; cycle < 80; ++cycle) {
    expected_delta += kDt * previous.u_delta;
    MatchedAdapterInput input =
        MakeInput(path, &path, (100 + cycle) * kDt);
    RebaseInputToAuthority(adapter, input);
    adapter.update(input, output);
    EXPECT_NEAR(output.delta, expected_delta, 1e-12);
    previous = output.projection.final_port;
  }
  EXPECT_FALSE(output.candidate_profile);
  EXPECT_FALSE(output.active_profile);
  EXPECT_EQ(output.tube_epoch_status.state, TubeEpochState::NO_ACTIVE_TUBE);
  EXPECT_EQ(output.diagnostics.size(), kManualDiagnosticCount);
  EXPECT_EQ(output.diagnostics[kTubeDisplayCertified], 0.0);
}

TEST(PhaseOffsetMatchedAdapterTest, SourceChangeForcesImmediateCandidateAttempt) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput first;
  BuildAndConsume(adapter, MakeInput(path, &path), first);
  ASSERT_TRUE(first.tube_update_due_this_cycle);
  const int second_identity = 7;
  MatchedAdapterOutput second;
  BuildAndConsume(adapter, MakeInput(path, &second_identity, kDt), second);
  EXPECT_TRUE(second.tube_update_due_this_cycle);
  EXPECT_EQ(second.tube_epoch_status.candidate_sequence,
            first.tube_epoch_status.candidate_sequence + 1U);
  EXPECT_EQ(second.tube_epoch_status.active_tube_epoch,
            first.tube_epoch_status.active_tube_epoch + 1U);
}

TEST(PhaseOffsetMatchedAdapterTest, EquivalentRefreshPreservesActivePointerIdentity) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput first;
  BuildAndConsume(adapter, MakeInput(path, &path), first);
  const auto* active = first.active_profile.get();
  MatchedAdapterOutput refreshed;
  for (int cycle = 1; cycle <= 5; ++cycle) {
    adapter.update(MakeInput(path, &path, cycle * kDt), refreshed);
  }
  ASSERT_TRUE(adapter.timerTick());
  adapter.update(MakeInput(path, &path, 6.0 * kDt), refreshed);
  EXPECT_TRUE(refreshed.tube_update_due_this_cycle);
  EXPECT_EQ(refreshed.active_profile.get(), active);
  EXPECT_EQ(refreshed.tube_epoch_status.active_tube_epoch,
            first.tube_epoch_status.active_tube_epoch);
  EXPECT_EQ(refreshed.tube_epoch_status.disposition, TubeInstallDisposition::EQUIVALENT_REFRESH);
}

TEST(PhaseOffsetMatchedAdapterTest, MarkersReadCandidateAndActiveProfilesSeparately) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output = MakeMarkerOutput();
  MatchedAdapterMarkerBundle markers;
  ASSERT_TRUE(adapter.buildMarkers(MakeInput(path, &path), output, markers));
  ExpectActions(markers.tube_candidate, visualization_msgs::Marker::ADD);
  ExpectActions(markers.tube, visualization_msgs::Marker::ADD);
  ExpectActions(markers.tube_certified_geometry,
                visualization_msgs::Marker::DELETE);
  EXPECT_NEAR(markers.tube_candidate.markers[0].points.front().x, 10.0, 1e-12);
  EXPECT_NEAR(markers.tube.markers[0].points.front().x, 20.0, 1e-12);
}

TEST(PhaseOffsetMatchedAdapterCandidateProvenance,
     EpochCopiesBuildPhaseBitExactly) {
  SyntheticPath path = MakePath();
  const double build_w = std::nextafter(0.4, 1.0);
  path.current = MakeState(build_w);
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  const MatchedAdapterInput input = MakeInput(path, &path, 0.0);

  ASSERT_FALSE(adapter.update(input, output));
  ASSERT_TRUE(adapter.timerTick());
  const std::shared_ptr<const TubeEpochSnapshot> epoch =
      std::atomic_load(&adapter.latest_candidate_epoch_snapshot_);
  ASSERT_TRUE(epoch);
  EXPECT_EQ(0, std::memcmp(&epoch->candidate_build_w, &build_w,
                           sizeof(build_w)));
}

TEST(PhaseOffsetMatchedAdapterCandidateProvenance,
     DelayedCommandPhaseUsesSameBuildAnchorForCandidateMarkers) {
  SyntheticPath build_path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  const MatchedAdapterInput build_input = MakeInput(build_path, &build_path,
                                                    0.0);
  ASSERT_FALSE(adapter.update(build_input, output));
  ASSERT_TRUE(adapter.timerTick());
  const std::shared_ptr<const TubeEpochSnapshot> epoch =
      std::atomic_load(&adapter.latest_candidate_epoch_snapshot_);
  ASSERT_TRUE(epoch);
  ASSERT_TRUE(epoch->candidate_profile);
  ASSERT_TRUE(epoch->candidate_profile->raw_build_samples.size() >= 2U);
  EXPECT_EQ(0, std::memcmp(&epoch->candidate_build_w, &build_input.path.w,
                           sizeof(build_input.path.w)));

  SyntheticPath later_path = build_path;
  later_path.current = MakeState(1.35);
  const MatchedAdapterInput later_input = MakeInput(later_path, &build_path,
                                                    0.02);
  ASSERT_FALSE(adapter.update(later_input, output));
  const std::shared_ptr<const ControlPublishSnapshot> control =
      std::atomic_load(&adapter.latest_control_snapshot_);
  ASSERT_TRUE(control);
  ASSERT_EQ(control->epoch_snapshot.get(), epoch.get());
  EXPECT_DOUBLE_EQ(control->current_w, later_input.path.w);

  const MatchedAdapterOutput output_before_marker = control->output;
  const std::shared_ptr<const TubeProfile> candidate_profile_before =
      control->output.candidate_profile;
  const std::shared_ptr<const TubeProfile> active_profile_before =
      control->output.active_profile;
  const std::shared_ptr<const TubeEpochSnapshot> epoch_before_marker =
      control->epoch_snapshot;
  const std::shared_ptr<const MatchedAdapterPathSamples> samples_before_marker =
      control->full_path_samples;
  const phase_offset_navigation::ActiveReferenceSnapshot authority_before =
      adapter.execution_authority_.snapshot();
  const std::shared_ptr<const phase_offset_navigation::ActiveReferenceSnapshot>
      authority_pointer_before = adapter.execution_authority_.snapshotPtr();
  const phase_offset_core::PortCommand previous_port_before =
      adapter.runtime_->previousFinalPort();
  const double retained_delta_before = adapter.runtime_->retainedDelta();
  const phase_offset_navigation::PhaseOffsetRuntime* runtime_pointer_before =
      adapter.runtime_.get();

  // Production publishManual() selects one immutable epoch and calls this
  // const marker-construction overload.  It receives no mutable planner or
  // governor handle; the adapter-boundary assertions below document the
  // visualization-only noninterference contract without adding a hook/state.

  MatchedAdapterMarkerBundle same_build;
  ASSERT_TRUE(adapter.buildMarkers(*control, epoch, same_build));
  ExpectActions(same_build.tube_candidate, visualization_msgs::Marker::ADD);
  for (const visualization_msgs::Marker& marker : same_build.tube_candidate.markers) {
    EXPECT_FALSE(marker.points.empty());
  }

  const auto& raw_samples = epoch->candidate_profile->raw_build_samples;
  std::size_t anchor_index = raw_samples.size();
  for (std::size_t index = 0U; index < raw_samples.size(); ++index) {
    if (std::memcmp(&raw_samples[index].w, &epoch->candidate_build_w,
                    sizeof(epoch->candidate_build_w)) == 0) {
      anchor_index = index;
      break;
    }
  }
  ASSERT_LT(anchor_index, raw_samples.size());
  const TubeRawSample& anchor_sample = raw_samples[anchor_index];
  const Eigen::Vector3d expected_lower =
      anchor_sample.p + anchor_sample.N * anchor_sample.filtered_lower;
  const Eigen::Vector3d expected_upper =
      anchor_sample.p + anchor_sample.N * anchor_sample.filtered_upper;
  const auto point_matches_bits = [](const geometry_msgs::Point& actual,
                                     const Eigen::Vector3d& expected) {
    return std::memcmp(&actual.x, &expected[0], sizeof(double)) == 0 &&
        std::memcmp(&actual.y, &expected[1], sizeof(double)) == 0 &&
        std::memcmp(&actual.z, &expected[2], sizeof(double)) == 0;
  };
  bool found_anchor_lower = false;
  bool found_anchor_upper = false;
  for (const geometry_msgs::Point& point :
       same_build.tube_candidate.markers[0].points) {
    found_anchor_lower = found_anchor_lower ||
        point_matches_bits(point, expected_lower);
  }
  for (const geometry_msgs::Point& point :
       same_build.tube_candidate.markers[1].points) {
    found_anchor_upper = found_anchor_upper ||
        point_matches_bits(point, expected_upper);
  }
  EXPECT_TRUE(found_anchor_lower);
  EXPECT_TRUE(found_anchor_upper);

  // The compatibility path intentionally models the old command-phase
  // helper; with the later phase it must fail exact-anchor selection.
  MatchedAdapterMarkerBundle later_phase;
  ASSERT_TRUE(adapter.buildMarkers(*control, later_phase));
  ExpectActions(later_phase.tube_candidate,
                visualization_msgs::Marker::DELETE);

  EXPECT_EQ(control->epoch_snapshot.get(), epoch_before_marker.get());
  EXPECT_EQ(control->full_path_samples.get(), samples_before_marker.get());
  EXPECT_EQ(control->output.candidate_profile.get(),
            candidate_profile_before.get());
  EXPECT_EQ(control->output.active_profile.get(), active_profile_before.get());
  EXPECT_EQ(control->output.selected, output_before_marker.selected);
  EXPECT_EQ(control->output.valid, output_before_marker.valid);
  EXPECT_DOUBLE_EQ(control->output.delta, output_before_marker.delta);
  EXPECT_DOUBLE_EQ(control->output.delta_ref, output_before_marker.delta_ref);
  EXPECT_EQ(control->output.tube_epoch_status.candidate_sequence,
            output_before_marker.tube_epoch_status.candidate_sequence);
  EXPECT_EQ(control->output.tube_epoch_status.active_tube_epoch,
            output_before_marker.tube_epoch_status.active_tube_epoch);
  EXPECT_TRUE((control->output.guidance.v_cmd -
               output_before_marker.guidance.v_cmd).isZero());
  EXPECT_DOUBLE_EQ(control->output.guidance.w_dot,
                   output_before_marker.guidance.w_dot);
  EXPECT_TRUE((control->output.base_guidance.v_cmd -
               output_before_marker.base_guidance.v_cmd).isZero());
  EXPECT_DOUBLE_EQ(control->output.base_guidance.w_dot,
                   output_before_marker.base_guidance.w_dot);
  EXPECT_TRUE((control->output.matched.v_cmd -
               output_before_marker.matched.v_cmd).isZero());
  EXPECT_DOUBLE_EQ(control->output.projection.final_port.u_w,
                   output_before_marker.projection.final_port.u_w);
  EXPECT_DOUBLE_EQ(control->output.projection.final_port.u_delta,
                   output_before_marker.projection.final_port.u_delta);
  EXPECT_EQ(adapter.runtime_.get(), runtime_pointer_before);
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), retained_delta_before);
  const phase_offset_core::PortCommand previous_port_after =
      adapter.runtime_->previousFinalPort();
  EXPECT_EQ(0, std::memcmp(&previous_port_after.u_w,
                           &previous_port_before.u_w,
                           sizeof(previous_port_before.u_w)));
  EXPECT_EQ(0, std::memcmp(&previous_port_after.u_delta,
                           &previous_port_before.u_delta,
                           sizeof(previous_port_before.u_delta)));
  EXPECT_EQ(adapter.execution_authority_.snapshotPtr().get(),
            authority_pointer_before.get());
  const phase_offset_navigation::ActiveReferenceSnapshot authority_after =
      adapter.execution_authority_.snapshot();
  EXPECT_EQ(authority_after.snapshotId(), authority_before.snapshotId());
  EXPECT_EQ(authority_after.sequence, authority_before.sequence);
  EXPECT_EQ(authority_after.owner_mode, authority_before.owner_mode);
  EXPECT_DOUBLE_EQ(authority_after.delta, authority_before.delta);
}

TEST(PhaseOffsetMatchedAdapterCandidateProvenance,
     ExactMismatchFailsClosedWithoutTolerance) {
  PairPublicationFixture fixture = MakePairPublicationFixture();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  std::shared_ptr<TubeEpochSnapshot> mismatched(
      new TubeEpochSnapshot(*fixture.timer_candidate_epoch));
  mismatched->candidate_build_w = std::nextafter(0.4, 1.0);
  MatchedAdapterMarkerBundle markers;
  ASSERT_TRUE(adapter.buildMarkers(
      fixture.control, std::shared_ptr<const TubeEpochSnapshot>(mismatched),
      markers));
  ExpectActions(markers.tube_candidate, visualization_msgs::Marker::DELETE);
}

TEST(PhaseOffsetMatchedAdapterR3,
     CurrentEsdfOffsetCertifiedCandidateProducesExactlyThreeAdds) {
  const R3CertifiedGeometryFixture fixture =
      MakeR3CertifiedGeometryFixture();
  EXPECT_EQ(fixture.owner->pathRevision(), 0U);
  EXPECT_EQ(fixture.frame->pathRevision(), fixture.request->source_revision);
  EXPECT_EQ(fixture.candidate->candidate_profile->path_revision,
            fixture.request->source_revision);
  PhaseOffsetMatchedAdapter adapter(fixture.config);
  adapter.task_generation_.store(3U, std::memory_order_release);
  std::atomic_store(&adapter.latest_build_request_, fixture.request);
  std::atomic_store(&adapter.latest_candidate_epoch_snapshot_,
                    fixture.candidate);

  EXPECT_TRUE(PhaseOffsetMatchedAdapter::certifiedGeometryCandidateEligible(
      *fixture.request, *fixture.candidate, 3U));
  const auto markers = adapter.certifiedGeometryMarkers(ros::Time(1.0));
  ExpectActions(markers, visualization_msgs::Marker::ADD);
  for (const auto& marker : markers.markers) {
    EXPECT_EQ(marker.ns, "phase_offset_manual_tube_certified_geometry");
  }
  EXPECT_EQ(markers.markers[0].type, visualization_msgs::Marker::LINE_STRIP);
  EXPECT_EQ(markers.markers[1].type, visualization_msgs::Marker::LINE_STRIP);
  EXPECT_EQ(markers.markers[2].type,
            visualization_msgs::Marker::TRIANGLE_LIST);
  EXPECT_EQ(markers.markers[2].points.size(), 12U);
}

TEST(PhaseOffsetMatchedAdapterR3,
     SemanticOwnerDefaultRevisionDoesNotMaskFrameProvenance) {
  const R3CertifiedGeometryFixture fixture =
      MakeR3CertifiedGeometryFixture();
  ASSERT_EQ(fixture.owner->pathRevision(), 0U);
  ASSERT_EQ(fixture.frame->pathRevision(), fixture.request->source_revision);
  ASSERT_EQ(fixture.frame->frameRevision(),
            fixture.candidate->candidate_profile->frame_revision);
  EXPECT_TRUE(PhaseOffsetMatchedAdapter::certifiedGeometryCandidateEligible(
      *fixture.request, *fixture.candidate, fixture.request->task_generation));

  std::shared_ptr<TubeBuildRequest> mismatch(
      new TubeBuildRequest(*fixture.request));
  mismatch->frame_owner = std::make_shared<const ContinuousPhaseNormalFrame>(
      fixture.owner, fixture.request->source_revision + 1U,
      fixture.frame->frameRevision());
  const std::shared_ptr<const TubeBuildRequest> mismatch_const(mismatch);
  EXPECT_FALSE(PhaseOffsetMatchedAdapter::certifiedGeometryCandidateEligible(
      *mismatch_const, *fixture.candidate,
      fixture.request->task_generation));
}

TEST(PhaseOffsetMatchedAdapterR3,
     CandidateMapProvenanceMismatchRejectsWithUnversionedSemanticOwner) {
  const R3CertifiedGeometryFixture fixture =
      MakeR3CertifiedGeometryFixture();
  ASSERT_EQ(fixture.owner->pathRevision(), 0U);
  std::shared_ptr<TubeProfile> mismatched_profile(
      new TubeProfile(*fixture.candidate->candidate_profile));
  ++mismatched_profile->map_revision;
  std::shared_ptr<TubeEpochSnapshot> mismatched_candidate(
      new TubeEpochSnapshot(*fixture.candidate));
  mismatched_candidate->candidate_profile =
      std::shared_ptr<const TubeProfile>(mismatched_profile);
  EXPECT_FALSE(PhaseOffsetMatchedAdapter::certifiedGeometryCandidateEligible(
      *fixture.request, *mismatched_candidate,
      fixture.request->task_generation));
}

TEST(PhaseOffsetMatchedAdapterR3,
     PredicateProvenanceAndCertificationMismatchesDelete) {
  const R3CertifiedGeometryFixture fixture =
      MakeR3CertifiedGeometryFixture();
  using Mismatch = std::function<void(TubeBuildRequest&, TubeEpochSnapshot&,
                                      TubeProfile&)>;
  const std::vector<std::pair<const char*, Mismatch>> cases = {
      {"task", [](TubeBuildRequest& request, TubeEpochSnapshot&, TubeProfile&) {
         request.task_generation = 4U;
       }},
      {"source", [](TubeBuildRequest&, TubeEpochSnapshot& candidate,
                     TubeProfile&) { candidate.source_revision = 8U; }},
      {"path", [](TubeBuildRequest&, TubeEpochSnapshot& candidate,
                   TubeProfile&) {
         candidate.epoch_status.candidate_path_source_revision = 8U;
       }},
      {"frame", [&fixture](TubeBuildRequest& request, TubeEpochSnapshot&,
                            TubeProfile&) {
         request.frame_owner =
             std::make_shared<const ContinuousPhaseNormalFrame>(
                 fixture.owner, fixture.request->source_revision,
                 fixture.frame->frameRevision() + 1U);
       }},
      {"map", [](TubeBuildRequest&, TubeEpochSnapshot& candidate,
                  TubeProfile&) { ++candidate.map_observation_sequence; }},
      {"profile_source", [](TubeBuildRequest&, TubeEpochSnapshot&,
                             TubeProfile& profile) {
         profile.source = TubeSource::FIXED;
       }},
      {"profile_path", [](TubeBuildRequest&, TubeEpochSnapshot&,
                           TubeProfile& profile) {
         profile.path_revision = 8U;
       }},
      {"tube", [](TubeBuildRequest&, TubeEpochSnapshot&, TubeProfile& profile) {
         ++profile.tube_revision;
       }},
      {"profile_classification", [](TubeBuildRequest&, TubeEpochSnapshot&,
                                     TubeProfile& profile) {
         profile.classification =
             phase_offset_navigation::TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE;
       }},
      {"status_classification", [](TubeBuildRequest&,
                                    TubeEpochSnapshot& candidate, TubeProfile&) {
         candidate.epoch_status.candidate_classification =
             phase_offset_navigation::TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE;
       }},
      {"profile_completion", [](TubeBuildRequest&, TubeEpochSnapshot&,
                                 TubeProfile& profile) {
         profile.complete = false;
       }},
      {"status_completion", [](TubeBuildRequest&,
                                TubeEpochSnapshot& candidate, TubeProfile&) {
         candidate.epoch_status.candidate_complete = false;
       }},
  };

  for (const auto& test_case : cases) {
    SCOPED_TRACE(test_case.first);
    std::shared_ptr<TubeBuildRequest> request(
        new TubeBuildRequest(*fixture.request));
    std::shared_ptr<TubeEpochSnapshot> candidate(
        new TubeEpochSnapshot(*fixture.candidate));
    std::shared_ptr<TubeProfile> profile(
        new TubeProfile(*fixture.candidate->candidate_profile));
    candidate->candidate_profile = std::shared_ptr<const TubeProfile>(profile);
    test_case.second(*request, *candidate, *profile);
    const std::shared_ptr<const TubeBuildRequest> request_const(request);
    const std::shared_ptr<const TubeEpochSnapshot> candidate_const(candidate);
    EXPECT_FALSE(PhaseOffsetMatchedAdapter::certifiedGeometryCandidateEligible(
        *request_const, *candidate_const, 3U));

    PhaseOffsetMatchedAdapter adapter(fixture.config);
    adapter.task_generation_.store(3U, std::memory_order_release);
    std::atomic_store(&adapter.latest_build_request_, request_const);
    std::atomic_store(&adapter.latest_candidate_epoch_snapshot_, candidate_const);
    ExpectActions(adapter.certifiedGeometryMarkers(ros::Time(1.0)),
                  visualization_msgs::Marker::DELETE);
  }
}

TEST(PhaseOffsetMatchedAdapterR3,
     CertifiedGeometryUsesFilteredSamplesAndIgnoresRawBuildWidth) {
  R3CertifiedGeometryFixture fixture = MakeR3CertifiedGeometryFixture();
  std::shared_ptr<TubeProfile> profile(
      new TubeProfile(*fixture.candidate->candidate_profile));
  for (const auto& filtered : profile->samples) {
    TubeRawSample raw = filtered;
    raw.raw_lower = -3.0;
    raw.raw_upper = 3.0;
    raw.filtered_lower = -3.0;
    raw.filtered_upper = 3.0;
    profile->raw_build_samples.push_back(raw);
  }
  std::shared_ptr<TubeEpochSnapshot> candidate(
      new TubeEpochSnapshot(*fixture.candidate));
  candidate->candidate_profile = std::shared_ptr<const TubeProfile>(profile);
  fixture.candidate = std::shared_ptr<const TubeEpochSnapshot>(candidate);
  PhaseOffsetMatchedAdapter adapter(fixture.config);
  adapter.task_generation_.store(3U, std::memory_order_release);
  std::atomic_store(&adapter.latest_build_request_, fixture.request);
  std::atomic_store(&adapter.latest_candidate_epoch_snapshot_,
                    fixture.candidate);
  const auto markers = adapter.certifiedGeometryMarkers(ros::Time(1.0));
  ExpectActions(markers, visualization_msgs::Marker::ADD);
  EXPECT_DOUBLE_EQ(markers.markers[0].points.front().y, -0.12);
  EXPECT_DOUBLE_EQ(markers.markers[1].points.front().y, 0.12);
}

TEST(PhaseOffsetMatchedAdapterR3,
     ZeroOnlyNoneIncompleteNonfiniteUnorderedDuplicateAndShortDelete) {
  const R3CertifiedGeometryFixture fixture =
      MakeR3CertifiedGeometryFixture();
  const auto expect_delete = [&](const std::function<void(TubeProfile&)>& edit) {
    std::shared_ptr<TubeProfile> profile(
        new TubeProfile(*fixture.candidate->candidate_profile));
    edit(*profile);
    std::shared_ptr<TubeEpochSnapshot> candidate(
        new TubeEpochSnapshot(*fixture.candidate));
    candidate->candidate_profile = std::shared_ptr<const TubeProfile>(profile);
    EXPECT_FALSE(PhaseOffsetMatchedAdapter::certifiedGeometryCandidateEligible(
        *fixture.request, *candidate, 3U));
    PhaseOffsetMatchedAdapter adapter(fixture.config);
    adapter.task_generation_.store(3U, std::memory_order_release);
    std::atomic_store(&adapter.latest_build_request_, fixture.request);
    std::atomic_store(&adapter.latest_candidate_epoch_snapshot_,
                      std::shared_ptr<const TubeEpochSnapshot>(candidate));
    ExpectActions(adapter.certifiedGeometryMarkers(ros::Time(1.0)),
                  visualization_msgs::Marker::DELETE);
  };
  expect_delete([](TubeProfile& profile) {
    profile.classification =
        phase_offset_navigation::TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE;
    profile.zero_only = true;
  });
  expect_delete([](TubeProfile& profile) { profile.source = TubeSource::NONE; });
  expect_delete([](TubeProfile& profile) { profile.complete = false; });
  expect_delete([](TubeProfile& profile) {
    profile.samples[0].filtered_lower =
        std::numeric_limits<double>::quiet_NaN();
  });
  expect_delete([](TubeProfile& profile) {
    profile.samples[1].w = profile.samples[0].w - 0.1;
  });
  expect_delete([](TubeProfile& profile) {
    profile.samples[1].w = profile.samples[0].w;
  });
  expect_delete([](TubeProfile& profile) { profile.samples.resize(1U); });
}

TEST(PhaseOffsetMatchedAdapterR3,
     CandidateOnlyRuntimeDenialAndAuthorityAbsenceDoNotHideGeometry) {
  R3CertifiedGeometryFixture fixture = MakeR3CertifiedGeometryFixture();
  PhaseOffsetMatchedAdapter adapter(fixture.config);
  adapter.task_generation_.store(3U, std::memory_order_release);
  std::atomic_store(&adapter.latest_build_request_, fixture.request);
  std::shared_ptr<TubeEpochSnapshot> candidate(
      new TubeEpochSnapshot(*fixture.candidate));
  candidate->epoch_status.active_available = false;
  candidate->epoch_status.active_current_validation_valid = false;
  candidate->active_profile.reset();
  std::atomic_store(&adapter.latest_candidate_epoch_snapshot_,
                    std::shared_ptr<const TubeEpochSnapshot>(candidate));
  EXPECT_TRUE(PhaseOffsetMatchedAdapter::certifiedGeometryCandidateEligible(
      *fixture.request, *candidate, 3U));
  EXPECT_EQ(adapter.runtime_.get() != nullptr, true);
  const auto markers = adapter.certifiedGeometryMarkers(ros::Time(1.0));
  ExpectActions(markers, visualization_msgs::Marker::ADD);
}

TEST(PhaseOffsetMatchedAdapterR3,
     NewerRequestMapAlonePreservesCompletedSameSourceCandidate) {
  R3CertifiedGeometryFixture fixture = MakeR3CertifiedGeometryFixture();
  std::shared_ptr<TubeBuildRequest> newer(new TubeBuildRequest(*fixture.request));
  newer->map_observation_sequence = 1234U;
  const std::shared_ptr<const TubeBuildRequest> newer_const(newer);
  EXPECT_TRUE(PhaseOffsetMatchedAdapter::certifiedGeometryCandidateEligible(
      *newer_const, *fixture.candidate, 3U));
}

TEST(PhaseOffsetMatchedAdapterR3,
     NewerValidCompletionReplacesAndInvalidCompletionNeverFallsBack) {
  R3CertifiedGeometryFixture fixture = MakeR3CertifiedGeometryFixture();
  PhaseOffsetMatchedAdapter adapter(fixture.config);
  adapter.task_generation_.store(3U, std::memory_order_release);
  std::atomic_store(&adapter.latest_build_request_, fixture.request);
  std::atomic_store(&adapter.latest_candidate_epoch_snapshot_,
                    fixture.candidate);
  const auto old_markers = adapter.certifiedGeometryMarkers(ros::Time(1.0));
  ExpectActions(old_markers, visualization_msgs::Marker::ADD);

  std::shared_ptr<TubeProfile> replacement_profile(
      new TubeProfile(*fixture.candidate->candidate_profile));
  replacement_profile->samples[0].p.x() = 9.0;
  replacement_profile->map_revision = 42U;
  replacement_profile->snapshot_sequence = 42U;
  std::shared_ptr<TubeEpochSnapshot> replacement(
      new TubeEpochSnapshot(*fixture.candidate));
  replacement->map_observation_sequence = 42U;
  replacement->candidate_profile =
      std::shared_ptr<const TubeProfile>(replacement_profile);
  replacement->epoch_status.candidate_map_observation_sequence = 42U;
  std::atomic_store(&adapter.latest_candidate_epoch_snapshot_,
                    std::shared_ptr<const TubeEpochSnapshot>(replacement));
  const auto new_markers = adapter.certifiedGeometryMarkers(ros::Time(1.0));
  ExpectActions(new_markers, visualization_msgs::Marker::ADD);
  EXPECT_DOUBLE_EQ(new_markers.markers[0].points.front().x, 9.0);

  std::shared_ptr<TubeEpochSnapshot> invalid(
      new TubeEpochSnapshot(*replacement));
  std::shared_ptr<TubeProfile> invalid_profile(
      new TubeProfile(*replacement_profile));
  invalid_profile->classification =
      phase_offset_navigation::TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE;
  invalid_profile->zero_only = true;
  invalid->candidate_profile = std::shared_ptr<const TubeProfile>(invalid_profile);
  invalid->epoch_status.candidate_classification =
      phase_offset_navigation::TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE;
  invalid->epoch_status.candidate_zero_only = true;
  std::atomic_store(&adapter.latest_candidate_epoch_snapshot_,
                    std::shared_ptr<const TubeEpochSnapshot>(invalid));
  ExpectActions(adapter.certifiedGeometryMarkers(ros::Time(1.0)),
                visualization_msgs::Marker::DELETE);
}

TEST(PhaseOffsetMatchedAdapterR3,
     PublicationDecisionTraceDeletesOldSlotThenAddsMatchingReplacement) {
  R3CertifiedGeometryFixture fixture = MakeR3CertifiedGeometryFixture();
  PhaseOffsetMatchedAdapter adapter(fixture.config);
  adapter.task_generation_.store(3U, std::memory_order_release);
  std::atomic_store(&adapter.latest_build_request_, fixture.request);
  std::atomic_store(&adapter.latest_candidate_epoch_snapshot_,
                    fixture.candidate);
  ExpectActions(adapter.certifiedGeometryMarkers(ros::Time(1.0)),
                visualization_msgs::Marker::ADD);

  std::shared_ptr<TubeBuildRequest> replacement_request(
      new TubeBuildRequest(*fixture.request));
  replacement_request->source_revision = 8U;
  replacement_request->control_sequence = 9U;
  replacement_request->frame_owner =
      std::make_shared<const ContinuousPhaseNormalFrame>(
          fixture.owner, 8U, 9U);
  const std::shared_ptr<const TubeBuildRequest> replacement_request_const(
      replacement_request);
  std::atomic_store(&adapter.latest_build_request_, replacement_request_const);
  // The request is already linearized, but the Candidate slot still exposes
  // the old source revision.  R3 must not display that stale geometry.
  ExpectActions(adapter.certifiedGeometryMarkers(ros::Time(1.0)),
                visualization_msgs::Marker::DELETE);

  std::shared_ptr<TubeProfile> replacement_profile(
      new TubeProfile(*fixture.candidate->candidate_profile));
  replacement_profile->source_revision = 8U;
  replacement_profile->path_revision = 8U;
  replacement_profile->frame_revision = 9U;
  replacement_profile->samples[0].p.x() = 9.0;
  for (TubeRawSample& sample : replacement_profile->samples) {
    sample.path_revision = 8U;
    sample.frame_revision = 9U;
  }
  std::shared_ptr<TubeEpochSnapshot> replacement_candidate(
      new TubeEpochSnapshot(*fixture.candidate));
  replacement_candidate->source_revision = 8U;
  replacement_candidate->candidate_profile =
      std::shared_ptr<const TubeProfile>(replacement_profile);
  replacement_candidate->epoch_status.candidate_path_source_revision = 8U;
  const std::shared_ptr<const TubeEpochSnapshot> replacement_candidate_const(
      replacement_candidate);
  std::atomic_store(&adapter.latest_candidate_epoch_snapshot_,
                    replacement_candidate_const);
  const auto replacement_markers =
      adapter.certifiedGeometryMarkers(ros::Time(1.0));
  ExpectActions(replacement_markers, visualization_msgs::Marker::ADD);
  EXPECT_DOUBLE_EQ(replacement_markers.markers[0].points.front().x, 9.0);
}

TEST(PhaseOffsetMatchedAdapterR3,
     SourceReplacementFinalRequestIdentityRaceDeletesOldGeometry) {
  R3CertifiedGeometryFixture fixture = MakeR3CertifiedGeometryFixture();
  PhaseOffsetMatchedAdapter adapter(fixture.config);
  adapter.task_generation_.store(3U, std::memory_order_release);
  std::atomic_store(&adapter.latest_build_request_, fixture.request);
  std::atomic_store(&adapter.latest_candidate_epoch_snapshot_,
                    fixture.candidate);
  std::shared_ptr<TubeBuildRequest> replacement(
      new TubeBuildRequest(*fixture.request));
  replacement->source_revision = 8U;
  replacement->control_sequence = 9U;
  replacement->frame_owner = std::make_shared<const ContinuousPhaseNormalFrame>(
      fixture.owner, 8U, 9U);
  const std::shared_ptr<const TubeBuildRequest> replacement_const(replacement);
  adapter.certified_geometry_linearization_test_hook_ = [&]() {
    std::atomic_store(&adapter.latest_build_request_, replacement_const);
  };
  ExpectActions(adapter.certifiedGeometryMarkers(ros::Time(1.0)),
                visualization_msgs::Marker::DELETE);
  adapter.certified_geometry_linearization_test_hook_ = std::function<void()>();
}

TEST(PhaseOffsetMatchedAdapterCandidateProvenance,
     AuthoritativeCandidateSubstitutionCouplesProfileAndAnchor) {
  PairPublicationFixture fixture = MakePairPublicationFixture();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  fixture.control.current_w = 1.3;

  MatchedAdapterMarkerBundle authoritative;
  ASSERT_TRUE(adapter.buildMarkers(
      fixture.control, fixture.timer_candidate_epoch, authoritative));
  ExpectActions(authoritative.tube_candidate,
                visualization_msgs::Marker::ADD);
  EXPECT_DOUBLE_EQ(6.0, TubeMarkerWidth(authoritative.tube_candidate));

  MatchedAdapterMarkerBundle pair_markers;
  ASSERT_TRUE(adapter.buildMarkers(
      fixture.control, fixture.control.epoch_snapshot, pair_markers));
  ExpectActions(pair_markers.tube_candidate,
                visualization_msgs::Marker::ADD);
  EXPECT_DOUBLE_EQ(5.0, TubeMarkerWidth(pair_markers.tube_candidate));
}

TEST(PhaseOffsetMatchedAdapterPairPublication,
     ExactLivePairUsesNewerTimerRawCandidateAndPairCertifiedGeometry) {
  PairPublicationFixture fixture = MakePairPublicationFixture();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  ASSERT_TRUE(PhaseOffsetMatchedAdapter::exactLivePairPublicationControl(
      fixture.control, fixture.request, fixture.pair,
      fixture.task_generation, fixture.authority_session));

  MatchedAdapterMarkerBundle markers;
  ASSERT_TRUE(adapter.buildMarkers(
      fixture.control, fixture.timer_candidate_profile, markers));
  ExpectActions(markers.tube_candidate, visualization_msgs::Marker::ADD);
  ExpectActions(markers.tube, visualization_msgs::Marker::ADD);
  EXPECT_DOUBLE_EQ(6.0, TubeMarkerWidth(markers.tube_candidate));
  EXPECT_DOUBLE_EQ(0.20, TubeMarkerWidth(markers.tube));
  EXPECT_EQ(fixture.control.output.candidate_profile.get(),
            fixture.pair_profile.get());
  EXPECT_EQ(fixture.control.output.active_profile.get(),
            fixture.pair_profile.get());
  EXPECT_NE(fixture.timer_candidate_profile.get(),
            fixture.pair_profile.get());
  EXPECT_DOUBLE_EQ(0.75,
                   fixture.control.output.diagnostics[kSelectedManual]);
}

TEST(PhaseOffsetMatchedAdapterPairPublication,
     ExactLivePairWithoutTimerCandidateUsesPreservedBroadRawBuildSamples) {
  PairPublicationFixture fixture = MakePairPublicationFixture();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  ASSERT_TRUE(PhaseOffsetMatchedAdapter::exactLivePairPublicationControl(
      fixture.control, fixture.request, fixture.pair,
      fixture.task_generation, fixture.authority_session));

  MatchedAdapterMarkerBundle markers;
  const std::shared_ptr<const TubeEpochSnapshot> candidate_epoch =
      adapter.selectCandidateEpochForPublication(
          true, std::shared_ptr<const TubeEpochSnapshot>(),
          fixture.control.epoch_snapshot);
  ASSERT_EQ(candidate_epoch.get(), fixture.control.epoch_snapshot.get());
  ASSERT_TRUE(adapter.buildMarkers(fixture.control, candidate_epoch, markers));
  ExpectActions(markers.tube_candidate, visualization_msgs::Marker::ADD);
  ExpectActions(markers.tube, visualization_msgs::Marker::ADD);
  EXPECT_DOUBLE_EQ(5.0, TubeMarkerWidth(markers.tube_candidate));
  EXPECT_DOUBLE_EQ(0.20, TubeMarkerWidth(markers.tube));
}

TEST(PhaseOffsetMatchedAdapterPairPublication,
     ProductionCandidateEpochSelectionUsesAuthoritativeThenLivePairAndFailsClosedWithoutEpoch) {
  PairPublicationFixture fixture = MakePairPublicationFixture();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  fixture.control.current_w = 1.3;
  ASSERT_TRUE(PhaseOffsetMatchedAdapter::exactLivePairPublicationControl(
      fixture.control, fixture.request, fixture.pair,
      fixture.task_generation, fixture.authority_session));

  const std::shared_ptr<const TubeEpochSnapshot> authoritative =
      adapter.selectCandidateEpochForPublication(
          true, fixture.timer_candidate_epoch, fixture.control.epoch_snapshot);
  ASSERT_EQ(authoritative.get(), fixture.timer_candidate_epoch.get());
  MatchedAdapterMarkerBundle authoritative_markers;
  ASSERT_TRUE(adapter.buildMarkers(fixture.control, authoritative,
                                   authoritative_markers));
  ExpectActions(authoritative_markers.tube_candidate,
                visualization_msgs::Marker::ADD);
  EXPECT_DOUBLE_EQ(6.0,
                   TubeMarkerWidth(authoritative_markers.tube_candidate));

  const std::shared_ptr<const TubeEpochSnapshot> live_pair_epoch =
      adapter.selectCandidateEpochForPublication(
          true, std::shared_ptr<const TubeEpochSnapshot>(),
          fixture.control.epoch_snapshot);
  ASSERT_EQ(live_pair_epoch.get(), fixture.control.epoch_snapshot.get());
  MatchedAdapterMarkerBundle live_pair_markers;
  ASSERT_TRUE(adapter.buildMarkers(fixture.control, live_pair_epoch,
                                   live_pair_markers));
  ExpectActions(live_pair_markers.tube_candidate,
                visualization_msgs::Marker::ADD);
  EXPECT_DOUBLE_EQ(5.0, TubeMarkerWidth(live_pair_markers.tube_candidate));

  const std::shared_ptr<const TubeEpochSnapshot> no_epoch =
      adapter.selectCandidateEpochForPublication(
          true, std::shared_ptr<const TubeEpochSnapshot>(),
          std::shared_ptr<const TubeEpochSnapshot>());
  EXPECT_FALSE(no_epoch);
  MatchedAdapterMarkerBundle deleted;
  ASSERT_TRUE(adapter.buildMarkers(fixture.control, no_epoch, deleted));
  ExpectActions(deleted.tube_candidate, visualization_msgs::Marker::DELETE);
}

TEST(PhaseOffsetMatchedAdapterPairPublication,
     RefreshedCopiedEpochRequiresExactCurrentAuthoritativePairPointer) {
  PairPublicationFixture fixture = MakePairPublicationFixture();
  std::shared_ptr<TubeEpochSnapshot> copied_epoch(
      new TubeEpochSnapshot(*fixture.control.epoch_snapshot));
  std::shared_ptr<PathTubePair> copied_pair(
      new PathTubePair(*fixture.pair));
  copied_pair->epoch_snapshot = copied_epoch;
  TubeBuildRequest copied_request = fixture.request;
  copied_request.base_path_tube_pair = copied_pair;
  copied_request.base_path_tube_pair_generation = copied_pair->generation;
  ControlPublishSnapshot copied_control = fixture.control;
  copied_control.epoch_snapshot = copied_epoch;
  EXPECT_TRUE(PhaseOffsetMatchedAdapter::exactLivePairPublicationControl(
      copied_control, copied_request,
      std::shared_ptr<const PathTubePair>(copied_pair),
      fixture.task_generation, fixture.authority_session));
  EXPECT_FALSE(PhaseOffsetMatchedAdapter::exactLivePairPublicationControl(
      copied_control, fixture.request,
      std::shared_ptr<const PathTubePair>(copied_pair),
      fixture.task_generation, fixture.authority_session));
}

TEST(PhaseOffsetMatchedAdapterPairPublication,
     StaleRetiredReplacedOrMalformedPairCannotCertifyLatestCandidate) {
  PairPublicationFixture fixture = MakePairPublicationFixture();
  std::shared_ptr<PathTubePair> replacement(
      new PathTubePair(*fixture.pair));
  ++replacement->generation;
  EXPECT_FALSE(PhaseOffsetMatchedAdapter::exactLivePairPublicationControl(
      fixture.control, fixture.request,
      std::shared_ptr<const PathTubePair>(replacement),
      fixture.task_generation, fixture.authority_session));
  EXPECT_FALSE(PhaseOffsetMatchedAdapter::exactLivePairPublicationControl(
      fixture.control, fixture.request, fixture.pair,
      fixture.task_generation, fixture.authority_session + 1U));

  std::shared_ptr<PathTubePair> malformed(
      new PathTubePair(*fixture.pair));
  malformed->active_profile.reset();
  TubeBuildRequest malformed_request = fixture.request;
  malformed_request.base_path_tube_pair = malformed;
  malformed_request.base_path_tube_pair_generation = malformed->generation;
  EXPECT_FALSE(PhaseOffsetMatchedAdapter::exactLivePairPublicationControl(
      fixture.control, malformed_request,
      std::shared_ptr<const PathTubePair>(malformed),
      fixture.task_generation, fixture.authority_session));

  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  ControlPublishSnapshot denied = fixture.control;
  denied.output.runtime_execution.mode =
      phase_offset_navigation::RuntimeExecutionMode::CERTIFICATE_DENIED;
  denied.output.runtime_execution.certificate_denied = true;
  MatchedAdapterMarkerBundle markers;
  ASSERT_TRUE(adapter.buildMarkers(
      denied, fixture.timer_candidate_profile, markers));
  ExpectActions(markers.tube_candidate, visualization_msgs::Marker::ADD);
  ExpectActions(markers.tube, visualization_msgs::Marker::DELETE);
  EXPECT_DOUBLE_EQ(6.0, TubeMarkerWidth(markers.tube_candidate));
}

TEST(PhaseOffsetMatchedAdapterPairPublication,
     OrdinaryMarkerCompositionRemainsSemanticallyIdentical) {
  PairPublicationFixture fixture = MakePairPublicationFixture();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterMarkerBundle ordinary;
  MatchedAdapterMarkerBundle explicit_same_owner;
  ASSERT_TRUE(adapter.buildMarkers(fixture.control, ordinary));
  ASSERT_TRUE(adapter.buildMarkers(
      fixture.control, fixture.control.output.candidate_profile,
      explicit_same_owner));
  ExpectMarkerArraysEqual(ordinary.tube_candidate,
                          explicit_same_owner.tube_candidate);
  ExpectMarkerArraysEqual(ordinary.tube, explicit_same_owner.tube);
  ExpectActions(ordinary.tube_candidate, visualization_msgs::Marker::ADD);
  ExpectActions(ordinary.tube, visualization_msgs::Marker::ADD);
}

TEST(PhaseOffsetMatchedAdapterTest, WaitingAndFailureLatchDeleteCertifiedButNotCompleteCandidate) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output = MakeMarkerOutput();
  output.tube_epoch_status.state = TubeEpochState::WAITING_FOR_CANDIDATE;
  output.tube_epoch_status.active_current_validation_valid = false;
  MatchedAdapterMarkerBundle markers;
  ASSERT_TRUE(adapter.buildMarkers(MakeInput(path, &path), output, markers));
  ExpectActions(markers.tube_candidate, visualization_msgs::Marker::ADD);
  ExpectActions(markers.tube, visualization_msgs::Marker::DELETE);
  output.failure_latched = true;
  ASSERT_TRUE(adapter.buildMarkers(MakeInput(path, &path), output, markers));
  ExpectActions(markers.tube_candidate, visualization_msgs::Marker::ADD);
  ExpectActions(markers.tube, visualization_msgs::Marker::DELETE);
}

TEST(PhaseOffsetMatchedAdapterTest,
     CertifiedDoesNotDependOnGateSelectionOrLatestCandidateSuffix) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output = MakeMarkerOutput();
  output.zero_gate_open = false;
  output.selected = false;
  output.tube_epoch_status.state = TubeEpochState::WAITING_FOR_CANDIDATE;
  // This emulates a newer candidate that was only suffix-truncated while the
  // independently installed active profile still has current exact evidence.
  output.candidate_profile = MakeMarkerProfile(30.0);
  MatchedAdapterMarkerBundle markers;
  ASSERT_TRUE(adapter.buildMarkers(MakeInput(path, &path), output, markers));
  ExpectActions(markers.tube_candidate, visualization_msgs::Marker::ADD);
  ExpectActions(markers.tube, visualization_msgs::Marker::ADD);
}

TEST(PhaseOffsetMatchedAdapterTest,
     RuntimeCertificateDenialDeletesCertifiedButLeavesCandidatePreviewVisible) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output = MakeMarkerOutput();
  output.runtime_execution.mode =
      phase_offset_navigation::RuntimeExecutionMode::CERTIFICATE_DENIED;
  output.runtime_execution.certificate_denied = true;
  MatchedAdapterMarkerBundle markers;
  ASSERT_TRUE(adapter.buildMarkers(MakeInput(path, &path), output, markers));
  ExpectActions(markers.tube_candidate, visualization_msgs::Marker::ADD);
  ExpectActions(markers.tube, visualization_msgs::Marker::DELETE);
}

TEST(PhaseOffsetMatchedAdapterTest, BuildMarkersNeverInvokesSamplingCallback) {
  const SyntheticPath path = MakePath();
  int sample_calls = 0;
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, &path);
  input.sampled_path.clear();
  input.sample_path = [&sample_calls, &path](double, std::vector<double>& w,
                                             std::vector<ContinuousPhasePathState>& states) {
    ++sample_calls;
    for (const auto& state : path.samples) {
      w.push_back(state.w);
      ContinuousPhasePathState converted;
      converted.p = state.p; converted.dp_dw = state.p_w; converted.d2p_dw2 = state.p_ww;
      converted.valid = state.valid; states.push_back(converted);
    }
    return true;
  };
  MatchedAdapterOutput output;
  adapter.update(input, output);
  ASSERT_EQ(sample_calls, 0);
  MatchedAdapterMarkerBundle markers;
  for (int index = 0; index < 20; ++index) adapter.buildMarkers(input, output, markers);
  EXPECT_EQ(sample_calls, 0);
}

TEST(PhaseOffsetMatchedAdapterTest,
     TubeSampleStepControlsCandidatePreviewSamplingDensity) {
  const SyntheticPath path = MakePath();
  auto run = [&path](const double tube_step, double& requested_step,
                     std::size_t& candidate_samples) {
    PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::FIXED);
    config.tube.sample_step_w = tube_step;
    config.preflight_sample_step_w = 0.10;
    PhaseOffsetMatchedAdapter adapter(config);
    MatchedAdapterInput input = MakeInput(path, &path);
    input.sampled_path.clear();
    requested_step = adapter.sampleStepW();
    for (double phase = 0.0; phase <= 3.0 + 1e-12; phase += requested_step) {
      input.sampled_path.push_back(MakeState(phase));
    }
    MatchedAdapterOutput output;
    BuildAndConsume(adapter, input, output);
    ASSERT_TRUE(output.candidate_profile);
    candidate_samples = output.candidate_profile->samples.size();
  };

  double fine_step = 0.0;
  double coarse_step = 0.0;
  std::size_t fine_samples = 0U;
  std::size_t coarse_samples = 0U;
  run(0.04, fine_step, fine_samples);
  run(0.08, coarse_step, coarse_samples);
  EXPECT_NEAR(fine_step, 0.04, 1e-12);
  EXPECT_NEAR(coarse_step, 0.08, 1e-12);
  EXPECT_GT(fine_samples, coarse_samples);
}

TEST(PhaseOffsetMatchedAdapterTest, ManualSchemaKeepsAliasAndStrictEightyThreeFields) {
  EXPECT_EQ(kManualLegacyDiagnosticCount, 34U);
  EXPECT_EQ(kTubeSource, kManualLegacyDiagnosticCount);
  EXPECT_EQ(kTubeReadinessEvaluated, 73U);
  EXPECT_EQ(kTubeDisplayCertified, 74U);
  EXPECT_EQ(kTubeMinSafetyMargin, 82U);
  EXPECT_EQ(kManualDiagnosticCount, 83U);
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  BuildAndConsume(adapter, MakeInput(path, &path), output);
  ASSERT_EQ(output.diagnostics.size(), 83U);
  for (double value : output.diagnostics) EXPECT_TRUE(std::isfinite(value));
  EXPECT_EQ(output.diagnostics[kTubeDisplayCertified], 1.0);
}

void InitializeKnownRawFreeMap(SDFMap& map) {
  map.mp_.resolution_ = map.mp_.resolution_inv_ = 1.0;
  map.mp_.map_origin_ = Eigen::Vector3d(-5.0, -5.0, -5.0);
  map.mp_.map_size_ = Eigen::Vector3d(10.0, 10.0, 10.0);
  map.mp_.map_voxel_num_ = Eigen::Vector3i(10, 10, 10);
  map.mp_.map_min_boundary_ = map.mp_.map_origin_;
  map.mp_.map_max_boundary_ = map.mp_.map_origin_ + map.mp_.map_size_;
  map.mp_.clamp_min_log_ = -2.0;
  map.mp_.min_occupancy_log_ = 0.0;
  map.md_.occupancy_buffer_.assign(1000U, -1.0);
  map.md_.occupancy_buffer_inflate_.assign(1000U, 1);
  map.md_.distance_buffer_all_.assign(1000U,
      std::numeric_limits<double>::quiet_NaN());
  map.md_.manual_boundary_enabled_ = false;
}

void InstallCompleteCloudSnapshot(SDFMap& map, std::uint64_t sequence = 1U) {
  auto snapshot = std::make_shared<plan_env::CloudOccupancySnapshot>();
  snapshot->valid = true;
  snapshot->observation_sequence = sequence;
  snapshot->observation_stamp = ros::Time(static_cast<double>(sequence));
  snapshot->map_min = map.mp_.map_min_boundary_;
  snapshot->map_max = map.mp_.map_max_boundary_;
  snapshot->observed_min = snapshot->map_min;
  snapshot->observed_max = snapshot->map_max;
  snapshot->grid_origin = map.mp_.map_origin_;
  snapshot->voxel_count = map.mp_.map_voxel_num_;
  snapshot->resolution = map.mp_.resolution_;
  snapshot->included_map_inflation = 0.10;
  const std::size_t size = static_cast<std::size_t>(snapshot->voxel_count.x()) *
      static_cast<std::size_t>(snapshot->voxel_count.y()) *
      static_cast<std::size_t>(snapshot->voxel_count.z());
  snapshot->occupied.assign(size, 0U);
  ASSERT_TRUE(plan_env::cloudOccupancySnapshotConsistent(*snapshot));
  ASSERT_TRUE(map.cloud_occupancy_snapshot_store_);
  std::lock_guard<std::mutex> lock(map.cloud_occupancy_snapshot_store_->mutex);
  map.cloud_occupancy_snapshot_store_->observation_sequence = sequence;
  map.cloud_occupancy_snapshot_store_->latest = snapshot;
}

void InitializeFineKnownRawFreeMap(SDFMap& map) {
  map.mp_.resolution_ = 0.10;
  map.mp_.resolution_inv_ = 10.0;
  map.mp_.map_origin_ = Eigen::Vector3d(-5.0, -5.0, -5.0);
  map.mp_.map_size_ = Eigen::Vector3d(10.0, 10.0, 10.0);
  map.mp_.map_voxel_num_ = Eigen::Vector3i(100, 100, 100);
  map.mp_.map_min_boundary_ = map.mp_.map_origin_;
  map.mp_.map_max_boundary_ = map.mp_.map_origin_ + map.mp_.map_size_;
  map.mp_.clamp_min_log_ = -2.0;
  map.mp_.min_occupancy_log_ = 0.0;
  const std::size_t size = 1000000U;
  map.md_.occupancy_buffer_.assign(size, -1.0);
  map.md_.occupancy_buffer_inflate_.assign(size, 1);
  map.md_.distance_buffer_all_.assign(
      size, std::numeric_limits<double>::quiet_NaN());
  map.md_.manual_boundary_enabled_ = false;
}

void MakeStraightRawPath(SyntheticPath& path) {
  for (auto& state : path.samples) {
    state.p = Eigen::Vector3d(state.w, 0.0, 1.0);
    state.p_w = Eigen::Vector3d::UnitX();
    state.p_ww.setZero();
  }
  path.current = path.samples[4U];
}

TEST(PhaseOffsetMatchedAdapterTest,
     PreparedEsdfTubePartitionsOwnerSegmentsAbsentFromPlannerSamples) {
  const std::shared_ptr<const ContinuousPhasePath> owner =
      MakeCertifiedThreePieceOwner();
  ASSERT_TRUE(owner);
  // These planner samples deliberately skip both immutable piece boundaries
  // w=1 and w=2.  A Tube cell built directly from this list would cross an
  // owner segment and `cellBounds()` would correctly reject it.
  const MatchedAdapterPathSamples supplied = SampleOwner(
      owner, {0.40, 0.80, 1.40, 1.80, 2.40, 2.80});
  ASSERT_EQ(supplied.size(), 6U);
  SDFMap map;
  InitializeFineKnownRawFreeMap(map);
  InstallCompleteCloudSnapshot(map, 88U);
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::ESDF));
  MatchedAdapterInput input;
  input.path = supplied.front();
  input.sampled_path = supplied;
  input.semantic_path_owner = owner;
  input.semantic_path_start_w = owner->startW();
  input.semantic_path_end_w = owner->endW();
  input.position = input.path.p;
  input.gains = MakeGains();
  input.dt = kDt;
  input.stamp = ros::Time(1.0);
  input.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
  const std::shared_ptr<const TubeBuildRequest> request =
      adapter.makeBuildRequest(input, 88U);
  ASSERT_TRUE(request);

  PreparedTubeBuildResult prepared;
  ASSERT_TRUE(adapter.buildPreparedTubeEpoch(
      *request, supplied, 0.40, 2.80, 1.20, 2.80, prepared));
  ASSERT_TRUE(prepared.active_profile);
  const TubeProfile& profile = *prepared.active_profile;
  EXPECT_TRUE(profile.cell_geometry_certified);
  EXPECT_EQ(profile.certified_cell_count, profile.samples.size() - 1U);
  EXPECT_TRUE(profile.zero_centerline_continuously_certified);
  bool saw_first_boundary = false;
  bool saw_second_boundary = false;
  for (const TubeRawSample& sample : profile.samples) {
    saw_first_boundary = saw_first_boundary || std::abs(sample.w - 1.0) <= 1e-10;
    saw_second_boundary = saw_second_boundary || std::abs(sample.w - 2.0) <= 1e-10;
  }
  EXPECT_TRUE(saw_first_boundary);
  EXPECT_TRUE(saw_second_boundary);
}

TEST(PhaseOffsetMatchedAdapterTest,
     CanonicalOwnerReuseRejectsRevisionMismatchAndCannotFabricateState) {
  const std::shared_ptr<const ContinuousPhasePath> owner =
      MakeSyntheticOwner();
  const MatchedAdapterPathSamples supplied = SampleOwner(
      owner, {0.40, 0.80, 1.40, 1.80, 2.40, 2.80});
  ASSERT_TRUE(owner);
  ASSERT_EQ(supplied.size(), 6U);
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(
      MakePath(), owner.get());
  input.semantic_path_owner = owner;
  input.semantic_path_start_w = owner->startW();
  input.semantic_path_end_w = owner->endW();
  const std::shared_ptr<const TubeBuildRequest> request =
      adapter.makeBuildRequest(input, 41U);
  ASSERT_TRUE(request);

  CanonicalOwnerStateReuse forged;
  forged.owner = owner;
  forged.state = request->current_path;
  forged.verified_samples = &supplied;
  forged.task_generation = request->task_generation;
  forged.source_revision = request->source_revision + 1U;
  forged.authority_session = request->authority_session;
  forged.map_observation_sequence = request->map_observation_sequence;
  forged.map_observation_is_snapshot = request->map_observation_is_snapshot;
  forged.valid = true;
  PreparedTubeBuildResult prepared;
  std::string failure_layer;
  EXPECT_FALSE(adapter.buildPreparedTubeEpoch(
      *request, supplied, 0.40, 2.80, 1.20, 2.80, prepared, nullptr,
      &failure_layer, &forged));
  EXPECT_EQ(failure_layer, "tube_build_owner_evaluate");
}

TEST(PhaseOffsetMatchedAdapterTest,
     ProductionBuildRequestsReuseOneImmutableFrameOwnerPerRevision) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;
  input.semantic_path_start_w = owner->startW();
  input.semantic_path_end_w = owner->endW();
  const std::shared_ptr<const TubeBuildRequest> first =
      adapter.makeBuildRequest(input, 55U);
  ASSERT_TRUE(first);
  ASSERT_TRUE(first->frame_owner);
  EXPECT_EQ(first->frame_owner->pathRevision(), 55U);
  EXPECT_EQ(first->frame_owner->frameRevision(), 55U);
  std::atomic_store(&adapter.latest_build_request_, first);
  const std::shared_ptr<const TubeBuildRequest> second =
      adapter.makeBuildRequest(input, 55U);
  ASSERT_TRUE(second);
  EXPECT_EQ(first->frame_owner, second->frame_owner);
  const std::shared_ptr<const TubeBuildRequest> successor =
      adapter.makeBuildRequest(input, 56U);
  ASSERT_TRUE(successor);
  ASSERT_TRUE(successor->frame_owner);
  EXPECT_NE(first->frame_owner, successor->frame_owner);
  EXPECT_EQ(successor->frame_owner->pathRevision(), 56U);
}

void SetInputPositionAndLegacy(MatchedAdapterInput& input,
                               const Eigen::Vector3d& position) {
  input.position = position;
  PhaseOffsetActiveAdapter zero;
  ActiveAdapterInput zero_input;
  zero_input.path = input.path;
  zero_input.position = input.position;
  zero_input.gains = input.gains;
  ActiveAdapterOutput zero_output;
  EXPECT_TRUE(zero.evaluate(zero_input, zero_output));
  input.legacy = LegacyGuidanceSnapshot(
      zero_output.guidance.v_cmd, zero_output.guidance.w_dot,
      zero_output.guidance.e_parallel, zero_output.guidance.e_perp,
      zero_output.guidance.ref_pt, zero_output.guidance.tangent,
      zero_output.guidance.valid);
}

const TubeRawSample* FindSampleAtCurrentW(const TubeProfile& profile,
                                          const double current_w) {
  for (const TubeRawSample& sample : profile.samples) {
    if (std::abs(sample.w - current_w) <= 1e-12) return &sample;
  }
  return nullptr;
}

void SetStraightCurrent(SyntheticPath& path, const double current_w) {
  path.current = MakeState(current_w);
  path.current.p = Eigen::Vector3d(current_w, 0.0, 1.0);
  path.current.p_w = Eigen::Vector3d::UnitX();
  path.current.p_ww.setZero();
}

void ExpectSingleCurrentAnchor(const TubeProfile& profile,
                               const double current_w) {
  ASSERT_TRUE(FindSampleAtCurrentW(profile, current_w));
  std::size_t anchor_count = 0U;
  for (std::size_t index = 0U; index < profile.samples.size(); ++index) {
    if (std::abs(profile.samples[index].w - current_w) <= 1e-9) {
      ++anchor_count;
    }
    if (index > 0U) {
      EXPECT_GT(profile.samples[index].w - profile.samples[index - 1U].w,
                1e-10);
    }
  }
  EXPECT_EQ(anchor_count, 1U);
}

void ExpectSingleCurrentMatchRepresentative(const TubeProfile& profile,
                                            const double current_w,
                                            const double representative_w) {
  std::size_t representative_count = 0U;
  const TubeRawSample* representative = nullptr;
  for (const TubeRawSample& sample : profile.samples) {
    if (std::abs(sample.w - current_w) <= 1e-9) {
      ++representative_count;
      representative = &sample;
    }
  }
  ASSERT_EQ(representative_count, 1U);
  ASSERT_NE(representative, nullptr);
  EXPECT_TRUE(representative->complete);
  EXPECT_DOUBLE_EQ(representative->w, representative_w);
}

void ExpectCertifiedOwnerPartitionWithoutSeamCrossing(
    const TubeProfile& profile,
    const std::shared_ptr<const ContinuousPhasePath>& owner) {
  ASSERT_TRUE(owner);
  ASSERT_TRUE(profile.cell_geometry_certified);
  ASSERT_EQ(profile.certified_cell_count, profile.samples.size() - 1U);
  ASSERT_GE(profile.samples.size(), 2U);
  for (std::size_t index = 1U; index < profile.samples.size(); ++index) {
    EXPECT_GT(profile.samples[index].w - profile.samples[index - 1U].w,
              1e-10);
    phase_offset_core::PathCellGeometryCertificate certificate;
    ASSERT_TRUE(owner->cellBounds(profile.samples[index - 1U].w,
                                  profile.samples[index].w, certificate));
    EXPECT_TRUE(certificate.complete);
  }
  ASSERT_FALSE(profile.raw_build_samples.empty());
  for (const TubeRawSample& sample : profile.raw_build_samples) {
    EXPECT_TRUE(sample.cell_geometry_certificate_used);
    EXPECT_DOUBLE_EQ(sample.continuous_inset, 0.0);
  }
}

phase_offset_navigation::PathCellBoundQuery
HorizontalNormalCertificateForTest(const double sup_normal_derivative) {
  return [sup_normal_derivative](
      const double w0, const double w1,
      phase_offset_core::PathCellGeometryCertificate& certificate) {
    certificate = phase_offset_core::PathCellGeometryCertificate();
    certificate.w0 = w0;
    certificate.w1 = w1;
    certificate.segment_w0 = 0.0;
    certificate.segment_w1 = 0.4;
    certificate.segment_identity = 17U;
    certificate.inf_p_w_norm = 1.0;
    certificate.inf_horizontal_p_w_norm = 1.0;
    certificate.sup_p_w_norm = 1.0;
    certificate.sup_p_ww_norm = 0.0;
    certificate.sup_p_www_norm = 0.0;
    certificate.sup_horizontal_p_ww_norm = sup_normal_derivative;
    certificate.horizontal_acceleration_bound_complete = true;
    certificate.sup_N_w_norm = sup_normal_derivative;
    certificate.sup_abs_curvature = 0.0;
    certificate.normal_variation_bound =
        sup_normal_derivative * (w1 - w0);
    certificate.tangent_variation_bound = 0.0;
    certificate.curvature_variation_bound = 0.0;
    certificate.midpoint_position_variation_bound = 0.0;
    certificate.chord_deviation_bound = 0.0;
    certificate.normal_frame_proof_complete = true;
    certificate.provenance =
        "ContinuousPhaseNormalFrame/WorldHorizontalCrossProduct";
    certificate.valid = std::isfinite(w0) && std::isfinite(w1) &&
        std::isfinite(sup_normal_derivative) && sup_normal_derivative >= 0.0 &&
        w1 > w0;
    certificate.complete = certificate.valid;
    return certificate.valid;
  };
}

TEST(PhaseOffsetMatchedAdapterTest,
     TubeBuilderConsumesHorizontalNormalDerivativeBound) {
  MatchedAdapterPathSamples line;
  for (int index = 0; index <= 4; ++index) {
    phase_offset_core::PathDifferentialState state;
    state.w = 0.1 * static_cast<double>(index);
    state.p = Eigen::Vector3d(state.w, 0.0, 1.0);
    state.p_w = Eigen::Vector3d::UnitX();
    state.p_ww.setZero();
    state.valid = true;
    line.push_back(state);
  }
  const phase_offset_navigation::PathStateQuery exact_line =
      [](const double w, phase_offset_core::PathDifferentialState& state) {
        state = phase_offset_core::PathDifferentialState();
        state.w = w;
        state.p = Eigen::Vector3d(w, 0.0, 1.0);
        state.p_w = Eigen::Vector3d::UnitX();
        state.p_ww.setZero();
        state.valid = true;
        return std::isfinite(w) && w >= -1e-12 && w <= 0.4 + 1e-12;
      };
  const phase_offset_navigation::ClearanceQuery open =
      [](const Eigen::Vector3d&, const double required) {
        phase_offset_navigation::ClearanceQueryResult result;
        result.status = phase_offset_navigation::DistanceStatus::KNOWN_FREE;
        result.clearance = std::max(10.0, required);
        result.clearance_certified = true;
        return result;
      };
  const phase_offset_navigation::TubeBuilderConfig builder_config =
      MakeManualConfig(TubeSource::ESDF).tube;

  TubeProfile exact_bound;
  ASSERT_TRUE(phase_offset_navigation::TubeBuilder(builder_config)
      .buildCloudClearance(
          TubeSource::ESDF, line, open, exact_line,
          HorizontalNormalCertificateForTest(0.0), 0.05, 0.0, 13U, 14U,
          exact_bound));
  ASSERT_TRUE(exact_bound.raw_complete);
  EXPECT_TRUE(exact_bound.cell_geometry_certified);
  EXPECT_TRUE(exact_bound.combined_regularity_proof_complete);
  EXPECT_NEAR(exact_bound.combined_regularity_speed_min, 1.0, 1e-12);

  // The complete certificate is retained, but the delayed combined regularity
  // proof rejects this nominal-width ribbon because its conservative speed
  // lower bound is below the configured minimum.  Contract A still keeps the
  // Builder inset at exact zero and leaves the Validator to fail closed.
  TubeProfile unsafe_bound;
  ASSERT_TRUE(phase_offset_navigation::TubeBuilder(builder_config)
      .buildCloudClearance(
          TubeSource::ESDF, line, open, exact_line,
          HorizontalNormalCertificateForTest(2.0), 0.05, 0.0, 13U, 15U,
          unsafe_bound));
  ASSERT_TRUE(unsafe_bound.raw_complete);
  EXPECT_TRUE(unsafe_bound.cell_geometry_certified);
  EXPECT_FALSE(unsafe_bound.combined_regularity_proof_complete);
  EXPECT_EQ(unsafe_bound.certified_cell_count,
            unsafe_bound.raw_build_samples.size() - 1U);
  ASSERT_FALSE(unsafe_bound.raw_build_samples.empty());
  // Contract A keeps Builder's continuous inset at exact zero even when the
  // certificate is rejected; no sampled fixed-half-voxel fallback is applied
  // by the Builder.
  EXPECT_DOUBLE_EQ(unsafe_bound.raw_build_samples.front().continuous_inset,
                   0.0);
  EXPECT_TRUE(unsafe_bound.raw_build_samples.front()
                  .cell_geometry_certificate_used);
}

TEST(PhaseOffsetMatchedAdapterTest,
     OrdinaryEsdfTimerKeepsEachOffGridCurrentPhaseAsCompleteAnchor) {
  SyntheticPath path = MakePath();
  MakeStraightRawPath(path);
  const std::shared_ptr<const ContinuousPhasePath> owner =
      MakeStraightSyntheticOwner();
  SDFMap map;
  InitializeFineKnownRawFreeMap(map);
  InstallCompleteCloudSnapshot(map, 901U);
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
  // The production failure requires current to be in the interior of the
  // owner-aligned preview rather than coincident with its start boundary.
  config.tube.back_w = 0.20;
  PhaseOffsetMatchedAdapter adapter(config);
  MatchedAdapterOutput output;

  const std::array<double, 3U> current_w = {{0.435, 0.783, 1.127}};
  for (std::size_t index = 0U; index < current_w.size(); ++index) {
    SetStraightCurrent(path, current_w[index]);
    MatchedAdapterInput input = MakeInput(path, owner.get(),
                                          static_cast<double>(index) * kDt);
    input.semantic_path_owner = owner;
    input.semantic_path_start_w = owner->startW();
    input.semantic_path_end_w = owner->endW();
    input.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
    SetInputPositionAndLegacy(input, path.current.p);

    EXPECT_FALSE(adapter.update(input, output));
    ASSERT_TRUE(adapter.timerTick());
    EXPECT_FALSE(adapter.update(input, output));
    ASSERT_TRUE(output.candidate_profile);
    EXPECT_TRUE(output.candidate_profile->raw_complete);
    EXPECT_TRUE(output.candidate_profile->filtered_complete);
    EXPECT_TRUE(output.candidate_profile->complete);
    EXPECT_EQ(output.tube_epoch_status.reason, TubeEpochReason::NONE);
    EXPECT_NE(output.tube_epoch_status.state,
              TubeEpochState::WAITING_FOR_CANDIDATE);
    ASSERT_TRUE(output.raw_candidate_diagnostics_generated);
    EXPECT_DOUBLE_EQ(output.raw_candidate_diagnostics[
        kRawCandidateCurrentSampleFound], 1.0);
    EXPECT_DOUBLE_EQ(output.raw_candidate_diagnostics[
        kRawCandidateCurrentSampleComplete], 1.0);
    ExpectSingleCurrentAnchor(*output.candidate_profile, current_w[index]);
  }
}

TEST(PhaseOffsetMatchedAdapterTest,
     OwnerAlignedTimerCanonicalizesNearKnotsAndSeamsToExactCurrentPhase) {
  const std::shared_ptr<const ContinuousPhasePath> owner =
      MakeCertifiedThreePieceOwner();
  ASSERT_TRUE(owner);
  const MatchedAdapterPathSamples supplied = SampleOwner(
      owner, {0.40, 0.80, 1.40, 1.80, 2.40, 2.80});
  ASSERT_EQ(supplied.size(), 6U);
  SDFMap map;
  InitializeFineKnownRawFreeMap(map);
  InstallCompleteCloudSnapshot(map, 902U);
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
  config.tube.back_w = 0.20;
  PhaseOffsetMatchedAdapter adapter(config);
  MatchedAdapterOutput output;

  // The first value is within the partition-dedup tolerance of a supplied
  // planner knot; the second is exactly an immutable owner seam.  Both must
  // remain a sole current anchor without duplicating a partition knot.
  const std::array<double, 2U> current_w = {{0.40000000005, 1.0}};
  for (std::size_t index = 0U; index < current_w.size(); ++index) {
    ContinuousPhasePathState owner_state;
    ASSERT_TRUE(owner->evaluate(current_w[index], owner_state, false));
    MatchedAdapterInput input;
    input.path = ConvertContinuousPhasePathStateForActive(owner_state,
                                                           current_w[index]);
    input.sampled_path = supplied;
    input.semantic_path_owner = owner;
    input.semantic_path_start_w = owner->startW();
    input.semantic_path_end_w = owner->endW();
    input.position = input.path.p;
    input.gains = MakeGains();
    input.dt = kDt;
    input.stamp = ros::Time(static_cast<double>(index) * kDt);
    input.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
    SetInputPositionAndLegacy(input, input.position);

    EXPECT_FALSE(adapter.update(input, output));
    const bool timer_result = adapter.timerTick();
    EXPECT_FALSE(adapter.update(input, output));
    ASSERT_TRUE(timer_result) << output.tube_epoch_status.reason_text;
    ASSERT_TRUE(output.candidate_profile);
    EXPECT_TRUE(output.candidate_profile->complete);
    ExpectSingleCurrentAnchor(*output.candidate_profile, current_w[index]);
  }
}

TEST(PhaseOffsetMatchedAdapterTest,
     OwnerAlignedTimerPreservesStructuralSeamNearCurrentOnBothSides) {
  const std::shared_ptr<const ContinuousPhasePath> owner =
      MakeCertifiedThreePieceOwner();
  ASSERT_TRUE(owner);
  const MatchedAdapterPathSamples supplied = SampleOwner(
      owner, {0.40, 0.80, 1.40, 1.80, 2.40, 2.80});
  ASSERT_EQ(supplied.size(), 6U);
  SDFMap map;
  InitializeFineKnownRawFreeMap(map);
  InstallCompleteCloudSnapshot(map, 905U);
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
  // Keep the preview start away from the deliberately omitted 0.80 planner
  // knot: otherwise the test itself would create an unrelated sub-epsilon
  // preview-boundary/knot cell before reaching the seam under test.
  config.tube.back_w = 0.13;

  constexpr double kStructuralSeamW = 1.0;
  // This is strictly outside the partition coverage tolerance but inside the
  // frozen TubeBuilder current-match tolerance.  The seam must consequently
  // become the sole current-match representative rather than being replaced
  // by either near-but-not-equal current phase.
  constexpr double kSeamOffset = 5e-10;
  EXPECT_GT(kSeamOffset, 1e-10);
  EXPECT_LT(kSeamOffset, 1e-9);
  const std::array<double, 2U> current_w = {{
      kStructuralSeamW - kSeamOffset,
      kStructuralSeamW + kSeamOffset,
  }};
  for (std::size_t index = 0U; index < current_w.size(); ++index) {
    ContinuousPhasePathState owner_state;
    ASSERT_TRUE(owner->evaluate(current_w[index], owner_state, false));
    PhaseOffsetMatchedAdapter adapter(config);
    MatchedAdapterInput input;
    input.path = ConvertContinuousPhasePathStateForActive(owner_state,
                                                           current_w[index]);
    input.sampled_path = supplied;
    input.semantic_path_owner = owner;
    input.semantic_path_start_w = owner->startW();
    input.semantic_path_end_w = owner->endW();
    input.position = input.path.p;
    input.gains = MakeGains();
    input.dt = kDt;
    input.stamp = ros::Time(static_cast<double>(index) * kDt);
    input.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
    SetInputPositionAndLegacy(input, input.position);

    MatchedAdapterOutput output;
    EXPECT_FALSE(adapter.update(input, output));
    ASSERT_TRUE(adapter.timerTick());
    EXPECT_FALSE(adapter.update(input, output));
    ASSERT_TRUE(output.candidate_profile);
    const TubeProfile& profile = *output.candidate_profile;
    EXPECT_TRUE(profile.raw_complete);
    EXPECT_TRUE(profile.filtered_complete);
    EXPECT_TRUE(profile.complete);
    ASSERT_TRUE(output.raw_candidate_diagnostics_generated);
    EXPECT_DOUBLE_EQ(output.raw_candidate_diagnostics[
        kRawCandidateCurrentSampleFound], 1.0);
    EXPECT_DOUBLE_EQ(output.raw_candidate_diagnostics[
        kRawCandidateCurrentSampleComplete], 1.0);

    const TubeRawSample* seam_sample =
        FindSampleAtCurrentW(profile, kStructuralSeamW);
    ASSERT_NE(seam_sample, nullptr);
    EXPECT_TRUE(seam_sample->complete);
    // No exact current knot is required when the frozen current-match class
    // is represented by an immutable owner seam.
    EXPECT_EQ(FindSampleAtCurrentW(profile, current_w[index]), nullptr);
    ExpectSingleCurrentMatchRepresentative(
        profile, current_w[index], kStructuralSeamW);
    ExpectCertifiedOwnerPartitionWithoutSeamCrossing(profile, owner);
  }
}

TEST(PhaseOffsetMatchedAdapterTest,
     OwnerAlignedTimerRetainsExactCurrentAtNearPreviewStartBoundary) {
  const std::shared_ptr<const ContinuousPhasePath> owner =
      MakeCertifiedThreePieceOwner();
  ASSERT_TRUE(owner);
  const MatchedAdapterPathSamples supplied = SampleOwner(
      owner, {0.40, 0.80, 1.40, 1.80, 2.40, 2.80});
  ASSERT_EQ(supplied.size(), 6U);
  SDFMap map;
  InitializeFineKnownRawFreeMap(map);
  InstallCompleteCloudSnapshot(map, 904U);
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
  // This makes preview_start_w differ from current by less than the legacy
  // partition-dedup tolerance.  The owner-aligned helper must preserve the
  // original current double as the canonical knot instead of snapping it to
  // the nearby preview boundary/supplied planner knot at 0.40.
  config.tube.back_w = 5e-11;
  PhaseOffsetMatchedAdapter adapter(config);
  MatchedAdapterOutput output;
  constexpr double kCurrentW = 0.40000000005;
  ContinuousPhasePathState owner_state;
  ASSERT_TRUE(owner->evaluate(kCurrentW, owner_state, false));
  MatchedAdapterInput input;
  input.path = ConvertContinuousPhasePathStateForActive(owner_state, kCurrentW);
  input.sampled_path = supplied;
  input.semantic_path_owner = owner;
  input.semantic_path_start_w = owner->startW();
  input.semantic_path_end_w = owner->endW();
  input.position = input.path.p;
  input.gains = MakeGains();
  input.dt = kDt;
  input.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
  SetInputPositionAndLegacy(input, input.position);

  EXPECT_FALSE(adapter.update(input, output));
  ASSERT_TRUE(adapter.timerTick());
  EXPECT_FALSE(adapter.update(input, output));
  ASSERT_TRUE(output.candidate_profile);
  EXPECT_TRUE(output.candidate_profile->raw_complete);
  EXPECT_TRUE(output.candidate_profile->filtered_complete);
  EXPECT_TRUE(output.candidate_profile->complete);
  EXPECT_DOUBLE_EQ(output.candidate_profile->preview_start_w, kCurrentW);
  ExpectSingleCurrentAnchor(*output.candidate_profile, kCurrentW);
}

TEST(PhaseOffsetMatchedAdapterTest,
     PreparedEsdfPairBuildKeepsOffGridCapturedPhaseAtStartBoundary) {
  const std::shared_ptr<const ContinuousPhasePath> owner =
      MakeStraightSyntheticOwner();
  ASSERT_TRUE(owner);
  const MatchedAdapterPathSamples supplied = SampleOwner(
      owner, {0.40, 0.80, 1.20, 1.60, 2.00, 2.40, 2.80});
  ASSERT_EQ(supplied.size(), 7U);
  constexpr double kCapturedW = 0.435;
  SDFMap map;
  InitializeFineKnownRawFreeMap(map);
  InstallCompleteCloudSnapshot(map, 903U);
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::ESDF));

  PathTubePairTransaction transaction;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, supplied, kCapturedW,
      1.20, 2.40, Eigen::Vector3d(kCapturedW, 0.0, 1.0), MakeGains(), kDt,
      map.cloudOccupancySnapshot(), transaction));
  ASSERT_TRUE(transaction.candidate_pair);
  ASSERT_TRUE(transaction.candidate_pair->active_profile);
  EXPECT_TRUE(transaction.candidate_pair->active_profile->raw_complete);
  EXPECT_TRUE(transaction.candidate_pair->active_profile->filtered_complete);
  EXPECT_TRUE(transaction.candidate_pair->active_profile->complete);
  ExpectSingleCurrentAnchor(*transaction.candidate_pair->active_profile,
                            kCapturedW);

  std::shared_ptr<const PathTubePair> committed;
  ASSERT_TRUE(PrepareAndFinalizePair(
      adapter, transaction, kCapturedW, Eigen::Vector3d(kCapturedW, 0.0, 1.0),
      MakeGains(), kDt, map.cloudOccupancySnapshot(), committed));
  ASSERT_TRUE(committed);
  ASSERT_TRUE(committed->active_profile);
  ExpectSingleCurrentAnchor(*committed->active_profile, kCapturedW);
}

TEST(PhaseOffsetMatchedAdapterTest,
     H2StageKeepsOneLockedIdentityAcrossConcurrentTaskReset) {
  constexpr double kCapturedW = 0.40;
  constexpr double kFutureSeamW = 1.20;
  constexpr double kExistingHorizonEndW = 2.40;
  const std::vector<double> sample_w{
      kCapturedW, 0.80, kFutureSeamW, 1.60, 2.00,
      kExistingHorizonEndW, 2.80};
  const std::shared_ptr<const ContinuousPhasePath> old_owner =
      MakeStraightSyntheticOwner();
  ASSERT_TRUE(old_owner);
  const MatchedAdapterPathSamples old_samples = SampleOwner(old_owner, sample_w);
  ASSERT_EQ(old_samples.size(), sample_w.size());

  SDFMap map;
  InitializeFineKnownRawFreeMap(map);
  InstallCompleteCloudSnapshot(map, 906U);
  const std::shared_ptr<const plan_env::CloudOccupancySnapshot> snapshot =
      map.cloudOccupancySnapshot();
  ASSERT_TRUE(snapshot);
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
  config.amplitude = 0.10;
  config.tube.interior_margin = 0.02;
  PhaseOffsetMatchedAdapter adapter(config);
  const Eigen::Vector3d position(kCapturedW, 0.0, 1.0);
  const guidance::IsfGains gains = MakeGains();

  PathTubePairTransaction bootstrap;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), old_owner, old_samples,
      kCapturedW, kFutureSeamW, kExistingHorizonEndW, position, gains, kDt,
      snapshot, bootstrap));
  std::shared_ptr<const PathTubePair> old_pair;
  ASSERT_TRUE(PrepareAndFinalizePair(
      adapter, bootstrap, kCapturedW, position, gains, kDt, snapshot,
      old_pair));
  ASSERT_TRUE(old_pair);
  std::unique_ptr<PathTubePairPin> old_pin =
      adapter.captureAndAcquirePathTubePairPin();
  ASSERT_TRUE(old_pin);
  ASSERT_TRUE(old_pin->valid());

  ASSERT_TRUE(adapter.runtime_);
  adapter.runtime_->delta_ = 0.08;
  adapter.runtime_->profile_started_ = true;
  adapter.runtime_->profile_completed_ = false;
  const double captured_retained_delta = adapter.runtime_->retainedDelta();
  const std::uint64_t captured_task_generation =
      adapter.task_generation_.load(std::memory_order_acquire);
  const std::uint64_t captured_authority_session =
      adapter.authority_session_.load(std::memory_order_acquire);
  ASSERT_EQ(captured_authority_session,
            old_pin->capture().authority_session);

  const std::shared_ptr<BlockingOwnerEvaluation> control(
      new BlockingOwnerEvaluation());
  const std::shared_ptr<const ContinuousPhasePath> new_owner =
      MakeBlockingStraightSyntheticOwner(control);
  ASSERT_TRUE(new_owner);
  const MatchedAdapterPathSamples new_samples = SampleOwner(new_owner, sample_w);
  ASSERT_EQ(new_samples.size(), sample_w.size());
  control->call_count.store(0U, std::memory_order_release);
  control->block_on_call = new_samples.size() + 1U;
  control->armed.store(true, std::memory_order_release);

  PathTubePairTransaction replacement;
  PathTubePairStageFailure stage_failure = PathTubePairStageFailure::NONE;
  bool stage_result = false;
  std::thread staging([&]() {
    stage_result = adapter.stagePathTubePair(
        old_pair, new_owner, new_samples, kCapturedW, kFutureSeamW,
        kExistingHorizonEndW, position, gains, kDt, snapshot, replacement,
        old_pin->capture().authority_session, &old_pin->capture(),
        old_pin->leaseId(), &stage_failure);
  });

  bool paused_after_capture = false;
  {
    std::unique_lock<std::mutex> lock(control->mutex);
    paused_after_capture = control->condition.wait_for(
        lock, std::chrono::seconds(5), [control]() {
          return control->paused;
        });
  }
  std::uint64_t retired_authority_session = 0U;
  const bool reset_result = paused_after_capture &&
      adapter.resetForNewNavigationTask(captured_authority_session,
                                        retired_authority_session);
  {
    std::lock_guard<std::mutex> lock(control->mutex);
    control->release = true;
  }
  control->condition.notify_all();
  staging.join();

  ASSERT_TRUE(paused_after_capture);
  ASSERT_TRUE(reset_result);
  ASSERT_TRUE(stage_result);
  EXPECT_EQ(stage_failure, PathTubePairStageFailure::NONE);
  EXPECT_EQ(adapter.task_generation_.load(std::memory_order_acquire),
            captured_task_generation + 1U);
  EXPECT_GT(retired_authority_session, captured_authority_session);
  EXPECT_EQ(adapter.authority_session_.load(std::memory_order_acquire),
            retired_authority_session);
  ASSERT_TRUE(replacement.candidate_pair);
  ASSERT_TRUE(replacement.candidate_pair->epoch_snapshot);
  ASSERT_TRUE(replacement.candidate_pair->active_profile);
  EXPECT_EQ(replacement.authority_session, captured_authority_session);
  EXPECT_EQ(replacement.candidate_pair->authority_session,
            captured_authority_session);
  EXPECT_EQ(replacement.candidate_pair->epoch_snapshot->task_generation,
            captured_task_generation);
  EXPECT_DOUBLE_EQ(replacement.captured_retained_delta,
                   captured_retained_delta);

  const TubeProfile& profile = *replacement.candidate_pair->active_profile;
  const TubeRawSample* current = FindSampleAtCurrentW(profile, kCapturedW);
  ASSERT_NE(current, nullptr);
  EXPECT_LT(current->raw_lower, -0.20);
  EXPECT_GT(current->raw_upper, 0.20);
  EXPECT_DOUBLE_EQ(current->filtered_lower, current->raw_lower);
  EXPECT_DOUBLE_EQ(current->filtered_upper, current->raw_upper);
  EXPECT_GE(captured_retained_delta,
            current->filtered_lower + config.tube.interior_margin - 1e-12);
  EXPECT_LE(captured_retained_delta,
            current->filtered_upper - config.tube.interior_margin + 1e-12);

  PathTubePairCommitPreparation stale_preparation;
  EXPECT_FALSE(adapter.preparePathTubePairCommit(
      replacement, kCapturedW, position, gains, kDt, snapshot,
      stale_preparation));
  EXPECT_FALSE(adapter.capturePathTubePair());
  old_pin->release();
}

TEST(PhaseOffsetMatchedAdapterTest,
     EsdfUsesImmutableCloudSnapshotRatherThanRawLogOddsOrLegacyMaxOffset) {
  SyntheticPath path = MakePath();
  for (auto& state : path.samples) {
    state.p = Eigen::Vector3d(state.w, 0.0, 1.0);
    state.p_w = Eigen::Vector3d::UnitX();
    state.p_ww.setZero();
  }
  path.current = path.samples[4U];
  SDFMap map;
  InitializeKnownRawFreeMap(map);
  // Raw log odds remain UNKNOWN and inflated map contents are irrelevant.  A
  // complete cloud-observation snapshot is the only production backing.
  InstallCompleteCloudSnapshot(map, 17U);
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::ESDF));
  MatchedAdapterInput input = MakeInput(path, &path);
  input.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
  MatchedAdapterOutput output;
  BuildAndConsume(adapter, input, output);
  EXPECT_FALSE(output.selected);
  EXPECT_TRUE(output.tube_update_due_this_cycle);
  EXPECT_TRUE(output.tube_epoch_status.raw_cross_section_path_used);
  EXPECT_TRUE(output.tube_epoch_status.map_observation_is_snapshot);
  EXPECT_EQ(output.tube_epoch_status.candidate_map_observation_sequence, 17U);
  ASSERT_TRUE(output.candidate_profile);
  ASSERT_TRUE(output.active_profile);
  EXPECT_TRUE(output.candidate_profile->complete);
  EXPECT_NEAR(output.diagnostics[kTubeRequiredReferenceClearance], 0.40, 1e-12);
  EXPECT_NEAR(output.diagnostics[kTubeRequiredActualClearance], 0.40, 1e-12);
  EXPECT_NEAR(output.diagnostics[kTubeTrackingBound], 0.15, 1e-12);
  ASSERT_FALSE(output.candidate_profile->samples.empty());
  const auto& sample = output.candidate_profile->samples.front();
  EXPECT_GT(sample.environment_width, 0.45);
  EXPECT_GT(sample.raw_upper, 0.20);
  EXPECT_LT(sample.raw_lower, -0.20);
  EXPECT_NEAR(sample.full_effective_radius, 0.55, 1e-12);
  EXPECT_NEAR(sample.preincluded_map_uncertainty, 0.10, 1e-12);
  EXPECT_NEAR(sample.residual_effective_radius, 0.40, 1e-12);
  ASSERT_TRUE(output.cloud_snapshot_diagnostics_generated);
  EXPECT_NEAR(output.cloud_snapshot_diagnostics[
                  kCloudSnapshotFullEffectiveRadius], 0.55, 1e-12);
  EXPECT_NEAR(output.cloud_snapshot_diagnostics[
                  kCloudSnapshotResidualEffectiveRadius], 0.45, 1e-12);
  EXPECT_DOUBLE_EQ(output.cloud_snapshot_diagnostics[
                       kCloudSnapshotRawStorageAccessed], 0.0);
  EXPECT_DOUBLE_EQ(output.cloud_snapshot_diagnostics[
                       kCloudSnapshotSelfFreeSeedUsed], 0.0);
}

TEST(PhaseOffsetMatchedAdapterTest,
     RawCandidateDiagnosticsAreGeneratedOnlyForDueManualEsdfAttempts) {
  SyntheticPath path = MakePath();
  for (auto& state : path.samples) {
    state.p = Eigen::Vector3d(state.w, 0.0, 1.0);
    state.p_w = Eigen::Vector3d::UnitX();
    state.p_ww.setZero();
  }
  path.current = path.samples[4U];
  SDFMap map;
  InitializeKnownRawFreeMap(map);
  InstallCompleteCloudSnapshot(map, 23U);
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::ESDF));
  MatchedAdapterInput input = MakeInput(path, &path);
  input.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
  MatchedAdapterOutput output;
  BuildAndConsume(adapter, input, output);
  ASSERT_TRUE(output.tube_update_due_this_cycle);
  ASSERT_TRUE(output.raw_candidate_diagnostics_generated);
  ASSERT_EQ(output.raw_candidate_diagnostics.size(),
            kRawCandidateDiagnosticCount);
  EXPECT_DOUBLE_EQ(output.raw_candidate_diagnostics[
                       kRawCandidateRawSourceConfigured], 1.0);
  EXPECT_DOUBLE_EQ(output.raw_candidate_diagnostics[
                       kRawCandidateRawStorageReady], 0.0);
  EXPECT_DOUBLE_EQ(output.raw_candidate_diagnostics[
                       kRawCandidateRawQueryInjected], 1.0);
  EXPECT_DOUBLE_EQ(output.raw_candidate_diagnostics[
                       kRawCandidateTubeUpdateDue], 1.0);
  ASSERT_TRUE(output.candidate_profile);
  EXPECT_EQ(output.raw_candidate_diagnostics[
                kRawCandidateSampleCount],
            static_cast<double>(output.candidate_profile->samples.size()));
  EXPECT_TRUE(output.active_profile);
  EXPECT_EQ(output.tube_epoch_status.state, TubeEpochState::ROLLING);
  ASSERT_TRUE(output.cloud_snapshot_diagnostics_generated);
  EXPECT_EQ(output.cloud_snapshot_diagnostics[
                kCloudSnapshotObservationSequence], 23.0);
  EXPECT_FALSE(output.selected);
  for (int cycle = 1; cycle <= 4; ++cycle) {
    input = MakeInput(path, &path, cycle * kDt);
    input.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
    adapter.update(input, output);
    EXPECT_FALSE(output.tube_update_due_this_cycle);
    EXPECT_FALSE(output.raw_candidate_diagnostics_generated);
    EXPECT_FALSE(output.cloud_snapshot_diagnostics_generated);
  }

  PhaseOffsetMatchedAdapter fixed_adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput fixed_output;
  BuildAndConsume(fixed_adapter, MakeInput(path, &path), fixed_output);
  EXPECT_TRUE(fixed_output.tube_update_due_this_cycle);
  EXPECT_FALSE(fixed_output.raw_candidate_diagnostics_generated);

  PhaseOffsetMatchedAdapter none_adapter(MakeManualConfig(TubeSource::NONE));
  MatchedAdapterOutput none_output;
  none_adapter.update(MakeInput(path, &path), none_output);
  EXPECT_FALSE(none_output.raw_candidate_diagnostics_generated);
  EXPECT_EQ(none_output.diagnostics.size(), kManualDiagnosticCount);
}

TEST(PhaseOffsetMatchedAdapterTest,
     CompleteCloudSnapshotBuildsCandidateAndKeepsOneObservationIdentity) {
  SyntheticPath path = MakePath();
  MakeStraightRawPath(path);
  SDFMap map;
  InitializeFineKnownRawFreeMap(map);
  // Deliberately leave raw log odds UNKNOWN.  No self-free seed exists in the
  // production path; the complete snapshot is sufficient on its own.
  InstallCompleteCloudSnapshot(map, 31U);

  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::ESDF));
  MatchedAdapterInput input = MakeInput(path, &path);
  SetInputPositionAndLegacy(input, path.current.p);
  input.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
  MatchedAdapterOutput output;
  BuildAndConsume(adapter, input, output);

  ASSERT_TRUE(output.tube_update_due_this_cycle);
  ASSERT_TRUE(output.raw_candidate_diagnostics_generated);
  ASSERT_TRUE(output.cloud_snapshot_diagnostics_generated);
  ASSERT_TRUE(output.candidate_profile);
  ASSERT_TRUE(output.active_profile);
  ASSERT_TRUE(output.candidate_profile->complete);
  const std::shared_ptr<const TubeBuildRequest> captured_request =
      std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(captured_request);
  ASSERT_TRUE(captured_request->authority_request.valid);
  EXPECT_DOUBLE_EQ(captured_request->authority_request.lower, -0.10);
  EXPECT_DOUBLE_EQ(captured_request->authority_request.upper, 0.10);
  EXPECT_TRUE(output.tube_epoch_status.raw_cross_section_path_used);
  EXPECT_TRUE(output.tube_epoch_status.map_observation_is_snapshot);
  EXPECT_EQ(output.tube_epoch_status.candidate_map_observation_sequence, 31U);
  EXPECT_EQ(output.tube_epoch_status.state, TubeEpochState::ROLLING);
  const TubeRawSample* current =
      FindSampleAtCurrentW(*output.candidate_profile, path.current.w);
  ASSERT_NE(current, nullptr);
  EXPECT_TRUE(current->complete);
  EXPECT_GT(current->environment_width, 0.45);
  EXPECT_DOUBLE_EQ(current->filtered_lower, current->raw_lower);
  EXPECT_DOUBLE_EQ(current->filtered_upper, current->raw_upper);
  EXPECT_GT(current->filtered_upper - current->filtered_lower,
            2.0 * adapter.config_.tube.fixed_delta_max);
  EXPECT_GT(current->raw_upper, 0.20);
  EXPECT_LT(current->raw_lower, -0.20);
  EXPECT_EQ(output.raw_candidate_diagnostics[
                kRawCandidateCurrentBaseRawStatus],
            static_cast<double>(DistanceStatus::KNOWN_FREE));
  EXPECT_EQ(output.raw_candidate_diagnostics[
                kRawCandidateActualPositionRawStatus],
            static_cast<double>(DistanceStatus::KNOWN_FREE));
  EXPECT_EQ(output.raw_candidate_diagnostics[
                kRawCandidateCurrentPlusStepRawStatus],
            static_cast<double>(DistanceStatus::KNOWN_FREE));
  EXPECT_EQ(output.raw_candidate_diagnostics[
                kRawCandidateCurrentMinusStepRawStatus],
            static_cast<double>(DistanceStatus::KNOWN_FREE));
  EXPECT_DOUBLE_EQ(output.raw_candidate_diagnostics[
                       kRawCandidateRawStorageReady], 0.0);
  EXPECT_EQ(output.diagnostics.size(), kManualDiagnosticCount);
  EXPECT_EQ(output.raw_candidate_diagnostics.size(),
            kRawCandidateDiagnosticCount);
  for (const double value : output.diagnostics) EXPECT_TRUE(std::isfinite(value));
  for (const double value : output.raw_candidate_diagnostics) {
    EXPECT_TRUE(std::isfinite(value));
  }
  MatchedAdapterMarkerBundle markers;
  ASSERT_TRUE(adapter.buildMarkers(input, output, markers));
  ExpectActions(markers.tube_candidate, visualization_msgs::Marker::ADD);

  const auto* candidate_before_non_due = output.candidate_profile.get();
  const std::uint64_t sequence_before_non_due =
      output.tube_epoch_status.candidate_sequence;
  MatchedAdapterInput non_due = MakeInput(path, &path, kDt);
  SetInputPositionAndLegacy(non_due, path.current.p + Eigen::Vector3d(1.0, 0.0, 0.0));
  non_due.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
  adapter.update(non_due, output);
  EXPECT_FALSE(output.tube_update_due_this_cycle);
  EXPECT_FALSE(output.raw_candidate_diagnostics_generated);
  EXPECT_EQ(output.tube_epoch_status.candidate_sequence, sequence_before_non_due);
  EXPECT_EQ(output.tube_epoch_status.candidate_map_observation_sequence, 31U);
  EXPECT_EQ(output.candidate_profile.get(), candidate_before_non_due);
}

TEST(PhaseOffsetMatchedAdapterTest,
     ValidatorFailureKeepsBroadRawDiagnosticsAndIncompleteCandidate) {
  SyntheticPath path = MakePath();
  MakeStraightRawPath(path);
  SDFMap map;
  InitializeFineKnownRawFreeMap(map);
  InstallCompleteCloudSnapshot(map, 32U);
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
  PhaseOffsetMatchedAdapter adapter(config);
  phase_offset_navigation::TubeEpochManagerConfig epoch_config;
  epoch_config.builder = config.tube;
  epoch_config.filter = config.filter;
  // Test-only terminal fixture: production defaults and launch values remain
  // unchanged.  This forces the existing fail-closed path after a real broad
  // Builder/Filter candidate has been produced.
  epoch_config.surface_validator.max_query_samples = 9U;
  adapter.tube_epoch_manager_.reset(
      new phase_offset_navigation::TubeEpochManager(epoch_config));
  MatchedAdapterInput input = MakeInput(path, &path);
  SetInputPositionAndLegacy(input, path.current.p);
  input.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
  MatchedAdapterOutput output;
  BuildAndConsume(adapter, input, output);

  ASSERT_TRUE(output.candidate_profile);
  EXPECT_EQ(output.candidate_profile->classification,
            phase_offset_navigation::TubeProfileClassification::NONE);
  EXPECT_FALSE(output.candidate_profile->obstacle_certified);
  EXPECT_FALSE(output.candidate_profile->complete);
  const TubeRawSample* current =
      FindSampleAtCurrentW(*output.candidate_profile, path.current.w);
  ASSERT_NE(current, nullptr);
  EXPECT_LT(current->raw_lower, -0.20);
  EXPECT_GT(current->raw_upper, 0.20);
  EXPECT_LT(current->environment_lower, -0.20);
  EXPECT_GT(current->environment_upper, 0.20);
  EXPECT_GT(current->environment_width, 0.40);
  EXPECT_DOUBLE_EQ(current->filtered_lower, current->raw_lower);
  EXPECT_DOUBLE_EQ(current->filtered_upper, current->raw_upper);
  ASSERT_TRUE(output.raw_candidate_diagnostics_generated);
  EXPECT_LT(output.raw_candidate_diagnostics[
                kRawCandidateCurrentEnvironmentLower],
            -0.20);
  EXPECT_GT(output.raw_candidate_diagnostics[
                kRawCandidateCurrentEnvironmentUpper],
            0.20);
  EXPECT_GT(output.raw_candidate_diagnostics[
                kRawCandidateCurrentEnvironmentWidth],
            0.40);

  MatchedAdapterMarkerBundle markers;
  ASSERT_TRUE(adapter.buildMarkers(input, output, markers));
  ExpectActions(markers.tube_candidate, visualization_msgs::Marker::ADD);
  ExpectActions(markers.tube, visualization_msgs::Marker::DELETE);
  ASSERT_FALSE(markers.tube_candidate.markers[0].points.empty());
  ASSERT_FALSE(markers.tube_candidate.markers[1].points.empty());
  double candidate_lower = std::numeric_limits<double>::infinity();
  double candidate_upper = -std::numeric_limits<double>::infinity();
  for (const geometry_msgs::Point& point :
       markers.tube_candidate.markers[0].points) {
    candidate_lower = std::min(candidate_lower, point.y);
  }
  for (const geometry_msgs::Point& point :
       markers.tube_candidate.markers[1].points) {
    candidate_upper = std::max(candidate_upper, point.y);
  }
  EXPECT_LT(candidate_lower, -0.20);
  EXPECT_GT(candidate_upper, 0.20);
}

TEST(PhaseOffsetMatchedAdapterTest, MissingCloudSnapshotFailsClosedWithoutLegacyFallback) {
  SyntheticPath path = MakePath();
  MakeStraightRawPath(path);
  SDFMap map;
  InitializeFineKnownRawFreeMap(map);
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::ESDF));
  MatchedAdapterInput input = MakeInput(path, &path);
  SetInputPositionAndLegacy(input, path.current.p);
  input.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
  MatchedAdapterOutput output;
  BuildAndConsume(adapter, input, output);

  ASSERT_TRUE(output.tube_update_due_this_cycle);
  ASSERT_TRUE(output.candidate_profile);
  EXPECT_FALSE(output.candidate_profile->complete);
  EXPECT_FALSE(output.active_profile);
  EXPECT_EQ(output.tube_epoch_status.state, TubeEpochState::WAITING_FOR_CANDIDATE);
  ASSERT_TRUE(output.cloud_snapshot_diagnostics_generated);
  EXPECT_DOUBLE_EQ(output.cloud_snapshot_diagnostics[
                       kCloudSnapshotAvailable], 0.0);
  EXPECT_DOUBLE_EQ(output.cloud_snapshot_diagnostics[
                       kCloudSnapshotUsable], 0.0);
  EXPECT_DOUBLE_EQ(output.raw_candidate_diagnostics[
                       kRawCandidateRawStorageReady], 0.0);
}

TEST(PhaseOffsetMatchedAdapterTest,
     SparseHitCloudContractRemainsObservableWithoutOffsetAuthority) {
  SyntheticPath path = MakePath();
  MakeStraightRawPath(path);
  SDFMap map;
  InitializeFineKnownRawFreeMap(map);
  InstallCompleteCloudSnapshot(map, 37U);
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
  config.cloud_obstacle_set_complete = false;
  PhaseOffsetMatchedAdapter adapter(config);
  MatchedAdapterInput input = MakeInput(path, &path);
  SetInputPositionAndLegacy(input, path.current.p);
  input.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
  MatchedAdapterOutput output;
  BuildAndConsume(adapter, input, output);

  ASSERT_TRUE(output.candidate_profile);
  EXPECT_TRUE(output.candidate_profile->complete);
  EXPECT_FALSE(output.active_profile);
  // The unusable snapshot remains candidate-only diagnostic evidence; it can
  // never create an active profile or request offset authority.
  EXPECT_FALSE(adapter.requiresPathTubePairBootstrap());
  ASSERT_TRUE(output.cloud_snapshot_diagnostics_generated);
  EXPECT_DOUBLE_EQ(output.cloud_snapshot_diagnostics[
                       kCloudSnapshotObstacleSetComplete], 0.0);
  EXPECT_DOUBLE_EQ(output.cloud_snapshot_diagnostics[
                       kCloudSnapshotUsable], 0.0);
}

TEST(PhaseOffsetMatchedAdapterTest, ActiveModeRetainsA3ZeroPortGateSemantics) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapterConfig config;
  config.mode = PhaseOffsetMatchedMode::ACTIVE; config.warmup_cycles = 100;
  PhaseOffsetMatchedAdapter adapter(config);
  MatchedAdapterOutput output;
  OpenGate(adapter, path, output);
  EXPECT_TRUE(output.zero_gate_open);
  EXPECT_TRUE(output.selected);
  EXPECT_NEAR((output.guidance.v_cmd - output.zero_port.guidance.v_cmd).norm(), 0.0, 1e-15);
}

TEST(PhaseOffsetMatchedAdapterTest,
     AdvertisedNormalRuntimeCandidateIsRejectedWithoutAllocator) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::NONE));
  // The test does not need ROS transport; this flips the production boundary
  // and disables the unadvertised Runtime fixture exception.
  adapter.advertised_ = true;
  adapter.execution_authority_.setTestOnlyRuntimeOwnerAllowed(false);
  MatchedAdapterOutput output;
  for (int cycle = 0; cycle < 100; ++cycle) {
    EXPECT_FALSE(adapter.update(MakeInput(path, &path, cycle * kDt), output));
  }
  EXPECT_TRUE(output.zero_gate_open);
  EXPECT_FALSE(output.selected);
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), 0.0);
  EXPECT_FALSE(adapter.execution_authority_.snapshot().valid);
}

TEST(PhaseOffsetMatchedAdapterTest,
     LegacyUnpairedPathRevisionRequiresTimerEpoch) {
  const SyntheticPath path = MakePath();
  int first_identity = 101;
  int revised_identity = 102;
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  BuildAndConsume(adapter, MakeInput(path, &first_identity), output);
  ASSERT_TRUE(output.active_profile);
  const auto* first_active = output.active_profile.get();

  MatchedAdapterInput revised = MakeInput(path, &revised_identity, kDt);
  EXPECT_FALSE(adapter.update(revised, output));
  EXPECT_FALSE(output.active_profile);
  const std::shared_ptr<const TubeBuildRequest> pending =
      std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(pending);
  EXPECT_EQ(pending->source_revision, 2U);
  ASSERT_TRUE(adapter.timerTick());
  EXPECT_FALSE(adapter.update(revised, output));
  ASSERT_TRUE(output.active_profile);
  EXPECT_NE(output.active_profile.get(), first_active);
  EXPECT_EQ(output.tube_epoch_status.active_path_source_revision, 2U);
}

TEST(PhaseOffsetMatchedAdapterTest,
     PreparedTubeEpochRejectsClaimedRangeOutsideSamples) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;
  const std::shared_ptr<const TubeBuildRequest> request =
      adapter.makeBuildRequest(input, 41U);
  ASSERT_TRUE(request);

  PreparedTubeBuildResult prepared;
  // The supplied samples really cover [0, 3], but a caller cannot claim a
  // narrower/different interval to evade the future-seam coverage checks.
  EXPECT_FALSE(adapter.buildPreparedTubeEpoch(*request, path.samples,
                                              0.1, 2.9, 1.0, 2.4, prepared));
  EXPECT_FALSE(prepared.complete);
}

TEST(PhaseOffsetMatchedAdapterTest,
     PreparedTubeEpochRejectsProvenanceMismatch) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;
  const std::shared_ptr<const TubeBuildRequest> request =
      adapter.makeBuildRequest(input, 42U);
  ASSERT_TRUE(request);

  TubeBuildRequest mismatched = *request;
  // FIXED has no frozen-map provenance.  A forged snapshot key must fail
  // before a staged manager can create a misleading active result.
  mismatched.map_observation_is_snapshot = true;
  mismatched.map_observation_sequence = 7U;
  PreparedTubeBuildResult prepared;
  EXPECT_FALSE(adapter.buildPreparedTubeEpoch(mismatched, path.samples,
                                              0.0, 3.0, 1.0, 2.4, prepared));
  EXPECT_FALSE(prepared.complete);
}

TEST(PhaseOffsetMatchedAdapterTest,
     PreparedTubeEpochRejectsCurrentStateThatDoesNotMatchNewOwner) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;
  const std::shared_ptr<const TubeBuildRequest> request =
      adapter.makeBuildRequest(input, 43U);
  ASSERT_TRUE(request);

  TubeBuildRequest mismatched = *request;
  mismatched.current_path.p.x() += 1e-3;
  PreparedTubeBuildResult prepared;
  EXPECT_FALSE(adapter.buildPreparedTubeEpoch(mismatched, path.samples,
                                              0.0, 3.0, 1.0, 2.4, prepared));
  EXPECT_FALSE(prepared.complete);
}

TEST(PhaseOffsetMatchedAdapterTest,
     PreparedTubeEpochOwnsVerifiedPathTubeAndRuntimeDryRunDoesNotMutateLiveState) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;
  const std::shared_ptr<const TubeBuildRequest> request =
      adapter.makeBuildRequest(input, 43U);
  ASSERT_TRUE(request);

  PreparedTubeBuildResult prepared;
  ASSERT_TRUE(adapter.buildPreparedTubeEpoch(*request, path.samples,
                                             0.0, 3.0, 1.0, 2.4, prepared));
  ASSERT_TRUE(prepared.complete);
  EXPECT_EQ(prepared.semantic_path_owner.get(), owner.get());
  EXPECT_FALSE(prepared.frozen_cloud_occupancy_snapshot);
  EXPECT_EQ(prepared.source_revision, 43U);
  EXPECT_EQ(prepared.epoch_status.candidate_path_source_revision, 43U);
  EXPECT_EQ(prepared.epoch_status.active_path_source_revision, 43U);
  EXPECT_TRUE(prepared.epoch_status.active_current_validation_valid);
  EXPECT_EQ(prepared.prepared_start_w, 0.0);
  EXPECT_EQ(prepared.prepared_end_w, 3.0);
  EXPECT_EQ(prepared.captured_w0, path.current.w);
  EXPECT_EQ(prepared.future_seam_w, 1.0);
  EXPECT_EQ(prepared.existing_future_horizon_end_w, 2.4);
  ASSERT_TRUE(prepared.full_path_samples);
  ASSERT_TRUE(prepared.active_profile);
  EXPECT_NE(prepared.full_path_samples.get(), &path.samples);
  EXPECT_NE(prepared.active_profile.get(),
            static_cast<const TubeProfile*>(nullptr));
  EXPECT_LE(prepared.active_profile->preview_start_w, path.current.w);
  EXPECT_GE(prepared.active_profile->preview_end_w, 2.4);
  EXPECT_LE(prepared.active_profile->certified_segment_start_w, path.current.w);
  EXPECT_GE(prepared.active_profile->certified_segment_end_w, 2.4);

  // Seed two equal live Runtime copies.  The staging call is allowed to run
  // the same exact U+ -> U_safe logic, but must leave every live state bit
  // untouched so their next real step remains identical.
  ASSERT_TRUE(adapter.runtime_);
  phase_offset_navigation::PhaseOffsetRuntime baseline(*adapter.runtime_);
  const double delta_before = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      adapter.runtime_->previousFinalPort();
  phase_offset_navigation::RuntimeDryRunResult staged;
  ASSERT_TRUE(adapter.dryRunPreparedRuntime(prepared, path.current,
                                            input.position, input.gains,
                                            input.dt, staged));
  EXPECT_TRUE(staged.valid);
  EXPECT_TRUE(staged.step.projection.valid);
  EXPECT_TRUE(staged.step.matched.valid);
  const double delta_after_dry_run = adapter.runtime_->retainedDelta();
  EXPECT_EQ(std::memcmp(&delta_before, &delta_after_dry_run,
                        sizeof(delta_before)), 0);
  const phase_offset_core::PortCommand previous_after =
      adapter.runtime_->previousFinalPort();
  EXPECT_EQ(std::memcmp(&previous_before.u_w, &previous_after.u_w,
                        sizeof(previous_before.u_w)), 0);
  EXPECT_EQ(std::memcmp(&previous_before.u_delta, &previous_after.u_delta,
                        sizeof(previous_before.u_delta)), 0);

  phase_offset_navigation::RuntimePrepareInput real;
  real.current_path = path.current;
  real.position = input.position;
  real.tube_view.active_profile = prepared.active_profile;
  real.tube_view.epoch_status = prepared.epoch_status;
  real.dt = input.dt;
  // The staged dry-run already captured the immutable owner/profile contract
  // used for its step-wise recomputation.  Reuse that value here rather than
  // reaching into the adapter implementation's translation-unit-local helper.
  real.future_step = staged.prepared.future_step;
  real.zero_gate_open = true;
  phase_offset_navigation::RuntimePreparedStep live_prepared;
  phase_offset_navigation::RuntimePreparedStep baseline_prepared;
  ASSERT_TRUE(adapter.runtime_->prepare(real, live_prepared));
  ASSERT_TRUE(baseline.prepare(real, baseline_prepared));
  guidance::ReferenceGeometry reference;
  reference.point = live_prepared.geometry.r;
  reference.tangent = live_prepared.geometry.T;
  reference.derivative_norm = live_prepared.geometry.r_w.norm();
  reference.valid = live_prepared.geometry.valid;
  guidance::IsfGuidance guidance;
  ASSERT_TRUE(guidance::IsfReferenceKernel::evaluate(input.position, reference,
                                                      input.gains, guidance));
  phase_offset_navigation::RuntimeStepOutput live_output;
  phase_offset_navigation::RuntimeStepOutput baseline_output;
  ASSERT_TRUE(adapter.runtime_->complete(live_prepared, guidance.v_cmd,
                                         guidance.w_dot, guidance.valid,
                                         live_output));
  ASSERT_TRUE(baseline.complete(baseline_prepared, guidance.v_cmd,
                                guidance.w_dot, guidance.valid,
                                baseline_output));
  EXPECT_EQ(std::memcmp(&live_output.projection.final_port.u_w,
                        &baseline_output.projection.final_port.u_w,
                        sizeof(double)), 0);
  EXPECT_EQ(std::memcmp(&live_output.projection.final_port.u_delta,
                        &baseline_output.projection.final_port.u_delta,
                        sizeof(double)), 0);
  const double live_delta_after_commit = adapter.runtime_->retainedDelta();
  const double baseline_delta_after_commit = baseline.retainedDelta();
  EXPECT_EQ(std::memcmp(&live_delta_after_commit, &baseline_delta_after_commit,
                        sizeof(double)), 0);
}

TEST(PhaseOffsetMatchedAdapterTest,
     PreparedOwnerContractRecomputesEveryH2DryRunStep) {
  constexpr double kCapturedW0 = 0.40;
  constexpr double kFutureSeamW = 1.20;
  constexpr double kExistingHorizonEndW = 2.40;
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;
  const std::shared_ptr<const TubeBuildRequest> request =
      adapter.makeBuildRequest(input, 57U);
  ASSERT_TRUE(request);
  PreparedTubeBuildResult prepared;
  ASSERT_TRUE(adapter.buildPreparedTubeEpoch(
      *request, path.samples, 0.0, 3.0, kFutureSeamW,
      kExistingHorizonEndW, prepared));
  ASSERT_TRUE(prepared.complete);
  ASSERT_TRUE(prepared.active_profile);

  // The production staging entry point creates the immutable owner/profile
  // contract itself.  A successful H2 dry run proves it can reevaluate every
  // finite future step without reaching into the adapter's private helper.
  phase_offset_navigation::RuntimeDryRunResult result;
  ASSERT_TRUE(adapter.dryRunPreparedRuntime(
      prepared, path.current, input.position, input.gains, input.dt, result));
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.prepared.future_step.evaluate);
  EXPECT_DOUBLE_EQ(result.prepared.future_step.profile_domain_end_w,
                   prepared.active_profile->certified_segment_end_w);

  // Directly exercise that production contract at two distinct predicted
  // states.  The owner-derived path, reference and base guidance must change
  // with phase and predicted position; no frozen current fact is accepted.
  phase_offset_navigation::RuntimeFutureStepResult first;
  phase_offset_navigation::RuntimeFutureStepResult second;
  phase_offset_navigation::RuntimeFutureStepInput first_input;
  first_input.step_index = 1U;
  first_input.phase = kCapturedW0 + 0.01;
  first_input.delta = 0.0;
  first_input.matched_position = input.position +
      Eigen::Vector3d(0.01, 0.0, 0.0);
  first_input.previous_matched_reference = input.position;
  first_input.dt = input.dt;
  ASSERT_TRUE(result.prepared.future_step.evaluate(first_input, first));
  phase_offset_navigation::RuntimeFutureStepInput second_input = first_input;
  second_input.step_index = 2U;
  second_input.phase += 0.01;
  second_input.matched_position += Eigen::Vector3d(0.01, 0.0, 0.0);
  second_input.previous_matched_reference = first.matched_reference;
  second_input.previous_final_port.u_w = 0.02;
  ASSERT_TRUE(result.prepared.future_step.evaluate(second_input, second));
  EXPECT_GT(second.path.w, first.path.w);
  EXPECT_GT((second.matched_reference - first.matched_reference).norm(),
            1e-12);
  EXPECT_GT((second.base_v_cmd - first.base_v_cmd).norm(), 1e-12);
}

TEST(PhaseOffsetMatchedAdapterTest,
     CapturedPrivateRosInvariantConflictHasExactRecomputedWitness) {
  // Self-contained replay of the task-owned private ROS/GDB capture at
  // /tmp/a5g_stage1_real_cohort_capture_20260814/
  // run_20260814_022317_3862752.  These are captured immutable path/profile
  // values, not a fitted path or frozen-base counterfactual.
  static const double kPath[][10] = {
      {0.34866291270099797, -0.13481657042839595, -0.32139029898675231,
       1.0000034121292194, -0.35431209098648686, -0.93512723305652778,
       5.4895133219225151e-07, 0.15011214678645432,
       -0.056876269714741579, -9.7901096996601936e-05},
      {0.39844006481783095, -0.15226947552478209, -0.36800733760007354,
       1.0000033055363025, -0.34696921327753655, -0.93787651897008406,
       -5.0686714849333555e-06, 0.14531219077782614,
       -0.053758522403519193, -0.00012652256317218058},
      {0.44821721693466393, -0.16936233698912287, -0.41475760064244305,
       1.0000028920881205, -0.33985318350216676, -0.94047850233622332,
       -1.1420720046199468e-05, 0.13989652579966735,
       -0.050553285601233733, -0.00011526880843270687},
      {0.49799436905149697, -0.18610823869439952, -0.46163327443222268,
       1.0000021928905938, -0.33302386564477676, -0.94291839756368001,
       -1.6443374002678492e-05, 0.13480237135210268,
       -0.047610064069112723, -8.762443419730238e-05},
      {0.54777152116832994, -0.2025198079749666, -0.50862705195189228,
       1.0000012754500711, -0.32640695008852999, -0.94522933849303403,
       -2.0235161833252442e-05, 0.13128633384278005,
       -0.045335844689779581, -6.5495163910671611e-05},
      {0.59754867328516292, -0.21860583326465266, -0.55573326177094051,
       1.000000194917507, -0.3199338386290162, -0.94743988632172638,
       -2.3028960613857851e-05, 0.12897405821386976,
       -0.043552277054942357, -4.731564590081784e-05},
      {0.64732582540199601, -0.23437240863713363, -0.60294735606715,
       0.99999900141462617, -0.31357987210454019, -0.94956182686104373,
       -2.4550761950833622e-05, 0.12588264147186162,
       -0.041571029268892445, -5.3606701157282166e-06},
      {0.69710297751882888, -0.24982691244717659, -0.6502645245997769,
       0.99999779378778952, -0.30739289233514572, -0.95158268637585064,
       -2.3561112854853054e-05, 0.12285713989420231,
       -0.039686948024446278, 4.4023942158604669e-05},
  };
  static const double kProfile[][3] = {
      {0.05, -1.1569754243517232, 2.1421537026582556},
      {0.099777152116833, -1.1171537026582568, 2.1023319809647893},
      {0.149554304233666, -1.0773319809647903, 2.062510259271323},
      {0.19933145635049898, -1.037510259271324, 2.022688537577857},
      {0.24910860846733196, -0.99768853757785758, 1.9828668158843905},
      {0.29888576058416494, -0.95786681588439115, 1.943045094190924},
      {0.34866291270099797, -0.91804509419092473, 1.9032233724974577},
      {0.39844006481783095, -0.8782233724974583, 1.8634016508039912},
      {0.44821721693466393, -0.83840165080399187, 1.8235799291105248},
      {0.49799436905149697, -0.79857992911052544, 1.7837582074170584},
      {0.54777152116832994, -0.75875820741705902, 1.743936485723592},
      {0.59754867328516292, -0.71893648572359259, 1.7041147640301255},
      {0.64732582540199601, -0.67911476403012616, 1.664293042336659},
      {0.69710297751882888, -0.63929304233665984, 1.6244713206431929},
      {0.74688012963566197, -0.59947132064319342, 1.5846495989497265},
      {0.79665728175249484, -0.5596495989497271, 1.5448278772562603},
      {0.84643443386932793, -0.51982787725626067, 1.5050061555627938},
      {0.89621158598616091, -0.4800061555627943, 1.4651844338693274},
      {0.94598873810299398, -0.44018443386932782, 1.425362712175861},
      {0.99576589021982687, -0.4003627121758615, 1.3855409904823948},
      {1.0455430423366598, -0.36054099048239513, 1.3457192687889283},
      {1.095320194453493, -0.32071926878892865, 1.305897547095462},
      {1.1450973465703258, -0.28089754709546233, 1.2660758254019957},
      {1.194874498687159, -0.24107582540199585, 1.2262541037085293},
      {1.244651650803992, -0.20125410370852936, 1.1864323820150628},
      {1.2944288029208249, -0.16143238201506308, 1.1466106603215966},
      {1.3442059550376577, -0.12161066032159679, 1.1067889386281304},
      {1.3939831071544908, -0.081788938628130317, 1.066967216934664},
      {1.443760259271324, -0.041967216934663841, 1.0271454952411976},
      {1.493537411388157, -0.0021454952411973649, 0.98732377354773115},
      {1.5433145635049896, 0.037676226452268757, 0.94750205185426506},
      {1.593091715621823, 0.077497948145735407, 0.90768033016079841},
      {1.6428688677386558, 0.1173196698392017, 0.86785860846733209},
      {1.6926460198554887, 0.15714139153266798, 0.82803688677386578},
      {1.7424231719723218, 0.19696311322613447, 0.78821516508039935},
      {1.767311748030738, 0.21687397407286754, 0.76830430423366625},
      {1.7922003240891546, 0.23678483491960078, 0.74839344338693303},
      {1.841977476205988, 0.27660655661306743, 0.70857172169346638},
      {1.8917546283228208, 0.31642827830653375, 0.66875000000000007},
      {1.9415317804396537, 0.35625000000000007, 0.63125000000000009},
      {1.9913089325564868, 0.38750000000000018, 0.6000000000000002},
      {2.0410860846733194, 0.40625000000000022, 0.58125000000000027},
  };

  phase_offset_navigation::RuntimePathSamples path;
  for (const auto& row : kPath) {
    phase_offset_core::PathDifferentialState state;
    state.w = row[0];
    state.p = Eigen::Vector3d(row[1], row[2], row[3]);
    state.p_w = Eigen::Vector3d(row[4], row[5], row[6]);
    state.p_ww = Eigen::Vector3d(row[7], row[8], row[9]);
    state.valid = true;
    path.push_back(state);
  }
  auto profile = std::make_shared<TubeProfile>();
  profile->source = TubeSource::ESDF;
  profile->preview_start_w = kProfile[0][0];
  profile->preview_end_w = kProfile[41][0];
  profile->certified_segment_start_w = kProfile[0][0];
  profile->certified_segment_end_w = kProfile[41][0];
  profile->raw_complete = true;
  profile->filtered_complete = true;
  profile->complete = true;
  // TubeFilter::query exposes the stored slope of the active PWL piece.  The
  // capture includes those values (0.8/-0.8 around the replay root); rebuild
  // them from the captured adjacent PWL endpoints so this fixture preserves
  // the same complete profile contract instead of silently defaulting both
  // fields to zero.
  constexpr std::size_t kProfileCount = sizeof(kProfile) / sizeof(kProfile[0]);
  for (std::size_t index = 0U; index < kProfileCount; ++index) {
    const auto& row = kProfile[index];
    TubeRawSample sample;
    sample.w = row[0];
    sample.raw_lower = sample.filtered_lower = row[1];
    sample.raw_upper = sample.filtered_upper = row[2];
    const std::size_t adjacent = index + 1U < kProfileCount
        ? index + 1U : index - 1U;
    const double dw = kProfile[adjacent][0] - row[0];
    ASSERT_GT(std::abs(dw), 0.0);
    sample.lower_w = (kProfile[adjacent][1] - row[1]) / dw;
    sample.upper_w = (kProfile[adjacent][2] - row[2]) / dw;
    sample.complete = true;
    profile->samples.push_back(sample);
  }

  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
  config.u_w_rate_max = 0.60;
  config.u_delta_abs_max = 0.25;
  config.u_delta_rate_max = 1.20;
  PhaseOffsetMatchedAdapter adapter(config);
  ASSERT_TRUE(adapter.runtime_);
  phase_offset_navigation::PhaseOffsetRuntime runtime(*adapter.runtime_);
  const Eigen::Vector3d position(-0.13067169648113633,
                                 -0.45679149458085927,
                                 0.95924771276966647);
  phase_offset_navigation::RuntimePreflightInput preflight;
  preflight.path = path;
  preflight.position = position;
  preflight.path_source_revision = 1U;
  ASSERT_TRUE(runtime.refreshPreflight(preflight));

  phase_offset_core::PathDifferentialState current;
  current.w = 0.38548839310449201;
  current.p = Eigen::Vector3d(-0.14776342864891789,
                              -0.3558647972985407,
                              1.0000033607574914);
  current.p_w = Eigen::Vector3d(-0.34885775713218697,
                                -0.93717568529077155,
                                -3.4731292683863717e-06);
  current.p_ww = Eigen::Vector3d(0.14634033466386856,
                                 -0.054474269156965738,
                                 -0.00011978926371536302);
  current.valid = true;
  const guidance::IsfGains gains(2.0, -2.2, 0.1, 0.5, 0.3, 0.05);
  const std::shared_ptr<std::vector<
      phase_offset_navigation::RuntimeFutureStepInput>> calls(
          new std::vector<phase_offset_navigation::RuntimeFutureStepInput>());
  const auto immutable_path = std::make_shared<const
      phase_offset_navigation::RuntimePathSamples>(path);

  phase_offset_navigation::RuntimeFutureStepContract contract;
  contract.tube_update_period = 0.10;
  contract.min_certified_forward_w = 0.40;
  contract.profile_domain_end_w = profile->certified_segment_end_w;
  contract.evaluate = [immutable_path, calls, gains](
      const phase_offset_navigation::RuntimeFutureStepInput& input,
      phase_offset_navigation::RuntimeFutureStepResult& output) {
    calls->push_back(input);
    output = phase_offset_navigation::RuntimeFutureStepResult();
    if (!std::isfinite(input.phase) || !input.matched_position.allFinite() ||
        input.phase < immutable_path->front().w - 1e-12 ||
        input.phase > immutable_path->back().w + 1e-12) {
      output.invalid_reason = "captured immutable path cannot evaluate phase";
      return false;
    }
    for (std::size_t index = 1U; index < immutable_path->size(); ++index) {
      const auto& left = (*immutable_path)[index - 1U];
      const auto& right = (*immutable_path)[index];
      if (input.phase > right.w + 1e-12) continue;
      const double alpha = (input.phase - left.w) / (right.w - left.w);
      output.path.w = input.phase;
      output.path.p = left.p + alpha * (right.p - left.p);
      output.path.p_w = left.p_w + alpha * (right.p_w - left.p_w);
      output.path.p_ww = left.p_ww + alpha * (right.p_ww - left.p_ww);
      output.path.valid = true;
      break;
    }
    phase_offset_core::GeometryParams params;
    params.regularity_margin = 0.10;
    phase_offset_core::GeometryEvaluator evaluator(params);
    phase_offset_core::PhaseOffsetGeometryState geometry;
    if (!output.path.valid || !evaluator.evaluate(
            output.path, input.matched_position, input.delta, geometry)) {
      output.invalid_reason = geometry.invalid_reason;
      return false;
    }
    guidance::ReferenceGeometry reference;
    reference.point = geometry.r;
    reference.tangent = geometry.T;
    reference.derivative_norm = geometry.r_w.norm();
    reference.valid = geometry.valid;
    guidance::IsfGuidance base;
    if (!guidance::IsfReferenceKernel::evaluate(
            input.matched_position, reference, gains, base)) {
      output.invalid_reason = base.invalid_reason;
      return false;
    }
    output.matched_reference = reference.point;
    output.matched_tangent = reference.tangent;
    output.matched_derivative_norm = reference.derivative_norm;
    output.base_v_cmd = base.v_cmd;
    output.base_w_dot = base.w_dot;
    output.base_guidance_valid = base.valid;
    output.valid = true;
    return true;
  };

  phase_offset_navigation::RuntimePrepareInput input;
  input.current_path = current;
  input.position = position;
  input.tube_view.active_profile = profile;
  input.tube_view.epoch_status.state = TubeEpochState::ROLLING;
  input.tube_view.epoch_status.active_available = true;
  input.tube_view.epoch_status.active_current_validation_valid = true;
  input.tube_view.epoch_status.current_state_admissible = true;
  input.dt = kDt;
  input.future_step = contract;
  input.zero_gate_open = false;
  phase_offset_navigation::RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(input, prepared));
  guidance::ReferenceGeometry reference;
  reference.point = prepared.geometry.r;
  reference.tangent = prepared.geometry.T;
  reference.derivative_norm = prepared.geometry.r_w.norm();
  reference.valid = prepared.geometry.valid;
  guidance::IsfGuidance base;
  ASSERT_TRUE(guidance::IsfReferenceKernel::evaluate(
      position, reference, gains, base));
  phase_offset_navigation::RuntimeStepOutput output;
  ASSERT_TRUE(runtime.complete(prepared, base.v_cmd, base.w_dot, base.valid,
                               output)) << output.invalid_reason;
  EXPECT_TRUE(output.valid);
  EXPECT_FALSE(output.selected);  // Captured before the existing warmup gate.
  EXPECT_EQ(output.execution.mode,
            phase_offset_navigation::RuntimeExecutionMode::NORMAL);
  EXPECT_NEAR(output.projection.final_port.u_w, 0.0, 1e-12);
  EXPECT_NEAR(output.projection.final_port.u_delta, 0.0, 1e-12);
  EXPECT_NEAR(output.projection.final_w_dot, 2.5421980481141033, 1e-9);
  EXPECT_NEAR(output.projection.upper_invariant_residual,
              -0.15999545031662277, 1e-9);
  EXPECT_NEAR(output.projection.lower_invariant_residual,
              -1.1451737286231558, 1e-9);
  EXPECT_TRUE(output.projection.diagnostics.core_polygon_nonempty);
  ASSERT_GE(calls->size(), 4U);
  for (std::size_t index = 1U; index < calls->size(); ++index) {
    EXPECT_GT((*calls)[index].phase, (*calls)[index - 1U].phase);
    EXPECT_GT(((*calls)[index].matched_position -
               (*calls)[index - 1U].matched_position).norm(), 1e-12);
  }
}

TEST(PhaseOffsetMatchedAdapterTest,
     BootstrapPreparedHorizonUsesExistingInstallHorizonInsteadOfWholePath) {
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  EXPECT_DOUBLE_EQ(0.8,
      adapter.bootstrapPreparedHorizonEnd(0.4, 0.5, 3.0));
  EXPECT_DOUBLE_EQ(0.8,
      adapter.bootstrapPreparedHorizonEnd(0.4, 0.5, 1.2));
  EXPECT_DOUBLE_EQ(2.8,
      adapter.bootstrapPreparedHorizonEnd(0.4, 2.8, 3.0));
  EXPECT_TRUE(std::isnan(
      adapter.bootstrapPreparedHorizonEnd(0.4, 0.4, 3.0)));
}

TEST(PhaseOffsetMatchedAdapterTest,
     PreparedRuntimeDryRunRejectsOwnerMismatchBeforePreparedStartAndSeam) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;
  const std::shared_ptr<const TubeBuildRequest> request =
      adapter.makeBuildRequest(input, 44U);
  ASSERT_TRUE(request);
  PreparedTubeBuildResult prepared;
  ASSERT_TRUE(adapter.buildPreparedTubeEpoch(*request, path.samples,
                                             0.0, 3.0, 1.0, 2.4, prepared));

  phase_offset_navigation::RuntimeDryRunResult result;
  phase_offset_core::PathDifferentialState owner_mismatch = path.current;
  owner_mismatch.p.y() += 1e-3;
  EXPECT_FALSE(adapter.dryRunPreparedRuntime(prepared, owner_mismatch,
                                             input.position, input.gains,
                                             input.dt, result));
  phase_offset_core::PathDifferentialState at_seam = MakeState(1.0);
  EXPECT_FALSE(adapter.dryRunPreparedRuntime(prepared, at_seam,
                                             input.position, input.gains,
                                             input.dt, result));

  // Build a transaction whose copied-prefix coverage deliberately begins at
  // w=0.2.  A syntactically valid owner state before that boundary must still
  // be rejected; it cannot be treated as eligible merely because the owner
  // itself has a wider domain.
  MatchedAdapterPathSamples limited_prefix(
      path.samples.begin() + 2, path.samples.end());
  PreparedTubeBuildResult limited;
  ASSERT_TRUE(adapter.buildPreparedTubeEpoch(*request, limited_prefix,
                                             0.2, 3.0, 1.0, 2.4, limited));
  const phase_offset_core::PathDifferentialState before_start = MakeState(0.1);
  EXPECT_FALSE(adapter.dryRunPreparedRuntime(limited, before_start,
                                             input.position, input.gains,
                                             input.dt, result));
}

TEST(PhaseOffsetMatchedAdapterTest,
     PathTubePairCommitPublishesOnlyWholeNewAuthorityAndKeepsRuntimeBits) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;
  const phase_offset_core::PortCommand port_before =
      adapter.runtime_->previousFinalPort();
  const double delta_before = adapter.runtime_->retainedDelta();
  PathTubePairTransaction transaction;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples, path.current.w,
      1.0, 2.4, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), transaction));
  ASSERT_TRUE(transaction.candidate_pair);
  EXPECT_FALSE(adapter.capturePathTubePair());
  std::shared_ptr<const PathTubePair> committed;
  ASSERT_TRUE(PrepareAndFinalizePair(adapter, transaction, path.current.w, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), committed));
  ASSERT_TRUE(committed);
  EXPECT_EQ(committed.get(), adapter.capturePathTubePair().get());
  EXPECT_EQ(committed->path_owner.get(), owner.get());
  EXPECT_EQ(committed->active_profile->source_revision,
            committed->source_revision);
  EXPECT_EQ(committed->epoch_status.active_path_source_revision,
            committed->source_revision);
  const double delta_after = adapter.runtime_->retainedDelta();
  EXPECT_EQ(0, std::memcmp(&delta_before, &delta_after, sizeof(double)));
  const phase_offset_core::PortCommand port_after =
      adapter.runtime_->previousFinalPort();
  EXPECT_EQ(0, std::memcmp(&port_before.u_w, &port_after.u_w, sizeof(double)));
  EXPECT_EQ(0, std::memcmp(&port_before.u_delta, &port_after.u_delta,
                           sizeof(double)));
}

TEST(PhaseOffsetMatchedAdapterTest,
     OneSidedReachableRetainedPortUsesRuntimeContinuousWitnessForAcceptance) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner =
      MakeSyntheticOwner();
  PhaseOffsetMatchedAdapterConfig config =
      MakeManualConfig(TubeSource::FIXED);
  // The retained port is .15 at the live replacement phase.  The old pair
  // must therefore be a real current authority as well as a pin/CAS fixture;
  // its fixed Tube cannot use the default +/- .04 test extent.
  config.tube.fixed_delta_max = 0.40;
  PhaseOffsetMatchedAdapter adapter(config);
  ASSERT_TRUE(adapter.runtime_);
  constexpr std::uint64_t kRevision = 71U;
  const std::shared_ptr<const TubeProfile> profile =
      MakeOneSidedPreparedProfile(owner, kRevision);
  ASSERT_TRUE(profile);

  // Install a neutral old authority first.  This is an armed replacement
  // history, not the unpaired zero-start special case.  Zero
  // is absent from every exact PWL section.  The current retained port .15 is
  // admissible at capture, while the future seam lower bound rises to .18 and
  // deliberately excludes an unchanged .15.  The acceptance proof must use
  // Runtime's continuous witness rather than static retained-at-seam reuse.
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;
  PathTubePairTransaction old_transaction;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples,
      path.current.w, 1.0, 2.4, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), old_transaction));
  std::shared_ptr<const PathTubePair> old_pair;
  ASSERT_TRUE(PrepareAndFinalizePair(
      adapter, old_transaction, path.current.w, input.position, input.gains,
      input.dt, std::shared_ptr<const plan_env::CloudOccupancySnapshot>(),
      old_pair));
  ASSERT_TRUE(old_pair);

  adapter.runtime_->delta_ = 0.15;
  adapter.runtime_->previous_final_port_ = phase_offset_core::PortCommand();
  const double retained_before = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      adapter.runtime_->previousFinalPort();

  std::shared_ptr<PathTubePair> candidate(new PathTubePair());
  candidate->source_revision = kRevision;
  candidate->authority_session = old_pair->authority_session;
  candidate->path_owner = owner;
  candidate->full_path_samples =
      std::make_shared<const MatchedAdapterPathSamples>(path.samples);
  candidate->active_profile = profile;
  candidate->epoch_status = MakeOneSidedEpochStatus(kRevision);
  candidate->captured_w0 = path.current.w;
  candidate->future_seam_w = 1.0;
  candidate->existing_future_horizon_end_w = 2.4;
  candidate->captured_retained_delta = retained_before;
  candidate->captured_previous_final_port = previous_before;

  PathTubePairTransaction transaction;
  std::unique_ptr<PathTubePairPin> old_pin =
      adapter.captureAndAcquirePathTubePairPin();
  ASSERT_TRUE(old_pin);
  ASSERT_TRUE(old_pin->valid());
  transaction.expected_pair = old_pair;
  transaction.candidate_pair = std::shared_ptr<const PathTubePair>(candidate);
  transaction.expected_capture = old_pin->capture();
  transaction.pin_lease_id = old_pin->leaseId();
  transaction.authority_session = candidate->authority_session;
  transaction.captured_retained_delta = retained_before;
  transaction.captured_previous_final_port = previous_before;
  PathTubePairCommitPreparation preparation;
  ASSERT_TRUE(adapter.preparePathTubePairCommit(
      transaction, path.current.w, Eigen::Vector3d(0.4, 0.1, 1.1),
      MakeGains(), kDt, std::shared_ptr<const plan_env::CloudOccupancySnapshot>(),
      preparation));
  EXPECT_TRUE(preparation.valid);
  EXPECT_EQ(0, std::memcmp(&retained_before, &adapter.runtime_->delta_,
                           sizeof(retained_before)));
  EXPECT_EQ(0, std::memcmp(&previous_before.u_w,
                           &adapter.runtime_->previous_final_port_.u_w,
                           sizeof(previous_before.u_w)));
  EXPECT_EQ(0, std::memcmp(&previous_before.u_delta,
                           &adapter.runtime_->previous_final_port_.u_delta,
                           sizeof(previous_before.u_delta)));

  std::shared_ptr<const PathTubePair> committed;
  ASSERT_TRUE(adapter.finalizePreparedPathTubePairCommit(preparation, committed));
  ASSERT_TRUE(committed);
  EXPECT_EQ(committed->path_owner.get(), owner.get());
  EXPECT_EQ(committed->active_profile.get(), profile.get());
}

TEST(PhaseOffsetMatchedAdapterTest,
     RawCurrentIntervalExcludingRetainedPortCannotCommitOneSidedOwner) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner =
      MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  ASSERT_TRUE(adapter.runtime_);
  constexpr std::uint64_t kRevision = 72U;
  const std::shared_ptr<const TubeProfile> profile =
      MakeOneSidedPreparedProfile(owner, kRevision);
  ASSERT_TRUE(profile);

  // Unlike the preceding fixture, zero is the actual retained port.  It is
  // outside raw and filtered [+.125, ...], so no connector may be invented.
  const double retained_before = adapter.runtime_->retainedDelta();
  ASSERT_DOUBLE_EQ(retained_before, 0.0);
  const phase_offset_core::PortCommand previous_before =
      adapter.runtime_->previousFinalPort();
  std::shared_ptr<PathTubePair> candidate(new PathTubePair());
  candidate->source_revision = kRevision;
  candidate->authority_session =
      adapter.authority_session_.load(std::memory_order_acquire);
  candidate->path_owner = owner;
  candidate->full_path_samples =
      std::make_shared<const MatchedAdapterPathSamples>(path.samples);
  candidate->active_profile = profile;
  candidate->epoch_status = MakeOneSidedEpochStatus(kRevision);
  candidate->captured_w0 = path.current.w;
  candidate->future_seam_w = 1.0;
  candidate->existing_future_horizon_end_w = 2.4;
  candidate->captured_retained_delta = retained_before;
  candidate->captured_previous_final_port = previous_before;
  PathTubePairTransaction transaction;
  transaction.candidate_pair = std::shared_ptr<const PathTubePair>(candidate);
  transaction.authority_session = candidate->authority_session;
  transaction.captured_retained_delta = retained_before;
  transaction.captured_previous_final_port = previous_before;
  PathTubePairCommitPreparation preparation;
  EXPECT_FALSE(adapter.preparePathTubePairCommit(
      transaction, path.current.w, Eigen::Vector3d(0.4, 0.1, 1.1),
      MakeGains(), kDt, std::shared_ptr<const plan_env::CloudOccupancySnapshot>(),
      preparation));
  EXPECT_FALSE(preparation.valid);
  EXPECT_FALSE(adapter.capturePathTubePair());
  EXPECT_EQ(0, std::memcmp(&retained_before, &adapter.runtime_->delta_,
                           sizeof(retained_before)));
  EXPECT_EQ(0, std::memcmp(&previous_before.u_w,
                           &adapter.runtime_->previous_final_port_.u_w,
                           sizeof(previous_before.u_w)));
  EXPECT_EQ(0, std::memcmp(&previous_before.u_delta,
                           &adapter.runtime_->previous_final_port_.u_delta,
                           sizeof(previous_before.u_delta)));
}

TEST(PhaseOffsetMatchedAdapterTest,
     ProductionQuinticFutureSeamHandoffCommitsWholeNewOwnerAndTube) {
  constexpr double kCapturedW0 = 0.40;
  constexpr double kFutureSeamW = 1.20;
  constexpr double kJoinW = 1.80;
  constexpr double kExistingHorizonEndW = 2.40;
  constexpr double kEndW = 3.00;
  // Deliberately omit the C2/segment boundaries.  The adapter must add the
  // immutable owner endpoints before TubeBuilder asks for per-cell proofs;
  // relying on this planner sample list alone would create cells crossing
  // the copied-prefix/connector/tail seams.
  const std::vector<double> sample_w{
      kCapturedW0, 0.80, 1.40, 1.60, 2.10, kExistingHorizonEndW,
      2.70, kEndW};

  const std::shared_ptr<const ContinuousPhasePath> old_owner =
      MakeH2ProductionOldOwner();
  const std::shared_ptr<const ContinuousPhasePath> new_owner =
      MakeH2ProductionNewOwner(old_owner, kCapturedW0, kFutureSeamW,
                               kJoinW, kEndW);
  ASSERT_TRUE(old_owner);
  ASSERT_TRUE(new_owner);

  // The production quintic preserves C2 at the future seam, while its strict
  // interior is not the old owner.  This is the geometric precondition that
  // makes old tube/sample reuse invalid.
  ContinuousPhasePathState old_seam;
  ContinuousPhasePathState new_seam;
  ASSERT_TRUE(old_owner->evaluate(kFutureSeamW, old_seam, false));
  ASSERT_TRUE(new_owner->evaluate(kFutureSeamW, new_seam, false));
  EXPECT_NEAR(0.0, (new_seam.p - old_seam.p).norm(), 1e-12);
  EXPECT_NEAR(0.0, (new_seam.dp_dw - old_seam.dp_dw).norm(), 1e-12);
  EXPECT_NEAR(0.0, (new_seam.d2p_dw2 - old_seam.d2p_dw2).norm(), 1e-12);
  ContinuousPhasePathState old_interior;
  ContinuousPhasePathState new_interior;
  ASSERT_TRUE(old_owner->evaluate(1.60, old_interior, false));
  ASSERT_TRUE(new_owner->evaluate(1.60, new_interior, false));
  ASSERT_GT((new_interior.p - old_interior.p).norm(), 1e-4);

  const MatchedAdapterPathSamples old_samples = SampleOwner(old_owner, sample_w);
  const MatchedAdapterPathSamples new_samples = SampleOwner(new_owner, sample_w);
  ASSERT_EQ(old_samples.size(), sample_w.size());
  ASSERT_EQ(new_samples.size(), sample_w.size());

  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  ASSERT_TRUE(adapter.runtime_);
  ContinuousPhasePathState captured_new_state;
  ASSERT_TRUE(new_owner->evaluate(kCapturedW0, captured_new_state, false));
  const Eigen::Vector3d position = captured_new_state.p;
  const guidance::IsfGains gains = MakeGains();
  const std::shared_ptr<const plan_env::CloudOccupancySnapshot> no_snapshot;

  // Install an old immutable pair first.  The second transaction must retain
  // it until its own atomic CAS; before that CAS no new owner is observable.
  PathTubePairTransaction old_transaction;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), old_owner, old_samples,
      kCapturedW0, kFutureSeamW, kExistingHorizonEndW, position, gains, kDt,
      no_snapshot, old_transaction));
  std::shared_ptr<const PathTubePair> old_pair;
  ASSERT_TRUE(PrepareAndFinalizePair(
      adapter, old_transaction, kCapturedW0, position, gains, kDt,
      no_snapshot, old_pair));
  ASSERT_TRUE(old_pair);
  ASSERT_EQ(adapter.capturePathTubePair().get(), old_pair.get());

  // The replacement's seam may be structurally legal even when the old
  // certificate ends earlier.  Only live w0 needs old-pair evidence; the
  // new C2 owner must prove the copied prefix through the future seam.
  std::shared_ptr<TubeProfile> lagging_old_profile(
      new TubeProfile(*old_pair->active_profile));
  lagging_old_profile->preview_end_w = 0.80;
  lagging_old_profile->requested_preview_end_w = 0.80;
  lagging_old_profile->certified_segment_end_w = 0.80;
  std::shared_ptr<PathTubePair> lagging_old_pair(new PathTubePair(*old_pair));
  lagging_old_pair->active_profile =
      std::shared_ptr<const TubeProfile>(lagging_old_profile);
  old_pair = std::shared_ptr<const PathTubePair>(lagging_old_pair);
  std::atomic_store(&adapter.authoritative_path_tube_pair_, old_pair);
  ASSERT_LT(old_pair->active_profile->certified_segment_end_w, kFutureSeamW);

  std::unique_ptr<PathTubePairPin> old_pin =
      adapter.captureAndAcquirePathTubePairPin();
  ASSERT_TRUE(old_pin);
  ASSERT_TRUE(old_pin->valid());
  // The pin freezes exact old-pair identity, not Runtime history.  A selected
  // command can advance the latter before the expensive new-owner build starts.
  adapter.runtime_->delta_ = 0.01;
  adapter.runtime_->previous_final_port_.u_w = 0.03;
  adapter.runtime_->previous_final_port_.u_delta = -0.02;
  const double rebased_delta = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand rebased_port =
      adapter.runtime_->previousFinalPort();
  EXPECT_NE(0, std::memcmp(&old_pin->capture().retained_delta, &rebased_delta,
                           sizeof(rebased_delta)));
  PathTubePairTransaction new_transaction;
  ASSERT_TRUE(adapter.stagePathTubePair(
      old_pair, new_owner, new_samples, kCapturedW0, kFutureSeamW,
      kExistingHorizonEndW, position, gains, kDt, no_snapshot,
      new_transaction, old_pin->capture().authority_session,
      &old_pin->capture(), old_pin->leaseId()));
  ASSERT_TRUE(new_transaction.candidate_pair);
  EXPECT_EQ(new_transaction.expected_pair.get(), old_pair.get());
  EXPECT_EQ(adapter.capturePathTubePair().get(), old_pair.get());
  EXPECT_EQ(0, std::memcmp(&rebased_delta,
                           &new_transaction.captured_retained_delta,
                           sizeof(rebased_delta)));
  EXPECT_EQ(0, std::memcmp(&rebased_port.u_w,
                           &new_transaction.captured_previous_final_port.u_w,
                           sizeof(rebased_port.u_w)));

  // Prepare is a distinct command-boundary proof.  It must rebase again
  // rather than reuse either the pin history or the stage-time bits.
  adapter.runtime_->delta_ = 0.015;
  adapter.runtime_->previous_final_port_.u_w = 0.04;
  adapter.runtime_->previous_final_port_.u_delta = -0.01;
  const double prepared_delta = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand prepared_port =
      adapter.runtime_->previousFinalPort();
  PathTubePairCommitPreparation preparation;
  ASSERT_TRUE(adapter.preparePathTubePairCommit(
      new_transaction, kCapturedW0, position, gains, kDt, no_snapshot,
      preparation));
  EXPECT_EQ(0, std::memcmp(&prepared_delta,
                           &preparation.expected_retained_delta,
                           sizeof(prepared_delta)));
  EXPECT_EQ(0, std::memcmp(&prepared_port.u_w,
                           &preparation.expected_previous_final_port.u_w,
                           sizeof(prepared_port.u_w)));
  EXPECT_EQ(adapter.capturePathTubePair().get(), old_pair.get());
  std::shared_ptr<const PathTubePair> committed_pair;
  ASSERT_TRUE(adapter.finalizePreparedPathTubePairCommit(
      preparation, committed_pair));
  old_pin->release();
  ASSERT_TRUE(committed_pair);
  EXPECT_EQ(adapter.capturePathTubePair().get(), committed_pair.get());
  EXPECT_NE(committed_pair.get(), old_pair.get());
  EXPECT_EQ(committed_pair->path_owner.get(), new_owner.get());
  EXPECT_NE(committed_pair->path_owner.get(), old_owner.get());
  ASSERT_TRUE(committed_pair->full_path_samples);
  ASSERT_TRUE(committed_pair->active_profile);
  EXPECT_NE(committed_pair->full_path_samples.get(), old_pair->full_path_samples.get());
  EXPECT_NE(committed_pair->active_profile.get(), old_pair->active_profile.get());
  EXPECT_EQ(committed_pair->source_revision,
            committed_pair->active_profile->source_revision);
  EXPECT_EQ(committed_pair->source_revision,
            committed_pair->epoch_status.candidate_path_source_revision);
  EXPECT_EQ(committed_pair->source_revision,
            committed_pair->epoch_status.active_path_source_revision);

  const TubeProfile& profile = *committed_pair->active_profile;
  EXPECT_TRUE(profile.complete);
  EXPECT_LE(profile.preview_start_w, kCapturedW0);
  EXPECT_GE(profile.preview_end_w, kExistingHorizonEndW);
  EXPECT_LE(profile.certified_segment_start_w, kCapturedW0);
  EXPECT_GE(profile.certified_segment_end_w, kExistingHorizonEndW);
  ASSERT_FALSE(profile.samples.empty());
  EXPECT_LE(profile.samples.front().w, kCapturedW0);
  EXPECT_GE(profile.samples.back().w, kExistingHorizonEndW);

  bool saw_future_seam_boundary = false;
  bool saw_connector_end_boundary = false;
  for (const phase_offset_navigation::TubeRawSample& sample : profile.samples) {
    saw_future_seam_boundary = saw_future_seam_boundary ||
        std::abs(sample.w - kFutureSeamW) <= 1e-10;
    saw_connector_end_boundary = saw_connector_end_boundary ||
        std::abs(sample.w - kJoinW) <= 1e-10;
  }
  EXPECT_TRUE(saw_future_seam_boundary);
  EXPECT_TRUE(saw_connector_end_boundary);

  bool saw_strict_connector_interior = false;
  for (const phase_offset_navigation::TubeRawSample& sample : profile.samples) {
    ContinuousPhasePathState owner_state;
    ASSERT_TRUE(committed_pair->frame_owner);
    ASSERT_TRUE(committed_pair->frame_owner->evaluatePathState(
        sample.w, owner_state));
    const phase_offset_core::PathDifferentialState owner_path =
        ConvertContinuousPhasePathStateForActive(owner_state, sample.w);
    phase_offset_core::PhaseOffsetGeometryState owner_geometry;
    phase_offset_core::GeometryEvaluator evaluator;
    ASSERT_TRUE(evaluator.evaluate(owner_path, owner_path.p, 0.0,
                                   owner_geometry));
    EXPECT_NEAR(0.0, (sample.p - owner_geometry.p).norm(), 1e-10);
    EXPECT_NEAR(0.0, (sample.N - owner_geometry.N).norm(), 1e-10);
    if (sample.w > kFutureSeamW + 1e-9 && sample.w < kJoinW - 1e-9) {
      ContinuousPhasePathState old_state;
      ASSERT_TRUE(old_owner->evaluate(sample.w, old_state, false));
      EXPECT_GT((sample.p - old_state.p).norm(), 1e-4);
      saw_strict_connector_interior = true;
    }
  }
  EXPECT_TRUE(saw_strict_connector_interior);

  const double retained_after = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand previous_after =
      adapter.runtime_->previousFinalPort();
  EXPECT_EQ(0, std::memcmp(&prepared_delta, &retained_after,
                           sizeof(prepared_delta)));
  EXPECT_EQ(0, std::memcmp(&prepared_port.u_w, &previous_after.u_w,
                           sizeof(prepared_port.u_w)));
  EXPECT_EQ(0, std::memcmp(&prepared_port.u_delta,
                           &previous_after.u_delta,
                           sizeof(prepared_port.u_delta)));
}

TEST(PhaseOffsetMatchedAdapterTest,
     AdvertisedSeededRecoveryUsesStagedSuccessorAcrossMultipleTicks) {
  const SyntheticPath path = MakeStraightSyntheticPath();
  PhaseOffsetMatchedAdapterConfig recovery_config =
      MakeManualConfig(TubeSource::FIXED);
  // Keep the recovery witness comfortably inside a broad, valid fixed Tube;
  // the test is about staged-owner continuity and publication, not an empty
  // local polygon caused by an intentionally razor-thin profile.
  recovery_config.tube.fixed_delta_max = 0.40;
  recovery_config.tube.interior_margin = 0.0;
  // Keep each ZOH recenter step below the seeded residual so the advertised
  // successor path remains authoritative across multiple RECOVERY ticks.
  recovery_config.u_delta_abs_max = 0.005;
  recovery_config.u_delta_rate_max = 5.0;
  PhaseOffsetMatchedAdapter adapter(recovery_config);
  ASSERT_TRUE(adapter.runtime_);
  const std::shared_ptr<const ContinuousPhasePath> old_owner =
      MakeStraightSyntheticOwner();
  ASSERT_TRUE(old_owner);
  const MatchedAdapterPathSamples old_samples = SampleOwner(
      old_owner, {0.4, 0.8, 1.0, 1.6, 2.0, 2.4, 2.8});
  const Eigen::Vector3d position(0.4, 0.1, 1.1);
  const guidance::IsfGains gains = MakeGains();
  const std::shared_ptr<const plan_env::CloudOccupancySnapshot> no_snapshot;
  // Production authority sessions are nonzero; seed the private lifecycle
  // token before constructing the bootstrap pair so RecoveryOwner can bind
  // every subsequent tick to one immutable session.
  adapter.authority_session_.store(1U, std::memory_order_release);

  PathTubePairTransaction bootstrap;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), old_owner, old_samples, 0.4,
      1.0, 2.4, position, gains, kDt, no_snapshot, bootstrap));
  std::shared_ptr<const PathTubePair> old_pair;
  ASSERT_TRUE(PrepareAndFinalizePair(
      adapter, bootstrap, 0.4, position, gains, kDt, no_snapshot, old_pair));
  ASSERT_TRUE(old_pair);

  // Seed a real nonzero predecessor through the ordinary unadvertised
  // Runtime path, then switch to the advertised owner contract.  This keeps
  // the test focused on the production RecoveryOwner transaction rather than
  // manufacturing a disconnected snapshot by hand.
  adapter.zero_gate_open_ = true;
  adapter.zero_gate_consecutive_count_ = 100;
  MatchedAdapterInput input = MakeInput(path, old_owner.get());
  input.path_state_query = [](const double w,
                              phase_offset_core::PathDifferentialState& state) {
    state = MakeStraightState(w);
    return true;
  };
  input.semantic_path_owner = old_owner;
  input.frame_owner = old_pair->frame_owner;
  input.semantic_path_start_w = old_owner->startW();
  input.semantic_path_end_w = old_owner->endW();
  input.path_tube_pair = old_pair;
  MatchedAdapterOutput bootstrap_output;
  const auto rebase_straight_input = [&]() {
    const phase_offset_navigation::ActiveReferenceSnapshot authority =
        adapter.execution_authority_.snapshot();
    if (!authority.valid || !std::isfinite(authority.proposed_next_w)) return;
    input.path = MakeStraightState(authority.proposed_next_w);
    ActiveAdapterInput zero_input;
    zero_input.path = input.path;
    zero_input.position = input.position;
    zero_input.gains = input.gains;
    PhaseOffsetActiveAdapter zero;
    ActiveAdapterOutput zero_output;
    EXPECT_TRUE(zero.evaluate(zero_input, zero_output));
    input.legacy = LegacyGuidanceSnapshot(
        zero_output.guidance.v_cmd, zero_output.guidance.w_dot,
        zero_output.guidance.e_parallel, zero_output.guidance.e_perp,
        zero_output.guidance.ref_pt, zero_output.guidance.tangent,
        zero_output.guidance.valid);
  };
  ASSERT_TRUE(adapter.update(input, bootstrap_output))
      << bootstrap_output.invalid_reason;
  ASSERT_TRUE(adapter.execution_authority_.snapshot().valid);
  adapter.runtime_->profile_elapsed_ = 0.20;
  // Build a residual larger than one bounded recovery increment while still
  // using the ordinary unadvertised Runtime owner as the predecessor.
  for (int index = 0; index < 8; ++index) {
    rebase_straight_input();
    ASSERT_TRUE(adapter.update(input, bootstrap_output))
        << bootstrap_output.invalid_reason;
  }
  MatchedAdapterOutput output;
  rebase_straight_input();
  ASSERT_TRUE(adapter.update(input, output)) << output.invalid_reason;
  ASSERT_GT(std::abs(adapter.runtime_->retainedDelta()), 1e-4);
  ASSERT_EQ(adapter.execution_authority_.snapshot().owner_mode,
            phase_offset_navigation::ActiveReferenceOwnerMode::NORMAL);

  adapter.advertised_ = true;
  adapter.execution_authority_.setTestOnlyRuntimeOwnerAllowed(false);
  std::unique_ptr<PathTubePairPin> pin =
      adapter.captureAndAcquirePathTubePairPin();
  ASSERT_TRUE(pin);
  ASSERT_TRUE(pin->valid());
  // H2 staging is bound to the exact authenticated predecessor command.  The
  // warm-up ticks above advance that predecessor away from the original
  // bootstrap anchor, so rebase the successor transaction's captured phase,
  // path state, and display position to the immutable authority snapshot.
  const phase_offset_navigation::ActiveReferenceSnapshot predecessor_authority =
      adapter.execution_authority_.snapshot();
  ASSERT_TRUE(predecessor_authority.valid);
  ASSERT_EQ(predecessor_authority.owner_mode,
            phase_offset_navigation::ActiveReferenceOwnerMode::NORMAL);
  const double replacement_captured_w0 = predecessor_authority.w;
  const double successor_end_w = 2.8;
  const std::shared_ptr<const ContinuousPhasePath> successor_owner =
      MakeH2ProductionNewOwner(old_owner, replacement_captured_w0,
                               1.0, 1.6, successor_end_w);
  ASSERT_TRUE(successor_owner);
  std::vector<double> successor_sample_w;
  for (int index = 0; index <= 6; ++index) {
    successor_sample_w.push_back(replacement_captured_w0 +
        (successor_end_w - replacement_captured_w0) *
            static_cast<double>(index) / 6.0);
  }
  const MatchedAdapterPathSamples successor_samples = SampleOwner(
      successor_owner, successor_sample_w);
  ASSERT_FALSE(successor_samples.empty());
  const phase_offset_core::PathDifferentialState replacement_path =
      MakeStraightState(replacement_captured_w0);
  const phase_offset_core::PathDifferentialState original_path =
      MakeStraightState(0.4);
  const Eigen::Vector3d replacement_position = replacement_path.p +
      (position - original_path.p);
  input.path = replacement_path;
  input.position = replacement_position;
  PhaseOffsetActiveAdapter replacement_zero;
  ActiveAdapterInput replacement_zero_input;
  replacement_zero_input.path = input.path;
  replacement_zero_input.position = input.position;
  replacement_zero_input.gains = input.gains;
  ActiveAdapterOutput replacement_zero_output;
  ASSERT_TRUE(replacement_zero.evaluate(replacement_zero_input,
                                        replacement_zero_output));
  input.legacy = LegacyGuidanceSnapshot(
      replacement_zero_output.guidance.v_cmd,
      replacement_zero_output.guidance.w_dot,
      replacement_zero_output.guidance.e_parallel,
      replacement_zero_output.guidance.e_perp,
      replacement_zero_output.guidance.ref_pt,
      replacement_zero_output.guidance.tangent,
      replacement_zero_output.guidance.valid);
  input.path_state_query = [](const double w,
                              phase_offset_core::PathDifferentialState& state) {
    state = MakeStraightState(w);
    return true;
  };
  PathTubePairTransaction replacement;
  ASSERT_TRUE(adapter.stagePathTubePair(
      old_pair, successor_owner, successor_samples, replacement_captured_w0,
      1.0, 2.4, replacement_position, gains, kDt, no_snapshot, replacement,
      pin->capture().authority_session, &pin->capture(), pin->leaseId()));
  ASSERT_TRUE(replacement.candidate_pair);

  // A successor with horizontal speed exactly at the single production
  // threshold is not a usable Horizontal-N frame.  This evidence must retain
  // the active nonzero predecessor and request renewed recovery evidence;
  // there is no neutral/centerline takeover and no pending PositionCommand.
  const auto degenerate_successor_owner = MakeHorizontalDegenerateOwner(
      replacement_captured_w0, successor_end_w,
      phase_offset_core::kHorizontalNormalSpeedEpsilon);
  ASSERT_TRUE(degenerate_successor_owner);
  auto degenerate_successor = std::make_shared<PathTubePair>(
      *replacement.candidate_pair);
  degenerate_successor->path_owner = degenerate_successor_owner;
  degenerate_successor->frame_owner =
      std::shared_ptr<const ContinuousPhaseNormalFrame>(
          new ContinuousPhaseNormalFrame(
              degenerate_successor_owner,
              degenerate_successor->path_revision,
              degenerate_successor->frame_revision));
  input.successor_path_tube_pair =
      std::shared_ptr<const PathTubePair>(degenerate_successor);
  rebase_straight_input();
  const double retained_before_degenerate =
      adapter.runtime_->retainedDelta();
  const phase_offset_navigation::ActiveReferenceSnapshot authority_before_degenerate =
      adapter.execution_authority_.snapshot();
  MatchedAdapterOutput degenerate_output;
  EXPECT_FALSE(adapter.update(input, degenerate_output));
  EXPECT_TRUE(degenerate_output.recovery_replan_required)
      << degenerate_output.invalid_reason;
  EXPECT_EQ(degenerate_output.recovery_status,
            phase_offset_navigation::RecoveryStepStatus::
                RECOVERY_REPLAN_REQUIRED);
  EXPECT_FALSE(degenerate_output.selected);
  EXPECT_FALSE(degenerate_output.valid);
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(),
                   retained_before_degenerate);
  const phase_offset_navigation::ActiveReferenceSnapshot authority_after_degenerate =
      adapter.execution_authority_.snapshot();
  EXPECT_EQ(authority_after_degenerate.owner_mode,
            phase_offset_navigation::ActiveReferenceOwnerMode::NORMAL);
  EXPECT_EQ(authority_after_degenerate.sequence,
            authority_before_degenerate.sequence);
  EXPECT_FALSE(authority_after_degenerate.planner_invalid);
  EXPECT_FALSE(authority_after_degenerate.current_state_unsafe);
  EXPECT_FALSE(adapter.hasPendingPositionCommand());
  input.successor_path_tube_pair.reset();

  phase_offset_core::NormalFrameQuery successor_start_frame;
  ASSERT_TRUE(replacement.candidate_pair->frame_owner->query(
      replacement_captured_w0, successor_start_frame));
  EXPECT_NEAR(successor_start_frame.N.z(), 0.0, 1e-12);
  phase_offset_core::NormalFrameQuery seed_a;
  phase_offset_core::NormalFrameQuery seed_b;
  const ContinuousPhaseNormalFrame frame_a(
      replacement.candidate_pair->path_owner,
      replacement.candidate_pair->path_revision,
      replacement.candidate_pair->frame_revision, Eigen::Vector3d::UnitX());
  const ContinuousPhaseNormalFrame frame_b(
      replacement.candidate_pair->path_owner,
      replacement.candidate_pair->path_revision,
      replacement.candidate_pair->frame_revision, Eigen::Vector3d::UnitY());
  ASSERT_TRUE(frame_a.query(replacement_captured_w0, seed_a));
  ASSERT_TRUE(frame_b.query(replacement_captured_w0, seed_b));
  EXPECT_NEAR((seed_a.N - seed_b.N).norm(), 0.0, 1e-12);
  EXPECT_NEAR((seed_a.N_w - seed_b.N_w).norm(), 0.0, 1e-12);
  ContinuousPhasePathState successor_start_state;
  ContinuousPhasePathState successor_future_state;
  ASSERT_TRUE(successor_owner->evaluate(
      replacement_captured_w0, successor_start_state, false));
  ASSERT_TRUE(successor_owner->evaluate(1.3, successor_future_state, false));
  EXPECT_LT(successor_start_state.dp_dw.normalized().dot(
                successor_future_state.dp_dw.normalized()), 0.999999);
  EXPECT_EQ(adapter.capturePathTubePair().get(), old_pair.get());
  // A staged successor whose immutable frame fails the continuation seam
  // proof must retain the current nonzero authority and surface a renewed
  // recovery/replan request; no base-only seam fallback may publish.
  auto bad_successor = std::make_shared<PathTubePair>(*replacement.candidate_pair);
  bad_successor->frame_owner = std::shared_ptr<const ContinuousPhaseNormalFrame>(
      new ContinuousPhaseNormalFrame(
          bad_successor->path_owner, bad_successor->path_revision,
          bad_successor->frame_revision + 1U));
  input.successor_path_tube_pair = std::shared_ptr<const PathTubePair>(bad_successor);
  rebase_straight_input();
  MatchedAdapterOutput rejected_successor_output;
  const double retained_before_rejected_successor =
      adapter.runtime_->retainedDelta();
  EXPECT_FALSE(adapter.update(input, rejected_successor_output));
  EXPECT_TRUE(rejected_successor_output.recovery_replan_required)
      << rejected_successor_output.invalid_reason;
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(),
                   retained_before_rejected_successor);
  EXPECT_EQ(adapter.execution_authority_.snapshot().owner_mode,
            phase_offset_navigation::ActiveReferenceOwnerMode::NORMAL);
  EXPECT_FALSE(adapter.hasPendingPositionCommand());
  input.successor_path_tube_pair = replacement.candidate_pair;
  // Exercise the ZERO_ONLY handoff branch once with a deliberately narrow
  // staged target.  The current tick must continue projecting in the old
  // owner's safe component; target bounds are evidence for lifecycle only.
  auto zero_profile = std::make_shared<TubeProfile>(
      *replacement.candidate_pair->active_profile);
  zero_profile->classification =
      phase_offset_navigation::TubeProfileClassification::
          ZERO_ONLY_PLANNER_BASELINE;
  for (TubeRawSample& sample : zero_profile->samples) {
    sample.filtered_lower = -1e-4;
    sample.filtered_upper = 1e-4;
    sample.raw_lower = sample.filtered_lower;
    sample.raw_upper = sample.filtered_upper;
  }
  auto zero_pair = std::make_shared<PathTubePair>(*replacement.candidate_pair);
  zero_pair->active_profile = std::shared_ptr<const TubeProfile>(zero_profile);
  input.successor_path_tube_pair =
      std::shared_ptr<const PathTubePair>(zero_pair);
  rebase_straight_input();
  ASSERT_TRUE(adapter.update(input, output)) << output.invalid_reason;
  ASSERT_TRUE(output.selected);
  phase_offset_navigation::TubeBounds old_owner_bounds;
  ASSERT_TRUE(phase_offset_navigation::TubeFilter::query(
      *old_pair->active_profile, input.path.w, old_owner_bounds));
  EXPECT_TRUE(output.tube_current_bounds.valid);
  EXPECT_NEAR(output.tube_current_bounds.lower, old_owner_bounds.lower, 1e-12);
  EXPECT_NEAR(output.tube_current_bounds.upper, old_owner_bounds.upper, 1e-12);
  EXPECT_FALSE(output.tube_current_bounds.lower >= -1e-4 &&
              output.tube_current_bounds.upper <= 1e-4);
  const PendingPositionCommandCapture zero_capture =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(zero_capture.pending);
  EXPECT_EQ(adapter.pending_recovery_source_pair_.get(), old_pair.get());
  EXPECT_EQ(adapter.pending_recovery_execution_pair_.get(), old_pair.get());
  ASSERT_TRUE(adapter.publishPendingPositionCommand(
      []() { return true; }, zero_capture.identity));
  input.successor_path_tube_pair = replacement.candidate_pair;

  std::size_t recovery_ticks = 0U;
  bool clone_negative_checked = false;
  double first_delta = std::abs(adapter.runtime_->retainedDelta());
  std::vector<double> delta_history;
  std::vector<std::uint64_t> executed_revision_history;
  bool saw_target_revision = false;
  bool terminal = false;
  for (; recovery_ticks < 64U; ++recovery_ticks) {
    rebase_straight_input();
    ASSERT_TRUE(adapter.update(input, output)) << output.invalid_reason;
    ASSERT_TRUE(output.selected) << output.invalid_reason;
    const PendingPositionCommandCapture capture =
        adapter.capturePendingPositionCommand();
    ASSERT_TRUE(capture.pending);
    ASSERT_TRUE(capture.valid);
    PendingPositionCommandCapture publish_capture = capture;
    if (!clone_negative_checked &&
        adapter.pending_recovery_execution_pair_.get() ==
            replacement.candidate_pair.get()) {
      EXPECT_EQ(adapter.pending_recovery_source_pair_.get(), old_pair.get());
      EXPECT_EQ(adapter.pending_recovery_target_pair_.get(),
                replacement.candidate_pair.get());
      // A same-revision clone in the live slot is not the immutable staged
      // target that produced this candidate and must fail before publication.
      const std::shared_ptr<const PathTubePair> target_clone(
          new PathTubePair(*replacement.candidate_pair));
      std::atomic_store(&adapter.authoritative_path_tube_pair_, target_clone);
      int callback_count = 0;
      EXPECT_FALSE(adapter.publishPendingPositionCommand(
          [&callback_count]() {
            ++callback_count;
            return true;
          }, capture.identity));
      EXPECT_EQ(callback_count, 0);
      EXPECT_FALSE(adapter.hasPendingPositionCommand());
      EXPECT_EQ(adapter.capturePathTubePair().get(), target_clone.get());
      std::atomic_store(&adapter.authoritative_path_tube_pair_, old_pair);

      // Re-stage the exact same immutable target and prove the positive
      // staged-successor path still publishes after the negative clone.
      rebase_straight_input();
      ASSERT_TRUE(adapter.update(input, output)) << output.invalid_reason;
      const PendingPositionCommandCapture restaged =
          adapter.capturePendingPositionCommand();
      ASSERT_TRUE(restaged.pending);
      ASSERT_TRUE(restaged.valid);
      EXPECT_EQ(adapter.pending_recovery_execution_pair_.get(),
                replacement.candidate_pair.get());
      publish_capture = restaged;
      clone_negative_checked = true;
    }
    ASSERT_TRUE(adapter.publishPendingPositionCommand(
        []() { return true; }, publish_capture.identity));
    const auto authority = adapter.execution_authority_.snapshot();
    delta_history.push_back(adapter.runtime_->retainedDelta());
    executed_revision_history.push_back(authority.executed_path_revision);
    if (authority.owner_mode ==
            phase_offset_navigation::ActiveReferenceOwnerMode::RECOVERY) {
      saw_target_revision = saw_target_revision ||
          authority.executed_path_revision ==
              replacement.candidate_pair->path_revision;
      EXPECT_LT(std::abs(adapter.runtime_->retainedDelta()), first_delta +
                1e-9);
      first_delta = std::abs(adapter.runtime_->retainedDelta());
      continue;
    }
    terminal = authority.owner_mode ==
        phase_offset_navigation::ActiveReferenceOwnerMode::PLANNER_ONLY;
    break;
  }
  EXPECT_TRUE(terminal);
  EXPECT_TRUE(clone_negative_checked);
  EXPECT_GT(recovery_ticks, 1U);
  EXPECT_TRUE(saw_target_revision);
  EXPECT_NE(std::find(executed_revision_history.begin(),
                      executed_revision_history.end(),
                      replacement.candidate_pair->path_revision),
            executed_revision_history.end());
  ASSERT_FALSE(delta_history.empty());
  for (std::size_t index = 1U; index < delta_history.size(); ++index) {
    EXPECT_LE(std::abs(delta_history[index]),
              std::abs(delta_history[index - 1U]) + 1e-9);
  }
  EXPECT_EQ(adapter.execution_authority_.snapshot().owner_mode,
            phase_offset_navigation::ActiveReferenceOwnerMode::PLANNER_ONLY);
  EXPECT_FALSE(adapter.runtime_->hasExecutedOffsetAuthority());
  EXPECT_FALSE(adapter.requiresAuthoritativeOffsetHandoff());
  pin->release();
}

TEST(PhaseOffsetMatchedAdapterTest,
     NullPairCommitRejectsExecutedRuntimeChangedAfterStage) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;
  PathTubePairTransaction transaction;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples, path.current.w,
      1.0, 2.4, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), transaction));
  const double staged_delta = transaction.captured_retained_delta;
  const phase_offset_core::PortCommand staged_port =
      transaction.captured_previous_final_port;
  adapter.runtime_->delta_ = 0.01;
  adapter.runtime_->previous_final_port_.u_w = 0.03;
  adapter.runtime_->previous_final_port_.u_delta = -0.02;
  const double latest_delta = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand latest_port =
      adapter.runtime_->previousFinalPort();
  std::shared_ptr<const PathTubePair> committed;
  EXPECT_FALSE(PrepareAndFinalizePair(
      adapter, transaction, path.current.w, input.position, input.gains,
      input.dt, std::shared_ptr<const plan_env::CloudOccupancySnapshot>(),
      committed));
  EXPECT_FALSE(committed);
  EXPECT_NE(0, std::memcmp(&staged_delta, &latest_delta,
                           sizeof(latest_delta)));
  EXPECT_NE(0, std::memcmp(&staged_port.u_w, &latest_port.u_w,
                           sizeof(latest_port.u_w)));
  EXPECT_EQ(0, std::memcmp(&latest_delta, &adapter.runtime_->delta_,
                           sizeof(latest_delta)));
  EXPECT_EQ(0, std::memcmp(&latest_port.u_w,
                           &adapter.runtime_->previous_final_port_.u_w,
                           sizeof(latest_port.u_w)));
  EXPECT_EQ(0, std::memcmp(&latest_port.u_delta,
                           &adapter.runtime_->previous_final_port_.u_delta,
                           sizeof(latest_port.u_delta)));
  EXPECT_FALSE(adapter.capturePathTubePair());
}

TEST(PhaseOffsetMatchedAdapterTest,
     PathTubePairCommitRejectsRuntimeChangedAfterPreparation) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;
  PathTubePairTransaction transaction;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples, path.current.w,
      1.0, 2.4, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), transaction));
  PathTubePairCommitPreparation preparation;
  ASSERT_TRUE(adapter.preparePathTubePairCommit(
      transaction, path.current.w, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), preparation));
  const double expected_delta = preparation.expected_retained_delta;
  const phase_offset_core::PortCommand expected_port =
      preparation.expected_previous_final_port;
  adapter.runtime_->delta_ = expected_delta + 0.01;
  adapter.runtime_->previous_final_port_.u_w = expected_port.u_w + 0.01;
  std::shared_ptr<const PathTubePair> committed;
  EXPECT_FALSE(adapter.finalizePreparedPathTubePairCommit(preparation,
                                                          committed));
  EXPECT_FALSE(committed);
  EXPECT_FALSE(adapter.capturePathTubePair());
  EXPECT_EQ(0, std::memcmp(&expected_delta,
                           &preparation.expected_retained_delta,
                           sizeof(expected_delta)));
  EXPECT_EQ(0, std::memcmp(&expected_port.u_w,
                           &preparation.expected_previous_final_port.u_w,
                           sizeof(expected_port.u_w)));
}

TEST(PhaseOffsetMatchedAdapterTest,
     PreparedPathTubePairRejectsRealtimePhaseAtFutureSeam) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;
  PathTubePairTransaction transaction;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples, path.current.w,
      1.0, 2.4, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), transaction));
  const double retained_before = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      adapter.runtime_->previousFinalPort();
  const std::shared_ptr<const PathTubePair> authority_before =
      adapter.capturePathTubePair();
  std::shared_ptr<const PathTubePair> committed;
  EXPECT_FALSE(PrepareAndFinalizePair(
      adapter, transaction, 1.0, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), committed));
  EXPECT_FALSE(adapter.capturePathTubePair());
  EXPECT_EQ(authority_before.get(), adapter.capturePathTubePair().get());
  EXPECT_EQ(0, std::memcmp(&retained_before, &adapter.runtime_->delta_,
                           sizeof(retained_before)));
  EXPECT_EQ(0, std::memcmp(&previous_before.u_w,
                           &adapter.runtime_->previous_final_port_.u_w,
                           sizeof(previous_before.u_w)));
  EXPECT_EQ(0, std::memcmp(&previous_before.u_delta,
                           &adapter.runtime_->previous_final_port_.u_delta,
                           sizeof(previous_before.u_delta)));
}

TEST(PhaseOffsetMatchedAdapterTest,
     PreparedPathTubePairRejectsChangedExpectedAuthority) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;
  PathTubePairTransaction transaction;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples, path.current.w,
      1.0, 2.4, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), transaction));
  std::shared_ptr<PathTubePair> other(new PathTubePair());
  other->source_revision = 99U;
  std::atomic_store(&adapter.authoritative_path_tube_pair_,
                    std::shared_ptr<const PathTubePair>(other));
  std::shared_ptr<const PathTubePair> committed;
  EXPECT_FALSE(PrepareAndFinalizePair(
      adapter, transaction, path.current.w, input.position, input.gains,
      input.dt, std::shared_ptr<const plan_env::CloudOccupancySnapshot>(),
      committed));
}

TEST(PhaseOffsetMatchedAdapterTest,
     RetiredAuthorityRejectsCapturedPairUpdateAndOldCommit) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;
  PathTubePairTransaction transaction;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples, path.current.w,
      1.0, 2.4, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), transaction));
  std::shared_ptr<const PathTubePair> installed;
  ASSERT_TRUE(PrepareAndFinalizePair(
      adapter, transaction, path.current.w, input.position, input.gains,
      input.dt, std::shared_ptr<const plan_env::CloudOccupancySnapshot>(),
      installed));
  ASSERT_TRUE(installed);
  input.path_tube_pair = installed;

  // Retirement deliberately preserves Runtime history, while removing pair
  // and timer authority.  A stale captured pair must not change either one.
  ASSERT_TRUE(adapter.runtime_);
  adapter.runtime_->delta_ = 0.0125;
  adapter.runtime_->previous_final_port_.u_w = 0.31;
  adapter.runtime_->previous_final_port_.u_delta = -0.27;

  const std::uint64_t retired = adapter.retirePathTubeAuthority(
      installed->authority_session + 1U);
  EXPECT_GT(retired, installed->authority_session);
  const double retained_before = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      adapter.runtime_->previousFinalPort();
  const std::uint64_t control_sequence_before = adapter.control_sequence_;
  const std::shared_ptr<const TubeBuildRequest> request_before =
      std::atomic_load(&adapter.latest_build_request_);
  EXPECT_FALSE(request_before);

  MatchedAdapterOutput output;
  EXPECT_FALSE(adapter.update(input, output));
  EXPECT_EQ(output.invalid_reason, "captured path-tube pair retired");
  EXPECT_FALSE(adapter.capturePathTubePair());
  const double retained_after = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand previous_after =
      adapter.runtime_->previousFinalPort();
  EXPECT_EQ(0, std::memcmp(&retained_before, &retained_after,
                           sizeof(retained_before)));
  EXPECT_EQ(0, std::memcmp(&previous_before.u_w, &previous_after.u_w,
                           sizeof(previous_before.u_w)));
  EXPECT_EQ(0, std::memcmp(&previous_before.u_delta,
                           &previous_after.u_delta,
                           sizeof(previous_before.u_delta)));
  EXPECT_EQ(adapter.control_sequence_, control_sequence_before);
  const std::shared_ptr<const TubeBuildRequest> request_after =
      std::atomic_load(&adapter.latest_build_request_);
  EXPECT_EQ(request_after.get(), request_before.get());

  // A staged transaction from the retired session cannot become current.
  PathTubePairCommitPreparation preparation;
  EXPECT_FALSE(adapter.preparePathTubePairCommit(
      transaction, path.current.w, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), preparation));
}

TEST(PhaseOffsetMatchedAdapterTest,
     NewTaskResetAtomicallyNeutralizesAndRequiresFreshBootstrap) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;
  PathTubePairTransaction transaction;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples, path.current.w,
      1.0, 2.4, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), transaction));
  std::shared_ptr<const PathTubePair> old_pair;
  ASSERT_TRUE(PrepareAndFinalizePair(
      adapter, transaction, path.current.w, input.position, input.gains,
      input.dt, std::shared_ptr<const plan_env::CloudOccupancySnapshot>(),
      old_pair));
  ASSERT_TRUE(old_pair);

  // Populate every command/timer exposure that belongs to the old task.
  input.path_tube_pair = old_pair;
  MatchedAdapterOutput output;
  EXPECT_FALSE(adapter.update(input, output));
  std::atomic_store(&adapter.latest_candidate_epoch_snapshot_,
                    std::shared_ptr<const TubeEpochSnapshot>(
                        new TubeEpochSnapshot()));
  std::atomic_store(&adapter.latest_epoch_snapshot_,
                    std::shared_ptr<const TubeEpochSnapshot>(
                        new TubeEpochSnapshot()));
  std::atomic_store(&adapter.latest_control_snapshot_,
                    std::shared_ptr<const ControlPublishSnapshot>(
                        new ControlPublishSnapshot()));
  adapter.zero_gate_open_ = true;
  adapter.zero_gate_consecutive_count_ = 100;
  adapter.source_identity_ = owner.get();
  adapter.source_revision_ = 77U;
  adapter.have_source_identity_ = true;
  adapter.have_preflight_revision_ = true;
  adapter.last_preflight_revision_ = 77U;
  adapter.runtime_->delta_ = 0.02;
  adapter.runtime_->previous_final_port_.u_w = 0.03;
  adapter.runtime_->previous_final_port_.u_delta = -0.04;
  adapter.runtime_->profile_elapsed_ = 0.8;
  adapter.runtime_->profile_started_ = true;
  adapter.runtime_->profile_completed_ = false;

  const std::uint64_t old_session = old_pair->authority_session;
  std::uint64_t reset_session = 0U;
  ASSERT_TRUE(adapter.resetForNewNavigationTask(old_session, reset_session));
  EXPECT_GT(reset_session, old_session);
  EXPECT_EQ(reset_session,
            adapter.authority_session_.load(std::memory_order_acquire));
  EXPECT_FALSE(adapter.capturePathTubePair());
  EXPECT_FALSE(std::atomic_load(&adapter.latest_build_request_));
  EXPECT_FALSE(std::atomic_load(&adapter.latest_candidate_epoch_snapshot_));
  EXPECT_FALSE(std::atomic_load(&adapter.latest_epoch_snapshot_));
  EXPECT_FALSE(std::atomic_load(&adapter.latest_control_snapshot_));
  EXPECT_FALSE(adapter.have_source_identity_);
  EXPECT_FALSE(adapter.have_preflight_revision_);
  EXPECT_EQ(77U, adapter.source_revision_);
  EXPECT_FALSE(adapter.zero_gate_open_);
  EXPECT_EQ(0, adapter.zero_gate_consecutive_count_);
  EXPECT_DOUBLE_EQ(0.0, adapter.runtime_->retainedDelta());
  EXPECT_DOUBLE_EQ(0.0, adapter.runtime_->previousFinalPort().u_w);
  EXPECT_DOUBLE_EQ(0.0, adapter.runtime_->previousFinalPort().u_delta);
  EXPECT_FALSE(adapter.runtime_->hasExecutedOffsetAuthority());

  // The old captured owner cannot revive a retired task.  A new planner-only
  // owner remains neutral until its fresh Pair is installed after warm-up.
  EXPECT_FALSE(adapter.update(input, output));
  EXPECT_EQ("captured path-tube pair retired", output.invalid_reason);
  input.path_tube_pair.reset();
  EXPECT_FALSE(adapter.update(input, output));
  EXPECT_FALSE(output.selected);
  EXPECT_DOUBLE_EQ(0.0, adapter.runtime_->retainedDelta());
  for (int cycle = 1; cycle < 100; ++cycle) {
    EXPECT_FALSE(adapter.update(MakeInput(path, owner.get(), cycle * kDt),
                                output));
  }
  // Z1 requires a fresh certified sidecar epoch before the new neutral task
  // may request an offset bootstrap; warm-up alone is deliberately not one.
  ASSERT_TRUE(adapter.timerTick());
  EXPECT_TRUE(adapter.requiresPathTubePairBootstrap());
  EXPECT_FALSE(adapter.runtime_->hasExecutedOffsetAuthority());

  PathTubePairTransaction fresh_transaction;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples, path.current.w,
      1.0, 2.4, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(),
      fresh_transaction, reset_session));
  std::shared_ptr<const PathTubePair> fresh_pair;
  ASSERT_TRUE(PrepareAndFinalizePair(
      adapter, fresh_transaction, path.current.w, input.position, input.gains,
      input.dt, std::shared_ptr<const plan_env::CloudOccupancySnapshot>(),
      fresh_pair));
  ASSERT_TRUE(fresh_pair);
  input.path_tube_pair = fresh_pair;
  EXPECT_TRUE(adapter.update(input, output));
  EXPECT_TRUE(output.selected);
  EXPECT_TRUE(adapter.runtime_->hasExecutedOffsetAuthority());
}

TEST(PhaseOffsetMatchedAdapterTest,
     NewTaskResetRejectsStaleSessionAndDoesNotClearFatalLatch) {
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  ASSERT_TRUE(adapter.runtime_);
  adapter.runtime_->delta_ = 0.0125;
  adapter.latchFailure(
      phase_offset_navigation::ControlFailureReason::ZERO_PORT_EQUIVALENCE);
  const std::uint64_t current =
      adapter.authority_session_.load(std::memory_order_acquire);
  std::uint64_t rejected_session = 999U;
  EXPECT_FALSE(adapter.resetForNewNavigationTask(current + 1U,
                                                  rejected_session));
  EXPECT_EQ(0U, rejected_session);
  EXPECT_EQ(current,
            adapter.authority_session_.load(std::memory_order_acquire));
  EXPECT_DOUBLE_EQ(0.0125, adapter.runtime_->retainedDelta());
  EXPECT_TRUE(adapter.failure_latched_);

  std::uint64_t reset_session = 0U;
  ASSERT_TRUE(adapter.resetForNewNavigationTask(current, reset_session));
  EXPECT_GT(reset_session, current);
  EXPECT_DOUBLE_EQ(0.0, adapter.runtime_->retainedDelta());
  EXPECT_TRUE(adapter.failure_latched_);
  EXPECT_EQ(phase_offset_navigation::ControlFailureReason::ZERO_PORT_EQUIVALENCE,
            adapter.control_failure_reason_);
}

TEST(PhaseOffsetMatchedAdapterTest,
     NewTaskResetDefersTimerStateRetirementAndPreventsRevisionCollision) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> old_owner =
      MakeSyntheticOwner();
  const std::shared_ptr<const ContinuousPhasePath> new_owner =
      MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput old_input = MakeInput(path, old_owner.get());
  old_input.semantic_path_owner = old_owner;
  MatchedAdapterOutput output;

  // Establish real persistent timer state for task A.  The second command
  // consumes the completed epoch, while the manager/cache/profile remain
  // timer-owned objects.
  BuildAndConsume(adapter, old_input, output);
  const std::shared_ptr<const TubeBuildRequest> old_request =
      std::atomic_load(&adapter.latest_build_request_);
  const std::shared_ptr<const TubeEpochSnapshot> old_epoch =
      std::atomic_load(&adapter.latest_epoch_snapshot_);
  ASSERT_TRUE(old_request);
  ASSERT_TRUE(old_epoch);
  ASSERT_TRUE(adapter.tube_epoch_manager_);
  ASSERT_TRUE(adapter.timer_active_profile_);
  ASSERT_TRUE(adapter.have_cached_path_);
  const auto* const old_manager = adapter.tube_epoch_manager_.get();
  const auto* const old_profile = adapter.timer_active_profile_.get();
  const std::uint64_t old_revision = old_request->source_revision;
  const std::uint64_t old_task_generation = old_request->task_generation;

  // Build a real old-task H2 replacement preparation while holding the exact
  // pin.  It must fail final CAS after the task reset, even though the pin
  // object itself survives until its owner releases it.
  PathTubePairTransaction bootstrap;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), old_owner, path.samples,
      path.current.w, 1.0, 2.4, old_input.position, old_input.gains,
      old_input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), bootstrap));
  std::shared_ptr<const PathTubePair> old_pair;
  ASSERT_TRUE(PrepareAndFinalizePair(
      adapter, bootstrap, path.current.w, old_input.position, old_input.gains,
      old_input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), old_pair));
  ASSERT_TRUE(old_pair);
  std::unique_ptr<PathTubePairPin> old_pin =
      adapter.captureAndAcquirePathTubePairPin();
  ASSERT_TRUE(old_pin);
  PathTubePairTransaction replacement;
  ASSERT_TRUE(adapter.stagePathTubePair(
      old_pair, new_owner, path.samples, path.current.w, 1.0, 2.4,
      old_input.position, old_input.gains, old_input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), replacement,
      old_pin->capture().authority_session, &old_pin->capture(),
      old_pin->leaseId()));
  PathTubePairCommitPreparation old_preparation;
  ASSERT_TRUE(adapter.preparePathTubePairCommit(
      replacement, path.current.w, old_input.position, old_input.gains,
      old_input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(),
      old_preparation));
  TubeEpochSnapshot old_built;
  ASSERT_TRUE(adapter.buildTubeEpoch(old_request, old_built));

  const std::uint64_t old_session = old_pair->authority_session;
  std::uint64_t reset_session = 0U;
  ASSERT_TRUE(adapter.resetForNewNavigationTask(old_session, reset_session));
  const std::uint64_t new_task_generation =
      adapter.task_generation_.load(std::memory_order_acquire);
  EXPECT_EQ(old_task_generation + 1U, new_task_generation);
  EXPECT_EQ(old_manager, adapter.tube_epoch_manager_.get());
  EXPECT_EQ(old_profile, adapter.timer_active_profile_.get());
  EXPECT_EQ(old_task_generation, adapter.timer_task_generation_);
  EXPECT_EQ(old_revision + 1U, adapter.source_revision_);

  // A stale reset is a complete no-op, and neither an old timer completion
  // nor the prepared old-pin CAS can repopulate/revive A authority.
  std::uint64_t stale_retired = 99U;
  EXPECT_FALSE(adapter.resetForNewNavigationTask(old_session, stale_retired));
  EXPECT_EQ(0U, stale_retired);
  EXPECT_EQ(new_task_generation,
            adapter.task_generation_.load(std::memory_order_acquire));
  EXPECT_FALSE(adapter.finalizeTubeEpoch(old_request, old_built));
  std::shared_ptr<const PathTubePair> stale_committed;
  EXPECT_FALSE(adapter.finalizePreparedPathTubePairCommit(
      old_preparation, stale_committed));
  EXPECT_FALSE(stale_committed);
  old_pin->release();

  // Task B has the same geometric samples but a different owner.  Its source
  // revision cannot collide with A, and the first timer tick must consume the
  // new task token before it builds B, replacing every persistent object.
  MatchedAdapterInput new_input = MakeInput(path, new_owner.get(), kDt);
  new_input.semantic_path_owner = new_owner;
  EXPECT_FALSE(adapter.update(new_input, output));
  EXPECT_FALSE(output.candidate_profile);
  EXPECT_FALSE(output.active_profile);
  const std::shared_ptr<const TubeBuildRequest> new_request =
      std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(new_request);
  EXPECT_EQ(new_task_generation, new_request->task_generation);
  EXPECT_GT(new_request->source_revision, old_revision);
  ASSERT_TRUE(adapter.timerTick());
  const std::shared_ptr<const TubeEpochSnapshot> new_epoch =
      std::atomic_load(&adapter.latest_epoch_snapshot_);
  ASSERT_TRUE(new_epoch);
  ASSERT_TRUE(adapter.timer_active_profile_);
  EXPECT_EQ(new_task_generation, adapter.timer_task_generation_);
  EXPECT_NE(old_profile, adapter.timer_active_profile_.get());
  EXPECT_EQ(new_task_generation, new_epoch->task_generation);
  EXPECT_EQ(new_request->source_revision, new_epoch->source_revision);
  EXPECT_EQ(1U, new_epoch->epoch_status.active_tube_epoch);
  EXPECT_EQ(1U, new_epoch->epoch_status.install_count);
  EXPECT_EQ(new_request->source_revision,
            adapter.cached_path_source_revision_);
}

TEST(PhaseOffsetMatchedAdapterTest,
     NewTaskResetLinearizesAgainstPausedOldTimerPublication) {
  const SyntheticPath path = MakePath();
  int identity = 401;
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  EXPECT_FALSE(adapter.update(MakeInput(path, &identity), output));
  const std::shared_ptr<const TubeBuildRequest> old_request =
      std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(old_request);
  TubeEpochSnapshot old_built;
  ASSERT_TRUE(adapter.buildTubeEpoch(old_request, old_built));

  std::mutex hook_mutex;
  std::condition_variable hook_cv;
  bool finalize_checked = false;
  bool release_finalize = false;
  adapter.finalize_publication_test_hook_ = [&]() {
    std::unique_lock<std::mutex> lock(hook_mutex);
    finalize_checked = true;
    hook_cv.notify_all();
    hook_cv.wait(lock, [&]() { return release_finalize; });
  };
  bool finalized = false;
  std::thread finalizer([&]() {
    finalized = adapter.finalizeTubeEpoch(old_request, old_built);
  });
  {
    std::unique_lock<std::mutex> lock(hook_mutex);
    hook_cv.wait(lock, [&]() { return finalize_checked; });
  }

  std::atomic<bool> reset_started(false);
  std::atomic<bool> reset_returned(false);
  std::uint64_t retired = 0U;
  const std::uint64_t old_session =
      adapter.authority_session_.load(std::memory_order_acquire);
  std::thread resetter([&]() {
    reset_started.store(true, std::memory_order_release);
    reset_returned.store(adapter.resetForNewNavigationTask(old_session, retired),
                         std::memory_order_release);
  });
  while (!reset_started.load(std::memory_order_acquire)) std::this_thread::yield();
  // The reset is blocked at the publication barrier; it cannot clear slots
  // between finalization's check and its stores, then let old evidence refill.
  EXPECT_FALSE(reset_returned.load(std::memory_order_acquire));
  {
    std::lock_guard<std::mutex> lock(hook_mutex);
    release_finalize = true;
  }
  hook_cv.notify_all();
  finalizer.join();
  resetter.join();
  adapter.finalize_publication_test_hook_ = std::function<void()>();

  EXPECT_TRUE(finalized);
  EXPECT_TRUE(reset_returned.load(std::memory_order_acquire));
  EXPECT_GT(retired, old_session);
  EXPECT_FALSE(std::atomic_load(&adapter.latest_candidate_epoch_snapshot_));
  EXPECT_FALSE(std::atomic_load(&adapter.latest_epoch_snapshot_));
  EXPECT_FALSE(std::atomic_load(&adapter.latest_control_snapshot_));
}

TEST(PhaseOffsetMatchedAdapterTest,
     NewTaskResetRejectsPausedOldInactiveCleanupAfterBSlotsAppear) {
  const SyntheticPath path = MakePath();
  int identity = 402;
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  EXPECT_FALSE(adapter.update(MakeInput(path, &identity), output));
  adapter.deactivate(ros::Time(1.0));
  const std::shared_ptr<const TubeBuildRequest> old_inactive =
      std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(old_inactive);
  ASSERT_FALSE(old_inactive->active);

  std::mutex hook_mutex;
  std::condition_variable hook_cv;
  bool paused = false;
  bool release = false;
  adapter.inactive_publication_test_hook_ = [&]() {
    std::unique_lock<std::mutex> lock(hook_mutex);
    paused = true;
    hook_cv.notify_all();
    hook_cv.wait(lock, [&]() { return release; });
  };
  bool old_tick_result = true;
  std::thread old_tick([&]() { old_tick_result = adapter.timerTick(); });
  {
    std::unique_lock<std::mutex> lock(hook_mutex);
    hook_cv.wait(lock, [&]() { return paused; });
  }

  std::uint64_t retired = 0U;
  const std::uint64_t session =
      adapter.authority_session_.load(std::memory_order_acquire);
  ASSERT_TRUE(adapter.resetForNewNavigationTask(session, retired));
  const std::shared_ptr<const TubeEpochSnapshot> b_epoch(
      new TubeEpochSnapshot());
  const std::shared_ptr<const ControlPublishSnapshot> b_control(
      new ControlPublishSnapshot());
  std::atomic_store(&adapter.latest_candidate_epoch_snapshot_, b_epoch);
  std::atomic_store(&adapter.latest_epoch_snapshot_, b_epoch);
  std::atomic_store(&adapter.latest_control_snapshot_, b_control);
  {
    std::lock_guard<std::mutex> lock(hook_mutex);
    release = true;
  }
  hook_cv.notify_all();
  old_tick.join();
  adapter.inactive_publication_test_hook_ = std::function<void()>();

  EXPECT_FALSE(old_tick_result);
  EXPECT_EQ(b_epoch.get(),
            std::atomic_load(&adapter.latest_candidate_epoch_snapshot_).get());
  EXPECT_EQ(b_epoch.get(),
            std::atomic_load(&adapter.latest_epoch_snapshot_).get());
  EXPECT_EQ(b_control.get(),
            std::atomic_load(&adapter.latest_control_snapshot_).get());
}

TEST(PhaseOffsetMatchedAdapterTest,
     NewTaskResetRejectsPausedOldDeactivateProducerAfterBUpdate) {
  const SyntheticPath path = MakePath();
  int a_identity = 403;
  int b_identity = 404;
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  EXPECT_FALSE(adapter.update(MakeInput(path, &a_identity), output));

  std::mutex hook_mutex;
  std::condition_variable hook_cv;
  bool paused = false;
  bool release = false;
  adapter.deactivate_test_hook_ = [&]() {
    std::unique_lock<std::mutex> lock(hook_mutex);
    paused = true;
    hook_cv.notify_all();
    hook_cv.wait(lock, [&]() { return release; });
  };
  std::thread old_deactivate([&]() { adapter.deactivate(ros::Time(1.0)); });
  {
    std::unique_lock<std::mutex> lock(hook_mutex);
    hook_cv.wait(lock, [&]() { return paused; });
  }
  std::uint64_t retired = 0U;
  ASSERT_TRUE(adapter.resetForNewNavigationTask(
      adapter.authority_session_.load(std::memory_order_acquire), retired));
  EXPECT_FALSE(adapter.update(MakeInput(path, &b_identity, kDt), output));
  const std::shared_ptr<const TubeBuildRequest> b_request =
      std::atomic_load(&adapter.latest_build_request_);
  const std::shared_ptr<const ControlPublishSnapshot> b_control =
      std::atomic_load(&adapter.latest_control_snapshot_);
  ASSERT_TRUE(b_request);
  ASSERT_TRUE(b_control);
  {
    std::lock_guard<std::mutex> lock(hook_mutex);
    release = true;
  }
  hook_cv.notify_all();
  old_deactivate.join();
  adapter.deactivate_test_hook_ = std::function<void()>();
  EXPECT_EQ(b_request.get(), std::atomic_load(&adapter.latest_build_request_).get());
  EXPECT_EQ(b_control.get(), std::atomic_load(&adapter.latest_control_snapshot_).get());
  EXPECT_TRUE(adapter.command_active_);
}

TEST(PhaseOffsetMatchedAdapterTest,
     PairInputNeverReadsIndependentEpochSlotsForRuntimeAuthority) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;
  PathTubePairTransaction transaction;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples, path.current.w,
      1.0, 2.4, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), transaction));
  std::shared_ptr<const PathTubePair> pair;
  ASSERT_TRUE(PrepareAndFinalizePair(adapter, transaction, path.current.w, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), pair));
  std::atomic_store(&adapter.latest_epoch_snapshot_,
                    std::shared_ptr<const TubeEpochSnapshot>());
  input.path_tube_pair = pair;
  MatchedAdapterOutput output;
  EXPECT_FALSE(adapter.update(input, output));
  EXPECT_EQ(output.active_profile.get(), pair->active_profile.get());
  EXPECT_EQ(output.tube_epoch_status.active_path_source_revision,
            pair->source_revision);
}

TEST(PhaseOffsetMatchedAdapterTest,
     PairTimerRefreshAtomicallyReplacesSamePathAuthorityWithNewEpoch) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;

  PathTubePairTransaction transaction;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples, path.current.w,
      1.0, 2.4, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), transaction));
  std::shared_ptr<const PathTubePair> installed;
  ASSERT_TRUE(PrepareAndFinalizePair(adapter, transaction, path.current.w, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), installed));
  ASSERT_TRUE(installed);

  input.path_tube_pair = installed;
  MatchedAdapterOutput output;
  // The gate is intentionally closed, so this command publishes a base-pair
  // timer request without changing Runtime's captured bits.
  EXPECT_FALSE(adapter.update(input, output));
  const std::shared_ptr<const TubeBuildRequest> request =
      std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(request);
  EXPECT_EQ(request->base_path_tube_pair.get(), installed.get());
  EXPECT_EQ(request->base_path_tube_pair_generation, installed->generation);

  ASSERT_TRUE(adapter.timerTick());
  const std::shared_ptr<const PathTubePair> refreshed =
      adapter.capturePathTubePair();
  ASSERT_TRUE(refreshed);
  EXPECT_NE(refreshed.get(), installed.get());
  EXPECT_EQ(refreshed->generation, installed->generation + 1U);
  EXPECT_EQ(refreshed->path_owner.get(), installed->path_owner.get());
  EXPECT_EQ(refreshed->source_revision, installed->source_revision);
  EXPECT_NE(refreshed->active_profile.get(), installed->active_profile.get());
  ASSERT_TRUE(refreshed->epoch_snapshot);
  EXPECT_NE(refreshed->epoch_snapshot.get(), installed->epoch_snapshot.get());
  EXPECT_TRUE(refreshed->epoch_status.active_available);
  EXPECT_TRUE(refreshed->epoch_status.active_current_validation_valid);
  EXPECT_EQ(refreshed->epoch_status.active_path_source_revision,
            refreshed->source_revision);
  EXPECT_EQ(refreshed->epoch_snapshot->active_profile.get(),
            refreshed->active_profile.get());
}

TEST(PhaseOffsetMatchedAdapterPairPublication,
     RealTimerRefreshKeepsBroadCandidateNarrowCertifiedAndExactEpochIdentity) {
  SyntheticPath path = MakePath();
  MakeStraightRawPath(path);
  SDFMap map;
  InitializeFineKnownRawFreeMap(map);
  InstallCompleteCloudSnapshot(map, 801U);
  const std::shared_ptr<const ContinuousPhasePath> owner =
      MakeStraightSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::ESDF));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;
  input.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
  SetInputPositionAndLegacy(input, path.current.p);

  PathTubePairTransaction bootstrap;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples,
      path.current.w, 1.0, 2.4, input.position, input.gains, input.dt,
      input.cloud_occupancy_snapshot, bootstrap));
  std::shared_ptr<const PathTubePair> installed;
  ASSERT_TRUE(PrepareAndFinalizePair(
      adapter, bootstrap, path.current.w, input.position, input.gains,
      input.dt, input.cloud_occupancy_snapshot, installed));
  ASSERT_TRUE(installed);

  input.path_tube_pair = installed;
  MatchedAdapterOutput output;
  EXPECT_FALSE(adapter.update(input, output));
  ASSERT_TRUE(adapter.timerTick());
  const std::shared_ptr<const PathTubePair> refreshed =
      adapter.capturePathTubePair();
  ASSERT_TRUE(refreshed);
  ASSERT_NE(refreshed.get(), installed.get());
  ASSERT_TRUE(refreshed->epoch_snapshot);
  ASSERT_TRUE(refreshed->epoch_snapshot->candidate_profile);
  ASSERT_TRUE(refreshed->epoch_snapshot->active_profile);
  EXPECT_NE(refreshed->epoch_snapshot->candidate_profile.get(),
            refreshed->epoch_snapshot->active_profile.get());
  EXPECT_EQ(refreshed->epoch_snapshot->active_profile.get(),
            refreshed->active_profile.get());

  input.path_tube_pair = refreshed;
  input.stamp = ros::Time(kDt);
  EXPECT_FALSE(adapter.update(input, output));
  EXPECT_TRUE(output.valid) << output.invalid_reason;
  const std::shared_ptr<const TubeBuildRequest> request =
      std::atomic_load(&adapter.latest_build_request_);
  const std::shared_ptr<const ControlPublishSnapshot> control =
      std::atomic_load(&adapter.latest_control_snapshot_);
  ASSERT_TRUE(request);
  ASSERT_TRUE(control);
  EXPECT_EQ(control->output.candidate_profile.get(),
            refreshed->epoch_snapshot->candidate_profile.get());
  EXPECT_EQ(control->output.active_profile.get(),
            refreshed->active_profile.get());
  EXPECT_NE(control->output.candidate_profile.get(),
            control->output.active_profile.get());
  EXPECT_TRUE(PhaseOffsetMatchedAdapter::exactLivePairPublicationControl(
      *control, *request, refreshed,
      adapter.task_generation_.load(std::memory_order_acquire),
      adapter.authority_session_.load(std::memory_order_acquire)));

  MatchedAdapterMarkerBundle markers;
  ASSERT_TRUE(adapter.buildMarkers(*control, markers));
  ExpectActions(markers.tube_candidate, visualization_msgs::Marker::ADD);
  ExpectActions(markers.tube, visualization_msgs::Marker::ADD);
  const double candidate_width = TubeMarkerWidth(markers.tube_candidate);
  const double certified_width = TubeMarkerWidth(markers.tube);
  EXPECT_GT(candidate_width, 0.40);
  EXPECT_NEAR(certified_width, candidate_width, 1e-12);
}

TEST(PhaseOffsetMatchedAdapterTest,
     TransactionPinIsExclusiveAndTimerRefreshRecoversAfterRelease) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;
  PathTubePairTransaction bootstrap;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples, path.current.w,
      1.0, 2.4, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), bootstrap));
  std::shared_ptr<const PathTubePair> installed;
  ASSERT_TRUE(PrepareAndFinalizePair(
      adapter, bootstrap, path.current.w, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), installed));
  input.path_tube_pair = installed;
  MatchedAdapterOutput output;
  EXPECT_FALSE(adapter.update(input, output));
  const std::shared_ptr<const TubeBuildRequest> request =
      std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(request);
  TubeEpochSnapshot built;
  ASSERT_TRUE(adapter.buildTubeEpoch(request, built));
  const std::shared_ptr<const TubeEpochSnapshot> epoch(
      new TubeEpochSnapshot(built));

  std::unique_ptr<PathTubePairPin> pin =
      adapter.captureAndAcquirePathTubePairPin();
  ASSERT_TRUE(pin);
  EXPECT_TRUE(pin->valid());
  EXPECT_FALSE(adapter.captureAndAcquirePathTubePairPin());
  EXPECT_FALSE(adapter.refreshPairFromTimerEpoch(request, epoch));
  EXPECT_EQ(adapter.capturePathTubePair().get(), installed.get());

  pin->release();
  EXPECT_FALSE(pin->valid());
  EXPECT_TRUE(adapter.refreshPairFromTimerEpoch(request, epoch));
  const std::shared_ptr<const PathTubePair> refreshed =
      adapter.capturePathTubePair();
  ASSERT_TRUE(refreshed);
  EXPECT_NE(refreshed.get(), installed.get());
  EXPECT_EQ(refreshed->generation, installed->generation + 1U);
}

TEST(PhaseOffsetMatchedAdapterTest,
     PinReleaseIsIdempotentAbaSafeAndSurvivesRetireAndShutdown) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;
  PathTubePairTransaction bootstrap;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples, path.current.w,
      1.0, 2.4, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), bootstrap));
  std::shared_ptr<const PathTubePair> installed;
  ASSERT_TRUE(PrepareAndFinalizePair(
      adapter, bootstrap, path.current.w, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), installed));

  std::unique_ptr<PathTubePairPin> first =
      adapter.captureAndAcquirePathTubePairPin();
  ASSERT_TRUE(first);
  const std::uint64_t first_lease = first->leaseId();
  // Deliberately retain a duplicate late-release fixture for the same first
  // lease.  Production ownership stays unique; this directly exercises the
  // lease-id ABA check after the real owner releases and a later lease starts.
  PathTubePairPin stale(adapter.path_tube_pin_registry_, first->capture(),
                        first_lease);
  first->release();
  first.reset();
  std::unique_ptr<PathTubePairPin> second =
      adapter.captureAndAcquirePathTubePairPin();
  ASSERT_TRUE(second);
  EXPECT_GT(second->leaseId(), first_lease);
  // The moved-from/stale lease cannot clear the later active lease.
  stale.release();
  stale.release();
  EXPECT_FALSE(adapter.captureAndAcquirePathTubePairPin());

  const std::uint64_t retired = adapter.retirePathTubeAuthority(
      installed->authority_session + 1U);
  EXPECT_GT(retired, installed->authority_session);
  second->release();
  second->release();
  adapter.shutdown();
  EXPECT_FALSE(adapter.captureAndAcquirePathTubePairPin());
}

TEST(PhaseOffsetMatchedAdapterTest,
     ReplacementStagePrepareAndFinalizeRejectMissingOrMismatchedPin) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> old_owner =
      MakeSyntheticOwner();
  const std::shared_ptr<const ContinuousPhasePath> new_owner =
      MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, old_owner.get());
  input.semantic_path_owner = old_owner;
  PathTubePairTransaction bootstrap;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), old_owner, path.samples,
      path.current.w, 1.0, 2.4, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), bootstrap));
  std::shared_ptr<const PathTubePair> installed;
  ASSERT_TRUE(PrepareAndFinalizePair(
      adapter, bootstrap, path.current.w, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), installed));

  PathTubePairTransaction replacement;
  EXPECT_FALSE(adapter.stagePathTubePair(
      installed, new_owner, path.samples, path.current.w, 1.0, 2.4,
      input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), replacement));
  std::unique_ptr<PathTubePairPin> pin =
      adapter.captureAndAcquirePathTubePairPin();
  ASSERT_TRUE(pin);
  PathTubePairPinCapture bad_capture = pin->capture();
  ++bad_capture.map_observation_sequence;
  EXPECT_FALSE(adapter.stagePathTubePair(
      installed, new_owner, path.samples, path.current.w, 1.0, 2.4,
      input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), replacement,
      pin->capture().authority_session, &bad_capture, pin->leaseId()));
  ASSERT_TRUE(adapter.stagePathTubePair(
      installed, new_owner, path.samples, path.current.w, 1.0, 2.4,
      input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), replacement,
      pin->capture().authority_session, &pin->capture(), pin->leaseId()));

  PathTubePairTransaction mismatched = replacement;
  ++mismatched.pin_lease_id;
  PathTubePairCommitPreparation preparation;
  EXPECT_FALSE(adapter.preparePathTubePairCommit(
      mismatched, path.current.w, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), preparation));
  ASSERT_TRUE(adapter.preparePathTubePairCommit(
      replacement, path.current.w, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), preparation));
  pin->release();
  std::shared_ptr<const PathTubePair> committed;
  EXPECT_FALSE(adapter.finalizePreparedPathTubePairCommit(preparation,
                                                          committed));
  EXPECT_EQ(adapter.capturePathTubePair().get(), installed.get());
}

TEST(PhaseOffsetMatchedAdapterAttribution,
     ReplacementDryRunFailureReturnsExistingEnumWithoutMutatingAuthority) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> old_owner =
      MakeSyntheticOwner();
  const std::shared_ptr<const ContinuousPhasePath> new_owner =
      MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, old_owner.get());
  input.semantic_path_owner = old_owner;

  PathTubePairTransaction bootstrap;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), old_owner, path.samples,
      path.current.w, 1.0, 2.4, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), bootstrap));
  std::shared_ptr<const PathTubePair> installed;
  ASSERT_TRUE(PrepareAndFinalizePair(
      adapter, bootstrap, path.current.w, input.position, input.gains,
      input.dt, std::shared_ptr<const plan_env::CloudOccupancySnapshot>(),
      installed));
  ASSERT_TRUE(installed);
  std::unique_ptr<PathTubePairPin> pin =
      adapter.captureAndAcquirePathTubePairPin();
  ASSERT_TRUE(pin);
  ASSERT_TRUE(pin->valid());

  const double retained_before = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      adapter.runtime_->previousFinalPort();
  for (int attempt = 0; attempt < 2; ++attempt) {
    PathTubePairTransaction replacement;
    PathTubePairStageFailure failure = PathTubePairStageFailure::NONE;
    EXPECT_FALSE(adapter.stagePathTubePair(
        installed, new_owner, path.samples, path.current.w, 1.0, 2.4,
        input.position, input.gains,
        std::numeric_limits<double>::quiet_NaN(),
        std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), replacement,
        pin->capture().authority_session, &pin->capture(), pin->leaseId(),
        &failure));
    EXPECT_EQ(failure, PathTubePairStageFailure::STAGING_DRY_RUN);
    EXPECT_FALSE(replacement.expected_pair);
    EXPECT_FALSE(replacement.candidate_pair);
    EXPECT_EQ(adapter.capturePathTubePair().get(), installed.get());
    EXPECT_TRUE(pin->valid());
  }
  const double retained_after = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand previous_after =
      adapter.runtime_->previousFinalPort();
  EXPECT_EQ(0, std::memcmp(&retained_before, &retained_after,
                           sizeof(retained_before)));
  EXPECT_EQ(0, std::memcmp(&previous_before.u_w, &previous_after.u_w,
                           sizeof(previous_before.u_w)));
  EXPECT_EQ(0, std::memcmp(&previous_before.u_delta,
                           &previous_after.u_delta,
                           sizeof(previous_before.u_delta)));
}

TEST(PhaseOffsetMatchedAdapterTest,
     StaleBaseTimerEpochCannotChangeNewPairOrPersistentTimerManager) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> old_owner =
      MakeSyntheticOwner();
  const std::shared_ptr<const ContinuousPhasePath> new_owner =
      MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput old_input = MakeInput(path, old_owner.get());
  old_input.semantic_path_owner = old_owner;

  PathTubePairTransaction bootstrap;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), old_owner, path.samples,
      path.current.w, 1.0, 2.4, old_input.position, old_input.gains,
      old_input.dt, std::shared_ptr<const plan_env::CloudOccupancySnapshot>(),
      bootstrap));
  std::shared_ptr<const PathTubePair> old_pair;
  ASSERT_TRUE(PrepareAndFinalizePair(adapter, bootstrap, path.current.w, old_input.position, old_input.gains, old_input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), old_pair));
  ASSERT_TRUE(old_pair);

  old_input.path_tube_pair = old_pair;
  MatchedAdapterOutput output;
  EXPECT_FALSE(adapter.update(old_input, output));
  const std::shared_ptr<const TubeBuildRequest> old_request =
      std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(old_request);
  ASSERT_EQ(old_request->base_path_tube_pair.get(), old_pair.get());
  TubeEpochSnapshot old_built;
  ASSERT_TRUE(adapter.buildTubeEpoch(old_request, old_built));
  // Base-pair work is isolated from the timer-owned manager before the new
  // authority is committed, so an old result has no persistent manager state
  // to corrupt after it becomes stale.
  const auto* const persistent_manager = adapter.tube_epoch_manager_.get();
  const std::shared_ptr<const phase_offset_navigation::TubeProfile>
      timer_profile_before = adapter.timer_active_profile_;

  MatchedAdapterInput new_input = MakeInput(path, new_owner.get(), kDt);
  new_input.semantic_path_owner = new_owner;
  std::unique_ptr<PathTubePairPin> old_pin =
      adapter.captureAndAcquirePathTubePairPin();
  ASSERT_TRUE(old_pin);
  PathTubePairTransaction replacement;
  ASSERT_TRUE(adapter.stagePathTubePair(
      old_pair, new_owner, path.samples, path.current.w, 1.0, 2.4,
      new_input.position, new_input.gains, new_input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), replacement,
      old_pin->capture().authority_session, &old_pin->capture(),
      old_pin->leaseId()));
  std::shared_ptr<const PathTubePair> new_pair;
  ASSERT_TRUE(PrepareAndFinalizePair(adapter, replacement, path.current.w, new_input.position, new_input.gains, new_input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), new_pair));
  old_pin->release();
  ASSERT_TRUE(new_pair);
  ASSERT_NE(new_pair.get(), old_pair.get());
  const std::uint64_t new_generation = new_pair->generation;
  const auto* const new_profile = new_pair->active_profile.get();
  const auto* const new_epoch = new_pair->epoch_snapshot.get();

  EXPECT_FALSE(adapter.finalizeTubeEpoch(old_request, old_built));
  const std::shared_ptr<const PathTubePair> after_stale =
      adapter.capturePathTubePair();
  ASSERT_TRUE(after_stale);
  EXPECT_EQ(after_stale.get(), new_pair.get());
  EXPECT_EQ(after_stale->generation, new_generation);
  EXPECT_EQ(after_stale->path_owner.get(), new_owner.get());
  EXPECT_EQ(after_stale->active_profile.get(), new_profile);
  EXPECT_EQ(after_stale->epoch_snapshot.get(), new_epoch);
  EXPECT_EQ(adapter.tube_epoch_manager_.get(), persistent_manager);
  EXPECT_EQ(adapter.timer_active_profile_.get(), timer_profile_before.get());

  // A fresh base request for the new authority is built by another local
  // manager and succeeds; the old completion cannot poison this refresh.
  new_input.path_tube_pair = new_pair;
  EXPECT_FALSE(adapter.update(new_input, output));
  ASSERT_TRUE(adapter.timerTick());
  const std::shared_ptr<const PathTubePair> refreshed =
      adapter.capturePathTubePair();
  ASSERT_TRUE(refreshed);
  EXPECT_NE(refreshed.get(), new_pair.get());
  EXPECT_EQ(refreshed->path_owner.get(), new_owner.get());
  EXPECT_EQ(refreshed->source_revision, new_pair->source_revision);
  EXPECT_EQ(refreshed->generation, new_generation + 1U);
}

TEST(PhaseOffsetMatchedAdapterTest,
     PairTimerRefreshRebasesACompletedEpochToLatestRuntimeBits) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;

  PathTubePairTransaction transaction;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples, path.current.w,
      1.0, 2.4, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), transaction));
  std::shared_ptr<const PathTubePair> installed;
  ASSERT_TRUE(PrepareAndFinalizePair(adapter, transaction, path.current.w, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), installed));
  ASSERT_TRUE(installed);

  input.path_tube_pair = installed;
  MatchedAdapterOutput output;
  EXPECT_FALSE(adapter.update(input, output));
  const std::shared_ptr<const TubeBuildRequest> request =
      std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(request);
  ASSERT_EQ(request->base_path_tube_pair.get(), installed.get());
  TubeEpochSnapshot built;
  ASSERT_TRUE(adapter.buildTubeEpoch(request, built));
  const std::shared_ptr<const TubeEpochSnapshot> epoch(
      new TubeEpochSnapshot(built));

  // This represents a normal Runtime commit during the timer build.  Refresh
  // must prove the new immutable profile against these latest bits, rather
  // than compare them with the obsolete request capture.
  adapter.runtime_->delta_ = request->base_retained_delta + 0.01;
  const double rebased_delta = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand rebased_port =
      adapter.runtime_->previousFinalPort();
  EXPECT_TRUE(adapter.refreshPairFromTimerEpoch(request, epoch));
  const std::shared_ptr<const PathTubePair> after =
      adapter.capturePathTubePair();
  ASSERT_TRUE(after);
  EXPECT_NE(after.get(), installed.get());
  EXPECT_EQ(after->generation, installed->generation + 1U);
  EXPECT_NE(after->active_profile.get(), installed->active_profile.get());
  EXPECT_EQ(0, std::memcmp(&rebased_delta, &adapter.runtime_->delta_,
                           sizeof(rebased_delta)));
  EXPECT_EQ(0, std::memcmp(&rebased_port.u_w,
                           &adapter.runtime_->previous_final_port_.u_w,
                           sizeof(rebased_port.u_w)));
  EXPECT_EQ(0, std::memcmp(&rebased_port.u_delta,
                           &adapter.runtime_->previous_final_port_.u_delta,
                           sizeof(rebased_port.u_delta)));
}

TEST(PhaseOffsetMatchedAdapterTest,
     PairTimerRefreshFinalCasRejectsPostPrepareRuntimeOrCommandChange) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;

  PathTubePairTransaction transaction;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples, path.current.w,
      1.0, 2.4, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), transaction));
  std::shared_ptr<const PathTubePair> installed;
  ASSERT_TRUE(PrepareAndFinalizePair(
      adapter, transaction, path.current.w, input.position, input.gains,
      input.dt, std::shared_ptr<const plan_env::CloudOccupancySnapshot>(),
      installed));
  ASSERT_TRUE(installed);

  input.path_tube_pair = installed;
  MatchedAdapterOutput output;
  EXPECT_FALSE(adapter.update(input, output));
  const std::shared_ptr<const TubeBuildRequest> request =
      std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(request);
  TubeEpochSnapshot built;
  ASSERT_TRUE(adapter.buildTubeEpoch(request, built));
  const std::shared_ptr<const TubeEpochSnapshot> epoch(
      new TubeEpochSnapshot(built));

  TimerPairRefreshPreparation preparation;
  ASSERT_TRUE(adapter.prepareTimerPairRefresh(request, epoch, preparation));
  ASSERT_TRUE(preparation.valid);
  const double delta_before = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand port_before =
      adapter.runtime_->previousFinalPort();

  // This mutation occurs after the local exact-PWL witness has completed;
  // final CAS must reject it and must not publish the prepared profile.
  adapter.runtime_->delta_ = delta_before + 0.01;
  std::shared_ptr<const PathTubePair> refreshed;
  EXPECT_FALSE(adapter.finalizePreparedTimerPairRefresh(preparation, refreshed));
  EXPECT_FALSE(refreshed);
  EXPECT_EQ(adapter.capturePathTubePair().get(), installed.get());
  EXPECT_EQ(adapter.capturePathTubePair()->generation, installed->generation);
  EXPECT_EQ(0, std::memcmp(&port_before.u_w,
                           &adapter.runtime_->previous_final_port_.u_w,
                           sizeof(port_before.u_w)));
  EXPECT_EQ(0, std::memcmp(&port_before.u_delta,
                           &adapter.runtime_->previous_final_port_.u_delta,
                           sizeof(port_before.u_delta)));

  // A fresh command identity is also part of the final predicate, even when
  // it does not itself commit Runtime (the gate is still closed here).
  adapter.runtime_->delta_ = delta_before;
  MatchedAdapterInput newer = input;
  newer.stamp = ros::Time(kDt);
  EXPECT_FALSE(adapter.update(newer, output));
  EXPECT_FALSE(adapter.finalizePreparedTimerPairRefresh(preparation, refreshed));
  EXPECT_EQ(adapter.capturePathTubePair().get(), installed.get());
}

TEST(PhaseOffsetMatchedAdapterTest,
     PairTimerRefreshRejectsCurrentValidCandidateWithoutExactPwlWitness) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;

  PathTubePairTransaction transaction;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples, path.current.w,
      1.0, 2.4, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), transaction));
  std::shared_ptr<const PathTubePair> installed;
  ASSERT_TRUE(PrepareAndFinalizePair(
      adapter, transaction, path.current.w, input.position, input.gains,
      input.dt, std::shared_ptr<const plan_env::CloudOccupancySnapshot>(),
      installed));
  ASSERT_TRUE(installed);

  input.path_tube_pair = installed;
  MatchedAdapterOutput output;
  EXPECT_FALSE(adapter.update(input, output));
  const std::shared_ptr<const TubeBuildRequest> request =
      std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(request);
  TubeEpochSnapshot built;
  ASSERT_TRUE(adapter.buildTubeEpoch(request, built));

  // Retain the candidate's current-valid epoch facts while shortening only
  // the future exact-PWL domain.  The old refresh would install it after the
  // structural owner check; the Runtime dry-run must now reject it.
  std::shared_ptr<TubeProfile> no_witness_profile(
      new TubeProfile(*built.active_profile));
  no_witness_profile->preview_end_w = path.current.w + 1e-7;
  no_witness_profile->certified_segment_end_w = path.current.w + 1e-7;
  std::shared_ptr<TubeEpochSnapshot> no_witness_epoch(
      new TubeEpochSnapshot(built));
  no_witness_epoch->active_profile =
      std::shared_ptr<const TubeProfile>(no_witness_profile);
  ASSERT_TRUE(no_witness_epoch->epoch_status.active_available);
  ASSERT_TRUE(no_witness_epoch->epoch_status.active_current_validation_valid);

  EXPECT_FALSE(adapter.refreshPairFromTimerEpoch(
      request, std::shared_ptr<const TubeEpochSnapshot>(no_witness_epoch)));
  EXPECT_EQ(adapter.capturePathTubePair().get(), installed.get());
  EXPECT_EQ(adapter.capturePathTubePair()->generation, installed->generation);
}

TEST(PhaseOffsetMatchedAdapterTest,
     PairTimerRefreshRejectsLatestCategoricalUnsafeBeforePairCas) {
  SyntheticPath path = MakePath();
  MakeStraightRawPath(path);
  SDFMap map;
  InitializeFineKnownRawFreeMap(map);
  InstallCompleteCloudSnapshot(map, 701U);
  const std::shared_ptr<const ContinuousPhasePath> owner =
      MakeStraightSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::ESDF));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;
  SetInputPositionAndLegacy(input, path.current.p);
  input.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();

  PathTubePairTransaction transaction;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples, path.current.w,
      1.0, 2.4, input.position, input.gains, input.dt,
      input.cloud_occupancy_snapshot, transaction));
  std::shared_ptr<const PathTubePair> installed;
  ASSERT_TRUE(PrepareAndFinalizePair(
      adapter, transaction, path.current.w, input.position, input.gains,
      input.dt, input.cloud_occupancy_snapshot, installed));
  ASSERT_TRUE(installed);

  input.path_tube_pair = installed;
  MatchedAdapterOutput output;
  EXPECT_FALSE(adapter.update(input, output));
  const std::shared_ptr<const TubeBuildRequest> request =
      std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(request);
  TubeEpochSnapshot built;
  ASSERT_TRUE(adapter.buildTubeEpoch(request, built));
  const std::shared_ptr<const TubeEpochSnapshot> epoch(
      new TubeEpochSnapshot(built));

  // A later complete observation explicitly occupies the current actual
  // position.  It is not candidate provenance, but it is the existing H2
  // latest-categorical installation veto.
  std::shared_ptr<plan_env::CloudOccupancySnapshot> occupied(
      new plan_env::CloudOccupancySnapshot(*input.cloud_occupancy_snapshot));
  occupied->observation_sequence = 702U;
  occupied->observation_stamp = ros::Time(702.0);
  const Eigen::Vector3i index(54, 50, 60);  // (0.4, 0.0, 1.0) at 0.1 m voxels.
  const std::size_t address =
      (static_cast<std::size_t>(index.x()) * 100U +
       static_cast<std::size_t>(index.y())) * 100U +
      static_cast<std::size_t>(index.z());
  ASSERT_LT(address, occupied->occupied.size());
  occupied->occupied[address] = 1U;
  ASSERT_TRUE(plan_env::cloudOccupancySnapshotConsistent(*occupied));
  input.cloud_occupancy_snapshot = occupied;
  input.stamp = ros::Time(kDt);
  EXPECT_FALSE(adapter.update(input, output));
  EXPECT_TRUE(output.tube_epoch_status.certificate_denied);

  EXPECT_FALSE(adapter.refreshPairFromTimerEpoch(request, epoch));
  EXPECT_EQ(adapter.capturePathTubePair().get(), installed.get());
  EXPECT_EQ(adapter.capturePathTubePair()->generation, installed->generation);
}

TEST(PhaseOffsetMatchedAdapterTest,
     PairBranchPublishesFinalSnapshotBeforeGateAndAfterSelection) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;

  PathTubePairTransaction bootstrap;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples, path.current.w,
      1.0, 2.4, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), bootstrap));
  std::shared_ptr<const PathTubePair> installed;
  ASSERT_TRUE(PrepareAndFinalizePair(
      adapter, bootstrap, path.current.w, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), installed));
  ASSERT_TRUE(installed);
  ASSERT_TRUE(installed->epoch_snapshot);
  input.path_tube_pair = installed;

  const double retained_before = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      adapter.runtime_->previousFinalPort();
  MatchedAdapterOutput output;
  EXPECT_FALSE(adapter.update(input, output));
  EXPECT_TRUE(output.valid) << output.invalid_reason;
  const std::shared_ptr<const ControlPublishSnapshot> before_gate =
      std::atomic_load(&adapter.latest_control_snapshot_);
  ASSERT_TRUE(before_gate);
  EXPECT_TRUE(before_gate->active);
  EXPECT_FALSE(before_gate->candidate_only);
  EXPECT_EQ(before_gate->source_revision, installed->source_revision);
  EXPECT_EQ(before_gate->epoch_snapshot.get(), installed->epoch_snapshot.get());
  EXPECT_EQ(before_gate->full_path_samples.get(),
            installed->epoch_snapshot->full_path_samples.get());
  EXPECT_EQ(before_gate->output.candidate_profile.get(),
            installed->epoch_snapshot->candidate_profile.get());
  EXPECT_EQ(before_gate->output.active_profile.get(),
            installed->epoch_snapshot->active_profile.get());
  EXPECT_EQ(before_gate->output.tube_epoch_status.active_tube_epoch,
            installed->epoch_snapshot->epoch_status.active_tube_epoch);
  EXPECT_EQ(before_gate->output.selected, output.selected);
  EXPECT_EQ(before_gate->output.valid, output.valid);
  const double retained_after_pre_gate = adapter.runtime_->retainedDelta();
  EXPECT_EQ(0, std::memcmp(&retained_before, &retained_after_pre_gate,
                           sizeof(retained_before)));
  const phase_offset_core::PortCommand previous_after_pre_gate =
      adapter.runtime_->previousFinalPort();
  EXPECT_EQ(0, std::memcmp(&previous_before.u_w, &previous_after_pre_gate.u_w,
                           sizeof(previous_before.u_w)));
  EXPECT_EQ(0, std::memcmp(&previous_before.u_delta,
                           &previous_after_pre_gate.u_delta,
                           sizeof(previous_before.u_delta)));
  EXPECT_EQ(adapter.capturePathTubePair().get(), installed.get());

  for (int cycle = 1; cycle < 99; ++cycle) {
    input.stamp = ros::Time(cycle * kDt);
    EXPECT_FALSE(adapter.update(input, output));
  }
  input.stamp = ros::Time(99.0 * kDt);
  EXPECT_TRUE(adapter.update(input, output)) << output.invalid_reason;
  const std::shared_ptr<const ControlPublishSnapshot> selected =
      std::atomic_load(&adapter.latest_control_snapshot_);
  ASSERT_TRUE(selected);
  EXPECT_TRUE(output.selected);
  EXPECT_TRUE(selected->output.selected);
  EXPECT_EQ(selected->output.valid, output.valid);
  EXPECT_EQ(selected->output.delta, output.delta);
  EXPECT_EQ(selected->epoch_snapshot.get(), installed->epoch_snapshot.get());
  EXPECT_EQ(adapter.capturePathTubePair().get(), installed.get());
}

TEST(PhaseOffsetMatchedAdapterTest,
     ObserveOnlyPairPublishesNonselectedSnapshotWithoutRuntimeMutation) {
  const SyntheticPath path = MakePath();
  const std::shared_ptr<const ContinuousPhasePath> owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::FIXED);
  config.observe_only = true;
  PhaseOffsetMatchedAdapter adapter(config);
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;

  PathTubePairTransaction bootstrap;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples, path.current.w,
      1.0, 2.4, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), bootstrap));
  std::shared_ptr<const PathTubePair> installed;
  ASSERT_TRUE(PrepareAndFinalizePair(
      adapter, bootstrap, path.current.w, input.position, input.gains, input.dt,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), installed));
  ASSERT_TRUE(installed);
  input.path_tube_pair = installed;

  const double retained_before = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      adapter.runtime_->previousFinalPort();
  MatchedAdapterOutput output;
  EXPECT_FALSE(adapter.update(input, output));
  const std::shared_ptr<const ControlPublishSnapshot> control =
      std::atomic_load(&adapter.latest_control_snapshot_);
  ASSERT_TRUE(control);
  EXPECT_TRUE(control->active);
  EXPECT_FALSE(control->candidate_only);
  EXPECT_FALSE(control->output.selected);
  EXPECT_EQ(control->output.valid, output.valid);
  EXPECT_EQ(control->epoch_snapshot.get(), installed->epoch_snapshot.get());
  EXPECT_EQ(control->output.active_profile.get(), installed->active_profile.get());
  const double retained_after = adapter.runtime_->retainedDelta();
  EXPECT_EQ(0, std::memcmp(&retained_before, &retained_after,
                           sizeof(retained_before)));
  const phase_offset_core::PortCommand previous_after =
      adapter.runtime_->previousFinalPort();
  EXPECT_EQ(0, std::memcmp(&previous_before.u_w, &previous_after.u_w,
                           sizeof(previous_before.u_w)));
  EXPECT_EQ(0, std::memcmp(&previous_before.u_delta, &previous_after.u_delta,
                           sizeof(previous_before.u_delta)));
  EXPECT_EQ(adapter.capturePathTubePair().get(), installed.get());
}

TEST(PhaseOffsetMatchedAdapterTest,
     PairBranchLatestCategoricalUnsafeMasksSelectionWithoutChangingAuthority) {
  SyntheticPath path = MakePath();
  MakeStraightRawPath(path);
  SDFMap map;
  InitializeFineKnownRawFreeMap(map);
  InstallCompleteCloudSnapshot(map, 601U);
  const std::shared_ptr<const ContinuousPhasePath> owner =
      MakeStraightSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::ESDF));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;
  SetInputPositionAndLegacy(input, path.current.p);
  input.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();

  PathTubePairTransaction transaction;
  ASSERT_TRUE(adapter.stagePathTubePair(
      std::shared_ptr<const PathTubePair>(), owner, path.samples, path.current.w,
      1.0, 2.4, input.position, input.gains, input.dt,
      input.cloud_occupancy_snapshot, transaction));
  std::shared_ptr<const PathTubePair> installed;
  ASSERT_TRUE(PrepareAndFinalizePair(adapter, transaction, path.current.w, input.position, input.gains, input.dt,
      input.cloud_occupancy_snapshot, installed));
  ASSERT_TRUE(installed);
  input.path_tube_pair = installed;

  // Establish an open gate once under the safe current snapshot.  This makes
  // the following rejection prove the pair-branch categorical mask rather
  // than merely a closed warmup gate.
  MatchedAdapterOutput output;
  for (int cycle = 0; cycle < 100; ++cycle) {
    input.stamp = ros::Time(cycle * kDt);
    ASSERT_EQ(adapter.update(input, output), cycle >= 99);
    input.path_tube_pair = adapter.capturePathTubePair();
    ASSERT_TRUE(input.path_tube_pair);
  }
  const std::shared_ptr<const PathTubePair> safe_pair =
      adapter.capturePathTubePair();
  ASSERT_TRUE(safe_pair);
  ASSERT_TRUE(output.selected);

  auto latest_snapshot = [](const std::uint64_t sequence) {
    auto snapshot = std::make_shared<plan_env::CloudOccupancySnapshot>();
    snapshot->valid = true;
    snapshot->observation_sequence = sequence;
    snapshot->observation_stamp = ros::Time(static_cast<double>(sequence));
    snapshot->map_min = Eigen::Vector3d(-5.0, -5.0, -5.0);
    snapshot->map_max = Eigen::Vector3d(5.0, 5.0, 5.0);
    snapshot->observed_min = snapshot->map_min;
    snapshot->observed_max = snapshot->map_max;
    snapshot->grid_origin = snapshot->map_min;
    snapshot->voxel_count = Eigen::Vector3i(100, 100, 100);
    snapshot->resolution = 0.10;
    snapshot->included_map_inflation = 0.10;
    snapshot->occupied.assign(1000000U, 0U);
    return snapshot;
  };
  auto address = [](const Eigen::Vector3i& index) {
    return (static_cast<std::size_t>(index.x()) * 100U +
            static_cast<std::size_t>(index.y())) * 100U +
        static_cast<std::size_t>(index.z());
  };

  // The actual UAV position lies in a latest explicit occupied voxel.  The
  // immutable installed pair remains the authority object, but Runtime must
  // deny selection before it can use its historical certificate.
  std::shared_ptr<plan_env::CloudOccupancySnapshot> actual_occupied =
      latest_snapshot(602U);
  actual_occupied->occupied[address(Eigen::Vector3i(54, 50, 60))] = 1U;
  ASSERT_TRUE(plan_env::cloudOccupancySnapshotConsistent(*actual_occupied));
  input.path_tube_pair = safe_pair;
  input.cloud_occupancy_snapshot = actual_occupied;
  input.stamp = ros::Time(3.0);
  const double retained_before_denial = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand previous_before_denial =
      adapter.runtime_->previousFinalPort();
  EXPECT_FALSE(adapter.update(input, output));
  EXPECT_TRUE(output.tube_epoch_status.certificate_denied);
  EXPECT_EQ(output.tube_epoch_status.current_safety_status,
            phase_offset_navigation::CurrentSafetyStatus::UNSAFE);
  EXPECT_TRUE(output.runtime_execution.certificate_denied);
  EXPECT_EQ(adapter.capturePathTubePair().get(), safe_pair.get());
  const std::shared_ptr<const ControlPublishSnapshot> denied =
      std::atomic_load(&adapter.latest_control_snapshot_);
  ASSERT_TRUE(denied);
  EXPECT_TRUE(denied->active);
  EXPECT_FALSE(denied->candidate_only);
  EXPECT_FALSE(denied->output.selected);
  EXPECT_TRUE(denied->output.runtime_execution.certificate_denied);
  EXPECT_EQ(denied->epoch_snapshot.get(), safe_pair->epoch_snapshot.get());
  EXPECT_EQ(denied->output.active_profile.get(), safe_pair->active_profile.get());
  const double retained_after_denial = adapter.runtime_->retainedDelta();
  EXPECT_EQ(0, std::memcmp(&retained_before_denial, &retained_after_denial,
                           sizeof(retained_before_denial)));
  const phase_offset_core::PortCommand previous_after_denial =
      adapter.runtime_->previousFinalPort();
  EXPECT_EQ(0, std::memcmp(&previous_before_denial.u_w,
                           &previous_after_denial.u_w,
                           sizeof(previous_before_denial.u_w)));
  EXPECT_EQ(0, std::memcmp(&previous_before_denial.u_delta,
                           &previous_after_denial.u_delta,
                           sizeof(previous_before_denial.u_delta)));

  // The actual point is free again, but the active reference r (at retained
  // delta zero here) lies outside the latest map.  This separately proves the
  // reference leg of the pair-branch query; it must not degrade to base-only
  // or ignore the new categorical snapshot.
  std::shared_ptr<plan_env::CloudOccupancySnapshot> reference_out_of_map =
      latest_snapshot(603U);
  reference_out_of_map->map_max.x() = 0.30;
  reference_out_of_map->observed_max.x() = 0.30;
  ASSERT_TRUE(plan_env::cloudOccupancySnapshotConsistent(
      *reference_out_of_map));
  input.path_tube_pair = safe_pair;
  input.position = Eigen::Vector3d(0.2, 0.0, 1.0);
  SetInputPositionAndLegacy(input, input.position);
  input.cloud_occupancy_snapshot = reference_out_of_map;
  input.stamp = ros::Time(3.0 + kDt);
  EXPECT_FALSE(adapter.update(input, output));
  EXPECT_TRUE(output.tube_epoch_status.certificate_denied);
  EXPECT_EQ(output.tube_epoch_status.current_safety_status,
            phase_offset_navigation::CurrentSafetyStatus::UNSAFE);
  EXPECT_TRUE(output.runtime_execution.certificate_denied);
  EXPECT_EQ(adapter.capturePathTubePair().get(), safe_pair.get());
}

TEST(PhaseOffsetMatchedAdapterTest,
     EsdfMapAdvanceSampleHoldsFrozenActiveUntilNextSameSourceEpoch) {
  SyntheticPath path = MakePath();
  MakeStraightRawPath(path);
  SDFMap map;
  InitializeFineKnownRawFreeMap(map);
  InstallCompleteCloudSnapshot(map, 51U);
  int identity = 103;
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::ESDF));
  MatchedAdapterOutput output;
  MatchedAdapterInput first = MakeInput(path, &identity);
  SetInputPositionAndLegacy(first, path.current.p);
  first.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
  BuildAndConsume(adapter, first, output);
  ASSERT_TRUE(output.active_profile);
  const auto* first_active = output.active_profile.get();

  InstallCompleteCloudSnapshot(map, 52U);
  MatchedAdapterInput revised = MakeInput(path, &identity, kDt);
  SetInputPositionAndLegacy(revised, path.current.p);
  revised.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
  // A 10 Hz map advance during an asynchronous build does not invalidate the
  // completed, immutable same-source epoch.  The new observation remains the
  // next build's provenance rather than an equality gate on this handoff.
  EXPECT_FALSE(adapter.update(revised, output));
  ASSERT_TRUE(output.active_profile);
  EXPECT_EQ(output.active_profile.get(), first_active);
  EXPECT_EQ(output.tube_epoch_status.candidate_map_observation_sequence, 51U);
  EXPECT_EQ(output.tube_epoch_status.active_map_observation_sequence, 51U);
  ASSERT_TRUE(adapter.timerTick());
  EXPECT_FALSE(adapter.update(revised, output));
  ASSERT_TRUE(output.active_profile);
  EXPECT_EQ(output.tube_epoch_status.candidate_map_observation_sequence, 52U);
  EXPECT_EQ(output.tube_epoch_status.active_map_observation_sequence, 52U);
}

TEST(PhaseOffsetMatchedAdapterTest,
     ContinuouslyAdvancingMapStillInstallsEachFrozenSameSourceEpoch) {
  SyntheticPath path = MakePath();
  MakeStraightRawPath(path);
  SDFMap map;
  InitializeFineKnownRawFreeMap(map);
  int identity = 109;
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::ESDF));
  MatchedAdapterOutput output;

  auto input_for = [&](const std::uint64_t sequence, const double stamp) {
    InstallCompleteCloudSnapshot(map, sequence);
    MatchedAdapterInput input = MakeInput(path, &identity, stamp);
    SetInputPositionAndLegacy(input, path.current.p);
    input.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
    return input;
  };

  std::uint64_t sequence = 201U;
  std::uint64_t previous_candidate_sequence = 0U;
  for (int round = 0; round < 4; ++round, ++sequence) {
    // The next observation arrives while this 10 Hz worker build is in
    // flight.  The completed request keeps its own immutable snapshot.
    MatchedAdapterInput captured = input_for(sequence, round * kDt);
    EXPECT_FALSE(adapter.update(captured, output));
    const std::shared_ptr<const TubeBuildRequest> captured_request =
        std::atomic_load(&adapter.latest_build_request_);
    ASSERT_TRUE(captured_request);
    ASSERT_EQ(captured_request->map_observation_sequence, sequence);
    TubeEpochSnapshot built;
    adapter.buildTubeEpoch(captured_request, built);

    MatchedAdapterInput current = input_for(sequence + 1U,
                                             (round + 1) * kDt);
    EXPECT_FALSE(adapter.update(current, output));
    EXPECT_TRUE(adapter.requestSourceStillCurrent(*captured_request));
    ASSERT_TRUE(adapter.finalizeTubeEpoch(captured_request, built));

    const std::shared_ptr<const TubeEpochSnapshot> candidate =
        std::atomic_load(&adapter.latest_candidate_epoch_snapshot_);
    ASSERT_TRUE(candidate);
    EXPECT_EQ(candidate->map_observation_sequence, sequence);
    EXPECT_GT(candidate->epoch_status.candidate_sequence,
              previous_candidate_sequence);
    previous_candidate_sequence = candidate->epoch_status.candidate_sequence;
    ASSERT_TRUE(candidate->candidate_profile);
    ASSERT_TRUE(candidate->active_profile);
    EXPECT_TRUE(candidate->epoch_status.active_available);
    EXPECT_EQ(candidate->epoch_status.state, TubeEpochState::ROLLING);

    EXPECT_FALSE(adapter.update(current, output));
    ASSERT_TRUE(output.candidate_profile);
    EXPECT_EQ(output.tube_epoch_status.candidate_map_observation_sequence,
              sequence);
    ASSERT_TRUE(output.active_profile);
    EXPECT_TRUE(output.tube_epoch_status.active_available);
    EXPECT_EQ(output.tube_epoch_status.state, TubeEpochState::ROLLING);
    MatchedAdapterMarkerBundle markers;
    ASSERT_TRUE(adapter.buildMarkers(current, output, markers));
    ExpectActions(markers.tube_candidate, visualization_msgs::Marker::ADD);
    ExpectActions(markers.tube, visualization_msgs::Marker::ADD);

    const std::shared_ptr<const TubeBuildRequest> latest_request =
        std::atomic_load(&adapter.latest_build_request_);
    const std::shared_ptr<const ControlPublishSnapshot> control =
        std::atomic_load(&adapter.latest_control_snapshot_);
    ASSERT_TRUE(latest_request);
    ASSERT_TRUE(control);
    EXPECT_TRUE(adapter.matchingEpochForRequest(latest_request));
    EXPECT_FALSE(control->candidate_only);
    EXPECT_EQ(control->map_observation_sequence, sequence + 1U);
    EXPECT_EQ(control->candidate_epoch_map_observation_sequence, sequence);
    EXPECT_EQ(control->candidate_epoch_source_revision,
              control->source_revision);
  }

  // The next completed epoch replaces the sample-held epoch atomically and
  // carries the latest request's snapshot provenance.
  MatchedAdapterInput stable = input_for(sequence, 10.0 * kDt);
  EXPECT_FALSE(adapter.update(stable, output));
  const std::shared_ptr<const TubeBuildRequest> stable_request =
      std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(stable_request);
  TubeEpochSnapshot stable_built;
  adapter.buildTubeEpoch(stable_request, stable_built);
  ASSERT_TRUE(adapter.finalizeTubeEpoch(stable_request, stable_built));
  EXPECT_FALSE(adapter.update(stable, output));
  ASSERT_TRUE(output.candidate_profile);
  ASSERT_TRUE(output.active_profile);
  EXPECT_EQ(output.tube_epoch_status.candidate_map_observation_sequence,
            sequence);
  EXPECT_EQ(output.tube_epoch_status.active_map_observation_sequence,
            sequence);
  EXPECT_FALSE(std::atomic_load(&adapter.latest_control_snapshot_)->candidate_only);
}

TEST(PhaseOffsetMatchedAdapterTest,
     PathStaleBuildEmitsRawAndCloudExactlyOnceWithoutCandidateOrRuntimeExposure) {
  SyntheticPath path = MakePath();
  MakeStraightRawPath(path);
  SDFMap map;
  InitializeFineKnownRawFreeMap(map);
  InstallCompleteCloudSnapshot(map, 301U);
  int first_identity = 110;
  int revised_identity = 111;
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::ESDF));
  MatchedAdapterOutput output;
  MatchedAdapterInput captured = MakeInput(path, &first_identity, 0.0);
  SetInputPositionAndLegacy(captured, path.current.p);
  captured.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
  EXPECT_FALSE(adapter.update(captured, output));
  const std::shared_ptr<const TubeBuildRequest> captured_request =
      std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(captured_request);
  TubeEpochSnapshot built;
  adapter.buildTubeEpoch(captured_request, built);
  ASSERT_TRUE(built.raw_candidate_diagnostics_generated);
  ASSERT_TRUE(built.cloud_snapshot_diagnostics_generated);

  MatchedAdapterInput revised = MakeInput(path, &revised_identity, kDt);
  SetInputPositionAndLegacy(revised, path.current.p);
  revised.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
  EXPECT_FALSE(adapter.update(revised, output));
  EXPECT_FALSE(adapter.requestSourceStillCurrent(*captured_request));
  EXPECT_FALSE(adapter.finalizeTubeEpoch(captured_request, built));
  EXPECT_EQ(adapter.timer_last_raw_diagnostic_build_sequence_,
            built.build_sequence);
  EXPECT_EQ(adapter.timer_last_cloud_diagnostic_build_sequence_,
            built.build_sequence);
  EXPECT_FALSE(std::atomic_load(&adapter.latest_candidate_epoch_snapshot_));
  EXPECT_FALSE(std::atomic_load(&adapter.latest_epoch_snapshot_));

  // The delivery decision is one-shot even when the same historical build is
  // inspected again; no path-stale Candidate/Runtime object is resurrected.
  adapter.publishBuildDiagnostics(built);
  EXPECT_EQ(adapter.timer_last_raw_diagnostic_build_sequence_,
            built.build_sequence);
  EXPECT_EQ(adapter.timer_last_cloud_diagnostic_build_sequence_,
            built.build_sequence);
  EXPECT_FALSE(std::atomic_load(&adapter.latest_candidate_epoch_snapshot_));
  EXPECT_FALSE(std::atomic_load(&adapter.latest_epoch_snapshot_));
}

TEST(PhaseOffsetMatchedAdapterTest,
     NewerSameKeyControlDoesNotDiscardPendingBuild) {
  const SyntheticPath path = MakePath();
  int identity = 104;
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  MatchedAdapterInput first = MakeInput(path, &identity, 0.0);
  adapter.update(first, output);
  const std::shared_ptr<const TubeBuildRequest> first_request =
      std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(first_request);
  MatchedAdapterInput newer = MakeInput(path, &identity, kDt);
  newer.position.y() += 0.02;
  SetInputPositionAndLegacy(newer, newer.position);
  adapter.update(newer, output);
  EXPECT_TRUE(adapter.requestSourceStillCurrent(*first_request));
  ASSERT_TRUE(adapter.timerTick());
  const std::shared_ptr<const TubeEpochSnapshot> epoch =
      std::atomic_load(&adapter.latest_epoch_snapshot_);
  ASSERT_TRUE(epoch);
  EXPECT_EQ(epoch->source_revision, first_request->source_revision);
  EXPECT_EQ(epoch->request_control_sequence, adapter.control_sequence_);
}

TEST(PhaseOffsetMatchedAdapterTest,
     TimerIsNonReentrantAndDeactivateStopsRebuildWithoutResettingRuntime) {
  const SyntheticPath path = MakePath();
  int identity = 105;
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  BuildAndConsume(adapter, MakeInput(path, &identity), output);
  const std::uint64_t completed_builds = adapter.timer_build_sequence_;
  const double retained_delta = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand previous =
      adapter.runtime_->previousFinalPort();

  adapter.timer_inflight_.store(true);
  EXPECT_FALSE(adapter.timerTick());
  EXPECT_EQ(adapter.timer_build_sequence_, completed_builds);
  adapter.timer_inflight_.store(false);

  adapter.deactivate(ros::Time(2.0));
  const std::shared_ptr<const TubeBuildRequest> inactive =
      std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(inactive);
  EXPECT_FALSE(inactive->active);
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), retained_delta);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_w, previous.u_w);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_delta, previous.u_delta);
  EXPECT_TRUE(adapter.timerTick());
  EXPECT_EQ(adapter.timer_build_sequence_, completed_builds);
  const std::uint64_t deactivate_sequence =
      adapter.timer_last_deactivate_sequence_;
  EXPECT_TRUE(adapter.timerTick());
  EXPECT_EQ(adapter.timer_last_deactivate_sequence_, deactivate_sequence);
}

TEST(PhaseOffsetMatchedAdapterTest,
     DeactivateDiscardsStagedPositionCommandWithoutResettingCommittedRuntime) {
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  adapter.command_active_ = true;
  adapter.pending_authority_valid_ = true;
  adapter.pending_authority_session_ = 17U;
  adapter.pending_runtime_commit_.valid = true;
  adapter.pending_authority_prepared_.valid = true;

  adapter.deactivate(ros::Time(1.0));

  EXPECT_FALSE(adapter.pending_authority_valid_);
  EXPECT_EQ(adapter.pending_authority_session_, 0U);
  EXPECT_FALSE(adapter.pending_runtime_commit_.valid);
  EXPECT_FALSE(adapter.pending_authority_prepared_.valid);
  EXPECT_FALSE(adapter.command_active_);
}

TEST(PhaseOffsetMatchedAdapterTest,
     PendingPositionCommandPublishesBeforeAuthorityAndRuntimeCommit) {
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  ASSERT_TRUE(StageValidPendingPositionCommand(adapter));
  std::shared_ptr<ControlPublishSnapshot> control(new ControlPublishSnapshot());
  control->active = true;
  control->task_generation = adapter.task_generation_.load(
      std::memory_order_acquire);
  std::atomic_store(&adapter.latest_control_snapshot_,
                    std::shared_ptr<const ControlPublishSnapshot>(control));
  const PendingPositionCommandCapture capture =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(capture.pending);
  ASSERT_TRUE(capture.valid);

  int callback_count = 0;
  bool callback_saw_uncommitted_state = false;
  EXPECT_TRUE(adapter.publishPendingPositionCommand(
      [&]() {
        ++callback_count;
        callback_saw_uncommitted_state =
            !adapter.execution_authority_.snapshot().valid &&
            std::abs(adapter.runtime_->retainedDelta()) <= 1e-12 &&
            adapter.pending_authority_valid_;
        return true;
      }, capture.identity));
  EXPECT_EQ(callback_count, 1);
  EXPECT_TRUE(callback_saw_uncommitted_state);
  EXPECT_TRUE(adapter.execution_authority_.snapshot().valid);
  EXPECT_NEAR(adapter.runtime_->retainedDelta(), -0.002, 1e-12);
  EXPECT_FALSE(adapter.hasPendingPositionCommand());
  const std::shared_ptr<const ControlPublishSnapshot> published_control =
      std::atomic_load(&adapter.latest_control_snapshot_);
  ASSERT_TRUE(published_control);
  ASSERT_TRUE(published_control->authority_snapshot);
  EXPECT_EQ(published_control->authority_snapshot->snapshotId(),
            adapter.execution_authority_.snapshot().snapshotId());
}

TEST(PhaseOffsetMatchedAdapterTest,
     PendingPositionCommandPublishFailureDiscardsWithoutCommit) {
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  ASSERT_TRUE(StageValidPendingPositionCommand(adapter));
  const PendingPositionCommandCapture capture =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(capture.pending);
  const double delta_before = adapter.runtime_->retainedDelta();
  int callback_count = 0;
  EXPECT_FALSE(adapter.publishPendingPositionCommand(
      [&]() {
        ++callback_count;
        return false;
      }, capture.identity));
  EXPECT_EQ(callback_count, 1);
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), delta_before);
  EXPECT_FALSE(adapter.execution_authority_.snapshot().valid);
  EXPECT_FALSE(adapter.hasPendingPositionCommand());
}

TEST(PhaseOffsetMatchedAdapterTest,
     PendingPositionCommandIdentityAndStateConflictsFailClosed) {
  {
    PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
    ASSERT_TRUE(StageValidPendingPositionCommand(adapter));
    const PendingPositionCommandCapture capture =
        adapter.capturePendingPositionCommand();
    adapter.authority_session_.store(8U, std::memory_order_release);
    int callback_count = 0;
    EXPECT_FALSE(adapter.publishPendingPositionCommand(
        [&]() { ++callback_count; return true; }, capture.identity));
    EXPECT_EQ(callback_count, 0);
    EXPECT_FALSE(adapter.execution_authority_.snapshot().valid);
    EXPECT_FALSE(adapter.hasPendingPositionCommand());
  }
  {
    PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
    ASSERT_TRUE(StageValidPendingPositionCommand(adapter));
    const PendingPositionCommandCapture capture =
        adapter.capturePendingPositionCommand();
    adapter.pending_runtime_commit_.expected_delta = 0.5;
    int callback_count = 0;
    EXPECT_FALSE(adapter.publishPendingPositionCommand(
        [&]() { ++callback_count; return true; }, capture.identity));
    EXPECT_EQ(callback_count, 0);
    EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), 0.0);
    EXPECT_FALSE(adapter.execution_authority_.snapshot().valid);
  }
  {
    PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
    ASSERT_TRUE(StageValidPendingPositionCommand(adapter));
    const PendingPositionCommandCapture capture =
        adapter.capturePendingPositionCommand();
    phase_offset_navigation::ActiveReferenceSnapshot changed =
        *adapter.pending_authority_prepared_.committed_snapshot;
    changed.executed_path_revision = 2U;
    adapter.pending_authority_prepared_.committed_snapshot =
        std::make_shared<const phase_offset_navigation::ActiveReferenceSnapshot>(
            changed);
    int callback_count = 0;
    EXPECT_FALSE(adapter.publishPendingPositionCommand(
        [&]() { ++callback_count; return true; }, capture.identity));
    EXPECT_EQ(callback_count, 0);
    EXPECT_FALSE(adapter.execution_authority_.snapshot().valid);
  }
  {
    PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
    ASSERT_TRUE(StageValidPendingPositionCommand(adapter));
    const PendingPositionCommandCapture capture =
        adapter.capturePendingPositionCommand();
    phase_offset_navigation::ActiveReferenceSnapshot changed =
        *adapter.pending_authority_prepared_.committed_snapshot;
    changed.owner_mode =
        phase_offset_navigation::ActiveReferenceOwnerMode::RECOVERY;
    changed.selected_u_owner = "PhaseOffsetRecoveryOwner";
    adapter.pending_authority_prepared_.committed_snapshot =
        std::make_shared<const phase_offset_navigation::ActiveReferenceSnapshot>(
            changed);
    int callback_count = 0;
    EXPECT_FALSE(adapter.publishPendingPositionCommand(
        [&]() { ++callback_count; return true; }, capture.identity));
    EXPECT_EQ(callback_count, 0);
    EXPECT_FALSE(adapter.execution_authority_.snapshot().valid);
  }
}

TEST(PhaseOffsetMatchedAdapterTest,
     ResetAfterPendingCaptureRejectsStalePositionPublication) {
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  ASSERT_TRUE(StageValidPendingPositionCommand(adapter));
  const PendingPositionCommandCapture capture =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(capture.pending);
  std::uint64_t retired = 0U;
  ASSERT_TRUE(adapter.resetForNewNavigationTask(
      adapter.authority_session_.load(std::memory_order_acquire), retired));
  int callback_count = 0;
  EXPECT_FALSE(adapter.publishPendingPositionCommand(
      [&]() { ++callback_count; return true; }, capture.identity));
  EXPECT_EQ(callback_count, 0);
  EXPECT_FALSE(adapter.execution_authority_.snapshot().valid);
  EXPECT_FALSE(adapter.hasPendingPositionCommand());
}

TEST(PhaseOffsetMatchedAdapterTest,
     InvalidPathOrLegacyClearsAsyncRequestAndCannotRebuildIt) {
  const SyntheticPath path = MakePath();
  int identity = 106;
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  BuildAndConsume(adapter, MakeInput(path, &identity), output);
  const std::uint64_t builds_before = adapter.timer_build_sequence_;

  MatchedAdapterInput invalid_path = MakeInput(path, &identity, 1.0);
  invalid_path.path.valid = false;
  EXPECT_FALSE(adapter.update(invalid_path, output));
  EXPECT_TRUE(adapter.timerTick());
  EXPECT_EQ(adapter.timer_build_sequence_, builds_before);
  std::shared_ptr<const TubeBuildRequest> request =
      std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(request);
  EXPECT_FALSE(request->active);

  BuildAndConsume(adapter, MakeInput(path, &identity, 2.0), output);
  const std::uint64_t rebuilt = adapter.timer_build_sequence_;
  MatchedAdapterInput invalid_legacy = MakeInput(path, &identity, 3.0);
  invalid_legacy.legacy.valid = false;
  EXPECT_FALSE(adapter.update(invalid_legacy, output));
  EXPECT_TRUE(adapter.timerTick());
  EXPECT_EQ(adapter.timer_build_sequence_, rebuilt);
}

TEST(PhaseOffsetMatchedAdapterTest,
     TimerOwnsOneShotEpochDueBitForTheShrunkenFiftyFieldSchema) {
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  EXPECT_TRUE(adapter.markEpochBuildPublished(7U));
  TubeEpochDiagnosticsInput diagnostics;
  diagnostics.tube_update_due_this_cycle = true;
  EXPECT_DOUBLE_EQ(makeTubeEpochDiagnostics(diagnostics)[kEpochTubeUpdateDueThisCycle],
                   1.0);
  EXPECT_FALSE(adapter.markEpochBuildPublished(7U));
  diagnostics.tube_update_due_this_cycle = false;
  EXPECT_DOUBLE_EQ(makeTubeEpochDiagnostics(diagnostics)[kEpochTubeUpdateDueThisCycle],
                   0.0);
  EXPECT_TRUE(adapter.markEpochBuildPublished(8U));
}

TEST(PhaseOffsetMatchedAdapterTest,
     ConcurrentCommandAndTimerStressKeepsSameKeyEpochsConsistent) {
  const SyntheticPath path = MakePath();
  int identity = 107;
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::FIXED);
  config.observe_only = true;
  PhaseOffsetMatchedAdapter adapter(config);
  std::atomic<bool> start(false);
  std::vector<std::uint64_t> timer_build_sequences;

  std::thread command_thread([&]() {
    while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
    MatchedAdapterOutput output;
    for (int cycle = 0; cycle < 160; ++cycle) {
      adapter.update(MakeInput(path, &identity, cycle * kDt), output);
      // Deactivation is command-owned; it deliberately interleaves with the
      // independent timer thread without touching Runtime state.
      if (cycle == 47 || cycle == 103) {
        adapter.deactivate(ros::Time(cycle * kDt));
      }
    }
  });
  std::thread timer_thread([&]() {
    while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
    for (int iteration = 0; iteration < 120; ++iteration) {
      adapter.timerTick();
      timer_build_sequences.push_back(adapter.timer_build_sequence_);
    }
  });
  start.store(true, std::memory_order_release);
  command_thread.join();
  timer_thread.join();

  for (std::size_t index = 1U; index < timer_build_sequences.size(); ++index) {
    EXPECT_LE(timer_build_sequences[index - 1U], timer_build_sequences[index]);
  }
  MatchedAdapterOutput final_output;
  adapter.update(MakeInput(path, &identity, 4.0), final_output);
  adapter.timerTick();
  adapter.update(MakeInput(path, &identity, 4.0 + kDt), final_output);
  const std::shared_ptr<const TubeBuildRequest> request =
      std::atomic_load(&adapter.latest_build_request_);
  const std::shared_ptr<const TubeEpochSnapshot> epoch =
      std::atomic_load(&adapter.latest_epoch_snapshot_);
  ASSERT_TRUE(request);
  ASSERT_TRUE(epoch);
  EXPECT_TRUE(adapter.epochMatchesRequest(*epoch, *request));
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), 0.0);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_w, 0.0);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_delta, 0.0);
}

TEST(PhaseOffsetMatchedAdapterTest,
     ImmutableRequestRetainsOwnersUntilDeactivateAndTimerClear) {
  const SyntheticPath path = MakePath();
  auto mutable_path = std::make_shared<ContinuousPhasePath>();
  ASSERT_TRUE(mutable_path->appendSegment(
      0.0, 3.0, "synthetic",
      [](const double w, ContinuousPhasePathState& state) {
        const phase_offset_core::PathDifferentialState source = MakeState(w);
        state.p = source.p;
        state.dp_dw = source.p_w;
        state.d2p_dw2 = source.p_ww;
        state.valid = source.valid;
        return true;
      }));
  std::shared_ptr<const ContinuousPhasePath> path_owner = mutable_path;
  auto cloud_owner = std::make_shared<plan_env::CloudOccupancySnapshot>();
  cloud_owner->valid = true;
  cloud_owner->observation_sequence = 108U;
  cloud_owner->map_min = Eigen::Vector3d(-5.0, -5.0, -5.0);
  cloud_owner->map_max = Eigen::Vector3d(5.0, 5.0, 5.0);
  cloud_owner->observed_min = cloud_owner->map_min;
  cloud_owner->observed_max = cloud_owner->map_max;
  cloud_owner->grid_origin = cloud_owner->map_min;
  cloud_owner->voxel_count = Eigen::Vector3i(10, 10, 10);
  cloud_owner->resolution = 1.0;
  cloud_owner->included_map_inflation = 0.10;
  cloud_owner->occupied.assign(1000U, 0U);
  ASSERT_TRUE(plan_env::cloudOccupancySnapshotConsistent(*cloud_owner));
  std::weak_ptr<const ContinuousPhasePath> weak_path = path_owner;
  std::weak_ptr<const plan_env::CloudOccupancySnapshot> weak_cloud = cloud_owner;

  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::ESDF));
  MatchedAdapterInput input = MakeInput(path, &path);
  input.sampled_path.clear();
  input.semantic_path_owner = path_owner;
  input.semantic_path_start_w = 0.0;
  input.semantic_path_end_w = 3.0;
  input.cloud_occupancy_snapshot = cloud_owner;
  SetInputPositionAndLegacy(input, path.current.p);
  MatchedAdapterOutput output;
  adapter.update(input, output);
  mutable_path.reset();
  path_owner.reset();
  cloud_owner.reset();
  input.semantic_path_owner.reset();
  input.cloud_occupancy_snapshot.reset();
  EXPECT_FALSE(weak_path.expired());
  EXPECT_FALSE(weak_cloud.expired());
  adapter.timerTick();
  adapter.deactivate(ros::Time(5.0));
  adapter.timerTick();
  EXPECT_TRUE(weak_path.expired());
  EXPECT_TRUE(weak_cloud.expired());
}

// Stage 1A scheduler/lifecycle coverage.  These tests intentionally inspect
// the adapter's private worker bookkeeping (the test file already exposes it
// above) so that one-slot/latest-only behavior is proven without requiring a
// ROS master or a long-running Tube build.
TEST(PhaseOffsetMatchedAdapterStage1A,
     SchedulerReturnsWithoutSynchronousTubeBuild) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  ASSERT_FALSE(adapter.update(MakeInput(path, &path), output));
  const std::uint64_t before = adapter.timer_build_sequence_;
  EXPECT_TRUE(adapter.scheduleTubeBuild());
  EXPECT_EQ(before, adapter.timer_build_sequence_);
  EXPECT_TRUE(adapter.pending_request_);
  EXPECT_EQ(1U, adapter.pending_request_ ? 1U : 0U);
}

TEST(PhaseOffsetMatchedAdapterStage1A, LatestOnlyCoalescesAThroughDToD) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  int ids[4] = {1, 2, 3, 4};
  std::shared_ptr<const TubeBuildRequest> requests[4];
  for (int i = 0; i < 4; ++i) {
    ASSERT_FALSE(adapter.update(MakeInput(path, &ids[i], i * kDt), output));
    ASSERT_TRUE(adapter.scheduleTubeBuild());
    requests[i] = std::atomic_load(&adapter.latest_build_request_);
  }
  ASSERT_TRUE(adapter.pending_request_);
  EXPECT_EQ(requests[3].get(), adapter.pending_request_.get());
  EXPECT_TRUE(adapter.pending_request_);
}

TEST(PhaseOffsetMatchedAdapterStage1A, PendingSlotNeverExceedsOne) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  for (int i = 0; i < 32; ++i) {
    ASSERT_FALSE(adapter.update(MakeInput(path, &i, i * kDt), output));
    adapter.scheduleTubeBuild();
    EXPECT_LE(adapter.pending_request_ ? 1U : 0U, 1U);
  }
}

TEST(PhaseOffsetMatchedAdapterStage1A,
     PendingRequestMadeStaleBeforeWorkerStartDoesNotBuild) {
  const SyntheticPath path = MakePath();
  int identity_a = 201;
  int identity_b = 202;
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;

  ASSERT_FALSE(adapter.update(MakeInput(path, &identity_a), output));
  const auto request_a = std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(request_a);
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  ASSERT_TRUE(adapter.pending_request_);
  EXPECT_EQ(request_a.get(), adapter.pending_request_.get());

  // Capture and consume exactly the permit that a worker would have taken.
  // Calling runTubeBuildJob() directly keeps this stale-before-worker-start
  // interleaving deterministic and proves the pre-build gate drops A before
  // any worker-local state or timer-build sequence is consumed.
  PhaseOffsetMatchedAdapter::SchedulePermit permit_a;
  {
    std::lock_guard<std::mutex> lock(adapter.worker_state_mutex_);
    permit_a.permit_id = adapter.pending_permit_id_;
    permit_a.request = adapter.pending_request_;
    permit_a.request_identity = adapter.pending_request_identity_;
    permit_a.work_identity = adapter.pending_work_identity_;
    adapter.pending_request_.reset();
    adapter.pending_permit_id_ = 0U;
    adapter.pending_request_identity_ =
        PhaseOffsetMatchedAdapter::RequestInstanceIdentity();
    adapter.pending_work_identity_ =
        PhaseOffsetMatchedAdapter::TubeWorkIdentity();
  }
  ASSERT_EQ(request_a.get(), permit_a.request.get());

  // Replace the immutable command request after the permit captured A but
  // before the unadvertised compatibility worker starts.  The worker must
  // drop stale A before prepareTubeJobLocalState()/timer_build_sequence.
  ASSERT_FALSE(adapter.update(MakeInput(path, &identity_b, kDt), output));
  const auto request_b = std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(request_b);
  EXPECT_NE(request_a.get(), request_b.get());
  ASSERT_FALSE(adapter.runTubeBuildJob(permit_a));

  EXPECT_EQ(0U, adapter.timer_build_sequence_);
  EXPECT_FALSE(adapter.pending_request_);
  EXPECT_FALSE(std::atomic_load(&adapter.latest_candidate_epoch_snapshot_));
  EXPECT_FALSE(std::atomic_load(&adapter.latest_epoch_snapshot_));
  ASSERT_TRUE(adapter.tube_epoch_manager_);
  EXPECT_EQ(0U, adapter.tube_epoch_manager_->candidate_sequence_);
  EXPECT_EQ(0U, adapter.cached_path_source_revision_);
  EXPECT_FALSE(adapter.have_cached_path_);
  EXPECT_FALSE(adapter.timer_active_profile_);
  EXPECT_EQ(0U, adapter.timer_installed_active_epoch_);

  // No FIFO/catch-up: B is selected only by the next timer permit (the next
  // compatibility tick below), never by the stale A completion itself.
  EXPECT_EQ(0U, adapter.timer_build_sequence_);
  ASSERT_TRUE(adapter.timerTick());
  EXPECT_EQ(1U, adapter.timer_build_sequence_);
  const auto committed = std::atomic_load(&adapter.latest_epoch_snapshot_);
  ASSERT_TRUE(committed);
  EXPECT_EQ(request_b->source_revision, committed->source_revision);
}

TEST(PhaseOffsetMatchedAdapterStage1A,
     NoDuplicateRebuildWithoutNewPermitOrTubeIdentity) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  ASSERT_FALSE(adapter.update(MakeInput(path, &path), output));
  ASSERT_TRUE(adapter.timerTick());
  const std::uint64_t completed = adapter.timer_build_sequence_;
  EXPECT_FALSE(adapter.scheduleTubeBuild());
  EXPECT_EQ(completed, adapter.timer_build_sequence_);
}

TEST(PhaseOffsetMatchedAdapterStage1A,
     Normal50HzCommandTrafficCannotCause50HzTubeBuildChurn) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  int identity = 55;
  for (int i = 0; i < 50; ++i) {
    ASSERT_FALSE(adapter.update(MakeInput(path, &identity, i * kDt), output));
  }
  EXPECT_EQ(0U, adapter.timer_build_sequence_);
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  ASSERT_TRUE(adapter.timerTick());
  EXPECT_EQ(1U, adapter.timer_build_sequence_);
}

TEST(PhaseOffsetMatchedAdapterStage1A,
     RequestArrivingAtWorkerCompletionWaitsAtMostNextPermitAndDoesNotFIFO) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  int ids[4] = {61, 62, 63, 64};

  // Start the joined worker directly so this remains a no-master deterministic
  // fixture.  The bounded finalization hook holds A at the completion seam
  // while B/C/D permits replace one pending slot.
  {
    std::lock_guard<std::mutex> lock(adapter.worker_state_mutex_);
    adapter.worker_started_ = true;
    adapter.worker_stop_requested_ = false;
  }
  adapter.tube_worker_ = std::thread(&PhaseOffsetMatchedAdapter::tubeWorkerMain,
                                    &adapter);
  ASSERT_FALSE(adapter.update(MakeInput(path, &ids[0], 0.0), output));
  const auto request_a = std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(request_a);

  std::mutex hook_mutex;
  std::condition_variable hook_cv;
  int hook_count = 0;
  bool release_first = false;
  bool release_second = false;
  adapter.finalize_publication_test_hook_ = [&]() {
    std::unique_lock<std::mutex> lock(hook_mutex);
    ++hook_count;
    hook_cv.notify_all();
    if (hook_count == 1) {
      hook_cv.wait(lock, [&]() { return release_first; });
    } else if (hook_count == 2) {
      hook_cv.wait(lock, [&]() { return release_second; });
    }
  };
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  {
    std::unique_lock<std::mutex> lock(hook_mutex);
    hook_cv.wait(lock, [&]() { return hook_count >= 1; });
  }

  std::shared_ptr<const TubeBuildRequest> request_d;
  for (int i = 1; i < 4; ++i) {
    ASSERT_FALSE(adapter.update(MakeInput(path, &ids[i], i * kDt), output));
    ASSERT_TRUE(adapter.scheduleTubeBuild());
    if (i == 3) request_d = std::atomic_load(&adapter.latest_build_request_);
  }
  ASSERT_TRUE(request_d);
  {
    std::lock_guard<std::mutex> lock(adapter.worker_state_mutex_);
    ASSERT_TRUE(adapter.pending_request_);
    EXPECT_EQ(request_d.get(), adapter.pending_request_.get());
    EXPECT_EQ(1U, adapter.pending_request_ ? 1U : 0U);
  }

  {
    std::lock_guard<std::mutex> lock(hook_mutex);
    release_first = true;
  }
  hook_cv.notify_all();
  {
    std::unique_lock<std::mutex> lock(hook_mutex);
    hook_cv.wait(lock, [&]() { return hook_count >= 2; });
  }
  {
    std::lock_guard<std::mutex> lock(adapter.worker_state_mutex_);
    EXPECT_EQ(request_d.get(), adapter.running_request_.get());
    EXPECT_EQ(request_d.get(), adapter.last_started_request_identity_.request);
    EXPECT_FALSE(adapter.pending_request_);
  }
  EXPECT_EQ(2U, adapter.timer_build_sequence_);
  {
    std::lock_guard<std::mutex> lock(hook_mutex);
    release_second = true;
  }
  hook_cv.notify_all();
  adapter.shutdown();
  adapter.finalize_publication_test_hook_ = std::function<void()>();
}

TEST(PhaseOffsetMatchedAdapterStage1A,
     StaleOrdinaryOwnerOrFrameCompletionCannotPublish) {
  const SyntheticPath path = MakePath();
  const auto owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput first = MakeInput(path, owner.get());
  first.semantic_path_owner = owner;
  MatchedAdapterOutput output;
  ASSERT_FALSE(adapter.update(first, output));
  const auto request = std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(request);
  TubeEpochSnapshot built;
  ASSERT_TRUE(adapter.buildTubeEpoch(request, built));
  MatchedAdapterInput replacement = first;
  replacement.frame_owner = std::make_shared<const ContinuousPhaseNormalFrame>(
      owner, 1U, 2U);
  ASSERT_FALSE(adapter.update(replacement, output));
  EXPECT_FALSE(adapter.finalizeTubeEpoch(request, built));
  EXPECT_FALSE(std::atomic_load(&adapter.latest_epoch_snapshot_));
}

TEST(PhaseOffsetMatchedAdapterStage1A,
     SourceOrFrameReplacementCannotBeFollowedByOldCandidateResurrection) {
  const SyntheticPath path = MakePath();
  const auto owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput first = MakeInput(path, owner.get());
  first.semantic_path_owner = owner;
  first.frame_owner = std::make_shared<const ContinuousPhaseNormalFrame>(
      owner, 1U, 1U);
  MatchedAdapterOutput output;
  ASSERT_FALSE(adapter.update(first, output));
  const auto request_a = std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(request_a);
  PhaseOffsetMatchedAdapter::SchedulePermit permit_a;
  permit_a.request = request_a;
  permit_a.request_identity = adapter.makeRequestInstanceIdentity(request_a);
  permit_a.work_identity = adapter.makeTubeWorkIdentity(request_a);
  PhaseOffsetMatchedAdapter::TubeJobLocalState local_a;
  adapter.prepareTubeJobLocalState(permit_a, local_a);
  TubeEpochSnapshot built_a;
  ASSERT_TRUE(adapter.buildTubeEpoch(request_a, built_a, &local_a));

  std::mutex hook_mutex;
  std::condition_variable hook_cv;
  bool entered = false;
  bool release = false;
  adapter.finalize_before_epoch_store_test_hook_ = [&]() {
    std::unique_lock<std::mutex> lock(hook_mutex);
    entered = true;
    hook_cv.notify_all();
    hook_cv.wait(lock, [&]() { return release; });
  };
  bool finalized = false;
  std::thread finalizer([&]() {
    finalized = adapter.finalizeTubeEpoch(request_a, built_a, &local_a);
  });
  {
    std::unique_lock<std::mutex> lock(hook_mutex);
    hook_cv.wait(lock, [&]() { return entered; });
  }

  // Currentness has passed and runtime_command_mutex_ is held by the
  // finalizer.  A concurrent frame-owner replacement can start, but update()
  // must remain blocked until the immutable A stores complete.
  MatchedAdapterInput replacement = first;
  replacement.frame_owner = std::make_shared<const ContinuousPhaseNormalFrame>(
      owner, 1U, 2U);
  bool replacement_started = false;
  bool replacement_done = false;
  std::thread replacer([&]() {
    {
      std::lock_guard<std::mutex> lock(hook_mutex);
      replacement_started = true;
      hook_cv.notify_all();
    }
    MatchedAdapterOutput replacement_output;
    adapter.update(replacement, replacement_output);
    {
      std::lock_guard<std::mutex> lock(hook_mutex);
      replacement_done = true;
      hook_cv.notify_all();
    }
  });
  {
    std::unique_lock<std::mutex> lock(hook_mutex);
    hook_cv.wait(lock, [&]() { return replacement_started; });
    EXPECT_FALSE(replacement_done);
  }
  {
    std::lock_guard<std::mutex> lock(hook_mutex);
    release = true;
  }
  hook_cv.notify_all();
  finalizer.join();
  replacer.join();
  adapter.finalize_before_epoch_store_test_hook_ = std::function<void()>();

  EXPECT_TRUE(finalized);
  const auto request_b = std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(request_b);
  EXPECT_EQ(request_a->source_revision, request_b->source_revision);
  EXPECT_NE(request_a->frame_owner.get(), request_b->frame_owner.get());
  EXPECT_FALSE(std::atomic_load(&adapter.latest_candidate_epoch_snapshot_));
  EXPECT_FALSE(std::atomic_load(&adapter.latest_epoch_snapshot_));
}

TEST(PhaseOffsetMatchedAdapterStage1A,
     StaleOrdinaryJobCannotMutateCommittedWorkerState) {
  const SyntheticPath path = MakePath();
  int identity = 71;
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  ASSERT_FALSE(adapter.update(MakeInput(path, &identity), output));
  const auto request = std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(request);
  PhaseOffsetMatchedAdapter::SchedulePermit permit;
  permit.request = request;
  permit.request_identity = adapter.makeRequestInstanceIdentity(request);
  permit.work_identity = adapter.makeTubeWorkIdentity(request);
  PhaseOffsetMatchedAdapter::TubeJobLocalState local;
  adapter.prepareTubeJobLocalState(permit, local);
  TubeEpochSnapshot built;
  ASSERT_TRUE(adapter.buildTubeEpoch(request, built, &local));
  ASSERT_EQ(0U, adapter.tube_epoch_manager_->candidate_sequence_);
  adapter.deactivate(ros::Time(1.0));
  EXPECT_FALSE(adapter.finalizeTubeEpoch(request, built, &local));
  EXPECT_EQ(0U, adapter.tube_epoch_manager_->candidate_sequence_);
}

TEST(PhaseOffsetMatchedAdapterStage1A,
     StalePairRefreshCannotMutatePersistentManagerOrNewPair) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  int identity = 81;
  ASSERT_FALSE(adapter.update(MakeInput(path, &identity), output));
  ASSERT_TRUE(adapter.timerTick());
  const auto old_manager = adapter.tube_epoch_manager_.get();
  const std::uint64_t old_sequence = old_manager->candidate_sequence_;
  const auto request = std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(request);
  PhaseOffsetMatchedAdapter::SchedulePermit permit;
  permit.request = std::shared_ptr<const TubeBuildRequest>(
      new TubeBuildRequest(*request));
  std::shared_ptr<TubeBuildRequest> pair_request(
      new TubeBuildRequest(*request));
  pair_request->base_path_tube_pair = std::shared_ptr<const PathTubePair>(
      new PathTubePair());
  permit.request = std::shared_ptr<const TubeBuildRequest>(pair_request);
  permit.request_identity = adapter.makeRequestInstanceIdentity(permit.request);
  permit.work_identity = adapter.makeTubeWorkIdentity(permit.request);
  PhaseOffsetMatchedAdapter::TubeJobLocalState local;
  adapter.prepareTubeJobLocalState(permit, local);
  EXPECT_TRUE(local.pair_refresh);
  EXPECT_NE(old_manager, local.tube_epoch_manager.get());
  EXPECT_EQ(old_sequence, adapter.tube_epoch_manager_->candidate_sequence_);
}

TEST(PhaseOffsetMatchedAdapterStage1A, TaskGenerationResetInvalidatesRunningJob) {
  const SyntheticPath path = MakePath();
  int identity = 91;
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  ASSERT_FALSE(adapter.update(MakeInput(path, &identity), output));
  const auto request = std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(request);
  TubeEpochSnapshot built;
  ASSERT_TRUE(adapter.buildTubeEpoch(request, built));
  std::uint64_t retired = 0U;
  ASSERT_TRUE(adapter.resetForNewNavigationTask(
      adapter.authority_session_.load(std::memory_order_acquire), retired));
  EXPECT_FALSE(adapter.finalizeTubeEpoch(request, built));
  EXPECT_FALSE(std::atomic_load(&adapter.latest_candidate_epoch_snapshot_));
}

TEST(PhaseOffsetMatchedAdapterStage1A,
     DeactivateDuringBuildProducesCorrectLifecycle) {
  const SyntheticPath path = MakePath();
  int identity = 101;
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  ASSERT_FALSE(adapter.update(MakeInput(path, &identity), output));
  const auto request = std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(request);
  TubeEpochSnapshot built;
  ASSERT_TRUE(adapter.buildTubeEpoch(request, built));
  adapter.deactivate(ros::Time(2.0));
  const auto inactive = std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(inactive);
  EXPECT_FALSE(inactive->active);
  EXPECT_FALSE(adapter.finalizeTubeEpoch(request, built));
}

TEST(PhaseOffsetMatchedAdapterStage1A,
     StaleDeactivateCannotClearNewTaskPendingRequest) {
  const SyntheticPath path = MakePath();
  int a_identity = 131;
  int b_identity = 132;
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  ASSERT_FALSE(adapter.update(MakeInput(path, &a_identity), output));
  const std::shared_ptr<const TubeBuildRequest> a_request =
      std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(a_request);

  std::mutex hook_mutex;
  std::condition_variable hook_cv;
  bool paused = false;
  bool release = false;
  adapter.deactivate_test_hook_ = [&]() {
    std::unique_lock<std::mutex> lock(hook_mutex);
    paused = true;
    hook_cv.notify_all();
    hook_cv.wait(lock, [&]() { return release; });
  };
  std::thread old_deactivate([&]() { adapter.deactivate(ros::Time(1.0)); });
  {
    std::unique_lock<std::mutex> lock(hook_mutex);
    hook_cv.wait(lock, [&]() { return paused; });
  }

  // Retire A while its deactivate producer is paused before the command
  // boundary.  Install and schedule B before resuming that stale producer.
  std::uint64_t retired = 0U;
  const std::uint64_t old_session =
      adapter.authority_session_.load(std::memory_order_acquire);
  ASSERT_TRUE(adapter.resetForNewNavigationTask(old_session, retired));
  ASSERT_FALSE(adapter.update(MakeInput(path, &b_identity, kDt), output));
  const std::shared_ptr<const TubeBuildRequest> b_request =
      std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(b_request);
  EXPECT_NE(a_request.get(), b_request.get());
  EXPECT_NE(a_request->source_revision, b_request->source_revision);
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  {
    std::lock_guard<std::mutex> lock(adapter.worker_state_mutex_);
    ASSERT_TRUE(adapter.pending_request_);
    EXPECT_EQ(b_request.get(), adapter.pending_request_.get());
  }

  {
    std::lock_guard<std::mutex> lock(hook_mutex);
    release = true;
  }
  hook_cv.notify_all();
  old_deactivate.join();
  adapter.deactivate_test_hook_ = std::function<void()>();

  // The stale A producer is a no-op at the lifecycle seam: B remains the one
  // pending permit, and only the normal next timer scheduler may consume it.
  {
    std::lock_guard<std::mutex> lock(adapter.worker_state_mutex_);
    ASSERT_TRUE(adapter.pending_request_);
    EXPECT_EQ(b_request.get(), adapter.pending_request_.get());
  }
  EXPECT_EQ(b_request.get(),
            std::atomic_load(&adapter.latest_build_request_).get());
  EXPECT_FALSE(std::atomic_load(&adapter.latest_candidate_epoch_snapshot_));
  EXPECT_FALSE(std::atomic_load(&adapter.latest_epoch_snapshot_));

  ASSERT_TRUE(adapter.timerTick());
  EXPECT_EQ(1U, adapter.timer_build_sequence_);
  EXPECT_FALSE(adapter.pending_request_);
  const auto candidate =
      std::atomic_load(&adapter.latest_candidate_epoch_snapshot_);
  const auto epoch = std::atomic_load(&adapter.latest_epoch_snapshot_);
  ASSERT_TRUE(candidate);
  ASSERT_TRUE(epoch);
  EXPECT_EQ(b_request->source_revision, candidate->source_revision);
  EXPECT_EQ(b_request->source_revision, epoch->source_revision);
  EXPECT_EQ(b_request->control_sequence, candidate->request_control_sequence);
  EXPECT_EQ(b_request->control_sequence, epoch->request_control_sequence);
}

TEST(PhaseOffsetMatchedAdapterStage1A,
     ShutdownJoinsAndPreventsPostShutdownPublication) {
  const SyntheticPath path = MakeStraightSyntheticPath();
  const std::shared_ptr<BlockingOwnerEvaluation> blocking(
      new BlockingOwnerEvaluation());
  const auto owner = MakeBlockingStraightSyntheticOwner(blocking);
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;
  input.sampled_path.clear();
  MatchedAdapterOutput output;
  ASSERT_FALSE(adapter.update(input, output));
  // The command update may sample the semantic owner for preflight.  Arm the
  // one-shot barrier only after that command-side work has completed so the
  // worker's build, rather than update(), is the operation held for shutdown.
  blocking->block_on_call = blocking->call_count.load(
      std::memory_order_acquire) + 1U;
  blocking->armed.store(true, std::memory_order_release);
  {
    std::lock_guard<std::mutex> lock(adapter.worker_state_mutex_);
    adapter.worker_started_ = true;
    adapter.worker_stop_requested_ = false;
  }
  adapter.tube_worker_ = std::thread(&PhaseOffsetMatchedAdapter::tubeWorkerMain,
                                    &adapter);
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  {
    std::unique_lock<std::mutex> lock(blocking->mutex);
    blocking->condition.wait(lock, [&]() { return blocking->paused; });
  }

  // requestShutdown() flips ownership before the build is released; the
  // worker then joins after its current build returns and must not publish.
  adapter.requestShutdown();
  EXPECT_TRUE(adapter.shutdown_requested_.load(std::memory_order_acquire));
  {
    std::lock_guard<std::mutex> lock(blocking->mutex);
    blocking->release = true;
  }
  blocking->condition.notify_all();
  adapter.shutdown();
  adapter.shutdown();
  EXPECT_TRUE(adapter.shutdown_requested_.load(std::memory_order_acquire));
  EXPECT_FALSE(adapter.worker_started_);
  EXPECT_FALSE(adapter.tube_worker_.joinable());
  EXPECT_EQ(1U, adapter.timer_build_sequence_);
  EXPECT_FALSE(std::atomic_load(&adapter.latest_candidate_epoch_snapshot_));
  EXPECT_FALSE(std::atomic_load(&adapter.latest_epoch_snapshot_));
}

TEST(PhaseOffsetMatchedAdapterStage1A,
     TimerTickCannotSynchronouslyBuildWhenWorkerStarted) {
  // The production worker is started by advertise().  Keep this test
  // independent of a ROS master by exercising the same guard directly.
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  const SyntheticPath path = MakePath();
  MatchedAdapterOutput output;
  ASSERT_FALSE(adapter.update(MakeInput(path, &path), output));
  {
    std::lock_guard<std::mutex> lock(adapter.worker_state_mutex_);
    adapter.worker_started_ = true;
  }
  const std::uint64_t before = adapter.timer_build_sequence_;
  EXPECT_TRUE(adapter.timerTick());
  EXPECT_EQ(before, adapter.timer_build_sequence_);
  EXPECT_TRUE(adapter.pending_request_);
  {
    std::lock_guard<std::mutex> lock(adapter.worker_state_mutex_);
    adapter.worker_started_ = false;
  }
}

TEST(PhaseOffsetMatchedAdapterStage1A,
     OrdinaryJobCopiesCommittedManagerButPairRefreshUsesFreshManager) {
  const SyntheticPath path = MakePath();
  int identity = 111;
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  ASSERT_FALSE(adapter.update(MakeInput(path, &identity), output));
  ASSERT_TRUE(adapter.timerTick());
  const auto request = std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(request);
  PhaseOffsetMatchedAdapter::SchedulePermit ordinary;
  ordinary.request = request;
  ordinary.request_identity = adapter.makeRequestInstanceIdentity(request);
  ordinary.work_identity = adapter.makeTubeWorkIdentity(request);
  PhaseOffsetMatchedAdapter::TubeJobLocalState ordinary_state;
  adapter.prepareTubeJobLocalState(ordinary, ordinary_state);
  EXPECT_FALSE(ordinary_state.pair_refresh);
  EXPECT_NE(adapter.tube_epoch_manager_.get(), ordinary_state.tube_epoch_manager.get());
  PhaseOffsetMatchedAdapter::SchedulePermit refresh = ordinary;
  std::shared_ptr<TubeBuildRequest> refresh_request(new TubeBuildRequest(*request));
  refresh_request->base_path_tube_pair = std::shared_ptr<const PathTubePair>(
      new PathTubePair());
  refresh.request = std::shared_ptr<const TubeBuildRequest>(refresh_request);
  refresh.request_identity = adapter.makeRequestInstanceIdentity(refresh.request);
  refresh.work_identity = adapter.makeTubeWorkIdentity(refresh.request);
  PhaseOffsetMatchedAdapter::TubeJobLocalState refresh_state;
  adapter.prepareTubeJobLocalState(refresh, refresh_state);
  EXPECT_TRUE(refresh_state.pair_refresh);
  EXPECT_EQ(0U, refresh_state.tube_epoch_manager->candidate_sequence_);
}

TEST(PhaseOffsetMatchedAdapterStage1A,
     CandidateSequenceTracksCommittedCandidateAndBuildSequenceTracksAttempts) {
  const SyntheticPath path = MakePath();
  int first_identity = 121;
  int second_identity = 122;
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  ASSERT_FALSE(adapter.update(MakeInput(path, &first_identity), output));
  const auto first_request = std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(first_request);
  PhaseOffsetMatchedAdapter::SchedulePermit permit;
  permit.request = first_request;
  permit.request_identity = adapter.makeRequestInstanceIdentity(first_request);
  permit.work_identity = adapter.makeTubeWorkIdentity(first_request);
  PhaseOffsetMatchedAdapter::TubeJobLocalState local;
  adapter.prepareTubeJobLocalState(permit, local);
  TubeEpochSnapshot built;
  ASSERT_TRUE(adapter.buildTubeEpoch(first_request, built, &local));
  adapter.update(MakeInput(path, &second_identity), output);
  EXPECT_FALSE(adapter.finalizeTubeEpoch(first_request, built, &local));
  ASSERT_FALSE(adapter.update(MakeInput(path, &second_identity), output));
  ASSERT_TRUE(adapter.timerTick());
  const auto epoch = std::atomic_load(&adapter.latest_epoch_snapshot_);
  ASSERT_TRUE(epoch);
  EXPECT_EQ(epoch->epoch_status.candidate_sequence,
            epoch->candidate_profile->profile_revision);
  EXPECT_GE(adapter.timer_build_sequence_, 2U);
}

TEST(PhaseOffsetMatchedAdapterStage1A, R3RequestCandidateRequestProtocolUnchanged) {
  const SyntheticPath path = MakePath();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterOutput output;
  ASSERT_FALSE(adapter.update(MakeInput(path, &path), output));
  ASSERT_TRUE(adapter.timerTick());
  adapter.update(MakeInput(path, &path, kDt), output);
  const auto request = std::atomic_load(&adapter.latest_build_request_);
  const auto candidate = std::atomic_load(&adapter.latest_candidate_epoch_snapshot_);
  ASSERT_TRUE(request);
  ASSERT_TRUE(candidate);
  EXPECT_EQ(request->source_revision, candidate->source_revision);
  EXPECT_TRUE(adapter.epochMatchesRequest(*candidate, *request));
}

TEST(PhaseOffsetMatchedAdapterStage1A,
     ConfiguredSchedulingPeriodIsStoredFor005And010) {
  for (const double period : {0.05, 0.10}) {
    PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::FIXED);
    config.tube_update_period = period;
    PhaseOffsetMatchedAdapter adapter(config);
    // This unit fixture has no ROS timer, so it verifies only the truthful
    // source/config wiring.  Real 50/100 ms cadence belongs to ROS acceptance.
    EXPECT_DOUBLE_EQ(period, adapter.config_.tube_update_period);
    EXPECT_GE(adapter.config_.tube_update_period, 0.05);
    EXPECT_LE(adapter.config_.tube_update_period, 0.10);
  }
}

TEST(PhaseOffsetMatchedAdapterStage1A, SourceRevisionSemanticOwnerInvariant) {
  const SyntheticPath path = MakePath();
  auto owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput input = MakeInput(path, owner.get());
  input.semantic_path_owner = owner;
  MatchedAdapterOutput output;
  adapter.update(input, output);
  const std::uint64_t first = adapter.source_revision_;
  input.semantic_path_start_w = 0.1;
  adapter.update(input, output);
  EXPECT_GT(adapter.source_revision_, first);
}

TEST(PhaseOffsetMatchedAdapterStage1A,
     FrameOwnerReplacementWithUnchangedSourceRevisionIsRejected) {
  const SyntheticPath path = MakePath();
  auto owner = MakeSyntheticOwner();
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  MatchedAdapterInput first = MakeInput(path, owner.get());
  first.semantic_path_owner = owner;
  first.frame_owner = std::make_shared<const ContinuousPhaseNormalFrame>(
      owner, 1U, 1U);
  MatchedAdapterOutput output;
  adapter.update(first, output);
  const auto request = std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(request);
  const std::uint64_t revision = request->source_revision;
  MatchedAdapterInput replacement = first;
  replacement.frame_owner = std::make_shared<const ContinuousPhaseNormalFrame>(
      owner, 1U, 2U);
  adapter.update(replacement, output);
  const auto newer = std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(newer);
  EXPECT_EQ(revision, newer->source_revision);
  EXPECT_NE(request->frame_owner.get(), newer->frame_owner.get());
  EXPECT_FALSE(adapter.requestSourceStillCurrent(*request));
  EXPECT_FALSE(std::atomic_load(&adapter.latest_candidate_epoch_snapshot_));
}

TEST(PhaseOffsetMatchedAdapterTest,
     NominalWidthConfigurationIsRetainedAsSoleEsdfOwner) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
  config.tube.cross_section.nominal_half_width = 1.5;
  config.tube.cross_section.nominal_width_source =
      phase_offset_navigation::TubeNominalWidthSource::EXPLICIT_PARAMETER;
  config.tube.cross_section.nominal_width_legacy_conflict = true;
  // Deliberately contradictory legacy values must remain diagnostics only.
  config.tube.fixed_delta_max = 0.04;
  config.tube.max_offset = 0.20;
  config.tube.cross_section.search_extent = 3.0;
  PhaseOffsetMatchedAdapter adapter(config);
  EXPECT_TRUE(adapter.configurationValid());
  EXPECT_DOUBLE_EQ(adapter.config_.tube.cross_section.nominal_half_width, 1.5);
  EXPECT_EQ(adapter.config_.tube.cross_section.nominal_width_source,
            phase_offset_navigation::TubeNominalWidthSource::EXPLICIT_PARAMETER);
  EXPECT_TRUE(adapter.config_.tube.cross_section.nominal_width_legacy_conflict);

  const SyntheticPath path = MakeStraightSyntheticPath();
  MatchedAdapterInput input = MakeInput(path, &path);
  const std::shared_ptr<const TubeBuildRequest> request =
      adapter.makeBuildRequest(input, 41U, 1.2);
  ASSERT_TRUE(request);
  EXPECT_DOUBLE_EQ(request->retained_delta, 1.2);
  EXPECT_DOUBLE_EQ(request->authority_request.lower,
                   -std::abs(config.amplitude));
  EXPECT_DOUBLE_EQ(request->authority_request.upper, 1.2);
}

TEST(PhaseOffsetMatchedAdapterTest,
     AbsentNominalWidthConfigurationUsesOneMetreDefault) {
  const PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::ESDF));
  EXPECT_DOUBLE_EQ(adapter.config_.tube.cross_section.nominal_half_width, 1.0);
  EXPECT_EQ(adapter.config_.tube.cross_section.nominal_width_source,
            phase_offset_navigation::TubeNominalWidthSource::DEFAULT_ABSENT);
  EXPECT_FALSE(adapter.config_.tube.cross_section.nominal_width_legacy_conflict);
}

TEST(PhaseOffsetMatchedAdapterMeasurement,
     TubeDueCsvCopiesInvocationCountsAndUsesStableNominalWidthTokens) {
  const std::string csv_path =
      "/tmp/phase_offset_matched_adapter_tube_due_measurement_test.csv";
  std::remove(csv_path.c_str());

  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig(TubeSource::ESDF);
  config.measurement_enabled = true;
  config.measurement_tube_due_csv_path = csv_path;
  {
    PhaseOffsetMatchedAdapter adapter(config);
    phase_offset_navigation::TubeBuildDiagnostics absent;
    absent.surface_validator_invocation_count = 7U;
    absent.inward_search_attempt_count = 11U;
    absent.nominal_width_source =
        phase_offset_navigation::TubeNominalWidthSource::DEFAULT_ABSENT;
    absent.effective_nominal_half_width_m = 1.0;
    adapter.recordTubeDueTiming(123U, 456U, true, false, &absent);

    phase_offset_navigation::TubeBuildDiagnostics explicit_parameter;
    explicit_parameter.surface_validator_invocation_count = 13U;
    explicit_parameter.inward_search_attempt_count = 17U;
    explicit_parameter.nominal_width_source =
        phase_offset_navigation::TubeNominalWidthSource::EXPLICIT_PARAMETER;
    explicit_parameter.nominal_width_legacy_conflict = true;
    explicit_parameter.effective_nominal_half_width_m = 1.5;
    adapter.recordTubeDueTiming(789U, 1011U, false, true,
                                &explicit_parameter);

    ASSERT_EQ(adapter.measurement_tube_due_samples_.size(), 2U);
    EXPECT_EQ(adapter.measurement_tube_due_samples_[0]
                  .surface_validator_invocation_count,
              7U);
    EXPECT_EQ(adapter.measurement_tube_due_samples_[0]
                  .inward_search_attempt_count,
              11U);
    EXPECT_EQ(adapter.measurement_tube_due_samples_[1]
                  .surface_validator_invocation_count,
              13U);
    EXPECT_EQ(adapter.measurement_tube_due_samples_[1]
                  .inward_search_attempt_count,
              17U);
    adapter.flushTubeDueTiming();
  }

  std::ifstream stream(csv_path.c_str());
  ASSERT_TRUE(stream.is_open());
  std::string header;
  std::string absent_row;
  std::string explicit_row;
  ASSERT_TRUE(static_cast<bool>(std::getline(stream, header)));
  ASSERT_TRUE(static_cast<bool>(std::getline(stream, absent_row)));
  ASSERT_TRUE(static_cast<bool>(std::getline(stream, explicit_row)));

  const auto split_csv = [](const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream parser(line);
    std::string field;
    while (std::getline(parser, field, ',')) fields.push_back(field);
    return fields;
  };
  const std::vector<std::string> headers = split_csv(header);
  const std::vector<std::string> absent_fields = split_csv(absent_row);
  const std::vector<std::string> explicit_fields = split_csv(explicit_row);
  ASSERT_EQ(headers.size(), absent_fields.size());
  ASSERT_EQ(headers.size(), explicit_fields.size());

  const auto field_index = [&headers](const char* name) {
    const auto found = std::find(headers.begin(), headers.end(), name);
    return static_cast<std::size_t>(found - headers.begin());
  };
  const std::size_t invocation_index =
      field_index("surface_validator_invocation_count");
  const std::size_t inward_index = field_index("inward_search_attempt_count");
  const std::size_t source_index = field_index("nominal_width_source");
  const std::size_t conflict_index =
      field_index("nominal_width_legacy_conflict");
  ASSERT_LT(invocation_index, headers.size());
  ASSERT_LT(inward_index, headers.size());
  ASSERT_LT(source_index, headers.size());
  ASSERT_LT(conflict_index, headers.size());
  EXPECT_EQ(absent_fields[invocation_index], "7");
  EXPECT_EQ(absent_fields[inward_index], "11");
  EXPECT_EQ(absent_fields[source_index], "DEFAULT_ABSENT");
  EXPECT_EQ(explicit_fields[invocation_index], "13");
  EXPECT_EQ(explicit_fields[inward_index], "17");
  EXPECT_EQ(explicit_fields[source_index], "EXPLICIT_PARAMETER");
  EXPECT_EQ(explicit_fields[conflict_index], "1");
  stream.close();
  std::remove(csv_path.c_str());
}

TEST(PhaseOffsetMatchedAdapterStage1A,
     AdvertisedProductionBootstrapRemainsNotRequired) {
  if (!ros::isInitialized()) {
    int argc = 1;
    static char name[] = "phase_offset_stage1a_test";
    static char* argv[] = {name, nullptr};
    ros::init(argc, argv, "phase_offset_stage1a_test",
              ros::init_options::AnonymousName |
                  ros::init_options::NoSigintHandler);
  }
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master unavailable for advertise/worker smoke test";
  }
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig(TubeSource::FIXED));
  ros::NodeHandle nh;
  adapter.advertise(nh);
  EXPECT_FALSE(adapter.requiresPathTubePairBootstrap());
  adapter.requestShutdown();
  adapter.shutdown();
}

}  // namespace
}  // namespace FLAG_Race

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
