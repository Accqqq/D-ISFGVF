#include <gtest/gtest.h>

#include "phase_offset_navigation/phase_offset_runtime.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <type_traits>
#include <vector>

namespace phase_offset_navigation {
namespace {

RuntimePathSamples MakePath() {
  RuntimePathSamples samples;
  for (int index = 0; index <= 20; ++index) {
    phase_offset_core::PathDifferentialState state;
    state.w = 0.1 * index;
    state.p = Eigen::Vector3d(state.w, 0.0, 1.0 + 0.1 * state.w);
    state.p_w = Eigen::Vector3d(1.0, 0.0, 0.1);
    state.p_ww = Eigen::Vector3d::Zero();
    state.valid = true;
    samples.push_back(state);
  }
  return samples;
}

RuntimePathSamples MakeFrameBoundPath(const double tangent_speed) {
  RuntimePathSamples samples;
  for (int index = 0; index <= 4; ++index) {
    phase_offset_core::PathDifferentialState state;
    state.w = 0.1 * index;
    state.p = Eigen::Vector3d(state.w, 0.0, 1.0);
    state.p_w = Eigen::Vector3d(tangent_speed, 0.0, 0.0);
    state.p_ww = Eigen::Vector3d::Zero();
    state.T = Eigen::Vector3d::UnitX();
    state.N = Eigen::Vector3d::UnitY();
    state.N_w = Eigen::Vector3d::Zero();
    state.frame_valid = true;
    state.frame_provenance =
        "ContinuousPhaseNormalFrame/WorldHorizontalCrossProduct";
    state.valid = true;
    samples.push_back(state);
  }
  return samples;
}

std::shared_ptr<const TubeProfile> FinalizeProfile(
    const std::shared_ptr<TubeProfile>& profile) {
  if (profile && !profile->samples.empty()) {
    profile->certified_segment_start_w = profile->samples.front().w;
    profile->certified_segment_end_w = profile->samples.back().w;
  }
  return profile;
}

RuntimeFutureStepContract MakeFutureContract(
    const std::shared_ptr<const TubeProfile>& profile,
    const double min_certified_forward_w = 0.0) {
  RuntimeFutureStepContract contract;
  if (!profile || profile->samples.size() < 2U) return contract;
  contract.tube_update_period = 0.10;
  contract.min_certified_forward_w = min_certified_forward_w;
  contract.profile_domain_end_w = profile->certified_segment_end_w;
  contract.evaluate = [](
      const RuntimeFutureStepInput& input,
      RuntimeFutureStepResult& output) {
    output = RuntimeFutureStepResult();
    if (!std::isfinite(input.phase) || !std::isfinite(input.delta) ||
        !input.matched_position.allFinite()) {
      return false;
    }
    output.path.w = input.phase;
    output.path.p = Eigen::Vector3d(
        input.phase, 0.0, 1.0 + 0.1 * input.phase);
    output.path.p_w = Eigen::Vector3d(1.0, 0.0, 0.1);
    output.path.p_ww = Eigen::Vector3d::Zero();
    output.path.valid = true;
    phase_offset_core::PhaseOffsetGeometryState geometry;
    if (!phase_offset_core::GeometryEvaluator().evaluate(
            output.path, input.matched_position, input.delta, geometry)) {
      return false;
    }
    output.matched_reference = geometry.r;
    output.matched_tangent = geometry.T;
    output.matched_derivative_norm = geometry.r_w.norm();
    output.base_v_cmd = geometry.T * 0.5;
    output.base_w_dot = 0.5;
    output.base_guidance_valid = true;
    output.valid = true;
    return true;
  };
  return contract;
}

RuntimeFutureStepContract MakePhaseLimitedFutureContract(
    const std::shared_ptr<const TubeProfile>& profile,
    const Eigen::Vector3d& base_v_cmd, const double base_w_dot,
    const double maximum_phase,
    const std::shared_ptr<std::vector<RuntimeFutureStepInput>>& calls) {
  RuntimeFutureStepContract contract;
  if (!profile || profile->samples.size() < 2U ||
      !base_v_cmd.allFinite() || !std::isfinite(base_w_dot)) {
    return contract;
  }
  contract.tube_update_period = 0.10;
  contract.profile_domain_end_w = profile->certified_segment_end_w;
  contract.evaluate = [base_v_cmd, base_w_dot, maximum_phase, calls](
      const RuntimeFutureStepInput& input,
      RuntimeFutureStepResult& output) {
    output = RuntimeFutureStepResult();
    if (calls) calls->push_back(input);
    if (!std::isfinite(input.phase) || !std::isfinite(input.delta) ||
        !std::isfinite(input.dt) || input.dt <= 0.0 ||
        !input.matched_position.allFinite() ||
        input.phase > maximum_phase + 1e-12) {
      return false;
    }
    output.path.w = input.phase;
    output.path.p = Eigen::Vector3d(
        input.phase, 0.0, 1.0 + 0.1 * input.phase);
    output.path.p_w = Eigen::Vector3d(1.0, 0.0, 0.1);
    output.path.p_ww = Eigen::Vector3d::Zero();
    output.path.valid = true;
    phase_offset_core::PhaseOffsetGeometryState geometry;
    if (!phase_offset_core::GeometryEvaluator().evaluate(
            output.path, input.matched_position, input.delta, geometry)) {
      return false;
    }
    output.matched_reference = geometry.r;
    output.matched_tangent = geometry.T;
    output.matched_derivative_norm = geometry.r_w.norm();
    output.base_v_cmd = base_v_cmd;
    output.base_w_dot = base_w_dot;
    output.base_guidance_valid = true;
    output.valid = true;
    return true;
  };
  return contract;
}

std::shared_ptr<const TubeProfile> MakeActiveProfile(double lower = -0.08,
                                                     double upper = 0.08) {
  auto profile = std::make_shared<TubeProfile>();
  profile->source = TubeSource::FIXED;
  profile->preview_start_w = 0.0;
  profile->preview_end_w = 2.0;
  profile->raw_complete = profile->filtered_complete = profile->complete = true;
  for (int index = 0; index <= 20; ++index) {
    TubeRawSample sample;
    sample.w = 0.1 * index;
    sample.p = Eigen::Vector3d(sample.w, 0.0, 1.0);
    sample.N = Eigen::Vector3d(0.0, 1.0, 0.0);
    sample.raw_lower = sample.filtered_lower = lower;
    sample.raw_upper = sample.filtered_upper = upper;
    sample.complete = true;
    profile->samples.push_back(sample);
  }
  return FinalizeProfile(profile);
}

std::shared_ptr<const TubeProfile> MakePosthocRejectingProfile() {
  auto profile = std::make_shared<TubeProfile>();
  profile->source = TubeSource::FIXED;
  profile->preview_start_w = 0.0;
  profile->preview_end_w = 2.0;
  profile->raw_complete = profile->filtered_complete = profile->complete = true;
  TubeRawSample current;
  current.w = 0.0;
  current.raw_lower = current.filtered_lower = -0.08;
  current.raw_upper = current.filtered_upper = 0.08;
  current.complete = true;
  TubeRawSample narrowing;
  narrowing.w = 0.015;
  narrowing.raw_lower = narrowing.filtered_lower = 0.07;
  narrowing.raw_upper = narrowing.filtered_upper = 0.08;
  narrowing.complete = true;
  TubeRawSample tail = narrowing;
  tail.w = 2.0;
  profile->samples.push_back(current);
  profile->samples.push_back(narrowing);
  profile->samples.push_back(tail);
  return FinalizeProfile(profile);
}

std::shared_ptr<const TubeProfile> MakeCrossKnotProfile() {
  auto profile = std::make_shared<TubeProfile>();
  profile->source = TubeSource::FIXED;
  profile->preview_start_w = 0.0;
  profile->preview_end_w = 2.0;
  profile->raw_complete = profile->filtered_complete = profile->complete = true;
  const double upper[] = {0.08, 0.08, 0.00, 0.08};
  for (int index = 0; index <= 3; ++index) {
    TubeRawSample sample;
    sample.w = 0.05 * index;
    sample.raw_lower = sample.filtered_lower = -0.08;
    sample.raw_upper = sample.filtered_upper = upper[index];
    sample.complete = true;
    profile->samples.push_back(sample);
  }
  return FinalizeProfile(profile);
}

std::shared_ptr<const TubeProfile> MakeTightCrossKnotProfile() {
  auto profile = std::make_shared<TubeProfile>();
  profile->source = TubeSource::FIXED;
  profile->preview_start_w = 0.0;
  profile->preview_end_w = 2.0;
  profile->raw_complete = profile->filtered_complete = profile->complete = true;
  const double upper[] = {0.08, 0.00, 0.08};
  for (int index = 0; index <= 2; ++index) {
    TubeRawSample sample;
    sample.w = 0.05 * index;
    sample.raw_lower = sample.filtered_lower = -0.08;
    sample.raw_upper = sample.filtered_upper = upper[index];
    sample.complete = true;
    profile->samples.push_back(sample);
  }
  return FinalizeProfile(profile);
}

std::shared_ptr<const TubeProfile> MakeImpossibleCrossKnotProfile() {
  auto profile = std::make_shared<TubeProfile>();
  profile->source = TubeSource::FIXED;
  profile->preview_start_w = 0.0;
  profile->preview_end_w = 0.10;
  profile->raw_complete = profile->filtered_complete = profile->complete = true;
  const double w[] = {0.0, 0.005, 0.10};
  const double lower[] = {-0.08, 0.05, 0.05};
  const double upper[] = {0.08, 0.08, 0.08};
  for (int index = 0; index <= 2; ++index) {
    TubeRawSample sample;
    sample.w = w[index];
    sample.raw_lower = sample.filtered_lower = lower[index];
    sample.raw_upper = sample.filtered_upper = upper[index];
    sample.complete = true;
    profile->samples.push_back(sample);
  }
  return FinalizeProfile(profile);
}

PhaseOffsetRuntimeConfig MakeConfig(TubeSource source) {
  PhaseOffsetRuntimeConfig config;
  config.manual.profile_period = 0.4;
  config.manual.u_delta_abs_max = 0.5;
  config.manual.u_w_rate_max = 100.0;
  config.manual.u_delta_rate_max = 100.0;
  config.tube.source = source;
  config.tube.tracking_error_bound = 0.15;
  return config;
}

std::shared_ptr<const SectionTubeProfile> MakeRuntimeSectionProfile(
    const double narrow_upper = 1.0) {
  std::shared_ptr<SectionTubeProfile> profile(new SectionTubeProfile());
  profile->valid_start = 0.0;
  profile->valid_end = 2.0;
  profile->status = SectionTubeStatus::COMPLETE;
  profile->usable = true;
  profile->complete = true;
  const double knot_w[] = {0.0, 0.5, 1.0, 2.0};
  for (double w : knot_w) {
    SectionTubeKnot knot;
    knot.w = w;
    knot.lower = -1.0;
    knot.upper = (w == 0.5) ? narrow_upper : 1.0;
    profile->knots.push_back(knot);
  }
  return profile;
}

NormalPreviewResult MakeRuntimeSectionPreview(
    const std::shared_ptr<const SectionTubeProfile>& profile,
    const double current_delta = 0.0, const double current_w = 0.0,
    const double preview_horizon = 2.0) {
  NormalPreviewProductionPolicy policy;
  policy.preview_horizon_w = preview_horizon;
  policy.sample_spacing_w = 0.5;
  policy.lower_nu = 0.5;
  policy.upper_nu = 1.0;
  policy.b_tight = 0.1;
  policy.b_open = 0.9;
  TubeViabilityInput input;
  input.current_w = current_w;
  input.current_delta = current_delta;
  input.policy = policy;
  input.upper_u_delta = 1.0;
  input.path_revision = 41U;
  input.frame_revision = 42U;
  input.expected_path_revision = 41U;
  input.expected_frame_revision = 42U;
  input.max_work = 100000U;
  NormalPreviewResult preview;
  if (profile) {
    EXPECT_TRUE(TubeViability::evaluate(*profile, input, preview))
        << preview.reason;
  }
  return preview;
}

RuntimeSectionPrepareInput MakeRuntimeSectionInput(
    const std::shared_ptr<const SectionTubeProfile>& profile,
    const NormalPreviewResult& preview, const double final_u_w = 0.1,
    const double final_u_delta = 0.1, const double dt = 0.1,
    const double current_delta = 0.0, const double current_w = 0.0) {
  RuntimeSectionPrepareInput input;
  input.profile = profile;
  input.preview = &preview;
  phase_offset_core::PathDifferentialState path;
  path.w = current_w;
  path.p = Eigen::Vector3d(current_w, 0.0, 1.0);
  path.p_w = Eigen::Vector3d(1.0, 0.0, 0.0);
  path.p_ww = Eigen::Vector3d::Zero();
  path.valid = true;
  phase_offset_core::GeometryEvaluator evaluator;
  EXPECT_TRUE(evaluator.evaluate(path, path.p, current_delta,
                                 input.matched.geometry));
  input.matched.geometry.path_revision = 41U;
  input.matched.geometry.frame_revision = 42U;
  input.matched.base_v_cmd = input.matched.geometry.T * 0.5;
  input.matched.base_w_dot = 0.5;
  input.matched.final_port.u_w = final_u_w;
  input.matched.final_port.u_delta = final_u_delta;
  input.previous_u = phase_offset_core::PortCommand();
  input.limits.lower_nu = 0.5;
  input.limits.upper_nu = 1.0;
  input.limits.u_w_abs_max = 0.2;
  input.limits.upper_u_delta = 1.0;
  input.limits.u_w_slew_rate = 10.0;
  input.limits.u_delta_slew_rate = 10.0;
  input.limits.zoh_dt = dt;
  input.dt = dt;
  input.max_work = 100000U;
  return input;
}

// Small immutable V2 admission fixture.  It mirrors the production value
// contract (complete path/config/map keys and certified PWL cells) without
// constructing any manager, map, or adapter authority.
phase_offset_core::Binary64Interval V2Interval(const double lower,
                                               const double upper) {
  phase_offset_core::Binary64Interval result;
  result.lower = lower;
  result.upper = upper;
  result.valid = true;
  return result;
}

phase_offset_core::Binary64VectorInterval V2VectorInterval() {
  phase_offset_core::Binary64VectorInterval result;
  result.valid = true;
  for (phase_offset_core::Binary64Interval& component : result.component) {
    component = V2Interval(0.0, 0.0);
  }
  return result;
}

phase_offset_core::CertifiedPathCellV2 V2PathCell(
    const double w0, const double w1, const std::uint64_t segment) {
  phase_offset_core::CertifiedPathCellV2 cell;
  cell.w0 = w0;
  cell.w1 = w1;
  cell.anchor_w = 0.5 * (w0 + w1);
  cell.path_revision = 11U;
  cell.frame_revision = 12U;
  cell.segment_identity = segment;
  cell.proof_identity = 100U + segment;
  cell.anchor_position = V2VectorInterval();
  cell.anchor_p_w = V2VectorInterval();
  cell.anchor_p_ww = V2VectorInterval();
  cell.inf_p_w_norm = V2Interval(1.0, 1.0);
  cell.sup_p_w_norm = V2Interval(1.0, 1.0);
  cell.inf_horizontal_p_w_norm = V2Interval(1.0, 1.0);
  cell.sup_p_ww_norm = V2Interval(0.0, 0.0);
  cell.sup_horizontal_p_ww_norm = V2Interval(0.0, 0.0);
  cell.sup_p_www_norm = V2Interval(0.0, 0.0);
  cell.sup_normal_derivative = V2Interval(0.0, 0.0);
  cell.normal_variation = V2Interval(0.0, 0.0);
  cell.tangent_variation = V2Interval(0.0, 0.0);
  cell.curvature_variation = V2Interval(0.0, 0.0);
  cell.midpoint_position_variation = V2Interval(0.0, 0.0);
  cell.chord_deviation = V2Interval(0.0, 0.0);
  cell.horizontal_acceleration_bound_complete = true;
  cell.normal_frame_proof_complete = true;
  cell.phase_map_proof_complete = true;
  cell.provenance = phase_offset_core::kWorldHorizontalCrossProductProvenance;
  cell.complete = true;
  cell.valid = true;
  return cell;
}

TubePathKey V2PathKey() {
  TubePathKey key;
  key.execution_generation = 21U;
  key.path_instance_id = 22U;
  key.path_revision = 11U;
  key.frame_revision = 12U;
  key.frame_convention_id = 13U;
  key.frame_convention =
      phase_offset_core::kWorldHorizontalCrossProductProvenance;
  key.phase_orientation = 1;
  key.domain_start = 0.0;
  key.domain_end = 2.0;
  return key;
}

TubeConfigurationKey V2ConfigurationKey() {
  TubeConfigurationKey key;
  key.configuration_id = 31U;
  key.epsilon = 0.1;
  key.nominal_half_width = 0.5;
  key.ray_step = 0.05;
  key.snapshot_resolution = 0.05;
  key.minimum_reference_speed = 1e-8;
  return key;
}

TubeMapCaptureKey V2MapKey() {
  TubeMapCaptureKey key;
  key.map_instance_id = 41U;
  key.state_id = 42U;
  key.accepted_sequence = 43U;
  key.configuration_generation = 44U;
  key.configuration_id = 31U;
  key.frame_provenance_id = 45U;
  key.frame_provenance = "test-authoritative-sdfmap";
  key.support_provenance_id = 46U;
  key.accepted_time_ticks = 100U;
  key.support_expiry_ticks = 10000U;
  key.support_halo = 0.1;
  key.halo_reconciled = true;
  key.grid_min_index_x = -10;
  key.grid_min_index_y = -10;
  key.grid_min_index_z = -10;
  key.grid_max_index_x = 10;
  key.grid_max_index_y = 10;
  key.grid_max_index_z = 10;
  key.grid_native_origin = Eigen::Vector3d::Zero();
  key.grid_voxel_resolution = Eigen::Vector3d::Constant(0.05);
  key.complete_support = true;
  return key;
}

std::shared_ptr<TubeProfileV2> MakeRuntimeV2Profile() {
  auto profile = std::make_shared<TubeProfileV2>();
  profile->path_key = V2PathKey();
  profile->configuration_key = V2ConfigurationKey();
  profile->map_capture_key = V2MapKey();
  profile->profile_id = 51U;
  profile->request_id = 52U;
  profile->requested_start = 0.0;
  profile->requested_end = 2.0;
  profile->anchor_w = 0.0;
  profile->certified_start = 0.0;
  profile->certified_end = 2.0;
  profile->valid = true;
  profile->complete = true;
  profile->contains_anchor = true;
  profile->contains_zero_everywhere = true;
  profile->nonzero_capacity = true;
  profile->capability = TubeProfileV2Capability::OFFSET_CERTIFIED;
  profile->path_owner = std::shared_ptr<const void>(new int(1));
  profile->capture_owner = std::shared_ptr<const void>(new int(2));
  profile->query_owner = std::shared_ptr<const void>(new int(3));
  profile->applicability_assumptions = "runtime-test-complete-support";
  profile->applicability_deadline_ticks = 10000U;
  profile->applicability_deadline_timeless = false;
  TubePwlKnotV2 first;
  first.w = 0.0;
  first.lower = -0.5;
  first.upper = 0.5;
  first.right_cell_id = 61U;
  first.right_lower_slope_interval = TubeDirectedRatioV2{0.0, 0.0, true};
  first.right_upper_slope_interval = TubeDirectedRatioV2{0.0, 0.0, true};
  first.valid = true;
  TubePwlKnotV2 middle = first;
  middle.w = 1.0;
  middle.left_cell_id = 61U;
  middle.right_cell_id = 62U;
  middle.left_lower_slope_interval = TubeDirectedRatioV2{0.0, 0.0, true};
  middle.left_upper_slope_interval = TubeDirectedRatioV2{0.0, 0.0, true};
  middle.right_lower_slope_interval = TubeDirectedRatioV2{0.0, 0.0, true};
  middle.right_upper_slope_interval = TubeDirectedRatioV2{0.0, 0.0, true};
  TubePwlKnotV2 last = middle;
  last.w = 2.0;
  last.left_cell_id = 62U;
  last.right_cell_id = 0U;
  last.right_lower_slope_interval = TubeDirectedRatioV2();
  last.right_upper_slope_interval = TubeDirectedRatioV2();
  profile->knots = {first, middle, last};
  TubeProofCellV2 cell0;
  cell0.w0 = 0.0;
  cell0.w1 = 1.0;
  cell0.lower = -0.5;
  cell0.upper = 0.5;
  cell0.cell_id = 61U;
  cell0.valid = true;
  cell0.complete = true;
  cell0.path_cell = V2PathCell(0.0, 1.0, 71U);
  TubeProofCellV2 cell1 = cell0;
  cell1.w0 = 1.0;
  cell1.w1 = 2.0;
  cell1.cell_id = 62U;
  cell1.path_cell = V2PathCell(1.0, 2.0, 72U);
  profile->cells = {cell0, cell1};
  return profile;
}

TubeExecutionIdentityV2 RuntimeV2Identity(const TubeProfileV2& profile) {
  TubeExecutionIdentityV2 identity;
  identity.execution_generation = profile.path_key.execution_generation;
  identity.path_instance_id = profile.path_key.path_instance_id;
  identity.path_revision = profile.path_key.path_revision;
  identity.frame_revision = profile.path_key.frame_revision;
  identity.frame_convention_id = profile.path_key.frame_convention_id;
  identity.configuration_id = profile.configuration_key.configuration_id;
  identity.map_instance_id = profile.map_capture_key.map_instance_id;
  identity.map_state_id = profile.map_capture_key.state_id;
  identity.accepted_sequence = profile.map_capture_key.accepted_sequence;
  identity.profile_id = profile.profile_id;
  identity.binding_sequence = 81U;
  return identity;
}

TubeExecutionLimitsV2 RuntimeV2Limits() {
  TubeExecutionLimitsV2 limits;
  limits.lower_phase_rate = 0.05;
  limits.upper_phase_rate = 0.5;
  limits.upper_nu = 0.5;
  limits.max_u_w = 0.5;
  limits.max_u_delta = 1.0;
  limits.u_w_slew_rate = 1.0;
  limits.u_delta_slew_rate = 1.0;
  limits.return_u_delta_max = 0.5;
  limits.return_u_delta_slew_rate = 0.5;
  limits.max_schedule_steps = 200U;
  limits.max_work = 1000U;
  limits.valid = true;
  return limits;
}

NormalPreviewProductionPolicy RuntimeV2Policy() {
  NormalPreviewProductionPolicy policy;
  policy.preview_horizon_w = 1.0;
  policy.sample_spacing_w = 0.25;
  policy.lower_nu = 0.05;
  policy.upper_nu = 0.5;
  policy.b_tight = 0.1;
  policy.b_open = 0.5;
  policy.policy_revision = 1U;
  policy.configuration_identity = V2ConfigurationKey().configuration_id;
  policy.configuration_id = "runtime-test-preview-policy";
  return policy;
}

RuntimeV2PrepareInput MakeRuntimeV2Input(
    const std::shared_ptr<const TubeProfileV2>& profile) {
  RuntimeV2PrepareInput input;
  input.profile = profile;
  input.identity = RuntimeV2Identity(*profile);
  input.current.w = 0.25;
  input.current.delta = 0.0;
  input.current.previous_u = phase_offset_core::PortCommand();
  input.preview_policy = RuntimeV2Policy();
  input.limits = RuntimeV2Limits();
  input.selected_u = phase_offset_core::PortCommand();
  input.selected_u.u_delta = 0.05;
  input.base_phase_rate = 0.1;
  input.phase_rate_lower = 0.05;
  input.phase_rate_upper = 0.5;
  input.horizon_w = 1.25;
  input.sample_spacing_w = 0.25;
  input.upper_u_delta = 0.2;
  input.dt = 0.1;
  input.now = 1.0;
  input.applicability_deadline = 100.0;
  input.applicability_deadline_valid = true;
  input.tracking = TubeExecutionTrackingEvidenceV2();
  input.max_work = 1000U;
  input.provenance = "runtime-test/v2-admission";
  return input;
}

RuntimeInstalledTubeView MakeView(TubeEpochState state = TubeEpochState::ROLLING,
                                  std::shared_ptr<const TubeProfile> profile = MakeActiveProfile()) {
  RuntimeInstalledTubeView view;
  view.active_profile = std::move(profile);
  view.epoch_status.state = state;
  view.epoch_status.active_available = static_cast<bool>(view.active_profile);
  view.epoch_status.active_current_validation_valid =
      static_cast<bool>(view.active_profile) &&
      state != TubeEpochState::CERTIFICATE_DENIED;
  view.epoch_status.certificate_denied =
      state == TubeEpochState::CERTIFICATE_DENIED;
  view.epoch_status.active_tube_epoch = view.active_profile ? 1U : 0U;
  view.epoch_status.current_state_admissible = true;
  return view;
}

RuntimePrepareInput MakeInput(const RuntimePathSamples& path,
                              const RuntimeInstalledTubeView& view,
                              bool gate_open = true) {
  RuntimePrepareInput input;
  input.current_path = path.front();
  input.position = path.front().p;
  input.tube_view = view;
  input.dt = 0.02;
  if (view.active_profile) {
    input.future_step = MakeFutureContract(view.active_profile);
  }
  input.zero_gate_open = gate_open;
  return input;
}

// `refreshPreflight`/`complete` belong to the retired runtime profile and are
// compiled out of the production library, so the helpers that call them follow
// the same guard as the tests that use them.
#ifndef PHASE_OFFSET_RUNTIME_V2_TEST_ONLY
void Refresh(PhaseOffsetRuntime& runtime, const RuntimePathSamples& path,
             std::uint64_t revision = 1U) {
  RuntimePreflightInput input;
  input.path = path;
  input.position = path.front().p;
  input.path_source_revision = revision;
  ASSERT_TRUE(runtime.refreshPreflight(input));
}

bool Complete(PhaseOffsetRuntime& runtime, const RuntimePreparedStep& prepared,
              RuntimeStepOutput& output) {
  return runtime.complete(prepared, prepared.geometry.T * 0.5, 0.5, true,
                          output);
}
#endif

RuntimeDryRunInput MakeDryRunInput(const RuntimePathSamples& path,
                                   const RuntimeInstalledTubeView& view) {
  RuntimeDryRunInput input;
  input.preflight.path = path;
  input.preflight.position = path.front().p;
  input.preflight.path_source_revision = 2U;
  input.prepare = MakeInput(path, view, true);
  input.base_v_cmd = input.prepare.current_path.p_w.normalized() * 0.5;
  input.base_w_dot = 0.5;
  input.base_guidance_valid = true;
  return input;
}

#ifndef PHASE_OFFSET_RUNTIME_V2_TEST_ONLY
TEST(PhaseOffsetRuntimeTest, NoneKeepsA4FinalPortDeltaIntegration) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::NONE));
  ASSERT_TRUE(runtime.configurationValid());
  Refresh(runtime, path);
  RuntimePreparedStep first;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), first));
  EXPECT_EQ(first.execution.mode, RuntimeExecutionMode::NO_TUBE_REQUIRED);
  RuntimeStepOutput output;
  ASSERT_TRUE(Complete(runtime, first, output));
  RuntimePreparedStep next;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), next));
  EXPECT_NEAR(next.delta, output.delta + 0.02 * output.projection.final_port.u_delta, 1e-12);
  ASSERT_TRUE(Complete(runtime, next, output));
  EXPECT_NEAR(output.matched.delta_dot, output.projection.final_port.u_delta, 1e-15);
  EXPECT_LE(output.matched.matched_residual_norm, 1e-12);
}

TEST(PhaseOffsetRuntimeTest,
     NewNavigationTaskResetDropsOnlyTaskScopedExecutionHistory) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::NONE));
  ASSERT_TRUE(runtime.configurationValid());
  Refresh(runtime, path, 41U);

  RuntimePreparedStep prepared;
  RuntimeStepOutput output;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()),
                              prepared));
  ASSERT_TRUE(Complete(runtime, prepared, output));
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()),
                              prepared));
  ASSERT_TRUE(Complete(runtime, prepared, output));
  ASSERT_TRUE(runtime.hasExecutedOffsetAuthority());
  EXPECT_NE(0.0, runtime.retainedDelta());
  EXPECT_TRUE(std::abs(runtime.previousFinalPort().u_delta) > 0.0 ||
              std::abs(runtime.previousFinalPort().u_w) > 0.0);

  runtime.resetForNewNavigationTask();
  EXPECT_TRUE(runtime.configurationValid());
  EXPECT_DOUBLE_EQ(0.0, runtime.retainedDelta());
  EXPECT_DOUBLE_EQ(0.0, runtime.previousFinalPort().u_w);
  EXPECT_DOUBLE_EQ(0.0, runtime.previousFinalPort().u_delta);
  EXPECT_FALSE(runtime.hasExecutedOffsetAuthority());
  EXPECT_TRUE(runtime.hasPendingOrActiveOffsetIntent());
  // The old path's preflight may not authorize the new task.
  EXPECT_FALSE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()),
                               prepared));

  // The operation is idempotent and preserves configuration validity.
  runtime.resetForNewNavigationTask();
  EXPECT_TRUE(runtime.configurationValid());
  EXPECT_DOUBLE_EQ(0.0, runtime.retainedDelta());
  EXPECT_FALSE(runtime.hasExecutedOffsetAuthority());
}

TEST(PhaseOffsetRuntimeTest, OrdinaryPrepareDoesNotImplicitlyResetTaskState) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::NONE));
  Refresh(runtime, path, 42U);
  RuntimePreparedStep prepared;
  RuntimeStepOutput output;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()),
                              prepared));
  ASSERT_TRUE(Complete(runtime, prepared, output));
  const double retained_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand port_before = runtime.previousFinalPort();
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()),
                              prepared));
  EXPECT_DOUBLE_EQ(retained_before, prepared.delta);
  EXPECT_DOUBLE_EQ(port_before.u_w, runtime.previousFinalPort().u_w);
  EXPECT_DOUBLE_EQ(port_before.u_delta, runtime.previousFinalPort().u_delta);
}

TEST(PhaseOffsetRuntimeTest, PreflightIsExplicitAndRevisionScoped) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::NONE));
  Refresh(runtime, path, 4U);
  RuntimePreflightInput same_revision;
  same_revision.position = path.front().p;
  same_revision.path_source_revision = 4U;
  EXPECT_TRUE(runtime.refreshPreflight(same_revision));
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), prepared));
  EXPECT_TRUE(prepared.preflight.complete);
  RuntimePreflightInput changed = same_revision;
  changed.path_source_revision = 5U;
  EXPECT_FALSE(runtime.refreshPreflight(changed));
  EXPECT_FALSE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), prepared));
  EXPECT_FALSE(prepared.execution.fatal_control_failure);
}

TEST(PhaseOffsetRuntimeTest, ConfiguredMinimumReferenceSpeedAgreesWithFramePreflight) {
  PhaseOffsetRuntimeConfig config = MakeConfig(TubeSource::NONE);
  config.tube.minimum_reference_speed = 0.50;
  PhaseOffsetRuntime slow(config);
  RuntimePreflightInput slow_input;
  slow_input.path = MakeFrameBoundPath(0.20);
  slow_input.position = slow_input.path.front().p;
  slow_input.path_source_revision = 1U;
  EXPECT_FALSE(slow.refreshPreflight(slow_input));

  PhaseOffsetRuntime fast(config);
  RuntimePreflightInput fast_input = slow_input;
  fast_input.path = MakeFrameBoundPath(0.80);
  fast_input.position = fast_input.path.front().p;
  EXPECT_TRUE(fast.refreshPreflight(fast_input));
}

TEST(PhaseOffsetRuntimeTest, GateClosedEvaluatesExactPortButCannotSelectOrCommit) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView(), false), prepared));
  EXPECT_TRUE(prepared.execution.current_geometry_valid);
  EXPECT_TRUE(prepared.execution.current_bounds_valid);
  EXPECT_TRUE(prepared.execution.retained_delta_current_inside);
  EXPECT_EQ(prepared.execution.mode, RuntimeExecutionMode::NORMAL);
  EXPECT_TRUE(prepared.requires_base_guidance);
  EXPECT_DOUBLE_EQ(prepared.delta, 0.0);
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before = runtime.previousFinalPort();
  RuntimeStepOutput output;
  ASSERT_TRUE(Complete(runtime, prepared, output));
  EXPECT_TRUE(output.valid);
  EXPECT_FALSE(output.selected);
  EXPECT_TRUE(output.projection.valid);
  EXPECT_TRUE(output.matched.valid);
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, previous_before.u_delta);
}

TEST(PhaseOffsetRuntimeTest,
     DryRunFailureDoesNotMutateLivePreflightDeltaOrPreviousPort) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path, 1U);
  RuntimePreparedStep prepared;
  RuntimeStepOutput committed;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), prepared));
  ASSERT_TRUE(Complete(runtime, prepared, committed));
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before = runtime.previousFinalPort();

  RuntimeDryRunInput staged = MakeDryRunInput(path, MakeView());
  staged.preflight.path.clear();
  RuntimeDryRunResult result;
  EXPECT_FALSE(runtime.dryRun(staged, result));
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, previous_before.u_delta);

  // The failed staging revision must not replace the live revision's
  // preflight.  A normal live prepare is still executable afterwards.
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), prepared));
  EXPECT_TRUE(prepared.preflight.complete);
}

TEST(PhaseOffsetRuntimeTest,
     DryRunUsesExactPortWithoutCommittingLiveRuntimeState) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path, 1U);
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before = runtime.previousFinalPort();

  RuntimeDryRunResult result;
  ASSERT_TRUE(runtime.dryRun(MakeDryRunInput(path, MakeView()), result));
  EXPECT_TRUE(result.step.valid);
  EXPECT_TRUE(result.step.projection.valid);
  EXPECT_TRUE(result.step.matched.valid);
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, previous_before.u_delta);
}

TEST(PhaseOffsetRuntimeTest,
     DryRunLeavesFollowupLivePrepareAndCompleteBitwiseEquivalentToBaseline) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path, 1U);
  // Start the profile once so this covers copied profile lifecycle state as
  // well as retained delta and the previous final port.
  RuntimePreparedStep initial;
  RuntimeStepOutput initial_output;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), initial));
  ASSERT_TRUE(Complete(runtime, initial, initial_output));
  PhaseOffsetRuntime baseline(runtime);
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before = runtime.previousFinalPort();

  RuntimeDryRunResult dry_run;
  ASSERT_TRUE(runtime.dryRun(MakeDryRunInput(path, MakeView()), dry_run));
  const double delta_after_dry_run = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_after_dry_run =
      runtime.previousFinalPort();
  EXPECT_EQ(std::memcmp(&delta_before, &delta_after_dry_run, sizeof(double)), 0);
  EXPECT_EQ(std::memcmp(&previous_before.u_w, &previous_after_dry_run.u_w,
                        sizeof(double)), 0);
  EXPECT_EQ(std::memcmp(&previous_before.u_delta, &previous_after_dry_run.u_delta,
                        sizeof(double)), 0);

  RuntimePreparedStep actual;
  RuntimePreparedStep expected;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), actual));
  ASSERT_TRUE(baseline.prepare(MakeInput(path, MakeView()), expected));
  RuntimeStepOutput actual_output;
  RuntimeStepOutput expected_output;
  ASSERT_TRUE(Complete(runtime, actual, actual_output));
  ASSERT_TRUE(Complete(baseline, expected, expected_output));
  EXPECT_EQ(std::memcmp(&actual_output.projection.final_port.u_w,
                        &expected_output.projection.final_port.u_w,
                        sizeof(double)), 0);
  EXPECT_EQ(std::memcmp(&actual_output.projection.final_port.u_delta,
                        &expected_output.projection.final_port.u_delta,
                        sizeof(double)), 0);
  const double actual_delta = runtime.retainedDelta();
  const double expected_delta = baseline.retainedDelta();
  EXPECT_EQ(std::memcmp(&actual_delta, &expected_delta, sizeof(double)), 0);
  const phase_offset_core::PortCommand actual_previous = runtime.previousFinalPort();
  const phase_offset_core::PortCommand expected_previous =
      baseline.previousFinalPort();
  EXPECT_EQ(std::memcmp(&actual_previous.u_w, &expected_previous.u_w,
                        sizeof(double)), 0);
  EXPECT_EQ(std::memcmp(&actual_previous.u_delta, &expected_previous.u_delta,
                        sizeof(double)), 0);
}

TEST(PhaseOffsetRuntimeTest, CompleteCommitsSelectedPortImmediately) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::NONE));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), prepared));
  RuntimeStepOutput output;
  ASSERT_TRUE(Complete(runtime, prepared, output));
  ASSERT_TRUE(output.selected);
  EXPECT_DOUBLE_EQ(output.projection.next_delta, runtime.retainedDelta());
  EXPECT_DOUBLE_EQ(output.projection.final_port.u_w,
                   runtime.previousFinalPort().u_w);
  EXPECT_DOUBLE_EQ(output.projection.final_port.u_delta,
                   runtime.previousFinalPort().u_delta);

  RuntimePreparedStep next;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), next));
  EXPECT_DOUBLE_EQ(output.projection.next_delta, next.delta);
}

TEST(PhaseOffsetRuntimeTest, InvalidBaseGuidanceDoesNotAdvanceRuntimeState) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::NONE));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), prepared));
  RuntimeStepOutput committed;
  ASSERT_TRUE(Complete(runtime, prepared, committed));
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before = runtime.previousFinalPort();
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), prepared));
  RuntimeStepOutput invalid;
  EXPECT_FALSE(runtime.complete(prepared, prepared.geometry.T * 0.5, 0.5,
                                false, invalid));
  EXPECT_EQ(invalid.execution.mode, RuntimeExecutionMode::FATAL_CONTROL_FAILURE);
  EXPECT_EQ(invalid.execution.failure_reason,
            ControlFailureReason::BASE_GUIDANCE_INVARIANT);
  EXPECT_DOUBLE_EQ(delta_before, runtime.retainedDelta());
  EXPECT_DOUBLE_EQ(previous_before.u_w, runtime.previousFinalPort().u_w);
  EXPECT_DOUBLE_EQ(previous_before.u_delta, runtime.previousFinalPort().u_delta);
}

TEST(PhaseOffsetRuntimeTest, GateClosedTrackingExcursionKeepsCertificateSeparateFromSelection) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePrepareInput input = MakeInput(path, MakeView(), false);
  input.position.y() = 0.16;
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before = runtime.previousFinalPort();
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(input, prepared));
  EXPECT_EQ(prepared.execution.mode, RuntimeExecutionMode::NORMAL);
  EXPECT_FALSE(prepared.execution.tracking_within_bound);
  EXPECT_FALSE(prepared.execution.certificate_denied);
  EXPECT_TRUE(prepared.requires_base_guidance);
  RuntimeStepOutput output;
  EXPECT_TRUE(Complete(runtime, prepared, output));
  EXPECT_FALSE(output.selected);
  EXPECT_FALSE(output.execution.certificate_denied);
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, previous_before.u_delta);
}

TEST(PhaseOffsetRuntimeTest, RollingFixedViewExecutesAndUsesSameFinalPortEverywhere) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), prepared));
  RuntimeStepOutput output;
  ASSERT_TRUE(Complete(runtime, prepared, output));
  EXPECT_EQ(output.execution.mode, RuntimeExecutionMode::NORMAL);
  EXPECT_TRUE(output.selected);
  EXPECT_NEAR(output.matched.delta_dot, output.projection.final_port.u_delta, 1e-15);
  EXPECT_LE(output.matched.matched_residual_norm, 1e-12);
}

TEST(PhaseOffsetRuntimeTest, ActivePointerReplacementDoesNotResetDeltaOrPreviousPort) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  RuntimeStepOutput output;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), prepared));
  ASSERT_TRUE(Complete(runtime, prepared, output));
  const double expected_delta = output.delta + 0.02 * output.projection.final_port.u_delta;
  auto replacement = MakeActiveProfile(-0.06, 0.06);
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView(TubeEpochState::ROLLING, replacement)), prepared));
  EXPECT_NEAR(prepared.delta, expected_delta, 1e-12);
  ASSERT_TRUE(Complete(runtime, prepared, output));
  EXPECT_LE(output.matched.matched_residual_norm, 1e-12);
}

TEST(PhaseOffsetRuntimeTest, WaitingCandidateWithValidActiveProfileKeepsExecution) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(
      MakeInput(path, MakeView(TubeEpochState::WAITING_FOR_CANDIDATE)), prepared));
  EXPECT_EQ(prepared.execution.mode, RuntimeExecutionMode::NORMAL);
  EXPECT_FALSE(prepared.execution.transient_blocked);
  RuntimeStepOutput output;
  EXPECT_TRUE(Complete(runtime, prepared, output));
  EXPECT_TRUE(output.selected);
}

TEST(PhaseOffsetRuntimeTest, WaitingBeforeStartBlocksWithoutPermanentLatch) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  EXPECT_FALSE(runtime.prepare(MakeInput(path, MakeView(TubeEpochState::WAITING_FOR_CANDIDATE,
                                                         nullptr)), prepared));
  EXPECT_EQ(prepared.execution.mode, RuntimeExecutionMode::WAITING_FOR_CANDIDATE);
  EXPECT_FALSE(prepared.execution.fatal_control_failure);
}

TEST(PhaseOffsetRuntimeTest,
     LatestCandidateExcludingRetainedDeltaBlocksOldActiveExecution) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimeInstalledTubeView view = MakeView();
  // The old active profile still contains zero, but TubeEpochManager has
  // invalidated its current certificate because the latest Candidate does
  // not.  Runtime must wait rather than use the old profile to side-step.
  view.epoch_status.active_current_validation_valid = false;
  RuntimePreparedStep prepared;
  EXPECT_FALSE(runtime.prepare(MakeInput(path, view), prepared));
  EXPECT_EQ(prepared.execution.mode, RuntimeExecutionMode::WAITING_FOR_CANDIDATE);
  EXPECT_TRUE(prepared.execution.transient_blocked);
  EXPECT_FALSE(prepared.requires_base_guidance);
  EXPECT_DOUBLE_EQ(prepared.delta, 0.0);
}

TEST(PhaseOffsetRuntimeTest, IncompleteCandidateDoesNotDropRetainedActiveControl) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), prepared));
  RuntimeStepOutput output;
  ASSERT_TRUE(Complete(runtime, prepared, output));
  ASSERT_TRUE(runtime.prepare(
      MakeInput(path, MakeView(TubeEpochState::WAITING_FOR_CANDIDATE)), prepared));
  EXPECT_EQ(prepared.execution.mode, RuntimeExecutionMode::NORMAL);
  EXPECT_FALSE(prepared.execution.transient_blocked);
  EXPECT_NEAR(prepared.delta, output.delta +
      0.02 * output.projection.final_port.u_delta, 1e-12);
}

TEST(PhaseOffsetRuntimeTest, LivePositiveFailureReprojectsSameRawPortAsSafe) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntimeConfig config = MakeConfig(TubeSource::FIXED);
  config.manual.u_w_rate_max = 0.0;
  config.tube.invariant_gain = 10.0;
  PhaseOffsetRuntime runtime(config);
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), prepared));
  prepared.raw_port.u_w = 0.1;
  prepared.raw_port.u_delta = 0.1;
  RuntimeStepOutput output;
  EXPECT_TRUE(runtime.complete(prepared, Eigen::Vector3d::Zero(), 0.0, true,
                               output));
  EXPECT_TRUE(output.selected);
  EXPECT_EQ(output.execution.mode, RuntimeExecutionMode::SAFETY_PRIORITY);
  EXPECT_GE(output.projection.final_w_dot, -1e-12);
  EXPECT_GE(output.projection.final_tangent_speed, -1e-12);
  EXPECT_FALSE(output.execution.certificate_denied);
  EXPECT_DOUBLE_EQ(output.raw_port.u_delta, 0.1);
  EXPECT_DOUBLE_EQ(output.projection.final_port.u_delta, 0.1);
}

TEST(PhaseOffsetRuntimeTest,
     ExactPwlSingleStepWithoutShortHorizonContinuationIsDenied) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(
      MakeInput(path, MakeView(TubeEpochState::ROLLING,
                               MakePosthocRejectingProfile())), prepared));
  RuntimeStepOutput output;
  EXPECT_FALSE(Complete(runtime, prepared, output));
  EXPECT_EQ(output.execution.mode, RuntimeExecutionMode::CERTIFICATE_DENIED);
  EXPECT_TRUE(output.execution.certificate_denied);
  EXPECT_NE(output.invalid_reason.find(
                "U+ first_failure_step=1 category=projector"),
            std::string::npos);
  EXPECT_NE(output.invalid_reason.find(
                "U_safe first_failure_step=1 category=projector"),
            std::string::npos);
}

TEST(PhaseOffsetRuntimeTest, ExactPwlCrossKnotProjectionKeepsHeldStepInsideTube) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntimeConfig config = MakeConfig(TubeSource::FIXED);
  config.manual.u_w_rate_max = 100.0;
  config.manual.u_delta_rate_max = 100.0;
  PhaseOffsetRuntime runtime(config);
  Refresh(runtime, path);
  RuntimePrepareInput input = MakeInput(
      path, MakeView(TubeEpochState::ROLLING, MakeCrossKnotProfile()));
  input.dt = 0.02;
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(input, prepared));
  // w+ crosses 0.05 and 0.10.  The PWL projector must use the actual narrow
  // knot rather than accept the current upper tangent of zero.
  prepared.raw_port.u_w = 0.0;
  prepared.raw_port.u_delta = 0.25;
  RuntimeStepOutput output;
  ASSERT_TRUE(Complete(runtime, prepared, output));
  EXPECT_TRUE(output.next_bounds.valid);
  EXPECT_LE(output.projection.next_delta, output.next_bounds.upper + 1e-12);
  EXPECT_GE(output.projection.next_delta, output.next_bounds.lower - 1e-12);
}

TEST(PhaseOffsetRuntimeTest, ExactPwlFailureDoesNotCommitLiveState) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntimeConfig config = MakeConfig(TubeSource::FIXED);
  config.manual.u_delta_abs_max = 0.01;
  PhaseOffsetRuntime runtime(config);
  Refresh(runtime, path);
  RuntimePrepareInput input = MakeInput(
      path, MakeView(TubeEpochState::ROLLING, MakeImpossibleCrossKnotProfile()));
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(input, prepared));
  prepared.raw_port.u_delta = 0.25;
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before = runtime.previousFinalPort();
  RuntimeStepOutput output;
  EXPECT_FALSE(Complete(runtime, prepared, output));
  EXPECT_EQ(output.execution.mode, RuntimeExecutionMode::CERTIFICATE_DENIED);
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, previous_before.u_delta);
}

TEST(PhaseOffsetRuntimeTest, ExactPwlFallsBackFromPositiveToSafeProgress) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntimeConfig config = MakeConfig(TubeSource::FIXED);
  config.manual.phase_dot_min = 0.02;
  config.manual.tangent_speed_min = 0.02;
  config.manual.u_w_abs_max = 0.0;
  config.manual.u_w_rate_max = 0.0;
  config.manual.u_delta_abs_max = 0.0;
  config.manual.u_delta_rate_max = 0.0;
  PhaseOffsetRuntime runtime(config);
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(MakeInput(
      path, MakeView(TubeEpochState::ROLLING, MakeTightCrossKnotProfile())), prepared));
  RuntimeStepOutput output;
  ASSERT_TRUE(runtime.complete(prepared, Eigen::Vector3d::Zero(), 0.0, true, output));
  EXPECT_EQ(output.execution.mode, RuntimeExecutionMode::SAFETY_PRIORITY);
  EXPECT_DOUBLE_EQ(output.projection.final_w_dot, 0.0);
  EXPECT_FALSE(output.execution.certificate_denied);
}

TEST(PhaseOffsetRuntimeTest,
     PositiveSingleStepCanFailViabilityThenSafeHoldProvidesWitness) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntimeConfig config = MakeConfig(TubeSource::FIXED);
  config.manual.u_w_abs_max = 0.12;
  config.manual.u_w_rate_max = 100.0;
  config.manual.u_delta_rate_max = 100.0;
  PhaseOffsetRuntime runtime(config);
  Refresh(runtime, path);
  const std::shared_ptr<const TubeProfile> profile = MakeActiveProfile();
  const std::shared_ptr<std::vector<RuntimeFutureStepInput>> calls(
      new std::vector<RuntimeFutureStepInput>());
  RuntimePrepareInput input = MakeInput(
      path, MakeView(TubeEpochState::ROLLING, profile));
  // U+ must advance by the positive phase/tangent margins.  Its first held
  // step is locally exact-feasible, but the immutable future evaluator rejects
  // that positive phase.  U_safe may hold at w=0 until the next refresh.
  input.future_step = MakePhaseLimitedFutureContract(
      profile, Eigen::Vector3d::Zero(), 0.0, 1e-4, calls);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(input, prepared));
  prepared.raw_port = phase_offset_core::PortCommand();
  RuntimeStepOutput output;
  ASSERT_TRUE(runtime.complete(prepared, Eigen::Vector3d::Zero(), 0.0, true,
                               output));
  EXPECT_EQ(output.execution.mode, RuntimeExecutionMode::SAFETY_PRIORITY);
  EXPECT_NEAR(output.projection.final_w_dot, 0.0, 1e-12);
  EXPECT_TRUE(output.selected);
  bool saw_rejected_positive_phase = false;
  for (const RuntimeFutureStepInput& call : *calls) {
    saw_rejected_positive_phase =
        saw_rejected_positive_phase || call.phase > 1e-4;
  }
  EXPECT_TRUE(saw_rejected_positive_phase);
}

TEST(PhaseOffsetRuntimeTest,
     DeterministicSearchUsesNonGreedyRootPortWhenItIsTheOnlyContinuation) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntimeConfig config = MakeConfig(TubeSource::FIXED);
  config.manual.u_w_abs_max = 0.12;
  config.manual.u_w_rate_max = 100.0;
  config.manual.u_delta_rate_max = 100.0;
  PhaseOffsetRuntime runtime(config);
  Refresh(runtime, path);
  const std::shared_ptr<const TubeProfile> profile = MakeActiveProfile();
  RuntimePrepareInput input = MakeInput(
      path, MakeView(TubeEpochState::ROLLING, profile));
  // The nearest root projection is u_w=0.10.  It is exact-feasible for one
  // held step, but reaches a phase from which the next positive step exceeds
  // this immutable profile's future contract.  The lower polygon vertex
  // u_w=phase_dot_min remains viable through the full refresh horizon.
  input.future_step = MakePhaseLimitedFutureContract(
      profile, Eigen::Vector3d::Zero(), 0.0, 0.0021,
      std::shared_ptr<std::vector<RuntimeFutureStepInput>>());
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(input, prepared));
  prepared.raw_port = phase_offset_core::PortCommand();
  prepared.raw_port.u_w = 0.10;
  RuntimeStepOutput output;
  ASSERT_TRUE(runtime.complete(prepared, Eigen::Vector3d::Zero(), 0.0, true,
                               output));
  EXPECT_EQ(output.execution.mode, RuntimeExecutionMode::NORMAL);
  EXPECT_LT(output.projection.final_port.u_w, 0.05);
  EXPECT_GE(output.projection.final_port.u_w,
            config.manual.phase_dot_min - 1e-12);
}

TEST(PhaseOffsetRuntimeTest,
     MissingOrMismatchedFutureContractDeniesWithoutCommittingRuntimeState) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      runtime.previousFinalPort();

  RuntimePreparedStep missing;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), missing));
  missing.future_step = RuntimeFutureStepContract();
  RuntimeStepOutput missing_output;
  EXPECT_FALSE(Complete(runtime, missing, missing_output));
  EXPECT_EQ(missing_output.execution.mode,
            RuntimeExecutionMode::CERTIFICATE_DENIED);
  EXPECT_NE(missing_output.invalid_reason.find(
                "U+ first_failure_step=0 category=contract"),
            std::string::npos);
  EXPECT_NE(missing_output.invalid_reason.find(
                "U_safe first_failure_step=0 category=contract"),
            std::string::npos);
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, previous_before.u_delta);

  RuntimePreparedStep mismatched;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), mismatched));
  mismatched.future_step.profile_domain_end_w += 0.01;
  RuntimeStepOutput mismatched_output;
  EXPECT_FALSE(Complete(runtime, mismatched, mismatched_output));
  EXPECT_EQ(mismatched_output.execution.mode,
            RuntimeExecutionMode::CERTIFICATE_DENIED);
  EXPECT_NE(mismatched_output.invalid_reason.find(
                "U+ first_failure_step=0 category=contract"),
            std::string::npos);
  EXPECT_NE(mismatched_output.invalid_reason.find(
                "U_safe first_failure_step=0 category=contract"),
            std::string::npos);
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, previous_before.u_delta);
}

TEST(PhaseOffsetRuntimeTest,
     InvalidFutureContractDryRunDeniesWithoutCommittingRuntimeState) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      runtime.previousFinalPort();
  RuntimeDryRunInput dry = MakeDryRunInput(path, MakeView());
  dry.prepare.future_step.profile_domain_end_w += 0.01;
  RuntimeDryRunResult result;
  EXPECT_FALSE(runtime.dryRun(dry, result));
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, previous_before.u_delta);
}

TEST(PhaseOffsetRuntimeTest,
     FutureEvaluatorIsCalledForEachPredictedStepWithEvolvingState) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  const std::shared_ptr<const TubeProfile> profile = MakeActiveProfile();
  const std::shared_ptr<std::vector<RuntimeFutureStepInput>> calls(
      new std::vector<RuntimeFutureStepInput>());
  RuntimePrepareInput input = MakeInput(
      path, MakeView(TubeEpochState::ROLLING, profile));
  RuntimeFutureStepContract contract = MakeFutureContract(profile);
  const RuntimeFutureStepEvaluator evaluator = contract.evaluate;
  contract.evaluate = [calls, evaluator](const RuntimeFutureStepInput& step,
                                         RuntimeFutureStepResult& result) {
    calls->push_back(step);
    return evaluator(step, result);
  };
  input.future_step = contract;
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(input, prepared));
  RuntimeStepOutput output;
  ASSERT_TRUE(Complete(runtime, prepared, output));
  ASSERT_EQ(calls->size(), 4U);
  for (std::size_t index = 0U; index < 4U; ++index) {
    EXPECT_EQ((*calls)[index].step_index, index + 1U);
    EXPECT_TRUE((*calls)[index].matched_position.allFinite());
    EXPECT_TRUE((*calls)[index].previous_matched_reference.allFinite());
    if (index > 0U) {
      EXPECT_GT((*calls)[index].phase, (*calls)[index - 1U].phase);
    }
  }
}

TEST(PhaseOffsetRuntimeTest,
     FutureEvaluatorStepTwoFailureReportsDeepestWitnessFrontier) {
  const RuntimePathSamples path = MakePath();
  const std::shared_ptr<const TubeProfile> profile = MakeActiveProfile();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePrepareInput input = MakeInput(
      path, MakeView(TubeEpochState::ROLLING, profile));
  RuntimeFutureStepContract contract = MakeFutureContract(profile);
  const RuntimeFutureStepEvaluator evaluator = contract.evaluate;
  contract.evaluate = [evaluator](const RuntimeFutureStepInput& step,
                                  RuntimeFutureStepResult& result) {
    if (step.step_index == 2U) {
      result = RuntimeFutureStepResult();
      result.invalid_reason = "fixture rejects step two";
      return false;
    }
    return evaluator(step, result);
  };
  input.future_step = contract;
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(input, prepared));
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      runtime.previousFinalPort();
  RuntimeStepOutput output;
  EXPECT_FALSE(Complete(runtime, prepared, output));
  EXPECT_EQ(output.execution.mode, RuntimeExecutionMode::CERTIFICATE_DENIED);
  EXPECT_NE(output.invalid_reason.find(
                "U+ first_failure_step=2 category=future evaluator "
                "detail=fixture rejects step two"),
            std::string::npos);
  EXPECT_NE(output.invalid_reason.find(
                "U_safe first_failure_step=2 category=future evaluator "
                "detail=fixture rejects step two"),
            std::string::npos);
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta,
                   previous_before.u_delta);
}

TEST(PhaseOffsetRuntimeTest, ExactPwlBothProgressSetsEmptyDoesNotCommit) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntimeConfig config = MakeConfig(TubeSource::FIXED);
  config.manual.u_delta_abs_max = 0.0;
  config.manual.u_delta_rate_max = 0.0;
  PhaseOffsetRuntime runtime(config);
  Refresh(runtime, path);
  RuntimePrepareInput input = MakeInput(
      path, MakeView(TubeEpochState::ROLLING, MakeImpossibleCrossKnotProfile()));
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(input, prepared));
  prepared.raw_port.u_delta = 0.25;
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before = runtime.previousFinalPort();
  RuntimeStepOutput output;
  EXPECT_FALSE(Complete(runtime, prepared, output));
  EXPECT_EQ(output.execution.mode, RuntimeExecutionMode::CERTIFICATE_DENIED);
  EXPECT_NE(output.invalid_reason.find(
                "U+ first_failure_step=0 category=projector"),
            std::string::npos);
  EXPECT_NE(output.invalid_reason.find(
                "U_safe first_failure_step=0 category=projector"),
            std::string::npos);
  EXPECT_NE(output.invalid_reason.find(
                "constraint=joint port polygon; "
                "joint port feasible polygon is empty"),
            std::string::npos);
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, previous_before.u_delta);
}

TEST(PhaseOffsetRuntimeTest, ExactPwlDryRunFailureDoesNotCommitState) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntimeConfig config = MakeConfig(TubeSource::FIXED);
  config.manual.u_delta_abs_max = 0.0;
  config.manual.u_delta_rate_max = 0.0;
  PhaseOffsetRuntime runtime(config);
  Refresh(runtime, path);
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before = runtime.previousFinalPort();
  RuntimeDryRunInput dry = MakeDryRunInput(
      path, MakeView(TubeEpochState::ROLLING, MakeImpossibleCrossKnotProfile()));
  dry.prepare.dt = 0.02;
  RuntimeDryRunResult result;
  EXPECT_FALSE(runtime.dryRun(dry, result));
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, previous_before.u_delta);
}

TEST(PhaseOffsetRuntimeTest, TrackingBoundRevokesOnlyCertificateAndFatalGeometryStaysFatal) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePrepareInput tracking = MakeInput(path, MakeView());
  tracking.position.y() = 1.0;
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(tracking, prepared));
  EXPECT_EQ(prepared.execution.mode, RuntimeExecutionMode::NORMAL);
  EXPECT_FALSE(prepared.execution.tracking_within_bound);
  EXPECT_FALSE(prepared.execution.certificate_denied);
  EXPECT_FALSE(prepared.execution.fatal_control_failure);
  RuntimeStepOutput recovery;
  ASSERT_TRUE(Complete(runtime, prepared, recovery));
  EXPECT_TRUE(recovery.selected);
  EXPECT_EQ(recovery.execution.mode, RuntimeExecutionMode::NORMAL);
  EXPECT_FALSE(recovery.execution.certificate_denied);
  RuntimePrepareInput invalid = MakeInput(path, MakeView());
  invalid.current_path.valid = false;
  EXPECT_FALSE(runtime.prepare(invalid, prepared));
  EXPECT_EQ(prepared.execution.mode, RuntimeExecutionMode::FATAL_CONTROL_FAILURE);
  EXPECT_TRUE(prepared.execution.fatal_control_failure);
}

TEST(PhaseOffsetRuntimeTest, CertificateDenialDoesNotMutateDeltaOrPreviousPort) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  RuntimeStepOutput output;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), prepared));
  ASSERT_TRUE(Complete(runtime, prepared, output));
  const double retained_before = output.delta +
      0.02 * output.projection.final_port.u_delta;

  EXPECT_FALSE(runtime.prepare(
      MakeInput(path, MakeView(TubeEpochState::CERTIFICATE_DENIED)), prepared));
  EXPECT_EQ(prepared.execution.mode, RuntimeExecutionMode::CERTIFICATE_DENIED);
  EXPECT_TRUE(prepared.execution.certificate_denied);
  EXPECT_FALSE(prepared.execution.fatal_control_failure);
  EXPECT_NEAR(prepared.delta, retained_before, 1e-12);
}

TEST(PhaseOffsetRuntimeTest,
     DiagnosticInvariantConflictDoesNotRejectExactPwlWitness) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), prepared));
  // The current tangent invariant is deliberately contradictory, while the
  // exact PWL profile remains wide and continuously viable.  G2h makes the
  // former diagnostic-only in this exact-PWL path.
  prepared.current_bounds.lower = -0.001;
  prepared.current_bounds.upper = 0.08;
  prepared.current_bounds.lower_w = 100.0;
  prepared.current_bounds.upper_w = 0.0;
  RuntimeStepOutput output;
  EXPECT_TRUE(Complete(runtime, prepared, output));
  EXPECT_EQ(output.execution.mode, RuntimeExecutionMode::NORMAL);
  EXPECT_FALSE(output.execution.certificate_denied);
  EXPECT_FALSE(output.execution.fatal_control_failure);
  EXPECT_FALSE(output.execution.genuine_fatal_invariant);
  EXPECT_EQ(output.execution.failure_reason, ControlFailureReason::NONE);
}

TEST(PhaseOffsetRuntimeTest,
     ChangedLiveBaseFactsRecomputeExactPwlWithoutInvariantLatch) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  const RuntimeInstalledTubeView installed = MakeView();
  RuntimePreparedStep prepared;
  RuntimeStepOutput output;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, installed), prepared));
  ASSERT_TRUE(Complete(runtime, prepared, output));
  // The high-rate base phase fact has changed.  Exact PWL viability is
  // recomputed from the current base facts; a contradictory tangent
  // invariant remains diagnostic and cannot create a false empty set.
  ASSERT_TRUE(runtime.prepare(MakeInput(path, installed), prepared));
  prepared.current_bounds.lower = -0.001;
  prepared.current_bounds.upper = 0.08;
  prepared.current_bounds.lower_w = 100.0;
  prepared.current_bounds.upper_w = 0.0;
  EXPECT_TRUE(runtime.complete(prepared, prepared.geometry.T * 0.5, 0.25,
                               true, output));
  EXPECT_EQ(output.execution.mode, RuntimeExecutionMode::NORMAL);
  EXPECT_FALSE(output.execution.certificate_denied);
  EXPECT_FALSE(output.execution.fatal_control_failure);
  EXPECT_FALSE(output.execution.genuine_fatal_invariant);
  EXPECT_EQ(output.execution.failure_reason, ControlFailureReason::NONE);
  EXPECT_LT(output.projection.lower_invariant_residual, 0.0);
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), output.projection.next_delta);
}

TEST(PhaseOffsetRuntimeTest, ImmutableActiveViewIsConsumedReadOnly) {
  static_assert(std::is_same<decltype(RuntimeInstalledTubeView().active_profile),
                             std::shared_ptr<const TubeProfile>>::value,
                "Runtime must consume immutable active profiles");
  const RuntimePathSamples path = MakePath();
  const auto profile = MakeActiveProfile();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView(TubeEpochState::ROLLING, profile)), prepared));
  EXPECT_EQ(prepared.active_profile.get(), profile.get());
}

TEST(PhaseOffsetRuntimeTest, RecenterIsContinuousAndDoesNotResetBeforeCommit) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::NONE));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  RuntimeStepOutput output;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), prepared));
  ASSERT_TRUE(Complete(runtime, prepared, output));
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), prepared));
  ASSERT_TRUE(Complete(runtime, prepared, output));
  const double nonzero = runtime.retainedDelta();
  ASSERT_GT(std::abs(nonzero), 0.0);
  runtime.requestRecenter();
  EXPECT_TRUE(runtime.recenterRequested());
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), prepared));
  EXPECT_DOUBLE_EQ(nonzero, prepared.delta);
  ASSERT_TRUE(Complete(runtime, prepared, output));
  EXPECT_LT(std::abs(runtime.retainedDelta()), std::abs(nonzero));
  for (int index = 0; index < 80 && runtime.hasExecutedOffsetAuthority(); ++index) {
    ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), prepared));
    ASSERT_TRUE(Complete(runtime, prepared, output));
  }
  EXPECT_NEAR(runtime.retainedDelta(), 0.0, 1e-3);
  EXPECT_FALSE(runtime.recenterRequested());
  EXPECT_FALSE(runtime.hasExecutedOffsetAuthority());
}

TEST(PhaseOffsetRuntimeTest,
     LegacySuccessfulCommitInvalidatesPreviouslyPreparedSectionValue) {
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::NONE));
  const std::shared_ptr<const SectionTubeProfile> profile =
      MakeRuntimeSectionProfile();
  const NormalPreviewResult preview = MakeRuntimeSectionPreview(profile);
  const RuntimeSectionPrepareInput section_input =
      MakeRuntimeSectionInput(profile, preview, 0.1, 0.0, 0.1);
  RuntimeSectionPreparedStep section_prepared;
  ASSERT_TRUE(runtime.prepareSection(section_input, section_prepared))
      << section_prepared.reason();
  ASSERT_TRUE(runtime.validateSectionBeforePublish(section_prepared));

  const RuntimePathSamples path = MakePath();
  Refresh(runtime, path);
  RuntimePreparedStep legacy_prepared;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()),
                              legacy_prepared));
  RuntimeStepOutput legacy_output;
  ASSERT_TRUE(Complete(runtime, legacy_prepared, legacy_output));
  EXPECT_FALSE(runtime.validateSectionBeforePublish(section_prepared));
}
#endif

#ifndef PHASE_OFFSET_RUNTIME_LEGACY_TEST_ONLY
TEST(PhaseOffsetRuntimeTest, V2PrepareIsPureAndOwnsImmutableProfileAndReserve) {
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  std::shared_ptr<TubeProfileV2> owned_profile = MakeRuntimeV2Profile();
  RuntimeV2PrepareInput input = MakeRuntimeV2Input(owned_profile);
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      runtime.previousFinalPort();
  RuntimeV2PreparedStep prepared;
  ASSERT_TRUE(runtime.prepareV2(input, prepared)) << prepared.invalid_reason;
  ASSERT_TRUE(prepared.valid);
  ASSERT_TRUE(prepared.admission.valid);
  ASSERT_TRUE(prepared.admission.successor_reserve.terminalExact());
  EXPECT_DOUBLE_EQ(prepared.successor.w,
                   input.current.w + input.dt * input.base_phase_rate);
  EXPECT_DOUBLE_EQ(prepared.successor.delta,
                   input.current.delta + input.dt * input.selected_u.u_delta);
  EXPECT_EQ(prepared.profile.get(), owned_profile.get());
  ASSERT_TRUE(prepared.reserve_owner);
  EXPECT_TRUE(prepared.reserve_owner->terminalExact());
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, previous_before.u_delta);
  // The preparation owner keeps the immutable profile alive independently of
  // the caller's input wrapper.
  input.profile.reset();
  owned_profile.reset();
  EXPECT_TRUE(prepared.profile);
  EXPECT_TRUE(prepared.profile->structurallyValid());
}

TEST(PhaseOffsetRuntimeTest, V2CommitTokenBindsExactCommandAndSuccessor) {
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  const std::shared_ptr<const TubeProfileV2> profile =
      MakeRuntimeV2Profile();
  const RuntimeV2PrepareInput input = MakeRuntimeV2Input(profile);
  RuntimeV2PreparedStep prepared;
  ASSERT_TRUE(runtime.prepareV2(input, prepared)) << prepared.invalid_reason;
  RuntimeV2CommitToken token;
  ASSERT_TRUE(runtime.makeCommitTokenV2(prepared, token));
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      runtime.previousFinalPort();
  RuntimeV2CommitToken substituted = token;
  substituted.prepared.selected_u.u_delta = 0.01;
  EXPECT_FALSE(runtime.commitV2(substituted));
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, previous_before.u_delta);
  ASSERT_TRUE(runtime.commitV2(token));
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), prepared.successor.delta);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w,
                   prepared.successor.previous_u.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta,
                   prepared.successor.previous_u.u_delta);
  ASSERT_NE(runtime.committedV2Reserve(), nullptr);
  EXPECT_TRUE(runtime.committedV2Reserve()->terminalExact());

  // A failed PositionCommand publication leaves the token uncommitted.  A
  // retry from the same immutable input must prepare again without observing
  // any partially installed Runtime state.
  PhaseOffsetRuntime retry_runtime(MakeConfig(TubeSource::FIXED));
  RuntimeV2PreparedStep first_retry;
  ASSERT_TRUE(retry_runtime.prepareV2(input, first_retry));
  RuntimeV2CommitToken retry_token;
  ASSERT_TRUE(retry_runtime.makeCommitTokenV2(first_retry, retry_token));
  EXPECT_EQ(retry_runtime.committedV2Reserve(), nullptr);
  RuntimeV2PreparedStep second_retry;
  ASSERT_TRUE(retry_runtime.prepareV2(input, second_retry));
  EXPECT_EQ(retry_runtime.committedV2Reserve(), nullptr);
  EXPECT_DOUBLE_EQ(retry_runtime.retainedDelta(), 0.0);

  ASSERT_TRUE(retry_runtime.commitV2(retry_token));
  const double committed_delta = retry_runtime.retainedDelta();
  const phase_offset_core::PortCommand committed_previous =
      retry_runtime.previousFinalPort();
  const TubeFiniteReserveV2* committed_reserve =
      retry_runtime.committedV2Reserve();
  RuntimeV2PrepareInput stale = input;
  stale.current = first_retry.successor;
  stale.identity.binding_sequence += 1U;
  EXPECT_FALSE(retry_runtime.prepareV2(stale, second_retry));
  EXPECT_DOUBLE_EQ(retry_runtime.retainedDelta(), committed_delta);
  EXPECT_DOUBLE_EQ(retry_runtime.previousFinalPort().u_w,
                   committed_previous.u_w);
  EXPECT_DOUBLE_EQ(retry_runtime.previousFinalPort().u_delta,
                   committed_previous.u_delta);
  EXPECT_EQ(retry_runtime.committedV2Reserve(), committed_reserve);
}

TEST(PhaseOffsetRuntimeTest,
     V2CopiedPrefixBindingTransitionSealsExpectedAndProposedIdentity) {
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  const std::shared_ptr<const TubeProfileV2> source_profile =
      MakeRuntimeV2Profile();
  const RuntimeV2PrepareInput source_input =
      MakeRuntimeV2Input(source_profile);
  RuntimeV2PreparedStep source_prepared;
  ASSERT_TRUE(runtime.prepareV2(source_input, source_prepared))
      << source_prepared.invalid_reason;
  RuntimeV2CommitToken source_token;
  ASSERT_TRUE(runtime.makeCommitTokenV2(source_prepared, source_token));
  ASSERT_TRUE(runtime.commitV2(source_token));

  std::shared_ptr<TubeProfileV2> successor_profile(
      new TubeProfileV2(*source_profile));
  ++successor_profile->path_key.path_instance_id;
  ++successor_profile->profile_id;
  ++successor_profile->request_id;
  ASSERT_TRUE(successor_profile->structurallyValid());
  RuntimeV2PrepareInput successor_input =
      MakeRuntimeV2Input(successor_profile);
  successor_input.current = source_prepared.successor;
  successor_input.binding_transition = true;
  successor_input.expected_identity = source_prepared.identity;
  successor_input.identity.binding_sequence =
      successor_input.expected_identity.binding_sequence + 1U;

  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      runtime.previousFinalPort();
  const TubeFiniteReserveV2* reserve_before = runtime.committedV2Reserve();
  RuntimeV2PreparedStep successor_prepared;
  ASSERT_TRUE(runtime.prepareV2(successor_input, successor_prepared))
      << successor_prepared.invalid_reason;
  EXPECT_TRUE(successor_prepared.binding_transition);
  EXPECT_EQ(successor_prepared.expected_identity.path_instance_id,
            source_profile->path_key.path_instance_id);
  EXPECT_EQ(successor_prepared.identity.path_instance_id,
            successor_profile->path_key.path_instance_id);
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta,
                   previous_before.u_delta);
  EXPECT_EQ(runtime.committedV2Reserve(), reserve_before);

  RuntimeV2CommitToken successor_token;
  ASSERT_TRUE(runtime.makeCommitTokenV2(successor_prepared,
                                        successor_token));
  RuntimeV2CommitToken substituted = successor_token;
  ++substituted.prepared.expected_identity.binding_sequence;
  EXPECT_FALSE(runtime.commitV2(substituted));
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_EQ(runtime.committedV2Reserve(), reserve_before);

  RuntimeV2PrepareInput stale_expected = successor_input;
  ++stale_expected.expected_identity.profile_id;
  EXPECT_FALSE(runtime.prepareV2(stale_expected, successor_prepared));
  RuntimeV2PrepareInput invalid_transition = successor_input;
  invalid_transition.identity.path_instance_id =
      invalid_transition.expected_identity.path_instance_id;
  EXPECT_FALSE(runtime.prepareV2(invalid_transition, successor_prepared));

  ASSERT_TRUE(runtime.commitV2(successor_token));
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(),
                   successor_token.prepared.successor.delta);
  EXPECT_EQ(runtime.committedV2Reserve(),
            successor_token.prepared.reserve_owner.get());
  EXPECT_FALSE(runtime.commitV2(successor_token));
}

TEST(PhaseOffsetRuntimeTest,
     V2FailedRetryAndStaleEvidenceLeaveSoleRuntimeStateUnchanged) {
  const std::shared_ptr<const TubeProfileV2> profile =
      MakeRuntimeV2Profile();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      runtime.previousFinalPort();
  RuntimeV2PrepareInput input = MakeRuntimeV2Input(profile);
  RuntimeV2PreparedStep prepared;
  input.identity.path_revision += 1U;
  EXPECT_FALSE(runtime.prepareV2(input, prepared));
  input = MakeRuntimeV2Input(profile);
  input.now = input.applicability_deadline;
  EXPECT_FALSE(runtime.prepareV2(input, prepared));
  input = MakeRuntimeV2Input(profile);
  input.dt = 0.0;
  EXPECT_FALSE(runtime.prepareV2(input, prepared));
  input = MakeRuntimeV2Input(profile);
  input.tracking.error_norm = input.tracking.error_bound + 1e-3;
  EXPECT_FALSE(runtime.prepareV2(input, prepared));
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, previous_before.u_delta);
}

TEST(PhaseOffsetRuntimeTest, V2DryRunDoesNotInstallReserveOrLifecycleState) {
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  RuntimeV2PrepareInput input = MakeRuntimeV2Input(MakeRuntimeV2Profile());
  RuntimeV2PreparedStep dry;
  ASSERT_TRUE(runtime.dryRunV2(input, dry)) << dry.invalid_reason;
  EXPECT_TRUE(dry.valid);
  EXPECT_TRUE(dry.reserve_owner);
  EXPECT_EQ(runtime.committedV2Reserve(), nullptr);
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), 0.0);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, 0.0);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, 0.0);
}

TEST(PhaseOffsetRuntimeTest,
     V2SuccessfulCommitInvalidatesPreviouslyPreparedSectionValue) {
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  const std::shared_ptr<const SectionTubeProfile> section_profile =
      MakeRuntimeSectionProfile();
  const NormalPreviewResult section_preview =
      MakeRuntimeSectionPreview(section_profile);
  const RuntimeSectionPrepareInput section_input =
      MakeRuntimeSectionInput(section_profile, section_preview, 0.1, 0.0, 0.1);
  RuntimeSectionPreparedStep section_prepared;
  ASSERT_TRUE(runtime.prepareSection(section_input, section_prepared))
      << section_prepared.reason();
  ASSERT_TRUE(runtime.validateSectionBeforePublish(section_prepared));

  const std::shared_ptr<TubeProfileV2> v2_profile = MakeRuntimeV2Profile();
  RuntimeV2PrepareInput v2_input = MakeRuntimeV2Input(v2_profile);
  RuntimeV2PreparedStep v2_prepared;
  ASSERT_TRUE(runtime.prepareV2(v2_input, v2_prepared))
      << v2_prepared.invalid_reason;
  RuntimeV2CommitToken token;
  ASSERT_TRUE(runtime.makeCommitTokenV2(v2_prepared, token));
  ASSERT_TRUE(runtime.commitV2(token));
  EXPECT_FALSE(runtime.validateSectionBeforePublish(section_prepared));
}
#endif

TEST(PhaseOffsetRuntimeSectionTest,
     SectionPrepareUsesOnlyRelevantConfigurationAndBindsExactPort) {
  PhaseOffsetRuntimeConfig config = MakeConfig(TubeSource::NONE);
  config.manual.amplitude = 0.0;  // Invalid only for the legacy profile gate.
  PhaseOffsetRuntime runtime(config);
  EXPECT_FALSE(runtime.configurationValid());
  EXPECT_TRUE(runtime.sectionConfigurationValid());

  const std::shared_ptr<const SectionTubeProfile> profile =
      MakeRuntimeSectionProfile();
  const NormalPreviewResult preview = MakeRuntimeSectionPreview(profile);
  RuntimeSectionPrepareInput input =
      MakeRuntimeSectionInput(profile, preview, 0.1, 0.1, 0.1);
  PhaseOffsetAllocatorInput allocator_input;
  allocator_input.geometry = input.matched.geometry;
  allocator_input.preview = &preview;
  allocator_input.g_des = allocator_input.geometry.T * 0.1 +
      allocator_input.geometry.N * 0.1;
  allocator_input.f_w0 = input.matched.base_w_dot;
  allocator_input.previous_u = input.previous_u;
  allocator_input.dt = input.dt;
  allocator_input.bounds = input.limits;
  allocator_input.expected_path_revision = 41U;
  allocator_input.expected_frame_revision = 42U;
  allocator_input.expected_section_profile = profile.get();
  PhaseOffsetAllocatorResult allocation;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(allocator_input, allocation))
      << allocation.reason;
  ASSERT_TRUE(allocation.selectedUConsistent(0.0));
  input.matched.final_port = allocation.selected_u;
  RuntimeSectionPreparedStep prepared;
  ASSERT_TRUE(runtime.prepareSection(input, prepared)) << prepared.reason();
  EXPECT_TRUE(prepared.valid());
  EXPECT_TRUE(runtime.validateSectionBeforePublish(prepared));
  EXPECT_DOUBLE_EQ(prepared.selectedPort().u_w, input.matched.final_port.u_w);
  EXPECT_DOUBLE_EQ(prepared.selectedPort().u_delta,
                   input.matched.final_port.u_delta);
  EXPECT_DOUBLE_EQ(prepared.matchedOutput().delta_dot,
                   prepared.selectedPort().u_delta);
  EXPECT_LE(prepared.matchedOutput().matched_residual_norm, 1e-12);
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), 0.0);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, 0.0);
}

TEST(PhaseOffsetRuntimeSectionTest,
     SectionPublishFailureDoesNotCommitAndSuccessfulCommitIsOneShot) {
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::NONE));
  const std::shared_ptr<const SectionTubeProfile> profile =
      MakeRuntimeSectionProfile();
  const NormalPreviewResult preview = MakeRuntimeSectionPreview(profile);
  const RuntimeSectionPrepareInput input =
      MakeRuntimeSectionInput(profile, preview, 0.1, 0.2, 0.1);
  RuntimeSectionPreparedStep prepared;
  ASSERT_TRUE(runtime.prepareSection(input, prepared)) << prepared.reason();
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      runtime.previousFinalPort();

  // A simulated failed publication simply drops the prepared value.
  EXPECT_TRUE(runtime.validateSectionBeforePublish(prepared));
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta,
                   previous_before.u_delta);

  ASSERT_TRUE(runtime.commitSection(prepared));
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), prepared.nextDelta());
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w,
                   prepared.selectedPort().u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta,
                   prepared.selectedPort().u_delta);
  EXPECT_FALSE(runtime.validateSectionBeforePublish(prepared));
  EXPECT_FALSE(runtime.commitSection(prepared));
}

TEST(PhaseOffsetRuntimeSectionTest,
     SectionPrepareValueIsBoundToRuntimeRevisionAndInstance) {
  const std::shared_ptr<const SectionTubeProfile> profile =
      MakeRuntimeSectionProfile();
  const NormalPreviewResult preview = MakeRuntimeSectionPreview(profile);
  PhaseOffsetRuntime first(MakeConfig(TubeSource::NONE));
  RuntimeSectionPrepareInput input =
      MakeRuntimeSectionInput(profile, preview, 0.1, 0.0, 0.1);
  RuntimeSectionPreparedStep prepared;
  ASSERT_TRUE(first.prepareSection(input, prepared)) << prepared.reason();

  PhaseOffsetRuntime other(MakeConfig(TubeSource::NONE));
  EXPECT_FALSE(other.validateSectionBeforePublish(prepared));

  first.requestRecenter();
  EXPECT_FALSE(first.validateSectionBeforePublish(prepared));
  RuntimeSectionPreparedStep replacement;
  ASSERT_TRUE(first.prepareSection(input, replacement)) << replacement.reason();
  first.resetForNewNavigationTask();
  EXPECT_FALSE(first.validateSectionBeforePublish(replacement));
}

TEST(PhaseOffsetRuntimeSectionTest,
     SectionRejectsProfileOrGeometryBindingAndDoesNotMutateState) {
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::NONE));
  const std::shared_ptr<const SectionTubeProfile> profile =
      MakeRuntimeSectionProfile();
  const NormalPreviewResult preview = MakeRuntimeSectionPreview(profile);
  RuntimeSectionPrepareInput input =
      MakeRuntimeSectionInput(profile, preview, 0.1, 0.0, 0.1);
  RuntimeSectionPreparedStep prepared;
  ASSERT_TRUE(runtime.prepareSection(input, prepared)) << prepared.reason();
  const double delta_before = runtime.retainedDelta();
  input.profile = MakeRuntimeSectionProfile();
  EXPECT_FALSE(runtime.prepareSection(input, prepared));
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  input = MakeRuntimeSectionInput(profile, preview, 0.1, 0.0, 0.1);
  input.matched.geometry.path_revision = 999U;
  EXPECT_FALSE(runtime.prepareSection(input, prepared));
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
}

TEST(PhaseOffsetRuntimeSectionTest,
     PreparedSectionValueDoesNotDereferenceExpiredPreviewOrProfile) {
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::NONE));
  RuntimeSectionPreparedStep prepared;
  {
    const std::shared_ptr<const SectionTubeProfile> profile =
        MakeRuntimeSectionProfile();
    const NormalPreviewResult preview = MakeRuntimeSectionPreview(profile);
    const RuntimeSectionPrepareInput input =
        MakeRuntimeSectionInput(profile, preview, 0.1, 0.0, 0.1);
    ASSERT_TRUE(runtime.prepareSection(input, prepared)) << prepared.reason();
  }
  // prepare stores only the already-evaluated value witness; the borrowed
  // profile/preview can leave scope before final validation.
  EXPECT_TRUE(runtime.validateSectionBeforePublish(prepared));
  EXPECT_TRUE(runtime.commitSection(prepared));
}

TEST(PhaseOffsetRuntimeSectionTest,
     AllocatorSelectedPortCanAdvanceAndThenRecoverNonzeroDelta) {
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::NONE));
  const std::shared_ptr<const SectionTubeProfile> profile =
      MakeRuntimeSectionProfile();
  const NormalPreviewResult first_preview =
      MakeRuntimeSectionPreview(profile, 0.0, 0.0, 1.0);
  ASSERT_EQ(first_preview.status, TubeViabilityStatus::FEASIBLE)
      << first_preview.reason;
  RuntimeSectionPrepareInput first_input =
      MakeRuntimeSectionInput(profile, first_preview, 0.05, 0.15, 0.1);
  PhaseOffsetAllocatorInput allocator_input;
  allocator_input.geometry = first_input.matched.geometry;
  allocator_input.preview = &first_preview;
  allocator_input.g_des = allocator_input.geometry.T * 0.05 +
      allocator_input.geometry.N * 0.15;
  allocator_input.f_w0 = first_input.matched.base_w_dot;
  allocator_input.previous_u = first_input.previous_u;
  allocator_input.dt = first_input.dt;
  allocator_input.bounds = first_input.limits;
  allocator_input.expected_path_revision = 41U;
  allocator_input.expected_frame_revision = 42U;
  allocator_input.expected_section_profile = profile.get();
  PhaseOffsetAllocatorResult first_allocation;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(allocator_input,
                                              first_allocation))
      << first_allocation.reason;
  first_input.matched.final_port = first_allocation.selected_u;
  RuntimeSectionPreparedStep first_prepared;
  ASSERT_TRUE(runtime.prepareSection(first_input, first_prepared))
      << first_prepared.reason();
  ASSERT_TRUE(runtime.commitSection(first_prepared));
  ASSERT_NE(runtime.retainedDelta(), 0.0);

  const double retained = runtime.retainedDelta();
  const NormalPreviewResult second_preview =
      MakeRuntimeSectionPreview(profile, retained, first_prepared.nextW(), 1.0);
  ASSERT_EQ(second_preview.status, TubeViabilityStatus::FEASIBLE)
      << second_preview.reason;
  RuntimeSectionPrepareInput second_input = MakeRuntimeSectionInput(
      profile, second_preview, 0.0, -0.15, 0.1, retained,
      first_prepared.nextW());
  second_input.previous_u = runtime.previousFinalPort();
  allocator_input.geometry = second_input.matched.geometry;
  allocator_input.preview = &second_preview;
  allocator_input.g_des = allocator_input.geometry.N * -0.15;
  allocator_input.previous_u = second_input.previous_u;
  PhaseOffsetAllocatorResult second_allocation;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(allocator_input,
                                              second_allocation))
      << second_allocation.reason;
  second_input.matched.final_port = second_allocation.selected_u;
  RuntimeSectionPreparedStep second_prepared;
  ASSERT_TRUE(runtime.prepareSection(second_input, second_prepared))
      << second_prepared.reason();
  EXPECT_GT(second_prepared.nextW(), first_prepared.nextW());
  ASSERT_TRUE(runtime.commitSection(second_prepared));
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta,
                   second_allocation.selected_u.u_delta);
  EXPECT_NE(runtime.retainedDelta(), retained);
}

TEST(PhaseOffsetRuntimeSectionTest,
     PublicationLockCoversValidatePublishAndNoFailCommit) {
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::NONE));
  const std::shared_ptr<const SectionTubeProfile> profile =
      MakeRuntimeSectionProfile();
  const NormalPreviewResult preview = MakeRuntimeSectionPreview(profile);
  const RuntimeSectionPrepareInput input =
      MakeRuntimeSectionInput(profile, preview, 0.1, 0.0, 0.1);
  RuntimeSectionPreparedStep prepared;
  ASSERT_TRUE(runtime.prepareSection(input, prepared)) << prepared.reason();
  std::mutex publication_mutex;
  const double before = runtime.retainedDelta();
  {
    std::lock_guard<std::mutex> lock(publication_mutex);
    ASSERT_TRUE(runtime.validateSectionBeforePublish(prepared));
    const bool published = false;
    if (published) runtime.commitSectionNoFail(prepared);
  }
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), before);
  {
    std::lock_guard<std::mutex> lock(publication_mutex);
    ASSERT_TRUE(runtime.validateSectionBeforePublish(prepared));
    const bool published = true;
    if (published) runtime.commitSectionNoFail(prepared);
  }
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), prepared.nextDelta());
  EXPECT_FALSE(runtime.validateSectionBeforePublish(prepared));
}

TEST(PhaseOffsetRuntimeSectionTest,
     SectionPreparationRejectsPhysicalAndZohBoundaryViolations) {
  const std::shared_ptr<const SectionTubeProfile> profile =
      MakeRuntimeSectionProfile();
  const NormalPreviewResult preview = MakeRuntimeSectionPreview(profile);
  const auto rejected_without_state_change =
      [&](const PhaseOffsetRuntimeConfig& config,
          const RuntimeSectionPrepareInput& input) {
        PhaseOffsetRuntime runtime(config);
        RuntimeSectionPreparedStep prepared;
        EXPECT_FALSE(runtime.prepareSection(input, prepared));
        EXPECT_DOUBLE_EQ(runtime.retainedDelta(), 0.0);
        EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, 0.0);
        EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, 0.0);
      };

  // M4C-2: exceeding the tracking-error bound is an assumption violation, not
  // a command condition.  The offset is shared by the reference and the
  // physical command through B*u, the error itself comes from base path
  // tracking, and the governor's normal loop recovers it.  Holding the tick
  // here would freeze the vehicle and leave the error outside forever.
  {
    RuntimeSectionPrepareInput tracking =
        MakeRuntimeSectionInput(profile, preview, 0.1, 0.0, 0.1);
    tracking.matched.geometry.error.y() = 0.2;
    PhaseOffsetRuntime runtime(MakeConfig(TubeSource::NONE));
    RuntimeSectionPreparedStep prepared;
    EXPECT_TRUE(runtime.prepareSection(tracking, prepared));
    EXPECT_TRUE(prepared.valid());
  }

  PhaseOffsetRuntimeConfig tangent_config = MakeConfig(TubeSource::NONE);
  tangent_config.manual.tangent_speed_min = 2.0;
  rejected_without_state_change(tangent_config,
                                MakeRuntimeSectionInput(profile, preview));

  rejected_without_state_change(
      MakeConfig(TubeSource::NONE),
      MakeRuntimeSectionInput(profile, preview, 0.3, 0.0, 0.1));

  RuntimeSectionPrepareInput slew =
      MakeRuntimeSectionInput(profile, preview, 0.1, 0.0, 0.1);
  slew.limits.u_w_slew_rate = 0.0;
  rejected_without_state_change(MakeConfig(TubeSource::NONE), slew);

  RuntimeSectionPrepareInput zoh =
      MakeRuntimeSectionInput(profile, preview, 0.1, 0.0, 0.1);
  zoh.limits.zoh_dt = 0.2;
  rejected_without_state_change(MakeConfig(TubeSource::NONE), zoh);
}

TEST(PhaseOffsetRuntimeSectionTest,
     AllocatorSaturatedPhaseWindowExecutesBoundaryPortWithoutFreezingPhase) {
  const std::shared_ptr<const SectionTubeProfile> profile =
      MakeRuntimeSectionProfile();
  const NormalPreviewResult preview = MakeRuntimeSectionPreview(profile);

  // M4E: the allocator owns the phase window.  Base guidance wants to run faster
  // than upper_nu and the +/-u_w_abs_max authority cannot bring it back, so the
  // allocator saturated on the maximum negative correction.  That saturated
  // tick must execute: re-applying the window here froze the phase and held an
  // otherwise planner-valid task.
  RuntimeSectionPrepareInput fast =
      MakeRuntimeSectionInput(profile, preview, -0.2, 0.0, 0.1);
  fast.matched.base_w_dot = 2.0;
  fast.phase_window_saturated = true;
  PhaseOffsetRuntime fast_runtime(MakeConfig(TubeSource::NONE));
  RuntimeSectionPreparedStep fast_prepared;
  ASSERT_TRUE(fast_runtime.prepareSection(fast, fast_prepared))
      << fast_prepared.reason();
  EXPECT_DOUBLE_EQ(fast_prepared.matchedOutput().w_dot, 1.8);
  EXPECT_GT(fast_prepared.nextW(), fast.matched.geometry.w);
  EXPECT_TRUE(fast_runtime.commitSection(fast_prepared));

  // The identical out-of-window command without the allocator's saturation
  // evidence stays fail-closed: the waiver is not a general window opening.
  RuntimeSectionPrepareInput unmarked = fast;
  unmarked.phase_window_saturated = false;
  PhaseOffsetRuntime unmarked_runtime(MakeConfig(TubeSource::NONE));
  RuntimeSectionPreparedStep unmarked_prepared;
  EXPECT_FALSE(unmarked_runtime.prepareSection(unmarked, unmarked_prepared));
  EXPECT_DOUBLE_EQ(unmarked_runtime.retainedDelta(), 0.0);
  EXPECT_DOUBLE_EQ(unmarked_runtime.previousFinalPort().u_w, 0.0);

  // Saturation evidence plus an off-boundary correction is not the allocator's
  // saturation either.  A correction that moves *away* from the window is the
  // clearest such case, so it also stays rejected.
  RuntimeSectionPrepareInput off_boundary = fast;
  off_boundary.matched.final_port.u_w = 0.1;
  PhaseOffsetRuntime off_boundary_runtime(MakeConfig(TubeSource::NONE));
  RuntimeSectionPreparedStep off_boundary_prepared;
  EXPECT_FALSE(off_boundary_runtime.prepareSection(off_boundary,
                                                   off_boundary_prepared));

  // The allocator also saturates on the *slew* boundary when the admissible
  // rate is more than one tick away from the last committed rate.  That tick
  // ramps towards the window rather than reaching it, and it must execute too:
  // dropping it would mean the committed rate never travels to the admissible
  // set, which is the observed `phase scalar slew interval is empty` freeze.
  RuntimeSectionPrepareInput ramping =
      MakeRuntimeSectionInput(profile, preview, -0.05, 0.0, 0.1);
  ramping.matched.base_w_dot = 2.0;
  ramping.phase_window_saturated = true;
  PhaseOffsetRuntime ramping_runtime(MakeConfig(TubeSource::NONE));
  RuntimeSectionPreparedStep ramping_prepared;
  ASSERT_TRUE(ramping_runtime.prepareSection(ramping, ramping_prepared))
      << ramping_prepared.reason();
  EXPECT_DOUBLE_EQ(ramping_prepared.matchedOutput().w_dot, 1.95);
  EXPECT_GT(ramping_prepared.nextW(), ramping.matched.geometry.w);
  EXPECT_TRUE(ramping_runtime.commitSection(ramping_prepared));

  // Base guidance wants the reference to retreat (the vehicle is behind it) and
  // the saturated rate is still negative.  Non-reversing phase progression
  // wins: the phase coordinate is held for this tick, and the transverse port,
  // physical command and committed delta stay the allocator's saturated pair.
  RuntimeSectionPrepareInput slow =
      MakeRuntimeSectionInput(profile, preview, 0.2, 0.0, 0.1);
  slow.matched.base_w_dot = -0.4;
  slow.phase_window_saturated = true;
  PhaseOffsetRuntime slow_runtime(MakeConfig(TubeSource::NONE));
  RuntimeSectionPreparedStep slow_prepared;
  ASSERT_TRUE(slow_runtime.prepareSection(slow, slow_prepared))
      << slow_prepared.reason();
  EXPECT_DOUBLE_EQ(slow_prepared.matchedOutput().w_dot, 0.0);
  EXPECT_DOUBLE_EQ(slow_prepared.nextW(), slow.matched.geometry.w);
  EXPECT_DOUBLE_EQ(slow_prepared.selectedPort().u_w,
                   slow.limits.u_w_abs_max);
  EXPECT_TRUE(slow_runtime.commitSection(slow_prepared));
  EXPECT_DOUBLE_EQ(slow_runtime.previousFinalPort().u_w,
                   slow.limits.u_w_abs_max);
}

}  // namespace
}  // namespace phase_offset_navigation

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
