#include <gtest/gtest.h>

#include "phase_offset_navigation/tube_execution_v2.h"

#include <phase_offset_core/path_state.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

namespace phase_offset_navigation {
namespace {

phase_offset_core::Binary64Interval Interval(const double lower,
                                             const double upper,
                                             const bool valid = true) {
  phase_offset_core::Binary64Interval result;
  result.lower = lower;
  result.upper = upper;
  result.valid = valid;
  return result;
}

phase_offset_core::Binary64VectorInterval VectorInterval() {
  phase_offset_core::Binary64VectorInterval result;
  result.valid = true;
  for (phase_offset_core::Binary64Interval& component : result.component) {
    component = Interval(0.0, 0.0);
  }
  return result;
}

phase_offset_core::CertifiedPathCellV2 PathCell(
    const double w0, const double w1, const std::uint64_t segment) {
  phase_offset_core::CertifiedPathCellV2 cell;
  cell.w0 = w0;
  cell.w1 = w1;
  cell.anchor_w = 0.5 * (w0 + w1);
  cell.path_revision = 11U;
  cell.frame_revision = 12U;
  cell.segment_identity = segment;
  cell.proof_identity = 100U + segment;
  cell.anchor_position = VectorInterval();
  cell.anchor_p_w = VectorInterval();
  cell.anchor_p_ww = VectorInterval();
  cell.inf_p_w_norm = Interval(1.0, 1.0);
  cell.sup_p_w_norm = Interval(1.0, 1.0);
  cell.inf_horizontal_p_w_norm = Interval(1.0, 1.0);
  cell.sup_p_ww_norm = Interval(0.0, 0.0);
  cell.sup_horizontal_p_ww_norm = Interval(0.0, 0.0);
  cell.sup_p_www_norm = Interval(0.0, 0.0);
  cell.sup_normal_derivative = Interval(0.0, 0.0);
  cell.normal_variation = Interval(0.0, 0.0);
  cell.tangent_variation = Interval(0.0, 0.0);
  cell.curvature_variation = Interval(0.0, 0.0);
  cell.midpoint_position_variation = Interval(0.0, 0.0);
  cell.chord_deviation = Interval(0.0, 0.0);
  cell.horizontal_acceleration_bound_complete = true;
  cell.normal_frame_proof_complete = true;
  cell.phase_map_proof_complete = true;
  cell.provenance = phase_offset_core::kWorldHorizontalCrossProductProvenance;
  cell.complete = true;
  cell.valid = true;
  return cell;
}

TubePathKey PathKey() {
  TubePathKey key;
  key.execution_generation = 21U;
  key.path_instance_id = 22U;
  key.path_revision = 11U;
  key.frame_revision = 12U;
  key.frame_convention_id = 13U;
  key.frame_convention = phase_offset_core::kWorldHorizontalCrossProductProvenance;
  key.phase_orientation = 1;
  key.domain_start = 0.0;
  key.domain_end = 2.0;
  return key;
}

TubeConfigurationKey ConfigurationKey() {
  TubeConfigurationKey key;
  key.configuration_id = 31U;
  key.epsilon = 0.1;
  key.nominal_half_width = 0.5;
  key.ray_step = 0.05;
  key.snapshot_resolution = 0.05;
  key.minimum_reference_speed = 1e-8;
  return key;
}

TubeMapCaptureKey MapKey() {
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

TubeProfileV2 Profile(const bool constriction = false) {
  TubeProfileV2 profile;
  profile.path_key = PathKey();
  profile.configuration_key = ConfigurationKey();
  profile.map_capture_key = MapKey();
  profile.profile_id = 51U;
  profile.request_id = 52U;
  profile.requested_start = 0.0;
  profile.requested_end = 2.0;
  profile.anchor_w = 0.0;
  profile.certified_start = 0.0;
  profile.certified_end = 2.0;
  profile.valid = true;
  profile.complete = true;
  profile.contains_anchor = true;
  profile.contains_zero_everywhere = true;
  profile.nonzero_capacity = !constriction;
  profile.capability = TubeProfileV2Capability::OFFSET_CERTIFIED;
  profile.path_owner = std::shared_ptr<const void>(new int(1));
  profile.capture_owner = std::shared_ptr<const void>(new int(2));
  profile.query_owner = std::shared_ptr<const void>(new int(3));
  profile.applicability_assumptions = "deterministic-complete-support";
  profile.applicability_deadline_ticks = 10000U;
  profile.applicability_deadline_timeless = false;

  const double lower_mid = constriction ? 0.0 : -0.5;
  const double upper_mid = constriction ? 0.0 : 0.5;
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
  middle.lower = lower_mid;
  middle.upper = upper_mid;
  middle.left_cell_id = 61U;
  middle.right_cell_id = 62U;
  middle.left_lower_slope_interval = TubeDirectedRatioV2{0.0, 0.0, true};
  middle.left_upper_slope_interval = TubeDirectedRatioV2{0.0, 0.0, true};
  middle.right_lower_slope_interval = TubeDirectedRatioV2{0.0, 0.0, true};
  middle.right_upper_slope_interval = TubeDirectedRatioV2{0.0, 0.0, true};
  TubePwlKnotV2 last = middle;
  last.w = 2.0;
  last.lower = -0.5;
  last.upper = 0.5;
  last.left_cell_id = 62U;
  last.right_cell_id = 0U;
  last.valid = true;
  last.right_lower_slope_interval = TubeDirectedRatioV2();
  last.right_upper_slope_interval = TubeDirectedRatioV2();
  profile.knots = {first, middle, last};

  TubeProofCellV2 cell0;
  cell0.w0 = 0.0;
  cell0.w1 = 1.0;
  cell0.lower = -0.5;
  cell0.upper = 0.5;
  cell0.cell_id = 61U;
  cell0.valid = true;
  cell0.complete = true;
  cell0.path_cell = PathCell(0.0, 1.0, 71U);
  TubeProofCellV2 cell1 = cell0;
  cell1.w0 = 1.0;
  cell1.w1 = 2.0;
  cell1.cell_id = 62U;
  cell1.path_cell = PathCell(1.0, 2.0, 72U);
  profile.cells = {cell0, cell1};
  return profile;
}

TubeProfileV2 ProfileWithFutureKContraction() {
  TubeProfileV2 profile = Profile();
  // Keep I broad at the crossed knot while making the future endpoint a
  // singleton.  The moving-window K therefore contracts inward at w=1 even
  // though the whole held command remains inside I there.
  profile.knots.back().lower = 0.0;
  profile.knots.back().upper = 0.0;
  return profile;
}

TubeExecutionIdentityV2 Identity(const TubeProfileV2& profile) {
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

TubeExecutionLimitsV2 Limits() {
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

NormalPreviewProductionPolicy ExecutionPreviewPolicy(
    const double horizon, const double spacing, const double lower_nu,
    const double upper_nu) {
  NormalPreviewProductionPolicy policy;
  policy.preview_horizon_w = horizon;
  policy.sample_spacing_w = spacing;
  policy.lower_nu = lower_nu;
  policy.upper_nu = upper_nu;
  policy.b_tight = 0.1;
  policy.b_open = 0.5;
  policy.policy_revision = 1U;
  policy.configuration_identity = ConfigurationKey().configuration_id;
  policy.configuration_id = "T15/execution-preview-policy";
  return policy;
}

TubeExecutionReserveInputV2 ReserveInput(const TubeProfileV2& profile) {
  TubeExecutionReserveInputV2 input;
  input.profile = &profile;
  input.identity = Identity(profile);
  input.initial.w = 0.1;
  input.initial.delta = 0.2;
  input.initial.previous_u.u_w = 0.2;
  input.initial.previous_u.u_delta = 0.5;
  input.base_phase_rate = 0.1;
  input.phase_rate_lower = 0.05;
  input.phase_rate_upper = 0.3;
  input.approved_upper_nu = 0.5;
  input.dt = 0.1;
  input.now = 1.0;
  input.deadline = 100.0;
  input.deadline_valid = true;
  input.limits = Limits();
  // Leave binary64 room on the BRAKE slew boundary.  RETURN retains its
  // independent 0.5 slew bound below.
  input.limits.u_delta_slew_rate = 1.25;
  input.max_schedule_steps = 200U;
  input.max_work = 1000U;
  input.provenance = "T15/fixed-reserve-fixture";
  return input;
}

TEST(TubeExecutionV2Test, LiveKRetainsEveryCrossedPwlKnot) {
  const TubeProfileV2 profile = Profile();
  TubeExecutionAdmissionInputV2 input;
  input.profile = &profile;
  input.identity = Identity(profile);
  input.current.w = 0.25;
  input.current.delta = 0.0;
  input.selected_u = phase_offset_core::PortCommand();
  input.base_phase_rate = 2.0;
  input.phase_rate_lower = 2.0;
  input.phase_rate_upper = 2.0;
  input.horizon_w = 1.25;
  input.sample_spacing_w = 0.25;
  input.preview_policy = ExecutionPreviewPolicy(1.0, 0.25, 2.0, 2.0);
  input.upper_u_delta = 0.2;
  input.dt = 0.15;
  input.now = 1.0;
  input.limits = Limits();
  input.limits.lower_phase_rate = 2.0;
  input.limits.upper_phase_rate = 2.0;
  input.limits.upper_nu = 2.0;
  input.tracking = TubeExecutionTrackingEvidenceV2();
  input.max_work = 1000U;
  input.provenance = "T15/live-k-fixture";
  TubeStepAdmissionV2 output;
  ASSERT_TRUE(TubeExecutionGuardV2::prepareAdmission(input, output))
      << output.reason;
  ASSERT_TRUE(output.live_k.valid);
  ASSERT_EQ(output.live_k.nodes.size(), 5U);
  EXPECT_DOUBLE_EQ(output.live_k.nodes[0].w, 0.25);
  EXPECT_DOUBLE_EQ(output.live_k.nodes[1].w, 0.5);
  EXPECT_DOUBLE_EQ(output.live_k.nodes[2].w, 0.75);
  EXPECT_DOUBLE_EQ(output.live_k.nodes[3].w, 1.0);
  EXPECT_DOUBLE_EQ(output.live_k.nodes[4].w, 1.25);
  EXPECT_TRUE(std::any_of(
      output.live_k.nodes.begin(), output.live_k.nodes.end(),
      [](const TubeExecutionIntervalV2& node) { return node.w == 0.5; }));
  EXPECT_TRUE(std::any_of(
      output.live_k.nodes.begin(), output.live_k.nodes.end(),
      [](const TubeExecutionIntervalV2& node) { return node.w == 1.0; }));
  EXPECT_EQ(output.crossed_breakpoint_count, 1U);
  EXPECT_TRUE(output.successor_reserve.valid);
  EXPECT_EQ(output.successor.w, input.current.w + input.dt * 2.0);

  TubeExecutionAdmissionInputV2 malformed_spacing = input;
  malformed_spacing.sample_spacing_w = 0.0;
  EXPECT_FALSE(TubeExecutionGuardV2::prepareAdmission(malformed_spacing, output));
  EXPECT_EQ(output.status, TubeExecutionStatusV2::INVALID_INPUT);
  TubeExecutionAdmissionInputV2 over_budget = input;
  over_budget.max_work = 1U;
  EXPECT_FALSE(TubeExecutionGuardV2::prepareAdmission(over_budget, output));
  EXPECT_EQ(output.status, TubeExecutionStatusV2::K_INFEASIBLE);

  TubeViabilityInput viability_input;
  viability_input.current_w = input.current.w;
  viability_input.current_delta = input.current.delta;
  viability_input.policy = input.preview_policy;
  viability_input.upper_u_delta = input.upper_u_delta;
  viability_input.max_work = input.max_work;
  viability_input.v2_provenance_bound = true;
  viability_input.expected_v2_path_key = profile.path_key;
  viability_input.expected_v2_configuration_key = profile.configuration_key;
  viability_input.expected_v2_map_capture_key = profile.map_capture_key;
  TubeViabilityResult viability;
  ASSERT_TRUE(TubeViability::evaluate(profile, viability_input, viability));
  TubeExecutionAdmissionInputV2 substituted = input;
  substituted.preview_result = &viability;
  substituted.upper_u_delta = std::nextafter(input.upper_u_delta, 1.0);
  EXPECT_FALSE(TubeExecutionGuardV2::prepareAdmission(substituted, output));
  EXPECT_EQ(output.status, TubeExecutionStatusV2::K_INFEASIBLE);
  substituted = input;
  substituted.preview_result = &viability;
  substituted.preview_policy.upper_nu = std::nextafter(
      input.preview_policy.upper_nu, 0.0);
  EXPECT_FALSE(TubeExecutionGuardV2::prepareAdmission(substituted, output));
  EXPECT_EQ(output.status, TubeExecutionStatusV2::INVALID_INPUT);
}

TEST(TubeExecutionV2Test, WholeHeldTickRejectsCrossedNarrowKnot) {
  const TubeProfileV2 profile = Profile(true);
  TubeExecutionAdmissionInputV2 input;
  input.profile = &profile;
  input.identity = Identity(profile);
  input.current.w = 0.8;
  input.current.delta = 0.0;
  input.selected_u.u_delta = 0.1;
  input.base_phase_rate = 2.0;
  input.phase_rate_lower = 2.0;
  input.phase_rate_upper = 2.0;
  input.horizon_w = 1.2;
  input.sample_spacing_w = 0.2;
  input.preview_policy = ExecutionPreviewPolicy(1.2 - 0.8, 0.2, 2.0, 2.0);
  input.upper_u_delta = 0.1;
  input.dt = 0.1;
  input.now = 1.0;
  input.limits = Limits();
  input.limits.lower_phase_rate = 2.0;
  input.limits.upper_phase_rate = 2.0;
  input.limits.upper_nu = 2.0;
  input.tracking = TubeExecutionTrackingEvidenceV2();
  input.max_work = 1000U;
  input.provenance = "T15/crossed-knot-fixture";
  TubeStepAdmissionV2 output;
  EXPECT_FALSE(TubeExecutionGuardV2::prepareAdmission(input, output));
  EXPECT_EQ(output.status, TubeExecutionStatusV2::K_INFEASIBLE);
}

TEST(TubeExecutionV2Test,
     NonDyadicWindowUsesAuthoritativeViabilityEndpointWithoutRecomputation) {
  const TubeProfileV2 profile = Profile();
  TubeExecutionAdmissionInputV2 input;
  input.profile = &profile;
  input.identity = Identity(profile);
  input.current.w = 0.1;
  input.current.delta = 0.0;
  input.selected_u = phase_offset_core::PortCommand();
  input.base_phase_rate = 0.1;
  input.phase_rate_lower = 0.05;
  input.phase_rate_upper = 0.5;
  // This ordinary endpoint is intentionally not the outward P07 SumCeilV2
  // endpoint for 0.1 + 0.2.  Admission must consume the returned endpoint.
  input.horizon_w = 0.3;
  input.sample_spacing_w = 0.1;
  input.preview_policy = ExecutionPreviewPolicy(0.2, 0.1, 0.05, 0.5);
  input.upper_u_delta = 0.2;
  input.dt = 0.1;
  input.now = 1.0;
  input.limits = Limits();
  input.tracking = TubeExecutionTrackingEvidenceV2();
  input.max_work = 1000U;
  input.provenance = "T15/non-dyadic-window-fixture";
  TubeStepAdmissionV2 output;
  ASSERT_TRUE(TubeExecutionGuardV2::prepareAdmission(input, output))
      << output.reason;
  ASSERT_TRUE(output.live_k.valid);
  ASSERT_TRUE(output.live_k.viability.valid);
  EXPECT_NE(output.live_k.viability.preview_end_w, input.horizon_w);
  EXPECT_DOUBLE_EQ(output.live_k.viability.preview_end_w,
                   output.live_k.nodes.back().w);
  EXPECT_LE(output.successor.w, output.live_k.viability.preview_end_w);
  EXPECT_TRUE(output.successor_reserve.valid);
}

TEST(TubeExecutionV2Test, WholeHeldTickRejectsCrossedLiveKKnot) {
  const TubeProfileV2 profile = ProfileWithFutureKContraction();
  TubeExecutionAdmissionInputV2 input;
  input.profile = &profile;
  input.identity = Identity(profile);
  input.current.w = 0.8;
  input.current.delta = 0.0;
  input.selected_u.u_delta = 0.1;
  input.base_phase_rate = 2.0;
  input.phase_rate_lower = 2.0;
  input.phase_rate_upper = 2.0;
  input.horizon_w = 2.0;
  input.sample_spacing_w = 0.05;
  input.preview_policy = ExecutionPreviewPolicy(2.0 - 0.8, 0.05, 2.0, 2.0);
  input.upper_u_delta = 0.01;
  input.dt = 0.15;
  input.now = 1.0;
  input.limits = Limits();
  input.limits.lower_phase_rate = 2.0;
  input.limits.upper_phase_rate = 2.0;
  input.limits.upper_nu = 2.0;
  input.tracking = TubeExecutionTrackingEvidenceV2();
  input.max_work = 1000U;
  input.provenance = "T15/crossed-live-k-knot-fixture";
  TubeStepAdmissionV2 output;
  EXPECT_FALSE(TubeExecutionGuardV2::prepareAdmission(input, output));
  EXPECT_EQ(output.status, TubeExecutionStatusV2::COMMAND_INFEASIBLE);
}

TEST(TubeExecutionV2Test, FixedReserveIncludesOvershootReturnAndExactTerminal) {
  const TubeProfileV2 profile = Profile();
  TubeExecutionReserveInputV2 input = ReserveInput(profile);
  TubeFiniteReserveV2 reserve;
  ASSERT_TRUE(TubeExecutionGuardV2::prepareReserve(input, reserve))
      << reserve.reason;
  ASSERT_TRUE(reserve.valid);
  ASSERT_TRUE(reserve.terminalExact());
  ASSERT_TRUE(TubeExecutionGuardV2::validateReserve(reserve));
  ASSERT_FALSE(reserve.steps.empty());
  EXPECT_EQ(reserve.steps.front().segment, TubeReserveSegmentKindV2::BRAKE);
  EXPECT_TRUE(std::any_of(
      reserve.steps.begin(), reserve.steps.end(), [](const TubeReserveStepV2& step) {
        return step.segment == TubeReserveSegmentKindV2::RETURN;
      }));
  EXPECT_EQ(reserve.terminal.delta, 0.0);
  EXPECT_EQ(reserve.terminal.previous_u.u_delta, 0.0);
  EXPECT_EQ(reserve.terminal.previous_u.u_w, 0.0);

  double maximum_brake_delta = reserve.initial.delta;
  bool saw_outward_brake = false;
  for (const TubeReserveStepV2& step : reserve.steps) {
    ASSERT_TRUE(step.after.w <= reserve.w_max);
    ASSERT_TRUE(step.before.delta >= reserve.common_lower);
    ASSERT_TRUE(step.before.delta <= reserve.common_upper);
    ASSERT_TRUE(step.after.delta >= reserve.common_lower);
    ASSERT_TRUE(step.after.delta <= reserve.common_upper);
    EXPECT_DOUBLE_EQ(step.after.previous_u.u_w, step.command.u_w);
    EXPECT_DOUBLE_EQ(step.after.previous_u.u_delta, step.command.u_delta);
    if (step.segment == TubeReserveSegmentKindV2::BRAKE) {
      maximum_brake_delta = std::max(maximum_brake_delta, step.after.delta);
      const double w_step = Limits().u_w_slew_rate * input.dt;
      const double delta_step = input.limits.u_delta_slew_rate * input.dt;
      EXPECT_LE(std::abs(step.command.u_w - step.before.previous_u.u_w),
                w_step);
      EXPECT_LE(std::abs(step.command.u_delta -
                         step.before.previous_u.u_delta), delta_step);
    }
    if (step.segment == TubeReserveSegmentKindV2::RETURN) {
      EXPECT_LE(std::abs(step.command.u_delta),
                input.limits.return_u_delta_max);
      EXPECT_LE(std::abs(step.command.u_delta -
                         step.before.previous_u.u_delta),
                input.limits.return_u_delta_slew_rate * input.dt);
      EXPECT_LE(step.after.delta, step.before.delta);
    }
  }
  saw_outward_brake = maximum_brake_delta > reserve.initial.delta;
  EXPECT_TRUE(saw_outward_brake);
  EXPECT_GE(reserve.w_max, reserve.terminal.w);
  double conservative_w_max = reserve.initial.w;
  for (const TubeReserveStepV2& step : reserve.steps) {
    const double phase_upper = input.phase_rate_upper + step.command.u_w;
    double contribution_upper = 0.0;
    ASSERT_TRUE(phase_offset_core::outwardUpperProduct(
        input.dt, phase_upper, contribution_upper));
    const double summed = conservative_w_max + contribution_upper;
    ASSERT_TRUE(std::isfinite(summed));
    conservative_w_max = std::nextafter(
        summed, std::numeric_limits<double>::infinity());
  }
  EXPECT_GE(reserve.w_max, conservative_w_max);
  EXPECT_EQ(reserve.steps.back().segment, TubeReserveSegmentKindV2::SETTLE);
  EXPECT_DOUBLE_EQ(reserve.steps.back().command.u_w, 0.0);
  EXPECT_DOUBLE_EQ(reserve.steps.back().command.u_delta, 0.0);
  const TubeFiniteReserveV2 valid_reserve = reserve;

  TubeExecutionReserveInputV2 exhausted = ReserveInput(profile);
  exhausted.max_schedule_steps = 1U;
  EXPECT_FALSE(TubeExecutionGuardV2::prepareReserve(exhausted, reserve));
  exhausted = ReserveInput(profile);
  exhausted.max_work = 1U;
  EXPECT_FALSE(TubeExecutionGuardV2::prepareReserve(exhausted, reserve));

  TubeProfileV2 malformed = profile;
  malformed.knots[1].w = 0.5;
  TubeExecutionReserveInputV2 malformed_input = ReserveInput(malformed);
  EXPECT_FALSE(TubeExecutionGuardV2::prepareReserve(malformed_input, reserve));
  TubeExecutionReserveInputV2 extrapolated = ReserveInput(profile);
  extrapolated.initial.w = profile.certified_end + 0.1;
  EXPECT_FALSE(TubeExecutionGuardV2::prepareReserve(extrapolated, reserve));

  TubeFiniteReserveV2 forged_after = valid_reserve;
  forged_after.steps.front().after.previous_u.u_delta = 0.0;
  EXPECT_FALSE(TubeExecutionGuardV2::validateReserve(forged_after));
  TubeFiniteReserveV2 forged_terminal = valid_reserve;
  forged_terminal.terminal.w += 0.001;
  EXPECT_FALSE(TubeExecutionGuardV2::validateReserve(forged_terminal));
}

TEST(TubeExecutionV2Test, ZeroSlewOrUnavailablePhaseBoundFailsClosed) {
  const TubeProfileV2 profile = Profile();
  TubeExecutionReserveInputV2 input = ReserveInput(profile);
  input.limits.u_delta_slew_rate = 0.0;
  TubeFiniteReserveV2 reserve;
  EXPECT_FALSE(TubeExecutionGuardV2::prepareReserve(input, reserve));

  input = ReserveInput(profile);
  input.phase_rate_upper = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(TubeExecutionGuardV2::prepareReserve(input, reserve));
}

TEST(TubeExecutionV2Test, ExtrapolatedIdentityAndTrackingAreUnavailable) {
  const TubeProfileV2 profile = Profile();
  TubeExecutionAdmissionInputV2 input;
  input.profile = &profile;
  input.identity = Identity(profile);
  input.identity.path_instance_id += 1U;
  input.current.w = 0.25;
  input.current.delta = 0.0;
  input.base_phase_rate = 0.1;
  input.phase_rate_lower = 0.05;
  input.phase_rate_upper = 0.5;
  input.horizon_w = 1.25;
  input.sample_spacing_w = 0.25;
  input.preview_policy = ExecutionPreviewPolicy(1.0, 0.25, 0.05, 0.5);
  input.upper_u_delta = 0.2;
  input.dt = 0.1;
  input.now = 1.0;
  input.limits = Limits();
  input.max_work = 1000U;
  input.provenance = "T15/identity-fixture";
  TubeStepAdmissionV2 output;
  EXPECT_FALSE(TubeExecutionGuardV2::prepareAdmission(input, output));
  EXPECT_EQ(output.status, TubeExecutionStatusV2::PROFILE_UNAVAILABLE);

  input.identity = Identity(profile);
  input.tracking.valid = false;
  EXPECT_FALSE(TubeExecutionGuardV2::prepareAdmission(input, output));
  EXPECT_EQ(output.status, TubeExecutionStatusV2::TRACKING_UNAVAILABLE);
}

}  // namespace
}  // namespace phase_offset_navigation

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
