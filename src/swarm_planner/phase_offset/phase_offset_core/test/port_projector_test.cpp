#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "phase_offset_core/port_projector.h"

namespace phase_offset_core {
namespace {

PortProjectionLimits MakeLimits() {
  PortProjectionLimits limits;
  limits.u_w_abs_max = 0.4;
  limits.u_delta_abs_max = 0.5;
  limits.u_w_rate_max = 10.0;
  limits.u_delta_rate_max = 10.0;
  limits.phase_dot_min = 0.1;
  limits.tangent_speed_min = 0.1;
  limits.regularity_margin = 0.1;
  return limits;
}

PortProjectionInput MakeInput() {
  PortProjectionInput input;
  input.dt = 0.1;
  input.delta = 0.0;
  input.curvature = 0.0;
  input.r_w_norm = 2.0;
  input.base_w_dot = 0.5;
  input.base_tangent_speed = 0.5;
  return input;
}

void ExpectFinite(const PortProjectionResult& output) {
  EXPECT_TRUE(std::isfinite(output.final_port.u_w));
  EXPECT_TRUE(std::isfinite(output.final_port.u_delta));
  EXPECT_TRUE(std::isfinite(output.final_w_dot));
  EXPECT_TRUE(std::isfinite(output.final_tangent_speed));
  EXPECT_TRUE(std::isfinite(output.next_delta));
  EXPECT_TRUE(std::isfinite(output.next_regularity));
}

TEST(PortProjectorTest, KeepsRawPortInsideAllLocalLimits) {
  PortProjectionInput input = MakeInput();
  input.raw.u_w = 0.2;
  input.raw.u_delta = -0.3;
  PortProjectionResult output;

  ASSERT_TRUE(PortProjector::project(input, MakeLimits(), output));
  EXPECT_NEAR(output.final_port.u_w, input.raw.u_w, 1e-15);
  EXPECT_NEAR(output.final_port.u_delta, input.raw.u_delta, 1e-15);
  EXPECT_FALSE(output.u_w_limited);
  EXPECT_FALSE(output.u_delta_limited);
  EXPECT_NEAR(output.final_w_dot, 0.7, 1e-15);
  EXPECT_NEAR(output.final_tangent_speed, 0.9, 1e-15);
  ExpectFinite(output);
}

TEST(PortProjectorTest, EnforcesBothAmplitudeBounds) {
  PortProjectionInput input = MakeInput();
  input.raw.u_w = 1.0;
  input.raw.u_delta = -2.0;
  PortProjectionResult output;

  ASSERT_TRUE(PortProjector::project(input, MakeLimits(), output));
  EXPECT_NEAR(output.final_port.u_w, 0.4, 1e-15);
  EXPECT_NEAR(output.final_port.u_delta, -0.5, 1e-15);
  EXPECT_TRUE(output.u_w_limited);
  EXPECT_TRUE(output.u_delta_limited);
  EXPECT_NE(output.u_w_limit_reason.find("amplitude"), std::string::npos);
  EXPECT_NE(output.u_delta_limit_reason.find("amplitude"), std::string::npos);
}

TEST(PortProjectorTest, EnforcesExplicitRateLimits) {
  PortProjectionInput input = MakeInput();
  input.previous_final.u_w = 0.1;
  input.previous_final.u_delta = -0.2;
  input.raw.u_w = 0.4;
  input.raw.u_delta = 0.4;
  PortProjectionLimits limits = MakeLimits();
  limits.u_w_rate_max = 0.5;
  limits.u_delta_rate_max = 1.0;
  PortProjectionResult output;

  ASSERT_TRUE(PortProjector::project(input, limits, output));
  EXPECT_NEAR(output.final_port.u_w, 0.15, 1e-15);
  EXPECT_NEAR(output.final_port.u_delta, -0.1, 1e-15);
  EXPECT_TRUE(output.u_w_limited);
  EXPECT_TRUE(output.u_delta_limited);
  EXPECT_NE(output.u_w_limit_reason.find("rate"), std::string::npos);
  EXPECT_NE(output.u_delta_limit_reason.find("rate"), std::string::npos);
}

TEST(PortProjectorTest, ActivatesStrictlyPositivePhaseAndTangentBounds) {
  PortProjectionInput input = MakeInput();
  input.base_w_dot = 0.03;
  input.base_tangent_speed = 0.02;
  input.r_w_norm = 2.0;
  input.raw.u_w = -0.3;
  PortProjectionLimits limits = MakeLimits();
  limits.phase_dot_min = 0.10;
  limits.tangent_speed_min = 0.18;
  PortProjectionResult output;

  ASSERT_TRUE(PortProjector::project(input, limits, output));
  EXPECT_NEAR(output.final_port.u_w, 0.08, 1e-15);
  EXPECT_NEAR(output.final_w_dot, 0.11, 1e-15);
  EXPECT_NEAR(output.final_tangent_speed, 0.18, 1e-15);
  EXPECT_TRUE(output.u_w_limited);
  EXPECT_NE(output.u_w_limit_reason.find("positive_margin"), std::string::npos);
}

TEST(PortProjectorTest, SupportsNonnegativeSafetyPriorityMargins) {
  PortProjectionInput input = MakeInput();
  input.base_w_dot = 0.0;
  input.base_tangent_speed = 0.0;
  input.raw.u_w = -0.3;
  PortProjectionLimits limits = MakeLimits();
  limits.u_w_abs_max = 0.0;
  limits.phase_dot_min = 0.0;
  limits.tangent_speed_min = 0.0;
  PortProjectionResult output;

  ASSERT_TRUE(PortProjector::project(input, limits, output));
  EXPECT_DOUBLE_EQ(output.final_port.u_w, 0.0);
  EXPECT_DOUBLE_EQ(output.final_w_dot, 0.0);
  EXPECT_DOUBLE_EQ(output.final_tangent_speed, 0.0);
}

TEST(PortProjectorTest, EnforcesPhaseAndTangentAtTheirSharedPositiveBound) {
  PortProjectionInput input = MakeInput();
  input.base_w_dot = 0.03;
  input.base_tangent_speed = 0.02;
  input.r_w_norm = 2.0;
  input.raw.u_w = -0.3;
  PortProjectionLimits limits = MakeLimits();
  limits.phase_dot_min = 0.12;
  limits.tangent_speed_min = 0.20;
  PortProjectionResult output;

  ASSERT_TRUE(PortProjector::project(input, limits, output));
  EXPECT_NEAR(output.final_port.u_w, 0.09, 1e-15);
  EXPECT_NEAR(output.final_w_dot, limits.phase_dot_min, 1e-15);
  EXPECT_NEAR(output.final_tangent_speed, limits.tangent_speed_min, 1e-15);
  EXPECT_TRUE(output.u_w_limited);
  EXPECT_NE(output.u_w_limit_reason.find("positive_margin"), std::string::npos);
}

TEST(PortProjectorTest, EnforcesPositiveCurvatureNextStepRegularity) {
  PortProjectionInput input = MakeInput();
  input.delta = 0.85;
  input.curvature = 1.0;
  input.raw.u_delta = 0.8;
  PortProjectionLimits limits = MakeLimits();
  limits.u_delta_abs_max = 1.0;
  PortProjectionResult output;

  ASSERT_TRUE(PortProjector::project(input, limits, output));
  EXPECT_NEAR(output.final_port.u_delta, 0.5, 1e-15);
  EXPECT_NEAR(output.next_delta, 0.9, 1e-15);
  EXPECT_NEAR(output.next_regularity, 0.1, 1e-15);
  EXPECT_TRUE(output.u_delta_limited);
  EXPECT_NE(output.u_delta_limit_reason.find("regularity"), std::string::npos);
}

TEST(PortProjectorTest, EnforcesNegativeCurvatureAndLeavesZeroCurvatureUnchanged) {
  PortProjectionInput input = MakeInput();
  input.delta = -0.85;
  input.curvature = -1.0;
  input.raw.u_delta = -0.8;
  PortProjectionLimits limits = MakeLimits();
  limits.u_delta_abs_max = 1.0;
  PortProjectionResult output;

  ASSERT_TRUE(PortProjector::project(input, limits, output));
  EXPECT_NEAR(output.final_port.u_delta, -0.5, 1e-15);
  EXPECT_NEAR(output.next_regularity, 0.1, 1e-15);

  input = MakeInput();
  input.raw.u_delta = 0.42;
  ASSERT_TRUE(PortProjector::project(input, MakeLimits(), output));
  EXPECT_NEAR(output.final_port.u_delta, 0.42, 1e-15);
  EXPECT_NEAR(output.next_regularity, 1.0, 1e-15);
}

TEST(PortProjectorTest, EmptyFeasibleIntervalsAndInvalidInputsFailSafely) {
  PortProjectionInput input = MakeInput();
  PortProjectionLimits limits = MakeLimits();
  limits.phase_dot_min = 1.0;
  PortProjectionResult output;

  EXPECT_FALSE(PortProjector::project(input, limits, output));
  EXPECT_FALSE(output.valid);
  EXPECT_FALSE(output.invalid_reason.empty());
  ExpectFinite(output);

  input = MakeInput();
  input.dt = 0.0;
  EXPECT_FALSE(PortProjector::project(input, MakeLimits(), output));
  ExpectFinite(output);

  input = MakeInput();
  input.raw.u_delta = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(PortProjector::project(input, MakeLimits(), output));
  ExpectFinite(output);

  input = MakeInput();
  input.raw.u_w = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(PortProjector::project(input, MakeLimits(), output));
  ExpectFinite(output);

  limits = MakeLimits();
  limits.u_delta_abs_max = -1.0;
  EXPECT_FALSE(PortProjector::project(MakeInput(), limits, output));
  ExpectFinite(output);
}

TEST(PortProjectorTest, DisabledOffsetConstraintIsExactlyA4Compatible) {
  PortProjectionInput baseline = MakeInput();
  baseline.raw.u_w = 0.31;
  baseline.raw.u_delta = -0.41;
  baseline.previous_final.u_w = 0.08;
  baseline.previous_final.u_delta = -0.06;
  PortProjectionLimits limits = MakeLimits();
  limits.u_w_rate_max = 1.0;
  limits.u_delta_rate_max = 1.0;

  PortProjectionResult default_result;
  ASSERT_TRUE(PortProjector::project(baseline, limits, default_result));
  baseline.offset_constraint.enabled = false;
  baseline.offset_constraint.valid = false;
  PortProjectionResult disabled_result;
  ASSERT_TRUE(PortProjector::project(baseline, limits, disabled_result));
  EXPECT_DOUBLE_EQ(default_result.final_port.u_w, disabled_result.final_port.u_w);
  EXPECT_DOUBLE_EQ(default_result.final_port.u_delta,
                   disabled_result.final_port.u_delta);
  EXPECT_EQ(default_result.u_w_limit_reason, disabled_result.u_w_limit_reason);
  EXPECT_EQ(default_result.u_delta_limit_reason,
            disabled_result.u_delta_limit_reason);
}

OffsetConstraint MakeOffsetConstraint() {
  OffsetConstraint constraint;
  constraint.lower = -0.5;
  constraint.upper = 0.5;
  constraint.invariant_gain = 1.0;
  constraint.interior_margin = 0.0;
  constraint.enabled = true;
  constraint.valid = true;
  return constraint;
}

PortProjectionPwlCell MakePwlCell(const double w0, const double w1,
                                  const double lower_at_w0,
                                  const double lower_w,
                                  const double upper_at_w0,
                                  const double upper_w) {
  PortProjectionPwlCell cell;
  cell.w0 = w0;
  cell.w1 = w1;
  cell.lower_at_w0 = lower_at_w0;
  cell.lower_w = lower_w;
  cell.upper_at_w0 = upper_at_w0;
  cell.upper_w = upper_w;
  return cell;
}

PortProjectionInput MakePwlInput() {
  PortProjectionInput input = MakeInput();
  input.dt = 1.0;
  input.phase = 0.0;
  input.base_w_dot = 0.8;
  input.base_tangent_speed = 0.8;
  input.offset_constraint = MakeOffsetConstraint();
  input.offset_constraint.lower = -1.0;
  input.offset_constraint.upper = 0.5;
  input.offset_constraint.lower_w = 0.0;
  input.offset_constraint.upper_w = 0.0;
  input.offset_constraint.invariant_gain = 10.0;
  return input;
}

PortProjectionLimits MakePwlLimits() {
  PortProjectionLimits limits = MakeLimits();
  limits.phase_dot_min = 0.8;
  limits.tangent_speed_min = 0.1;
  return limits;
}

TEST(PortProjectorTest, ConstantTubeBoundsConstrainUpperAndLowerSteps) {
  PortProjectionInput input = MakeInput();
  input.delta = 0.48;
  input.raw.u_delta = 0.4;
  input.offset_constraint = MakeOffsetConstraint();
  PortProjectionResult output;
  ASSERT_TRUE(PortProjector::project(input, MakeLimits(), output));
  EXPECT_LE(output.next_delta, 0.5 + 1e-12);
  EXPECT_TRUE(output.u_delta_limited);
  EXPECT_GE(output.upper_invariant_residual, -1e-12);

  input = MakeInput();
  input.delta = -0.48;
  input.raw.u_delta = -0.4;
  input.offset_constraint = MakeOffsetConstraint();
  ASSERT_TRUE(PortProjector::project(input, MakeLimits(), output));
  EXPECT_GE(output.next_delta, -0.5 - 1e-12);
  EXPECT_TRUE(output.u_delta_limited);
  EXPECT_GE(output.lower_invariant_residual, -1e-12);
}

TEST(PortProjectorTest, ExactNextUpperPlaneProjectsToTheClosestFeasiblePort) {
  PortProjectionInput input = MakeInput();
  input.delta = 0.4;
  input.raw.u_w = 0.4;
  input.raw.u_delta = 0.4;
  input.offset_constraint = MakeOffsetConstraint();
  input.offset_constraint.upper_w = -1.0;
  input.offset_constraint.invariant_gain = 20.0;
  PortProjectionResult output;

  ASSERT_TRUE(PortProjector::project(input, MakeLimits(), output));
  EXPECT_NEAR(output.final_port.u_w, 0.25, 1e-12);
  EXPECT_NEAR(output.final_port.u_delta, 0.25, 1e-12);
  const double next_upper = input.offset_constraint.upper +
      input.offset_constraint.upper_w * input.dt * output.final_w_dot -
      input.offset_constraint.interior_margin;
  const double exact_residual = input.offset_constraint.upper_w *
      output.final_w_dot - output.final_port.u_delta +
      (input.offset_constraint.upper - input.offset_constraint.interior_margin -
       input.delta) / input.dt;
  EXPECT_NEAR(output.next_delta, next_upper, 1e-12);
  EXPECT_NEAR(exact_residual, 0.0, 1e-12);
  EXPECT_TRUE(output.offset_constraint_limited);
}

TEST(PortProjectorTest, ExactNextLowerPlaneProjectsToTheClosestFeasiblePort) {
  PortProjectionInput input = MakeInput();
  input.delta = -0.4;
  input.raw.u_w = 0.4;
  input.raw.u_delta = -0.4;
  input.offset_constraint = MakeOffsetConstraint();
  input.offset_constraint.lower_w = 1.0;
  input.offset_constraint.invariant_gain = 20.0;
  PortProjectionResult output;

  ASSERT_TRUE(PortProjector::project(input, MakeLimits(), output));
  EXPECT_NEAR(output.final_port.u_w, 0.25, 1e-12);
  EXPECT_NEAR(output.final_port.u_delta, -0.25, 1e-12);
  const double next_lower = input.offset_constraint.lower +
      input.offset_constraint.lower_w * input.dt * output.final_w_dot +
      input.offset_constraint.interior_margin;
  const double exact_residual = output.final_port.u_delta -
      input.offset_constraint.lower_w * output.final_w_dot +
      (input.delta - input.offset_constraint.lower -
       input.offset_constraint.interior_margin) / input.dt;
  EXPECT_NEAR(output.next_delta, next_lower, 1e-12);
  EXPECT_NEAR(exact_residual, 0.0, 1e-12);
  EXPECT_TRUE(output.offset_constraint_limited);
}

TEST(PortProjectorTest, ExactNextWideningDoesNotKeepCurrentStepGate) {
  PortProjectionInput input = MakeInput();
  input.delta = 0.48;
  input.raw.u_w = 0.4;
  input.raw.u_delta = 0.4;
  input.offset_constraint = MakeOffsetConstraint();
  input.offset_constraint.upper_w = 2.0;
  input.offset_constraint.invariant_gain = 20.0;
  PortProjectionResult output;

  ASSERT_TRUE(PortProjector::project(input, MakeLimits(), output));
  EXPECT_NEAR(output.final_port.u_w, input.raw.u_w, 1e-12);
  EXPECT_NEAR(output.final_port.u_delta, input.raw.u_delta, 1e-12);
  const double next_upper = input.offset_constraint.upper +
      input.offset_constraint.upper_w * input.dt * output.final_w_dot;
  EXPECT_LE(output.next_delta, next_upper + 1e-12);
}

TEST(PortProjectorTest, SlopedUpperUsesJointProjection) {
  PortProjectionInput input = MakeInput();
  input.base_w_dot = 0.2;
  input.base_tangent_speed = 0.2;
  input.delta = 0.3;
  input.raw.u_w = 0.0;
  input.raw.u_delta = 0.4;
  input.offset_constraint = MakeOffsetConstraint();
  input.offset_constraint.upper_w = 1.0;
  input.offset_constraint.invariant_gain = 0.0;
  PortProjectionResult output;
  ASSERT_TRUE(PortProjector::project(input, MakeLimits(), output));
  EXPECT_GT(output.final_port.u_w, input.raw.u_w);
  EXPECT_LT(output.final_port.u_delta, input.raw.u_delta);
  EXPECT_TRUE(output.offset_constraint_limited);
  EXPECT_GE(output.upper_invariant_residual, -1e-12);
}

TEST(PortProjectorTest, SlopedLowerUsesJointProjection) {
  PortProjectionInput input = MakeInput();
  input.base_w_dot = 0.2;
  input.base_tangent_speed = 0.2;
  input.delta = -0.3;
  input.raw.u_w = 0.4;
  input.raw.u_delta = 0.0;
  input.offset_constraint = MakeOffsetConstraint();
  input.offset_constraint.lower_w = 1.0;
  input.offset_constraint.upper_w = 1.0;
  input.offset_constraint.invariant_gain = 0.0;
  PortProjectionResult output;
  ASSERT_TRUE(PortProjector::project(input, MakeLimits(), output));
  EXPECT_LT(output.final_port.u_w, input.raw.u_w);
  EXPECT_GT(output.final_port.u_delta, input.raw.u_delta);
  EXPECT_TRUE(output.offset_constraint_limited);
  EXPECT_GE(output.lower_invariant_residual, -1e-12);
}

TEST(PortProjectorTest, JointProjectionIsFeasibleAndDeterministic) {
  PortProjectionInput input = MakeInput();
  input.delta = 0.2;
  input.raw.u_w = 0.4;
  input.raw.u_delta = 0.5;
  input.offset_constraint = MakeOffsetConstraint();
  input.offset_constraint.upper_w = 0.3;
  input.offset_constraint.lower_w = -0.2;
  input.offset_constraint.interior_margin = 0.05;
  PortProjectionResult first;
  PortProjectionResult second;
  ASSERT_TRUE(PortProjector::project(input, MakeLimits(), first));
  ASSERT_TRUE(PortProjector::project(input, MakeLimits(), second));
  EXPECT_DOUBLE_EQ(first.final_port.u_w, second.final_port.u_w);
  EXPECT_DOUBLE_EQ(first.final_port.u_delta, second.final_port.u_delta);
  EXPECT_GE(first.final_w_dot, MakeLimits().phase_dot_min - 1e-12);
  EXPECT_GE(first.final_tangent_speed, MakeLimits().tangent_speed_min - 1e-12);
  EXPECT_GE(first.next_regularity, MakeLimits().regularity_margin - 1e-12);
  EXPECT_GE(first.upper_invariant_residual, -1e-12);
  EXPECT_GE(first.lower_invariant_residual, -1e-12);
}

TEST(PortProjectorTest, InvalidOffsetConstraintFailsWithoutRelaxation) {
  PortProjectionInput input = MakeInput();
  input.offset_constraint = MakeOffsetConstraint();
  input.offset_constraint.upper = std::numeric_limits<double>::infinity();
  PortProjectionResult output;
  EXPECT_FALSE(PortProjector::project(input, MakeLimits(), output));
  EXPECT_FALSE(output.valid);
  ExpectFinite(output);

  input = MakeInput();
  input.offset_constraint = MakeOffsetConstraint();
  input.offset_constraint.lower = 0.2;
  input.offset_constraint.upper = 0.3;
  input.delta = 0.0;
  EXPECT_FALSE(PortProjector::project(input, MakeLimits(), output));
  EXPECT_FALSE(output.valid);
}

TEST(PortProjectorTest, ExactPwlTerminalUsesTrueCellRatherThanCurrentTangent) {
  PortProjectionInput input = MakePwlInput();
  input.delta = 0.4;
  input.raw = PortCommand();
  input.offset_pwl_cells = {
      MakePwlCell(0.0, 0.5, -1.0, 0.0, 0.5, 0.0),
      MakePwlCell(0.5, 1.0, -1.0, 0.0, 0.5, -1.0)};
  PortProjectionResult output;
  ASSERT_TRUE(PortProjector::project(input, MakePwlLimits(), output));
  EXPECT_NEAR(output.final_port.u_w, 0.0, 1e-12);
  // At w+=0.8 the true upper boundary is 0.2.  The old current-tangent
  // extrapolation saw 0.5 and accepted q=0; F2 projects to q=-0.2.
  EXPECT_NEAR(output.final_port.u_delta, -0.2, 1e-12);
  EXPECT_NEAR(output.next_delta, 0.2, 1e-12);
}

TEST(PortProjectorTest, ExactPwlChecksEveryCrossedKnot) {
  PortProjectionInput input = MakePwlInput();
  input.delta = 0.5;
  input.raw = PortCommand();
  input.offset_constraint.upper = 0.6;
  input.offset_pwl_cells = {
      MakePwlCell(0.0, 0.25, -1.0, 0.0, 0.6, -2.0),
      MakePwlCell(0.25, 0.5, -1.0, 0.0, 0.1, 2.0),
      MakePwlCell(0.5, 1.0, -1.0, 0.0, 0.6, 0.0)};
  PortProjectionResult output;
  // The terminal cell is wide, but the narrow 0.25-knot cannot be reached
  // under the lateral limit.  It must not be skipped as a remote heuristic.
  EXPECT_FALSE(PortProjector::project(input, MakePwlLimits(), output));
  EXPECT_FALSE(output.valid);
}

TEST(PortProjectorTest, ExactPwlAtInteriorKnotUsesForwardCell) {
  PortProjectionInput input = MakePwlInput();
  input.phase = 0.5;
  input.base_w_dot = 0.1;
  input.base_tangent_speed = 0.1;
  input.delta = 0.45;
  input.raw = PortCommand();
  input.offset_constraint.upper = 0.5;
  input.offset_pwl_cells = {
      MakePwlCell(0.0, 0.5, -1.0, 0.0, 0.5, 0.0),
      MakePwlCell(0.5, 1.0, -1.0, 0.0, 0.5, -1.0)};
  PortProjectionLimits limits = MakePwlLimits();
  limits.phase_dot_min = 0.1;
  PortProjectionResult output;
  ASSERT_TRUE(PortProjector::project(input, limits, output));
  EXPECT_NEAR(output.final_port.u_delta, -0.05, 1e-12);
  EXPECT_NEAR(output.next_delta, 0.4, 1e-12);
}

TEST(PortProjectorTest, InvalidOrDiscontinuousPwlCellsFailWithoutFallback) {
  PortProjectionInput input = MakePwlInput();
  input.offset_pwl_cells = {
      MakePwlCell(0.0, 0.5, -1.0, 0.0, 0.5, 0.0),
      // Discontinuous lower boundary at the shared knot.
      MakePwlCell(0.5, 1.0, -0.5, 0.0, 0.5, 0.0)};
  PortProjectionResult output;
  EXPECT_FALSE(PortProjector::project(input, MakePwlLimits(), output));
  EXPECT_FALSE(output.valid);
}

TEST(PortProjectorTest, ExactPwlAllowsZeroProgressAtCurrentContainment) {
  PortProjectionInput input = MakePwlInput();
  input.base_w_dot = 0.0;
  input.base_tangent_speed = 0.0;
  input.delta = 0.4;
  input.raw = PortCommand();
  input.offset_pwl_cells = {
      MakePwlCell(0.0, 0.5, -1.0, 0.0, 0.5, -1.0),
      MakePwlCell(0.5, 1.0, -1.0, 0.0, 0.0, 0.0)};
  PortProjectionLimits limits = MakePwlLimits();
  limits.phase_dot_min = 0.0;
  limits.tangent_speed_min = 0.0;
  PortProjectionResult output;
  ASSERT_TRUE(PortProjector::project(input, limits, output));
  EXPECT_DOUBLE_EQ(output.final_w_dot, 0.0);
  EXPECT_DOUBLE_EQ(output.next_delta, 0.4);
}

TEST(PortProjectorTest,
     ExactPwlWitnessSurvivesContradictoryContinuousInvariantDiagnostic) {
  PortProjectionInput input = MakePwlInput();
  input.delta = 0.0;
  input.raw = PortCommand();
  input.offset_constraint.lower = -1.0;
  input.offset_constraint.upper = 1.0;
  input.offset_constraint.lower_w = 100.0;
  input.offset_constraint.upper_w = 0.0;
  input.offset_constraint.invariant_gain = 1.0;
  input.offset_pwl_cells = {
      MakePwlCell(0.0, 0.5, -1.0, 0.0, 1.0, 0.0),
      MakePwlCell(0.5, 1.0, -1.0, 0.0, 1.0, 0.0)};
  PortProjectionResult output;
  ASSERT_TRUE(PortProjector::project(input, MakePwlLimits(), output));
  EXPECT_TRUE(output.valid);
  EXPECT_LT(output.lower_invariant_residual, 0.0);
  EXPECT_GE(output.next_delta, -1.0 - 1e-12);
  EXPECT_LE(output.next_delta, 1.0 + 1e-12);
}

}  // namespace
}  // namespace phase_offset_core

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
