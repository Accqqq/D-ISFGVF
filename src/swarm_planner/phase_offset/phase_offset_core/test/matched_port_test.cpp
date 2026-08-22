#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "phase_offset_core/geometry.h"
#include "phase_offset_core/matched_port.h"

namespace phase_offset_core {
namespace {

void ExpectVectorNear(const Eigen::Vector3d& actual,
                      const Eigen::Vector3d& expected,
                      const double tolerance) {
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

void ExpectFinite(const MatchedPortOutput& output) {
  EXPECT_TRUE(output.physical_port.allFinite());
  EXPECT_TRUE(output.v_cmd.allFinite());
  EXPECT_TRUE(std::isfinite(output.w_dot));
  EXPECT_TRUE(std::isfinite(output.delta_dot));
  EXPECT_TRUE(output.matched_residual.allFinite());
  EXPECT_TRUE(std::isfinite(output.matched_residual_norm));
}

PhaseOffsetGeometryState Evaluate(const PathDifferentialState& path,
                                  const Eigen::Vector3d& position,
                                  const double delta) {
  GeometryEvaluator evaluator;
  PhaseOffsetGeometryState geometry;
  EXPECT_TRUE(evaluator.evaluate(path, position, delta, geometry));
  return geometry;
}

PathDifferentialState MakeLine(const Eigen::Vector3d& derivative) {
  PathDifferentialState path;
  path.p = Eigen::Vector3d(1.0, -2.0, 0.8);
  path.p_w = derivative;
  path.p_ww = Eigen::Vector3d::Zero();
  path.w = 0.4;
  path.valid = true;
  return path;
}

PathDifferentialState MakeCircle(const double radius,
                                 const double w,
                                 const double vertical_rate) {
  PathDifferentialState path;
  path.p = Eigen::Vector3d(radius * std::cos(w), radius * std::sin(w),
                            1.0 + vertical_rate * w);
  path.p_w = Eigen::Vector3d(-radius * std::sin(w), radius * std::cos(w),
                              vertical_rate);
  path.p_ww = Eigen::Vector3d(-radius * std::cos(w), -radius * std::sin(w),
                               0.0);
  path.w = w;
  path.valid = true;
  return path;
}

MatchedPortInput MakeInput(const PhaseOffsetGeometryState& geometry,
                           const Eigen::Vector3d& base_v,
                           const double base_w_dot,
                           const double u_w,
                           const double u_delta) {
  MatchedPortInput input;
  input.geometry = geometry;
  input.base_v_cmd = base_v;
  input.base_w_dot = base_w_dot;
  input.final_port.u_w = u_w;
  input.final_port.u_delta = u_delta;
  return input;
}

TEST(MatchedPortTest, ZeroPortStrictlyRecoversBaseGuidance) {
  const PhaseOffsetGeometryState geometry =
      Evaluate(MakeLine(Eigen::Vector3d(3.0, 4.0, 0.0)),
               Eigen::Vector3d::Zero(), 0.0);
  const Eigen::Vector3d base_v(0.8, -0.2, 0.4);
  MatchedPortOutput output;

  ASSERT_TRUE(MatchedPort::evaluate(
      MakeInput(geometry, base_v, 0.72, 0.0, 0.0), output));
  ExpectVectorNear(output.physical_port, Eigen::Vector3d::Zero(), 0.0);
  ExpectVectorNear(output.v_cmd, base_v, 0.0);
  EXPECT_DOUBLE_EQ(output.w_dot, 0.72);
  EXPECT_DOUBLE_EQ(output.delta_dot, 0.0);
  EXPECT_LE(output.matched_residual_norm, 1e-15);
  ExpectFinite(output);
}

TEST(MatchedPortTest, FixedHeightLineUsesOneFinalPortInAllChannels) {
  const PhaseOffsetGeometryState geometry =
      Evaluate(MakeLine(Eigen::Vector3d(3.0, 4.0, 0.0)),
               Eigen::Vector3d(1.0, 1.0, 0.8), 0.25);
  const MatchedPortInput input = MakeInput(
      geometry, Eigen::Vector3d(0.4, 0.6, -0.1), 0.7, 0.2, -0.15);
  MatchedPortOutput output;

  ASSERT_TRUE(MatchedPort::evaluate(input, output));
  const Eigen::Vector3d expected_port =
      geometry.r_w * input.final_port.u_w + geometry.N * input.final_port.u_delta;
  ExpectVectorNear(output.physical_port, expected_port, 1e-15);
  ExpectVectorNear(output.v_cmd, input.base_v_cmd + expected_port, 1e-15);
  EXPECT_NEAR(output.w_dot, input.base_w_dot + input.final_port.u_w, 1e-15);
  EXPECT_NEAR(output.delta_dot, input.final_port.u_delta, 1e-15);
  EXPECT_LE(output.matched_residual_norm, 1e-14);
}

TEST(MatchedPortTest, CurvedOffsetHasMachinePrecisionCompleteResidual) {
  const PhaseOffsetGeometryState geometry =
      Evaluate(MakeCircle(5.0, 0.7, 0.0), Eigen::Vector3d(4.2, 1.0, 1.0), 0.35);
  const MatchedPortInput input = MakeInput(
      geometry, Eigen::Vector3d(-0.7, 1.2, 0.3), 0.45, -0.18, 0.21);
  MatchedPortOutput output;

  ASSERT_TRUE(MatchedPort::evaluate(input, output));
  EXPECT_LE(output.matched_residual.norm(), 1e-14);
  EXPECT_LE(output.matched_residual_norm, 1e-14);
  EXPECT_NEAR(geometry.T.dot(output.physical_port),
              geometry.r_w.norm() * input.final_port.u_w, 1e-12);
}

TEST(MatchedPortTest, HeightVaryingGeometryPreservesMatchedCancellation) {
  const PathDifferentialState path = MakeCircle(3.0, 1.1, 0.8);
  const PhaseOffsetGeometryState geometry =
      Evaluate(path, Eigen::Vector3d(0.5, -1.0, 2.5), -0.22);
  const MatchedPortInput input = MakeInput(
      geometry, Eigen::Vector3d(0.6, -0.3, 0.5), 0.9, 0.08, -0.12);
  MatchedPortOutput output;

  ASSERT_TRUE(MatchedPort::evaluate(input, output));
  EXPECT_GT(std::abs(geometry.T.z()), 1e-3);
  EXPECT_NEAR(geometry.r.z(), path.p.z(), 1e-12);
  EXPECT_LE(output.matched_residual_norm, 1e-14);
  ExpectFinite(output);
}

TEST(MatchedPortTest, PositiveAndNegativePortsUseCompleteDynamics) {
  const PhaseOffsetGeometryState geometry =
      Evaluate(MakeCircle(4.0, 0.3, 0.35), Eigen::Vector3d::Zero(), 0.2);
  for (const double u_w : {-0.25, -0.05, 0.0, 0.11, 0.31}) {
    for (const double u_delta : {-0.3, -0.02, 0.0, 0.17, 0.29}) {
      MatchedPortOutput output;
      ASSERT_TRUE(MatchedPort::evaluate(
          MakeInput(geometry, Eigen::Vector3d(0.4, -0.8, 0.2), 0.6,
                    u_w, u_delta), output));
      EXPECT_LE(output.matched_residual_norm, 1e-14);
      EXPECT_NEAR(output.delta_dot, u_delta, 1e-15);
      EXPECT_NEAR(output.w_dot, 0.6 + u_w, 1e-15);
    }
  }
}

TEST(MatchedPortTest, InvalidInputsFailWithFiniteOutputs) {
  MatchedPortOutput output;
  MatchedPortInput invalid_geometry;
  invalid_geometry.geometry.valid = false;
  EXPECT_FALSE(MatchedPort::evaluate(invalid_geometry, output));
  EXPECT_FALSE(output.valid);
  EXPECT_FALSE(output.invalid_reason.empty());
  ExpectFinite(output);

  const PhaseOffsetGeometryState geometry =
      Evaluate(MakeLine(Eigen::Vector3d::UnitX()), Eigen::Vector3d::Zero(), 0.0);
  MatchedPortInput invalid_base =
      MakeInput(geometry, Eigen::Vector3d::Zero(), 0.4, 0.1, 0.2);
  invalid_base.base_v_cmd.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(MatchedPort::evaluate(invalid_base, output));
  EXPECT_FALSE(output.valid);
  EXPECT_FALSE(output.invalid_reason.empty());
  ExpectFinite(output);

  MatchedPortInput invalid_port =
      MakeInput(geometry, Eigen::Vector3d::Zero(), 0.4, 0.1, 0.2);
  invalid_port.final_port.u_delta = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(MatchedPort::evaluate(invalid_port, output));
  EXPECT_FALSE(output.valid);
  EXPECT_FALSE(output.invalid_reason.empty());
  ExpectFinite(output);
}

}  // namespace
}  // namespace phase_offset_core

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
