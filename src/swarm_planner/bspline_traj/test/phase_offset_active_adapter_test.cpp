#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "bspline_race/integration/phase_offset_active_adapter.h"

namespace FLAG_Race {
namespace {

void ExpectVectorNear(const Eigen::Vector3d& actual,
                      const Eigen::Vector3d& expected,
                      const double tolerance) {
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

void ExpectFinite(const ActiveAdapterOutput& output) {
  EXPECT_TRUE(output.guidance.v_cmd.allFinite());
  EXPECT_TRUE(std::isfinite(output.guidance.w_dot));
  EXPECT_TRUE(std::isfinite(output.guidance.e_parallel));
  EXPECT_TRUE(output.guidance.e_perp.allFinite());
  EXPECT_TRUE(output.guidance.ref_pt.allFinite());
  EXPECT_TRUE(output.guidance.tangent.allFinite());
  EXPECT_TRUE(std::isfinite(output.zero_port.r_minus_p_norm));
  EXPECT_TRUE(std::isfinite(output.zero_port.r_w_minus_p_w_norm));
  EXPECT_TRUE(std::isfinite(output.zero_port.tangent_residual));
}

guidance::IsfGains MakeGains() {
  guidance::IsfGains gains;
  gains.k1 = 2.0;
  gains.k2 = -2.2;
  gains.convergence_bandwidth = 0.1;
  gains.progress_rho0 = 0.5;
  gains.progress_delta = 0.3;
  gains.alpha_min = 0.05;
  return gains;
}

ActiveAdapterInput MakeInput(const phase_offset_core::PathDifferentialState& path,
                             const Eigen::Vector3d& position) {
  ActiveAdapterInput input;
  input.path = path;
  input.position = position;
  input.gains = MakeGains();
  return input;
}

TEST(PhaseOffsetActiveAdapterTest, ConvertsContinuousStateFieldByField) {
  ContinuousPhasePathState source;
  source.p = Eigen::Vector3d(1.0, 2.0, 3.0);
  source.dp_dw = Eigen::Vector3d(4.0, 5.0, 6.0);
  source.d2p_dw2 = Eigen::Vector3d(7.0, 8.0, 9.0);
  source.vel = Eigen::Vector3d(10.0, 11.0, 12.0);
  source.valid = true;

  const phase_offset_core::PathDifferentialState converted =
      ConvertContinuousPhasePathStateForActive(source, 12.5);
  ExpectVectorNear(converted.p, source.p, 1e-12);
  ExpectVectorNear(converted.p_w, source.dp_dw, 1e-12);
  ExpectVectorNear(converted.p_ww, source.d2p_dw2, 1e-12);
  EXPECT_NEAR(converted.w, 12.5, 1e-12);
  EXPECT_EQ(converted.valid, source.valid);
}

TEST(PhaseOffsetActiveAdapterTest, SlopedLinePreservesZeroPortThreeDimensionalGeometry) {
  phase_offset_core::PathDifferentialState path;
  path.p = Eigen::Vector3d(1.0, -2.0, 1.5);
  path.p_w = Eigen::Vector3d(2.0, 0.0, 1.0);
  path.p_ww = Eigen::Vector3d::Zero();
  path.w = 1.0;
  path.valid = true;
  const PhaseOffsetActiveAdapter adapter;
  ActiveAdapterOutput output;

  ASSERT_TRUE(adapter.evaluate(MakeInput(path, path.p + Eigen::Vector3d(0.0, 0.4, 0.0)),
                               output));
  ExpectVectorNear(output.geometry.r, path.p, 1e-12);
  ExpectVectorNear(output.geometry.r_w, path.p_w, 1e-12);
  ExpectVectorNear(output.geometry.T, path.p_w / std::sqrt(5.0), 1e-12);
  EXPECT_NEAR(output.geometry.T.z(), 1.0 / std::sqrt(5.0), 1e-12);
  EXPECT_NEAR(output.zero_port.r_minus_p_norm, 0.0, 1e-12);
  EXPECT_NEAR(output.zero_port.r_w_minus_p_w_norm, 0.0, 1e-12);
  EXPECT_NEAR(output.zero_port.tangent_residual, 0.0, 1e-12);
}

TEST(PhaseOffsetActiveAdapterTest, HeightVaryingPathMatchesDirectKernel) {
  phase_offset_core::PathDifferentialState path;
  const double w = 0.8;
  path.p = Eigen::Vector3d(w, 0.0, 1.0 + 0.25 * w * w);
  path.p_w = Eigen::Vector3d(1.0, 0.0, 0.5 * w);
  path.p_ww = Eigen::Vector3d(0.0, 0.0, 0.5);
  path.w = w;
  path.valid = true;
  const Eigen::Vector3d position = path.p + Eigen::Vector3d(0.1, -0.3, 0.2);
  const ActiveAdapterInput input = MakeInput(path, position);
  const PhaseOffsetActiveAdapter adapter;
  ActiveAdapterOutput output;

  ASSERT_TRUE(adapter.evaluate(input, output));
  guidance::ReferenceGeometry reference;
  reference.point = output.geometry.r;
  reference.tangent = output.geometry.T;
  reference.derivative_norm = output.geometry.r_w.norm();
  reference.valid = true;
  guidance::IsfGuidance direct;
  ASSERT_TRUE(guidance::IsfReferenceKernel::evaluate(
      position, reference, input.gains, direct));
  ExpectVectorNear(output.guidance.v_cmd, direct.v_cmd, 1e-12);
  EXPECT_NEAR(output.guidance.w_dot, direct.w_dot, 1e-12);
  EXPECT_NEAR(output.guidance.e_parallel, direct.e_parallel, 1e-12);
  ExpectVectorNear(output.guidance.e_perp, direct.e_perp, 1e-12);
  ExpectVectorNear(output.guidance.ref_pt, direct.ref_pt, 1e-12);
  ExpectVectorNear(output.guidance.tangent, direct.tangent, 1e-12);
  EXPECT_NEAR(output.geometry.r.z(), path.p.z(), 1e-12);
}

TEST(PhaseOffsetActiveAdapterTest, MatchesHardCodedLegacyRepresentativeAtMachinePrecision) {
  phase_offset_core::PathDifferentialState path;
  path.p = Eigen::Vector3d(1.0, 2.0, 3.0);
  path.p_w = Eigen::Vector3d(2.5, 0.0, 0.0);
  path.p_ww = Eigen::Vector3d::Zero();
  path.w = 4.0;
  path.valid = true;
  const PhaseOffsetActiveAdapter adapter;
  ActiveAdapterOutput active;
  ASSERT_TRUE(adapter.evaluate(MakeInput(path, Eigen::Vector3d(1.0, 2.2, 3.0)), active));

  LegacyGuidanceSnapshot legacy;
  legacy.v_cmd = Eigen::Vector3d(1.7379310344827585, -2.1208606761667970, 0.0);
  legacy.w_dot = 0.6951724137931035;
  legacy.e_parallel = 0.0;
  legacy.e_perp = Eigen::Vector3d(0.0, 0.2, 0.0);
  legacy.ref_pt = path.p;
  legacy.tangent = Eigen::Vector3d::UnitX();
  legacy.valid = true;
  ActiveEquivalenceResult comparison;

  ASSERT_TRUE(PhaseOffsetActiveAdapter::compareWithLegacy(
      active, legacy, 1e-12, comparison));
  EXPECT_TRUE(comparison.valid);
  EXPECT_TRUE(comparison.equivalent);
  EXPECT_NEAR(comparison.v_cmd_residual, 0.0, 1e-12);
  EXPECT_NEAR(comparison.w_dot_residual, 0.0, 1e-12);
  EXPECT_NEAR(comparison.e_parallel_residual, 0.0, 1e-12);
  EXPECT_NEAR(comparison.e_perp_residual, 0.0, 1e-12);
  EXPECT_NEAR(comparison.reference_residual, 0.0, 1e-12);
  EXPECT_NEAR(comparison.tangent_residual, 0.0, 1e-12);
}

TEST(PhaseOffsetActiveAdapterTest, InvalidPathAndGainsFailWithoutNonFiniteOutput) {
  const PhaseOffsetActiveAdapter adapter;
  phase_offset_core::PathDifferentialState vertical_path;
  vertical_path.p = Eigen::Vector3d(0.0, 0.0, 1.0);
  vertical_path.p_w = Eigen::Vector3d(0.0, 0.0, 1.0);
  vertical_path.p_ww = Eigen::Vector3d::Zero();
  vertical_path.valid = true;
  ActiveAdapterOutput output;
  EXPECT_FALSE(adapter.evaluate(MakeInput(vertical_path, Eigen::Vector3d::Zero()), output));
  EXPECT_FALSE(output.valid);
  EXPECT_FALSE(output.invalid_reason.empty());
  ExpectFinite(output);

  phase_offset_core::PathDifferentialState valid_path;
  valid_path.p = Eigen::Vector3d::Zero();
  valid_path.p_w = Eigen::Vector3d::UnitX();
  valid_path.p_ww = Eigen::Vector3d::Zero();
  valid_path.valid = true;
  ActiveAdapterInput invalid_gains = MakeInput(valid_path, Eigen::Vector3d::Zero());
  invalid_gains.gains.k1 = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(adapter.evaluate(invalid_gains, output));
  EXPECT_FALSE(output.valid);
  ExpectFinite(output);
}

TEST(PhaseOffsetActiveAdapterTest, DiagnosticsFollowRequiredFiniteLayout) {
  ActiveAdapterOutput active;
  active.zero_port.r_minus_p_norm = 1e-14;
  active.zero_port.r_w_minus_p_w_norm = 2e-14;
  active.zero_port.tangent_residual = 3e-14;
  ActiveEquivalenceResult comparison;
  comparison.v_cmd_residual = 4e-14;
  comparison.w_dot_residual = 5e-14;
  comparison.e_parallel_residual = 6e-14;
  comparison.e_perp_residual = 7e-14;
  comparison.reference_residual = 8e-14;
  comparison.tangent_residual = std::numeric_limits<double>::quiet_NaN();

  const std::array<double, 14> diagnostics =
      PhaseOffsetActiveAdapter::makeDiagnostics(
          active, comparison, true, 100, false, true);
  EXPECT_EQ(diagnostics.size(), 14U);
  EXPECT_NEAR(diagnostics[0], 1.0, 0.0);
  EXPECT_NEAR(diagnostics[1], 1.0, 0.0);
  EXPECT_NEAR(diagnostics[2], 100.0, 0.0);
  EXPECT_NEAR(diagnostics[3], 0.0, 0.0);
  EXPECT_NEAR(diagnostics[4], comparison.v_cmd_residual, 0.0);
  EXPECT_NEAR(diagnostics[5], comparison.w_dot_residual, 0.0);
  EXPECT_NEAR(diagnostics[6], comparison.e_parallel_residual, 0.0);
  EXPECT_NEAR(diagnostics[7], comparison.e_perp_residual, 0.0);
  EXPECT_NEAR(diagnostics[8], comparison.reference_residual, 0.0);
  EXPECT_NEAR(diagnostics[9], 0.0, 0.0);
  EXPECT_NEAR(diagnostics[10], active.zero_port.r_minus_p_norm, 0.0);
  EXPECT_NEAR(diagnostics[11], active.zero_port.r_w_minus_p_w_norm, 0.0);
  EXPECT_NEAR(diagnostics[12], active.zero_port.tangent_residual, 0.0);
  EXPECT_NEAR(diagnostics[13], 1.0, 0.0);
  for (const double value : diagnostics) {
    EXPECT_TRUE(std::isfinite(value));
  }
}

}  // namespace
}  // namespace FLAG_Race

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
