#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "bspline_race/guidance/isf_reference_kernel.h"

namespace FLAG_Race {
namespace guidance {
namespace {

void ExpectVectorNear(const Eigen::Vector3d& actual,
                      const Eigen::Vector3d& expected,
                      const double tolerance) {
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

void ExpectFinite(const IsfGuidance& output) {
  EXPECT_TRUE(output.v_cmd.allFinite());
  EXPECT_TRUE(std::isfinite(output.w_dot));
  EXPECT_TRUE(std::isfinite(output.e_parallel));
  EXPECT_TRUE(output.e_perp.allFinite());
  EXPECT_TRUE(output.ref_pt.allFinite());
  EXPECT_TRUE(output.tangent.allFinite());
}

ReferenceGeometry MakeReference(const Eigen::Vector3d& point,
                                const Eigen::Vector3d& tangent,
                                const double derivative_norm) {
  ReferenceGeometry reference;
  reference.point = point;
  reference.tangent = tangent;
  reference.derivative_norm = derivative_norm;
  reference.valid = true;
  return reference;
}

IsfGains MakeGains() {
  IsfGains gains;
  gains.k1 = 2.0;
  gains.k2 = -2.2;
  gains.convergence_bandwidth = 0.1;
  gains.progress_rho0 = 0.5;
  gains.progress_delta = 0.3;
  gains.alpha_min = 0.05;
  return gains;
}

TEST(IsfReferenceKernelTest, ThreeDimensionalZeroErrorMatchesGoldenGuidance) {
  const Eigen::Vector3d point(1.0, -2.0, 0.5);
  const Eigen::Vector3d tangent(3.0 / 13.0, 4.0 / 13.0, 12.0 / 13.0);
  const ReferenceGeometry reference = MakeReference(point, tangent, 2.5);
  IsfGuidance output;

  ASSERT_TRUE(IsfReferenceKernel::evaluate(point, reference, MakeGains(), output));
  ExpectVectorNear(output.v_cmd,
                   Eigen::Vector3d(0.46153846153846156,
                                   0.61538461538461542,
                                   1.8461538461538463),
                   1e-12);
  EXPECT_NEAR(output.w_dot, 0.8, 1e-12);
  EXPECT_NEAR(output.e_parallel, 0.0, 1e-12);
  ExpectVectorNear(output.e_perp, Eigen::Vector3d::Zero(), 1e-12);
  ExpectVectorNear(output.ref_pt, point, 1e-12);
  ExpectVectorNear(output.tangent, tangent, 1e-12);
  ExpectFinite(output);
}

TEST(IsfReferenceKernelTest, AlongTrackSignsMatchGoldenProgressRates) {
  const Eigen::Vector3d point(1.0, -2.0, 0.5);
  const Eigen::Vector3d tangent(3.0 / 13.0, 4.0 / 13.0, 12.0 / 13.0);
  const ReferenceGeometry reference = MakeReference(point, tangent, 2.5);
  IsfGuidance forward;
  IsfGuidance backward;

  ASSERT_TRUE(IsfReferenceKernel::evaluate(
      point + 0.3 * tangent, reference, MakeGains(), forward));
  ASSERT_TRUE(IsfReferenceKernel::evaluate(
      point - 0.3 * tangent, reference, MakeGains(), backward));
  EXPECT_NEAR(forward.e_parallel, 0.3, 1e-12);
  EXPECT_NEAR(backward.e_parallel, -0.3, 1e-12);
  EXPECT_NEAR(forward.w_dot, 1.409275324764612, 1e-12);
  EXPECT_NEAR(backward.w_dot, 0.1907246752353881, 1e-12);
  ExpectVectorNear(forward.v_cmd, backward.v_cmd, 1e-12);
}

TEST(IsfReferenceKernelTest, NormalErrorKeepsExistingNegativeK2Sign) {
  const ReferenceGeometry reference = MakeReference(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 2.5);
  IsfGuidance output;

  ASSERT_TRUE(IsfReferenceKernel::evaluate(
      Eigen::Vector3d(0.0, 0.2, 0.0), reference, MakeGains(), output));
  ExpectVectorNear(output.v_cmd,
                   Eigen::Vector3d(1.7379310344827585,
                                   -2.1208606761667970,
                                   0.0),
                   1e-12);
  EXPECT_NEAR(output.w_dot, 0.6951724137931035, 1e-12);
  EXPECT_LT(output.v_cmd.y(), 0.0);
  EXPECT_NEAR(output.e_parallel, 0.0, 1e-12);
  ExpectVectorNear(output.e_perp, Eigen::Vector3d(0.0, 0.2, 0.0), 1e-12);
}

TEST(IsfReferenceKernelTest, TinyNormalErrorUsesFiniteSmallRhoBranch) {
  const ReferenceGeometry reference = MakeReference(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 2.0);
  IsfGuidance output;

  ASSERT_TRUE(IsfReferenceKernel::evaluate(
      Eigen::Vector3d(0.0, 5e-7, 0.0), reference, MakeGains(), output));
  ExpectVectorNear(output.v_cmd,
                   Eigen::Vector3d(1.9999999999980997, -0.000011, 0.0),
                   1e-12);
  EXPECT_NEAR(output.w_dot, 0.9999999999990499, 1e-12);
  ExpectFinite(output);
}

TEST(IsfReferenceKernelTest, ExistingOneEMinusSixClampsRemainFiniteAndGolden) {
  const ReferenceGeometry reference = MakeReference(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 2.0);
  IsfGains gains = MakeGains();
  gains.convergence_bandwidth = 0.0;
  gains.progress_rho0 = 0.0;
  gains.progress_delta = 0.0;
  IsfGuidance output;

  ASSERT_TRUE(IsfReferenceKernel::evaluate(
      Eigen::Vector3d(1e-7, 1e-8, 0.0), reference, gains, output));
  ExpectVectorNear(output.v_cmd,
                   Eigen::Vector3d(1.9998100189981003, -0.022, 0.0),
                   1e-12);
  EXPECT_NEAR(output.w_dot, 1.099573004124006, 1e-12);
  ExpectFinite(output);
}

TEST(IsfReferenceKernelTest, InvalidInputsRejectWithFiniteDiagnostics) {
  const IsfGains gains = MakeGains();
  IsfGuidance output;
  ReferenceGeometry nonunit = MakeReference(
      Eigen::Vector3d::Zero(), Eigen::Vector3d(2.0, 0.0, 0.0), 1.0);
  EXPECT_FALSE(IsfReferenceKernel::evaluate(
      Eigen::Vector3d::Zero(), nonunit, gains, output));
  EXPECT_FALSE(output.valid);
  EXPECT_FALSE(output.invalid_reason.empty());
  ExpectFinite(output);

  ReferenceGeometry zero_derivative = MakeReference(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 0.0);
  EXPECT_FALSE(IsfReferenceKernel::evaluate(
      Eigen::Vector3d::Zero(), zero_derivative, gains, output));
  ExpectFinite(output);

  IsfGains nonfinite_gains = gains;
  nonfinite_gains.k2 = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(IsfReferenceKernel::evaluate(
      Eigen::Vector3d::Zero(),
      MakeReference(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 1.0),
      nonfinite_gains, output));
  ExpectFinite(output);
}

}  // namespace
}  // namespace guidance
}  // namespace FLAG_Race

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
