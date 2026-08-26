#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

#include "phase_offset_core/geometry.h"

namespace phase_offset_core {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::uint64_t Bits(const double value) {
  std::uint64_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

void ExpectVectorBits(const Eigen::Vector3d& actual,
                      const Eigen::Vector3d& expected) {
  EXPECT_EQ(Bits(actual.x()), Bits(expected.x()));
  EXPECT_EQ(Bits(actual.y()), Bits(expected.y()));
  EXPECT_EQ(Bits(actual.z()), Bits(expected.z()));
}

void ExpectVectorNear(const Eigen::Vector3d& actual,
                      const Eigen::Vector3d& expected,
                      const double tolerance) {
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

void ExpectFiniteState(const PhaseOffsetGeometryState& state) {
  EXPECT_TRUE(state.p.allFinite());
  EXPECT_TRUE(state.p_w.allFinite());
  EXPECT_TRUE(state.p_ww.allFinite());
  EXPECT_TRUE(state.T.allFinite());
  EXPECT_TRUE(state.N.allFinite());
  EXPECT_TRUE(state.N_w.allFinite());
  EXPECT_TRUE(state.r.allFinite());
  EXPECT_TRUE(state.r_w.allFinite());
  EXPECT_TRUE(state.error.allFinite());
  EXPECT_TRUE(state.e_perp.allFinite());
  EXPECT_TRUE(std::isfinite(state.w));
  EXPECT_TRUE(std::isfinite(state.delta));
  EXPECT_TRUE(std::isfinite(state.path_speed));
  EXPECT_TRUE(std::isfinite(state.horizontal_path_speed));
  EXPECT_TRUE(std::isfinite(state.curvature));
  EXPECT_TRUE(std::isfinite(state.regularity));
  EXPECT_TRUE(std::isfinite(state.e_parallel));
}

PathDifferentialState MakeLine() {
  PathDifferentialState path;
  path.p = Eigen::Vector3d(2.0, -1.0, 1.0);
  path.p_w = Eigen::Vector3d(3.0, 4.0, 0.0);
  path.p_ww = Eigen::Vector3d::Zero();
  path.w = 1.25;
  path.valid = true;
  return path;
}

PathDifferentialState MakeCircle(const double radius,
                                 const double w,
                                 const double height = 1.0) {
  PathDifferentialState path;
  path.p = Eigen::Vector3d(radius * std::cos(w), radius * std::sin(w), height);
  path.p_w = Eigen::Vector3d(-radius * std::sin(w), radius * std::cos(w), 0.0);
  path.p_ww =
      Eigen::Vector3d(-radius * std::cos(w), -radius * std::sin(w), 0.0);
  path.w = w;
  path.valid = true;
  return path;
}

PathDifferentialState MakeFigureEight(const double amplitude,
                                      const double w,
                                      const double height = 1.0) {
  PathDifferentialState path;
  path.p = Eigen::Vector3d(amplitude * std::sin(w),
                            amplitude * std::sin(w) * std::cos(w), height);
  path.p_w = Eigen::Vector3d(amplitude * std::cos(w),
                              amplitude * std::cos(2.0 * w), 0.0);
  path.p_ww = Eigen::Vector3d(-amplitude * std::sin(w),
                               -2.0 * amplitude * std::sin(2.0 * w), 0.0);
  path.w = w;
  path.valid = true;
  return path;
}

PathDifferentialState MakeEllipse(const double x_radius,
                                  const double y_radius,
                                  const double w,
                                  const double height = 1.0) {
  PathDifferentialState path;
  path.p = Eigen::Vector3d(x_radius * std::cos(w),
                            y_radius * std::sin(w), height);
  path.p_w = Eigen::Vector3d(-x_radius * std::sin(w),
                              y_radius * std::cos(w), 0.0);
  path.p_ww = Eigen::Vector3d(-x_radius * std::cos(w),
                               -y_radius * std::sin(w), 0.0);
  path.w = w;
  path.valid = true;
  return path;
}

PathDifferentialState MakeSlopedLine(const double slope, const double w) {
  PathDifferentialState path;
  path.p = Eigen::Vector3d(w, 0.0, 1.0 + slope * w);
  path.p_w = Eigen::Vector3d(1.0, 0.0, slope);
  path.p_ww = Eigen::Vector3d::Zero();
  path.w = w;
  path.valid = true;
  return path;
}

PathCellGeometryCertificate MakeConsistentCellCertificate() {
  PathCellGeometryCertificate certificate;
  certificate.w0 = 0.0;
  certificate.w1 = 1.0;
  certificate.segment_w0 = 0.0;
  certificate.segment_w1 = 1.0;
  certificate.segment_identity = 1U;
  certificate.inf_p_w_norm = 2.0;
  certificate.inf_horizontal_p_w_norm = 2.0;
  certificate.sup_p_w_norm = 2.0;
  certificate.sup_p_ww_norm = 1.0;
  certificate.sup_p_www_norm = 0.0;
  certificate.sup_horizontal_p_ww_norm = 1.0;
  certificate.horizontal_acceleration_bound_complete = true;
  certificate.sup_N_w_norm = 0.5;
  certificate.sup_abs_curvature = 0.5;
  certificate.normal_variation_bound = 0.5;
  certificate.tangent_variation_bound = 0.5;
  certificate.curvature_variation_bound = 0.0;
  certificate.midpoint_position_variation_bound = 0.0;
  certificate.chord_deviation_bound = 0.0;
  certificate.valid = true;
  certificate.complete = true;
  return certificate;
}

PathDifferentialState MakeHeightVaryingParabola(const double coefficient,
                                                 const double w) {
  PathDifferentialState path;
  path.p = Eigen::Vector3d(w, 0.0, 1.0 + coefficient * w * w);
  path.p_w = Eigen::Vector3d(1.0, 0.0, 2.0 * coefficient * w);
  path.p_ww = Eigen::Vector3d(0.0, 0.0, 2.0 * coefficient);
  path.w = w;
  path.valid = true;
  return path;
}

PathDifferentialState MakeHelix(const double radius,
                                const double vertical_rate,
                                const double w,
                                const double height = 1.0) {
  PathDifferentialState path;
  path.p = Eigen::Vector3d(radius * std::cos(w), radius * std::sin(w),
                            height + vertical_rate * w);
  path.p_w = Eigen::Vector3d(-radius * std::sin(w), radius * std::cos(w),
                              vertical_rate);
  path.p_ww = Eigen::Vector3d(-radius * std::cos(w), -radius * std::sin(w), 0.0);
  path.w = w;
  path.valid = true;
  return path;
}

PhaseOffsetGeometryState Evaluate(const GeometryEvaluator& evaluator,
                                  const PathDifferentialState& path,
                                  const Eigen::Vector3d& position,
                                  const double delta) {
  PhaseOffsetGeometryState state;
  EXPECT_TRUE(evaluator.evaluate(path, position, delta, state));
  return state;
}

void ExpectInvalid(const GeometryEvaluator& evaluator,
                   const PathDifferentialState& path,
                   const Eigen::Vector3d& position,
                   const double delta) {
  PhaseOffsetGeometryState state;
  EXPECT_FALSE(evaluator.evaluate(path, position, delta, state));
  EXPECT_FALSE(state.valid);
  EXPECT_FALSE(state.invalid_reason.empty());
  ExpectFiniteState(state);
}

void ExpectPreparedReferenceMatchesFull(
    const GeometryEvaluator& evaluator,
    const PathDifferentialState& path,
    const Eigen::Vector3d& position,
    const double delta) {
  PreparedPathGeometry prepared;
  evaluator.preparePath(path, prepared);
  PhaseOffsetGeometryState full;
  PreparedReferenceResult reference;
  const bool full_valid = evaluator.evaluate(path, position, delta, full);
  const bool reference_valid = evaluator.evaluatePreparedReference(
      prepared, position, delta, reference);
  EXPECT_EQ(full_valid, reference_valid);
  EXPECT_EQ(full.valid, reference.valid);
  EXPECT_STREQ(full.invalid_reason.c_str(), reference.invalid_reason);
  ExpectVectorBits(full.r, reference.r);
  EXPECT_EQ(Bits(full.regularity), Bits(reference.regularity));
}

TEST(GeometryEvaluatorTest, PlanarStraightLineComputesReferenceAndErrorDecomposition) {
  const GeometryEvaluator evaluator;
  const PathDifferentialState path = MakeLine();
  const Eigen::Vector3d position(4.0, 2.0, 1.5);
  const double delta = 0.5;

  const PhaseOffsetGeometryState state = Evaluate(evaluator, path, position, delta);

  const Eigen::Vector3d expected_tangent(0.6, 0.8, 0.0);
  const Eigen::Vector3d expected_normal(-0.8, 0.6, 0.0);
  ExpectVectorNear(state.T, expected_tangent, 1e-12);
  ExpectVectorNear(state.N, expected_normal, 1e-12);
  ExpectVectorNear(state.N_w, Eigen::Vector3d::Zero(), 1e-12);
  EXPECT_NEAR(state.path_speed, 5.0, 1e-12);
  EXPECT_NEAR(state.horizontal_path_speed, 5.0, 1e-12);
  EXPECT_NEAR(state.curvature, 0.0, 1e-12);
  ExpectVectorNear(state.r, path.p + expected_normal * delta, 1e-12);
  ExpectVectorNear(state.r_w, path.p_w, 1e-12);
  EXPECT_NEAR(state.T.dot(state.N), 0.0, 1e-12);
  EXPECT_NEAR(state.N.dot(state.p_w), 0.0, 1e-12);
  EXPECT_NEAR(state.N.dot(state.r_w), 0.0, 1e-12);
  EXPECT_NEAR(state.T.norm(), 1.0, 1e-12);
  EXPECT_NEAR(state.N.norm(), 1.0, 1e-12);
  ExpectVectorNear(state.e_perp + state.e_parallel * state.T, state.error, 1e-12);
  ExpectFiniteState(state);
}

TEST(GeometryEvaluatorTest, PlanarCircleRetainsOriginalFormulaAndNumericalDerivative) {
  const GeometryEvaluator evaluator;
  const double radius = 5.0;
  const double delta = 0.4;
  const double w = 0.7;
  const PathDifferentialState path = MakeCircle(radius, w);
  const PhaseOffsetGeometryState state =
      Evaluate(evaluator, path, Eigen::Vector3d::Zero(), delta);

  EXPECT_NEAR(state.curvature, 1.0 / radius, 1e-12);
  EXPECT_NEAR(state.path_speed, radius, 1e-12);
  EXPECT_NEAR(state.horizontal_path_speed, radius, 1e-12);
  EXPECT_NEAR(state.T.norm(), 1.0, 1e-12);
  EXPECT_NEAR(state.N.norm(), 1.0, 1e-12);
  EXPECT_NEAR(state.T.dot(state.N), 0.0, 1e-12);
  ExpectVectorNear(state.N_w, -state.curvature * path.p_w, 1e-12);
  ExpectVectorNear(state.r_w, (1.0 - delta / radius) * path.p_w, 1e-12);

  const PhaseOffsetGeometryState at_zero =
      Evaluate(evaluator, MakeCircle(radius, 0.0), Eigen::Vector3d::Zero(), delta);
  ExpectVectorNear(at_zero.N, Eigen::Vector3d(-1.0, 0.0, 0.0), 1e-12);
  ExpectVectorNear(at_zero.r, Eigen::Vector3d(radius - delta, 0.0, 1.0), 1e-12);

  const double h = 1e-5;
  const Eigen::Vector3d r_plus =
      Evaluate(evaluator, MakeCircle(radius, w + h), Eigen::Vector3d::Zero(), delta).r;
  const Eigen::Vector3d r_minus =
      Evaluate(evaluator, MakeCircle(radius, w - h), Eigen::Vector3d::Zero(), delta).r;
  ExpectVectorNear((r_plus - r_minus) / (2.0 * h), state.r_w, 2e-9);
}

TEST(GeometryEvaluatorTest, FigureEightKeepsPhaseLocalGeometry) {
  const GeometryEvaluator evaluator;
  const double amplitude = 3.0;
  const double delta = 0.25;
  const std::vector<double> samples = {0.25, 0.8, 1.4, 2.1, 2.7, 4.1, 5.5};

  for (const double w : samples) {
    const PhaseOffsetGeometryState state =
        Evaluate(evaluator, MakeFigureEight(amplitude, w), Eigen::Vector3d::Zero(), delta);
    EXPECT_NEAR(state.T.dot(state.N), 0.0, 1e-12);
    EXPECT_NEAR(state.N.dot(state.N_w), 0.0, 1e-12);
    EXPECT_NEAR(state.N.dot(state.r_w), 0.0, 1e-12);
    EXPECT_NEAR(state.T.norm(), 1.0, 1e-12);
    EXPECT_NEAR(state.N.norm(), 1.0, 1e-12);
    ExpectFiniteState(state);

    const double h = 1e-5;
    const Eigen::Vector3d r_plus =
        Evaluate(evaluator, MakeFigureEight(amplitude, w + h), Eigen::Vector3d::Zero(), delta)
            .r;
    const Eigen::Vector3d r_minus =
        Evaluate(evaluator, MakeFigureEight(amplitude, w - h), Eigen::Vector3d::Zero(), delta)
            .r;
    ExpectVectorNear((r_plus - r_minus) / (2.0 * h), state.r_w, 2e-8);
  }

  const PhaseOffsetGeometryState first =
      Evaluate(evaluator, MakeFigureEight(amplitude, 0.0), Eigen::Vector3d::Zero(), delta);
  const PhaseOffsetGeometryState second =
      Evaluate(evaluator, MakeFigureEight(amplitude, kPi), Eigen::Vector3d::Zero(), delta);
  ExpectVectorNear(first.p, second.p, 1e-12);
  EXPECT_NEAR(first.w, 0.0, 1e-12);
  EXPECT_NEAR(second.w, kPi, 1e-12);
  EXPECT_GT((first.T - second.T).norm(), 1.0);
}

TEST(GeometryEvaluatorTest, ZeroOffsetRecoversFullThreeDimensionalPath) {
  const GeometryEvaluator evaluator;
  const std::vector<PathDifferentialState> paths = {
      MakeLine(), MakeCircle(4.0, 0.3), MakeFigureEight(3.0, 0.6),
      MakeSlopedLine(1.5, 0.7), MakeHelix(3.0, 0.8, 1.1),
      MakeHeightVaryingParabola(2.0, 0.5)};

  for (const PathDifferentialState& path : paths) {
    const PhaseOffsetGeometryState state =
        Evaluate(evaluator, path, Eigen::Vector3d(1.0, -2.0, 0.5), 0.0);
    ExpectVectorNear(state.r, path.p, 1e-12);
    ExpectVectorNear(state.r_w, path.p_w, 1e-12);
    ExpectVectorNear(state.T, path.p_w / path.p_w.norm(), 1e-12);
  }
}

TEST(GeometryEvaluatorTest, SlopedLineUsesHorizontalNormalAndThreeDimensionalTangent) {
  const GeometryEvaluator evaluator;
  const PathDifferentialState path = MakeSlopedLine(2.0, 3.0);
  const PhaseOffsetGeometryState state =
      Evaluate(evaluator, path, Eigen::Vector3d::Zero(), 0.4);

  ExpectVectorNear(state.N, Eigen::Vector3d(0.0, 1.0, 0.0), 1e-12);
  ExpectVectorNear(state.N_w, Eigen::Vector3d::Zero(), 1e-12);
  ExpectVectorNear(state.r, path.p + Eigen::Vector3d(0.0, 0.4, 0.0), 1e-12);
  ExpectVectorNear(state.r_w, path.p_w, 1e-12);
  ExpectVectorNear(state.T, path.p_w / std::sqrt(5.0), 1e-12);
  EXPECT_NEAR(state.N.z(), 0.0, 1e-12);
  EXPECT_NEAR(state.N.dot(state.p_w), 0.0, 1e-12);
  EXPECT_NEAR(state.N.dot(state.N_w), 0.0, 1e-12);
  EXPECT_NEAR(state.N.dot(state.r_w), 0.0, 1e-12);
  EXPECT_NEAR(state.r.z(), path.p.z(), 1e-12);
}

TEST(GeometryEvaluatorTest, HelixMatchesNormalAndReferenceNumericalDerivatives) {
  const GeometryEvaluator evaluator;
  const double radius = 3.0;
  const double vertical_rate = 1.5;
  const double w = 0.8;
  const double delta = 0.3;
  const PathDifferentialState path = MakeHelix(radius, vertical_rate, w);
  const PhaseOffsetGeometryState state =
      Evaluate(evaluator, path, Eigen::Vector3d::Zero(), delta);

  const Eigen::Vector3d planar_p_w(path.p_w.x(), path.p_w.y(), 0.0);
  ExpectVectorNear(state.N_w, -state.curvature * planar_p_w, 1e-12);
  EXPECT_NEAR(state.curvature, 1.0 / radius, 1e-12);
  EXPECT_NEAR(state.N.z(), 0.0, 1e-12);
  EXPECT_NEAR(state.N.dot(state.p_w), 0.0, 1e-12);
  EXPECT_NEAR(state.N.dot(state.N_w), 0.0, 1e-12);
  EXPECT_NEAR(state.N.dot(state.r_w), 0.0, 1e-12);
  EXPECT_NEAR(state.r.z(), path.p.z(), 1e-12);
  EXPECT_GT(std::abs(state.T.z()), 1e-3);

  const double h = 1e-5;
  const PhaseOffsetGeometryState plus =
      Evaluate(evaluator, MakeHelix(radius, vertical_rate, w + h), Eigen::Vector3d::Zero(),
               delta);
  const PhaseOffsetGeometryState minus =
      Evaluate(evaluator, MakeHelix(radius, vertical_rate, w - h), Eigen::Vector3d::Zero(),
               delta);
  ExpectVectorNear((plus.N - minus.N) / (2.0 * h), state.N_w, 2e-9);
  ExpectVectorNear((plus.r - minus.r) / (2.0 * h), state.r_w, 2e-9);
}

TEST(GeometryEvaluatorTest, DescendingHelixPreservesAltitudeAndHorizontalOffset) {
  const GeometryEvaluator evaluator;
  const double radius = 2.5;
  const double vertical_rate = -1.25;
  const double w = 1.1;
  const double delta = 0.35;
  const PathDifferentialState path =
      MakeHelix(radius, vertical_rate, w, 4.0);
  const PhaseOffsetGeometryState state =
      Evaluate(evaluator, path, Eigen::Vector3d::Zero(), delta);

  EXPECT_TRUE(state.valid);
  EXPECT_LT(state.T.z(), -0.40);
  EXPECT_NEAR(state.r.z(), path.p.z(), 1e-12);
  EXPECT_NEAR(state.r_w.z(), path.p_w.z(), 1e-12);
  EXPECT_NEAR(state.N.z(), 0.0, 1e-12);
  EXPECT_NEAR(state.N_w.z(), 0.0, 1e-12);

  const double h = 1e-5;
  const PhaseOffsetGeometryState plus = Evaluate(
      evaluator, MakeHelix(radius, vertical_rate, w + h, 4.0),
      Eigen::Vector3d::Zero(), delta);
  const PhaseOffsetGeometryState minus = Evaluate(
      evaluator, MakeHelix(radius, vertical_rate, w - h, 4.0),
      Eigen::Vector3d::Zero(), delta);
  ExpectVectorNear((plus.N - minus.N) / (2.0 * h), state.N_w, 2e-9);
  ExpectVectorNear((plus.r - minus.r) / (2.0 * h), state.r_w, 2e-9);
}

TEST(GeometryEvaluatorTest, LargeFiniteVerticalDerivativesAreAccepted) {
  const GeometryEvaluator evaluator;
  const PathDifferentialState path = MakeHeightVaryingParabola(2.0, 1.5);
  ASSERT_GT(std::abs(path.p_w.z()), 1.0);
  ASSERT_GT(std::abs(path.p_ww.z()), 1.0);

  const PhaseOffsetGeometryState state =
      Evaluate(evaluator, path, Eigen::Vector3d::Zero(), 0.4);
  EXPECT_TRUE(state.valid);
  ExpectVectorNear(state.N, Eigen::Vector3d(0.0, 1.0, 0.0), 1e-12);
  ExpectVectorNear(state.N_w, Eigen::Vector3d::Zero(), 1e-12);
  ExpectVectorNear(state.r_w, path.p_w, 1e-12);
  EXPECT_NEAR(state.r.z(), path.p.z(), 1e-12);
}

TEST(GeometryEvaluatorTest, Full3DRegularityDoesNotUseLegacyCurvatureMargin) {
  GeometryParams params;
  params.regularity_margin = 0.1;
  const GeometryEvaluator evaluator(params);
  PhaseOffsetGeometryState state;
  EXPECT_TRUE(evaluator.evaluate(
      MakeCircle(1.0, 0.0), Eigen::Vector3d::Zero(), 0.95, state));
  EXPECT_GT(state.regularity, params.minimum_reference_speed);
}

TEST(GeometryEvaluatorTest, ConfiguredMinimumReferenceSpeedUsesFull3DFrameBound) {
  GeometryParams params;
  params.minimum_reference_speed = 0.50;
  PathDifferentialState frame_bound = MakeLine();
  frame_bound.p_w = Eigen::Vector3d(0.20, 0.0, 0.0);
  frame_bound.T = Eigen::Vector3d::UnitX();
  frame_bound.N = Eigen::Vector3d::UnitY();
  frame_bound.N_w = Eigen::Vector3d::UnitX();
  frame_bound.frame_valid = true;
  frame_bound.frame_provenance =
      "ContinuousPhaseNormalFrame/WorldHorizontalCrossProduct";
  const GeometryEvaluator evaluator(params);
  PhaseOffsetGeometryState safe;
  const bool safe_ok = evaluator.evaluate(
      frame_bound, Eigen::Vector3d::Zero(), -0.80, safe);
  EXPECT_TRUE(safe_ok) << safe.invalid_reason;
  EXPECT_NEAR(safe.r_w.norm(), 0.60, 1e-12);
  PhaseOffsetGeometryState unsafe;
  EXPECT_FALSE(evaluator.evaluate(frame_bound, Eigen::Vector3d::Zero(), 0.0,
                                  unsafe));
  EXPECT_FALSE(unsafe.valid);
}

TEST(GeometryEvaluatorTest, IncompatibleFrameProvenanceFailsClosed) {
  PathDifferentialState incompatible = MakeLine();
  incompatible.p_w = Eigen::Vector3d(0.20, 0.0, 0.0);
  incompatible.T = Eigen::Vector3d::UnitX();
  incompatible.N = Eigen::Vector3d::UnitY();
  incompatible.N_w = Eigen::Vector3d::UnitX();
  incompatible.frame_valid = true;
  incompatible.frame_provenance =
      "ContinuousPhaseNormalFrame/ProjectedHermiteTransport";

  PhaseOffsetGeometryState output;
  EXPECT_FALSE(GeometryEvaluator().evaluate(
      incompatible, Eigen::Vector3d::Zero(), 0.1, output));
  EXPECT_EQ(output.invalid_reason, "normal frame provenance is incompatible");
  EXPECT_FALSE(output.valid);
  ExpectFiniteState(output);

  // An unbound synthetic value is allowed to construct the unique
  // Horizontal-N representation from p_w/p_ww instead of consuming the
  // incompatible diagnostic string as frame authority.
  incompatible.frame_valid = false;
  output = PhaseOffsetGeometryState();
  ASSERT_TRUE(GeometryEvaluator().evaluate(
      incompatible, Eigen::Vector3d::Zero(), 0.1, output));
  ExpectVectorNear(output.N, Eigen::Vector3d::UnitY(), 1e-12);
  EXPECT_EQ(output.provenance,
            "ContinuousPhaseNormalFrame/WorldHorizontalCrossProduct");
}

TEST(GeometryEvaluatorTest, CanonicalFrameThresholdIsStrictForNonzeroOnly) {
  const double epsilon = phase_offset_core::kHorizontalNormalSpeedEpsilon;
  const double values[] = {
      std::nextafter(epsilon, 0.0), epsilon,
      std::nextafter(epsilon, std::numeric_limits<double>::infinity())};
  for (const double q : values) {
    PathDifferentialState path = MakeSlopedLine(1.0, 1.0);
    path.p_w = Eigen::Vector3d(q, 0.0, 1.0);
    path.T = path.p_w.normalized();
    path.N = Eigen::Vector3d::UnitY();
    path.N_w = Eigen::Vector3d::Zero();
    path.frame_valid = true;
    path.frame_provenance = kWorldHorizontalCrossProductProvenance;
    PhaseOffsetGeometryState neutral;
    ASSERT_TRUE(GeometryEvaluator().evaluate(
        path, Eigen::Vector3d::Zero(), 0.0, neutral));
    PhaseOffsetGeometryState nonzero;
    const bool ok = GeometryEvaluator().evaluate(
        path, Eigen::Vector3d::Zero(), 0.1, nonzero);
    EXPECT_EQ(ok, q > epsilon);
    EXPECT_EQ(nonzero.valid, q > epsilon);
  }
}

TEST(GeometryEvaluatorTest, VerticalOrNearVerticalPathsRetainNeutralCenterline) {
  const GeometryEvaluator evaluator;
  PathDifferentialState near_vertical = MakeSlopedLine(1.0, 0.0);
  near_vertical.p_w = Eigen::Vector3d(1e-10, 0.0, 1.0);
  PhaseOffsetGeometryState neutral;
  ASSERT_TRUE(evaluator.evaluate(
      near_vertical, Eigen::Vector3d::Zero(), 0.0, neutral));
  EXPECT_NEAR(neutral.r.z(), near_vertical.p.z(), 1e-12);
  ExpectInvalid(evaluator, near_vertical, Eigen::Vector3d::Zero(), 0.1);

  PathDifferentialState vertical = MakeSlopedLine(1.0, 0.0);
  vertical.p_w = Eigen::Vector3d(0.0, 0.0, 1.0);
  ASSERT_TRUE(evaluator.evaluate(
      vertical, Eigen::Vector3d::Zero(), 0.0, neutral));
  EXPECT_NEAR(neutral.r_w.z(), vertical.p_w.z(), 1e-12);
  ExpectInvalid(evaluator, vertical, Eigen::Vector3d::Zero(), 0.1);
}

TEST(GeometryEvaluatorTest, HorizontalNormalThresholdIsStrictForNonzeroOnly) {
  const double epsilon = phase_offset_core::kHorizontalNormalSpeedEpsilon;
  const double below = std::nextafter(epsilon, 0.0);
  const double equal = epsilon;
  const double above = std::nextafter(epsilon,
                                      std::numeric_limits<double>::infinity());
  for (const double q : {below, equal, above}) {
    PathDifferentialState path = MakeSlopedLine(1.0, 1.0);
    path.p_w = Eigen::Vector3d(q, 0.0, 1.0);
    PhaseOffsetGeometryState neutral;
    ASSERT_TRUE(GeometryEvaluator().evaluate(
        path, Eigen::Vector3d::Zero(), 0.0, neutral));
    EXPECT_NEAR(neutral.r.z(), path.p.z(), 1e-12);
    PhaseOffsetGeometryState nonzero;
    const bool ok = GeometryEvaluator().evaluate(
        path, Eigen::Vector3d::Zero(), 0.1, nonzero);
    EXPECT_EQ(ok, q > epsilon);
    EXPECT_EQ(nonzero.valid, q > epsilon);
  }
}

TEST(GeometryEvaluatorTest,
     UnavailableHorizontalNormalRejectsFiniteTinyNonzeroOffsets) {
  PathDifferentialState path = MakeSlopedLine(1.0, 0.0);
  path.p_w = Eigen::Vector3d(0.0, 0.0, 1.0);
  path.T = Eigen::Vector3d::UnitZ();
  path.N = Eigen::Vector3d::UnitX();
  path.N_w = Eigen::Vector3d::Zero();
  path.frame_valid = true;
  path.frame_provenance = kWorldHorizontalCrossProductProvenance;

  const double tiny = std::numeric_limits<double>::denorm_min();
  for (const double delta : {tiny, -tiny}) {
    PhaseOffsetGeometryState output;
    EXPECT_FALSE(GeometryEvaluator().evaluate(
        path, Eigen::Vector3d::Zero(), delta, output));
    EXPECT_EQ(output.invalid_reason, "horizontal path speed is too small");
    EXPECT_FALSE(output.valid);
    ExpectFiniteState(output);
  }

  PhaseOffsetGeometryState neutral;
  ASSERT_TRUE(GeometryEvaluator().evaluate(
      path, Eigen::Vector3d::Zero(), 0.0, neutral));
  EXPECT_TRUE(neutral.valid);
}

TEST(GeometryEvaluatorTest, ClosedCellHorizontalSpeedThresholdIsStrict) {
  const double epsilon = phase_offset_core::kHorizontalNormalSpeedEpsilon;
  const double values[] = {
      std::nextafter(epsilon, 0.0), epsilon,
      std::nextafter(epsilon, std::numeric_limits<double>::infinity())};
  for (const double q : values) {
    PathCellGeometryCertificate certificate;
    certificate.w0 = 0.0;
    certificate.w1 = 1.0;
    certificate.segment_w0 = 0.0;
    certificate.segment_w1 = 1.0;
    certificate.segment_identity = 1U;
    certificate.inf_p_w_norm = 1.0;
    certificate.inf_horizontal_p_w_norm = q;
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
    certificate.valid = true;
    certificate.complete = true;
    EXPECT_EQ(pathCellGeometryCertificateIsComplete(certificate), q > epsilon);
  }
}

TEST(GeometryEvaluatorTest, FrameCellCertificateProvenanceIsStrict) {
  PathCellGeometryCertificate certificate;
  certificate.w0 = 0.0;
  certificate.w1 = 1.0;
  certificate.path_revision = 7U;
  certificate.frame_revision = 8U;
  certificate.segment_w0 = 0.0;
  certificate.segment_w1 = 1.0;
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
  certificate.normal_frame_proof_complete = true;
  certificate.valid = true;
  certificate.complete = true;

  certificate.provenance =
      "ContinuousPhaseNormalFrame/ProjectedHermiteTransport";
  EXPECT_FALSE(pathCellGeometryCertificateIsComplete(certificate));
  certificate.provenance = kWorldHorizontalCrossProductProvenance;
  EXPECT_TRUE(pathCellGeometryCertificateIsComplete(certificate));
}

TEST(GeometryEvaluatorTest, CellCertificateRejectsUnderstatedDerivedBounds) {
  const PathCellGeometryCertificate consistent =
      MakeConsistentCellCertificate();
  ASSERT_TRUE(pathCellGeometryCertificateIsComplete(consistent));

  PathCellGeometryCertificate understated_normal = consistent;
  understated_normal.sup_N_w_norm =
      std::nextafter(consistent.sup_N_w_norm, 0.0);
  EXPECT_FALSE(pathCellGeometryCertificateIsComplete(understated_normal));

  PathCellGeometryCertificate understated_normal_variation = consistent;
  understated_normal_variation.normal_variation_bound =
      std::nextafter(consistent.normal_variation_bound, 0.0);
  EXPECT_FALSE(pathCellGeometryCertificateIsComplete(
      understated_normal_variation));

  PathCellGeometryCertificate missing_horizontal_acceleration = consistent;
  missing_horizontal_acceleration.horizontal_acceleration_bound_complete =
      false;
  EXPECT_FALSE(pathCellGeometryCertificateIsComplete(
      missing_horizontal_acceleration));

  PathCellGeometryCertificate understated_tangent_variation = consistent;
  understated_tangent_variation.tangent_variation_bound =
      std::nextafter(consistent.tangent_variation_bound, 0.0);
  EXPECT_FALSE(pathCellGeometryCertificateIsComplete(
      understated_tangent_variation));
}

TEST(GeometryEvaluatorTest, CellCertificateBindsProvenanceRevisionAndRange) {
  PathCellGeometryCertificate bound = MakeConsistentCellCertificate();
  bound.path_revision = 7U;
  bound.frame_revision = 8U;
  bound.normal_frame_proof_complete = true;
  bound.provenance = kWorldHorizontalCrossProductProvenance;
  ASSERT_TRUE(pathCellGeometryCertificateIsComplete(bound));
  EXPECT_TRUE(pathCellGeometryCertificateMatches(bound, 7U, 8U));
  EXPECT_FALSE(pathCellGeometryCertificateMatches(bound, 7U, 9U));

  PathCellGeometryCertificate one_revision = bound;
  one_revision.frame_revision = 0U;
  EXPECT_FALSE(pathCellGeometryCertificateIsComplete(one_revision));

  PathCellGeometryCertificate range_mismatch = MakeConsistentCellCertificate();
  range_mismatch.w0 = -0.1;
  EXPECT_FALSE(pathCellGeometryCertificateIsComplete(range_mismatch));

  PathCellGeometryCertificate provenance_mismatch = bound;
  provenance_mismatch.provenance =
      "ContinuousPhaseNormalFrame/ProjectedHermiteTransport";
  EXPECT_FALSE(pathCellGeometryCertificateIsComplete(provenance_mismatch));
}

TEST(GeometryEvaluatorTest,
     PositiveSubnormalNormalVariationCannotCollapseToZero) {
  const double positive_subnormal = std::numeric_limits<double>::denorm_min();
  PathCellGeometryCertificate certificate = MakeConsistentCellCertificate();
  certificate.w1 = 0.5;
  certificate.inf_horizontal_p_w_norm = 1.0;
  certificate.sup_horizontal_p_ww_norm = positive_subnormal;
  certificate.sup_N_w_norm = positive_subnormal;
  certificate.normal_variation_bound = 0.0;
  EXPECT_FALSE(pathCellGeometryCertificateIsComplete(certificate));
  certificate.normal_variation_bound = positive_subnormal;
  EXPECT_TRUE(pathCellGeometryCertificateIsComplete(certificate));

  NormalFrameCellProof proof;
  proof.w0 = 0.0;
  proof.w1 = 0.5;
  proof.path_revision = 7U;
  proof.frame_revision = 8U;
  proof.inf_path_speed = 1.0;
  proof.sup_path_speed = 1.0;
  proof.sup_path_acceleration = 0.0;
  proof.sup_path_jerk = 0.0;
  proof.inf_horizontal_path_speed = 1.0;
  proof.sup_horizontal_p_ww_norm = positive_subnormal;
  proof.horizontal_acceleration_bound_complete = true;
  proof.sup_normal_derivative = positive_subnormal;
  proof.normal_variation_bound = 0.0;
  proof.tangent_variation_bound = 0.0;
  proof.valid = true;
  proof.complete = true;
  proof.provenance = kWorldHorizontalCrossProductProvenance;
  EXPECT_FALSE(normalFrameCellProofIsComplete(proof));
  proof.normal_variation_bound = positive_subnormal;
  EXPECT_TRUE(normalFrameCellProofIsComplete(proof));
}

TEST(GeometryEvaluatorTest,
     OutwardUpperProductUsesExactDyadicComparisonNearUnderflow) {
  const double lhs = 0x1.1a283fd4e7426p-376;
  const double rhs = 0x1.f3c83c1c1ef05p-645;
  const double ordinary = lhs * rhs;
  const double expected = std::nextafter(
      ordinary, std::numeric_limits<double>::infinity());
  double upper = 0.0;
  ASSERT_TRUE(outwardUpperProduct(lhs, rhs, upper));
  EXPECT_DOUBLE_EQ(upper, expected);
  EXPECT_GT(upper, ordinary);

  double exact = 0.0;
  ASSERT_TRUE(outwardUpperProduct(0.5, 2.0, exact));
  EXPECT_DOUBLE_EQ(exact, 1.0);
  ASSERT_TRUE(outwardUpperProduct(
      std::numeric_limits<double>::denorm_min(), 0.5, exact));
  EXPECT_DOUBLE_EQ(exact, std::numeric_limits<double>::denorm_min());
}

TEST(GeometryEvaluatorTest,
     OutwardUpperRatioUsesExactDyadicComparisonNearUnderflow) {
  const double numerator = 0x1.b1c0d9e3ba427p-1022;
  const double denominator = 0x1.2934d2a911067p-1;
  const double ordinary = numerator / denominator;
  const double expected = std::nextafter(
      ordinary, std::numeric_limits<double>::infinity());
  double upper = 0.0;
  ASSERT_TRUE(outwardUpperRatio(numerator, denominator, upper));
  EXPECT_DOUBLE_EQ(upper, expected);
  EXPECT_GT(upper, ordinary);

  double exact = 0.0;
  ASSERT_TRUE(outwardUpperRatio(1.0, 2.0, exact));
  EXPECT_DOUBLE_EQ(exact, 0.5);
  ASSERT_TRUE(outwardUpperRatio(
      std::numeric_limits<double>::denorm_min(), 2.0, exact));
  EXPECT_DOUBLE_EQ(exact,
                   std::numeric_limits<double>::denorm_min());
}

TEST(GeometryEvaluatorTest, InvalidInputsAndParametersHaveFiniteDiagnostics) {
  const GeometryEvaluator evaluator;
  const Eigen::Vector3d position = Eigen::Vector3d::Zero();

  PathDifferentialState invalid_path = MakeLine();
  invalid_path.valid = false;
  ExpectInvalid(evaluator, invalid_path, position, 0.0);

  PathDifferentialState zero_tangent = MakeLine();
  zero_tangent.p_w = Eigen::Vector3d::Zero();
  ExpectInvalid(evaluator, zero_tangent, position, 0.0);

  PathDifferentialState non_finite_path = MakeLine();
  non_finite_path.p.x() = std::numeric_limits<double>::quiet_NaN();
  ExpectInvalid(evaluator, non_finite_path, position, 0.0);

  ExpectInvalid(evaluator, MakeLine(),
                Eigen::Vector3d(std::numeric_limits<double>::infinity(), 0.0, 0.0),
                0.0);

  PathDifferentialState overflowing_speed = MakeLine();
  overflowing_speed.p_w = Eigen::Vector3d(std::numeric_limits<double>::max(),
                                            std::numeric_limits<double>::max(), 1.0);
  ExpectInvalid(evaluator, overflowing_speed, position, 0.0);

  GeometryParams invalid_tangent_params;
  invalid_tangent_params.tangent_epsilon = 0.0;
  ExpectInvalid(GeometryEvaluator(invalid_tangent_params), MakeLine(), position, 0.0);

  GeometryParams invalid_horizontal_params;
  invalid_horizontal_params.horizontal_tangent_epsilon = 0.0;
  ExpectInvalid(GeometryEvaluator(invalid_horizontal_params), MakeLine(), position, 0.0);
}

TEST(GeometryEvaluatorTest, PreparedPathDelaysPathDerivedFailuresUntilPointChecks) {
  const GeometryEvaluator evaluator;
  PathDifferentialState zero_tangent = MakeLine();
  zero_tangent.p_w = Eigen::Vector3d::Zero();
  PreparedPathGeometry prepared;
  evaluator.preparePath(zero_tangent, prepared);
  ASSERT_TRUE(prepared.pre_point_valid);
  ASSERT_FALSE(prepared.delayed_path_valid);
  EXPECT_STREQ("path speed is too small", prepared.delayed_path_reason);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const Eigen::Vector3d infinite_position(
      std::numeric_limits<double>::infinity(), 0.0, 0.0);
  PreparedReferenceResult reference;
  EXPECT_FALSE(evaluator.evaluatePreparedReference(
      prepared, infinite_position, nan, reference));
  EXPECT_STREQ("position is not finite", reference.invalid_reason);
  ExpectPreparedReferenceMatchesFull(evaluator, zero_tangent, infinite_position, nan);

  EXPECT_FALSE(evaluator.evaluatePreparedReference(
      prepared, Eigen::Vector3d::Zero(), nan, reference));
  EXPECT_STREQ("offset is not finite", reference.invalid_reason);
  ExpectPreparedReferenceMatchesFull(
      evaluator, zero_tangent, Eigen::Vector3d::Zero(), nan);

  EXPECT_FALSE(evaluator.evaluatePreparedReference(
      prepared, Eigen::Vector3d::Zero(), 0.0, reference));
  EXPECT_STREQ("path speed is too small", reference.invalid_reason);
  ExpectPreparedReferenceMatchesFull(
      evaluator, zero_tangent, Eigen::Vector3d::Zero(), 0.0);
}

TEST(GeometryEvaluatorTest, PreparedReferenceMatchesFullForCollisionAndThresholdCases) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();

  GeometryParams invalid_params;
  invalid_params.tangent_epsilon = 0.0;
  PathDifferentialState invalid_path = MakeLine();
  invalid_path.valid = false;
  ExpectPreparedReferenceMatchesFull(
      GeometryEvaluator(invalid_params), invalid_path,
      Eigen::Vector3d(infinity, 0.0, 0.0), nan);

  ExpectPreparedReferenceMatchesFull(
      GeometryEvaluator(), invalid_path, Eigen::Vector3d(infinity, 0.0, 0.0), nan);

  PathDifferentialState non_finite_path = MakeLine();
  non_finite_path.p.x() = nan;
  ExpectPreparedReferenceMatchesFull(
      GeometryEvaluator(), non_finite_path, Eigen::Vector3d(infinity, 0.0, 0.0), nan);

  PathDifferentialState zero_tangent = MakeLine();
  zero_tangent.p_w = Eigen::Vector3d::Zero();
  ExpectPreparedReferenceMatchesFull(
      GeometryEvaluator(), zero_tangent, Eigen::Vector3d(nan, 0.0, 0.0), nan);
  ExpectPreparedReferenceMatchesFull(GeometryEvaluator(), zero_tangent, zero, nan);
  ExpectPreparedReferenceMatchesFull(GeometryEvaluator(), zero_tangent, zero, 0.0);

  PathDifferentialState curvature_overflow = MakeLine();
  curvature_overflow.p_w = Eigen::Vector3d::UnitX();
  curvature_overflow.p_ww = Eigen::Vector3d(
      0.0, std::numeric_limits<double>::max(), 0.0);
  ExpectPreparedReferenceMatchesFull(
      GeometryEvaluator(), curvature_overflow, zero, nan);
  ExpectPreparedReferenceMatchesFull(
      GeometryEvaluator(), curvature_overflow, zero, 2.0);

  const double epsilon = 1e-8;
  PathDifferentialState tangent_below = MakeLine();
  tangent_below.p_w = Eigen::Vector3d(std::nextafter(epsilon, 0.0), 0.0, 0.0);
  ExpectPreparedReferenceMatchesFull(GeometryEvaluator(), tangent_below, zero, 0.0);
  PathDifferentialState tangent_above = MakeLine();
  tangent_above.p_w = Eigen::Vector3d(std::nextafter(epsilon, infinity), 0.0, 0.0);
  ExpectPreparedReferenceMatchesFull(GeometryEvaluator(), tangent_above, zero, -0.0);

  PathDifferentialState horizontal_below = MakeSlopedLine(1.0, 0.0);
  horizontal_below.p_w =
      Eigen::Vector3d(std::nextafter(epsilon, 0.0), 0.0, 1.0);
  ExpectPreparedReferenceMatchesFull(GeometryEvaluator(), horizontal_below, zero, 0.0);
  PathDifferentialState horizontal_above = MakeSlopedLine(1.0, 0.0);
  horizontal_above.p_w =
      Eigen::Vector3d(std::nextafter(epsilon, infinity), 0.0, 1.0);
  ExpectPreparedReferenceMatchesFull(GeometryEvaluator(), horizontal_above, zero, 0.0);
}

TEST(GeometryEvaluatorTest, PreparedReferenceRetainsReferenceFieldsOnTerminalFailures) {
  GeometryParams params;
  params.regularity_margin = 0.1;
  const GeometryEvaluator evaluator(params);
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();

  const PathDifferentialState margin_path = MakeCircle(1.0, 0.0);
  PreparedPathGeometry margin_prepared;
  evaluator.preparePath(margin_path, margin_prepared);
  PreparedReferenceResult margin_reference;
  EXPECT_TRUE(evaluator.evaluatePreparedReference(
      margin_prepared, zero, 0.95, margin_reference));
  EXPECT_STREQ("", margin_reference.invalid_reason);
  EXPECT_TRUE(std::isfinite(margin_reference.regularity));
  ExpectPreparedReferenceMatchesFull(evaluator, margin_path, zero, 0.95);

  const double radius = 1.5e-8;
  const PathDifferentialState active_speed_path = MakeCircle(radius, 0.0);
  PreparedPathGeometry active_speed_prepared;
  evaluator.preparePath(active_speed_path, active_speed_prepared);
  PreparedReferenceResult active_speed_reference;
  EXPECT_FALSE(evaluator.evaluatePreparedReference(
      active_speed_prepared, zero, 0.5 * radius, active_speed_reference));
  EXPECT_STREQ("active reference speed is too small",
               active_speed_reference.invalid_reason);
  ExpectPreparedReferenceMatchesFull(
      evaluator, active_speed_path, zero, 0.5 * radius);

  PathDifferentialState final_finite_path = MakeLine();
  final_finite_path.p.x() = -std::numeric_limits<double>::max();
  const Eigen::Vector3d overflowing_position(
      std::numeric_limits<double>::max(), 0.0, 0.0);
  PreparedPathGeometry final_finite_prepared;
  evaluator.preparePath(final_finite_path, final_finite_prepared);
  PreparedReferenceResult final_finite_reference;
  EXPECT_FALSE(evaluator.evaluatePreparedReference(
      final_finite_prepared, overflowing_position, 0.0, final_finite_reference));
  EXPECT_STREQ("computed geometry is not finite",
               final_finite_reference.invalid_reason);
  EXPECT_TRUE(final_finite_reference.r.allFinite());
  EXPECT_TRUE(std::isfinite(final_finite_reference.regularity));
  ExpectPreparedReferenceMatchesFull(
      evaluator, final_finite_path, overflowing_position, 0.0);
}

TEST(GeometryEvaluatorTest, PreparedReferenceMatchesFullForFixedSeedRandomCorpus) {
  const GeometryEvaluator evaluator;
  std::mt19937_64 generator(0xA51C20260812ULL);
  std::uniform_real_distribution<double> radius_distribution(0.25, 8.0);
  std::uniform_real_distribution<double> vertical_distribution(-3.0, 3.0);
  std::uniform_real_distribution<double> phase_distribution(-6.0, 6.0);
  std::uniform_real_distribution<double> position_distribution(-100.0, 100.0);
  std::uniform_real_distribution<double> delta_distribution(-0.20, 0.20);

  for (int sample = 0; sample < 256; ++sample) {
    const PathDifferentialState path = MakeHelix(
        radius_distribution(generator), vertical_distribution(generator),
        phase_distribution(generator));
    const Eigen::Vector3d position(
        position_distribution(generator), position_distribution(generator),
        position_distribution(generator));
    for (const double delta : {
             -0.20, 0.0, 0.20, delta_distribution(generator)}) {
      ExpectPreparedReferenceMatchesFull(evaluator, path, position, delta);
    }
  }
}

TEST(GeometryEvaluatorTest, MultisampleEllipseMaintainsDifferentialConsistency) {
  const GeometryEvaluator evaluator;
  std::mt19937 generator(17U);
  std::uniform_real_distribution<double> phase_distribution(-kPi, kPi);
  std::uniform_real_distribution<double> offset_distribution(-0.25, 0.25);

  for (int sample = 0; sample < 100; ++sample) {
    const double w = phase_distribution(generator);
    const double delta = offset_distribution(generator);
    const PathDifferentialState path = MakeEllipse(3.0, 2.0, w);
    const PhaseOffsetGeometryState state =
        Evaluate(evaluator, path, Eigen::Vector3d(0.5, -0.25, 1.2), delta);
    EXPECT_NEAR(state.T.dot(state.N), 0.0, 1e-12);
    EXPECT_NEAR(state.N.dot(state.N_w), 0.0, 1e-12);
    EXPECT_NEAR(state.N.dot(state.r_w), 0.0, 1e-12);
    EXPECT_NEAR(state.T.norm(), 1.0, 1e-12);
    EXPECT_NEAR(state.N.norm(), 1.0, 1e-12);
    ExpectFiniteState(state);

    const double h = 1e-5;
    const Eigen::Vector3d r_plus =
        Evaluate(evaluator, MakeEllipse(3.0, 2.0, w + h), Eigen::Vector3d::Zero(), delta)
            .r;
    const Eigen::Vector3d r_minus =
        Evaluate(evaluator, MakeEllipse(3.0, 2.0, w - h), Eigen::Vector3d::Zero(), delta)
            .r;
    ExpectVectorNear((r_plus - r_minus) / (2.0 * h), state.r_w, 3e-8);
  }
}

}  // namespace
}  // namespace phase_offset_core

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
