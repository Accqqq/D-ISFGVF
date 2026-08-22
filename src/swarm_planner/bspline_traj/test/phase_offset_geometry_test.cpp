#include <gtest/gtest.h>

#include <cmath>

#include <bspline_race/phase_offset_geometry.h>

namespace
{

using FLAG_Race::ContinuousPhasePathState;
using FLAG_Race::PhaseOffsetGeometry;
using FLAG_Race::PhaseOffsetGeometryEvaluator;
using FLAG_Race::PhaseOffsetGeometryParams;

ContinuousPhasePathState makeLineState(double w)
{
  ContinuousPhasePathState s;
  s.p = Eigen::Vector3d(w, 0.0, 1.0);
  s.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
  s.d2p_dw2 = Eigen::Vector3d::Zero();
  return s;
}

ContinuousPhasePathState makeCircleState(double w, double R)
{
  ContinuousPhasePathState s;
  s.p = Eigen::Vector3d(R * std::cos(w / R), R * std::sin(w / R), 1.0);
  s.dp_dw = Eigen::Vector3d(-std::sin(w / R), std::cos(w / R), 0.0);
  s.d2p_dw2 = Eigen::Vector3d(-(1.0 / R) * std::cos(w / R),
                              -(1.0 / R) * std::sin(w / R), 0.0);
  return s;
}

// Lemniscate: x = R sin(w), y = (R/2) sin(2w).  p(0) == p(pi) == center,
// but the tangents are opposite -> the branch is distinguished by w.
ContinuousPhasePathState makeFigureEightState(double w, double R)
{
  ContinuousPhasePathState s;
  s.p = Eigen::Vector3d(R * std::sin(w), 0.5 * R * std::sin(2.0 * w), 1.0);
  s.dp_dw = Eigen::Vector3d(R * std::cos(w), R * std::cos(2.0 * w), 0.0);
  s.d2p_dw2 =
    Eigen::Vector3d(-R * std::sin(w), -2.0 * R * std::sin(2.0 * w), 0.0);
  return s;
}

PhaseOffsetGeometryParams makeParams()
{
  PhaseOffsetGeometryParams p;
  p.mu_regular = 0.20;
  p.max_delta = 5.0;
  return p;
}

TEST(PhaseOffsetGeometry, StraightLineDeltaZero)
{
  PhaseOffsetGeometryEvaluator ev;
  PhaseOffsetGeometry out;
  ASSERT_TRUE(ev.evaluate(makeLineState(3.0), Eigen::Vector3d(3.2, 0.1, 1.0),
                          0.0, makeParams(), out));
  EXPECT_NEAR(0.0, (out.r - out.p).norm(), 1e-12);
  EXPECT_NEAR(0.0, (out.r_w - out.dp_dw).norm(), 1e-12);
  EXPECT_NEAR(0.0, out.curvature, 1e-12);
  EXPECT_NEAR(0.0, out.e_perp.dot(out.T), 1e-9);
}

TEST(PhaseOffsetGeometry, StraightLinePositiveNegativeDelta)
{
  PhaseOffsetGeometryEvaluator ev;
  const Eigen::Vector3d pos(3.2, 0.5, 1.0);
  PhaseOffsetGeometry out_p;
  PhaseOffsetGeometry out_n;
  ASSERT_TRUE(ev.evaluate(makeLineState(3.0), pos, 0.5, makeParams(), out_p));
  ASSERT_TRUE(ev.evaluate(makeLineState(3.0), pos, -0.5, makeParams(), out_n));
  // N = (0, 1, 0) for a line along +x
  EXPECT_NEAR(0.5, out_p.r.y(), 1e-12);
  EXPECT_NEAR(-0.5, out_n.r.y(), 1e-12);
  EXPECT_NEAR(0.0, out_p.r.x() - out_p.p.x(), 1e-12);
  EXPECT_NEAR(0.0, out_n.r.x() - out_n.p.x(), 1e-12);
  // r_w unchanged by delta on a straight line (kappa = 0)
  EXPECT_NEAR(0.0, (out_p.r_w - out_p.dp_dw).norm(), 1e-12);
}

TEST(PhaseOffsetGeometry, CircleCurvatureMatchesAnalytic)
{
  PhaseOffsetGeometryEvaluator ev;
  const double R = 5.0;
  PhaseOffsetGeometry out;
  ASSERT_TRUE(ev.evaluate(makeCircleState(2.0, R),
                          Eigen::Vector3d(R, 0.1, 1.0), 0.3, makeParams(),
                          out));
  EXPECT_NEAR(1.0 / R, out.curvature, 1e-9);
  EXPECT_NEAR(1.0 - (1.0 / R) * 0.3, 1.0 - out.curvature * out.delta, 1e-9);
  // r_w = (1 - kappa*delta) * p_w
  EXPECT_NEAR(0.0,
              (out.r_w - (1.0 - out.curvature * out.delta) * out.dp_dw).norm(),
              1e-9);
}

TEST(PhaseOffsetGeometry, RwOrthogonalToN)
{
  PhaseOffsetGeometryEvaluator ev;
  PhaseOffsetGeometry out;
  ASSERT_TRUE(ev.evaluate(makeCircleState(1.7, 4.0),
                          Eigen::Vector3d(4.0, 0.2, 1.0), 0.4, makeParams(),
                          out));
  EXPECT_LT(std::abs(out.r_w.dot(out.N)), 1e-9);
  EXPECT_NEAR(1.0, out.T.norm(), 1e-12);
  EXPECT_NEAR(1.0, out.N.norm(), 1e-12);
}

TEST(PhaseOffsetGeometry, DegenerateRegularityDetected)
{
  PhaseOffsetGeometryEvaluator ev;
  PhaseOffsetGeometry out;
  const double R = 3.0;
  // delta = R -> 1 - kappa*delta = 0 < mu
  EXPECT_FALSE(ev.evaluate(makeCircleState(1.0, R),
                           Eigen::Vector3d(0.0, 0.0, 1.0), R, makeParams(),
                           out));
  EXPECT_FALSE(ev.evaluate(makeCircleState(1.0, R),
                           Eigen::Vector3d(0.0, 0.0, 1.0), R + 1.0,
                           makeParams(), out));
  EXPECT_FALSE(out.valid);
}

TEST(PhaseOffsetGeometry, ErrorDecompositionOnActiveReference)
{
  PhaseOffsetGeometryEvaluator ev;
  PhaseOffsetGeometry out;
  const Eigen::Vector3d pos(4.2, 0.6, 1.0);
  ASSERT_TRUE(ev.evaluate(makeLineState(4.0), pos, 0.5, makeParams(), out));
  const Eigen::Vector3d e = pos - out.r;
  const Eigen::Vector3d tau = out.r_w.normalized();
  EXPECT_NEAR(out.e_parallel, tau.dot(e), 1e-9);
  EXPECT_NEAR(0.0, (e - out.e_parallel * tau - out.e_perp).norm(), 1e-9);
  EXPECT_NEAR(out.rho, out.e_perp.norm(), 1e-9);
}

TEST(PhaseOffsetGeometry, FiniteDifferenceRwMatchesAnalytic)
{
  PhaseOffsetGeometryEvaluator ev;
  const double R = 5.0;
  const double w = 2.0;
  const double delta = 0.3;
  const double eps = 1e-5;
  PhaseOffsetGeometry out;
  ASSERT_TRUE(ev.evaluate(makeCircleState(w, R),
                          Eigen::Vector3d(5.0, 0.0, 1.0), delta, makeParams(),
                          out));
  PhaseOffsetGeometry out_p;
  PhaseOffsetGeometry out_m;
  ASSERT_TRUE(ev.evaluate(makeCircleState(w + eps, R),
                          Eigen::Vector3d(5.0, 0.0, 1.0), delta, makeParams(),
                          out_p));
  ASSERT_TRUE(ev.evaluate(makeCircleState(w - eps, R),
                          Eigen::Vector3d(5.0, 0.0, 1.0), delta, makeParams(),
                          out_m));
  const Eigen::Vector3d fd = (out_p.r - out_m.r) / (2.0 * eps);
  EXPECT_LT((fd - out.r_w).norm(), 1e-6);
}

TEST(PhaseOffsetGeometry, FigureEightBranchesDistinguishedByW)
{
  PhaseOffsetGeometryEvaluator ev;
  const double R = 6.0;
  const double delta = 0.5;
  PhaseOffsetGeometry out0;
  PhaseOffsetGeometry out1;
  // w = 0 and w = pi both map to the center (0,0), opposite tangents.
  ASSERT_TRUE(ev.evaluate(makeFigureEightState(0.0, R),
                          Eigen::Vector3d(0.0, 0.0, 1.0), delta, makeParams(),
                          out0));
  ASSERT_TRUE(ev.evaluate(makeFigureEightState(M_PI, R),
                          Eigen::Vector3d(0.0, 0.0, 1.0), delta, makeParams(),
                          out1));
  EXPECT_NEAR(0.0, (out0.p - out1.p).norm(), 1e-9);
  // Same position, different branch: tangents differ (45 vs 135 deg), so the
  // active references separate by delta * |N0 - N1| = 0.5 * sqrt(2).
  EXPECT_GT((out0.T - out1.T).norm(), 1.0);
  EXPECT_NEAR(delta * std::sqrt(2.0), (out0.r - out1.r).norm(), 1e-6);
  EXPECT_NEAR(0.0, out0.N.dot(out1.N), 1e-6);
}

}  // namespace

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
