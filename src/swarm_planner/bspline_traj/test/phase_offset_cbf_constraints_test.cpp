#include <gtest/gtest.h>

#include <cmath>

#include <bspline_race/continuous_phase_path.h>
#include <bspline_race/phase_offset_cbf_constraints.h>
#include <bspline_race/phase_offset_geometry.h>

namespace
{

using FLAG_Race::LinearConstraint2D;
using FLAG_Race::ContinuousPhasePathState;
using FLAG_Race::PhaseOffsetCbfConstraints;
using FLAG_Race::PhaseOffsetGeometry;
using FLAG_Race::PhaseOffsetGeometryEvaluator;
using FLAG_Race::PhaseOffsetGeometryParams;
using FLAG_Race::PredictedNeighborState;
using FLAG_Race::TubeBounds;

PhaseOffsetGeometry makeGeometry()
{
  ContinuousPhasePathState s;
  s.p = Eigen::Vector3d(2.0, 0.0, 1.0);
  s.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
  s.d2p_dw2 = Eigen::Vector3d::Zero();
  PhaseOffsetGeometryEvaluator ev;
  PhaseOffsetGeometryParams gp;
  gp.mu_regular = 0.20;
  gp.max_delta = 5.0;
  PhaseOffsetGeometry g;
  EXPECT_TRUE(ev.evaluate(s, Eigen::Vector3d(2.0, 0.0, 1.0), 0.0, gp, g));
  return g;
}

TEST(PhaseOffsetCbf, TubeUpperConstraintLimitsUDelta)
{
  PhaseOffsetCbfConstraints cbf;
  const PhaseOffsetGeometry g = makeGeometry();
  TubeBounds tube;
  tube.valid = true;
  tube.lower = -0.8;
  tube.upper = 0.8;
  tube.lower_dw = 0.0;
  tube.upper_dw = 0.0;
  PhaseOffsetGeometry g_near = g;
  g_near.delta = 0.78;  // near the upper bound
  std::vector<LinearConstraint2D> cons;
  cbf.appendTubeConstraints(g_near, tube, cons);
  ASSERT_EQ(2u, cons.size());
  // upper: -u_delta >= -gamma*(upper - delta - margin)
  const LinearConstraint2D& upper = cons[0];
  EXPECT_NEAR(0.0, upper.a(0), 1e-12);
  EXPECT_NEAR(-1.0, upper.a(1), 1e-12);
  const double h = tube.upper - g_near.delta - cbf.tube_margin;
  EXPECT_NEAR(-cbf.cbf_gamma_tube * h, upper.b, 1e-9);
  // u_delta must not exceed gamma*h (small positive), i.e. the CBF keeps the
  // delta inside the tube.
  EXPECT_LE(-upper.b, cbf.cbf_gamma_tube * h + 1e-9);
}

TEST(PhaseOffsetCbf, TubeLowerConstraint)
{
  PhaseOffsetCbfConstraints cbf;
  PhaseOffsetGeometry g = makeGeometry();
  TubeBounds tube;
  tube.valid = true;
  tube.lower = -0.8;
  tube.upper = 0.8;
  tube.lower_dw = 0.0;
  tube.upper_dw = 0.0;
  g.delta = -0.78;
  std::vector<LinearConstraint2D> cons;
  cbf.appendTubeConstraints(g, tube, cons);
  const LinearConstraint2D& lower = cons[1];
  EXPECT_NEAR(0.0, lower.a(0), 1e-12);
  EXPECT_NEAR(1.0, lower.a(1), 1e-12);
  const double h = g.delta - tube.lower - cbf.tube_margin;
  EXPECT_NEAR(-cbf.cbf_gamma_tube * h, lower.b, 1e-9);
}

TEST(PhaseOffsetCbf, PairwiseConstraintForm)
{
  PhaseOffsetCbfConstraints cbf;
  const PhaseOffsetGeometry g = makeGeometry();
  const Eigen::Vector3d self(2.0, 0.0, 1.0);
  PredictedNeighborState n;
  n.robot_id = 1;
  n.predicted_position = Eigen::Vector3d(2.0, 0.5, 1.0);
  n.advertised_velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
  n.velocity_uncertainty_bound = 0.2;
  std::vector<PredictedNeighborState,
              Eigen::aligned_allocator<PredictedNeighborState>> neigh;
  neigh.push_back(n);
  std::vector<LinearConstraint2D> cons;
  cbf.appendPairwiseConstraints(g, self, neigh, cons);
  ASSERT_EQ(1u, cons.size());
  const LinearConstraint2D& c = cons[0];
  const Eigen::Vector3d r = self - n.predicted_position;
  // a = 2 J^T r, J = [r_w, N]
  EXPECT_NEAR(2.0 * r.dot(g.r_w), c.a(0), 1e-9);
  EXPECT_NEAR(2.0 * r.dot(g.N), c.a(1), 1e-9);
  EXPECT_EQ("pair_cbf_1", c.label);
}

TEST(PhaseOffsetCbf, PairwiseConstraintActivatesWhenClose)
{
  PhaseOffsetCbfConstraints cbf;
  cbf.cbf_gamma_pair = 2.0;
  const PhaseOffsetGeometry g = makeGeometry();
  const Eigen::Vector3d self(2.0, 0.0, 1.0);
  PredictedNeighborState n;
  n.robot_id = 1;
  n.predicted_position = Eigen::Vector3d(2.0, 0.3, 1.0);  // close
  n.advertised_velocity = Eigen::Vector3d::Zero();
  n.velocity_uncertainty_bound = 0.2;
  std::vector<PredictedNeighborState,
              Eigen::aligned_allocator<PredictedNeighborState>> neigh;
  neigh.push_back(n);
  std::vector<LinearConstraint2D> cons;
  cbf.appendPairwiseConstraints(g, self, neigh, cons);
  const LinearConstraint2D& c = cons[0];
  // The unconstrained zero port must violate the CBF for a close neighbor.
  EXPECT_LT(c.a.dot(Eigen::Vector2d::Zero()), c.b);
  // A port that moves the reference away must satisfy it: u_delta negative
  // pushes the active reference away from the neighbor (N = +y).
  Eigen::Vector2d u_away(0.0, -3.0);
  EXPECT_GE(c.a.dot(u_away), c.b - 1e-9);
}

}  // namespace

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
