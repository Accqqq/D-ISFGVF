#include <gtest/gtest.h>

#include <cmath>

#include <bspline_race/phase_offset_allocator.h>
#include <bspline_race/continuous_phase_path.h>
#include <bspline_race/phase_offset_geometry.h>

namespace
{

using FLAG_Race::AllocatorParams;
using FLAG_Race::ContinuousPhasePathState;
using FLAG_Race::LinearConstraint2D;
using FLAG_Race::PhaseOffsetAllocator;
using FLAG_Race::PhaseOffsetGeometry;
using FLAG_Race::PhaseOffsetGeometryEvaluator;
using FLAG_Race::PhaseOffsetGeometryParams;
using FLAG_Race::PortCommand;
using FLAG_Race::SwarmControlMode;
using FLAG_Race::SwarmIntent;
using FLAG_Race::TubeBounds;

ContinuousPhasePathState makeLineState(double w)
{
  ContinuousPhasePathState s;
  s.p = Eigen::Vector3d(w, 0.0, 1.0);
  s.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
  s.d2p_dw2 = Eigen::Vector3d::Zero();
  return s;
}

PhaseOffsetGeometry makeGeometry(double delta = 0.3, bool on_reference = false)
{
  PhaseOffsetGeometryEvaluator ev;
  PhaseOffsetGeometryParams gp;
  gp.mu_regular = 0.20;
  gp.max_delta = 5.0;
  PhaseOffsetGeometry g;
  const Eigen::Vector3d pos =
    on_reference ? Eigen::Vector3d(2.0, delta, 1.0)
                 : Eigen::Vector3d(2.3, 0.1, 1.0);
  EXPECT_TRUE(ev.evaluate(makeLineState(2.0), pos, delta, gp, g));
  return g;
}

TubeBounds makeTube(double lower = -0.8, double upper = 0.8)
{
  TubeBounds t;
  t.lower = lower;
  t.upper = upper;
  t.beta = 1.0;
  t.valid = true;
  return t;
}

SwarmIntent makeIntent(const Eigen::Vector3d& g)
{
  SwarmIntent intent;
  intent.g_swarm = g;
  intent.g_des = g;
  return intent;
}

AllocatorParams zeroRegularization()
{
  AllocatorParams p;
  p.weight_world_error = 1.0;
  p.weight_u_w = 0.0;
  p.weight_u_delta = 0.0;
  p.weight_delta_u_w = 0.0;
  p.weight_delta_u_delta = 0.0;
  return p;
}

AllocatorParams testParams()
{
  AllocatorParams p = zeroRegularization();
  p.phase_speed_max = 3.0;
  p.tangent_speed_max = 3.0;
  p.phase_speed_min = 0.1;
  return p;
}

TEST(PhaseOffsetAllocator, UnconstrainedMatchesAnalyticRawPort)
{
  PhaseOffsetAllocator alloc;
  alloc.params = zeroRegularization();
  const PhaseOffsetGeometry g = makeGeometry();
  const Eigen::Vector3d g_des(0.2, 0.5, 0.0);
  PortCommand prev;
  const Eigen::Vector2d u0 = alloc.unconstrainedSolution(g, g_des,
                                                         alloc.params, prev);
  const Eigen::Vector2d uraw = alloc.analyticRawPort(g, g_des);
  EXPECT_LT((u0 - uraw).norm(), 1e-9);
}

TEST(PhaseOffsetAllocator, SingleConstraintProjection)
{
  PhaseOffsetAllocator alloc;
  alloc.params = testParams();
  const PhaseOffsetGeometry g = makeGeometry(0.3, true);
  const Eigen::Vector3d g_des(0.0, 0.5, 0.0);  // u_w raw = 0
  PortCommand prev;
  prev.u_w = 0.3;
  prev.u_delta = 0.5;
  const PortCommand port = alloc.solve(
      g, makeTube(), makeIntent(g_des), {}, 0.02, prev,
      SwarmControlMode::ROLLING,
      {LinearConstraint2D{Eigen::Vector2d(1.0, 0.0), 0.3, "uw_min_test"}});
  ASSERT_TRUE(port.feasible);
  EXPECT_NEAR(0.3, port.u_w, 1e-6);  // on the active boundary
}

TEST(PhaseOffsetAllocator, TwoConstraintIntersection)
{
  PhaseOffsetAllocator alloc;
  alloc.params = testParams();
  const PhaseOffsetGeometry g = makeGeometry(0.3, true);
  PortCommand prev;
  prev.u_w = 0.4;
  prev.u_delta = -0.2;
  const PortCommand port = alloc.solve(
      g, makeTube(), makeIntent(Eigen::Vector3d(0.0, 0.0, 0.0)), {}, 0.02,
      prev, SwarmControlMode::ROLLING,
      {LinearConstraint2D{Eigen::Vector2d(1.0, 0.0), 0.4, "a"},
       LinearConstraint2D{Eigen::Vector2d(0.0, -1.0), 0.2, "b"}});
  ASSERT_TRUE(port.feasible);
  EXPECT_NEAR(0.4, port.u_w, 1e-6);
  EXPECT_NEAR(-0.2, port.u_delta, 1e-6);
}

TEST(PhaseOffsetAllocator, MultiConstraintBestCandidate)
{
  PhaseOffsetAllocator alloc;
  alloc.params = testParams();
  const PhaseOffsetGeometry g = makeGeometry(0.3, true);
  PortCommand prev;
  prev.u_w = 0.5;
  prev.u_delta = -0.1;
  // The unconstrained optimum satisfies u_w=0.6; add constraints that force
  // u_w into [0.2, 0.5] and u_delta = -0.1: the best feasible candidate is
  // the intersection of the two box edges closest to the optimum.
  const PortCommand port = alloc.solve(
      g, makeTube(), makeIntent(Eigen::Vector3d(0.6, -0.1, 0.0)), {}, 0.02,
      prev, SwarmControlMode::ROLLING,
      {LinearConstraint2D{Eigen::Vector2d(1.0, 0.0), 0.2, "uw_low"},
       LinearConstraint2D{Eigen::Vector2d(-1.0, 0.0), -0.5, "uw_high"},
       LinearConstraint2D{Eigen::Vector2d(0.0, 1.0), -0.1, "ud"},
       LinearConstraint2D{Eigen::Vector2d(0.0, -1.0), 0.1, "ud_hi"}});
  ASSERT_TRUE(port.feasible);
  EXPECT_NEAR(0.5, port.u_w, 1e-6);
  EXPECT_NEAR(-0.1, port.u_delta, 1e-6);
}

TEST(PhaseOffsetAllocator, RollingFeasible)
{
  PhaseOffsetAllocator alloc;
  alloc.params = testParams();
  const PhaseOffsetGeometry g = makeGeometry(0.3, true);
  PortCommand prev;
  const PortCommand port = alloc.solve(
      g, makeTube(), makeIntent(Eigen::Vector3d(0.1, 0.1, 0.0)), {}, 0.02,
      prev, SwarmControlMode::ROLLING);
  ASSERT_TRUE(port.feasible);
  EXPECT_EQ(SwarmControlMode::ROLLING, port.mode);
}

TEST(PhaseOffsetAllocator, OnlyNonnegativeFeasibleGivesSafetyPriority)
{
  PhaseOffsetAllocator alloc;
  alloc.params = testParams();
  alloc.params.phase_speed_min = 0.5;
  const PhaseOffsetGeometry g = makeGeometry(0.3, true);
  PortCommand prev;
  prev.u_w = 0.0;
  // w_dot_base is small (line at alpha~1, K1=2: w_dot ~ 2); phase_min needs
  // u_w >= 0.5 - w_dot_base < 0 -> satisfied... instead force infeasibility
  // of the rolling set with a tight slew rate and a previous port far below
  // the required u_w: make w_dot_base small by using a slow path? We keep the
  // line (w_dot ~ 2) so phase_min is trivially satisfied. To make rolling
  // infeasible we add a custom constraint that only the nonneg set lacks:
  // rolling has tangent_min; choose rw_norm and alpha so tangent_min binds
  // beyond the slew limit.
  alloc.params.u_w_slew_rate = 0.01;  // ~0.0002 per step
  prev.u_w = -2.0;
  const PortCommand port = alloc.solve(
      g, makeTube(), makeIntent(Eigen::Vector3d(0.0, 0.0, 0.0)), {}, 0.02,
      prev, SwarmControlMode::ROLLING);
  // The slew constraint allows u_w in [-2.0002, -1.9998]; phase_min requires
  // u_w >= 0.5 - w_dot_base(~2) = -1.5 -> infeasible for rolling. The
  // nonneg set only requires u_w >= -w_dot_base = -2 -> feasible.
  ASSERT_TRUE(port.feasible);
  EXPECT_EQ(SwarmControlMode::SAFETY_PRIORITY, port.mode);
}

TEST(PhaseOffsetAllocator, BothInfeasibleGivesEmergency)
{
  PhaseOffsetAllocator alloc;
  alloc.params = zeroRegularization();
  const PhaseOffsetGeometry g = makeGeometry(0.79);  // near upper bound
  TubeBounds tube = makeTube(-0.8, 0.8);
  PortCommand prev;
  prev.u_delta = 1.0;
  alloc.params.u_delta_slew_rate = 0.01;
  // delta at 0.79: delta_upper -> u_delta <= (0.8-0.79)/dt ~ 0.5 (dt=0.02)
  // slew low -> u_delta >= 1 - 0.0002 = 0.9998 -> contradiction.
  const PortCommand port = alloc.solve(
      g, tube, makeIntent(Eigen::Vector3d(0.0, 0.0, 0.0)), {}, 0.02, prev,
      SwarmControlMode::ROLLING);
  EXPECT_FALSE(port.feasible);
  EXPECT_EQ(SwarmControlMode::EMERGENCY, port.mode);
}

TEST(PhaseOffsetAllocator, PhaseNeverReversesInRolling)
{
  PhaseOffsetAllocator alloc;
  alloc.params = testParams();
  alloc.params.phase_speed_min = 0.1;
  const PhaseOffsetGeometry g = makeGeometry(0.3, true);
  PortCommand prev;
  // Intent pushing the phase backwards.
  const PortCommand port = alloc.solve(
      g, makeTube(), makeIntent(Eigen::Vector3d(-1.0, 0.0, 0.0)), {}, 0.02,
      prev, SwarmControlMode::ROLLING);
  ASSERT_TRUE(port.feasible);
  EXPECT_GE(g.base_w_dot + port.u_w, alloc.params.phase_speed_min - 1e-6);
}

TEST(PhaseOffsetAllocator, TangentMarginHolds)
{
  PhaseOffsetAllocator alloc;
  alloc.params = testParams();
  const PhaseOffsetGeometry g = makeGeometry(0.3, true);
  PortCommand prev;
  const PortCommand port = alloc.solve(
      g, makeTube(), makeIntent(Eigen::Vector3d(-1.0, 0.0, 0.0)), {}, 0.02,
      prev, SwarmControlMode::ROLLING);
  ASSERT_TRUE(port.feasible);
  const double rw = g.r_w.norm();
  EXPECT_GE(alloc.params.K1 * g.alpha + rw * port.u_w,
            alloc.params.tangent_speed_min - 1e-6);
}

TEST(PhaseOffsetAllocator, TerminalModeAllowsPhaseDecay)
{
  PhaseOffsetAllocator alloc;
  alloc.params = testParams();
  alloc.params.phase_speed_min = 0.5;
  const PhaseOffsetGeometry g = makeGeometry(0.3, true);
  PortCommand prev;
  prev.u_w = -1.9;
  const PortCommand port = alloc.solve(
      g, makeTube(), makeIntent(Eigen::Vector3d(-1.0, 0.0, 0.0)), {}, 0.02,
      prev, SwarmControlMode::TERMINAL);
  ASSERT_TRUE(port.feasible);
  EXPECT_EQ(SwarmControlMode::TERMINAL, port.mode);
  EXPECT_GE(g.base_w_dot + port.u_w, -1e-6);  // nonnegative progress
  EXPECT_LT(g.base_w_dot + port.u_w, alloc.params.phase_speed_min + 1e-6);
}

}  // namespace

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
