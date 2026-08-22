#include <gtest/gtest.h>

#include <cmath>
#include <set>

#include <ros/ros.h>

#include <bspline_race/swarm_neighbor_model.h>

namespace
{

using FLAG_Race::ElasticSwarmIntent;
using FLAG_Race::NeighborSelectionParams;
using FLAG_Race::NeighborSelector;
using FLAG_Race::NeighborState;
using FLAG_Race::NeighborStateBuffer;
using FLAG_Race::PredictedNeighborState;
using FLAG_Race::SwarmIntent;
using FLAG_Race::SwarmIntentParams;

const double SQRT3 = std::sqrt(3.0);

// Hexagon from the plan (d = 1.0 m).
const std::vector<Eigen::Vector3d> kHex = {
  Eigen::Vector3d(-0.5, SQRT3 / 2.0, 1.0),  // A / 0
  Eigen::Vector3d(0.5, SQRT3 / 2.0, 1.0),   // B / 1
  Eigen::Vector3d(-1.0, 0.0, 1.0),          // C / 2
  Eigen::Vector3d(0.0, 0.0, 1.0),           // D / 3
  Eigen::Vector3d(1.0, 0.0, 1.0),           // E / 4
  Eigen::Vector3d(-0.5, -SQRT3 / 2.0, 1.0), // F / 5
  Eigen::Vector3d(0.5, -SQRT3 / 2.0, 1.0),  // G / 6
};

std::vector<NeighborState> makeNeighbors(
    const std::vector<int>& ids, const ros::Time& now)
{
  std::vector<NeighborState> out;
  for (int id : ids)
  {
    NeighborState s;
    s.robot_id = id;
    s.stamp = now;
    s.position = kHex[id];
    s.velocity = Eigen::Vector3d::Zero();
    s.fresh = true;
    out.push_back(s);
  }
  return out;
}

NeighborSelectionParams makeParams()
{
  NeighborSelectionParams p;
  p.organization_enter_radius = 1.55;
  p.organization_exit_radius = 1.65;
  p.safety_radius = 1.40;
  p.ttc_safe = 2.00;
  p.neighbor_timeout = 0.30;
  p.neighbor_retention_timeout = 1.00;
  p.use_los = false;
  return p;
}

SwarmIntentParams makeIntentParams()
{
  SwarmIntentParams p;
  p.d_minus = 0.80;
  p.d_plus = 1.20;
  p.k_repulsion = 1.20;
  p.k_cohesion = 0.25;
  p.k_damping = 0.80;
  p.soft_safety_gain = 0.50;
  p.soft_safety_max = 0.80;
  p.k_recenter = 0.40;
  return p;
}

std::set<int> orgIds(const FLAG_Race::NeighborSelection& s)
{
  std::set<int> ids;
  for (const NeighborState& n : s.organization)
    ids.insert(n.robot_id);
  return ids;
}

TEST(SwarmNeighborModel, SevenUavNeighborTableMatchesProposal)
{
  NeighborSelector sel;
  const ros::Time now = ros::Time(1000.0);
  for (int i = 0; i < 7; ++i)
  {
    std::vector<int> others;
    for (int j = 0; j < 7; ++j)
      if (j != i)
        others.push_back(j);
    auto sel_out = sel.select(now, kHex[i], Eigen::Vector3d::Zero(),
                              makeNeighbors(others, now), {}, nullptr,
                              makeParams());
    std::set<int> ids = orgIds(sel_out);
    std::set<int> expected;
    if (i == 0) expected = {1, 2, 3};
    if (i == 1) expected = {0, 3, 4};
    if (i == 2) expected = {0, 3, 5};
    if (i == 3) expected = {0, 1, 2, 4, 5, 6};
    if (i == 4) expected = {1, 3, 6};
    if (i == 5) expected = {2, 3, 6};
    if (i == 6) expected = {3, 4, 5};
    EXPECT_EQ(expected, ids) << "robot " << i;
  }
}

TEST(SwarmNeighborModel, ACompressedNeighborsGiveOutwardIntent)
{
  NeighborSelector sel;
  ElasticSwarmIntent intent_model;
  const ros::Time now = ros::Time(1000.0);
  // Three artificial neighbors at equal distance d_test < d_minus along the
  // AB / AC / AD directions (per the plan's stage-4 test).
  const double d_test = 0.6;
  const Eigen::Vector3d self = kHex[0];
  Eigen::Vector3d n_ab = (kHex[1] - kHex[0]).normalized();
  Eigen::Vector3d n_ac = (kHex[2] - kHex[0]).normalized();
  Eigen::Vector3d n_ad = (kHex[3] - kHex[0]).normalized();
  std::vector<NeighborState> neighbors;
  for (int k = 0; k < 3; ++k)
  {
    NeighborState s;
    s.robot_id = 10 + k;
    s.stamp = now;
    const Eigen::Vector3d dir =
        k == 0 ? n_ab : (k == 1 ? n_ac : n_ad);
    s.position = self + dir * d_test;
    s.velocity = Eigen::Vector3d::Zero();
    s.fresh = true;
    neighbors.push_back(s);
  }
  auto sel_out = sel.select(now, self, Eigen::Vector3d::Zero(), neighbors, {},
                            nullptr, makeParams());
  SwarmIntent intent = intent_model.compute(
      self, Eigen::Vector3d::Zero(), 1.0, sel_out, {}, nullptr,
      makeIntentParams());
  // All edges compressed -> net position intent points outward along
  // o_A = -(n_AB + n_AC + n_AD)/|...|.
  const Eigen::Vector3d o_A =
    -(n_ab + n_ac + n_ad).normalized();
  EXPECT_GT(intent.g_pos.dot(o_A), 0.0);
  EXPECT_GT(intent.g_pos.norm(), 1e-3);
}

TEST(SwarmNeighborModel, DSymmetricPositionIntentCancels)
{
  NeighborSelector sel;
  ElasticSwarmIntent intent_model;
  const ros::Time now = ros::Time(1000.0);
  auto sel_out = sel.select(now, kHex[3], Eigen::Vector3d::Zero(),
                            makeNeighbors({0, 1, 2, 4, 5, 6}, now), {},
                            nullptr, makeParams());
  SwarmIntent intent = intent_model.compute(
      kHex[3], Eigen::Vector3d::Zero(), 1.0, sel_out, {}, nullptr,
      makeIntentParams());
  EXPECT_LT(intent.g_pos.norm(), 1e-9);
}

TEST(SwarmNeighborModel, EIsNotDirectOrganizationNeighborOfC)
{
  NeighborSelector sel;
  const ros::Time now = ros::Time(1000.0);
  auto sel_out = sel.select(now, kHex[2], Eigen::Vector3d::Zero(),
                            makeNeighbors({0, 1, 3, 4, 5, 6}, now), {},
                            nullptr, makeParams());
  EXPECT_EQ(0, orgIds(sel_out).count(4));  // E / robot 4 not in C's org set
}

TEST(SwarmNeighborModel, DistanceBandBehavior)
{
  NeighborSelector sel;
  ElasticSwarmIntent intent_model;
  SwarmIntentParams p = makeIntentParams();
  const ros::Time now = ros::Time(1000.0);

  // Two robots along +x at distance d; self at origin.
  auto run = [&](double d) {
    NeighborState n;
    n.robot_id = 9;
    n.stamp = now;
    n.position = Eigen::Vector3d(d, 0.0, 1.0);
    n.velocity = Eigen::Vector3d::Zero();
    n.fresh = true;
    auto sel_out = sel.select(now, Eigen::Vector3d(0, 0, 1.0),
                              Eigen::Vector3d::Zero(), {n}, {}, nullptr,
                              makeParams());
    return intent_model.compute(Eigen::Vector3d(0, 0, 1.0),
                                Eigen::Vector3d::Zero(), 1.0, sel_out, {},
                                nullptr, p);
  };

  SwarmIntent rep = run(0.5);   // d < d_minus
  SwarmIntent dead = run(1.0);  // d in [d_minus, d_plus]
  SwarmIntent coh = run(1.4);   // d > d_plus, < Rc
  EXPECT_LT(rep.g_pos.x(), 0.0);  // repulsion: -x
  EXPECT_NEAR(0.0, dead.g_pos.norm(), 1e-12);
  EXPECT_GT(coh.g_pos.x(), 0.0);  // weak cohesion: +x
}

TEST(SwarmNeighborModel, BetaZeroOnlyDisablesWeakCohesion)
{
  NeighborSelector sel;
  ElasticSwarmIntent intent_model;
  const ros::Time now = ros::Time(1000.0);
  NeighborState n;
  n.robot_id = 9;
  n.stamp = now;
  n.position = Eigen::Vector3d(1.4, 0.0, 1.0);
  n.velocity = Eigen::Vector3d::Zero();
  n.fresh = true;
  auto sel_out = sel.select(now, Eigen::Vector3d(0, 0, 1.0),
                            Eigen::Vector3d::Zero(), {n}, {}, nullptr,
                            makeParams());
  SwarmIntent with_beta = intent_model.compute(
      Eigen::Vector3d(0, 0, 1.0), Eigen::Vector3d::Zero(), 1.0, sel_out, {},
      nullptr, makeIntentParams());
  SwarmIntent no_beta = intent_model.compute(
      Eigen::Vector3d(0, 0, 1.0), Eigen::Vector3d::Zero(), 0.0, sel_out, {},
      nullptr, makeIntentParams());
  EXPECT_GT(with_beta.g_pos.x(), 0.0);
  EXPECT_NEAR(0.0, no_beta.g_pos.x(), 1e-12);
  // Repulsion at d < d_minus is NOT attenuated by beta.
  n.position = Eigen::Vector3d(0.5, 0.0, 1.0);
  auto sel_rep = sel.select(now, Eigen::Vector3d(0, 0, 1.0),
                            Eigen::Vector3d::Zero(), {n}, {}, nullptr,
                            makeParams());
  SwarmIntent rep_no_beta = intent_model.compute(
      Eigen::Vector3d(0, 0, 1.0), Eigen::Vector3d::Zero(), 0.0, sel_rep, {},
      nullptr, makeIntentParams());
  EXPECT_LT(rep_no_beta.g_pos.x(), 0.0);
}

TEST(SwarmNeighborModel, CommonTranslationGivesZeroDamping)
{
  NeighborSelector sel;
  ElasticSwarmIntent intent_model;
  const ros::Time now = ros::Time(1000.0);
  NeighborState n;
  n.robot_id = 9;
  n.stamp = now;
  n.position = Eigen::Vector3d(1.0, 0.0, 1.0);
  n.velocity = Eigen::Vector3d(1.0, 0.5, 0.0);
  n.fresh = true;
  const Eigen::Vector3d self_vel(1.0, 0.5, 0.0);
  auto sel_out = sel.select(now, Eigen::Vector3d(0, 0, 1.0), self_vel, {n},
                            {}, nullptr, makeParams());
  SwarmIntent intent = intent_model.compute(
      Eigen::Vector3d(0, 0, 1.0), self_vel, 1.0, sel_out, {}, nullptr,
      makeIntentParams());
  EXPECT_NEAR(0.0, intent.g_damp.norm(), 1e-12);
}

TEST(SwarmNeighborModel, RelativeExpansionDampingSign)
{
  NeighborSelector sel;
  ElasticSwarmIntent intent_model;
  const ros::Time now = ros::Time(1000.0);
  NeighborState n;
  n.robot_id = 9;
  n.stamp = now;
  n.position = Eigen::Vector3d(1.0, 0.0, 1.0);
  n.velocity = Eigen::Vector3d(1.0, 0.0, 0.0);  // neighbor moves away
  n.fresh = true;
  auto sel_out = sel.select(now, Eigen::Vector3d(0, 0, 1.0),
                            Eigen::Vector3d::Zero(), {n}, {}, nullptr,
                            makeParams());
  SwarmIntent intent = intent_model.compute(
      Eigen::Vector3d(0, 0, 1.0), Eigen::Vector3d::Zero(), 1.0, sel_out, {},
      nullptr, makeIntentParams());
  // Expanding pair -> damping pulls self toward the neighbor (+x).
  EXPECT_GT(intent.g_damp.x(), 0.0);
}

TEST(SwarmNeighborModel, StaleStateKeepsSafetyAndFlagsFault)
{
  NeighborSelector sel;
  const ros::Time now = ros::Time(1000.0);
  NeighborState n;
  n.robot_id = 9;
  n.stamp = now - ros::Duration(0.5);  // stale (> timeout 0.3)
  n.position = Eigen::Vector3d(0.6, 0.0, 1.0);
  n.velocity = Eigen::Vector3d::Zero();
  n.fresh = false;
  auto sel_out = sel.select(now, Eigen::Vector3d(0, 0, 1.0),
                            Eigen::Vector3d::Zero(), {n}, {}, nullptr,
                            makeParams());
  EXPECT_EQ(1u, sel_out.safety.size());
  EXPECT_TRUE(sel_out.communication_fault);
  EXPECT_EQ(0u, sel_out.organization.size());
}

}  // namespace

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
