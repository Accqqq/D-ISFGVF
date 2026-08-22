#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "phase_offset_swarm/neighbor_manager.h"
#include "phase_offset_swarm/swarm_intent.h"

namespace {

using phase_offset_swarm::NeighborFreshness;
using phase_offset_swarm::NeighborManager;
using phase_offset_swarm::NeighborSnapshot;
using phase_offset_swarm::NeighborState;
using phase_offset_swarm::SwarmAgentState;
using phase_offset_swarm::SwarmIntentCalculator;
using phase_offset_swarm::SwarmParameters;

SwarmParameters params() {
  SwarmParameters result;
  result.r_comm = 2.0;
  result.r_conf = 1.5;
  result.r_safe = 1.2;
  result.d_minus = 0.8;
  result.d_plus = 1.2;
  result.k_sep = 1.0;
  result.k_coh = 0.2;
  result.k_conf = 0.8;
  result.g_max = 0.5;
  result.neighbor_hysteresis = 0.1;
  return result;
}

SwarmAgentState self(int id = 1) {
  SwarmAgentState result;
  result.id = id;
  return result;
}

NeighborState neighbor(int id, double x, double y = 0.0, double vx = 0.0,
                       double vy = 0.0,
                       NeighborFreshness freshness = NeighborFreshness::FRESH) {
  NeighborState result;
  result.id = id;
  result.position = Eigen::Vector2d(x, y);
  result.predicted_position = result.position;
  result.velocity = Eigen::Vector2d(vx, vy);
  result.freshness = freshness;
  return result;
}

NeighborSnapshot org(std::initializer_list<NeighborState> states) {
  NeighborSnapshot result;
  for (const NeighborState& state : states) {
    result.organization.push_back(state);
  }
  return result;
}

NeighborSnapshot conflict(std::initializer_list<NeighborState> states) {
  NeighborSnapshot result;
  for (const NeighborState& state : states) {
    result.conflict.push_back(state);
  }
  return result;
}

TEST(SwarmIntent, TooCloseAndComfortBandCases) {
  SwarmIntentCalculator calculator(params());
  const auto close = calculator.compute(self(), org({neighbor(2, 0.5)}), 1.0);
  EXPECT_LT(close.g_sep.x(), 0.0);
  EXPECT_LT(close.g_coord.x(), 0.0);
  const auto comfort = calculator.compute(self(), org({neighbor(2, 1.0)}), 1.0);
  EXPECT_NEAR(0.0, comfort.g_sep.norm(), 1e-12);
  EXPECT_NEAR(0.0, comfort.g_coh.norm(), 1e-12);
  EXPECT_NEAR(0.0, comfort.g_conf.norm(), 1e-12);
}

TEST(SwarmIntent, WeakCohesionAndLocalBeta) {
  SwarmIntentCalculator calculator(params());
  const auto full = calculator.compute(self(), org({neighbor(2, 1.5)}), 1.0);
  const auto zero = calculator.compute(self(), org({neighbor(2, 1.5)}), 0.0);
  EXPECT_GT(full.g_coh.x(), 0.0);
  EXPECT_NEAR(0.0, zero.g_coh.norm(), 1e-12);
  EXPECT_NEAR(0.0, full.g_sep.norm(), 1e-12);
  EXPECT_NEAR(0.0, zero.g_sep.norm(), 1e-12);
}

TEST(SwarmIntent, NonFiniteBetaFailsClosedToZero) {
  SwarmIntentCalculator calculator(params());
  const auto nan_output = calculator.compute(
      self(), org({neighbor(2, 1.5)}),
      std::numeric_limits<double>::quiet_NaN());
  const auto inf_output = calculator.compute(
      self(), org({neighbor(2, 1.5)}),
      std::numeric_limits<double>::infinity());
  EXPECT_NEAR(0.0, nan_output.g_coh.norm(), 1e-12);
  EXPECT_NEAR(0.0, inf_output.g_coh.norm(), 1e-12);
  EXPECT_TRUE(nan_output.g_coord.allFinite());
  EXPECT_TRUE(inf_output.g_coord.allFinite());
}

TEST(SwarmIntent, ConflictCases) {
  SwarmIntentCalculator calculator(params());
  const auto approaching = calculator.compute(
      self(), conflict({neighbor(2, 1.0, 0.0, -1.0, 0.0)}), 1.0);
  EXPECT_LT(approaching.g_conf.x(), 0.0);
  const auto separating = calculator.compute(
      self(), conflict({neighbor(2, 1.0, 0.0, 1.0, 0.0)}), 1.0);
  EXPECT_NEAR(0.0, separating.g_conf.norm(), 1e-12);
  const auto translating = calculator.compute(
      self(),
      conflict({neighbor(2, 1.0, 0.0, 0.0, 0.0)}), 1.0);
  EXPECT_NEAR(0.0, translating.g_conf.norm(), 1e-12);
}

TEST(SwarmIntent, MultipleNeighborsAggregate) {
  SwarmIntentCalculator calculator(params());
  NeighborSnapshot snapshot = org({neighbor(2, 0.5), neighbor(3, -0.5)});
  const auto output = calculator.compute(self(), snapshot, 1.0);
  EXPECT_EQ(2, output.num_org_neighbors);
  EXPECT_TRUE(output.g_sep.allFinite());
}

TEST(SwarmIntent, StaleAndZeroDistanceAreRobust) {
  SwarmParameters p = params();
  p.g_max = 2.0;
  SwarmIntentCalculator calculator(p);
  NeighborSnapshot snapshot = org({neighbor(2, 0.0, 0.0, 0.0, 0.0),
                                   neighbor(3, 0.5, 0.0, 0.0, 0.0,
                                           NeighborFreshness::STALE)});
  snapshot.fresh_count = 0;
  snapshot.stale_count = 1;
  snapshot.lost_count = 0;
  const auto output = calculator.compute(self(), snapshot, 1.0);
  EXPECT_TRUE(output.g_coord.allFinite());
  EXPECT_TRUE(output.g_sep.allFinite());
}

TEST(SwarmIntent, FinalSaturationOnlyScalesRawSum) {
  SwarmParameters p = params();
  p.g_max = 0.25;
  SwarmIntentCalculator calculator(p);
  const auto output = calculator.compute(
      self(), org({neighbor(2, 0.1), neighbor(3, 0.2)}), 1.0);
  EXPECT_TRUE(output.output_saturated);
  EXPECT_NEAR(p.g_max, output.g_coord.norm(), 1e-12);
  EXPECT_GT(output.g_sep.norm(), p.g_max);
}

TEST(SwarmIntent, UnsaturatedComponentsSumExactly) {
  SwarmParameters p = params();
  p.g_max = 10.0;
  SwarmIntentCalculator calculator(p);
  NeighborSnapshot snapshot = org({neighbor(2, 0.5)});
  snapshot.conflict.push_back(neighbor(3, 1.0, 0.0, -1.0, 0.0));
  const auto output = calculator.compute(self(), snapshot, 0.7);
  EXPECT_FALSE(output.output_saturated);
  EXPECT_TRUE((output.g_coord - output.g_sep - output.g_coh - output.g_conf)
                  .norm() < 1e-12);
}

TEST(SwarmIntent, DiagnosticsUseFreshPredictedNeighborsOnly) {
  SwarmParameters p = params();
  p.g_max = 10.0;
  SwarmIntentCalculator calculator(p);
  NeighborSnapshot snapshot;
  snapshot.all.push_back(neighbor(2, 2.0, 0.0, -1.0, 0.0));
  snapshot.all.push_back(neighbor(3, 0.5, 0.0, 0.0, 0.0,
                                  NeighborFreshness::STALE));
  snapshot.fresh_count = 1;
  snapshot.stale_count = 1;
  const auto output = calculator.compute(self(), snapshot, 1.0);
  EXPECT_NEAR(2.0, output.min_distance, 1e-12);
  EXPECT_TRUE(std::isfinite(output.min_ttc));
}

TEST(SwarmIntent, InvalidParametersAreRejected) {
  SwarmParameters invalid = params();
  invalid.d_plus = invalid.d_minus;
  EXPECT_THROW(SwarmIntentCalculator calculator(invalid), std::invalid_argument);
}

TEST(SwarmIntent, OutputFiniteExceptPermittedInfinitySentinels) {
  SwarmIntentCalculator calculator(params());
  const auto output = calculator.compute(self(), NeighborSnapshot(), 5.0);
  EXPECT_TRUE(output.g_coord.allFinite());
  EXPECT_TRUE(output.g_sep.allFinite());
  EXPECT_TRUE(output.g_coh.allFinite());
  EXPECT_TRUE(output.g_conf.allFinite());
  EXPECT_TRUE(std::isinf(output.min_distance));
  EXPECT_TRUE(std::isinf(output.min_ttc));
}

TEST(SwarmIntent, ManagerIntegrationCountsFreshAndStale) {
  NeighborManager manager(1, params());
  SwarmAgentState incoming = self(2);
  incoming.stamp = 0.0;
  incoming.position = Eigen::Vector2d(0.5, 0.0);
  ASSERT_TRUE(manager.update(incoming));
  const auto snapshot = manager.snapshot(self(), 0.4);
  EXPECT_EQ(1, snapshot.stale_count);
  SwarmIntentCalculator calculator(params());
  const auto output = calculator.compute(self(), snapshot, 1.0);
  EXPECT_EQ(0, output.num_org_neighbors);
  EXPECT_EQ(1, output.num_stale_neighbors);
  EXPECT_NEAR(0.0, output.g_coord.norm(), 1e-12);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
