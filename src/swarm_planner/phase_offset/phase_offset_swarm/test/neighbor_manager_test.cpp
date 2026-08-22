#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "phase_offset_swarm/neighbor_manager.h"

namespace {

using phase_offset_swarm::NeighborFreshness;
using phase_offset_swarm::NeighborManager;
using phase_offset_swarm::SwarmAgentState;
using phase_offset_swarm::SwarmParameters;

SwarmAgentState state(int id, double stamp, double x, double y, double vx,
                      double vy) {
  SwarmAgentState result;
  result.id = id;
  result.stamp = stamp;
  result.position = Eigen::Vector2d(x, y);
  result.velocity = Eigen::Vector2d(vx, vy);
  return result;
}

SwarmParameters params() {
  SwarmParameters result;
  result.r_comm = 2.0;
  result.r_conf = 1.5;
  result.r_safe = 1.2;
  result.neighbor_hysteresis = 0.1;
  result.fresh_timeout = 0.3;
  result.stale_timeout = 0.6;
  result.lost_retention_timeout = 1.5;
  return result;
}

SwarmAgentState self() { return state(1, 0.0, 0.0, 0.0, 0.0, 0.0); }

TEST(NeighborManager, PredictsConstantVelocity) {
  NeighborManager manager(1, params());
  ASSERT_TRUE(manager.update(state(2, 10.0, 1.0, -2.0, 0.5, 2.0)));
  const auto snapshot = manager.snapshot(self(), 10.2);
  ASSERT_EQ(1u, snapshot.all.size());
  EXPECT_NEAR(0.2, snapshot.all.front().message_age, 1e-12);
  EXPECT_NEAR(1.1, snapshot.all.front().predicted_position.x(), 1e-12);
  EXPECT_NEAR(-1.6, snapshot.all.front().predicted_position.y(), 1e-12);
  EXPECT_EQ(NeighborFreshness::FRESH, snapshot.all.front().freshness);
}

TEST(NeighborManager, RejectsSelfAndInvalidStates) {
  NeighborManager manager(1, params());
  EXPECT_FALSE(manager.update(state(1, 1.0, 0.0, 0.0, 0.0, 0.0)));
  SwarmAgentState invalid = state(2, 1.0, 0.0, 0.0, 0.0, 0.0);
  invalid.position.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(manager.update(invalid));
  invalid = state(2, 1.0, 0.0, 0.0, 0.0, 0.0);
  invalid.id = -1;
  EXPECT_FALSE(manager.update(invalid));
}

TEST(NeighborManager, RejectsOlderOrEqualTimestamp) {
  NeighborManager manager(1, params());
  ASSERT_TRUE(manager.update(state(2, 2.0, 1.0, 0.0, 0.0, 0.0)));
  EXPECT_FALSE(manager.update(state(2, 2.0, 2.0, 0.0, 0.0, 0.0)));
  EXPECT_FALSE(manager.update(state(2, 1.0, 3.0, 0.0, 0.0, 0.0)));
  ASSERT_TRUE(manager.update(state(2, 3.0, 4.0, 0.0, 0.0, 0.0)));
  EXPECT_NEAR(4.0, manager.snapshot(self(), 3.0).all.front().position.x(),
              1e-12);
}

TEST(NeighborManager, FreshStaleLostAndRetentionExpiry) {
  NeighborManager manager(1, params());
  ASSERT_TRUE(manager.update(state(2, 0.0, 0.5, 0.0, 0.0, 0.0)));
  EXPECT_EQ(1, manager.snapshot(self(), 0.3).fresh_count);
  EXPECT_EQ(1, manager.snapshot(self(), 0.300001).stale_count);
  EXPECT_EQ(1, manager.snapshot(self(), 0.600001).lost_count);
  EXPECT_EQ(1u, manager.snapshot(self(), 1.5).all.size());
  EXPECT_TRUE(manager.snapshot(self(), 1.500001).all.empty());
}

TEST(NeighborManager, OrganizationDistanceHysteresis) {
  NeighborManager manager(1, params());
  ASSERT_TRUE(manager.update(state(2, 0.0, 1.9, 0.0, 0.0, 0.0)));
  EXPECT_EQ(1u, manager.snapshot(self(), 0.0).organization.size());
  ASSERT_TRUE(manager.update(state(2, 0.1, 1.95, 0.0, 0.0, 0.0)));
  EXPECT_EQ(1u, manager.snapshot(self(), 0.1).organization.size());
  ASSERT_TRUE(manager.update(state(2, 0.2, 2.01, 0.0, 0.0, 0.0)));
  EXPECT_EQ(0u, manager.snapshot(self(), 0.2).organization.size());
}

TEST(NeighborManager, StaleStateDoesNotEnterSelectedSets) {
  NeighborManager manager(1, params());
  ASSERT_TRUE(manager.update(state(2, 0.0, 0.5, 0.0, 0.0, 0.0)));
  const auto snapshot = manager.snapshot(self(), 0.4);
  EXPECT_EQ(1, snapshot.stale_count);
  EXPECT_TRUE(snapshot.organization.empty());
  EXPECT_TRUE(snapshot.conflict.empty());
  EXPECT_TRUE(snapshot.safety.empty());
}

TEST(NeighborManager, ConflictCanActivateByDistance) {
  NeighborManager manager(1, params());
  ASSERT_TRUE(manager.update(state(2, 0.0, 1.0, 0.0, -1.0, 0.0)));
  const auto snapshot = manager.snapshot(self(), 0.0);
  ASSERT_EQ(1u, snapshot.conflict.size());
  EXPECT_EQ(1u, snapshot.safety.size());
}

TEST(NeighborManager, ConflictCanActivateByTtcOutsideDistance) {
  NeighborManager manager(1, params());
  ASSERT_TRUE(manager.update(state(2, 0.0, 3.0, 0.0, -2.0, 0.0)));
  const auto snapshot = manager.snapshot(self(), 0.0);
  EXPECT_TRUE(snapshot.organization.empty());
  ASSERT_EQ(1u, snapshot.conflict.size());
  EXPECT_EQ(1u, snapshot.safety.size());
}

TEST(NeighborManager, SafetyDiagnosticsUseDistanceOrTtc) {
  NeighborManager manager(1, params());
  ASSERT_TRUE(manager.update(state(2, 0.0, 1.1, 0.0, 0.0, 0.0)));
  EXPECT_EQ(1u, manager.snapshot(self(), 0.0).safety.size());
}

TEST(NeighborManager, FutureTimestampOutsideToleranceIsLostForSnapshot) {
  NeighborManager manager(1, params());
  ASSERT_TRUE(manager.update(state(2, 10.0, 0.5, 0.0, 0.0, 0.0)));
  const auto snapshot = manager.snapshot(self(), 0.0);
  ASSERT_EQ(1u, snapshot.all.size());
  EXPECT_EQ(NeighborFreshness::LOST, snapshot.all.front().freshness);
  EXPECT_TRUE(snapshot.organization.empty());
  EXPECT_TRUE(snapshot.conflict.empty());
  EXPECT_TRUE(snapshot.safety.empty());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
