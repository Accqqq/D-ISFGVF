#include <gtest/gtest.h>

#include <cmath>

#include "phase_offset_swarm/conflict_friction.h"

namespace {

using phase_offset_swarm::ConflictFriction;
using phase_offset_swarm::NeighborFreshness;
using phase_offset_swarm::NeighborSnapshot;
using phase_offset_swarm::NeighborState;
using phase_offset_swarm::SwarmAgentState;
using phase_offset_swarm::SwarmParameters;

SwarmParameters params() {
  SwarmParameters result;
  result.r_conf = 1.5;
  result.r_safe = 1.2;
  result.ttc_activation = 2.0;
  result.closing_speed_activation = 0.5;
  result.k_conf = 0.8;
  return result;
}

SwarmAgentState self(int id = 1, double vx = 0.0, double vy = 0.0) {
  SwarmAgentState result;
  result.id = id;
  result.velocity = Eigen::Vector2d(vx, vy);
  return result;
}

SwarmAgentState selfAt(int id, double x, double y, double vx, double vy) {
  SwarmAgentState result = self(id, vx, vy);
  result.position = Eigen::Vector2d(x, y);
  return result;
}

NeighborState neighbor(int id, double x, double y, double vx, double vy) {
  NeighborState result;
  result.id = id;
  result.predicted_position = Eigen::Vector2d(x, y);
  result.position = result.predicted_position;
  result.velocity = Eigen::Vector2d(vx, vy);
  result.freshness = NeighborFreshness::FRESH;
  return result;
}

NeighborSnapshot conflict(std::initializer_list<NeighborState> states) {
  NeighborSnapshot result;
  for (const NeighborState& state : states) {
    result.conflict.push_back(state);
  }
  return result;
}

TEST(ConflictFriction, ApproachingPairOpposesClosing) {
  ConflictFriction model(params());
  const Eigen::Vector2d output = model.compute(
      self(), conflict({neighbor(2, 1.0, 0.0, -1.0, 0.0)}));
  EXPECT_LT(output.x(), 0.0);
  EXPECT_TRUE(output.allFinite());
}

TEST(ConflictFriction, SeparatingPairIsExactlyZero) {
  ConflictFriction model(params());
  const Eigen::Vector2d output = model.compute(
      self(), conflict({neighbor(2, 1.0, 0.0, 1.0, 0.0)}));
  EXPECT_NEAR(0.0, output.norm(), 1e-12);
}

TEST(ConflictFriction, CommonTranslationIsZero) {
  ConflictFriction model(params());
  const Eigen::Vector2d output = model.compute(
      self(1, 1.0, 0.5), conflict({neighbor(2, 1.0, 0.0, 1.0, 0.5)}));
  EXPECT_NEAR(0.0, output.norm(), 1e-12);
}

TEST(ConflictFriction, NearZeroClosingSpeedIsFiniteZero) {
  ConflictFriction model(params());
  const Eigen::Vector2d output = model.compute(
      self(), conflict({neighbor(2, 1.0, 0.0, -1e-9, 0.0)}));
  EXPECT_TRUE(output.allFinite());
  EXPECT_NEAR(0.0, output.norm(), 1e-12);
}

TEST(ConflictFriction, DistanceAtSafetyReferenceIsFinite) {
  ConflictFriction model(params());
  const Eigen::Vector2d output = model.compute(
      self(), conflict({neighbor(2, 0.6, 0.0, -1.0, 0.0)}));
  EXPECT_TRUE(output.allFinite());
}

TEST(ConflictFriction, FarRapidlyApproachingPairUsesTtcActivation) {
  ConflictFriction model(params());
  const Eigen::Vector2d output = model.compute(
      self(), conflict({neighbor(2, 3.0, 0.0, -2.0, 0.0)}));
  EXPECT_LT(output.x(), 0.0);
  EXPECT_GT(output.norm(), 0.0);
}

TEST(ConflictFriction, SymmetricPairContributionsAreOpposite) {
  ConflictFriction model(params());
  const Eigen::Vector2d first = model.compute(
      selfAt(1, 0.0, 0.0, 0.0, 0.0),
      conflict({neighbor(2, 1.0, 0.0, -1.0, 0.0)}));
  const Eigen::Vector2d second = model.compute(
      selfAt(2, 1.0, 0.0, -1.0, 0.0),
      conflict({neighbor(1, 0.0, 0.0, 0.0, 0.0)}));
  EXPECT_NEAR(0.0, (first + second).norm(), 1e-12);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
