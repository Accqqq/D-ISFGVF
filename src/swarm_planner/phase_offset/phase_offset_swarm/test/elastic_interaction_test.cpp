#include <gtest/gtest.h>

#include <cmath>

#include "phase_offset_swarm/elastic_interaction.h"

namespace {

using phase_offset_swarm::ElasticInteraction;
using phase_offset_swarm::NeighborFreshness;
using phase_offset_swarm::NeighborSnapshot;
using phase_offset_swarm::NeighborState;
using phase_offset_swarm::SwarmAgentState;
using phase_offset_swarm::SwarmParameters;

SwarmAgentState self(int id = 1) {
  SwarmAgentState result;
  result.id = id;
  return result;
}

NeighborState neighbor(int id, double x, double y = 0.0) {
  NeighborState result;
  result.id = id;
  result.position = Eigen::Vector2d(x, y);
  result.predicted_position = result.position;
  result.freshness = NeighborFreshness::FRESH;
  return result;
}

NeighborSnapshot organization(std::initializer_list<NeighborState> states) {
  NeighborSnapshot result;
  for (const NeighborState& state : states) {
    result.organization.push_back(state);
  }
  return result;
}

SwarmParameters params() {
  SwarmParameters result;
  result.r_comm = 2.0;
  result.d_minus = 0.8;
  result.d_plus = 1.2;
  result.k_sep = 1.0;
  result.k_coh = 0.2;
  return result;
}

TEST(ElasticInteraction, TooCloseSeparationPointsAway) {
  ElasticInteraction model(params());
  const Eigen::Vector2d output =
      model.computeSeparation(self(), organization({neighbor(2, 0.5)}));
  EXPECT_LT(output.x(), 0.0);
  EXPECT_NEAR(1.0, output.norm(), 1e-12);
}

TEST(ElasticInteraction, ComfortBandHasNoPositionIntent) {
  ElasticInteraction model(params());
  const auto snapshot = organization({neighbor(2, 1.0)});
  EXPECT_NEAR(0.0, model.computeSeparation(self(), snapshot).norm(), 1e-12);
  EXPECT_NEAR(0.0, model.computeCohesionUnscaled(self(), snapshot).norm(),
              1e-12);
}

TEST(ElasticInteraction, WeakCohesionPointsTowardNeighbor) {
  ElasticInteraction model(params());
  const Eigen::Vector2d output = model.computeCohesionUnscaled(
      self(), organization({neighbor(2, 1.5)}));
  EXPECT_GT(output.x(), 0.0);
  EXPECT_GT(output.norm(), 0.0);
}

TEST(ElasticInteraction, BetaZeroLeavesSeparationUnchanged) {
  ElasticInteraction model(params());
  const auto snapshot = organization({neighbor(2, 0.5), neighbor(3, 1.5)});
  const Eigen::Vector2d separation = model.computeSeparation(self(), snapshot);
  const Eigen::Vector2d cohesion = model.computeCohesionUnscaled(self(), snapshot);
  EXPECT_GT(cohesion.norm(), 0.0);
  EXPECT_NEAR(0.0, (0.0 * cohesion).norm(), 1e-12);
  EXPECT_LT(separation.x(), 0.0);
}

TEST(ElasticInteraction, KernelsAreBoundedAndHaveEndpointValues) {
  EXPECT_NEAR(0.0, ElasticInteraction::smootherstep(0.0), 1e-12);
  EXPECT_NEAR(1.0, ElasticInteraction::smootherstep(1.0), 1e-12);
  EXPECT_NEAR(0.0, ElasticInteraction::cohesionBump(0.0), 1e-12);
  EXPECT_NEAR(0.0, ElasticInteraction::cohesionBump(1.0), 1e-12);
  for (int i = 0; i <= 100; ++i) {
    const double s = static_cast<double>(i) / 100.0;
    EXPECT_GE(ElasticInteraction::smootherstep(s), 0.0);
    EXPECT_LE(ElasticInteraction::smootherstep(s), 1.0);
    EXPECT_GE(ElasticInteraction::cohesionBump(s), 0.0);
    EXPECT_LE(ElasticInteraction::cohesionBump(s), 1.0 + 1e-12);
  }
}

TEST(ElasticInteraction, MultipleNeighborsAggregate) {
  ElasticInteraction model(params());
  const auto one = model.computeSeparation(self(),
                                            organization({neighbor(2, 0.5)}));
  const auto two = model.computeSeparation(
      self(), organization({neighbor(2, 0.5), neighbor(3, 0.5, 0.1)}));
  EXPECT_GT(two.norm(), one.norm());
}

TEST(ElasticInteraction, CoincidentPositionUsesFiniteDeterministicAxis) {
  ElasticInteraction model(params());
  const auto forward = model.computeSeparation(
      self(1), organization({neighbor(2, 0.0)}));
  const auto reverse = model.computeSeparation(
      self(2), organization({neighbor(1, 0.0)}));
  EXPECT_TRUE(forward.allFinite());
  EXPECT_TRUE(reverse.allFinite());
  EXPECT_NEAR(0.0, (forward + reverse).norm(), 1e-12);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
