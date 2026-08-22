#include "phase_offset_swarm/elastic_interaction.h"

#include <algorithm>
#include <cmath>

#include "phase_offset_swarm/pair_geometry.h"

namespace phase_offset_swarm {

ElasticInteraction::ElasticInteraction(const SwarmParameters& parameters)
    : parameters_(parameters) {
  parameters_.validateOrThrow();
}

double ElasticInteraction::smootherstep(double s) {
  s = std::max(0.0, std::min(1.0, s));
  return 6.0 * std::pow(s, 5) - 15.0 * std::pow(s, 4) +
         10.0 * std::pow(s, 3);
}

double ElasticInteraction::cohesionBump(double s) {
  s = std::max(0.0, std::min(1.0, s));
  return 64.0 * std::pow(s, 3) * std::pow(1.0 - s, 3);
}

Eigen::Vector2d ElasticInteraction::computeSeparation(
    const SwarmAgentState& self, const NeighborSnapshot& neighbors) const {
  Eigen::Vector2d result = Eigen::Vector2d::Zero();
  const double denominator = parameters_.d_minus - parameters_.d_safe;
  for (const NeighborState& neighbor : neighbors.organization) {
    if (neighbor.freshness != NeighborFreshness::FRESH) {
      continue;
    }
    const PairGeometry geometry = computePairGeometry(
        self, neighbor, parameters_.distance_epsilon, parameters_.d_safe,
        parameters_.ttc_epsilon);
    if (!std::isfinite(geometry.distance) ||
        !geometry.direction.allFinite()) {
      continue;
    }
    const double s = (parameters_.d_minus - geometry.distance) / denominator;
    const double magnitude = parameters_.k_sep * smootherstep(s);
    if (std::isfinite(magnitude)) {
      result -= magnitude * geometry.direction;
    }
  }
  return result.allFinite() ? result : Eigen::Vector2d::Zero();
}

Eigen::Vector2d ElasticInteraction::computeCohesionUnscaled(
    const SwarmAgentState& self, const NeighborSnapshot& neighbors) const {
  Eigen::Vector2d result = Eigen::Vector2d::Zero();
  const double denominator = parameters_.r_comm - parameters_.d_plus;
  for (const NeighborState& neighbor : neighbors.organization) {
    if (neighbor.freshness != NeighborFreshness::FRESH) {
      continue;
    }
    const PairGeometry geometry = computePairGeometry(
        self, neighbor, parameters_.distance_epsilon, parameters_.d_safe,
        parameters_.ttc_epsilon);
    if (!std::isfinite(geometry.distance) ||
        !geometry.direction.allFinite()) {
      continue;
    }
    if (geometry.distance > parameters_.d_plus &&
        geometry.distance < parameters_.r_comm) {
      const double s = (geometry.distance - parameters_.d_plus) / denominator;
      const double magnitude = parameters_.k_coh * cohesionBump(s);
      if (std::isfinite(magnitude)) {
        result += magnitude * geometry.direction;
      }
    }
  }
  return result.allFinite() ? result : Eigen::Vector2d::Zero();
}

}  // namespace phase_offset_swarm
