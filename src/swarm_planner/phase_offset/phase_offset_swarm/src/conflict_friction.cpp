#include "phase_offset_swarm/conflict_friction.h"

#include <algorithm>
#include <cmath>

#include "phase_offset_swarm/elastic_interaction.h"
#include "phase_offset_swarm/pair_geometry.h"

namespace phase_offset_swarm {

ConflictFriction::ConflictFriction(const SwarmParameters& parameters)
    : parameters_(parameters) {
  parameters_.validateOrThrow();
}

Eigen::Vector2d ConflictFriction::compute(
    const SwarmAgentState& self, const NeighborSnapshot& neighbors) const {
  Eigen::Vector2d result = Eigen::Vector2d::Zero();
  const double distance_denominator = parameters_.r_conf - parameters_.d_safe;
  for (const NeighborState& neighbor : neighbors.conflict) {
    if (neighbor.freshness != NeighborFreshness::FRESH) {
      continue;
    }
    const PairGeometry geometry = computePairGeometry(
        self, neighbor, parameters_.distance_epsilon, parameters_.d_safe,
        parameters_.ttc_epsilon);
    if (!std::isfinite(geometry.distance) ||
        !std::isfinite(geometry.distance_rate) ||
        !geometry.direction.allFinite() ||
        geometry.closing_speed <= parameters_.ttc_epsilon) {
      continue;
    }
    const double omega_d = ElasticInteraction::smootherstep(
        (parameters_.r_conf - geometry.distance) / distance_denominator);
    const double omega_c = ElasticInteraction::smootherstep(
        geometry.closing_speed / parameters_.closing_speed_activation);
    double omega_t = 0.0;
    if (std::isfinite(geometry.ttc)) {
      omega_t = ElasticInteraction::smootherstep(
          (parameters_.ttc_activation - geometry.ttc) /
          parameters_.ttc_activation);
    }
    const double omega = omega_c * std::max(omega_d, omega_t);
    const double scalar = parameters_.k_conf * omega * geometry.distance_rate;
    if (std::isfinite(scalar)) {
      result += scalar * geometry.direction;
    }
  }
  return result.allFinite() ? result : Eigen::Vector2d::Zero();
}

}  // namespace phase_offset_swarm
