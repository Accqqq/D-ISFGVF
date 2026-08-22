#include "phase_offset_swarm/swarm_intent.h"

#include <algorithm>
#include <cmath>

#include "phase_offset_swarm/pair_geometry.h"

namespace phase_offset_swarm {

SwarmIntentCalculator::SwarmIntentCalculator(
    const SwarmParameters& parameters)
    : parameters_(parameters),
      elastic_interaction_(parameters),
      conflict_friction_(parameters) {
  parameters_.validateOrThrow();
}

SwarmOutput SwarmIntentCalculator::compute(
    const SwarmAgentState& self, const NeighborSnapshot& neighbors,
    double beta_preview) const {
  SwarmOutput output;
  output.num_org_neighbors =
      static_cast<int>(neighbors.organization.size());
  output.num_conflict_neighbors =
      static_cast<int>(neighbors.conflict.size());
  output.num_safety_neighbors = static_cast<int>(neighbors.safety.size());
  output.num_fresh_neighbors = neighbors.fresh_count;
  output.num_stale_neighbors = neighbors.stale_count;
  output.num_lost_neighbors = neighbors.lost_count;

  output.g_sep = elastic_interaction_.computeSeparation(self, neighbors);
  const Eigen::Vector2d cohesion_unscaled =
      elastic_interaction_.computeCohesionUnscaled(self, neighbors);
  const double beta = std::isfinite(beta_preview)
                          ? std::max(0.0, std::min(1.0, beta_preview))
                          : 0.0;
  output.g_coh = beta * cohesion_unscaled;
  output.g_conf = conflict_friction_.compute(self, neighbors);

  const Eigen::Vector2d raw = output.g_sep + output.g_coh + output.g_conf;
  const double raw_norm = raw.norm();
  if (raw.allFinite() && std::isfinite(raw_norm) &&
      raw_norm > parameters_.g_max) {
    output.g_coord = raw * (parameters_.g_max / raw_norm);
    output.output_saturated = true;
  } else if (raw.allFinite()) {
    output.g_coord = raw;
  }

  for (const NeighborState& neighbor : neighbors.all) {
    if (neighbor.freshness != NeighborFreshness::FRESH) {
      continue;
    }
    const PairGeometry geometry = computePairGeometry(
        self, neighbor, parameters_.distance_epsilon, parameters_.d_safe,
        parameters_.ttc_epsilon);
    if (std::isfinite(geometry.distance)) {
      output.min_distance = std::min(output.min_distance, geometry.distance);
    }
    if (geometry.closing_speed > parameters_.ttc_epsilon &&
        std::isfinite(geometry.ttc)) {
      output.min_ttc = std::min(output.min_ttc, geometry.ttc);
    }
  }
  return output;
}

}  // namespace phase_offset_swarm
