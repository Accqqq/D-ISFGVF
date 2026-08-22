#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>

#include "phase_offset_swarm/agent_state.h"
#include "phase_offset_swarm/neighbor_state.h"

namespace phase_offset_swarm {

struct PairGeometry {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double distance = 0.0;
  Eigen::Vector2d direction = Eigen::Vector2d::Zero();
  double distance_rate = 0.0;
  double closing_speed = 0.0;
  double ttc = std::numeric_limits<double>::infinity();
};

inline PairGeometry computePairGeometry(const SwarmAgentState& self,
                                         const NeighborState& neighbor,
                                         double distance_epsilon,
                                         double d_safe,
                                         double ttc_epsilon) {
  PairGeometry result;
  const Eigen::Vector2d displacement =
      neighbor.predicted_position - self.position;
  result.distance = displacement.norm();
  if (!std::isfinite(result.distance) || !displacement.allFinite()) {
    return result;
  }
  if (result.distance <= distance_epsilon) {
    // The deterministic axis is anti-symmetric when the two endpoint IDs are
    // swapped, which keeps the coincident-position test finite and repeatable.
    result.direction = Eigen::Vector2d::UnitX();
    if (self.id > neighbor.id) {
      result.direction = -result.direction;
    }
  } else {
    result.direction = displacement / result.distance;
  }
  result.distance_rate =
      (neighbor.velocity - self.velocity).dot(result.direction);
  if (!std::isfinite(result.distance_rate)) {
    result.distance_rate = 0.0;
  }
  result.closing_speed = std::max(0.0, -result.distance_rate);
  if (result.closing_speed > ttc_epsilon) {
    const double numerator = std::max(0.0, result.distance - d_safe);
    result.ttc = numerator / (result.closing_speed + ttc_epsilon);
    if (!std::isfinite(result.ttc)) {
      result.ttc = std::numeric_limits<double>::infinity();
    }
  }
  return result;
}

}  // namespace phase_offset_swarm
