#pragma once

#include <Eigen/Core>

#include <cmath>

namespace phase_offset_swarm {

// Minimal state used by the ROS-free swarm-intent core.  The timestamp is a
// seconds value supplied by the adapter; no clock or message type belongs in
// this layer.
struct SwarmAgentState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int id = -1;
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  double stamp = 0.0;

  bool isFinite() const {
    return std::isfinite(stamp) && position.allFinite() && velocity.allFinite();
  }

  bool isValid() const { return id >= 0 && isFinite(); }
};

}  // namespace phase_offset_swarm
