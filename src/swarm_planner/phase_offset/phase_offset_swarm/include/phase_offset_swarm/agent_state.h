#pragma once

#include <Eigen/Core>

#include <cmath>
#include <string>

namespace phase_offset_swarm {

// Minimal state used by the ROS-free swarm-intent core.  The timestamp is a
// seconds value supplied by the adapter; no clock or message type belongs in
// this layer.
struct SwarmAgentState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int id = -1;
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  // The X/Y members above are retained for the existing planar SPH API.
  // SIM-C stores the complete world state in these scalar Z members and
  // exposes it through the vector helpers below.
  double position_z = 0.0;
  double velocity_z = 0.0;
  double stamp = 0.0;
  std::string frame_id;

  Eigen::Vector3d positionWorld() const {
    return Eigen::Vector3d(position.x(), position.y(), position_z);
  }

  Eigen::Vector3d velocityWorld() const {
    return Eigen::Vector3d(velocity.x(), velocity.y(), velocity_z);
  }

  // Descriptive aliases make the world-frame contract explicit at adapter
  // call sites without changing the established planar field names.
  Eigen::Vector3d worldPosition() const { return positionWorld(); }
  Eigen::Vector3d worldVelocity() const { return velocityWorld(); }
  Eigen::Vector3d position3d() const { return positionWorld(); }
  Eigen::Vector3d velocity3d() const { return velocityWorld(); }

  bool isFinite() const {
    return std::isfinite(stamp) && std::isfinite(position_z) &&
           std::isfinite(velocity_z) && position.allFinite() &&
           velocity.allFinite();
  }

  bool isValid() const { return id >= 0 && isFinite(); }
};

}  // namespace phase_offset_swarm
