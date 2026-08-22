#pragma once

#include <Eigen/Core>

#include <string>

#include "phase_offset_core/geometry_types.h"
#include "phase_offset_core/port_types.h"

namespace phase_offset_core {

struct MatchedPortInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PhaseOffsetGeometryState geometry;
  Eigen::Vector3d base_v_cmd = Eigen::Vector3d::Zero();
  double base_w_dot = 0.0;
  PortCommand final_port;
};

struct MatchedPortOutput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d physical_port = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_cmd = Eigen::Vector3d::Zero();
  double w_dot = 0.0;
  double delta_dot = 0.0;
  Eigen::Vector3d matched_residual = Eigen::Vector3d::Zero();
  double matched_residual_norm = 0.0;
  bool valid = false;
  std::string invalid_reason;
};

// Stateless composition of a final port with the A3 base guidance.
class MatchedPort {
 public:
  static bool evaluate(const MatchedPortInput& input,
                       MatchedPortOutput& output);
};

}  // namespace phase_offset_core
