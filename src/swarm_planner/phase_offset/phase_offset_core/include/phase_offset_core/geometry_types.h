#pragma once

#include <Eigen/Core>

#include <string>

namespace phase_offset_core {

struct GeometryParams {
  double tangent_epsilon = 1e-8;
  double horizontal_tangent_epsilon = 1e-8;
  double regularity_margin = 0.1;
};

struct PreparedPathGeometry {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_ww = Eigen::Vector3d::Zero();
  Eigen::Vector3d N = Eigen::Vector3d::Zero();
  Eigen::Vector3d N_w = Eigen::Vector3d::Zero();
  double w = 0.0;
  double path_speed = 0.0;
  double horizontal_path_speed = 0.0;
  double curvature = 0.0;

  // These failures have the original evaluator's pre-position/delta priority.
  bool pre_point_valid = false;
  const char* pre_point_reason = nullptr;
  // These facts may be precomputed, but their failure is consumed only after
  // position and delta have been checked for an individual surface point.
  bool delayed_path_valid = false;
  const char* delayed_path_reason = nullptr;
};

struct PreparedReferenceResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d r = Eigen::Vector3d::Zero();
  double regularity = 0.0;
  bool valid = false;
  const char* invalid_reason = nullptr;
};

struct PhaseOffsetGeometryState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_ww = Eigen::Vector3d::Zero();
  Eigen::Vector3d T = Eigen::Vector3d::Zero();
  Eigen::Vector3d N = Eigen::Vector3d::Zero();
  Eigen::Vector3d N_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d r = Eigen::Vector3d::Zero();
  Eigen::Vector3d r_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d error = Eigen::Vector3d::Zero();
  Eigen::Vector3d e_perp = Eigen::Vector3d::Zero();
  double w = 0.0;
  double delta = 0.0;
  double path_speed = 0.0;
  double horizontal_path_speed = 0.0;
  double curvature = 0.0;
  double regularity = 0.0;
  double e_parallel = 0.0;
  bool valid = false;
  std::string invalid_reason;
};

}  // namespace phase_offset_core
