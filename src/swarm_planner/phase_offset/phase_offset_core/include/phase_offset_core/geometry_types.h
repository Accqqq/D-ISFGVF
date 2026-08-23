#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <string>

namespace phase_offset_core {

struct GeometryParams {
  double tangent_epsilon = 1e-8;
  double horizontal_tangent_epsilon = 1e-8;
  double regularity_margin = 0.1;
  // Production regularity is a full 3D active-reference speed bound. The
  // legacy horizontal/curvature fields remain diagnostics and compatibility
  // metadata only.
  double minimum_reference_speed = 1e-8;
  bool require_frame_binding = false;
};

struct PreparedPathGeometry {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_ww = Eigen::Vector3d::Zero();
  Eigen::Vector3d N = Eigen::Vector3d::Zero();
  Eigen::Vector3d N_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d T = Eigen::Vector3d::Zero();
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
  std::uint64_t path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  bool frame_bound = false;
  std::string frame_provenance;
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
  Eigen::Matrix<double, 3, 2> J = Eigen::Matrix<double, 3, 2>::Zero();
  double w = 0.0;
  double delta = 0.0;
  std::uint64_t path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  double path_speed = 0.0;
  double horizontal_path_speed = 0.0;
  double curvature = 0.0;
  double regularity = 0.0;
  double e_parallel = 0.0;
  bool valid = false;
  std::string invalid_reason;
  std::string provenance;
};

}  // namespace phase_offset_core
