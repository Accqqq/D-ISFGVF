#pragma once

#include <Eigen/Core>

#include <string>

namespace FLAG_Race {
namespace guidance {

struct ReferenceGeometry {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d tangent = Eigen::Vector3d::Zero();
  double derivative_norm = 0.0;
  bool valid = false;
};

struct IsfGains {
  double k1 = 0.0;
  double k2 = 0.0;
  double convergence_bandwidth = 0.0;
  double progress_rho0 = 0.0;
  double progress_delta = 0.0;
  double alpha_min = 0.0;

  IsfGains() = default;
  IsfGains(const double k1_value,
           const double k2_value,
           const double convergence_bandwidth_value,
           const double progress_rho0_value,
           const double progress_delta_value,
           const double alpha_min_value)
      : k1(k1_value),
        k2(k2_value),
        convergence_bandwidth(convergence_bandwidth_value),
        progress_rho0(progress_rho0_value),
        progress_delta(progress_delta_value),
        alpha_min(alpha_min_value) {}
};

struct IsfGuidance {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d v_cmd = Eigen::Vector3d::Zero();
  double w_dot = 0.0;
  double e_parallel = 0.0;
  Eigen::Vector3d e_perp = Eigen::Vector3d::Zero();
  Eigen::Vector3d ref_pt = Eigen::Vector3d::Zero();
  Eigen::Vector3d tangent = Eigen::Vector3d::Zero();
  bool valid = false;
  std::string invalid_reason;
};

class IsfReferenceKernel {
 public:
  static bool evaluate(const Eigen::Vector3d& position,
                       const ReferenceGeometry& reference,
                       const IsfGains& gains,
                       IsfGuidance& output);
};

}  // namespace guidance
}  // namespace FLAG_Race
