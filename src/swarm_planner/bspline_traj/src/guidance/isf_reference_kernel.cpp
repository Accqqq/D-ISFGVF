#include "bspline_race/guidance/isf_reference_kernel.h"

#include <cmath>
#include <utility>

namespace FLAG_Race {
namespace guidance {
namespace {

constexpr double kMinimumDerivativeNorm = 1e-6;
constexpr double kUnitTangentTolerance = 1e-9;

bool IsFinite(const double value) {
  return std::isfinite(value);
}

bool IsFinite(const Eigen::Vector3d& value) {
  return value.allFinite();
}

bool HasFiniteGains(const IsfGains& gains) {
  return IsFinite(gains.k1) && IsFinite(gains.k2) &&
         IsFinite(gains.convergence_bandwidth) &&
         IsFinite(gains.progress_rho0) && IsFinite(gains.progress_delta) &&
         IsFinite(gains.alpha_min);
}

void EnsureFinite(IsfGuidance& output) {
  if (!IsFinite(output.v_cmd)) {
    output.v_cmd.setZero();
  }
  if (!IsFinite(output.w_dot)) {
    output.w_dot = 0.0;
  }
  if (!IsFinite(output.e_parallel)) {
    output.e_parallel = 0.0;
  }
  if (!IsFinite(output.e_perp)) {
    output.e_perp.setZero();
  }
  if (!IsFinite(output.ref_pt)) {
    output.ref_pt.setZero();
  }
  if (!IsFinite(output.tangent)) {
    output.tangent.setZero();
  }
}

bool Invalidate(IsfGuidance& output, std::string reason) {
  EnsureFinite(output);
  output.valid = false;
  output.invalid_reason = std::move(reason);
  return false;
}

}  // namespace

bool IsfReferenceKernel::evaluate(const Eigen::Vector3d& position,
                                  const ReferenceGeometry& reference,
                                  const IsfGains& gains,
                                  IsfGuidance& output) {
  output = IsfGuidance();

  if (!reference.valid) {
    return Invalidate(output, "reference geometry is invalid");
  }
  if (!IsFinite(position) || !IsFinite(reference.point) ||
      !IsFinite(reference.tangent) || !IsFinite(reference.derivative_norm) ||
      !HasFiniteGains(gains)) {
    return Invalidate(output, "kernel input is not finite");
  }
  if (reference.derivative_norm < kMinimumDerivativeNorm) {
    return Invalidate(output, "reference derivative norm is too small");
  }

  const double tangent_norm = reference.tangent.norm();
  if (!IsFinite(tangent_norm) ||
      std::abs(tangent_norm - 1.0) > kUnitTangentTolerance) {
    return Invalidate(output, "reference tangent is not unit length");
  }

  const Eigen::Vector3d error = position - reference.point;
  const double e_parallel = reference.tangent.dot(error);
  const Eigen::Vector3d e_perp = error - e_parallel * reference.tangent;
  const double rho = e_perp.norm();
  const double bandwidth = std::max(1e-6, gains.convergence_bandwidth);
  const double q = rho > 1e-6 ? std::tanh(rho / bandwidth) / rho
                               : 1.0 / bandwidth;
  const double rho0 = std::max(1e-6, gains.progress_rho0);
  const double alpha = gains.alpha_min + (1.0 - gains.alpha_min) /
      (1.0 + (rho / rho0) * (rho / rho0));
  const double progress_delta = std::max(1e-6, gains.progress_delta);
  const double sigma = std::tanh(e_parallel / progress_delta);

  output.v_cmd = gains.k1 * alpha * reference.tangent + gains.k2 * q * e_perp;
  output.w_dot = gains.k1 * (alpha + sigma) / reference.derivative_norm;
  output.e_parallel = e_parallel;
  output.e_perp = e_perp;
  output.ref_pt = reference.point;
  output.tangent = reference.tangent;

  if (!IsFinite(output.v_cmd) || !IsFinite(output.w_dot) ||
      !IsFinite(output.e_parallel) || !IsFinite(output.e_perp) ||
      !IsFinite(output.ref_pt) || !IsFinite(output.tangent)) {
    return Invalidate(output, "computed guidance is not finite");
  }

  output.valid = true;
  output.invalid_reason.clear();
  return true;
}

}  // namespace guidance
}  // namespace FLAG_Race
