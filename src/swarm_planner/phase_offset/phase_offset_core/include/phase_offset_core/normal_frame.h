#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

namespace phase_offset_core {

// Sole production owner for Horizontal-N capability.  A nonzero transverse
// direction is available only when the world-horizontal path speed is
// strictly above this value.  Test-only parameter overrides must not replace
// this frame threshold.
constexpr double kHorizontalNormalSpeedEpsilon = 1e-8;

constexpr char kWorldHorizontalCrossProductProvenance[] =
    "ContinuousPhaseNormalFrame/WorldHorizontalCrossProduct";

inline bool isWorldHorizontalCrossProductProvenance(
    const std::string& provenance) {
  return provenance == kWorldHorizontalCrossProductProvenance;
}

// The following bounded dyadic helpers compare binary64 values exactly.  A
// finite non-negative double is an integer significand (at most 53 bits)
// times a power of two.  Products of two such significands fit in unsigned
// __int128 (at most 106 bits), so no wider floating-point precision or
// arbitrary epsilon is needed for directed certificate checks.
struct Binary64Dyadic {
  std::uint64_t significand = 0U;
  int exponent = 0;
};

inline Binary64Dyadic decomposeBinary64(const double value) {
  Binary64Dyadic result;
  const double magnitude = std::abs(value);
  std::uint64_t bits = 0U;
  std::memcpy(&bits, &magnitude, sizeof(bits));
  const std::uint64_t fraction =
      bits & ((std::uint64_t(1) << 52U) - std::uint64_t(1));
  const std::uint64_t exponent_bits = (bits >> 52U) & std::uint64_t(0x7ff);
  if (exponent_bits == 0U) {
    result.significand = fraction;
    result.exponent = -1074;
  } else {
    result.significand = (std::uint64_t(1) << 52U) | fraction;
    result.exponent = static_cast<int>(exponent_bits) - 1023 - 52;
  }
  return result;
}

inline int binary128BitLength(unsigned __int128 value) {
  int length = 0;
  while (value != 0U) {
    ++length;
    value >>= 1U;
  }
  return length;
}

inline int compareDyadic(unsigned __int128 lhs_significand,
                         const int lhs_exponent,
                         unsigned __int128 rhs_significand,
                         const int rhs_exponent) {
  if (lhs_significand == 0U) return rhs_significand == 0U ? 0 : -1;
  if (rhs_significand == 0U) return 1;
  const int lhs_length = binary128BitLength(lhs_significand);
  const int rhs_length = binary128BitLength(rhs_significand);
  const int lhs_top = lhs_exponent + lhs_length;
  const int rhs_top = rhs_exponent + rhs_length;
  if (lhs_top < rhs_top) return -1;
  if (lhs_top > rhs_top) return 1;
  const int aligned_length = std::max(lhs_length, rhs_length);
  lhs_significand <<= aligned_length - lhs_length;
  rhs_significand <<= aligned_length - rhs_length;
  if (lhs_significand < rhs_significand) return -1;
  if (lhs_significand > rhs_significand) return 1;
  return 0;
}

inline bool binary64ProductLeq(const double lhs, const double rhs,
                               const double candidate) {
  const Binary64Dyadic lhs_parts = decomposeBinary64(lhs);
  const Binary64Dyadic rhs_parts = decomposeBinary64(rhs);
  const Binary64Dyadic candidate_parts = decomposeBinary64(candidate);
  const unsigned __int128 product =
      static_cast<unsigned __int128>(lhs_parts.significand) *
      static_cast<unsigned __int128>(rhs_parts.significand);
  return compareDyadic(product, lhs_parts.exponent + rhs_parts.exponent,
                       candidate_parts.significand,
                       candidate_parts.exponent) <= 0;
}

inline bool binary64RatioLeq(const double numerator, const double denominator,
                             const double candidate) {
  const Binary64Dyadic numerator_parts = decomposeBinary64(numerator);
  const Binary64Dyadic denominator_parts = decomposeBinary64(denominator);
  const Binary64Dyadic candidate_parts = decomposeBinary64(candidate);
  const unsigned __int128 candidate_product =
      static_cast<unsigned __int128>(candidate_parts.significand) *
      static_cast<unsigned __int128>(denominator_parts.significand);
  return compareDyadic(candidate_product,
                       candidate_parts.exponent + denominator_parts.exponent,
                       numerator_parts.significand,
                       numerator_parts.exponent) >= 0;
}

inline double binary64FromBits(const std::uint64_t bits) {
  double value = 0.0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

// Compute an upper bound for a non-negative quotient/product by an exact
// dyadic comparison.  The ordinary candidate is normally accepted directly;
// a one-ulp repair handles the usual rounded-down case.  The bounded binary
// search is a complete fallback for residuals too small to survive binary64
// fma/subnormal rounding, and also handles positive underflow (0 -> denorm).
inline bool outwardUpperRatio(const double numerator,
                              const double denominator,
                              double& upper) {
  upper = std::numeric_limits<double>::quiet_NaN();
  if (!std::isfinite(numerator) || !std::isfinite(denominator) ||
      numerator < 0.0 || denominator <= 0.0) {
    return false;
  }
  if (numerator == 0.0) {
    upper = 0.0;
    return true;
  }
  double candidate = numerator / denominator;
  if (std::isfinite(candidate) && candidate >= 0.0 &&
      binary64RatioLeq(numerator, denominator, candidate)) {
    upper = candidate;
    return true;
  }
  if (std::isfinite(candidate) && candidate >= 0.0 &&
      candidate < std::numeric_limits<double>::max()) {
    candidate = std::nextafter(candidate,
                               std::numeric_limits<double>::infinity());
    if (binary64RatioLeq(numerator, denominator, candidate)) {
      upper = candidate;
      return true;
    }
  }
  const double maximum = std::numeric_limits<double>::max();
  if (!binary64RatioLeq(numerator, denominator, maximum)) return false;
  std::uint64_t lower = 0U;
  std::uint64_t higher = 0x7fefffffffffffffULL;
  while (lower < higher) {
    const std::uint64_t middle = lower + (higher - lower) / 2U;
    if (binary64RatioLeq(numerator, denominator,
                         binary64FromBits(middle))) {
      higher = middle;
    } else {
      lower = middle + 1U;
    }
  }
  upper = binary64FromBits(lower);
  return true;
}

inline bool outwardUpperProduct(const double lhs, const double rhs,
                                double& upper) {
  upper = std::numeric_limits<double>::quiet_NaN();
  if (!std::isfinite(lhs) || !std::isfinite(rhs) || lhs < 0.0 ||
      rhs < 0.0) {
    return false;
  }
  if (lhs == 0.0 || rhs == 0.0) {
    upper = 0.0;
    return true;
  }
  double candidate = lhs * rhs;
  if (std::isfinite(candidate) && candidate >= 0.0 &&
      binary64ProductLeq(lhs, rhs, candidate)) {
    upper = candidate;
    return true;
  }
  if (std::isfinite(candidate) && candidate >= 0.0 &&
      candidate < std::numeric_limits<double>::max()) {
    candidate = std::nextafter(candidate,
                               std::numeric_limits<double>::infinity());
    if (binary64ProductLeq(lhs, rhs, candidate)) {
      upper = candidate;
      return true;
    }
  }
  const double maximum = std::numeric_limits<double>::max();
  if (!binary64ProductLeq(lhs, rhs, maximum)) return false;
  std::uint64_t lower = 0U;
  std::uint64_t higher = 0x7fefffffffffffffULL;
  while (lower < higher) {
    const std::uint64_t middle = lower + (higher - lower) / 2U;
    if (binary64ProductLeq(lhs, rhs, binary64FromBits(middle))) {
      higher = middle;
    } else {
      lower = middle + 1U;
    }
  }
  upper = binary64FromBits(lower);
  return true;
}

// Result of querying the one immutable normal frame associated with a path
// revision.  The query is deliberately a value type: consumers cannot retain
// a mutable frame or reconstruct a competing normal from the path derivative.
struct NormalFrameQuery {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double w = 0.0;
  std::uint64_t path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  Eigen::Vector3d T = Eigen::Vector3d::Zero();
  Eigen::Vector3d N = Eigen::Vector3d::Zero();
  Eigen::Vector3d N_w = Eigen::Vector3d::Zero();
  bool valid = false;
  std::string provenance;
  std::string invalid_reason;
};

// A closed-cell proof for the frame and the path derivatives used by the
// geometric Tube.  This is evidence, not a runtime gate.  A consumer must
// reject the proof unless complete is true and the revisions/range match the
// immutable path it is evaluating.
struct NormalFrameCellProof {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double w0 = 0.0;
  double w1 = 0.0;
  std::uint64_t path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  double inf_path_speed = 0.0;
  double sup_path_speed = 0.0;
  double sup_path_acceleration = 0.0;
  double sup_path_jerk = 0.0;
  // Horizontal-N certificate evidence.  q_min is represented by
  // inf_horizontal_path_speed and a_xy by this explicit acceleration bound.
  double inf_horizontal_path_speed = 0.0;
  double sup_horizontal_p_ww_norm = 0.0;
  bool horizontal_acceleration_bound_complete = false;
  double sup_normal_derivative = 0.0;
  double normal_variation_bound = 0.0;
  double tangent_variation_bound = 0.0;
  bool valid = false;
  bool complete = false;
  std::string provenance;
};

inline bool normalFrameCellProofIsComplete(
    const NormalFrameCellProof& proof) {
  if (!proof.valid || !proof.complete || !(proof.w1 > proof.w0) ||
      !std::isfinite(proof.w0) || !std::isfinite(proof.w1) ||
      proof.path_revision == 0U || proof.frame_revision == 0U ||
      !std::isfinite(proof.inf_path_speed) || proof.inf_path_speed <= 0.0 ||
      !std::isfinite(proof.sup_path_speed) ||
      proof.sup_path_speed < proof.inf_path_speed ||
      !std::isfinite(proof.sup_path_acceleration) ||
      proof.sup_path_acceleration < 0.0 ||
      !std::isfinite(proof.sup_path_jerk) || proof.sup_path_jerk < 0.0 ||
      !std::isfinite(proof.inf_horizontal_path_speed) ||
      proof.inf_horizontal_path_speed <= kHorizontalNormalSpeedEpsilon ||
      !std::isfinite(proof.sup_horizontal_p_ww_norm) ||
      proof.sup_horizontal_p_ww_norm < 0.0 ||
      !proof.horizontal_acceleration_bound_complete ||
      !std::isfinite(proof.sup_normal_derivative) ||
      proof.sup_normal_derivative < 0.0 ||
      !std::isfinite(proof.normal_variation_bound) ||
      proof.normal_variation_bound < 0.0 ||
      !std::isfinite(proof.tangent_variation_bound) ||
      proof.tangent_variation_bound < 0.0 ||
      !isWorldHorizontalCrossProductProvenance(proof.provenance)) {
    return false;
  }

  double required_normal_derivative = 0.0;
  if (!outwardUpperRatio(proof.sup_horizontal_p_ww_norm,
                         proof.inf_horizontal_path_speed,
                         required_normal_derivative) ||
      proof.sup_normal_derivative < required_normal_derivative) {
    return false;
  }
  const double span = proof.w1 - proof.w0;
  double required_normal_variation = 0.0;
  if (!outwardUpperProduct(proof.sup_normal_derivative, span,
                           required_normal_variation) ||
      proof.normal_variation_bound < required_normal_variation) {
    return false;
  }
  double tangent_rate = 0.0;
  if (!outwardUpperRatio(proof.sup_path_acceleration, proof.inf_path_speed,
                         tangent_rate)) {
    return false;
  }
  double required_tangent_variation = 0.0;
  if (!outwardUpperProduct(tangent_rate, span,
                           required_tangent_variation) ||
      proof.tangent_variation_bound < required_tangent_variation) {
    return false;
  }
  return true;
}

// ImmutableNormalFrame is the only production-facing normal-frame contract.
// Implementations own their path/frame data and expose read-only queries.
class ImmutableNormalFrame {
 public:
  virtual ~ImmutableNormalFrame() = default;

  virtual bool query(double w, NormalFrameQuery& result) const = 0;
  virtual bool certifyCell(double w0, double w1,
                           NormalFrameCellProof& proof) const = 0;
  virtual double startW() const = 0;
  virtual double endW() const = 0;
  virtual std::uint64_t pathRevision() const = 0;
  virtual std::uint64_t frameRevision() const = 0;

  bool query(double w, NormalFrameQuery* result) const {
    return result != nullptr && query(w, *result);
  }
};

}  // namespace phase_offset_core
