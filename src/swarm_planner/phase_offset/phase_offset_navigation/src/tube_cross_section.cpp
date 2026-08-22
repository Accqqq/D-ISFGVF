#include "phase_offset_navigation/tube_cross_section.h"

#include <algorithm>
#include <cmath>

namespace phase_offset_navigation {
namespace {

constexpr double kNormalEpsilon = 1e-12;
constexpr std::size_t kMaxCrossSectionSamples = 100000U;

bool IsFinite(const double value) {
  return std::isfinite(value);
}

bool IsFinite(const Eigen::Vector3d& value) {
  return value.allFinite();
}

bool ValidMargins(const RobustTubeMargins& margins) {
  return IsFinite(margins.uav_radius) && margins.uav_radius >= 0.0 &&
      IsFinite(margins.map_uncertainty) && margins.map_uncertainty >= 0.0 &&
      IsFinite(margins.localization_uncertainty) &&
      margins.localization_uncertainty >= 0.0 &&
      IsFinite(margins.tracking_error_bound) &&
      margins.tracking_error_bound >= 0.0 &&
      IsFinite(margins.preincluded_map_uncertainty) &&
      margins.preincluded_map_uncertainty >= 0.0 &&
      margins.preincluded_map_uncertainty <= margins.map_uncertainty &&
      IsFinite(margins.fullEffectiveRadius()) &&
      IsFinite(margins.residualEffectiveRadius());
}

TubeRayTermination ToTermination(const DistanceStatus status) {
  switch (status) {
    case DistanceStatus::OCCUPIED:
      return TubeRayTermination::OCCUPIED;
    case DistanceStatus::UNKNOWN:
      return TubeRayTermination::UNKNOWN;
    case DistanceStatus::OUT_OF_MAP:
      return TubeRayTermination::OUT_OF_MAP;
    case DistanceStatus::UNAVAILABLE:
      return TubeRayTermination::UNAVAILABLE;
    case DistanceStatus::KNOWN_FREE:
      return TubeRayTermination::SEARCH_EXTENT;
  }
  return TubeRayTermination::UNAVAILABLE;
}

TubeCrossSectionReason CenterReason(const DistanceStatus status) {
  switch (status) {
    case DistanceStatus::UNAVAILABLE:
      return TubeCrossSectionReason::CENTER_UNAVAILABLE;
    case DistanceStatus::OUT_OF_MAP:
      return TubeCrossSectionReason::CENTER_OUT_OF_MAP;
    case DistanceStatus::UNKNOWN:
      return TubeCrossSectionReason::CENTER_UNKNOWN;
    case DistanceStatus::OCCUPIED:
      return TubeCrossSectionReason::CENTER_OCCUPIED;
    case DistanceStatus::KNOWN_FREE:
      return TubeCrossSectionReason::EMPTY_AFTER_OBSTACLE_BOUNDS;
  }
  return TubeCrossSectionReason::EMPTY_AFTER_OBSTACLE_BOUNDS;
}

bool ClearanceSafe(const ClearanceQueryResult& result,
                   const double required_radius) {
  return result.status == DistanceStatus::KNOWN_FREE &&
      result.clearance_certified && IsFinite(result.clearance) &&
      result.clearance >= required_radius;
}

// Returns the safe side of a safe/unsafe transition.  The query's bounded
// ball contract means an UNKNOWN result is never crossed as if it were free.
double RefineSafeBoundary(const Eigen::Vector3d& origin,
                          const Eigen::Vector3d& normal,
                          const ClearanceQuery& query,
                          const double required_radius,
                          double unsafe_delta,
                          double safe_delta,
                          const double tolerance) {
  for (int iteration = 0; iteration < 128 &&
       std::abs(safe_delta - unsafe_delta) > tolerance; ++iteration) {
    const double middle = unsafe_delta + 0.5 * (safe_delta - unsafe_delta);
    if (!IsFinite(middle) || middle == unsafe_delta || middle == safe_delta) {
      break;
    }
    const ClearanceQueryResult result = query(origin + normal * middle,
                                              required_radius);
    if (ClearanceSafe(result, required_radius)) {
      safe_delta = middle;
    } else {
      unsafe_delta = middle;
    }
  }
  return safe_delta;
}

void SetFinalFacts(TubeCrossSectionResult& result) {
  result.width = std::max(0.0, result.upper_final - result.lower_final);
  if (result.lower_final > result.upper_final) return;
  result.contains_zero = result.lower_final <= 0.0 && 0.0 <= result.upper_final;
}

void SetZeroOnly(const TubeCrossSectionReason reason,
                 const TubeRayTermination termination,
                 TubeCrossSectionResult& result) {
  result.c_plus_raw = 0.0;
  result.c_minus_raw = 0.0;
  result.lower_obstacle = 0.0;
  result.upper_obstacle = 0.0;
  result.lower_final = 0.0;
  result.upper_final = 0.0;
  result.positive_termination = termination;
  result.negative_termination = termination;
  result.reason = reason;
  SetFinalFacts(result);
  result.valid = true;
}

struct RayExpansion {
  double boundary = 0.0;
  TubeRayTermination termination = TubeRayTermination::SEARCH_EXTENT;
};

// Expand exactly one side of the component which contains delta = 0.  The
// first non-safe query is a hard boundary: we never resume searching on the
// other side of it, so a disconnected non-zero free region cannot enter the
// production interval.
RayExpansion ExpandFromZero(const Eigen::Vector3d& origin,
                            const Eigen::Vector3d& normal,
                            const ClearanceQuery& query,
                            const double required_radius,
                            const double direction,
                            const double extent,
                            const double step,
                            const double tolerance) {
  RayExpansion expansion;
  if (extent <= 0.0) return expansion;

  double safe_distance = 0.0;
  while (safe_distance < extent) {
    const double next_distance = std::min(extent, safe_distance + step);
    if (next_distance <= safe_distance) break;
    const double next_delta = direction * next_distance;
    const ClearanceQueryResult sample = query(
        origin + normal * next_delta, required_radius);
    if (!ClearanceSafe(sample, required_radius)) {
      const double safe_delta = direction * safe_distance;
      expansion.boundary = RefineSafeBoundary(
          origin, normal, query, required_radius, next_delta, safe_delta,
          tolerance);
      expansion.termination = ToTermination(sample.status);
      return expansion;
    }
    safe_distance = next_distance;
  }

  expansion.boundary = direction * safe_distance;
  return expansion;
}

}  // namespace

double RobustTubeMargins::fullEffectiveRadius() const {
  return uav_radius + map_uncertainty + localization_uncertainty +
      tracking_error_bound;
}

double RobustTubeMargins::residualEffectiveRadius() const {
  return uav_radius + (map_uncertainty - preincluded_map_uncertainty) +
      localization_uncertainty + tracking_error_bound;
}

double RobustTubeMargins::effectiveRadius() const {
  return residualEffectiveRadius();
}

TubeCrossSectionSolver::TubeCrossSectionSolver(
    const TubeCrossSectionConfig& config)
    : config_(config) {}

bool TubeCrossSectionSolver::configurationValid() const {
  if (!IsFinite(config_.search_extent) || config_.search_extent <= 0.0 ||
      !IsFinite(config_.ray_step) || config_.ray_step <= 0.0 ||
      config_.ray_step > config_.search_extent ||
      !IsFinite(config_.boundary_tolerance) ||
      config_.boundary_tolerance <= 0.0 ||
      config_.boundary_tolerance > config_.ray_step ||
      !IsFinite(config_.regularity_margin) ||
      config_.regularity_margin <= 0.0 || config_.regularity_margin >= 1.0 ||
      !IsFinite(config_.curvature_epsilon) || config_.curvature_epsilon < 0.0 ||
      !IsFinite(config_.planner_safe_distance) ||
      config_.planner_safe_distance < 0.0) {
    return false;
  }
  const double count = std::ceil(2.0 * config_.search_extent / config_.ray_step);
  return IsFinite(count) && count >= 1.0 &&
      count <= static_cast<double>(kMaxCrossSectionSamples);
}

TubeCrossSectionResult TubeCrossSectionSolver::solve(
    const TubeCrossSectionInput& input) const {
  TubeCrossSectionResult result;
  // Deprecated accounting remains observable, but invalid legacy values must
  // not suppress a planner-authoritative production Tube.
  if (ValidMargins(config_.margins)) {
    result.full_effective_radius = config_.margins.fullEffectiveRadius();
    result.preincluded_map_uncertainty =
        config_.margins.preincluded_map_uncertainty;
  }
  result.residual_effective_radius = config_.planner_safe_distance;
  result.effective_radius = result.residual_effective_radius;
  if (!configurationValid() || !input.clearance_query) {
    result.reason = TubeCrossSectionReason::INVALID_CONFIGURATION;
    return result;
  }
  const double normal_norm = input.N.norm();
  if (!IsFinite(input.p) || !IsFinite(input.N) || !IsFinite(normal_norm) ||
      normal_norm <= kNormalEpsilon) {
    result.reason = TubeCrossSectionReason::INVALID_GEOMETRY;
    return result;
  }
  if (!IsFinite(input.curvature)) {
    result.reason = TubeCrossSectionReason::CURVATURE_NUMERICAL_FAILURE;
    return result;
  }

  const Eigen::Vector3d normal = input.N / normal_norm;
  result.lower_curvature = -config_.search_extent;
  result.upper_curvature = config_.search_extent;
  if (input.curvature > config_.curvature_epsilon) {
    result.upper_curvature =
        (1.0 - config_.regularity_margin) / input.curvature;
  } else if (input.curvature < -config_.curvature_epsilon) {
    result.lower_curvature =
        (1.0 - config_.regularity_margin) / input.curvature;
  }
  if (!IsFinite(result.lower_curvature) || !IsFinite(result.upper_curvature)) {
    result.reason = TubeCrossSectionReason::CURVATURE_NUMERICAL_FAILURE;
    return result;
  }

  const ClearanceQueryResult centre = input.clearance_query(
      input.p, result.residual_effective_radius);
  if (!ClearanceSafe(centre, result.residual_effective_radius)) {
    // The planner remains authoritative for delta = 0.  A stale, unavailable,
    // or stricter Tube snapshot therefore removes only offset capacity.
    SetZeroOnly(CenterReason(centre.status), ToTermination(centre.status), result);
    return result;
  }

  const double positive_extent = std::min(
      config_.search_extent, std::max(0.0, result.upper_curvature));
  const double negative_extent = std::min(
      config_.search_extent, std::max(0.0, -result.lower_curvature));
  const RayExpansion positive = ExpandFromZero(
      input.p, normal, input.clearance_query, result.residual_effective_radius,
      1.0, positive_extent, config_.ray_step, config_.boundary_tolerance);
  const RayExpansion negative = ExpandFromZero(
      input.p, normal, input.clearance_query, result.residual_effective_radius,
      -1.0, negative_extent, config_.ray_step, config_.boundary_tolerance);

  result.lower_obstacle = negative.boundary;
  result.upper_obstacle = positive.boundary;
  result.lower_final = std::max(result.lower_obstacle, result.lower_curvature);
  result.upper_final = std::min(result.upper_obstacle, result.upper_curvature);
  if (result.lower_final > 0.0 || result.upper_final < 0.0 ||
      result.lower_final > result.upper_final) {
    SetZeroOnly(TubeCrossSectionReason::EMPTY_AFTER_CURVATURE_INTERSECTION,
                TubeRayTermination::UNAVAILABLE, result);
    return result;
  }
  // A ray may stop immediately, which produces a valid one-sided or zero-only
  // interval.  The bounds deliberately never bridge past that first stop.
  result.lower_final = std::min(0.0, result.lower_final);
  result.upper_final = std::max(0.0, result.upper_final);
  result.c_plus_raw = std::max(0.0, result.upper_final);
  result.c_minus_raw = std::max(0.0, -result.lower_final);
  result.positive_termination = positive.termination;
  result.negative_termination = negative.termination;
  SetFinalFacts(result);
  result.valid = true;
  result.reason = TubeCrossSectionReason::NONE;
  return result;
}

}  // namespace phase_offset_navigation
