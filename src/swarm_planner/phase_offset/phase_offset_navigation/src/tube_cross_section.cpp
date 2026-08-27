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

ClearanceQueryResult ConstructionQuery(
    const TubeCrossSectionInput& input,
    const Eigen::Vector3d& point,
    const double delta,
    const double required_radius) {
  if (input.directional_query_count != nullptr) {
    ++(*input.directional_query_count);
  }
  if (input.max_bounded_construction_abs_delta != nullptr &&
      IsFinite(delta)) {
    *input.max_bounded_construction_abs_delta = std::max(
        *input.max_bounded_construction_abs_delta, std::abs(delta));
  }
  if (!input.clearance_query) {
    ClearanceQueryResult result;
    result.status = DistanceStatus::UNAVAILABLE;
    return result;
  }
  return input.clearance_query(point, required_radius);
}

bool IntersectFull3DRegularity(const TubeCrossSectionInput& input,
                               const TubeCrossSectionConfig& config,
                               const double selected_delta,
                               double& lower, double& upper) {
  const bool bound = IsFinite(input.minimum_reference_speed) &&
      input.minimum_reference_speed > 0.0 && IsFinite(input.p_w) &&
      IsFinite(input.N_w) && input.p_w.norm() > kNormalEpsilon;
  if (!bound) return true;
  const double minimum = input.minimum_reference_speed;
  const double a = input.N_w.squaredNorm();
  const double b = 2.0 * input.p_w.dot(input.N_w);
  const double c = input.p_w.squaredNorm() - minimum * minimum;
  if (!IsFinite(a) || !IsFinite(b) || !IsFinite(c)) return false;
  if (a <= 1e-14) {
    if (c >= 0.0) return true;
    if (std::abs(b) <= 1e-14) return false;
    const double root = -c / b;
    if (!IsFinite(root)) return false;
    const double value = b * selected_delta + c;
    if (value < 0.0) return false;
    if (b > 0.0) lower = std::max(lower, root);
    else upper = std::min(upper, root);
    return lower <= upper + config.boundary_tolerance;
  }
  const double discriminant = b * b - 4.0 * a * c;
  if (!IsFinite(discriminant)) return false;
  if (discriminant <= 0.0) {
    if (c >= 0.0) return true;
    return false;
  }
  const double root = std::sqrt(discriminant);
  double r0 = (-b - root) / (2.0 * a);
  double r1 = (-b + root) / (2.0 * a);
  if (!IsFinite(r0) || !IsFinite(r1)) return false;
  if (r0 > r1) std::swap(r0, r1);
  // Retain exactly the safe connected component containing the selected
  // current delta.  For c<0 the interval between roots is unsafe; for c>=0
  // it is the unsafe interval when the discriminant is positive.  Never
  // convexify the two outside components.
  const bool selected_in_unsafe = selected_delta > r0 && selected_delta < r1;
  if (selected_in_unsafe) return false;
  if (selected_delta <= r0) upper = std::min(upper, r0);
  else lower = std::max(lower, r1);
  return lower <= upper + config.boundary_tolerance;
}

// Returns the safe side of a safe/unsafe transition.  The query's bounded
// ball contract means an UNKNOWN result is never crossed as if it were free.
double RefineSafeBoundary(const Eigen::Vector3d& origin,
                          const Eigen::Vector3d& normal,
                          const TubeCrossSectionInput& input,
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
    const ClearanceQueryResult result = ConstructionQuery(
        input, origin + normal * middle, middle, required_radius);
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
RayExpansion ExpandFromAnchor(const Eigen::Vector3d& origin,
                            const Eigen::Vector3d& normal,
                            const TubeCrossSectionInput& input,
                            const double required_radius,
                            const double seed_delta,
                            const double direction,
                            const double nominal_lower,
                            const double nominal_upper,
                            const double step,
                            const double tolerance) {
  RayExpansion expansion;
  const double endpoint = direction > 0.0 ? nominal_upper : nominal_lower;
  const double extent = direction > 0.0
      ? endpoint - seed_delta : seed_delta - endpoint;
  if (!IsFinite(extent) || extent < 0.0) return expansion;

  double safe_distance = 0.0;
  while (safe_distance < extent) {
    const double next_distance = std::min(extent, safe_distance + step);
    if (next_distance <= safe_distance) break;
    const double next_delta = seed_delta + direction * next_distance;
    const ClearanceQueryResult sample = ConstructionQuery(
        input, origin + normal * next_delta, next_delta, required_radius);
    if (!ClearanceSafe(sample, required_radius)) {
      const double safe_delta = seed_delta + direction * safe_distance;
      expansion.boundary = RefineSafeBoundary(
          origin, normal, input, required_radius, next_delta, safe_delta,
          tolerance);
      expansion.termination = ToTermination(sample.status);
      return expansion;
    }
    safe_distance = next_distance;
  }

  expansion.boundary = endpoint;
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

const char* tubeNominalWidthSourceName(
    const TubeNominalWidthSource source) {
  switch (source) {
    case TubeNominalWidthSource::DEFAULT_ABSENT:
      return "DEFAULT_ABSENT";
    case TubeNominalWidthSource::EXPLICIT_PARAMETER:
      return "EXPLICIT_PARAMETER";
  }
  return "DEFAULT_ABSENT";
}

const char* nominalWidthSourceName(
    const TubeNominalWidthSource source) {
  return tubeNominalWidthSourceName(source);
}

TubeCrossSectionSolver::TubeCrossSectionSolver(
    const TubeCrossSectionConfig& config)
    : config_(config) {}

bool TubeCrossSectionSolver::configurationValid() const {
  if (!IsFinite(config_.nominal_half_width) ||
      config_.nominal_half_width <= 0.0 ||
      !IsFinite(config_.ray_step) || config_.ray_step <= 0.0 ||
      !IsFinite(config_.boundary_tolerance) ||
      config_.boundary_tolerance <= 0.0 ||
      config_.boundary_tolerance > config_.ray_step ||
      !IsFinite(config_.regularity_margin) ||
      config_.regularity_margin <= 0.0 || config_.regularity_margin >= 1.0 ||
      !IsFinite(config_.curvature_epsilon) || config_.curvature_epsilon < 0.0 ||
      !IsFinite(config_.planner_safe_distance) ||
      config_.planner_safe_distance < 0.0 ||
      !IsFinite(config_.minimum_reference_speed) ||
      config_.minimum_reference_speed <= 0.0) {
    return false;
  }
  const double count = std::ceil(2.0 * config_.nominal_half_width /
                                 config_.ray_step);
  return IsFinite(count) && count >= 1.0 &&
      count <= static_cast<double>(kMaxCrossSectionSamples);
}

TubeCrossSectionResult TubeCrossSectionSolver::solve(
    const TubeCrossSectionInput& input) const {
  TubeCrossSectionResult result;
  result.nominal_half_width = config_.nominal_half_width;
  result.nominal_width_source = config_.nominal_width_source;
  result.nominal_width_legacy_conflict =
      config_.nominal_width_legacy_conflict;
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
  const double requested_delta = input.current_delta_valid
      ? input.current_delta : 0.0;
  // A retained current offset outside the bounded nominal interval is never
  // queried as a transverse construction point.  Use the exact planner
  // centreline anchor instead; the owner above the solver restores the
  // retained metadata and lets EpochManager report CURRENT_OFFSET_OUTSIDE.
  const double selected_delta = requested_delta >= -config_.nominal_half_width &&
      requested_delta <= config_.nominal_half_width ? requested_delta : 0.0;
  const double nominal_lower = -config_.nominal_half_width;
  const double nominal_upper = config_.nominal_half_width;
  result.lower_curvature = nominal_lower;
  result.upper_curvature = nominal_upper;
  if (IsFinite(input.minimum_reference_speed) &&
      input.minimum_reference_speed > 0.0 && IsFinite(input.p_w) &&
      IsFinite(input.N_w) && input.p_w.norm() > kNormalEpsilon) {
    if (!IntersectFull3DRegularity(input, config_, selected_delta,
                                   result.lower_curvature,
                                   result.upper_curvature)) {
      result.reason = selected_delta == 0.0
          ? TubeCrossSectionReason::REGULARITY_ZERO_UNSAFE
          : TubeCrossSectionReason::EMPTY_AFTER_CURVATURE_INTERSECTION;
      result.valid = false;
      return result;
    }
    result.regularity_proven = true;
    result.regularity_speed_at_zero = input.p_w.norm();
    result.regularity_speed_min = input.minimum_reference_speed;
  } else if (input.curvature > config_.curvature_epsilon) {
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

  const ClearanceQueryResult centre = ConstructionQuery(
      input, input.p + normal * selected_delta, selected_delta,
      result.residual_effective_radius);
  if (!ClearanceSafe(centre, result.residual_effective_radius)) {
    // The planner remains authoritative for delta = 0.  A stale, unavailable,
    // or stricter Tube snapshot therefore removes only offset capacity.
    if (selected_delta == 0.0) {
      SetZeroOnly(CenterReason(centre.status), ToTermination(centre.status), result);
    } else {
      result.reason = CenterReason(centre.status);
      result.positive_termination = ToTermination(centre.status);
      result.negative_termination = ToTermination(centre.status);
      result.valid = false;
    }
    return result;
  }

  // Obstacles clip the absolute nominal endpoints independently.  Expansion
  // starts at the selected connected-component anchor (zero for the bounded
  // out-of-range transition) but always terminates at [-rho_nom,+rho_nom],
  // never at a seed-relative travel distance.
  const RayExpansion positive = ExpandFromAnchor(
      input.p, normal, input, result.residual_effective_radius,
      selected_delta, 1.0, nominal_lower, nominal_upper,
      config_.ray_step, config_.boundary_tolerance);
  const RayExpansion negative = ExpandFromAnchor(
      input.p, normal, input, result.residual_effective_radius,
      selected_delta, -1.0, nominal_lower, nominal_upper,
      config_.ray_step, config_.boundary_tolerance);

  result.lower_obstacle = negative.boundary;
  result.upper_obstacle = positive.boundary;
  result.lower_final = std::max(result.lower_obstacle, result.lower_curvature);
  result.upper_final = std::min(result.upper_obstacle, result.upper_curvature);
  if (result.lower_final > result.upper_final ||
      selected_delta < result.lower_final - config_.boundary_tolerance ||
      selected_delta > result.upper_final + config_.boundary_tolerance) {
    result.reason = TubeCrossSectionReason::EMPTY_AFTER_CURVATURE_INTERSECTION;
    result.valid = false;
    return result;
  }
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
