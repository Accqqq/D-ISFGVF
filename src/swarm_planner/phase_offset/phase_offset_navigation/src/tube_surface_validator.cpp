#include "phase_offset_navigation/tube_surface_validator.h"

#include "phase_offset_navigation/tube_filter.h"

#include <phase_offset_core/geometry.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace phase_offset_navigation {
namespace {

constexpr double kEpsilon = 1e-10;
constexpr double kAnchorTolerance = 1e-8;

bool IsFinite(const double value) {
  return std::isfinite(value);
}

bool IsFinite(const Eigen::Vector3d& value) {
  return value.allFinite();
}

TubeStopReason StopReason(const DistanceStatus status) {
  switch (status) {
    case DistanceStatus::UNAVAILABLE:
      return TubeStopReason::UNAVAILABLE;
    case DistanceStatus::OUT_OF_MAP:
      return TubeStopReason::OUT_OF_MAP;
    case DistanceStatus::UNKNOWN:
      return TubeStopReason::UNKNOWN;
    case DistanceStatus::OCCUPIED:
      return TubeStopReason::OCCUPIED;
    case DistanceStatus::KNOWN_FREE:
      return TubeStopReason::INSUFFICIENT_CLEARANCE;
  }
  return TubeStopReason::UNKNOWN;
}

struct SurfacePoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  double w = 0.0;
};

using SurfacePointArray = std::array<SurfacePoint, 9U>;

struct SurfaceCell {
  double w0 = 0.0;
  double w1 = 0.0;
  double v0 = 0.0;
  double v1 = 1.0;
  int depth = 0;
};

struct CellCoverBreakdown {
  double fixed_cover = 0.0;
  double w_reducible_cover = 0.0;
  double v_reducible_cover = 0.0;
  bool decomposable = false;
};

struct ValidationContext {
  TubeProfile* profile = nullptr;
  const PathStateQuery* path_state_query = nullptr;
  const PathCellBoundQuery* path_cell_bound_query = nullptr;
  const ClearanceQuery* clearance_query = nullptr;
  double snapshot_resolution = 0.0;
  double current_w = 0.0;
  double required_clearance = 0.0;
  double regularity_margin = 0.0;
  double cover_epsilon = 0.0;
  const TubeSurfaceValidatorConfig* config = nullptr;
  phase_offset_core::GeometryEvaluator geometry_evaluator;
  TubeSurfaceValidationResult* result = nullptr;
};

void RecordFailure(ValidationContext& context,
                   TubeStopReason reason,
                   double w);

bool CertifiedCellCoverRadius(ValidationContext& context,
                               const SurfaceCell& cell,
                               double& cover_radius,
                               bool& certificate_attempted,
                               CellCoverBreakdown& breakdown) {
  cover_radius = 0.0;
  certificate_attempted = false;
  breakdown = CellCoverBreakdown();
  if (!context.profile->cell_geometry_certified ||
      context.path_cell_bound_query == nullptr ||
      !(*context.path_cell_bound_query) ||
      cell.w1 <= cell.w0 + kEpsilon) {
    return false;
  }
  certificate_attempted = true;
  phase_offset_core::PathCellGeometryCertificate certificate;
  if (!(*context.path_cell_bound_query)(cell.w0, cell.w1, certificate) ||
      !phase_offset_core::pathCellGeometryCertificateIsComplete(certificate) ||
      std::abs(certificate.w0 - cell.w0) > kAnchorTolerance ||
      std::abs(certificate.w1 - cell.w1) > kAnchorTolerance) {
    return false;
  }
  TubeBounds first;
  TubeBounds second;
  if (!TubeFilter::query(*context.profile, cell.w0, first) ||
      !TubeFilter::query(*context.profile, cell.w1, second) || !first.valid ||
      !second.valid || !IsFinite(first.lower) || !IsFinite(first.upper) ||
      !IsFinite(second.lower) || !IsFinite(second.upper) ||
      first.lower > first.upper + kEpsilon ||
      second.lower > second.upper + kEpsilon) {
    return false;
  }
  const double h = cell.w1 - cell.w0;
  const double maximum_delta = std::max({std::abs(first.lower),
                                          std::abs(first.upper),
                                          std::abs(second.lower),
                                          std::abs(second.upper)});
  const double delta_slope = std::max(std::abs(second.lower - first.lower),
                                      std::abs(second.upper - first.upper)) / h;
  const double maximum_width = std::max(first.upper - first.lower,
                                        second.upper - second.lower);
  const double v_span = cell.v1 - cell.v0;
  const double w_reducible_cover =
      certificate.midpoint_position_variation_bound +
      0.5 * maximum_delta * certificate.normal_variation_bound +
      0.5 * delta_slope * h;
  const double v_reducible_cover = 0.5 * maximum_width * v_span;
  const double geometric_cover = certificate.midpoint_position_variation_bound +
      0.5 * maximum_delta * certificate.normal_variation_bound +
      0.5 * delta_slope * h + 0.5 * maximum_width * v_span;
  // The only discretization residual here is the half-voxel support of the
  // immutable cloud.  Builder's certificate residual intentionally did not
  // charge it, so this adds it once.
  cover_radius = geometric_cover + 0.5 * context.snapshot_resolution;
  breakdown.fixed_cover = 0.5 * context.snapshot_resolution;
  breakdown.w_reducible_cover = w_reducible_cover;
  breakdown.v_reducible_cover = v_reducible_cover;
  // Point samples cannot prove that an intermediate offset surface remains
  // a valid GeometryEvaluator reference.  Use the certificate directly for
  // the two delayed invariants before accepting the smaller cover.
  const double active_speed_lower = certificate.inf_p_w_norm -
      certificate.sup_N_w_norm * maximum_delta;
  const double regularity_lower = 1.0 -
      certificate.sup_abs_curvature * maximum_delta;
  if (!IsFinite(active_speed_lower) || !IsFinite(regularity_lower) ||
      active_speed_lower <= 1e-8 ||
      regularity_lower < context.regularity_margin) {
    return false;
  }
  const bool valid = IsFinite(maximum_delta) && IsFinite(delta_slope) &&
      IsFinite(maximum_width) && IsFinite(v_span) &&
      IsFinite(geometric_cover) && geometric_cover >= 0.0 &&
      IsFinite(cover_radius) && cover_radius >= 0.0;
  breakdown.decomposable = valid;
  return valid;
}

void RecordFailure(ValidationContext& context,
                   const TubeStopReason reason,
                   const double w) {
  if (context.result->first_failure_reason != TubeStopReason::NONE) return;
  context.result->first_failure_reason = reason;
  context.result->first_failure_w = IsFinite(w) ? w : 0.0;
}

bool IsProfileKnot(const TubeProfile& profile, const double w) {
  for (const TubeRawSample& sample : profile.samples) {
    if (IsFinite(sample.w) && std::abs(sample.w - w) <= kAnchorTolerance) {
      return true;
    }
  }
  return false;
}

void RecordKnotEvidence(ValidationContext& context,
                        const double w,
                        const double cover_radius,
                        const double requested_radius,
                        const bool clearance_safe) {
  if (!IsFinite(w) || !IsProfileKnot(*context.profile, w)) return;
  TubeBounds bounds;
  const bool bounds_valid = TubeFilter::query(*context.profile, w, bounds) &&
      bounds.valid && IsFinite(bounds.lower) && IsFinite(bounds.upper);
  const bool contains_zero = bounds_valid && bounds.lower <= 0.0 &&
      0.0 <= bounds.upper;
  TubeValidatorKnotEvidence* evidence = nullptr;
  for (TubeValidatorKnotEvidence& existing :
       context.profile->validator_knot_evidence) {
    if (std::abs(existing.w - w) <= kAnchorTolerance) {
      evidence = &existing;
      break;
    }
  }
  if (evidence == nullptr) {
    TubeValidatorKnotEvidence created;
    created.w = w;
    context.profile->validator_knot_evidence.push_back(created);
    evidence = &context.profile->validator_knot_evidence.back();
  }
  evidence->observed = true;
  evidence->max_cover_radius = std::max(evidence->max_cover_radius, cover_radius);
  evidence->max_requested_clearance = std::max(
      evidence->max_requested_clearance, requested_radius);
  evidence->filtered_contains_zero = contains_zero;
  evidence->zero_surface_covered = evidence->zero_surface_covered ||
      (clearance_safe && contains_zero);
}

bool ZeroCentrelineCoveredAtAllProfileKnots(const TubeProfile& profile) {
  if (profile.samples.empty()) return false;
  for (const TubeRawSample& sample : profile.samples) {
    const TubeValidatorKnotEvidence* evidence = nullptr;
    for (const TubeValidatorKnotEvidence& candidate :
         profile.validator_knot_evidence) {
      if (std::abs(candidate.w - sample.w) <= kAnchorTolerance) {
        evidence = &candidate;
        break;
      }
    }
    if (evidence == nullptr || !evidence->observed ||
        !evidence->filtered_contains_zero ||
        !evidence->zero_surface_covered) {
      return false;
    }
  }
  return true;
}

bool SurfacePointArgumentsValid(ValidationContext& context,
                                const double w,
                                const double v) {
  if (!IsFinite(w) || !IsFinite(v) || v < -kEpsilon || v > 1.0 + kEpsilon) {
    RecordFailure(context, TubeStopReason::INVALID_PATH, w);
    return false;
  }
  return true;
}

bool EvaluateSurfaceRow(ValidationContext& context,
                        const double w,
                        const double v_values[3U],
                        SurfacePointArray& points,
                        std::size_t& point_index) {
  // Preserve the first point's argument check before the row-level queries.
  if (!SurfacePointArgumentsValid(context, w, v_values[0U])) return false;

  TubeBounds bounds;
  if (!TubeFilter::query(*context.profile, w, bounds) || !bounds.valid ||
      !IsFinite(bounds.lower) || !IsFinite(bounds.upper) ||
      bounds.lower > bounds.upper + kEpsilon) {
    RecordFailure(context, TubeStopReason::INVALID_PATH, w);
    return false;
  }
  phase_offset_core::PathDifferentialState path;
  if (!(*context.path_state_query)(w, path) || !path.valid ||
      !IsFinite(path.w) || std::abs(path.w - w) > kAnchorTolerance) {
    RecordFailure(context, TubeStopReason::INVALID_PATH, w);
    return false;
  }

  phase_offset_core::PreparedPathGeometry prepared;
  context.geometry_evaluator.preparePath(path, prepared);
  for (std::size_t value_index = 0U; value_index < 3U; ++value_index) {
    const double v = v_values[value_index];
    if (value_index != 0U && !SurfacePointArgumentsValid(context, w, v)) {
      return false;
    }
    const double clamped_v = std::max(0.0, std::min(1.0, v));
    const double delta = bounds.lower + clamped_v * (bounds.upper - bounds.lower);
    phase_offset_core::PreparedReferenceResult geometry;
    if (!context.geometry_evaluator.evaluatePreparedReference(
            prepared, path.p, delta, geometry) ||
        !geometry.valid || !IsFinite(geometry.r)) {
      RecordFailure(context, TubeStopReason::REGULARITY, w);
      return false;
    }
    points[point_index].point = geometry.r;
    points[point_index].w = w;
    ++point_index;
  }
  return true;
}

bool CollectCellPoints(ValidationContext& context,
                       const SurfaceCell& cell,
                       SurfacePointArray& points,
                       double& cover_radius,
                       CellCoverBreakdown& breakdown) {
  breakdown = CellCoverBreakdown();
  ++context.result->geometry_cell_count;
  context.result->max_depth_observed = std::max(
      context.result->max_depth_observed, cell.depth);
  const double w_values[] = {cell.w0, 0.5 * (cell.w0 + cell.w1), cell.w1};
  const double v_values[] = {cell.v0, 0.5 * (cell.v0 + cell.v1), cell.v1};
  std::size_t point_index = 0U;
  for (const double w : w_values) {
    if (!EvaluateSurfaceRow(context, w, v_values, points, point_index)) return false;
  }
  const SurfacePoint& center = points[4U];
  double sampled_radius = 0.0;
  for (const SurfacePoint& point : points) {
    const double distance = (point.point - center.point).norm();
    if (!IsFinite(distance)) {
      RecordFailure(context, TubeStopReason::INVALID_PATH, cell.w0);
      return false;
    }
    sampled_radius = std::max(sampled_radius, distance);
  }
  bool certificate_attempted = false;
  if (CertifiedCellCoverRadius(context, cell, cover_radius,
                               certificate_attempted, breakdown)) {
    if (!context.result->cover_accounting_observed) {
      context.result->min_cover_radius = cover_radius;
      context.result->max_cover_radius = cover_radius;
      context.result->cover_accounting_observed = true;
    } else {
      context.result->min_cover_radius = std::min(
          context.result->min_cover_radius, cover_radius);
      context.result->max_cover_radius = std::max(
          context.result->max_cover_radius, cover_radius);
    }
    return true;
  }
  if (certificate_attempted) {
    // A Builder profile with zero local inset has explicitly opted into the
    // certificate contract.  A failed regularity/speed/cell proof cannot
    // silently fall back to the empirical 3x3 cover, which is not a proof.
    RecordFailure(context, TubeStopReason::REGULARITY, cell.w0);
    return false;
  }
  // Without a complete immutable certificate preserve the established
  // fixed-inset sampled cover.  The exact current-anchor cell is the one
  // proven degenerate branch: at fixed w, r(v)=p+N*delta(v) is affine and the
  // closed-voxel query already owns voxel support, so only sampled geometry is
  // charged there.  Nondegenerate sampled fallback cells retain the fixed
  // half-voxel term; G4 did not prove that term removable for them.
  const bool exact_current_anchor = cell.w0 == context.current_w &&
      cell.w1 == context.current_w;
  cover_radius = 1.1 * sampled_radius +
      (exact_current_anchor ? 0.0 : 0.5 * context.snapshot_resolution);
  if (!IsFinite(cover_radius) || cover_radius < 0.0) {
    RecordFailure(context, TubeStopReason::INVALID_PATH, cell.w0);
    return false;
  }
  if (!context.result->cover_accounting_observed) {
    context.result->min_cover_radius = cover_radius;
    context.result->max_cover_radius = cover_radius;
    context.result->cover_accounting_observed = true;
  } else {
    context.result->min_cover_radius = std::min(
        context.result->min_cover_radius, cover_radius);
    context.result->max_cover_radius = std::max(
        context.result->max_cover_radius, cover_radius);
  }
  return true;
}

bool CellClearancePasses(ValidationContext& context,
                         const SurfacePointArray& points,
                         const double cover_radius) {
  const double requested_radius = context.required_clearance + cover_radius +
      context.cover_epsilon;
  if (IsFinite(cover_radius)) {
    if (!context.result->cover_accounting_observed) {
      context.result->min_cover_radius = cover_radius;
      context.result->max_cover_radius = cover_radius;
      context.result->cover_accounting_observed = true;
    } else {
      context.result->min_cover_radius = std::min(
          context.result->min_cover_radius, cover_radius);
      context.result->max_cover_radius = std::max(
          context.result->max_cover_radius, cover_radius);
    }
  }
  if (IsFinite(requested_radius)) {
    context.result->max_requested_clearance = std::max(
        context.result->max_requested_clearance, requested_radius);
  }
  if (!IsFinite(requested_radius) || requested_radius < context.required_clearance) {
    RecordFailure(context, TubeStopReason::INVALID_PATH, points.empty() ? 0.0 : points.front().w);
    return false;
  }
  if (context.result->query_sample_count + points.size() >
      context.config->max_query_samples) {
    context.result->limit_exceeded = true;
    RecordFailure(context, TubeStopReason::INSUFFICIENT_CLEARANCE,
                  points.empty() ? 0.0 : points.front().w);
    return false;
  }
  ++context.result->clearance_leaf_cell_count;
  bool all_safe = true;
  for (const SurfacePoint& point : points) {
    ++context.result->query_sample_count;
    const ClearanceQueryResult query = (*context.clearance_query)(
        point.point, requested_radius);
    const bool safe = query.status == DistanceStatus::KNOWN_FREE &&
        query.clearance_certified && IsFinite(query.clearance) &&
        query.clearance >= requested_radius;
    if (!safe) {
      RecordFailure(context, StopReason(query.status), point.w);
      all_safe = false;
      continue;
    }
    context.result->min_clearance_margin = std::min(
        context.result->min_clearance_margin,
        query.clearance - context.required_clearance);
  }
  // A successful cell is a continuous-cover fact for each raw/filter knot it
  // touches.  Record the actual cover and requested radius rather than trying
  // to infer them later from an aggregate maximum.
  for (const SurfacePoint& point : points) {
    RecordKnotEvidence(context, point.w, cover_radius, requested_radius,
                       all_safe);
  }
  return all_safe;
}

void RecordSplitEvidence(ValidationContext& context,
                         const bool split_w,
                         const bool split_v,
                         const bool anisotropic) {
  if (split_w && split_v) {
    ++context.result->split_both_count;
  } else if (split_w) {
    ++context.result->split_w_count;
  } else if (split_v) {
    ++context.result->split_v_count;
  }
  if (anisotropic && (split_w != split_v)) {
    ++context.result->anisotropic_split_count;
  }
}

void ChooseProofDerivedSplit(const CellCoverBreakdown& breakdown,
                             const double target_cover,
                             const bool can_split_w,
                             const bool can_split_v,
                             bool& split_w,
                             bool& split_v) {
  split_w = false;
  split_v = false;
  if (!can_split_w && !can_split_v) return;
  if (!breakdown.decomposable) {
    split_w = can_split_w;
    split_v = can_split_v;
    return;
  }

  const double allowable_reducible = target_cover - breakdown.fixed_cover;
  const bool w_exceeds = breakdown.w_reducible_cover >
      allowable_reducible;
  const bool v_exceeds = breakdown.v_reducible_cover >
      allowable_reducible;
  if (w_exceeds && v_exceeds) {
    split_w = can_split_w;
    split_v = can_split_v;
  } else if (w_exceeds) {
    split_w = can_split_w;
  } else if (v_exceeds) {
    split_v = can_split_v;
  } else if (breakdown.w_reducible_cover + breakdown.v_reducible_cover >
             allowable_reducible) {
    // Exact equality is intentionally resolved in favour of w to keep the
    // split order deterministic without adding hysteresis.
    if (breakdown.w_reducible_cover >= breakdown.v_reducible_cover) {
      split_w = can_split_w;
    } else {
      split_v = can_split_v;
    }
  } else {
    // This branch is only reachable through floating-point boundary
    // roundoff.  Keep the conservative proof-preserving fallback.
    split_w = can_split_w;
    split_v = can_split_v;
  }

  // If the proof-selected dimension is unavailable, use the other available
  // dimension.  No independent depth is introduced by this fallback.
  if (!split_w && !split_v) {
    if (can_split_w) {
      split_w = true;
    } else if (can_split_v) {
      split_v = true;
    }
  }
}

bool ValidateCell(ValidationContext& context, const SurfaceCell& cell) {
  SurfacePointArray points;
  double cover_radius = 0.0;
  CellCoverBreakdown breakdown;
  const bool geometry_valid = CollectCellPoints(
      context, cell, points, cover_radius, breakdown);
  const double target_cover = std::max(context.required_clearance,
                                       context.snapshot_resolution);
  bool clearance_safe = false;
  if (geometry_valid && cover_radius <= target_cover) {
    // Accepted leaves retain the exact historical 3x3 proof.  A cell whose
    // certified cover is already too large cannot be accepted as-is, so its
    // parent clearance samples would be redundant and are skipped.
    clearance_safe = CellClearancePasses(context, points, cover_radius);
  } else if (geometry_valid && cover_radius > target_cover) {
    ++context.result->prequery_cover_split_count;
  }
  if (geometry_valid && clearance_safe) {
    return true;
  }
  if (cell.depth >= context.config->max_subdivision_depth) {
    context.result->limit_exceeded = true;
    if (context.result->first_failure_reason == TubeStopReason::NONE) {
      RecordFailure(context, TubeStopReason::INSUFFICIENT_CLEARANCE,
                    0.5 * (cell.w0 + cell.w1));
    }
    return false;
  }

  const bool can_split_w = cell.w1 - cell.w0 > kEpsilon;
  const bool can_split_v = cell.v1 - cell.v0 > kEpsilon;
  bool split_w = false;
  bool split_v = false;
  const bool cover_too_large = geometry_valid && cover_radius > target_cover;
  if (cover_too_large) {
    ChooseProofDerivedSplit(breakdown, target_cover, can_split_w,
                            can_split_v, split_w, split_v);
    RecordSplitEvidence(context, split_w, split_v,
                        breakdown.decomposable);
  } else {
    // A clearance/geometry failure after a potentially acceptable cover keeps
    // the historical conservative both-dimension subdivision.
    split_w = can_split_w;
    split_v = can_split_v;
    RecordSplitEvidence(context, split_w, split_v, false);
  }
  if (!split_w && !split_v) {
    context.result->limit_exceeded = true;
    RecordFailure(context, TubeStopReason::INSUFFICIENT_CLEARANCE, cell.w0);
    return false;
  }
  const double w_mid = 0.5 * (cell.w0 + cell.w1);
  const double v_mid = 0.5 * (cell.v0 + cell.v1);
  if (split_w && split_v) {
    return ValidateCell(context, {cell.w0, w_mid, cell.v0, v_mid, cell.depth + 1}) &&
        ValidateCell(context, {cell.w0, w_mid, v_mid, cell.v1, cell.depth + 1}) &&
        ValidateCell(context, {w_mid, cell.w1, cell.v0, v_mid, cell.depth + 1}) &&
        ValidateCell(context, {w_mid, cell.w1, v_mid, cell.v1, cell.depth + 1});
  }
  if (split_w) {
    return ValidateCell(context, {cell.w0, w_mid, cell.v0, cell.v1, cell.depth + 1}) &&
        ValidateCell(context, {w_mid, cell.w1, cell.v0, cell.v1, cell.depth + 1});
  }
  return ValidateCell(context, {cell.w0, cell.w1, cell.v0, v_mid, cell.depth + 1}) &&
      ValidateCell(context, {cell.w0, cell.w1, v_mid, cell.v1, cell.depth + 1});
}

std::size_t FindAnchor(const TubeProfile& profile, const double current_w) {
  std::size_t selected = profile.samples.size();
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < profile.samples.size(); ++index) {
    const double error = std::abs(profile.samples[index].w - current_w);
    if (IsFinite(error) && error <= kAnchorTolerance &&
        (error < best - kEpsilon ||
         (std::abs(error - best) <= kEpsilon && index < selected))) {
      selected = index;
      best = error;
    }
  }
  return selected;
}

}  // namespace

TubeSurfaceValidator::TubeSurfaceValidator(
    const TubeSurfaceValidatorConfig& config)
    : config_(config) {}

bool TubeSurfaceValidator::configurationValid() const {
  return config_.max_subdivision_depth >= 0 &&
      config_.max_subdivision_depth <= 24 &&
      config_.max_query_samples >= 9U;
}

bool TubeSurfaceValidator::validate(
    TubeProfile& profile,
    const double current_w,
    const PathStateQuery& path_state_query,
    const ClearanceQuery& clearance_query,
    const double snapshot_resolution,
    const double required_clearance,
    const double regularity_margin,
    TubeSurfaceValidationResult& result) const {
  return validate(profile, current_w, path_state_query, PathCellBoundQuery(),
                  clearance_query, snapshot_resolution, required_clearance,
                  regularity_margin, result);
}

bool TubeSurfaceValidator::validate(
    TubeProfile& profile,
    const double current_w,
    const PathStateQuery& path_state_query,
    const PathCellBoundQuery& path_cell_bound_query,
    const ClearanceQuery& clearance_query,
    const double snapshot_resolution,
    const double required_clearance,
    const double regularity_margin,
    TubeSurfaceValidationResult& result) const {
  result = TubeSurfaceValidationResult();
  result.min_clearance_margin = std::numeric_limits<double>::infinity();
  result.min_cover_radius = std::numeric_limits<double>::infinity();
  profile.validator_knot_evidence.clear();
  profile.zero_centerline_continuously_certified = false;
  profile.obstacle_certified = false;
  if (!configurationValid() || !profile.complete || profile.samples.size() < 2U ||
      !path_state_query || !clearance_query || !IsFinite(current_w) ||
      !IsFinite(snapshot_resolution) || snapshot_resolution <= 0.0 ||
      !IsFinite(required_clearance) || required_clearance < 0.0 ||
      !IsFinite(regularity_margin) || regularity_margin <= 0.0) {
    result.min_cover_radius = 0.0;
    result.first_failure_reason = TubeStopReason::INVALID_PATH;
    result.first_failure_w = IsFinite(current_w) ? current_w : 0.0;
    return false;
  }

  phase_offset_core::GeometryParams geometry_params;
  geometry_params.regularity_margin = regularity_margin;
  ValidationContext context;
  context.profile = &profile;
  context.path_state_query = &path_state_query;
  context.path_cell_bound_query = &path_cell_bound_query;
  context.clearance_query = &clearance_query;
  context.snapshot_resolution = snapshot_resolution;
  context.current_w = current_w;
  context.required_clearance = required_clearance;
  context.regularity_margin = regularity_margin;
  context.cover_epsilon = std::min(1e-6, 0.01 * snapshot_resolution);
  context.config = &config_;
  context.geometry_evaluator = phase_offset_core::GeometryEvaluator(geometry_params);
  context.result = &result;

  const std::size_t anchor = FindAnchor(profile, current_w);
  if (anchor >= profile.samples.size()) {
    RecordFailure(context, TubeStopReason::INVALID_PATH, current_w);
    if (!IsFinite(result.min_cover_radius)) result.min_cover_radius = 0.0;
    return false;
  }
  result.current_anchor_valid = ValidateCell(context,
      {current_w, current_w, 0.0, 1.0, 0});
  if (!result.current_anchor_valid) {
    if (!IsFinite(result.min_cover_radius)) result.min_cover_radius = 0.0;
    return false;
  }

  std::vector<bool> certified(profile.samples.size() - 1U, false);
  for (std::size_t index = 0U; index < certified.size(); ++index) {
    certified[index] = ValidateCell(context,
        {profile.samples[index].w, profile.samples[index + 1U].w, 0.0, 1.0, 0});
  }
  std::size_t first = anchor;
  std::size_t last = anchor;
  while (first > 0U && certified[first - 1U]) --first;
  while (last + 1U < profile.samples.size() && certified[last]) ++last;
  if (first == last) {
    RecordFailure(context, TubeStopReason::INSUFFICIENT_CLEARANCE, current_w);
    if (!IsFinite(result.min_cover_radius)) result.min_cover_radius = 0.0;
    return false;
  }

  const bool truncated_before = first > 0U;
  const bool truncated_after = last + 1U < profile.samples.size();
  const std::vector<TubeRawSample, Eigen::aligned_allocator<TubeRawSample>>
      original_samples = profile.samples;
  profile.samples.assign(original_samples.begin() + first,
                         original_samples.begin() + last + 1U);
  profile.preview_start_w = profile.samples.front().w;
  profile.preview_end_w = profile.samples.back().w;
  profile.certified_segment_start_w = profile.preview_start_w;
  profile.certified_segment_end_w = profile.preview_end_w;
  profile.certified_segment_truncated_before =
      profile.certified_segment_truncated_before || truncated_before;
  profile.certified_segment_truncated_after =
      profile.certified_segment_truncated_after || truncated_after;
  if ((truncated_before || truncated_after) &&
      profile.first_truncated_reason == TubeStopReason::NONE) {
    const std::size_t discarded = truncated_before ? first - 1U : last + 1U;
    profile.first_truncated_w = original_samples[discarded].w;
    profile.first_truncated_reason = result.first_failure_reason ==
        TubeStopReason::NONE ? TubeStopReason::INSUFFICIENT_CLEARANCE
                              : result.first_failure_reason;
  }
  profile.filtered_complete = true;
  profile.complete = true;
  profile.obstacle_certified = true;
  if (IsFinite(result.min_clearance_margin)) {
    profile.diagnostics.min_safety_margin = std::min(
        profile.diagnostics.min_safety_margin, result.min_clearance_margin);
  }
  result.complete = true;
  result.truncated_before = truncated_before;
  result.truncated_after = truncated_after;
  result.certified_start_w = profile.preview_start_w;
  result.certified_end_w = profile.preview_end_w;
  if (!IsFinite(result.min_clearance_margin)) result.min_clearance_margin = 0.0;
  if (!IsFinite(result.min_cover_radius)) result.min_cover_radius = 0.0;
  profile.zero_centerline_continuously_certified =
      ZeroCentrelineCoveredAtAllProfileKnots(profile);
  result.zero_centerline_continuously_certified =
      profile.zero_centerline_continuously_certified;
  result.knot_evidence = profile.validator_knot_evidence;
  return true;
}

}  // namespace phase_offset_navigation
