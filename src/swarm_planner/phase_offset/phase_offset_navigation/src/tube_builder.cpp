#include "phase_offset_navigation/tube_builder.h"

#include <phase_offset_core/geometry.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace phase_offset_navigation {
namespace {

constexpr double kEpsilon = 1e-9;
constexpr double kCurrentPhaseMatchTolerance = 1e-9;

bool IsFinite(double value) {
  return std::isfinite(value);
}

bool IsFinite(const Eigen::Vector3d& value) {
  return value.allFinite();
}

double BoundedConstructionDelta(const TubeBuilderConfig& config,
                                const double retained_delta) {
  if (!IsFinite(retained_delta)) return 0.0;
  const double rho = config.cross_section.nominal_half_width;
  if (!IsFinite(rho) || rho <= 0.0) return 0.0;
  return retained_delta >= -rho && retained_delta <= rho
      ? retained_delta : 0.0;
}

void FinalizeConstructionDiagnostics(TubeBuildDiagnostics& diagnostics) {
  diagnostics.certified_cell_bound_query_count =
      diagnostics.builder_certified_cell_bound_query_count +
      diagnostics.validator_certified_cell_bound_query_count;
  diagnostics.total_tube_construction_clearance_query_count =
      diagnostics.cross_section_directional_query_count +
      diagnostics.adaptive_refinement_centerline_query_count +
      diagnostics.adaptive_sample_base_clearance_query_count +
      diagnostics.validator_surface_query_count;
  diagnostics.total_tube_construction_query_count =
      diagnostics.total_tube_construction_clearance_query_count +
      diagnostics.certified_cell_bound_query_count;
}

void CountStop(const TubeStopReason reason, TubeBuildDiagnostics& diagnostics) {
  switch (reason) {
    case TubeStopReason::UNAVAILABLE:
      ++diagnostics.unavailable_count;
      break;
    case TubeStopReason::OUT_OF_MAP:
      ++diagnostics.out_of_map_count;
      break;
    case TubeStopReason::UNKNOWN:
      ++diagnostics.unknown_count;
      break;
    case TubeStopReason::OCCUPIED:
      ++diagnostics.occupied_count;
      break;
    case TubeStopReason::INSUFFICIENT_CLEARANCE:
      ++diagnostics.insufficient_clearance_count;
      break;
    default:
      break;
  }
}

void RecordFirstInvalid(const double w, const int side,
                        const TubeStopReason reason,
                        TubeBuildDiagnostics& diagnostics) {
  if (reason == TubeStopReason::NONE ||
      diagnostics.first_stop_reason != static_cast<int>(TubeStopReason::NONE)) return;
  diagnostics.first_invalid_w = w;
  diagnostics.first_invalid_side = side;
  diagnostics.first_stop_reason = static_cast<int>(reason);
}

TubeStopReason StopReason(const DistanceQueryResult& result,
                          const double required_clearance) {
  switch (result.status) {
    case DistanceStatus::UNAVAILABLE:
      return TubeStopReason::UNAVAILABLE;
    case DistanceStatus::OUT_OF_MAP:
      return TubeStopReason::OUT_OF_MAP;
    case DistanceStatus::UNKNOWN:
      return TubeStopReason::UNKNOWN;
    case DistanceStatus::OCCUPIED:
      return TubeStopReason::OCCUPIED;
    case DistanceStatus::KNOWN_FREE:
      if (!IsFinite(result.signed_distance)) return TubeStopReason::UNKNOWN;
      return result.signed_distance < required_clearance
          ? TubeStopReason::INSUFFICIENT_CLEARANCE : TubeStopReason::NONE;
  }
  return TubeStopReason::UNKNOWN;
}

struct RayResult {
  double bound = 0.0;
  bool certified = false;
  TubeStopReason stop = TubeStopReason::UNAVAILABLE;
};

RayResult TraceRay(const Eigen::Vector3d& origin,
                   const Eigen::Vector3d& direction,
                   const DistanceQuery& distance_query,
                   const double ray_step,
                   const double max_offset,
                   const double required_clearance) {
  RayResult ray;
  ray.certified = true;
  ray.stop = TubeStopReason::NONE;
  if (!distance_query) {
    ray.certified = false;
    ray.stop = TubeStopReason::UNAVAILABLE;
    return ray;
  }
  double offset = 0.0;
  while (offset < max_offset - kEpsilon) {
    offset = std::min(max_offset, offset + ray_step);
    const DistanceQueryResult result = distance_query(origin + direction * offset);
    const TubeStopReason reason = StopReason(result, required_clearance);
    if (reason == TubeStopReason::NONE) {
      ray.bound = offset;
      continue;
    }
    ray.stop = reason;
    if (reason == TubeStopReason::UNAVAILABLE ||
        reason == TubeStopReason::OUT_OF_MAP ||
        reason == TubeStopReason::UNKNOWN) {
      ray.certified = false;
    }
    return ray;
  }
  return ray;
}

bool IntersectRegularity3D(const Eigen::Vector3d& p_w,
                           const Eigen::Vector3d& N_w,
                           const double minimum_reference_speed,
                           const double selected_delta,
                           double& lower,
                           double& upper) {
  if (!IsFinite(p_w) || !IsFinite(N_w) ||
      !IsFinite(minimum_reference_speed) || minimum_reference_speed <= 0.0) {
    return false;
  }
  const double a = N_w.squaredNorm();
  const double b = 2.0 * p_w.dot(N_w);
  const double c = p_w.squaredNorm() - minimum_reference_speed *
      minimum_reference_speed;
  if (!IsFinite(a) || !IsFinite(b) || !IsFinite(c)) return false;
  if (a <= 1e-14) {
    if (c >= 0.0) return true;
    if (std::abs(b) <= 1e-14) return false;
    const double root = -c / b;
    if (!IsFinite(root)) return false;
    if (b * selected_delta + c < 0.0) return false;
    if (b > 0.0) lower = std::max(lower, root);
    else upper = std::min(upper, root);
    return lower <= upper + kEpsilon;
  }
  const double discriminant = b * b - 4.0 * a * c;
  if (!IsFinite(discriminant)) return false;
  if (discriminant <= 0.0) return c >= 0.0;
  const double root = std::sqrt(discriminant);
  double r0 = (-b - root) / (2.0 * a);
  double r1 = (-b + root) / (2.0 * a);
  if (!IsFinite(r0) || !IsFinite(r1)) return false;
  if (r0 > r1) std::swap(r0, r1);
  if (selected_delta > r0 + kEpsilon && selected_delta < r1 - kEpsilon) {
    return false;
  }
  if (selected_delta <= r0) upper = std::min(upper, r0);
  else lower = std::max(lower, r1);
  return IsFinite(lower) && IsFinite(upper) && lower <= upper + kEpsilon;
}

TubeStopReason StopReason(const TubeRayTermination termination) {
  switch (termination) {
    case TubeRayTermination::SEARCH_EXTENT:
      return TubeStopReason::NONE;
    case TubeRayTermination::OCCUPIED:
      return TubeStopReason::OCCUPIED;
    case TubeRayTermination::UNKNOWN:
      return TubeStopReason::UNKNOWN;
    case TubeRayTermination::OUT_OF_MAP:
      return TubeStopReason::OUT_OF_MAP;
    case TubeRayTermination::UNAVAILABLE:
      return TubeStopReason::UNAVAILABLE;
  }
  return TubeStopReason::UNAVAILABLE;
}

TubeStopReason CrossSectionFailureStop(const TubeCrossSectionResult& result) {
  switch (result.reason) {
    case TubeCrossSectionReason::CENTER_UNAVAILABLE:
      return TubeStopReason::UNAVAILABLE;
    case TubeCrossSectionReason::CENTER_OUT_OF_MAP:
      return TubeStopReason::OUT_OF_MAP;
    case TubeCrossSectionReason::CENTER_UNKNOWN:
      return TubeStopReason::UNKNOWN;
    case TubeCrossSectionReason::CENTER_OCCUPIED:
      return TubeStopReason::OCCUPIED;
    case TubeCrossSectionReason::EMPTY_AFTER_OBSTACLE_BOUNDS:
      return TubeStopReason::INSUFFICIENT_CLEARANCE;
    case TubeCrossSectionReason::CURVATURE_NUMERICAL_FAILURE:
    case TubeCrossSectionReason::EMPTY_AFTER_CURVATURE_INTERSECTION:
    case TubeCrossSectionReason::REGULARITY_ZERO_UNSAFE:
      return TubeStopReason::REGULARITY;
    case TubeCrossSectionReason::INVALID_GEOMETRY:
    case TubeCrossSectionReason::INVALID_CONFIGURATION:
      return TubeStopReason::INVALID_PATH;
    case TubeCrossSectionReason::NONE:
      return TubeStopReason::NONE;
  }
  return TubeStopReason::INVALID_PATH;
}

bool CrossSectionFactsFinite(const TubeCrossSectionResult& result) {
  return IsFinite(result.c_plus_raw) && IsFinite(result.c_minus_raw) &&
      IsFinite(result.full_effective_radius) &&
      IsFinite(result.preincluded_map_uncertainty) &&
      IsFinite(result.residual_effective_radius) &&
      IsFinite(result.effective_radius) && IsFinite(result.lower_obstacle) &&
      IsFinite(result.upper_obstacle) && IsFinite(result.lower_curvature) &&
      IsFinite(result.upper_curvature) && IsFinite(result.lower_final) &&
      IsFinite(result.upper_final) && IsFinite(result.width);
}

bool CrossSectionRaysCertified(const TubeCrossSectionResult& result) {
  switch (result.reason) {
    case TubeCrossSectionReason::INVALID_CONFIGURATION:
    case TubeCrossSectionReason::INVALID_GEOMETRY:
    case TubeCrossSectionReason::CENTER_UNAVAILABLE:
    case TubeCrossSectionReason::CENTER_OUT_OF_MAP:
    case TubeCrossSectionReason::CENTER_UNKNOWN:
    case TubeCrossSectionReason::CENTER_OCCUPIED:
    case TubeCrossSectionReason::REGULARITY_ZERO_UNSAFE:
      return false;
    case TubeCrossSectionReason::NONE:
    case TubeCrossSectionReason::EMPTY_AFTER_OBSTACLE_BOUNDS:
    case TubeCrossSectionReason::CURVATURE_NUMERICAL_FAILURE:
    case TubeCrossSectionReason::EMPTY_AFTER_CURVATURE_INTERSECTION:
      return true;
  }
  return false;
}

void CopyCrossSectionFacts(const TubeCrossSectionResult& result,
                           TubeRawSample& sample) {
  sample.c_plus_raw = result.c_plus_raw;
  sample.c_minus_raw = result.c_minus_raw;
  sample.full_effective_radius = result.full_effective_radius;
  sample.preincluded_map_uncertainty = result.preincluded_map_uncertainty;
  sample.residual_effective_radius = result.residual_effective_radius;
  sample.effective_radius = result.effective_radius;
  sample.obstacle_lower = result.lower_obstacle;
  sample.obstacle_upper = result.upper_obstacle;
  sample.pre_inset_lower = result.lower_final;
  sample.pre_inset_upper = result.upper_final;
  sample.continuous_inset = 0.0;
  sample.curvature_lower = result.lower_curvature;
  sample.curvature_upper = result.upper_curvature;
  sample.environment_lower = result.lower_final;
  sample.environment_upper = result.upper_final;
  sample.environment_width = result.width;
  sample.environment_interval_nonempty = result.valid;
  sample.environment_contains_zero = result.contains_zero;
  sample.pre_inset_contains_zero = result.contains_zero;
  sample.post_inset_contains_zero = result.contains_zero;
  sample.filter_input_contains_zero = result.contains_zero;
  sample.filtered_contains_zero = result.contains_zero;
  sample.cross_section_reason = result.reason;
  sample.regularity_speed_at_delta = result.regularity_speed_at_zero;
  sample.regularity_speed_min = result.regularity_speed_min;
  sample.positive_ray_termination = result.positive_termination;
  sample.negative_ray_termination = result.negative_termination;
}

bool ExactPathState(const PathStateQuery& query,
                    const double w,
                    phase_offset_core::PathDifferentialState& state) {
  if (!query || !IsFinite(w) || !query(w, state) || !state.valid ||
      !IsFinite(state.p) || !IsFinite(state.p_w) || !IsFinite(state.p_ww) ||
      !IsFinite(state.w) || std::abs(state.w - w) > kCurrentPhaseMatchTolerance) {
    return false;
  }
  return true;
}

bool EvaluateNormal(const phase_offset_core::GeometryEvaluator& evaluator,
                    const phase_offset_core::PathDifferentialState& path,
                    Eigen::Vector3d& normal) {
  // Adaptive sampling needs only the immutable spatial normal.  When a
  // frame-bound path already supplies it, do not run a delta=0 regularity
  // gate here: the selected current component is checked by the Builder's
  // full 3D quadratic contract below.
  if (path.frame_valid && IsFinite(path.N) && path.N.norm() > 1e-12) {
    normal = path.N.normalized();
    return true;
  }
  phase_offset_core::PhaseOffsetGeometryState geometry;
  if (!evaluator.evaluate(path, path.p, 0.0, geometry) || !geometry.valid ||
      !IsFinite(geometry.N)) {
    return false;
  }
  normal = geometry.N;
  return true;
}

struct AdaptiveSamplingContext {
  const TubeBuilderConfig* config = nullptr;
  const PathStateQuery* path_state_query = nullptr;
  const ClearanceQuery* clearance_query = nullptr;
  double snapshot_resolution = 0.0;
  double required_clearance = 0.0;
  phase_offset_core::GeometryEvaluator geometry_evaluator;
  std::vector<phase_offset_core::PathDifferentialState,
              Eigen::aligned_allocator<phase_offset_core::PathDifferentialState>>*
      samples = nullptr;
  TubeBuildDiagnostics* diagnostics = nullptr;
  bool limit_exceeded = false;
};

bool AdaptiveNeedsSubdivision(AdaptiveSamplingContext& context,
                              const phase_offset_core::PathDifferentialState& first,
                              const phase_offset_core::PathDifferentialState& middle,
                              const phase_offset_core::PathDifferentialState& second,
                              bool& geometry_requires_subdivision,
                              bool& clearance_changed) {
  geometry_requires_subdivision = false;
  clearance_changed = false;
  Eigen::Vector3d first_normal;
  Eigen::Vector3d middle_normal;
  Eigen::Vector3d second_normal;
  if (!EvaluateNormal(context.geometry_evaluator, first, first_normal) ||
      !EvaluateNormal(context.geometry_evaluator, middle, middle_normal) ||
      !EvaluateNormal(context.geometry_evaluator, second, second_normal)) {
    geometry_requires_subdivision = true;
    return true;
  }
  const double midpoint_deviation = (middle.p - 0.5 * (first.p + second.p)).norm();
  const double normal_deviation = std::max(
      (middle_normal - first_normal).norm(),
      (second_normal - middle_normal).norm());
  const double admissible_range = std::max({
      std::abs(first.admissible_delta_lower),
      std::abs(first.admissible_delta_upper),
      std::abs(middle.admissible_delta_lower),
      std::abs(middle.admissible_delta_upper),
      std::abs(second.admissible_delta_lower),
      std::abs(second.admissible_delta_upper), 0.0});
  const double boundary_scale = std::min(
      context.config->cross_section.nominal_half_width,
      admissible_range > 0.0 ? admissible_range
                             : context.config->cross_section.nominal_half_width);
  const double boundary_span = std::max({
      (first.p + boundary_scale * first_normal -
       (second.p + boundary_scale * second_normal)).norm(),
      (first.p - boundary_scale * first_normal -
       (second.p - boundary_scale * second_normal)).norm(),
      (first.p - second.p).norm()});
  if (!IsFinite(midpoint_deviation) || !IsFinite(normal_deviation) ||
      !IsFinite(boundary_span)) {
    geometry_requires_subdivision = true;
    return true;
  }
  if (context.diagnostics != nullptr) {
    // Reaching this clearance-comparison block always contributes exactly
    // three centreline callbacks (delta == 0) to the P1 taxonomy.
    context.diagnostics->adaptive_refinement_centerline_query_count += 3U;
  }
  const ClearanceQueryResult first_clearance = (*context.clearance_query)(
      first.p, context.required_clearance);
  const ClearanceQueryResult middle_clearance = (*context.clearance_query)(
      middle.p, context.required_clearance);
  const ClearanceQueryResult second_clearance = (*context.clearance_query)(
      second.p, context.required_clearance);
  const bool first_safe = first_clearance.status == DistanceStatus::KNOWN_FREE &&
      first_clearance.clearance_certified && IsFinite(first_clearance.clearance) &&
      first_clearance.clearance >= context.required_clearance;
  const bool middle_safe = middle_clearance.status == DistanceStatus::KNOWN_FREE &&
      middle_clearance.clearance_certified && IsFinite(middle_clearance.clearance) &&
      middle_clearance.clearance >= context.required_clearance;
  const bool second_safe = second_clearance.status == DistanceStatus::KNOWN_FREE &&
      second_clearance.clearance_certified && IsFinite(second_clearance.clearance) &&
      second_clearance.clearance >= context.required_clearance;
  clearance_changed = first_clearance.status != middle_clearance.status ||
      middle_clearance.status != second_clearance.status ||
      first_safe != middle_safe || middle_safe != second_safe;
  const double geometry_tolerance = 0.25 * context.snapshot_resolution;
  geometry_requires_subdivision =
      second.w - first.w > context.config->sample_step_w + kEpsilon ||
      midpoint_deviation > geometry_tolerance ||
      normal_deviation * boundary_scale >
          geometry_tolerance ||
      boundary_span > context.snapshot_resolution;
  return geometry_requires_subdivision || clearance_changed;
}

bool AppendAdaptiveSample(AdaptiveSamplingContext& context,
                          const phase_offset_core::PathDifferentialState& state) {
  constexpr std::size_t kMaxAdaptiveSamples = 4096U;
  if (context.samples->size() >= kMaxAdaptiveSamples) {
    context.limit_exceeded = true;
    return false;
  }
  context.samples->push_back(state);
  return true;
}

bool SubdivideAdaptiveSegment(
    AdaptiveSamplingContext& context,
    const phase_offset_core::PathDifferentialState& first,
    const phase_offset_core::PathDifferentialState& second,
    const int depth) {
  constexpr int kMaxAdaptiveDepth = 12;
  if (!IsFinite(first.w) || !IsFinite(second.w) ||
      second.w <= first.w + kEpsilon) {
    return false;
  }
  const double middle_w = first.w + 0.5 * (second.w - first.w);
  phase_offset_core::PathDifferentialState middle;
  if (!ExactPathState(*context.path_state_query, middle_w, middle)) return false;
  bool geometry_requires_subdivision = false;
  bool clearance_changed = false;
  if (!AdaptiveNeedsSubdivision(context, first, middle, second,
                                geometry_requires_subdivision,
                                clearance_changed)) {
    return AppendAdaptiveSample(context, second);
  }
  // A clearance/status transition itself is a reason to retain the exact
  // midpoint sample, not a reason to recurse forever toward a legitimate
  // nonzero Candidate component.  Geometric cover refinement remains bounded
  // and fail-closed below; the continuous surface validator verifies all
  // intervening cells afterwards.
  if (!geometry_requires_subdivision && clearance_changed) {
    return AppendAdaptiveSample(context, middle) &&
        AppendAdaptiveSample(context, second);
  }
  if (depth >= kMaxAdaptiveDepth) {
    context.limit_exceeded = true;
    return false;
  }
  return SubdivideAdaptiveSegment(context, first, middle, depth + 1) &&
      SubdivideAdaptiveSegment(context, middle, second, depth + 1);
}

bool BuildAdaptivePathSamples(
    const TubeBuilderConfig& config,
    const std::vector<phase_offset_core::PathDifferentialState,
                      Eigen::aligned_allocator<phase_offset_core::PathDifferentialState>>&
        preview,
    const PathStateQuery& path_state_query,
    const ClearanceQuery& clearance_query,
    const double snapshot_resolution,
    std::vector<phase_offset_core::PathDifferentialState,
                Eigen::aligned_allocator<phase_offset_core::PathDifferentialState>>&
        output,
    bool& limit_exceeded,
    TubeBuildDiagnostics* diagnostics) {
  output.clear();
  limit_exceeded = false;
  if (preview.size() < 2U || !path_state_query || !clearance_query ||
      !IsFinite(snapshot_resolution) || snapshot_resolution <= 0.0) {
    return false;
  }
  phase_offset_core::GeometryParams geometry_params;
  geometry_params.regularity_margin = config.cross_section.regularity_margin;
  geometry_params.minimum_reference_speed =
      config.cross_section.minimum_reference_speed;
  AdaptiveSamplingContext context;
  context.config = &config;
  context.path_state_query = &path_state_query;
  context.clearance_query = &clearance_query;
  context.snapshot_resolution = snapshot_resolution;
  context.required_clearance = config.cross_section.planner_safe_distance;
  context.geometry_evaluator = phase_offset_core::GeometryEvaluator(geometry_params);
  context.samples = &output;
  context.diagnostics = diagnostics;
  phase_offset_core::PathDifferentialState first;
  if (!ExactPathState(path_state_query, preview.front().w, first) ||
      !AppendAdaptiveSample(context, first)) {
    limit_exceeded = context.limit_exceeded;
    return false;
  }
  for (std::size_t index = 1U; index < preview.size(); ++index) {
    phase_offset_core::PathDifferentialState second;
    if (!ExactPathState(path_state_query, preview[index].w, second) ||
        !SubdivideAdaptiveSegment(context, output.back(), second, 0)) {
      limit_exceeded = context.limit_exceeded;
      return false;
    }
  }
  limit_exceeded = context.limit_exceeded;
  return !output.empty();
}

bool BuildCertifiedCellInsets(
    const std::vector<phase_offset_core::PathDifferentialState,
                      Eigen::aligned_allocator<phase_offset_core::PathDifferentialState>>&
        samples,
    const PathCellBoundQuery& path_cell_bound_query,
    const double nominal_half_width,
    std::vector<double>& sample_insets,
    std::size_t& certified_cell_count,
    std::vector<phase_offset_core::PathCellGeometryCertificate>* certificates,
    TubeBuildDiagnostics* diagnostics) {
  sample_insets.assign(samples.size(), 0.0);
  certified_cell_count = 0U;
  if (certificates != nullptr) certificates->clear();
  if (!path_cell_bound_query || samples.size() < 2U ||
      !IsFinite(nominal_half_width) || nominal_half_width <= 0.0) {
    return false;
  }
  std::vector<double> cell_insets(samples.size() - 1U, 0.0);
  const std::uint64_t expected_path_revision = samples.front().path_revision;
  const std::uint64_t expected_frame_revision = samples.front().frame_revision;
  for (std::size_t index = 0U; index + 1U < samples.size(); ++index) {
    const double w0 = samples[index].w;
    const double w1 = samples[index + 1U].w;
    phase_offset_core::PathCellGeometryCertificate certificate;
    if (diagnostics != nullptr) {
      ++diagnostics->builder_certified_cell_bound_query_count;
    }
    const bool query_ok = IsFinite(w0) && IsFinite(w1) &&
        w1 > w0 + kEpsilon && path_cell_bound_query(w0, w1, certificate);
    const bool cert_complete =
        phase_offset_core::pathCellGeometryCertificateIsComplete(certificate);
    const bool w_match = std::abs(certificate.w0 - w0) <=
            kCurrentPhaseMatchTolerance &&
        std::abs(certificate.w1 - w1) <= kCurrentPhaseMatchTolerance;
    if (!query_ok || !cert_complete || !w_match) {
      sample_insets.clear();
      return false;
    }
    if (samples[index].path_revision != expected_path_revision ||
        samples[index + 1U].path_revision != expected_path_revision ||
        samples[index].frame_revision != expected_frame_revision ||
        samples[index + 1U].frame_revision != expected_frame_revision ||
        certificate.path_revision != expected_path_revision ||
        certificate.frame_revision != expected_frame_revision) {
      sample_insets.clear();
      return false;
    }
    // The certificate is consumed by the Validator as one complete cover
    // proof (p variation, N*delta variation, PWL delta variation, and the
    // geometric residual).  Builder therefore applies the proven nonnegative
    // local erosion zero here; charging chord/normal a second time would
    // unnecessarily exclude delta=0 at a narrow knot.
    const double residual = 0.0;
    if (!IsFinite(residual) || residual < 0.0) {
      sample_insets.clear();
      return false;
    }
    cell_insets[index] = residual;
    if (certificates != nullptr) certificates->push_back(certificate);
    ++certified_cell_count;
  }
  for (std::size_t index = 0U; index < samples.size(); ++index) {
    double inset = 0.0;
    if (index > 0U) inset = std::max(inset, cell_insets[index - 1U]);
    if (index + 1U < samples.size()) {
      inset = std::max(inset, cell_insets[index]);
    }
    if (!IsFinite(inset) || inset < 0.0) {
      sample_insets.clear();
      return false;
    }
    sample_insets[index] = inset;
  }
  return certified_cell_count == cell_insets.size();
}

}  // namespace

const char* tubeSourceName(const TubeSource source) {
  switch (source) {
    case TubeSource::NONE:
      return "none";
    case TubeSource::FIXED:
      return "fixed";
    case TubeSource::ESDF:
      return "esdf";
  }
  return "none";
}

const char* tubeStopReasonName(const TubeStopReason reason) {
  switch (reason) {
    case TubeStopReason::NONE:
      return "none";
    case TubeStopReason::INVALID_PATH:
      return "invalid_path";
    case TubeStopReason::UNAVAILABLE:
      return "unavailable";
    case TubeStopReason::OUT_OF_MAP:
      return "out_of_map";
    case TubeStopReason::UNKNOWN:
      return "unknown";
    case TubeStopReason::OCCUPIED:
      return "occupied";
    case TubeStopReason::INSUFFICIENT_CLEARANCE:
      return "insufficient_clearance";
    case TubeStopReason::REGULARITY:
      return "regularity";
  }
  return "none";
}

const char* tubeProofLevelName(const TubeProofLevel level) {
  switch (level) {
    case TubeProofLevel::NONE: return "none";
    case TubeProofLevel::SAMPLED_EVIDENCE: return "sampled_evidence";
    case TubeProofLevel::FRAME_CELL_PROOF: return "frame_cell_proof";
    case TubeProofLevel::CONTINUOUS_COVER_PROOF: return "continuous_cover_proof";
  }
  return "none";
}

const char* tubeComponentSelectionName(
    const TubeComponentSelection selection) {
  switch (selection) {
    case TubeComponentSelection::NONE: return "none";
    case TubeComponentSelection::ZERO_CONNECTED: return "zero_connected";
    case TubeComponentSelection::CURRENT_DELTA_CONNECTED:
      return "current_delta_connected";
  }
  return "none";
}

TubeBuilder::TubeBuilder(const TubeBuilderConfig& config) : config_(config) {}

bool TubeBuilder::configurationValid() const {
  return IsFinite(config_.sample_step_w) && config_.sample_step_w > 0.0 &&
      config_.sample_step_w <= 0.10 + kEpsilon &&
      IsFinite(config_.lookahead_w) && config_.lookahead_w >= 0.0 &&
      IsFinite(config_.back_w) && config_.back_w >= 0.0 &&
      IsFinite(config_.min_certified_forward_w) &&
      config_.min_certified_forward_w >= 0.0 &&
      IsFinite(config_.ray_step) && config_.ray_step > 0.0 &&
      config_.ray_step <= 0.05 + kEpsilon &&
      IsFinite(config_.regularity_margin) && config_.regularity_margin > 0.0 &&
      config_.regularity_margin < 1.0 && IsFinite(config_.invariant_gain) &&
      config_.invariant_gain >= 0.0 && IsFinite(config_.interior_margin) &&
      config_.interior_margin >= 0.0 &&
      TubeCrossSectionSolver(config_.cross_section).configurationValid();
}

bool TubeBuilder::configurationValidForSource(const TubeSource source) const {
  const bool common =
      IsFinite(config_.sample_step_w) && config_.sample_step_w > 0.0 &&
      config_.sample_step_w <= 0.10 + kEpsilon &&
      IsFinite(config_.lookahead_w) && config_.lookahead_w >= 0.0 &&
      IsFinite(config_.back_w) && config_.back_w >= 0.0 &&
      IsFinite(config_.min_certified_forward_w) &&
      config_.min_certified_forward_w >= 0.0 &&
      IsFinite(config_.ray_step) && config_.ray_step > 0.0 &&
      config_.ray_step <= 0.05 + kEpsilon &&
      IsFinite(config_.regularity_margin) && config_.regularity_margin > 0.0 &&
      config_.regularity_margin < 1.0 && IsFinite(config_.invariant_gain) &&
      config_.invariant_gain >= 0.0 && IsFinite(config_.interior_margin) &&
      config_.interior_margin >= 0.0 &&
      TubeCrossSectionSolver(config_.cross_section).configurationValid();
  if (!common) return false;
  if (source == TubeSource::ESDF) return true;
  if (source == TubeSource::FIXED) {
    return IsFinite(config_.fixed_delta_max) &&
        config_.fixed_delta_max >= 0.0;
  }
  return false;
}

bool TubeBuilder::build(
    const TubeSource source,
    const std::vector<phase_offset_core::PathDifferentialState,
                      Eigen::aligned_allocator<phase_offset_core::PathDifferentialState>>&
        preview,
    const DistanceQuery& distance_query,
    const std::uint64_t source_revision,
    const std::uint64_t tube_revision,
    TubeProfile& profile,
    const double current_delta) const {
  profile = TubeProfile();
  profile.source = source;
  profile.source_revision = source_revision;
  profile.tube_revision = tube_revision;
  profile.profile_revision = tube_revision;
  profile.current_delta = current_delta;
  profile.current_delta_valid = IsFinite(current_delta);
  profile.diagnostics.nominal_width_source =
      config_.cross_section.nominal_width_source;
  profile.diagnostics.nominal_width_legacy_conflict =
      config_.cross_section.nominal_width_legacy_conflict;
  profile.diagnostics.effective_nominal_half_width_m =
      config_.cross_section.nominal_half_width;
  if (!configurationValidForSource(source) || source == TubeSource::NONE ||
      preview.empty()) {
    profile.diagnostics.invalid_reason = "tube configuration, source, or preview is invalid";
    return false;
  }
  if (source == TubeSource::ESDF && !distance_query) {
    profile.diagnostics.invalid_reason = "distance source is unavailable";
    profile.diagnostics.unavailable_count = 1U;
    return false;
  }

  phase_offset_core::GeometryParams geometry_params;
  geometry_params.regularity_margin = config_.regularity_margin;
  geometry_params.minimum_reference_speed =
      config_.cross_section.minimum_reference_speed;
  phase_offset_core::GeometryEvaluator geometry_evaluator(geometry_params);
  profile.preview_start_w = preview.front().w;
  profile.preview_end_w = preview.back().w;
  profile.requested_preview_start_w = profile.preview_start_w;
  profile.requested_preview_end_w = profile.preview_end_w;
  profile.certified_segment_start_w = profile.preview_start_w;
  profile.certified_segment_end_w = profile.preview_end_w;
  if (!IsFinite(profile.preview_start_w) || !IsFinite(profile.preview_end_w) ||
      profile.preview_end_w < profile.preview_start_w) {
    profile.diagnostics.invalid_reason = "preview phase ordering is invalid";
    return false;
  }

  profile.diagnostics.min_width = std::numeric_limits<double>::infinity();
  profile.diagnostics.min_safety_margin = std::numeric_limits<double>::infinity();
  bool all_complete = true;
  for (const phase_offset_core::PathDifferentialState& path : preview) {
    TubeRawSample sample;
    sample.w = path.w;
    sample.path_revision = path.path_revision;
    sample.frame_revision = path.frame_revision;
    sample.proof_level = TubeProofLevel::SAMPLED_EVIDENCE;
    if (profile.path_revision == 0U) profile.path_revision = path.path_revision;
    if (profile.frame_revision == 0U) profile.frame_revision = path.frame_revision;
    if ((profile.path_revision != 0U &&
         path.path_revision != profile.path_revision) ||
        (profile.frame_revision != 0U &&
         path.frame_revision != profile.frame_revision)) {
      sample.complete = false;
      sample.positive_certified = false;
      sample.negative_certified = false;
      sample.positive_stop = TubeStopReason::INVALID_PATH;
      sample.negative_stop = TubeStopReason::INVALID_PATH;
      RecordFirstInvalid(path.w, 0, TubeStopReason::INVALID_PATH,
                         profile.diagnostics);
      ++profile.diagnostics.invalid_count;
      all_complete = false;
      profile.samples.push_back(sample);
      continue;
    }
    ++profile.diagnostics.sample_count;
    const double nominal_limit = source == TubeSource::FIXED
        ? config_.fixed_delta_max : config_.cross_section.nominal_half_width;
    // A finite retained delta is authoritative for connected-component
    // selection even when it lies outside this Builder's nominal interval.
    // The resulting evidence is clipped by [fixed_lower,fixed_upper], while
    // Filter/Epoch retain the unchanged delta and report containment outside.
    const double selected_delta = source == TubeSource::ESDF
        ? BoundedConstructionDelta(config_, current_delta)
        : (IsFinite(current_delta) ? current_delta : 0.0);
    phase_offset_core::PhaseOffsetGeometryState geometry;
    if (!geometry_evaluator.evaluate(path, path.p, selected_delta, geometry) ||
        !geometry.valid || !IsFinite(geometry.p) || !IsFinite(geometry.N) ||
        !IsFinite(geometry.r_w)) {
      sample.positive_stop = TubeStopReason::INVALID_PATH;
      sample.negative_stop = TubeStopReason::INVALID_PATH;
      RecordFirstInvalid(path.w, 0, TubeStopReason::INVALID_PATH,
                         profile.diagnostics);
      ++profile.diagnostics.invalid_count;
      all_complete = false;
      profile.samples.push_back(sample);
      continue;
    }
    sample.p = geometry.p;
    sample.N = geometry.N;
    sample.fixed_lower = -nominal_limit;
    sample.fixed_upper = nominal_limit;
    sample.regularity_lower = -nominal_limit;
    sample.regularity_upper = nominal_limit;
    sample.regularity_intersection = IntersectRegularity3D(
        geometry.p_w, geometry.N_w,
        config_.cross_section.minimum_reference_speed, selected_delta,
        sample.regularity_lower, sample.regularity_upper);
    if (!sample.regularity_intersection) {
      sample.positive_stop = TubeStopReason::REGULARITY;
      sample.negative_stop = TubeStopReason::REGULARITY;
      RecordFirstInvalid(path.w, 0, TubeStopReason::REGULARITY,
                         profile.diagnostics);
      ++profile.diagnostics.invalid_count;
      all_complete = false;
      profile.samples.push_back(sample);
      continue;
    }

    sample.esdf_lower = -nominal_limit;
    sample.esdf_upper = nominal_limit;
    sample.positive_certified = true;
    sample.negative_certified = true;
    sample.positive_stop = TubeStopReason::NONE;
    sample.negative_stop = TubeStopReason::NONE;
    sample.base_signed_distance = 0.0;
    if (source == TubeSource::ESDF) {
      const DistanceQueryResult base = distance_query(sample.p);
      sample.base_signed_distance = IsFinite(base.signed_distance)
          ? base.signed_distance : 0.0;
      const TubeStopReason base_reason = StopReason(
          base, config_.cross_section.planner_safe_distance);
      if (base_reason != TubeStopReason::NONE) {
        sample.positive_certified = false;
        sample.negative_certified = false;
        sample.positive_stop = base_reason;
        sample.negative_stop = base_reason;
        CountStop(base_reason, profile.diagnostics);
        RecordFirstInvalid(path.w, 0, base_reason, profile.diagnostics);
        ++profile.diagnostics.invalid_count;
        all_complete = false;
        profile.samples.push_back(sample);
        continue;
      }
      const RayResult positive = TraceRay(
          sample.p, sample.N, distance_query, config_.ray_step,
          nominal_limit, config_.cross_section.planner_safe_distance);
      const RayResult negative = TraceRay(
          sample.p, -sample.N, distance_query, config_.ray_step,
          nominal_limit, config_.cross_section.planner_safe_distance);
      sample.esdf_upper = positive.bound;
      sample.esdf_lower = -negative.bound;
      sample.positive_certified = positive.certified;
      sample.negative_certified = negative.certified;
      sample.positive_stop = positive.stop;
      sample.negative_stop = negative.stop;
      CountStop(positive.stop, profile.diagnostics);
      CountStop(negative.stop, profile.diagnostics);
    }

    sample.raw_lower = std::max(sample.fixed_lower,
        std::max(sample.regularity_lower, sample.esdf_lower));
    sample.raw_upper = std::min(sample.fixed_upper,
        std::min(sample.regularity_upper, sample.esdf_upper));
    sample.filtered_lower = sample.raw_lower;
    sample.filtered_upper = sample.raw_upper;
    sample.complete = IsFinite(sample.raw_lower) && IsFinite(sample.raw_upper) &&
        sample.raw_lower <= sample.raw_upper + kEpsilon &&
        sample.positive_certified && sample.negative_certified;
    if (!sample.complete) {
      ++profile.diagnostics.invalid_count;
      all_complete = false;
      if (!sample.positive_certified) {
        RecordFirstInvalid(path.w, 1, sample.positive_stop, profile.diagnostics);
      } else if (!sample.negative_certified) {
        RecordFirstInvalid(path.w, -1, sample.negative_stop, profile.diagnostics);
      } else {
        RecordFirstInvalid(path.w, 0, TubeStopReason::INSUFFICIENT_CLEARANCE,
                           profile.diagnostics);
      }
    }
    profile.diagnostics.min_width = std::min(profile.diagnostics.min_width,
        std::max(0.0, sample.raw_upper - sample.raw_lower));
    if (source == TubeSource::ESDF) {
      profile.diagnostics.min_safety_margin = std::min(
          profile.diagnostics.min_safety_margin,
          sample.base_signed_distance - config_.cross_section.planner_safe_distance);
    }
    profile.samples.push_back(sample);
  }
  if (!IsFinite(profile.diagnostics.min_width)) profile.diagnostics.min_width = 0.0;
  if (!IsFinite(profile.diagnostics.min_safety_margin)) {
    profile.diagnostics.min_safety_margin = 0.0;
  }
  profile.raw_build_samples = profile.samples;
  profile.raw_complete = all_complete && !profile.samples.empty();
  profile.filtered_complete = false;
  profile.complete = false;
  profile.obstacle_certified = source == TubeSource::ESDF && profile.raw_complete;
  profile.zero_component_contains_zero = profile.raw_complete;
  profile.selected_component = TubeComponentSelection::ZERO_CONNECTED;
  profile.proof_level = profile.cell_geometry_certified
      ? TubeProofLevel::FRAME_CELL_PROOF : TubeProofLevel::SAMPLED_EVIDENCE;
  if (!profile.raw_complete && profile.diagnostics.invalid_reason.empty()) {
    profile.diagnostics.invalid_reason = "preview contains an uncertified sample";
  }
  FinalizeConstructionDiagnostics(profile.diagnostics);
  return profile.raw_complete;
}

bool TubeBuilder::buildRawOccupancy(
    const TubeSource source,
    const std::vector<phase_offset_core::PathDifferentialState,
                      Eigen::aligned_allocator<phase_offset_core::PathDifferentialState>>&
        preview,
    const RawOccupancyQuery& occupancy_query,
    const double current_w,
    const std::uint64_t source_revision,
    const std::uint64_t tube_revision,
    TubeProfile& profile,
    const double current_delta) const {
  // FIXED deliberately retains its existing map-free behavior.  The raw query
  // is only a temporary ESDF compatibility boundary in G2a.
  if (source == TubeSource::FIXED) {
    return build(source, preview, DistanceQuery(), source_revision, tube_revision,
                 profile, current_delta);
  }

  profile = TubeProfile();
  profile.source = source;
  profile.source_revision = source_revision;
  profile.tube_revision = tube_revision;
  profile.current_delta = current_delta;
  profile.current_delta_valid = IsFinite(current_delta);
  profile.diagnostics.nominal_width_source =
      config_.cross_section.nominal_width_source;
  profile.diagnostics.nominal_width_legacy_conflict =
      config_.cross_section.nominal_width_legacy_conflict;
  profile.diagnostics.effective_nominal_half_width_m =
      config_.cross_section.nominal_half_width;
  const TubeCrossSectionSolver cross_section_solver(config_.cross_section);
  if (source != TubeSource::ESDF || !cross_section_solver.configurationValid() ||
      preview.empty()) {
    profile.diagnostics.invalid_reason =
        "raw tube source, cross-section configuration, or preview is invalid";
    return false;
  }
  if (!occupancy_query) {
    profile.diagnostics.invalid_reason = "raw occupancy source is unavailable";
    profile.diagnostics.unavailable_count = 1U;
    return false;
  }

  phase_offset_core::GeometryParams geometry_params;
  geometry_params.regularity_margin = config_.cross_section.regularity_margin;
  geometry_params.minimum_reference_speed =
      config_.cross_section.minimum_reference_speed;
  phase_offset_core::GeometryEvaluator geometry_evaluator(geometry_params);
  profile.preview_start_w = preview.front().w;
  profile.preview_end_w = preview.back().w;
  profile.requested_preview_start_w = profile.preview_start_w;
  profile.requested_preview_end_w = profile.preview_end_w;
  profile.certified_segment_start_w = profile.preview_start_w;
  profile.certified_segment_end_w = profile.preview_end_w;
  if (!IsFinite(profile.preview_start_w) || !IsFinite(profile.preview_end_w) ||
      profile.preview_end_w < profile.preview_start_w) {
    profile.diagnostics.invalid_reason = "preview phase ordering is invalid";
    return false;
  }

  profile.diagnostics.min_width = std::numeric_limits<double>::infinity();
  // Raw occupancy is categorical, so no raw Euclidean signed-distance value is
  // available for this legacy diagnostic.
  profile.diagnostics.min_safety_margin = 0.0;
  std::vector<TubeRawSample, Eigen::aligned_allocator<TubeRawSample>> evaluated;
  std::vector<bool> sample_complete;
  evaluated.reserve(preview.size());
  sample_complete.reserve(preview.size());
  std::size_t current_index = preview.size();
  double current_match_error = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < preview.size(); ++index) {
    const phase_offset_core::PathDifferentialState& path = preview[index];
    TubeRawSample sample;
    sample.w = path.w;
    ++profile.diagnostics.sample_count;
    if (IsFinite(current_w) && IsFinite(path.w)) {
      const double match_error = std::abs(path.w - current_w);
      if (match_error <= kCurrentPhaseMatchTolerance &&
          (match_error < current_match_error - kEpsilon ||
           (std::abs(match_error - current_match_error) <= kEpsilon &&
            index < current_index))) {
        current_index = index;
        current_match_error = match_error;
      }
    }
    phase_offset_core::PhaseOffsetGeometryState geometry;
    const double selected_delta = BoundedConstructionDelta(config_, current_delta);
    if (!geometry_evaluator.evaluate(path, path.p, selected_delta, geometry) ||
        !geometry.valid || !IsFinite(geometry.p) || !IsFinite(geometry.N) ||
        !IsFinite(geometry.r_w)) {
      sample.cross_section_reason = TubeCrossSectionReason::INVALID_GEOMETRY;
      sample.positive_stop = TubeStopReason::INVALID_PATH;
      sample.negative_stop = TubeStopReason::INVALID_PATH;
      RecordFirstInvalid(path.w, 0, TubeStopReason::INVALID_PATH,
                         profile.diagnostics);
      ++profile.diagnostics.invalid_count;
      evaluated.push_back(sample);
      sample_complete.push_back(false);
      continue;
    }

    sample.p = geometry.p;
    sample.N = geometry.N;
    // Preserve these legacy fields for consumers that still inspect them, but
    // do not use them to clamp the raw environmental interval below.
    sample.fixed_lower = -config_.fixed_delta_max;
    sample.fixed_upper = config_.fixed_delta_max;
    sample.esdf_lower = 0.0;
    sample.esdf_upper = 0.0;
    sample.base_signed_distance = 0.0;

    TubeCrossSectionInput cross_section_input;
    cross_section_input.p = geometry.p;
    cross_section_input.N = geometry.N;
    cross_section_input.curvature = geometry.curvature;
    cross_section_input.p_w = geometry.p_w;
    cross_section_input.N_w = geometry.N_w;
    cross_section_input.minimum_reference_speed =
        config_.cross_section.minimum_reference_speed;
    cross_section_input.current_delta = selected_delta;
    cross_section_input.current_delta_valid = IsFinite(selected_delta);
    cross_section_input.path_revision = path.path_revision;
    cross_section_input.frame_revision = path.frame_revision;
    cross_section_input.clearance_query =
        [occupancy_query](const Eigen::Vector3d& point, const double required) {
          ClearanceQueryResult result;
          result.status = occupancy_query(point);
          result.clearance = result.status == DistanceStatus::KNOWN_FREE
              ? required : 0.0;
          result.clearance_certified = result.status == DistanceStatus::KNOWN_FREE;
          return result;
        };
    cross_section_input.directional_query_count =
        &profile.diagnostics.cross_section_directional_query_count;
    cross_section_input.max_bounded_construction_abs_delta =
        &profile.diagnostics.max_bounded_construction_abs_delta;
    const TubeCrossSectionResult cross_section =
        cross_section_solver.solve(cross_section_input);
    CopyCrossSectionFacts(cross_section, sample);
    sample.regularity_lower = cross_section.lower_curvature;
    sample.regularity_upper = cross_section.upper_curvature;
    sample.regularity_intersection = cross_section.reason !=
            TubeCrossSectionReason::CURVATURE_NUMERICAL_FAILURE &&
        cross_section.reason !=
            TubeCrossSectionReason::EMPTY_AFTER_CURVATURE_INTERSECTION;
    sample.raw_lower = cross_section.lower_final;
    sample.raw_upper = cross_section.upper_final;
    sample.filtered_lower = sample.raw_lower;
    sample.filtered_upper = sample.raw_upper;
    sample.positive_stop = StopReason(cross_section.positive_termination);
    sample.negative_stop = StopReason(cross_section.negative_termination);
    // Once the free center has been accepted, both rays are independently
    // certified even if their final interval later proves empty.  This includes
    // a conservative truncation at the last known-free point before UNKNOWN or
    // a map boundary.
    sample.positive_certified = CrossSectionRaysCertified(cross_section);
    sample.negative_certified = CrossSectionRaysCertified(cross_section);
    sample.complete = cross_section.valid && CrossSectionFactsFinite(cross_section);
    if (!sample.complete) {
      const TubeStopReason failure = CrossSectionFailureStop(cross_section);
      if (failure != TubeStopReason::NONE) {
        CountStop(failure, profile.diagnostics);
        RecordFirstInvalid(path.w, 0, failure, profile.diagnostics);
      }
      ++profile.diagnostics.invalid_count;
    }
    profile.diagnostics.min_width = std::min(profile.diagnostics.min_width,
        std::max(0.0, sample.environment_width));
    evaluated.push_back(sample);
    sample_complete.push_back(sample.complete);
  }
  if (!IsFinite(profile.diagnostics.min_width)) profile.diagnostics.min_width = 0.0;
  profile.raw_build_samples = evaluated;
  if (current_index == preview.size()) {
    profile.samples = evaluated;
    profile.diagnostics.invalid_reason =
        "requested preview does not contain the current phase sample";
    RecordFirstInvalid(current_w, 0, TubeStopReason::INVALID_PATH,
                       profile.diagnostics);
    ++profile.diagnostics.invalid_count;
    profile.raw_complete = false;
  } else if (!sample_complete[current_index]) {
    // A map result at the actual current phase is authoritative: no partial
    // prefix can make this candidate executable or displayable.
    profile.samples = evaluated;
    profile.raw_complete = false;
  } else {
    std::size_t first = current_index;
    std::size_t last = current_index;
    while (first > 0U && sample_complete[first - 1U]) --first;
    while (last + 1U < sample_complete.size() && sample_complete[last + 1U]) ++last;
    profile.samples.assign(evaluated.begin() + first,
                           evaluated.begin() + last + 1U);
    profile.preview_start_w = profile.samples.front().w;
    profile.preview_end_w = profile.samples.back().w;
    profile.certified_segment_start_w = profile.preview_start_w;
    profile.certified_segment_end_w = profile.preview_end_w;
    profile.certified_segment_truncated_before = first > 0U;
    profile.certified_segment_truncated_after = last + 1U < evaluated.size();
    profile.raw_complete = true;
  }
  for (std::size_t index = 0U; index < sample_complete.size(); ++index) {
    if (!sample_complete[index]) {
      profile.first_truncated_w = evaluated[index].w;
      profile.first_truncated_reason = evaluated[index].positive_stop !=
          TubeStopReason::NONE ? evaluated[index].positive_stop :
          evaluated[index].negative_stop;
      if (profile.first_truncated_reason == TubeStopReason::NONE) {
        profile.first_truncated_reason = TubeStopReason::INVALID_PATH;
      }
      break;
    }
  }
  profile.filtered_complete = false;
  profile.complete = false;
  profile.obstacle_certified = profile.raw_complete;
  if (!profile.raw_complete && profile.diagnostics.invalid_reason.empty()) {
    profile.diagnostics.invalid_reason =
        "preview contains an incomplete raw occupancy cross-section";
  } else if (profile.raw_complete &&
             (profile.certified_segment_truncated_before ||
              profile.certified_segment_truncated_after)) {
    profile.diagnostics.invalid_reason = "requested preview was truncated to the current certified segment";
  }
  FinalizeConstructionDiagnostics(profile.diagnostics);
  return profile.raw_complete;
}

bool TubeBuilder::buildCloudClearance(
    const TubeSource source,
    const std::vector<phase_offset_core::PathDifferentialState,
                      Eigen::aligned_allocator<phase_offset_core::PathDifferentialState>>&
        preview,
    const ClearanceQuery& clearance_query,
    const PathStateQuery& path_state_query,
    const double snapshot_resolution,
    const double current_w,
    const std::uint64_t source_revision,
    const std::uint64_t tube_revision,
    TubeProfile& profile,
    const double current_delta) const {
  return buildCloudClearance(
      source, preview, clearance_query, path_state_query, PathCellBoundQuery(),
      snapshot_resolution, current_w, source_revision,
      tube_revision, profile, current_delta);
}

bool TubeBuilder::buildCloudClearance(
    const TubeSource source,
    const std::vector<phase_offset_core::PathDifferentialState,
                      Eigen::aligned_allocator<phase_offset_core::PathDifferentialState>>&
        preview,
    const ClearanceQuery& clearance_query,
    const PathStateQuery& path_state_query,
    const PathCellBoundQuery& path_cell_bound_query,
    const double snapshot_resolution,
    const double current_w,
    const std::uint64_t source_revision,
    const std::uint64_t tube_revision,
    TubeProfile& profile,
    const double current_delta) const {
  profile = TubeProfile();
  profile.source = source;
  profile.source_revision = source_revision;
  profile.tube_revision = tube_revision;
  profile.snapshot_resolution = snapshot_resolution;
  profile.profile_revision = tube_revision;
  profile.current_delta = current_delta;
  profile.current_delta_valid = IsFinite(current_delta);
  profile.obstacle_contract_id = "direct-clearance/planner-safe-distance";
  profile.diagnostics.nominal_width_source =
      config_.cross_section.nominal_width_source;
  profile.diagnostics.nominal_width_legacy_conflict =
      config_.cross_section.nominal_width_legacy_conflict;
  profile.diagnostics.effective_nominal_half_width_m =
      config_.cross_section.nominal_half_width;
  const TubeCrossSectionSolver cross_section_solver(config_.cross_section);
  if (source != TubeSource::ESDF || !configurationValidForSource(source) ||
      !cross_section_solver.configurationValid() || !clearance_query ||
      !path_state_query || preview.size() < 2U || !IsFinite(current_w) ||
      !IsFinite(snapshot_resolution) || snapshot_resolution <= 0.0) {
    profile.diagnostics.invalid_reason =
        "cloud clearance source, configuration, or path evaluator is invalid";
    return false;
  }

  std::vector<phase_offset_core::PathDifferentialState,
              Eigen::aligned_allocator<phase_offset_core::PathDifferentialState>>
      adaptive_preview;
  bool adaptive_limit_exceeded = false;
  if (!BuildAdaptivePathSamples(config_, preview, path_state_query,
                                clearance_query, snapshot_resolution,
                                adaptive_preview, adaptive_limit_exceeded,
                                &profile.diagnostics)) {
    profile.diagnostics.invalid_reason = adaptive_limit_exceeded
        ? "adaptive lifted-state sampling limit exceeded"
        : "adaptive lifted-state sampling failed";
    ++profile.diagnostics.invalid_count;
    return false;
  }
  profile.requested_preview_start_w = adaptive_preview.front().w;
  profile.requested_preview_end_w = adaptive_preview.back().w;
  profile.preview_start_w = profile.requested_preview_start_w;
  profile.preview_end_w = profile.requested_preview_end_w;
  profile.certified_segment_start_w = profile.preview_start_w;
  profile.certified_segment_end_w = profile.preview_end_w;
  if (!IsFinite(profile.preview_start_w) || !IsFinite(profile.preview_end_w) ||
      profile.preview_end_w < profile.preview_start_w) {
    profile.diagnostics.invalid_reason = "adaptive preview phase ordering is invalid";
    return false;
  }

  std::vector<double> certified_sample_insets;
  std::size_t certified_cell_count = 0U;
  std::vector<phase_offset_core::PathCellGeometryCertificate> certified_cells;
  const bool all_active_cells_certified = BuildCertifiedCellInsets(
      adaptive_preview, path_cell_bound_query,
      config_.cross_section.nominal_half_width, certified_sample_insets,
      certified_cell_count, &certified_cells, &profile.diagnostics);
  profile.cell_geometry_certified = all_active_cells_certified;
  profile.combined_regularity_proof_complete = false;
  profile.combined_regularity_speed_min = 0.0;
  profile.certified_cell_count = all_active_cells_certified
      ? certified_cell_count : 0U;

  phase_offset_core::GeometryParams geometry_params;
  geometry_params.regularity_margin = config_.cross_section.regularity_margin;
  geometry_params.minimum_reference_speed =
      config_.cross_section.minimum_reference_speed;
  phase_offset_core::GeometryEvaluator geometry_evaluator(geometry_params);
  profile.diagnostics.min_width = std::numeric_limits<double>::infinity();
  profile.diagnostics.min_safety_margin = std::numeric_limits<double>::infinity();
  std::vector<TubeRawSample, Eigen::aligned_allocator<TubeRawSample>> evaluated;
  std::vector<bool> sample_complete;
  evaluated.reserve(adaptive_preview.size());
  sample_complete.reserve(adaptive_preview.size());
  std::size_t current_index = adaptive_preview.size();
  double current_match_error = std::numeric_limits<double>::infinity();

  for (std::size_t index = 0U; index < adaptive_preview.size(); ++index) {
    const phase_offset_core::PathDifferentialState& path = adaptive_preview[index];
    TubeRawSample sample;
    sample.w = path.w;
    sample.path_revision = path.path_revision;
    sample.frame_revision = path.frame_revision;
    sample.proof_level = TubeProofLevel::SAMPLED_EVIDENCE;
    if (profile.path_revision == 0U) profile.path_revision = path.path_revision;
    if (profile.frame_revision == 0U) profile.frame_revision = path.frame_revision;
    if ((profile.path_revision != 0U &&
         path.path_revision != profile.path_revision) ||
        (profile.frame_revision != 0U &&
         path.frame_revision != profile.frame_revision)) {
      sample.complete = false;
      sample.positive_certified = false;
      sample.negative_certified = false;
      sample.positive_stop = TubeStopReason::INVALID_PATH;
      sample.negative_stop = TubeStopReason::INVALID_PATH;
      RecordFirstInvalid(path.w, 0, TubeStopReason::INVALID_PATH,
                         profile.diagnostics);
      ++profile.diagnostics.invalid_count;
      evaluated.push_back(sample);
      sample_complete.push_back(false);
      continue;
    }
    ++profile.diagnostics.sample_count;
    const double match_error = std::abs(path.w - current_w);
    if (match_error <= kCurrentPhaseMatchTolerance &&
        (match_error < current_match_error - kEpsilon ||
         (std::abs(match_error - current_match_error) <= kEpsilon &&
          index < current_index))) {
      current_index = index;
      current_match_error = match_error;
    }
    phase_offset_core::PhaseOffsetGeometryState geometry;
    const double selected_delta = BoundedConstructionDelta(config_, current_delta);
    // ESDF construction is anchored on the planner centreline.  The retained
    // in-range delta is passed to CrossSectionSolver only for component
    // selection; adaptive/base clearance callbacks stay at delta == 0.
    if (!geometry_evaluator.evaluate(path, path.p, 0.0, geometry) ||
        !geometry.valid || !IsFinite(geometry.p) || !IsFinite(geometry.N) ||
        !IsFinite(geometry.r_w)) {
      sample.cross_section_reason = TubeCrossSectionReason::INVALID_GEOMETRY;
      sample.positive_stop = TubeStopReason::INVALID_PATH;
      sample.negative_stop = TubeStopReason::INVALID_PATH;
      RecordFirstInvalid(path.w, 0, TubeStopReason::INVALID_PATH,
                         profile.diagnostics);
      ++profile.diagnostics.invalid_count;
      evaluated.push_back(sample);
      sample_complete.push_back(false);
      continue;
    }
    sample.p = geometry.p;
    sample.N = geometry.N;
    sample.fixed_lower = -config_.fixed_delta_max;
    sample.fixed_upper = config_.fixed_delta_max;
    sample.esdf_lower = 0.0;
    sample.esdf_upper = 0.0;
    ++profile.diagnostics.adaptive_sample_base_clearance_query_count;
    const ClearanceQueryResult base = clearance_query(
        sample.p, config_.cross_section.planner_safe_distance);
    sample.base_signed_distance = IsFinite(base.clearance) ? base.clearance : 0.0;

    TubeCrossSectionInput cross_section_input;
    cross_section_input.p = geometry.p;
    cross_section_input.N = geometry.N;
    cross_section_input.curvature = geometry.curvature;
    cross_section_input.p_w = geometry.p_w;
    cross_section_input.N_w = geometry.N_w;
    cross_section_input.minimum_reference_speed =
        config_.cross_section.minimum_reference_speed;
    cross_section_input.current_delta = selected_delta;
    cross_section_input.current_delta_valid = IsFinite(selected_delta);
    cross_section_input.path_revision = path.path_revision;
    cross_section_input.frame_revision = path.frame_revision;
    cross_section_input.clearance_query = clearance_query;
    cross_section_input.directional_query_count =
        &profile.diagnostics.cross_section_directional_query_count;
    cross_section_input.max_bounded_construction_abs_delta =
        &profile.diagnostics.max_bounded_construction_abs_delta;
    const TubeCrossSectionResult cross_section =
        cross_section_solver.solve(cross_section_input);
    CopyCrossSectionFacts(cross_section, sample);
    sample.regularity_lower = cross_section.lower_curvature;
    sample.regularity_upper = cross_section.upper_curvature;
    sample.regularity_intersection = cross_section.reason !=
            TubeCrossSectionReason::CURVATURE_NUMERICAL_FAILURE &&
        cross_section.reason !=
            TubeCrossSectionReason::EMPTY_AFTER_CURVATURE_INTERSECTION;
    // Contract A charges the geometric cover exactly once in the Validator.
    // Builder never erodes the ESDF interval by voxel resolution (nor by a
    // certificate residual); all centre queries request planner_safe_distance
    // only.  Keep the inset fields as zero diagnostic metadata.
    const double continuous_inset = 0.0;
    sample.continuous_inset = 0.0;
    sample.cell_geometry_certificate_used = all_active_cells_certified;
    sample.cell_geometry_inset = all_active_cells_certified
        ? continuous_inset : 0.0;
    sample.raw_lower = std::min(0.0,
        cross_section.lower_final + continuous_inset);
    sample.raw_upper = std::max(0.0,
        cross_section.upper_final - continuous_inset);
    sample.filtered_lower = sample.raw_lower;
    sample.filtered_upper = sample.raw_upper;
    sample.environment_lower = sample.raw_lower;
    sample.environment_upper = sample.raw_upper;
    sample.environment_width = std::max(0.0,
        sample.environment_upper - sample.environment_lower);
    sample.environment_interval_nonempty =
        sample.environment_lower <= sample.environment_upper + kEpsilon;
    sample.environment_contains_zero = sample.environment_interval_nonempty &&
        sample.environment_lower <= 0.0 && 0.0 <= sample.environment_upper;
    sample.post_inset_contains_zero = sample.environment_contains_zero;
    sample.filter_input_contains_zero = sample.post_inset_contains_zero;
    sample.filtered_contains_zero = sample.filter_input_contains_zero;
    sample.positive_stop = StopReason(cross_section.positive_termination);
    sample.negative_stop = StopReason(cross_section.negative_termination);
    sample.positive_certified = cross_section.valid;
    sample.negative_certified = cross_section.valid;
    sample.complete = cross_section.valid && CrossSectionFactsFinite(cross_section) &&
        sample.environment_interval_nonempty;
    if (!sample.complete) {
      const TubeStopReason failure = cross_section.valid
          ? TubeStopReason::INSUFFICIENT_CLEARANCE
          : CrossSectionFailureStop(cross_section);
      CountStop(failure, profile.diagnostics);
      RecordFirstInvalid(path.w, 0, failure, profile.diagnostics);
      ++profile.diagnostics.invalid_count;
    }
    profile.diagnostics.min_width = std::min(profile.diagnostics.min_width,
        std::max(0.0, sample.environment_width));
    if (base.status == DistanceStatus::KNOWN_FREE &&
        base.clearance_certified && IsFinite(base.clearance)) {
      profile.diagnostics.min_safety_margin = std::min(
          profile.diagnostics.min_safety_margin,
          base.clearance - config_.cross_section.planner_safe_distance);
    }
    evaluated.push_back(sample);
    sample_complete.push_back(sample.complete);
  }

  // A complete matching cell certificate opts the Validator into its
  // fail-closed proof path.  Builder records the certificate facts but never
  // applies a geometric inset; if the delayed combined-regularity proof fails
  // the Validator must observe the attempted certificate and reject without a
  // sampled fallback.  Missing/malformed callbacks (all_active=false) remain
  // eligible for the Validator's sampled fallback cover.
  if (all_active_cells_certified) {
    bool certificate_offset_eligible = true;
    double combined_speed_min = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0U; index + 1U < evaluated.size(); ++index) {
      const bool queried = index < certified_cells.size();
      const phase_offset_core::PathCellGeometryCertificate* certificate =
          queried ? &certified_cells[index] : nullptr;
      const double maximum_delta = std::max({
          std::abs(evaluated[index].pre_inset_lower),
          std::abs(evaluated[index].pre_inset_upper),
          std::abs(evaluated[index + 1U].pre_inset_lower),
          std::abs(evaluated[index + 1U].pre_inset_upper)});
      const double conservative_speed = queried && certificate != nullptr
          ? certificate->inf_p_w_norm - certificate->sup_N_w_norm * maximum_delta
          : -std::numeric_limits<double>::infinity();
      combined_speed_min = std::min(combined_speed_min, conservative_speed);
      if (!queried || certificate == nullptr ||
          !phase_offset_core::pathCellGeometryCertificateIsComplete(*certificate) ||
          !IsFinite(maximum_delta) ||
          conservative_speed < config_.cross_section.minimum_reference_speed) {
        certificate_offset_eligible = false;
        break;
      }
    }
    if (certificate_offset_eligible) {
      profile.combined_regularity_proof_complete = true;
      profile.combined_regularity_speed_min = IsFinite(combined_speed_min)
          ? combined_speed_min : 0.0;
    }
  }

  if (!IsFinite(profile.diagnostics.min_width)) profile.diagnostics.min_width = 0.0;
  if (!IsFinite(profile.diagnostics.min_safety_margin)) {
    profile.diagnostics.min_safety_margin = 0.0;
  }
  // Preserve all raw knots before selecting the current-containing segment.
  // This is evidence from this Builder invocation only; no map query is
  // repeated by downstream auditing.
  profile.raw_build_samples = evaluated;
  if (current_index == adaptive_preview.size()) {
    profile.samples = evaluated;
    profile.diagnostics.invalid_reason =
        "adaptive preview does not contain the current phase sample";
    RecordFirstInvalid(current_w, 0, TubeStopReason::INVALID_PATH,
                       profile.diagnostics);
    ++profile.diagnostics.invalid_count;
    profile.raw_complete = false;
  } else if (!sample_complete[current_index]) {
    // A nonempty remote component cannot make an invalid current
    // cross-section displayable or installable.
    profile.samples = evaluated;
    profile.raw_complete = false;
  } else {
    std::size_t first = current_index;
    std::size_t last = current_index;
    while (first > 0U && sample_complete[first - 1U]) --first;
    while (last + 1U < sample_complete.size() && sample_complete[last + 1U]) ++last;
    profile.samples.assign(evaluated.begin() + first,
                           evaluated.begin() + last + 1U);
    profile.preview_start_w = profile.samples.front().w;
    profile.preview_end_w = profile.samples.back().w;
    profile.certified_segment_start_w = profile.preview_start_w;
    profile.certified_segment_end_w = profile.preview_end_w;
    profile.certified_segment_truncated_before = first > 0U;
    profile.certified_segment_truncated_after = last + 1U < evaluated.size();
    profile.raw_complete = true;
  }
  for (std::size_t index = 0U; index < sample_complete.size(); ++index) {
    if (!sample_complete[index]) {
      profile.first_truncated_w = evaluated[index].w;
      profile.first_truncated_reason = evaluated[index].positive_stop !=
          TubeStopReason::NONE ? evaluated[index].positive_stop :
          evaluated[index].negative_stop;
      if (profile.first_truncated_reason == TubeStopReason::NONE) {
        profile.first_truncated_reason = TubeStopReason::INSUFFICIENT_CLEARANCE;
      }
      break;
    }
  }
  profile.filtered_complete = false;
  profile.complete = false;
  profile.obstacle_certified = profile.raw_complete;
  profile.zero_component_contains_zero = profile.raw_complete;
  profile.selected_component = TubeComponentSelection::ZERO_CONNECTED;
  if (profile.cell_geometry_certified &&
      profile.combined_regularity_proof_complete && profile.raw_complete) {
    const std::uint64_t proof_path_revision = profile.samples.front().path_revision;
    const std::uint64_t proof_frame_revision = profile.samples.front().frame_revision;
    bool matching_proof = true;
    for (const TubeRawSample& sample : profile.samples) {
      matching_proof = matching_proof && sample.complete &&
          sample.path_revision == proof_path_revision &&
          sample.frame_revision == proof_frame_revision;
    }
    if (matching_proof) {
      for (TubeRawSample& sample : profile.samples) {
        sample.proof_level = TubeProofLevel::FRAME_CELL_PROOF;
      }
      profile.path_revision = proof_path_revision;
      profile.frame_revision = proof_frame_revision;
      profile.proof_level = TubeProofLevel::FRAME_CELL_PROOF;
    } else {
      profile.cell_geometry_certified = false;
      profile.certified_cell_count = 0U;
      profile.proof_level = TubeProofLevel::SAMPLED_EVIDENCE;
    }
  } else {
    profile.proof_level = TubeProofLevel::SAMPLED_EVIDENCE;
  }
  if (!profile.raw_complete && profile.diagnostics.invalid_reason.empty()) {
    profile.diagnostics.invalid_reason =
        "current cloud-clearance cross-section is incomplete";
  } else if (profile.raw_complete &&
             (profile.certified_segment_truncated_before ||
              profile.certified_segment_truncated_after)) {
    profile.diagnostics.invalid_reason =
        "requested preview was truncated to the current cloud-clearance segment";
  }
  FinalizeConstructionDiagnostics(profile.diagnostics);
  return profile.raw_complete;
}

bool TubeBuilder::buildRawOccupancy(
    const TubeSource source,
    const std::vector<phase_offset_core::PathDifferentialState,
                      Eigen::aligned_allocator<phase_offset_core::PathDifferentialState>>&
        preview,
    const RawOccupancyQuery& occupancy_query,
    const std::uint64_t source_revision,
    const std::uint64_t tube_revision,
    TubeProfile& profile,
    const double current_delta) const {
  const double current_w = preview.empty() ? 0.0 : preview.front().w;
  return buildRawOccupancy(source, preview, occupancy_query, current_w,
                           source_revision, tube_revision, profile,
                           current_delta);
}

}  // namespace phase_offset_navigation
