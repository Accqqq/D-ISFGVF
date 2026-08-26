#include "phase_offset_navigation/tube_epoch_manager.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace phase_offset_navigation {
namespace {

bool IsFinite(double value) {
  return std::isfinite(value);
}

bool IsFinite(const Eigen::Vector3d& value) {
  return value.allFinite();
}

bool NearlyEqual(double first, double second, double tolerance) {
  return IsFinite(first) && IsFinite(second) &&
      std::abs(first - second) <= tolerance;
}

bool SampleFinite(const TubeRawSample& sample) {
  return IsFinite(sample.w) && IsFinite(sample.p) && IsFinite(sample.N) &&
      IsFinite(sample.raw_lower) && IsFinite(sample.raw_upper) &&
      IsFinite(sample.filtered_lower) && IsFinite(sample.filtered_upper) &&
      IsFinite(sample.lower_w) && IsFinite(sample.upper_w) &&
      IsFinite(sample.fixed_lower) && IsFinite(sample.fixed_upper) &&
      IsFinite(sample.regularity_lower) && IsFinite(sample.regularity_upper) &&
      IsFinite(sample.esdf_lower) && IsFinite(sample.esdf_upper) &&
      IsFinite(sample.base_signed_distance) && IsFinite(sample.c_plus_raw) &&
      IsFinite(sample.c_minus_raw) &&
      IsFinite(sample.full_effective_radius) &&
      IsFinite(sample.preincluded_map_uncertainty) &&
      IsFinite(sample.residual_effective_radius) &&
      IsFinite(sample.effective_radius) &&
      IsFinite(sample.obstacle_lower) && IsFinite(sample.obstacle_upper) &&
      IsFinite(sample.curvature_lower) && IsFinite(sample.curvature_upper) &&
      IsFinite(sample.environment_lower) && IsFinite(sample.environment_upper) &&
      IsFinite(sample.environment_width);
}

bool SampleEquivalent(const TubeRawSample& first,
                      const TubeRawSample& second,
                      double tolerance) {
  return SampleFinite(first) && SampleFinite(second) &&
      NearlyEqual(first.w, second.w, tolerance) &&
      NearlyEqual(first.raw_lower, second.raw_lower, tolerance) &&
      NearlyEqual(first.raw_upper, second.raw_upper, tolerance) &&
      NearlyEqual(first.filtered_lower, second.filtered_lower, tolerance) &&
      NearlyEqual(first.filtered_upper, second.filtered_upper, tolerance) &&
      NearlyEqual(first.lower_w, second.lower_w, tolerance) &&
      NearlyEqual(first.upper_w, second.upper_w, tolerance) &&
      NearlyEqual(first.c_plus_raw, second.c_plus_raw, tolerance) &&
      NearlyEqual(first.c_minus_raw, second.c_minus_raw, tolerance) &&
      NearlyEqual(first.full_effective_radius, second.full_effective_radius,
                  tolerance) &&
      NearlyEqual(first.preincluded_map_uncertainty,
                  second.preincluded_map_uncertainty, tolerance) &&
      NearlyEqual(first.residual_effective_radius,
                  second.residual_effective_radius, tolerance) &&
      NearlyEqual(first.effective_radius, second.effective_radius, tolerance) &&
      NearlyEqual(first.obstacle_lower, second.obstacle_lower, tolerance) &&
      NearlyEqual(first.obstacle_upper, second.obstacle_upper, tolerance) &&
      NearlyEqual(first.curvature_lower, second.curvature_lower, tolerance) &&
      NearlyEqual(first.curvature_upper, second.curvature_upper, tolerance) &&
      NearlyEqual(first.environment_lower, second.environment_lower, tolerance) &&
      NearlyEqual(first.environment_upper, second.environment_upper, tolerance) &&
      NearlyEqual(first.environment_width, second.environment_width, tolerance) &&
      first.complete == second.complete &&
      first.positive_certified == second.positive_certified &&
      first.negative_certified == second.negative_certified &&
      first.regularity_intersection == second.regularity_intersection &&
      first.environment_interval_nonempty == second.environment_interval_nonempty &&
      first.environment_contains_zero == second.environment_contains_zero &&
      first.positive_stop == second.positive_stop &&
      first.negative_stop == second.negative_stop &&
      first.cross_section_reason == second.cross_section_reason &&
      first.positive_ray_termination == second.positive_ray_termination &&
      first.negative_ray_termination == second.negative_ray_termination;
}

bool ProfileFinite(const TubeProfile& profile) {
  if (!IsFinite(profile.preview_start_w) || !IsFinite(profile.preview_end_w) ||
      profile.preview_end_w < profile.preview_start_w ||
      !IsFinite(profile.requested_preview_start_w) ||
      !IsFinite(profile.requested_preview_end_w) ||
      profile.requested_preview_end_w < profile.requested_preview_start_w ||
      !IsFinite(profile.certified_segment_start_w) ||
      !IsFinite(profile.certified_segment_end_w) ||
      profile.certified_segment_end_w < profile.certified_segment_start_w ||
      !IsFinite(profile.first_truncated_w) ||
      !IsFinite(profile.combined_regularity_speed_min)) {
    return false;
  }
  for (const TubeRawSample& sample : profile.samples) {
    if (!SampleFinite(sample)) return false;
  }
  return true;
}

bool IsInside(const TubeBounds& bounds,
              double delta,
              double interior_margin,
              double tolerance) {
  return bounds.valid && IsFinite(delta) && IsFinite(interior_margin) &&
      IsFinite(tolerance) && delta >= bounds.lower + interior_margin - tolerance &&
      delta <= bounds.upper - interior_margin + tolerance;
}

enum class QuerySafety {
  INDETERMINATE,
  SAFE,
  UNSAFE,
};

QuerySafety CheckClearance(const ClearanceQuery& query,
                           const Eigen::Vector3d& point,
                           const double required_clearance,
                           double& clearance) {
  clearance = 0.0;
  if (!query || !IsFinite(point) || !IsFinite(required_clearance) ||
      required_clearance < 0.0) {
    return QuerySafety::INDETERMINATE;
  }
  const ClearanceQueryResult result = query(point, required_clearance);
  if (result.status == DistanceStatus::OCCUPIED ||
      result.status == DistanceStatus::OUT_OF_MAP) {
    return QuerySafety::UNSAFE;
  }
  if (result.status != DistanceStatus::KNOWN_FREE ||
      !result.clearance_certified || !IsFinite(result.clearance)) {
    return QuerySafety::INDETERMINATE;
  }
  clearance = result.clearance;
  return clearance >= required_clearance ? QuerySafety::SAFE
                                         : QuerySafety::UNSAFE;
}

}  // namespace

const char* tubeEpochReasonName(TubeEpochReason reason) {
  switch (reason) {
    case TubeEpochReason::NONE: return "none";
    case TubeEpochReason::INVALID_CONFIGURATION: return "invalid_configuration";
    case TubeEpochReason::SOURCE_NONE: return "source_none";
    case TubeEpochReason::CANDIDATE_INCOMPLETE: return "candidate_incomplete";
    case TubeEpochReason::CURRENT_GEOMETRY_INVALID: return "current_geometry_invalid";
    case TubeEpochReason::CURRENT_BOUNDS_INVALID: return "current_bounds_invalid";
    case TubeEpochReason::CURRENT_OFFSET_OUTSIDE: return "current_offset_outside";
    case TubeEpochReason::REFERENCE_INDETERMINATE: return "reference_indeterminate";
    case TubeEpochReason::ACTUAL_INDETERMINATE: return "actual_indeterminate";
    case TubeEpochReason::REFERENCE_CLEARANCE_INSUFFICIENT:
      return "reference_clearance_insufficient";
    case TubeEpochReason::ACTUAL_CLEARANCE_INSUFFICIENT:
      return "actual_clearance_insufficient";
    case TubeEpochReason::BASE_CENTERLINE_CLEARANCE_INSUFFICIENT:
      return "base_centerline_clearance_insufficient";
    case TubeEpochReason::BASE_CENTERLINE_INDETERMINATE:
      return "base_centerline_indeterminate";
    case TubeEpochReason::BASE_CENTERLINE_CONTINUITY_UNCERTIFIED:
      return "base_centerline_continuity_uncertified";
    case TubeEpochReason::FORWARD_HORIZON_SHORT: return "forward_horizon_short";
  }
  return "none";
}

const char* controlFailureReasonName(const ControlFailureReason reason) {
  switch (reason) {
    case ControlFailureReason::NONE: return "none";
    case ControlFailureReason::ZERO_PORT_EQUIVALENCE:
      return "zero_port_equivalence";
    case ControlFailureReason::GEOMETRY_INVARIANT:
      return "geometry_invariant";
    case ControlFailureReason::BASE_GUIDANCE_INVARIANT:
      return "base_guidance_invariant";
    case ControlFailureReason::PORT_PROJECTOR_CONTRADICTION:
      return "port_projector_contradiction";
    case ControlFailureReason::MATCHED_PORT_INVARIANT:
      return "matched_port_invariant";
  }
  return "none";
}

TubeEpochManager::TubeEpochManager(const TubeEpochManagerConfig& config)
    : config_(config), certified_builder_(config.builder, config.filter,
                                          config.surface_validator) {
  phase_offset_core::GeometryParams geometry_config;
  geometry_config.regularity_margin = config_.builder.regularity_margin;
  geometry_config.minimum_reference_speed =
      config_.builder.cross_section.minimum_reference_speed;
  geometry_evaluator_ = phase_offset_core::GeometryEvaluator(geometry_config);
  configuration_valid_ = certified_builder_.configurationValid() &&
      IsFinite(config_.profile_equivalence_tolerance) &&
      config_.profile_equivalence_tolerance >= 0.0 &&
      IsFinite(config_.inside_tolerance) && config_.inside_tolerance >= 0.0;
}

bool TubeEpochManager::configurationValid() const {
  return configuration_valid_;
}

bool TubeEpochManager::profilesEquivalent(const TubeProfile& first,
                                          const TubeProfile& second,
                                          double tolerance) {
  if (!IsFinite(tolerance) || tolerance < 0.0 || !ProfileFinite(first) ||
      !ProfileFinite(second) || first.source != second.source ||
      first.source_revision != second.source_revision ||
      !NearlyEqual(first.preview_start_w, second.preview_start_w, tolerance) ||
      !NearlyEqual(first.preview_end_w, second.preview_end_w, tolerance) ||
      !NearlyEqual(first.requested_preview_start_w,
                   second.requested_preview_start_w, tolerance) ||
      !NearlyEqual(first.requested_preview_end_w,
                   second.requested_preview_end_w, tolerance) ||
      !NearlyEqual(first.certified_segment_start_w,
                   second.certified_segment_start_w, tolerance) ||
      !NearlyEqual(first.certified_segment_end_w,
                   second.certified_segment_end_w, tolerance) ||
      first.certified_segment_truncated_before !=
          second.certified_segment_truncated_before ||
      first.certified_segment_truncated_after !=
          second.certified_segment_truncated_after ||
      !NearlyEqual(first.first_truncated_w, second.first_truncated_w,
                   tolerance) ||
      first.first_truncated_reason != second.first_truncated_reason ||
      first.samples.size() != second.samples.size() ||
      first.raw_complete != second.raw_complete ||
      first.filtered_complete != second.filtered_complete ||
      first.complete != second.complete ||
      first.obstacle_certified != second.obstacle_certified ||
      first.combined_regularity_proof_complete !=
          second.combined_regularity_proof_complete ||
      !NearlyEqual(first.combined_regularity_speed_min,
                   second.combined_regularity_speed_min, tolerance) ||
      first.classification != second.classification) {
    return false;
  }
  for (std::size_t index = 0U; index < first.samples.size(); ++index) {
    if (!SampleEquivalent(first.samples[index], second.samples[index], tolerance)) {
      return false;
    }
  }
  return true;
}

void TubeEpochManager::copyPersistentStatus(TubeEpochStatus& status) const {
  status.candidate_sequence = candidate_sequence_;
  status.active_tube_epoch = active_tube_epoch_;
  status.active_available = active_available_;
  status.active_classification = active_available_
      ? active_profile_.classification : TubeProfileClassification::NONE;
  status.active_zero_only = active_available_ && active_profile_.zero_only;
  status.active_zero_component_contains_zero = active_available_ &&
      active_profile_.zero_component_contains_zero;
  status.active_current_validation_valid = active_current_validation_valid_;
  status.active_path_source_revision = active_path_source_revision_;
  status.active_map_observation_sequence = active_map_observation_sequence_;
  status.equivalent_refresh_count = equivalent_refresh_count_;
  status.install_count = install_count_;
  status.reject_count = reject_count_;
  status.wait_count = wait_count_;
}

void TubeEpochManager::copyResult(const TubeEpochStatus& status,
                                  TubeEpochUpdateResult& result) const {
  result = TubeEpochUpdateResult();
  result.candidate_profile = candidate_profile_;
  if (active_available_) result.active_profile = active_profile_;
  result.status = status;
}

void TubeEpochManager::installActive(const TubeProfile& candidate,
                                     const TubeEpochUpdateInput& input,
                                     bool safety_replacement,
                                     TubeEpochStatus& status) {
  const bool equivalent = active_available_ && profilesEquivalent(
      active_profile_, candidate, config_.profile_equivalence_tolerance);
  if (equivalent) {
    active_map_observation_sequence_ = input.map_observation_sequence;
    ++equivalent_refresh_count_;
    status.disposition = TubeInstallDisposition::EQUIVALENT_REFRESH;
  } else {
    const bool first_install = !active_available_;
    active_profile_ = candidate;
    active_available_ = true;
    active_path_source_revision_ = input.path_source_revision;
    active_map_observation_sequence_ = input.map_observation_sequence;
    ++active_tube_epoch_;
    ++install_count_;
    status.disposition = safety_replacement ? TubeInstallDisposition::SAFETY_REPLACEMENT :
        (first_install ? TubeInstallDisposition::INITIAL_INSTALL :
         TubeInstallDisposition::REPLACED_ACTIVE);
  }
}

void TubeEpochManager::markWaiting(TubeEpochReason reason,
                                   TubeEpochStatus& status) {
  ++wait_count_;
  status.state = TubeEpochState::WAITING_FOR_CANDIDATE;
  status.disposition = TubeInstallDisposition::REJECTED_CANDIDATE;
  status.reason = reason;
  status.reason_text = tubeEpochReasonName(reason);
  status.transient_blocked = true;
}

bool TubeEpochManager::update(const TubeEpochUpdateInput& input,
                              TubeEpochUpdateResult& result) {
  TubeEpochStatus status;
  status.retained_delta = IsFinite(input.retained_delta) ? input.retained_delta : 0.0;
  status.candidate_path_source_revision = input.path_source_revision;
  status.candidate_map_observation_sequence = input.map_observation_sequence;
  status.map_observation_is_snapshot = input.map_observation_is_snapshot;
  if (!configuration_valid_) {
    status.state = TubeEpochState::CONFIGURATION_ERROR;
    status.reason = TubeEpochReason::INVALID_CONFIGURATION;
    status.reason_text = tubeEpochReasonName(status.reason);
    copyPersistentStatus(status);
    copyResult(status, result);
    return false;
  }
  if (input.source == TubeSource::NONE) {
    candidate_profile_ = TubeProfile();
    active_profile_ = TubeProfile();
    active_available_ = false;
    active_current_validation_valid_ = false;
    status.state = TubeEpochState::NO_ACTIVE_TUBE;
    status.reason = TubeEpochReason::SOURCE_NONE;
    status.reason_text = tubeEpochReasonName(status.reason);
    copyPersistentStatus(status);
    copyResult(status, result);
    return true;
  }

  ++candidate_sequence_;
  // Cloud clearance is the only production ESDF certificate.  A missing
  // immutable cloud query stays fail-closed; there is no legacy distance-query
  // fallback.
  const bool cloud_clearance_source = input.source == TubeSource::ESDF;
  status.raw_cross_section_path_used = cloud_clearance_source &&
      static_cast<bool>(input.cloud_clearance_query);
  CertifiedTubeBuildInput build_input;
  build_input.source = input.source;
  build_input.preview_path = input.preview_path;
  build_input.authority_request = input.authority_request;
  build_input.cloud_clearance_query = input.cloud_clearance_query;
  build_input.path_state_query = input.path_state_query;
  build_input.path_cell_bound_query = input.path_cell_bound_query;
  build_input.cloud_snapshot_resolution = input.cloud_snapshot_resolution;
  build_input.current_w = input.current_path.w;
  build_input.current_delta = input.retained_delta;
  build_input.path_source_revision = input.path_source_revision;
  build_input.tube_revision = candidate_sequence_;
  build_input.map_observation_sequence = input.map_observation_sequence;
  build_input.map_observation_is_snapshot = input.map_observation_is_snapshot;
  CertifiedTubeBuildResult build_result;
  certified_builder_.build(build_input, build_result);
  TubeProfile candidate = build_result.profile;
  candidate_profile_ = candidate;
  status.candidate_raw_complete = build_result.raw_complete;
  status.candidate_filtered_complete = build_result.filtered_complete;
  status.candidate_complete = build_result.complete;
  status.candidate_classification = candidate.classification;
  status.candidate_zero_only = candidate.zero_only;
  status.candidate_zero_component_contains_zero =
      candidate.zero_component_contains_zero;
  if (!status.candidate_complete) {
    // An incomplete/unknown candidate normally leaves the installed active
    // profile alone.  It is only allowed to revoke that ownership when the
    // newest categorical query itself says OCCUPIED or OUT_OF_MAP at the
    // current reference/actual point.
    phase_offset_core::GeometryParams geometry_params;
    geometry_params.regularity_margin = cloud_clearance_source
        ? config_.builder.cross_section.regularity_margin
        : config_.builder.regularity_margin;
    geometry_params.minimum_reference_speed =
        config_.builder.cross_section.minimum_reference_speed;
    phase_offset_core::GeometryEvaluator evaluator(geometry_params);
    phase_offset_core::PhaseOffsetGeometryState geometry;
    status.current_geometry_valid = evaluator.evaluate(
        input.current_path, input.actual_position, input.retained_delta, geometry) &&
        geometry.valid;
    QuerySafety reference_safety = QuerySafety::INDETERMINATE;
    QuerySafety actual_safety = QuerySafety::INDETERMINATE;
    if (input.source == TubeSource::ESDF && status.current_geometry_valid) {
      const double required_clearance =
          config_.builder.cross_section.planner_safe_distance;
      reference_safety = CheckClearance(input.cloud_clearance_query, geometry.r,
                                        required_clearance,
                                        status.reference_signed_distance);
      actual_safety = CheckClearance(input.cloud_clearance_query,
                                     input.actual_position, required_clearance,
                                     status.actual_signed_distance);
      status.reference_clearance_sufficient = reference_safety == QuerySafety::SAFE;
      status.actual_clearance_sufficient = actual_safety == QuerySafety::SAFE;
      const bool explicit_unsafe = reference_safety == QuerySafety::UNSAFE ||
          actual_safety == QuerySafety::UNSAFE;
      status.current_safety_status = explicit_unsafe ? CurrentSafetyStatus::UNSAFE
          : ((reference_safety == QuerySafety::INDETERMINATE ||
              actual_safety == QuerySafety::INDETERMINATE)
                 ? CurrentSafetyStatus::INDETERMINATE : CurrentSafetyStatus::SAFE);
      if (explicit_unsafe) {
        ++reject_count_;
        active_current_validation_valid_ = false;
        status.state = TubeEpochState::CERTIFICATE_DENIED;
        status.disposition = TubeInstallDisposition::REJECTED_CANDIDATE;
        status.reason = reference_safety == QuerySafety::UNSAFE
            ? TubeEpochReason::REFERENCE_CLEARANCE_INSUFFICIENT
            : TubeEpochReason::ACTUAL_CLEARANCE_INSUFFICIENT;
        status.reason_text = tubeEpochReasonName(status.reason);
        status.certificate_denied = true;
        copyPersistentStatus(status);
        copyResult(status, result);
        return true;
      }
    }
    ++reject_count_;
    markWaiting(TubeEpochReason::CANDIDATE_INCOMPLETE, status);
    copyPersistentStatus(status);
    copyResult(status, result);
    return false;
  }

  phase_offset_core::PhaseOffsetGeometryState geometry;
  phase_offset_core::GeometryParams current_geometry_params;
  current_geometry_params.regularity_margin = cloud_clearance_source
      ? config_.builder.cross_section.regularity_margin
      : config_.builder.regularity_margin;
  current_geometry_params.minimum_reference_speed =
      config_.builder.cross_section.minimum_reference_speed;
  phase_offset_core::GeometryEvaluator current_geometry_evaluator(
      current_geometry_params);
  status.current_geometry_valid = current_geometry_evaluator.evaluate(
      input.current_path, input.actual_position, input.retained_delta, geometry) &&
      geometry.valid;
  TubeBounds current_bounds;
  if (status.current_geometry_valid) {
    status.current_bounds_valid = TubeFilter::query(candidate, input.current_path.w,
                                                    current_bounds);
    status.current_interval_nonempty = current_bounds.valid &&
        current_bounds.lower <= current_bounds.upper + config_.inside_tolerance;
    status.current_interval_contains_zero = current_bounds.valid &&
        current_bounds.lower <= 0.0 && current_bounds.upper >= 0.0;
    status.current_interval_contains_retained_delta = current_bounds.valid &&
        current_bounds.lower <= input.retained_delta &&
        current_bounds.upper >= input.retained_delta;
    status.retained_delta_current_inside = IsInside(current_bounds, input.retained_delta,
        config_.builder.interior_margin, config_.inside_tolerance);
    status.tracking_error_bound = cloud_clearance_source
        ? config_.builder.cross_section.margins.tracking_error_bound
        : config_.builder.erosion.tracking_error_bound;
    status.tracking_error_norm = (input.actual_position - geometry.r).norm();
    status.tracking_within_bound = IsFinite(status.tracking_error_norm) &&
        status.tracking_error_norm <= status.tracking_error_bound +
            config_.inside_tolerance;
  }
  status.full_effective_radius = cloud_clearance_source
      ? config_.builder.cross_section.margins.fullEffectiveRadius() : 0.0;
  status.preincluded_map_uncertainty = cloud_clearance_source
      ? config_.builder.cross_section.margins.preincluded_map_uncertainty : 0.0;
  status.residual_effective_radius = cloud_clearance_source
      ? config_.builder.cross_section.planner_safe_distance : 0.0;
  // Full/preincluded fields above are deprecated accounting diagnostics.
  // Production geometry and all current snapshot queries use only the
  // planner clearance below.
  status.required_reference_clearance = cloud_clearance_source
      ? config_.builder.cross_section.planner_safe_distance
      : config_.builder.erosion.requiredReferenceClearance();
  status.required_actual_clearance = cloud_clearance_source
      ? config_.builder.cross_section.planner_safe_distance
      : config_.builder.erosion.requiredActualClearance();
  status.certified_forward_w = status.current_geometry_valid &&
      IsFinite(candidate.certified_segment_end_w) && IsFinite(input.current_path.w)
      ? std::max(0.0, candidate.certified_segment_end_w - input.current_path.w) : 0.0;
  status.forward_horizon_sufficient = status.certified_forward_w +
      config_.inside_tolerance >= config_.builder.min_certified_forward_w;
  const bool candidate_classified_zero_only = candidate.classification ==
      TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE;
  const bool candidate_is_zero_only = candidate_classified_zero_only &&
      candidate.zero_only && candidate.current_delta_valid &&
      candidate.current_delta == 0.0;
  const bool zero_only_neutral = candidate_is_zero_only &&
      input.retained_delta == 0.0;
  bool current_offset_geometrically_inside = cloud_clearance_source
      ? status.current_interval_contains_retained_delta
      : status.retained_delta_current_inside;
  // A zero-width planner baseline is never a current-safe witness for a
  // nonzero retained offset, even when the generic inside tolerance would
  // otherwise absorb that offset into [0,0].
  if (candidate_classified_zero_only && input.retained_delta != 0.0) {
    current_offset_geometrically_inside = false;
  }
  // Tracking error is a robust-certificate monitor, not categorical obstacle
  // evidence.  It therefore never turns an otherwise usable current tube
  // into a certificate denial here; Runtime still evaluates its live U+ then
  // U_safe port sequence independently.
  // A fresh candidate that cannot contain the retained offset is not an
  // installable owner, but it is not categorical map evidence against a
  // previously installed local corridor.  Only an explicit current map fact
  // may revoke that ownership here; Runtime owns the exact current port test.
  bool explicit_unsafe = false;
  bool indeterminate = !status.current_geometry_valid || !status.current_bounds_valid;
  QuerySafety reference_safety = QuerySafety::INDETERMINATE;
  QuerySafety actual_safety = QuerySafety::INDETERMINATE;
  QuerySafety base_centerline_safety = QuerySafety::INDETERMINATE;
  if (cloud_clearance_source) {
    if (status.current_geometry_valid) {
      const double required_clearance =
          config_.builder.cross_section.planner_safe_distance;
      reference_safety = CheckClearance(input.cloud_clearance_query, geometry.r,
                                        required_clearance,
                                        status.reference_signed_distance);
      actual_safety = CheckClearance(input.cloud_clearance_query,
                                     input.actual_position, required_clearance,
                                     status.actual_signed_distance);
      double base_clearance = 0.0;
      base_centerline_safety = CheckClearance(
          input.cloud_clearance_query, input.current_path.p, required_clearance,
          base_clearance);
      status.base_centerline_clearance_sufficient =
          base_centerline_safety == QuerySafety::SAFE;
    }
    status.reference_clearance_sufficient = reference_safety == QuerySafety::SAFE;
    status.actual_clearance_sufficient = actual_safety == QuerySafety::SAFE;
    explicit_unsafe = explicit_unsafe || reference_safety == QuerySafety::UNSAFE ||
        actual_safety == QuerySafety::UNSAFE;
    if (!zero_only_neutral) {
      indeterminate = indeterminate ||
          reference_safety == QuerySafety::INDETERMINATE ||
          actual_safety == QuerySafety::INDETERMINATE;
    }
    const bool observed_indeterminate =
        reference_safety == QuerySafety::INDETERMINATE ||
        actual_safety == QuerySafety::INDETERMINATE;
    status.current_safety_status = explicit_unsafe ? CurrentSafetyStatus::UNSAFE :
        (observed_indeterminate ? CurrentSafetyStatus::INDETERMINATE :
         CurrentSafetyStatus::SAFE);
  }
  status.current_state_admissible = status.current_geometry_valid &&
      status.current_bounds_valid && current_offset_geometrically_inside &&
      (input.source == TubeSource::FIXED ||
      (cloud_clearance_source
            ? (zero_only_neutral ||
               (status.reference_clearance_sufficient &&
                status.actual_clearance_sufficient))
            : status.current_safety_status == CurrentSafetyStatus::SAFE));

  // A zero-only candidate is a valid planner-baseline Tube.  It cannot replace
  // an active nonzero authority while that authority still retains delta != 0,
  // but its snapshot observation is not a planner-path veto.
  if (cloud_clearance_source && !current_offset_geometrically_inside) {
    explicit_unsafe = false;
  }
  // A latest Tube snapshot may observe an occupied/unknown nominal centreline,
  // but in neutral operation that observation only removes offset capacity.
  // Planner/C2 remains responsible for the executable delta=0 path and must
  // not receive a Tube-side certificate denial.  Retained nonzero execution
  // intentionally keeps the existing fail-closed current-safety behaviour.
  if (zero_only_neutral) {
    explicit_unsafe = false;
  }

  const auto unsafe_reason = [&]() {
    if (reference_safety == QuerySafety::UNSAFE) {
      return TubeEpochReason::REFERENCE_CLEARANCE_INSUFFICIENT;
    }
    if (actual_safety == QuerySafety::UNSAFE) {
      return TubeEpochReason::ACTUAL_CLEARANCE_INSUFFICIENT;
    }
    return TubeEpochReason::CURRENT_GEOMETRY_INVALID;
  };

  if (explicit_unsafe) {
    // Latest observation has disproved current safety.  Retaining an older
    // active profile for ownership continuity is allowed, but it must not be
    // used as a current certificate or a manual execution constraint.
    ++reject_count_;
    active_current_validation_valid_ = false;
    status.state = TubeEpochState::CERTIFICATE_DENIED;
    status.disposition = TubeInstallDisposition::REJECTED_CANDIDATE;
    status.reason = unsafe_reason();
    status.reason_text = tubeEpochReasonName(status.reason);
    status.certificate_denied = true;
    copyPersistentStatus(status);
    copyResult(status, result);
    return true;
  }
  if (!status.current_state_admissible || indeterminate) {
    ++reject_count_;
    // A complete latest geometric corridor which excludes the retained state
    // is useful Candidate evidence, but it disproves the current robust
    // certificate.  Keep the older active profile for ownership/history only:
    // Runtime and Certified must not execute or claim safety from it until a
    // fresh candidate contains the retained delta again.  In the ordinary
    // single-UAV case that retained delta is zero, so this cannot turn a
    // one-sided corridor into an implicit lateral-offset request.
    if (!current_offset_geometrically_inside && !zero_only_neutral) {
      active_current_validation_valid_ = false;
    }
    TubeEpochReason reason = !status.current_geometry_valid
        ? TubeEpochReason::CURRENT_GEOMETRY_INVALID
        : (!status.current_bounds_valid ? TubeEpochReason::CURRENT_BOUNDS_INVALID
        : (!current_offset_geometrically_inside
            ? TubeEpochReason::CURRENT_OFFSET_OUTSIDE
            : (reference_safety == QuerySafety::INDETERMINATE
            ? TubeEpochReason::REFERENCE_INDETERMINATE
            : TubeEpochReason::ACTUAL_INDETERMINATE)));
    markWaiting(reason, status);
    copyPersistentStatus(status);
    copyResult(status, result);
    return false;
  }

  // A complete candidate without the existing certified horizon is not a
  // valid rolling certificate.  It is a transient observation shortage, not
  // a software/control failure and it must not replace a previously active
  // profile.
  if (!status.forward_horizon_sufficient) {
    ++reject_count_;
    markWaiting(TubeEpochReason::FORWARD_HORIZON_SHORT, status);
    copyPersistentStatus(status);
    copyResult(status, result);
    return false;
  }

  installActive(candidate, input, false, status);
  active_current_validation_valid_ = true;
  // Tracking is a current certificate fact.  It must not preselect a runtime
  // mode or turn an installable local profile into a separate recovery state.
  status.state = TubeEpochState::ROLLING;
  status.reason = TubeEpochReason::NONE;
  status.reason_text = tubeEpochReasonName(status.reason);
  copyPersistentStatus(status);
  copyResult(status, result);
  return true;
}

}  // namespace phase_offset_navigation
