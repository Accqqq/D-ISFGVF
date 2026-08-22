#include "bspline_race/integration/phase_offset_clearance_audit.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <phase_offset_navigation/tube_types.h>

namespace FLAG_Race {
namespace {

constexpr double kSegmentEps = 1e-8;

bool IsFinite(const double value) { return std::isfinite(value); }

int SegmentCodeForW(const ContinuousPhasePath& path, const double w) {
  if (!std::isfinite(w)) return 0;
  for (const auto& segment : path.segments()) {
    if (w >= segment.w0 - kSegmentEps && w <= segment.w1 + kSegmentEps) {
      if (segment.label == "mapped_bspline") return 1;
      if (segment.label == "c2_quintic") return 2;
      if (segment.label == "nominal_circle") return 3;
      if (segment.label == "nominal_figure8") return 4;
      return 5;
    }
  }
  return 0;
}

void AddUniqueW(std::vector<double>& samples, const double w,
                const double domain_start, const double domain_end) {
  if (std::isfinite(w) && w >= domain_start - kSegmentEps &&
      w <= domain_end + kSegmentEps) {
    samples.push_back(w);
  }
}

std::vector<double> BuildSampleW(const ContinuousPhasePath& path,
                                 const double domain_start,
                                 const double domain_end,
                                 const double step_w) {
  std::vector<double> samples;
  if (!std::isfinite(step_w) || step_w <= 0.0 ||
      !std::isfinite(domain_start) || !std::isfinite(domain_end) ||
      domain_end <= domain_start + kSegmentEps) {
    return samples;
  }
  const int count = static_cast<int>(std::ceil((domain_end - domain_start) / step_w)) + 1;
  for (int index = 0; index <= count; ++index) {
    const double w = domain_start + step_w * static_cast<double>(index);
    if (w <= domain_end + kSegmentEps) samples.push_back(w);
  }
  for (const auto& segment : path.segments()) {
    AddUniqueW(samples, segment.w0, domain_start, domain_end);
    AddUniqueW(samples, segment.w1, domain_start, domain_end);
  }
  AddUniqueW(samples, domain_end, domain_start, domain_end);
  std::sort(samples.begin(), samples.end());
  samples.erase(std::unique(samples.begin(), samples.end(),
      [](const double a, const double b) { return std::abs(a - b) <= kSegmentEps; }), samples.end());
  return samples;
}

std::vector<double> BuildAuditSampleW(
    const PhaseOffsetClearanceAuditInput& input,
    const double domain_start, const double domain_end) {
  if (input.tube_profile != nullptr &&
      !input.tube_profile->raw_build_samples.empty()) {
    std::vector<double> samples;
    samples.reserve(input.tube_profile->raw_build_samples.size());
    for (const auto& sample : input.tube_profile->raw_build_samples) {
      AddUniqueW(samples, sample.w, domain_start, domain_end);
    }
    std::sort(samples.begin(), samples.end());
    samples.erase(std::unique(samples.begin(), samples.end(),
        [](const double a, const double b) {
          return std::abs(a - b) <= kSegmentEps;
        }), samples.end());
    if (!samples.empty()) return samples;
  }
  return BuildSampleW(*input.path, domain_start, domain_end,
                      input.sample_step_w);
}

const phase_offset_navigation::TubeRawSample* FindProfileSample(
    const phase_offset_navigation::TubeProfile* profile, const double w) {
  if (profile == nullptr || !IsFinite(w)) return nullptr;
  const phase_offset_navigation::TubeRawSample* selected = nullptr;
  double best = std::numeric_limits<double>::infinity();
  for (const auto& sample : profile->samples) {
    const double error = std::abs(sample.w - w);
    if (!IsFinite(error) || error > 1e-7) continue;
    if (error < best) {
      selected = &sample;
      best = error;
    }
  }
  return selected;
}

const phase_offset_navigation::TubeRawSample* FindRawBuildSample(
    const phase_offset_navigation::TubeProfile* profile, const double w) {
  if (profile == nullptr || !IsFinite(w)) return nullptr;
  const phase_offset_navigation::TubeRawSample* selected = nullptr;
  double best = std::numeric_limits<double>::infinity();
  for (const auto& sample : profile->raw_build_samples) {
    const double error = std::abs(sample.w - w);
    if (!IsFinite(error) || error > 1e-7) continue;
    if (error < best) {
      selected = &sample;
      best = error;
    }
  }
  return selected;
}

const phase_offset_navigation::TubeValidatorKnotEvidence*
FindValidatorEvidence(const phase_offset_navigation::TubeProfile* profile,
                       const phase_offset_navigation::TubeSurfaceValidationResult*
                           validation,
                       const double w) {
  if (!IsFinite(w)) return nullptr;
  if (profile != nullptr) {
    for (const auto& evidence : profile->validator_knot_evidence) {
      if (std::abs(evidence.w - w) <= 1e-7) return &evidence;
    }
  }
  if (validation != nullptr) {
    for (const auto& evidence : validation->knot_evidence) {
      if (std::abs(evidence.w - w) <= 1e-7) return &evidence;
    }
  }
  return nullptr;
}

phase_offset_navigation::DistanceQueryResult LegacyAsDistance(
    const phase_offset_navigation::DistanceQuery& query,
    const Eigen::Vector3d& point) {
  phase_offset_navigation::DistanceQueryResult result;
  if (!query || !point.allFinite()) return result;
  return query(point);
}

phase_offset_navigation::ClearanceQueryResult LegacyAsClearance(
    const phase_offset_navigation::DistanceQuery& query,
    const Eigen::Vector3d& point, const double required_radius) {
  phase_offset_navigation::ClearanceQueryResult result;
  const auto legacy = LegacyAsDistance(query, point);
  result.status = legacy.status;
  result.clearance = legacy.signed_distance;
  result.clearance_certified = legacy.status ==
      phase_offset_navigation::DistanceStatus::KNOWN_FREE &&
      IsFinite(legacy.signed_distance) &&
      legacy.signed_distance >= required_radius;
  return result;
}

bool ContainsZero(const double lower, const double upper) {
  return IsFinite(lower) && IsFinite(upper) && lower <= 0.0 &&
      0.0 <= upper;
}

int ReasonForStatus(const phase_offset_navigation::DistanceStatus status) {
  using phase_offset_navigation::DistanceStatus;
  using phase_offset_navigation::TubeStopReason;
  switch (status) {
    case DistanceStatus::UNAVAILABLE:
      return static_cast<int>(TubeStopReason::UNAVAILABLE);
    case DistanceStatus::OUT_OF_MAP:
      return static_cast<int>(TubeStopReason::OUT_OF_MAP);
    case DistanceStatus::UNKNOWN:
      return static_cast<int>(TubeStopReason::UNKNOWN);
    case DistanceStatus::OCCUPIED:
      return static_cast<int>(TubeStopReason::OCCUPIED);
    case DistanceStatus::KNOWN_FREE:
      return static_cast<int>(TubeStopReason::NONE);
  }
  return static_cast<int>(TubeStopReason::UNAVAILABLE);
}

struct FailureRecord {
  bool present = false;
  double reason = 0.0;
  bool distance_valid = false;
  double distance = 0.0;
  bool deficit_valid = false;
  double deficit = 0.0;
  double w = 0.0;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  int segment_code = 0;
};

void RecordFailure(FailureRecord& failure,
                   const double reason,
                   const bool distance_valid,
                   const double distance,
                   const bool deficit_valid,
                   const double deficit,
                   const double w,
                   const Eigen::Vector3d& point,
                   const int segment_code) {
  if (failure.present) return;
  failure.present = true;
  failure.reason = reason;
  failure.distance_valid = distance_valid;
  failure.distance = distance;
  failure.deficit_valid = deficit_valid;
  failure.deficit = deficit;
  failure.w = w;
  failure.point = point;
  failure.segment_code = segment_code;
}

void RecordProbe(const phase_offset_navigation::DistanceQueryResult& query,
                 double& known_free_count,
                 double& invalid_count,
                 bool& min_valid,
                 double& min_distance,
                 double& min_w,
                 const double w) {
  using phase_offset_navigation::DistanceStatus;
  if (query.status == DistanceStatus::KNOWN_FREE &&
      std::isfinite(query.signed_distance)) {
    known_free_count += 1.0;
    if (!min_valid || query.signed_distance < min_distance) {
      min_valid = true; min_distance = query.signed_distance; min_w = w;
    }
  } else {
    invalid_count += 1.0;
  }
}

}  // namespace

PhaseOffsetClearanceAuditResult RunPhaseOffsetClearanceAudit(
    const PhaseOffsetClearanceAuditInput& input) {
  PhaseOffsetClearanceAuditResult result;
  auto& v = result.values;
  v[kAuditSchemaVersion] = 1.0; v[kAuditTubeSource] = input.tube_source;
  v[kAuditSourceRevision] = static_cast<double>(input.source_revision); v[kAuditTubeRevision] = static_cast<double>(input.tube_revision);
  v[kAuditZeroGateOpen] = input.zero_gate_open ? 1.0 : 0.0; v[kAuditFailureLatched] = input.failure_latched ? 1.0 : 0.0;
  v[kAuditCurrentW] = input.current_w; v[kAuditSampleStepW] = input.sample_step_w;
  v[kAuditLateralProbeHalfWidth] = input.lateral_probe_half_width; v[kAuditRequiredReferenceClearance] = input.required_reference_clearance;
  v[kAuditPlannerSafeDistance] = input.planner_safe_distance; v[kAuditConfiguredSearchMargin] = input.configured_search_margin;
  v[kAuditProfileComplete] = input.profile_complete ? 1.0 : 0.0; v[kAuditObstacleCertified] = input.obstacle_certified ? 1.0 : 0.0;
  v[kAuditTubeFirstInvalidW] = input.tube_first_invalid_w; v[kAuditTubeFirstStopReason] = input.tube_first_stop_reason;
  v[kAuditTubeInsufficientClearanceCount] = input.tube_insufficient_clearance_count;

  const phase_offset_navigation::DistanceQuery planner_query =
      input.planner_distance_query ? input.planner_distance_query
                                   : input.distance_query;
  const phase_offset_navigation::DistanceQuery probe_query =
      input.distance_query ? input.distance_query : planner_query;
  const bool input_ok = input.path && planner_query &&
      IsFinite(input.sample_step_w) && input.sample_step_w > 0.0 &&
      IsFinite(input.lateral_probe_half_width) &&
      input.lateral_probe_half_width >= 0.0 &&
      IsFinite(input.required_reference_clearance) &&
      input.required_reference_clearance > 0.0 &&
      IsFinite(input.planner_safe_distance) &&
      input.planner_safe_distance >= 0.0 &&
      IsFinite(input.configured_search_margin) &&
      input.configured_search_margin >= 0.0 &&
      IsFinite(input.back_w) && IsFinite(input.lookahead_w);
  if (!input_ok) return result;

  const double domain_start = std::max(input.path->startW(), input.current_w - input.back_w);
  const double domain_end = std::min(input.path->endW(), input.current_w + input.lookahead_w);
  if (domain_end <= domain_start + kSegmentEps) return result;

  const std::vector<double> sample_w = BuildAuditSampleW(
      input, domain_start, domain_end);
  v[kAuditPreviewStartW] = domain_start;
  v[kAuditPreviewEndW] = domain_end;
  v[kAuditSampleCount] = static_cast<double>(sample_w.size());
  v[kAuditCurrentSegmentCode] = SegmentCodeForW(*input.path, input.current_w);

  phase_offset_core::GeometryEvaluator evaluator;
  FailureRecord failure;
  bool min_base_valid = false;
  double min_base_distance = 0.0;
  double min_base_w = 0.0;
  Eigen::Vector3d min_base_point = Eigen::Vector3d::Zero();
  int min_base_segment = 0;
  bool positive_min_valid = false;
  double positive_min_distance = 0.0;
  double positive_min_w = 0.0;
  bool negative_min_valid = false;
  double negative_min_distance = 0.0;
  double negative_min_w = 0.0;
  std::size_t queried_base_count = 0U;
  // Z1 production geometry has one clearance contract: planner_safe_distance.
  // The historical robust-margin fields remain observable diagnostics only;
  // they no longer participate in the audit's production-consistency verdict.
  const double configured_full_radius = input.margins.fullEffectiveRadius();
  const double configured_residual_radius = input.planner_safe_distance;
  const double continuous_inset = input.continuous_inset >= 0.0
      ? input.continuous_inset
      : std::max(0.0, input.snapshot_resolution);
  const double recorded_cover_radius = input.validator_cover_radius >= 0.0
      ? input.validator_cover_radius
      : (input.surface_validation != nullptr
             ? input.surface_validation->max_cover_radius : 0.0);
  bool accounting_complete = !sample_w.empty();
  bool raw_zero_all = !sample_w.empty();
  bool filter_zero_all = !sample_w.empty();
  bool validator_zero_all = !sample_w.empty();
  bool map_alignment_all = !sample_w.empty();
  bool path_profile_match_all = !sample_w.empty();
  bool inset_zero_exclusion_observed = false;
  bool filter_zero_exclusion_observed = false;
  bool validator_zero_exclusion_observed = false;

  for (const double w : sample_w) {
    const int segment_code = SegmentCodeForW(*input.path, w);
    ContinuousPhasePathState state;
    if (!input.path->evaluate(w, state, false)) {
      ++v[kAuditGeometryInvalidCount];
      accounting_complete = false;
      raw_zero_all = false;
      filter_zero_all = false;
      validator_zero_all = false;
      map_alignment_all = false;
      path_profile_match_all = false;
      RecordFailure(failure,
          static_cast<double>(static_cast<int>(
              phase_offset_navigation::TubeStopReason::INVALID_PATH)),
          false, 0.0, false, 0.0, w, Eigen::Vector3d::Zero(), segment_code);
      continue;
    }
    phase_offset_core::PathDifferentialState path_state;
    path_state.p = state.p; path_state.p_w = state.dp_dw; path_state.p_ww = state.d2p_dw2;
    path_state.w = w; path_state.valid = state.valid;
    phase_offset_core::PhaseOffsetGeometryState geometry;
    if (!evaluator.evaluate(path_state, state.p, 0.0, geometry)) {
      ++v[kAuditGeometryInvalidCount];
      accounting_complete = false;
      raw_zero_all = false;
      filter_zero_all = false;
      validator_zero_all = false;
      map_alignment_all = false;
      path_profile_match_all = false;
      RecordFailure(failure,
          static_cast<double>(static_cast<int>(
              phase_offset_navigation::TubeStopReason::INVALID_PATH)),
          false, 0.0, false, 0.0, w, state.p, segment_code);
      continue;
    }

    const auto base = planner_query(state.p);
    ++queried_base_count;
    using phase_offset_navigation::DistanceStatus;
    if (base.status == DistanceStatus::KNOWN_FREE &&
        std::isfinite(base.signed_distance)) {
      ++v[kAuditBaseKnownFreeCount];
      const double d = base.signed_distance;
      if (d < input.planner_safe_distance) {
        ++v[kAuditCountDltPlannerSafe];
      } else if (d < input.configured_search_margin) {
        ++v[kAuditCountPlannerSafeLeDltSearchMargin];
      } else if (d < input.required_reference_clearance) {
        ++v[kAuditCountSearchMarginLeDltRequired];
      } else {
        ++v[kAuditCountDgeRequired];
      }
      if (!min_base_valid || d < min_base_distance) {
        min_base_valid = true;
        min_base_distance = d;
        min_base_w = w;
        min_base_point = state.p;
        min_base_segment = segment_code;
      }
      // The planner owns production centreline acceptance.  Keep the legacy
      // required-reference value in the histogram above, but do not let it
      // manufacture a second Tube-side failure threshold.
      if (d < input.planner_safe_distance) {
        RecordFailure(failure,
            static_cast<double>(static_cast<int>(
                phase_offset_navigation::TubeStopReason::INSUFFICIENT_CLEARANCE)),
            true, d, true, input.planner_safe_distance - d,
            w, state.p, segment_code);
      }
    } else {
      const int reason = ReasonForStatus(base.status);
      if (base.status == DistanceStatus::UNAVAILABLE) ++v[kAuditBaseUnavailableCount];
      else if (base.status == DistanceStatus::OUT_OF_MAP) ++v[kAuditBaseOutOfMapCount];
      else if (base.status == DistanceStatus::UNKNOWN) ++v[kAuditBaseUnknownCount];
      else ++v[kAuditBaseOccupiedCount];
      RecordFailure(failure, reason, false, 0.0, false, 0.0,
                    w, state.p, segment_code);
    }

    PhaseOffsetClearanceAuditPoint point;
    point.w = w;
    point.point = state.p;
    point.source_revision = input.source_revision;
    point.tube_revision = input.tube_revision;
    point.snapshot_sequence = input.snapshot_sequence;
    point.snapshot_stamp = input.snapshot_stamp;
    point.segment_code = segment_code;
    point.path_valid = true;
    point.planner_clearance = base.signed_distance;
    point.planner_clearance_valid = base.status == DistanceStatus::KNOWN_FREE &&
        IsFinite(base.signed_distance);
    point.uav_radius = input.margins.uav_radius;
    point.map_uncertainty = input.margins.map_uncertainty;
    point.preincluded_map_uncertainty =
        input.margins.preincluded_map_uncertainty;
    point.localization_uncertainty = input.margins.localization_uncertainty;
    point.tracking_error_bound = input.margins.tracking_error_bound;
    point.full_effective_radius = configured_full_radius;
    point.residual_effective_radius = configured_residual_radius;
    point.snapshot_included_map_inflation =
        input.snapshot_included_map_inflation;
    point.snapshot_resolution = input.snapshot_resolution;
    point.continuous_inset = continuous_inset;
    point.validator_cover_radius = recorded_cover_radius;
    const auto* raw_build_sample = FindRawBuildSample(input.tube_profile, w);
    point.profile_sample_found = raw_build_sample != nullptr;
    point.path_profile_match = raw_build_sample != nullptr &&
        IsFinite(raw_build_sample->p.x()) &&
        (raw_build_sample->p - state.p).norm() <= 1e-9;
    if (raw_build_sample != nullptr) {
      point.full_effective_radius = raw_build_sample->full_effective_radius;
      point.preincluded_map_uncertainty =
          raw_build_sample->preincluded_map_uncertainty;
      point.residual_effective_radius =
          raw_build_sample->residual_effective_radius;
      point.pre_inset_lower = raw_build_sample->pre_inset_lower;
      point.pre_inset_upper = raw_build_sample->pre_inset_upper;
      point.raw_lower = raw_build_sample->raw_lower;
      point.raw_upper = raw_build_sample->raw_upper;
      point.filtered_lower = raw_build_sample->raw_lower;
      point.filtered_upper = raw_build_sample->raw_upper;
      point.pre_inset_contains_zero = ContainsZero(point.pre_inset_lower,
                                                    point.pre_inset_upper);
      point.pre_inset_interval_valid = IsFinite(point.pre_inset_lower) &&
          IsFinite(point.pre_inset_upper) &&
          point.pre_inset_lower <= point.pre_inset_upper + kSegmentEps;
      point.pre_inset_cross_section_valid = point.pre_inset_interval_valid &&
          raw_build_sample->cross_section_reason ==
              phase_offset_navigation::TubeCrossSectionReason::NONE;
      point.raw_contains_zero = ContainsZero(point.raw_lower, point.raw_upper);
    }
    const auto* filtered_sample = FindProfileSample(input.tube_profile, w);
    point.filter_sample_found = filtered_sample != nullptr;
    if (filtered_sample != nullptr) {
      point.filtered_lower = filtered_sample->filtered_lower;
      point.filtered_upper = filtered_sample->filtered_upper;
      point.filtered_contains_zero = ContainsZero(point.filtered_lower,
                                                   point.filtered_upper);
    }
    const auto* validator_evidence = FindValidatorEvidence(
        input.tube_profile, input.surface_validation, w);
    point.validator_sample_found = validator_evidence != nullptr;
    point.validator_complete = input.tube_profile != nullptr &&
        input.tube_profile->obstacle_certified &&
        (input.surface_validation == nullptr ||
         input.surface_validation->complete);
    point.validator_contains_zero = point.validator_complete &&
        validator_evidence != nullptr &&
        validator_evidence->filtered_contains_zero &&
        validator_evidence->zero_surface_covered;
    if (raw_build_sample != nullptr) {
      point.continuous_inset = raw_build_sample->continuous_inset;
    }
    if (validator_evidence != nullptr) {
      point.validator_cover_radius = validator_evidence->max_cover_radius;
      point.validator_requested_radius =
          validator_evidence->max_requested_clearance;
    }
    // Compatibility field name retained for the existing audit payload.  In
    // Z1 it records agreement with the shared planner clearance, not a second
    // independent robust-margin subtraction.
    point.residual_matches_full_minus_preincluded =
        IsFinite(point.residual_effective_radius) &&
        std::abs(point.residual_effective_radius -
                 input.planner_safe_distance) <= 1e-9;
    point.snapshot_inflation_covers_preincluded =
        IsFinite(point.snapshot_included_map_inflation) &&
        point.snapshot_included_map_inflation + 1e-9 >=
            point.preincluded_map_uncertainty;
    point.zero_excluded_by_inset = point.pre_inset_contains_zero &&
        !point.raw_contains_zero;
    // A production cross-section is zero-connected by construction.  Keep the
    // legacy diagnostic bit false rather than reintroducing a selected
    // disconnected robust component into the audit.
    point.planner_tube_robust_contract_mismatch = false;
    if (point.validator_requested_radius <= 0.0) {
      point.validator_requested_radius = point.residual_effective_radius +
          point.validator_cover_radius;
    }
    const double raw_request_radius = std::max(0.0,
        input.planner_safe_distance);
    point.raw_clearance_request_radius = raw_request_radius;
    const auto raw = input.raw_clearance_query
        ? input.raw_clearance_query(state.p, raw_request_radius)
        : LegacyAsClearance(input.distance_query, state.p, raw_request_radius);
    point.raw_clearance_status = static_cast<int>(raw.status);
    point.raw_clearance_lower_bound = raw.clearance;
    point.raw_clearance_valid = raw.status == DistanceStatus::KNOWN_FREE &&
        raw.clearance_certified && IsFinite(raw.clearance) &&
        raw.clearance >= raw_request_radius;
    point.centerline_clearance_sufficient = point.raw_clearance_valid;
    point.unknown_or_out_of_map = raw.status == DistanceStatus::UNKNOWN ||
        raw.status == DistanceStatus::OUT_OF_MAP ||
        base.status == DistanceStatus::UNKNOWN ||
        base.status == DistanceStatus::OUT_OF_MAP;
    accounting_complete = accounting_complete && point.profile_sample_found &&
        point.path_profile_match && point.filter_sample_found &&
        point.validator_sample_found && point.raw_clearance_valid &&
        point.residual_matches_full_minus_preincluded &&
        static_cast<bool>(input.raw_clearance_query);
    raw_zero_all = raw_zero_all && point.raw_contains_zero;
    filter_zero_all = filter_zero_all && point.filtered_contains_zero;
    validator_zero_all = validator_zero_all && point.validator_contains_zero;
    inset_zero_exclusion_observed = inset_zero_exclusion_observed ||
        point.zero_excluded_by_inset;
    filter_zero_exclusion_observed = filter_zero_exclusion_observed ||
        (point.raw_contains_zero && !point.filtered_contains_zero);
    validator_zero_exclusion_observed = validator_zero_exclusion_observed ||
        (point.filtered_contains_zero && !point.validator_contains_zero);
    result.planner_tube_robust_contract_mismatch_observed =
        result.planner_tube_robust_contract_mismatch_observed ||
        point.planner_tube_robust_contract_mismatch;
    map_alignment_all = map_alignment_all &&
        point.snapshot_inflation_covers_preincluded;
    path_profile_match_all = path_profile_match_all && point.path_profile_match;
    result.points.push_back(point);

    const Eigen::Vector3d positive = state.p + geometry.N * input.lateral_probe_half_width;
    const Eigen::Vector3d negative = state.p - geometry.N * input.lateral_probe_half_width;
    RecordProbe(probe_query(positive),
                v[kAuditPositiveProbeKnownFreeCount],
                v[kAuditPositiveProbeInvalidCount],
                positive_min_valid, positive_min_distance, positive_min_w, w);
    RecordProbe(probe_query(negative),
                v[kAuditNegativeProbeKnownFreeCount],
                v[kAuditNegativeProbeInvalidCount],
                negative_min_valid, negative_min_distance, negative_min_w, w);
  }

  v[kAuditMinBaseDistanceValid] = min_base_valid ? 1.0 : 0.0; v[kAuditMinBaseDistance] = min_base_distance;
  v[kAuditMinBaseW] = min_base_w; v[kAuditMinBaseX] = min_base_point.x();
  v[kAuditMinBaseY] = min_base_point.y(); v[kAuditMinBaseZ] = min_base_point.z();
  v[kAuditMinBaseSegmentCode] = min_base_segment;

  v[kAuditFirstContractFailurePresent] = failure.present ? 1.0 : 0.0; v[kAuditFirstContractFailureReason] = failure.reason;
  v[kAuditFirstContractFailureDistanceValid] = failure.distance_valid ? 1.0 : 0.0; v[kAuditFirstContractFailureDistance] = failure.distance;
  v[kAuditFirstContractFailureDeficitValid] = failure.deficit_valid ? 1.0 : 0.0; v[kAuditFirstContractFailureDeficit] = failure.deficit;
  v[kAuditFirstContractFailureW] = failure.w; v[kAuditFirstContractFailureX] = failure.point.x();
  v[kAuditFirstContractFailureY] = failure.point.y(); v[kAuditFirstContractFailureZ] = failure.point.z();
  v[kAuditFirstContractFailureSegmentCode] = failure.segment_code;

  v[kAuditPositiveProbeMinDistanceValid] = positive_min_valid ? 1.0 : 0.0; v[kAuditPositiveProbeMinDistance] = positive_min_distance;
  v[kAuditPositiveProbeMinW] = positive_min_w; v[kAuditNegativeProbeMinDistanceValid] = negative_min_valid ? 1.0 : 0.0;
  v[kAuditNegativeProbeMinDistance] = negative_min_distance; v[kAuditNegativeProbeMinW] = negative_min_w;
  if (queried_base_count > 0U) v[kAuditValid] = 1.0;

  const bool immutable_provenance_complete = input.tube_profile != nullptr &&
      input.raw_clearance_query && input.snapshot_sequence != 0U &&
      input.tube_profile->source_revision == input.source_revision &&
      input.tube_profile->tube_revision == input.tube_revision &&
      input.tube_profile->snapshot_provenance_is_immutable &&
      input.tube_profile->snapshot_sequence == input.snapshot_sequence &&
      IsFinite(input.tube_profile->snapshot_resolution) &&
      IsFinite(input.snapshot_resolution) &&
      std::abs(input.tube_profile->snapshot_resolution -
               input.snapshot_resolution) <= 1e-9;
  result.margin_accounting_complete = accounting_complete &&
      immutable_provenance_complete;
  result.same_path_snapshot = result.margin_accounting_complete &&
      path_profile_match_all;
  result.zero_contained_raw_all_points = raw_zero_all;
  result.zero_contained_filter_all_points = filter_zero_all;
  result.zero_contained_validator_all_points = validator_zero_all;
  result.map_inflation_alignment_evidence = map_alignment_all;
  // Snapshot voxel inflation is not automatically equivalent to the configured
  // map-uncertainty bound.  This becomes a proven duplicate only when that
  // provenance relationship has been supplied explicitly by the caller.
  result.duplicate_margin_proven = input.snapshot_inflation_is_map_uncertainty &&
      input.snapshot_included_map_inflation >
          input.margins.preincluded_map_uncertainty + 1e-9 &&
      input.margins.map_uncertainty > 0.0;
  // T1 classification is deliberately precedence-ordered.  A pre-inset
  // exclusion is Branch B and stays fail-closed even if a later stage happens
  // to contain zero.  In particular, it must not be relabelled as a Filter or
  // Validator regression.  Branch A remains evidence for a future, fully
  // proved inset/cover correction; this audit never changes the profile.
  if (result.planner_tube_robust_contract_mismatch_observed) {
    result.disposition =
        PhaseOffsetClearanceDisposition::PLANNER_TUBE_ROBUST_CONTRACT_MISMATCH;
  } else if (inset_zero_exclusion_observed) {
    result.disposition =
        PhaseOffsetClearanceDisposition::INSET_ZERO_EXCLUSION_REQUIRES_PROOF;
  } else if (filter_zero_exclusion_observed) {
    result.disposition = PhaseOffsetClearanceDisposition::FILTER_ZERO_EXCLUSION;
  } else if (validator_zero_exclusion_observed) {
    result.disposition =
        PhaseOffsetClearanceDisposition::VALIDATOR_ZERO_EXCLUSION;
  } else if (result.margin_accounting_complete &&
             result.zero_contained_raw_all_points &&
             result.zero_contained_filter_all_points &&
             result.zero_contained_validator_all_points) {
    result.disposition =
        PhaseOffsetClearanceDisposition::ZERO_CENTERLINE_CERTIFIED;
  }

  for (double& value : v) {
    if (!std::isfinite(value)) value = 0.0;
  }
  return result;
}

}  // namespace FLAG_Race
