#include "bspline_race/integration/phase_offset_raw_candidate_diagnostics.h"

#include <cmath>
#include <limits>

namespace FLAG_Race {
namespace {

constexpr double kCurrentPhaseMatchTolerance = 1e-9;
constexpr double kMissingEnumOrStatus = -1.0;
constexpr double kNormalEpsilon = 1e-12;

bool IsFinite(const double value) { return std::isfinite(value); }

bool IsFinite(const Eigen::Vector3d& value) { return value.allFinite(); }

double FiniteOrZero(const double value) { return IsFinite(value) ? value : 0.0; }

double AsDouble(const bool value) { return value ? 1.0 : 0.0; }

double RawStatusOrMissing(
    const phase_offset_navigation::RawOccupancyQuery& query,
    const Eigen::Vector3d& point) {
  if (!query || !IsFinite(point)) return kMissingEnumOrStatus;
  return static_cast<double>(query(point));
}

const phase_offset_navigation::TubeRawSample* FindCurrentSample(
    const phase_offset_navigation::TubeProfile* profile,
    const double current_w) {
  if (profile == nullptr || !IsFinite(current_w)) return nullptr;
  const phase_offset_navigation::TubeRawSample* match = nullptr;
  double match_error = std::numeric_limits<double>::infinity();
  for (const phase_offset_navigation::TubeRawSample& sample : profile->samples) {
    if (!IsFinite(sample.w)) continue;
    const double error = std::abs(sample.w - current_w);
    if (error <= kCurrentPhaseMatchTolerance && error < match_error) {
      match = &sample;
      match_error = error;
    }
  }
  return match;
}

}  // namespace

const std::array<const char*, kRawCandidateDiagnosticCount>&
rawCandidateDiagnosticFieldNames() {
  static const std::array<const char*, kRawCandidateDiagnosticCount> names = {{
      "schema_version", "raw_source_configured", "raw_storage_ready",
      "raw_query_injected", "tube_update_due", "candidate_sequence",
      "epoch_state", "epoch_reason", "candidate_raw_complete",
      "candidate_filtered_complete", "candidate_complete",
      "candidate_sample_count", "candidate_invalid_count",
      "candidate_unavailable_count", "candidate_out_of_map_count",
      "candidate_unknown_count", "candidate_occupied_count",
      "candidate_insufficient_clearance_count", "requested_preview_start_w",
      "requested_preview_end_w", "certified_segment_start_w",
      "certified_segment_end_w", "certified_segment_truncated_before",
      "certified_segment_truncated_after", "first_truncated_w",
      "first_truncated_reason", "current_w", "current_sample_found",
      "current_sample_complete", "current_cross_section_reason",
      "current_positive_ray_termination",
      "current_negative_ray_termination", "current_base_raw_status",
      "actual_position_raw_status", "current_plus_step_raw_status",
      "current_minus_step_raw_status", "current_c_plus_raw",
      "current_c_minus_raw", "current_effective_radius",
      "current_obstacle_lower", "current_obstacle_upper",
      "current_curvature_lower", "current_curvature_upper",
      "current_environment_lower", "current_environment_upper",
      "current_environment_width", "current_contains_zero",
      "deprecated_current_contains_preferred_delta",
      "current_base_to_actual_norm"}};
  return names;
}

std::array<double, kRawCandidateDiagnosticCount> makeRawCandidateDiagnostics(
    const RawCandidateDiagnosticsInput& input) {
  std::array<double, kRawCandidateDiagnosticCount> values = {{0.0}};
  const phase_offset_navigation::TubeProfile* profile = input.candidate_profile;
  const phase_offset_navigation::TubeRawSample* current = FindCurrentSample(
      profile, input.current_w);
  const auto& epoch = input.epoch_status;

  values[kRawCandidateSchemaVersion] = 1.0;
  values[kRawCandidateRawSourceConfigured] = AsDouble(input.raw_source_configured);
  values[kRawCandidateRawStorageReady] = AsDouble(input.raw_storage_ready);
  values[kRawCandidateRawQueryInjected] = AsDouble(input.raw_query_injected);
  values[kRawCandidateTubeUpdateDue] = AsDouble(input.tube_update_due);
  values[kRawCandidateSequence] = static_cast<double>(epoch.candidate_sequence);
  values[kRawCandidateEpochState] = static_cast<double>(epoch.state);
  values[kRawCandidateEpochReason] = static_cast<double>(epoch.reason);
  values[kRawCandidateProfileRawComplete] = AsDouble(epoch.candidate_raw_complete);
  values[kRawCandidateProfileFilteredComplete] =
      AsDouble(epoch.candidate_filtered_complete);
  values[kRawCandidateProfileComplete] = AsDouble(epoch.candidate_complete);
  if (profile != nullptr) {
    const auto& diagnostics = profile->diagnostics;
    values[kRawCandidateSampleCount] = static_cast<double>(profile->samples.size());
    values[kRawCandidateInvalidCount] = static_cast<double>(diagnostics.invalid_count);
    values[kRawCandidateUnavailableCount] =
        static_cast<double>(diagnostics.unavailable_count);
    values[kRawCandidateOutOfMapCount] =
        static_cast<double>(diagnostics.out_of_map_count);
    values[kRawCandidateUnknownCount] = static_cast<double>(diagnostics.unknown_count);
    values[kRawCandidateOccupiedCount] =
        static_cast<double>(diagnostics.occupied_count);
    values[kRawCandidateInsufficientClearanceCount] =
        static_cast<double>(diagnostics.insufficient_clearance_count);
    values[kRawCandidateRequestedPreviewStartW] = profile->requested_preview_start_w;
    values[kRawCandidateRequestedPreviewEndW] = profile->requested_preview_end_w;
    values[kRawCandidateCertifiedSegmentStartW] =
        profile->certified_segment_start_w;
    values[kRawCandidateCertifiedSegmentEndW] = profile->certified_segment_end_w;
    values[kRawCandidateCertifiedSegmentTruncatedBefore] =
        AsDouble(profile->certified_segment_truncated_before);
    values[kRawCandidateCertifiedSegmentTruncatedAfter] =
        AsDouble(profile->certified_segment_truncated_after);
    values[kRawCandidateFirstTruncatedW] = profile->first_truncated_w;
    values[kRawCandidateFirstTruncatedReason] =
        static_cast<double>(profile->first_truncated_reason);
  } else {
    values[kRawCandidateFirstTruncatedReason] = kMissingEnumOrStatus;
  }
  values[kRawCandidateCurrentW] = input.current_w;
  values[kRawCandidateCurrentSampleFound] = AsDouble(current != nullptr);
  values[kRawCandidateCurrentSampleComplete] =
      current == nullptr ? 0.0 : AsDouble(current->complete);
  values[kRawCandidateCurrentCrossSectionReason] = current == nullptr
      ? kMissingEnumOrStatus
      : static_cast<double>(current->cross_section_reason);
  values[kRawCandidateCurrentPositiveRayTermination] = current == nullptr
      ? kMissingEnumOrStatus
      : static_cast<double>(current->positive_ray_termination);
  values[kRawCandidateCurrentNegativeRayTermination] = current == nullptr
      ? kMissingEnumOrStatus
      : static_cast<double>(current->negative_ray_termination);

  const Eigen::Vector3d base = current != nullptr ? current->p : input.current_path.p;
  values[kRawCandidateCurrentBaseRawStatus] = RawStatusOrMissing(
      input.raw_occupancy_query, base);
  values[kRawCandidateActualPositionRawStatus] = RawStatusOrMissing(
      input.raw_occupancy_query, input.actual_position);
  values[kRawCandidateCurrentPlusStepRawStatus] = kMissingEnumOrStatus;
  values[kRawCandidateCurrentMinusStepRawStatus] = kMissingEnumOrStatus;
  if (current != nullptr && IsFinite(base) && IsFinite(current->N) &&
      IsFinite(input.raw_ray_step) && input.raw_ray_step > 0.0) {
    const double normal_norm = current->N.norm();
    if (IsFinite(normal_norm) && normal_norm > kNormalEpsilon) {
      const Eigen::Vector3d normal = current->N / normal_norm;
      values[kRawCandidateCurrentPlusStepRawStatus] = RawStatusOrMissing(
          input.raw_occupancy_query, base + input.raw_ray_step * normal);
      values[kRawCandidateCurrentMinusStepRawStatus] = RawStatusOrMissing(
          input.raw_occupancy_query, base - input.raw_ray_step * normal);
    }
  }

  if (current != nullptr) {
    values[kRawCandidateCurrentCPlusRaw] = current->c_plus_raw;
    values[kRawCandidateCurrentCMinusRaw] = current->c_minus_raw;
    values[kRawCandidateCurrentEffectiveRadius] = current->effective_radius;
    values[kRawCandidateCurrentObstacleLower] = current->obstacle_lower;
    values[kRawCandidateCurrentObstacleUpper] = current->obstacle_upper;
    values[kRawCandidateCurrentCurvatureLower] = current->curvature_lower;
    values[kRawCandidateCurrentCurvatureUpper] = current->curvature_upper;
    values[kRawCandidateCurrentEnvironmentLower] = current->environment_lower;
    values[kRawCandidateCurrentEnvironmentUpper] = current->environment_upper;
    values[kRawCandidateCurrentEnvironmentWidth] = current->environment_width;
    values[kRawCandidateCurrentContainsZero] =
        AsDouble(current->environment_contains_zero);
    // Kept as a stable diagnostic-array slot while consumers migrate.  The
    // zero-connected production Tube no longer accepts a preferred component.
    values[kRawCandidateCurrentContainsPreferredDelta] = 0.0;
  }
  if (IsFinite(base) && IsFinite(input.actual_position)) {
    values[kRawCandidateCurrentBaseToActualNorm] =
        (base - input.actual_position).norm();
  }

  for (double& value : values) value = FiniteOrZero(value);
  return values;
}

}  // namespace FLAG_Race
