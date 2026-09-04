#include "bspline_race/integration/phase_offset_tube_epoch_diagnostics.h"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace FLAG_Race {
namespace {

double FiniteOrZero(double value) { return std::isfinite(value) ? value : 0.0; }
double AsDouble(bool value) { return value ? 1.0 : 0.0; }

}  // namespace

const std::array<const char*, kTubeEpochDiagnosticCount>&
tubeEpochDiagnosticFieldNames() {
  static const std::array<const char*, kTubeEpochDiagnosticCount> names = {{
      "schema_version", "tube_source", "epoch_state", "install_disposition",
      "reason", "candidate_sequence", "active_tube_epoch",
      "candidate_path_source_revision", "active_path_source_revision",
      "candidate_map_observation_sequence", "active_map_observation_sequence",
      "map_observation_is_snapshot", "candidate_raw_complete",
      "candidate_filtered_complete", "candidate_complete", "active_available",
      "active_current_validation_valid", "current_geometry_valid",
      "current_bounds_valid", "retained_delta_current_inside",
      "current_state_admissible", "current_safety_status",
      "reference_clearance_sufficient", "actual_clearance_sufficient",
      "tracking_within_bound", "forward_horizon_sufficient",
      "retained_delta", "tracking_error_norm", "tracking_error_bound",
      "reference_signed_distance", "actual_signed_distance",
      "required_reference_clearance", "required_actual_clearance",
      "certified_forward_w", "equivalent_refresh_count", "install_count",
      "reject_count", "wait_count",
      "candidate_sample_count", "active_sample_count", "active_profile_complete",
      "active_obstacle_certified", "active_display_certified",
      "tube_update_due_this_cycle", "runtime_execution_mode", "control_selected",
      "certificate_denied",
      "transient_blocked", "genuine_fatal_invariant",
      "runtime_failure_reason",
      "candidate_surface_outcome",
      "candidate_surface_inconclusive_reason",
      "candidate_surface_truncation_outcome",
      "candidate_surface_terminal_w",
      "candidate_surface_witness_clearance",
      "candidate_surface_witness_clearance_exact",
      "candidate_surface_midpoint_position_cover",
      "candidate_surface_normal_variation_cover",
      "candidate_surface_delta_slope_cover",
      "candidate_surface_v_span_cover",
      "candidate_surface_geometric_cover",
      "candidate_surface_support_alignment_bound",
      "candidate_surface_numerical_epsilon",
      "candidate_surface_proof_residual",
      "candidate_surface_max_depth_observed",
      "candidate_surface_query_sample_count",
      "candidate_surface_depth_guard_reached",
      "candidate_surface_query_budget_reached",
      "candidate_surface_split_w_count",
      "candidate_surface_split_v_count",
      "candidate_surface_split_both_count",
      "candidate_zero_centerline_contiguous",
      "candidate_zero_centerline_start_w",
      "candidate_zero_centerline_end_w"}};
  return names;
}

std::array<double, kTubeEpochDiagnosticCount> makeTubeEpochDiagnostics(
    const TubeEpochDiagnosticsInput& input) {
  std::array<double, kTubeEpochDiagnosticCount> values = {{0.0}};
  const auto& epoch = input.epoch;
  const auto* candidate = input.candidate_profile;
  const auto* active = input.active_profile;
  values[kEpochSchemaVersion] = 4.0;
  values[kEpochTubeSource] = static_cast<int>(input.source);
  values[kEpochState] = static_cast<int>(epoch.state);
  values[kEpochInstallDisposition] = static_cast<int>(epoch.disposition);
  values[kEpochReason] = static_cast<int>(epoch.reason);
  values[kEpochCandidateSequence] = static_cast<double>(epoch.candidate_sequence);
  values[kEpochActiveTubeEpoch] = static_cast<double>(epoch.active_tube_epoch);
  values[kEpochCandidatePathSourceRevision] =
      static_cast<double>(epoch.candidate_path_source_revision);
  values[kEpochActivePathSourceRevision] = static_cast<double>(epoch.active_path_source_revision);
  values[kEpochCandidateMapObservationSequence] =
      static_cast<double>(epoch.candidate_map_observation_sequence);
  values[kEpochActiveMapObservationSequence] =
      static_cast<double>(epoch.active_map_observation_sequence);
  values[kEpochMapObservationIsSnapshot] =
      AsDouble(epoch.map_observation_is_snapshot);
  values[kEpochCandidateRawComplete] = AsDouble(epoch.candidate_raw_complete);
  values[kEpochCandidateFilteredComplete] = AsDouble(epoch.candidate_filtered_complete);
  values[kEpochCandidateComplete] = AsDouble(epoch.candidate_complete);
  values[kEpochActiveAvailable] = AsDouble(epoch.active_available);
  values[kEpochActiveCurrentValidationValid] =
      AsDouble(epoch.active_current_validation_valid);
  values[kEpochCurrentGeometryValid] = AsDouble(input.runtime.current_geometry_valid);
  values[kEpochCurrentBoundsValid] = AsDouble(input.runtime.current_bounds_valid);
  values[kEpochRetainedDeltaCurrentInside] =
      AsDouble(input.runtime.retained_delta_current_inside);
  values[kEpochCurrentStateAdmissible] = AsDouble(epoch.current_state_admissible);
  values[kEpochCurrentSafetyStatus] = static_cast<int>(epoch.current_safety_status);
  values[kEpochReferenceClearanceSufficient] =
      AsDouble(epoch.reference_clearance_sufficient);
  values[kEpochActualClearanceSufficient] = AsDouble(epoch.actual_clearance_sufficient);
  values[kEpochTrackingWithinBound] = AsDouble(input.runtime.tracking_within_bound);
  values[kEpochForwardHorizonSufficient] = AsDouble(epoch.forward_horizon_sufficient);
  values[kEpochRetainedDelta] = input.retained_delta;
  values[kEpochTrackingErrorNorm] = input.runtime.tracking_error_norm;
  values[kEpochTrackingErrorBound] = input.runtime.tracking_error_bound;
  values[kEpochReferenceSignedDistance] = epoch.reference_signed_distance;
  values[kEpochActualSignedDistance] = epoch.actual_signed_distance;
  values[kEpochRequiredReferenceClearance] = epoch.required_reference_clearance;
  values[kEpochRequiredActualClearance] = epoch.required_actual_clearance;
  values[kEpochCertifiedForwardW] = epoch.certified_forward_w;
  values[kEpochEquivalentRefreshCount] = static_cast<double>(epoch.equivalent_refresh_count);
  values[kEpochInstallCount] = static_cast<double>(epoch.install_count);
  values[kEpochRejectCount] = static_cast<double>(epoch.reject_count);
  values[kEpochWaitCount] = static_cast<double>(epoch.wait_count);
  values[kEpochCandidateSampleCount] = candidate == nullptr
      ? 0.0 : static_cast<double>(candidate->samples.size());
  values[kEpochActiveSampleCount] = active == nullptr
      ? 0.0 : static_cast<double>(active->samples.size());
  values[kEpochActiveProfileComplete] = AsDouble(active != nullptr && active->complete);
  values[kEpochActiveObstacleCertified] =
      AsDouble(active != nullptr && active->obstacle_certified);
  values[kEpochActiveDisplayCertified] = AsDouble(input.active_display_certified);
  values[kEpochTubeUpdateDueThisCycle] = AsDouble(input.tube_update_due_this_cycle);
  values[kEpochRuntimeExecutionMode] = static_cast<int>(input.runtime.mode);
  values[kEpochControlSelected] = AsDouble(input.control_selected);
  // This is the active Runtime certificate-denial fact, not a control action;
  // a rejected candidate therefore cannot be mistaken for an actuator command.
  values[kEpochCertificateDenied] = AsDouble(input.runtime.certificate_denied);
  values[kEpochTransientBlocked] = AsDouble(epoch.transient_blocked ||
      input.runtime.transient_blocked);
  values[kEpochGenuineFatalInvariant] = AsDouble(
      epoch.genuine_fatal_invariant || input.runtime.genuine_fatal_invariant ||
      input.failure_latched);
  const auto failure_reason = input.control_failure_reason !=
      phase_offset_navigation::ControlFailureReason::NONE
      ? input.control_failure_reason : input.runtime.failure_reason;
  values[kEpochRuntimeFailureReason] = static_cast<int>(failure_reason);
  const auto* candidate_diagnostics = candidate == nullptr
      ? nullptr : &candidate->diagnostics;
  if (candidate_diagnostics != nullptr &&
      candidate_diagnostics->surface_summary_present) {
    values[kCandidateSurfaceOutcome] = static_cast<int>(
        candidate_diagnostics->surface_outcome);
    values[kCandidateSurfaceInconclusiveReason] = static_cast<int>(
        candidate_diagnostics->surface_inconclusive_reason);
    values[kCandidateSurfaceTruncationOutcome] = static_cast<int>(
        candidate_diagnostics->surface_truncation_outcome);
    values[kCandidateSurfaceTerminalW] =
        candidate_diagnostics->surface_terminal_w;
    values[kCandidateSurfaceWitnessClearance] =
        candidate_diagnostics->surface_witness_clearance;
    values[kCandidateSurfaceWitnessClearanceExact] = AsDouble(
        candidate_diagnostics->surface_witness_clearance_exact);
    values[kCandidateSurfaceMidpointPositionCover] =
        candidate_diagnostics->surface_midpoint_position_cover;
    values[kCandidateSurfaceNormalVariationCover] =
        candidate_diagnostics->surface_normal_variation_cover;
    values[kCandidateSurfaceDeltaSlopeCover] =
        candidate_diagnostics->surface_delta_slope_cover;
    values[kCandidateSurfaceVSpanCover] =
        candidate_diagnostics->surface_v_span_cover;
    values[kCandidateSurfaceGeometricCover] =
        candidate_diagnostics->surface_geometric_cover;
    values[kCandidateSurfaceSupportAlignmentBound] =
        candidate_diagnostics->surface_support_alignment_bound;
    values[kCandidateSurfaceNumericalEpsilon] =
        candidate_diagnostics->surface_numerical_epsilon;
    values[kCandidateSurfaceProofResidual] =
        candidate_diagnostics->surface_proof_residual;
    values[kCandidateSurfaceMaxDepthObserved] = static_cast<double>(
        candidate_diagnostics->surface_max_depth_observed);
    values[kCandidateSurfaceQuerySampleCount] = static_cast<double>(
        candidate_diagnostics->surface_query_sample_count);
    values[kCandidateSurfaceDepthGuardReached] = AsDouble(
        candidate_diagnostics->surface_depth_guard_reached);
    values[kCandidateSurfaceQueryBudgetReached] = AsDouble(
        candidate_diagnostics->surface_query_budget_reached);
    values[kCandidateSurfaceSplitWCount] = static_cast<double>(
        candidate_diagnostics->surface_split_w_count);
    values[kCandidateSurfaceSplitVCount] = static_cast<double>(
        candidate_diagnostics->surface_split_v_count);
    values[kCandidateSurfaceSplitBothCount] = static_cast<double>(
        candidate_diagnostics->surface_split_both_count);
    values[kCandidateZeroCenterlineContiguous] = AsDouble(
        candidate_diagnostics->zero_centerline_contiguous);
    values[kCandidateZeroCenterlineStartW] =
        candidate_diagnostics->zero_centerline_start_w;
    values[kCandidateZeroCenterlineEndW] =
        candidate_diagnostics->zero_centerline_end_w;
  }
  for (double& value : values) value = FiniteOrZero(value);
  return values;
}

std::string formatTubeSurfaceForwardExcludedLog(
    const TubeSurfaceForwardExcludedLogInput& input) {
  const auto* evidence = input.evidence;
  const bool valid = evidence != nullptr && evidence->valid;
  const auto finite_or_zero = [](const double value) {
    return std::isfinite(value) ? value : 0.0;
  };
  const auto bool_value = [](const bool value) { return value ? 1 : 0; };
  const auto enum_value = [](const int value) { return value; };

  std::ostringstream stream;
  stream << std::setprecision(17) << std::defaultfloat;
  stream << "[PHASE_OFFSET][TUBE][FORWARD_EXCLUDED]";
  stream << " schema=1";
  stream << " valid=" << bool_value(valid);
  stream << " build_sequence=" << input.build_sequence;
  stream << " candidate_sequence=" << input.candidate_sequence;
  stream << " task_generation=" << input.task_generation;
  stream << " authority_session=" << input.authority_session;
  stream << " source_revision=" << input.source_revision;
  stream << " path_revision=" << input.path_revision;
  stream << " frame_revision=" << input.frame_revision;
  stream << " map_observation_sequence=" << input.map_observation_sequence;
  stream << " current_w=" << finite_or_zero(input.current_w);
  stream << " certified_segment_end_w="
         << finite_or_zero(input.certified_segment_end_w);

  stream << " w0=" << (valid ? finite_or_zero(evidence->w0) : 0.0);
  stream << " w1=" << (valid ? finite_or_zero(evidence->w1) : 0.0);
  stream << " v0=" << (valid ? finite_or_zero(evidence->v0) : 0.0);
  stream << " v1=" << (valid ? finite_or_zero(evidence->v1) : 0.0);
  stream << " depth=" << (valid ? evidence->depth : -1);
  stream << " outcome=" << (valid
      ? enum_value(static_cast<int>(evidence->outcome)) : -1);
  stream << " reason=" << (valid
      ? enum_value(static_cast<int>(evidence->inconclusive_reason)) : -1);

  const bool query_attempted = valid && evidence->clearance_query_attempted;
  stream << " clearance_query_attempted=" << bool_value(query_attempted);
  stream << " clearance_status=" << (query_attempted
      ? enum_value(static_cast<int>(evidence->clearance_status)) : -1);
  const bool witness_valid = valid && evidence->witness_clearance_valid &&
      std::isfinite(evidence->witness_clearance);
  const bool witness_certified = query_attempted &&
      evidence->witness_clearance_certified;
  const bool witness_exact = witness_valid && evidence->witness_clearance_exact;
  stream << " witness_clearance_valid=" << bool_value(witness_valid);
  stream << " witness_clearance_certified=" << bool_value(witness_certified);
  stream << " witness_clearance_exact=" << bool_value(witness_exact);
  stream << " witness_clearance=" << (witness_valid
      ? finite_or_zero(evidence->witness_clearance) : 0.0);
  const bool exact_valid = valid && evidence->exact_d_c_valid &&
      witness_valid && witness_exact && std::isfinite(evidence->exact_d_c);
  stream << " exact_d_c_valid=" << bool_value(exact_valid);
  stream << " exact_d_c=" << (exact_valid
      ? finite_or_zero(evidence->exact_d_c) : 0.0);
  const bool requested_valid = valid && evidence->requested_clearance_valid &&
      std::isfinite(evidence->requested_clearance);
  stream << " requested_clearance_valid=" << bool_value(requested_valid);
  stream << " requested_clearance=" << (requested_valid
      ? finite_or_zero(evidence->requested_clearance) : 0.0);

  stream << " cell_certificate_attempted=" << bool_value(
      valid && evidence->cell_certificate_attempted);
  stream << " cell_certificate_complete=" << bool_value(
      valid && evidence->cell_certificate_complete);
  stream << " cell_certificate_revision_match=" << bool_value(
      valid && evidence->cell_certificate_revision_match);
  stream << " query_budget_exhausted=" << bool_value(
      valid && evidence->query_budget_exhausted);
  stream << " max_depth_reached=" << bool_value(
      valid && evidence->max_depth_reached);
  const bool geometric_valid = valid && evidence->geometric_evidence_valid;
  stream << " geometric_evidence_valid=" << bool_value(geometric_valid);
  stream << " midpoint_position_cover=" << (geometric_valid
      ? finite_or_zero(evidence->midpoint_position_cover) : 0.0);
  stream << " normal_variation_cover=" << (geometric_valid
      ? finite_or_zero(evidence->normal_variation_cover) : 0.0);
  stream << " delta_slope_cover=" << (geometric_valid
      ? finite_or_zero(evidence->delta_slope_cover) : 0.0);
  stream << " v_span_cover=" << (geometric_valid
      ? finite_or_zero(evidence->v_span_cover) : 0.0);
  stream << " geometric_cover=" << (geometric_valid
      ? finite_or_zero(evidence->geometric_cover) : 0.0);
  stream << " support_alignment_bound=" << (geometric_valid
      ? finite_or_zero(evidence->support_alignment_bound) : 0.0);
  stream << " numerical_epsilon=" << (geometric_valid
      ? finite_or_zero(evidence->numerical_epsilon) : 0.0);
  stream << " allowable_cover=" << (geometric_valid
      ? finite_or_zero(evidence->allowable_cover) : 0.0);
  stream << " proof_residual=" << (geometric_valid
      ? finite_or_zero(evidence->proof_residual) : 0.0);
  return stream.str();
}

}  // namespace FLAG_Race
