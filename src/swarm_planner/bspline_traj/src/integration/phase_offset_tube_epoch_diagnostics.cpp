#include "bspline_race/integration/phase_offset_tube_epoch_diagnostics.h"

#include <cmath>

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
      "runtime_failure_reason"}};
  return names;
}

std::array<double, kTubeEpochDiagnosticCount> makeTubeEpochDiagnostics(
    const TubeEpochDiagnosticsInput& input) {
  std::array<double, kTubeEpochDiagnosticCount> values = {{0.0}};
  const auto& epoch = input.epoch;
  const auto* candidate = input.candidate_profile;
  const auto* active = input.active_profile;
  values[kEpochSchemaVersion] = 3.0;
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
  for (double& value : values) value = FiniteOrZero(value);
  return values;
}

}  // namespace FLAG_Race
