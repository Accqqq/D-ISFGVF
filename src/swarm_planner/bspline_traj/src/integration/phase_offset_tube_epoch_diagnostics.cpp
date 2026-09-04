#include "bspline_race/integration/phase_offset_tube_epoch_diagnostics.h"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
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
  stream << " schema=2";
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

  // Schema-2 observed-domain operands are diagnostic-only.  The witness is
  // copied from the exact validator EvaluateSurfacePoint invocation, while
  // the snapshot is a synchronous non-owning view of the request's immutable
  // cloud snapshot.  No map/query helper is called here.
  const bool exact_witness_valid = valid && evidence->witness_valid &&
      std::isfinite(evidence->center_w) && std::isfinite(evidence->center_v) &&
      std::isfinite(evidence->witness_x) &&
      std::isfinite(evidence->witness_y) &&
      std::isfinite(evidence->witness_z);
  stream << " witness_valid=" << bool_value(exact_witness_valid);
  stream << " center_w=" << (exact_witness_valid
      ? finite_or_zero(evidence->center_w) : 0.0);
  stream << " center_v=" << (exact_witness_valid
      ? finite_or_zero(evidence->center_v) : 0.0);
  stream << " witness_x=" << (exact_witness_valid
      ? finite_or_zero(evidence->witness_x) : 0.0);
  stream << " witness_y=" << (exact_witness_valid
      ? finite_or_zero(evidence->witness_y) : 0.0);
  stream << " witness_z=" << (exact_witness_valid
      ? finite_or_zero(evidence->witness_z) : 0.0);

  const bool snapshot_available = input.cloud_status.snapshot_available;
  const bool snapshot_valid = input.cloud_status.snapshot_valid;
  const bool snapshot_usable = input.cloud_status.usable;
  const bool request_sequence_valid = input.map_observation_sequence != 0U;
  const bool snapshot_sequence_present = input.snapshot != nullptr &&
      input.snapshot->observation_sequence != 0U;
  const bool status_sequence_valid =
      input.cloud_status.observation_sequence != 0U;
  const bool sidecar_sequence_valid = valid &&
      evidence->map_observation_sequence_valid &&
      evidence->map_observation_sequence != 0U;
  const bool snapshot_observation_sequence_valid = sidecar_sequence_valid &&
      request_sequence_valid && snapshot_sequence_present &&
      status_sequence_valid;
  const bool snapshot_identity_match = snapshot_observation_sequence_valid &&
      evidence->map_observation_sequence == input.map_observation_sequence &&
      input.snapshot->observation_sequence == input.map_observation_sequence &&
      input.cloud_status.observation_sequence == input.map_observation_sequence;
  stream << " snapshot_available=" << bool_value(snapshot_available);
  stream << " snapshot_valid=" << bool_value(snapshot_valid);
  stream << " snapshot_usable=" << bool_value(snapshot_usable);
  stream << " snapshot_observation_sequence_valid=" <<
      bool_value(snapshot_observation_sequence_valid);
  stream << " snapshot_observation_sequence=" <<
      (snapshot_observation_sequence_valid
          ? input.snapshot->observation_sequence : 0U);
  stream << " snapshot_identity_match=" <<
      bool_value(snapshot_identity_match);

  const bool snapshot_domain_valid = snapshot_available && snapshot_valid &&
      snapshot_usable && snapshot_identity_match && input.snapshot != nullptr &&
      input.snapshot->valid;
  const bool finite_ordered_bounds = input.snapshot != nullptr &&
      std::isfinite(input.snapshot->observed_min.x()) &&
      std::isfinite(input.snapshot->observed_min.y()) &&
      std::isfinite(input.snapshot->observed_min.z()) &&
      std::isfinite(input.snapshot->observed_max.x()) &&
      std::isfinite(input.snapshot->observed_max.y()) &&
      std::isfinite(input.snapshot->observed_max.z()) &&
      input.snapshot->observed_min.x() <= input.snapshot->observed_max.x() &&
      input.snapshot->observed_min.y() <= input.snapshot->observed_max.y() &&
      input.snapshot->observed_min.z() <= input.snapshot->observed_max.z();
  const bool observed_bounds_valid = snapshot_domain_valid &&
      finite_ordered_bounds;
  stream << " observed_bounds_valid=" << bool_value(observed_bounds_valid);
  stream << " observed_min_x=" << (observed_bounds_valid
      ? finite_or_zero(input.snapshot->observed_min.x()) : 0.0);
  stream << " observed_min_y=" << (observed_bounds_valid
      ? finite_or_zero(input.snapshot->observed_min.y()) : 0.0);
  stream << " observed_min_z=" << (observed_bounds_valid
      ? finite_or_zero(input.snapshot->observed_min.z()) : 0.0);
  stream << " observed_max_x=" << (observed_bounds_valid
      ? finite_or_zero(input.snapshot->observed_max.x()) : 0.0);
  stream << " observed_max_y=" << (observed_bounds_valid
      ? finite_or_zero(input.snapshot->observed_max.y()) : 0.0);
  stream << " observed_max_z=" << (observed_bounds_valid
      ? finite_or_zero(input.snapshot->observed_max.z()) : 0.0);

  const bool requested_radius_valid = valid &&
      evidence->requested_clearance_valid &&
      std::isfinite(evidence->requested_clearance) &&
      evidence->requested_clearance >= 0.0;
  const double requested_radius = requested_radius_valid
      ? evidence->requested_clearance : 0.0;
  const bool base_radius_valid = exact_witness_valid &&
      snapshot_domain_valid && observed_bounds_valid &&
      requested_radius_valid;
  stream << " requested_radius_valid=" << bool_value(requested_radius_valid);
  stream << " requested_radius=" << finite_or_zero(requested_radius);
  stream << " base_radius_valid=" << bool_value(base_radius_valid);
  stream << " base_radius=" << (base_radius_valid ? 0.4 : 0.0);

  const bool domain_predicates_valid = exact_witness_valid &&
      snapshot_domain_valid && observed_bounds_valid &&
      requested_radius_valid;
  bool point_in_observed_box = false;
  bool requested_ball_in_observed_box = false;
  bool base_ball_in_observed_box = false;
  if (domain_predicates_valid) {
    const Eigen::Vector3d witness(evidence->witness_x, evidence->witness_y,
                                  evidence->witness_z);
    const Eigen::Vector3d& observed_min = input.snapshot->observed_min;
    const Eigen::Vector3d& observed_max = input.snapshot->observed_max;
    point_in_observed_box = witness.x() >= observed_min.x() &&
        witness.x() <= observed_max.x() &&
        witness.y() >= observed_min.y() &&
        witness.y() <= observed_max.y() &&
        witness.z() >= observed_min.z() &&
        witness.z() <= observed_max.z();
    requested_ball_in_observed_box =
        witness.x() - requested_radius >= observed_min.x() &&
        witness.x() + requested_radius <= observed_max.x() &&
        witness.y() - requested_radius >= observed_min.y() &&
        witness.y() + requested_radius <= observed_max.y() &&
        witness.z() - requested_radius >= observed_min.z() &&
        witness.z() + requested_radius <= observed_max.z();
    constexpr double kBaseRadius = 0.4;
    base_ball_in_observed_box = witness.x() - kBaseRadius >= observed_min.x() &&
        witness.x() + kBaseRadius <= observed_max.x() &&
        witness.y() - kBaseRadius >= observed_min.y() &&
        witness.y() + kBaseRadius <= observed_max.y() &&
        witness.z() - kBaseRadius >= observed_min.z() &&
        witness.z() + kBaseRadius <= observed_max.z();
  }
  stream << " domain_predicates_valid=" << bool_value(domain_predicates_valid);
  stream << " point_in_observed_box=" << bool_value(point_in_observed_box);
  stream << " requested_ball_in_observed_box=" <<
      bool_value(requested_ball_in_observed_box);
  stream << " base_0p4_ball_in_observed_box=" <<
      bool_value(base_ball_in_observed_box);

  // Compute each raw signed difference before accepting the margin group.  A
  // finite-input subtraction may still overflow (for example, DBL_MAX -
  // (-DBL_MAX)); one nonfinite result invalidates the whole group so a
  // partially useful set of faces is never serialized as a certificate.
  const bool margin_operands_valid = exact_witness_valid &&
      snapshot_domain_valid && observed_bounds_valid;
  const double unavailable_margin =
      std::numeric_limits<double>::quiet_NaN();
  double raw_x_minus_margin = unavailable_margin;
  double raw_x_plus_margin = unavailable_margin;
  double raw_y_minus_margin = unavailable_margin;
  double raw_y_plus_margin = unavailable_margin;
  double raw_z_minus_margin = unavailable_margin;
  double raw_z_plus_margin = unavailable_margin;
  if (margin_operands_valid) {
    const Eigen::Vector3d& observed_min = input.snapshot->observed_min;
    const Eigen::Vector3d& observed_max = input.snapshot->observed_max;
    raw_x_minus_margin = evidence->witness_x - observed_min.x();
    raw_x_plus_margin = observed_max.x() - evidence->witness_x;
    raw_y_minus_margin = evidence->witness_y - observed_min.y();
    raw_y_plus_margin = observed_max.y() - evidence->witness_y;
    raw_z_minus_margin = evidence->witness_z - observed_min.z();
    raw_z_plus_margin = observed_max.z() - evidence->witness_z;
  }
  const bool raw_margins_finite =
      std::isfinite(raw_x_minus_margin) && std::isfinite(raw_x_plus_margin) &&
      std::isfinite(raw_y_minus_margin) && std::isfinite(raw_y_plus_margin) &&
      std::isfinite(raw_z_minus_margin) && std::isfinite(raw_z_plus_margin);
  const bool margins_valid = raw_margins_finite;
  double x_minus_margin = 0.0;
  double x_plus_margin = 0.0;
  double y_minus_margin = 0.0;
  double y_plus_margin = 0.0;
  double z_minus_margin = 0.0;
  double z_plus_margin = 0.0;
  double min_observed_margin = 0.0;
  if (margins_valid) {
    x_minus_margin = raw_x_minus_margin;
    x_plus_margin = raw_x_plus_margin;
    y_minus_margin = raw_y_minus_margin;
    y_plus_margin = raw_y_plus_margin;
    z_minus_margin = raw_z_minus_margin;
    z_plus_margin = raw_z_plus_margin;
    min_observed_margin = std::min({x_minus_margin, x_plus_margin,
                                    y_minus_margin, y_plus_margin,
                                    z_minus_margin, z_plus_margin});
  }
  stream << " margins_valid=" << bool_value(margins_valid);
  stream << " x_minus_margin=" << (margins_valid ? x_minus_margin : 0.0);
  stream << " x_plus_margin=" << (margins_valid ? x_plus_margin : 0.0);
  stream << " y_minus_margin=" << (margins_valid ? y_minus_margin : 0.0);
  stream << " y_plus_margin=" << (margins_valid ? y_plus_margin : 0.0);
  stream << " z_minus_margin=" << (margins_valid ? z_minus_margin : 0.0);
  stream << " z_plus_margin=" << (margins_valid ? z_plus_margin : 0.0);
  stream << " min_observed_margin=" <<
      (margins_valid ? min_observed_margin : 0.0);

  int limiting_axis = -1;
  int limiting_side = -1;
  if (margins_valid) {
    double limiting_margin = x_minus_margin;
    limiting_axis = 0;
    limiting_side = 0;
    const auto consider_face = [&](const double margin, const int axis,
                                   const int side) {
      if (margin < limiting_margin) {
        limiting_margin = margin;
        limiting_axis = axis;
        limiting_side = side;
      }
    };
    // Stable order is X_MINUS, X_PLUS, Y_MINUS, Y_PLUS, Z_MINUS, Z_PLUS;
    // strict replacement preserves the first face on exact ties.
    consider_face(x_plus_margin, 0, 1);
    consider_face(y_minus_margin, 1, 0);
    consider_face(y_plus_margin, 1, 1);
    consider_face(z_minus_margin, 2, 0);
    consider_face(z_plus_margin, 2, 1);
  }
  const bool limiting_face_valid = margins_valid && limiting_axis >= 0 &&
      limiting_side >= 0;
  stream << " limiting_face_valid=" << bool_value(limiting_face_valid);
  stream << " limiting_axis=" << limiting_axis;
  stream << " limiting_side=" << limiting_side;
  const bool deficit_operands_valid = requested_radius_valid && margins_valid &&
      std::isfinite(requested_radius) && std::isfinite(min_observed_margin);
  double raw_requested_ball_deficit =
      std::numeric_limits<double>::quiet_NaN();
  if (deficit_operands_valid) {
    raw_requested_ball_deficit = requested_radius - min_observed_margin;
  }
  const bool requested_ball_deficit_valid =
      std::isfinite(raw_requested_ball_deficit);
  const double requested_ball_deficit = requested_ball_deficit_valid
      ? std::max(0.0, raw_requested_ball_deficit) : 0.0;
  stream << " requested_ball_deficit_valid=" <<
      bool_value(requested_ball_deficit_valid);
  stream << " requested_ball_deficit=" <<
      finite_or_zero(requested_ball_deficit);
  return stream.str();
}

}  // namespace FLAG_Race
