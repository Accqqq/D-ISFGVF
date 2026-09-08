#include "bspline_race/integration/phase_offset_matched_adapter.h"
#include <bspline_race/continuous_phase_normal_frame.h>
#include <bspline_race/integration/phase_offset_executed_reference_query.h>
#include <bspline_race/integration/phase_offset_tube_markers.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <utility>

namespace FLAG_Race {

const char* phaseOffsetCoordinationBackendName(
    const PhaseOffsetCoordinationBackend backend) {
  switch (backend) {
    case PhaseOffsetCoordinationBackend::DISABLED:
      return "disabled";
    case PhaseOffsetCoordinationBackend::D1B:
      return "d1b";
    case PhaseOffsetCoordinationBackend::SPH:
      return "sph";
  }
  return "disabled";
}

namespace {
constexpr double kRecoveryNonzeroDeltaTolerance = 1e-3;

bool IsFinite(double value) { return std::isfinite(value); }
bool IsFinite(const Eigen::Vector3d& value) { return value.allFinite(); }

// An accepted-map refresh changes the state/acceptance portion of a capture
// key while retaining the same map backing, frame, support geometry and
// configuration.  Such a refresh must not blindly retire an otherwise useful
// incumbent.  Changes to any authority-bearing backing/support field are
// incompatible and fail closed; the next coherent profile must then replace
// the old one before it can be used.  Compatibility only permits bounded use
// of the old profile under its own applicability deadline; it never claims
// that the old profile proves the incoming map contents.
bool V2MapRefreshAuthorityCompatible(
    const phase_offset_navigation::TubeMapCaptureKey& incumbent,
    const phase_offset_navigation::TubeMapCaptureKey& incoming) {
  // A refresh is an accepted-state advance, not a relabel of an older
  // capture.  Do not allow any state/time component to move backwards, and
  // require at least one accepted-state component to advance.  Request-ID
  // monotonicity is enforced by PhaseOffsetTubeWorkerV2 separately.
  if (incoming.state_id < incumbent.state_id ||
      incoming.accepted_sequence < incumbent.accepted_sequence ||
      incoming.accepted_time_ticks < incumbent.accepted_time_ticks ||
      (incoming.state_id == incumbent.state_id &&
       incoming.accepted_sequence == incumbent.accepted_sequence &&
       incoming.accepted_time_ticks == incumbent.accepted_time_ticks)) {
    return false;
  }
  if (!incoming.complete_support) return false;
  // Reuse the key's complete equality for every authority-bearing backing
  // field.  Only accepted-state identity/time and its support-expiry bound
  // are allowed to advance during a compatible refresh; normalizing those
  // fields makes the policy explicit without a parallel field checklist.
  phase_offset_navigation::TubeMapCaptureKey normalized = incoming;
  normalized.state_id = incumbent.state_id;
  normalized.accepted_sequence = incumbent.accepted_sequence;
  normalized.accepted_time_ticks = incumbent.accepted_time_ticks;
  normalized.support_expiry_ticks = incumbent.support_expiry_ticks;
  return normalized == incumbent;
}

phase_offset_navigation::RuntimeV2PrepareInput MakeV2RuntimePrepareInput(
    const MatchedAdapterInput& input,
    const std::shared_ptr<const phase_offset_navigation::TubeProfileV2>& profile,
    const MatchedAdapterInput::TubeV2AdmissionEvidence& evidence,
    const bool use_profile_deadline,
    const double retained_delta,
    const phase_offset_core::PortCommand& previous_final_port) {
  phase_offset_navigation::RuntimeV2PrepareInput runtime_input;
  runtime_input.profile = profile;
  runtime_input.identity.execution_generation = profile->path_key.execution_generation;
  runtime_input.identity.path_instance_id = profile->path_key.path_instance_id;
  runtime_input.identity.path_revision = profile->path_key.path_revision;
  runtime_input.identity.frame_revision = profile->path_key.frame_revision;
  runtime_input.identity.frame_convention_id = profile->path_key.frame_convention_id;
  runtime_input.identity.configuration_id = profile->configuration_key.configuration_id;
  runtime_input.identity.map_instance_id = profile->map_capture_key.map_instance_id;
  runtime_input.identity.map_state_id = profile->map_capture_key.state_id;
  runtime_input.identity.accepted_sequence = profile->map_capture_key.accepted_sequence;
  runtime_input.identity.profile_id = profile->profile_id;
  runtime_input.identity.binding_sequence = evidence.binding_sequence;
  runtime_input.current.w = input.path.w;
  runtime_input.current.delta = retained_delta;
  runtime_input.current.previous_u = previous_final_port;
  runtime_input.preview_policy = evidence.preview_policy;
  runtime_input.selected_u = evidence.selected_u;
  runtime_input.selected_u_owner = evidence.selected_u_owner;
  runtime_input.base_phase_rate = evidence.base_phase_rate;
  runtime_input.phase_rate_lower = evidence.phase_rate_lower;
  runtime_input.phase_rate_upper = evidence.phase_rate_upper;
  runtime_input.upper_u_delta = evidence.upper_u_delta;
  runtime_input.horizon_w = profile->certified_end;
  runtime_input.sample_spacing_w = evidence.preview_policy.sample_spacing_w;
  runtime_input.dt = input.dt;
  runtime_input.now = evidence.now;
  runtime_input.applicability_deadline_valid = use_profile_deadline
      ? !profile->applicability_deadline_timeless
      : evidence.applicability_deadline_valid;
  runtime_input.applicability_deadline = use_profile_deadline
      ? (runtime_input.applicability_deadline_valid
          ? static_cast<double>(profile->applicability_deadline_ticks)
          : std::numeric_limits<double>::quiet_NaN())
      : evidence.applicability_deadline;
  runtime_input.tracking = evidence.tracking;
  runtime_input.max_work = evidence.max_work;
  runtime_input.limits = evidence.limits;
  runtime_input.provenance = evidence.provenance;
  return runtime_input;
}

phase_offset_navigation::TubeExecutionIdentityV2 MakeV2ExecutionIdentity(
    const phase_offset_navigation::TubeProfileV2& profile,
    const std::uint64_t binding_sequence) {
  phase_offset_navigation::TubeExecutionIdentityV2 identity;
  identity.execution_generation = profile.path_key.execution_generation;
  identity.path_instance_id = profile.path_key.path_instance_id;
  identity.path_revision = profile.path_key.path_revision;
  identity.frame_revision = profile.path_key.frame_revision;
  identity.frame_convention_id = profile.path_key.frame_convention_id;
  identity.configuration_id = profile.configuration_key.configuration_id;
  identity.map_instance_id = profile.map_capture_key.map_instance_id;
  identity.map_state_id = profile.map_capture_key.state_id;
  identity.accepted_sequence = profile.map_capture_key.accepted_sequence;
  identity.profile_id = profile.profile_id;
  identity.binding_sequence = binding_sequence;
  return identity;
}

bool SameV2ExecutionIdentity(
    const phase_offset_navigation::TubeExecutionIdentityV2& lhs,
    const phase_offset_navigation::TubeExecutionIdentityV2& rhs) {
  return lhs.execution_generation == rhs.execution_generation &&
      lhs.path_instance_id == rhs.path_instance_id &&
      lhs.path_revision == rhs.path_revision &&
      lhs.frame_revision == rhs.frame_revision &&
      lhs.frame_convention_id == rhs.frame_convention_id &&
      lhs.configuration_id == rhs.configuration_id &&
      lhs.map_instance_id == rhs.map_instance_id &&
      lhs.map_state_id == rhs.map_state_id &&
      lhs.accepted_sequence == rhs.accepted_sequence &&
      lhs.profile_id == rhs.profile_id &&
      lhs.binding_sequence == rhs.binding_sequence;
}

std::uint64_t MixPolicyBits(std::uint64_t hash, const double value) {
  std::uint64_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  hash ^= bits;
  hash *= 1099511628211ULL;
  return hash;
}

std::uint64_t NormalPreviewPolicyIdentity(
    const phase_offset_navigation::NormalPreviewProductionPolicy& policy) {
  std::uint64_t hash = 1469598103934665603ULL;
  hash = MixPolicyBits(hash, policy.preview_horizon_w);
  hash = MixPolicyBits(hash, policy.sample_spacing_w);
  hash = MixPolicyBits(hash, policy.lower_nu);
  hash = MixPolicyBits(hash, policy.upper_nu);
  hash = MixPolicyBits(hash, policy.b_tight);
  hash = MixPolicyBits(hash, policy.b_open);
  return hash == 0U ? 1U : hash;
}

bool BitsEqual(const double first, const double second) {
  return std::memcmp(&first, &second, sizeof(double)) == 0;
}

bool BitsEqual(const phase_offset_core::PortCommand& first,
               const phase_offset_core::PortCommand& second) {
  return BitsEqual(first.u_w, second.u_w) &&
      BitsEqual(first.u_delta, second.u_delta);
}

bool BitsEqual(const Eigen::Vector3d& first, const Eigen::Vector3d& second) {
  return BitsEqual(first.x(), second.x()) &&
      BitsEqual(first.y(), second.y()) &&
      BitsEqual(first.z(), second.z());
}

bool HasUsablePathInput(const phase_offset_core::PathDifferentialState& path) {
  return path.valid && IsFinite(path.w) && IsFinite(path.p) &&
      IsFinite(path.p_w) && IsFinite(path.p_ww);
}

bool HasUsableLegacyInput(const LegacyGuidanceSnapshot& legacy) {
  return legacy.valid && IsFinite(legacy.v_cmd) && IsFinite(legacy.w_dot) &&
      IsFinite(legacy.e_parallel) && IsFinite(legacy.e_perp) &&
      IsFinite(legacy.ref_pt) && IsFinite(legacy.tangent);
}

phase_offset_navigation::PhaseOffsetRuntimeConfig MakeRuntimeConfig(const PhaseOffsetMatchedAdapterConfig& config) {
  phase_offset_navigation::PhaseOffsetRuntimeConfig runtime;
  auto& manual = runtime.manual;
  manual.amplitude = config.amplitude; manual.profile_period = config.profile_period;
  manual.delta_tracking_gain = config.delta_tracking_gain; manual.u_w_amplitude = config.u_w_amplitude;
  manual.u_w_abs_max = config.u_w_abs_max; manual.u_delta_abs_max = config.u_delta_abs_max;
  manual.u_w_rate_max = config.u_w_rate_max; manual.u_delta_rate_max = config.u_delta_rate_max;
  manual.phase_dot_min = config.phase_dot_min; manual.tangent_speed_min = config.tangent_speed_min;
  manual.preflight_sample_step_w = config.preflight_sample_step_w;
  runtime.tube.source = config.tube_source; runtime.tube.invariant_gain = config.tube.invariant_gain;
  runtime.tube.interior_margin = config.tube.interior_margin;
  const bool categorical_environment = config.tube_source ==
      phase_offset_navigation::TubeSource::ESDF;
  runtime.tube.tracking_error_bound = categorical_environment
      ? config.tube.cross_section.margins.tracking_error_bound
      : config.tube.erosion.tracking_error_bound;
  runtime.tube.regularity_margin = categorical_environment
      ? config.tube.cross_section.regularity_margin
      : config.tube.regularity_margin;
  runtime.tube.minimum_reference_speed =
      config.tube.cross_section.minimum_reference_speed;
  return runtime;
}

void SetGuidance(const guidance::IsfGuidance& base,
                 const phase_offset_core::MatchedPortOutput& matched,
                 guidance::IsfGuidance& output) {
  output = base; output.v_cmd = matched.v_cmd; output.w_dot = matched.w_dot;
  output.valid = matched.valid; output.invalid_reason = matched.invalid_reason;
}

bool PopulateV2LiveCommandEvidence(
    const PhaseOffsetMatchedAdapterConfig& config,
    const phase_offset_core::PhaseOffsetGeometryState& geometry,
    const guidance::IsfGuidance& base,
    const phase_offset_core::PortCommand& selected_u,
    MatchedAdapterInput::TubeV2AdmissionEvidence& evidence) {
  if (!geometry.valid || !base.valid || !IsFinite(selected_u.u_w) ||
      !IsFinite(selected_u.u_delta)) {
    return false;
  }
  phase_offset_core::MatchedPortInput matched_input;
  matched_input.geometry = geometry;
  matched_input.base_v_cmd = base.v_cmd;
  matched_input.base_w_dot = base.w_dot;
  matched_input.final_port = selected_u;
  phase_offset_core::MatchedPortOutput matched;
  if (!phase_offset_core::MatchedPort::evaluate(matched_input, matched) ||
      !matched.valid) {
    return false;
  }
  const double tangent_speed = geometry.T.dot(matched.v_cmd);
  const phase_offset_navigation::PhaseOffsetRuntimeConfig runtime_config =
      MakeRuntimeConfig(config);
  evidence.selected_u = selected_u;
  evidence.selected_u_owner =
      phase_offset_navigation::PhaseOffsetAllocator::ownerName();
  evidence.base_phase_rate = base.w_dot;
  evidence.phase_rate_lower = config.normal_preview_policy.lower_nu;
  evidence.phase_rate_upper = config.normal_preview_policy.upper_nu;
  evidence.upper_u_delta = std::max(0.0, config.u_delta_abs_max);
  evidence.preview_policy = config.normal_preview_policy;
  evidence.limits.lower_phase_rate = config.normal_preview_policy.lower_nu;
  evidence.limits.upper_phase_rate = config.normal_preview_policy.upper_nu;
  evidence.limits.upper_nu = config.normal_preview_policy.upper_nu;
  evidence.limits.max_u_w = std::max(0.0, config.u_w_abs_max);
  evidence.limits.max_u_delta = std::max(0.0, config.u_delta_abs_max);
  evidence.limits.u_w_slew_rate = std::max(0.0, config.u_w_rate_max);
  evidence.limits.u_delta_slew_rate = std::max(0.0, config.u_delta_rate_max);
  evidence.limits.return_u_delta_max =
      std::max(0.0, config.u_delta_abs_max);
  evidence.limits.return_u_delta_slew_rate =
      std::max(0.0, config.u_delta_rate_max);
  evidence.limits.max_schedule_steps =
      config.tube_certificate_v2.budgets.max_cells;
  evidence.limits.max_work =
      config.tube_certificate_v2.budgets.max_queries;
  evidence.limits.valid = evidence.limits.complete();
  evidence.tracking.error_norm = geometry.error.norm();
  evidence.tracking.error_bound = runtime_config.tube.tracking_error_bound;
  evidence.tracking.physical_tangent_valid = IsFinite(tangent_speed) &&
      tangent_speed >= config.tangent_speed_min;
  evidence.tracking.valid = IsFinite(evidence.tracking.error_norm) &&
      IsFinite(evidence.tracking.error_bound) &&
      evidence.tracking.error_bound >= 0.0 &&
      evidence.tracking.physical_tangent_valid;
  evidence.max_work = config.tube_certificate_v2.budgets.max_queries;
  return evidence.limits.valid && IsFinite(evidence.base_phase_rate);
}

bool RuntimeExecutionExactlyNeutral(
    const phase_offset_navigation::PhaseOffsetRuntime* runtime) {
  if (!runtime || !IsFinite(runtime->retainedDelta()) ||
      runtime->retainedDelta() != 0.0) {
    return false;
  }
  const phase_offset_core::PortCommand previous = runtime->previousFinalPort();
  return IsFinite(previous.u_w) && IsFinite(previous.u_delta) &&
      previous.u_w == 0.0 && previous.u_delta == 0.0;
}

bool PendingV2ApplicabilityEvidenceMatches(
    const MatchedAdapterInput& input,
    const TubeV2ShadowAdmissionCandidate& pending) {
  if (!pending.profile || !pending.commit_token.sealed_prepared) return false;
  const MatchedAdapterInput::TubeV2AdmissionEvidence& evidence =
      input.tube_v2_admission;
  const phase_offset_navigation::RuntimeV2PreparedStep& prepared =
      *pending.commit_token.sealed_prepared;
  const std::uint64_t expected_binding_sequence = prepared.binding_transition
      ? prepared.expected_identity.binding_sequence
      : prepared.identity.binding_sequence;
  if (!evidence.valid || expected_binding_sequence == 0U ||
      evidence.binding_sequence != expected_binding_sequence ||
      !IsFinite(evidence.now) || !IsFinite(input.dt) || input.dt <= 0.0 ||
      !BitsEqual(input.dt, prepared.dt) ||
      evidence.latest_accepted_state_sequence !=
          pending.latest_accepted_state_sequence ||
      evidence.latest_accepted_state_notification_sequence !=
          pending.latest_accepted_state_notification_sequence ||
      evidence.latest_accepted_time_ticks !=
          pending.latest_accepted_time_ticks ||
      evidence.latest_map_instance_id != pending.latest_map_instance_id ||
      evidence.latest_configuration_generation !=
          pending.latest_configuration_generation ||
      evidence.latest_configuration_key != pending.latest_configuration_key ||
      evidence.latest_support_provenance_id !=
          pending.latest_support_provenance_id ||
      evidence.latest_frame_provenance != pending.latest_frame_provenance ||
      evidence.latest_accepted_state_notification_sequence <
          evidence.latest_accepted_state_sequence) {
    return false;
  }
  if (!pending.profile->applicability_deadline_timeless &&
      (evidence.now >= static_cast<double>(
           pending.profile->applicability_deadline_ticks) ||
       evidence.latest_accepted_time_ticks >=
           pending.profile->applicability_deadline_ticks)) {
    return false;
  }
  const phase_offset_navigation::TubeMapCaptureKey& captured =
      pending.profile->map_capture_key;
  if (!captured.support_expiry_timeless &&
      (evidence.now >= static_cast<double>(captured.support_expiry_ticks) ||
       evidence.latest_accepted_time_ticks >= captured.support_expiry_ticks)) {
    return false;
  }
  return true;
}

bool PopulateV2RecoveryOutput(
    const PhaseOffsetMatchedAdapterConfig& config,
    const MatchedAdapterInput& input,
    const TubeV2ShadowAdmissionCandidate& candidate,
    MatchedAdapterOutput& output) {
  const phase_offset_navigation::RecoveryPreparedStep& step =
      candidate.recovery_step;
  if (!step.valid || !step.selectedUExact() ||
      step.proof_kind !=
          phase_offset_navigation::RecoveryStepProofKind::FINITE_RESERVE_V2 ||
      !candidate.commit_token.sealed_prepared ||
      !BitsEqual(input.path.w, step.current_w) ||
      !BitsEqual(input.dt, step.dt)) {
    return false;
  }
  phase_offset_core::GeometryParams geometry_params;
  geometry_params.regularity_margin =
      MakeRuntimeConfig(config).tube.regularity_margin;
  geometry_params.minimum_reference_speed =
      MakeRuntimeConfig(config).tube.minimum_reference_speed;
  phase_offset_core::GeometryEvaluator evaluator(geometry_params);
  phase_offset_core::PhaseOffsetGeometryState geometry;
  if (!evaluator.evaluate(input.path, input.position, step.current_delta,
                          geometry) || !geometry.valid) {
    return false;
  }
  guidance::ReferenceGeometry reference;
  reference.point = geometry.r;
  reference.tangent = geometry.T;
  reference.derivative_norm = geometry.r_w.norm();
  reference.valid = geometry.valid;
  guidance::IsfGuidance base;
  if (!guidance::IsfReferenceKernel::evaluate(
          input.position, reference, input.gains, base) || !base.valid) {
    return false;
  }
  // The reserve was certified from the actual kernel rate captured at
  // admission.  Never overwrite a newly evaluated kernel result to make a
  // stale recurrence appear exact.
  if (!BitsEqual(base.w_dot,
                 candidate.commit_token.sealed_prepared->base_phase_rate)) {
    return false;
  }
  phase_offset_core::MatchedPortInput matched_input;
  matched_input.geometry = geometry;
  matched_input.base_v_cmd = base.v_cmd;
  matched_input.base_w_dot = base.w_dot;
  matched_input.final_port = step.selected_u;
  phase_offset_core::MatchedPortOutput matched;
  if (!phase_offset_core::MatchedPort::evaluate(matched_input, matched) ||
      !matched.valid ||
      !BitsEqual(step.next_w, step.current_w + step.dt * matched.w_dot) ||
      !BitsEqual(step.next_delta,
                 step.current_delta + step.dt * matched.delta_dot)) {
    return false;
  }
  output.base_guidance = base;
  output.geometry = geometry;
  output.raw_port = step.selected_u;
  output.projection.final_port = step.selected_u;
  output.projection.final_w_dot = matched.w_dot;
  output.projection.final_tangent_speed = geometry.T.dot(matched.v_cmd);
  output.projection.next_delta = step.next_delta;
  output.projection.next_regularity = geometry.regularity;
  output.projection.valid = IsFinite(output.projection.final_w_dot) &&
      IsFinite(output.projection.final_tangent_speed) &&
      IsFinite(output.projection.next_delta) &&
      IsFinite(output.projection.next_regularity);
  if (!output.projection.valid) return false;
  output.matched = matched;
  SetGuidance(base, matched, output.guidance);
  output.delta = step.current_delta;
  output.delta_ref = 0.0;
  output.profile_active = false;
  output.selected = true;
  output.valid = true;
  output.recovery_status = phase_offset_navigation::RecoveryStepStatus::PREPARED;
  output.recovery_replan_required = false;
  output.runtime_execution.mode =
      phase_offset_navigation::RuntimeExecutionMode::SAFETY_PRIORITY;
  output.runtime_execution.active_profile_available = true;
  output.runtime_execution.current_geometry_valid = true;
  output.runtime_execution.retained_delta_current_inside = true;
  output.runtime_execution.tracking_within_bound = true;
  output.runtime_execution.executable = true;
  output.invalid_reason.clear();
  return true;
}

bool PopulateV2NormalOutput(
    const PhaseOffsetMatchedAdapterConfig& config,
    const MatchedAdapterInput& input,
    const TubeV2ShadowAdmissionCandidate& candidate,
    MatchedAdapterOutput& output) {
  if (candidate.nonselecting || !candidate.applicable || !candidate.prepared ||
      !candidate.proposed_binding || !candidate.proposed_binding->complete() ||
      !candidate.commit_token.sealed_prepared) {
    return false;
  }
  const phase_offset_navigation::RuntimeV2PreparedStep& step =
      candidate.prepared_step;
  if (!step.valid || !step.expected_current.finite() ||
      !step.successor.finite() ||
      !BitsEqual(input.path.w, step.expected_current.w) ||
      !BitsEqual(input.dt, step.dt)) {
    return false;
  }
  phase_offset_core::GeometryParams geometry_params;
  geometry_params.regularity_margin =
      MakeRuntimeConfig(config).tube.regularity_margin;
  geometry_params.minimum_reference_speed =
      MakeRuntimeConfig(config).tube.minimum_reference_speed;
  phase_offset_core::GeometryEvaluator evaluator(geometry_params);
  phase_offset_core::PhaseOffsetGeometryState geometry;
  if (!evaluator.evaluate(input.path, input.position,
                          step.expected_current.delta, geometry) ||
      !geometry.valid) {
    return false;
  }
  guidance::ReferenceGeometry reference;
  reference.point = geometry.r;
  reference.tangent = geometry.T;
  reference.derivative_norm = geometry.r_w.norm();
  reference.valid = true;
  guidance::IsfGuidance base;
  if (!guidance::IsfReferenceKernel::evaluate(
          input.position, reference, input.gains, base) || !base.valid ||
      !BitsEqual(base.w_dot, step.base_phase_rate)) {
    return false;
  }
  phase_offset_core::MatchedPortInput matched_input;
  matched_input.geometry = geometry;
  matched_input.base_v_cmd = base.v_cmd;
  matched_input.base_w_dot = base.w_dot;
  matched_input.final_port = step.selected_u;
  phase_offset_core::MatchedPortOutput matched;
  if (!phase_offset_core::MatchedPort::evaluate(matched_input, matched) ||
      !matched.valid ||
      !BitsEqual(step.successor.w,
                 step.expected_current.w + step.dt * matched.w_dot) ||
      !BitsEqual(step.successor.delta,
                 step.expected_current.delta + step.dt * matched.delta_dot)) {
    return false;
  }
  output.base_guidance = base;
  output.geometry = geometry;
  output.raw_port = step.selected_u;
  output.projection.final_port = step.selected_u;
  output.projection.final_w_dot = matched.w_dot;
  output.projection.final_tangent_speed = geometry.T.dot(matched.v_cmd);
  output.projection.next_delta = step.successor.delta;
  output.projection.next_regularity = geometry.regularity;
  output.projection.valid = IsFinite(output.projection.final_w_dot) &&
      IsFinite(output.projection.final_tangent_speed) &&
      IsFinite(output.projection.next_delta) &&
      IsFinite(output.projection.next_regularity);
  if (!output.projection.valid) return false;
  output.matched = matched;
  SetGuidance(base, matched, output.guidance);
  output.g_des = candidate.g_des;
  output.g_des_valid = candidate.g_des_valid;
  output.normal_preview = candidate.normal_preview;
  output.allocator = candidate.allocator;
  output.allocator_evaluated = candidate.allocator_evaluated;
  output.delta = step.expected_current.delta;
  output.delta_ref = step.successor.delta;
  output.profile_active = true;
  output.selected = true;
  output.valid = true;
  output.recovery_status = phase_offset_navigation::RecoveryStepStatus::NONE;
  output.recovery_replan_required = false;
  output.runtime_execution.mode =
      phase_offset_navigation::RuntimeExecutionMode::NORMAL;
  output.runtime_execution.active_profile_available = true;
  output.runtime_execution.current_geometry_valid = true;
  output.runtime_execution.retained_delta_current_inside = true;
  output.runtime_execution.tracking_within_bound = true;
  output.runtime_execution.executable = true;
  output.invalid_reason.clear();
  return true;
}

bool PopulateV2SuccessorOutput(
    const PhaseOffsetMatchedAdapterConfig& config,
    const MatchedAdapterInput& input,
    const TubeV2ShadowAdmissionCandidate& candidate,
    MatchedAdapterOutput& output) {
  return candidate.purpose == TubeWorkerPurposeV2::SUCCESSOR &&
      candidate.prepared_step.binding_transition &&
      PopulateV2NormalOutput(config, input, candidate, output);
}
}  // namespace

bool TubeV2ExecutionBinding::complete() const {
  return valid && profile && source_input && frame_owner &&
      identity.complete() &&
      source_input->complete() && profile->valid && profile->complete &&
      profile->path_key == source_input->path_key &&
      profile->configuration_key == source_input->configuration_key &&
      profile->map_capture_key == source_input->map_capture_key &&
      profile->request_id == source_input->request_id &&
      profile->path_owner == source_input->path_owner &&
      profile->capture_owner == source_input->capture_owner &&
      profile->query_owner == source_input->query_owner &&
      frame_owner->pathRevision() == profile->path_key.path_revision &&
      frame_owner->frameRevision() == profile->path_key.frame_revision &&
      frame_owner->startW() == profile->path_key.domain_start &&
      frame_owner->endW() == profile->path_key.domain_end &&
      identity.execution_generation == profile->path_key.execution_generation &&
      identity.path_instance_id == profile->path_key.path_instance_id &&
      identity.path_revision == profile->path_key.path_revision &&
      identity.frame_revision == profile->path_key.frame_revision &&
      identity.frame_convention_id == profile->path_key.frame_convention_id &&
      identity.configuration_id == profile->configuration_key.configuration_id &&
      identity.map_instance_id == profile->map_capture_key.map_instance_id &&
      identity.map_state_id == profile->map_capture_key.state_id &&
      identity.accepted_sequence == profile->map_capture_key.accepted_sequence &&
      identity.profile_id == profile->profile_id;
}

PhaseOffsetMatchedAdapterConfig PhaseOffsetMatchedAdapter::loadConfig(
    ros::NodeHandle& nh, PhaseOffsetMatchedMode mode) {
  PhaseOffsetMatchedAdapterConfig config;
  config.mode = mode;
  nh.param("phase_offset/active_equivalence_tolerance", config.equivalence_tolerance, 1e-10);
  nh.param("phase_offset/active_required_consecutive_cycles", config.warmup_cycles, 100);
  nh.param("phase_offset/measurement/enable", config.measurement_enabled, false);
  nh.param<std::string>("phase_offset/measurement/tube_due_csv_path",
                        config.measurement_tube_due_csv_path,
                        std::string());
  if (mode == PhaseOffsetMatchedMode::ACTIVE) return config;
  nh.param("phase_offset/manual/amplitude", config.amplitude, 0.10);
  nh.param("phase_offset/manual/observe_only", config.observe_only, true);
  nh.param("phase_offset/manual/profile_period", config.profile_period, 10.0);
  nh.param("phase_offset/manual/warmup_cycles", config.warmup_cycles, 100); nh.param("phase_offset/manual/delta_tracking_gain", config.delta_tracking_gain, 3.0);
  nh.param("phase_offset/manual/u_w_amplitude", config.u_w_amplitude, 0.05); nh.param("phase_offset/manual/u_w_abs_max", config.u_w_abs_max, 0.12);
  nh.param("phase_offset/manual/u_delta_abs_max", config.u_delta_abs_max, 0.25); nh.param("phase_offset/manual/u_w_rate_max", config.u_w_rate_max, 0.60);
  nh.param("phase_offset/manual/u_delta_rate_max", config.u_delta_rate_max, 1.20); nh.param("phase_offset/manual/phase_dot_min", config.phase_dot_min, 0.02);
  nh.param("phase_offset/manual/tangent_speed_min", config.tangent_speed_min, 0.02); nh.param("phase_offset/manual/preflight_sample_step_w", config.preflight_sample_step_w, 0.10);
  // NORMAL Preview production policy is required configuration.  Read each
  // value only when explicitly present; absent/invalid values remain invalid
  // and are never replaced by an adapter or Tube default.
  const auto read_required_double = [&nh](const std::string& key,
                                           double& value) {
    return nh.hasParam(key) && nh.getParam(key, value);
  };
  bool preview_policy_complete = true;
  preview_policy_complete = read_required_double(
      "phase_offset/normal_preview/preview_horizon_w",
      config.normal_preview_policy.preview_horizon_w) &&
      preview_policy_complete;
  preview_policy_complete = read_required_double(
      "phase_offset/normal_preview/sample_spacing_w",
      config.normal_preview_policy.sample_spacing_w) &&
      preview_policy_complete;
  preview_policy_complete = read_required_double(
      "phase_offset/normal_preview/lower_nu",
      config.normal_preview_policy.lower_nu) && preview_policy_complete;
  preview_policy_complete = read_required_double(
      "phase_offset/normal_preview/upper_nu",
      config.normal_preview_policy.upper_nu) && preview_policy_complete;
  preview_policy_complete = read_required_double(
      "phase_offset/normal_preview/b_tight",
      config.normal_preview_policy.b_tight) && preview_policy_complete;
  preview_policy_complete = read_required_double(
      "phase_offset/normal_preview/b_open",
      config.normal_preview_policy.b_open) && preview_policy_complete;
  if (preview_policy_complete &&
      config.normal_preview_policy.immutable &&
      IsFinite(config.normal_preview_policy.preview_horizon_w) &&
      IsFinite(config.normal_preview_policy.sample_spacing_w) &&
      IsFinite(config.normal_preview_policy.lower_nu) &&
      IsFinite(config.normal_preview_policy.upper_nu) &&
      IsFinite(config.normal_preview_policy.b_tight) &&
      IsFinite(config.normal_preview_policy.b_open)) {
    config.normal_preview_policy.policy_revision = 1U;
    config.normal_preview_policy.configuration_identity =
        NormalPreviewPolicyIdentity(config.normal_preview_policy);
    config.normal_preview_policy.configuration_id =
        "normal-preview-w-v1";
    config.normal_preview_policy_explicit = true;
  }
  // Stage-7 production has one Tube backend.  Synthetic FIXED values remain
  // usable only by direct, unadvertised unit fixtures; ROS configuration no
  // longer selects a live proof implementation.
  config.tube_source = phase_offset_navigation::TubeSource::ESDF;
  // Sole P1 ESDF nominal-width owner.  Use hasParam so diagnostics can
  // distinguish the exact absent/default path from an explicit value.
  const std::string nominal_key = "phase_offset/tube/nominal_half_width";
  if (nh.hasParam(nominal_key)) {
    nh.param(nominal_key, config.tube.cross_section.nominal_half_width, 1.0);
    config.tube.cross_section.nominal_width_source =
        phase_offset_navigation::TubeNominalWidthSource::EXPLICIT_PARAMETER;
  } else {
    config.tube.cross_section.nominal_half_width = 1.0;
    config.tube.cross_section.nominal_width_source =
        phase_offset_navigation::TubeNominalWidthSource::DEFAULT_ABSENT;
  }
  nh.param("phase_offset/tube/fixed_delta_max", config.tube.fixed_delta_max, 0.06); nh.param("phase_offset/tube/sample_step_w", config.tube.sample_step_w, 0.10);
  nh.param("phase_offset/tube/max_offset", config.tube.max_offset, 0.20);
  nh.param("phase_offset/tube/lookahead_w", config.tube.lookahead_w, 2.0); nh.param("phase_offset/tube/back_w", config.tube.back_w, 0.20);
  nh.param("phase_offset/tube/min_certified_forward_w", config.tube.min_certified_forward_w, 0.40);
  nh.param("phase_offset/tube/boundary_slope_max", config.filter.boundary_slope_max, 0.80);
  nh.param("phase_offset/tube/invariant_gain", config.tube.invariant_gain, 1.0); nh.param("phase_offset/tube/interior_margin", config.tube.interior_margin, 0.0);
  nh.param("phase_offset/tube/update_period", config.tube_update_period, 0.10);
  nh.param("phase_offset/tube/environment_search_extent", config.tube.cross_section.search_extent, 3.0);
  nh.param("phase_offset/tube/raw_ray_step", config.tube.cross_section.ray_step, 0.05);
  nh.param("phase_offset/tube/boundary_tolerance", config.tube.cross_section.boundary_tolerance, 0.01);
  nh.param("phase_offset/tube/raw_regularity_margin", config.tube.cross_section.regularity_margin, 0.10);
  nh.param("phase_offset/tube/curvature_epsilon", config.tube.cross_section.curvature_epsilon, 1e-9);
  nh.param("planning/safe_distance",
           config.tube.cross_section.planner_safe_distance, 0.4);
  double sdf_map_resolution = -1.0;
  nh.param("sdf_map/resolution", sdf_map_resolution, -1.0);
  // The raw robust-margin parameters below are retained only for legacy
  // diagnostics.  Production Tube geometry uses planning/safe_distance.
  nh.param("phase_offset/tube/raw_uav_radius", config.tube.cross_section.margins.uav_radius, 0.25);
  nh.param("phase_offset/tube/raw_map_uncertainty", config.tube.cross_section.margins.map_uncertainty, 0.10);
  nh.param("phase_offset/tube/raw_localization_uncertainty", config.tube.cross_section.margins.localization_uncertainty, 0.05);
  nh.param("phase_offset/tube/raw_tracking_error_bound", config.tube.cross_section.margins.tracking_error_bound, 0.15);
  nh.param("phase_offset/tube/cloud_obstacle_set_complete",
           config.cloud_obstacle_set_complete, false);
  nh.param("phase_offset/tube/preincluded_map_uncertainty",
           config.tube.cross_section.margins.preincluded_map_uncertainty, 0.0);
  config.tube.fixed_delta_max_explicit =
      nh.hasParam("phase_offset/tube/fixed_delta_max");
  config.tube.max_offset_explicit =
      nh.hasParam("phase_offset/tube/max_offset");
  config.tube.search_extent_explicit =
      nh.hasParam("phase_offset/tube/environment_search_extent");
  const double rho = config.tube.cross_section.nominal_half_width;
  const auto differs_from_nominal = [rho](const double value) {
    return !std::isfinite(value) || !std::isfinite(rho) || value != rho;
  };
  config.tube.cross_section.nominal_width_legacy_conflict =
      (config.tube.fixed_delta_max_explicit &&
       differs_from_nominal(config.tube.fixed_delta_max)) ||
      (config.tube.max_offset_explicit &&
       differs_from_nominal(config.tube.max_offset)) ||
      (config.tube.search_extent_explicit &&
       differs_from_nominal(config.tube.cross_section.search_extent));
  // Final V2 configuration is copied from the existing frozen authorities;
  // no legacy profile or runtime builder is consulted.  Missing/nonpositive
  // safety or map inputs remain invalid and disable MANUAL V2 construction.
  config.tube_certificate_v2.configuration_id =
      config.normal_preview_policy.configuration_identity;
  config.tube_certificate_v2.epsilon =
      config.tube.cross_section.planner_safe_distance;
  config.tube_certificate_v2.nominal_half_width =
      config.tube.cross_section.nominal_half_width;
  config.tube_certificate_v2.ray_step =
      config.tube.cross_section.ray_step;
  config.tube_certificate_v2.snapshot_resolution = sdf_map_resolution;
  config.tube_certificate_v2.minimum_reference_speed =
      config.tube.cross_section.minimum_reference_speed;
  config.tube_certificate_v2.sample_step_w = config.tube.sample_step_w;
  nh.param<std::string>("phase_offset/frame_id", config.frame_id, "world");
  return config;
}
PhaseOffsetMatchedAdapter::PhaseOffsetMatchedAdapter(
    const PhaseOffsetMatchedAdapterConfig& config)
    : config_(config) {
  // Stage 7 retires backend selection from the production lifecycle.  Every
  // MANUAL adapter owns exactly one V2 worker; ACTIVE remains the unchanged
  // zero-port comparator.
  configuration_valid_ = (config_.mode == PhaseOffsetMatchedMode::ACTIVE ||
      config_.mode == PhaseOffsetMatchedMode::MANUAL) &&
      IsFinite(config_.equivalence_tolerance) && config_.equivalence_tolerance >= 0.0 &&
      config_.warmup_cycles >= 100;
  if (config_.frame_id.empty()) config_.frame_id = "world";
  if (config_.mode == PhaseOffsetMatchedMode::MANUAL) {
    runtime_.reset(new phase_offset_navigation::PhaseOffsetRuntime(MakeRuntimeConfig(config_)));
    configuration_valid_ = configuration_valid_ && runtime_->configurationValid() &&
        config_.tube_source == phase_offset_navigation::TubeSource::ESDF &&
        config_.tube_certificate_v2.complete() &&
        config_.normal_preview_policy_explicit &&
        config_.normal_preview_policy.valid() &&
        config_.normal_preview_policy.configuration_identity ==
            config_.tube_certificate_v2.configuration_id &&
        IsFinite(config_.tube_update_period) &&
        config_.tube_update_period >= 0.05 &&
        config_.tube_update_period <= 0.10;
    if (configuration_valid_) {
      v2_shadow_worker_.reset(new PhaseOffsetTubeWorkerV2(
          config_.tube_certificate_v2));
    }
  }
}

PhaseOffsetMatchedAdapter::V2ShadowRequestIdentity
PhaseOffsetMatchedAdapter::makeV2ShadowRequestIdentity(
    const TubeWorkerPurposeV2 purpose,
    const phase_offset_navigation::TubeBuildInputV2& input) const {
  V2ShadowRequestIdentity identity;
  identity.purpose = purpose;
  identity.request_id = input.request_id;
  identity.execution_generation = input.path_key.execution_generation;
  identity.accepted_state_demand = input.map_capture_key.accepted_sequence;
  identity.useful_start = input.requested_start;
  identity.useful_end = input.requested_end;
  identity.path_key = input.path_key;
  identity.configuration_key = input.configuration_key;
  identity.map_capture_key = input.map_capture_key;
  return identity;
}

void PhaseOffsetMatchedAdapter::retainV2ShadowCompletionLocked(
    const TubeWorkerCompletionV2& completion) {
  if (completion.purpose == TubeWorkerPurposeV2::CURRENT &&
      pinned_current_cohort_ && pinned_current_cohort_->tube_worker_input_v2 &&
      completion.request_id == pinned_current_cohort_->tube_worker_input_v2->request_id &&
      !completion.built()) pinned_current_cohort_.reset();
  std::shared_ptr<const TubeWorkerCompletionV2>& retained =
      completion.purpose == TubeWorkerPurposeV2::CURRENT
      ? latest_v2_shadow_current_completion_
      : latest_v2_shadow_successor_completion_;
  const auto retain = [&]() {
    try {
      retained = std::make_shared<const TubeWorkerCompletionV2>(completion);
    } catch (const std::exception&) {
      retained.reset();
    }
  };
  if (!retained) {
    retain();
    return;
  }
  // Failed/cancelled evidence never displaces a useful built proof.  A
  // Advancing windows may extend the forward end without retaining the old
  // backward prefix. This replaces a whole immutable result, never rebinds
  // its proof to another map/path or merges proof cells across captures.
  if (!completion.built()) return;
  if (!retained->built()) {
    retain();
    return;
  }
  const bool same_authority =
      completion.execution_generation == retained->execution_generation &&
      completion.path_key == retained->path_key &&
      completion.configuration_key == retained->configuration_key &&
      completion.map_capture_key == retained->map_capture_key;
  bool dominates = false;
  const auto live_request = std::atomic_load(&latest_build_request_);
  const bool replaces_passed_range = live_request && live_request->active &&
      live_request->tube_worker_input_v2 &&
      live_request->tube_worker_input_v2->path_key == completion.path_key &&
      live_request->task_generation == completion.execution_generation &&
      std::isfinite(live_request->current_w) &&
      live_request->current_w >= retained->build.profile.certified_end &&
      live_request->current_w >= completion.build.profile.certified_start &&
      live_request->current_w < completion.build.profile.certified_end;
  const bool forward_extension =
      completion.path_key == retained->path_key &&
      completion.configuration_key == retained->configuration_key &&
      completion.map_capture_key.map_instance_id == retained->map_capture_key.map_instance_id &&
      completion.map_capture_key.configuration_generation == retained->map_capture_key.configuration_generation &&
      completion.map_capture_key.configuration_id == retained->map_capture_key.configuration_id &&
      completion.map_capture_key.frame_provenance == retained->map_capture_key.frame_provenance &&
      ((completion.useful_start > retained->useful_start &&
        completion.useful_start <= retained->useful_end &&
        completion.useful_end > retained->useful_end) || replaces_passed_range);
  if (!same_authority) {
    dominates = completion.execution_generation ==
            retained->execution_generation &&
        completion.request_id > retained->request_id &&
        completion.accepted_state_demand >=
            retained->accepted_state_demand &&
        ((completion.useful_start <= retained->useful_start &&
          completion.useful_end >= retained->useful_end) || forward_extension);
  } else {
    const bool wider_prefix = completion.useful_start <=
            retained->useful_start &&
        completion.useful_end >= retained->useful_end;
    dominates = completion.accepted_state_demand >=
            retained->accepted_state_demand &&
        completion.request_id > retained->request_id && (wider_prefix || forward_extension);
  }
  if (dominates) {
    retain();
  }
}

bool PhaseOffsetMatchedAdapter::evaluateV2NormalAllocator(
    const MatchedAdapterInput& input,
    const std::shared_ptr<const phase_offset_navigation::TubeProfileV2>& profile,
    phase_offset_core::PhaseOffsetGeometryState& geometry,
    guidance::IsfGuidance& base,
    phase_offset_navigation::NormalPreviewResult& preview,
    phase_offset_navigation::PhaseOffsetAllocatorResult& allocator,
    Eigen::Vector3d& g_des,
    std::string& reason) const {
  geometry = phase_offset_core::PhaseOffsetGeometryState();
  base = guidance::IsfGuidance();
  preview = phase_offset_navigation::NormalPreviewResult();
  allocator = phase_offset_navigation::PhaseOffsetAllocatorResult();
  g_des.setZero();
  reason.clear();
  if (!runtime_ || !profile || !profile->structurallyValid() ||
      !config_.normal_preview_policy_explicit ||
      !config_.normal_preview_policy.valid()) {
    reason = "V2 NORMAL profile or preview policy is unavailable";
    return false;
  }
  phase_offset_core::GeometryParams geometry_params;
  geometry_params.regularity_margin =
      runtime_->config().tube.regularity_margin;
  geometry_params.minimum_reference_speed =
      runtime_->config().tube.minimum_reference_speed;
  geometry_params.require_frame_binding = true;
  phase_offset_core::GeometryEvaluator evaluator(geometry_params);
  if (!evaluator.evaluate(input.path, input.position,
                          runtime_->retainedDelta(), geometry) ||
      !geometry.valid ||
      geometry.path_revision != profile->path_key.path_revision ||
      geometry.frame_revision != profile->path_key.frame_revision) {
    reason = "V2 NORMAL live geometry is unavailable or stale";
    return false;
  }
  guidance::ReferenceGeometry reference;
  reference.point = geometry.r;
  reference.tangent = geometry.T;
  reference.derivative_norm = geometry.r_w.norm();
  reference.valid = true;
  if (!guidance::IsfReferenceKernel::evaluate(
          input.position, reference, input.gains, base) || !base.valid) {
    reason = "V2 NORMAL base guidance is unavailable";
    return false;
  }

  phase_offset_navigation::TubeViabilityInput preview_input;
  preview_input.current_w = geometry.w;
  preview_input.current_delta = runtime_->retainedDelta();
  preview_input.policy = config_.normal_preview_policy;
  preview_input.upper_u_delta = std::max(0.0, config_.u_delta_abs_max);
  preview_input.boundary_tolerance = 0.0;
  preview_input.path_revision = profile->path_key.path_revision;
  preview_input.frame_revision = profile->path_key.frame_revision;
  preview_input.profile_revision = profile->profile_id;
  preview_input.expected_path_revision = profile->path_key.path_revision;
  preview_input.expected_frame_revision = profile->path_key.frame_revision;
  preview_input.expected_profile_revision = profile->profile_id;
  preview_input.max_work = config_.tube_certificate_v2.budgets.max_queries;
  preview_input.v2_provenance_bound = true;
  preview_input.expected_v2_path_key = profile->path_key;
  preview_input.expected_v2_configuration_key = profile->configuration_key;
  preview_input.expected_v2_map_capture_key = profile->map_capture_key;
  if (!phase_offset_navigation::TubeViability::evaluate(
          *profile, preview_input, preview) || !preview.valid ||
      !preview.feasible || !preview.rate_feasible ||
      !preview.contraction_rate_feasible || !preview.current_delta_inside) {
    reason = preview.reason.empty()
        ? "V2 NORMAL live preview is infeasible" : preview.reason;
    return false;
  }

  const Eigen::Vector3d recenter = -config_.delta_tracking_gain *
      runtime_->retainedDelta() * geometry.N;
  if (!recenter.allFinite()) {
    reason = "V2 NORMAL recenter motion is nonfinite";
    return false;
  }
  Eigen::Vector3d g_swarm = Eigen::Vector3d::Zero();
  switch (config_.coordination_backend) {
    case PhaseOffsetCoordinationBackend::SPH:
      if (input.sph_bridge != nullptr) {
        const bspline_race::integration::GCoordResolveResult resolved =
            input.sph_bridge->resolveGCoord(
                input.captured_gcoord, ros::Time::now().toSec(),
                ros::SteadyTime::now().toSec());
        if (resolved.usable()) {
          g_swarm = Eigen::Vector3d(resolved.sample.g_coord.x,
                                    resolved.sample.g_coord.y, 0.0);
        }
      }
      g_des = g_swarm + recenter;
      break;
    case PhaseOffsetCoordinationBackend::D1B:
      g_des = input.g_des_valid ? input.g_des : recenter;
      break;
    case PhaseOffsetCoordinationBackend::DISABLED:
      g_des = recenter;
      break;
  }
  if (!g_des.allFinite()) {
    reason = "V2 NORMAL desired reference motion is nonfinite";
    return false;
  }

  phase_offset_navigation::PhaseOffsetAllocatorInput allocator_input;
  allocator_input.geometry = geometry;
  allocator_input.preview = &preview;
  allocator_input.g_des = g_des;
  allocator_input.f_w0 = base.w_dot;
  allocator_input.previous_u = runtime_->previousFinalPort();
  allocator_input.dt = input.dt;
  allocator_input.bounds.lower_nu = config_.normal_preview_policy.lower_nu;
  allocator_input.bounds.upper_nu = config_.normal_preview_policy.upper_nu;
  allocator_input.bounds.u_w_abs_max = std::max(0.0, config_.u_w_abs_max);
  allocator_input.bounds.upper_u_delta =
      std::max(0.0, config_.u_delta_abs_max);
  allocator_input.bounds.u_w_slew_rate =
      std::max(0.0, config_.u_w_rate_max);
  allocator_input.bounds.u_delta_slew_rate =
      std::max(0.0, config_.u_delta_rate_max);
  allocator_input.bounds.zoh_dt = input.dt;
  allocator_input.expected_path_revision = preview.provenance.path_revision;
  allocator_input.expected_frame_revision = preview.provenance.frame_revision;
  allocator_input.expected_profile_revision = preview.provenance.profile_revision;
  allocator_input.expected_source_revision = preview.provenance.source_revision;
  allocator_input.expected_tube_revision = preview.provenance.tube_revision;
  allocator_input.expected_map_revision = preview.provenance.map_revision;
  allocator_input.expected_obstacle_contract_id =
      preview.provenance.obstacle_contract_id;
  if (!phase_offset_navigation::PhaseOffsetAllocator::allocate(
          allocator_input, allocator) || !allocator.valid ||
      !allocator.feasible ||
      allocator.selected_u_owner !=
          phase_offset_navigation::PhaseOffsetAllocator::ownerName() ||
      !allocator.selectedUConsistent(0.0)) {
    reason = allocator.reason.empty()
        ? "V2 NORMAL allocator rejected the command" : allocator.reason;
    return false;
  }
  return true;
}

bool PhaseOffsetMatchedAdapter::prepareV2ShadowAdmission(
    const MatchedAdapterInput& input,
    const std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>&
        source_owner,
    const std::shared_ptr<const TubeWorkerCompletionV2>& completion,
    std::shared_ptr<const TubeV2ShadowAdmissionCandidate>& candidate,
    const TubeWorkerPurposeV2 purpose) {
  candidate.reset();
  TubeV2ShadowAdmissionCandidate staged;
  if (!source_owner) return false;
  const phase_offset_navigation::TubeBuildInputV2& source = *source_owner;
  staged.nonselecting = config_.observe_only || !zero_gate_open_ ||
      failure_latched_;
  staged.purpose = purpose;
  staged.request_id = source.request_id;
  staged.execution_generation = source.path_key.execution_generation;
  staged.accepted_state_demand = source.map_capture_key.accepted_sequence;
  staged.accepted_update_visible = false;
  staged.accepted_update_compatible = false;
  staged.path_key = source.path_key;
  staged.configuration_key = source.configuration_key;
  staged.map_capture_key = source.map_capture_key;

  const auto fail = [&staged, &candidate](
      const phase_offset_navigation::TubeExecutionStatusV2 status,
      const std::string& reason) {
    staged.status = status;
    staged.reason = reason;
    try {
      candidate = std::make_shared<const TubeV2ShadowAdmissionCandidate>(
          staged);
    } catch (const std::exception&) {
      candidate.reset();
    }
    return false;
  };

  if (!completion || completion->purpose != purpose ||
      !completion->built() || completion->request_id != source.request_id ||
      completion->execution_generation != source.path_key.execution_generation ||
      completion->path_key != source.path_key ||
      completion->configuration_key != source.configuration_key ||
      completion->map_capture_key != source.map_capture_key ||
      !source.complete() ||
      source.path_key.execution_generation !=
          task_generation_.load(std::memory_order_acquire)) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                purpose == TubeWorkerPurposeV2::CURRENT
                    ? "CURRENT completion is stale or identity-mismatched"
                    : "SUCCESSOR completion is stale or identity-mismatched");
  }

  const auto& certified = completion->build.profile;
  if (!IsFinite(certified.certified_start) || !IsFinite(certified.certified_end) ||
      source.requested_start > certified.certified_start ||
      certified.certified_start > certified.certified_end ||
      certified.certified_end > source.requested_end ||
      completion->useful_start != certified.certified_start ||
      completion->useful_end != certified.certified_end) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                "V2 certified range is inconsistent with original source");
  }
  const std::shared_ptr<const phase_offset_navigation::TubeProfileV2> profile_copy(
      completion, &completion->build.profile);
  if (!profile_copy->structurallyValid() ||
      profile_copy->path_key != source.path_key ||
      profile_copy->configuration_key != source.configuration_key ||
      profile_copy->map_capture_key != source.map_capture_key ||
      profile_copy->request_id != source.request_id ||
      profile_copy->path_owner != source.path_owner ||
      profile_copy->query_owner != source.query_owner ||
      profile_copy->capture_owner != source.capture_owner ||
      profile_copy->applicability_assumptions !=
          source.applicability_assumptions ||
      profile_copy->applicability_deadline_ticks !=
          source.applicability_deadline_ticks ||
      profile_copy->applicability_deadline_timeless !=
          source.applicability_deadline_timeless ||
      !source.map_capture_key.complete() ||
      !source.map_capture_key.complete_support) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::PROFILE_UNAVAILABLE,
                "V2 profile support or applicability is stale");
  }

  MatchedAdapterInput evaluated_input = input;
  MatchedAdapterInput::TubeV2AdmissionEvidence& evidence =
      evaluated_input.tube_v2_admission;
  // Captured provenance is pinned; latest_* and command evidence remain live.
  evidence.accepted_state_sequence = source.map_capture_key.accepted_sequence;
  evidence.accepted_state_notification_sequence = source.map_capture_key.accepted_sequence;
  evidence.accepted_time_ticks = source.map_capture_key.accepted_time_ticks;
  evidence.map_instance_id = source.map_capture_key.map_instance_id;
  evidence.configuration_generation = source.map_capture_key.configuration_generation;
  evidence.configuration_key = source.map_capture_key.configuration_id;
  evidence.support_provenance_id = source.map_capture_key.support_provenance_id;
  evidence.frame_provenance = source.map_capture_key.frame_provenance;
  evidence.support_expiry_timeless = source.map_capture_key.support_expiry_timeless;
  evidence.support_expiry_ticks = source.map_capture_key.support_expiry_ticks;
  evidence.applicability_deadline_valid = !source.applicability_deadline_timeless;
  evidence.applicability_deadline = static_cast<double>(source.applicability_deadline_ticks);
  if (!evidence.valid || evidence.binding_sequence == 0U) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                "V2 admission applicability evidence is unavailable");
  }
  staged.binding_sequence = evidence.binding_sequence;
  if (!IsFinite(evidence.now) || !IsFinite(input.dt) || input.dt <= 0.0 ||
      evidence.provenance.empty() ||
      (evidence.applicability_deadline_valid &&
       !IsFinite(evidence.applicability_deadline)) ||
      evidence.accepted_state_sequence !=
          source.map_capture_key.accepted_sequence ||
      evidence.accepted_state_sequence != source.map_capture_key.state_id ||
      evidence.accepted_state_notification_sequence <
          evidence.accepted_state_sequence ||
      evidence.accepted_time_ticks != source.map_capture_key.accepted_time_ticks ||
      evidence.map_instance_id != source.map_capture_key.map_instance_id ||
      evidence.configuration_generation !=
          source.map_capture_key.configuration_generation ||
      evidence.configuration_key != source.map_capture_key.configuration_id ||
      evidence.support_provenance_id !=
          source.map_capture_key.support_provenance_id ||
      evidence.frame_provenance != source.map_capture_key.frame_provenance ||
      evidence.support_expiry_timeless !=
          source.map_capture_key.support_expiry_timeless ||
      (evidence.support_expiry_timeless && evidence.support_expiry_ticks != 0U) ||
      (!evidence.support_expiry_timeless &&
       evidence.support_expiry_ticks != source.map_capture_key.support_expiry_ticks) ||
      evidence.applicability_deadline_valid !=
          !profile_copy->applicability_deadline_timeless ||
      (evidence.applicability_deadline_valid &&
       evidence.applicability_deadline != static_cast<double>(
           profile_copy->applicability_deadline_ticks)) ||
      evidence.latest_accepted_state_sequence <
          evidence.accepted_state_sequence ||
      evidence.latest_accepted_state_notification_sequence <
          evidence.latest_accepted_state_sequence ||
      evidence.latest_accepted_time_ticks < evidence.accepted_time_ticks) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                "V2 admission applicability identity or timing is invalid");
  }
  if (evidence.latest_map_instance_id != evidence.map_instance_id ||
      evidence.latest_configuration_generation !=
          evidence.configuration_generation ||
      evidence.latest_configuration_key != evidence.configuration_key ||
      evidence.latest_support_provenance_id !=
          evidence.support_provenance_id ||
      evidence.latest_frame_provenance != evidence.frame_provenance) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                "latest accepted map identity is not coherent");
  }
  if (!source.map_capture_key.support_expiry_timeless) {
    const double expiry = static_cast<double>(
        source.map_capture_key.support_expiry_ticks);
    if (!IsFinite(expiry) || evidence.now >= expiry ||
        (evidence.applicability_deadline_valid &&
         evidence.applicability_deadline > expiry) ||
        evidence.latest_accepted_time_ticks >=
            source.map_capture_key.support_expiry_ticks ||
        (evidence.applicability_deadline_valid &&
         evidence.latest_accepted_time_ticks >= static_cast<std::uint64_t>(
             evidence.applicability_deadline))) {
      return fail(phase_offset_navigation::TubeExecutionStatusV2::DEADLINE_EXPIRED,
                  "V2 map support applicability has expired");
    }
  }
  staged.accepted_update_visible =
      evidence.latest_accepted_state_sequence >
          evidence.accepted_state_sequence ||
      evidence.latest_accepted_state_notification_sequence >
          evidence.accepted_state_notification_sequence;
  staged.latest_accepted_state_sequence =
      evidence.latest_accepted_state_sequence;
  staged.latest_accepted_state_notification_sequence =
      evidence.latest_accepted_state_notification_sequence;
  staged.latest_accepted_time_ticks = evidence.latest_accepted_time_ticks;
  staged.latest_map_instance_id = evidence.latest_map_instance_id;
  staged.latest_configuration_generation =
      evidence.latest_configuration_generation;
  staged.latest_configuration_key = evidence.latest_configuration_key;
  staged.latest_support_provenance_id = evidence.latest_support_provenance_id;
  staged.latest_frame_provenance = evidence.latest_frame_provenance;
  staged.accepted_update_compatible =
      !staged.accepted_update_visible || source.map_capture_key.complete_support;
  if (!staged.accepted_update_compatible) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                "newer accepted map state is not covered by the captured support");
  }
  const std::shared_ptr<const TubeV2ShadowAdmissionCandidate>
      retained_admission = std::atomic_load_explicit(
          &retained_v2_shadow_admission_candidate_,
          std::memory_order_acquire);
  if (retained_admission &&
      retained_admission->request_id == source.request_id &&
      retained_admission->execution_generation ==
          source.path_key.execution_generation &&
      retained_admission->path_key == source.path_key &&
      retained_admission->configuration_key == source.configuration_key &&
      retained_admission->map_capture_key ==
          source.map_capture_key &&
      retained_admission->prepared &&
      retained_admission->applicable &&
      retained_admission->binding_sequence != 0U &&
      retained_admission->binding_sequence != evidence.binding_sequence) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                "V2 admission binding identity was substituted");
  }

  phase_offset_core::PhaseOffsetGeometryState live_geometry;
  guidance::IsfGuidance live_base;
  phase_offset_navigation::NormalPreviewResult live_preview;
  phase_offset_navigation::PhaseOffsetAllocatorResult live_allocator;
  Eigen::Vector3d live_g_des = Eigen::Vector3d::Zero();
  std::string allocator_reason;
  if (!evaluateV2NormalAllocator(
          input, profile_copy, live_geometry, live_base, live_preview,
          live_allocator, live_g_des, allocator_reason)) {
    staged.allocator_evaluated = true;
    staged.normal_preview = live_preview;
    staged.allocator = live_allocator;
    staged.g_des = live_g_des;
    staged.g_des_valid = live_g_des.allFinite();
    return fail(phase_offset_navigation::TubeExecutionStatusV2::COMMAND_INFEASIBLE,
                allocator_reason.empty()
                    ? "V2 NORMAL allocator is unavailable"
                    : allocator_reason);
  }
  if (!PopulateV2LiveCommandEvidence(
          config_, live_geometry, live_base, live_allocator.selected_u,
          evidence)) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                "V2 NORMAL live command evidence is unavailable");
  }
  staged.g_des = live_g_des;
  staged.g_des_valid = true;
  staged.normal_preview = live_preview;
  staged.allocator = live_allocator;
  staged.allocator_evaluated = true;
  phase_offset_navigation::RuntimeV2PrepareInput runtime_input =
      MakeV2RuntimePrepareInput(
          evaluated_input, profile_copy, evidence, false,
          runtime_->retainedDelta(),
          runtime_->previousFinalPort());
  if (purpose == TubeWorkerPurposeV2::SUCCESSOR) {
    // The successor profile is the proposed binding, while Runtime still
    // owns the installed source binding until the later publish-first
    // transaction.  Keep the stale-identity CAS strict by carrying both
    // identities explicitly through the value-only prepare seam.
    if (!v2_execution_binding_ || !v2_execution_binding_->complete() ||
        evidence.binding_sequence ==
            std::numeric_limits<std::uint64_t>::max()) {
      return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                  "SUCCESSOR binding transition identity is unavailable");
    }
    runtime_input.binding_transition = true;
    runtime_input.expected_identity = MakeV2ExecutionIdentity(
        *v2_execution_binding_->profile, evidence.binding_sequence);
    runtime_input.identity.binding_sequence = evidence.binding_sequence + 1U;
  } else if (v2_execution_binding_ && v2_execution_binding_->complete() &&
             !SameV2ExecutionIdentity(runtime_input.identity,
                                      v2_execution_binding_->identity)) {
    if (evidence.binding_sequence ==
            std::numeric_limits<std::uint64_t>::max() ||
        evidence.binding_sequence !=
            v2_execution_binding_->identity.binding_sequence ||
        source.path_key != v2_execution_binding_->profile->path_key ||
        source.configuration_key !=
            v2_execution_binding_->profile->configuration_key ||
        !V2MapRefreshAuthorityCompatible(
            v2_execution_binding_->profile->map_capture_key,
            source.map_capture_key)) {
      return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                  "CURRENT replacement binding is incompatible");
    }
    runtime_input.binding_transition = true;
    runtime_input.expected_identity = v2_execution_binding_->identity;
    runtime_input.identity.binding_sequence = evidence.binding_sequence + 1U;
  }
  phase_offset_navigation::RuntimeV2PreparedStep prepared;
  if (!runtime_->prepareV2(runtime_input, prepared)) {
    staged.status = prepared.admission.status;
    if (staged.status == phase_offset_navigation::TubeExecutionStatusV2::UNAVAILABLE) {
      staged.status = phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT;
    }
    return fail(staged.status, prepared.invalid_reason.empty()
                    ? "V2 Runtime admission was rejected"
                    : prepared.invalid_reason);
  }
  phase_offset_navigation::RuntimeV2CommitToken token;
  if (!runtime_->makeCommitTokenV2(prepared, token)) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::RESERVE_UNAVAILABLE,
                "V2 Runtime commit token could not be materialized");
  }
  staged.profile = profile_copy;
  staged.prepared_step = std::move(prepared);
  staged.binding_sequence = staged.prepared_step.identity.binding_sequence;
  staged.commit_token = std::move(token);
  try {
    std::shared_ptr<TubeV2ExecutionBinding> binding(
        new TubeV2ExecutionBinding());
    binding->profile = profile_copy;
    binding->source_input = source_owner;
    const std::shared_ptr<const ContinuousPhasePath> path_owner =
        std::static_pointer_cast<const ContinuousPhasePath>(
            source_owner->path_owner);
    if (!path_owner || path_owner->empty() ||
        path_owner->pathRevision() != source.path_key.path_revision ||
        path_owner->startW() != source.path_key.domain_start ||
        path_owner->endW() != source.path_key.domain_end) {
      return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                  "V2 proposed path owner is incomplete");
    }
    binding->frame_owner =
        std::make_shared<const ContinuousPhaseNormalFrame>(
            path_owner, source.path_key.path_revision,
            source.path_key.frame_revision);
    binding->identity = staged.prepared_step.identity;
    binding->valid = true;
    if (!binding->complete()) {
      return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                  "V2 proposed execution binding is incomplete");
    }
    staged.proposed_binding =
        std::shared_ptr<const TubeV2ExecutionBinding>(binding);
  } catch (const std::exception&) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                "V2 proposed execution binding could not be materialized");
  }
  staged.status = phase_offset_navigation::TubeExecutionStatusV2::ADMISSIBLE;
  staged.applicable = true;
  staged.prepared = true;
  staged.reason.clear();
  try {
    candidate = std::make_shared<const TubeV2ShadowAdmissionCandidate>(
        std::move(staged));
  } catch (const std::exception&) {
    candidate.reset();
    return false;
  }
  if (purpose == TubeWorkerPurposeV2::CURRENT) {
    std::atomic_store_explicit(&retained_v2_shadow_admission_candidate_,
                               candidate, std::memory_order_release);
  }
  return true;
}

bool PhaseOffsetMatchedAdapter::prepareV2ShadowSuccessorAdmission(
    const MatchedAdapterInput& input,
    const TubeV2SuccessorHandoffEvidence& handoff,
    const std::shared_ptr<const TubeWorkerCompletionV2>& completion,
    std::shared_ptr<const TubeV2ShadowAdmissionCandidate>& candidate) {
  candidate.reset();
  TubeV2ShadowAdmissionCandidate staged;
  staged.nonselecting = true;
  staged.purpose = TubeWorkerPurposeV2::SUCCESSOR;
  staged.source_path_key = handoff.source_path_key;
  staged.copied_prefix_start_w = handoff.copied_prefix_start_w;
  staged.copied_prefix_end_w = handoff.copied_prefix_end_w;

  const auto fail = [&staged, &candidate](
      const phase_offset_navigation::TubeExecutionStatusV2 status,
      const std::string& reason) {
    staged.status = status;
    staged.reason = reason;
    try {
      candidate = std::make_shared<const TubeV2ShadowAdmissionCandidate>(
          std::move(staged));
    } catch (const std::exception&) {
      candidate.reset();
    }
    return false;
  };

  // The manager attests the copied prefix at construction time.  Admission
  // additionally checks every immutable owner/key and the exact request
  // pointer, so an independently coincident path cannot use the attestation.
  if (!handoff.valid || !handoff.structurally_copied_prefix ||
      handoff.expected_execution_generation == 0U ||
      !handoff.source_path_key.complete() ||
      !handoff.successor_path_key.complete() ||
      handoff.source_path_key == handoff.successor_path_key ||
      handoff.source_path_key.execution_generation !=
          handoff.expected_execution_generation ||
      handoff.successor_path_key.execution_generation !=
          handoff.expected_execution_generation ||
      !handoff.source_path_owner || !handoff.successor_path_owner ||
      !handoff.successor_request || !handoff.successor_request->complete() ||
      handoff.successor_request->path_key != handoff.successor_path_key ||
      handoff.successor_request->path_owner.get() !=
          handoff.successor_path_owner.get() ||
      handoff.provenance.empty() ||
      !IsFinite(handoff.phase_after_w) ||
      !IsFinite(handoff.copied_prefix_start_w) ||
      !IsFinite(handoff.copied_prefix_end_w) ||
      handoff.phase_after_w != handoff.copied_prefix_start_w ||
      !(handoff.copied_prefix_end_w > handoff.copied_prefix_start_w) ||
      handoff.phase_after_w < handoff.source_path_key.domain_start ||
      handoff.copied_prefix_end_w > handoff.source_path_key.domain_end ||
      handoff.phase_after_w < handoff.successor_path_key.domain_start ||
      handoff.copied_prefix_end_w > handoff.successor_path_key.domain_end ||
      handoff.source_path_key.phase_orientation !=
          handoff.successor_path_key.phase_orientation ||
      handoff.source_path_key.frame_convention_id !=
          handoff.successor_path_key.frame_convention_id ||
      handoff.source_path_key.frame_convention !=
          handoff.successor_path_key.frame_convention ||
      handoff.source_path_owner->startW() !=
          handoff.source_path_key.domain_start ||
      handoff.source_path_owner->endW() !=
          handoff.source_path_key.domain_end ||
      handoff.source_path_owner->pathRevision() !=
          handoff.source_path_key.path_revision ||
      handoff.successor_path_owner->startW() !=
          handoff.successor_path_key.domain_start ||
      handoff.successor_path_owner->endW() !=
          handoff.successor_path_key.domain_end ||
      handoff.successor_path_owner->pathRevision() !=
          handoff.successor_path_key.path_revision ||
      handoff.successor_request->requested_start !=
          handoff.copied_prefix_start_w ||
      handoff.successor_request->anchor_w != handoff.phase_after_w ||
      handoff.successor_request->requested_end <
          handoff.copied_prefix_end_w) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                "SUCCESSOR copied-prefix handoff is malformed");
  }
  const std::uint64_t generation =
      task_generation_.load(std::memory_order_acquire);
  if (handoff.expected_execution_generation != generation ||
      handoff.successor_request->path_key.execution_generation != generation ||
      !runtime_ || !v2_execution_binding_ ||
      !v2_execution_binding_->complete() ||
      v2_execution_binding_->profile->path_key != handoff.source_path_key ||
      v2_execution_binding_->profile->path_owner.get() !=
          handoff.source_path_owner.get() ||
      v2_execution_binding_->source_input->path_key != handoff.source_path_key ||
      v2_execution_binding_->source_input->path_owner.get() !=
          handoff.source_path_owner.get() ||
      !input.semantic_path_owner ||
      input.semantic_path_owner.get() != handoff.source_path_owner.get()) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                "SUCCESSOR source binding is stale or substituted");
  }
  if (!std::isfinite(input.path.w) ||
      input.path.w < handoff.copied_prefix_start_w ||
      input.path.w > handoff.copied_prefix_end_w) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::RANGE_UNAVAILABLE,
                "live phase is outside the copied prefix");
  }

  ContinuousPhasePathState source_state;
  ContinuousPhasePathState successor_state;
  if (!handoff.source_path_owner->evaluate(input.path.w, source_state, false) ||
      !handoff.successor_path_owner->evaluate(input.path.w, successor_state,
                                               false) ||
      !source_state.valid || !successor_state.valid ||
      !source_state.p.allFinite() || !successor_state.p.allFinite() ||
      !source_state.dp_dw.allFinite() || !successor_state.dp_dw.allFinite()) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                "copied-prefix path evaluation is unavailable");
  }
  if (!BitsEqual(input.path.p, source_state.p) ||
      !BitsEqual(input.path.p_w, source_state.dp_dw) ||
      !BitsEqual(input.path.p_ww, source_state.d2p_dw2)) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                "live path state is not the installed source owner");
  }

  // Structural provenance is the authority for continuity.  These residuals
  // are evaluated only as immutable diagnostics (and must be finite); using a
  // sampled epsilon here would turn a copied-prefix proof into a numerical
  // coincidence test.
  staged.path_position_residual =
      (successor_state.p - source_state.p).norm();
  staged.path_derivative_residual =
      (successor_state.dp_dw - source_state.dp_dw).norm();
  const ContinuousPhaseNormalFrame source_frame(
      handoff.source_path_owner, handoff.source_path_key.path_revision,
      handoff.source_path_key.frame_revision);
  const ContinuousPhaseNormalFrame successor_frame(
      handoff.successor_path_owner,
      handoff.successor_path_key.path_revision,
      handoff.successor_path_key.frame_revision);
  phase_offset_core::NormalFrameQuery source_normal;
  phase_offset_core::NormalFrameQuery successor_normal;
  if (!source_frame.query(input.path.w, source_normal) ||
      !successor_frame.query(input.path.w, successor_normal) ||
      !source_normal.valid || !successor_normal.valid) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                "copied-prefix normal-frame evaluation is unavailable");
  }
  const double live_delta = runtime_->retainedDelta();
  const phase_offset_core::PortCommand live_previous =
      runtime_->previousFinalPort();
  const Eigen::Vector3d source_r = source_state.p + source_normal.N * live_delta;
  const Eigen::Vector3d successor_r =
      successor_state.p + successor_normal.N * live_delta;
  const Eigen::Vector3d source_rw =
      source_state.dp_dw + source_normal.N_w * live_delta;
  const Eigen::Vector3d successor_rw =
      successor_state.dp_dw + successor_normal.N_w * live_delta;
  guidance::ReferenceGeometry source_reference;
  source_reference.point = source_r;
  source_reference.tangent = source_normal.T;
  source_reference.derivative_norm = source_rw.norm();
  source_reference.valid = source_reference.point.allFinite() &&
      source_reference.tangent.allFinite() &&
      IsFinite(source_reference.derivative_norm);
  guidance::IsfGuidance live_base;
  if (!source_reference.valid ||
      !guidance::IsfReferenceKernel::evaluate(
          input.position, source_reference, input.gains, live_base) ||
      !live_base.valid) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                "copied-prefix live base guidance is unavailable");
  }
  const double nu = live_base.w_dot + live_previous.u_w;
  const Eigen::Vector3d source_r_dot =
      source_rw * nu + source_normal.N * live_previous.u_delta;
  const Eigen::Vector3d successor_r_dot =
      successor_rw * nu + successor_normal.N * live_previous.u_delta;
  staged.reference_position_residual = (successor_r - source_r).norm();
  staged.reference_derivative_residual =
      (successor_r_dot - source_r_dot).norm();
  if (!IsFinite(staged.path_position_residual) ||
      !IsFinite(staged.path_derivative_residual) ||
      !IsFinite(staged.reference_position_residual) ||
      !IsFinite(staged.reference_derivative_residual) || !IsFinite(nu)) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                "copied-prefix continuity diagnostics are nonfinite");
  }
  // Structural copied-prefix provenance is the continuity proof.  Evaluated
  // source/successor values may use different, mathematically equivalent
  // representations and are retained only as finite residual diagnostics;
  // bitwise coincidence is not an additional availability gate.
  staged.copied_prefix_continuity_valid = true;

  MatchedAdapterInput successor_input = input;
  successor_input.path.p = successor_state.p;
  successor_input.path.p_w = successor_state.dp_dw;
  successor_input.path.p_ww = successor_state.d2p_dw2;
  successor_input.path.w = input.path.w;
  successor_input.path.path_revision =
      handoff.successor_path_key.path_revision;
  successor_input.path.frame_revision =
      handoff.successor_path_key.frame_revision;
  successor_input.path.T = successor_normal.T;
  successor_input.path.N = successor_normal.N;
  successor_input.path.N_w = successor_normal.N_w;
  successor_input.path.frame_valid = true;
  successor_input.path.frame_provenance = successor_normal.provenance;
  successor_input.path.valid = true;
  successor_input.semantic_path_owner = handoff.successor_path_owner;
  successor_input.semantic_path_start_w =
      handoff.successor_path_key.domain_start;
  successor_input.semantic_path_end_w = handoff.successor_path_key.domain_end;
  if (!prepareV2ShadowAdmission(
          successor_input, handoff.successor_request, completion,
                                candidate, TubeWorkerPurposeV2::SUCCESSOR) ||
      !candidate) {
    return false;
  }
  // Preserve the continuity evidence while retaining the immutable candidate
  // produced by the common K/command/reserve preparation path.
  TubeV2ShadowAdmissionCandidate enriched = *candidate;
  enriched.purpose = TubeWorkerPurposeV2::SUCCESSOR;
  enriched.nonselecting = candidate->nonselecting;
  enriched.source_path_key = handoff.source_path_key;
  enriched.copied_prefix_start_w = handoff.copied_prefix_start_w;
  enriched.copied_prefix_end_w = handoff.copied_prefix_end_w;
  enriched.copied_prefix_continuity_valid = true;
  enriched.path_position_residual = staged.path_position_residual;
  enriched.path_derivative_residual = staged.path_derivative_residual;
  enriched.reference_position_residual = staged.reference_position_residual;
  enriched.reference_derivative_residual = staged.reference_derivative_residual;
  if (!enriched.prepared || !enriched.applicable ||
      !enriched.prepared_step.valid ||
      !enriched.prepared_step.admission.valid ||
      !enriched.prepared_step.admission.successor_reserve.valid ||
      enriched.prepared_step.expected_current.w <
          handoff.copied_prefix_start_w ||
      enriched.prepared_step.expected_current.w >
          handoff.copied_prefix_end_w ||
      enriched.prepared_step.successor.w < handoff.copied_prefix_start_w ||
      enriched.prepared_step.successor.w > handoff.copied_prefix_end_w) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::RANGE_UNAVAILABLE,
                "SUCCESSOR first held step crosses copied-prefix seam");
  }
  try {
    candidate = std::make_shared<const TubeV2ShadowAdmissionCandidate>(
        std::move(enriched));
  } catch (const std::exception&) {
    candidate.reset();
    return false;
  }
  return true;
}

bool PhaseOffsetMatchedAdapter::prepareV2ShadowIncumbentApplicability(
    const MatchedAdapterInput& input,
    const std::shared_ptr<const phase_offset_navigation::TubeProfileV2>&
        incumbent,
    std::shared_ptr<const TubeV2ShadowAdmissionCandidate>& candidate,
    phase_offset_navigation::RecoveryPreparedStep* recovery_step,
    const bool force_reserve) {
  candidate.reset();
  if (recovery_step) *recovery_step = phase_offset_navigation::RecoveryPreparedStep();

  TubeV2ShadowAdmissionCandidate staged;
  staged.nonselecting = true;
  staged.profile = incumbent;
  const auto fail = [&staged, &candidate](
      const phase_offset_navigation::TubeExecutionStatusV2 status,
      const std::string& reason) {
    staged.status = status;
    staged.reason = reason;
    try {
      candidate = std::make_shared<const TubeV2ShadowAdmissionCandidate>(
          std::move(staged));
    } catch (const std::exception&) {
      candidate.reset();
    }
    return false;
  };
  if (!incumbent) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::PROFILE_UNAVAILABLE,
                "installed V2 incumbent is unavailable");
  }
  staged.request_id = incumbent->request_id;
  staged.execution_generation = incumbent->path_key.execution_generation;
  staged.accepted_state_demand = incumbent->map_capture_key.accepted_sequence;
  staged.path_key = incumbent->path_key;
  staged.configuration_key = incumbent->configuration_key;
  staged.map_capture_key = incumbent->map_capture_key;
  if (!runtime_ || !incumbent->structurallyValid() || !incumbent->complete ||
      !incumbent->path_key.complete() || !incumbent->map_capture_key.complete() ||
      !incumbent->map_capture_key.complete_support ||
      incumbent->path_key.execution_generation !=
          task_generation_.load(std::memory_order_acquire)) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::PROFILE_UNAVAILABLE,
                "installed V2 incumbent is incomplete or stale");
  }

  const MatchedAdapterInput::TubeV2AdmissionEvidence& evidence =
      input.tube_v2_admission;
  if (!evidence.valid || evidence.binding_sequence == 0U ||
      !IsFinite(evidence.now) || !IsFinite(input.dt) || input.dt <= 0.0 ||
      evidence.provenance.empty()) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                "incumbent applicability evidence is unavailable");
  }
  if (!v2_execution_binding_ || !v2_execution_binding_->complete() ||
      v2_execution_binding_->profile != incumbent ||
      v2_execution_binding_->identity.binding_sequence !=
          evidence.binding_sequence) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                "incumbent execution binding is stale or substituted");
  }

  // Reconstruct one observed key and run the same semantic refresh predicate
  // used by worker coalescing.  All support/grid authority stays bound to the
  // incumbent; only accepted-state identity/time may advance.
  phase_offset_navigation::TubeMapCaptureKey observed =
      incumbent->map_capture_key;
  observed.map_instance_id = evidence.latest_map_instance_id;
  observed.state_id = evidence.latest_accepted_state_sequence;
  observed.accepted_sequence = evidence.latest_accepted_state_sequence;
  observed.configuration_generation = evidence.latest_configuration_generation;
  observed.configuration_id = evidence.latest_configuration_key;
  observed.frame_provenance = evidence.latest_frame_provenance;
  observed.support_provenance_id = evidence.latest_support_provenance_id;
  observed.accepted_time_ticks = evidence.latest_accepted_time_ticks;
  observed.support_expiry_ticks = evidence.support_expiry_ticks;
  observed.support_expiry_timeless = evidence.support_expiry_timeless;
  const bool support_expiry_compatible =
      incumbent->map_capture_key.support_expiry_timeless
          ? evidence.support_expiry_timeless &&
                evidence.support_expiry_ticks == 0U
          : !evidence.support_expiry_timeless &&
                evidence.support_expiry_ticks >=
                    incumbent->map_capture_key.support_expiry_ticks;
  if (evidence.latest_map_instance_id != evidence.map_instance_id ||
      evidence.latest_configuration_generation !=
          evidence.configuration_generation ||
      evidence.latest_configuration_key != evidence.configuration_key ||
      evidence.latest_support_provenance_id != evidence.support_provenance_id ||
      evidence.latest_frame_provenance != evidence.frame_provenance ||
      evidence.latest_accepted_state_sequence <
          evidence.accepted_state_sequence ||
      evidence.latest_accepted_state_notification_sequence <
          evidence.latest_accepted_state_sequence ||
      evidence.latest_accepted_time_ticks < evidence.accepted_time_ticks ||
      !support_expiry_compatible ||
      (observed != incumbent->map_capture_key &&
       !V2MapRefreshAuthorityCompatible(
           incumbent->map_capture_key, observed))) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                "incumbent accepted-map authority is incompatible");
  }
  if (!incumbent->applicability_deadline_timeless &&
      (evidence.now >= static_cast<double>(
           incumbent->applicability_deadline_ticks) ||
       evidence.latest_accepted_time_ticks >=
           incumbent->applicability_deadline_ticks)) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::DEADLINE_EXPIRED,
                "incumbent applicability deadline expired");
  }
  if (!incumbent->map_capture_key.support_expiry_timeless &&
      (evidence.now >= static_cast<double>(
           incumbent->map_capture_key.support_expiry_ticks) ||
       evidence.latest_accepted_time_ticks >=
           incumbent->map_capture_key.support_expiry_ticks)) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::DEADLINE_EXPIRED,
                "incumbent map support applicability expired");
  }

  staged.binding_sequence = evidence.binding_sequence;
  staged.accepted_update_visible = observed != incumbent->map_capture_key;
  staged.accepted_update_compatible = !staged.accepted_update_visible ||
      incumbent->map_capture_key.complete_support;
  staged.latest_accepted_state_sequence =
      evidence.latest_accepted_state_sequence;
  staged.latest_accepted_state_notification_sequence =
      evidence.latest_accepted_state_notification_sequence;
  staged.latest_accepted_time_ticks = evidence.latest_accepted_time_ticks;
  staged.latest_map_instance_id = evidence.latest_map_instance_id;
  staged.latest_configuration_generation =
      evidence.latest_configuration_generation;
  staged.latest_configuration_key = evidence.latest_configuration_key;
  staged.latest_support_provenance_id = evidence.latest_support_provenance_id;
  staged.latest_frame_provenance = evidence.latest_frame_provenance;
  staged.purpose = TubeWorkerPurposeV2::CURRENT;
  staged.proposed_binding = v2_execution_binding_;
  staged.nonselecting = config_.observe_only || !zero_gate_open_ ||
      failure_latched_;

  phase_offset_core::PhaseOffsetGeometryState live_geometry;
  guidance::IsfGuidance live_base;
  phase_offset_navigation::NormalPreviewResult live_preview;
  phase_offset_navigation::PhaseOffsetAllocatorResult live_allocator;
  Eigen::Vector3d live_g_des = Eigen::Vector3d::Zero();
  std::string normal_reason;
  const bool normal_allocator_available = evaluateV2NormalAllocator(
      input, incumbent, live_geometry, live_base, live_preview,
      live_allocator, live_g_des, normal_reason);
  staged.g_des = live_g_des;
  staged.g_des_valid = live_g_des.allFinite();
  staged.normal_preview = live_preview;
  staged.allocator = live_allocator;
  staged.allocator_evaluated = true;

  phase_offset_navigation::RuntimeV2PreparedStep prepared;
  const phase_offset_navigation::RecoveryOwnerStatus& recovery_status =
      recovery_owner_.status();
  const phase_offset_navigation::TubeFiniteReserveV2* runtime_reserve =
      runtime_->committedV2Reserve();
  const bool recovery_running = recovery_status.active &&
      recovery_status.proof_kind ==
          phase_offset_navigation::RecoveryStepProofKind::FINITE_RESERVE_V2 &&
      recovery_status.reserve_cursor < recovery_status.reserve_size;
  if (recovery_running &&
      (!runtime_reserve ||
       runtime_reserve->reserve_id != recovery_status.reserve_id ||
       runtime_reserve->cursor != recovery_status.reserve_cursor ||
       runtime_reserve->steps.size() != recovery_status.reserve_size)) {
    return fail(phase_offset_navigation::TubeExecutionStatusV2::
                    RESERVE_UNAVAILABLE,
                "running V2 recovery reserve identity is stale");
  }
  if (!force_reserve && !recovery_running && normal_allocator_available) {
    MatchedAdapterInput::TubeV2AdmissionEvidence live_evidence = evidence;
    if (!PopulateV2LiveCommandEvidence(
            config_, live_geometry, live_base, live_allocator.selected_u,
            live_evidence)) {
      return fail(phase_offset_navigation::TubeExecutionStatusV2::INVALID_INPUT,
                  "incumbent live command evidence is unavailable");
    }
    phase_offset_navigation::RuntimeV2PrepareInput runtime_input =
        MakeV2RuntimePrepareInput(input, incumbent, live_evidence, true,
                                  runtime_->retainedDelta(),
                                  runtime_->previousFinalPort());
    if (runtime_->dryRunV2(runtime_input, prepared) && prepared.valid &&
        runtime_->makeCommitTokenV2(prepared, staged.commit_token)) {
      staged.prepared_step = std::move(prepared);
      staged.status =
          phase_offset_navigation::TubeExecutionStatusV2::ADMISSIBLE;
      staged.applicable = true;
      staged.prepared = true;
      try {
        candidate = std::make_shared<const TubeV2ShadowAdmissionCandidate>(
            std::move(staged));
      } catch (const std::exception&) {
        candidate.reset();
        return false;
      }
      return true;
    }
  }

  // The P10 execution guard is the single owner of the audited tracking and
  // physical-tangent predicate.  A violation denies stale offset execution;
  // it cannot be reclassified as an applicable certified recovery merely
  // because an older reserve value is still stored in Runtime.
  if (normal_allocator_available && !recovery_running &&
      prepared.admission.status ==
      phase_offset_navigation::TubeExecutionStatusV2::TRACKING_UNAVAILABLE) {
    return fail(prepared.admission.status,
                prepared.invalid_reason.empty()
                    ? "incumbent tracking contract is unavailable"
                    : prepared.invalid_reason);
  }

  if (recovery_step && runtime_reserve) {
    const phase_offset_navigation::TubeFiniteReserveV2& committed_reserve =
        *runtime_reserve;
    const std::size_t cursor = committed_reserve.cursor;
    if (cursor >= committed_reserve.steps.size()) {
      return fail(
          phase_offset_navigation::TubeExecutionStatusV2::RESERVE_UNAVAILABLE,
          "committed V2 reserve cursor is exhausted");
    }
    const phase_offset_navigation::TubeReserveStepV2& reserve_step =
        committed_reserve.steps[cursor];
    MatchedAdapterInput::TubeV2AdmissionEvidence recovery_evidence = evidence;
    if (!PopulateV2LiveCommandEvidence(
            config_, live_geometry, live_base, reserve_step.command,
            recovery_evidence) ||
        !BitsEqual(live_base.w_dot, reserve_step.base_phase_rate) ||
        !recovery_evidence.tracking.valid ||
        recovery_evidence.tracking.error_norm >
            recovery_evidence.tracking.error_bound) {
      return fail(phase_offset_navigation::TubeExecutionStatusV2::
                      TRACKING_UNAVAILABLE,
                  "incumbent reserve live geometry, rate, or tracking is unavailable");
    }
    const phase_offset_navigation::RuntimeV2PrepareInput runtime_input =
        MakeV2RuntimePrepareInput(input, incumbent, recovery_evidence, true,
                                  runtime_->retainedDelta(),
                                  runtime_->previousFinalPort());
    phase_offset_navigation::CertifiedReservePrepareInputV2 recovery_input;
    recovery_input.recovery_session = incumbent->path_key.execution_generation;
    recovery_input.reserve = committed_reserve;
    recovery_input.expected_identity = recovery_input.reserve.identity;
    recovery_input.cursor = recovery_input.reserve.cursor;
    recovery_input.expected_state.w = input.path.w;
    recovery_input.expected_state.delta = runtime_->retainedDelta();
    recovery_input.expected_state.previous_u = runtime_->previousFinalPort();
    recovery_input.dt = input.dt;
    recovery_input.now = evidence.now;
    recovery_input.deadline = runtime_input.applicability_deadline;
    recovery_input.deadline_valid = runtime_input.applicability_deadline_valid;
    recovery_input.provenance =
        "PhaseOffsetMatchedAdapter/V2/incumbent-reserve";
    if (recovery_owner_.prepareCertifiedReserveV2(
            recovery_input, *recovery_step)) {
      if (recovery_step->reserve_cursor != cursor) {
        return fail(
            phase_offset_navigation::TubeExecutionStatusV2::
                RESERVE_UNAVAILABLE,
            "committed V2 reserve cursor changed during preparation");
      }
      // Materialize the exact post-publication Runtime value while every
      // operation may still fail.  The finite schedule itself is unchanged;
      // only its immutable consumption cursor advances by one step.
      phase_offset_navigation::TubeFiniteReserveV2 advanced_reserve =
          committed_reserve;
      advanced_reserve.cursor = cursor + 1U;
      std::string reserve_reason;
      if (!phase_offset_navigation::TubeExecutionGuardV2::validateReserve(
              advanced_reserve, &reserve_reason)) {
        return fail(
            phase_offset_navigation::TubeExecutionStatusV2::
                RESERVE_UNAVAILABLE,
            reserve_reason.empty()
                ? "advanced V2 reserve is invalid" : reserve_reason);
      }
      phase_offset_navigation::RuntimeV2PreparedStep recovery_prepared;
      recovery_prepared.admission.valid = true;
      recovery_prepared.admission.status =
          phase_offset_navigation::TubeExecutionStatusV2::ADMISSIBLE;
      recovery_prepared.admission.identity = committed_reserve.identity;
      recovery_prepared.admission.current = reserve_step.before;
      recovery_prepared.admission.successor = reserve_step.after;
      recovery_prepared.admission.successor_reserve = advanced_reserve;
      recovery_prepared.admission.live_k.min_width = 0.0;
      recovery_prepared.admission.provenance = recovery_input.provenance;
      recovery_prepared.profile = incumbent;
      recovery_prepared.identity = committed_reserve.identity;
      recovery_prepared.expected_identity = recovery_prepared.identity;
      recovery_prepared.binding_transition = false;
      recovery_prepared.expected_current = reserve_step.before;
      recovery_prepared.successor = reserve_step.after;
      recovery_prepared.selected_u = reserve_step.command;
      recovery_prepared.preview_policy = runtime_input.preview_policy;
      recovery_prepared.limits = runtime_input.limits;
      recovery_prepared.tracking = runtime_input.tracking;
      recovery_prepared.base_phase_rate = reserve_step.base_phase_rate;
      recovery_prepared.phase_rate_lower =
          reserve_step.phase_rate_lower - reserve_step.command.u_w;
      recovery_prepared.phase_rate_upper =
          reserve_step.phase_rate_upper - reserve_step.command.u_w;
      recovery_prepared.horizon_w = incumbent->certified_end;
      recovery_prepared.sample_spacing_w = runtime_input.sample_spacing_w;
      recovery_prepared.upper_u_delta = runtime_input.upper_u_delta;
      recovery_prepared.dt = committed_reserve.dt;
      recovery_prepared.now = runtime_input.now;
      recovery_prepared.applicability_deadline =
          runtime_input.applicability_deadline_valid
              ? runtime_input.applicability_deadline : 0.0;
      recovery_prepared.applicability_deadline_valid =
          runtime_input.applicability_deadline_valid;
      recovery_prepared.max_work = runtime_input.max_work;
      recovery_prepared.provenance = recovery_input.provenance;
      try {
        recovery_prepared.reserve_owner =
            std::make_shared<const phase_offset_navigation::TubeFiniteReserveV2>(
                advanced_reserve);
      } catch (const std::exception&) {
        return fail(
            phase_offset_navigation::TubeExecutionStatusV2::
                RESERVE_UNAVAILABLE,
            "advanced V2 reserve owner could not be materialized");
      }
      recovery_prepared.valid = true;
      if (!runtime_->makeCommitTokenV2(recovery_prepared,
                                       staged.commit_token)) {
        return fail(
            phase_offset_navigation::TubeExecutionStatusV2::
                RESERVE_UNAVAILABLE,
            "V2 recovery Runtime token could not be sealed");
      }
      staged.prepared_step = std::move(recovery_prepared);
      staged.recovery_step = *recovery_step;
      staged.prepared = true;
      staged.nonselecting = false;
    }
  }
  return fail(phase_offset_navigation::TubeExecutionStatusV2::COMMAND_INFEASIBLE,
              !prepared.invalid_reason.empty()
                  ? prepared.invalid_reason
                  : (!normal_reason.empty()
                         ? normal_reason
                         : "incumbent next command or exact successor is unavailable"));
}

bool PhaseOffsetMatchedAdapter::stageV2ShadowBootstrapLocked(
    const MatchedAdapterInput& input,
    const phase_offset_navigation::TubeBuildInputV2& source,
    const std::shared_ptr<const TubeV2ShadowAdmissionCandidate>& candidate) {
  if (config_.mode != PhaseOffsetMatchedMode::MANUAL ||
      config_.observe_only || !runtime_ ||
      pending_v2_shadow_bootstrap_candidate_ ||
      !candidate || !candidate->applicable ||
      !candidate->prepared || candidate->nonselecting ||
      candidate->status != phase_offset_navigation::TubeExecutionStatusV2::ADMISSIBLE ||
      !candidate->profile || !candidate->commit_token.valid ||
      !candidate->commit_token.sealed_prepared ||
      candidate->commit_token.sealed_prepared->profile != candidate->profile ||
      !candidate->proposed_binding ||
      !candidate->proposed_binding->complete() ||
      candidate->proposed_binding->profile != candidate->profile ||
      candidate->proposed_binding->source_input.get() != &source ||
      candidate->request_id != source.request_id ||
      candidate->path_key != source.path_key ||
      candidate->configuration_key != source.configuration_key ||
      candidate->map_capture_key != source.map_capture_key ||
      candidate->execution_generation != source.path_key.execution_generation ||
      source.path_key.execution_generation !=
          task_generation_.load(std::memory_order_acquire) ||
      !source.complete() || failure_latched_) {
    return false;
  }
  const phase_offset_navigation::RuntimeV2PreparedStep& sealed =
      *candidate->commit_token.sealed_prepared;
  if (!sealed.valid ||
      !SameV2ExecutionIdentity(
          sealed.identity, candidate->proposed_binding->identity) ||
      !BitsEqual(runtime_->retainedDelta(), sealed.expected_current.delta) ||
      !BitsEqual(runtime_->previousFinalPort(),
                 sealed.expected_current.previous_u) ||
      !BitsEqual(input.path.w, sealed.expected_current.w)) {
    return false;
  }
  if (v2_execution_binding_) {
    if (!v2_execution_binding_->complete()) return false;
    const bool same_identity = SameV2ExecutionIdentity(
        sealed.identity, v2_execution_binding_->identity);
    if ((same_identity && sealed.binding_transition) ||
        (!same_identity &&
         (!sealed.binding_transition ||
          !SameV2ExecutionIdentity(
              sealed.expected_identity, v2_execution_binding_->identity)))) {
      return false;
    }
    pending_v2_shadow_bootstrap_candidate_ = candidate;
    if (!validatePendingV2ShadowBootstrapLocked(nullptr)) {
      pending_v2_shadow_bootstrap_candidate_.reset();
      return false;
    }
    return true;
  }
  const auto exact_zero = [](const double value) {
    return std::isfinite(value) && value == 0.0;
  };
  if (!exact_zero(runtime_->retainedDelta()) ||
      runtime_->hasExecutedOffsetAuthority()) {
    return false;
  }
  const phase_offset_core::PortCommand previous =
      runtime_->previousFinalPort();
  if (!exact_zero(previous.u_w) || !exact_zero(previous.u_delta)) return false;
  if (execution_authority_.snapshot().valid) {
    return false;
  }
  if (!std::isfinite(input.path.w) ||
      !candidate->prepared_step.expected_current.finite() ||
      candidate->prepared_step.expected_current.w != input.path.w ||
      candidate->prepared_step.expected_current.delta != 0.0 ||
      !exact_zero(candidate->prepared_step.expected_current.previous_u.u_w) ||
      !exact_zero(candidate->prepared_step.expected_current.previous_u.u_delta)) {
    return false;
  }
  pending_v2_shadow_bootstrap_candidate_ = candidate;
  if (!validatePendingV2ShadowBootstrapLocked(nullptr)) {
    pending_v2_shadow_bootstrap_candidate_.reset();
    return false;
  }
  return true;
}

bool PhaseOffsetMatchedAdapter::stageV2ShadowRecoveryLocked(
    const MatchedAdapterInput& input,
    const std::shared_ptr<const TubeV2ShadowAdmissionCandidate>& candidate) {
  if (config_.mode != PhaseOffsetMatchedMode::MANUAL ||
      config_.observe_only || !runtime_ ||
      pending_v2_shadow_bootstrap_candidate_ || !v2_execution_binding_ ||
      !candidate || candidate->profile != v2_execution_binding_->profile ||
      candidate->applicable || candidate->nonselecting ||
      !candidate->prepared || !candidate->recovery_step.valid ||
      candidate->recovery_step.proof_kind !=
          phase_offset_navigation::RecoveryStepProofKind::FINITE_RESERVE_V2 ||
      !candidate->commit_token.valid ||
      !candidate->commit_token.sealed_prepared ||
      candidate->execution_generation !=
          task_generation_.load(std::memory_order_acquire) ||
      !IsFinite(input.path.w) ||
      !BitsEqual(input.path.w, candidate->recovery_step.current_w) ||
      !recovery_owner_.validateCommit(candidate->recovery_step)) {
    return false;
  }
  pending_v2_shadow_bootstrap_candidate_ = candidate;
  if (!validatePendingV2ShadowBootstrapLocked(nullptr)) {
    pending_v2_shadow_bootstrap_candidate_.reset();
    return false;
  }
  return true;
}

bool PhaseOffsetMatchedAdapter::stageV2ShadowSuccessorLocked(
    const MatchedAdapterInput& input,
    const TubeV2SuccessorHandoffEvidence& handoff,
    const std::shared_ptr<const TubeV2ShadowAdmissionCandidate>& candidate) {
  if (!runtime_ || pending_v2_shadow_bootstrap_candidate_ ||
      !v2_execution_binding_ || !v2_execution_binding_->complete() ||
      !candidate || !candidate->applicable || !candidate->prepared ||
      candidate->nonselecting ||
      candidate->purpose != TubeWorkerPurposeV2::SUCCESSOR ||
      !candidate->copied_prefix_continuity_valid ||
      !candidate->proposed_binding ||
      !candidate->proposed_binding->complete() ||
      !candidate->prepared_step.binding_transition ||
      !candidate->commit_token.valid ||
      !candidate->commit_token.sealed_prepared ||
      candidate->commit_token.sealed_prepared->profile != candidate->profile ||
      candidate->proposed_binding->profile != candidate->profile ||
      candidate->proposed_binding->source_input != handoff.successor_request ||
      candidate->source_path_key != handoff.source_path_key ||
      candidate->path_key != handoff.successor_path_key ||
      v2_execution_binding_->profile->path_key != handoff.source_path_key ||
      !SameV2ExecutionIdentity(
          v2_execution_binding_->identity,
          candidate->prepared_step.expected_identity) ||
      !SameV2ExecutionIdentity(
          candidate->proposed_binding->identity,
          candidate->prepared_step.identity) ||
      candidate->execution_generation !=
          task_generation_.load(std::memory_order_acquire) ||
      !BitsEqual(input.path.w,
                 candidate->prepared_step.expected_current.w) ||
      !BitsEqual(runtime_->retainedDelta(),
                 candidate->prepared_step.expected_current.delta) ||
      !BitsEqual(runtime_->previousFinalPort(),
                 candidate->prepared_step.expected_current.previous_u) ||
      failure_latched_) {
    return false;
  }
  pending_v2_shadow_bootstrap_candidate_ = candidate;
  if (!validatePendingV2ShadowBootstrapLocked(nullptr)) {
    pending_v2_shadow_bootstrap_candidate_.reset();
    return false;
  }
  return true;
}

bool PhaseOffsetMatchedAdapter::prepareV2ExecutionAuthorityLocked(
    const MatchedAdapterInput& input,
    const MatchedAdapterOutput& output,
    const phase_offset_navigation::ActiveReferenceOwnerMode owner_mode,
    const std::shared_ptr<const TubeV2ExecutionBinding>& binding,
    TubeV2ShadowAdmissionCandidate& candidate,
    std::string* reason) {
  if (reason) reason->clear();
  const auto fail = [reason](const std::string& text) {
    if (reason) *reason = text;
    return false;
  };
  candidate.authority_prepared =
      phase_offset_navigation::AuthorityPreparedStep();
  if (!runtime_ || !binding || !binding->complete() ||
      !candidate.prepared || !candidate.commit_token.valid ||
      !candidate.commit_token.sealed_prepared ||
      candidate.profile != binding->profile || !output.selected ||
      !output.valid || !output.matched.valid || !output.guidance.valid ||
      !output.base_guidance.valid || !output.geometry.valid) {
    return fail("V2 authority input is incomplete");
  }
  const phase_offset_navigation::RuntimeV2PreparedStep& prepared =
      *candidate.commit_token.sealed_prepared;
  if (!prepared.valid || prepared.profile != binding->profile ||
      !prepared.expected_current.finite() || !prepared.successor.finite() ||
      !BitsEqual(prepared.expected_current.w, output.geometry.w) ||
      !BitsEqual(prepared.expected_current.delta, output.delta) ||
      !BitsEqual(prepared.selected_u, output.projection.final_port) ||
      !BitsEqual(prepared.dt, input.dt)) {
    return fail("V2 authority state or selected command was substituted");
  }

  const std::shared_ptr<const ContinuousPhasePath> path_owner =
      std::static_pointer_cast<const ContinuousPhasePath>(
          binding->source_input->path_owner);
  if (!path_owner || path_owner->empty() ||
      path_owner->pathRevision() != binding->profile->path_key.path_revision) {
    return fail("V2 authority path owner is unavailable");
  }
  const std::uint64_t query_revision = binding->profile->profile_id;
  const std::uint64_t owner_revision =
      binding->identity.binding_sequence != 0U
          ? binding->identity.binding_sequence
          : binding->identity.execution_generation;
  phase_offset_navigation::ImmutableExecutedReferenceQueryPtr query;
  try {
    query = phase_offset_navigation::ImmutableExecutedReferenceQueryPtr(
        new PhaseOffsetExecutedReferenceQuery(
            path_owner, prepared.expected_current.delta,
            binding->frame_owner, binding->profile->path_key.path_revision,
            binding->profile->path_key.frame_revision, owner_revision,
            query_revision));
  } catch (const std::exception&) {
    return fail("V2 immutable executed-reference query allocation failed");
  }
  phase_offset_navigation::ExecutedReferenceQueryResult reference;
  phase_offset_core::NormalFrameQuery frame;
  if (!query ||
      !query->query(prepared.expected_current.w, reference) ||
      !reference.valid ||
      !binding->frame_owner->query(prepared.expected_current.w, frame) ||
      !frame.valid) {
    return fail("V2 immutable executed-reference query failed");
  }

  const phase_offset_navigation::ActiveReferenceSnapshot current =
      execution_authority_.snapshot();
  phase_offset_navigation::ActiveReferenceSnapshot snapshot;
  snapshot.authority_session = current.valid
      ? current.authority_session
      : task_generation_.load(std::memory_order_acquire);
  snapshot.planner_path_revision = input.path.path_revision != 0U
      ? input.path.path_revision : binding->profile->path_key.path_revision;
  snapshot.executed_path_revision = query->pathRevision();
  snapshot.frame_revision = query->frameRevision();
  snapshot.tube_revision = binding->profile->profile_id;
  snapshot.profile_revision = binding->profile->profile_id;
  snapshot.map_revision = binding->profile->map_capture_key.state_id;
  snapshot.obstacle_contract_id =
      "phase_offset_navigation/tube_certificate_v2";
  snapshot.owner_mode = owner_mode;
  snapshot.selected_u_owner = owner_mode ==
          phase_offset_navigation::ActiveReferenceOwnerMode::RECOVERY
      ? "PhaseOffsetRecoveryOwner"
      : phase_offset_navigation::PhaseOffsetAllocator::ownerName();
  snapshot.w = prepared.expected_current.w;
  snapshot.delta = prepared.expected_current.delta;
  snapshot.dt = prepared.dt;
  snapshot.u_prev = prepared.expected_current.previous_u;
  snapshot.selected_u = prepared.selected_u;
  snapshot.selected_u_w = prepared.selected_u.u_w;
  snapshot.selected_u_delta = prepared.selected_u.u_delta;
  snapshot.proposed_next_w = prepared.successor.w;
  snapshot.proposed_next_delta = prepared.successor.delta;
  snapshot.proposed_next_u_prev = prepared.successor.previous_u;
  snapshot.r = reference.r;
  snapshot.r_w = reference.r_w;
  snapshot.r_ww = reference.r_ww;
  snapshot.r_ww_valid = reference.r_ww_valid;
  snapshot.matched_base_v_cmd = output.base_guidance.v_cmd;
  snapshot.matched_base_w_dot = output.base_guidance.w_dot;
  snapshot.executed_N = frame.N;
  snapshot.executed_reference_query = query;
  snapshot.reference_query_revision = query->queryRevision();
  snapshot.provenance = owner_mode ==
          phase_offset_navigation::ActiveReferenceOwnerMode::RECOVERY
      ? "PhaseOffsetMatchedAdapter/V2/recovery"
      : (candidate.purpose == TubeWorkerPurposeV2::SUCCESSOR
             ? "PhaseOffsetMatchedAdapter/V2/successor"
             : "PhaseOffsetMatchedAdapter/V2/normal");
  snapshot.safety_status = owner_mode ==
          phase_offset_navigation::ActiveReferenceOwnerMode::RECOVERY
      ? "SAFETY_PRIORITY" : "SAFE";
  snapshot.handoff_state = candidate.purpose ==
          TubeWorkerPurposeV2::SUCCESSOR
      ? "COPIED_PREFIX_SUCCESSOR" : "V2_BOUND";
  snapshot.valid = true;

  phase_offset_navigation::AuthorityPrepareInput authority_input;
  authority_input.candidate = snapshot;
  if (current.valid) {
    authority_input.expected_authority_session = current.authority_session;
    authority_input.expected_sequence = current.sequence;
  }
  authority_input.matched_output_valid = true;
  authority_input.reference_valid = snapshot.governorViewValid();
  authority_input.provenance = snapshot.provenance;
  if (!execution_authority_.prepare(
          authority_input, candidate.authority_prepared)) {
    return fail(candidate.authority_prepared.failure_reason.empty()
                    ? "V2 execution authority rejected the command"
                    : candidate.authority_prepared.failure_reason);
  }
  return true;
}

bool PhaseOffsetMatchedAdapter::validatePendingV2ShadowBootstrapLocked(
    std::string* reason) const {
  if (reason) reason->clear();
  const auto fail = [reason](const char* text) {
    if (reason) *reason = text;
    return false;
  };
  const std::shared_ptr<const TubeV2ShadowAdmissionCandidate>& pending =
      pending_v2_shadow_bootstrap_candidate_;
  if (!pending) return true;
  const bool recovery = pending->recovery_step.valid &&
      pending->recovery_step.proof_kind ==
          phase_offset_navigation::RecoveryStepProofKind::FINITE_RESERVE_V2;
  const bool successor = pending->purpose == TubeWorkerPurposeV2::SUCCESSOR &&
      pending->prepared_step.binding_transition;
  if (config_.mode != PhaseOffsetMatchedMode::MANUAL ||
      config_.observe_only || !runtime_ ||
      !pending->prepared || !pending->profile ||
      !pending->commit_token.valid || !pending->commit_token.sealed_prepared ||
      pending->commit_token.sealed_prepared->profile != pending->profile) {
    return fail("pending V2 transaction is unavailable");
  }
  if (failure_latched_) {
    return fail("pending V2 transaction was retired by a control failure");
  }
  if (!pending->authority_prepared.valid ||
      !pending->authority_prepared.committed_snapshot ||
      !pending->authority_prepared.committed_snapshot->governorViewValid()) {
    return fail("pending V2 execution authority is unavailable");
  }
  std::string authority_reason;
  if (!execution_authority_.finalValidate(
          pending->authority_prepared, &authority_reason)) {
    return fail("pending V2 execution authority changed");
  }
  if (recovery) {
    const phase_offset_navigation::TubeFiniteReserveV2* committed =
        runtime_->committedV2Reserve();
    const phase_offset_navigation::RecoveryPreparedStep& step =
        pending->recovery_step;
    const phase_offset_navigation::RuntimeV2PreparedStep& sealed =
        *pending->commit_token.sealed_prepared;
    if (pending->nonselecting || pending->applicable ||
        !v2_execution_binding_ ||
        v2_execution_binding_->profile != pending->profile || !committed ||
        step.reserve_id != committed->reserve_id ||
        step.reserve_cursor != committed->cursor ||
        step.reserve_size != committed->steps.size() ||
        step.reserve_cursor >= committed->steps.size() ||
        !step.selectedUExact() || !step.committed_status ||
        !recovery_owner_.validateCommit(step) ||
        !sealed.valid || sealed.profile != pending->profile ||
        !sealed.reserve_owner ||
        !sealed.identity.complete() ||
        sealed.identity.profile_id != pending->profile->profile_id ||
        sealed.identity.execution_generation != pending->execution_generation ||
        sealed.identity.binding_sequence != pending->binding_sequence ||
        !sealed.expected_current.finite() ||
        !sealed.successor.finite() ||
        !BitsEqual(sealed.expected_current.w,
                   committed->steps[committed->cursor].before.w) ||
        !BitsEqual(sealed.expected_current.delta,
                   committed->steps[committed->cursor].before.delta) ||
        !BitsEqual(sealed.expected_current.previous_u,
                   committed->steps[committed->cursor].before.previous_u) ||
        !BitsEqual(sealed.successor.w,
                   committed->steps[committed->cursor].after.w) ||
        !BitsEqual(sealed.successor.delta,
                   committed->steps[committed->cursor].after.delta) ||
        !BitsEqual(sealed.successor.previous_u,
                   committed->steps[committed->cursor].after.previous_u) ||
        !BitsEqual(sealed.selected_u,
                   committed->steps[committed->cursor].command) ||
        !BitsEqual(runtime_->retainedDelta(), step.current_delta) ||
        !BitsEqual(runtime_->previousFinalPort(), step.u_prev) ||
        sealed.reserve_owner->cursor != committed->cursor + 1U ||
        sealed.admission.successor_reserve.cursor != committed->cursor + 1U ||
        sealed.reserve_owner->reserve_id != committed->reserve_id ||
        sealed.reserve_owner->steps.size() != committed->steps.size() ||
        sealed.admission.successor_reserve.reserve_id != committed->reserve_id ||
        sealed.admission.successor_reserve.steps.size() !=
            committed->steps.size()) {
      return fail("pending V2 recovery binding or cursor changed");
    }
    std::string committed_reason;
    std::string advanced_reason;
    if (!phase_offset_navigation::TubeExecutionGuardV2::validateReserve(
            *committed, &committed_reason) ||
        !phase_offset_navigation::TubeExecutionGuardV2::validateReserve(
            *sealed.reserve_owner, &advanced_reason)) {
      return fail("pending V2 recovery reserve is invalid");
    }
    return true;
  }
  if (successor) {
    const phase_offset_navigation::RuntimeV2PreparedStep& sealed =
        *pending->commit_token.sealed_prepared;
    if (pending->nonselecting || !pending->applicable ||
        !pending->copied_prefix_continuity_valid ||
        !pending->proposed_binding ||
        !pending->proposed_binding->complete() ||
        !v2_execution_binding_ || !v2_execution_binding_->complete() ||
        sealed.profile != pending->profile || !sealed.valid ||
        !sealed.binding_transition ||
        !SameV2ExecutionIdentity(
            sealed.expected_identity, v2_execution_binding_->identity) ||
        !SameV2ExecutionIdentity(
            sealed.identity, pending->proposed_binding->identity) ||
        pending->proposed_binding->profile != pending->profile ||
        !BitsEqual(runtime_->retainedDelta(), sealed.expected_current.delta) ||
        !BitsEqual(runtime_->previousFinalPort(),
                   sealed.expected_current.previous_u)) {
      return fail("pending V2 successor binding changed");
    }
    std::shared_ptr<const TubeWorkerCompletionV2> completion;
    {
      std::lock_guard<std::mutex> worker_lock(worker_state_mutex_);
      completion = latest_v2_shadow_successor_completion_;
    }
    if (!completion || !completion->built() ||
        completion->request_id != pending->request_id ||
        completion->path_key != pending->path_key ||
        completion->configuration_key != pending->configuration_key ||
        completion->map_capture_key != pending->map_capture_key ||
        completion->build.profile.profile_id != pending->profile->profile_id) {
      return fail("pending V2 successor completion changed");
    }
    return true;
  }
  const phase_offset_navigation::RuntimeV2PreparedStep& sealed =
      *pending->commit_token.sealed_prepared;
  if (pending->purpose != TubeWorkerPurposeV2::CURRENT ||
      pending->nonselecting || !pending->applicable || !sealed.valid ||
      !pending->proposed_binding ||
      !pending->proposed_binding->complete() ||
      pending->proposed_binding->profile != pending->profile ||
      !SameV2ExecutionIdentity(
          sealed.identity, pending->proposed_binding->identity) ||
      !BitsEqual(runtime_->retainedDelta(), sealed.expected_current.delta) ||
      !BitsEqual(runtime_->previousFinalPort(),
                 sealed.expected_current.previous_u)) {
    return fail("pending V2 NORMAL transaction is unavailable");
  }
  const bool incumbent_tick = v2_execution_binding_ &&
      pending->proposed_binding == v2_execution_binding_;
  if (v2_execution_binding_) {
    if (!v2_execution_binding_->complete()) {
      return fail("pending V2 incumbent binding is incomplete");
    }
    const bool same_identity = SameV2ExecutionIdentity(
        sealed.identity, v2_execution_binding_->identity);
    if ((same_identity && sealed.binding_transition) ||
        (!same_identity &&
         (!sealed.binding_transition ||
          !SameV2ExecutionIdentity(
              sealed.expected_identity, v2_execution_binding_->identity)))) {
      return fail("pending V2 CURRENT replacement binding changed");
    }
  } else if (sealed.binding_transition) {
    return fail("pending V2 first binding unexpectedly replaces an incumbent");
  }
  const auto exact_zero = [](const double value) {
    return std::isfinite(value) && value == 0.0;
  };
  if (!v2_execution_binding_) {
    if (!exact_zero(runtime_->retainedDelta()) ||
        runtime_->hasExecutedOffsetAuthority()) {
      return fail("pending V2 Runtime state is no longer neutral");
    }
    const phase_offset_core::PortCommand previous =
        runtime_->previousFinalPort();
    if (!exact_zero(previous.u_w) || !exact_zero(previous.u_delta)) {
      return fail("pending V2 previous port is no longer neutral");
    }
  }
  if (!pending->prepared_step.expected_current.finite() ||
      !BitsEqual(runtime_->previousFinalPort(),
                 pending->prepared_step.expected_current.previous_u) ||
      !BitsEqual(runtime_->retainedDelta(),
                 pending->prepared_step.expected_current.delta)) {
    return fail("pending V2 prepared current state changed");
  }
  if (!incumbent_tick) {
    const std::shared_ptr<const TubeBuildRequestV2> request =
        std::atomic_load(&latest_build_request_);
    if (!request || !request->active || !request->tube_worker_input_v2 ||
        request->tube_worker_purpose_v2 != TubeWorkerPurposeV2::CURRENT ||
        request->task_generation !=
            task_generation_.load(std::memory_order_acquire) ||
        request->tube_worker_input_v2 !=
            pending->proposed_binding->source_input ||
        request->tube_worker_input_v2->request_id != pending->request_id ||
        request->tube_worker_input_v2->path_key != pending->path_key ||
        request->tube_worker_input_v2->configuration_key !=
            pending->configuration_key ||
        request->tube_worker_input_v2->map_capture_key !=
            pending->map_capture_key ||
        request->tube_worker_input_v2->path_key.execution_generation !=
            pending->execution_generation ||
        pending->prepared_step.expected_current.w != request->current_w) {
      return fail("pending V2 request cohort changed");
    }
    std::shared_ptr<const TubeWorkerCompletionV2> completion;
    {
      std::lock_guard<std::mutex> worker_lock(worker_state_mutex_);
      completion = latest_v2_shadow_current_completion_;
    }
    if (!completion || !completion->built() ||
        completion->request_id != pending->request_id ||
        completion->path_key != pending->path_key ||
        completion->configuration_key != pending->configuration_key ||
        completion->map_capture_key != pending->map_capture_key ||
        completion->build.profile.profile_id != pending->profile->profile_id) {
      return fail("pending V2 completion cohort changed");
    }
  }
  return true;
}

PhaseOffsetMatchedAdapter::~PhaseOffsetMatchedAdapter() {
  shutdown();
}

bool PhaseOffsetMatchedAdapter::configurationValid() const {
  return configuration_valid_;
}

bool PhaseOffsetMatchedAdapter::requiresPathSamples() const { return false; }

double PhaseOffsetMatchedAdapter::sampleStepW() const {
  return config_.preflight_sample_step_w;
}

bool PhaseOffsetMatchedAdapter::requiresTubeTimer() const {
  return config_.mode == PhaseOffsetMatchedMode::MANUAL &&
      configuration_valid_ && static_cast<bool>(v2_shadow_worker_);
}

bool PhaseOffsetMatchedAdapter::requiresAuthoritativeOffsetHandoff() const {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  return requiresAuthoritativeOffsetHandoffLocked();
}

bool PhaseOffsetMatchedAdapter::requiresAuthoritativeOffsetHandoffLocked() const {
  return config_.mode == PhaseOffsetMatchedMode::MANUAL && runtime_ &&
      (static_cast<bool>(v2_execution_binding_) ||
       static_cast<bool>(pending_v2_shadow_bootstrap_candidate_) ||
       runtime_->hasExecutedOffsetAuthority() ||
       std::abs(runtime_->retainedDelta()) >
           kRecoveryNonzeroDeltaTolerance);
}

bool PhaseOffsetMatchedAdapter::requestRecenter() {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  if (!runtime_ || !v2_execution_binding_ ||
      !v2_execution_binding_->complete()) {
    return false;
  }
  runtime_->requestRecenter();
  return true;
}

bool PhaseOffsetMatchedAdapter::recenterRequested() const {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  return runtime_ && runtime_->recenterRequested();
}

void PhaseOffsetMatchedAdapter::requestShutdown() {
  {
    std::lock_guard<std::mutex> publication_lock(task_publication_mutex_);
    shutdown_requested_.store(true, std::memory_order_release);
  }
  {
    std::lock_guard<std::mutex> command_lock(runtime_command_mutex_);
    clearPendingPositionCommandLocked();
    v2_execution_binding_.reset();
    std::atomic_store(&latest_build_request_,
                      std::shared_ptr<const TubeBuildRequestV2>());
  }
  {
    std::lock_guard<std::mutex> worker_lock(worker_state_mutex_);
    worker_stop_requested_ = true;
    pinned_current_cohort_.reset();
    latest_v2_shadow_current_completion_.reset();
    latest_v2_shadow_successor_completion_.reset();
    latest_v2_shadow_admission_candidate_.reset();
    std::atomic_store_explicit(
        &retained_v2_shadow_admission_candidate_,
        std::shared_ptr<const TubeV2ShadowAdmissionCandidate>(),
        std::memory_order_release);
    v2_shadow_last_current_identity_ = V2ShadowRequestIdentity();
    v2_shadow_last_successor_identity_ = V2ShadowRequestIdentity();
    v2_shadow_last_current_path_key_ = phase_offset_navigation::TubePathKey();
    v2_shadow_last_successor_path_key_ = phase_offset_navigation::TubePathKey();
    v2_shadow_have_current_path_key_ = false;
    v2_shadow_have_successor_path_key_ = false;
  }
}

void PhaseOffsetMatchedAdapter::shutdown() {
  requestShutdown();
  joinTubeWorker();
  std::lock_guard<std::mutex> command_lock(runtime_command_mutex_);
  clearPendingPositionCommandLocked();
  v2_execution_binding_.reset();
  active_diagnostics_pub_ = ros::Publisher();
  {
    std::lock_guard<std::mutex> marker_lock(marker_publication_mutex_);
    manual_tube_pub_ = ros::Publisher();
    manual_tube_candidate_pub_ = ros::Publisher();
    published_active_marker_identity_ = TubeMarkerIdentityV2();
    published_candidate_marker_identity_ = TubeMarkerIdentityV2();
    marker_snapshot_published_ = false;
  }
}

bool PhaseOffsetMatchedAdapter::resetForNewNavigationTask() {
  std::lock_guard<std::mutex> publication_lock(task_publication_mutex_);
  std::lock_guard<std::mutex> command_lock(runtime_command_mutex_);
  if (shutdown_requested_.load(std::memory_order_acquire)) return false;

  const std::uint64_t next_generation =
      task_generation_.fetch_add(1U, std::memory_order_acq_rel) + 1U;
  {
    std::lock_guard<std::mutex> worker_lock(worker_state_mutex_);
    if (v2_shadow_worker_) v2_shadow_worker_->reset(next_generation);
    pinned_current_cohort_.reset();
    latest_v2_shadow_current_completion_.reset();
    latest_v2_shadow_successor_completion_.reset();
    latest_v2_shadow_admission_candidate_.reset();
    std::atomic_store_explicit(
        &retained_v2_shadow_admission_candidate_,
        std::shared_ptr<const TubeV2ShadowAdmissionCandidate>(),
        std::memory_order_release);
    v2_shadow_last_current_identity_ = V2ShadowRequestIdentity();
    v2_shadow_last_successor_identity_ = V2ShadowRequestIdentity();
    v2_shadow_last_current_path_key_ = phase_offset_navigation::TubePathKey();
    v2_shadow_last_successor_path_key_ = phase_offset_navigation::TubePathKey();
    v2_shadow_have_current_path_key_ = false;
    v2_shadow_have_successor_path_key_ = false;
  }
  clearPendingPositionCommandLocked();
  v2_execution_binding_.reset();
  std::atomic_store(&latest_build_request_,
                    std::shared_ptr<const TubeBuildRequestV2>());
  control_sequence_ = 0U;
  command_active_ = false;
  zero_gate_consecutive_count_ = 0;
  zero_gate_open_ = false;
  failure_latched_ = false;
  control_failure_reason_ =
      phase_offset_navigation::ControlFailureReason::NONE;
  if (runtime_) runtime_->resetForNewNavigationTask();
  recovery_owner_.reset(next_generation);
  execution_authority_.resetForNewTask(next_generation);
  return true;
}

void PhaseOffsetMatchedAdapter::deactivate(const ros::Time& stamp) {
  const std::uint64_t generation =
      task_generation_.load(std::memory_order_acquire);
  std::lock_guard<std::mutex> publication_lock(task_publication_mutex_);
  std::lock_guard<std::mutex> command_lock(runtime_command_mutex_);
  deactivateLocked(stamp, generation);
}

void PhaseOffsetMatchedAdapter::deactivateLocked(
    const ros::Time&, const std::uint64_t expected_task_generation) {
  if (shutdown_requested_.load(std::memory_order_acquire) ||
      expected_task_generation !=
          task_generation_.load(std::memory_order_acquire)) {
    return;
  }
  clearPendingPositionCommandLocked();
  if (RuntimeExecutionExactlyNeutral(runtime_.get())) {
    v2_execution_binding_.reset();
  }
  command_active_ = false;
  std::shared_ptr<TubeBuildRequestV2> request(new TubeBuildRequestV2());
  request->active = false;
  request->task_generation = expected_task_generation;
  request->control_sequence = ++control_sequence_;
  std::atomic_store(
      &latest_build_request_,
      std::shared_ptr<const TubeBuildRequestV2>(std::move(request)));
  std::lock_guard<std::mutex> worker_lock(worker_state_mutex_);
  if (v2_shadow_worker_) {
    v2_shadow_worker_->cancel(TubeWorkerPurposeV2::CURRENT);
    v2_shadow_worker_->cancel(TubeWorkerPurposeV2::SUCCESSOR);
  }
  pinned_current_cohort_.reset();
  latest_v2_shadow_current_completion_.reset();
  latest_v2_shadow_successor_completion_.reset();
}

bool PhaseOffsetMatchedAdapter::prepareDeactivationLocked(
    const ros::Time&, const std::uint64_t expected_task_generation,
    DeactivationCommitToken& token) const {
  token = DeactivationCommitToken();
  if (shutdown_requested_.load(std::memory_order_acquire) ||
      expected_task_generation !=
          task_generation_.load(std::memory_order_acquire)) {
    return false;
  }
  token.task_generation = expected_task_generation;
  token.next_control_sequence = control_sequence_ + 1U;
  token.command_active_before = command_active_;
  if (command_active_) {
    try {
      std::shared_ptr<TubeBuildRequestV2> request(new TubeBuildRequestV2());
      request->active = false;
      request->task_generation = expected_task_generation;
      request->control_sequence = token.next_control_sequence;
      token.inactive_request =
          std::shared_ptr<const TubeBuildRequestV2>(std::move(request));
    } catch (const std::bad_alloc&) {
      return false;
    }
  }
  token.valid = true;
  return true;
}

void PhaseOffsetMatchedAdapter::commitDeactivationNoFailLocked(
    const DeactivationCommitToken& token) noexcept {
  clearPendingPositionCommandLocked();
  if (RuntimeExecutionExactlyNeutral(runtime_.get())) {
    v2_execution_binding_.reset();
  }
  if (!token.command_active_before) return;
  command_active_ = false;
  control_sequence_ = token.next_control_sequence;
  std::atomic_store(&latest_build_request_, token.inactive_request);
}

void PhaseOffsetMatchedAdapter::advertise(ros::NodeHandle& nh) {
  if (!configuration_valid_ ||
      shutdown_requested_.load(std::memory_order_acquire)) {
    return;
  }
  if (config_.mode == PhaseOffsetMatchedMode::ACTIVE) {
    if (!active_diagnostics_pub_) {
      active_diagnostics_pub_ =
          nh.advertise<std_msgs::Float64MultiArray>(
              "phase_offset_active/diagnostics", 1);
    }
    return;
  }
  {
    std::lock_guard<std::mutex> marker_lock(marker_publication_mutex_);
    if (!manual_tube_pub_) {
      manual_tube_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
          "phase_offset_manual/tube", 1, true);
    }
    if (!manual_tube_candidate_pub_) {
      manual_tube_candidate_pub_ =
          nh.advertise<visualization_msgs::MarkerArray>(
              "phase_offset_manual/tube_candidate", 1, true);
    }
    marker_snapshot_published_ = false;
  }
  std::lock_guard<std::mutex> worker_lock(worker_state_mutex_);
  worker_stop_requested_ = false;
  worker_started_ = static_cast<bool>(v2_shadow_worker_);
}

std::shared_ptr<const TubeBuildRequestV2>
PhaseOffsetMatchedAdapter::makeBuildRequestV2(
    const MatchedAdapterInput& input) {
  std::lock_guard<std::mutex> worker_lock(worker_state_mutex_);
  std::shared_ptr<TubeBuildRequestV2> request(new TubeBuildRequestV2());
  request->active = true;
  request->task_generation =
      task_generation_.load(std::memory_order_acquire);
  request->control_sequence = ++control_sequence_;
  request->current_w = input.path.w;
  request->tube_worker_input_v2 = input.tube_worker_input_v2;
  request->tube_worker_purpose_v2 = input.tube_worker_purpose_v2;
  if (pinned_current_cohort_) {
    if (v2_shadow_worker_ && !v2_shadow_worker_->running()) {
      const auto stats = v2_shadow_worker_->stats();
      const bool delivered = latest_v2_shadow_current_completion_ &&
          latest_v2_shadow_current_completion_->request_id ==
              pinned_current_cohort_->tube_worker_input_v2->request_id;
      if (stats.pending_current == 0U && stats.retained_current == 0U && !delivered)
        pinned_current_cohort_.reset();  // Cancelled/discarded job has no result.
    }
  }
  if (pinned_current_cohort_) {
    const auto& pinned = pinned_current_cohort_->tube_worker_input_v2;
    const auto& incoming = input.tube_worker_input_v2;
    if (pinned_current_cohort_->task_generation != request->task_generation ||
        (incoming && incoming->path_key != pinned->path_key)) {
      if (v2_shadow_worker_) v2_shadow_worker_->invalidatePath(pinned->path_key);
      pinned_current_cohort_.reset();
      latest_v2_shadow_current_completion_.reset();
    } else if (input.tube_worker_purpose_v2 == TubeWorkerPurposeV2::CURRENT) {
      request->tube_worker_input_v2 = pinned;
    }
  }
  std::atomic_store(&latest_build_request_,
      std::shared_ptr<const TubeBuildRequestV2>(request));
  command_active_ = true;
  return std::shared_ptr<const TubeBuildRequestV2>(std::move(request));
}

void PhaseOffsetMatchedAdapter::releaseCurrentCohort(const std::uint64_t request_id) {
  std::lock_guard<std::mutex> lock(worker_state_mutex_);
  if (pinned_current_cohort_ && pinned_current_cohort_->tube_worker_input_v2 &&
      pinned_current_cohort_->tube_worker_input_v2->request_id == request_id)
    pinned_current_cohort_.reset();
}

void PhaseOffsetMatchedAdapter::joinTubeWorker() {
  std::lock_guard<std::mutex> lock(worker_state_mutex_);
  if (v2_shadow_worker_) v2_shadow_worker_->shutdown();
  latest_v2_shadow_current_completion_.reset();
  latest_v2_shadow_successor_completion_.reset();
  v2_shadow_last_current_identity_ = V2ShadowRequestIdentity();
  v2_shadow_last_successor_identity_ = V2ShadowRequestIdentity();
  v2_shadow_last_current_path_key_ =
      phase_offset_navigation::TubePathKey();
  v2_shadow_last_successor_path_key_ =
      phase_offset_navigation::TubePathKey();
  v2_shadow_have_current_path_key_ = false;
  v2_shadow_have_successor_path_key_ = false;
  worker_started_ = false;
}
bool PhaseOffsetMatchedAdapter::scheduleTubeBuild() {
  const bool scheduled = scheduleV2ShadowBuild();
  // Direct deterministic fixtures intentionally construct the adapter
  // without starting ROS.  Marker publication is a production observation,
  // so do not introduce a ros::Time global-state requirement into the worker
  // scheduler when no NodeHandle can have advertised these topics.
  bool markers_advertised = false;
  {
    std::lock_guard<std::mutex> marker_lock(marker_publication_mutex_);
    markers_advertised = manual_tube_pub_ && manual_tube_candidate_pub_;
  }
  if (markers_advertised) {
    publishV2TubeMarkers(ros::Time::now());
  }
  return scheduled;
}
std::uint64_t PhaseOffsetMatchedAdapter::executionGenerationV2() const {
  return shutdown_requested_.load(std::memory_order_acquire)
      ? 0U : task_generation_.load(std::memory_order_acquire);
}
bool PhaseOffsetMatchedAdapter::captureV2SuccessorSource(
    phase_offset_navigation::TubePathKey& source_path_key,
    std::uint64_t& execution_generation,
    std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>&
        source_input) const {
  source_path_key = phase_offset_navigation::TubePathKey();
  execution_generation = 0U;
  source_input.reset();
  std::lock_guard<std::mutex> command_lock(runtime_command_mutex_);
  if (config_.mode != PhaseOffsetMatchedMode::MANUAL || !v2_shadow_worker_ ||
      shutdown_requested_.load(std::memory_order_acquire) ||
      !v2_execution_binding_ || !v2_execution_binding_->complete()) {
    return false;
  }
  const std::uint64_t generation =
      task_generation_.load(std::memory_order_acquire);
  if (v2_execution_binding_->profile->path_key.execution_generation !=
      generation) {
    return false;
  }
  source_path_key = v2_execution_binding_->profile->path_key;
  execution_generation = generation;
  source_input = v2_execution_binding_->source_input;
  return true;
}

std::shared_ptr<const TubeV2ExecutionBinding>
PhaseOffsetMatchedAdapter::captureV2ExecutionBinding() const {
  std::lock_guard<std::mutex> command_lock(runtime_command_mutex_);
  return v2_execution_binding_ && v2_execution_binding_->complete()
      ? v2_execution_binding_
      : std::shared_ptr<const TubeV2ExecutionBinding>();
}

bool PhaseOffsetMatchedAdapter::enqueueV2SuccessorRequest(
    const phase_offset_navigation::TubePathKey& source_path_key,
    const phase_offset_navigation::TubeBuildInputV2& successor_input) {
  if (config_.mode != PhaseOffsetMatchedMode::MANUAL || !v2_shadow_worker_ ||
      shutdown_requested_.load(std::memory_order_acquire) ||
      !source_path_key.complete() || !successor_input.complete() ||
      successor_input.path_key == source_path_key ||
      successor_input.configuration_key !=
          config_.tube_certificate_v2.key()) {
    return false;
  }

  // Match update()/reset lock ordering: Runtime/source ownership first,
  // then bounded worker transport.  No path/map callback or builder executes
  // under either adapter mutex.
  std::lock_guard<std::mutex> command_lock(runtime_command_mutex_);
  const std::uint64_t generation =
      task_generation_.load(std::memory_order_acquire);
  if (successor_input.path_key.execution_generation != generation ||
      source_path_key.execution_generation != generation ||
      !v2_execution_binding_ || !v2_execution_binding_->complete() ||
      v2_execution_binding_->profile->path_key != source_path_key ||
      v2_execution_binding_->profile->configuration_key !=
          successor_input.configuration_key) {
    return false;
  }

  std::lock_guard<std::mutex> worker_lock(worker_state_mutex_);
  if (worker_stop_requested_ ||
      shutdown_requested_.load(std::memory_order_acquire)) {
    return false;
  }
  TubeWorkerCompletionV2 delivered;
  while (v2_shadow_worker_->tryTakeAny(delivered)) {
    retainV2ShadowCompletionLocked(delivered);
    delivered = TubeWorkerCompletionV2();
  }

  const V2ShadowRequestIdentity identity = makeV2ShadowRequestIdentity(
      TubeWorkerPurposeV2::SUCCESSOR, successor_input);
  if (!identity.valid() ||
      (v2_shadow_last_successor_identity_.valid() &&
       v2_shadow_last_successor_identity_ == identity)) {
    return false;
  }
  if (v2_shadow_have_successor_path_key_ &&
      v2_shadow_last_successor_path_key_ != successor_input.path_key) {
    // Purpose-local cancellation cannot disturb a CURRENT refresh.  The
    // worker's global invalidatePath primitive is intentionally not used at
    // this copied-prefix enqueue seam.
    v2_shadow_worker_->cancel(TubeWorkerPurposeV2::SUCCESSOR);
    latest_v2_shadow_successor_completion_.reset();
  }

  TubeWorkerRequestV2 request;
  request.purpose = TubeWorkerPurposeV2::SUCCESSOR;
  request.request_id = successor_input.request_id;
  request.execution_generation = generation;
  request.accepted_state_demand =
      successor_input.map_capture_key.accepted_sequence;
  request.useful_start = successor_input.requested_start;
  request.useful_end = successor_input.requested_end;
  request.input = successor_input;
  const bool accepted = v2_shadow_worker_->submit(request);
  v2_shadow_last_successor_identity_ = identity;
  v2_shadow_last_successor_path_key_ = successor_input.path_key;
  v2_shadow_have_successor_path_key_ = true;
  while (v2_shadow_worker_->tryTakeAny(delivered)) {
    retainV2ShadowCompletionLocked(delivered);
    delivered = TubeWorkerCompletionV2();
  }
  return accepted;
}

void PhaseOffsetMatchedAdapter::consumeV2ShadowCompletions() {
  if (!v2_shadow_worker_) return;
  std::lock_guard<std::mutex> lock(worker_state_mutex_);
  if (shutdown_requested_.load(std::memory_order_acquire) ||
      worker_stop_requested_) return;
  TubeWorkerCompletionV2 completion;
  // The worker retains at most one useful completion per purpose.  Drain both
  // bounded slots without touching Runtime, Pair, epoch, or publication
  // state; this channel is observational evidence only.
  while (v2_shadow_worker_->tryTakeAny(completion)) {
    retainV2ShadowCompletionLocked(completion);
    completion = TubeWorkerCompletionV2();
  }
}

bool PhaseOffsetMatchedAdapter::scheduleV2ShadowBuild() {
  if (config_.mode != PhaseOffsetMatchedMode::MANUAL || !v2_shadow_worker_ ||
      shutdown_requested_.load(std::memory_order_acquire)) {
    return false;
  }
  std::lock_guard<std::mutex> lock(worker_state_mutex_);
  // Consume both worker purpose slots while holding the adapter worker-state
  // mutex.  Completion delivery, dedup identity and path retirement therefore
  // have one linearization domain with reset/shutdown/join.
  if (shutdown_requested_.load(std::memory_order_acquire) ||
      worker_stop_requested_) return false;
  TubeWorkerCompletionV2 delivered;
  while (v2_shadow_worker_->tryTakeAny(delivered)) {
    retainV2ShadowCompletionLocked(delivered);
    delivered = TubeWorkerCompletionV2();
  }
  const std::shared_ptr<const TubeBuildRequestV2> request =
      std::atomic_load(&latest_build_request_);
  if (!request || !request->active || !request->tube_worker_input_v2 ||
      request->task_generation !=
          task_generation_.load(std::memory_order_acquire)) {
    return false;
  }
  const phase_offset_navigation::TubeBuildInputV2& source =
      *request->tube_worker_input_v2;
  const V2ShadowRequestIdentity identity = makeV2ShadowRequestIdentity(
      request->tube_worker_purpose_v2, source);
  V2ShadowRequestIdentity& last_identity =
      request->tube_worker_purpose_v2 == TubeWorkerPurposeV2::CURRENT
      ? v2_shadow_last_current_identity_ : v2_shadow_last_successor_identity_;
  std::shared_ptr<const TubeWorkerCompletionV2>& retained_completion =
      request->tube_worker_purpose_v2 == TubeWorkerPurposeV2::CURRENT
      ? latest_v2_shadow_current_completion_
      : latest_v2_shadow_successor_completion_;
  phase_offset_navigation::TubePathKey& last_path_key =
      request->tube_worker_purpose_v2 == TubeWorkerPurposeV2::CURRENT
      ? v2_shadow_last_current_path_key_ : v2_shadow_last_successor_path_key_;
  bool& have_path_key = request->tube_worker_purpose_v2 ==
      TubeWorkerPurposeV2::CURRENT ? v2_shadow_have_current_path_key_
                                   : v2_shadow_have_successor_path_key_;
  if (last_identity.valid() && last_identity == identity) {
    return false;
  }
  // The supplied input is one immutable coherent capture.  Reject a stale
  // generation rather than relabelling it with the adapter's current task.
  if (source.path_key.execution_generation != request->task_generation ||
      source.request_id == 0U || !source.complete()) {
    last_identity = identity;
    return false;
  }
  // A consumed successful completion remains a bounded retention record at
  // the adapter transport boundary.  Preserve it across delayed command/map
  // notifications so a narrower request for the exact same authority does
  // not recertify.  Failed completions deliberately do not suppress retry.
  if (retained_completion) {
    const TubeWorkerCompletionV2& retained = *retained_completion;
    const bool retained_covers = retained.built() &&
        retained.purpose == request->tube_worker_purpose_v2 &&
        retained.execution_generation == source.path_key.execution_generation &&
        retained.path_key == source.path_key &&
        retained.configuration_key == source.configuration_key &&
        retained.map_capture_key == source.map_capture_key &&
        retained.accepted_state_demand >=
            source.map_capture_key.accepted_sequence &&
        retained.useful_start <= source.requested_start &&
        retained.useful_end >= source.requested_end;
    if (retained_covers) {
      last_identity = identity;
      return false;
    }
  }
  const bool incoming_path_replaced = have_path_key &&
      last_path_key != source.path_key;
  if (incoming_path_replaced) {
    // Retire only the incoming purpose's old path.  The worker has one global
    // path-invalidation primitive, so do not call it when the other purpose
    // still uses that exact key; otherwise a successor request could cancel
    // a useful current request (or vice versa).
    const phase_offset_navigation::TubePathKey retired_key = last_path_key;
    const bool other_purpose_uses_key =
        (request->tube_worker_purpose_v2 == TubeWorkerPurposeV2::CURRENT)
        ? (v2_shadow_have_successor_path_key_ &&
           v2_shadow_last_successor_path_key_ == retired_key)
        : (v2_shadow_have_current_path_key_ &&
           v2_shadow_last_current_path_key_ == retired_key);
    if (!other_purpose_uses_key) {
      v2_shadow_worker_->invalidatePath(retired_key);
    } else {
      // invalidatePath() is intentionally global to both worker purpose
      // slots, so it cannot be used while the other purpose still owns this
      // path key.  Retire only the incoming purpose's old pending/running
      // request instead; the other purpose remains eligible and retained.
      v2_shadow_worker_->cancel(request->tube_worker_purpose_v2);
    }
    if (request->tube_worker_purpose_v2 == TubeWorkerPurposeV2::CURRENT) {
      if (latest_v2_shadow_current_completion_ &&
          latest_v2_shadow_current_completion_->path_key == retired_key) {
        latest_v2_shadow_current_completion_.reset();
      }
      v2_shadow_last_current_identity_ = V2ShadowRequestIdentity();
      v2_shadow_have_current_path_key_ = false;
      // Retire only a binding that still belongs to the old path.  A command
      // update can prepare the new path while the scheduler is retiring the
      // previous worker key; compare-exchange prevents that newer binding
      // from being cleared by this old-path lifecycle event.
      std::shared_ptr<const TubeV2ShadowAdmissionCandidate> retired_admission =
          std::atomic_load_explicit(
              &retained_v2_shadow_admission_candidate_,
              std::memory_order_acquire);
      if (retired_admission && retired_admission->path_key == retired_key) {
        std::atomic_compare_exchange_strong_explicit(
            &retained_v2_shadow_admission_candidate_, &retired_admission,
            std::shared_ptr<const TubeV2ShadowAdmissionCandidate>(),
            std::memory_order_acq_rel, std::memory_order_acquire);
      }
    } else {
      if (latest_v2_shadow_successor_completion_ &&
          latest_v2_shadow_successor_completion_->path_key == retired_key) {
        latest_v2_shadow_successor_completion_.reset();
      }
      v2_shadow_last_successor_identity_ = V2ShadowRequestIdentity();
      v2_shadow_have_successor_path_key_ = false;
    }
  }
  TubeWorkerRequestV2 worker_request;
  worker_request.purpose = request->tube_worker_purpose_v2;
  worker_request.request_id = source.request_id;
  worker_request.execution_generation = source.path_key.execution_generation;
  worker_request.accepted_state_demand = source.map_capture_key.accepted_sequence;
  worker_request.useful_start = source.requested_start;
  worker_request.useful_end = source.requested_end;
  worker_request.input = source;
  const bool accepted = v2_shadow_worker_->submit(worker_request);
  if (accepted && worker_request.purpose == TubeWorkerPurposeV2::CURRENT)
    pinned_current_cohort_ = request;
  // Mark the immutable request identity even when worker-level high-water or
  // coverage dedup rejects it, preventing repeated timer permits from
  // creating a second transport attempt.
  last_identity = identity;
  last_path_key = source.path_key;
  have_path_key = true;
  while (v2_shadow_worker_->tryTakeAny(delivered)) {
    retainV2ShadowCompletionLocked(delivered);
    delivered = TubeWorkerCompletionV2();
  }
  return accepted;
}

bool PhaseOffsetMatchedAdapter::timerTick() {
  return scheduleTubeBuild();
}

bool PhaseOffsetMatchedAdapter::candidateMarkerApplicableV2(
    const TubeWorkerCompletionV2& completion, const TubeBuildRequestV2& request,
    const std::uint64_t generation, const std::uint64_t now_ticks) {
  const auto& profile = completion.build.profile;
  return completion.built() && request.active && request.tube_worker_input_v2 &&
      request.task_generation == generation && completion.execution_generation == generation &&
      profile.path_key == request.tube_worker_input_v2->path_key &&
      profile.configuration_key == request.tube_worker_input_v2->configuration_key &&
      profile.map_capture_key.map_instance_id == request.tube_worker_input_v2->map_capture_key.map_instance_id &&
      profile.map_capture_key.configuration_generation == request.tube_worker_input_v2->map_capture_key.configuration_generation &&
      profile.map_capture_key.configuration_id == request.tube_worker_input_v2->map_capture_key.configuration_id &&
      profile.map_capture_key.frame_provenance == request.tube_worker_input_v2->map_capture_key.frame_provenance &&
      std::isfinite(request.current_w) && request.current_w >= profile.certified_start &&
      request.current_w < profile.certified_end &&
      (profile.applicability_deadline_timeless || now_ticks <= profile.applicability_deadline_ticks);
}

void PhaseOffsetMatchedAdapter::publishV2TubeMarkers(
    const ros::Time& stamp) {
  std::shared_ptr<const TubeV2ExecutionBinding> active_binding;
  {
    std::lock_guard<std::mutex> command_lock(runtime_command_mutex_);
    if (v2_execution_binding_ && v2_execution_binding_->complete()) {
      active_binding = v2_execution_binding_;
    }
  }

  std::shared_ptr<const TubeWorkerCompletionV2> current_completion;
  std::shared_ptr<const TubeWorkerCompletionV2> successor_completion;
  {
    std::lock_guard<std::mutex> worker_lock(worker_state_mutex_);
    current_completion = latest_v2_shadow_current_completion_;
    successor_completion = latest_v2_shadow_successor_completion_;
  }

  const auto same_as_active = [&active_binding](
      const std::shared_ptr<const TubeWorkerCompletionV2>& completion) {
    if (!active_binding || !completion || !completion->built()) return false;
    const phase_offset_navigation::TubeProfileV2& profile =
        completion->build.profile;
    return profile.profile_id == active_binding->profile->profile_id &&
        profile.request_id == active_binding->profile->request_id &&
        profile.path_key == active_binding->profile->path_key &&
        profile.configuration_key == active_binding->profile->configuration_key &&
        profile.map_capture_key == active_binding->profile->map_capture_key;
  };
  const auto display_request = std::atomic_load(&latest_build_request_);
  const auto generation = task_generation_.load(std::memory_order_acquire);
  const std::uint64_t now_ticks = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
  if (active_binding && (!display_request || !display_request->active ||
      display_request->task_generation != generation ||
      active_binding->profile->path_key.execution_generation != generation ||
      !std::isfinite(display_request->current_w) ||
      display_request->current_w < active_binding->profile->certified_start ||
      display_request->current_w >= active_binding->profile->certified_end ||
      (!active_binding->profile->applicability_deadline_timeless &&
       now_ticks > active_binding->profile->applicability_deadline_ticks))) {
    active_binding.reset();  // Display only; never changes the execution binding.
  }
  const auto eligible = [&](
      const std::shared_ptr<const TubeWorkerCompletionV2>& completion) {
    return completion && display_request && !same_as_active(completion) &&
        candidateMarkerApplicableV2(*completion, *display_request, generation, now_ticks);
  };
  std::shared_ptr<const TubeWorkerCompletionV2> candidate_completion;
  if (eligible(current_completion)) candidate_completion = current_completion;
  if (eligible(successor_completion) &&
      (!candidate_completion ||
       successor_completion->build_end_ticks >
           candidate_completion->build_end_ticks ||
       (successor_completion->build_end_ticks ==
            candidate_completion->build_end_ticks &&
        successor_completion->request_id > candidate_completion->request_id))) {
    candidate_completion = successor_completion;
  }

  std::shared_ptr<const phase_offset_navigation::TubeProfileV2> active_profile;
  std::shared_ptr<const ContinuousPhaseNormalFrame> active_frame;
  if (active_binding) {
    active_profile = active_binding->profile;
    active_frame = active_binding->frame_owner;
  }
  std::shared_ptr<const phase_offset_navigation::TubeProfileV2>
      candidate_profile;
  std::shared_ptr<const ContinuousPhaseNormalFrame> candidate_frame;
  if (candidate_completion) {
    candidate_profile =
        std::shared_ptr<const phase_offset_navigation::TubeProfileV2>(
            candidate_completion, &candidate_completion->build.profile);
    try {
      const std::shared_ptr<const ContinuousPhasePath> path_owner =
          std::static_pointer_cast<const ContinuousPhasePath>(
              candidate_profile->path_owner);
      if (path_owner && !path_owner->empty() &&
          path_owner->pathRevision() ==
              candidate_profile->path_key.path_revision &&
          path_owner->startW() == candidate_profile->path_key.domain_start &&
          path_owner->endW() == candidate_profile->path_key.domain_end) {
        candidate_frame =
            std::make_shared<const ContinuousPhaseNormalFrame>(
                path_owner, candidate_profile->path_key.path_revision,
                candidate_profile->path_key.frame_revision);
      }
    } catch (const std::exception&) {
      candidate_profile.reset();
      candidate_frame.reset();
    }
  }

  const auto marker_identity = [](
      const std::shared_ptr<const phase_offset_navigation::TubeProfileV2>&
          profile) {
    TubeMarkerIdentityV2 identity;
    if (profile) {
      identity.execution_generation = profile->path_key.execution_generation;
      identity.path_instance_id = profile->path_key.path_instance_id;
      identity.profile_id = profile->profile_id;
      identity.request_id = profile->request_id;
      identity.map_state_id = profile->map_capture_key.state_id;
      identity.valid = true;
    }
    return identity;
  };
  const TubeMarkerIdentityV2 active_identity = marker_identity(active_profile);
  const TubeMarkerIdentityV2 candidate_identity =
      marker_identity(candidate_profile);
  // A short visualization-only lifetime also clears the last latched ADD if
  // the publisher stops. Refresh cached geometry at the existing Tube timer;
  // this neither rebuilds a proof nor grants execution authority.
  const auto publish_cached = [&]() {
    const auto trim_passed_samples = [&](visualization_msgs::MarkerArray& array,
        const std::shared_ptr<const phase_offset_navigation::TubeProfileV2>& profile,
        std::size_t& first) {
      if (!profile || !display_request || profile->knots.size() < 2U) return;
      const auto next = std::upper_bound(profile->knots.begin(), profile->knots.end(),
          display_request->current_w, [](double w, const phase_offset_navigation::TubePwlKnotV2& knot) {
            return w < knot.w;
          });
      if (next == profile->knots.begin()) return;
      const std::size_t desired = std::min(profile->knots.size() - 2U,
          static_cast<std::size_t>(next - profile->knots.begin() - 1));
      if (desired <= first) return;
      // Keep the cell containing live w and all remaining certified cells.
      // Erase only old display vertices/triangles: no new geometry queries,
      // no proof/profile mutation, and no extrapolation beyond certification.
      for (auto& marker : array.markers) {
        if (marker.action != visualization_msgs::Marker::ADD) continue;
        const std::size_t per_cell = marker.type == visualization_msgs::Marker::TRIANGLE_LIST ? 6U : 1U;
        const std::size_t removed = std::min(marker.points.size(), per_cell * (desired - first));
        marker.points.erase(marker.points.begin(), marker.points.begin() + removed);
      }
      first = desired;
    };
    trim_passed_samples(cached_active_markers_v2_, active_profile, cached_active_first_knot_v2_);
    trim_passed_samples(cached_candidate_markers_v2_, candidate_profile, cached_candidate_first_knot_v2_);
    for (auto* array : {&cached_active_markers_v2_, &cached_candidate_markers_v2_}) {
      for (auto& marker : array->markers) {
        marker.header.stamp = stamp;
        if (marker.action == visualization_msgs::Marker::ADD)
          marker.lifetime = ros::Duration(2.0 * config_.tube_update_period);
      }
    }
    if (manual_tube_pub_) manual_tube_pub_.publish(cached_active_markers_v2_);
    if (manual_tube_candidate_pub_) manual_tube_candidate_pub_.publish(cached_candidate_markers_v2_);
  };
  {
    std::lock_guard<std::mutex> marker_lock(marker_publication_mutex_);
    if (shutdown_requested_.load(std::memory_order_acquire) ||
        generation != task_generation_.load(std::memory_order_acquire)) return;
    if (marker_snapshot_published_ &&
        published_active_marker_identity_ == active_identity &&
        published_candidate_marker_identity_ == candidate_identity) {
      publish_cached();
      return;
    }
  }

  visualization_msgs::MarkerArray active_markers;
  visualization_msgs::MarkerArray candidate_markers;
  try {
    active_markers = MakeCertifiedTubeMarkersV2(
        stamp, config_.frame_id, active_profile, active_frame);
    candidate_markers = MakeCandidateTubeMarkersV2(
        stamp, config_.frame_id, candidate_profile, candidate_frame);
  } catch (const std::exception&) {
    // Allocation/serialization preparation failure is visualization-only.
    // Keep the previous identity so the next Tube timer retries; execution
    // authority and the 50 Hz command path remain untouched.
    return;
  }

  std::lock_guard<std::mutex> marker_lock(marker_publication_mutex_);
  if (shutdown_requested_.load(std::memory_order_acquire) ||
      generation != task_generation_.load(std::memory_order_acquire)) {
    return;
  }
  cached_active_markers_v2_ = std::move(active_markers);
  cached_candidate_markers_v2_ = std::move(candidate_markers);
  cached_active_first_knot_v2_ = cached_candidate_first_knot_v2_ = 0U;
  publish_cached();
  published_active_marker_identity_ = active_identity;
  published_candidate_marker_identity_ = candidate_identity;
  marker_snapshot_published_ = true;
}
void PhaseOffsetMatchedAdapter::latchFailure(
    const phase_offset_navigation::ControlFailureReason reason) {
  if (!failure_latched_) {
    control_failure_reason_ = reason;
  }
  failure_latched_ = true;
  clearPendingPositionCommandLocked();
}

bool PhaseOffsetMatchedAdapter::updateGate(const MatchedAdapterInput& input,
                                           MatchedAdapterOutput& output) {
  const bool path_available = HasUsablePathInput(input.path);
  const bool legacy_available = HasUsableLegacyInput(input.legacy);
  if (!path_available || !legacy_available) {
    const std::string reason = !path_available
        ? "matched adapter path input is unavailable"
        : "matched adapter legacy guidance input is unavailable";
    output.zero_port.invalid_reason = reason;
    output.zero_comparison.invalid_reason = reason;
    output.invalid_reason = reason;
    output.zero_gate_open = zero_gate_open_;
    output.zero_gate_consecutive_count = zero_gate_consecutive_count_;
    output.failure_latched = failure_latched_;
    output.control_failure_reason = control_failure_reason_;
    return false;
  }
  ActiveAdapterInput zero_input;
  zero_input.path = input.path; zero_input.position = input.position; zero_input.gains = input.gains;
  zero_port_adapter_.evaluate(zero_input, output.zero_port);
  PhaseOffsetActiveAdapter::compareWithLegacy(output.zero_port, input.legacy,
      config_.equivalence_tolerance, output.zero_comparison);
  const bool equivalent = output.zero_comparison.valid && output.zero_comparison.equivalent;
  if (output.zero_comparison.valid) {
    if (!failure_latched_ && !zero_gate_open_) {
      zero_gate_consecutive_count_ = equivalent ? zero_gate_consecutive_count_ + 1 : 0;
      zero_gate_open_ = zero_gate_consecutive_count_ >= config_.warmup_cycles;
    } else if (!failure_latched_ && !equivalent) {
      latchFailure(phase_offset_navigation::ControlFailureReason::ZERO_PORT_EQUIVALENCE);
    }
  }
  output.zero_gate_open = zero_gate_open_; output.zero_gate_consecutive_count = zero_gate_consecutive_count_;
  output.failure_latched = failure_latched_; output.geometry = output.zero_port.geometry;
  output.control_failure_reason = control_failure_reason_;
  output.base_guidance = output.zero_port.guidance; output.guidance = output.zero_port.guidance;
  return true;
}

bool PhaseOffsetMatchedAdapter::hasPendingPositionCommand() const {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  return static_cast<bool>(pending_v2_shadow_bootstrap_candidate_);
}
void PhaseOffsetMatchedAdapter::clearPendingPositionCommandLocked() {
  pending_v2_shadow_bootstrap_candidate_.reset();
}
bool PhaseOffsetMatchedAdapter::validatePendingPositionCommandLocked(
    std::string* reason) const {
  if (reason) reason->clear();
  if (!pending_v2_shadow_bootstrap_candidate_) return true;
  return validatePendingV2ShadowBootstrapLocked(reason);
}
void PhaseOffsetMatchedAdapter::discardPendingPositionCommand() {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  clearPendingPositionCommandLocked();
}

bool PhaseOffsetMatchedAdapter::validatePendingPositionCommand() const {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  return validatePendingPositionCommandLocked(nullptr);
}

PendingPositionCommandCapture
PhaseOffsetMatchedAdapter::capturePendingPositionCommand() const {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  PendingPositionCommandCapture capture;
  if (!pending_v2_shadow_bootstrap_candidate_) {
    capture.valid = true;
    return capture;
  }
  capture.pending = true;
  const phase_offset_navigation::AuthorityPreparedStep& authority =
      pending_v2_shadow_bootstrap_candidate_->authority_prepared;
  capture.identity = authority.committed_snapshot
      ? authority.committed_snapshot->snapshotId() : 0U;
  capture.reference_query = authority.committed_snapshot
      ? authority.committed_snapshot->executed_reference_query
      : phase_offset_navigation::ImmutableExecutedReferenceQueryPtr();
  capture.valid = capture.identity != 0U &&
      static_cast<bool>(capture.reference_query) &&
      validatePendingPositionCommandLocked(nullptr);
  capture.v2_binding_transition =
      pending_v2_shadow_bootstrap_candidate_->purpose ==
          TubeWorkerPurposeV2::SUCCESSOR &&
      pending_v2_shadow_bootstrap_candidate_->prepared_step.binding_transition;
  capture.proposed_v2_binding =
      pending_v2_shadow_bootstrap_candidate_->proposed_binding;
  return capture;
}
phase_offset_navigation::ImmutableExecutedReferenceQueryPtr
PhaseOffsetMatchedAdapter::pendingExecutedReferenceQuery() const {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  if (!pending_v2_shadow_bootstrap_candidate_ ||
      !pending_v2_shadow_bootstrap_candidate_->authority_prepared
           .committed_snapshot) {
    return phase_offset_navigation::ImmutableExecutedReferenceQueryPtr();
  }
  return pending_v2_shadow_bootstrap_candidate_->authority_prepared
      .committed_snapshot->executed_reference_query;
}
bool PhaseOffsetMatchedAdapter::publishPendingPositionCommand(
    const std::function<bool()>& local_publish,
    const std::uint64_t expected_identity,
    const bool cancel_pending_for_goal_override,
    const std::function<void()>& post_publish_no_fail) {
  std::unique_lock<std::mutex> task_lock(task_publication_mutex_);
  std::unique_lock<std::mutex> runtime_lock(runtime_command_mutex_);
  if (pending_v2_shadow_bootstrap_candidate_) {
    const std::shared_ptr<const TubeV2ShadowAdmissionCandidate> pending =
        pending_v2_shadow_bootstrap_candidate_;
    const std::uint64_t authority_id =
        pending->authority_prepared.committed_snapshot
            ? pending->authority_prepared.committed_snapshot->snapshotId()
            : 0U;
    if (cancel_pending_for_goal_override ||
        !pending->profile || authority_id == 0U ||
        expected_identity == 0U || expected_identity != authority_id ||
        !validatePendingV2ShadowBootstrapLocked(nullptr) ||
        (!pending->recovery_step.valid &&
         (!pending->proposed_binding ||
          !pending->proposed_binding->complete()))) {
      return false;
    }
    std::string authority_reason;
    if (!execution_authority_.finalValidate(
            pending->authority_prepared, &authority_reason)) {
      return false;
    }
    if (!local_publish || !local_publish()) return false;
    execution_authority_.commitNoFail(pending->authority_prepared);
    if (pending->recovery_step.valid) {
      recovery_owner_.commitNoFail(pending->recovery_step);
      runtime_->commitV2NoFail(pending->commit_token);
    } else {
      runtime_->commitV2NoFail(pending->commit_token);
      v2_execution_binding_ = pending->proposed_binding;
      if (pending->purpose == TubeWorkerPurposeV2::CURRENT)
        releaseCurrentCohort(pending->request_id);
    }
    if (post_publish_no_fail) post_publish_no_fail();
    clearPendingPositionCommandLocked();
    return true;
  }
  if (expected_identity != 0U) return false;
  if (cancel_pending_for_goal_override) {
    DeactivationCommitToken token;
    if (!prepareDeactivationLocked(
            ros::Time::now(),
            task_generation_.load(std::memory_order_acquire), token) ||
        !local_publish || !local_publish()) {
      return false;
    }
    commitDeactivationNoFailLocked(token);
    if (post_publish_no_fail) post_publish_no_fail();
    return true;
  }
  if (!local_publish || !local_publish()) return false;
  if (post_publish_no_fail) post_publish_no_fail();
  return true;
}
bool PhaseOffsetMatchedAdapter::update(const MatchedAdapterInput& input, MatchedAdapterOutput& output) {
  output = MatchedAdapterOutput();
  const std::uint64_t task_generation =
      task_generation_.load(std::memory_order_acquire);
  if (shutdown_requested_.load(std::memory_order_acquire) || !configuration_valid_) {
    output.invalid_reason = "matched adapter configuration is invalid or shutting down";
    return false;
  }
  std::lock_guard<std::mutex> command_lock(runtime_command_mutex_);
  if (task_generation != task_generation_.load(std::memory_order_acquire)) {
    output.invalid_reason = "matched adapter task retired";
    return false;
  }
  const bool input_available = updateGate(input, output);
  if (!input_available) {
    // V2 shadow is strictly observational.  Invalid command input must not
    // run deactivation/Runtime publication machinery or mutate any legacy
    // epoch/control snapshot; the previously retained authority remains
    // untouched while this tick simply reports the zero-port baseline.
    return false;
  }
  const bool equivalent = output.zero_comparison.valid && output.zero_comparison.equivalent;
  if (config_.mode == PhaseOffsetMatchedMode::ACTIVE) {
    output.selected = zero_gate_open_ && !failure_latched_ && equivalent;
    output.valid = output.zero_port.valid && output.zero_comparison.valid;
    output.invalid_reason = output.selected ? std::string() : output.zero_comparison.invalid_reason;
    if (active_diagnostics_pub_) {
      const auto values = PhaseOffsetActiveAdapter::makeDiagnostics(output.zero_port, output.zero_comparison,
          zero_gate_open_, zero_gate_consecutive_count_, failure_latched_, output.selected);
      std_msgs::Float64MultiArray diagnostics; diagnostics.data.assign(values.begin(), values.end()); active_diagnostics_pub_.publish(diagnostics);
    }
    return output.selected;
  }
  {
    consumeV2ShadowCompletions();
    // The shadow seam transports one immutable builder request only.  It
    // never exposes legacy Candidate/Epoch evidence or mutates Pair/beta;
    // NORMAL stays observational, while an installed incumbent may stage its
    // already-certified reserve through the existing publish-first seam.
    const std::shared_ptr<const TubeBuildRequestV2> request =
        makeBuildRequestV2(input);
    // Never erase the sole committed binding merely because incoming
    // capture/path evidence is missing, incompatible, or expired.  The
    // applicability path below either executes the incumbent under its exact
    // still-valid contract, consumes its committed reserve, or fails closed;
    // only a publish-first replacement or an exactly neutral task retirement
    // changes ownership.
    output.base_guidance = output.zero_port.guidance;
    output.guidance = output.zero_port.guidance;
    output.delta = runtime_ ? runtime_->retainedDelta() : 0.0;
    output.delta_ref = 0.0;
    output.profile_active = false;
    output.selected = false;
    output.valid = output.zero_port.valid && output.zero_comparison.valid &&
        !failure_latched_;
    output.invalid_reason = output.valid ? std::string()
                                         : output.zero_comparison.invalid_reason;
    output.failure_latched = failure_latched_;
    output.control_failure_reason = control_failure_reason_;
    // A failed local publication retains the exact immutable recovery
    // transaction.  Re-expose that command for retry without admitting a
    // newer completion or advancing either owner in update().
    if (pending_v2_shadow_bootstrap_candidate_ &&
        pending_v2_shadow_bootstrap_candidate_->recovery_step.valid) {
      const std::shared_ptr<const TubeV2ShadowAdmissionCandidate> pending =
          pending_v2_shadow_bootstrap_candidate_;
      if (PendingV2ApplicabilityEvidenceMatches(input, *pending) &&
          validatePendingV2ShadowBootstrapLocked(nullptr) &&
          PopulateV2RecoveryOutput(config_, input, *pending, output)) {
        latest_v2_shadow_admission_candidate_ = pending;
        output.v2_shadow_admission_candidate = pending;
        return true;
      }
      pending_v2_shadow_bootstrap_candidate_.reset();
    }
    if (pending_v2_shadow_bootstrap_candidate_ &&
        pending_v2_shadow_bootstrap_candidate_->purpose ==
            TubeWorkerPurposeV2::SUCCESSOR) {
      const std::shared_ptr<const TubeV2ShadowAdmissionCandidate> pending =
          pending_v2_shadow_bootstrap_candidate_;
      if (PendingV2ApplicabilityEvidenceMatches(input, *pending) &&
          validatePendingV2ShadowBootstrapLocked(nullptr) &&
          PopulateV2SuccessorOutput(config_, input, *pending, output)) {
        latest_v2_shadow_admission_candidate_ = pending;
        output.v2_shadow_admission_candidate = pending;
        return true;
      }
      pending_v2_shadow_bootstrap_candidate_.reset();
    }
    if (pending_v2_shadow_bootstrap_candidate_ &&
        pending_v2_shadow_bootstrap_candidate_->purpose ==
            TubeWorkerPurposeV2::CURRENT &&
        !pending_v2_shadow_bootstrap_candidate_->recovery_step.valid) {
      const std::shared_ptr<const TubeV2ShadowAdmissionCandidate> pending =
          pending_v2_shadow_bootstrap_candidate_;
      if (PendingV2ApplicabilityEvidenceMatches(input, *pending) &&
          validatePendingV2ShadowBootstrapLocked(nullptr) &&
          PopulateV2NormalOutput(config_, input, *pending, output)) {
        latest_v2_shadow_admission_candidate_ = pending;
        output.v2_shadow_admission_candidate = pending;
        return true;
      }
      releaseCurrentCohort(pending->request_id);
      pending_v2_shadow_bootstrap_candidate_.reset();
    }
    // Completed CURRENT profiles and the still-applicable incumbent share the
    // same live NORMAL preparation and publish-first transaction.  Neither
    // path creates legacy Candidate/Active authority.
    std::shared_ptr<const TubeV2ShadowAdmissionCandidate> admission_candidate;
    std::shared_ptr<const TubeV2ShadowAdmissionCandidate>
        successor_admission_candidate;
    phase_offset_navigation::RecoveryPreparedStep incumbent_recovery_step;
    const phase_offset_navigation::RecoveryOwnerStatus& recovery_status =
        recovery_owner_.status();
    const bool recovery_running = recovery_status.active &&
        recovery_status.proof_kind ==
            phase_offset_navigation::RecoveryStepProofKind::FINITE_RESERVE_V2 &&
        recovery_status.reserve_cursor < recovery_status.reserve_size;
    if (!recovery_running && input.tube_v2_successor_handoff) {
      std::shared_ptr<const TubeWorkerCompletionV2> completion;
      {
        std::lock_guard<std::mutex> worker_lock(worker_state_mutex_);
        completion = latest_v2_shadow_successor_completion_;
      }
      if (completion) {
        prepareV2ShadowSuccessorAdmission(
            input, *input.tube_v2_successor_handoff, completion,
            successor_admission_candidate);
      }
    }
    if (!recovery_running && request->tube_worker_input_v2 &&
        request->tube_worker_purpose_v2 == TubeWorkerPurposeV2::CURRENT) {
      std::shared_ptr<const TubeWorkerCompletionV2> completion;
      {
        std::lock_guard<std::mutex> worker_lock(worker_state_mutex_);
        completion = latest_v2_shadow_current_completion_;
      }
      if (completion && completion->request_id == request->tube_worker_input_v2->request_id) {
        const bool admitted = prepareV2ShadowAdmission(
            input, request->tube_worker_input_v2, completion, admission_candidate);
        if (!admitted && completion->request_id == request->tube_worker_input_v2->request_id)
          releaseCurrentCohort(completion->request_id);
      }
    }
    if (successor_admission_candidate &&
        successor_admission_candidate->applicable &&
        input.tube_v2_successor_handoff) {
      MatchedAdapterOutput selected_output = output;
      TubeV2ShadowAdmissionCandidate enriched =
          *successor_admission_candidate;
      std::string authority_reason;
      if (PopulateV2SuccessorOutput(
              config_, input, enriched, selected_output) &&
          prepareV2ExecutionAuthorityLocked(
              input, selected_output,
              phase_offset_navigation::ActiveReferenceOwnerMode::NORMAL,
              enriched.proposed_binding, enriched, &authority_reason)) {
        successor_admission_candidate =
            std::make_shared<const TubeV2ShadowAdmissionCandidate>(
                std::move(enriched));
      }
      if (successor_admission_candidate->authority_prepared.valid &&
          stageV2ShadowSuccessorLocked(
              input, *input.tube_v2_successor_handoff,
              successor_admission_candidate)) {
        output = selected_output;
        latest_v2_shadow_admission_candidate_ =
            successor_admission_candidate;
        output.v2_shadow_admission_candidate =
            successor_admission_candidate;
        return true;
      }
      pending_v2_shadow_bootstrap_candidate_.reset();
    }
    // A refresh gap or failed replacement must still re-check the installed
    // immutable profile against the whole next ZOH command and its exact
    // successor reserve.  If that command is unavailable, prepare the first
    // step of the already committed finite reserve through the existing
    // RecoveryOwner seam; its commit remains publish-gated.
    const bool successor_prefix_expired = successor_admission_candidate &&
        !successor_admission_candidate->applicable &&
        successor_admission_candidate->status ==
            phase_offset_navigation::TubeExecutionStatusV2::RANGE_UNAVAILABLE;
    if (v2_execution_binding_ &&
        (recovery_running || !admission_candidate ||
         !admission_candidate->applicable || successor_prefix_expired)) {
      std::shared_ptr<const TubeV2ShadowAdmissionCandidate> incumbent_candidate;
      const bool incumbent_applicable =
          prepareV2ShadowIncumbentApplicability(
              input, v2_execution_binding_->profile, incumbent_candidate,
              &incumbent_recovery_step, successor_prefix_expired);
      if (incumbent_applicable) {
        admission_candidate = incumbent_candidate;
      } else if (incumbent_candidate && incumbent_candidate->recovery_step.valid) {
        // Keep the exact finite-reserve step and its preallocated Runtime
        // successor together; the existing pending PositionCommand seam is
        // the only place allowed to publish and advance either owner.
        admission_candidate = incumbent_candidate;
      } else if (incumbent_candidate &&
                 incumbent_candidate->status ==
                     phase_offset_navigation::TubeExecutionStatusV2::
                         TRACKING_UNAVAILABLE) {
        // Tracking/physical applicability failure is still useful
        // value-only evidence.  Retain the immutable incumbent so this
        // categorical denial cannot be mistaken for profile loss or cause
        // an unrelated retirement; no certified recovery is attached.
        admission_candidate = incumbent_candidate;
      }
    }
    if (incumbent_recovery_step.status ==
        phase_offset_navigation::RecoveryStepStatus::PREPARED) {
      output.recovery_status = incumbent_recovery_step.status;
      output.recovery_replan_required = false;
    } else if (v2_execution_binding_ && admission_candidate &&
               !admission_candidate->applicable &&
               input.tube_v2_admission.valid) {
      output.recovery_status =
          phase_offset_navigation::RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED;
      output.recovery_replan_required = true;
    }
    if (admission_candidate &&
        admission_candidate->recovery_step.valid) {
      MatchedAdapterOutput selected_output = output;
      TubeV2ShadowAdmissionCandidate enriched = *admission_candidate;
      std::string authority_reason;
      if (PopulateV2RecoveryOutput(
              config_, input, enriched, selected_output) &&
          prepareV2ExecutionAuthorityLocked(
              input, selected_output,
              phase_offset_navigation::ActiveReferenceOwnerMode::RECOVERY,
              v2_execution_binding_, enriched, &authority_reason)) {
        admission_candidate =
            std::make_shared<const TubeV2ShadowAdmissionCandidate>(
                std::move(enriched));
      }
      if (admission_candidate->authority_prepared.valid &&
          stageV2ShadowRecoveryLocked(input, admission_candidate)) {
        output = selected_output;
        latest_v2_shadow_admission_candidate_ = admission_candidate;
        output.v2_shadow_admission_candidate = admission_candidate;
        return true;
      }
      output.selected = false;
      output.valid = false;
      output.recovery_status =
          phase_offset_navigation::RecoveryStepStatus::
              RECOVERY_REPLAN_REQUIRED;
      output.recovery_replan_required = true;
      output.invalid_reason = "V2 recovery publication transaction is unavailable";
    }
    if (admission_candidate && admission_candidate->applicable &&
        !admission_candidate->recovery_step.valid &&
        admission_candidate->proposed_binding &&
        admission_candidate->proposed_binding->source_input) {
      MatchedAdapterOutput selected_output = output;
      TubeV2ShadowAdmissionCandidate enriched = *admission_candidate;
      std::string authority_reason;
      if (PopulateV2NormalOutput(config_, input, enriched, selected_output) &&
          prepareV2ExecutionAuthorityLocked(
              input, selected_output,
              phase_offset_navigation::ActiveReferenceOwnerMode::NORMAL,
              enriched.proposed_binding, enriched, &authority_reason)) {
        admission_candidate =
            std::make_shared<const TubeV2ShadowAdmissionCandidate>(
                std::move(enriched));
      }
      if (admission_candidate->authority_prepared.valid &&
          stageV2ShadowBootstrapLocked(
              input, *admission_candidate->proposed_binding->source_input,
              admission_candidate)) {
        output = selected_output;
        latest_v2_shadow_admission_candidate_ = admission_candidate;
        output.v2_shadow_admission_candidate = admission_candidate;
        return true;
      }
      if (!admission_candidate->nonselecting)
        releaseCurrentCohort(admission_candidate->request_id);
      pending_v2_shadow_bootstrap_candidate_.reset();
    }
    const std::shared_ptr<const TubeV2ShadowAdmissionCandidate>& reported =
        successor_admission_candidate ? successor_admission_candidate
                                      : admission_candidate;
    latest_v2_shadow_admission_candidate_ = reported;
    output.v2_shadow_admission_candidate = reported;
    return false;
  }
}
}  // namespace FLAG_Race
