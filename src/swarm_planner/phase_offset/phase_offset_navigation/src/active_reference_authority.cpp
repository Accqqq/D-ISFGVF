#include "phase_offset_navigation/active_reference_authority.h"

#include <algorithm>
#include <cmath>

namespace phase_offset_navigation {
namespace {

bool Finite(const double value) { return std::isfinite(value); }
bool Finite(const phase_offset_core::PortCommand& value) {
  return Finite(value.u_w) && Finite(value.u_delta);
}

bool OwnerAllowed(const ActiveReferenceSnapshot& snapshot,
                  const PhaseOffsetExecutionAuthorityConfig& config) {
  if (snapshot.selected_u_owner.empty()) return false;
  if (snapshot.owner_mode == ActiveReferenceOwnerMode::RECOVERY) {
    return snapshot.selected_u_owner == "PhaseOffsetRecoveryOwner";
  }
  if (snapshot.owner_mode == ActiveReferenceOwnerMode::NORMAL ||
      snapshot.owner_mode == ActiveReferenceOwnerMode::COORDINATION) {
    return snapshot.selected_u_owner == "PhaseOffsetAllocator" ||
        (config.allow_test_only_runtime_owner &&
         snapshot.selected_u_owner == "PhaseOffsetMatchedAdapterRuntime");
  }
  if (snapshot.owner_mode == ActiveReferenceOwnerMode::PLANNER_ONLY) {
    return snapshot.selected_u_owner == "PlannerOwner" ||
        snapshot.selected_u_owner == "PLANNER_ONLY" ||
        snapshot.selected_u_owner == "";
  }
  // NONE/unknown modes have no execution owner.  They must never be allowed
  // to enter the authoritative transaction merely because the value fields
  // happen to be finite; unknown modes fail closed until an explicit owner
  // contract is added above.
  return false;
}

bool ValidConfig(const PhaseOffsetExecutionAuthorityConfig& config) {
  return config.initial_authority_session != 0U &&
      Finite(config.comparison_epsilon) && config.comparison_epsilon >= 0.0;
}

}  // namespace

const char* executionAuthorityStatusName(const ExecutionAuthorityStatus status) {
  switch (status) {
    case ExecutionAuthorityStatus::NONE: return "NONE";
    case ExecutionAuthorityStatus::PREPARED: return "PREPARED";
    case ExecutionAuthorityStatus::COMMITTED: return "COMMITTED";
    case ExecutionAuthorityStatus::STALE: return "STALE";
    case ExecutionAuthorityStatus::REVISION_MISMATCH: return "REVISION_MISMATCH";
    case ExecutionAuthorityStatus::OWNER_MISMATCH: return "OWNER_MISMATCH";
    case ExecutionAuthorityStatus::REFERENCE_MISMATCH:
      return "REFERENCE_MISMATCH";
    case ExecutionAuthorityStatus::MATCHED_OUTPUT_INVALID:
      return "MATCHED_OUTPUT_INVALID";
    case ExecutionAuthorityStatus::CURRENT_STATE_UNSAFE:
      return "CURRENT_STATE_UNSAFE";
    case ExecutionAuthorityStatus::PLANNER_INVALID: return "PLANNER_INVALID";
    case ExecutionAuthorityStatus::RECOVERY_REPLAN_REQUIRED:
      return "RECOVERY_REPLAN_REQUIRED";
    case ExecutionAuthorityStatus::PUBLISH_FAILED: return "PUBLISH_FAILED";
    case ExecutionAuthorityStatus::TRANSACTION_CONFLICT:
      return "TRANSACTION_CONFLICT";
    case ExecutionAuthorityStatus::INVALID: return "INVALID";
  }
  return "UNKNOWN";
}

AuthoritySnapshotDiagnostics makeAuthoritySnapshotDiagnostics(
    const ActiveReferenceSnapshot& snapshot) {
  AuthoritySnapshotDiagnostics diagnostics;
  diagnostics.authority_session = snapshot.authority_session;
  diagnostics.sequence = snapshot.sequence;
  diagnostics.snapshot_id = snapshot.snapshotId();
  diagnostics.planner_path_revision = snapshot.planner_path_revision;
  diagnostics.executed_path_revision = snapshot.executed_path_revision;
  diagnostics.frame_revision = snapshot.frame_revision;
  diagnostics.tube_revision = snapshot.tube_revision;
  diagnostics.profile_revision = snapshot.profile_revision;
  diagnostics.reference_query_revision = snapshot.reference_query_revision;
  diagnostics.owner_mode = snapshot.owner_mode;
  diagnostics.selected_u_owner = snapshot.selected_u_owner;
  diagnostics.selected_u = snapshot.selected_u;
  diagnostics.w = snapshot.w;
  diagnostics.delta = snapshot.delta;
  diagnostics.proposed_next_w = snapshot.proposed_next_w;
  diagnostics.proposed_next_delta = snapshot.proposed_next_delta;
  diagnostics.valid = snapshot.valid;
  diagnostics.provenance = snapshot.provenance;
  return diagnostics;
}

PhaseOffsetExecutionAuthority::PhaseOffsetExecutionAuthority(
    const PhaseOffsetExecutionAuthorityConfig& config)
    : config_(config), configuration_valid_(ValidConfig(config)) {}

ActiveReferenceSnapshot PhaseOffsetExecutionAuthority::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_ ? *snapshot_ : ActiveReferenceSnapshot();
}

std::shared_ptr<const ActiveReferenceSnapshot>
PhaseOffsetExecutionAuthority::snapshotPtr() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

AuthoritySnapshotDiagnostics PhaseOffsetExecutionAuthority::diagnostics() const {
  return makeAuthoritySnapshotDiagnostics(snapshot());
}

bool PhaseOffsetExecutionAuthority::seed(
    const ActiveReferenceSnapshot& initial, std::string* failure_reason) {
  if (failure_reason) failure_reason->clear();
  if (!configuration_valid_ || !initial.valid ||
      !OwnerAllowed(initial, config_) ||
      !initial.selectedUConsistent(config_.comparison_epsilon) ||
      !Finite(initial.w) || !Finite(initial.delta) || !Finite(initial.dt) ||
      initial.dt <= 0.0 || !Finite(initial.u_prev) ||
      !Finite(initial.matched_base_w_dot) ||
      !Finite(initial.proposed_next_w) || !Finite(initial.proposed_next_delta) ||
      !Finite(initial.proposed_next_u_prev)) {
    if (failure_reason) *failure_reason = "initial authority snapshot is invalid";
    return false;
  }
  ActiveReferenceSnapshot copy = initial;
  if (copy.authority_session == 0U) {
    copy.authority_session = config_.initial_authority_session;
  }
  if (copy.sequence == 0U) copy.sequence = 1U;
  if (copy.reference_query_revision == 0U && copy.executed_reference_query) {
    copy.reference_query_revision = copy.executed_reference_query->queryRevision();
  }
  if (copy.executed_reference_query && !copy.governorViewValid()) {
    if (failure_reason) *failure_reason = "initial reference query provenance mismatches snapshot";
    return false;
  }
  const double expected_next_w = copy.w + copy.dt *
      (copy.matched_base_w_dot + copy.selected_u.u_w);
  const double expected_next_delta =
      copy.delta + copy.dt * copy.selected_u.u_delta;
  if (!Finite(expected_next_w) || !Finite(expected_next_delta) ||
      std::abs(expected_next_w - copy.proposed_next_w) >
          config_.comparison_epsilon ||
      std::abs(expected_next_delta - copy.proposed_next_delta) >
          config_.comparison_epsilon ||
      std::abs(copy.proposed_next_u_prev.u_w - copy.selected_u.u_w) >
          config_.comparison_epsilon ||
      std::abs(copy.proposed_next_u_prev.u_delta - copy.selected_u.u_delta) >
          config_.comparison_epsilon) {
    if (failure_reason) *failure_reason =
        "initial authority snapshot has an inexact selected-u recurrence";
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.reset(new ActiveReferenceSnapshot(copy));
  return true;
}

bool PhaseOffsetExecutionAuthority::validateCandidate(
    const ActiveReferenceSnapshot& candidate,
    const AuthorityPrepareInput& input,
    const ActiveReferenceSnapshot& current,
    std::string& reason) const {
  reason.clear();
  if (!candidate.valid) {
    reason = "candidate snapshot is invalid";
    return false;
  }
  if (!OwnerAllowed(candidate, config_)) {
    reason = "selected-u owner is missing or incompatible with mode";
    return false;
  }
  if (!candidate.selectedUConsistent(config_.comparison_epsilon)) {
    reason = "selected-u scalar aliases do not match exact command";
    return false;
  }
  if (!Finite(candidate.w) || !Finite(candidate.delta) || !Finite(candidate.dt) ||
      candidate.dt <= 0.0 || !Finite(candidate.u_prev) ||
      !Finite(candidate.matched_base_w_dot) ||
      !Finite(candidate.proposed_next_w) ||
      !Finite(candidate.proposed_next_delta) ||
      !Finite(candidate.proposed_next_u_prev)) {
    reason = "candidate state is not finite";
    return false;
  }
  const bool test_runtime_fixture = config_.allow_test_only_runtime_owner &&
      candidate.selected_u_owner == "PhaseOffsetMatchedAdapterRuntime";
  if ((!test_runtime_fixture &&
       (candidate.planner_path_revision == 0U ||
        candidate.executed_path_revision == 0U ||
        candidate.frame_revision == 0U)) ||
      !candidate.r.allFinite() || !candidate.r_w.allFinite()) {
    reason = "candidate reference revisions or jet are invalid";
    return false;
  }
  if (candidate.owner_mode == ActiveReferenceOwnerMode::PLANNER_ONLY &&
      (std::abs(candidate.selected_u.u_w) > config_.comparison_epsilon ||
       std::abs(candidate.selected_u.u_delta) > config_.comparison_epsilon)) {
    reason = "planner-only handoff cannot execute a nonzero selected-u";
    return false;
  }
  if (input.current_state_unsafe || candidate.current_state_unsafe) {
    reason = "current state is explicitly unsafe";
    return false;
  }
  if (input.planner_invalid || candidate.planner_invalid) {
    reason = "planner is invalid";
    return false;
  }
  if (input.stale || candidate.stale) {
    reason = "candidate snapshot is stale";
    return false;
  }
  if (!input.matched_output_valid) {
    reason = "matched output is invalid";
    return false;
  }
  if (!input.reference_valid ||
      (config_.require_reference_query && !candidate.executed_reference_query)) {
    reason = "immutable executed-reference query is unavailable";
    return false;
  }
  if (candidate.executed_reference_query) {
    if (candidate.executed_reference_query->pathRevision() !=
            candidate.executed_path_revision ||
        candidate.executed_reference_query->frameRevision() !=
            candidate.frame_revision ||
        (candidate.reference_query_revision != 0U &&
         candidate.executed_reference_query->queryRevision() !=
             candidate.reference_query_revision)) {
      reason = "reference query revision mismatch";
      return false;
    }
    ExecutedReferenceQueryResult reference;
    if (!candidate.executed_reference_query->query(candidate.w, reference) ||
        !reference.valid ||
        (reference.r - candidate.r).norm() > config_.comparison_epsilon ||
        (reference.r_w - candidate.r_w).norm() > config_.comparison_epsilon ||
        candidate.r_ww_valid != reference.r_ww_valid ||
        (candidate.r_ww_valid &&
         (reference.r_ww - candidate.r_ww).norm() >
             config_.comparison_epsilon)) {
      reason = "candidate reference jet mismatches immutable query";
      return false;
    }
    double domain_start = 0.0;
    double domain_end = 0.0;
    if (!candidate.executed_reference_query->domain(domain_start, domain_end) ||
        !Finite(domain_start) || !Finite(domain_end) ||
        candidate.w < domain_start - config_.comparison_epsilon ||
        candidate.w > domain_end + config_.comparison_epsilon) {
      reason = "candidate phase lies outside immutable reference domain";
      return false;
    }
  }
  const double expected_next_w = candidate.w + candidate.dt *
      (candidate.matched_base_w_dot + candidate.selected_u.u_w);
  const double expected_next_delta =
      candidate.delta + candidate.dt * candidate.selected_u.u_delta;
  if (!Finite(expected_next_w) || !Finite(expected_next_delta) ||
      std::abs(expected_next_w - candidate.proposed_next_w) >
          config_.comparison_epsilon ||
      std::abs(expected_next_delta - candidate.proposed_next_delta) >
          config_.comparison_epsilon ||
      std::abs(candidate.proposed_next_u_prev.u_w -
                   candidate.selected_u.u_w) > config_.comparison_epsilon ||
      std::abs(candidate.proposed_next_u_prev.u_delta -
                   candidate.selected_u.u_delta) > config_.comparison_epsilon) {
    reason = "candidate next state is not the exact selected-u recurrence";
    return false;
  }
  if (candidate.owner_mode == ActiveReferenceOwnerMode::RECOVERY &&
      candidate.selected_u_owner != "PhaseOffsetRecoveryOwner") {
    reason = "recovery selected-u owner is not exact";
    return false;
  }
  if (!current.valid) return true;
  if (input.expected_authority_session != 0U &&
      input.expected_authority_session != current.authority_session) {
    reason = "authority session mismatch";
    return false;
  }
  if (input.expected_sequence != 0U && input.expected_sequence != current.sequence) {
    reason = "authority sequence mismatch";
    return false;
  }
  if (candidate.authority_session != current.authority_session) {
    reason = "candidate authority session is stale";
    return false;
  }
  if (candidate.sequence != 0U && candidate.sequence != current.sequence &&
      candidate.sequence != current.sequence + 1U) {
    reason = "candidate sequence is stale or skipped";
    return false;
  }
  // Every committed tick must be an exact predecessor-state continuation.
  // Session/sequence identity alone cannot prevent a candidate from
  // silently jumping over the last committed phase, delta, or previous port.
  const double predecessor_eps = config_.comparison_epsilon;
  if (!Finite(current.proposed_next_w) ||
      !Finite(current.proposed_next_delta) ||
      !Finite(current.proposed_next_u_prev) ||
      std::abs(candidate.w - current.proposed_next_w) > predecessor_eps ||
      std::abs(candidate.delta - current.proposed_next_delta) > predecessor_eps ||
      std::abs(candidate.u_prev.u_w - current.proposed_next_u_prev.u_w) >
          predecessor_eps ||
      std::abs(candidate.u_prev.u_delta - current.proposed_next_u_prev.u_delta) >
          predecessor_eps) {
    reason = "candidate does not chain from exact predecessor state";
    return false;
  }
  return true;
}

bool PhaseOffsetExecutionAuthority::prepare(const AuthorityPrepareInput& input,
                                            AuthorityPreparedStep& output) const {
  output = AuthorityPreparedStep();
  output.side_effect_free = true;
  ActiveReferenceSnapshot current;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (snapshot_) current = *snapshot_;
  }
  output.expected = current;
  output.expected_snapshot_id = current.snapshotId();
  if (!configuration_valid_) {
    output.status = ExecutionAuthorityStatus::INVALID;
    output.failure_reason = "authority configuration is invalid";
    return false;
  }
  if (input.current_state_unsafe || input.candidate.current_state_unsafe) {
    output.status = ExecutionAuthorityStatus::CURRENT_STATE_UNSAFE;
    output.failure_reason = "current state is explicitly unsafe";
    return false;
  }
  if (input.planner_invalid || input.candidate.planner_invalid) {
    output.status = ExecutionAuthorityStatus::PLANNER_INVALID;
    output.failure_reason = "planner is invalid";
    return false;
  }
  if (input.stale || input.candidate.stale) {
    output.status = ExecutionAuthorityStatus::STALE;
    output.failure_reason = "candidate is stale";
    return false;
  }
  if (input.recovery_replan_required || input.deadline_expired) {
    output.status = ExecutionAuthorityStatus::RECOVERY_REPLAN_REQUIRED;
    output.failure_reason = input.deadline_expired
        ? "recovery deadline expired" : "recovery evidence requires replanning";
    return false;
  }
  if (!input.matched_output_valid) {
    output.status = ExecutionAuthorityStatus::MATCHED_OUTPUT_INVALID;
    output.failure_reason = "matched output is invalid";
    return false;
  }
  std::string reason;
  if (!validateCandidate(input.candidate, input, current, reason)) {
    output.failure_reason = reason;
    if (reason.find("owner") != std::string::npos) {
      output.status = ExecutionAuthorityStatus::OWNER_MISMATCH;
    } else if (reason.find("reference") != std::string::npos) {
      output.status = ExecutionAuthorityStatus::REFERENCE_MISMATCH;
    } else if (reason.find("session") != std::string::npos ||
               reason.find("sequence") != std::string::npos) {
      output.status = ExecutionAuthorityStatus::REVISION_MISMATCH;
    } else {
      output.status = ExecutionAuthorityStatus::INVALID;
    }
    return false;
  }

  ActiveReferenceSnapshot candidate = input.candidate;
  if (!current.valid) {
    if (candidate.authority_session == 0U) {
      candidate.authority_session = config_.initial_authority_session;
    }
    candidate.sequence = candidate.sequence == 0U ? 1U : candidate.sequence;
  } else {
    candidate.authority_session = current.authority_session;
    candidate.sequence = current.sequence + 1U;
  }
  if (candidate.reference_query_revision == 0U && candidate.executed_reference_query) {
    candidate.reference_query_revision = candidate.executed_reference_query->queryRevision();
  }
  candidate.selected_u_w = candidate.selected_u.u_w;
  candidate.selected_u_delta = candidate.selected_u.u_delta;
  candidate.provenance = input.provenance.empty() ? candidate.provenance
                                                  : input.provenance;
  output.candidate = candidate;
  output.committed_snapshot =
      std::make_shared<const ActiveReferenceSnapshot>(candidate);
  output.status = ExecutionAuthorityStatus::PREPARED;
  output.valid = true;
  return true;
}

bool PhaseOffsetExecutionAuthority::finalValidate(
    const AuthorityPreparedStep& prepared, std::string* failure_reason) const {
  if (failure_reason) failure_reason->clear();
  if (!configuration_valid_ || !prepared.valid ||
      prepared.status != ExecutionAuthorityStatus::PREPARED ||
      !prepared.side_effect_free || !prepared.committed_snapshot) {
    if (failure_reason) *failure_reason = "prepared authority step is invalid";
    return false;
  }
  ActiveReferenceSnapshot current;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (snapshot_) current = *snapshot_;
  }
  if (current.valid && current.snapshotId() != prepared.expected_snapshot_id) {
    if (failure_reason) *failure_reason = "authority snapshot changed after prepare";
    return false;
  }
  AuthorityPrepareInput final_input;
  final_input.matched_output_valid = true;
  final_input.reference_valid = true;
  // The candidate value is intentionally exposed in the diagnostics DTO, but
  // the immutable snapshot allocated during prepare is the transaction's
  // source of truth.  Never let a caller mutate the public DTO after prepare
  // and thereby validate or publish a different value than the one that will
  // be committed.
  const ActiveReferenceSnapshot& candidate = *prepared.committed_snapshot;
  std::string reason;
  if (!validateCandidate(candidate, final_input, current, reason)) {
    if (failure_reason) *failure_reason = reason;
    return false;
  }
  if (!candidate.selectedUConsistent(config_.comparison_epsilon)) {
    if (failure_reason) *failure_reason = "exact selected-u identity changed";
    return false;
  }
  return true;
}

bool PhaseOffsetExecutionAuthority::commit(
    const AuthorityPreparedStep& prepared, const AuthorityPublishCallback& publish,
    AuthorityCommitResult& result) {
  result = AuthorityCommitResult();
  if (!prepared.valid || prepared.status != ExecutionAuthorityStatus::PREPARED ||
      !prepared.side_effect_free || !prepared.committed_snapshot) {
    result.status = prepared.status == ExecutionAuthorityStatus::NONE
        ? ExecutionAuthorityStatus::INVALID : prepared.status;
    result.failure_reason = "prepared authority step is invalid";
    return false;
  }
  if (!publish) {
    result.status = ExecutionAuthorityStatus::PUBLISH_FAILED;
    result.failure_reason = "a local PositionCommand publisher is required";
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const ActiveReferenceSnapshot current = snapshot_ ? *snapshot_
                                                   : ActiveReferenceSnapshot();
  if (current.valid && current.snapshotId() != prepared.expected_snapshot_id) {
    result.status = ExecutionAuthorityStatus::TRANSACTION_CONFLICT;
    result.failure_reason = "authority snapshot changed after prepare";
    return false;
  }
  // Commit and publication must use exactly the immutable value staged during
  // prepare.  AuthorityPreparedStep::candidate remains a value DTO for
  // diagnostics/backward compatibility, but is not authoritative here.
  const ActiveReferenceSnapshot& next = *prepared.committed_snapshot;
  AuthorityPrepareInput final_input;
  final_input.matched_output_valid = true;
  final_input.reference_valid = true;
  std::string final_reason;
  if (!validateCandidate(next, final_input, current, final_reason)) {
    result.status = final_reason.find("owner") != std::string::npos
        ? ExecutionAuthorityStatus::OWNER_MISMATCH
        : (final_reason.find("reference") != std::string::npos
            ? ExecutionAuthorityStatus::REFERENCE_MISMATCH
            : ExecutionAuthorityStatus::REVISION_MISMATCH);
    result.failure_reason = final_reason;
    return false;
  }
  if (!next.selectedUConsistent(config_.comparison_epsilon)) {
    result.status = ExecutionAuthorityStatus::OWNER_MISMATCH;
    result.failure_reason = "selected-u changed after prepare";
    return false;
  }
  // Materialize diagnostics before invoking the publisher.  The success path
  // below is then limited to a callback result, a shared_ptr swap and scalar
  // status updates; it performs no post-publication heap work.
  result.snapshot = next;
  result.diagnostics = makeAuthoritySnapshotDiagnostics(next);
  if (!publish(next)) {
    result.status = ExecutionAuthorityStatus::PUBLISH_FAILED;
    result.failure_reason = "publisher rejected exact selected-u snapshot";
    return false;
  }
  result.published = true;
  // Keep the compatibility API's commit path transactional as well.  The
  // production adapter uses commitNoFail() explicitly after its own
  // publication seam and prevalidated owner/runtime side effects.
  snapshot_ = prepared.committed_snapshot;
  result.status = ExecutionAuthorityStatus::COMMITTED;
  result.committed = true;
  return true;
}

void PhaseOffsetExecutionAuthority::commitNoFail(
    const AuthorityPreparedStep& prepared) noexcept {
  if (!prepared.committed_snapshot) return;
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_ = prepared.committed_snapshot;
}

void PhaseOffsetExecutionAuthority::resetForNewTask(const std::uint64_t new_session) {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::uint64_t session = new_session != 0U ? new_session
      : (snapshot_ && snapshot_->authority_session != 0U
          ? snapshot_->authority_session + 1U
          : config_.initial_authority_session);
  snapshot_.reset();
  (void)session;
}

}  // namespace phase_offset_navigation
