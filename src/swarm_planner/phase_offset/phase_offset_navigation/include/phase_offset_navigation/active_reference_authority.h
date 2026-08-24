#pragma once

#include "phase_offset_navigation/active_reference_snapshot.h"

#include <functional>
#include <memory>
#include <mutex>
#include <string>

namespace phase_offset_navigation {

enum class ExecutionAuthorityStatus {
  NONE,
  PREPARED,
  COMMITTED,
  STALE,
  REVISION_MISMATCH,
  OWNER_MISMATCH,
  REFERENCE_MISMATCH,
  MATCHED_OUTPUT_INVALID,
  CURRENT_STATE_UNSAFE,
  PLANNER_INVALID,
  RECOVERY_REPLAN_REQUIRED,
  PUBLISH_FAILED,
  TRANSACTION_CONFLICT,
  INVALID,
};

const char* executionAuthorityStatusName(ExecutionAuthorityStatus status);

struct PhaseOffsetExecutionAuthorityConfig {
  std::uint64_t initial_authority_session = 1U;
  double comparison_epsilon = 1e-10;
  bool require_reference_query = true;
  // Production ownership is exact.  This narrowly scoped opt-in exists only
  // for unadvertised unit/observe fixtures that exercise the legacy Runtime
  // without claiming to be a production allocator.  Production adapters
  // clear it when they advertise.
  bool allow_test_only_runtime_owner = false;
};

struct AuthorityPrepareInput {
  ActiveReferenceSnapshot candidate;
  std::uint64_t expected_authority_session = 0U;
  std::uint64_t expected_sequence = 0U;
  bool current_state_unsafe = false;
  bool planner_invalid = false;
  bool stale = false;
  bool matched_output_valid = true;
  bool reference_valid = true;
  bool recovery_replan_required = false;
  bool deadline_expired = false;
  std::string provenance;
};

struct AuthorityPreparedStep {
  ActiveReferenceSnapshot candidate;
  ActiveReferenceSnapshot expected;
  // Allocated during side-effect-free preparation so commit after local
  // publication is a noexcept shared_ptr swap/value publication.
  std::shared_ptr<const ActiveReferenceSnapshot> committed_snapshot;
  std::uint64_t expected_snapshot_id = 0U;
  ExecutionAuthorityStatus status = ExecutionAuthorityStatus::NONE;
  bool valid = false;
  bool side_effect_free = true;
  std::string failure_reason;
};

// A compact diagnostics value copied from the same immutable snapshot that is
// handed to the publisher.  It prevents command/marker/diagnostic state splits.
struct AuthoritySnapshotDiagnostics {
  std::uint64_t authority_session = 0U;
  std::uint64_t sequence = 0U;
  std::uint64_t snapshot_id = 0U;
  std::uint64_t planner_path_revision = 0U;
  std::uint64_t executed_path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  std::uint64_t tube_revision = 0U;
  std::uint64_t profile_revision = 0U;
  std::uint64_t reference_query_revision = 0U;
  ActiveReferenceOwnerMode owner_mode = ActiveReferenceOwnerMode::NONE;
  std::string selected_u_owner;
  phase_offset_core::PortCommand selected_u;
  double w = 0.0;
  double delta = 0.0;
  double proposed_next_w = 0.0;
  double proposed_next_delta = 0.0;
  bool valid = false;
  std::string provenance;
};

struct AuthorityCommitResult {
  ActiveReferenceSnapshot snapshot;
  AuthoritySnapshotDiagnostics diagnostics;
  ExecutionAuthorityStatus status = ExecutionAuthorityStatus::NONE;
  bool published = false;
  bool committed = false;
  std::string failure_reason;
};

AuthoritySnapshotDiagnostics makeAuthoritySnapshotDiagnostics(
    const ActiveReferenceSnapshot& snapshot);

using AuthorityPublishCallback =
    std::function<bool(const ActiveReferenceSnapshot&)>;

class PhaseOffsetExecutionAuthority {
 public:
  explicit PhaseOffsetExecutionAuthority(
      const PhaseOffsetExecutionAuthorityConfig& config =
          PhaseOffsetExecutionAuthorityConfig());

  bool configurationValid() const { return configuration_valid_; }
  const PhaseOffsetExecutionAuthorityConfig& config() const { return config_; }
  // Test/observe-only adapters may explicitly enable their fixture owner
  // before ROS advertisement.  Production callers must leave this disabled.
  void setTestOnlyRuntimeOwnerAllowed(bool allowed) {
    config_.allow_test_only_runtime_owner = allowed;
  }

  ActiveReferenceSnapshot snapshot() const;
  std::shared_ptr<const ActiveReferenceSnapshot> snapshotPtr() const;
  AuthoritySnapshotDiagnostics diagnostics() const;

  // Establishes a new external planner task/session.  This is the only
  // operation that may seed authority without an existing snapshot.
  bool seed(const ActiveReferenceSnapshot& initial,
            std::string* failure_reason = nullptr);

  // Pure transaction preparation.  It never changes the stored snapshot,
  // sequence, owner, delta, or previous command.
  bool prepare(const AuthorityPrepareInput& input,
               AuthorityPreparedStep& output) const;

  // Final, side-effect-free revalidation immediately before local command
  // publication.  It rechecks the live snapshot identity, revisions, exact
  // owner, immutable query jet/domain and selected-u recurrence.
  bool finalValidate(const AuthorityPreparedStep& prepared,
                     std::string* failure_reason = nullptr) const;

  bool commit(const AuthorityPreparedStep& prepared,
              const AuthorityPublishCallback& publish,
              AuthorityCommitResult& result);
  // The post-publication half is deliberately narrow: callers must have
  // completed finalValidate() and a successful local PositionCommand
  // publication before invoking this no-fail snapshot swap.  It performs no
  // allocation, validation, owner callback, or selected-u substitution.
  void commitNoFail(const AuthorityPreparedStep& prepared) noexcept;

  // Retirement is reserved for explicit task reset; transaction failures and
  // recovery denials intentionally leave the last nonzero snapshot intact.
  void resetForNewTask(std::uint64_t new_session = 0U);

 private:
  bool validateCandidate(const ActiveReferenceSnapshot& candidate,
                         const AuthorityPrepareInput& input,
                         const ActiveReferenceSnapshot& current,
                         std::string& reason) const;

  PhaseOffsetExecutionAuthorityConfig config_;
  bool configuration_valid_ = false;
  mutable std::mutex mutex_;
  std::shared_ptr<const ActiveReferenceSnapshot> snapshot_;
};

using ExecutionAuthority = PhaseOffsetExecutionAuthority;

}  // namespace phase_offset_navigation
