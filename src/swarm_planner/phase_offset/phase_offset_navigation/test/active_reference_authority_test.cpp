#include <gtest/gtest.h>

#include <limits>

#include "phase_offset_navigation/active_reference_authority.h"
#include "phase_offset_navigation/handoff_state_machine.h"
#include "phase_offset_navigation/preview_feasibility.h"
#include "phase_offset_navigation/phase_offset_recovery_owner.h"
#include "phase_offset_core/matched_port.h"
#include "phase_offset_core/port_projector.h"

namespace phase_offset_navigation {
namespace {

ImmutableExecutedReferenceQueryPtr Query(std::uint64_t path_revision = 7U,
                                          std::uint64_t frame_revision = 8U,
                                          std::uint64_t owner_revision = 9U,
                                          std::uint64_t query_revision = 10U) {
  return std::make_shared<CallbackExecutedReferenceQuery>(
      [](double w, ExecutedReferenceQueryResult& result) {
        result.r = Eigen::Vector3d(w, 1.0, 2.0);
        result.r_w = Eigen::Vector3d(1.0, 0.0, 0.0);
        result.valid = true;
        return true;
      }, 0.0, 10.0, path_revision, frame_revision, owner_revision,
      query_revision, "test-query");
}

ActiveReferenceSnapshot Snapshot() {
  ActiveReferenceSnapshot snapshot;
  snapshot.valid = true;
  snapshot.authority_session = 3U;
  snapshot.sequence = 1U;
  snapshot.planner_path_revision = 7U;
  snapshot.executed_path_revision = 7U;
  snapshot.frame_revision = 8U;
  snapshot.owner_mode = ActiveReferenceOwnerMode::NORMAL;
  snapshot.selected_u_owner = "PhaseOffsetAllocator";
  snapshot.w = 1.0;
  snapshot.delta = 0.02;
  snapshot.dt = 0.1;
  snapshot.u_prev = phase_offset_core::PortCommand();
  snapshot.selected_u.u_w = 0.2;
  snapshot.selected_u.u_delta = -0.1;
  snapshot.selected_u_w = 0.2;
  snapshot.selected_u_delta = -0.1;
  snapshot.proposed_next_w = 1.02;
  snapshot.proposed_next_delta = 0.01;
  snapshot.proposed_next_u_prev = snapshot.selected_u;
  snapshot.r = Eigen::Vector3d(1.0, 1.0, 2.0);
  snapshot.r_w = Eigen::Vector3d::UnitX();
  snapshot.executed_reference_query = Query();
  snapshot.reference_query_revision = 10U;
  return snapshot;
}

// Every authority tick after the seed must begin from the exact predecessor
// state committed by the previous tick.  Keep the fixture values readable by
// deriving that predecessor here, including the immutable reference query
// jet bound to the new phase.
void ChainFromPredecessor(const ActiveReferenceSnapshot& predecessor,
                          ActiveReferenceSnapshot& candidate) {
  candidate.w = predecessor.proposed_next_w;
  candidate.delta = predecessor.proposed_next_delta;
  candidate.u_prev = predecessor.proposed_next_u_prev;
  if (candidate.executed_reference_query) {
    ExecutedReferenceQueryResult reference;
    ASSERT_TRUE(candidate.executed_reference_query->query(candidate.w,
                                                          reference));
    ASSERT_TRUE(reference.valid);
    candidate.r = reference.r;
    candidate.r_w = reference.r_w;
    candidate.r_ww = reference.r_ww;
    candidate.r_ww_valid = reference.r_ww_valid;
  }
}

TEST(ActiveReferenceAuthorityTest, PrepareIsSideEffectFreeAndCommitIsExact) {
  PhaseOffsetExecutionAuthority authority;
  ASSERT_TRUE(authority.seed(Snapshot()));
  ActiveReferenceSnapshot before = authority.snapshot();
  AuthorityPrepareInput input;
  input.candidate = before;
  input.candidate.sequence = before.sequence;
  input.candidate.selected_u.u_w = 0.17;
  input.candidate.selected_u.u_delta = -0.04;
  input.candidate.selected_u_w = 0.17;
  input.candidate.selected_u_delta = -0.04;
  ChainFromPredecessor(before, input.candidate);
  input.candidate.proposed_next_w = input.candidate.w +
      input.candidate.dt * (input.candidate.matched_base_w_dot +
                            input.candidate.selected_u.u_w);
  input.candidate.proposed_next_delta = input.candidate.delta +
      input.candidate.dt * input.candidate.selected_u.u_delta;
  input.candidate.proposed_next_u_prev = input.candidate.selected_u;
  AuthorityPreparedStep prepared;
  ASSERT_TRUE(authority.prepare(input, prepared));
  EXPECT_EQ(authority.snapshot().sequence, before.sequence);
  EXPECT_DOUBLE_EQ(authority.snapshot().delta, before.delta);
  AuthorityCommitResult result;
  ASSERT_TRUE(authority.commit(
      prepared,
      [](const ActiveReferenceSnapshot&) { return true; }, result));
  EXPECT_TRUE(result.committed);
  EXPECT_DOUBLE_EQ(result.snapshot.selected_u.u_w, 0.17);
  EXPECT_DOUBLE_EQ(result.snapshot.selected_u.u_delta, -0.04);
  EXPECT_EQ(result.snapshot.sequence, before.sequence + 1U);
  EXPECT_EQ(result.diagnostics.snapshot_id, result.snapshot.snapshotId());
}

TEST(ActiveReferenceAuthorityTest,
     CommitUsesImmutablePreparedSnapshotAfterPublicDtoMutation) {
  PhaseOffsetExecutionAuthority authority;
  ASSERT_TRUE(authority.seed(Snapshot()));
  AuthorityPrepareInput input;
  input.candidate = authority.snapshot();
  input.candidate.selected_u.u_w = 0.17;
  input.candidate.selected_u_w = 0.17;
  ChainFromPredecessor(input.candidate, input.candidate);
  input.candidate.proposed_next_w = input.candidate.w +
      input.candidate.dt * (input.candidate.matched_base_w_dot +
                            input.candidate.selected_u.u_w);
  input.candidate.proposed_next_delta = input.candidate.delta +
      input.candidate.dt * input.candidate.selected_u.u_delta;
  input.candidate.proposed_next_u_prev = input.candidate.selected_u;
  AuthorityPreparedStep prepared;
  ASSERT_TRUE(authority.prepare(input, prepared));
  ASSERT_TRUE(prepared.committed_snapshot);

  // The public DTO is retained for diagnostics/backward compatibility.  A
  // caller must not be able to alter the value that the transaction commits.
  prepared.candidate.selected_u.u_w = 0.91;
  prepared.candidate.selected_u_w = 0.91;
  prepared.candidate.proposed_next_w = 1.091;
  prepared.candidate.proposed_next_u_prev = prepared.candidate.selected_u;

  ActiveReferenceSnapshot published;
  AuthorityCommitResult result;
  ASSERT_TRUE(authority.commit(
      prepared,
      [&published](const ActiveReferenceSnapshot& snapshot) {
        published = snapshot;
        return true;
      },
      result));
  EXPECT_DOUBLE_EQ(published.selected_u.u_w, 0.17);
  EXPECT_DOUBLE_EQ(result.snapshot.selected_u.u_w, 0.17);
  EXPECT_DOUBLE_EQ(authority.snapshot().selected_u.u_w, 0.17);
}

TEST(ActiveReferenceAuthorityTest, StaleAndUnsafeNeverMutateSnapshot) {
  PhaseOffsetExecutionAuthority authority;
  ASSERT_TRUE(authority.seed(Snapshot()));
  const ActiveReferenceSnapshot before = authority.snapshot();
  AuthorityPrepareInput input;
  input.candidate = before;
  input.stale = true;
  AuthorityPreparedStep prepared;
  EXPECT_FALSE(authority.prepare(input, prepared));
  EXPECT_EQ(prepared.status, ExecutionAuthorityStatus::STALE);
  input.stale = false;
  input.current_state_unsafe = true;
  EXPECT_FALSE(authority.prepare(input, prepared));
  EXPECT_EQ(prepared.status, ExecutionAuthorityStatus::CURRENT_STATE_UNSAFE);
  EXPECT_EQ(authority.snapshot().snapshotId(), before.snapshotId());
}

TEST(ActiveReferenceAuthorityTest, PublisherReceivesSameSnapshotAsDiagnostics) {
  PhaseOffsetExecutionAuthority authority;
  ASSERT_TRUE(authority.seed(Snapshot()));
  AuthorityPrepareInput input;
  input.candidate = authority.snapshot();
  ChainFromPredecessor(input.candidate, input.candidate);
  input.candidate.proposed_next_w = input.candidate.w +
      input.candidate.dt * (input.candidate.matched_base_w_dot +
                            input.candidate.selected_u.u_w);
  input.candidate.proposed_next_delta = input.candidate.delta +
      input.candidate.dt * input.candidate.selected_u.u_delta;
  input.candidate.proposed_next_u_prev = input.candidate.selected_u;
  AuthorityPreparedStep prepared;
  ASSERT_TRUE(authority.prepare(input, prepared));
  AuthorityCommitResult result;
  ActiveReferenceSnapshot published;
  ASSERT_TRUE(authority.commit(prepared,
      [&published](const ActiveReferenceSnapshot& snapshot) {
        published = snapshot;
        return true;
      }, result));
  EXPECT_EQ(published.snapshotId(), result.diagnostics.snapshot_id);
  EXPECT_DOUBLE_EQ(published.selected_u.u_w, result.diagnostics.selected_u.u_w);
}

TEST(ActiveReferenceAuthorityTest,
     PublisherFailureCommitsNothingAndRetainsExactState) {
  PhaseOffsetExecutionAuthority authority;
  ASSERT_TRUE(authority.seed(Snapshot()));
  const ActiveReferenceSnapshot before = authority.snapshot();
  AuthorityPrepareInput input;
  input.candidate = before;
  input.candidate.selected_u.u_w = 0.13;
  input.candidate.selected_u_w = 0.13;
  ChainFromPredecessor(before, input.candidate);
  input.candidate.proposed_next_w = input.candidate.w +
      input.candidate.dt * (input.candidate.matched_base_w_dot +
                            input.candidate.selected_u.u_w);
  input.candidate.proposed_next_delta = input.candidate.delta +
      input.candidate.dt * input.candidate.selected_u.u_delta;
  input.candidate.proposed_next_u_prev = input.candidate.selected_u;
  AuthorityPreparedStep prepared;
  ASSERT_TRUE(authority.prepare(input, prepared));
  AuthorityCommitResult result;
  EXPECT_FALSE(authority.commit(
      prepared, [](const ActiveReferenceSnapshot&) { return false; }, result));
  EXPECT_EQ(result.status, ExecutionAuthorityStatus::PUBLISH_FAILED);
  EXPECT_FALSE(result.committed);
  EXPECT_EQ(authority.snapshot().snapshotId(), before.snapshotId());
  EXPECT_DOUBLE_EQ(authority.snapshot().selected_u.u_w,
                   before.selected_u.u_w);
}

TEST(ActiveReferenceAuthorityTest, TransactionConflictCommitsNothing) {
  PhaseOffsetExecutionAuthority authority;
  ASSERT_TRUE(authority.seed(Snapshot()));
  AuthorityPrepareInput first_input;
  first_input.candidate = authority.snapshot();
  ChainFromPredecessor(authority.snapshot(), first_input.candidate);
  first_input.candidate.selected_u.u_w = 0.11;
  first_input.candidate.selected_u_w = 0.11;
  first_input.candidate.proposed_next_w =
      first_input.candidate.w + first_input.candidate.dt *
          (first_input.candidate.matched_base_w_dot +
           first_input.candidate.selected_u.u_w);
  first_input.candidate.proposed_next_delta =
      first_input.candidate.delta + first_input.candidate.dt *
          first_input.candidate.selected_u.u_delta;
  first_input.candidate.proposed_next_u_prev = first_input.candidate.selected_u;
  AuthorityPreparedStep first_prepared;
  ASSERT_TRUE(authority.prepare(first_input, first_prepared));

  AuthorityPrepareInput competing_input;
  competing_input.candidate = authority.snapshot();
  ChainFromPredecessor(authority.snapshot(), competing_input.candidate);
  competing_input.candidate.selected_u.u_w = 0.12;
  competing_input.candidate.selected_u_w = 0.12;
  competing_input.candidate.proposed_next_w =
      competing_input.candidate.w + competing_input.candidate.dt *
          (competing_input.candidate.matched_base_w_dot +
           competing_input.candidate.selected_u.u_w);
  competing_input.candidate.proposed_next_delta =
      competing_input.candidate.delta + competing_input.candidate.dt *
          competing_input.candidate.selected_u.u_delta;
  competing_input.candidate.proposed_next_u_prev =
      competing_input.candidate.selected_u;
  AuthorityPreparedStep competing_prepared;
  ASSERT_TRUE(authority.prepare(competing_input, competing_prepared));
  AuthorityCommitResult competing_result;
  ASSERT_TRUE(authority.commit(
      competing_prepared,
      [](const ActiveReferenceSnapshot&) { return true; }, competing_result));
  const ActiveReferenceSnapshot after_competing = authority.snapshot();

  AuthorityCommitResult first_result;
  EXPECT_FALSE(authority.commit(
      first_prepared,
      [](const ActiveReferenceSnapshot&) { return true; }, first_result));
  EXPECT_EQ(first_result.status, ExecutionAuthorityStatus::TRANSACTION_CONFLICT);
  EXPECT_EQ(authority.snapshot().snapshotId(), after_competing.snapshotId());
}

TEST(ActiveReferenceAuthorityTest, ReferenceRevisionMismatchFailsClosed) {
  PhaseOffsetExecutionAuthority authority;
  ASSERT_TRUE(authority.seed(Snapshot()));
  AuthorityPrepareInput input;
  input.candidate = authority.snapshot();
  input.candidate.executed_path_revision = 99U;
  AuthorityPreparedStep prepared;
  EXPECT_FALSE(authority.prepare(input, prepared));
  EXPECT_EQ(prepared.status, ExecutionAuthorityStatus::REFERENCE_MISMATCH);
  EXPECT_EQ(authority.snapshot().snapshotId(), Snapshot().snapshotId());
}

TEST(ActiveReferenceAuthorityTest, PreparedNextStateMismatchCommitsNothing) {
  PhaseOffsetExecutionAuthority authority;
  ASSERT_TRUE(authority.seed(Snapshot()));
  const ActiveReferenceSnapshot before = authority.snapshot();
  AuthorityPrepareInput input;
  input.candidate = before;
  input.candidate.proposed_next_w += 0.01;
  AuthorityPreparedStep prepared;
  EXPECT_FALSE(authority.prepare(input, prepared));
  EXPECT_EQ(prepared.status, ExecutionAuthorityStatus::INVALID);
  EXPECT_EQ(authority.snapshot().snapshotId(), before.snapshotId());
  EXPECT_DOUBLE_EQ(authority.snapshot().delta, before.delta);
}

TEST(ActiveReferenceAuthorityTest, RecoveryReplanRetainsNonzeroSnapshot) {
  PhaseOffsetExecutionAuthority authority;
  ASSERT_TRUE(authority.seed(Snapshot()));
  const ActiveReferenceSnapshot before = authority.snapshot();
  AuthorityPrepareInput input;
  input.candidate = before;
  input.recovery_replan_required = true;
  AuthorityPreparedStep prepared;
  EXPECT_FALSE(authority.prepare(input, prepared));
  EXPECT_EQ(prepared.status, ExecutionAuthorityStatus::RECOVERY_REPLAN_REQUIRED);
  EXPECT_DOUBLE_EQ(authority.snapshot().delta, before.delta);
  EXPECT_EQ(authority.snapshot().snapshotId(), before.snapshotId());
}

TEST(ActiveReferenceAuthorityTest, ProductionOwnerContractIsExact) {
  PhaseOffsetExecutionAuthority authority;
  ActiveReferenceSnapshot normal = Snapshot();
  normal.selected_u_owner = "PhaseOffsetMatchedAdapterRuntime";
  EXPECT_FALSE(authority.seed(normal));

  ActiveReferenceSnapshot recovery = Snapshot();
  recovery.owner_mode = ActiveReferenceOwnerMode::RECOVERY;
  recovery.selected_u_owner = "RecoveryOwnerAlias";
  EXPECT_FALSE(authority.seed(recovery));
  recovery.selected_u_owner = "PhaseOffsetRecoveryOwner";
  EXPECT_TRUE(authority.seed(recovery));
}

TEST(ActiveReferenceAuthorityTest, UnknownOwnerModeFailsClosed) {
  PhaseOffsetExecutionAuthority authority;
  ActiveReferenceSnapshot unknown = Snapshot();
  unknown.owner_mode = ActiveReferenceOwnerMode::NONE;
  unknown.selected_u_owner = "PhaseOffsetAllocator";
  EXPECT_FALSE(authority.seed(unknown));
}

TEST(ActiveReferenceAuthorityTest, SeedRejectsNonFiniteNextPreviousPort) {
  PhaseOffsetExecutionAuthority authority;
  ActiveReferenceSnapshot invalid = Snapshot();
  invalid.proposed_next_u_prev.u_w =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(authority.seed(invalid));
}

TEST(ActiveReferenceAuthorityTest,
     SeededRecoveryEndToEndPublishesThenCommitsAndNeutralHandoffs) {
  TubeProfile profile;
  profile.complete = true;
  profile.filtered_complete = true;
  profile.preview_start_w = 0.0;
  profile.preview_end_w = 1.0;
  for (int index = 0; index < 2; ++index) {
    TubeRawSample sample;
    sample.w = static_cast<double>(index);
    sample.filtered_lower = -0.4;
    sample.filtered_upper = 0.4;
    sample.complete = true;
    profile.samples.push_back(sample);
  }
  PreviewFeasibilityInput preview_input;
  preview_input.profile = &profile;
  preview_input.current_w = 0.2;
  preview_input.current_delta = 0.2;
  preview_input.target_w = 0.2;
  preview_input.target_delta = 0.0;
  preview_input.dt = 0.1;
  preview_input.horizon = 1.0;
  preview_input.max_phase_rate = 1.0;
  preview_input.max_delta_rate = 2.0;
  preview_input.max_delta_slew = 2.0;
  preview_input.path_revision = 7U;
  preview_input.frame_revision = 8U;
  preview_input.profile_revision = 9U;
  preview_input.expected_path_revision = 7U;
  preview_input.expected_frame_revision = 8U;
  preview_input.expected_profile_revision = 9U;
  PreviewFeasibilityResult preview;
  ASSERT_TRUE(PreviewFeasibility::evaluate(preview_input, preview));

  HandoffStateMachine handoff_machine;
  HandoffStateInput handoff_input;
  handoff_input.owner_mode = ActiveReferenceOwnerMode::RECOVERY;
  handoff_input.delta = 0.2;
  handoff_input.successor_available = true;
  handoff_input.successor_contains_current_delta = true;
  handoff_input.preview_feasible = preview.valid;
  handoff_input.preview_target_overlap = preview.target_overlap;
  handoff_input.event = HandoffEvent::TEMPORARY_DENY;
  HandoffDecision handoff;
  ASSERT_TRUE(handoff_machine.evaluate(handoff_input, handoff));
  EXPECT_TRUE(handoff.retain_current_owner);
  EXPECT_FALSE(handoff.planner_veto);

  ImmutableExecutedReferenceQueryPtr query = Query();
  RecoveryReferenceJet jet;
  ASSERT_TRUE(makeRecoveryReferenceJet(*query, 0.2, 0.2, jet));
  RecoveryPrepareInput recovery_input;
  recovery_input.recovery_session = 11U;
  recovery_input.source_path_revision = 7U;
  recovery_input.target_path_revision = 7U;
  recovery_input.source_frame_revision = 8U;
  recovery_input.target_frame_revision = 8U;
  recovery_input.source_owner_revision = 9U;
  recovery_input.target_owner_revision = 9U;
  recovery_input.reference_jet = jet;
  recovery_input.provenance = "seeded-recovery-input";
  recovery_input.current_w = 0.2;
  recovery_input.current_delta = 0.2;
  recovery_input.dt = 0.1;
  recovery_input.measure = 0.2;
  recovery_input.now = 0.0;
  recovery_input.deadline = PhaseOffsetRecoveryOwner::computeDeadline(
      0.0, 0.2, 1e-3, 0.01, 0.2);
  recovery_input.phase_domain_start = 0.0;
  recovery_input.phase_domain_end = 1.0;
  recovery_input.max_slew_rate = 2.0;
  RecoveryCandidate candidate;
  candidate.command.u_w = 0.1;
  candidate.command.u_delta = -1.0;
  candidate.next_w = 0.21;
  candidate.next_delta = 0.1;
  candidate.measure = 0.2;
  candidate.next_measure_upper_bound = 0.1;
  candidate.progress_bound_valid = true;
  candidate.forward_progress = 0.1;
  candidate.slew_cost = 1.0;
  candidate.admissible_u_w_lower = -1.0;
  candidate.admissible_u_w_upper = 1.0;
  candidate.admissible_u_delta_lower = -2.0;
  candidate.admissible_u_delta_upper = 2.0;
  candidate.command_membership_valid = true;
  candidate.closed_bounded_set_valid = true;
  candidate.phase_domain_start = 0.0;
  candidate.phase_domain_end = 1.0;
  candidate.reference_jet = jet;
  candidate.provenance = "seeded-recovery-fixture";
  candidate.s_dot = 0.1;
  candidate.v_s_min = 1e-3;
  candidate.time_progress_valid = true;
  candidate.progress_provenance = "seeded-recovery-fixture/P_k";
  recovery_input.candidates.push_back(candidate);
  PhaseOffsetRecoveryOwner recovery_owner;
  RecoveryPreparedStep recovery_step;
  ASSERT_TRUE(recovery_owner.prepare(recovery_input, recovery_step));
  EXPECT_EQ(recovery_step.selected_u_owner, "PhaseOffsetRecoveryOwner");
  EXPECT_DOUBLE_EQ(recovery_step.progress, 0.1);

  ActiveReferenceSnapshot initial = Snapshot();
  initial.owner_mode = ActiveReferenceOwnerMode::RECOVERY;
  initial.selected_u_owner = "PhaseOffsetRecoveryOwner";
  initial.authority_session = 11U;
  initial.w = 0.2;
  initial.delta = 0.2;
  initial.dt = 0.1;
  initial.selected_u = recovery_step.selected_u;
  initial.selected_u_w = recovery_step.selected_u.u_w;
  initial.selected_u_delta = recovery_step.selected_u.u_delta;
  initial.proposed_next_w = recovery_step.next_w;
  initial.proposed_next_delta = recovery_step.next_delta;
  initial.proposed_next_u_prev = recovery_step.selected_u;
  initial.executed_reference_query = query;
  initial.reference_query_revision = query->queryRevision();
  ExecutedReferenceQueryResult reference;
  ASSERT_TRUE(query->query(initial.w, reference));
  initial.r = reference.r;
  initial.r_w = reference.r_w;
  PhaseOffsetExecutionAuthority authority;
  ASSERT_TRUE(authority.seed(initial));

  AuthorityPrepareInput authority_input;
  authority_input.candidate = initial;
  ChainFromPredecessor(initial, authority_input.candidate);
  authority_input.candidate.sequence = initial.sequence;
  authority_input.candidate.proposed_next_w = authority_input.candidate.w +
      authority_input.candidate.dt *
          (authority_input.candidate.matched_base_w_dot +
           authority_input.candidate.selected_u.u_w);
  authority_input.candidate.proposed_next_delta =
      authority_input.candidate.delta +
      authority_input.candidate.dt * authority_input.candidate.selected_u.u_delta;
  authority_input.candidate.proposed_next_u_prev =
      authority_input.candidate.selected_u;
  AuthorityPreparedStep authority_prepared;
  ASSERT_TRUE(authority.prepare(authority_input, authority_prepared));
  bool position_command_published = false;
  AuthorityCommitResult authority_result;
  ASSERT_TRUE(authority.commit(
      authority_prepared,
      [&position_command_published](const ActiveReferenceSnapshot& snapshot) {
        ExecutedReferenceQueryResult governor_reference;
        const bool governor_ok = snapshot.executed_reference_query &&
            snapshot.executed_reference_query->query(
                snapshot.w, governor_reference) && governor_reference.valid;
        position_command_published = governor_ok;
        return governor_ok;  // local PositionCommand publication succeeds
      }, authority_result));
  EXPECT_TRUE(position_command_published);
  EXPECT_TRUE(authority_result.committed);
  EXPECT_DOUBLE_EQ(authority.snapshot().selected_u.u_delta, -1.0);

  ActiveReferenceSnapshot neutral = authority.snapshot();
  neutral.owner_mode = ActiveReferenceOwnerMode::RECOVERY;
  neutral.selected_u_owner = "PhaseOffsetRecoveryOwner";
  ChainFromPredecessor(authority.snapshot(), neutral);
  neutral.selected_u = phase_offset_core::PortCommand();
  neutral.selected_u_w = 0.0;
  neutral.selected_u_delta = 0.0;
  neutral.proposed_next_w = neutral.w + neutral.dt *
      (neutral.matched_base_w_dot + neutral.selected_u.u_w);
  neutral.proposed_next_delta = 0.0;
  neutral.proposed_next_u_prev = neutral.selected_u;
  AuthorityPrepareInput neutral_input;
  neutral_input.candidate = neutral;
  AuthorityPreparedStep neutral_prepared;
  ASSERT_TRUE(authority.prepare(neutral_input, neutral_prepared));
  AuthorityCommitResult neutral_result;
  ASSERT_TRUE(authority.commit(
      neutral_prepared,
      [](const ActiveReferenceSnapshot&) { return true; }, neutral_result));
  HandoffStateInput neutral_handoff;
  neutral_handoff.delta = 0.0;
  neutral_handoff.event = HandoffEvent::ATOMIC_NEUTRAL_HANDOFF;
  neutral_handoff.neutral_handoff_committed = true;
  ASSERT_TRUE(handoff_machine.evaluate(neutral_handoff, handoff));
  EXPECT_TRUE(handoff.planner_only_allowed);
  EXPECT_EQ(handoff.next_state, HandoffState::PLANNER_ONLY);
}

}  // namespace
}  // namespace phase_offset_navigation

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
