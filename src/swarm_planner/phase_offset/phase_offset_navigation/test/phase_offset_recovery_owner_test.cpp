#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "phase_offset_navigation/phase_offset_recovery_owner.h"

namespace phase_offset_navigation {
namespace {

RecoveryReferenceJet Jet() {
  RecoveryReferenceJet jet;
  jet.valid = true;
  jet.path_revision = 1U;
  jet.frame_revision = 2U;
  jet.owner_revision = 3U;
  jet.query_revision = 4U;
  jet.r = Eigen::Vector3d(1.0, 0.0, 0.0);
  jet.r_w = Eigen::Vector3d::UnitX();
  jet.provenance = "test-reference-jet";
  return jet;
}

RecoveryPrepareInput Input() {
  RecoveryPrepareInput input;
  input.recovery_session = 5U;
  input.source_path_revision = 1U;
  input.target_path_revision = 1U;
  input.source_frame_revision = 2U;
  input.target_frame_revision = 2U;
  input.source_owner_revision = 3U;
  input.target_owner_revision = 3U;
  input.reference_jet = Jet();
  input.current_w = 0.0;
  input.current_delta = 0.2;
  input.dt = 0.1;
  input.measure = 2.0;
  input.now = 0.0;
  input.deadline = 10.0;
  input.phase_domain_start = -1.0;
  input.phase_domain_end = 1.0;
  input.max_slew_rate = 1.0;
  input.v_s_min = 0.1;
  input.provenance = "test-recovery-input";
  input.reference_jet.w = input.current_w;
  input.reference_jet.delta = input.current_delta;
  return input;
}

RecoveryCandidate Candidate(const double u_w, const double u_delta,
                            const double progress,
                            const double forward_progress,
                            const double slew_cost) {
  RecoveryCandidate candidate;
  candidate.command.u_w = u_w;
  candidate.command.u_delta = u_delta;
  candidate.next_w = 0.01;
  candidate.next_delta = 0.2 + 0.1 * u_delta;
  candidate.progress = progress;
  candidate.forward_progress = forward_progress;
  candidate.slew_cost = slew_cost;
  candidate.reference_jet = Jet();
  candidate.reference_jet.w = 0.0;
  candidate.reference_jet.delta = 0.2;
  candidate.admissible_u_w_lower = -10.0;
  candidate.admissible_u_w_upper = 10.0;
  candidate.admissible_u_delta_lower = -10.0;
  candidate.admissible_u_delta_upper = 10.0;
  candidate.measure = 2.0;
  candidate.next_measure_upper_bound = 2.0 - progress;
  candidate.progress_bound_valid = true;
  candidate.progress_provenance = "test-recovery-candidate/P_k";
  candidate.phase_domain_start = -1.0;
  candidate.phase_domain_end = 1.0;
  candidate.s_dot = 0.1;
  candidate.v_s_min = 0.1;
  candidate.provenance = "test-recovery-candidate";
  return candidate;
}

TEST(RecoveryOwnerTest, SelectionIsOrderIndependentAndUsesFrozenPriority) {
  PhaseOffsetRecoveryOwner owner;
  RecoveryPrepareInput input = Input();
  RecoveryCandidate first;
  first.command.u_w = 0.1;
  first.command.u_delta = -0.5;
  first.progress = 0.02;
  first.forward_progress = 0.2;
  first.slew_cost = 1.0;
  first.reference_jet = Jet();
  first.reference_jet.w = 0.0;
  first.reference_jet.delta = 0.2;
  first.admissible_u_w_lower = -10.0;
  first.admissible_u_w_upper = 10.0;
  first.admissible_u_delta_lower = -10.0;
  first.admissible_u_delta_upper = 10.0;
  first.measure = 2.0;
  first.next_measure_upper_bound = 2.0 - first.progress;
  first.progress_bound_valid = true;
  first.phase_domain_start = -1.0;
  first.phase_domain_end = 1.0;
  first.s_dot = 0.1;
  first.v_s_min = 0.1;
  first.next_w = 0.01;
  first.next_delta = 0.2 + 0.1 * first.command.u_delta;
  first.progress_provenance = "first/P_k";
  first.provenance = "first";
  RecoveryCandidate second = first;
  second.command.u_w = 0.2;
  second.progress = 0.03;
  second.measure = 2.0;
  second.next_measure_upper_bound = 2.0 - second.progress;
  second.forward_progress = 0.1;
  second.slew_cost = 2.0;
  second.provenance = "second";
  input.candidates.push_back(first);
  input.candidates.push_back(second);
  RecoveryPreparedStep prepared;
  ASSERT_TRUE(owner.prepare(input, prepared));
  EXPECT_DOUBLE_EQ(prepared.selected_u.u_w, 0.2);
  std::reverse(input.candidates.begin(), input.candidates.end());
  RecoveryPreparedStep reversed;
  ASSERT_TRUE(owner.prepare(input, reversed));
  EXPECT_DOUBLE_EQ(reversed.selected_u.u_w, prepared.selected_u.u_w);
}

TEST(RecoveryOwnerTest, EmptySetRequiresReplanWithoutClearingAuthority) {
  PhaseOffsetRecoveryOwner owner;
  RecoveryPrepareInput input = Input();
  RecoveryPreparedStep prepared;
  EXPECT_FALSE(owner.prepare(input, prepared));
  EXPECT_EQ(prepared.status, RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED);
  EXPECT_FALSE(owner.status().active);
}

TEST(RecoveryOwnerTest, MissingRevisionOrProvenanceEvidenceFailsClosed) {
  PhaseOffsetRecoveryOwner owner;
  RecoveryPrepareInput input = Input();
  input.source_frame_revision = 0U;
  RecoveryPreparedStep prepared;
  EXPECT_FALSE(owner.prepare(input, prepared));
  EXPECT_EQ(prepared.status, RecoveryStepStatus::REFERENCE_MISMATCH);

  input = Input();
  input.provenance.clear();
  EXPECT_FALSE(owner.prepare(input, prepared));
  EXPECT_EQ(prepared.status, RecoveryStepStatus::INVALID);
}

TEST(RecoveryOwnerTest, MissingCandidateProofEvidenceRequiresReplan) {
  PhaseOffsetRecoveryOwner owner;
  RecoveryPrepareInput input = Input();
  RecoveryCandidate candidate = Candidate(0.2, 0.0, 1.0, 0.1, 0.0);
  candidate.progress_bound_valid = false;
  input.candidates.push_back(candidate);
  RecoveryPreparedStep prepared;
  EXPECT_FALSE(owner.prepare(input, prepared));
  EXPECT_EQ(prepared.status, RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED);
}

TEST(RecoveryOwnerTest, InexactCandidateNextStateRequiresReplan) {
  PhaseOffsetRecoveryOwner owner;
  RecoveryPrepareInput input = Input();
  RecoveryCandidate candidate = Candidate(0.2, 0.0, 1.0, 0.1, 0.0);
  candidate.next_w += 0.01;
  input.candidates.push_back(candidate);
  RecoveryPreparedStep prepared;
  EXPECT_FALSE(owner.prepare(input, prepared));
  EXPECT_EQ(prepared.status, RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED);
}

TEST(RecoveryOwnerTest, DeadlineExpiryRequiresRenewedEvidence) {
  PhaseOffsetRecoveryOwner owner;
  RecoveryPrepareInput input = Input();
  input.now = 10.0;
  RecoveryPreparedStep prepared;
  EXPECT_FALSE(owner.prepare(input, prepared));
  EXPECT_EQ(prepared.status, RecoveryStepStatus::DEADLINE_EXPIRED);
}

TEST(RecoveryOwnerTest, ProgressToleranceGroupsBeforeForwardPriority) {
  PhaseOffsetRecoveryOwnerConfig config;
  config.recovery_progress_tolerance = 0.1;
  PhaseOffsetRecoveryOwner owner(config);
  RecoveryPrepareInput input = Input();
  input.candidates.push_back(Candidate(0.4, 0.0, 1.0, 0.1, 1.0));
  input.candidates.push_back(Candidate(0.2, 0.0, 1.05, 0.2, 2.0));

  RecoveryPreparedStep prepared;
  ASSERT_TRUE(owner.prepare(input, prepared));
  EXPECT_DOUBLE_EQ(prepared.selected_u.u_w, 0.2);
  EXPECT_DOUBLE_EQ(prepared.progress, 1.05);
}

TEST(RecoveryOwnerTest, ForwardToleranceGroupsBeforeSlewPriority) {
  PhaseOffsetRecoveryOwnerConfig config;
  config.recovery_forward_progress_tolerance = 0.1;
  PhaseOffsetRecoveryOwner owner(config);
  RecoveryPrepareInput input = Input();
  input.candidates.push_back(Candidate(0.4, 0.0, 1.0, 0.2, 2.0));
  input.candidates.push_back(Candidate(0.2, 0.0, 1.0, 0.25, 1.0));

  RecoveryPreparedStep prepared;
  ASSERT_TRUE(owner.prepare(input, prepared));
  EXPECT_DOUBLE_EQ(prepared.selected_u.u_w, 0.2);
}

TEST(RecoveryOwnerTest, SlewToleranceUsesNumericalCommandTieBreaks) {
  PhaseOffsetRecoveryOwnerConfig config;
  config.recovery_slew_cost_tolerance = 0.1;
  PhaseOffsetRecoveryOwner owner(config);
  RecoveryPrepareInput input = Input();
  input.candidates.push_back(Candidate(0.3, 0.8, 1.0, 0.2, 1.0));
  input.candidates.push_back(Candidate(0.2, 0.9, 1.0, 0.2, 1.05));
  input.candidates.push_back(Candidate(0.2, 0.7, 1.0, 0.2, 1.05));

  RecoveryPreparedStep prepared;
  ASSERT_TRUE(owner.prepare(input, prepared));
  EXPECT_DOUBLE_EQ(prepared.selected_u.u_w, 0.2);
  EXPECT_DOUBLE_EQ(prepared.selected_u.u_delta, 0.7);
}

TEST(RecoveryOwnerTest, ExactArrivalDoesNotOverrideFrozenProgressPriority) {
  PhaseOffsetRecoveryOwnerConfig config;
  config.recovery_progress_tolerance = 0.1;
  RecoveryCandidate ordinary = Candidate(0.1, -0.2, 1.0, 1.1, 0.0);
  RecoveryCandidate exact = Candidate(0.2, -0.3, 0.95, 1.0, 0.0);
  exact.next_measure_upper_bound = 2.0 - 0.95;
  ordinary.exact_terminal_predicate = false;
  exact.exact_terminal_predicate = true;

  RecoveryCandidate selected;
  ASSERT_TRUE(PhaseOffsetRecoveryOwner::selectDeterministic(
      {exact, ordinary}, config, selected));
  // Progress values are in one tolerance group, so the frozen forward
  // priority selects the nonterminal candidate in either container order.
  EXPECT_DOUBLE_EQ(selected.command.u_w, ordinary.command.u_w);
  EXPECT_DOUBLE_EQ(selected.command.u_delta, ordinary.command.u_delta);
  EXPECT_FALSE(selected.exact_terminal_predicate);

  std::vector<RecoveryCandidate> reversed{ordinary, exact};
  ASSERT_TRUE(PhaseOffsetRecoveryOwner::selectDeterministic(
      reversed, config, selected));
  EXPECT_DOUBLE_EQ(selected.command.u_w, ordinary.command.u_w);
  EXPECT_FALSE(selected.exact_terminal_predicate);
}

TEST(RecoveryOwnerTest, InsufficientProgressRequiresReplan) {
  PhaseOffsetRecoveryOwner owner;
  RecoveryPrepareInput input = Input();
  input.dt = 0.1;
  input.candidates.push_back(Candidate(0.1, 0.0, 1e-6, 0.0, 0.0));

  RecoveryPreparedStep prepared;
  EXPECT_FALSE(owner.prepare(input, prepared));
  EXPECT_EQ(prepared.status, RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED);
}

TEST(RecoveryOwnerTest,
     SmallNonterminalResidualWithZeroProgressRequiresReplan) {
  PhaseOffsetRecoveryOwnerConfig config;
  config.recovery_progress_tolerance = 1e-6;
  PhaseOffsetRecoveryOwner owner(config);
  RecoveryPrepareInput input = Input();
  input.current_delta = 5e-7;
  input.measure = input.current_delta;
  input.reference_jet.delta = input.current_delta;
  RecoveryCandidate candidate = Candidate(0.1, 0.0, 0.0, 0.0, 0.0);
  candidate.measure = input.measure;
  candidate.next_measure_upper_bound = input.measure;
  candidate.next_delta = input.current_delta;
  candidate.reference_jet.delta = input.current_delta;
  candidate.next_w = input.current_w + input.dt * candidate.s_dot;
  input.candidates.push_back(candidate);

  RecoveryPreparedStep prepared;
  EXPECT_FALSE(owner.prepare(input, prepared));
  EXPECT_EQ(prepared.status, RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED);
  EXPECT_FALSE(owner.status().active);
}

TEST(RecoveryOwnerTest,
     IneligibleNonterminalCannotBeatCertifiedExactTerminalArrival) {
  PhaseOffsetRecoveryOwnerConfig config;
  config.recovery_progress_tolerance = 1e-4;
  config.v_rec_min = 0.01;
  PhaseOffsetRecoveryOwner owner(config);
  RecoveryPrepareInput input = Input();
  input.current_delta = 8e-4;
  input.measure = input.current_delta;
  input.reference_jet.delta = input.current_delta;
  input.base_w_dot = 0.1;
  input.base_w_dot_valid = true;
  input.target_delta_lower = 0.0;
  input.target_delta_upper = 0.0;
  input.target_interval_valid = true;
  input.u_prev.u_w = 0.5;
  input.u_prev.u_delta = -0.0079;

  RecoveryCandidate exact = Candidate(0.4, -0.008, 0.0, 0.0, 0.0);
  exact.exact_terminal_predicate = true;
  exact.provenance = "PortProjector/exact-selected-ZOH-arrival";
  exact.reference_jet.delta = input.current_delta;
  exact.next_w = input.current_w + input.dt * (input.base_w_dot +
                                                exact.command.u_w);

  RecoveryCandidate nonterminal = Candidate(0.5, -0.0079, 0.0, 0.0, 0.0);
  nonterminal.reference_jet.delta = input.current_delta;
  nonterminal.next_w = input.current_w + input.dt * (input.base_w_dot +
                                                       nonterminal.command.u_w);
  input.candidates = {nonterminal, exact};

  RecoveryPreparedStep prepared;
  ASSERT_TRUE(owner.prepare(input, prepared));
  EXPECT_DOUBLE_EQ(prepared.selected_u.u_delta, exact.command.u_delta);
  EXPECT_TRUE(prepared.exact_terminal_predicate);
  EXPECT_LT(prepared.progress, prepared.required_progress);
}

TEST(RecoveryOwnerTest, GuaranteedProgressThresholdIsInclusive) {
  PhaseOffsetRecoveryOwnerConfig config;
  config.recovery_progress_tolerance = 0.0;
  config.v_rec_min = 1.0;
  PhaseOffsetRecoveryOwner owner(config);
  RecoveryPrepareInput input = Input();
  input.dt = 0.25;

  RecoveryCandidate at_threshold = Candidate(0.1, 0.0, 0.25, 0.0, 0.0);
  at_threshold.next_measure_upper_bound = at_threshold.measure - 0.25;
  at_threshold.next_w = input.current_w + input.dt * at_threshold.s_dot;
  input.candidates = {at_threshold};
  RecoveryPreparedStep prepared;
  ASSERT_TRUE(owner.prepare(input, prepared));
  EXPECT_DOUBLE_EQ(prepared.required_progress, 0.25);
  EXPECT_DOUBLE_EQ(prepared.progress, 0.25);

  RecoveryCandidate below = Candidate(0.1, 0.0, 0.25 - 1e-6, 0.0, 0.0);
  below.next_measure_upper_bound = below.measure - (0.25 - 1e-6);
  below.next_w = input.current_w + input.dt * below.s_dot;
  input.candidates = {below};
  RecoveryPreparedStep rejected;
  EXPECT_FALSE(owner.prepare(input, rejected));
  EXPECT_EQ(rejected.status, RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED);
}

TEST(RecoveryOwnerTest, CertifiedExactTerminalArrivalMayBeatOneTickBound) {
  PhaseOffsetRecoveryOwnerConfig config;
  config.recovery_progress_tolerance = 0.0;
  config.v_rec_min = 1.0;
  PhaseOffsetRecoveryOwner owner(config);
  RecoveryPrepareInput input = Input();
  input.current_delta = 5e-5;
  input.measure = input.current_delta;
  input.reference_jet.delta = input.current_delta;
  input.base_w_dot = 0.1;
  input.base_w_dot_valid = true;
  input.target_delta_lower = 0.0;
  input.target_delta_upper = 0.0;
  input.target_interval_valid = true;

  RecoveryCandidate candidate = Candidate(0.1, -5e-4, 0.0, 0.0, 0.0);
  candidate.exact_terminal_predicate = true;
  candidate.provenance = "PortProjector/exact-selected-ZOH-arrival";
  candidate.reference_jet.delta = input.current_delta;
  candidate.next_w = input.current_w + input.dt * (input.base_w_dot +
                                                     candidate.command.u_w);
  input.candidates.push_back(candidate);

  RecoveryPreparedStep prepared;
  ASSERT_TRUE(owner.prepare(input, prepared));
  EXPECT_TRUE(prepared.exact_terminal_predicate);
  EXPECT_DOUBLE_EQ(prepared.next_delta, 0.0);
  EXPECT_LT(prepared.progress, prepared.required_progress);
}

TEST(RecoveryOwnerTest, InadmissibleAndOutOfBoundsCandidatesDoNotBecomeAuthority) {
  PhaseOffsetRecoveryOwner owner;
  RecoveryPrepareInput input = Input();
  RecoveryCandidate inadmissible = Candidate(0.1, 0.0, 1.0, 1.0, 0.0);
  inadmissible.admissible = false;
  RecoveryCandidate uncertified = Candidate(0.2, 0.0, 2.0, 2.0, 0.0);
  uncertified.proof_valid = false;
  RecoveryCandidate out_of_bounds = Candidate(20.0, 0.0, 3.0, 3.0, 0.0);
  input.candidates = {inadmissible, uncertified, out_of_bounds};

  RecoveryCandidate selected;
  // True closed/bounded membership is checked before ranking; the only
  // surviving candidates are inadmissible/uncertified/out-of-envelope, so
  // no command may become RecoveryOwner authority.
  EXPECT_FALSE(PhaseOffsetRecoveryOwner::selectDeterministic(
      input.candidates, owner.config(), selected));

  RecoveryPreparedStep prepared;
  EXPECT_FALSE(owner.prepare(input, prepared));
  EXPECT_EQ(prepared.status, RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED);
  EXPECT_FALSE(owner.status().active);
}

TEST(RecoveryOwnerTest, AdmissibleSetMustBeClosedAndBounded) {
  phase_offset_core::PortProjector::AdmissibleSet set;
  set.valid = true;
  set.bounded = true;
  set.u_w.valid = true;
  set.u_delta.valid = true;
  set.u_w.lower = -1.0;
  set.u_w.upper = 1.0;
  set.u_delta.lower = -2.0;
  set.u_delta.upper = 2.0;
  EXPECT_TRUE(PhaseOffsetRecoveryOwner::verifyAdmissibleSet(set));

  set.u_w.upper = -2.0;
  EXPECT_FALSE(PhaseOffsetRecoveryOwner::verifyAdmissibleSet(set));
  set.u_w.upper = 1.0;
  set.bounded = false;
  EXPECT_FALSE(PhaseOffsetRecoveryOwner::verifyAdmissibleSet(set));
  set.bounded = true;
  set.u_delta.lower = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(PhaseOffsetRecoveryOwner::verifyAdmissibleSet(set));
}

TEST(RecoveryOwnerTest, SelectedUCommitMismatchLeavesOwnerStateUnchanged) {
  PhaseOffsetRecoveryOwner owner;
  RecoveryPrepareInput input = Input();
  RecoveryCandidate candidate = Candidate(0.2, 0.3, 1.0, 0.0, 0.0);
  candidate.next_w = 0.01;
  candidate.next_delta = 0.23;
  input.candidates.push_back(candidate);
  RecoveryPreparedStep prepared;
  ASSERT_TRUE(owner.prepare(input, prepared));
  ASSERT_TRUE(owner.commit(prepared));
  const RecoveryOwnerStatus before = owner.status();

  RecoveryPreparedStep mismatch = prepared;
  mismatch.selected_u.u_w += 0.01;
  EXPECT_FALSE(owner.commit(mismatch));
  EXPECT_EQ(owner.status().recovery_session, before.recovery_session);
  EXPECT_EQ(owner.status().committed_sequence, before.committed_sequence);
  EXPECT_EQ(owner.status().last_status, before.last_status);
  EXPECT_DOUBLE_EQ(owner.status().selected_u.u_w, before.selected_u.u_w);
  EXPECT_DOUBLE_EQ(owner.status().selected_u.u_delta, before.selected_u.u_delta);
  EXPECT_DOUBLE_EQ(owner.status().current_w, before.current_w);
  EXPECT_DOUBLE_EQ(owner.status().current_delta, before.current_delta);
  EXPECT_EQ(owner.status().active, before.active);
}

TEST(RecoveryOwnerTest, ExpiredTickRetainsPreviouslyCommittedNonzeroOwner) {
  PhaseOffsetRecoveryOwner owner;
  RecoveryPrepareInput input = Input();
  input.deadline = 1.0;
  RecoveryCandidate candidate = Candidate(0.2, 0.3, 1.0, 0.0, 0.0);
  candidate.next_w = 0.01;
  candidate.next_delta = 0.23;
  input.candidates.push_back(candidate);
  RecoveryPreparedStep prepared;
  ASSERT_TRUE(owner.prepare(input, prepared));
  ASSERT_TRUE(owner.commit(prepared));
  const RecoveryOwnerStatus before = owner.status();
  ASSERT_TRUE(before.active);
  ASSERT_TRUE(before.nonzero_authority_retained);

  input.now = input.deadline;
  RecoveryPreparedStep expired;
  EXPECT_FALSE(owner.prepare(input, expired));
  EXPECT_EQ(expired.status, RecoveryStepStatus::DEADLINE_EXPIRED);
  EXPECT_TRUE(owner.status().active);
  EXPECT_TRUE(owner.status().nonzero_authority_retained);
  EXPECT_EQ(owner.status().committed_sequence, before.committed_sequence);
  EXPECT_DOUBLE_EQ(owner.status().current_delta, before.current_delta);
  EXPECT_DOUBLE_EQ(owner.status().selected_u.u_delta, before.selected_u.u_delta);
}

TEST(RecoveryOwnerTest, DeadlineHelperIsFiniteOnlyForBoundedAssumptions) {
  const double deadline = PhaseOffsetRecoveryOwner::computeDeadline(
      2.0, 1.0, 0.5, 0.1, 0.75);
  EXPECT_TRUE(std::isfinite(deadline));
  EXPECT_DOUBLE_EQ(deadline, 4.85);
  EXPECT_TRUE(std::isnan(PhaseOffsetRecoveryOwner::computeDeadline(
      std::numeric_limits<double>::quiet_NaN(), 1.0, 0.5, 0.1, 0.75)));
  EXPECT_TRUE(std::isnan(PhaseOffsetRecoveryOwner::computeDeadline(
      2.0, -1.0, 0.5, 0.1, 0.75)));
  EXPECT_TRUE(std::isnan(PhaseOffsetRecoveryOwner::computeDeadline(
      2.0, 1.0, 0.0, 0.1, 0.75)));
  EXPECT_TRUE(std::isnan(PhaseOffsetRecoveryOwner::computeDeadline(
      2.0, 1.0, 0.5, -0.1, 0.75)));
  EXPECT_TRUE(std::isnan(PhaseOffsetRecoveryOwner::computeDeadline(
      2.0, 1.0, 0.5, 0.1,
      std::numeric_limits<double>::infinity())));
}

TEST(RecoveryOwnerTest,
     NonTransitiveToleranceChainUsesGlobalEquivalenceSetOrderIndependently) {
  PhaseOffsetRecoveryOwnerConfig config;
  config.recovery_progress_tolerance = 0.1;
  RecoveryCandidate a = Candidate(0.3, 0.0, 1.00, 0.0, 0.0);
  RecoveryCandidate b = Candidate(0.2, 0.0, 1.09, 0.0, 0.0);
  RecoveryCandidate c = Candidate(0.1, 0.0, 1.18, 0.0, 0.0);
  std::vector<RecoveryCandidate> forward = {a, b, c};
  std::vector<RecoveryCandidate> reverse = {c, b, a};
  RecoveryCandidate first;
  RecoveryCandidate second;
  ASSERT_TRUE(PhaseOffsetRecoveryOwner::selectDeterministic(
      forward, config, first));
  ASSERT_TRUE(PhaseOffsetRecoveryOwner::selectDeterministic(
      reverse, config, second));
  EXPECT_DOUBLE_EQ(first.command.u_w, second.command.u_w);
  EXPECT_DOUBLE_EQ(first.command.u_w, 0.1);
}

TEST(RecoveryOwnerTest, BoundedDeadlineRejectsUnprovedSlewOrDomain) {
  EXPECT_TRUE(std::isnan(PhaseOffsetRecoveryOwner::computeBoundedDeadline(
      0.0, 1.0, 0.5, 0.1, 0.2, 0.02, 1.0, 0.0, 0.0, 2.0)));
  EXPECT_TRUE(std::isnan(PhaseOffsetRecoveryOwner::computeBoundedDeadline(
      0.0, 1.0, 0.5, 0.1, 0.2, 0.02, 1.0, 1.0, 2.0, 2.0)));
  const double deadline = PhaseOffsetRecoveryOwner::computeBoundedDeadline(
      0.0, 1.0, 0.5, 0.1, 0.2, 0.02, 10.0, 2.0, 0.0, 2.0);
  ASSERT_TRUE(std::isfinite(deadline));
  // The recovery-rate term is the certified lower bound (1/0.5 s); the
  // actuator amplitude 10.0 is used only in the independent slew-ramp term.
  EXPECT_DOUBLE_EQ(deadline, 1.0 / 0.5 + 0.1 + 10.0 / 2.0);
}

TEST(RecoveryOwnerTest, InsufficientRemainingDomainRequiresReplan) {
  PhaseOffsetRecoveryOwner owner;
  RecoveryPrepareInput input = Input();
  input.phase_domain_start = 0.0;
  input.phase_domain_end = 0.01;
  input.required_domain_duration = 0.20;
  RecoveryCandidate candidate = Candidate(0.1, 0.0, 1.0, 0.1, 0.0);
  candidate.phase_domain_start = input.phase_domain_start;
  candidate.phase_domain_end = input.phase_domain_end;
  candidate.next_w = input.current_w + input.dt * candidate.s_dot;
  candidate.next_delta = input.current_delta;
  input.candidates.push_back(candidate);
  RecoveryPreparedStep prepared;
  EXPECT_FALSE(owner.prepare(input, prepared));
  EXPECT_EQ(prepared.status, RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED);
  EXPECT_NE(prepared.proof.find("domain"), std::string::npos);
}

TEST(RecoveryOwnerTest, TerminalPredicateIsRecomputedFromNextDelta) {
  PhaseOffsetRecoveryOwner owner;
  RecoveryPrepareInput input = Input();
  RecoveryCandidate candidate = Candidate(0.2, 0.0, 1.0, 0.1, 0.0);
  candidate.next_delta = input.current_delta;
  candidate.terminal_predicate = true;  // forged evidence must be ignored
  input.candidates.push_back(candidate);
  RecoveryPreparedStep prepared;
  ASSERT_TRUE(owner.prepare(input, prepared));
  EXPECT_FALSE(prepared.terminal_predicate);
  ASSERT_TRUE(owner.commit(prepared));
  EXPECT_FALSE(owner.status().terminal_predicate);
}

TEST(RecoveryOwnerTest,
     NearNeutralNonterminalDeltaContinuesWithoutResidualReset) {
  PhaseOffsetRecoveryOwner owner;
  RecoveryPrepareInput input = Input();
  input.current_delta = 5e-4;
  input.measure = 5e-4;
  input.reference_jet.delta = input.current_delta;
  RecoveryCandidate candidate = Candidate(0.0, -2e-3, 1.0, 0.1, 0.0);
  candidate.reference_jet.delta = input.current_delta;
  candidate.measure = input.measure;
  candidate.next_delta = 3e-4;
  candidate.next_measure_upper_bound = 3e-4;
  candidate.progress = 2e-4;
  candidate.terminal_predicate = true;  // forged near-neutral terminality
  candidate.next_w = input.current_w + input.dt * candidate.s_dot;
  input.candidates.push_back(candidate);

  RecoveryPreparedStep prepared;
  ASSERT_TRUE(owner.prepare(input, prepared));
  EXPECT_NEAR(prepared.next_delta, 3e-4, 1e-12);
  EXPECT_FALSE(prepared.terminal_predicate);
  ASSERT_TRUE(owner.commit(prepared));
  EXPECT_NEAR(owner.status().current_delta, 3e-4, 1e-12);
  EXPECT_FALSE(owner.status().terminal_predicate);
}

TEST(RecoveryOwnerTest,
     ImmutableTargetIntervalRecomputesForgedProgressOrderIndependently) {
  PhaseOffsetRecoveryOwner owner;
  RecoveryPrepareInput input = Input();
  input.measure = std::abs(input.current_delta);
  input.base_w_dot = 0.1;
  input.base_w_dot_valid = true;
  input.target_delta_lower = 0.0;
  input.target_delta_upper = 0.0;
  input.target_interval_valid = true;

  RecoveryCandidate stronger = Candidate(0.0, -1.0, -100.0, -50.0, 99.0);
  RecoveryCandidate weaker = Candidate(0.0, -0.5, 100.0, 50.0, -99.0);
  // All copied progress/next-measure/ranking values are intentionally forged;
  // prepare() must derive them from the immutable interval and ZOH command.
  stronger.next_measure_upper_bound = 0.0;
  weaker.next_measure_upper_bound = 0.0;
  input.candidates = {weaker, stronger};

  RecoveryPreparedStep prepared;
  ASSERT_TRUE(owner.prepare(input, prepared));
  EXPECT_DOUBLE_EQ(prepared.selected_u.u_delta, -1.0);
  EXPECT_NEAR(prepared.next_delta, 0.1, 1e-12);
  EXPECT_NEAR(prepared.progress, 0.1, 1e-12);
  EXPECT_NE(prepared.proof.find("RecoveryOwner/P_k=recomputed"),
            std::string::npos);

  std::reverse(input.candidates.begin(), input.candidates.end());
  RecoveryPreparedStep reversed;
  ASSERT_TRUE(owner.prepare(input, reversed));
  EXPECT_DOUBLE_EQ(reversed.selected_u.u_delta, prepared.selected_u.u_delta);
  EXPECT_DOUBLE_EQ(reversed.next_delta, prepared.next_delta);
  EXPECT_DOUBLE_EQ(reversed.progress, prepared.progress);
}

TEST(RecoveryOwnerTest, ImmutableTargetIntervalRejectsForgedCurrentMeasure) {
  PhaseOffsetRecoveryOwner owner;
  RecoveryPrepareInput input = Input();
  input.measure = 9.0;
  input.target_delta_lower = 0.0;
  input.target_delta_upper = 0.0;
  input.target_interval_valid = true;
  input.candidates.push_back(Candidate(0.0, -1.0, 1.0, 0.1, 0.0));

  RecoveryPreparedStep prepared;
  EXPECT_FALSE(owner.prepare(input, prepared));
  EXPECT_EQ(prepared.status, RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED);
  EXPECT_NE(prepared.proof.find("immutable target interval"),
            std::string::npos);
}

TEST(RecoveryOwnerTest, ExactSelectedZohArrivalIsTheOnlyTerminalProof) {
  PhaseOffsetRecoveryOwner owner;
  RecoveryPrepareInput input = Input();
  input.base_w_dot = 0.1;
  input.base_w_dot_valid = true;
  input.target_delta_lower = 0.0;
  input.target_delta_upper = 0.0;
  input.target_interval_valid = true;
  RecoveryCandidate candidate = Candidate(0.0, -2.0, 0.0, 0.1, 0.0);
  candidate.next_w = input.current_w + input.dt * 0.1;
  candidate.next_delta = 0.0;
  candidate.measure = input.current_delta;
  candidate.next_measure_upper_bound = 0.0;
  candidate.progress = input.current_delta;
  candidate.exact_terminal_predicate = true;
  candidate.provenance = "PortProjector/exact-selected-ZOH-arrival";
  input.measure = input.current_delta;
  input.candidates.push_back(candidate);
  RecoveryPreparedStep prepared;
  ASSERT_TRUE(owner.prepare(input, prepared));
  EXPECT_TRUE(prepared.exact_terminal_predicate);
  ASSERT_TRUE(owner.commit(prepared));
  EXPECT_TRUE(owner.status().exact_terminal_predicate);
}

TEST(RecoveryOwnerTest, TinyNonzeroTargetResidualIsNotCanonicalizedToTerminal) {
  const double denorm = std::numeric_limits<double>::denorm_min();
  const double residuals[] = {0.0, 1e-13, -1e-13, denorm, -denorm};
  for (const double residual : residuals) {
    PhaseOffsetRecoveryOwnerConfig config;
    // Permit a zero-progress diagnostic candidate so this test isolates the
    // exact-arrival predicate and residual preservation semantics.
    config.recovery_progress_tolerance = 1.0;
    PhaseOffsetRecoveryOwner owner(config);
    RecoveryPrepareInput input = Input();
    input.current_delta = residual;
    input.measure = std::abs(residual);
    input.reference_jet.delta = residual;
    input.base_w_dot = 0.1;
    input.base_w_dot_valid = true;
    input.target_delta_lower = 0.0;
    input.target_delta_upper = 0.0;
    input.target_interval_valid = true;

    RecoveryCandidate candidate = Candidate(0.0, 0.0, 0.0, 0.0, 0.0);
    candidate.exact_terminal_predicate = true;
    candidate.provenance = "PortProjector/exact-selected-ZOH-arrival";
    candidate.reference_jet.delta = residual;
    candidate.measure = std::abs(residual);
    candidate.next_measure_upper_bound = std::abs(residual);
    candidate.next_delta = residual;
    candidate.next_w = input.current_w + input.dt * input.base_w_dot;
    input.candidates.push_back(candidate);

    RecoveryPreparedStep prepared;
    ASSERT_TRUE(owner.prepare(input, prepared)) << residual;
    EXPECT_DOUBLE_EQ(prepared.next_delta, residual);
    EXPECT_EQ(prepared.exact_terminal_predicate, residual == 0.0);
    EXPECT_EQ(prepared.terminal_predicate, residual == 0.0);
    if (residual == 0.0) {
      ASSERT_TRUE(owner.commit(prepared));
      EXPECT_TRUE(owner.status().exact_terminal_predicate);
      EXPECT_FALSE(owner.status().nonzero_authority_retained);
    } else {
      EXPECT_FALSE(prepared.exact_terminal_predicate);
      ASSERT_TRUE(owner.commit(prepared));
      EXPECT_FALSE(owner.status().exact_terminal_predicate);
      EXPECT_TRUE(owner.status().nonzero_authority_retained);
      EXPECT_DOUBLE_EQ(owner.status().current_delta, residual);
    }
  }
}

TubeFiniteReserveV2 FiniteReserveFixture() {
  TubeFiniteReserveV2 reserve;
  reserve.valid = true;
  reserve.reserve_id = 901U;
  reserve.identity.execution_generation = 1U;
  reserve.identity.path_instance_id = 2U;
  reserve.identity.path_revision = 3U;
  reserve.identity.frame_revision = 4U;
  reserve.identity.frame_convention_id = 5U;
  reserve.identity.configuration_id = 6U;
  reserve.identity.map_instance_id = 7U;
  reserve.identity.map_state_id = 8U;
  reserve.identity.accepted_sequence = 9U;
  reserve.identity.profile_id = 10U;
  reserve.identity.binding_sequence = 11U;
  reserve.initial.w = 0.0;
  reserve.initial.delta = 0.1;
  reserve.initial.previous_u = phase_offset_core::PortCommand();
  reserve.dt = 0.1;
  TubeReserveStepV2 brake;
  brake.ordinal = 0U;
  brake.segment = TubeReserveSegmentKindV2::BRAKE;
  brake.before = reserve.initial;
  brake.command.u_delta = -1.0;
  brake.base_phase_rate = 0.1;
  brake.phase_rate_lower = 0.1;
  brake.phase_rate_upper = 0.2;
  brake.after = brake.before;
  brake.after.w = brake.before.w +
      reserve.dt * (brake.base_phase_rate + brake.command.u_w);
  brake.after.delta = brake.before.delta +
      reserve.dt * brake.command.u_delta;
  brake.after.previous_u = brake.command;
  brake.valid = true;
  TubeReserveStepV2 settle;
  settle.ordinal = 1U;
  settle.segment = TubeReserveSegmentKindV2::SETTLE;
  settle.before = brake.after;
  settle.command = phase_offset_core::PortCommand();
  settle.base_phase_rate = 0.1;
  settle.phase_rate_lower = 0.1;
  settle.phase_rate_upper = 0.2;
  settle.after = settle.before;
  settle.after.w = settle.before.w +
      reserve.dt * (settle.base_phase_rate + settle.command.u_w);
  settle.after.delta = settle.before.delta +
      reserve.dt * settle.command.u_delta;
  settle.after.previous_u = settle.command;
  settle.valid = true;
  reserve.steps = {brake, settle};
  reserve.terminal = settle.after;
  reserve.w_max = settle.after.w;
  reserve.common_lower = -1.0;
  reserve.common_upper = 1.0;
  reserve.cursor = 0U;
  reserve.work_count = 2U;
  reserve.provenance = "T23/finite-reserve-fixture";
  return reserve;
}

TEST(RecoveryOwnerTest, TypedFiniteReserveAdvancesOnlyAfterCommit) {
  PhaseOffsetRecoveryOwner owner;
  const TubeFiniteReserveV2 reserve = FiniteReserveFixture();
  ASSERT_TRUE(TubeExecutionGuardV2::validateReserve(reserve));
  CertifiedReservePrepareInputV2 input;
  input.recovery_session = 17U;
  input.reserve = reserve;
  input.expected_identity = reserve.identity;
  input.cursor = 0U;
  input.expected_state = reserve.initial;
  input.dt = reserve.dt;
  input.now = 0.0;
  input.deadline = 1.0;
  input.deadline_valid = true;
  input.provenance = "T23/typed-reserve";
  RecoveryPreparedStep prepared;
  ASSERT_TRUE(owner.prepareCertifiedReserveV2(input, prepared));
  EXPECT_EQ(owner.status().reserve_cursor, 0U);
  EXPECT_EQ(prepared.proof_kind, RecoveryStepProofKind::FINITE_RESERVE_V2);
  EXPECT_EQ(prepared.reserve_cursor, 0U);
  ASSERT_TRUE(owner.commit(prepared));
  EXPECT_EQ(owner.status().reserve_id, reserve.reserve_id);
  EXPECT_EQ(owner.status().reserve_cursor, 1U);
  EXPECT_DOUBLE_EQ(owner.status().current_delta, 0.0);
  EXPECT_TRUE(owner.status().nonzero_authority_retained);

  // The cursor advances only after the first publication/commit.  Prepare
  // the actual SETTLE step against a reserve copy carrying cursor=1, then
  // verify that the exact neutral terminality is published with cursor=2.
  CertifiedReservePrepareInputV2 settle_input = input;
  settle_input.reserve.cursor = 1U;
  settle_input.cursor = 1U;
  settle_input.expected_state = reserve.steps[1].before;
  RecoveryPreparedStep settle_prepared;
  ASSERT_TRUE(owner.prepareCertifiedReserveV2(settle_input, settle_prepared));
  EXPECT_EQ(owner.status().reserve_cursor, 1U);
  EXPECT_EQ(settle_prepared.reserve_segment, TubeReserveSegmentKindV2::SETTLE);
  EXPECT_TRUE(settle_prepared.exact_terminal_predicate);
  ASSERT_TRUE(owner.commit(settle_prepared));
  EXPECT_EQ(owner.status().reserve_cursor, 2U);
  EXPECT_TRUE(owner.status().exact_terminal_predicate);
  EXPECT_FALSE(owner.status().nonzero_authority_retained);

  CertifiedReservePrepareInputV2 stale_session = settle_input;
  stale_session.recovery_session += 1U;
  RecoveryPreparedStep stale_output;
  EXPECT_FALSE(owner.prepareCertifiedReserveV2(stale_session, stale_output));
  EXPECT_EQ(stale_output.status, RecoveryStepStatus::STALE);
  EXPECT_EQ(owner.status().reserve_cursor, 2U);

  RecoveryPreparedStep replay;
  EXPECT_FALSE(owner.prepareCertifiedReserveV2(input, replay));
  EXPECT_EQ(replay.status, RecoveryStepStatus::STALE);
  EXPECT_EQ(owner.status().reserve_cursor, 2U);
}

TEST(RecoveryOwnerTest, TypedFiniteReserveRejectsSubstitutionAndExpiredPublish) {
  PhaseOffsetRecoveryOwner owner;
  const TubeFiniteReserveV2 reserve = FiniteReserveFixture();
  CertifiedReservePrepareInputV2 input;
  input.recovery_session = 18U;
  input.reserve = reserve;
  input.expected_identity = reserve.identity;
  input.cursor = 0U;
  input.expected_state = reserve.initial;
  input.dt = reserve.dt;
  input.now = 0.0;
  input.deadline = 1.0;
  input.deadline_valid = true;
  input.provenance = "T23/failed-publish";
  RecoveryPreparedStep prepared;
  ASSERT_TRUE(owner.prepareCertifiedReserveV2(input, prepared));
  const RecoveryOwnerStatus before = owner.status();
  prepared.selected_u.u_delta = -0.5;
  EXPECT_FALSE(owner.validateCommit(prepared));
  EXPECT_EQ(owner.status().reserve_cursor, before.reserve_cursor);

  // A caller cannot substitute a different expected execution state for the
  // selected reserve cursor.  Preparation must reject it without advancing
  // or otherwise mutating the owner status.
  CertifiedReservePrepareInputV2 mismatched_state = input;
  mismatched_state.expected_state.delta = 0.2;
  RecoveryPreparedStep mismatch_output;
  EXPECT_FALSE(owner.prepareCertifiedReserveV2(mismatched_state,
                                               mismatch_output));
  EXPECT_EQ(mismatch_output.status, RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED);
  EXPECT_EQ(owner.status().reserve_cursor, before.reserve_cursor);

  CertifiedReservePrepareInputV2 mismatched_identity = input;
  mismatched_identity.expected_identity.path_revision += 1U;
  RecoveryPreparedStep identity_output;
  EXPECT_FALSE(owner.prepareCertifiedReserveV2(mismatched_identity,
                                               identity_output));
  EXPECT_EQ(identity_output.status, RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED);
  EXPECT_EQ(owner.status().reserve_cursor, before.reserve_cursor);

  CertifiedReservePrepareInputV2 mismatched_dt = input;
  mismatched_dt.dt = reserve.dt + 0.01;
  RecoveryPreparedStep dt_output;
  EXPECT_FALSE(owner.prepareCertifiedReserveV2(mismatched_dt, dt_output));
  EXPECT_EQ(dt_output.status, RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED);
  EXPECT_EQ(owner.status().reserve_cursor, before.reserve_cursor);

  // The typed validator re-derives each after-state recurrence; a forged
  // reserve endpoint is unavailable even when the cursor/identity match.
  CertifiedReservePrepareInputV2 forged = input;
  forged.reserve.steps[0].after.w += 0.001;
  RecoveryPreparedStep forged_output;
  EXPECT_FALSE(owner.prepareCertifiedReserveV2(forged, forged_output));
  EXPECT_EQ(forged_output.status, RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED);
  EXPECT_EQ(owner.status().reserve_cursor, before.reserve_cursor);

  prepared = RecoveryPreparedStep();
  input.now = input.deadline;
  EXPECT_FALSE(owner.prepareCertifiedReserveV2(input, prepared));
  EXPECT_EQ(prepared.status, RecoveryStepStatus::DEADLINE_EXPIRED);
  EXPECT_EQ(owner.status().reserve_cursor, before.reserve_cursor);
}

}  // namespace
}  // namespace phase_offset_navigation

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
