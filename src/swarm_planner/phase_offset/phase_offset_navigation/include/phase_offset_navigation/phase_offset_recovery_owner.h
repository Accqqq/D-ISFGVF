#pragma once

#include "phase_offset_navigation/recovery_prepared_step.h"

#include <phase_offset_core/port_projector.h>

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace phase_offset_navigation {

struct PhaseOffsetRecoveryOwnerConfig {
  double recovery_progress_tolerance = 1e-9;
  double recovery_forward_progress_tolerance = 1e-9;
  double recovery_slew_cost_tolerance = 1e-9;
  double recovery_command_comparison_epsilon = 1e-12;
  double w_w = 1.0;
  double w_delta = 1.0;
  double v_rec_min = 1e-3;
  double max_dt = 0.25;
  double max_u_w = 10.0;
  double max_u_delta = 10.0;
  bool require_finite_deadline = true;
};

struct RecoveryCandidate {
  phase_offset_core::PortCommand command;
  double next_w = 0.0;
  double next_delta = 0.0;
  double progress = 0.0;
  double next_measure_upper_bound = 0.0;
  double forward_progress = 0.0;
  double slew_cost = 0.0;
  double admissible_u_w_lower = 0.0;
  double admissible_u_w_upper = 0.0;
  double admissible_u_delta_lower = 0.0;
  double admissible_u_delta_upper = 0.0;
  // Optional certified measure evidence.  When progress_bound_valid is true,
  // the owner recomputes P_k = measure - next_measure_upper_bound and records
  // that provenance instead of trusting a copied progress scalar.
  double measure = 0.0;
  bool progress_bound_valid = false;
  std::string progress_provenance;
  bool command_membership_valid = true;
  bool closed_bounded_set_valid = true;
  bool finite_domain_valid = true;
  double phase_domain_start = 0.0;
  double phase_domain_end = 0.0;
  // Minimum certified time horizon that the remaining phase domain must
  // support for a multi-tick recovery transaction.  Zero is retained for
  // legacy value-only fixtures; production supplies a positive bound.
  double required_domain_duration = 0.0;
  double s_dot = 0.0;
  double v_s_min = 0.0;
  double minimum_time_progress = 0.0;
  std::uint64_t s_dot_revision = 0U;
  bool actual_s_dot_valid = false;
  bool require_time_progress = false;
  bool time_progress_valid = true;
  std::string s_dot_provenance;
  RecoveryReferenceJet reference_jet;
  bool admissible = true;
  bool proof_valid = true;
  bool terminal_predicate = false;
  bool exact_terminal_predicate = false;
  std::string provenance;
};

struct RecoveryPrepareInput {
  std::uint64_t recovery_session = 0U;
  std::uint64_t source_path_revision = 0U;
  std::uint64_t target_path_revision = 0U;
  std::uint64_t source_frame_revision = 0U;
  std::uint64_t target_frame_revision = 0U;
  std::uint64_t source_owner_revision = 0U;
  std::uint64_t target_owner_revision = 0U;
  RecoveryReferenceJet reference_jet;
  phase_offset_core::PortCommand u_prev;
  double current_w = 0.0;
  double current_delta = 0.0;
  // Production supplies the immutable base phase rate so every ranking
  // metric and next-state recurrence can be recomputed from the same tick
  // inputs.  Legacy unit fixtures may leave this opt-in flag false.
  double base_w_dot = 0.0;
  bool base_w_dot_valid = false;
  double dt = 0.0;
  double measure = 0.0;
  double now = 0.0;
  double deadline = 0.0;
  // The producer must prove these bounded-domain assumptions before a
  // command can become RECOVERY authority.  Defaults preserve legacy unit
  // fixtures; production callers set them explicitly.
  double phase_domain_start = 0.0;
  double phase_domain_end = 0.0;
  // Immutable target interval used to derive the recovery measure.  When
  // valid, prepare() recomputes both dist(current_delta, I_target) and
  // dist(next_delta, I_target) from the input/command recurrence; caller
  // supplied measure/progress scalars then remain diagnostics only.
  double target_delta_lower = 0.0;
  double target_delta_upper = 0.0;
  bool target_interval_valid = false;
  // Minimum certified time horizon that the remaining phase domain must
  // support for a multi-tick recovery transaction.  Production callers set
  // this from the immutable domain/velocity certificate; value-only unit
  // fixtures may leave it at zero.
  double required_domain_duration = 0.0;
  double phase_rate_upper_bound = 0.0;
  bool phase_rate_upper_bound_valid = false;
  double certified_recovery_rate = 0.0;
  bool certified_recovery_rate_valid = false;
  double max_slew_rate = 0.0;
  double v_s_min = 0.0;
  bool admissible_set_closed_bounded = true;
  bool finite_domain_valid = true;
  bool time_progress_valid = true;
  bool require_time_progress = false;
  bool current_state_safe = true;
  bool source_reference_matches = true;
  bool stale = false;
  std::vector<RecoveryCandidate> candidates;
  std::string provenance;
};

struct RecoveryOwnerStatus {
  std::uint64_t recovery_session = 0U;
  std::uint64_t committed_sequence = 0U;
  RecoveryStepStatus last_status = RecoveryStepStatus::NONE;
  phase_offset_core::PortCommand selected_u;
  double current_w = 0.0;
  double current_delta = 0.0;
  bool active = false;
  bool deadline_expired = false;
  bool nonzero_authority_retained = false;
  // True only when the last committed recovery step's independently
  // recomputed next-delta predicate proved terminal.  Handoff code must not
  // infer terminality from a tolerance check on a forged snapshot alone.
  bool terminal_predicate = false;
  bool exact_terminal_predicate = false;
  RecoveryStepProofKind proof_kind = RecoveryStepProofKind::LEGACY;
  std::uint64_t reserve_id = 0U;
  std::size_t reserve_cursor = 0U;
  std::size_t reserve_size = 0U;
  TubeReserveSegmentKindV2 reserve_segment = TubeReserveSegmentKindV2::BRAKE;
  std::string reason;
};

// A typed V2 cursor transaction.  The reserve is copied into this value before
// publication; owner state changes only when the prepared step is committed
// after the command publication succeeds.
struct CertifiedReservePrepareInputV2 {
  std::uint64_t recovery_session = 0U;
  TubeFiniteReserveV2 reserve;
  // The caller's immutable execution binding must match every field captured
  // by the reserve.  A reserve ID alone is not sufficient identity evidence.
  TubeExecutionIdentityV2 expected_identity;
  std::size_t cursor = 0U;
  TubeExecutionStateV2 expected_state;
  double dt = 0.0;
  double now = 0.0;
  double deadline = std::numeric_limits<double>::quiet_NaN();
  bool deadline_valid = false;
  std::string provenance;
};

using RecoveryReservePrepareInputV2 = CertifiedReservePrepareInputV2;

class PhaseOffsetRecoveryOwner {
 public:
  explicit PhaseOffsetRecoveryOwner(
      const PhaseOffsetRecoveryOwnerConfig& config =
          PhaseOffsetRecoveryOwnerConfig());

  bool configurationValid() const { return configuration_valid_; }
  const PhaseOffsetRecoveryOwnerConfig& config() const { return config_; }
  const RecoveryOwnerStatus& status() const { return *status_; }

  // Pure preparation: no selected command or owner state is committed.
  bool prepare(const RecoveryPrepareInput& input,
               RecoveryPreparedStep& output) const;

  // V2 preparation consumes one immutable finite reserve step.  It does not
  // apply the legacy distance-progress criterion and does not mutate owner
  // state; commit() or commitNoFail() advances the typed cursor later.
  bool prepareCertifiedReserveV2(
      const CertifiedReservePrepareInputV2& input,
      RecoveryPreparedStep& output) const;

  // Commit accepts exactly the selected-u contained in the prepared step.  It
  // never projects, ranks, clamps, or substitutes the command.
  bool commit(const RecoveryPreparedStep& prepared);
  // Side-effect-free validation for the publication seam.  The caller must
  // run this before publishing the PositionCommand.
  bool validateCommit(const RecoveryPreparedStep& prepared) const;
  // No-fail post-publication state write.  All validation and allocation must
  // have completed through validateCommit() before this is called.
  void commitNoFail(const RecoveryPreparedStep& prepared) noexcept;

  void reset(std::uint64_t recovery_session = 0U);

  static bool selectDeterministic(
      const std::vector<RecoveryCandidate>& candidates,
      const PhaseOffsetRecoveryOwnerConfig& config,
      RecoveryCandidate& selected);

  // A closed bounded admissible-set audit used by tests and by a feasibility
  // kernel caller.  The owner does not use it to rank commands.
  static bool verifyAdmissibleSet(
      const phase_offset_core::PortProjector::AdmissibleSet& set);

  static double computeDeadline(double now, double initial_measure,
                                double v_rec_min, double finite_margin,
                                double worst_case_ramp_time);

  // Strict bounded-domain deadline helper.  It returns NaN unless every
  // assumption used by the finite bound is explicit and validated.
  static double computeBoundedDeadline(
      double now, double initial_measure, double v_rec_min,
      double finite_margin, double worst_case_ramp_time, double max_dt,
      double max_u_delta, double max_slew_rate, double phase_domain_start,
      double phase_domain_end);

 private:
  PhaseOffsetRecoveryOwnerConfig config_;
  bool configuration_valid_ = false;
  std::shared_ptr<const RecoveryOwnerStatus> status_;
};

using RecoveryOwner = PhaseOffsetRecoveryOwner;

}  // namespace phase_offset_navigation
