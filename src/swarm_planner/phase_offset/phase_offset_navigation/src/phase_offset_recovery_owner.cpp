#include "phase_offset_navigation/phase_offset_recovery_owner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace phase_offset_navigation {
namespace {

bool Finite(const double value) { return std::isfinite(value); }

bool Finite(const phase_offset_core::PortCommand& command) {
  return Finite(command.u_w) && Finite(command.u_delta);
}

double DistanceToClosedInterval(const double value, const double lower,
                                const double upper) {
  if (!Finite(value) || !Finite(lower) || !Finite(upper) || lower > upper) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (value < lower) return lower - value;
  if (value > upper) return value - upper;
  return 0.0;
}

bool NonzeroRevision(const std::uint64_t revision) { return revision != 0U; }

bool ValidReferenceJet(const RecoveryReferenceJet& jet) {
  return jet.valid && Finite(jet.w) && Finite(jet.delta) &&
      NonzeroRevision(jet.path_revision) &&
      NonzeroRevision(jet.frame_revision) &&
      NonzeroRevision(jet.owner_revision) &&
      NonzeroRevision(jet.query_revision) && jet.r.allFinite() &&
      jet.r_w.allFinite() && (!jet.r_ww_valid || jet.r_ww.allFinite()) &&
      !jet.provenance.empty();
}

bool ValidRevisionContract(const RecoveryPrepareInput& input) {
  const bool source_reference_match =
      input.reference_jet.path_revision == input.source_path_revision &&
      input.reference_jet.frame_revision == input.source_frame_revision &&
      input.reference_jet.owner_revision == input.source_owner_revision;
  const bool target_reference_match =
      input.reference_jet.path_revision == input.target_path_revision &&
      input.reference_jet.frame_revision == input.target_frame_revision &&
      input.reference_jet.owner_revision == input.target_owner_revision;
  return NonzeroRevision(input.source_path_revision) &&
      NonzeroRevision(input.target_path_revision) &&
      NonzeroRevision(input.source_frame_revision) &&
      NonzeroRevision(input.target_frame_revision) &&
      NonzeroRevision(input.source_owner_revision) &&
      NonzeroRevision(input.target_owner_revision) &&
      ValidReferenceJet(input.reference_jet) &&
      (source_reference_match || target_reference_match);
}

bool BetterFinalTieBreak(const RecoveryCandidate& first,
                         const RecoveryCandidate& second,
                         const PhaseOffsetRecoveryOwnerConfig& config) {
  // Tolerances have already grouped the first three priority metrics.  The
  // final numerical ordering is intentionally exact so two distinct finite
  // commands cannot inherit container order merely because they are close to
  // the comparison epsilon.
  (void)config;
  if (first.command.u_w != second.command.u_w) {
    return first.command.u_w < second.command.u_w;
  }
  return first.command.u_delta < second.command.u_delta;
}

bool InClosedBoundedSet(const RecoveryCandidate& candidate,
                        const PhaseOffsetRecoveryOwnerConfig& config) {
  if (!candidate.command_membership_valid ||
      !candidate.closed_bounded_set_valid || !candidate.finite_domain_valid ||
      !candidate.time_progress_valid || !ValidReferenceJet(candidate.reference_jet) ||
      !Finite(candidate.command) || !Finite(candidate.next_w) ||
      !Finite(candidate.next_delta) || !Finite(candidate.s_dot) ||
      !Finite(candidate.v_s_min) || candidate.v_s_min < 0.0 ||
      candidate.s_dot < candidate.v_s_min -
          config.recovery_command_comparison_epsilon ||
      !Finite(candidate.phase_domain_start) ||
      !Finite(candidate.phase_domain_end) ||
      candidate.phase_domain_end < candidate.phase_domain_start ||
      candidate.next_w < candidate.phase_domain_start -
          config.recovery_command_comparison_epsilon ||
      candidate.next_w > candidate.phase_domain_end +
          config.recovery_command_comparison_epsilon ||
      !Finite(candidate.measure) || candidate.measure < 0.0 ||
      !candidate.progress_bound_valid ||
      !Finite(candidate.next_measure_upper_bound) ||
      candidate.next_measure_upper_bound < 0.0 ||
      candidate.provenance.empty() || candidate.progress_provenance.empty()) {
    return false;
  }
  if (candidate.require_time_progress &&
      (!candidate.actual_s_dot_valid || candidate.s_dot_revision == 0U ||
       candidate.s_dot_provenance.empty() ||
       !Finite(candidate.minimum_time_progress) ||
       candidate.minimum_time_progress < 0.0 ||
       (candidate.minimum_time_progress > 0.0 && candidate.s_dot <= 0.0))) {
    return false;
  }
  const bool explicit_bounds =
      Finite(candidate.admissible_u_w_lower) &&
      Finite(candidate.admissible_u_w_upper) &&
      Finite(candidate.admissible_u_delta_lower) &&
      Finite(candidate.admissible_u_delta_upper) &&
      candidate.admissible_u_w_lower <= candidate.admissible_u_w_upper &&
      candidate.admissible_u_delta_lower <= candidate.admissible_u_delta_upper;
  const double eps = config.recovery_command_comparison_epsilon;
  return explicit_bounds &&
      candidate.command.u_w >= candidate.admissible_u_w_lower - eps &&
      candidate.command.u_w <= candidate.admissible_u_w_upper + eps &&
      candidate.command.u_delta >= candidate.admissible_u_delta_lower - eps &&
      candidate.command.u_delta <= candidate.admissible_u_delta_upper + eps &&
      std::abs(candidate.command.u_w) <= config.max_u_w + eps &&
      std::abs(candidate.command.u_delta) <= config.max_u_delta + eps;
}

template <typename ValueFn>
bool GlobalToleranceFilter(std::vector<RecoveryCandidate>& candidates,
                           ValueFn value, const bool maximize,
                           const double tolerance) {
  if (candidates.empty()) return false;
  double optimum = maximize ? -std::numeric_limits<double>::infinity()
                            : std::numeric_limits<double>::infinity();
  for (const RecoveryCandidate& candidate : candidates) {
    const double v = value(candidate);
    if (!Finite(v)) continue;
    optimum = maximize ? std::max(optimum, v) : std::min(optimum, v);
  }
  if (!Finite(optimum)) {
    candidates.clear();
    return false;
  }
  std::size_t write = 0U;
  const std::size_t original_size = candidates.size();
  for (std::size_t index = 0U; index < original_size; ++index) {
    const RecoveryCandidate& candidate = candidates[index];
    const double v = value(candidate);
    if (!Finite(v)) continue;
    const bool equivalent = maximize ? v >= optimum - tolerance
                                     : v <= optimum + tolerance;
    if (equivalent) {
      if (write != index) candidates[write] = candidates[index];
      ++write;
    }
  }
  candidates.resize(write);
  return !candidates.empty();
}

bool ValidConfig(const PhaseOffsetRecoveryOwnerConfig& config) {
  return Finite(config.recovery_progress_tolerance) &&
      config.recovery_progress_tolerance >= 0.0 &&
      Finite(config.recovery_forward_progress_tolerance) &&
      config.recovery_forward_progress_tolerance >= 0.0 &&
      Finite(config.recovery_slew_cost_tolerance) &&
      config.recovery_slew_cost_tolerance >= 0.0 &&
      Finite(config.recovery_command_comparison_epsilon) &&
      config.recovery_command_comparison_epsilon >= 0.0 && Finite(config.w_w) &&
      config.w_w >= 0.0 && Finite(config.w_delta) && config.w_delta >= 0.0 &&
      Finite(config.v_rec_min) && config.v_rec_min > 0.0 &&
      Finite(config.max_dt) && config.max_dt > 0.0 && Finite(config.max_u_w) &&
      config.max_u_w >= 0.0 && Finite(config.max_u_delta) &&
      config.max_u_delta >= 0.0 &&
      (!config.require_finite_deadline || config.max_dt > 0.0);
}

bool SameExecutionState(const TubeExecutionStateV2& first,
                        const TubeExecutionStateV2& second) {
  return first.finite() && second.finite() && first.w == second.w &&
      first.delta == second.delta &&
      first.previous_u.u_w == second.previous_u.u_w &&
      first.previous_u.u_delta == second.previous_u.u_delta;
}

bool SameExecutionIdentity(const TubeExecutionIdentityV2& first,
                           const TubeExecutionIdentityV2& second) {
  return first.complete() && second.complete() &&
      first.execution_generation == second.execution_generation &&
      first.path_instance_id == second.path_instance_id &&
      first.path_revision == second.path_revision &&
      first.frame_revision == second.frame_revision &&
      first.frame_convention_id == second.frame_convention_id &&
      first.configuration_id == second.configuration_id &&
      first.map_instance_id == second.map_instance_id &&
      first.map_state_id == second.map_state_id &&
      first.accepted_sequence == second.accepted_sequence &&
      first.profile_id == second.profile_id &&
      first.binding_sequence == second.binding_sequence;
}

}  // namespace

PhaseOffsetRecoveryOwner::PhaseOffsetRecoveryOwner(
    const PhaseOffsetRecoveryOwnerConfig& config)
    : config_(config), configuration_valid_(ValidConfig(config)),
      status_(std::make_shared<RecoveryOwnerStatus>()) {}

bool PhaseOffsetRecoveryOwner::selectDeterministic(
    const std::vector<RecoveryCandidate>& candidates,
    const PhaseOffsetRecoveryOwnerConfig& config,
    RecoveryCandidate& selected) {
  std::vector<RecoveryCandidate> admissible;
  admissible.reserve(candidates.size());
  for (const RecoveryCandidate& candidate : candidates) {
    if (!candidate.admissible || !candidate.proof_valid ||
        !InClosedBoundedSet(candidate, config)) {
      continue;
    }
    RecoveryCandidate normalized = candidate;
    // P_k is a derived fact.  A producer may provide the measure and the
    // certified upper bound; when it does, recompute it here instead of
    // trusting a stale scalar copied from another candidate.
    if (candidate.progress_bound_valid) {
      normalized.progress = candidate.measure -
          candidate.next_measure_upper_bound;
      normalized.progress_provenance = "RecoveryOwner/P_k=recomputed";
    }
    if (!Finite(normalized.progress) || !Finite(normalized.forward_progress) ||
        !Finite(normalized.slew_cost)) {
      continue;
    }
    if (!Finite(normalized.progress) || normalized.progress < 0.0) continue;
    admissible.push_back(normalized);
  }
  if (admissible.empty()) return false;

  // Each priority level is a global optimum followed by a tolerance
  // equivalence set.  This deliberately avoids pairwise folding, whose
  // non-transitive tolerance relation made the result depend on container
  // order (a,b,c with |a-b|<=tol and |b-c|<=tol but |a-c|>tol).
  if (!GlobalToleranceFilter(
          admissible, [](const RecoveryCandidate& c) { return c.progress; },
          true, config.recovery_progress_tolerance) ||
      !GlobalToleranceFilter(
          admissible,
          [](const RecoveryCandidate& c) { return c.forward_progress; }, true,
          config.recovery_forward_progress_tolerance) ||
      !GlobalToleranceFilter(
          admissible, [](const RecoveryCandidate& c) { return c.slew_cost; },
          false, config.recovery_slew_cost_tolerance)) {
    return false;
  }

  RecoveryCandidate best = admissible.front();
  for (std::size_t index = 1U; index < admissible.size(); ++index) {
    if (BetterFinalTieBreak(admissible[index], best, config)) {
      best = admissible[index];
    }
  }
  selected = best;
  return true;
}

bool PhaseOffsetRecoveryOwner::verifyAdmissibleSet(
    const phase_offset_core::PortProjector::AdmissibleSet& set) {
  const bool implicit_rectangle = set.vertices.empty() && set.edges.empty();
  const bool has_vertices = !set.vertices.empty();
  return set.valid && set.bounded && set.u_w.valid && set.u_delta.valid &&
      set.closed && (implicit_rectangle || has_vertices) &&
      Finite(set.u_w.lower) && Finite(set.u_w.upper) &&
      Finite(set.u_delta.lower) && Finite(set.u_delta.upper) &&
      set.u_w.lower <= set.u_w.upper && set.u_delta.lower <= set.u_delta.upper &&
      (set.vertices.empty() ||
       std::all_of(set.vertices.begin(), set.vertices.end(),
                   [&set](const phase_offset_core::PortCommand& command) {
                     return set.contains(command);
                   }));
}

double PhaseOffsetRecoveryOwner::computeDeadline(
    const double now, const double initial_measure, const double v_rec_min,
    const double finite_margin, const double worst_case_ramp_time) {
  if (!Finite(now) || !Finite(initial_measure) || initial_measure < 0.0 ||
      !Finite(v_rec_min) || v_rec_min <= 0.0 || !Finite(finite_margin) ||
      finite_margin < 0.0 || !Finite(worst_case_ramp_time) ||
      worst_case_ramp_time < 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  // The finite margin is an additive conservative allowance.  Subtracting
  // it from the distance would make the deadline shorter than the certified
  // initial-measure/v_rec_min bound and is therefore not a valid liveness
  // contract.
  return now + initial_measure / v_rec_min + finite_margin +
      worst_case_ramp_time;
}

double PhaseOffsetRecoveryOwner::computeBoundedDeadline(
    const double now, const double initial_measure, const double v_rec_min,
    const double finite_margin, const double worst_case_ramp_time,
    const double max_dt, const double max_u_delta,
    const double max_slew_rate, const double phase_domain_start,
    const double phase_domain_end) {
  if (!Finite(now) || !Finite(initial_measure) || initial_measure < 0.0 ||
      !Finite(v_rec_min) || v_rec_min <= 0.0 || !Finite(finite_margin) ||
      finite_margin < 0.0 || !Finite(worst_case_ramp_time) ||
      worst_case_ramp_time < 0.0 || !Finite(max_dt) || max_dt <= 0.0 ||
      !Finite(max_u_delta) || max_u_delta < 0.0 ||
      !Finite(max_slew_rate) || max_slew_rate < 0.0 ||
      !Finite(phase_domain_start) || !Finite(phase_domain_end) ||
      phase_domain_end <= phase_domain_start ||
      (max_u_delta > 0.0 && max_slew_rate <= 0.0)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  // The rate/slew/domain facts are deliberately checked even though the
  // scalar measure bound is conservative; omitting them would turn an
  // unproved deadline into apparent liveness evidence.
  const double bounded_ramp = std::max(worst_case_ramp_time,
      max_slew_rate > 0.0 ? max_u_delta / max_slew_rate : 0.0);
  // Domain span is a separate admissibility/liveness proof.  It must not be
  // added to the recovery deadline as if phase-domain distance were recovery
  // measure; doing so conflates s-dot with delta recovery rate and makes the
  // deadline slide in the wrong direction.
  const double deadline = now + initial_measure / v_rec_min +
      finite_margin + bounded_ramp;
  return Finite(deadline) ? deadline
                          : std::numeric_limits<double>::quiet_NaN();
}

bool PhaseOffsetRecoveryOwner::prepare(const RecoveryPrepareInput& input,
                                       RecoveryPreparedStep& output) const {
  output = RecoveryPreparedStep();
  output.recovery_session = input.recovery_session;
  output.source_path_revision = input.source_path_revision;
  output.target_path_revision = input.target_path_revision;
  output.source_frame_revision = input.source_frame_revision;
  output.target_frame_revision = input.target_frame_revision;
  output.source_owner_revision = input.source_owner_revision;
  output.target_owner_revision = input.target_owner_revision;
  output.reference_jet = input.reference_jet;
  output.u_prev = input.u_prev;
  output.current_w = input.current_w;
  output.current_delta = input.current_delta;
  output.dt = input.dt;
  output.measure = input.measure;
  output.deadline = input.deadline;
  output.now = input.now;
  output.remaining_domain_duration =
      (Finite(input.phase_domain_start) && Finite(input.phase_domain_end) &&
       input.phase_domain_end >= input.phase_domain_start &&
       Finite(input.phase_rate_upper_bound) &&
       input.phase_rate_upper_bound > 0.0)
          ? (input.phase_domain_end - input.phase_domain_start) /
                input.phase_rate_upper_bound
          : (Finite(input.v_s_min) && input.v_s_min > 0.0)
          ? (input.phase_domain_end - input.phase_domain_start) /
                input.v_s_min
          : 0.0;
  output.selected_u_owner = "PhaseOffsetRecoveryOwner";
  output.provenance = input.provenance.empty()
      ? "phase_offset_navigation/recovery-owner" : input.provenance;
  const bool revision_contract_valid = ValidRevisionContract(input);
  const bool target_interval_shape_valid = !input.target_interval_valid ||
      (Finite(input.target_delta_lower) &&
       Finite(input.target_delta_upper) &&
       input.target_delta_lower <= input.target_delta_upper);
  const double canonical_measure = input.target_interval_valid
      ? DistanceToClosedInterval(input.current_delta,
                                 input.target_delta_lower,
                                 input.target_delta_upper)
      : input.measure;
  if (!configuration_valid_ || input.recovery_session == 0U ||
      !Finite(input.current_w) || !Finite(input.current_delta) ||
      !Finite(input.u_prev) || !Finite(input.dt) || input.dt <= 0.0 ||
      input.dt > config_.max_dt || !Finite(input.measure) || input.measure < 0.0 ||
      !Finite(input.now) || !revision_contract_valid ||
      !target_interval_shape_valid || !Finite(canonical_measure) ||
      canonical_measure < 0.0 ||
      input.provenance.empty() || !input.source_reference_matches) {
    output.status = input.source_reference_matches && revision_contract_valid
        ? RecoveryStepStatus::INVALID : RecoveryStepStatus::REFERENCE_MISMATCH;
    output.proof = input.source_reference_matches && revision_contract_valid
        ? "recovery input or provenance is invalid"
        : "recovery source/target revision or reference provenance mismatches";
    return false;
  }
  if (input.target_interval_valid &&
      std::abs(input.measure - canonical_measure) >
          config_.recovery_command_comparison_epsilon) {
    output.status = RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED;
    output.proof = "recovery measure is not bound to immutable target interval";
    return false;
  }
  const bool phase_rate_bound_valid =
      !input.phase_rate_upper_bound_valid ||
      (Finite(input.phase_rate_upper_bound) &&
       input.phase_rate_upper_bound > 0.0);
  const bool recovery_rate_valid =
      !input.certified_recovery_rate_valid ||
      (Finite(input.certified_recovery_rate) &&
       input.certified_recovery_rate > 0.0);
  const bool horizon_contract_requested =
      input.phase_rate_upper_bound_valid ||
      input.required_domain_duration > 0.0 || input.require_time_progress;
  double phase_rate_for_horizon = input.phase_rate_upper_bound_valid
      ? input.phase_rate_upper_bound : input.v_s_min;
  if (!horizon_contract_requested &&
      (!Finite(phase_rate_for_horizon) || phase_rate_for_horizon <= 0.0)) {
    phase_rate_for_horizon = 1.0;
  }
  const double required_domain_span =
      input.required_domain_duration * std::max(phase_rate_for_horizon, 0.0);
  if (!input.admissible_set_closed_bounded || !input.finite_domain_valid ||
      !input.time_progress_valid || !phase_rate_bound_valid ||
      !recovery_rate_valid ||
      !Finite(input.phase_domain_start) || !Finite(input.phase_domain_end) ||
      input.phase_domain_end < input.phase_domain_start ||
      !Finite(input.max_slew_rate) || input.max_slew_rate < 0.0 ||
      !Finite(input.v_s_min) || input.v_s_min < 0.0 ||
      !Finite(input.required_domain_duration) ||
      input.required_domain_duration < 0.0 ||
      !Finite(phase_rate_for_horizon) || phase_rate_for_horizon <= 0.0 ||
      input.phase_domain_end - input.phase_domain_start <
          required_domain_span) {
    output.status = RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED;
    output.proof = "closed bounded recovery/domain/time contract is not proved";
    return false;
  }
  if (!input.current_state_safe) {
    output.status = RecoveryStepStatus::CURRENT_STATE_UNSAFE;
    output.proof = "current state is explicitly unsafe";
    return false;
  }
  if (input.stale) {
    output.status = RecoveryStepStatus::STALE;
    output.proof = "recovery evidence is stale";
    return false;
  }
  output.deadline_valid = Finite(input.deadline);
  if (config_.require_finite_deadline && !output.deadline_valid) {
    output.status = RecoveryStepStatus::INVALID;
    output.proof = "finite recovery deadline is not proved";
    return false;
  }
  if (output.deadline_valid && input.now >= input.deadline) {
    output.status = RecoveryStepStatus::DEADLINE_EXPIRED;
    output.proof = "recovery deadline expired";
    return false;
  }

  RecoveryCandidate selected;
  std::vector<RecoveryCandidate> ranking_candidates = input.candidates;
  if (input.base_w_dot_valid) {
    if (!Finite(input.base_w_dot)) {
      output.status = RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED;
      output.proof = "immutable base phase rate is not finite";
      return false;
    }
    for (RecoveryCandidate& candidate : ranking_candidates) {
      if (!Finite(candidate.command) || !Finite(candidate.measure) ||
          !Finite(candidate.next_measure_upper_bound) ||
          !Finite(candidate.reference_jet.w) ||
          !Finite(candidate.reference_jet.delta)) {
        candidate.admissible = false;
        continue;
      }
      candidate.forward_progress = input.base_w_dot + candidate.command.u_w;
      candidate.s_dot = candidate.forward_progress;
      candidate.next_w = input.current_w + input.dt * candidate.forward_progress;
      candidate.next_delta = input.current_delta +
          input.dt * candidate.command.u_delta;
      candidate.slew_cost = config_.w_w *
          std::pow(candidate.command.u_w - input.u_prev.u_w, 2.0) +
          config_.w_delta *
          std::pow(candidate.command.u_delta - input.u_prev.u_delta, 2.0);
      candidate.progress = candidate.measure -
          candidate.next_measure_upper_bound;
      candidate.progress_bound_valid = true;
      candidate.progress_provenance = "RecoveryOwner/P_k=recomputed";
    }
  }
  if (input.target_interval_valid) {
    if (!input.base_w_dot_valid || !Finite(input.base_w_dot)) {
      output.status = RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED;
      output.proof = "immutable target interval requires a base phase-rate witness";
      return false;
    }
    for (RecoveryCandidate& candidate : ranking_candidates) {
      if (!Finite(candidate.command)) {
        candidate.admissible = false;
        continue;
      }
      const double computed_next_delta = input.current_delta +
          input.dt * candidate.command.u_delta;
      // Numerical zero canonicalization is applied only after the selected
      // command's exact ZOH recurrence is recomputed.  It is not a selector
      // priority and does not snap a merely near-neutral current state.
      const bool exact_candidate_provenance =
          candidate.exact_terminal_predicate &&
          candidate.provenance ==
              "PortProjector/exact-selected-ZOH-arrival";
      const bool certified_exact_arrival = exact_candidate_provenance &&
          computed_next_delta == input.target_delta_lower;
      const double canonical_next_delta = certified_exact_arrival
          ? input.target_delta_lower : computed_next_delta;
      const double canonical_next_measure = DistanceToClosedInterval(
          canonical_next_delta, input.target_delta_lower,
          input.target_delta_upper);
      if (!Finite(canonical_next_delta) || !Finite(canonical_next_measure)) {
        candidate.admissible = false;
        continue;
      }
      candidate.measure = canonical_measure;
      candidate.next_delta = canonical_next_delta;
      candidate.next_measure_upper_bound = canonical_next_measure;
      candidate.progress = canonical_measure - canonical_next_measure;
      candidate.progress_bound_valid = true;
      candidate.progress_provenance =
          "RecoveryOwner/immutable-target-interval-distance";
      candidate.exact_terminal_predicate = certified_exact_arrival;
      candidate.terminal_predicate = certified_exact_arrival;
    }
  }

  // Guaranteed progress is an eligibility contract, not a post-selection
  // diagnostic.  Keep the selector's frozen priority order unchanged, but
  // remove every nonterminal candidate whose recomputed P_k is below the
  // one-tick lower bound before ranking.  The validated progress tolerance is
  // the Frozen epsilon_progress term; the command-comparison epsilon remains
  // reserved for exact ZOH recurrence/provenance checks.
  const double epsilon_progress = config_.recovery_progress_tolerance;
  const double required_progress = std::max(
      0.0, config_.v_rec_min * input.dt - epsilon_progress);
  for (RecoveryCandidate& candidate : ranking_candidates) {
    if (candidate.progress_bound_valid) {
      if (!Finite(candidate.measure) ||
          !Finite(candidate.next_measure_upper_bound)) {
        candidate.admissible = false;
        continue;
      }
      candidate.progress = candidate.measure -
          candidate.next_measure_upper_bound;
      candidate.progress_provenance = "RecoveryOwner/P_k=recomputed";
    }
    const bool certified_exact_arrival = input.target_interval_valid &&
        input.target_delta_lower == input.target_delta_upper &&
        candidate.exact_terminal_predicate &&
        candidate.provenance == "PortProjector/exact-selected-ZOH-arrival" &&
        Finite(candidate.command) &&
        Finite(input.current_delta + input.dt * candidate.command.u_delta) &&
        input.current_delta + input.dt * candidate.command.u_delta ==
            input.target_delta_lower;
    if (!certified_exact_arrival &&
        (!Finite(candidate.progress) ||
         candidate.progress < required_progress)) {
      candidate.admissible = false;
    }
  }
  if (!selectDeterministic(ranking_candidates, config_, selected)) {
    output.status = RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED;
    output.proof = "no admissible command satisfies deterministic recovery progress";
    return false;
  }
  if (!selected.closed_bounded_set_valid || !selected.command_membership_valid ||
      !selected.finite_domain_valid || !selected.time_progress_valid) {
    output.status = RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED;
    output.proof = "selected command membership or finite-domain proof is absent";
    return false;
  }
  if (input.require_time_progress &&
      (!selected.require_time_progress || !selected.actual_s_dot_valid ||
       selected.s_dot_revision != input.target_owner_revision ||
       selected.s_dot_provenance != input.provenance ||
       !Finite(selected.minimum_time_progress) ||
       selected.minimum_time_progress < 0.0 ||
       selected.s_dot * input.dt < selected.minimum_time_progress)) {
    output.status = RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED;
    output.proof = "revision-bound recovery time-progress witness is absent";
    return false;
  }
  const double evidence_eps = config_.recovery_command_comparison_epsilon;
  if (std::abs(selected.measure - input.measure) > evidence_eps ||
      std::abs(selected.reference_jet.w - input.current_w) > evidence_eps ||
      std::abs(selected.reference_jet.delta - input.current_delta) >
          evidence_eps ||
      std::abs(selected.phase_domain_start - input.phase_domain_start) >
          evidence_eps ||
      std::abs(selected.phase_domain_end - input.phase_domain_end) >
          evidence_eps || selected.v_s_min < input.v_s_min - evidence_eps ||
      selected.s_dot < input.v_s_min - evidence_eps) {
    output.status = RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED;
    output.proof = "selected recovery evidence is not bound to the input state/domain";
    return false;
  }
  const double expected_next_w = input.current_w + input.dt * selected.s_dot;
  const double expected_next_delta =
      input.current_delta + input.dt * selected.command.u_delta;
  if (!Finite(expected_next_w) || !Finite(expected_next_delta) ||
      std::abs(selected.next_w - expected_next_w) >
          config_.recovery_command_comparison_epsilon ||
      std::abs(selected.next_delta - expected_next_delta) >
          config_.recovery_command_comparison_epsilon) {
    output.status = RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED;
    output.proof = "selected next state is not the exact ZOH recurrence";
    return false;
  }
  if (!ValidReferenceJet(selected.reference_jet) ||
      !selected.reference_jet.matches(input.reference_jet)) {
    output.status = RecoveryStepStatus::REFERENCE_MISMATCH;
    output.proof = "selected recovery candidate reference jet mismatches source";
    return false;
  }
  const bool exact_selected_arrival = input.target_interval_valid &&
      input.target_delta_lower == input.target_delta_upper &&
      selected.exact_terminal_predicate &&
      selected.provenance == "PortProjector/exact-selected-ZOH-arrival" &&
      Finite(expected_next_delta) &&
      expected_next_delta == input.target_delta_lower &&
      selected.next_delta == input.target_delta_lower;
  if (selected.progress < required_progress && !exact_selected_arrival) {
    output.status = RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED;
    output.proof = "guaranteed recovery progress lower bound is not met";
    return false;
  }
  if (std::abs(selected.command.u_w) > config_.max_u_w +
      config_.recovery_command_comparison_epsilon ||
      std::abs(selected.command.u_delta) > config_.max_u_delta +
      config_.recovery_command_comparison_epsilon) {
    output.status = RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED;
    output.proof = "selected command exceeds bounded recovery envelope";
    return false;
  }
  output.selected_u = selected.command;
  output.selected_u_w = selected.command.u_w;
  output.selected_u_delta = selected.command.u_delta;
  output.u_w_lower = selected.admissible_u_w_lower;
  output.u_w_upper = selected.admissible_u_w_upper;
  output.u_delta_lower = selected.admissible_u_delta_lower;
  output.u_delta_upper = selected.admissible_u_delta_upper;
  output.next_w = Finite(selected.next_w)
      ? selected.next_w : input.current_w + input.dt * selected.command.u_w;
  output.next_delta = Finite(selected.next_delta)
      ? selected.next_delta
      : input.current_delta + input.dt * selected.command.u_delta;
  if (selected.progress_bound_valid && Finite(selected.measure) &&
      Finite(selected.next_measure_upper_bound)) {
    output.measure = selected.measure;
  }
  output.next_measure_upper_bound = selected.next_measure_upper_bound;
  output.progress = selected.progress;
  output.required_progress = required_progress;
  // Terminality is a derived fact of the exact selected next state.  Ignore
  // any copied candidate boolean so a forged metric cannot trigger neutral
  // handoff.
  // Tolerance-based proximity is useful for ranking diagnostics, but it is
  // never a terminal proof.  Only an exact selected-ZOH recurrence to the
  // immutable singleton target interval can authorize neutral handoff.
  const double recomputed_selected_next_delta = input.current_delta +
      input.dt * selected.command.u_delta;
  const bool selected_exact_candidate = selected.exact_terminal_predicate &&
      selected.provenance == "PortProjector/exact-selected-ZOH-arrival" &&
      Finite(recomputed_selected_next_delta) &&
      recomputed_selected_next_delta == input.target_delta_lower;
  output.exact_terminal_predicate = input.target_interval_valid &&
      input.target_delta_lower == input.target_delta_upper &&
      selected_exact_candidate && output.next_delta == input.target_delta_lower;
  output.terminal_predicate = output.exact_terminal_predicate;
  output.proof_valid = selected.proof_valid;
  output.status = RecoveryStepStatus::PREPARED;
  output.valid = output.finite() && output.selectedUConsistent() &&
      output.proof_valid &&
      (output.exact_terminal_predicate ||
       output.progress >= output.required_progress);
  if (!output.valid) {
    output.status = RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED;
    output.proof = "prepared recovery step is not finite or progress-certified";
    return false;
  }
  output.proof = selected.progress_provenance.empty()
      ? (selected.provenance.empty() ? "deterministic recovery progress"
                                     : selected.provenance)
      : selected.progress_provenance;
  try {
    auto committed = std::make_shared<RecoveryOwnerStatus>();
    committed->recovery_session = output.recovery_session;
    committed->committed_sequence = status_->committed_sequence + 1U;
    committed->last_status = RecoveryStepStatus::PREPARED;
    committed->selected_u = output.selected_u;
    committed->current_w = output.next_w;
    committed->current_delta = output.next_delta;
    committed->active = true;
    committed->deadline_expired = false;
    committed->nonzero_authority_retained = output.next_delta != 0.0;
    committed->terminal_predicate = output.exact_terminal_predicate;
    committed->exact_terminal_predicate = output.exact_terminal_predicate;
    committed->reason = output.proof;
    output.committed_status = std::move(committed);
  } catch (const std::bad_alloc&) {
    output.status = RecoveryStepStatus::INVALID;
    output.valid = false;
    output.proof = "recovery committed status could not be materialized";
    return false;
  }
  return true;
}

bool PhaseOffsetRecoveryOwner::prepareCertifiedReserveV2(
    const CertifiedReservePrepareInputV2& input,
    RecoveryPreparedStep& output) const {
  output = RecoveryPreparedStep();
  output.recovery_session = input.recovery_session;
  output.proof_kind = RecoveryStepProofKind::FINITE_RESERVE_V2;
  output.reserve_id = input.reserve.reserve_id;
  output.reserve_cursor = input.cursor;
  output.reserve_size = input.reserve.steps.size();
  output.now = input.now;
  output.deadline = input.deadline_valid ? input.deadline : input.now;
  output.deadline_valid = input.deadline_valid;
  output.selected_u_owner = "PhaseOffsetRecoveryOwner";
  output.provenance = input.provenance;

  std::string reserve_reason;
  if (!configuration_valid_ || input.recovery_session == 0U ||
      input.provenance.empty() || !input.expected_identity.complete() ||
      !SameExecutionIdentity(input.expected_identity, input.reserve.identity) ||
      !input.expected_state.finite() ||
      !TubeExecutionGuardV2::validateReserve(input.reserve,
                                              &reserve_reason) ||
      input.cursor >= input.reserve.steps.size() ||
      input.cursor != input.reserve.cursor ||
      !Finite(input.dt) || input.dt <= 0.0 || input.dt != input.reserve.dt ||
      !input.deadline_valid || !Finite(input.now) || !Finite(input.deadline) ||
      input.now >= input.deadline) {
    output.status = input.deadline_valid && Finite(input.now) &&
            Finite(input.deadline) && input.now >= input.deadline
        ? RecoveryStepStatus::DEADLINE_EXPIRED
        : RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED;
    output.proof = reserve_reason.empty() ? "finite V2 reserve is unavailable"
                                          : reserve_reason;
    return false;
  }
  if (status_->proof_kind == RecoveryStepProofKind::FINITE_RESERVE_V2 &&
      status_->reserve_id == input.reserve.reserve_id &&
      (status_->recovery_session != input.recovery_session ||
       status_->reserve_cursor != input.cursor)) {
    output.status = RecoveryStepStatus::STALE;
    output.proof = "finite V2 reserve session or cursor is stale";
    return false;
  }
  const TubeReserveStepV2& step = input.reserve.steps[input.cursor];
  if (!step.valid || !SameExecutionState(input.expected_state, step.before) ||
      step.command.u_w != step.after.previous_u.u_w ||
      step.command.u_delta != step.after.previous_u.u_delta) {
    output.status = RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED;
    output.proof = "finite V2 reserve expected state or command mismatches";
    return false;
  }
  output.current_w = step.before.w;
  output.current_delta = step.before.delta;
  output.next_w = step.after.w;
  output.next_delta = step.after.delta;
  output.reserve_segment = step.segment;
  output.u_prev = step.before.previous_u;
  output.selected_u = step.command;
  output.selected_u_w = step.command.u_w;
  output.selected_u_delta = step.command.u_delta;
  output.u_w_lower = step.command.u_w;
  output.u_w_upper = step.command.u_w;
  output.u_delta_lower = step.command.u_delta;
  output.u_delta_upper = step.command.u_delta;
  output.dt = input.dt;
  output.measure = std::abs(step.before.delta);
  output.next_measure_upper_bound = std::abs(step.after.delta);
  output.progress = output.measure - output.next_measure_upper_bound;
  output.required_progress = 0.0;
  output.remaining_domain_duration = 0.0;
  output.proof_valid = true;
  output.exact_terminal_predicate = input.cursor + 1U == input.reserve.steps.size() &&
      step.after.delta == 0.0 && step.after.previous_u.u_delta == 0.0 &&
      step.after.previous_u.u_w == 0.0;
  output.terminal_predicate = output.exact_terminal_predicate;
  output.status = RecoveryStepStatus::PREPARED;
  output.valid = output.finite() && output.selectedUExact();
  if (!output.valid) {
    output.status = RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED;
    output.proof = "finite V2 prepared step is not finite or exact";
    return false;
  }
  output.proof = "finite V2 BRAKE-RETURN-SETTLE step";
  try {
    auto committed = std::make_shared<RecoveryOwnerStatus>();
    committed->recovery_session = output.recovery_session;
    committed->committed_sequence = status_->committed_sequence + 1U;
    committed->last_status = RecoveryStepStatus::PREPARED;
    committed->selected_u = output.selected_u;
    committed->current_w = output.next_w;
    committed->current_delta = output.next_delta;
    committed->active = true;
    committed->deadline_expired = false;
    committed->nonzero_authority_retained = output.next_delta != 0.0 ||
        output.selected_u.u_delta != 0.0 || output.selected_u.u_w != 0.0;
    committed->terminal_predicate = output.exact_terminal_predicate;
    committed->exact_terminal_predicate = output.exact_terminal_predicate;
    committed->proof_kind = RecoveryStepProofKind::FINITE_RESERVE_V2;
    committed->reserve_id = input.reserve.reserve_id;
    committed->reserve_cursor = input.cursor + 1U;
    committed->reserve_size = input.reserve.steps.size();
    committed->reserve_segment = step.segment;
    committed->reason = output.proof;
    output.committed_status = std::move(committed);
  } catch (const std::bad_alloc&) {
    output.status = RecoveryStepStatus::INVALID;
    output.valid = false;
    output.proof = "finite V2 committed status could not be materialized";
    return false;
  }
  return true;
}

bool PhaseOffsetRecoveryOwner::validateCommit(
    const RecoveryPreparedStep& prepared) const {
  if (!configuration_valid_ || !prepared.valid || prepared.owner_committed ||
      prepared.selected_u_owner != "PhaseOffsetRecoveryOwner" ||
      !prepared.selectedUConsistent() || !prepared.finite() ||
      prepared.status != RecoveryStepStatus::PREPARED ||
      !prepared.committed_status ||
      (prepared.deadline_valid && prepared.now >= prepared.deadline)) {
    return false;
  }
  if (prepared.proof_kind == RecoveryStepProofKind::FINITE_RESERVE_V2) {
    if (!prepared.selectedUExact() || prepared.reserve_id == 0U ||
        prepared.reserve_size == 0U || prepared.reserve_cursor >=
            prepared.reserve_size || !prepared.committed_status ||
        prepared.committed_status->proof_kind !=
            RecoveryStepProofKind::FINITE_RESERVE_V2 ||
        prepared.committed_status->recovery_session != prepared.recovery_session ||
        prepared.committed_status->reserve_id != prepared.reserve_id ||
        prepared.committed_status->reserve_cursor != prepared.reserve_cursor + 1U) {
      return false;
    }
    if (status_->proof_kind == RecoveryStepProofKind::FINITE_RESERVE_V2 &&
        status_->reserve_id == prepared.reserve_id &&
        (status_->recovery_session != prepared.recovery_session ||
         status_->reserve_cursor != prepared.reserve_cursor)) {
      return false;
    }
  }
  return true;
}

void PhaseOffsetRecoveryOwner::commitNoFail(
    const RecoveryPreparedStep& prepared) noexcept {
  // All strings and status storage were materialized before publication.
  // This seam is intentionally limited to a noexcept shared-pointer swap;
  // no post-publication allocation/copy/validation is permitted.
  status_ = prepared.committed_status;
}

bool PhaseOffsetRecoveryOwner::commit(const RecoveryPreparedStep& prepared) {
  if (!validateCommit(prepared)) return false;
  commitNoFail(prepared);
  return true;
}

void PhaseOffsetRecoveryOwner::reset(const std::uint64_t recovery_session) {
  auto reset_status = std::make_shared<RecoveryOwnerStatus>();
  reset_status->recovery_session = recovery_session;
  status_ = std::move(reset_status);
}

}  // namespace phase_offset_navigation
