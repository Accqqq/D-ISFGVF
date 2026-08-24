#include "phase_offset_navigation/handoff_state_machine.h"

#include <cmath>

namespace phase_offset_navigation {
namespace {

bool NearlyZero(const double value, const double tolerance) {
  return std::isfinite(value) && std::isfinite(tolerance) &&
      tolerance >= 0.0 && std::abs(value) <= tolerance;
}

}  // namespace

const char* handoffStateName(const HandoffState state) {
  switch (state) {
    case HandoffState::NORMAL: return "NORMAL";
    case HandoffState::RECENTERING_FOR_HANDOFF:
      return "RECENTERING_FOR_HANDOFF";
    case HandoffState::HANDOFF_READY: return "HANDOFF_READY";
    case HandoffState::PLANNER_ONLY: return "PLANNER_ONLY";
    case HandoffState::WAIT_FOR_PLANNER_RECOVERY:
      return "WAIT_FOR_PLANNER_RECOVERY";
    case HandoffState::CURRENT_STATE_UNSAFE: return "CURRENT_STATE_UNSAFE";
    case HandoffState::PLANNER_INVALID: return "PLANNER_INVALID";
  }
  return "UNKNOWN";
}

const char* handoffEventName(const HandoffEvent event) {
  switch (event) {
    case HandoffEvent::NONE: return "NONE";
    case HandoffEvent::SUCCESSOR_COMPATIBLE: return "SUCCESSOR_COMPATIBLE";
    case HandoffEvent::SUCCESSOR_ZERO_ONLY: return "SUCCESSOR_ZERO_ONLY";
    case HandoffEvent::SUCCESSOR_DISCONNECTED: return "SUCCESSOR_DISCONNECTED";
    case HandoffEvent::PREVIEW_INFEASIBLE: return "PREVIEW_INFEASIBLE";
    case HandoffEvent::TEMPORARY_DENY: return "TEMPORARY_DENY";
    case HandoffEvent::STALE: return "STALE";
    case HandoffEvent::RECENTER_PROGRESS: return "RECENTER_PROGRESS";
    case HandoffEvent::NEUTRAL_REACHED: return "NEUTRAL_REACHED";
    case HandoffEvent::ATOMIC_NEUTRAL_HANDOFF:
      return "ATOMIC_NEUTRAL_HANDOFF";
    case HandoffEvent::CURRENT_UNSAFE: return "CURRENT_UNSAFE";
    case HandoffEvent::PLANNER_NO_PATH: return "PLANNER_NO_PATH";
  }
  return "UNKNOWN";
}

bool HandoffStateMachine::evaluate(const HandoffStateInput& input,
                                   HandoffDecision& decision) const {
  decision = HandoffDecision();
  decision.previous_state = state_;
  decision.next_state = state_;
  decision.event = input.event;
  decision.transition_sequence = transition_sequence_ + 1U;

  if (!input.current_state_safe || input.event == HandoffEvent::CURRENT_UNSAFE) {
    decision.next_state = HandoffState::CURRENT_STATE_UNSAFE;
    decision.planner_veto = true;
    decision.reason = "explicit current-state safety failure";
    return true;
  }
  if (!input.planner_valid || input.event == HandoffEvent::PLANNER_NO_PATH) {
    decision.next_state = HandoffState::PLANNER_INVALID;
    decision.planner_veto = true;
    decision.reason = "planner declared no valid path";
    return true;
  }

  const bool nonzero = !NearlyZero(input.delta, input.neutral_tolerance);
  const bool zero_only = input.successor_zero_only;
  const bool successor_unusable = !input.successor_available ||
      input.successor_disconnected || input.stale || input.preview_denied ||
      (!input.preview_feasible && input.successor_available);

  if (input.event == HandoffEvent::ATOMIC_NEUTRAL_HANDOFF ||
      input.neutral_handoff_committed) {
    if (!NearlyZero(input.delta, input.neutral_tolerance)) {
      decision.next_state = HandoffState::RECENTERING_FOR_HANDOFF;
      decision.request_recenter = true;
      decision.retain_current_owner = true;
      decision.reason = "neutral handoff requested before recenter is complete";
      return true;
    }
    decision.next_state = HandoffState::PLANNER_ONLY;
    decision.neutral_handoff_ready = true;
    decision.planner_only_allowed = true;
    decision.reason = "atomic neutral handoff is complete";
    return true;
  }

  if (zero_only || (nonzero && !input.successor_contains_current_delta)) {
    if (nonzero) {
      decision.next_state = HandoffState::RECENTERING_FOR_HANDOFF;
      decision.request_recenter = true;
      decision.request_recovery = true;
      decision.retain_current_owner = true;
      decision.reason = zero_only ? "successor is ZERO_ONLY"
                                  : "successor excludes current component";
      return true;
    }
    if (input.successor_available && input.successor_contains_current_delta &&
        input.preview_feasible) {
      decision.next_state = HandoffState::NORMAL;
      decision.reason = "neutral successor is compatible";
      return true;
    }
    decision.next_state = HandoffState::PLANNER_ONLY;
    decision.planner_only_allowed = true;
    decision.reason = "neutral planner-only successor is allowed";
    return true;
  }

  if (successor_unusable || input.event == HandoffEvent::PREVIEW_INFEASIBLE ||
      input.event == HandoffEvent::TEMPORARY_DENY ||
      input.event == HandoffEvent::STALE) {
    decision.next_state = nonzero ? HandoffState::WAIT_FOR_PLANNER_RECOVERY
                                   : HandoffState::PLANNER_ONLY;
    decision.retain_current_owner = nonzero;
    decision.request_recovery = nonzero;
    decision.reason = nonzero ? "retain safe owner while successor is pending"
                              : "planner-only owner remains valid";
    return true;
  }

  if (input.recovery_progress_certified && nonzero &&
      NearlyZero(input.delta - input.target_delta, input.neutral_tolerance)) {
    decision.next_state = HandoffState::HANDOFF_READY;
    decision.neutral_handoff_ready = NearlyZero(input.target_delta,
                                                input.neutral_tolerance);
    decision.retain_current_owner = true;
    decision.reason = "recovery reached a handoff-compatible state";
    return true;
  }

  if (input.event == HandoffEvent::RECENTER_PROGRESS && nonzero) {
    decision.next_state = HandoffState::RECENTERING_FOR_HANDOFF;
    decision.request_recenter = true;
    decision.request_recovery = true;
    decision.retain_current_owner = true;
    decision.reason = "in-owner recenter progress retains the safe owner";
    return true;
  }

  decision.next_state = HandoffState::NORMAL;
  decision.reason = "successor and preview are compatible";
  return true;
}

bool HandoffStateMachine::transition(const HandoffStateInput& input,
                                     HandoffDecision& decision) {
  if (!evaluate(input, decision)) return false;
  commitNoFail(decision);
  return true;
}

void HandoffStateMachine::commitNoFail(
    const HandoffDecision& decision) noexcept {
  state_ = decision.next_state;
  transition_sequence_ = decision.transition_sequence;
}

void HandoffStateMachine::reset(const HandoffState state) {
  state_ = state;
  transition_sequence_ = 0U;
}

}  // namespace phase_offset_navigation
