#pragma once

#include "phase_offset_navigation/active_reference_snapshot.h"
#include "phase_offset_navigation/preview_feasibility.h"

#include <cstdint>
#include <string>

namespace phase_offset_navigation {

enum class HandoffState {
  NORMAL,
  RECENTERING_FOR_HANDOFF,
  HANDOFF_READY,
  PLANNER_ONLY,
  WAIT_FOR_PLANNER_RECOVERY,
  CURRENT_STATE_UNSAFE,
  PLANNER_INVALID,
};

enum class HandoffEvent {
  NONE,
  SUCCESSOR_COMPATIBLE,
  SUCCESSOR_ZERO_ONLY,
  SUCCESSOR_DISCONNECTED,
  PREVIEW_INFEASIBLE,
  TEMPORARY_DENY,
  STALE,
  RECENTER_PROGRESS,
  NEUTRAL_REACHED,
  ATOMIC_NEUTRAL_HANDOFF,
  CURRENT_UNSAFE,
  PLANNER_NO_PATH,
};

const char* handoffStateName(HandoffState state);
const char* handoffEventName(HandoffEvent event);

struct HandoffStateInput {
  HandoffState state = HandoffState::NORMAL;
  ActiveReferenceOwnerMode owner_mode = ActiveReferenceOwnerMode::NONE;
  double delta = 0.0;
  double target_delta = 0.0;
  double neutral_tolerance = 1e-9;
  bool current_state_safe = true;
  bool planner_valid = true;
  bool successor_available = false;
  bool successor_zero_only = false;
  bool successor_contains_current_delta = false;
  bool successor_disconnected = false;
  bool preview_feasible = false;
  bool preview_target_overlap = false;
  bool preview_denied = false;
  bool stale = false;
  bool recovery_progress_certified = false;
  bool neutral_handoff_committed = false;
  HandoffEvent event = HandoffEvent::NONE;
};

struct HandoffDecision {
  HandoffState previous_state = HandoffState::NORMAL;
  HandoffState next_state = HandoffState::NORMAL;
  HandoffEvent event = HandoffEvent::NONE;
  bool retain_current_owner = false;
  bool request_recovery = false;
  bool request_recenter = false;
  bool neutral_handoff_ready = false;
  bool planner_only_allowed = false;
  bool planner_veto = false;
  std::uint64_t transition_sequence = 0U;
  std::string reason;
};

// Small deterministic lifecycle FSM.  It owns only lifecycle state; it never
// owns selected-u or writes ActiveReferenceSnapshot.  `evaluate` is pure with
// respect to the object and `transition` is the explicit state-commit API.
class HandoffStateMachine {
 public:
  HandoffStateMachine() = default;

  HandoffState state() const { return state_; }
  std::uint64_t transitionSequence() const { return transition_sequence_; }

  bool evaluate(const HandoffStateInput& input,
                HandoffDecision& decision) const;
  bool transition(const HandoffStateInput& input,
                  HandoffDecision& decision);
  // Commit an already evaluated decision after the local command publication
  // seam.  The decision carries the exact predecessor state/sequence that was
  // validated before publication; this write cannot fail or allocate.
  void commitNoFail(const HandoffDecision& decision) noexcept;
  void reset(HandoffState state = HandoffState::NORMAL);

 private:
  HandoffState state_ = HandoffState::NORMAL;
  std::uint64_t transition_sequence_ = 0U;
};

}  // namespace phase_offset_navigation
