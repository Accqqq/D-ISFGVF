#include <gtest/gtest.h>

#include "phase_offset_navigation/handoff_state_machine.h"

namespace phase_offset_navigation {
namespace {

TEST(HandoffStateMachineTest, ZeroOnlyRecenterThenAtomicNeutralHandoff) {
  HandoffStateMachine machine;
  HandoffStateInput input;
  input.delta = 0.2;
  input.successor_available = true;
  input.successor_zero_only = true;
  HandoffDecision decision;
  ASSERT_TRUE(machine.transition(input, decision));
  EXPECT_EQ(machine.state(), HandoffState::RECENTERING_FOR_HANDOFF);
  EXPECT_TRUE(decision.request_recovery);
  input.state = machine.state();
  input.delta = 0.0;
  input.successor_zero_only = true;
  input.event = HandoffEvent::ATOMIC_NEUTRAL_HANDOFF;
  input.neutral_handoff_committed = true;
  ASSERT_TRUE(machine.transition(input, decision));
  EXPECT_EQ(machine.state(), HandoffState::PLANNER_ONLY);
  EXPECT_TRUE(decision.planner_only_allowed);
}

TEST(HandoffStateMachineTest, DenyRetainsNonzeroOwnerAndUnsafeFailsClosed) {
  HandoffStateMachine machine;
  HandoffStateInput input;
  input.delta = 0.1;
  input.successor_available = true;
  input.successor_contains_current_delta = false;
  input.event = HandoffEvent::TEMPORARY_DENY;
  HandoffDecision decision;
  ASSERT_TRUE(machine.evaluate(input, decision));
  EXPECT_TRUE(decision.retain_current_owner);
  EXPECT_FALSE(decision.planner_veto);
  input.current_state_safe = false;
  input.event = HandoffEvent::CURRENT_UNSAFE;
  ASSERT_TRUE(machine.evaluate(input, decision));
  EXPECT_EQ(decision.next_state, HandoffState::CURRENT_STATE_UNSAFE);
  EXPECT_TRUE(decision.planner_veto);
}

TEST(HandoffStateMachineTest,
     RepeatedTemporarySuccessorDenialsNeverCreatePlannerHold) {
  HandoffStateMachine machine;
  const HandoffEvent temporary_events[] = {
      HandoffEvent::SUCCESSOR_DISCONNECTED,
      HandoffEvent::PREVIEW_INFEASIBLE,
      HandoffEvent::TEMPORARY_DENY,
      HandoffEvent::STALE};
  for (const HandoffEvent event : temporary_events) {
    HandoffStateInput input;
    input.delta = 0.12;
    input.successor_available = true;
    input.successor_contains_current_delta = false;
    input.event = event;
    HandoffDecision decision;
    for (int repeat = 0; repeat < 8; ++repeat) {
      ASSERT_TRUE(machine.evaluate(input, decision));
      EXPECT_TRUE(decision.retain_current_owner);
      EXPECT_FALSE(decision.planner_veto);
      EXPECT_NE(decision.next_state, HandoffState::PLANNER_ONLY);
    }
  }
}

TEST(HandoffStateMachineTest,
     ExplicitRepeatedZeroOnlyAndTemporaryFailuresRetainNonzeroOwner) {
  const HandoffEvent events[] = {
      HandoffEvent::SUCCESSOR_ZERO_ONLY,
      HandoffEvent::SUCCESSOR_DISCONNECTED,
      HandoffEvent::PREVIEW_INFEASIBLE,
      HandoffEvent::TEMPORARY_DENY,
      HandoffEvent::STALE};
  for (const HandoffEvent event : events) {
    HandoffStateMachine machine;
    HandoffStateInput input;
    input.delta = 0.12;
    input.successor_available = true;
    input.successor_contains_current_delta = true;
    input.preview_feasible = true;
    input.event = event;
    if (event == HandoffEvent::SUCCESSOR_ZERO_ONLY) {
      input.successor_zero_only = true;
    } else if (event == HandoffEvent::SUCCESSOR_DISCONNECTED) {
      input.successor_disconnected = true;
    } else if (event == HandoffEvent::PREVIEW_INFEASIBLE) {
      input.preview_feasible = false;
    } else if (event == HandoffEvent::TEMPORARY_DENY) {
      input.preview_denied = true;
    } else if (event == HandoffEvent::STALE) {
      input.stale = true;
    }

    HandoffDecision decision;
    for (int repeat = 0; repeat < 4; ++repeat) {
      ASSERT_TRUE(machine.transition(input, decision));
      EXPECT_TRUE(decision.retain_current_owner);
      EXPECT_TRUE(decision.request_recovery);
      EXPECT_FALSE(decision.planner_only_allowed);
      EXPECT_FALSE(decision.planner_veto);
      EXPECT_NE(machine.state(), HandoffState::PLANNER_ONLY);
    }
  }
}

TEST(HandoffStateMachineTest, AtomicNeutralHandoffIsTerminalOnlyAtNeutral) {
  HandoffStateMachine machine;
  HandoffStateInput input;
  input.delta = 0.2;
  input.event = HandoffEvent::ATOMIC_NEUTRAL_HANDOFF;
  input.neutral_handoff_committed = true;
  HandoffDecision decision;
  ASSERT_TRUE(machine.transition(input, decision));
  EXPECT_EQ(machine.state(), HandoffState::RECENTERING_FOR_HANDOFF);
  EXPECT_TRUE(decision.request_recenter);
  EXPECT_TRUE(decision.retain_current_owner);
  EXPECT_FALSE(decision.planner_only_allowed);

  input.delta = 0.0;
  ASSERT_TRUE(machine.transition(input, decision));
  EXPECT_EQ(machine.state(), HandoffState::PLANNER_ONLY);
  EXPECT_TRUE(decision.neutral_handoff_ready);
  EXPECT_TRUE(decision.planner_only_allowed);
  EXPECT_FALSE(decision.retain_current_owner);
}

TEST(HandoffStateMachineTest, PlannerVetoIsRestrictedToUnsafeOrInvalidPlanner) {
  HandoffStateMachine machine;
  HandoffStateInput input;
  input.delta = 0.15;
  input.successor_available = true;
  input.successor_contains_current_delta = true;
  input.preview_feasible = true;
  input.event = HandoffEvent::SUCCESSOR_COMPATIBLE;
  HandoffDecision decision;
  ASSERT_TRUE(machine.evaluate(input, decision));
  EXPECT_FALSE(decision.planner_veto);

  input.event = HandoffEvent::TEMPORARY_DENY;
  input.preview_denied = true;
  ASSERT_TRUE(machine.evaluate(input, decision));
  EXPECT_FALSE(decision.planner_veto);

  input.event = HandoffEvent::CURRENT_UNSAFE;
  input.current_state_safe = false;
  ASSERT_TRUE(machine.evaluate(input, decision));
  EXPECT_EQ(decision.next_state, HandoffState::CURRENT_STATE_UNSAFE);
  EXPECT_TRUE(decision.planner_veto);

  machine.reset();
  input = HandoffStateInput();
  input.event = HandoffEvent::PLANNER_NO_PATH;
  input.planner_valid = false;
  ASSERT_TRUE(machine.evaluate(input, decision));
  EXPECT_EQ(decision.next_state, HandoffState::PLANNER_INVALID);
  EXPECT_TRUE(decision.planner_veto);
}

}  // namespace
}  // namespace phase_offset_navigation

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
