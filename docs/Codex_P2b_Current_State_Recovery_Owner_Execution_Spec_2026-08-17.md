# Codex P2b T6: current-state recovery owner execution specification

```text
DOCUMENT_ROLE=DEDICATED_CURRENT_STAGE_EXECUTION_SPECIFICATION
DOCUMENT_STATUS=AUTHORIZED_BY_USER_2026-08-17
STAGE=P2B_T6_CURRENT_STATE_RECOVERY_OWNER
AUTO_ADVANCE=false
PREDECESSORS=P1_PASS__P2A_MAILBOX_INFRASTRUCTURE_PASS
GO=NOT_GRANTED_PENDING_THIS_STAGE_ACCEPTANCE
```

## 1. Authorized objective

This stage may add one route-specific `current_state_recovery_owner` and connect
it to the already-isolated P2a mutex mailbox.  Its only purpose is to prevent a
command cycle carrying an existing non-executable current-state denial from
silently publishing `VEL_MATCH_GOVERNOR`.

The owner is not a tube builder, Filter change, H2 repair, generic HOLD, old
certificate reuse, or an uncontrolled replan.  It may issue a recovery command
only after it proves, from the actual current state, a finite physical recovery
segment under the exact immutable snapshot/provenance of the request.  The FSM
remains the only `exec_state_` writer.  The P2a mutex mailbox remains the sole
producer-to-FSM transport and is not connected to H2 pin/CAS/session machinery.

## 2. Theory contract frozen for implementation

For an exact `PathTubePair` identity `(pointer, revision, generation, session,
snapshot)`, let `x=(p,v)` be the state captured with the denial.  A P2b command
is publishable only if all of the following are established in one pure-C++
calculation against that pair's immutable cloud snapshot:

1. `p` and every point of the finite swept recovery segment are categorically
   `KNOWN_FREE`, and their closed-ball clearance is certified at the explicitly
   selected recovery radius.  The recovery radius must be derived without
   changing the approved tube margins.  It is reported as a *hard physical
   clearance* contract and must never be represented as restoring the tube's
   residual/full robust certificate.
2. The whole position-command segment, including the plant's bounded one-cycle
   motion from `(p,v)` to its commanded endpoint, is covered by an analytic or
   conservative swept-volume certificate.  Sampling points alone are not a
   certificate unless their spacing is proven from the snapshot grid and the
   swept ball radius.
3. The commanded velocity has nonnegative projection on the owner path's
   physical tangent, and is compatible with the existing command period and
   port-rate/non-reverse contract.  A recovery owner may not reverse phase or
   publish a port that contradicts its own path/tube owner.
4. A recovery command can exit only when a fresh P1 continuous witness is
   selected from the exact still-live identity.  It cannot exit by an old
   profile, baseline governor, a new H2 candidate, a timer retry, or a state
   reset.

`CERTIFICATE_DENIED` with an actually known-free, tube-inside current state is
not an empty set and cannot be relabelled as emergency.  `UNKNOWN`,
`OUT_OF_MAP`, `OCCUPIED`, unavailable snapshot, or a failure of any swept/dynamic
proof is fail-closed.  If a physically publishable action for any such input
cannot be proved in the files listed below, no product command is implemented;
the stage stops with that minimal blocker.

## 3. Exclusive whitelist

- `src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/current_state_recovery_owner.h`
- `src/swarm_planner/phase_offset/phase_offset_navigation/src/current_state_recovery_owner.cpp`
- `src/swarm_planner/phase_offset/phase_offset_navigation/test/current_state_recovery_owner_test.cpp`
- `src/swarm_planner/phase_offset/phase_offset_navigation/CMakeLists.txt`
- `src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h`
- `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`
- `src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp`
- `src/swarm_planner/bspline_traj/CMakeLists.txt` only if the existing target
  needs an explicit link registration
- this specification and its P2b self-audit.

## 4. Explicit prohibitions

- Do not alter A5 geometry, TubeFilter, cross-section, SurfaceValidator,
  margins, speed/rate/lookahead/tracking parameters, TubeEpochManager, P1
  witness, Adapter, H2 CAS/pin/session/runtime-bit, launch files, or AGENTS.
- Do not add a product gate, latch, mode, state, reason, enum, diagnostic
  schema, ROS parameter, topic, thread, global queue, or low-level/SO3 mux.
- Do not change or reinterpret the existing `CURRENT_OFFSET_OUTSIDE` and
  `CERTIFICATE_DENIED` facts.  Do not use an old tube/C2/profile/sample/clearance
  result after an owner, map, snapshot, or session change.
- Do not call a generic position HOLD, legacy replan, baseline governor, or
  simulator timeout a recovery owner.  Do not start, attach to, or stop any
  user ROS process.

## 5. Required source audit before product edits

The implementation author must record, in the self-audit, the exact existing
source of (a) actual pose and velocity, (b) command period and command-to-plant
kinematics, (c) immutable snapshot clearance query and its coverage semantics,
and (d) a command-publication seam capable of overriding the same-cycle legacy
governor.  It must also establish whether P2a's pending request carries the
state and snapshot needed to satisfy §2 without reading mutable map/odom after
its producer linearization point.

If any item is absent, or supplying it requires a non-whitelisted message,
Adapter/mailbox payload, low-level controller, parameter, H2, or state-machine
change, stop before product code.  A test-only synthetic input is permitted to
prove the pure owner contract; it cannot be used to claim production wiring.

## 6. Required tests and acceptance

Focused pure C++ tests must cover a fully certified swept segment, exact
snapshot provenance mismatch, stale owner/session, insufficient hard clearance,
unknown/out-of-map/occupied input, non-finite state, phase/tangent reversal,
port-rate violation, and the exit requirement.  Tests must distinguish hard
clearance from residual/full tube certification.

If production wiring is proved feasible, `gvf_switch_policy_test` must also
prove: denied/outside final command source is not baseline in the same cycle;
mailbox identity is consumed exactly once; stale request is consumed zero times;
the FSM alone writes `exec_state_`; and 100-cycle P1/H2 authority regressions
remain valid.  P1, H2, and focused recovery tests plus `git diff --check`,
whitelist audit, and final status are required.

No physical ROS acceptance is authorized unless the existing unchanged
PositionCommand-to-plant dynamic bound is first proven adequate by the source
audit.  A private ROS run, if later justified, must use an owned master and
process cleanup only.

## 7. Stop conditions

Stop with no product approximation if the exact P2a request lacks an immutable
actual state/snapshot, the current command-to-plant dynamics lack a bounded
one-cycle contract, the snapshot cannot certify a swept ball, the required
hard recovery radius has no authorized derivation, or continuation requires a
new multi-cycle execution state/exit mux.  These are architecture-contract
blockers, not reasons to add a gate or weaken a tube condition.

After implementation and self-audit, stop.  P2 completion, dynamic GO, A6
changes, tube-model work, and any follow-on physical-control architecture need
new authorization.
