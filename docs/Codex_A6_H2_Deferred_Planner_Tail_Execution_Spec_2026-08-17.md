# A6/H2 deferred planner-tail — execution specification

```text
DOCUMENT_ROLE=DEDICATED_CURRENT_STAGE_EXECUTION_SPECIFICATION
DATE=2026-08-17
STATUS=AUTHORIZED_BY_USER_2026-08-17
STAGE=A6_H2_EVENT_DRIVEN_DEFERRED_PLANNER_TAIL
IMPLEMENTATION_AUTHORIZED=YES__MANAGER_AND_H2_TEST_ONLY
AUTO_ADVANCE=false
```

## Objective

Repair only the observed availability gap in which a point-phase replan has
already been accepted by the existing policy but has no certified future seam
on its currently captured pair.  Preserve that accepted planner result as a
private immutable, non-authoritative deferred tail.  It may be considered only
once: at the next direct, same-owner command refresh (`generation == base + 1`).
It must then undergo the existing latest-state candidate acceptance, C2,
new-owner tube construction, Runtime rebase/dry-run, and exact H2 CAS
transaction.  It is not a retry loop and it does not grant path or tube
authority.

## Evidence and boundary

The 2026-08-17 no-certified-future-seam audit proves that terminal path
headroom cannot produce a legal old-prefix seam.  The nonterminal audit proves
that active certificate availability can lag a still-valid path.  The timer
refresh audit proves same-owner refreshes can advance an installed active
certificate.  Therefore the only authorised response is to retain the already
accepted planner artifact until the next direct refresh, not to relax the seam,
certificate, Filter, Runtime, or CAS contracts.

## Exclusive whitelist

- `src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h`
- `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`
- `src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp`
- this specification and its self-audit

## Required design

1. Add one private manager-owned `DeferredPlannerTail` value.  It holds only
   immutable planner artifacts and base authority facts: exact base pair,
   session, source revision, generation, semantic path owner, goal, phase
   anchor/capture time, and candidate spline/sample data.  It holds no pin,
   pending/completed H2 slot, Runtime state, tube/profile/sample/witness or
   authority.
2. Save exactly one value only when the existing point-phase candidate has
   already passed `shouldAcceptCandidate` and H2 staging explicitly fails for
   no currently certified future seam.  A save does not publish a path/tube
   pair, mirror, control command, or manager epoch.
3. Consume only on the next direct same-owner refresh: the live pair must be
   the exact base pair/session/source/owner and live authoritative generation
   must equal `base_generation + 1`.  Any skipped generation, identity drift,
   reset/new goal/new replan, or occupied pending/completed H2 mailbox clears
   the value.  Successful or failed consumption clears it and never requeues.
4. Before any pin acquisition, re-evaluate the candidate against the newest
   progress, path end, anchor and goal using the existing acceptance logic.
   Rebuild global phase samples, C2 and a wholly new new-owner tube with the
   latest snapshot.  The old deferred value is planner input only: no old tube,
   profile, sample, clearance or Runtime witness may cross owner boundaries.
5. The stage helper must accept the expected refreshed pair.  After acquiring
   the existing pin, it requires its capture pair to be exactly that expected
   pair, preventing a check-to-pin second-refresh race.  Existing H2
   Runtime-rebase, exact CAS, pin, session, and `current_w < future_seam_w`
   requirements remain unchanged.

## Prohibitions

Do not alter Adapter, A5 geometry, Filter, validator, TubeEpochManager,
margins, speed, rate, lookahead, Runtime-rebase CAS/pin/session semantics,
parameters, launch files, AGENTS, control state, gate, mode, reason, enum or
diagnostic schema.  Do not introduce a timer or periodic retry, a second
planner execution, a physical recovery behaviour, old owner/profile/sample
reuse, a path-only publish, or a safety-condition relaxation.

## Required deterministic coverage

- no seam saves a tail and publishes nothing;
- base generation `G` then direct refresh `G+1` consumes once;
- `G+2`, authority drift, reset/new goal/new replan and H2-slot conflict clear;
- consumption rebuilds latest samples/C2/snapshot/new owner and has no old
  tube/profile/sample reuse;
- passed seam, Runtime/CAS drift and latest acceptance rejection publish
  nothing;
- all existing H2 and 100-cycle runtime regressions remain green.

## Verification and stop boundary

Run focused build and the H2/GVF, Adapter, Runtime, tube-epoch and continuous
path regressions; run dependency and whitelist searches, `git diff --check`,
and record final status.  Then run one task-owned private active ESDF episode
with only `phase_offset_manual_observe_only:=false`, specifically seeking
`no seam -> same-owner refresh -> one H2 replacement`; it cannot use a
deadline as evidence.  Clean up only task-owned processes.  If the evidence
does not establish the required event chain, report NOT PASS without tuning or
adding a gate.

## Specification self-audit

The whitelist is limited to H2 manager orchestration and its test.  The value
is neither an execution authority nor a control state: it is discarded on all
identity/lifecycle uncertainty, before any old authority is changed.  Every
actual installation still goes through the existing fresh new-owner H2
transaction and its latest Runtime/CAS proof.  This plan does not reinterpret
terminal path exhaustion as a seam-availability problem and does not claim an
unobserved physical-flight pass.
