# A6/H2 same-authority timer refresh — Runtime rebase execution specification

```text
DOCUMENT_ROLE=DEDICATED_CURRENT_STAGE_EXECUTION_SPECIFICATION
DATE=2026-08-17
STATUS=AUTHORIZED_BY_USER_DELEGATION_2026-08-17
STAGE=A6_H2_SAME_AUTHORITY_TIMER_REFRESH_RUNTIME_REBASE
IMPLEMENTATION_AUTHORIZED=YES__MINIMAL_ADAPTER_ONLY
AUTO_ADVANCE=false
```

## Objective

Repair only the same-owner timer-refresh transaction.  A completed timer epoch
is still an immutable candidate.  Before it replaces the exact live pair, its
new profile must be checked against the latest command state for that same
pair/session/revision/generation, using the existing Runtime exact-PWL
`dryRun()` witness.  The final pair CAS must still require the Runtime bits
captured for that dry run to be bitwise unchanged.

The change addresses the demonstrated starvation mechanism: the old refresh
compared live Runtime bits with an older build request, so an ordinary selected
command during the 90--100 ms build discarded the completed refresh without
ever revalidating it against the current state.

## Evidence boundary

The private bag `active_pair_diagnostics.bag` proves the following and no more:

- rev3/map564 was the H2-installed initial pair; at 50.776960 it was NORMAL
  and selected, and at 50.868337 it was `CERTIFICATE_DENIED`;
- simultaneous raw timer builds use map565 and map566.  The archived control
  snapshots remain map564, so this bag does **not** prove that a timer refresh
  installed a bad profile or caused that particular denial;
- source proves that `refreshPairFromTimerEpoch()` currently requires
  `runtime == request.base_*` before CAS and performs no Runtime exact-PWL
  dry-run.  A selected command can legitimately change those bits while the
  timer builds.

Therefore this stage does not claim to cure the recorded rev3 denial.  It
repairs the independent, source-proven timer-refresh transaction gap.

## Authorized files

- `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h`
- `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
- `src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp`
- this execution specification and its self-audit

## Required transaction

1. Extend only the private immutable command-to-timer request with the
   already-existing command `gains` and `dt`; it is not a ROS schema,
   parameter, control state, or diagnostic.
2. Keep the timer-built epoch's path/profile/frozen-map provenance unchanged.
   At prepare, capture the newest request only when it has the exact same base
   pair pointer, generation, session, source revision and owner.
3. Under the existing command mutex, capture the exact live pair and a local
   Runtime copy plus retained-delta/previous-port bits.  Outside the mutex,
   invoke the existing `PhaseOffsetRuntime::dryRun()` with the candidate
   epoch's immutable profile/provenance and the latest command's current
   path, position, gains and `dt`.  Its existing exact-PWL future witness is
   the acceptance proof.  Reuse H2's existing latest categorical
   OCCUPIED/OUT_OF_MAP veto against that latest command observation; the
   candidate's frozen snapshot remains construction provenance.
4. Under the same short command mutex, require the current request pointer,
   live pair/generation/session and Runtime bits to equal the prepare capture;
   keep the existing pin exclusion and perform the one shared-pointer CAS.
   The CAS writes only the new immutable pair/profile/epoch.  It never writes,
   resets, or fabricates Runtime or path state.

## Explicit prohibitions

- No A5 geometry, Filter, validator, margin, map policy, H2 path transaction,
  pin/session semantics, control selection, physical command, launch or
  parameter change.
- No gate, retry loop, mode, reason, enum, latch, topic or diagnostics schema.
- No old-profile reuse across owners, no stale-request acceptance, and no
  relaxation from exact pair/generation/session/runtime-bit identity.

## Required tests

1. A normal Runtime commit during the build makes the timer refresh rebase to
   latest same-pair state and install only if its exact witness passes.
2. A valid latest rebase installs one new same-owner pair with a new epoch;
   Runtime state remains bit-identical across the pair CAS.
3. A Runtime-bit or latest-request change after prepare prevents final CAS;
   no pair/profile/path/Runtime mutation occurs.
4. Candidate witness failure, latest categorical unsafe, owner/provenance
   mismatch, stale pair/session or active H2 pin rejects without publication.
5. Existing 100-cycle selected execution remains unpolluted, and the H2
   rebase/pin/session regressions remain unchanged.

Run Adapter, Runtime, H2/integration, GVF and continuous-path focused
regressions, `git diff --check`, whitelist audit, and final status.  A dynamic
flight is not part of this code correction; its acceptance needs a separate
authorized run.
