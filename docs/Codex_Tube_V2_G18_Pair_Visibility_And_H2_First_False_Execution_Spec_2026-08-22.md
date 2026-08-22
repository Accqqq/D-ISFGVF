# Tube V2 G18 Pair visibility and H2 first-false execution spec

    DOCUMENT_ROLE=PRODUCT_CORRECTION_AND_BOUNDED_ATTRIBUTION_SPEC
    DOCUMENT_STATUS=ACTIVE
    DATE=2026-08-22
    PLAN_OWNER=PRIMARY_CODEX_AGENT
    EXECUTOR=LUNA_MAX_SINGLE_AGENT
    SUBAGENT_CREATION_ALLOWED=false
    PRODUCT_SCOPE=P1_MARKER_OWNERSHIP_PLUS_P0_EXISTING_FAILURE_ATTRIBUTION
    NAVIGATION_RECOVERY_CHANGE_ALLOWED=false
    NO_NEW_GATE_CACHE_RETRY_TIMER_MODULE_STATE=true
    NO_LIMIT_MARGIN_RATE_HORIZON_LAUNCH_CHANGE=true

## 1. Dynamic evidence and goal

The authoritative G17 replacement run is:

    /dev/shm/tube_v2_g17_dynamic_20260822_182050/

Its status is `PARTIALLY_FIXED`:

- G1 reached and G2 timed out after the unchanged 240 s bound;
- G3--G5 were not sent;
- raw Candidate produced 7,617 ADD markers with maximum width 6.0 m;
- Certified produced zero ADD and 7,716 DELETE markers;
- two Pair bootstraps committed and ten nonzero delta commands executed;
- no 249993/250000 query explosion and no 101-attempt cohort remained;
- two H2 stages returned false;
- the retained old path reached its end and produced 1,186 exact 36/0/36
  HOLD events, lasting 235.10 s through G2 timeout.

G18 keeps two owners separate:

1. **P1 visibility correction:** publish the genuine active Pair certificate
   while continuing to show the broad timer Candidate evidence.
2. **P0 attribution only:** expose the already-computed H2 stage failure enum
   and, only for a staging dry-run failure, the already-computed Runtime result.

G18 does not claim or implement a navigation-liveness repair.  No recovery,
recenter, authority reset, direct neutral switch, controller, retry, gate, or
limit change is authorized.  The new P0 evidence decides the next plan.

## 2. Frozen contracts

The following remain unchanged:

- G17 authority-subset construction and request formula;
- Builder, Filter, Validator, Runtime, PortProjector, Pair CAS and H2 safety;
- raw/environment/raw_build_samples and broad Candidate marker geometry;
- Candidate remains explicitly uncertified evidence;
- Certified marker remains active-control evidence and may ADD only when
  `tubeDisplayCertified()` returns true;
- latest categorical unsafe masking and Runtime certificate denial;
- neutral retirement must continue rejecting an executed nonzero authority;
- H2 stays one C2/Tube staging attempt per replan callback;
- later callbacks may retry only through the existing callback behavior;
- no launch, parameter, map, query/depth/attempt count, safety margin, rate,
  horizon, profile, gain, timer, or ROS schema change.

Do not modify Tube construction, geometry, Runtime, PortProjector, planner
selection, governor, recovery handling, or neutral predicates in G18.

## 3. Phase A: exact live-Pair publication composition

### 3.1 Proven mismatch

The Pair command branch publishes a `ControlPublishSnapshot` whose epoch is
the exact Pair epoch and whose candidate/active profiles are the same installed
narrow profile.  Independently, the timer owns a later broad raw Candidate
epoch.  `publishManual()` currently requires the Pair epoch pointer to equal
the timer Candidate epoch pointer.  They are intentionally different owners,
so the integrity-failure branch replaces Candidate with the timer profile,
forces Certified DELETE, and returns before manual/epoch diagnostics.

This explains the exact dynamic tuple:

    Pair commit/execution > 0
    Candidate ADD > 0
    Certified ADD = 0
    manual diagnostics = 0
    epoch diagnostics = 0

### 3.2 Exact live-Pair predicate

Inside existing `publishManual()`, form a transaction-local boolean only.  It
is not persistent state or a new gate.  The Pair control is exact and live only
when all existing identities agree:

- current request is active and its task/control/source identity already
  passed `current_control_identity`;
- `request->base_path_tube_pair` is non-null and pointer-equal to the current
  atomically loaded authoritative PathTubePair;
- request base Pair generation and authority session equal the live Pair;
- control epoch pointer equals `live_pair->epoch_snapshot`;
- control output active profile equals `live_pair->active_profile`;
- control output candidate profile equals the Pair-owned profile expected by
  the command branch;
- control epoch/status/profile identities already satisfy
  `output_matches_epoch`;
- task generation and authority session remain current.

Do not weaken or delete any normal timer epoch/request/map checks.  This is an
alternate valid ownership composition for the exact live Pair only.  A stale,
retired, replaced, malformed, or pointer-mismatched Pair continues through the
existing fail-closed Certified DELETE branch.

### 3.3 Marker composition

For an exact live-Pair control:

- base path, active path, frame, Certified marker, manual diagnostics, and
  epoch diagnostics use the Pair command snapshot;
- Certified geometry uses the Pair active profile and the unchanged
  `tubeDisplayCertified(control.output)` predicate;
- Candidate marker uses the latest authoritative timer Candidate profile when
  one exists, so it continues to show current broad raw `I_geo`;
- if no current timer Candidate exists, the Pair profile may supply Candidate
  raw geometry only through its preserved `raw_build_samples` path;
- never use filtered narrow Pair endpoints as Candidate display geometry;
- never use timer Candidate status to certify the Pair;
- never use Pair certification to promote the Candidate namespace.

Implement this by extending the existing local identity composition in
`publishManual()`.  Do not add another publisher, topic, timer, cache, mailbox,
marker module, or persistent owner.  Prefer allowing the existing common
publish/diagnostics tail to execute, with only a Candidate marker override for
the dual-owner case.  Avoid duplicating the whole publication block.

### 3.4 Phase A tests

Add focused tests that prove:

1. exact live Pair + distinct newer timer Candidate:
   - Candidate ADD uses the timer profile's broad raw geometry;
   - Certified ADD uses the Pair profile's narrow filtered geometry;
   - manual and epoch diagnostics publish;
   - Candidate and Certified pointers/widths are not swapped;
2. exact live Pair with no timer Candidate:
   - Candidate remains broad through Pair `raw_build_samples`;
   - Certified remains narrow;
3. Pair refreshed/copied epoch is accepted only when it is the exact current
   authoritative Pair snapshot;
4. stale/replaced/retired Pair snapshot still forces Certified DELETE and
   cannot overwrite the latest Candidate;
5. any `tubeDisplayCertified()` false fact still forces Certified DELETE;
6. candidate-only and ordinary timer publication behavior is bitwise/semantic
   regression-equivalent;
7. marker counts and namespace ownership remain three ADD or three DELETE
   elements with no ghost geometry.

## 4. Phase B: existing H2 stage failure attribution

### 4.1 Propagate the existing enum

`PathTubePairStageFailure` already contains:

    INPUT_PRECONDITION
    TRANSACTION_PRECONDITION
    PAIR_SESSION_RUNTIME_SNAPSHOT
    OWNER_EVALUATE
    TUBE_BUILD_PRECONDITION
    TUBE_RAW_BUILD
    TUBE_FILTER
    TUBE_SURFACE_VALIDATOR
    TUBE_COVERAGE_OR_VALIDATOR
    STAGING_DRY_RUN

`stagePathTubePair()` already computes it, but the production H2 call omits the
output pointer.  In `stageFutureSeamPathTubeTransaction()`:

- create one local `PathTubePairStageFailure`, initialized to `NONE`;
- pass its address to the existing `stagePathTubePair()` call;
- return behavior remains exactly unchanged;
- expose that value to the two existing H2 lifecycle log sites, either by one
  optional output parameter or one concise log at the existing stage boundary;
- log Pair generation/session, captured w, seam w, and the existing enum name;
- do not store it in manager state, a mailbox, diagnostics schema, or a retry
  decision.

Early failures before `stagePathTubePair()` retain `NONE` and remain classified
by their existing C2/structural log.  Do not invent a second failure enum.

### 4.2 Dry-run detail

Only when the existing enum is `STAGING_DRY_RUN`, emit one concise event from
the existing `stagePathTubePair()` failure branch using facts already present
in `RuntimeDryRunResult`:

- replacement/bootstrap identity;
- `step.valid`, `step.selected`, `step.projection.valid`;
- Runtime execution mode, certificate-denied and fatal flags;
- invalid reason;
- retained delta and previous final port captured for this transaction;
- current/seam/owner end.

Do not add a test hook, counter, latch, throttle state, schema field, retry, or
new Runtime/Projector query.  Never infer that witness failure proves the full
viable set empty.

### 4.3 Phase B tests

Use existing fixtures and add only focused assertions needed to prove:

- a known `STAGING_DRY_RUN` replacement returns the same false result and enum;
- Tube raw/filter/validator failures retain their existing enum;
- successful staging remains `NONE`;
- caller output/front-end preservation, pending mailbox, Pair pointer, Runtime
  state, retained delta, and previous port are unchanged by attribution;
- later callback behavior is unchanged; no synchronous retry appears;
- log formatting/name mapping covers every existing enum without changing
  decisions.

## 5. Allowed files

Product files, only as necessary:

- `phase_offset_matched_adapter.cpp`;
- `gvf_manager.cpp` and its header only if an optional local output parameter
  is required;
- existing adapter marker and switch-policy tests;
- this execution spec, a G18 self-audit, and `/home/cxq/ISF-GVF/handoff.md`.

Read-only:

- TubeBuilder, TubeFilter, CertifiedTubeBuilder, SurfaceValidator;
- Runtime and PortProjector;
- launch, maps, parameters, CMake and ROS messages;
- planner/governor/recovery/neutral behavior except the single attribution
  value flowing through the existing H2 call boundary.

## 6. Static verification

Run and preserve exact logs under `/dev/shm/tube_v2_g18_static_<id>/` because
the root filesystem is full:

1. marker focused tests, including dual-owner Pair/timer composition;
2. adapter full suite;
3. GVF switch policy and H2 lifecycle tests;
4. Epoch, Integration, Runtime and G17 CertifiedBuilder regressions;
5. full incremental `catkin_make -j2` without cleaning;
6. `git diff --check` and allowed-file audit;
7. frozen Builder/Filter/Validator/Runtime/Projector/launch/parameter hashes;
8. forbidden-symbol audit for new gate/cache/retry/timer/module/state/recovery.

The pre-existing SurfaceValidator fixture remains separately classified and
must not be changed.

## 7. Bounded dynamic verification

After static and independent read-only review, reuse the exact G17 private
split-launch plan and unchanged parameters.  Use `/dev/shm`, a fresh port,
recorder-before-publishers, and the state-driven G1--G5 harness.  Do not run a
new dynamic test merely to prove unit logging; the run must also verify Phase A.

Required outcomes:

- when a Pair is active and `tubeDisplayCertified()` is true, Certified emits
  genuine narrow ADD geometry;
- Candidate simultaneously remains broad raw `I_geo`;
- stale/denied periods emit Certified DELETE;
- manual and epoch diagnostics are present;
- H2 `stage_success=0` includes the exact existing stage-failure enum;
- if it is `STAGING_DRY_RUN`, the existing dry-run facts are present;
- G1--G5 results are reported honestly.

G18 may be marked `VISIBILITY_FIXED` if Phase A passes even if navigation still
times out.  It may not be marked system `FIXED` unless all five goals pass.
The exact P0 failure layer becomes the input to the next primary-agent plan;
G18 itself must not implement recovery.
