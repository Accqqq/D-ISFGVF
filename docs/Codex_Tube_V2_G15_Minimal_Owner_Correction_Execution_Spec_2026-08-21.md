# Tube V2 G15 reviewed authority, visibility, and neutral-liveness correction spec

    DOCUMENT_ROLE=PRODUCT_CORRECTION_EXECUTION_SPEC
    DOCUMENT_STATUS=PARTIALLY_FIXED_DYNAMIC_CONSTRUCTION_BLOCKED
    STAGE=TUBE_V2_G15_FINAL_RECORDED
    REVIEW_DATE=2026-08-22
    IMPLEMENTATION_AUTHORIZED=true
    EXECUTOR=LUNA_MAX_SINGLE_AGENT
    SUBAGENT_CREATION_ALLOWED=false
    DELEGATION_ALLOWED=false
    PARALLEL_AGENT_WORK_ALLOWED=false
    AUTO_ADVANCE=A_TO_B_TO_C_TO_STATIC_TO_DYNAMIC
    NO_NEW_GATE_CACHE_RETRY_TIMER_WATCHDOG_MODULE=true
    NO_LIMIT_OR_SAFETY_MARGIN_CHANGE=true

## 1. Outcome and priority

G15 fixes three separate owners in this order:

1. A -- make the real current Tube build visible on the existing UNCERTIFIED
   candidate topic without weakening certified semantics.
2. B -- remove duplicate exact evaluations of identical owner knots during H2
   staging while preserving structural-seam, Validator, Runtime, and live
   commit checks.
3. C -- restore the original navigation contract: Tube authorizes nonzero
   offset, but a failed Tube must not permanently prevent installation of an
   accepted neutral planner frontend.

The main user-visible acceptance is not merely that tests pass. With the two
launches started separately, the candidate Tube must be observable while it is
being built, a genuinely certified Tube must remain distinct, and a real Tube
failure must not leave normal navigation permanently stuck at the end of an
old path.

G14 evidence is authoritative for the before state:

- /tmp/tube_v2_g14_owner_audit_212745/tube_owner_classification.tsv
- /tmp/tube_v2_g14_owner_audit_212745/tube_validator_attempts.tsv
- /tmp/tube_v2_g14_owner_audit_212745/h2_first_false.tsv
- /tmp/tube_v2_g14_owner_audit_212745/h2_user_timeline.tsv
- /tmp/tube_v2_g14_owner_audit_212745/causal_classification.md
- /tmp/tube_v2_g14_owner_audit_212745/private_run/evidence/g14_targeted.bag

Known facts which must not be rewritten:

- certified /tube had twelve message arrays and every marker was DELETE;
- candidate /tube_candidate had real ADD geometry followed by DELETE;
- candidate 6 had 101 inward attempts ending in real insufficient-clearance
  rejection;
- candidate 9 reached the existing query terminal;
- H2 stage failure retained the old frontend until 36/0/36 path-end HOLD;
- G14 only localized the H2 adapter failure to OWNER_EVALUATE / the
  tube_build_owner_evaluate layer. It did not prove which inner call failed in
  the user session.

## 2. Original proposal contracts which remain frozen

G15 must preserve the T0--G1 proposal architecture:

- TubeFilter remains cell-local conservative PWL;
- filtered knot bounds remain the raw knot bounds;
- no fixed-slope global envelope, anchored range enumeration, remote
  bottleneck back-propagation, or Filter REGULARITY truncation returns;
- the existing SurfaceValidator remains the continuous geometric safety
  authority;
- the existing Runtime remains the dynamic-viability authority;
- normal Tube remains connected to delta=0 and cannot cross a disconnected
  branch;
- planner and Tube continue to share planner-authoritative clearance;
- Tube grants nonzero-offset authority and is not a neutral-navigation gate;
- a nonzero Path+Tube handoff remains atomic and fail-closed.

Do not change TubeFilter, Validator limits, clearance, tracking bounds,
regularity, map handling, Runtime constraints, launch defaults, or user launch
parameters in G15.

## 3. Mandatory preflight: remove the interrupted half patch only

Luna was interrupted after partially editing the adapter source/header and
before a complete build or tests. Do not continue layering changes onto that
half patch.

1. Record current hashes and exact snippets for the adapter cpp/header,
   gvf_manager.cpp, marker cpp/header, and focused tests.
2. Use an exact inverse apply-patch for changes made by Luna in the interrupted
   G15 turn only. Do not use git reset, checkout, restore, clean, stash, rebase,
   or replacement from another branch.
3. The adapter cpp must return to the G14 frozen hash:
   4f324cb9f83fc7d60fbea9bc32338244cfcc00292660165d9ac6f930ca93442a
4. Restore the header signature paired with that G14 adapter implementation.
   Remove only the interrupted canonical-current/reuse-supplied additions;
   preserve all earlier user/A6/G1/G2 assets.
5. Compile the focused adapter target after the inverse patch. If the frozen
   cpp hash cannot be recovered from Luna's own edit record, stop editing and
   report the exact mismatch; never reconstruct the file from memory or erase
   unrelated user work.
6. Store preflight hashes, inverse-diff description, and compile result in a
   new /tmp/tube_v2_g15_<id>/ directory.

The G15 execution spec itself is retained; only the interrupted product patch
is inverted.

## 4. Phase A -- marker and candidate-geometry ownership

### 4.1 Files and functions

Inspect and minimally modify only what is required among:

- phase_offset_matched_adapter.cpp:
  publishManual, publishManualDelete, buildMarkers, shutdown/inactive lifecycle;
- phase_offset_tube_markers.cpp/.h:
  MakeCandidateTubeMarkers, MakeCertifiedTubeMarkers, AppendBoundaryPoints;
- existing marker and adapter tests.

No new topic, ROS message, publisher, cache, timer, or visualization module is
allowed. Keep publishers non-latched and keep the existing namespaces, ids,
colors, and explicit UNCERTIFIED/Certified RViz displays.

### 4.2 Required four-way publication semantics

Classify using the existing immutable task/source/control/map/build/epoch
identity. This corrects current ownership; it is not a new runtime gate.

1. Truly stale identity:
   - an older task, source, control, map, build, or epoch callback has no marker
     ownership;
   - return without publishing ADD or DELETE;
   - it must never erase a newer epoch.
2. Current identity with a real displayable candidate but no certification:
   - publish candidate ADD on /tube_candidate;
   - publish certified DELETE on /tube;
   - never copy candidate geometry into the certified topic.
3. Current authoritative identity explicitly containing no displayable
   candidate:
   - publish candidate DELETE and certified DELETE once through the existing
     current lifecycle owner.
4. Current identity with internally inconsistent epoch/profile/status pointers:
   - do not treat this as ordinary stale no-op;
   - certified must fail closed to DELETE;
   - candidate is decided only from the current authoritative epoch, never from
     a mismatched callback payload;
   - do not leave a certified ghost marker.

Retain the existing inactive/manual-exit deletion. Add shutdown deletion only
through the existing shutdown lifecycle before publisher teardown; do not add
a timer or persistent marker store. A new task may delete the preceding task's
markers only when the new current lifecycle owner takes authority.

### 4.3 What candidate geometry means

The final candidate samples may collapse to zero-only after all nonzero
attempts fail. That object describes the final safe fallback, not the nonzero
geometry the user wants to observe during construction.

The candidate topic must use the same authoritative epoch's existing
pre-certification build geometry when it is complete enough to draw:

- prefer existing raw_build_samples or the equivalent already-owned
  pre-certification Builder/Filter samples;
- include only the contiguous range containing the exact current anchor;
- reject incomplete, nonfinite, unordered, duplicate, or remote-gap samples
  for display;
- do not run a second safety validator in the marker layer;
- label and color remain UNCERTIFIED;
- this geometry has no control, Runtime, Pair, or certification authority.

If the current epoch has no displayable pre-certification geometry, the current
owner publishes candidate DELETE. Do not fabricate width, interpolate across a
gap, or display geometry from an older epoch.

### 4.4 Phase A tests

Add focused deterministic tests before advancing:

- newer candidate ADD followed by older callback: older callback is no-op;
- current complete pre-certification candidate: candidate ADD and certified
  DELETE;
- failed nonzero certification with final zero-only profile: real current
  pre-certification candidate remains visible only on candidate topic;
- current epoch without displayable candidate: both DELETE;
- current internally inconsistent status: no certified ghost and no stale
  candidate substitution;
- genuine certified active profile: certified ADD;
- manual exit, new task ownership, and shutdown: correct namespace deletion;
- identical marker ids on separate topics cannot cross-delete.

Run marker tests and focused adapter tests. Do not advance with a failed test.

## 5. Phase B -- exact owner-evaluation reuse without deleting safety proofs

### 5.1 Scope

Inspect manager stageFutureSeamPathTubeTransaction, adapter stagePathTubePair,
buildPreparedTubeEpoch, BuildOwnerAlignedTubePreview, EvaluateOwnerState,
PreparedSamplesMatchOwner, and structural seam/cellBounds/Builder/Validator/
Runtime/commit call sites.

Do not assume every owner query is duplicate. The following are distinct:

- exact current state at captured_w0;
- planner trial samples at trial_w;
- immutable structural seam partition;
- prepared preview over the required horizon;
- Validator subdivision/future-phase queries;
- Runtime future-step queries;
- commit-time live-state validation.

### 5.2 Permitted reuse

Within one immutable transaction, reuse a state only when all are exact:

- the same semantic owner shared pointer/identity;
- the same task, source revision, map observation, session, and transaction;
- the exact same w bit pattern, not a nearest knot;
- the state passes existing finite/state-equivalence and owner-membership checks.

Build the structural partition once. Reuse exact current and identical trial
knots. Evaluate newly inserted structural seam/range knots exactly once.
Prepared preview consumes that partition/sample set instead of rebuilding and
rewalking all ordinary knots.

### 5.3 Checks that must remain

Do not remove or reuse away:

- owner membership proof at the adapter boundary;
- exact structural seam preservation;
- cellBounds owner-segment certificate;
- SurfaceValidator subdivision/path queries;
- profile owner postcheck;
- Runtime dry-run and future queries;
- atomic Pair generation/session/epoch matching;
- commit-time live current state check.

An owner-evaluation failure remains OWNER_EVALUATE and commits nothing. A
Validator or Runtime failure retains its existing enum and fail-closed result.
Do not retry.

### 5.4 Phase B tests

Use a test-only evaluator counter or existing fake owner; add no production
counter or ROS diagnostic.

- identical current/trial knot is evaluated once per immutable transaction;
- structural seam is preserved and evaluated when near an ordinary knot;
- fabricated canonical state not belonging to owner is rejected;
- owner pointer, task, revision, session, or exact-w mismatch forbids reuse;
- OWNER_EVALUATE failure produces zero commit;
- success produces exactly one atomic Pair with one owner interval;
- Validator and Runtime failures retain original classifications;
- no TubeFilter, margin, query/depth, or Runtime predicate changes.

Run focused adapter, structural-seam, Builder, Epoch, Runtime, and manager tests
before Phase C.

## 6. Phase C -- restore neutral planner liveness

### 6.1 Required control responsibility

Inspect both point-phase and closed-loop accepted-replan branches around
stageFutureSeamPathTubeTransaction, frontend_ready/installed,
commitNeutralPlannerFrontend, the existing neutral-retirement predicate,
H2/return-to-center continuation, and governor path-end HOLD.

The accepted planner payload belongs to the current replan callback. Preserve
it locally through Pair staging; this is not a cache.

Required behavior:

1. If current executed offset authority is neutral and the existing
   neutral-retirement predicate is satisfied, Tube/Pair stage failure rejects
   offset authority but must not reject the accepted planner frontend. Install
   it through existing commitNeutralPlannerFrontend.
2. If nonzero authority is active, do not directly switch to a neutral path.
   Continue fail-closed. Existing H2/return-to-center/periodic replan flow must
   complete a safe Pair or existing safe neutral retirement before the old
   executable horizon is exhausted.
3. Tube failure is never rewritten as Tube success. It rejects Pair and
   nonzero offset authority. Only the independently accepted neutral planner
   payload may install under the existing neutral predicate.
4. Governor HOLD remains the final safety consequence when neither safe
   continuation exists. It is not normal fallback or acceptance success.

Do not add a fallback flag, new gate, retry, timer, watchdog, deferred queue,
mailbox, or cached frontend. Correct the existing installed/keep-current branch
and reuse the existing neutral commit path.

### 6.2 Phase C tests

- neutral authority plus Tube failure: accepted planner frontend installs;
- nonzero authority plus Tube failure: unsafe direct neutral switch is refused;
- after existing return-to-zero becomes true, next accepted replan installs via
  the existing neutral path;
- successful H2 Pair still installs atomically and retains nonzero authority;
- failed Pair never appears COMMITTED;
- old frontend is not retained to path end when safe neutral accepted frontend
  is already available;
- point-phase and closed-loop branches have equivalent liveness behavior;
- path-end HOLD remains possible only when neither safe continuation exists.

## 7. Builder/Validator complexity boundary for G15

G15 measures but does not hide the known 101 inward attempts and full-width
query terminal. Phase A makes the real attempt visible; Phase C prevents real
Tube failures from permanently blocking neutral navigation.

Do not redesign SurfaceValidator or increase limits in the same A/B/C patch.
After static tests pass, record full-width/inward attempt and query counts,
whether identical failure proof repeats, certified/nonzero success rate, and
candidate-display duration.

If the user scenario still has low certified construction because of the
101-attempt strategy, report it as the next isolated simplification stage. Do
not claim G15 fixed construction success merely because candidate geometry is
visible or navigation remains live.

## 8. Static verification order

Save commands/results under the G15 evidence directory:

1. marker tests;
2. phase_offset_matched_adapter tests;
3. manager switch-policy tests;
4. continuous-phase/path tests;
5. TubeFilter, TubeBuilder, SurfaceValidator, CertifiedBuilder, Epoch, Runtime;
6. repository-required focused CTest set;
7. catkin_make -j2;
8. git diff --check;
9. source and binary hashes.

Static audit must prove no new gate/cache/retry/timer/watchdog/module, no limit
or attempt-count increase, no safety/default/launch change, no candidate to
certified promotion, and no unrelated dirty-tree asset changed.

## 9. Independent split-launch dynamic acceptance

Use a fresh private ROS master, fresh ROS_HOME, isolated evidence directory,
and recorder started before publishers. Do not attach to or signal the user's
existing ROS processes. Record PID/starttime/command/port and clean only
task-owned processes.

Launch separately:

    roslaunch so3_quadrotor_simulator simulator.launch

    roslaunch bspline_race test_gvf.launch \
      phase_offset_mode:=manual \
      phase_offset_manual_amplitude:=0.10 \
      phase_offset_manual_observe_only:=false \
      phase_offset_manual_profile_period:=2.0 \
      phase_offset_manual_tube_source:=esdf \
      phase_offset_tube_fixed_delta_max:=0.04 \
      phase_offset_tube_cloud_obstacle_set_complete:=true \
      phase_offset_tube_preincluded_map_uncertainty:=0.10

Use the known five goals in order. G1--G4 wait for REACHED. G5 waits for
REACHED or bounded evidence-complete terminal classification; do not build a
new harness framework.

Dynamic checks:

- each authoritative pre-certification build produces candidate ADD or
  explicit current DELETE;
- old epoch never sends later cross-epoch DELETE;
- certified ADD occurs iff active profile is genuinely display certified;
- candidate 6/9 real failures remain fail-closed and retain categories;
- at least one valid nonzero Pair still commits and executes when available;
- neutral plus Tube failure installs accepted neutral planner frontend;
- nonzero plus Tube failure does not perform unsafe direct switch;
- the same stage_success=0 -> keep old -> 36/0/36 stable HOLD chain must not
  recur when safe neutral continuation was available;
- G5 reaches, or remaining failure is directly classified and not called fixed.

Bag acceptance:

- /tube_candidate has ADD geometry during actual build attempts;
- /tube has ADD only with active_display_certified=1;
- manual exit/shutdown produces DELETE for both namespaces;
- no ghost certified marker and no stale cross-epoch deletion.

## 10. Completion and reporting

Update this spec, G15 self-audit, and /home/cxq/ISF-GVF/handoff.md with exact
files/functions, deleted/merged responsibilities, tests/builds, five-goal
result, marker counts, Pair/H2/neutral-install timeline, remaining 101-attempt
facts, cleanup proof, and explicit FIXED/PARTIALLY_FIXED/NOT_FIXED status.

Do not write FIXED unless visibility ownership and navigation liveness both pass
the user split-launch scenario. Low certified construction success, if still
present, remains separately disclosed.

## 11. Absolute prohibitions

- no subagents or delegation;
- no git reset/clean/checkout/restore/stash/rebase;
- no new gate/cache/retry/timer/watchdog/module/state-machine layer;
- no query/depth/attempt-limit increase;
- no clearance/tracking/map/safety-margin relaxation;
- no launch/default tuning;
- no uncertified geometry on certified topic;
- no replacing exact owner/Validator/Runtime queries with interpolation;
- no treating Tube failure or HOLD as success;
- no touching external ROS processes.

## 12. Final G15 execution record (2026-08-22)

The implementation and static verification are complete.  The independent
split-launch run completed its bounded dynamic portion, but it did not satisfy
the full five-goal acceptance because construction/liveness blocked at G2.
The final status is:

```text
PARTIALLY_FIXED_DYNAMIC_CONSTRUCTION_BLOCKED
```

### 12.1 Static result

The following focused results are recorded under
`/tmp/tube_v2_g15_static_20260822/`:

- marker 13/13;
- matched adapter 83/83;
- GVF switch 49/49;
- continuous path 11/11;
- TubeBuilder 9/9;
- CertifiedTubeBuilder 11/11;
- SurfaceValidator 22/22;
- TubeEpochManager 53/53;
- Runtime 35/35;
- full `catkin_make -j2` succeeded (`catkin_make_full.log`).

The epoch-integration executable remains 6/10 because four legacy cycle-99
fixed-source no-Pair assertions are incompatible with the current neutral/
Pair contract.  This is recorded in `epoch_integration.log` and is not counted
as a G15 construction success.

### 12.2 Dynamic run identity and readiness

The run directory is:

```text
/tmp/tube_v2_g15_dynamic_20260822_021000/
```

It used a fresh ROS master on port `13028`, fresh ROS_HOME/ROS_LOG_DIR, and a
recorder started before either publisher launch.  `run_identity.txt` records
the task PIDs/start times and cleanup sequence.  Readiness passed with:

```text
/sim/local_map                         900
/mock_map                              300
/particle0sdf_map/occupancy            899
/particle0sdf_map/esdf                 899
/particle0sdf_map/update_range         899
```

The bag is
`/tmp/tube_v2_g15_dynamic_20260822_021000/evidence/g15_dynamic.bag`;
`bag_info.txt` reports 69,245 messages, 7.4 MB, and 343 seconds.

### 12.3 Marker ownership and geometry

`marker_summary.tsv` contains 2,427 messages for each namespace:

- `/tube_candidate`: 7,206 ADD markers, 75 DELETE messages, no OTHER action;
  ADD geometry is finite, with 42 points in each recorded candidate marker
  batch.  The first candidate ADD is at `1787334667.0777786`, and candidate
  ADD geometry is present during the real build attempts.
- `/tube`: 0 ADD markers and 7,281 DELETE markers.  Certified geometry was
  never published, and no candidate geometry was copied to the certified
  namespace.

The last candidate ADD is at `1787334907.0799785`; the bag does not independently
show a subsequent shutdown DELETE for that final candidate ADD.  Therefore the
shutdown-marker dynamic proof is not claimed, although the lifecycle delete
path and focused shutdown tests pass.

### 12.4 Pair/H2 and builder timeline

The derived timeline records:

```text
1787334664.685759783  bootstrap Tube-build precondition failure, session=10
1787334665.352633715  Pair generation=1/session=10 committed
1787334665.388500452  nonzero delta selected (retained delta=-0.000384455)
1787334666.212331772  H2 point replan stage_success=0, generation=3/session=10
1787334666.511911631  exact-port witness/projector invalid; joint port polygon empty
1787334666.912158251  governor HOLD, 36/0/36 path-end-clamped
```

The runtime nonzero delta executed before the later H2 failure.  The existing
frontend was retained after `stage_success=0`; the callback-local neutral
frontend path was not dynamically exercised because the run entered the
construction/H2 blocker before the later goals.

### 12.5 Z1/Z2 construction facts

`derived/parser_summary.tsv` reports 2,409 zero classifications: Z1=2 and
Z2=2,407.  Z1's first full-width terminal is
`queries=249993`, `max_queries=250000`.  Z2 attempts commonly report
`attempts=101`, `last_reason=insufficient_clearance` (the first five are
explicitly listed in `derived/zero_baseline_timeline.tsv`).  These are
unchanged Builder/Validator limits and are reported as the remaining
construction blocker, not as a safety relaxation.

### 12.6 Goal result and cleanup

The harness log records:

```text
G1 (7.381, 0.378, 0) reached; distance=0.183
G2 (-6.823, -0.685, 0) published/received; bounded wait timed out
```

G3--G5 were not sent because the harness correctly waits for REACHED before
advancing.  The private port was free after teardown, task-owned processes
were gone from the after-scan, and before/after binary and launch/map hashes
match.  Existing external ROS processes were left untouched.

The dynamic result supports the visibility-ownership correction and the
fail-closed certified namespace.  It does not support a claim that neutral
liveness passed, that G5 was exercised, or that G15 is `FIXED`.
