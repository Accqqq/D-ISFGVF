# A6-P1 Owner-Aligned Current-Phase Preview Correction — Self Audit

Date: 2026-08-20
Workspace: `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`
Scope: A6-P1 only.  This audit does not authorize bootstrap observability,
M6 recenter, M4 authority, M7 removal, Tube geometry, Runtime, planner/C2,
manager, schema, parameter, launch, or thread changes.

## Result

`PASS_WITH_ACCURATE_ACTIVE_STOP_BOUNDARY`.

The owner-aligned preview now carries the original current global phase into
both production construction paths.  The A6-P1 isolated ESDF-active capture
shows that all captured raw candidates retain a complete current anchor; it
does show active `TubeEpoch` profile installs, but it shows no selected offset
control authority.  It has no direct PathTubePair authority/first-false
lifecycle field, so it cannot certify Pair activation or H2 retry/consume
beyond the facts listed below.

## Authorized code and tests

Only the two A6-P1 implementation/test source files were edited:

- `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
- `src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp`

`BuildOwnerAlignedTubePreview()` now accepts
`required_current_w`, fail-closes on invalid owner/range/current inputs, first
evaluates the immutable owner at that exact double, and explicitly inserts
that double into the sorted partition.  It removes all knots in its existing
current-anchor equivalence class and reinserts the original current double as
the sole canonical representative.  Every output state is still constructed
by `owner->evaluate(..., false)`; no sampled interpolation is introduced.

The ordinary timer call explicitly supplies `request->current_path.w`; the
prepared/PathTubePair call explicitly supplies `captured_w0`.  Ordinary timer
retains its existing `makePreview()` fallback only if the owner-aligned helper
truly fails; prepared construction remains fail-closed.

During independent review a boundary defect was found and corrected: the old
range helper normalized any near-boundary input to `start_w`/`end_w`.  The
required current phase now bypasses that endpoint normalization.  A current
within `kPreparedCoverageTolerance` of a preview boundary therefore remains
its original global double, while the endpoint-equivalence coverage check
still permits that canonical current knot to represent the roundoff-equivalent
range boundary.  No anchor tolerance, physical phase, safety margin, or
certificate query was changed.

Added/retained relevant test coverage:

- three ordinary off-grid current phases (`0.435`, `0.783`, `1.127`) produce
  complete raw/filter profiles with a unique exact current anchor;
- near supplied knot and owner-seam canonicalization remains unique and
  strictly ordered;
- new near-preview-start test uses `current_w=0.40000000005` and
  `back_w=5e-11`, proving `preview_start_w` and the sole anchor retain the
  exact current double rather than snapping to `0.4`;
- prepared PathTubePair construction omits the captured off-grid phase from
  supplied samples yet yields a complete profile with the captured start
  anchor;
- existing range/provenance/owner mismatch, dry-run, CAS and pair regressions
  are retained in the full adapter suite.

## Build and regression evidence

Commands executed successfully:

```text
catkin_make -j2 phase_offset_matched_adapter_test
./devel/lib/bspline_race/phase_offset_matched_adapter_test
ctest --output-on-failure -R '(_ctest_phase_offset_navigation_gtest_phase_offset_tube_(cross_section|builder|filter|surface_validator)_test|_ctest_phase_offset_navigation_gtest_phase_offset_(certified_tube_builder|runtime|tube_epoch_manager)_test|_ctest_bspline_race_gtest_(gvf_switch_policy|continuous_phase_path)_test)$'
catkin_make -j2
```

Results:

- `phase_offset_matched_adapter_test`: 79/79 passed, including the new
  boundary regression and the ordinary/prepared A6-P1 tests.
- CrossSection/Builder/Filter/SurfaceValidator regression set: all passed.
- `phase_offset_tube_cross_section_test`, `phase_offset_tube_builder_test`,
  `phase_offset_tube_filter_test`, `phase_offset_tube_surface_validator_test`,
  `phase_offset_certified_tube_builder_test`,
  `phase_offset_tube_epoch_manager_test`, `phase_offset_runtime_test`,
  `gvf_switch_policy_test`, and `continuous_phase_path_test`: all passed
  (9/9 selected CTest entries, 0 failures).
- Full `catkin_make -j2`: passed, including `formation_planning`.

## Static and scope checks

Commands executed:

```text
rg -n -C 2 'BuildOwnerAlignedTubePreview\\(' src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
rg -n 'std::(round|floor|ceil)|\\bround_to|snap_to|snap' src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
rg -n 'TubeBuilder|CertifiedTubeBuilder|TubeFilter|TubeSurfaceValidator|TubeEpochManager' src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
git diff --check
git diff --no-index --check /dev/null <each of the two untracked A6-P1 source files>
git status --short
```

Findings:

- the only two production helper calls are at the ordinary timer and prepared
  paths, and both pass the explicit current phase;
- no executable rounding or snapping operation was added (the sole `snapped`
  match is explanatory prose);
- this adapter constructs candidates only through `TubeEpochManager::update`;
  it does not instantiate a legacy `TubeBuilder`/`CertifiedTubeBuilder` path;
  `TubeFilter::query` remains an existing read-only profile query;
- no Tube/Runtime/manager/launch/parameter/header source was edited for A6-P1;
- `git diff --check` passed.  The two authorized implementation/test files are
  pre-existing untracked prototype-stage files in this dirty worktree, so the
  additional `--no-index --check` verified each has no whitespace error
  (its exit 1 is the expected nonempty untracked diff, with no diagnostic);
- `git status --short` still lists extensive user-owned historical tracked and
  untracked changes.  They were not altered by this stage.  The A6-P1 source
  changes are confined to the two files above; this audit file and the
  repository-external handoff append are the only documentation updates.

## A6-P1 isolated ESDF-active evidence

Observe-only evidence is inherited, not re-executed: the immediately prior
M3-L1 handoff records that the fixed ESDF observe-only launch reached its
goal.  Separately, A6-P1 itself launched the isolated ESDF-active capture
below.  The active evidence is bounded to the six files produced by that
A6-P1 run.

Parsed the A6-P1 capture directory:

```text
/tmp/a6_p1_20260820_esdf_active_1/
  raw_stream.txt epoch_stream.txt manual_stream.txt cloud_stream.txt
  launch.log final_odom.txt
```

Schema facts used:

- raw schema v1: candidate sequence/state/reason, raw/filter/complete,
  sample count/invalid count, `current_w`, `current_sample_found`, and
  `current_sample_complete`;
- epoch schema v3: state/disposition/reason, candidate/active completeness,
  current validation, Runtime mode, control-selected, certificate-denied,
  transient-blocked and fatal-invariant;
- manual schema v1: warmup gate, selected/valid, active profile facts;
- cloud schema v1: immutable snapshot usability and categorical status.

Observed facts:

- 37 raw messages; all 37 have raw/filter/complete=`1/1/1` and
  current-found/current-complete=`1/1`.  Current `w` advances continuously
  from `0.138228531794946` through `7.900168144940532`; this is direct
  evidence that the prior between-knot missing-anchor failure is absent in
  this capture.
- 35 raw entries are `ROLLING/NONE`; the other two are
  `WAITING_FOR_CANDIDATE/FORWARD_HORIZON_SHORT` (reason 14), but are still
  raw/filter/complete=`1/1/1` with complete current anchors.  There is no
  `CANDIDATE_INCOMPLETE` observation.
- 25 epoch messages: all candidates are complete; 23 have active profiles,
  active-current-validation and admissible current state.  Runtime is
  `NORMAL` 22 times, `WAITING_FOR_CANDIDATE` twice, and
  `CERTIFICATE_DENIED` once; it is never a fatal invariant.  The epoch
  install count advances from 1 to 24.  The single certificate denial is not
  a raw-anchor incompleteness signal.  `retained_delta=0` in all 25 messages
  and epoch `control_selected=0` in all 25.
- cloud diagnostics: all 37 immutable snapshots are usable; base/actual/plus/
  minus categorical statuses are `KNOWN_FREE` (enum 3); raw storage and
  self-free-seed indicators remain zero.
- manual diagnostics: the 100-cycle warmup gate opens in 11 captured messages;
  `valid=1` and `profile_active=1` each occur 22 times (both are zero in the
  other 3), but `control_selected=0` and `delta=0` in all 25.  Therefore the
  stream proves active epoch profiles, not selected offset control authority.

The present schemas/log have no Pair pointer/authority flag, bootstrap/stage/
dry-run/CAS first-false, or H2 pending/retry/commit/consume event.  Planner
C2 logs show accepted connector switches at phases `3.685`, `5.804`, and
`7.297`, followed by a connector failure at `8.294`; accepted C2 switches are
not PathTubePair proof.  Consequently the exact next first-false and the H2
single-seam retry/consume outcome are unobservable in this authorized capture.
No diagnostic, manager, or launch modification was made to guess at it.

## Planner and termination facts from the same capture

`launch.log` records the requested goal `(8,0,1)` and
`[GVF][POINT_GOAL][REACHED] distance=0.197`.  Near the end it records two
commands with `path_end_clamped_count=2` but still `valid_count=3`; there is
no `all_candidates_path_end_clamped` log.  The only
`GOVERNOR_INVALID_HOLD` is the expected pre-initialization
`guidance_invalid` record before the initial trajectory exists; subsequent
logged commands use `VEL_MATCH_GOVERNOR` with `fallback_reason=none`.

`final_odom.txt`, captured later, reports approximately `(8.0, 0.0, 2.0)`.
This agrees with the existing `gvf_manager::goalCallback()` message mapping:
the logged goal message `(8,0,1)` becomes the internal
`goal_pt.z = msg.z + 1.0`, i.e. `(8,0,2)`.  Thus the terminal odometry is
consistent with the internal goal; no unexplained one-metre test difference
is asserted.

## Stop boundary

A6-P1 is complete: preview-current plumbing, its boundary contract, unit and
regression coverage, full build, static checks, and the A6-P1 dynamic
evidence are recorded.  The remaining active-control/Pair/H2 lifecycle fact
is not identified by these streams.  Per the execution specification, stop
here; do not add observability, change manager/Runtime/Tube/launch/parameters,
or begin a later authority/recenter stage.
