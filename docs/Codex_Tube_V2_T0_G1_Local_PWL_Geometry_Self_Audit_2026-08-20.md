# Tube V2 T0--G1 Local PWL Geometry Self-Audit

Date: 2026-08-20  
Workspace: `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
Stage: `TUBE_V2_T0_G1`  
Status: `G1_IMPLEMENTATION_COMPLETE_DYNAMIC_ACCEPTANCE_UNMET`

## Result and stop decision

The authorized G1 local-PWL implementation, focused tests, required CTest
regressions, static audit, and full `catkin_make -j2` completed successfully.
The two isolated after launches reached the internal goal and produced complete
raw current anchors, but neither observed the required production Pair
`COMMITTED -> selected -> retained nonzero delta` lifecycle.  Per execution
spec §8, this is recorded as:

```text
DYNAMIC_ACCEPTANCE_UNMET / OFFSET_AUTHORITY_NOT_OBSERVED_AFTER_G1
```

No third launch, gate, timer, parameter, Runtime, H2, planner, or viability
change was made.  Work stops at G1.

## Exact files changed by this stage

Only these whitelist files were edited:

```text
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_filter_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/certified_tube_builder_test.cpp
docs/Codex_Tube_V2_T0_G1_Local_PWL_Geometry_Self_Audit_2026-08-20.md
/home/cxq/ISF-GVF/handoff.md
```

The worktree is intentionally user-owned and highly dirty.  Existing tracked
and untracked files were preserved.  No Builder, SurfaceValidator,
CertifiedTubeBuilder, EpochManager, Runtime, Projector/QP, planner, launch,
parameter, CMake, package, ROS schema, or proposal production file was edited
by G1.  The `boundary_slope_max` field and launch/ROS parameter remain for
compatibility.

## Before/after algorithm

Before, production `TubeFilter` used `boundary_slope_max=0.80` as a geometric
decision: a global exact Lipschitz forward/backward inner envelope, anchored
range enumeration, slope-cap assertion, and Filter-created `REGULARITY`
truncation.

After, for a complete strictly ordered raw profile:

```text
filtered_lower[k] = raw_lower[k]
filtered_upper[k] = raw_upper[k]
lower_w[k] = (raw_lower[k+1] - raw_lower[k]) / (w[k+1] - w[k])
upper_w[k] = (raw_upper[k+1] - raw_upper[k]) / (w[k+1] - w[k])
```

Each cell is the local affine interpolation between its raw endpoints.  An
interior query returns the right-cell one-sided derivatives; the final knot
returns the left-cell derivative.  Filter geometry, success, preview domain,
bounds, slopes, and classification do not depend on `boundary_slope_max`.
The Filter does not write truncation provenance or `REGULARITY`; pre-existing
Builder/Validator provenance is preserved.  Invalid/nonfinite/empty,
unordered, incomplete, missing-current-anchor, and single-knot inputs still
fail closed.

## T0 current before baseline

Evidence directory: `/tmp/tube_v2_t0_20260820_225509/`

```text
HEAD=9a0e97560b8ddf15ac1ed1581030ed4b911f6e43
private master=127.0.0.1:12885
fresh ROS_HOME=/tmp/tube_v2_t0_20260820_225509/ros_home
launch=phase_offset_esdf_tube_single.launch
map=pillar.pcd
observe_only=false
goal message=(8,0,1), internal target=(8,0,2)
bag=evidence/t0_acceptance.bag, 198 s, 20,672 messages
raw cohorts=40
raw current anchor found=40/40
raw current anchor complete=40/40
raw sample count=7..49 (median 45)
```

The current raw diagnostics expose current-knot fields (`c_plus_raw`,
`c_minus_raw`, and current bounds) but not adjacent knot bounds.  Therefore
the baseline explicitly records:

```text
CURRENT_SCHEMA_CANNOT_RECONSTRUCT_RAW_ADJACENT_SLOPES
```

No ROS schema was expanded.  Static T0 facts froze the old fixed-slope Filter,
current-connected SurfaceValidator, unique CertifiedTubeBuilder pipeline,
planner `safe_distance` provenance, and Runtime endpoint-based local-slope
reconstruction.  The same T0 run reached the goal (`distance=0.189`) and its
rosout contains the prior A6-P2 lifecycle evidence: Pair 1 `COMMITTED`, first
Runtime selection at `delta=0`, retained delta `0 -> -0.000384455`, a transient
joint-port certificate denial with `fatal_control_failure=0`, and later selected
nonzero updates.  This historical/current T0 lifecycle evidence is not used to
claim after acceptance.

## G1 tests and builds

Focused builds and binaries passed:

```text
phase_offset_tube_filter_test                 9/9
phase_offset_tube_surface_validator_test    11/11
phase_offset_certified_tube_builder_test     6/6
phase_offset_tube_epoch_manager_test        53/53
phase_offset_runtime_test                   35/35
gvf_switch_policy_test                      48/48
phase_offset_matched_adapter_test           82/82
```

The required CTest expression passed 10/10:

```text
/tmp/tube_v2_g1_ctest10.log
```

The final full workspace build passed:

```text
catkin_make -j2 2>&1 | tee /tmp/tube_v2_g1_catkin_make_full_retry2.log
```

An earlier full-build attempt was temporarily blocked by concurrently missing
user-owned swarm test files, and the next attempt reached a concurrent
user-owned Eigen ternary error in `phase_offset_swarm/pair_geometry.h`.  Those
files were repaired by their owner; G1 did not edit them.  The final retry
completed 100% and linked `formation_planning`.

`git diff --check` passed.  Static audit evidence:

```text
/tmp/tube_v2_t0_20260820_225509/static_audit_after.txt
/tmp/tube_v2_t0_20260820_225509/hash_comparison.txt
/tmp/tube_v2_g1_diff_check.log
```

The static audit finds no global envelope/range enumeration/slope-cap or
Filter-generated `REGULARITY` decision in production `tube_filter.cpp`.
Validator current-anchor contiguous-cell trimming and fail-closed UNKNOWN,
OUT_OF_MAP, regularity, clearance, and query-limit behavior remain.  Runtime
still computes cell slopes from filtered endpoint differences.  Protected
production hashes are unchanged; the only intended production hash change is
`tube_filter.cpp`.

## Isolated after launches

Both after runs used fresh private masters, fresh ROS_HOME directories, fresh
evidence directories, the unchanged pillar launch, `observe_only=false`, the
same `(8,0,1)` goal, and no artificial goal delay or parameter/timer change.
Only each run's own PID/PGID and private port were cleaned.

| Run | Evidence | Goal / final odom | Raw anchors | Epoch evidence | Cleanup |
|---|---|---|---|---|---|
| after-1 | `/tmp/tube_v2_g1_after_20260820_232029/` (`127.0.0.1:12886`) | rosout `distance=0.197`; final odom `(8.000000005,-3.71e-9,1.999999916)` | 22/22 found+complete | `control_selected=0`, `certificate_denied=0`, `fatal=0`; no Pair lifecycle logs | `processes_final_check.txt` and `port_final_check.txt` are 0 bytes |
| after-2 | `/tmp/tube_v2_g1_after_20260820_232633/` (`127.0.0.1:12887`) | rosout `distance=0.194`; final odom `(7.9999999998,-2.88e-10,1.999999916)`; internal-goal distance `8.38e-8` | 20/20 found+complete | `control_selected=0`, `certificate_denied=1`, `transient_blocked=2`, `fatal=0`; no `COMMITTED`, selected, or nonzero lifecycle log | `processes_final_check.txt` and `port_final_check.txt` are 0 bytes |

Both bags contain no `all_candidates_path_end_clamped` rosout line.  Run 2's
offline odometry extraction is in `after_summary.txt`; the goal message's z=1
is mapped by existing callback semantics to the internal z=2 target.  Runtime
certificate denial in Run 2 is retained as a dynamic fact (`fatal=0`), not
converted into geometry empty.  No after run observed the required Pair
authority lifecycle, so the dynamic criterion is unmet despite goal reach and
raw anchor completeness.

## Frozen-boundary audit

- Planner-authoritative clearance and margin accounting were not changed.
- Normal Tube zero-connected/zero-only behavior was not changed.
- Existing `TubeBuilder -> TubeFilter -> TubeSurfaceValidator ->
  CertifiedTubeBuilder -> TubeEpochManager` remains the sole production path.
- SurfaceValidator, clearance predicates, UNKNOWN/OUT_OF_MAP handling,
  subdivision/query limits, and regularity checks remain fail-closed.
- Runtime exact-PWL crossed-knot witness and `U+ -> U_safe` semantics were not
  changed.  Run 2's denial is dynamic evidence only.
- A6-P2/H2 authority source was not changed.  Because no after Pair lifecycle
  was observed, this audit does not infer an H2/A6 regression root cause.

## Limitations and stop

The current ROS raw schema cannot reconstruct adjacent raw slopes; this was
reported without schema expansion.  The after runs do not satisfy selected
nonzero offset acceptance, so this audit does not claim dynamic offset
authority, H2 replacement success, or a causal diagnosis.  Per execution spec
§8, stop here with:

```text
DYNAMIC_ACCEPTANCE_UNMET / OFFSET_AUTHORITY_NOT_OBSERVED_AFTER_G1
```

Do not enter backward viability, Runtime/QP/Projector changes, disconnected
component recovery, H2 stage repair, recenter/M4/M6, planner or parameter
tuning, multi-UAV work, or proposal/paper changes in this stage.
