# Tube V2 G3 SurfaceValidator proof-capacity self-audit

Date: 2026-08-21
Workspace: `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`
HEAD: `9a0e97560b8ddf15ac1ed1581030ed4b911f6e43`
Stage status: `G3_IMPLEMENTATION_COMPLETE_QUERY_BUDGET_RESOLVED_DYNAMIC_ACCEPTANCE_UNMET / CURRENT_ANCHOR_INSUFFICIENT_CLEARANCE_REFINED_TO_MAX_DEPTH`
Canonical G3 classification: `G3_PROOF_CAPACITY_CORRECTED_NEW_FIRST_FALSE`

## Outcome

G3 corrected the proof-capacity failure authorized by the execution
specification.  The Validator now skips a clearance query for a valid parent
whose certified cover is already larger than the acceptance target, and it
chooses an anisotropic split only from the existing certified cover
decomposition.  Accepted leaves still use the original nine-point clearance
set and the original requested-radius predicate.

The deterministic wide-ribbon stress moved from a query-budget terminal to a
successful certificate with substantial query headroom.  The readiness-matched
ROS run also moved away from the former G2 terminal (`249993/250000` query
budget exhaustion).  Its new first false is a genuine current-anchor clearance
failure refined to the unchanged maximum subdivision depth.  It remains
fail-closed; no clearance, depth, Builder, Runtime, Pair, planner, launch,
parameter, or schema change is authorized by this result.

## Authorized files and exact source identity

Production Validator files changed:

```text
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_surface_validator.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
```

Tests changed:

```text
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/certified_tube_builder_test.cpp
```

The Builder test change only raises two fixture depths from zero to eight so
that UNKNOWN/UNAVAILABLE leaves remain queryable after the G3 parent-query
ordering.  No Builder production file changed.  No protected adapter,
EpochManager, Runtime, planner, launch, parameter, CMake, package, message, or
ROS schema file changed.

G3-T0 before -> final SHA-256:

| File | Before | Final |
|---|---|---|
| `tube_surface_validator.h` | `65d35bf70dbd3049363ba756b649cf72836693e156ef951022ef0ca19331cf00` | `577bb4bb1da19a9f0d7ad61c2811d0c5588181af0477795215f406c759cff9c4` |
| `tube_surface_validator.cpp` | `8f01af578fdd3b2d8e97ab5d9b6fbb3183ebc3f0c3d0a89e2c5a0a3b67bb6db4` | `6d5b641dbfa13df5c98915d2cf234c75f0349754e6a306c63ede5b38931cc21a` |
| `tube_surface_validator_test.cpp` | `15cfee4752e815b02186eacd8bd3a12c5fb5940bff405938c6a8ac3415a109e6` | `b2f622613587e48bfc6156d02be530e8adfb57dbe2b4f7248caca168967d7c1e` |
| `certified_tube_builder_test.cpp` | `a1f7bf1b46abc5d9817f6e009dc45feddf96d52bdb5831771d5e072ab987be86` | `7ffa20fdc5ad252674675dda3dd1b34c12fccb5a236e0c5bf11ed7d671a3fcf3` |

Unchanged protected Builder identity:

```text
certified_tube_builder.h   6dcf9856d8f3aeaa459ea5b76f151346e6e7dec5453007cef64e5aa8357f475e
certified_tube_builder.cpp 475fc672178758cad17fa9c2099c462e1feca2cd5ca6f7154766025884e5a1ad
```

The worktree was intentionally dirty before and after this stage.  Existing
user changes were preserved; no reset, clean, checkout, restore, stash,
rebase, commit, or push was performed.

## T0 and deterministic capacity gates

T0 was frozen at `/tmp/tube_v2_g3_t0_20260821_134010/`, including source
hashes, the static Validator model, G2 readiness evidence, and the
pre-production negative-control list.  The immutable limits were and remain:

```text
max_subdivision_depth = 12
max_query_samples     = 250000
```

The pre-change deterministic fixture was geometry/path-valid and used an
all-KNOWN_FREE query.  Its terminal was proof-capacity only:

```text
query_sample_count=288/288
limit_exceeded=1
first_failure_reason=insufficient_clearance
max_cover_radius=1.345
```

The callback count equaled the Validator query count.  The T2 leaf-only change
removed impossible-parent clearance calls, but the same v-only fixture still
exhausted its fixed test budget:

```text
query_sample_count=288/288
geometry_cell_count=55
clearance_leaf_cell_count=32
prequery_cover_split_count=17
max_depth_observed=8
limit_exceeded=1
```

That exceeded the G3 T2 50%-budget gate, so the authorized T3 correction was
implemented.  The final deterministic result is:

```text
path_calls=90
callback_calls=144
known_free_calls=144
query_sample_count=144/288
geometry_cell_count=30
clearance_leaf_cell_count=16
prequery_cover_split_count=14
max_depth_observed=3
split_w_count=0
split_v_count=14
split_both_count=0
anisotropic_split_count=7
limit_exceeded=0
first_failure_reason=none
max_cover_radius=1.345
```

The fixture's analytic cover is unchanged.  Its fixed half-voxel term is
`0.025`, its v-reducible term is `1.2 * v_span`, and the target cover is
`0.20`.  The accepted leaf obligation is 16 cells x 9 queries = 144.  The
reduction is execution scheduling and proof-derived dimension choice, not a
weaker certificate.

## Proof-preservation audit

The old and new proof obligations are:

```text
old: CollectCellPoints -> query all 9 parent points -> test cover -> split
new: CollectCellPoints -> test certified cover
     cover > target: skip impossible parent query -> proof-derived split
     cover <= target: query the same 9 points with the same radius predicate
```

The certified decomposition remains:

```text
fixed_cover       = 0.5 * snapshot_resolution
w_reducible_cover = midpoint variation
                    + 0.5 * maximum_delta * normal variation
                    + 0.5 * delta slope * w span
v_reducible_cover = 0.5 * maximum width * v span
cover             = fixed + w_reducible + v_reducible
```

When a certified cover is too large, the split is selected only by comparing
those existing proof terms with `target_cover - fixed_cover`.  A missing or
invalid certificate keeps the conservative sampled fallback.  If a clearance
query has already failed, the code does not infer an obstacle direction from
the cover terms and retains conservative subdivision.  No cache, map, hidden
axis depth, new gate, new parameter, new schema, or second Validator was
introduced.

The accepted-leaf nine-point set, required clearance, cover epsilon,
snapshot half-voxel charge, speed/regularity certificate checks, current-anchor
first ordering, current-connected truncation, UNKNOWN/OUT_OF_MAP/OCCUPIED/
UNAVAILABLE fail-closed behavior, and G2 inward/zero-baseline semantics are
unchanged.

## Readiness-matched ROS evidence

The first G3 launch lookup attempt is preserved at
`/tmp/tube_v2_g3_readiness_20260821_145000/` as a non-product launch lookup
failure.  The successful fresh run is:

```text
/tmp/tube_v2_g3_readiness_20260821_145100/
private master: 12892
fresh ROS_HOME: yes
launch: unchanged phase_offset_esdf_tube_single.launch (absolute path)
Phase A local map count: 900
goal: one publication, (8,0,1) -> internal (8,0,2)
bag: 43,521 messages, about 100 s, 4.4 GB
```

The run reached the goal without a fatal control failure.  It did not open a
nonzero active certification path: there was no Pair `COMMITTED`, no selected
Runtime authority, no retained nonzero delta, and no active obstacle
certification.  This is the authorized stopping point after the proof-capacity
correction.

### Exact new limit attribution

The launch log first records the new failure at candidate 5/map 917 and repeats
it for later candidates.  Candidate 7/map 919 has an independently decoded
raw diagnostic record:

```text
candidate_sequence=7
candidate_sample_count=45
requested_preview=[2.9851342137905053, 5.185134213790505]
current_w=3.1851342137905054
current_sample_found=1
current_sample_complete=1
```

Its attribution is:

```text
first_stop_reason=insufficient_clearance
first_stop_w=3.185134214
invalid_reason="inward search attempted but no candidate certified; attempts=101
last_reason=insufficient_clearance last_queries=117 last_limit_exceeded=1;
using planner zero baseline"
```

The 117 calls are exactly `13 * 9`.  `ValidateCell` starts with the current
anchor cell `{w=current_w,w=current_w,v=0..1,depth=0}`.  Its w span is zero, so
the first failing anchor recursion follows only the first v child (the `&&`
short-circuit), performing one complete nine-point clearance query at each
depth 0 through 12.  At depth 12 the unchanged line
`cell.depth >= max_subdivision_depth` sets `limit_exceeded` and returns false.

This is not query-budget exhaustion: `117 <= 250000 - 9`, so the Builder's
`QueryBudgetExhausted` predicate is false.  It is not the unsplittable-axis
branch either: at the terminal depth the v span is
`1 / 2^12 = 0.000244140625`, which is much larger than `kEpsilon = 1e-10`,
so `can_split_v` is true.  The actual new first false is therefore recorded
as:

```text
CURRENT_ANCHOR_INSUFFICIENT_CLEARANCE_REFINED_TO_MAX_DEPTH
```

This remains a conservative clearance/depth failure.  G3 does not raise the
depth, lower clearance, add an independent axis depth, or repair another layer.

## Verification

Focused tests after the final source state:

```text
SurfaceValidator                  18/18
CertifiedTubeBuilder              11/11
TubeFilter                         9/9
TubeEpochManager                  53/53
Runtime                            35/35
gvf_switch_policy                 48/48
phase_offset_matched_adapter      82/82
required CTest regex              10/10
full catkin_make -j2              PASS
tracked git diff --check          PASS
untracked-stage whitespace check  PASS
```

Forbidden-mechanism searches found no cache/map, sleep/retry, new ROS/schema,
new gate/parameter, independent depth, or second Validator mechanism.  The
final source hashes above and the clean diff check establish attribution to
the four authorized source/test files only.

## Cleanup and stop boundary

The task-owned ROS process group from the successful run was stopped.  An
external post-parent check found no task-owned G3 ROS process and no listener
on TCP port 12892.  Unrelated pre-existing ROS masters on other ports were not
touched.

Stop at `G3_IMPLEMENTATION_COMPLETE_QUERY_BUDGET_RESOLVED_DYNAMIC_ACCEPTANCE_UNMET /
CURRENT_ANCHOR_INSUFFICIENT_CLEARANCE_REFINED_TO_MAX_DEPTH`.  Do not claim
dynamic acceptance, do not tune or raise `max_query_samples` or
`max_subdivision_depth`, and do not modify clearance, Builder, Runtime, Pair,
H2, planner, launch, parameters, or schemas.  Any further correction requires
a new execution specification.
