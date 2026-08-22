# Tube V2 G5 exact current-anchor sampled-cover self-audit

Date: 2026-08-21
Workspace: `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`
Execution spec: `Codex_Tube_V2_G5_Exact_Current_Anchor_Sampled_Cover_Execution_Spec_2026-08-21.md`
Final classification: `G5_IMPLEMENTATION_COMPLETE_NEW_FIRST_FALSE`

## Outcome

G5 completed its authorized implementation and verification boundary. The
production correction is limited to the exact sampled current-anchor branch:
the duplicate fixed half-voxel charge is removed only when
`cell.w0 == current_w && cell.w1 == current_w`. The sampled factor, residual
epsilon, nine-point query set, fail-closed status/certification predicate,
current-anchor ordering, v-only recursion, nondegenerate sampled fallback, and
certified nondegenerate cover are unchanged.

The same-snapshot G4 replay passes with the new request
`0.4000010000016007` and returned certified lower bound equal to that request;
the exact nearest closed-voxel distance remains `0.44280071140075605`.

The first ROS attempt at
`/tmp/tube_v2_g5_readiness_20260821_153300/` exposed the launch default
`observe_only=true` and is retained only as
`PRECONDITION_MISMATCH_OBSERVE_ONLY_TRUE`. It was not used for acceptance.
The corrected run at
`/tmp/tube_v2_g5_readiness_20260821_154000/` used
`phase_offset_manual_observe_only:=false`, reached the goal once, and exposed a
later inward-search no-certified-candidate terminal at candidate 8/map 919.
Its 2,484-query last validation is far below the 250,000-query budget
threshold, so full-width query-budget exhaustion is excluded. That later
failure is evidence-only; no additional production layer was changed.

## Authorized source identity

G5 T0 is frozen at `/tmp/tube_v2_g5_t0_20260821_152514/` with HEAD
`9a0e97560b8ddf15ac1ed1581030ed4b911f6e43`. The only authorized source/test
files changed after T0 are:

```text
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
```

| File | G5 T0 SHA-256 | Final SHA-256 |
|---|---|---|
| `src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp` | `6d5b641dbfa13df5c98915d2cf234c75f0349754e6a306c63ede5b38931cc21a` | `986ca3b3a479595bafe1e99ac1bfc2318bfbe14c55859d6269198627e5e221a9` |
| `src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp` | `b2f622613587e48bfc6156d02be530e8adfb57dbe2b4f7248caca168967d7c1e` | `153ac916fe0d35152fce69c58d76431328d15910af5536484687be093d585524` |

The following protected hashes are unchanged from the G5 T0 manifest:

```text
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_surface_validator.h  577bb4bb1da19a9f0d7ad61c2811d0c5588181af0477795215f406c759cff9c4
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/certified_tube_builder.h  6dcf9856d8f3aeaa459ea5b76f151346e6e7dec5453007cef64e5aa8357f475e
src/swarm_planner/phase_offset/phase_offset_navigation/src/certified_tube_builder.cpp  475fc672178758cad17fa9c2099c462e1feca2cd5ca6f7154766025884e5a1ad
src/swarm_planner/phase_offset/phase_offset_navigation/test/certified_tube_builder_test.cpp  7ffa20fdc5ad252674675dda3dd1b34c12fccb5a236e0c5bf11ed7d671a3fcf3
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp  d9248801e9f34629c269f0bb11dcf0daa698cded03dec2f59ca4db3fdd7893f3
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp  7544db8040dfbe7411676bc531de7eb803ccc0509a834c71491c90908ac7c4ed
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp  2a0af9e58dd7d77e89676e916f4daf4a15a63b9ee148fff51b1d34c95dc34d8a
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp  aac734d83fb5c60c48a2b2cd25bf1104151a121c314b5502aec0d6f152855abd
src/swarm_planner/plan_env/src/cloud_occupancy_snapshot.cpp  06902ef0eef8fc8871999d55531802ccb2b076f332e5aaf5a8d40a9e396c468e
src/swarm_planner/bspline_traj/src/integration/phase_offset_cloud_occupancy_query.cpp  a80f7a4470e94548bf103e4ae4253e829f2a62c83b820f44b5e6a4deea871094
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp  4f324cb9f83fc7d60fbea9bc32338244cfcc00292660165d9ac6f930ca93442a
src/swarm_planner/bspline_traj/launch/phase_offset_esdf_tube_single.launch  334d807751ad6aa1739c241f07b5e22c554a3ff2470838e975a9cad8009e3813
```

The worktree was already intentionally dirty. Existing user changes were
preserved; no reset, clean, checkout, restore, stash, rebase, commit, or push
was performed. T0/T1 hash manifests are retained in
`/tmp/tube_v2_g5_t0_20260821_152514/` and
`/tmp/tube_v2_g5_t1_20260821_152904/`.

## Exact implementation boundary

Before G5, the sampled fallback was:

```text
cover_radius = 1.1 * sampled_radius + 0.5 * snapshot_resolution
```

The final production code is:

```text
const bool exact_current_anchor =
    cell.w0 == context.current_w && cell.w1 == context.current_w;

cover_radius = 1.1 * sampled_radius +
    (exact_current_anchor ? 0.0 : 0.5 * context.snapshot_resolution);
```

The equality is exact. No tolerance, near-degenerate width test, candidate
classification, map sequence, clearance value, or ROS state participates in
the branch. `current_w` is file-local `ValidationContext` state only; no
public header, configuration, parameter, topic, schema, scheduler, or query
API was added.

The certified nondegenerate path still computes:

```text
cover = 0.5 * snapshot_resolution + w_reducible_cover + v_reducible_cover
```

The nondegenerate sampled fallback still has its fixed half-voxel term. The
comment in `CollectCellPoints()` records the proof boundary: the immutable
closed-voxel query already owns voxel support for the exact affine fixed-w
current-anchor surface, while G4 did not prove removal for nondegenerate
sampled cells.

No changes were made to `CellClearancePasses()`, its requested radius or
acceptance predicate, the row-major 3x3 loop, `CertifiedCellCoverRadius()`,
the proof-derived split policy, depth/query defaults, Builder, Filter, Epoch,
Runtime, Pair, H2, planner, map, adapter, launch, or ROS schema.

## Boundary tests and static accounting

The single existing SurfaceValidator test binary now contains 22 passing tests
(the G5 T0 binary had 18). The G5-directed tests cover:

* exact current-anchor sampled cover with all first nine requests equal to
  `0.400001` and returned certified clearances `0.425`;
* true certified clearance `0.4000005` remaining fail-closed with exactly nine
  calls and `INSUFFICIENT_CLEARANCE`;
* UNKNOWN, OUT_OF_MAP, OCCUPIED, UNAVAILABLE, uncertified KNOWN_FREE, and NaN
  clearance remaining fail-closed;
* nondegenerate sampled fallback retaining
  `1.1 * sampled_radius + 0.5 * snapshot_resolution`;
* certified nondegenerate cover retaining its legacy fixed plus reducible
  breakdown; and
* the deterministic wide-ribbon accounting freeze.

The wide-ribbon output is:

```text
query_sample_count=144
limit_exceeded=0
geometry_cell_count=30
clearance_leaf_cell_count=16
prequery_cover_split_count=14
max_depth_observed=3
split_w_count=0
split_v_count=14
split_both_count=0
anisotropic_split_count=7
first_failure_reason=none
max_cover_radius=1.32
```

The changed `1.32` maximum is only the exact current-anchor sampled oracle;
the certified nondegenerate minimum remains the legacy fixed-plus-reducible
total. Accepted leaves remain 16 cells x 9 queries = 144.

## Same-snapshot replay

The replay is recorded at
`/tmp/tube_v2_g5_t1_20260821_152904/same_snapshot_replay.json` and reuses the
G4 candidate 7/map 919/snapshot `1787292633.0334985` evidence:

```text
point=(3.286887963172443,0.34280071140075674,1.0002026651862932)
snapshot_resolution=0.1
planner_safe_distance=0.4
sampled_radius=1.4551915228366853e-12
cover=1.600710675120354e-12
cover_epsilon=1e-6
requested_radius=0.4000010000016007
returned_clearance=0.4000010000016007
exact_nearest_closed_voxel_distance=0.44280071140075605
status=KNOWN_FREE
clearance_certified=true
pass=true
```

This is an accounting replay, not a substitute for the independent ROS run.

## Build and regression evidence

Logs are retained under `/tmp/tube_v2_g5_t1_20260821_152904/`.

Focused build/tests:

```text
SurfaceValidator                  22/22
CertifiedTubeBuilder              11/11
CloudOccupancySnapshot            14/14
Cloud occupancy query              4/4
```

G3 regression tests:

```text
TubeFilter                         9/9
TubeEpochManager                  53/53
Runtime                            35/35
gvf_switch_policy                 48/48
continuous_phase_path             11/11
matched adapter                   82/82
```

The required CTest regex passed `10/10`; the complete `catkin_make -j2`
passed; and `git diff --check` passed. No new test binary, CMake target,
fixture module, or test framework was introduced.

## Corrected independent ROS run

Identity and raw evidence are under
`/tmp/tube_v2_g5_readiness_20260821_154000/`:

```text
private ROS master port       12911
fresh ROS_HOME                yes
launch                        phase_offset_esdf_tube_single.launch
launch path                   /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/src/swarm_planner/bspline_traj/launch/phase_offset_esdf_tube_single.launch
launch override               phase_offset_manual_observe_only:=false
map                           pillar.pcd
goal                          (8,0,1), internal (8,0,2)
goal publications             exactly one
pre-goal /sim/local_map       900 messages
bag                           43,361 messages, 100 s, about 4.4 GB
harness exit                  0
```

The effective parameter dump records `observe_only: false`. The readiness
events show map maturity before the goal and a single goal/reached sequence;
the launch log records `[POINT_GOAL][REACHED] distance=0.200`.

The corrected bag was decoded into `decoded/` and field-preserving
`evidence_parsed/`. All nine raw candidate records have
`current_sample_found=1` and `current_sample_complete=1`. The relevant
cohorts are:

```text
candidate 7 / map 918 / current_w=2.912493798106726
  first stop: insufficient_clearance at 2.912493798
  attribution: 117 queries, depth terminal, limit flag 1

candidate 8 / map 919 / snapshot stamp=1787298468.9096804
  current_w=3.113361964553144, sample found+complete
  first stop: insufficient_clearance at 3.113361965
  invalid_reason="inward search attempted but no candidate certified; attempts=101
  last_reason=insufficient_clearance last_queries=2484 last_limit_exceeded=1;
  using planner zero baseline"
```

The exact remaining first-false detail is
`INWARD_SEARCH_NO_CERTIFIED_CANDIDATE /
LAST_VALIDATION_INSUFFICIENT_CLEARANCE_WITH_LIMIT_FLAG`; it is recorded in
`new_first_false.tsv` as `G5_IMPLEMENTATION_COMPLETE_NEW_FIRST_FALSE`.
The `limit_exceeded=1` flag is retained verbatim, but G5 does not infer
maximum-depth or unsplittable-axis attribution from that flag. Since
`2484 < 249992`, full-width query-budget exhaustion is explicitly excluded;
distinguishing any remaining Validator terminal cause is deferred to a future
read-only audit. The unchanged planner safe distance is `0.4`; cloud
diagnostics retain `included_map_inflation=0.1`, full effective radius `0.55`,
and residual effective radius `0.45`.

The run reached the goal, but the complete dynamic offset acceptance chain did
not occur: no terminal nonzero `OFFSET_CERTIFIED` profile was retained, no
Pair `COMMITTED`/selected Runtime authority or retained nonzero delta was
observed, and the readiness state ended with the zero gate closed and active
profile unavailable/incomplete. A transient earlier diagnostic cohort did
show an obstacle-certified active profile, but it was not the terminal
candidate and does not constitute G5 acceptance. The run had no genuine
fatal invariant; path-end clamp counts were not treated as a G5 source change.

The first run's `observe_only=true` mismatch is not conflated with this
corrected-run product evidence. No repeat was launched for the product first
false.

## Cleanup

The harness-generated `processes_final_check.txt` contains its own shell/rg
command text and is therefore not authoritative. The independent post-run
check is:

`/tmp/tube_v2_g5_readiness_20260821_154000/cleanup_external_check.txt`

It performed a post-parent `/proc` command-line scan and `ss` listener scan for
task-owned ROS processes and ports 12910/12911. Both sections are empty. The
earlier observe-only run has the corresponding empty external proof at
`/tmp/tube_v2_g5_readiness_20260821_153300/cleanup_external_check.txt`.

## No-bloat and protected-boundary audit

The final source and hash audit establishes:

```text
one exact local predicate only                       PASS
no tolerance-based branch widening                   PASS
no public header/API/config/default change            PASS
no new gate/parameter/cache/retry/timer/thread       PASS
no new topic/diagnostic/schema field                 PASS
accepted current anchor remains exactly 9 queries    PASS
nondegenerate sampled half-voxel unchanged           PASS
certified nondegenerate half-voxel unchanged         PASS
planner clearance and cover epsilon unchanged        PASS
depth=12 and query budget=250000 unchanged            PASS
Builder/Filter/Epoch/Runtime/Pair/H2/planner/map     PASS (protected hashes)
launch and ROS schema unchanged                       PASS (protected hashes)
```

The new candidate 8 inward-search no-certified-candidate terminal is explicitly
not repaired in G5. No nondegenerate half-voxel was removed, no query/depth
limit was raised, and no downstream mechanism was added.

## Final status

```text
G5_IMPLEMENTATION_COMPLETE_NEW_FIRST_FALSE
```

G5 exact current-anchor accounting is implemented, unit-tested, statically
audited, replayed, and exercised in the corrected independent ROS run. The
remaining dynamic first false is recorded only; any correction to that layer
requires a new execution specification.
