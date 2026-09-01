# D-ISFGVF R1 Planner–Tube Clearance Metric Root Repair Plan

    DOCUMENT_ROLE=PLAN_ONLY
    DOCUMENT_STATUS=ACCEPTED_AFTER_FRESH_SOL_MAX_AUDIT
    DATE=2026-09-01
    REPOSITORY=/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
    BASELINE=6549f2eb238ff17790e92069adb087b7bb5024e6
    BRANCH=main
    AUTHORITATIVE_REMOTE=d_isfgvf/main
    IMPLEMENTATION_AUTHORIZED=false
    LUNA_AUTHORIZED=false

## 1. Scope and hard stop

This is a root-cause repair plan only. It authorizes no implementation, source
or test edit, launch/config edit, parameter tuning, stage, commit, push, Luna,
D3, planner change, display implementation, or marker-lifecycle work.

The planner is accepted and remains authoritative. The repair is confined to
the immutable Tube clearance-query interpretation. It must never reject,
retire, hold, replan, block installation of, or prevent command execution on a
previously planner-valid path.

The mandatory liveness invariant remains:

    A planner-valid path remains navigable regardless of Tube construction
    failure, local ZERO_ONLY, Tube certificate failure, Tube visualization
    failure, or Tube diagnostic failure.

Only `PLANNER_INVALID` or explicit `CURRENT_STATE_UNSAFE` may own true stop or
replan behavior. This plan adds no such decision.

## 2. Frozen root cause

The root cause is accepted evidence and is not reopened:

    PROVEN_ROOT_CAUSE=PLANNER_TUBE_CLEARANCE_CONTRACT_MISMATCH

The planner and Tube use the same source map, inflated occupied voxel set,
inflation, and body model, but different clearance metrics against the same
runtime `planning/safe_distance=0.4 m`:

- planner ESDF: distance-transform geometry generated from inflated occupied
  voxel grid centers;
- current Tube snapshot query: exact distance to closed occupied voxel AABB
  volumes.

At five complete finite Tube knots in
`w=[2.679822524195818,2.8782996958332387]`, closed-volume clearance is below
0.4, so the unchanged `TubeCrossSectionSolver` correctly performs:

    ClearanceSafe=false
      -> EMPTY_AFTER_OBSTACLE_BOUNDS
      -> SetZeroOnly()
      -> valid complete [0,0]

Marker publication, marker lifecycle, SurfaceValidator, half-voxel behavior,
sample completeness, temporal map change, and final-trajectory loss under the
planner's own ESDF metric are already excluded.

## 3. Exact current source map

### 3.1 Planner ESDF construction and queries

| File / symbol | Caller / context | Data read | Data written | Locks / ownership | Failure or query semantics |
|---|---|---|---|---|---|
| `plan_env/src/sdf_map.cpp` — `SDFMap::cloudCallback()` | ROS PointCloud2 subscriber; may run on the AsyncSpinner | cloud points, current camera position, map origin/resolution/bounds, `obstacles_inflation` | mutable planner `occupancy_buffer_inflate_`; separately builds one immutable snapshot | snapshot sequence/latest pointer uses `CloudOccupancySnapshotStore::mutex`; planner buffers remain SDFMap-owned | ignores cloud points outside the local update range; sets ESDF update due |
| `plan_env/src/sdf_map.cpp` — `SDFMap::updateESDF3d()` | existing ESDF update timer/callback | `occupancy_buffer_inflate_`, local ESDF bounds, resolution | `distance_buffer_`, negative distance, and `distance_buffer_all_` | mutable SDFMap-owned buffers; no Tube dependency | three 1-D squared distance-transform passes |
| `plan_env/include/plan_env/sdf_map.h` — `SDFMap::getDistance(pos)` | Kinodynamic A*, coarse planner checks, GVF/manager diagnostics | containing voxel index and `distance_buffer_all_` | none | mutable planner map read | piecewise-constant containing-grid-cell sample; index is bounded to map |
| `plan_env/src/edt_environment.cpp` — `EDTEnvironment::evaluateEDTWithGrad()` | B-spline optimizer and related planner consumers | eight surrounding `getDistance()` grid samples | local distance/gradient only | planner-owned EDT/SDFMap | trilinear interpolation of ESDF grid samples |
| `path_searching/src/kinodynamic_astar.cpp` | Kino primitive collision loop | inflated categorical occupancy and `getDistance(pos)` | search-local state | planner search context | rejects a primitive sample when occupied/out-of-map or distance is below the current planner margin |
| `bspline_traj/src/bspline_opt_3d.cpp` | optimizer cost/collision functions | trilinear EDT and existing `planning/safe_distance` | optimizer-local cost/gradient | planner optimizer context | unchanged soft cost and sampled collision semantics |

For an inflated occupied grid-index set `I`, resolution `h`, and grid-center
position:

    c_i = grid_origin + h * (i + [0.5,0.5,0.5])

the positive EDT written at a free grid center `c_k` is exactly:

    distance_buffer(c_k) = h * sqrt(min_i ||k-i||^2)
                         = min_i ||c_k-c_i||.

For a free grid cell, `distance_buffer_all_` equals that positive distance.
This is the planner ESDF's generating geometry.

### 3.2 Immutable snapshot construction and current clearance

| File / symbol | Caller / context | Data read | Data written | Locks / ownership | Failure semantics |
|---|---|---|---|---|---|
| `plan_env/src/cloud_occupancy_snapshot.cpp` — `buildCloudOccupancySnapshot()` | `SDFMap::cloudCallback()` | the same cloud, camera, map grid, resolution, local range, and inflation | one value-type snapshot plus private occupied-column index | builder-local; published later as `shared_ptr<const ...>` | invalid input produces `valid=false`; an empty complete cloud is a valid all-free local observation |
| `cloudOccupancySnapshotConsistent()` | every snapshot query and adapter inspection | immutable metadata and exact dense occupied-vector size | none | lock-free const read | invalid metadata/vector size fails closed |
| `queryCloudOccupancySnapshot()` | categorical diagnostics/current-state mask | immutable dense occupancy and observed/map bounds | local result | lock-free const read | invalid=UNAVAILABLE, map outside=OUT_OF_MAP, observed outside=UNKNOWN, else occupied/free |
| `queryCloudOccupancySnapshotClearance()` | current production Tube adapter | same snapshot, private column index, required radius | local closed-volume result | lock-free const read | requires the complete requested ball in observed/grid bounds; bounded local scan; invalid arithmetic fails UNAVAILABLE |

`buildCloudOccupancySnapshot()` matches `SDFMap::cloudCallback()` for this cloud
path: identical floor index mapping, identical XY `ceil(inflation/h)` steps,
identical one-voxel Z inflation, and the same inflated occupied indices. The
snapshot retains observation sequence/stamp, map and observed bounds, grid
origin/count, resolution, included map inflation, dense occupancy, and an
immutable acceleration index.

The old clearance primitive explicitly measures distance to each occupied
voxel's closed box. It remains correct for that documented contract and is not
globally changed or deleted by R1.

### 3.3 Tube adapter and consumers

| File / symbol | Caller / context | Data read | Data written | Locks / ownership | Failure semantics |
|---|---|---|---|---|---|
| `bspline_traj/src/integration/phase_offset_cloud_occupancy_query.cpp` — `inspectCloudOccupancyQuery()` | request admission and Tube worker | config completeness requirement, snapshot consistency/inflation | local captured status | immutable snapshot; no lock | usable only when configuration, `obstacle_set_complete`, snapshot, and included inflation all pass |
| `makeCloudOccupancyClearanceQuery()` | `buildTubeEpoch()` and `buildPreparedTubeEpoch()` | one captured `shared_ptr<const CloudOccupancySnapshot>` and one captured inspection result | returns stateless `ClearanceQuery` lambda | no new lock; lambda holds immutable cohort | unusable or invalid inputs return navigation UNAVAILABLE |
| `PhaseOffsetMatchedAdapter::buildTubeEpoch()` | existing Tube worker/timer pipeline | immutable request/path/frame/snapshot cohort | Candidate/Epoch outputs through existing manager | existing scheduling/currentness only; R1 adds no synchronization | existing Tube failure is candidate failure, not planner failure |
| `PhaseOffsetMatchedAdapter::buildPreparedTubeEpoch()` | existing prepared Tube transaction | same immutable request/path/frame/snapshot cohort | isolated staged Tube result | existing stack-local manager; no R1 lock | no change to planner installation or fallback behavior |
| `TubeBuilder::buildCloudClearance()` | `CertifiedTubeBuilder` | generic `ClearanceQuery` at center and every offset | Tube profile only | navigation layer has no plan_env dependency | unchanged fail-closed sample construction |
| `TubeCrossSectionSolver::solve()` | TubeBuilder | the same generic query for center and both rays | cross-section result only | pure navigation code | unchanged `ClearanceSafe`, boundary refinement, and `SetZeroOnly()` |
| `TubeSurfaceValidator::validate()` | CertifiedTubeBuilder | the same query for every continuous surface-cover sample | Candidate proof/trim facts only | no Runtime/planner ownership | unchanged Euclidean-Lipschitz cover and bounded subdivision |
| `TubeEpochManager::CheckClearance()` | existing current reference/actual/base checks | the same query and the same configured safe distance | Tube epoch safety status | no planner write | occupied/out-of-map or certified clearance below threshold is existing explicit unsafe; unknown/unavailable is indeterminate |

The adapter already injects one `ClearanceQuery` into every center, nonzero
offset, surface, and current-state Tube consumer. Changing only its immutable
low-level callee changes the metric coherently; there is no delta-zero special
case and no second query pipeline.

## 4. Selected metric

The selected Tube metric is:

    d_C(x) = min over inflated occupied voxel indices i of ||x-c_i||

where `c_i` is the same grid-center position whose index seeds the planner
EDT. The bounded query returns `min(d_C(x), required_radius)` together with a
certificate that the true `d_C(x)` is at least the returned lower bound.

Exact contract name:

    PLANNER_ESDF_BASE_OCCUPIED_CENTER_CLEARANCE

Exact proposed plan_env API owner:

    plan_env/include/plan_env/cloud_occupancy_snapshot.h
      CloudOccupancySnapshotPlannerEsdfBaseClearanceResult
      queryCloudOccupancySnapshotPlannerEsdfBaseClearance(...)

    plan_env/src/cloud_occupancy_snapshot.cpp
      queryCloudOccupancySnapshotPlannerEsdfBaseClearance(...)

The new result has a clearly distinct field:

    nearest_inflated_occupied_voxel_center_distance

It must not reuse the old field name or silently change the old function's
meaning.

### 4.1 Planner semantic fidelity

At every free planner grid center, `d_C` is bit-for-bit/mathematically the same
Euclidean center-set distance computed by `updateESDF3d()` before ordinary
floating-point implementation differences. At an arbitrary point:

- `getDistance(point)` evaluates that geometry at the containing grid center;
- planner optimizer trilinear evaluates an interpolation of eight such grid
  samples;
- the new Tube query evaluates the underlying center-set distance directly at
  the actual point.

Therefore R1 aligns the obstacle-distance geometry, occupied set, inflation,
and 0.4 threshold. It does not claim that three different sampling operators
are numerically identical away from grid centers.

### 4.2 Frozen Snapshot-904 replay correction

The original read-only bag and GDB evidence remain available outside the
workspace. Reconstructing snapshot 904 from its exact cloud, map origin
`(-10,-15,-0.01)`, resolution 0.1, and inflation produces the following:

| w | old closed-AABB | new exact center `d_C` | planner containing-cell discrete | planner trilinear |
|---|---:|---:|---:|---:|
| 2.679822524195818 | 0.39910242275510055 | 0.46581958181362049 | 0.44721359549995798 | 0.46557192104095740 |
| 2.7294418171051733 | 0.39125439961484526 | 0.45440903807385757 | 0.41231056256176607 | 0.45513191661937469 |
| 2.7790611100145286 | 0.38956445853076954 | 0.44817903921392843 | 0.41231056256176607 | 0.44772583498113094 |
| 2.8286804029238835 | 0.39392247595057017 | 0.44734004889258666 | 0.40000000000000002 | 0.44819621090364559 |
| 2.8782996958332387 | 0.39999823642274368 | 0.45191618762219005 | 0.40000000000000002 | 0.44999823642274445 |

The provisional suggestion that an exact-center query should return
0.44721359549995715 and 0.412310562561765 is rejected by current source and
exact replay. Those are containing-grid-cell discrete samples. The actual
point-to-center values are 0.46581958181362049 and 0.44817903921392843. This is
not a change to frozen planner evidence; it distinguishes the new metric from
the planner's discrete lookup operator.

All five selected values are strictly above 0.4. Their positive clearance
margins are also larger than zero, so continuity of `d_C` guarantees a
nonzero neighborhood around each centerline knot. The old center-query cause
of the exact `[0,0] x5` raw run is therefore predicted to disappear. Other
Tube constraints remain free to create legitimate zero-only geometry.

## 5. Rejected alternatives

### 5.1 Planner discrete lookup

Using `getDistance(point)` semantics would evaluate `d_C` at the containing
cell center. It is piecewise constant and discontinuous across voxel faces, so
it cannot satisfy the existing 1-Lipschitz continuous-cover proof. It is not
selected.

### 5.2 Trilinear ESDF

Each adjacent EDT grid sample differs by at most one resolution, so each
trilinear partial derivative is bounded by 1 in magnitude. The immediately
available Euclidean gradient bound is therefore `sqrt(3)`, not the existing
unit Lipschitz bound. No current source proves a global 1-Lipschitz trilinear
field.

Selecting trilinear semantics would require changing cover radii or
SurfaceValidator proof mathematics. That is unnecessary scope expansion and
is rejected for R1.

### 5.3 Centerline-only metric switch

A planner metric at delta zero plus closed-AABB metric at nonzero delta is
forbidden. It would make the cross-section contract discontinuous. R1 routes
the exact same center-set query to every `ClearanceQuery` consumer.

### 5.4 Threshold or half-voxel workaround

No threshold reduction, epsilon bypass, 0.39 special case, or subtraction of
half a voxel is permitted. The metric changes; the 0.4 runtime parameter does
not.

## 6. New immutable plan_env primitive

The future implementation, if separately authorized, shall add a sibling to
the closed-volume primitive rather than changing it.

The new function must reuse the existing immutable snapshot metadata,
`occupied` vector, and private occupied-column acceleration. It shall:

1. call the same snapshot consistency checks;
2. require finite point/radius and nonnegative radius;
3. preserve OUT_OF_MAP for a point outside the map;
4. preserve UNKNOWN unless the full requested closed ball lies inside the
   observed AABB;
5. preserve OUT_OF_MAP unless that ball lies inside the snapshot grid;
6. identify the point's voxel; if it is occupied, return OCCUPIED with no
   clearance certificate;
7. use the same bounded padded index range and the existing
   `kMaxClearanceVoxelChecks` bound;
8. scan occupied indices through the existing private column index, with the
   existing dense fallback for manually constructed/copied snapshots;
9. compute each candidate center as
   `grid_origin + resolution*(index+[0.5,0.5,0.5])` and its Euclidean distance
   to the exact query point;
10. return KNOWN_FREE, the nearest distance capped at `required_radius`, and
    `clearance_certified=true`; an empty local occupied search returns the
    requested lower bound, never infinity;
11. fall back to the authoritative dense vector when the optional acceleration
    binding is absent or malformed, and return UNAVAILABLE on invalid
    metadata, overflow, excess bounded-work range, or nonfinite arithmetic,
    matching the current fail-closed pattern.

The old `queryCloudOccupancySnapshotClearance()` implementation, result type,
tests, and closed-AABB numbers remain unchanged. Private scan code may be
factored internally only if existing indexed-versus-dense old-query results
remain bitwise exact.

## 7. Adapter routing and safe-distance ownership

`makeCloudOccupancyClearanceQuery()` retains its public signature. Its only
behavioral change is to call the new planner-ESDF-base function and translate
the new result field into generic navigation `ClearanceQueryResult`.

The surrounding checks remain byte-for-byte/predicate-equivalent:

    cloudOccupancyQueryConfigurationValid
    inspectCloudOccupancyQuery
    obstacle_set_complete
    snapshot consistency
    included_map_inflation sufficiency
    observed/map domain statuses

The sole Tube threshold owner remains:

    PhaseOffsetMatchedAdapter::LoadConfig()
      nh.param("planning/safe_distance",
               config.tube.cross_section.planner_safe_distance, 0.4)

`MakeEpochConfig()` copies that one value into the existing Builder,
CrossSection, Validator, and current-safety consumers. No new constant,
parameter, alias, or fallback is added. The launch value remains 0.4 m.

The snapshot builder continues to record the existing rounded map inflation:

    ceil(0.099 / 0.1) * 0.1 = 0.1 m

`included_map_inflation` remains metadata/eligibility evidence and is not
subtracted from the distance or 0.4 threshold. There is no second inflation,
half-voxel charge, UAV-radius charge, or margin double accounting in R1.

## 8. Continuous-proof compatibility

Let `C={c_i}` be the fixed occupied-center set of one immutable snapshot. For
any points x and y and any c in C:

    ||x-c|| <= ||x-y|| + ||y-c||.

Taking the infimum over c gives:

    d_C(x) <= ||x-y|| + d_C(y).

Swapping x and y gives:

    |d_C(x)-d_C(y)| <= ||x-y||.

Thus `d_C` is globally 1-Lipschitz. Capping it by a constant requested radius
also preserves a unit Lipschitz bound.

The current SurfaceValidator samples a surface cell, proves every cell point
lies within `cover_radius` of those samples, and requests:

    required_clearance + cover_radius + cover_epsilon.

The unit Lipschitz inequality therefore proves every covered surface point
retains at least `required_clearance + cover_epsilon`. The new metric meets the
exact assumption already used. No Builder, CrossSectionSolver,
SurfaceValidator, certificate, cover, subdivision, or half-voxel change is
required.

Adaptive Builder queries use only status/safety transitions and receive the
same coherent metric. Cross-section boundary refinement queries the same
continuous function at every delta. Current reference/actual/base checks use
the same function and threshold. No mixed metric exists inside one Tube.

## 9. Snapshot coherence and fail-closed completeness

R1 keeps the existing cohort:

    one TubeBuildRequest
      -> one shared_ptr<const CloudOccupancySnapshot>
      -> one captured inspectCloudOccupancyQuery result
      -> one ClearanceQuery lambda
      -> all Builder/CrossSection/Validator/current checks for that build

The lambda never reads mutable SDFMap or ESDF buffers. It performs no atomic
snapshot reload and cannot cross observation sequences mid-build.

`obstacle_set_complete=false` remains UNAVAILABLE because the adapter captures
`status.usable=false` before entering plan_env. A valid numerical center
distance cannot bypass that gate. Invalid snapshots, insufficient preincluded
inflation, unobserved balls, and grid/map boundary violations remain
fail-closed.

## 10. Performance and navigation liveness

The new primitive reuses the existing bounded neighborhood and occupied-column
index:

- dense fallback: `O(X*Y*Z)` over the already bounded requested-radius voxel
  box, capped by 4,194,304 candidate voxels;
- indexed path: `O(X*Y)` column visits plus lower-bound searches and the
  occupied entries inside the selected Z ranges;
- memory: no second dense occupancy or ESDF buffer; reuse the existing private
  immutable index;
- per occupied hit: one center-coordinate construction and Euclidean norm,
  comparable to the old closed-box distance calculation;
- Tube query count: exactly unchanged because no consumer or subdivision
  algorithm changes.

The work remains in existing Tube construction/prepared-Tube contexts. R1
does not add work to Kinodynamic A*, optimizer callbacks, planner FSM,
odometry callbacks, trajectory installation locks, or command publication. It
adds no wait, condition variable, mutex, retry, handshake, thread, worker,
timer, publisher, subscriber, or queue.

If the new query fails, existing Tube/certificate failure semantics apply.
Planner authority and centerline navigation are not changed or revoked.

## 11. Exact implementation symbol plan

### 11.1 Behavior-changing production paths

`plan_env/include/plan_env/cloud_occupancy_snapshot.h`:

- declare the distinct planner-ESDF-base result and function;
- add only the friend access needed to reuse the immutable private index;
- retain the old API and semantics.

`plan_env/src/cloud_occupancy_snapshot.cpp`:

- add center-distance evaluation and the new bounded query;
- share validation/index traversal without changing old results;
- no SDFMap/planner dependency.

`bspline_traj/src/integration/phase_offset_cloud_occupancy_query.cpp`:

- route `makeCloudOccupancyClearanceQuery()` to the new function;
- preserve all inspection/completeness/inflation predicates and status
  translation.

### 11.2 Contract-comment-only production paths

These headers currently say “occupied voxel volume” even though the future
generic `ClearanceQuery` will carry the planner-ESDF-base contract. Only those
comments may change; declarations, data layout, and behavior remain fixed:

- `bspline_traj/include/bspline_race/integration/phase_offset_cloud_occupancy_query.h`
- `phase_offset_navigation/include/phase_offset_navigation/tube_cross_section.h`
- `phase_offset_navigation/include/phase_offset_navigation/tube_builder.h`
- `phase_offset_navigation/include/phase_offset_navigation/tube_epoch_types.h`

No `.cpp` in `phase_offset_navigation` is authorized.

### 11.3 Test paths

`plan_env/test/cloud_occupancy_snapshot_test.cpp` owns direct primitive,
planner-EDT identity, old-query preservation, dense/indexed equivalence,
domain, and frozen-fixture tests.

`bspline_traj/test/phase_offset_cloud_occupancy_query_test.cpp` owns adapter
selection, completeness/inflation fail-closed behavior, and a direct
query-to-CrossSection integration regression.

## 12. Exact future implementation whitelist

Only a separately authorized implementation may modify these paths:

1. `src/swarm_planner/plan_env/include/plan_env/cloud_occupancy_snapshot.h`
2. `src/swarm_planner/plan_env/src/cloud_occupancy_snapshot.cpp`
3. `src/swarm_planner/plan_env/test/cloud_occupancy_snapshot_test.cpp`
4. `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_cloud_occupancy_query.h`
5. `src/swarm_planner/bspline_traj/src/integration/phase_offset_cloud_occupancy_query.cpp`
6. `src/swarm_planner/bspline_traj/test/phase_offset_cloud_occupancy_query_test.cpp`
7. `src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_cross_section.h` — comment only
8. `src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_builder.h` — comment only
9. `src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_types.h` — comment only

No planner, `gvf_manager.cpp`, SDFMap/EDT source, Kino, B-spline optimizer,
Tube algorithm `.cpp`, SurfaceValidator, Runtime, launch, or configuration
path is authorized. Any required behavior edit outside paths 1, 2, and 5 is a
scope-expansion blocker and must return to Main.

## 13. Deterministic future regression plan

### A — metric identity with planner EDT base

Construct one SDFMap/test snapshot with identical grid origin, resolution, and
inflated occupied indices. Run the existing `updateESDF3d()` fixture and, for
representative free grid centers, assert:

    new exact-center query == SDFMap positive EDT grid value

within an implementation-appropriate near-exact tolerance. Include axial,
diagonal, multiple-obstacle, occupied, and empty local sets.

### B — frozen mismatch point 1

Embed the minimal reproducible fixture:

    grid_origin=(-10,-15,-0.01)
    resolution=0.1
    occupied index=(128,148,10)
    point=(2.6643275039478231,
           0.27533414135136092,
           0.99994409891574243)

Assert:

    old closed-AABB = 0.39910242275510055
    planner containing-cell discrete = 0.4472135954999579...
    new exact occupied-center = 0.46581958181362049
    new clearance > unchanged 0.4

This explicitly prevents confusing the planner discrete sample with the new
continuous center-set value.

### C — frozen mismatch point 2

Using the same grid and occupied index with:

    point=(2.7627822162288762,
           0.28778254721111168,
           0.99995064617094009)

assert:

    old closed-AABB = 0.38956445853076954
    planner containing-cell discrete = 0.412310562561766...
    new exact occupied-center = 0.44817903921392843
    new clearance > unchanged 0.4

### D — threshold ownership unchanged

Retain existing adapter/config tests showing the one runtime value is 0.4 and
the query receives that requested radius. Add no hard-coded production
threshold. The fixture constant 0.4 is test evidence only.

### E — snapshot completeness

With a consistent numerical snapshot but
`obstacle_set_complete=false`, `makeCloudOccupancyClearanceQuery()` returns
UNAVAILABLE. Repeat for null/invalid snapshot and insufficient included map
inflation.

### F — observed and map domains

For the new primitive, a point outside map remains OUT_OF_MAP, a point outside
the observed AABB remains UNKNOWN, and a requested closed ball crossing the
observed or grid boundary remains UNKNOWN/OUT_OF_MAP exactly as today.

### G — inflation accounting

Build snapshot and planner cloud occupancy from the same cloud fixture. Assert
identical occupied indices and unchanged included inflation. Changing the
metadata value alone must not numerically subtract or add distance in either
primitive.

### H — old closed-volume primitive preserved

Run every existing closed-AABB unit/differential test unchanged, including
face/diagonal values, metadata failure, nonbinary occupancy, copy/index
binding, randomized indexed-versus-dense bitwise equality, and capped free
results.

### I — coherent Tube semantics

Feed `makeCloudOccupancyClearanceQuery()` from the frozen minimal fixture into
an unchanged `TubeCrossSectionSolver`. Assert the center no longer follows
`EMPTY_AFTER_OBSTACLE_BOUNDS -> SetZeroOnly`, and a positive local interval is
found/refined using the same query for positive and negative offsets. Retain
existing independent tests proving a genuinely sub-threshold query still
executes unchanged `SetZeroOnly()`.

### J — navigation isolation

Source/whitelist assertions must prove no planner/FSM/authority file changed.
Run existing adapter liveness/currentness tests and assert query failure still
produces only existing Tube candidate/indeterminate behavior unless the
unchanged categorical/current query gives explicit OCCUPIED or OUT_OF_MAP.

### Additional proof/performance regressions

- add randomized new-query indexed-versus-fresh-dense differential tests;
- test exact center coordinates use `(index+0.5)*resolution`;
- test multiple occupied centers select the true nearest center;
- test bounded-work overflow/excess returns UNAVAILABLE;
- record that construction/validator query counts remain unchanged for an
  identical synthetic Tube input;
- run the full existing `cloud_occupancy_snapshot_test`,
  `phase_offset_cloud_occupancy_query_test`, `tube_cross_section_test`,
  `tube_builder_test`, `tube_surface_validator_test`,
  `tube_epoch_manager_test`, `certified_tube_builder_test`, and
  `phase_offset_matched_adapter_test` targets without modifying the latter
  seven test paths.

## 14. Same-scene ROS A/B acceptance

Reuse the same pillar scene and configuration that produced candidate 9,
snapshot 904, and the five-knot gap. Preserve the original bag/GDB evidence
read-only and capture a new after-run cohort.

Before/after equality requirements:

- identical planner source, algorithms, path geometry, goal, and normal
  navigation outcome;
- identical map topic/source, occupied indices, resolution, inflation, and
  `planning/safe_distance=0.4`;
- no intentional planner timing/search/optimizer/FSM change;
- no new HOLD, replan loop, path retirement, command gap, planner failure, or
  authority denial;
- same snapshot completeness, sequence/stamp, observed-domain, and inflation
  eligibility rules.

After-run metric requirements:

- production Tube queries identify the planner-ESDF-base center metric;
- the old closed-volume replay remains available and unchanged as diagnostic
  evidence;
- the five exact centerline points return the selected exact-center values
  above 0.4;
- those five raw Builder knots no longer become `[0,0]` because the center
  query alone is below 0.4;
- CrossSection and continuous Validator use the same selected metric for
  center and all offsets;
- ordinary Tube construction and normal goal completion succeed without a
  liveness regression.

Acceptance does not require every Tube location to be nonzero. ZERO_ONLY from
legitimate occupancy, observed-domain, regularity, curvature, continuous
cover, or other unchanged constraints remains valid.

## 15. Protected scope

The future repair must leave unchanged:

- Kinodynamic A*, all path searching, search margins/check counts;
- B-spline parameterization, optimizer, costs, ESDF queries, SDFMap,
  EDTEnvironment, planner installation, retry, FSM, HOLD, and replan;
- launch/config and `planning/safe_distance`;
- occupancy inflation and snapshot build semantics;
- TubeCrossSectionSolver implementation, `ClearanceSafe`, boundary search,
  `SetZeroOnly`, TubeBuilder implementation, TubeFilter, SurfaceValidator,
  CertifiedTubeBuilder, Tube classification, Runtime, Recovery, Handoff,
  allocator, selected-u, authority, Pair/session/currentness;
- publishers, queues, workers, timers, callbacks, locks, and marker lifecycle;
- the accepted display-only P1 plan, which remains unimplemented and outside
  R1.

## 16. Main Q1-Q13 review

Q1. Is the proven root cause being repaired rather than hidden visually?

    YES. The production Tube clearance metric is aligned at its immutable
    query owner; no marker change is included.

Q2. Is planner code completely unchanged?

    YES. No planner, SDFMap/EDT, Kino, optimizer, manager, or installation
    source is in the whitelist.

Q3. Can a planner-valid path still navigate if Tube construction fails?

    YES. The query remains in the existing passive/independent Tube pipeline
    and adds no planner or command authority edge.

Q4. Is `planning/safe_distance` unchanged?

    YES. The same one loaded runtime value is passed through unchanged.

Q5. Is map inflation unchanged?

    YES. Snapshot build and included inflation are unchanged.

Q6. Does the new Tube metric faithfully represent the selected existing
planner ESDF geometry?

    YES. It is the exact point-set distance whose grid-center samples seed the
    planner EDT. The plan explicitly does not conflate it with discrete or
    trilinear sampling away from grid centers.

Q7. Is the metric coherent for center and all nonzero offsets?

    YES. The one adapter lambda is the sole query for CrossSection, Builder,
    Validator, and current checks.

Q8. Is snapshot completeness still fail-closed?

    YES. `inspectCloudOccupancyQuery().usable` is unchanged and captured
    before the new plan_env call.

Q9. Is the snapshot/map cohort immutable and coherent?

    YES. One captured const snapshot and observation identity feed the full
    build without mutable SDFMap access.

Q10. Are existing continuous Tube proof assumptions still mathematically
valid?

    YES. Distance to a fixed point set is globally 1-Lipschitz, exactly the
    current Euclidean cover assumption.

Q11. Are TubeCrossSectionSolver, `SetZeroOnly`, TubeBuilder, TubeFilter, and
SurfaceValidator behavior unchanged?

    YES. No implementation `.cpp` for any of them is authorized. Three
    navigation headers receive contract-comment wording only.

Q12. Does frozen replay predict elimination of the five `[0,0]` knots caused
by the mismatch?

    YES. Exact snapshot replay gives all five center distances strictly above
    0.4, with a positive continuous neighborhood at every knot.

Q13. Can the repair cause HOLD, replan, or planner rejection?

    NO. It has no planner/FSM/authority write or synchronization path.

    MAIN_PLAN_REVIEW=PASS

## 17. Fresh Sol audit and stop boundary

Only after the Main review above, exactly one fresh `gpt-5.6-sol` with
`reasoning=max` shall audit this plan read-only. Its first question is:

    CAN_THIS PLAN MODIFY, REJECT, HOLD, RETIRE, OR PREVENT EXECUTION OF A
    PREVIOUSLY PLANNER-VALID PATH?

Required answer is NO. Any blocker, major, YES first answer, or non-acceptable
exact verdict stops this plan as not accepted. There is no second Sol or Luna
in this task.

The one fresh Sol audit independently verified the baseline, replay numbers,
center-set/EDT identity, unit-Lipschitz proof, completeness/inflation gates,
safe-distance ownership, bounded scan, dependency direction, whitelist, tests,
same-scene acceptance, and navigation non-interference. No second Sol is
permitted or needed.

    SOL_FIRST_QUESTION_ANSWER=NO
    SOL_BLOCKER_COUNT=0
    SOL_MAJOR_COUNT=0
    SOL_VERDICT=R1_PLANNER_TUBE_CLEARANCE_METRIC_ROOT_REPAIR_PLAN_ACCEPTABLE
    PLAN_STATUS=ACCEPTED

## 18. Current-task state

    PLAN_MODE=R1_PLANNER_TUBE_CLEARANCE_METRIC_ROOT_REPAIR

    BASELINE=6549f2eb238ff17790e92069adb087b7bb5024e6

    PROVEN_ROOT_CAUSE=PLANNER_TUBE_CLEARANCE_CONTRACT_MISMATCH

    SELECTED_TUBE_CLEARANCE_METRIC=exact Euclidean distance from the query point to the nearest inflated occupied voxel center in one immutable CloudOccupancySnapshot, capped at the requested radius

    METRIC_OWNER=plan_env/cloud_occupancy_snapshot.h/.cpp::queryCloudOccupancySnapshotPlannerEsdfBaseClearance

    PLANNER_CODE_CHANGED=false
    KINO_SEARCH_CHANGED=false
    BSPLINE_OPTIMIZER_CHANGED=false
    PLANNER_INSTALL_CHANGED=false
    FSM_CHANGED=false

    PLANNING_SAFE_DISTANCE_CHANGED=false
    MAP_INFLATION_CHANGED=false

    SNAPSHOT_COMPLETENESS_PRESERVED=YES
    CONTINUOUS_PROOF_COMPATIBLE=YES

    TUBE_CROSS_SECTION_CHANGED=NO
    SURFACE_VALIDATOR_CHANGED=NO

    NAVIGATION_LIVENESS_PRESERVED=YES
    FROZEN_GAP_EXPECTED_TO_BE_REMOVED_BY_ROOT_REPAIR=YES

    IMPLEMENTATION_WHITELIST=cloud_occupancy_snapshot.h; cloud_occupancy_snapshot.cpp; cloud_occupancy_snapshot_test.cpp; phase_offset_cloud_occupancy_query.h; phase_offset_cloud_occupancy_query.cpp; phase_offset_cloud_occupancy_query_test.cpp; tube_cross_section.h(comment-only); tube_builder.h(comment-only); tube_epoch_types.h(comment-only)

    MAIN_PLAN_REVIEW=PASS

    DISPLAY_ONLY_P1_IMPLEMENTED=false
    IMPLEMENTATION_AUTHORIZED=false
    LUNA_AUTHORIZED=false

    CODE_MODIFIED=false
    STAGED_PATHS=0
    COMMIT_PUSH=false
