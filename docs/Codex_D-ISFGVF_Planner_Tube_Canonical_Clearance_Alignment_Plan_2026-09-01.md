# D-ISFGVF Planner–Tube Canonical Clearance Alignment Plan

Date: 2026-09-01
Status: Candidate, plan only
Baseline: 6549f2eb238ff17790e92069adb087b7bb5024e6
Repository: Accqqq/D-ISFGVF; authoritative remote: d_isfgvf
Implementation authorized: No

## 1. Scope

This is a design-only plan. It defines the smallest correctness repair for
the proven planner–Tube centerline-clearance mismatch. It authorizes no
implementation, launch/config edit, parameter tuning, staging, commit, push,
Luna, D3, D4, Stage1B, marker-lifecycle work, Runtime/Recovery/Handoff work,
or C1/C2/C3 redesign.

The first repair only:

1. reuses the existing immutable closed-occupied-voxel clearance primitive;
2. adds a planner-owned bounded continuous-path certificate;
3. rejects a new path before planner authority installation if that certificate
   is absent or false; and
4. binds validation and same-callback Tube construction to one immutable
   CloudOccupancySnapshot identity.

TubeCrossSectionSolver, ClearanceSafe, SetZeroOnly, map resolution,
obstacle inflation, SurfaceValidator, TubeFilter, half-voxel semantics, and
the existing runtime failure behavior remain unchanged.

## 2. Proven diagnosis

The frozen cohort is path/source revision 1, Tube candidate 9, snapshot 904.
The internal candidate gap is:

~~~
w = [2.679822524195818, 2.8782996958332387]
~~~

The five Tube closed-volume clearances are:

~~~
0.39910242275510055
0.39125439961484526
0.38956445853076954
0.39392247595057017
0.39999823642274368
~~~

Each is below planner_safe_distance=0.4. The current solver therefore
correctly performs:

~~~
ClearanceSafe=false -> EMPTY_AFTER_OBSTACLE_BOUNDS -> SetZeroOnly() -> [0,0]
~~~

The prior read-only audit proved:

~~~
SAME_SOURCE_MAP=YES
SAME_OCCUPIED_GEOMETRY=YES
SAME_INFLATION=YES
SAME_BODY_MODEL=YES
SAME_CLEARANCE_METRIC=NO
~~~

Planner uses inflated voxel occupancy plus voxel-centre/trilinear ESDF.
Tube uses exact Euclidean distance to the union of closed occupied-voxel AABB
volumes. At w=2.679822524195818:

~~~
Tube       = 0.39910242275510055
Planner ESDF discrete   = 0.44721359549995715
Planner ESDF trilinear  = 0.4655719210409569
~~~

At w=2.7790611100145286:

~~~
Tube       = 0.38956445853076954
Planner ESDF discrete   = 0.412310562561765
Planner ESDF trilinear  = 0.4477258349811303
~~~

Replay against the planning-time cloud and snapshot 904 is identical.
Temporal map change and final-trajectory loss under planner ESDF are not
involved.

~~~
PROVEN_ROOT_CAUSE=PLANNER_TUBE_CLEARANCE_CONTRACT_MISMATCH
~~~

## 3. Canonical primitive owner and dependency direction

~~~
CANONICAL_CLEARANCE_PRIMITIVE_OWNER=plan_env::CloudOccupancySnapshot clearance query infrastructure
~~~

The existing primitive is:

~~~
plan_env::queryCloudOccupancySnapshotClearance(
    const CloudOccupancySnapshot&, const Eigen::Vector3d&, double)
~~~

It already owns the immutable observation identity, voxel origin/resolution/
bounds, included map inflation, exact point-to-closed-occupied-voxel-AABB
distance, observed-domain fail-closed behavior, and bounded indexed/dense
lookup.

The current dependency direction is plan_env below bspline_race and the
PhaseOffset adapter. bspline_race already depends on plan_env; plan_env does
not depend on bspline_race or phase_offset_navigation. Calling the existing
primitive directly from the planner introduces no cycle and does not make Tube
the authority. No plan_env production source change is required.

The continuous validator itself belongs to the bspline_race planner frontend,
because that module owns UniformBspline/ContinuousPhasePath representation and
the decision to install planner authority. It must not call Tube, Runtime,
Recovery, Handoff, or marker code.

## 4. Canonical mathematical contract

For immutable snapshot S with resolution h, grid origin g, and occupied voxel
indices I, define the closed occupied set:

~~~
O_S = union over i in I of
      [g_x+h*i_x, g_x+h*(i_x+1)]
    x [g_y+h*i_y, g_y+h*(i_y+1)]
    x [g_z+h*i_z, g_z+h*(i_z+1)].
~~~

The existing effective map inflation is part of O_S:
ceil(0.099 / 0.1) * 0.1 = 0.1 m. No additional physical/body margin is
silently added. The body model remains center point plus scalar radius.

For final planner path p:[a,b] -> R3, validity against the same S means for
every w in [a,b]:

1. p(w) is finite and inside map/grid bounds;
2. the complete closed ball B_bar(p(w), 0.4) lies in the observed AABB and
   grid bounds; and
3. dist(p(w), O_S) >= 0.4 m.

This is a world-frame, full-3-D contract. Existing planner z bounds and
kinematic checks remain separate obligations. The legacy raw UAV radius,
uncertainty, and tracking fields are diagnostics-only and are not added to
this production inequality.

## 5. Continuous proof method

### 5.1 Sampling is not a proof

The current 115 trajectory samples, Kino primitive samples, and optimizer
control-point checks are not a continuous guarantee. A sampled-only validator
is not acceptable.

### 5.2 Position-only cell certificate

ContinuousPhasePath already has complete-cell differential certificates, but
its existing cellBounds requires positive speed for Tube normal-frame geometry.
That requirement is unrelated to centerline clearance and would reject valid
stationary endpoints. Add a separate position-only certificate/callback:

~~~
ContinuousPathPositionCellCertificate:
  w0, w1
  path_revision, segment_identity, segment_w0, segment_w1
  midpoint_position_variation_bound
  valid, complete
~~~

Mapped cubic B-splines, quintic C2 connectors, periodic references, and slices
must supply an outward-rounded complete-cell bound. Use the existing Bernstein
or analytic derivative controls and the existing UpperBound/nextafter
rounding convention. Existing Tube cellBounds remains untouched.

For a raw UniformBspline, validateSpline partitions the executable time domain
at every cubic knot span. Its derivative control polygon gives a complete
supremum V_C for that span by the B-spline convex-hull property, and the
position-only bound is B_C = UpperBound(0.5 * V_C * span_width). This direct
time-domain proof does not require a phase mapping or a positive-speed lower
bound.

### 5.3 Lipschitz certificate

For each closed cell C=[w0,w1] inside exactly one structural segment:

1. evaluate q=p((w0+w1)/2);
2. obtain B_C >= sup over C of ||p(w)-q||;
3. set R_C = 0.4 + B_C + epsilon_num, epsilon_num=1e-6 m;
4. query queryCloudOccupancySnapshotClearance(S,q,R_C); and
5. accept the cell only if status is KNOWN_FREE, clearance_certified is true,
   the result is finite, and clearance >= R_C.

Distance to a closed set is 1-Lipschitz:

~~~
dist(p(w),O_S) >= dist(q,O_S) - ||p(w)-q||
                 >= (0.4+B_C+epsilon_num)-B_C
                 >= 0.4.
~~~

The requested radius simultaneously proves the observed/grid domain contains
the complete 0.4 m ball around every point in the cell.

### 5.4 Bounded subdivision

Initial cells are each ContinuousPhasePath segment; no cell crosses a
structural boundary. A legacy UniformBspline is exposed through a temporary
mapped path over its complete executable domain.

For each cell first issue a canonical-radius probe at 0.4 m. If that probe is
not KNOWN_FREE, certified, finite, and observed, reject immediately. Then issue
the expanded probe at R_C. If the expanded probe proves the cell, accept it.
If the midpoint is canonically safe but the expanded probe is unknown or too
small, bisect at the representable midpoint and retry both probes on the
children. Any unsupported certificate, unknown canonical probe, out-of-map,
unavailable, nonfinite, or incomplete result rejects.

Fixed first-repair bounds:

~~~
max_subdivision_depth = 20
max_processed_cells   = 8192
max_clearance_queries = 4096
max_query_radius_m    = 0.8
minimum_cell_width_w  = 1e-8
numerical_epsilon_m   = 1e-6
~~~

Cells whose proof radius exceeds 0.8 m are split before querying. Use an
explicit stack, never unbounded recursion. Exhaustion rejects. If PASS is
returned, finitely many certified cells cover the complete path domain, so
the inequality in Section 4 holds for every path point against one S.

## 6. Installation seams and failure routing

The final gate is after final continuous geometry exists and before any
authority/mirror mutation.

1. gvf_manager::astaropt(): after final spline/time feasibility and before a
   successful return, validate the exact UniformBspline for legacy direct
   installation. Failure returns false.
2. installInitialPointPhaseFrontend(): after buildMappedPhaseFrontend(),
   capture one immutable snapshot, validate the final path, and pass that same
   pointer into bootstrap Tube preparation. Failure returns false before
   frontend/authority mutation.
3. installInitialClosedPhaseFrontend(): validate the nominal continuous path
   after construction and before bootstrap preparation/install.
4. commitNeutralPlannerFrontend(): immediately before existing authority
   retirement/commit, capture one snapshot and validate the final path.
   Failure returns false before locks and before pm.last_* mutation.
5. stageFutureSeamPathTubeTransaction(): after buildPhaseV2C2Frontend() and
   before stagePathTubePair(), validate trial_path against the supplied frozen
   snapshot. Failure leaves staged outputs empty. The existing neutral fallback
   remains independently guarded by commitNeutralPlannerFrontend().

These cover legacy direct assignments, point-phase initial/replan,
closed-phase initial/replan, mapped paths, and H2 staged replacements. No new
FSM state is added.

Initial validation failure reuses GEN_NEW_TRAJ retry. Replacement failure
reuses existing keep-current-frontend/EXEC_TRAJ behavior. Legacy
astaropt()==false reuses its current keep-old branch. A failed candidate is
never installed and then hidden by Tube [0,0].

## 7. Snapshot ownership

Every proof records the exact immutable shared pointer and:

~~~
observation_sequence
observation_stamp
resolution
included_map_inflation
~~~

Initial and H2 paths use the same pointer for validation and same-callback Tube
construction. Neutral replacement captures one pointer immediately before its
validation/commit. Legacy astaropt validates against one captured pointer
before returning success. No map lock is held during validation. Later map
evolution is handled by the existing collision/replan mechanism and does not
change the installation-time proof.

## 8. Search, optimizer, and Tube policy

~~~
KINO_SEARCH_CHANGE_REQUIRED=NO
BSPLINE_OPTIMIZER_CHANGE_REQUIRED=NO
TUBE_CROSS_SECTION_CHANGE_REQUIRED=NO
SURFACE_VALIDATOR_CHANGE_REQUIRED=NO
~~~

Kino and the ESDF optimizer remain candidate generators. A future heuristic or
optimizer alignment may improve candidate yield, but is not part of this
correctness whitelist.

Tube remains responsible only for additional transverse capacity. A canonically
valid centreline may still legitimately produce local [0,0]. Tube never vetoes
an already-installed planner path.

## 9. Performance bound

The captured initial candidate had 30 control points, about 27 cubic spans,
115 execution samples, and about 5.7 m path length. Expected validation work:

~~~
ordinary clear scene:       50-200 clearance queries
moderate proximity:         200-1000 clearance queries
hard near-threshold scene:  bounded at 4096 queries / 8192 cells, else reject
~~~

Each query is capped at 0.8 m and uses the snapshot occupied-column index when
available. Work is O(processed cells times bounded local voxel lookup).
Historical planning was about 6 ms; same-scene measurement is required before
any performance tuning. Correctness-first behavior on budget exhaustion is
rejection, not a weakened check.

## 10. Exact symbol map

| File/symbol | Responsibility | Caller/callee | State and context | Failure/owner |
|---|---|---|---|---|
| continuous_phase_path.h/.cpp: position certificate and positionCellBounds | Complete position-only bounds for mapped B-spline, C2, periodic and slices | validator -> evaluator callback | immutable path; no lock; pure computation | false/incomplete; path module owns geometry |
| new planner_clearance_validator.h/.cpp: validatePath/validateSpline | Iterative subdivision, Lipschitz proof, bounded accounting | manager -> path evaluator and plan_env query | immutable path/snapshot; no lock | explicit fail; planner owns validity |
| gvf_manager.cpp: astaropt | Gate legacy final spline | FSM -> validator | reads spline/snapshot/safe distance; no authority write | return false; exact legacy seam |
| gvf_manager.cpp: installInitialPointPhaseFrontend | Gate final mapped initial path | FSM -> validator -> existing bootstrap | snapshot captured before Tube prep | return false; existing retry |
| gvf_manager.cpp: installInitialClosedPhaseFrontend | Gate initial continuous path | FSM -> validator -> existing bootstrap | same as above | return false; existing retry |
| gvf_manager.cpp: commitNeutralPlannerFrontend | Gate direct replacement | replan -> validator -> current commit | validate before existing locks/mirror | return false; old path kept |
| gvf_manager.cpp: stageFutureSeamPathTubeTransaction | Gate C2 trial before Tube stage | replan -> validator -> stagePathTubePair | supplied frozen snapshot; handoff locks unchanged | false/empty; no pending invalid path |
| gvf_manager.h and CMakeLists.txt | declarations/build wiring | manager/library/tests | no runtime state | compile failure only; minimal package integration |

No validator runs while an authority mutex is held.

## 11. Future implementation whitelist

This is a possible future whitelist, not authorization.

Required production:

~~~
src/swarm_planner/bspline_traj/include/bspline_race/continuous_phase_path.h
src/swarm_planner/bspline_traj/src/continuous_phase_path.cpp
src/swarm_planner/bspline_traj/include/bspline_race/planner_clearance_validator.h
src/swarm_planner/bspline_traj/src/planner_clearance_validator.cpp
src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/CMakeLists.txt
~~~

Required tests:

~~~
src/swarm_planner/bspline_traj/test/planner_clearance_validator_test.cpp
src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp
~~~

Run unchanged:

~~~
src/swarm_planner/plan_env/test/cloud_occupancy_snapshot_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_cross_section_test.cpp
src/swarm_planner/bspline_traj/test/continuous_phase_path_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_integration_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
~~~

Deferred: Kinodynamic A*, bspline_opt_3d.cpp, plan_env production, Tube
production, launch/config, marker/display, D3/D4, Stage1B. Reusing the
plan_env primitive is safer than duplicating metric logic.

## 12. Deterministic tests

1. Snapshot-904 primary point must return 0.39910242275510055 and reject.
2. Snapshot-904 worst point must return 0.38956445853076954 and reject.
3. A path with minimum clearance strictly above 0.4 plus its proof bound must
   pass.
4. A cubic curve whose finite samples pass but whose interior lobe violates
   0.4 must fail continuous validation.
5. Missing/invalid/unknown/out-of-map/incomplete snapshots and work exhaustion
   must fail closed.
6. Existing valid planner path must still install.
7. A failed replacement must preserve old path owner, pm.last_*, authority
   session, and mailboxes.
8. Existing TubeCrossSection tests must retain below-0.4 -> [0,0].
9. Existing C1/C2/C3, Tube epoch, matched-adapter, and continuous-path tests
   must remain semantically unchanged.

Future implementation verification also requires build, all listed tests,
git diff --check, dependency-boundary inspection, and final whitelist audit.

## 13. Real ROS same-scene A/B

Use the existing pillar scene, launch, goals, map, and safety parameters.
Baseline A is 6549f2e. Repair B must record path revision, snapshot
sequence/stamp, validator result/reason, processed cells, query count, depth,
minimum certified slack, latency, and install/reject decision.

Repair B must prove every newly installed path has a complete PASS certificate
against its recorded immutable snapshot; the known metric-mismatch [0,0] cause
no longer occurs; navigation succeeds without a pathological replan loop;
commands/control remain healthy; Tube semantics remain unchanged; and
legitimate [0,0] remains possible where only transverse capacity is absent.

## 14. Stop conditions and display deferral

Stop if baseline/remote differs, any final authority path lacks a sound
position certificate, snapshot identity cannot be preserved, bounded
validation cannot complete, or any requested change expands into Tube,
Runtime, Recovery, Handoff, C1/C2/C3, SurfaceValidator, half-voxel,
marker-lifecycle, Stage1A, D3, or D4 work.

Future display-only enhancement:

~~~
represent mathematically legitimate [0,0] Tube sections as an explicit
centerline indicator rather than an apparently missing ribbon
~~~

No display change belongs in this repair.

## 15. Main independent plan review

| Question | Result |
|---|---|
| Q1 one canonical metric mathematically defined | PASS |
| Q2 planner owns planner-path validity | PASS |
| Q3 whole continuous path proven | PASS |
| Q4 environment observation coherent | PASS |
| Q5 all installation/replacement seams covered | PASS |
| Q6 TubeCrossSection unchanged | PASS |
| Q7 implementation scope minimal | PASS |
| Q8 existing failure semantics reused | PASS |
| Q9 bounded termination proven | PASS |
| Q10 unrelated stages/components closed | PASS |

~~~
PLAN_MODE=PLANNER_TUBE_CANONICAL_CLEARANCE_ALIGNMENT
MAIN_PLAN_REVIEW=PASS
IMPLEMENTATION_AUTHORIZED=false
LUNA_AUTHORIZED=false
~~~

This candidate now requires exactly one fresh read-only gpt-5.6-sol/max plan
audit. Sol acceptance means plan accepted only; implementation remains
unauthorized.
