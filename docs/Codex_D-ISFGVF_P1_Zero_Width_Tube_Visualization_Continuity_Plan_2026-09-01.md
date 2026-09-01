# D-ISFGVF P1 Zero-Width Tube Visualization Continuity Plan

    DOCUMENT_ROLE=PLAN_ONLY
    DOCUMENT_STATUS=STOPPED_AFTER_FRESH_SOL_MAX_BLOCKER
    DATE=2026-09-01
    REPOSITORY=/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
    BASELINE=6549f2eb238ff17790e92069adb087b7bb5024e6
    BRANCH=main
    AUTHORITATIVE_REMOTE=d_isfgvf/main
    IMPLEMENTATION_AUTHORIZED=false
    LUNA_AUTHORIZED=false

## 1. Scope and hard stop

This document defines one display-only P1 repair. It does not authorize
implementation, source or test edits, launch/config edits, parameter tuning,
staging, commit, push, Luna, D3, D4, Stage1A, Stage1B, C1/C2/C3, planner work,
Tube geometry work, Runtime/Recovery/Handoff work, or marker-lifecycle
redesign.

The frozen liveness invariant is:

    A planner-valid path remains navigable regardless of local ZERO_ONLY Tube,
    Tube construction/certificate failure, or visualization/diagnostic failure.

Only `PLANNER_INVALID` or explicit `CURRENT_STATE_UNSAFE` may own a true stop
or replan. This plan creates no such decision and cannot convert a Tube display
fact into one.

## 2. Frozen diagnosis and semantic interpretation

The user-visible gap is on:

    /formation_planning/phase_offset_manual/tube_candidate

at:

    w=[2.679822524195818,2.8782996958332387]

The five raw-build Tube samples are complete and finite, but their displayed
bounds are exactly `[0,0]`. Closed-volume clearance below
`planning/safe_distance=0.4 m` correctly produces:

    ClearanceSafe=false
      -> EMPTY_AFTER_OBSTACLE_BOUNDS
      -> SetZeroOnly()
      -> valid complete [0,0]

Therefore the missing ribbon area is not publication loss. It is the expected
degeneration of a triangle ribbon whose lower and upper vertices coincide.
Marker publication, marker lifecycle, SurfaceValidator, half-voxel behavior,
Stage1A, and sample completeness are not the root cause.

The governing interpretation remains:

    planner = authority for centerline navigation validity
    Tube    = authority for additional transverse PhaseOffset capacity

Thus planner-valid plus local `[0,0]` means continued centerline navigation
with no nonzero transverse offset capacity at that location. `[0,0]` remains a
valid Tube state and is not repaired, widened, reclassified, or rejected.

## 3. Current source facts

### 3.1 Marker construction

`phase_offset_tube_markers.cpp` currently constructs, per Tube topic namespace:

    ID 0: lower LINE_STRIP
    ID 1: upper LINE_STRIP
    ID 2: TRIANGLE_LIST ribbon

`AppendBoundaryPoints()` uses `sample.p + sample.N * filtered_bound`. At an
exact `[0,0]` knot, both boundary vertices equal `sample.p`; triangles spanning
an all-zero cell have zero area and can appear as a visual gap.

The existing invalid/not-qualified result is an all-DELETE bundle for IDs
0..2. Marker lifetime is zero, so every new stable marker key must be actively
updated or deleted to prevent stale geometry.

### 3.2 Topic sources and qualification

`tube_candidate`:

- source: the immutable Candidate epoch's `raw_build_samples` when present,
  otherwise the complete profile samples used by the existing fallback;
- qualification: MANUAL mode plus the unchanged exact-current-anchor and
  contiguous-displayable-sample selection in `SelectCandidateDisplaySamples()`;
- current geometry fields: the selected samples' `filtered_lower` and
  `filtered_upper`. In Builder-produced `raw_build_samples`, these are captured
  equal to the raw-build bounds before Filter. The repair must not switch this
  topic to a different field.

`tube_certified_geometry`:

- source: immutable Candidate `profile.samples`;
- qualification: the unchanged atomic request/Candidate/request protocol,
  `certifiedGeometryCandidateEligible()`, final request identity reread, the
  caller's `displayable` value, and
  `CertifiedGeometryProfileDisplayable()`;
- current geometry fields: filtered bounds only;
- an entirely `ZERO_ONLY_PLANNER_BASELINE` Candidate remains ineligible under
  the existing `OFFSET_CERTIFIED && !zero_only` rule. This plan does not make
  that topic visible. It only exposes local exact-zero knots inside a profile
  that already passes the topic's existing eligibility.

`/tube`:

- source: the immutable Active profile already selected by the adapter;
- qualification: the unchanged `tubeDisplayCertified(output)` result plus the
  existing complete finite marker-geometry checks;
- current geometry fields: filtered Active bounds only;
- if execution qualification says DELETE, both the existing ribbon and the
  new passive zero indicator say DELETE.

The production adapter already publishes one `MarkerArray` per topic. That
interface can carry the two additional markers without an adapter production
change or a second publication pipeline.

### 3.3 Cohort and component evidence

`TubeProfile` carries source/tube/path/frame/map/profile revisions and the
profile-level selected component. Each `TubeRawSample` carries path and frame
revisions. There is no per-sample component ID.

The existing source establishes the usable topology boundary:

- Candidate selection retains one maximal displayable raw-build sequence
  around the exact current anchor;
- Builder/Filter retain the current-containing profile segment;
- certified-geometry qualification requires one strictly increasing,
  revision-consistent sample sequence.

The visualization must not invent a second component model. It may connect
only adjacent samples inside the already selected vector/profile and must
break on any failed zero predicate, nonzero sample, non-increasing phase, or
revision mismatch.

## 4. Frozen display design

### 4.1 Existing markers remain unchanged

For a qualified topic, IDs 0, 1, and 2 retain their current namespace, type,
scale, color, alpha, point ordering, and geometry byte-for-byte. In particular,
the existing ribbon is neither clipped around zero knots nor replaced by a
different primitive.

### 4.2 Two additional passive markers

Each qualified topic's existing MarkerArray gains exactly two markers:

    zero namespace = <existing-topic-namespace> + "_zero_centerline"

    ID 3:
      type   = LINE_LIST
      meaning= all adjacent segments of maximal multi-knot exact-zero runs
      scale.x= 0.08 m glyph stroke

    ID 4:
      type   = POINTS
      meaning= all maximal exact-zero runs containing exactly one knot
      scale.x=scale.y=scale.z=0.12 m glyph size

    both:
      pose.orientation.w=1
      color=(r=1.0,g=0.85,b=0.0,a=1.0)
      lifetime=0, matching the existing persistent marker convention

The amber stroke/point size is an RViz glyph property only. It is not Tube
width, clearance, capacity, erosion, or a controller limit. Every stored
vertex is an exact `sample.p`; no `sample.p +/- epsilon*N`, synthetic ribbon,
or nonzero bound is created.

For a qualified profile, ID 3 is ADD only when it has at least one multi-knot
run; otherwise ID 3 is an explicit DELETE. Likewise ID 4 is ADD only when it
has at least one singleton run; otherwise ID 4 is an explicit DELETE. This
avoids relying on RViz's handling of an empty ADD marker and makes a transition
from an old run to no run deterministic. Invalid, incomplete, absent, or
not-qualified topics emit an all-DELETE bundle for the existing keys and the
two new keys:

    base namespace IDs 0,1,2 = DELETE
    zero namespace IDs 3,4   = DELETE

There are no dynamic per-run IDs. A changing number of runs therefore cannot
leave stale lifetime-zero markers.

### 4.3 Exact zero-width predicate

A sample contributes to the zero overlay only when all of the following are
true:

1. it belongs to the exact sample vector already selected for that topic;
2. `sample.complete` is true;
3. `sample.w`, `sample.p`, and `sample.N` are finite;
4. the topic-owned displayed lower and upper bounds are finite and ordered;
5. `profile.path_revision` and `profile.frame_revision` are nonzero;
6. the sample path/frame revisions exactly equal the containing profile's
   path/frame revisions;
7. displayed lower is exactly `0.0` and displayed upper is exactly `0.0`.

For all three topics, "displayed bounds" means the same
`filtered_lower/filtered_upper` fields used by the existing marker geometry.
For Candidate these fields come from the selected raw-build sample and are the
Builder-captured raw-build display interval; for the other topics they are the
filtered profile interval.

Exact IEEE equality is intentional. `-0.0 == 0.0` is accepted. No tolerance is
permitted because a tolerance could label small certified nonzero capacity as
zero or visually erase it. Profile classification, `zero_only`, stop reason,
and diagnostics are not substitutes for this local geometry predicate.

The predicate does not decide whether a topic is current or eligible. Outer
topic qualification remains the sole currentness/eligibility owner; the
sample/profile revision checks only prevent malformed or cross-revision
geometry inside an already supplied profile from being called valid zero
geometry.

### 4.4 Maximal-run algorithm

For the exact vector already used by the existing ribbon builder:

1. scan once in original order;
2. start a run at a sample satisfying the zero predicate;
3. extend only to the immediately adjacent sample when it also satisfies the
   predicate and `next.w > current.w`;
4. terminate on a nonzero, invalid, incomplete, malformed, revision-mismatched,
   or non-increasing sample;
5. for a run of `n >= 2`, append `(p_i,p_{i+1})` for every adjacent pair to
   the shared ID-3 LINE_LIST;
6. for a run of `n == 1`, append that exact `p_i` once to ID-4 POINTS.

LINE_LIST rather than one LINE_STRIP is required: separate runs stored in one
marker cannot be accidentally connected. A singleton receives a POINTS glyph
because a one-vertex line is not reliably visible; duplicating it as a
zero-length line would obscure the intended topology.

The algorithm never searches for a nearby sample, fills a missing knot,
queries the path, interpolates across a separator, or joins separate selected
vectors. Straight segments between adjacent vertices are the same sampled
polyline convention already used by the Tube boundary markers. With no
per-sample component ID in the model, the already selected profile/vector is
the outer component boundary; the visualization does not infer more.

## 5. Topic-by-topic output contract

| Topic | Existing outer qualification | Zero source | Qualified output | Failed qualification |
|---|---|---|---|---|
| `phase_offset_manual/tube_candidate` | MANUAL plus unchanged exact-anchor `SelectCandidateDisplaySamples()` | selected raw-build display samples; fallback remains exactly as today | existing IDs 0..2 ADD unchanged; each zero ID is ADD only when its matching run class exists, otherwise explicit DELETE | all five stable keys DELETE |
| `phase_offset_manual/tube_certified_geometry` | unchanged request/Candidate identity, provenance, OFFSET_CERTIFIED/non-zero-only policy, caller `displayable`, and geometry predicate | Candidate `profile.samples`, filtered bounds | same fixed five-key qualified contract | all five stable keys DELETE |
| `phase_offset_manual/tube` | unchanged `tubeDisplayCertified(output)` and existing profile geometry checks | Active `profile.samples`, filtered bounds | same fixed five-key qualified contract | all five stable keys DELETE |

No zero marker may cause a previously failed topic qualification to become
ADD. No data may be borrowed from Candidate for `/tube`, from Active/Pair for
certified geometry, or from a later command phase for Candidate.

## 6. Navigation non-interference proof

The future production edit is confined to the existing stateless marker
translation unit. The added logic:

- accepts the same `const TubeProfile&` and already selected immutable sample
  vectors;
- reads only sample geometry, bounds, and revision values;
- writes only local `visualization_msgs::Marker` and `MarkerArray` objects;
- returns through the same three existing publishers;
- performs one bounded O(n) scan, with n no greater than the sample vector
  already scanned by existing ribbon construction;
- performs no map/environment query, Tube rebuild, path query, ROS parameter
  access, service call, file I/O, logging, retry, sleep, or asynchronous work;
- adds no mutex and takes no existing mutex;
- creates no publisher, subscriber, callback, worker, queue, timer, or thread.

It does not write or call into planner, Runtime, Recovery, Handoff,
ExecutionAuthority, allocator, selected-u, TubeBuilder, TubeFilter,
TubeSurfaceValidator, TubeCrossSectionSolver, Candidate/Epoch, PathTubePair,
session, task, source, or currentness state.

Production marker construction remains on the existing passive publication
path from already captured immutable inputs. ROS publication remains on the
existing topics and queues. Therefore the overlay cannot reject or retire a
path, change a Tube, cause HOLD, schedule replan, deny authority, prevent a
position command, or wait on a command/planner critical lock. Marker
allocation/publication failure has exactly its existing consequence: missing
diagnostic visualization, never navigation invalidation.

## 7. Exact implementation symbol map

Future implementation, if separately authorized, is limited to:

`phase_offset_tube_markers.cpp`:

- retain `MakeTubeLine()`, `MakeTubeRibbon()`, `AppendBoundaryPoints()`,
  `SelectCandidateDisplaySamples()`, and all outer qualifications unchanged;
- add internal constructors for the fixed ID-3 LINE_LIST and ID-4 POINTS;
- add one internal sample predicate parameterized by the containing profile
  and its already selected sample vector;
- add one internal maximal-run appender;
- extend `MakeDeleteAll()` to emit base namespace IDs 0..2 and matching zero
  namespace IDs 3..4;
- append the two zero markers only after existing geometry construction has
  succeeded, preserving the all-DELETE failure path.

`phase_offset_tube_markers_test.cpp`:

- update bundle helpers for the five stable keys and topic-specific
  namespaces;
- add Tests A-J below.

`phase_offset_matched_adapter_test.cpp`:

- update only marker-bundle assertions/helpers that hard-code a three-marker
  bundle or assume every marker uses the base namespace;
- retain all existing qualification/currentness scenarios;
- add no production hook and authorize no adapter behavior change.

`phase_offset_tube_epoch_integration_test.cpp`:

- update only `ExpectThreeActions()` and any direct three-marker-count or
  namespace assertions affected by the fixed ID-3/ID-4 extension;
- retain all epoch construction, publication, qualification, and liveness
  assertions unchanged;
- add no production hook and authorize no adapter behavior change.

No public declaration changes are needed in
`phase_offset_tube_markers.h`. No production edit is needed in
`phase_offset_matched_adapter.cpp` or `.h`: the existing MarkerArray and call
sites already transport the additional passive markers.

## 8. Exact future implementation whitelist

Only these paths may be modified by a separately authorized implementation:

1. `src/swarm_planner/bspline_traj/src/integration/phase_offset_tube_markers.cpp`
2. `src/swarm_planner/bspline_traj/test/phase_offset_tube_markers_test.cpp`
3. `src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp`
4. `src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_integration_test.cpp`

The third and fourth paths are required because their shared marker assertion
helpers currently hard-code the three-marker bundle. They are test
compatibility scope only, not adapter seams.

Any required production change outside path 1 is a plan violation and must
stop implementation for a new Main decision.

## 9. Deterministic future tests

### Test A — nonzero baseline preservation

For an all-nonzero qualified profile, compare IDs 0..2 against the frozen
baseline for type, action, namespace, ID, scale, color, alpha, and every point.
They must be unchanged. IDs 3 and 4 must be explicit DELETE markers because
this qualified profile contains no zero run.

### Test B — internal three-knot zero run

Use ordered samples:

    nonzero, [0,0], [0,0], [0,0], nonzero

IDs 0..2 retain existing geometry. ID 3 contains exactly four vertices:

    p1,p2,p2,p3

ID 4 is an explicit DELETE. No artificial width is present.

### Test C — exact coordinates

Use non-axis-aligned, non-integer `p` and `N`. Every zero-overlay coordinate
must equal the corresponding `sample.p` with exact double equality and must be
independent of `N`.

### Test D — invalid zero-looking samples

Cover incomplete, nonfinite w/p/N/bound, inverted bounds, zero/mismatched
path revision, and zero/mismatched frame revision. Such samples never
contribute a line endpoint or point. Where existing outer qualification fails,
the complete five-key bundle is DELETE; where baseline qualification still
allows the base ribbon, only the valid zero runs contribute to the overlay.

### Test E — topology and separate runs

Use at least two zero runs separated in turn by a nonzero sample, an invalid
sample, a revision mismatch, and non-increasing w. Assert ID 3 contains only
adjacent pairs within each valid run and no pair whose endpoints straddle a
separator. Assert no LINE_STRIP is used.

### Test F — DELETE completeness

For Candidate manual=false/exact-anchor failure, certified=false,
certified-geometry displayable=false, absent source, incomplete geometry, and
malformed geometry, assert DELETE for base IDs 0..2 and zero IDs 3..4 with the
exact matching namespaces.

### Test G — qualification equivalence

Run the existing Candidate exact-anchor/current-epoch tests,
certified-geometry request/Candidate/revision tests, and `/tube` execution
qualification tests without changing their input decisions. For every case,
the baseline IDs 0..2 retain their previous ADD/DELETE action. The new IDs
must never turn a baseline DELETE into ADD geometry.

### Test H — Tube state non-mutation

Pass a profile containing raw, filtered, classification, proof, stop, revision,
and diagnostics fields; snapshot it before marker construction and compare it
afterward. No field or vector changes. The function's const API and absence of
navigation-source edits provide the structural proof.

### Test I — stable-key lifecycle transition

Publish/build messages in sequence:

    qualified zero run -> qualified nonzero -> invalid/absent

The first message has nonempty ID 3/4 as appropriate; the second explicitly
DELETEs both stable zero keys; the third also DELETEs both keys as part of the
full invalid bundle. No dynamic ID exists and no old run can persist.

### Test J — singleton policy

For `nonzero,[0,0],nonzero`, ID 3 is an explicit DELETE and ID 4 contains
exactly the one zero sample's exact `p`. Multiple singleton runs appear as
separate POINTS entries and are not connected. This test deliberately retains
the existing outer qualification; a Candidate vector that consists only of a
single current raw sample still follows the baseline DELETE decision.

## 10. Build and regression plan for future implementation

A separately authorized implementation must run:

1. the `phase_offset_tube_markers_test` target;
2. the complete `phase_offset_matched_adapter_test` target;
3. the complete `phase_offset_tube_epoch_integration_test` target;
4. the existing bspline_traj test set affected by the marker library;
5. `git diff --check`;
6. a whitelist audit proving no production path except the marker `.cpp`
   changed;
7. final staged-path count and worktree status without cleaning user-owned
   files.

No test may be made to pass by changing planner/Tube qualification, replacing
exact zero with a tolerance, or weakening an existing DELETE.

## 11. Same-scene ROS acceptance plan

Reuse the pillar scenario and the frozen gap:

    GAP_W_START=2.679822524195818
    GAP_W_END=2.8782996958332387

Capture before/after bags or equivalent deterministic evidence from the same
scene and configuration. Acceptance requires:

1. identical planner path identity/geometry and goal result;
2. identical five raw Tube knots and exact `[0,0]` bounds;
3. identical Tube classifications, stop reasons, safety/clearance facts, and
   Candidate/Active qualification decisions;
4. no new HOLD, replan, path retirement, authority denial, command gap, or
   failure latch;
5. unchanged IDs 0..2 on each qualified topic;
6. on `tube_candidate`, an amber centerline overlay spanning the five-knot
   exact-zero run, with eight LINE_LIST vertices corresponding exactly to its
   four adjacent pairs;
7. on certified-geometry and `/tube`, the same overlay only when each topic's
   unchanged qualification independently says ADD; their existing DELETE is
   acceptable and must not be overridden;
8. after moving to a qualified nonzero profile, the same zero marker keys
   receive explicit DELETE; after invalidation/absence they remain covered by
   the full DELETE bundle;
9. RViz interpretation visibly distinguishes nonzero ribbon, valid zero-width
   centerline indicator, and invalid/no-Tube absence.

The success criterion is:

    SAME_GEOMETRY + SAME_SAFETY + SAME_NAVIGATION + BETTER_VISUAL_SEMANTICS

It is explicitly not `WIDER_TUBE`.

## 12. Protected scope

The future repair must not change:

- Kinodynamic A*, B-spline optimizer, ESDF, map representation, planner
  margins, safe-distance values, planner gates, retry behavior, HOLD/replan;
- TubeRawSample bounds, TubeCrossSectionSolver, `SetZeroOnly()`, TubeBuilder,
  TubeFilter, TubeSurfaceValidator, TubeProfile classification, ZERO_ONLY;
- Candidate, certified-geometry, or execution eligibility/currentness;
- Runtime, Recovery, Handoff, allocator, selected-u, ExecutionAuthority,
  Pair/session/task/source ownership;
- publisher advertisement, queue sizes, callbacks, timers, locks, worker
  scheduling, marker lifetime, or prior marker-lifecycle protocol;
- Stage1A, Stage1B, C1/C2/C3, D3, D4.

## 13. Main Q1-Q10 review

Q1. Does the Plan leave planner code and planner semantics completely
unchanged?

    YES. No planner path is in the whitelist or called by the design.

Q2. Can Tube `[0,0]` still occur exactly as before?

    YES. The existing bounds are read const and rendered without mutation.

Q3. Can the visualization change ever create nonzero certified capacity?

    NO. It writes only exact sample.p vertices to Marker messages; glyph scale
    is not Tube geometry.

Q4. Can it veto or delay navigation?

    NO. It adds no decision, lock, query, worker, or control/planner call and
    remains a bounded pass in the existing passive publication path.

Q5. Can it trigger HOLD or replan?

    NO. It has no dependency or write path to either behavior.

Q6. Does it remain passive diagnostic/visualization only?

    YES. The only production object modified is a local MarkerArray.

Q7. Does it preserve topic-specific qualification?

    YES. Every current qualification function and call site is unchanged;
    zero markers are appended only after existing geometry succeeds.

Q8. Does it avoid reintroducing the old marker-lifecycle problem?

    YES. Two fixed namespace/ID keys are updated on every qualified bundle and
    explicitly deleted on every failed bundle; there are no dynamic run IDs or
    second publisher.

Q9. Is the whitelist minimal?

    YES after the recorded Sol correction. One production marker translation
    unit plus its direct test and the two existing integration tests whose
    helpers hard-code the old three-marker bundle. No production adapter or
    public-header change is needed.

Q10. Does it solve the user's actual visual symptom?

    YES. The proven five-knot local exact-zero interval becomes an explicit
    centerline run while the same degenerate ribbon, Tube, and navigation
    semantics remain intact.

    MAIN_PLAN_REVIEW=PASS

## 14. Audit and stop boundary

After this Main review, exactly one fresh `gpt-5.6-sol` with `reasoning=max`
must audit this plan read-only. Its first question is the mandated navigation
liveness question. Any blocker, any major, a nonzero answer to that question,
or a verdict other than the exact acceptable verdict stops the plan as not
accepted. There is no second Sol audit in this task.

The one fresh Sol audit answered the mandated first question NO and found no
navigation, geometry, qualification, or lifecycle defect, but found one
whitelist blocker: `phase_offset_tube_epoch_integration_test.cpp` also asserts
three-marker bundles and is an affected CMake test target. The plan therefore
stops as not accepted after adding that path to this candidate's corrective
whitelist. No second Sol is permitted.

Sol PASS would accept this plan only. It does not authorize implementation or
Luna.

    SOL_PLAN_VERDICT=P1_ZERO_WIDTH_TUBE_VISUALIZATION_CONTINUITY_PLAN_NOT_ACCEPTABLE
    SOL_BLOCKER_COUNT=1
    SOL_MAJOR_COUNT=0
    PLAN_STATUS=NOT_ACCEPTED

## 15. Current-task state

    PLAN_MODE=P1_ZERO_WIDTH_TUBE_VISUALIZATION_CONTINUITY

    PLANNER_CHANGED=false
    PLANNER_VALIDITY_CHANGED=false
    TUBE_GEOMETRY_CHANGED=false
    TUBE_SAFETY_CHANGED=false
    ZERO_ONLY_SEMANTICS_CHANGED=false

    NAVIGATION_LIVENESS_PRESERVED=YES

    VISUAL_FIX_TYPE=stable per-topic amber exact-sample.p zero-centerline LINE_LIST (ID 3) plus singleton POINTS (ID 4), with existing ribbon unchanged

    IMPLEMENTATION_WHITELIST=phase_offset_tube_markers.cpp; phase_offset_tube_markers_test.cpp; phase_offset_matched_adapter_test.cpp; phase_offset_tube_epoch_integration_test.cpp

    MAIN_PLAN_REVIEW=PASS

    IMPLEMENTATION_AUTHORIZED=false
    LUNA_AUTHORIZED=false

    CODE_MODIFIED=false
    STAGED_PATHS=0
    COMMIT_PUSH=false
