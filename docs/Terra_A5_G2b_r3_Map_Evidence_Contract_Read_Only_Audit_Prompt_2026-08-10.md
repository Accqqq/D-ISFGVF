# A5-G2b-r3 Execution Specification: Read-Only Map Evidence Contract Audit

Date: 2026-08-10

Execution model: one `gpt-5.6-terra` agent at `reasoning_effort=max`.

`AUTO_ADVANCE=false`.

This document authorizes only A5-G2b-r3, a read-only architecture and evidence
audit. Determine which map representation can safely and proposal-consistently
support the phase-offset tube in this repository. Produce a decisive evidence-
backed recommendation and the minimum boundary of a later implementation
stage, then stop.

Do not modify repository source, tests, build files, launch files, parameters,
maps, or this specification. Do not implement the recommendation. Do not enter
A5-G3, A5.3, A6, A7, or multi-UAV work. Do not delegate to another agent.

## 1. Why this audit is required

A5-G1 established a proposal-consistent asymmetric cross-section kernel:

```text
planned ContinuousPhasePath
  -> lifted geometry p, T, N, signed curvature
  -> independent +N/-N environment clearance
  -> subtract r_eff exactly once
  -> intersect obstacle and curvature intervals
  -> candidate tube
```

The default robust radius remains:

```text
r_eff = 0.25 UAV radius
      + 0.10 map uncertainty
      + 0.05 localization uncertainty
      + 0.15 tracking error bound
      = 0.55 m
```

G2b selected the raw occupancy buffer so the environment tube would not be
capped by legacy `max_offset=0.20` or eroded twice by planner inflation.

G2b-r1 proved the current path center and actual position were raw UNKNOWN.
G2b-r2 added a strictly local and safe self-free seed over UNKNOWN inside the
actual UAV radius only. That correction worked, but the isolated pillar run
still produced no Candidate ADD:

```text
raw due rows                    46
raw storage/query injected      46/46
actual status                   KNOWN_FREE 46/46
base status                     KNOWN_FREE 43, UNKNOWN 3
current reason                  EMPTY_AFTER_OBSTACLE_BOUNDS 43
                                CENTER_UNKNOWN 3
c_plus                          0.00-0.25 m
c_minus                         0.00-0.30 m
r_eff                           0.55 m
Candidate ADD                   0
Candidate DELETE                214
```

Evidence directory:

```text
/tmp/a5g2b_r2_self_free_seed_20260810_KDY2Gt
```

The self-free seed must not be enlarged to `r_eff`: physical occupancy of the
vehicle certifies only its own radius and cannot fabricate sensor evidence for
map, localization, or tracking margins.

The unresolved architecture question is now:

> What exact map evidence should the tube consume so it is safe with UNKNOWN,
> avoids double-counting inflation, remains compatible with the proposal's
> latest-SDF semantics, and can actually certify a local cross-section under
> the repository's sensing pipeline?

This stage answers that question only.

## 2. Mandatory references

Read completely before analysis:

1. `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/AGENTS.md`
2. `/home/cxq/ISF-GVF/Paper/ICRA/PhaseOffsetSwarm_Detailed_Proposal_2026-08-05.md`
3. Proposal Sections 10.2-10.7, 13.5-13.6, 21, and 22.5 in context
4. `docs/PhaseOffsetSwarm_Code_Architecture_2026-08-08.md`
5. `docs/Terra_A5_G1_Asymmetric_Tube_Cross_Section_Kernel_Prompt_2026-08-09.md`
6. `docs/Terra_A5_G2a_Navigation_Asymmetric_Tube_Integration_Prompt_2026-08-09.md`
7. `docs/Terra_A5_G2b_Raw_Occupancy_ROS_And_Certified_Segment_Prompt_2026-08-09.md`
8. `docs/Terra_A5_G2b_r1_Raw_Candidate_Failure_Diagnostics_Prompt_2026-08-10.md`
9. `docs/Terra_A5_G2b_r2_Self_Free_Seed_Contract_Prompt_2026-08-10.md`
10. Current complete source and relevant tests for:
    - SDFMap initialization, raw log-odds update, raycast, reset/refresh;
    - obstacle inflation and manual/static layers;
    - ESDF generation and all public query functions;
    - SDFMap visualization publishers;
    - actual launch/remap/sensor inputs used by the pillar run;
    - raw occupancy bridge and self-free seed;
    - raw TubeCrossSectionSolver/TubeBuilder;
    - legacy distance bridge;
    - planner and B-spline map queries.
11. G2b, G2b-r1, and G2b-r2 bags/reports/analyzers/logs.

Do not use the old uncompiled prototype as authoritative evidence.

## 3. Precondition and preservation audit

Before any ROS or build command, record:

```bash
git branch --show-current
git rev-parse HEAD
git status --short
git diff --cached --name-only
git stash list
sha256sum docs/Terra_A5_G2b_r3_Map_Evidence_Contract_Read_Only_Audit_Prompt_2026-08-10.md
```

Required:

- branch `main`;
- HEAD `9a0e97560b8ddf15ac1ed1581030ed4b911f6e43`;
- prototype stash exists;
- no staged content;
- the dirty/untracked worktree and all completed A stages are user-owned.

Stop if these differ. Never reset, restore, clean, stash-pop, stage, commit,
branch, tag, or push.

Create a SHA-256 and mtime manifest for the entire repository excluding build,
devel, logs, Testing, `.git`, and temporary artifacts. The final manifest must
prove no repository source/document/config file changed during this audit.

## 4. Exact write boundary

No repository file may be added, removed, renamed, or modified.

All generated audit artifacts must live under a new directory equivalent to:

```text
/tmp/a5g2b_r3_map_evidence_audit_<unique>
```

Allowed temporary artifacts include:

- command outputs and source excerpts with line numbers;
- tables/CSV/JSON/Markdown reports;
- scripts used only to parse existing bags or ROS messages;
- rosbag files from one isolated read-only reproduction;
- launch/process/parameter/topic manifests;
- test logs.

Do not compile or load a repository plugin that changes runtime behavior. Do
not patch source and then restore it. Do not use GDB to mutate variables or
call non-const map update methods.

## 5. Required static dataflow trace

Produce a line-numbered trace from sensor input to every map representation.
The trace must answer each item explicitly.

### 5.1 Raw occupancy buffer

Document:

- initial log-odds value and the exact UNKNOWN threshold;
- hit/miss update and clamp behavior;
- whether ray endpoints and traversed voxels are updated differently;
- ray direction and whether the camera-origin voxel is guaranteed to be
  traversed;
- the effect of duplicate-ray early termination;
- actual use or non-use of `min_ray_length`;
- reset/refresh behavior for raw occupancy;
- exact semantics of `isUnknown`, `getOccupancy`, and `isKnownFree`;
- whether manual obstacles and static preinflated layers affect raw occupancy;
- whether raw UNKNOWN can persist next to the vehicle despite successful depth
  updates.

### 5.2 Inflated occupancy buffer

Document:

- exact source voxels used for inflation;
- configured `obstacles_inflation` in the actual pillar launch;
- resolution and computed `inf_step`;
- exact shape generated by `inflatePoint`;
- the distinction between requested metric inflation and realized voxel
  dilation;
- manual-map and static-preinflated contributions;
- reset/refresh behavior;
- whether UNKNOWN is represented separately in this buffer.

For the pillar defaults, do not merely state `0.099 m`. Derive the actual
voxel-index dilation at resolution `0.1 m` and explain why a scalar subtraction
of `0.099` cannot automatically invert a cubical grid dilation in every ray
direction.

### 5.3 ESDF/distance buffer

Document:

- the exact occupancy buffer used by `updateESDF3d`;
- how positive and negative distance transforms are formed;
- whether raw UNKNOWN and raw observed-free are distinguishable to the ESDF;
- what values remain outside the updated local bounds;
- whether the public `getDistance` query carries an observation-validity bit;
- whether the ESDF already includes planner/manual/static inflation;
- whether the existing legacy `esdf_unobserved_threshold` can reliably recover
  UNKNOWN inside the updated local region.

### 5.4 Published ROS map topics

Trace the actual backing buffer used by:

```text
/sdf_map/occupancy
/sdf_map/occupancy_inflate
/sdf_map/esdf
/sdf_map/unknown
/sdf_map/depth_cloud
```

Do not trust topic names. Report whether `/sdf_map/occupancy` really publishes
raw occupied voxels or another buffer. State which topics are actually called
from the visualization timer and which advertised topics remain silent.

### 5.5 Actual sensing geometry

Trace the exact SDFMap input selected by the pillar launch:

- depth image versus point cloud path;
- odometry/pose and camera frame/orientation;
- image intrinsics and effective horizontal/vertical FOV if derivable;
- depth filter minimum/maximum range, pixel skipping, and max ray length;
- whether the sensor provides omnidirectional, forward-frustum, or another
  coverage pattern;
- relation between the path normal `N` and the sensor viewing direction in the
  failing run, insofar as the existing bag/TF/odom evidence permits.

If an exact runtime angle cannot be recovered, say so and give only the proven
coverage limitation. Do not invent a 360-degree sensing claim.

### 5.6 Planner versus tube consumers

Create a table listing each consumer and its actual map query:

```text
kinodynamic A*
B-spline optimizer/collision check
C2/path installation checks, if any
legacy tube bridge
G2 raw tube bridge
manual-map behavior
```

For each, state whether it consumes raw, inflated occupancy, ESDF, manual
layer, and UNKNOWN validity.

## 6. Required evidence reconciliation

Reconcile source semantics with the three ROS diagnoses.

At minimum explain:

1. Why G2b-r1 returned CENTER_UNKNOWN at the base and actual position.
2. Why the r2 seed changed actual/current probes to KNOWN_FREE.
3. Why r2 ray clearance stopped around `0.20-0.30 m` rather than reaching
   `search_extent=3.0 m`.
4. Why subtracting `r_eff=0.55 m` then necessarily produces
   `lower_obstacle > upper_obstacle`.
5. Whether this proves a tube-kernel error, a sensing-evidence limitation, a
   map-representation mismatch, or a combination.
6. Whether the current pillar run actually contains enough observed-free
   evidence to certify a 0.55 m robust lateral clearance on either side.

Use the exact r2 current facts:

```text
c_plus  = 0.00-0.25 m
c_minus = 0.00-0.30 m
r_eff   = 0.55 m
```

Do not reinterpret UNKNOWN as an obstacle location. It is an evidence boundary.

## 7. Optional single isolated ROS observation

Run at most one fresh isolated ROS observation only if it materially resolves a
question not settled by source and existing bags. Do not attach to user port
`11311` or any existing ROS graph.

If run, use:

```text
original pillar.pcd
same phase_offset_esdf_tube_single.launch
same goal (8,0,1) only if motion is needed
refresh 3.0
all current parameters unchanged
fresh isolated ROS port
```

The run is read-only with respect to behavior. Record only existing topics,
including available map/depth/TF/odom/raw diagnostics topics. Do not change
launch arguments to manufacture coverage. Do not add diagnostic publishers.

The analyzer may compare point sets from `/sdf_map/occupancy` and
`/sdf_map/occupancy_inflate`, inspect PointCloud2 fields, publication rates,
TF/camera orientation, and correlate with raw candidate facts. It must state
what cannot be inferred because raw free/unknown state is not publicly exposed.

Clean up only this audit's processes and verify the isolated port is released.

## 8. Candidate map-evidence contracts to evaluate

Evaluate all of the following; do not skip directly to a preferred answer.

### Contract A: current raw occupancy raycast

Assess:

- correct fail-closed UNKNOWN semantics;
- no planner-inflation double counting;
- current sensor coverage and observed-free continuity;
- whether it can satisfy the proposal under the present sensor model;
- whether self-free seed r2 should be retained as a valid local correction.

### Contract B: current inflated ESDF directly

Assess:

- inflation already present;
- UNKNOWN loss;
- manual/static layer inclusion;
- double-counting if full `r_eff` is subtracted;
- whether adding/subtracting `0.099` is geometrically exact or safe.

### Contract C: current inflated ESDF plus raw-knownness mask

Assess whether combining scalar distance with pointwise raw-knownness actually
solves ray-continuity and preview certification, or merely rejects the same
lateral UNKNOWN boundary. Account for cubical grid inflation and manual layers.

### Contract D: observed-aware uninflated distance field

Assess a later map-layer interface that returns both:

```text
status: UNAVAILABLE / OUT_OF_MAP / UNKNOWN / OCCUPIED / KNOWN_FREE
distance to uninflated environmental occupancy when status is known
```

Determine whether this is the cleanest way to subtract `r_eff` exactly once,
and what sensing coverage is still required. Distinguish the abstract contract
from any concrete SDFMap implementation.

### Contract E: globally known/static simulation map

Assess use of the full pillar map as authoritative static free/occupied evidence
in simulation only. State whether it is acceptable as a test mode, whether it
matches the final local-SDF research claim, and how it must be isolated from
production/local-perception semantics.

### Contract F: change sensor coverage

Assess whether an omnidirectional or otherwise sufficient local sensor is a
scenario prerequisite for a two-sided tube. Do not authorize a sensor change;
state only the geometric evidence requirement.

## 9. Mandatory decision matrix

Produce a compact table with these columns:

```text
contract
UNKNOWN safe?
double erosion avoided?
manual/static obstacles represented?
works with current sensor evidence?
proposal-consistent production use?
required future code boundary
verdict
```

Verdict must be one of:

```text
RETAIN
REJECT
TEST-ONLY
REQUIRES-NEW-STAGE
```

Then give one primary production recommendation and, if justified, one
separate simulation-only recommendation. Do not give several equal options.

## 10. Required architecture conclusion

The final conclusion must explicitly answer:

1. Should G2b-r2 self-free seed be retained, reverted, or superseded? Why?
2. Should the tube continue using current raw occupancy as its sole production
   evidence source?
3. Can the current `distance_buffer_all_` be used safely without changing its
   data contract?
4. Is subtracting or adding back `0.099 m` theoretically and discretely valid?
5. Is a new observed-aware uninflated distance interface required?
6. Does the current forward/local sensor provide enough evidence for a
   two-sided `r_eff=0.55 m` tube in this scenario?
7. Is the absence of Candidate ADD currently the correct fail-closed outcome?
8. What is the smallest next implementation stage, if any?

The recommendation must preserve:

- planned path/lifted geometry before tube expansion;
- asymmetric `+N/-N` clearance;
- one-time robust erosion;
- UNKNOWN fail-closed semantics;
- candidate/active epoch separation;
- baseline and current control invariants.

## 11. Next-stage boundary proposal

If a later implementation is recommended, define but do not execute:

- stage name and one-sentence goal;
- exact packages/files that would need authorization;
- new abstract query/data type, if any;
- whether SDFMap must change;
- whether sensor/simulation configuration must change;
- compatibility plan for planner and baseline;
- deterministic unit tests;
- isolated ROS acceptance criteria;
- stop conditions.

Keep the proposed whitelist minimal. Do not authorize planner tuning, speed
changes, larger seeds, smaller `r_eff`, modified tube mathematics, or control
changes.

If source evidence shows that a safe production tube cannot be demonstrated
under the current sensor coverage, say so directly. Do not write an execution
specification that promises Candidate ADD by weakening safety.

## 12. Regression and no-change verification

No build is required because repository code must not change. Verify the
current compiled state with:

```bash
catkin_test_results --verbose
devel/lib/bspline_race/phase_offset_raw_occupancy_query_test
devel/lib/bspline_race/phase_offset_matched_adapter_test
```

Do not rebuild unless the existing binaries are missing. A rebuild does not
authorize source changes.

Before stopping:

```bash
git diff --check
git diff --cached --name-only
git status --short
git branch --show-current
git rev-parse HEAD
git stash list
```

Compare the full pre/post repository SHA and mtime manifests. Any unexpected
repository change is a stop condition and must be reported without repair.

## 13. Forbidden actions

Do not:

- modify SDFMap, raw bridge, adapter, navigation, Runtime, Marker, launch,
  planner, C2, optimizer, control, simulator, map, or tests;
- enlarge the self-free seed;
- reduce `r_eff` or any uncertainty term;
- reinterpret UNKNOWN as free;
- use existing inflated ESDF as a safety certificate without accounting for
  UNKNOWN and exact grid inflation;
- claim `0.099 m` is exactly reversible without proof;
- change sensor FOV, map source, goal, speed, refresh, or planner weights;
- add a new publisher/topic or temporary repository instrumentation;
- treat `/sdf_map/occupancy` topic naming as proof of raw content;
- infer raw free/unknown from an occupied-only point cloud;
- implement the next stage;
- commit, stage, branch, tag, or push.

## 14. Stop conditions

Stop and report without expansion if:

- branch/HEAD/stash/staged preconditions differ;
- a repository file changes;
- source evidence is sufficient and another ROS run would add no material fact;
- ROS cannot be safely isolated;
- the public topics cannot expose raw free/unknown state;
- an unrelated historical failure blocks reporting.

Lack of a safe Candidate tube under current sensing is an acceptable audit
conclusion, not permission to weaken safety.

## 15. Required final artifacts and report

Write under the `/tmp` evidence directory:

```text
map_evidence_trace.md
consumer_matrix.csv
contract_decision_matrix.csv
architecture_recommendation.md
next_stage_boundary.md
source_line_manifest.txt
pre_post_integrity.txt
```

If ROS is run, also retain bag, analyzer, parameters, process manifests, and
SHA-256 values.

The final response must state:

1. whether the repository remained byte-for-byte unchanged;
2. exact raw/inflated/ESDF dataflow;
3. exact UNKNOWN handling in each representation;
4. actual voxel inflation induced by `0.099 m` at `0.1 m` resolution;
5. actual sensing coverage evidence and its relation to the failed normal rays;
6. reconciliation of r1/r2 numeric facts;
7. complete decision matrix verdicts;
8. one primary production recommendation;
9. whether the r2 seed should remain;
10. smallest proposed next-stage boundary;
11. evidence paths and hashes;
12. git/HEAD/stash/process/test audit;
13. explicit statement that no implementation or parameter change occurred;
14. explicit stop statement.

A5 remains blocked after A5-G2b-r3. Any implementation requires a new dedicated
execution specification and explicit authorization.
