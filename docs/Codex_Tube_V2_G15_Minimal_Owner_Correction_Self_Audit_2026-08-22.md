# G15 minimal owner correction self-audit (2026-08-22)

Final status: `PARTIALLY_FIXED_DYNAMIC_CONSTRUCTION_BLOCKED`.

## A — visibility ownership

- `phase_offset_matched_adapter.cpp/.h` and `phase_offset_tube_markers.cpp/.h` now require exact current task/control/source/map identity for marker ownership.
- Stale callbacks are no-op; current internal epoch mismatch fails certified closed and selects candidate geometry only from the current authoritative epoch.
- Candidate publication prefers the same epoch's contiguous `raw_build_samples` range containing the exact current anchor. Certified publication still uses only the active certified profile and strict `tubeDisplayCertified()`.
- New-task reset, inactive/manual exit, and shutdown use the existing lifecycle delete path.

## B — H2 owner reuse

- `CanonicalOwnerStateReuse` is transaction-local and validates owner pointer, exact double-bit phase, task/source/session/map identity, and prepared-sample provenance.
- Existing owner-membership, structural seam/cellBounds, Validator, profile postcheck, Runtime dry-run/future query, Pair/session/CAS, and live commit checks remain.
- No TubeFilter, limit, margin, query-budget, or Validator change.

## C — neutral planner liveness

- `stageFutureSeamPathTubeTransaction()` preserves a callback-local accepted planner frontend when Tube staging fails.
- Point/closed-loop replan branches offer it to the existing atomic `commitNeutralPlannerFrontend()` path; active/pending nonzero authority remains fail-closed.
- No retry, fallback gate, cache, queue, timer, watchdog, or Tube-success rewrite was added.

## Static evidence

- Marker 13/13, adapter 83/83, GVF switch 49/49, continuous path 11/11, TubeBuilder 9/9, CertifiedBuilder 11/11, SurfaceValidator 22/22, EpochManager 53/53, Runtime 35/35.
- Full catkin build succeeded: `/tmp/tube_v2_g15_static_20260822/catkin_make_full.log`.
- Epoch-integration binary retains four legacy cycle-99 fixed-source no-pair assertions under the current neutral/Pair contract; it is recorded at `/tmp/tube_v2_g15_static_20260822/epoch_integration.log` and is not treated as a G15 construction success.

## Dynamic evidence

The final independent split-launch run is:

```text
/tmp/tube_v2_g15_dynamic_20260822_021000/
```

It used a fresh private ROS master on port `13028`, fresh ROS_HOME/ROS_LOG_DIR,
and started the recorder before the simulator and planner publishers.
Readiness passed:

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

### Marker result

From `marker_summary.tsv` (2,427 messages per namespace):

- candidate: 7,206 ADD markers, 75 DELETE messages, no OTHER action;
  candidate ADD geometry was finite and contained 42 points per recorded
  marker batch.  The first candidate ADD was at
  `1787334667.0777786`, during real build attempts.
- certified: 0 ADD markers and 7,281 DELETE markers;
  no candidate geometry was promoted to `/tube`.

The final candidate ADD was at `1787334907.0799785`; the bag does not
independently contain a later shutdown DELETE for that final ADD, so a dynamic
shutdown-marker proof is not claimed.  The static lifecycle/shutdown tests
remain passing.

### Pair/H2 and construction result

The derived H2 timeline records bootstrap `TUBE_BUILD_PRECONDITION` failure at
`1787334664.685759783`, Pair generation 1/session 10 committed at
`1787334665.352633715`, nonzero delta execution at
`1787334665.388500452`, and a later point H2 `stage_success=0` at
`1787334666.212331772` (generation 3/session 10).  The subsequent exact-port
projector witness was invalid because the joint port polygon was empty, and
the governor entered the existing 36/0/36 path-end-clamped HOLD at
`1787334666.912158251`.  The old frontend was retained.  The callback-local
neutral frontend was not dynamically exercised because the run stopped at
this construction/H2 blocker.

`derived/parser_summary.tsv` reports 2,409 zero classifications: Z1=2 and
Z2=2,407.  The first full-width Z1 terminal reports `queries=249993` with
`max_queries=250000`; Z2 attempts commonly report `attempts=101` and
`insufficient_clearance`.  No query/depth/attempt limit was changed.

### Goal and cleanup result

`goal_harness.log` records G1 `(7.381, 0.378, 0)` reached at distance `0.183`.
G2 `(-6.823, -0.685, 0)` was published/received and then timed out after the
bounded wait.  G3--G5 were not sent because the harness waits for REACHED.

The private port was free after teardown, task-owned processes were absent from
the after-scan, and before/after binary plus launch/map hashes match.  No
external ROS process was touched.  The run therefore supports dynamic
visibility ownership and certified fail-closed behavior, but not full
navigation-liveness acceptance or a `FIXED` classification.
