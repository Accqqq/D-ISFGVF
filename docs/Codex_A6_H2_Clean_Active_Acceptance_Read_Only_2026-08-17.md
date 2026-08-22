# A6/H2 clean active acceptance — read-only result

```text
DOCUMENT_ROLE=READ_ONLY_DYNAMIC_ACCEPTANCE
DATE=2026-08-17
RUN=ONE_PRIVATE_MANUAL_PLUS_ESDF_EPISODE__NO_GDB
ONLY_LAUNCH_OVERRIDE=phase_offset_manual_observe_only:=false
PRODUCT_SOURCE_OR_PARAMETER_EDITS=NONE
RESULT=NOT_PASS__H2_REPLACEMENT_NOT_OBSERVED__OLD_PATH_END_HOLD
```

## Run integrity

- Private master: `http://127.0.0.1:11759`; private `ROS_HOME` and evidence:
  `/tmp/clean_active_acceptance_20260817/evidence`.
- Launch was the checked-in `phase_offset_esdf_tube_single.launch`.  The only
  argument override was the authorised `phase_offset_manual_observe_only:=false`.
  No GDB, attach, pause, product source edit, parameter mutation, or launch-file
  edit occurred.
- The target was `(8.0, 0.0, 1.0)`.  All task-owned recorder, launch, and
  master PIDs were sent SIGINT in that order and the bag was indexed.  Port
  11759 was released.  No user ROS process was touched.
- Bag: `clean_active_acceptance.bag`, SHA-256
  `da222327f1ecbb1976f6765c6dcdc0e1597efc6a0041ba74faa0f6f067d76bcb`,
  62 s, 11,885 messages, 346.4 MB; `rosbag info` passed.
- Current binary/map/launch provenance is in `input_sha256.txt`.  In particular:
  formation build ID `7b4e5c488c4c5f7402c5060485a93a20dddd0ad8`, adapter
  build ID `432e58b750d280b0a4c8cbdd39b6a136f29cdf05`, and unchanged pillar map
  SHA-256 `609cab63db1209be0d0390931782eb702859a0d85964a7f3ff8394966ec23414`.

## Topic and recorder validation

The actual topic namespace was obtained from the private live graph before the
goal.  Every required name appeared in `rostopic list`; a managed
`rostopic echo -n 1` subscriber was registered before the goal and each received
one message.  This avoids the root-level namespace error in the preceding GDB
audit.  Path, tube, and goal data semantically require a goal, so their
subscribers were established before the goal and completed afterward; this is
the only truthful possible ordering.

| Topic | Bag messages |
| --- | ---: |
| `/formation_planning/phase_offset_manual/diagnostics` | 1 |
| `/formation_planning/phase_offset_manual/tube_epoch_diagnostics` | 1 |
| `/formation_planning/phase_offset_manual/tube_raw_candidate_diagnostics` | 582 |
| `/formation_planning/phase_offset_manual/tube_cloud_snapshot_diagnostics` | 582 |
| `/particle0/path` | 2 (one nonempty path) |
| `/position_cmd` / `/sim/odom` / goal | 2,948 / 6,218 / 1 |

The sparse 83/50 control snapshots are existing publication semantics; they
are not 50 Hz telemetry.  They support only point observations below, not a
continuous-time selected-port or tracking claim.

## What the episode establishes

Bootstrap succeeded: the log has one `POINT_PHASE_V2 INIT` with
`phase_w=0.050`, path range `[0.050, 5.707]`, and the only nonempty published
path has 115 poses from approximately `(0.050, 0.001, 1.000)` to
`(5.684, 0.441, 1.000)`.

The one control/epoch snapshot, at bag time `1786972862.779`, records the
initial installed authority only:

| Fact | Observation |
| --- | --- |
| epoch state / disposition / reason | `ROLLING` / `INITIAL_INSTALL` / `NONE` |
| candidate and active source revision | 1 / 1 |
| candidate and active map observation | 72 / 72 (lag 0 in this one snapshot) |
| active/current validation / geometry | true / true |
| current bounds / retained delta inside | false / false |
| Runtime mode / selected / valid | `WAITING_FOR_CANDIDATE` / false / false |
| certificate denied / fatal / failure reason | false / false / `NONE` |
| tracking norm / bound | 0.065421 m / 0.15 m |
| cumulative install / equivalent-refresh / reject / wait | 1 / 0 / 0 / 0 |

This early nonselected snapshot is before a full control-telemetry sequence; it
does **not** prove that no later normal Runtime command was selected.  Its
single 0.065421 m sample is above 0.05 m, but it is neither a selected
steady-state sample nor a continuous maximum: S4 remains **NOT PASS / not
measured by this run**.

The raw/cloud timer path did run throughout the episode: all 582 raw profiles
are raw-complete, filtered-complete, and candidate-complete; all 582 cloud
snapshots are available, valid, and usable.  Cloud observation sequences
advance from 73 through 662 (580 distinct observations).  This demonstrates
that current-map candidate construction continued; it does not show installation
of those candidates.  After the old path was exhausted, 559 raw snapshots carry
the existing `FORWARD_HORIZON_SHORT` reason, which is a consequence of the
unchanged finite old path, not evidence that the Filter or raw cross section
failed.  No filter/margin/geometry change follows from this fact.

Active-map progression after the initial snapshot is **0 observed advances**:
there is only one 50-field control epoch row.  The bag schemas also have no
`PathTubePair::generation` field, so pair generation is **NOT OBSERVED**.
Likewise, the actual timer-refresh CAS count is **NOT OBSERVABLE**: the one
cumulative epoch snapshot says zero equivalent refreshes, but there is no
timer-CAS publication stream and no later control row from which to infer it.

## H2 and physical outcome

At `phase_w` 4.863 through 5.161 the log records seven C2
`connector_success` constructions.  It then records `replan not installed`;
there are 55 such log entries over the recorded episode.  There is no later
nonempty `/particle0/path` and no later path-`w` range than `[0.050, 5.707]`.
Thus this run observes C2 construction but **does not observe an H2 authority
replacement or new-path application**.  The existing public schemas cannot say
which internal H2 predicate rejected it.

After phase reached 5.691, the log reports 319
`all_candidates_path_end_clamped` and 321 `GOVERNOR_INVALID_HOLD` entries.  The
final odometry is `(5.8241, 0.3753, 0.9802)`, whose Euclidean distance to the
commanded goal is **2.2081 m**.  This is an old-path terminal hold away from
the goal, not successful navigation.

No `CERTIFICATE_DENIED` or `CURRENT_OUTSIDE` text appears in the captured
`/rosout`, and the one control snapshot has `certificate_denied=false`.  That
is an absence of an observation in this episode, not proof those Runtime paths
cannot occur.

## Acceptance boundary

| Required fact | Result |
| --- | --- |
| Correct live topic namespace and controlled recording | PASS |
| Initial ESDF path+tube authority | PASS for this episode |
| Candidate build/cloud evidence remains available | PASS for this episode |
| C2 constructed | PASS (7 candidates) |
| H2 replacement / new path before old-path exhaustion | FAIL / NOT OBSERVED |
| No path-end hold while still far from goal | FAIL |
| Selected/valid control sequence and timer-refresh installs | NOT OBSERVED (one sparse control snapshot only) |
| `CERTIFICATE_DENIED` / `CURRENT_OUTSIDE` event | NOT OBSERVED |
| S4 tracking `<0.05 m` | NOT PASS / insufficient evidence |
| Physical mission reaches target | FAIL |

This evidence identifies no new safe product change.  It specifically does
not justify loosening H2 identity, reusing an old tube on a new owner, adding a
gate/retry, or tuning A5 geometry, Filter, margin, map, speed, or lookahead.
The first internal false branch remains unobserved in this clean run.
