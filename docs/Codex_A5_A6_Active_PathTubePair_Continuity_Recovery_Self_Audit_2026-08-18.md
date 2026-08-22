# A5/A6 active PathTubePair continuity recovery self-audit — 2026-08-18

```text
DOCUMENT_ROLE=PRIVATE_A5_A6_CONTINUITY_RECOVERY_SELF_AUDIT
STAGE=A5_G1_FAIL_CLOSED_AND_A6_H2_ACTIVE_PATH_TUBE_CONTINUITY
PRIVATE_MASTER=http://127.0.0.1:11766
ROS_HOME=/tmp/a5a6_afterfix_20260818/ros_home
USER_ROS_MASTER_TOUCHED=NO
AUTO_ADVANCE_TO_A7=false
```

## Scope and changed files

This audit is limited to the A5/A6 continuity findings.  The only product
changes in this recovery are:

```text
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
```

The adapter now requires a joint `PathTubePair` handoff when active manual
mode is non-observe-only, the tube source is not `NONE`, and the manual
amplitude is nonzero, even when `retainedDelta == 0`.  It does not fabricate a
zero tube, reuse an old tube, or bypass the safety gates.  The regression test
pins this case and preserves the observe-only exception.

No Filter, clearance, margin, geometry, planner/Kino/B-spline, launch, map,
configuration, or recovery-owner changes were made.

## G1 first-false finding

G1 is a real physical future-knot zero-exclusion, not an evidence-mapping
defect.  For `seq=2` on one profile, the first false knot is:

```text
w = 2.431715365
raw interval      = [0.018750000, 1.525000000]
filtered interval = [0.022804744, 1.520945256]
filtered_contains_zero = false
```

The preceding knot (`w=2.382096295`) still contains zero, with
`cover=0.961717985`, requested clearance `1.411718985`, and observed
clearance `0.527746230`.  A5 therefore remains fail-closed at this physical
future-knot boundary.  No attempted workaround is authorised.

## G3 finding and correction

The old `retainedDelta == 0` branch classified an active manual, nonzero future
intent as a neutral direct planner handoff.  It retired the old authority
before a replacement pair was installed, creating a command-path owner gap.
The minimal correction keeps that intent on the joint H2 handoff path.  A
neutral handoff remains neutral only for the genuinely neutral/observe-only
cases.

## Focused regression and build

```text
cmake --build build -j2                                      PASS
phase_offset_tube_surface_validator_test                     9/9 PASS
phase_offset_tube_epoch_manager_test                        50/50 PASS
phase_offset_runtime_test                                    33/33 PASS
phase_offset_matched_adapter_test                            65/65 PASS
phase_offset_tube_epoch_diagnostics_test                      5/5 PASS
phase_offset_tube_epoch_integration_test                     10/10 PASS
gvf_switch_policy_test                                       80/80 PASS
```

## Dynamic episode and provenance

The episode used the checked-in `phase_offset_esdf_tube_single.launch`, the
original `pillar.pcd`, active manual mode (`observe_only=false`), and a task-
owned ROS master at `127.0.0.1:11766`.  The existing user master on 11311 was
not queried, attached to, signalled, or stopped.

Evidence directory and bag:

```text
/tmp/a5a6_afterfix_20260818/evidence
/tmp/a5a6_afterfix_20260818/evidence/a5a6_afterfix.bag
duration = 207 s (22:18:14.48–22:21:42.06)
size = 16.0 MB, messages = 21417
sha256 = 4ef1873f870d2d4c9894b50627e464d7dcc5386461c080ca7c86815d7a34d695
```

Input/runtime hashes:

```text
phase_offset_esdf_tube_single.launch
  334d807751ad6aa1739c241f07b5e22c554a3ff2470838e975a9cad8009e3813
pillar.pcd
  609cab63db1209be0d0390931782eb702859a0d85964a7f3ff8394966ec23414
formation_planning binary
  93f7b51c33255c3a4e372e776c7dbb50a4ef947635c2464b219c70769880969a
  Build-ID c339b2bd7fc326307da24ce55847254bf2adbb41
```

## Dynamic results

The recorder subscribed before the goal.  Candidate and certified evidence
showed the expected atomic transition:

```text
tube_candidate: 25 messages, 6 ADD markers (two 3-marker candidate epochs), 69 DELETE
tube:           25 messages, 3 ADD markers (one certified 3-marker pair), 72 DELETE
active_path:    2 ADD snapshots (115 points, then 77 points), 23 DELETE
epoch diagnostics: 2 messages × 50 fields
```

The replacement active path starts at `(4.5482, 0.4115, 1.0000)` while the
previous active path ends at `(5.6836, 0.4410, 1.0000)`; the bag contains no
semantic-owner-unavailable persistence or command-path gap at the handoff.
The epoch records report candidate/raw/filtered/complete and active validity,
`certificate_denied=0`, and `genuine_fatal_invariant=0` for the observed
installed epochs.

The only `GOVERNOR_INVALID_HOLD` was the startup guidance-unavailable sample.
It was followed by four `VEL_MATCH_GOVERNOR` command samples and a terminal
goal event:

```text
[GVF][POINT_GOAL][REACHED] distance=0.197
```

## Disposition and stopping boundary

```text
G1 physical future-knot zero exclusion: NOT_PASS (intentional fail-closed)
G3 command-path owner continuity repair: PASS
A6/H2 joint PathTubePair dynamic handoff and goal arrival: PASS
Scoped A5/A6 continuity-recovery episode: PASS_WITH_G1_RECORDED_NOT_PASS
```

This audit stops at the requested A5/A6 boundary.  It does not advance to A7,
change the G1 geometry/clearance contract, or broaden the work to planner,
Filter, margin, launch/config/map, or recovery-owner design.
