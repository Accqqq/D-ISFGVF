# A5/A6 dynamic acceptance self-audit — 2026-08-18

```text
DOCUMENT_ROLE=PRIVATE_DYNAMIC_ACCEPTANCE_SELF_AUDIT
STAGE=A5_A6_ACTIVE_ESDF_AND_H2
PRIVATE_MASTER=http://127.0.0.1:11761
ONLY_LAUNCH_OVERRIDE=phase_offset_manual_observe_only:=false
PRODUCT_SOURCE_OR_LAUNCH_EDITS=NONE
USER_ROS_OR_PROCESSES_TOUCHED=NO
RESULT=NOT_PASS__A5_BASE_CENTERLINE_CONTINUITY_CERTIFICATE_NOT_INSTALLED__A6_JOINT_PATH_TUBE_NOT_OBSERVED
AUTO_ADVANCE=false
```

## Run identity and provenance

The episode used the checked-in `phase_offset_esdf_tube_single.launch`, the
original `pillar.pcd`, and all launch parameters unchanged except the one
authorised active-mode override above.  A task-owned ROS master, `ROS_HOME`,
logs, recorder and simulator/planner nodes were used.  The existing user ROS
master on port 11311 was not queried, attached to, signalled or stopped.

Evidence directory:

```text
/tmp/a5a6_dynamic_terra_20260818/evidence
```

The bag is `a5a6_active.bag` (SHA-256
`878c1b02dc21d8c9c5304ec5888f0cce0ffcb9f3fe74f771aa7e7de60107a3b1`).
Launch/map and binary hashes, Build-IDs, and the unchanged dirty-worktree
provenance are in `evidence/provenance.txt`.

## Build and focused regression

```text
cmake --build build -j2                              PASS
phase_offset_matched_adapter_test                    65/65 PASS
phase_offset_tube_epoch_integration_test             10/10 PASS
phase_offset_tube_markers_test                       10/10 PASS
phase_offset_raw_candidate_diagnostics_test            8/8 PASS
phase_offset_runtime_test                             33/33 PASS
phase_offset_tube_builder_test                         7/7 PASS
phase_offset_tube_cross_section_test                   7/7 PASS
phase_offset_tube_epoch_manager_test                  50/50 PASS
phase_offset_tube_filter_test                         15/15 PASS
phase_offset_tube_surface_validator_test               9/9 PASS
```

The old `phase_offset_tube_dynamic_feasibility_test` executable is an orphan
of a previous CMake graph: it has no current build target and fails at startup
with an undefined symbol.  It is recorded as stale-build evidence, not as a
product test pass or a dynamic acceptance result.

## Topic schemas and counts

The recorder subscribed before the goal.  The bag contains:

| Topic | Messages | Schema/result |
|---|---:|---|
| `tube_raw_candidate_diagnostics` | 22 | 49 fields; complete raw records |
| `tube_cloud_snapshot_diagnostics` | 22 | 18 fields; usable snapshots |
| `diagnostics` | 17 | 83 fields |
| `tube_epoch_diagnostics` | 17 | 50 fields |
| `tube_candidate` | 21 | 17 ADD, 4 DELETE; IDs 0/1 boundaries and 2 ribbon |
| `tube` | 21 | 0 ADD, 21 DELETE; no certified active profile |
| `/particle0/path` | 5 | 3 non-empty paths: 115, 117, 118 poses |
| `/position_cmd` | 5,121 | active command stream and terminal hold |
| `/sim/odom` | 11,522 | final sample available |
| `/rosout` | 1,687 | C2/replan/governor correlation |

Candidate ADD geometry was finite and structurally valid.  Certified DELETE
was consistent with the epoch state; it was not a marker publication failure.

## A5 active ESDF result and first false layer

Raw/cloud construction ran and remained current.  The first raw record's
`[0.05, 2.034673]` values are the requested preview range, **not** lateral
bounds.  Its actual current raw cross-section evidence is:

```text
current_w = 0.083399
raw environment bounds = [-1.93125, 2.90000]
raw current_contains_zero = 1
raw current_contains_preferred_delta = 1
```

All raw records observed a complete current sample; zero containment holds in
the first ten records, then becomes false while the planner enters the
obstacle-side segment.  The first observable false layer is therefore **after
raw construction**, at the Candidate-to-Active current-validation/installation
boundary.  The carried epoch-reason values give the earliest attribution:

```text
candidate sequence 2..10: reason=13 (base_centerline_continuity_uncertified)
candidate sequence 11:    raw zero becomes false
candidate sequence 12..18: reason=11 (base_centerline_clearance_insufficient)
```

The later positive-only raw cross-sections are a subsequent geometric
consequence; they are not the first false layer.

The corresponding observable active-validation state is:

```text
candidate raw/filtered/complete = 1 / 1 / 1
candidate marker              = ADD
epoch current_geometry_valid  = 1
epoch current_bounds_valid    = 0
epoch retained_delta_current_inside = 0
epoch current_state_admissible = 0
active_available / active_tube_epoch / active_display_certified = 0 / 0 / 0
```

The 50-field epoch publication is sparse and does not expose all per-build
post-filter bounds, but the independent raw stream carries the reason=13/11
evidence above.  It would be incorrect to call this a raw zero-exclusion-only
failure, a Filter defect, or a margin duplication.  Candidate ADD plus
Certified DELETE is nevertheless the expected fail-closed result.  No
geometry, Filter, margin, planner or parameter change is authorised by this
result.

## A6/H2 and physical outcome

The planner emitted two C2 connector successes and accepted two new planner
paths:

```text
initial path: 115 poses, w=[0.050, 5.706]
C2 replacement 1: 117 poses, w=[0.050, 5.848]
C2 replacement 2: 118 poses, w=[2.618, 8.446]
```

These are planner path installations only.  No corresponding new
`PathTubePair` reached Active/Certified ownership in this episode.  After the
local path reached `phase_w=4.994`, 96 further replans were not installed and
the governor produced 572 `GOVERNOR_INVALID_HOLD` records.  The final odometry
was approximately `(5.623, 0.250, 0.839)`, 2.395 m from the goal `(8,0,1)`;
there was no goal-reached event.

Thus A6's joint requirement — a real C2/H2 path+tube replacement before old
path exhaustion, Runtime rebase/final CAS, and terminal goal arrival — is not
observed here.  The planner's accepted C2 path messages must not be reported
as H2 success.

## Disposition

This episode is stopped at the first reproducible blocker.  No retry, tuning,
new gate, stale/revision relaxation, old-profile reuse, or Runtime/CAS change
was made.  Existing separate audits that observed H2 replacements or reached a
goal remain separate partial evidence and do not convert this joint A5/A6
episode into a PASS.  A5 active and A6 joint dynamic acceptance remain open;
the next authorised action is a margin/clearance accounting investigation or a
newly specified safe-path fixture, not A7 implementation.
