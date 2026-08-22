# Zero-centerline installability — self-audit

Date: 2026-08-18
Stage: `ZERO_CENTERLINE_INSTALLABILITY`
Status: T0–T2 complete; planner/tube safety parameters remain frozen.

## Implemented within the stage whitelist

- `TubeBuilder` now retains the complete output of the actual cloud-clearance
  build in `TubeProfile::raw_build_samples`.  Each knot records path geometry,
  pre-inset bounds, post-inset raw bounds, the resolution-derived inset, and
  zero-containment facts.  The profile also carries the snapshot resolution;
  `TubeEpochManager` attaches the immutable observation sequence to that same
  profile rather than issuing another query.
- `TubeSurfaceValidator` records per-knot cover/requested-clearance evidence
  on that same profile.  `zero_centerline_continuously_certified` is true only
  after the complete filtered ribbon has passed validation and every retained
  segment cell has supplied a successful cover fact at every retained knot
  whose filtered interval contains zero.  No `{delta=0}` interval is created
  when the post-inset interval excluded zero.
- `TubeEpochManager` now treats centerline safety and the continuous zero-line
  certificate as Candidate admissibility conditions.  A nonzero retained
  offset cannot make an unsafe or uncertified nominal `p(w)` roll.  This uses
  the existing WAITING/CERTIFICATE_DENIED decisions; no latch, Runtime gate,
  planner, Filter, margin, map, launch, or swarm behavior was changed.
- The clearance-audit seam consumes the raw-build/filter/validator sidecar
  from one actual profile.  It does not infer pre-inset facts from the legacy
  60-field payload and does not claim planner/tube immutable-map identity when
  provenance is absent.

## Evidence and tests

The following passed:

```text
phase_offset_tube_cross_section_test       7/7
phase_offset_tube_builder_test             7/7
phase_offset_tube_filter_test             15/15
phase_offset_tube_surface_validator_test   9/9
phase_offset_tube_epoch_manager_test      50/50
phase_offset_runtime_test                 33/33
phase_offset_clearance_audit_test         20/20
phase_offset_raw_candidate_diagnostics     8/8
phase_offset_matched_adapter              63/63
```

The audit suite includes a real `TubeBuilder → TubeFilter →
TubeSurfaceValidator → clearance audit` test using one profile and one
clearance query.  It also covers:

- pre-inset zero retained after inset;
- pre-inset zero excluded by inset, with no unproved repair;
- pre-inset zero excluded and retained nonzero, rejected by the epoch manager;
- pre-inset zero present but no continuous validator certificate, fail-closed;
- unknown/out-of-map evidence;
- all-zero raw/filter/validator containment and snapshot/path provenance.

The complete workspace build passed, including `formation_planning`.
`git diff --check` passed.  Existing unrelated tracked and untracked worktree
changes were preserved and are not attributed to this stage.

## Safety disposition and remaining boundary

No evidence in this stage proves a physical duplicate UAV/map/discretization
margin, so no numerical margin mapping was tuned.  No live planner ESDF
adapter was added because that file is outside the current whitelist; absent
an immutable same-source query, the audit remains fail-closed and reports
provenance as unknown.  The post-inset zero-exclusion case therefore remains
rejected unless a future authorized stage supplies a continuous certificate
from the same immutable snapshot.

Stage complete.  Stop here; do not tune planner `safe_distance`, relax exact
PWL Filter rules, remove the inset, or add swarm offsets under this execution
specification.
