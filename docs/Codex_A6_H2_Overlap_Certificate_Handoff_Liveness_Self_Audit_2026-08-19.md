# A6/H2 Overlap Certificate Handoff Liveness Self-Audit

Date: 2026-08-20

## Scope

This audit covers the A6/H2 Path+Tube handoff and the continuous Tube
certificate failure observed after a planner C2 replan.  The intended
semantics remain unchanged:

- the immutable planner/C2 path is the nominal centreline;
- `delta=0` is not manufactured by the Tube or Filter;
- nonzero swarm intent is projected through the existing Runtime port;
- a failed Tube certificate cannot replace the authoritative pair or alter
  planner phase continuity.

## Root cause

The H2 prepared path already carried the immutable owner, but the ordinary
timer `buildTubeEpoch()` still built its preview from uniformly sampled path
points.  Adaptive Tube sampling could therefore create a cell crossing the
old-prefix/C2/mapped-tail segment boundary.  `ContinuousPhasePath::cellBounds()`
correctly rejects such a cell.  The builder then fell back to the historical
snapshot-resolution inset; on a narrow but physically valid interval this
inset removed the nominal centreline from the retained profile, producing
`base_centerline_continuity_uncertified` and eventually an invalid governor
hold.

Two related certificate conservatisms were also found:

1. `MakeMappedBsplineCertificate()` included unrelated adjacent arc-length map
   cells at an endpoint, allowing a neighbouring non-positive Bernstein speed
   coefficient to invalidate an otherwise local cell.
2. A mapped B-spline whose strict arclength map could not be built retained a
   valid linear `t(w)` point evaluator but exposed no generic cell certificate.
   The fallback remains fail-closed when its actual derivative lower bound is
   not positive, but can now certify executable interior cells.

## Implemented correction

- `BuildOwnerAlignedTubePreview()` now partitions every ordinary and prepared
  Tube preview at immutable owner segment endpoints before adaptive refinement.
- `MakeMappedBsplineCertificate()` considers only arc-length cells actually
  intersecting the requested phase interval.
- The linear mapped-B-spline fallback exposes a cell-local Bernstein
  certificate when speed, horizontal speed, acceleration, jerk, and regularity
  bounds are finite and positive; otherwise the historical conservative inset
  is retained.
- Quintic connector certificates use deterministic local Bernstein
  subdivision when a single control polygon is too conservative.  The
  aggregate certificate is still fail-closed if any subcell lacks a positive
  speed proof.
- No planner clearance, Tube margin, Filter predicate, Runtime port condition,
  phase reset, speed limit, or new navigation gate was added or relaxed.
- Temporary root-cause logging was removed after dynamic attribution.

## Evidence

Dynamic pillar runs with a valid immutable ESDF snapshot showed:

- before the correction: adaptive cells crossed C2 seams, `cell_geometry_certified`
  became false, `continuous_inset=0.10`, and zero-centreline continuity failed;
- after the correction (diag8): no seam-related `CERT_CELL_FALSE`,
  `QUERY_CELL_FALSE`, or `OWNER_CELL_BOUND_FALSE`; ordinary profiles commonly
  had `cell_geometry_certified=1`, `certified_cell_count>0`, and
  `continuous_inset=0`;
- remaining rejected epochs were classified as genuine clearance,
  regularity, candidate-incomplete, or forward-horizon observations, not as
  seam partition failures.  The run reached the point goal.

## Regression results

The final focused build passed:

- `continuous_phase_path_test`: 11/11;
- `phase_offset_matched_adapter_test`: 74/74;
- `gvf_switch_policy_test`: 33/33;
- `phase_offset_runtime_test`: 35/35;
- `phase_offset_tube_epoch_manager_test`: 51/51;
- `catkin_make -j2`;
- `git diff --check`.

## Remaining interpretation

If a future profile still has a raw interval that excludes zero, or a valid
ESDF query reports insufficient reference/actual clearance, that is a physical
Tube feasibility result.  It must remain fail-closed and be handled by planner
replanning or map/margin accounting, not by forcing `{0}` into the profile.

