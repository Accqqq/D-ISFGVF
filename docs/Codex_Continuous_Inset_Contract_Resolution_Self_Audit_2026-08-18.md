# Continuous-inset contract resolution — self-audit

Date: 2026-08-18
Stage: `CONTINUOUS_INSET_CONTRACT_RESOLUTION`
Status: T0/T1 audit seam and regressions complete; planner/tube safety
parameters remain frozen.

## Scope and implementation

The audit keeps the existing planner path as the nominal tube centreline
`p(w)`.  It records the pre-inset interval, post-inset raw interval, Filter
interval, Validator evidence, and zero containment for the same sampled path
and profile.  The legacy 60-field diagnostic payload is unchanged; the new
classification is sidecar evidence.

The audit now distinguishes these cases:

- `INSET_ZERO_EXCLUSION_REQUIRES_PROOF`: pre-inset contains zero but the
  post-inset interval does not;
- `PLANNER_TUBE_ROBUST_CONTRACT_MISMATCH`: the actual profile has a valid
  pre-inset cross-section that excludes zero, the path/profile points match,
  and an explicitly supplied planner clearance query meets the planner's
  `safe_distance`;
- `FILTER_ZERO_EXCLUSION` and `VALIDATOR_ZERO_EXCLUSION` for later-stage
  exclusions;
- `UNKNOWN` for invalid, unknown, unavailable, or otherwise unproven facts.

An invalid or unknown cross-section is therefore never promoted to a planner /
tube mismatch merely because its interval happens not to contain zero.  The
classification remains fail-closed and does not install or execute a nonzero
offset.

## Changes made

- Fixed the stale `profile_sample` reference in
  `src/swarm_planner/bspline_traj/src/integration/phase_offset_clearance_audit.cpp`
  to use the raw-build sample that owns the cross-section reason.
- Parenthesized the Filter and Validator zero-exclusion accumulators to make
  the intended precedence explicit.
- Added `UnknownPreInsetCrossSectionIsNotLabelledAsPlannerTubeMismatch`, which
  covers `CENTER_UNKNOWN`, `INVALID_GEOMETRY`, and
  `CURVATURE_NUMERICAL_FAILURE` while the pre-inset interval excludes zero.

No planner path generation, `KinoA*/B-spline` parameters, planner
`safe_distance`, exact-PWL Filter rule, Runtime policy, margin values, map
inflation, launch, message schema, or swarm behavior was changed.

## Verification

The targeted audit binary passed 22/22 tests, including the real
`TubeBuilder -> TubeFilter -> TubeSurfaceValidator -> clearance audit` chain
test and the Branch-B synthetic classification test.  Existing stage
regressions also passed:

```text
TubeBuilder                         7/7
TubeSurfaceValidator                9/9
TubeEpochManager                   50/50
TubeCrossSection                    7/7
TubeFilter                         15/15
PhaseOffsetRuntime                 33/33
RawCandidateDiagnostics             8/8
PhaseOffsetMatchedAdapter          63/63
PhaseOffsetClearanceAudit          22/22
```

The complete workspace build passed:

```text
cmake --build build -j2
```

`git diff --check` passed, and no trailing whitespace was found in the
continuous-inset audit header, implementation, or test.  The worktree contains
many pre-existing user-owned tracked and untracked changes; they were
preserved and are not attributed to this stage.

## Evidence boundary and disposition

The current workspace does not provide live immutable planner ESDF provenance
for one ROS execution of the planner and TubeBuilder.  The audit therefore
does not claim that the synthetic regression fixtures are live planner
evidence, and it does not select Branch A or Branch B for the physical system
without that provenance.  Callers must supply the same-path planner query,
raw-clearance query, snapshot sequence/resolution, and TubeProfile sidecar for
an operational classification; missing or mismatched provenance remains
`UNKNOWN`/fail-closed.

The numerical context remains unchanged: planner `safe_distance` is 0.40,
the representative residual radius is 0.45, and the snapshot-resolution
continuous inset is approximately 0.10 m.  No parameter tuning is justified
by this stage.  If a future same-snapshot audit proves pre-inset exclusion of
zero despite a planner-safe centreline, the result is a robust-contract
mismatch to resolve at the clearance interface—not a reason to relax Filter
or rescue the path with a nonzero swarm offset.

Stage complete.  Stop here under the current execution specification; do not
advance to planner tuning or swarm-offset integration.
