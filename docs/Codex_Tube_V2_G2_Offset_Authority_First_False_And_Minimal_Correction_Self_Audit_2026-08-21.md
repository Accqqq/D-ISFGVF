# Tube V2 G2 offset-authority first-false and minimal-correction self-audit

Date: 2026-08-21
Workspace: `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`
Status: `G2_IMPLEMENTATION_COMPLETE_DYNAMIC_ACCEPTANCE_UNMET / VALIDATOR_QUERY_BUDGET_EXHAUSTED`

## Authorized implementation

G2 first implemented the conditional deterministic inward correction, then
added the final observability attribution.  The old production path was:

```text
full-width ESDF ribbon fails SurfaceValidator
-> collapse all filtered bounds to the planner zero baseline
-> ZERO_ONLY_PLANNER_BASELINE
```

The authorized new path is:

```text
complete raw/filter profile with nonzero capacity
-> full-width validation with the same immutable snapshot/query
-> if retryable, fixed both-sided/positive-only/negative-only inward families
   with deterministic halving toward delta=0
-> mutate only filtered bounds, recompute local PWL slopes/zero containment/
   minimum width, preserve raw/environment/filter-input evidence
-> revalidate every candidate with the same SurfaceValidator/query/snapshot
-> select by current interval width then certified forward horizon
-> OFFSET_CERTIFIED only for a nonzero obstacle-certified current-connected result
-> otherwise collapse fail-closed to the planner zero baseline
```

Actual full-width query-budget exhaustion and nonretryable invalid/configuration
failures remain terminal; they do not enter inward search.  The Builder's
zero-baseline reason now distinguishes:

1. actual full-width query-budget exhaustion;
2. nonretryable invalid/configuration failure;
3. inward search attempted but no candidate certified, including deterministic
   attempt count and last inward validation reason/query count/limit flag.

The accepted inward path retains the established `retained inward certified
ribbon` outcome.  All inward candidates remain monotone toward `delta=0`, use
the same immutable snapshot/query/Validator, and are independently revalidated.

The matched adapter adds exactly one unthrottled ROS attribution line per
affected ESDF build when the complete candidate is zero-only but the retained
raw Builder samples still have nonzero capacity.  The line is read-only
observability and carries candidate sequence, map sequence, obstacle flag,
first stop reason/w, and invalid reason.  No new state, gate, parameter, topic,
schema, query limit, subdivision limit, or Runtime behavior was introduced.

## Exact stage files and source identity

```text
docs/Codex_Tube_V2_G2_Offset_Authority_First_False_And_Minimal_Correction_Execution_Spec_2026-08-21.md
docs/Codex_Tube_V2_G2_Offset_Authority_First_False_And_Minimal_Correction_Self_Audit_2026-08-21.md
/home/cxq/ISF-GVF/handoff.md
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/certified_tube_builder.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/certified_tube_builder.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/certified_tube_builder_test.cpp
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
```

G2-T0 before → final after SHA-256 for the four source/test files:

| File | Before | After |
|---|---|---|
| `certified_tube_builder.h` | `97e645559397039e044aa5d4e25a8628ddb5735181473100b5cc7bb063ecb954` | `6dcf9856d8f3aeaa459ea5b76f151346e6e7dec5453007cef64e5aa8357f475e` |
| `certified_tube_builder.cpp` | `2c6f3225796c8eb6fc76ee7222f4aa4acaff4cfe1884d15d532870543e41ba4d` | `475fc672178758cad17fa9c2099c462e1feca2cd5ca6f7154766025884e5a1ad` |
| `certified_tube_builder_test.cpp` | `ea27923c73ddb33545f973c57010a8b628a885c60714b39ecc845a18c0a533e2` | `a1f7bf1b46abc5d9817f6e009dc45feddf96d52bdb5831771d5e072ab987be86` |
| `phase_offset_matched_adapter.cpp` | `dd15261bc4ed55941c6555779690e9cbbb2ac2b69295aa52184ca0af42d2edd8` | `4f324cb9f83fc7d60fbea9bc32338244cfcc00292660165d9ac6f930ca93442a` |
| `tube_types.h` protected interface | `e11dd5777f542cdd147bd05bb564ea0afa3c6a314510b26b1164c2ea55d12e7b` | unchanged |

Protected `TubeFilter`, `SurfaceValidator`, `TubeEpochManager`, Runtime,
planner/manager, launch, parameter, and ROS schema hashes remained unchanged;
the only production algorithm source changed by G2 is the CertifiedTubeBuilder
listed above, plus the adapter's read-only log.

## Deterministic verification

```text
TubeFilter                         9/9
SurfaceValidator                  11/11
CertifiedTubeBuilder              11/11
TubeEpochManager                  53/53
Runtime                            35/35
gvf_switch_policy                 48/48
phase_offset_matched_adapter      82/82
required CTest regex              10/10
full catkin_make -j2              PASS
git diff --check                  PASS
```

The builder tests cover true zero capacity, query-budget terminal, accepted
strict inward repair, asymmetric one-sided repair, remote truncation, current
anchor failure, provenance preservation, and raw-incomplete fail-closed
behavior.  The inward-repair tests pass deterministically; this is not a claim
that a dynamic run entered inward search.

## Dynamic readiness evidence

The initial one-stage readiness attempt is retained separately:

`/tmp/tube_v2_g2_readiness_baseline_20260821_015000/` (port 12888) ended as
`HARNESS_PRECONDITION_MISMATCH` / `G2_T1_PREGOAL_READINESS_TIMEOUT`.  The harness
required profile/epoch/zero-gate topics before sending the first goal, but this
launch does not create a planner owner or emit those phase-offset streams until
after a goal.  It therefore sent no goal and was not a product, geometry, or
bootstrap failure.  G2 §4.2 was revised to a two-stage contract: first wait on
actual map-topic maturity, then send the single goal once and observe existing
profile/epoch facts.

All three readiness-matched runs used fresh private loopback masters/ROS_HOME,
the unchanged `phase_offset_esdf_tube_single.launch`, pillar map,
`phase_offset_manual_observe_only=false`, and one `(8,0,1)` goal (mapped by the
existing callback to internal `(8,0,2)`).

| Run | Result |
|---|---|
| `/tmp/tube_v2_g2_readiness_baseline_t1b_20260821_020000/` (12889) | local map 900; raw current anchors 21/21; goal distance 0.196; active obstacle certification later false; no Pair/authority. |
| `/tmp/tube_v2_g2_readiness_repair_20260821_023748/` (12890) | local/mock/occupancy/esdf/update 900/300/903/903/903; goal distance 0.184; candidate/active complete and zero gate open; no Pair/selected/retained nonzero. |
| `/tmp/tube_v2_g2_readiness_final_20260821_025413/` (12891) | local/mock/occupancy/esdf/update 900/300/902/902/902; goal distance 0.193; valid 104 s, 10,885-message bag; no Pair/selected/retained nonzero. |

The final ROS log and bag contain exactly one affected-build attribution:

```text
candidate_seq=7 map_seq=928 obstacle_certified=0
first_stop_reason=insufficient_clearance first_stop_w=4.989959832
invalid_reason="full-width query-budget exhaustion; reason=insufficient_clearance
queries=249993 limit_exceeded=1 max_queries=250000; using planner zero baseline"
```

This is a genuine terminal budget condition.  Therefore the final candidate
did not enter inward search, and the dynamic acceptance remains unmet.  Final
port/process checks are clean; only the task-owned ROS process groups were
stopped.

## Final source identity and boundary

```text
HEAD 9a0e97560b8ddf15ac1ed1581030ed4b911f6e43
certified_tube_builder.cpp 475fc672178758cad17fa9c2099c462e1feca2cd5ca6f7154766025884e5a1ad
certified_tube_builder_test.cpp a1f7bf1b46abc5d9817f6e009dc45feddf96d52bdb5831771d5e072ab987be86
phase_offset_matched_adapter.cpp 4f324cb9f83fc7d60fbea9bc32338244cfcc00292660165d9ac6f930ca93442a
```

Stop at this boundary.  Do not claim dynamic inward repair, tune or raise
`max_query_samples`, alter subdivision limits, add gates/parameters/schema,
or enter Runtime/QP/Pair/H2/planner work.  Any future capacity/cover/Validator
algorithm change requires a new execution specification.
