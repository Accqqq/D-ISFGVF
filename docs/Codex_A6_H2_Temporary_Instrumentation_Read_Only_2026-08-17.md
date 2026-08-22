# A6/H2 temporary instrumentation: one-run evidence

## Scope and restoration

This was a single private active episode for cause classification only.  The
temporary `TEMP_H2_TRACE` statements reported existing false returns; they did
not change a condition, lock, parameter, path/tube authority, Runtime state,
gate, reason, schema, or control selection.  The private master was
`http://127.0.0.1:11761`, with its own `ROS_HOME`, and used only
`phase_offset_manual_observe_only:=false`.

The one episode is recorded at:

`/tmp/h2_temp_trace_20260817/evidence/h2_temp_trace.bag`

It has duration `66 s` and `10,803` messages.  The owned master, launch, bag,
and planner were shut down; port 11761 is absent from the post-shutdown port
record.  User-owned ROS processes shown in the broad process snapshot were not
owned or stopped by this task.

The temporary instrumentation was removed unconditionally after the run.  The
four baseline source SHA-256 values were restored exactly:

| File | Restored SHA-256 |
| --- | --- |
| `phase_offset_matched_adapter.h` | `414015f908eecc51c75bd226a25163b96673b11f4016864bd67ef2447d24d7c1` |
| `phase_offset_matched_adapter.cpp` | `f6b7b0a5fdf77ece8372175c0a949c6b40518bef268b26e6b7229fee72450521` |
| `phase_offset_matched_adapter_test.cpp` | `bfa37d54d0765bc2951491ef7c944b857572e56c4f8c568153730f560422ca34` |
| `gvf_manager.cpp` | `078ae5378b49b7942b12ecef927f928f70fda0b0f5642d386691e650fdb07c18` |

`rg TEMP_H2_TRACE src/swarm_planner/bspline_traj` has no output.  Rebuilding
the restored source produced adapter Build-ID
`432e58b750d280b0a4c8cbdd39b6a136f29cdf05`, the same Build-ID used by the
previous clean and function-relative audits.  `git diff --check` passes.

## Observation

The instrumented adapter Build-ID was
`be67fe7417731628bc17883da5f0c47b725a6467`.  During the 60-second active
window its only trace was:

```text
manager=stage reason=no_certified_future_seam: 118
```

No adapter `stagePathTubePair`, `preparePathTubePairCommit`, or `finalize...`
trace occurred, and there was no H2 consume attempt.  In this episode the H2
transaction therefore stopped *before* adapter staging because the old active
profile supplied no certified future seam.  It does not identify a Runtime-bit,
prepared-tube-build, exact-PWL, phase-tuple, finalize, or CAS failure.

This result is compatible with the existing log's repeated
`all_candidates_path_end_clamped` evidence.  It is not evidence for loosening
any H2/A5 condition, adding a retry, adding a gate, changing Filter parameters,
or modifying C2 geometry.

## Relation to the function-relative audit

The earlier function-relative GDB run did reach adapter staging.  Its corrected
interpretation remains: eight `r14=0` visits on the audited
`buildPreparedTubeEpoch` result path indicate that this call returned false;
the raw cleanup-label counts must not be treated as independent failure counts.
The present run reached an earlier manager boundary and cannot confirm or
contradict that adapter-level observation.  Manager phase-tuple/finalize and
consume conditions remain unobserved in this episode.

## Restored-source checks

- `phase_offset_matched_adapter_test`: 63/63 PASS
- `phase_offset_tube_epoch_diagnostics_test`: 5/5 PASS
- `phase_offset_tube_epoch_integration_test`: 10/10 PASS
- `gvf_switch_policy_test` (including H2 suites): 74/74 PASS
- `continuous_phase_path_test`: 6/6 PASS

The legacy `matched_phase_offset_test` executable could not start because its
already-built devel artifact references a missing
`PhaseOffsetGeometryEvaluator::evaluate` ABI symbol.  This is an existing test
artifact/source ABI mismatch, not a failure of the restored product binary or
of this temporary instrumentation.  It requires a separately scoped full test
binary rebuild before it can be counted as a current-source regression result.

