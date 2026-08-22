# A6/H2 `no_certified_future_seam` findings-first audit

## Conclusion

The observed `no_certified_future_seam` is a **terminal path-headroom
exhaustion**, not an A5 Filter/geometry failure and not an adapter Runtime/CAS
failure.

At the representative terminal replan in the one private trace episode:

```text
captured_w0 / phase_w       = 5.6894637843
old path range              = [0.050, 5.707]
remaining old-path phase    = 0.0175362157
min_certified_forward_w     = 0.4000000000
earliest legal H2 seam      = 6.0894637843
```

An H2 seam must be an already-existing old-pair sample.  No sample can be at
or beyond `6.08946` when the immutable old path ends at `5.707`.  Thus the
candidate set is necessarily empty, irrespective of the Filter's boundary
slope, ESDF build time, Runtime bits, or any downstream commit condition.

No product change is authorized by this audit.

## Evidence

The single temporary-instrumentation episode is
`/tmp/h2_temp_trace_20260817/evidence`.  Its log records:

```text
[POINT_PHASE_V2][INIT] phase_w=0.050 path_start_w=0.050 path_end_w=5.707
[REPLAN_REASON] ... remaining_w=0.017
TEMP_H2_TRACE manager=stage reason=no_certified_future_seam
[POINT_PHASE_V2] replan not installed; keep current frontend and phase_w=5.689
```

The trace reason occurred 118 times.  There was no adapter
`stagePathTubePair`, prepare, finalize, or consume trace: the H2 transaction
stopped before those calls.

The active private configuration records
`phase_offset/tube/min_certified_forward_w: 0.4`.  The static call passes that
exact value to `certifiedFutureSeamCandidates`.

The clean active bag independently shows the terminal geometry.  Its last raw
candidate profile has:

```text
current_w                         = 5.690995
candidate certified/profile range = [5.508094, 5.706570]
slack to candidate end            = 0.015575
```

This raw-candidate profile is not asserted to be the H2 old active profile.
It is supporting evidence only.  The conclusion above instead follows from
the old immutable path end, which bounds every old-pair sample used as an H2
seam.

At the same endpoint, governor logs report:

```text
path_w_end=5.707
candidate_count=36
path_end_clamped_count=36
valid_count=0
fallback_reason=all_candidates_path_end_clamped
```

`all_candidates_path_end_clamped` is produced by the separate command
governor: every positive lookahead query is at or beyond the same path end.
It does not call the H2 seam selector, but the two symptoms have the same
terminal-headroom cause.  The ensuing hold keeps the phase at the exhausted
endpoint; it does not manufacture a legal old-prefix seam.

## Static predicate reconstruction

`certifiedFutureSeamCandidates(old_pair, captured_w0, min_certified_forward_w)`
returns an ordered subset of `old_pair->full_path_samples`.

Before iterating, it requires a non-null immutable old owner, sample vector,
and active profile; matching source revisions; active availability/current
validation; a complete profile; finite nonnegative input; and a certified end
not before `captured_w0 + min_certified_forward_w`.

For each old sample `w`, it additionally requires:

```text
w > captured_w0
w >= captured_w0 + min_certified_forward_w
certified_start <= w <= certified_end
strictly ordered finite valid old samples
owner evaluation equals the stored p, p_w, p_ww
TubeFilter::query(active_profile, w) is valid
```

Therefore, for this episode the feasible set is contained in:

```text
{ old sample w | w >= 6.0894637843 and w <= 5.707 }
```

which is empty.  The candidate list is empty before C2 construction and before
all adapter transaction checks.  `stageFutureSeamPathTubeTransaction` returns
false, and the point-phase caller correctly leaves the existing frontend
installed (`replan not installed`).

## Evidence boundary

The existing diagnostics do **not** publish the active H2 pair's
`certified_segment_start_w`, `certified_segment_end_w`, full-sample first/last
`w`, or a per-filter rejection count.  Consequently this audit cannot say
whether the selector's *first* empty-return branch was the certified-end
precheck or a later per-sample filter.  That distinction is unnecessary for
the terminal root cause: no old sample can satisfy the mandatory forward
distance before the old path endpoint.

The raw-candidate diagnostic's certified interval describes the current
candidate profile, not an authoritative assertion about the old H2 active
profile.  It must not be substituted for the missing active-pair fields.

## Next measurement only if broader classification is needed

No new measurement is needed to establish this episode's terminal-headroom
root cause.  If a later read-only audit must distinguish non-terminal empty
sets, one low-rate transaction trace should capture exactly:

```text
captured_w0
min_certified_forward_w
old_owner_start_w, old_owner_end_w
active_profile_sample_first_w, active_profile_sample_last_w
active_certified_segment_start_w, active_certified_segment_end_w
old_full_sample_count, old_full_sample_first_w, old_full_sample_last_w
structural_precheck_pass
certified_end_precheck_pass
count_after_phase_threshold
count_after_certified_interval
count_after_owner_equivalence
count_after_tube_query
first/last surviving seam w
```

This should be a temporary, private, one-run measurement only—not a new
runtime schema, reason enum, gate, retry, or policy change.

