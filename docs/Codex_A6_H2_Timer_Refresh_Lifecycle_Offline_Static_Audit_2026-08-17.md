# A6/H2 timer-refresh lifecycle — offline/static audit

```text
DOCUMENT_ROLE=READ_ONLY_OFFLINE_AND_STATIC_AUDIT
DATE=2026-08-17
INPUT=/tmp/h2_lifecycle_correlation_20260817/evidence
ROS_STARTED=NO
SOURCE_OR_PARAMETER_CHANGE=NO
RESULT=FORWARD_HORIZON_SHORT_IS_PATH_END_EXHAUSTION__ONE_PREPARE_FALSE_REMAINS_UNCLASSIFIED
```

## Scope and source material

This is an offline read of the single `11763` episode already described in
the lifecycle-correlation audit.  It starts no ROS process and changes no
source, parameter, gate, or runtime state.  It reads:

- `TEMP_H2_LIFECYCLE.transcript.txt`;
- `h2_lifecycle_correlation.bag` raw-candidate, cloud, and epoch diagnostic
  topics through `rostopic echo -b` only;
- the restored `refreshPairFromTimerEpoch` and
  `prepareTimerPairRefresh` implementation.

The source SHA and restored Build-IDs remain those recorded in the preceding
audit; `git diff --check` passes.

## `FORWARD_HORIZON_SHORT` classification

The transcript has 727 `reason=11` timer builds.  Every one has
`candidate_cert.end_w = 5.706307`, the path endpoint in the corresponding
request.  All 727 also reached `finalizeTubeEpoch` (`finalized=1`), so this
is not a stale-build discard.

The bag contains raw-candidate payload for 704 of those 727 builds.  For
every one of the 704, existing payload fields prove:

| existing field/fact | result |
| --- | --- |
| `candidate_raw_complete` | 704/704 true |
| `candidate_filtered_complete` | 704/704 true |
| `candidate_complete` | 704/704 true |
| current sample found/complete | 704/704 true |
| `certified_segment_truncated_before/after` | 704/704 false/false |
| `certified_segment_end_w == requested_preview_end_w` | 704/704 true |
| requested preview end | 5.706307 for all 704 |
| certified forward distance | 0.016800–0.350793 m, all below 0.4 m |

The remaining 23 have no raw payload in the bag, so raw/filter fields cannot
be claimed for them.  They do have the same transcript facts: complete
Candidate, `reason=11`, and candidate certified tail `5.706307`.  The bag
has 727 raw payloads total while the transcript records 750 timer builds, so
the missing rows must remain explicitly labelled as unavailable rather than
fabricated by pairing nearby messages.

Cloud diagnostics independently show all 727 recorded snapshots complete,
configured, available, valid, and usable.  Base, actual, and plus/minus
normal-step statuses are all `KNOWN_FREE` (the existing numeric status 3).
No raw occupancy store or self-free seed was used.

Therefore the 727 failures are **path-end coverage exhaustion**, not a raw
construction failure, Filter-tail truncation, cloud/source failure, or stale
build.  `FORWARD_HORIZON_SHORT` is the existing correct fail-closed result:
the candidate ends exactly where the executable path ends but cannot provide
the required 0.4 m forward certificate.

For contrast, request 67/build 9 has a Filter-truncated candidate
(`requested_preview_end=4.118572`, certified tail `4.068955`,
`truncated_after=1`) but still had 1.932898 m of forward coverage and was a
complete active candidate.  Thus the bag distinguishes Filter truncation
from the terminal `reason=11` cohort; the terminal cohort has no such
truncation.

## Static refresh chain and `req=67`

`refreshPairFromTimerEpoch` has no independent predicate: it calls
`prepareTimerPairRefresh`, then (only if preparation succeeds) finalizes a
short pair CAS.  The prepare path can first return false at these existing
categories, in order:

1. **structural epoch/pair** — request, epoch and base pair exist;
   `epochMatchesRequest`; epoch full samples/profile; source revision and
   profile/sample owner agreement;
2. **latest command identity** — latest request active; same source/session,
   exact base pair and generation, same semantic owner/current path, finite
   position and positive finite `dt`;
3. **short authority boundary** — `runtime_command_mutex_` try-lock, no H2
   pin lease, live pair/generation/session and adapter authority session;
4. **exact current proof** — GeometryEvaluator, base ISF guidance, and local
   Runtime exact-PWL dry run against the latest request;
5. **latest categorical veto** — existing current reference/actual
   OCCUPIED/OUT_OF_MAP check.

Only after those tests are true does `finalizePreparedTimerPairRefresh` check
the preparation value, try-lock/pin, live pair/session/latest-request pointer,
bitwise Runtime retained delta and previous port, then performs shared-pointer
CAS.  A `result=prepare_false` never reaches this finalize/CAS sequence.

The 737 `prepare_false` events divide without inference:

| group | count | static cause |
| --- | ---: | --- |
| reason 6 (`CURRENT_OFFSET_OUTSIDE`) | 9 | no active profile, so prepare's structural active-profile requirement fails |
| reason 11 (`FORWARD_HORIZON_SHORT`) | 727 | no active profile, so the same structural requirement fails |
| req=67/build=9 | 1 | complete active epoch but unknown prepare category |

The other four complete active builds did not reach refresh at all:
`finalizeTubeEpoch` was false before `refreshPairFromTimerEpoch` because the
timer request was stale under its pre-existing source/currentness check.
The remaining nine complete active builds installed a refreshed pair.

For req=67 specifically, the transcript and existing bag fields rule out the
following as an explanation for the candidate itself:

- source revision is 1, base generation/session are 5/1, and map sequence is
  45 in the timer request;
- Candidate is complete, Active and current-valid; bounds and retained delta
  are valid/inside; certified tail is 4.068955;
- the frozen build snapshot is valid/usable and its categorical probes are
  `KNOWN_FREE`;
- `finalizeTubeEpoch` succeeds, which proves the build request is still
  current in its existing source/session/base-pair check.

The available data does **not** expose which of prepare categories 1–5 was
the first false for req=67.  In particular it does not record the latest
request's exact owner/current state, try-lock/pin result, live Runtime bits,
Runtime dry-run result, or the exact latest categorical observation used in
prepare.  No claim selecting one of those branches is justified.

## Narrow next-measurement recommendation (not authorised to execute)

The unresolved fact is one categorical value, not a request for a product
change: `prepare_first_false` for a complete/current-valid same-owner epoch.
A future **temporary-only** measurement specification can be limited to
`phase_offset_matched_adapter.cpp` and emit one fixed-prefix stderr line at
each existing false return in `prepareTimerPairRefresh`:

```text
TEMP_H2_REFRESH_PREPARE req/build/revision/base_generation/session/map
first_false={structural,latest_request,runtime_try_lock,pin,live_authority,
             geometry,base_guidance,runtime_dry_run,latest_categorical}
```

It needs no header, topic/schema, state, enum, parameter, gate, retry, or
control change.  One private episode must stop after the first
complete/current-valid `prepare_false`; it must then remove the line and
byte-verify the source before analysis.  This is a recommendation only,
with `IMPLEMENTATION_AUTHORIZED=NO` until a dedicated execution specification
is approved.

There is no basis in this audit for changing A5 geometry, Filter slope/model,
H2 seam conditions, margins, lookahead, or terminal fail-closed behavior.
