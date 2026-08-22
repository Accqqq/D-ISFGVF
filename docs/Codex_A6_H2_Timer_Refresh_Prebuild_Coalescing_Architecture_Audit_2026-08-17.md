# A6/H2 timer-refresh prebuild coalescing — read-only architecture audit

```text
DOCUMENT_ROLE=READ_ONLY_ARCHITECTURE_AUDIT
DATE=2026-08-17
STAGE=A6_H2_TIMER_REFRESH_PREBUILD_COALESCING_DECISION
PRODUCT_SOURCE_OR_PARAMETER_CHANGE=NONE
RESULT=STOP__PREBUILD_COALESCING_IS_SAFE_ONLY_AS_AN_OPTIMIZATION_AND_CANNOT_FIX_BUILD_TO_LATEST_RUNTIME_RACE
```

## Question and scope

The successful-epoch first-false measurement proved one exact situation: an
epoch was complete/current-valid at its timer-build snapshot but
`PhaseOffsetRuntime::prepare` returned `WAITING_FOR_CANDIDATE` at the later
latest command snapshot.  This audit decides whether replacing a pending timer
request with a newer same-authority command snapshot *before construction*
can safely and effectively fix that gap.

This is static/read-only.  It changes no source, parameter, process, ROS
state, A5/Filter/C2 geometry, gate, schema, reason, enum, state, retry, or
control behavior.

## Existing ownership and sequencing

| boundary | current mechanism | safety role |
| --- | --- | --- |
| command capture | `update()` holds `runtime_command_mutex_`, validates its input pair/session, copies the immutable owner, base-pair pointer/generation, current path/position/gains/dt, map snapshot and Runtime history into a new `TubeBuildRequest`, then atomically stores `latest_build_request_` | every request owns values and shared owners; no callback pointer crosses threads |
| timer dequeue | `timerTick()` atomically loads `latest_build_request_` once, claims `timer_inflight_`, and immediately calls `buildTubeEpoch(request, built)` | there is no pending work queue or separate worker/dequeue stage |
| construction | same-owner pair refresh constructs with a stack-local `TubeEpochManager`; it uses only the request's immutable path, current state and frozen snapshot | no Runtime, pair authority, or persistent timer manager is mutated by a stale pair build |
| timer completion | `finalizeTubeEpoch()` publishes raw/cloud evidence, checks source/session/base pair currentness, stores Candidate evidence, then calls refresh | a stale build cannot clear or replace newer authority |
| install prepare | `prepareTimerPairRefresh()` re-reads the latest request, serializes briefly with Runtime, requires the live exact pair/session, runs geometry/base guidance and local exact Runtime dry-run, then vetoes latest categorical unsafe evidence | no profile built for an old state can become authority without proof at the latest state |
| install final | `finalizePreparedTimerPairRefresh()` rechecks latest-request pointer and bitwise Runtime delta/previous port under the short Runtime lock, then shared-pointer CASes the exact old pair | a post-prepare command/Runtime change rejects rather than installs |

The H2 pin is separate transaction ownership.  A timer build may occur while a
pin exists, but timer refresh does not wait: prepare/finalize reject an active
lease.  `authority_session_`, exact `PathTubePair` pointer/generation and
path owner prevent an old goal or path authority from being revived.

## What prebuild coalescing could safely mean

The only defensible candidate operation is an extra atomic load immediately
before `buildTubeEpoch()`.  It may replace the timer's first loaded request
only when all existing authority identity facts agree:

```text
active
source revision
authority session
base PathTubePair shared_ptr identity
base pair generation
semantic path-owner shared_ptr and domain
```

The map snapshot must remain construction provenance of whichever immutable
request is selected; it is not an equality key.  The selected request must
still independently pass all existing sampling/build validation.  No Runtime
state can be read or written by construction, no pair may be CASed there, and
the current post-build latest-state dry-run/categorical/CAS protocol must stay
unchanged.

Under those conditions the extra load is memory-safe: it atomically copies a
`shared_ptr<const TubeBuildRequest>` and all transitive path/pair/map owners
remain alive.  It does not alter ownership or grant authority.

## Why it cannot solve the measured fault

There is no queued request waiting between timer dequeue and construction:
after the current load, `timerTick()` immediately enters the expensive build.
Therefore an extra prebuild load can at most close a tiny instruction-scale
race before the call.  It cannot constrain a command arriving at any time
during construction, which is the interval relevant to a build that finishes
with a current-state mismatch.

No safe linearization point exists without one of the following prohibited or
unsafe changes:

1. holding `runtime_command_mutex_` over construction would block the command
   loop and violate the existing short-lock rule;
2. retrying/rebuilding when a newer request arrives would add a retry loop and
   can starve under 50 Hz command updates;
3. accepting the old profile without the final latest Runtime proof would
   weaken current containment/exact PWL safety;
4. overwriting Runtime/pair state with build-snapshot bits would violate the
   existing prepare/finalize CAS proof.

The first-false trace demonstrates the existing intended outcome: the epoch
was valid at its build request, but the latest state was not executable on
that profile.  The failure occurred in Runtime `prepare` before the PWL
witness search or pair CAS.  An added prebuild atomic load does not change
that later state transition.  The existing post-build revalidation is already
the required fail-closed response.

Thus prebuild coalescing is potentially a harmless micro-optimization, but it
is neither a correctness repair nor a reliable reduction of the observed
build-to-latest mismatch.  It must not be represented as A5, Filter or H2
completion recovery.

## Hypothetical proof/test matrix

If a separately justified performance optimization is later considered, its
specification must prove all rows below; this audit does not authorise it.

| test | required result |
| --- | --- |
| newer same-pair request before the extra load | the newer immutable request alone is built |
| revision/session/pair/generation/owner mismatch | old request is retained or completion is discarded; no mixed request |
| newer map snapshot, same authority | chosen request owns its own frozen map; latest categorical veto remains after build |
| command changes during construction | final exact Runtime dry-run rejects if current bounds/inside fail; old pair remains authoritative |
| H2 pin active | no timer refresh CAS/install |
| Runtime command after prepare | existing bitwise drift rejection remains |
| final shared-pointer CAS loss | no source bookkeeping update or authority change |
| 100-cycle gate and manual selection | unchanged; dry-run/coalescing never commits Runtime |
| timer reentrancy/shutdown | one build at a time; shutdown/deactivate retains existing semantics |

## STOP and only possible next measurement

There is no safe/effective product modification in the proposed prebuild
coalescing scope, so this audit stops without a source change.

If a future owner needs to quantify whether the micro-optimization is worth
considering, the smallest new temporary measurement is a single adapter-only
record of the immutable request identity immediately before build and the
latest identity immediately after build, plus elapsed build duration.  It
must classify same-authority replacement during construction, make no
decision, run once privately, and be byte-for-byte removed.  That measurement
cannot authorise a containment/PWL relaxation or a retry policy.
