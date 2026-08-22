# A6 dual-horizon viability / early-replan read-only audit

```text
DOCUMENT_ROLE=READ_ONLY_EVIDENCE_FIRST_AUDIT
DOCUMENT_STATUS=STOP__NO_EXECUTION_SPEC_OR_PRODUCT_CHANGE
DATE=2026-08-17
AUTO_ADVANCE=false
```

## Decision

The proposed mechanism has the right high-level separation: continue a sound
short exact-PWL witness while arranging a replacement before the local front
end is exhausted.  It is nevertheless **not implementable as a
safety-guaranteed early-replan stage under the current contracts**.  No
execution specification is issued and no product source is changed.

The independent blockers are:

1. available certified horizon is a phase length, not a bounded time-to-end;
2. replan plus H2 has observations but no execution-time upper bound;
3. the P2a mailbox means current-state recovery, not selected-owner replan;
4. Runtime/Adapter receive no goal/terminal fact.

Changing any of these is an architecture expansion.  It must not be hidden as
a Runtime gate, a relabelled `CERTIFICATE_DENIED`, a forced switch, or a new
use of the P2a recovery slot.

## Evidence

### Existing Runtime horizon

`PhaseOffsetRuntime::buildContinuousExactPwlWitness()` deliberately sets its
finite rollout horizon to `contract.tube_update_period`; the active ESDF run
uses 0.10 s.  This is a sound **short** witness, not a proof that an A*/C2/H2
replacement can finish in that period.

The same active configuration has `min_certified_forward_w=0.4`; a live pair
at the earlier H2 attempt reported `certified_forward_w=1.4803933246683609`.
Both are phase lengths, not time budgets.  The candidate's profile and exact
PWL cells could support a longer geometric trial, but no public Runtime API
exposes one.  More importantly, P1 defines finite deterministic no-witness as
a no-certificate result, not proof that the viable set is empty.  It cannot be
relabelled to force a safety transition.

### Observed H2 timing is not a deadline

In the successful private runtime-rebase episode:

| Event | ROS time |
| --- | ---: |
| first successful replan request (`remaining_w=2.058`) | 1786967536.773694957 |
| C2 connector success | 1786967536.798793651 |
| new authoritative reparameterized frontend | 1786967537.805127882 |
| next successful C2 connector | 1786967539.045699166 |
| next new frontend | 1786967539.555391170 |

The observed request-to-install durations are **1.031433 s** and **0.509692
s**.  They demonstrate that valid H2 can take far longer than the 0.10 s
Runtime witness, but they are samples, not a worst-case bound.  H2 includes
Kino/A*, C2 enumeration, immutable-cloud tube building and normal ROS
scheduling; none has an enforced wall-clock completion bound.

The FSM already asks for replanning every `planInterval=0.5 s`; it also has an
existing `governor_path_short` force-accept branch when
`remaining_w <= governor_l_max + margin_w = 1.6 + 0.2`.  The successful
episode had several earlier plan-interval replans rejected as
`reject_worse_new`.  Thus another request alone neither explains nor fixes the
former terminal HOLD.  The A6 runtime-rebase acceptance already demonstrates
valid H2 installation across the old front-end end; this audit does not reopen
that result.

### No usable time-to-boundary inequality

For an owner path, source computes:

```text
w_dot = base_w_dot + u_w
base_w_dot = K1 * (alpha + sigma) / ||r_w||
```

The projector bounds `u_w`, not total `w_dot`.  The current kernel only rejects
`||r_w|| < 1e-6`; it does not establish an immutable per-owner derivative lower
bound or a total phase-speed upper bound usable for a handoff deadline.  The
physical governor's `cmd_vel_max` is downstream and does not bound this phase
state update.  A phase remainder therefore cannot become a useful guaranteed
number of seconds before the seam/path end.  The numerical `1e-6` rejection
threshold would yield a vacuous bound that cannot cover a 20 ms callback,
much less unbounded H2 work.  Moreover `U_safe` may choose zero phase progress,
so a phase-domain certificate alone cannot provide a handoff deadline.

## Contract audit

Proposal §10.4 calls for centreline update on forward-evolution infeasibility.
§10.6 requires a sufficient preview assumption covering worst-case tube
adjustment, replanning and low-level execution distance.  §17.2 requires
positive-speed connector feasibility; §21 again calls for a new local path
candidate on preview positive-progress failure.  These are requirements for a
bounded end-to-end availability contract.  They do not authorize inferring a
deadline from an unbounded computation or overwriting a selected short witness.

`PendingCurrentStateRecoveryRequest` carries only exact owner identity and a
ticket.  Its producer accepts only `!selected && !valid`
`CERTIFICATE_DENIED` or eligible `CURRENT_OFFSET_OUTSIDE`; the FSM consumes and
deliberately discards the slot because P2b has no recovery owner.  A selected
short-witness/long-preview-missing result is neither fact.  Reusing the slot
would conflate normal replanning with unsafe/current-certificate recovery and
would require a new payload semantic plus an FSM action.

`SAFETY_PRIORITY` is also not reusable as a long-preview event: it already
means the selected current output used the nonnegative exact witness.  It says
neither that a full profile was checked nor that a replan deadline was proved.

Finally, goal-reached handling is in `gvf_manager::FSMCallback`, using the
actual goal and distance.  Runtime, Adapter and the P2a payload have no such
fact.  A local profile end therefore cannot be classified there as final-goal
terminal rather than required local-front-end replacement.  The revision-3
`CERTIFICATE_DENIED` snapshot followed legacy baseline motion for 0.631 s and
then goal arrival; that is a regression fact, not a safe terminal-owner proof.

## Required design before a later implementation stage

A later architecture proposal would first need all of the following:

1. an immutable per-owner upper bound on total phase advance and bounded
   replan/H2 service time, or a proved controlled-hold contract that keeps the
   vehicle in the exact tube until service finishes;
2. a separate typed selected-owner `replacement_requested` transport, distinct
   from P2a recovery, carrying exact owner/session identity and terminal fact;
3. explicit FSM outcomes for terminal arrival, rejected candidate and failed
   H2; and
4. a long exact-PWL oracle whose no-witness result remains an ordinary replan
   request unless search completeness and deadline assumptions are proved.

Only after that would a narrow stage whitelist plausibly include Runtime
long-trial code/tests, Adapter binding and owner-specific FSM request/consume.
Required adversarial tests would cover selected short/long-missing, stale
owner/session, terminal no-request, candidate rejection, late H2 completion,
exact-CAS preservation and no baseline publication.  Dynamic acceptance would
require replacement before a **proved** deadline, not an average latency.

## Frozen prohibitions

This audit changes none of A5 geometry/filter/margins/slope,
velocity/rate/lookahead/tracking parameters, H2 CAS/pin/session/runtime-bit,
tube/profile ownership, P2a semantics, modes, reasons, gates, diagnostics or
ROS parameters.  No old tube/profile/sample is reusable on a new connector.
