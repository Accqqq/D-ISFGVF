# A6/H2 nonterminal seam-counter — one-run read-only audit

## Findings first

The nonterminal `no_certified_future_seam` calls were classified. Their first
elimination layer in this one episode was the existing **certified-end
precheck**, not a structural failure, malformed sample order, owner mismatch,
or `TubeFilter::query` rejection.

| Captured phase | Minimum required seam | Active certified end | First result |
| ---: | ---: | ---: | --- |
| 3.400860 | 3.800860 | 3.771396 | reject: certified end too short |
| 4.450337 | 4.850337 | 5.706521 | 18 seams survive every selector filter |
| 5.474848 | 5.874848 | 5.706521 | reject: certified end too short |
| 5.689123 | 6.089123 | 5.706521 | reject: terminal certified/path headroom exhausted |

Thus “no certified future seam” is not a fixed A5/Filter geometric rejection.
It occurs when the installed active profile's certified tail has not reached
the required H2 forward seam. In the first row, the immutable path still ends
at 5.706521 (2.306 phase units remain), yet the active certified interval ends
at 3.771396—only 0.029464 before the required seam. This is nonterminal
certificate-headroom unavailability.

No product repair is implemented or authorized by this audit.

## Scope and private-run integrity

This follows the dedicated execution specification
`Codex_A6_H2_Nonterminal_Seam_Counter_Temporary_Measurement_Execution_Spec_2026-08-17.md`.

- exactly one private active episode ran at
  `/tmp/h2_nonterminal_seam_counter_20260817/evidence`;
- private master: `127.0.0.1:11762`; it is absent after cleanup;
- only runtime override: `phase_offset_manual_observe_only:=false`;
- one `(8,0,1)` goal and a 70-second post-goal maximum; bag duration is 76 s
  including startup/teardown and has 14,682 messages;
- no user-owned ROS process was stopped or modified.

The forward instrumentation was confined to `gvf_manager.cpp` and printed a
fixed `TEMP_H2_SEAM_COUNTER` line from the low-frequency selector only. It
did not change any branch condition, return, lock, parameter, schema, enum,
gate, retry, state, Runtime value, tube, C2 path, or physical command.

## Selector evidence

All four representative calls had a structurally valid pair:

```text
old owner/full-sample range = [0.050000, 5.706521]
old full sample count        = 115
min_certified_forward_w      = 0.400000
ordered sample validation    = pass
```

At `w=3.400860`, the active profile sample/certified range was
`[1.637796, 3.771396]`. Since `3.771396 < 3.800860`, the selector returned
before evaluating any per-sample phase, certificate-interval, owner, or tube
filter predicate.

At `w=4.450337`, the active range had refreshed to
`[4.069107, 5.706521]`. Eighteen samples survived phase threshold,
certificate interval, exact owner equivalence, and valid `TubeFilter::query`;
the selector returned those 18 seams, from 4.863005 through 5.706521. The log
then shows C2 connector successes at 4.863, 4.913, and 4.962. A new frontend
was nevertheless not installed later in that replan. This measurement does
not instrument the post-selector C2/adapter transaction, so it cannot assign
that downstream failure to a particular layer.

At `w=5.474848`, the same active certificate end 5.706521 was below the
required 5.874848. The terminal call similarly failed the precheck, with the
required seam beyond both certificate and path end.

## Relationship to the prior trace

The prior trace reported nonterminal empty calls near 3.3757, 4.4238, and
5.4196. This single, later episode observed nearby but not bit-identical
phases. It directly proves the first elimination layer for its 3.4009 and
5.4748 empties; it does not prove that the earlier 4.4238 call had the same
active profile. The 4.4503 counterexample proves that a nearby phase can have
a fully viable seam set after the active profile advances.

Consequently, the safe conclusion is temporal: availability depends on the
then-current active profile certificate horizon, rather than on an immutable
path/Filter impossibility at every nonterminal phase.

## Minimal next action: STOP pending a dedicated lifecycle plan

No selector condition should be relaxed: the precheck correctly refuses a
handoff when its required old-prefix certificate does not exist. Adding a
fallback seam, reducing the 0.4 requirement, reusing a different owner,
adding a retry/gate, or changing A5/Filter/C2 would be an unproven policy
change and is outside this stage.

If the user authorizes follow-up, the minimal next investigation is not a
geometry rewrite. It is a separate one-run timer-lifecycle measurement that
correlates, at each replan, the pair's active profile certificate tail with:

```text
timer due time / request identity
frozen map sequence
build start and finish
same-owner refresh prepare/final-CAS outcome
active pair generation/source revision
active profile certified start/end before and after refresh
replan capture w and required seam w
```

Only after that evidence can a new execution specification decide whether the
minimal repair belongs in timer scheduling/refresh availability, replan timing,
or another already-approved path-continuation mechanism. This audit makes no
such repair.

## Mandatory restoration and regression evidence

The temporary marker was removed unconditionally. Product source is restored
byte-for-byte:

```text
gvf_manager.cpp SHA-256
078ae5378b49b7942b12ecef927f928f70fda0b0f5642d386691e650fdb07c18

rg TEMP_H2_SEAM_COUNTER src/swarm_planner/bspline_traj
# no output
```

Restored builds return the previous Build-IDs:

```text
libbspline_gvf.so       e742720e7863e4f3770efa5abe2f8cb05ccfa817
formation_planning      7b4e5c488c4c5f7402c5060485a93a20dddd0ad8
```

Focused restored-product tests passed:

- `phase_offset_matched_adapter_test`: 63/63
- `phase_offset_tube_epoch_diagnostics_test`: 5/5
- `phase_offset_tube_epoch_integration_test`: 10/10
- `gvf_switch_policy_test`: 74/74
- `continuous_phase_path_test`: 6/6

`git diff --check` passes. The temporary selector instrumentation leaves no
product source change; only this audit and its execution specification remain.

