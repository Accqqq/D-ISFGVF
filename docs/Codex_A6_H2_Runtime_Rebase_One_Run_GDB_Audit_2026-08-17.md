# A6/H2 Runtime-rebase one-run GDB audit

```text
DOCUMENT_ROLE=READ_ONLY_DYNAMIC_AUDIT
DATE=2026-08-17
STATUS=NOT_CONCLUSIVE__DYNAMIC_PROBE_INVALID_FOR_ROOT_CAUSE
PRODUCT_CHANGE_AUTHORIZED=NO
FOLLOW_ON_IMPLEMENTATION_AUTHORIZED=NO
```

## Scope

This was one isolated ROS run to distinguish an H2 prepare-to-finalize
Runtime-bit drift, phase/current-`w` drift, pair race, or another failure.
It did not alter source, parameters, tube geometry, Filter, validator,
margins, H2 protocol, control selection, or diagnostic schema.

The private launch differed from the checked-in ESDF launch only by a
temporary `/tmp` `launch-prefix` which made GDB the direct parent of the
private `formation_planning` node.  It did not attach to, signal, or reuse any
user-owned ROS process.

## Result

**INVALID for H2 root-cause attribution; no dynamic conclusion is allowed.**

The run used Python GDB `Breakpoint.stop()` handlers which immediately returned
`False`; there was no `finish`, step, manual pause, or inferior function call.
Nevertheless, the released optimized binary's source-line table aliased several
requested branch-line breakpoints to high-frequency neighbouring instructions.
The recording therefore had **3,719** probe hits in 46.8 seconds.  This is far
too intrusive to represent a 50 Hz control experiment and, crucially, destroys
the requested correspondence between a labelled probe and the first false
branch.

The following four independent defects make the requested classification
invalid:

1. **O2 source-line aliasing.**  Breakpoints requested at source lines such as
   the manager finalize/return locations fired repeatedly on unrelated callback
   instructions.  Some events identify a different lexical function in GDB.
2. **Condition-before-evaluation ambiguity.**  A source-line breakpoint on an
   `if (...) return false;` is reached before its condition is known.  It is
   evidence that the condition was *tested*, not that it was false.  Thus labels
   such as `H2_PIN_ACQUIRE_FALSE` and `H2_CAPTURE_FALSE` cannot establish a
   rejection.
3. **Out-of-scope locals.**  Many aliased events report `No symbol` for the
   advertised local variables (`live_w`, `preparation`, `committed_pair`, and
   occasionally the H2 arguments).  Those values cannot be reconstructed from
   the probe record.
4. **Material perturbation.**  3,719 trap/print/continue events introduce
   scheduling delay into the very prepare/finalize interval under examination.
   The event spacing and timing cannot be used to estimate starvation,
   callback latency, or commit probability.
5. **Diagnostic-topic namespace mismatch.**  The recorder requested four
   root-level `/phase_offset_manual/...` diagnostic topics, whereas this node
   advertises them under `/formation_planning/phase_offset_manual/...`.
   `rosbag info` confirms that all four requested diagnostics have zero
   messages.  There is therefore no bag-side H2/tube-state stream with which
   to associate the physical messages.

There are two function-entry observations with readable arguments
(`captured_w0=3.4255305100`, then `4.5024801044`), but these only show that the
symbol was entered around replanning.  They do **not** prove which path returned
false or whether a path/tube pair committed.

Consequently this audit must not be cited as evidence for any of these claims:

- Runtime-bit CAS starvation is present or absent;
- phase/current-`w` drift is present or absent;
- pair/pin/mailbox race is present or absent;
- a timer refresh installed, failed, or caused a certificate denial;
- the observed physical/control performance passes or fails A6/H2 acceptance.

In particular, **no command-boundary pending commit, Runtime-mutex critical
section extension, retry, gate, or other product change is authorized from this
run.**

## Preserved valid artefacts

All task-owned processes were stopped before this report was written.  The
private master was `http://127.0.0.1:11758`; its port was released.  The
private planner had `Pid=63795`, `PPid=63745`, and `TracerPid=63745`, confirming
the temporary GDB-parent arrangement.  User-owned ROS processes were not part
of this process tree.

| Artefact | Status | Meaning retained |
| --- | --- | --- |
| `/tmp/a6_h2_runtime_rebase_gdb_20260817/evidence/a6_h2_runtime_rebase_active.bag` | valid transport capture | `rosbag info` succeeded: 46.8 s, 5,277 messages, 246.4 MB. It contains `/sim/odom`, `/position_cmd`, `/particle0/path`, `/move_base_simple/goal`, and `/sim/local_map`. The four requested root-level diagnostic names contain zero messages because the live node namespace is `/formation_planning`; it has no H2/tube-state association stream. It is not an unperturbed performance run. |
| `rosbag_info.txt`, `shutdown_trace.log`, `processes_*`, `ports_*` | valid process/cleanup record | recorder received SIGINT then was indexed; private launch and master received SIGINT afterward. |
| `formation_build_id.txt`, `input_sha256.txt` | valid provenance | formation build ID `7b4e5c488c4c5f7402c5060485a93a20dddd0ad8`; pillar map SHA-256 `609cab63db1209be0d0390931782eb702859a0d85964a7f3ff8394966ec23414`. |
| temporary launch and GDB scripts, their SHA-256 values | valid method record | preserves exactly why the dynamic probe is invalid and prevents accidental reuse as an acceptance instrument. |
| `gdb_probe_transcript.txt` / `roslaunch.stdout.log` | valid failure record | demonstrates the aliasing, condition-before-evaluation issue, and out-of-scope variables; it is not a transaction event log. |

The complete evidence directory is
`/tmp/a6_h2_runtime_rebase_gdb_20260817/evidence`.

## Separate clean acceptance

A later separately authorised **non-GDB** episode corrected the diagnostic
topic namespace and ran normal 50 Hz control.  Its independent result is
recorded in `Codex_A6_H2_Clean_Active_Acceptance_Read_Only_2026-08-17.md`.
That episode neither repairs nor reinterprets this GDB record: this document's
H2 root-cause classification remains invalid.

## Static lock-order audit

This is source inspection only, not a dynamic proof:

| Path | Observed lock sequence |
| --- | --- |
| H2 staging (`stageFutureSeamPathTubeTransaction`) | capture/acquire adapter Runtime then pin, release both, then manager handoff lock for mailbox publication. |
| H2 command boundary (`prepareAndCommitPendingPathTubeHandoff`) | manager handoff → manager authoritative phase → adapter Runtime/CAS. |
| adapter staging/prepare/finalize | Runtime → pin only for the short pin identity check; no manager lock is acquired by the adapter. |
| timer refresh | Runtime try-lock → pin identity check; it does not acquire manager locks. |
| pin release | pin registry only, after manager and Runtime scopes have left. |

No source-visible reverse cycle was found: the manager does not acquire a
handoff/phase mutex while holding adapter Runtime/pin, and the adapter does not
acquire a manager mutex.  This rules out an obvious static lock-order inversion
in the inspected paths.  It does **not** establish the absence of a schedule
race or CAS starvation; the one dynamic run is invalid for that question.

## Explicit stop

The request permitted exactly one active GDB run.  It has been consumed and
preserved.  No repeat run, source edit, parameter adjustment, or architecture
change follows from this audit.
