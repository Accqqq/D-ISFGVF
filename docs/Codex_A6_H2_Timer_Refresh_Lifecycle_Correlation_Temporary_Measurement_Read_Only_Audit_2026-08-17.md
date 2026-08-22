# A6/H2 timer-refresh lifecycle correlation — read-only audit

```text
DOCUMENT_ROLE=READ_ONLY_TEMPORARY_MEASUREMENT_AUDIT
DATE=2026-08-17
STAGE=A6_H2_TIMER_REFRESH_TO_SEAM_AVAILABILITY_CORRELATION
PRODUCT_CHANGE=NONE
RESULT=VALID_LIFECYCLE_MEASUREMENT__DOWNSTREAM_NONEMPTY_SELECTOR_NOT_OBSERVED
```

## Method and restoration

Exactly one private ESDF active episode used `http://127.0.0.1:11763`, a
private ROS home/log directory, the checked-in
`phase_offset_esdf_tube_single.launch`, and only the authorised
`phase_offset_manual_observe_only:=false` override.  It published one
`(8,0,1)` goal and stopped 70 seconds later.  It did not attach to, modify,
or stop any user ROS process.

Evidence is retained at
`/tmp/h2_lifecycle_correlation_20260817/evidence`.  The bag is 76 seconds,
422.4 MB, and contains 14,681 messages.  The task-owned master, launch,
planner/simulator descendants and recorder were stopped; the post-shutdown
port capture has no `11763` listener.

Temporary `TEMP_H2_LIFECYCLE` stderr statements changed no branch condition,
lock, state write, ROS schema/topic, parameter, gate, retry, or physical
control output.  Immediately after the episode they were removed.  Restoration
is byte exact:

```text
gvf_manager.cpp                 078ae5378b49b7942b12ecef927f928f70fda0b0f5642d386691e650fdb07c18
phase_offset_matched_adapter.cpp f6b7b0a5fdf77ece8372175c0a949c6b40518bef268b26e6b7229fee72450521
phase_offset_matched_adapter.h   414015f908eecc51c75bd226a25163b96673b11f4016864bd67ef2447d24d7c1
TEMP_H2_LIFECYCLE source marker  absent
restored libbspline_gvf Build-ID e742720e7863e4f3770efa5abe2f8cb05ccfa817
restored adapter Build-ID        432e58b750d280b0a4c8cbdd39b6a136f29cdf05
```

The instrumented Build-IDs are preserved in the evidence directory:
`bspline_gvf=1fc75af4ab5b29b8d1566ba8e9e2a0685214f107` and
`adapter=9c6f5c712856b82ddba8661fac8b30195bd1fa44`.

Focused restored regressions all passed: adapter 63/63, epoch diagnostics
5/5, epoch integration 10/10, GVF/H2 74/74, and continuous path 6/6.
`git diff --check` passed.

## Measured facts

The transcript has 3,138 lifecycle lines and its timer-build aggregate is:

| fact | count |
| --- | ---: |
| timer build completions | 750 |
| `update_ok=1` | 14 |
| same-owner refresh installed | 9 |
| refresh prepare false | 737 |
| timer-finalize false | 4 |
| `CURRENT_OFFSET_OUTSIDE` (reason 6) build failures | 9 |
| `FORWARD_HORIZON_SHORT` (reason 11) build failures | 727 |

Thus this episode does establish that same-owner refresh can install later
complete profiles (generation 1 through 10) and that it is not universally
blocked by build completion or the final CAS.  For example, generation 5 had
certified tail `3.473554`; a later valid build installed generation 6 with
tail `5.706307`.

It does **not** reproduce the prior nonterminal condition in which the H2
selector had surviving seams (the earlier counter run recorded 18 at
`captured_w=4.450337`).  Here all 139 selector calls had zero survivors:

| captured w | required seam | active certified end | result |
| ---: | ---: | ---: | --- |
| 3.439005 | 3.839005 | 3.473554 | zero survivors |
| 4.495139 | 4.895139 | 3.473554 | zero survivors |
| 5.515939 | 5.915939 | 5.706307 | zero survivors |
| 5.689507 | 6.089507 | 5.706307 | zero survivors |

The first two rows occur while completed Candidate profiles existed but were
not Active because the retained port was outside the current interval (reason
6).  The later valid active profile arrived only after phase had advanced to
the terminal region, where the required future seam lay past path end (reason
11).  No non-bootstrap selector emitted a C2 attempt, `stagePathTubePair`
attempt, pending-slot handoff, or H2 commit in this episode.

## Conclusion and required next authority

This run neither implicates nor clears the downstream C2/adapter lifecycle
for the previously observed 18-survivor case: its necessary premise did not
occur.  It does rule out using this run as evidence for a product repair,
parameter change, added gate, A5 geometry change, Filter change, or H2
condition change.

The minimum next action, if separately authorised, is a read-only,
single-episode reproduction specification whose stop condition requires the
already-observed nonterminal `selector_survivors > 0` premise.  It must then
correlate that one transaction through C2, `stagePathTubePair`, manager slot,
prepare/finalize, and consume.  Until such evidence exists, no downstream
lifecycle repair is justified.
