# Tube V2 G18 Pair visibility and H2 first-false static self-audit

    DOCUMENT_ROLE=STATIC_IMPLEMENTATION_SELF_AUDIT
    DOCUMENT_STATUS=COMPLETE
    DATE=2026-08-22
    STAGE=G18
    STATIC_RESULT=PASS
    DYNAMIC_RESULT=NOT_RUN
    NEXT_OWNER=PRIMARY_CODEX_REVIEW_THEN_BOUNDED_G1_G5_DYNAMIC

## 1. Result

G18's bounded static implementation is complete.

- An exact current authoritative `PathTubePair` is now a second legal
  publication owner in `publishManual()` without weakening the ordinary timer
  epoch/request/map checks.
- A Pair command snapshot takes Candidate identity from its own epoch Candidate
  and Active identity from the Pair active profile.  This preserves the legal
  timer-refreshed Candidate/Active pointer separation while satisfying the
  existing complete `output_matches_epoch` predicate.
- For an exact live Pair, the Candidate marker alone is overridden by the
  latest authoritative timer Candidate when present.  Certified geometry,
  paths, frame, manual diagnostics, and epoch diagnostics continue to use the
  Pair command snapshot and the unchanged `tubeDisplayCertified()` predicate.
- Without a timer Candidate, the Pair profile's preserved
  `raw_build_samples` supplies broad Candidate geometry; filtered Pair
  endpoints are not promoted to Candidate geometry.
- Stale, replaced, retired, or malformed Pair snapshots continue through the
  fail-closed Certified DELETE path and cannot replace the latest Candidate.
- Production H2 staging now passes the existing
  `PathTubePairStageFailure` output through the manager boundary.  Both
  existing lifecycle logs name the enum; a failed stage boundary also records
  Pair generation/session, captured phase, seam, and owner end.
- Only `STAGING_DRY_RUN` emits additional detail, entirely from the already
  computed `RuntimeDryRunResult` and already captured transaction values.

No navigation-liveness, recovery, recenter, authority-reset, neutral-control,
planner, governor, retry, limit, margin, rate, horizon, launch, parameter, map,
CMake, or ROS-message change was made.

## 2. Product changes

### Adapter publication composition

- `phase_offset_matched_adapter.cpp:3316` binds Pair command Candidate to the
  Pair epoch Candidate when present and leaves Runtime/prepare on the Pair
  active profile.
- `phase_offset_matched_adapter.cpp:3623` adds a stateless marker overload so
  only Candidate marker geometry can use a different immutable profile.
- `phase_offset_matched_adapter.cpp:3748` validates the exact live Pair's
  request pointer, generation, authority session, task/source/map identity,
  epoch pointer, Candidate/Active pointers, and existing status identities.
- `phase_offset_matched_adapter.cpp:3872` requires the existing
  `output_matches_epoch` predicate before the Pair alternate can be accepted.
- `phase_offset_matched_adapter.cpp:3901` keeps the ordinary timer owner
  predicate unchanged and accepts only either that owner or the exact Pair.
- `phase_offset_matched_adapter.cpp:3936` chooses the timer Candidate only for
  Candidate marker construction, then runs the existing common publication
  and diagnostics tail.

### H2 attribution

- `phase_offset_matched_adapter.cpp:2564` logs existing dry-run result fields
  only on the existing `STAGING_DRY_RUN` false branch.
- `gvf_manager.cpp:3247` passes one local existing failure enum to
  `stagePathTubePair()` and logs the stage boundary without changing its
  boolean result or caller output preservation.
- `gvf_manager.cpp:7382` and `gvf_manager.cpp:7593` pass that local enum to the
  existing closed/point H2 lifecycle logs.
- `gvf_manager.h:650` adds only the optional transaction-local output pointer;
  no manager state or mailbox field was added.

## 3. Focused proofs

The Pair publication focused suite has six tests and passes 6/6.

1. Exact Pair plus distinct newer timer Candidate produces Candidate ADD with
   width 6.0 m and Certified ADD with width 0.20 m; pointers are not swapped.
2. Exact Pair without a timer Candidate produces a 5.0 m Candidate from
   `raw_build_samples` and a 0.20 m Certified marker.
3. A copied/refreshed epoch is accepted only with the exact current Pair
   pointer and all epoch/profile identities.
4. Stale, replaced, retired, and malformed Pair cases reject certification;
   latest Candidate remains ADD while Certified is DELETE.
5. Ordinary marker composition through the new overload is semantically
   identical to the prior path.
6. A real ESDF timer refresh preserves distinct Candidate/Active pointers,
   satisfies the exact Pair and epoch identities, publishes broad Candidate
   ADD and narrow 0.20 m Certified ADD, and leaves Runtime on Pair Active.

The replacement dry-run attribution test calls the same failing transaction
twice.  Both calls return false with `STAGING_DRY_RUN`; Pair pointer, Runtime
retained delta, previous final port, and the transaction pin remain unchanged.
Existing bootstrap attribution, later-callback seam behavior, and H2 mailbox
tests remain green.

## 4. Static verification

Evidence root:

    /dev/shm/tube_v2_g18_static_20260822_190146/

Direct final suites:

| Suite | Result |
|---|---:|
| Matched Adapter, including G18 Pair/H2 tests | 94/94 |
| GVF switch policy and H2 lifecycle | 49/49 |
| Tube markers | 14/14 |
| Tube EpochManager | 55/55 |
| Tube epoch integration | 10/10 |
| Tube epoch diagnostics | 5/5 |
| Runtime | 35/35 |
| G17 CertifiedBuilder regressions | 20/20 |
| Total | 282/282 |

The G17 cost witness remains unchanged:

    raw_queries=864
    narrow_queries=99
    raw_geometry_cells=181
    narrow_geometry_cells=11
    raw_v_splits=85
    narrow_v_splits=0

Full incremental build:

    catkin_make -j2 = PASS

Integrity:

- `git diff --check`: empty.
- Frozen Builder/Filter/CertifiedBuilder/SurfaceValidator/Runtime/
  PortProjector plus launch/YAML manifest: 47/47 hashes unchanged.
- No new publisher, subscriber, timer, mutex, atomic state, gate, cache, retry,
  controller, recovery owner, limit, margin, rate, or horizon was added.
- The only new `atomic_load` found by the mechanical symbol audit reads the
  already-existing authoritative Pair slot transaction-locally; it creates no
  state.
- Source changes from the G18 start snapshot are limited to the adapter
  implementation/header, manager attribution implementation/header, this
  adapter test, this self-audit, and the handoff entry.  The existing GVF test
  file was not changed.

The separately classified pre-existing SurfaceValidator fixture was neither
modified nor counted as a G18 failure.  G18 did not run a clean build and did
not write large logs to the root filesystem.

## 5. Stop boundary

No ROS dynamic run was started.  Static completion does not claim navigation
recovery or system `FIXED`.  The primary agent must perform independent review
before authorizing the unchanged split-launch G1--G5 dynamic verification.

    G18_STATIC_IMPLEMENTATION_PASS
    STOP__AWAIT_PRIMARY_REVIEW_AND_DYNAMIC_AUTHORIZATION
