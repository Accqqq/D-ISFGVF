# P1 dynamic acceptance — read-only result

```text
DOCUMENT_ROLE=READ_ONLY_DYNAMIC_ACCEPTANCE
DATE=2026-08-17
RESULT=NOT_PASS__H2_INSTALL_BLOCKS_P1_PHYSICAL_ACCEPTANCE
PRODUCT_SOURCE_OR_PARAMETER_EDITS=NONE
USER_ROS_TOUCHED=NO
```

## Run identity

- Private ROS master: `http://127.0.0.1:11731`; private `ROS_HOME` and log root:
  `/tmp/p1_dynamic_acceptance_terra_20260817`.
- Unchanged launch: `phase_offset_esdf_tube_single.launch`; original `pillar.pcd`;
  its unchanged manual mode has `observe_only=true`.
- Test input only: private `/move_base_simple/goal=(8.0, 0.0, 1.0)`.
- Artifact hashes, ELF Build-IDs, launch/map hashes, bag checksum and complete raw
  evidence are in `/tmp/p1_dynamic_acceptance_terra_20260817/evidence`.
- Bag: `p1_dynamic.bag`, 38.0 seconds and 6093 messages.  It contains raw/cloud
  tube evidence, command, odometry, paths and `/rosout`.

## Observed sequence

The initial frontend ran from `phase_w=0.050` to `5.691` with
`path_end_w=5.706`.  The goal remained `d_goal=2.22 m` at terminal hold.  Kino
replanning succeeded repeatedly, including near-end replans, but the log records
`[GVF][POINT_PHASE_V2] replan not installed`.  The terminal governor then reported
`candidate_count=36`, `valid_count=0`, `path_end_clamped_count=36`, and published
`GOVERNOR_INVALID_HOLD` with
`fallback_reason=all_candidates_path_end_clamped`.

This is a real navigation failure: the old local path was consumed before a
replacement H2 path+tube authority installed.  It is not final-goal arrival.

## Tube/build facts

- `tube_raw_candidate_diagnostics`: 259 messages; cloud-snapshot diagnostics: 259.
- All 259 raw candidates were complete; all 259 snapshots were available, valid and
  usable.
- 12 build snapshots reached TubeEpoch `ROLLING`, reason `NONE`, with a complete
  candidate profile.  The rest were candidate/waiting snapshots, predominantly
  reason `FORWARD_HORIZON_SHORT` after the old path was nearly exhausted.
- No manual control diagnostics and no tube-epoch control diagnostics were published.

The missing manual/epoch topics are an observation-wiring failure, not evidence
that the initial pair was absent.  The recorded
`[GVF][POINT_PHASE_V2][INIT]` is emitted only after bootstrap
`finalizePreparedPathTubePairCommit()` CAS has succeeded, so the initial
immutable `PathTubePair` was installed.  The pair-owned branch of
`PhaseOffsetMatchedAdapter::update()` then returns immediately after filling
the output; it does not call the sole `makeControlPublishSnapshot()` site.
Timer publication of manual/epoch diagnostics requires that snapshot.  In
contrast, raw/cloud diagnostics are independently published by the timer, so
their 259 messages are expected.

Consequently the bag cannot directly test P1's cycle 1–99/100 witness
selection, one-sided selected port, or Runtime candidate-to-mailbox behavior.
The unchanged launch is also `observe_only=true`, so baseline
`VEL_MATCH_GOVERNOR` commands are not evidence of a selected P1 matched port.

## P2a classifier audit

The dynamic bag has no Runtime `CERTIFICATE_DENIED` event, because Runtime did not
receive an active pair.  Thus it cannot dynamically prove mailbox stage/consume.
Read-only source audit nevertheless finds a real candidate for misclassification:
`PhaseOffsetRuntime::complete()` changes a no-witness outcome to
`CERTIFICATE_DENIED` but does not clear the `executable=true` inherited from the
prepared normal step.  `gvf_manager::requiresCurrentStateRecovery()` requires
`!output.runtime_execution.executable`, so such a real no-witness denial would not
stage the P2a mailbox.  This is a source fact, not a claimed dynamic reproduction.

## Acceptance

| Required observation | Result |
| --- | --- |
| 100-cycle P1 arming and selected continuous witness | NOT OBSERVABLE: pair-branch control diagnostics are not published |
| One-sided tube selected port | NOT OBSERVABLE: pair-branch control diagnostics are not published |
| Replacement H2 installation before old-path exhaustion | FAIL |
| No `all_candidates_path_end_clamped` while goal remains far | FAIL |
| Dynamic `CERTIFICATE_DENIED.executable` and P2a mailbox result | NOT OBSERVED |
| A5 build/filter/map evidence | PASS for this episode; not the blocking layer |

No product conclusion beyond this boundary is justified.  The next corrective
work must first restore/install the replacement H2 authority and separately test
the identified Runtime/P2a executable-status classification; it must not alter
the tube geometry, filter, margins, or add a gate.

## First missing-link audit

Run evidence rules out a launch/topic-subscription error and rules out absence
of the old active authority:

- The topics were advertised before recording, and raw/cloud values arrived.
- Bootstrap logged `POINT_PHASE_V2 INIT`; source control flow proves the initial
  pair CAS had succeeded before that line.
- The H2 attempt logged seven `C2 connector_success` results, at seam phases
  `4.912` through `5.210`, followed by `replan not installed`.

Thus the first unobserved product failure is after C2 construction, inside
`stagePathTubePair()` or the immediately following prepared-tube/dry-run/commit
steps.  It is not "no old active pair" and not an A5 build, Filter, map or
diagnostic subscription failure.  Existing logs do not distinguish these
internal false returns.

A second private launch was created only to attach GDB before publishing a goal;
it was stopped without a flight when Linux Yama policy denied same-user attach
and passwordless privileged attach was unavailable.  No user process was
attached.  The attempted GDB transcript and the second run's identity evidence
are in `/tmp/p1_h2_gdb_20260817/evidence`.  No additional same-parameter flight
was run.  A future dynamic probe requires an authorised task-owned launch-under-
debugger arrangement or an existing non-product observation seam; neither was
created here.

## Existing active-launch audit

No pre-existing launch supports a genuine active P1 ESDF selected-port run
without an override:

- `phase_offset_active_zero_single.launch` selects `mode=active`, but leaves the
  included launch's `tube_source=none`.  Independently, source defines the
  active branch as zero-port equivalence and returns before tube-timer/H2
  Runtime composition; it cannot demonstrate a one-sided ESDF tube.
- `phase_offset_esdf_tube_single.launch` is the only existing single-UAV ESDF
  entrypoint.  It is manual mode but has `manual_observe_only=true` as its
  unchanged default.  Its only non-observe option is a command-line launch-arg
  override, prohibited for this acceptance task.
- `phase_offset_fixed_tube_single.launch` is both fixed-tube and observe-only;
  `phase_offset_manual_single.launch` inherits `tube_source=none` and
  observe-only defaults.  Existing agent/swarm `enable_tube` flags are not this
  Runtime ESDF configuration.

The requested 100-cycle selected-port/commit/physical-command acceptance is
therefore `CONFIG BLOCKED`, not `NOT RUN for lack of effort`.  Running the
active-zero or observe-only launch would not answer it and must not be reported
as a P1 one-sided-tube result.  A later authorised execution specification must
name an existing non-observe ESDF configuration or explicitly authorise one;
this read-only task neither created nor overrode one.

## Authorised active execution addendum

```text
DOCUMENT_ROLE=DEDICATED_DYNAMIC_ACCEPTANCE_ADDENDUM
DOCUMENT_STATUS=AUTHORIZED_BY_USER_2026-08-17
AUTO_ADVANCE=false
ONLY_LAUNCH_OVERRIDE=phase_offset_manual_observe_only:=false
ALL_OTHER_LAUNCH_ARGUMENTS=UNCHANGED_DEFAULTS
PRODUCT_SOURCE_OR_LAUNCH_FILE_EDITS=FORBIDDEN
```

This narrow override activates the existing manual+ESDF execution mode; it does
not change a safety, geometry, Filter, margin, speed, lookahead, rate, tracking
or map parameter.  Before the active run the executor must save the launch and
map hashes, ELF Build-IDs, complete live rosparam dump, and a hash of the dump.
The run uses a task-owned master, `ROS_HOME`, log directory and bag only.  It
must stop after one episode; a failure becomes evidence for a separate plan,
not a reason to retry, tune or add gating.

## Authoritative active-run result (supersedes the prior config-blocked finding)

```text
RUN=ONE_PRIVATE_MANUAL_PLUS_ESDF_EPISODE
PRIVATE_MASTER=http://127.0.0.1:11733
ONLY_OVERRIDE=phase_offset_manual_observe_only:=false
PRODUCT_SOURCE_OR_LAUNCH_FILE_EDITS=NONE
USER_ROS_OR_PROCESSES_TOUCHED=NO
RESULT=NOT_PASS__H2_REPLACEMENT_INSTALL_FAILED
```

The launch used exactly the addendum's one authorized override.  The complete
parameter dump and its hash, pre-run source hashes, relevant ELF Build-IDs,
pre-debug ROS graph, debugger transcript and 613-second bag are retained under
`/tmp/p1_active_dynamic_20260817/evidence`.  The bag SHA-256 is
`c3fbd404b6efdf5875ce6d8e064691675a6f97bcdc0abd602da57b7c4cb13087`.

### What this run positively establishes

- Bootstrap was healthy: `stagePathTubePair()` reported
  `buildPreparedTubeEpoch=true`, `dryRunPreparedRuntime=true`, and success;
  `[GVF][POINT_PHASE_V2][INIT]` subsequently appeared.  Thus the initial
  immutable path+tube pair was actually installed.
- The active P1 gate opened: GDB observed `zero_gate_open_=true`,
  `zero_gate_consecutive_count_=100`, and `failure_latched_=false`.  The live
  Runtime retained state was nonzero (`delta=-0.035817306589090242`, previous
  final port `{u_w=0.016437002918046038, u_delta=-0.21862238886092472}`), which
  is direct evidence that active Runtime commits occurred.
- The pair at the replacement attempt was current-valid, not certificate
  denied, and had reason `NONE` with `certified_forward_w=1.4803933246683609`.
  This episode is therefore not a TubeFilter, A5 geometry, epoch-certificate,
  map-snapshot, or bootstrap-build rejection.

The pair-owned control branch still does not publish the manual/epoch snapshot,
so those two empty diagnostic topics remain a telemetry-wiring limitation.  The
debugger facts above, not those empty topics, establish the active P1 state.
`final_cmd_source=VEL_MATCH_GOVERNOR` is an implementation label for the
governor consuming matched guidance; by itself it is not evidence that P1 was
unselected.

### Decisive H2 first-false observation

For the observed C2 replacement, `stagePathTubePair()` reached the exact-CAS
check at `phase_offset_matched_adapter.cpp:1577` and then returned `false`
(`gdb` transcript value `$59`).  Before that check, the capture and live pair
agreed on source revision (1), generation (5), authority session (1), map
observation sequence (573), snapshot flag, active lease (2), active-pair
identity, path owner, cloud snapshot, full samples, profile, and epoch
snapshot.  The GDB transcript records these equalities.

The Runtime state captured at pin acquisition did not equal the live state when
the new C2/prepared replacement reached the check:

| Runtime field | captured | live at exact-CAS check |
| --- | ---: | ---: |
| retained delta | -0.0037596066975834628 | -0.035817306589090242 |
| previous final port `u_w` | 0.0014675412986572556 | 0.016437002918046038 |
| previous final port `u_delta` | -0.074337806486138358 | -0.21862238886092472 |

The condition uses bitwise equality for those values, so this Runtime snapshot
drift is the first false predicate for this replacement call.  It is not
evidence that every possible H2 failure has the same cause, nor does it identify
which intervening command cycle performed the write.

After the false return, the recorded logs show C2 `connector_success`, then
`replan not installed`, and finally old-path exhaustion:
`candidate_count=36`, `valid_count=0`, `path_end_clamped_count=36`,
`fallback_reason=all_candidates_path_end_clamped`, and `d_goal=2.37 m`.  This
is a navigation failure away from the goal.  The later
`GOVERNOR_INVALID_HOLD` is the fail-closed consequence of the failed H2
replacement, not a reason to weaken the transactional check or reuse the old
tube on the new path.

### Acceptance boundary after the active run

| Required item | Result |
| --- | --- |
| Initial path+tube install | PASS for this episode |
| P1 100-cycle arming and active Runtime commit | PASS for this episode |
| Replacement H2 exact-CAS installation while Runtime is active | FAIL |
| H2 replacement before old-path exhaustion | FAIL |
| P2a `CERTIFICATE_DENIED` mailbox stage/consume | NOT OBSERVED |
| Complete physical closed-loop mission | NOT PASS |

This evidence authorizes no code change by itself.  A correction must preserve
the safety meaning of H2 exact identity and address the ownership/transaction
of changing Runtime state.  It must not pause Runtime, weaken `BitsEqual`, reuse
an old tube for a new path, add a retry gate, or tune geometry/filter/margins to
hide the failure.

## Runtime-rebase active H2 revalidation

```text
RUN=ONE_PRIVATE_MANUAL_PLUS_ESDF_EPISODE
PRIVATE_MASTER=http://127.0.0.1:11735
ONLY_OVERRIDE=phase_offset_manual_observe_only:=false
OTHER_PARAMETERS=UNCHANGED_DEFAULTS
PRODUCT_SOURCE_OR_LAUNCH_FILE_EDITS_DURING_RUN=NONE
RESULT=PASS_FOR_THIS_H2_REPLACEMENT_EPISODE
```

This is a separate, task-owned run of the subsequently rebuilt runtime-rebase
implementation.  The complete artifact root is
`/tmp/p1_active_runtime_rebase_20260817/evidence`; it contains the active bag,
GDB transcript and command file, parameter dump, and run identities.  The
bag is `p1_active_runtime_rebase.bag` (217 seconds, 22,184 messages) with
SHA-256 `5060f746061b07046af7308b0044b32678e3fac500fdb31d8ec398316288474b`.

### Rebuilt identity and preflight

- `phase_offset_matched_adapter_test`: **58/58 passed**.
- Adapter library SHA-256:
  `9be7371a9fe5fd0f920b7018970a62fab054f8e3c96f7f2eb63b484b46901317`; Build-ID
  `c4737baab2c98f92121c0cf398c6f7d1d66b6b39`.
- `libbspline_gvf.so` SHA-256:
  `da75f36ddf969905b36b0a536cc3703430f22080c95f689b7e06d0c39533f850`; Build-ID
  `06f2f295dbe5e660493771515de06053a2e8d450`.
- `formation_planning` SHA-256:
  `0c7c9f6e07b4b3743e507c2a04d87420134b9c4d03a870051048aaebeb004a13`; Build-ID
  `5f67f74129946f96c33e94b0a5cf5b6a016a7a57`.
- The live ROS parameter dump SHA-256 is
  `b49888cc6268374368cfd3877bcda75845968512e2c8795a219713d68ebea4ad`.
  The unchanged `pillar.pcd` and launch hashes are respectively
  `609cab63db1209be0d0390931782eb702859a0d85964a7f3ff8394966ec23414` and
  `334d807751ad6aa1739c241f07b5e22c554a3ff2470838e975a9cad8009e3813`.

### H2 evidence

The run first logged `POINT_PHASE_V2 INIT` with old-path end `w=5.707`.
At least one nonzero retained Runtime delta and previous final port were
observed before the replacement, which is direct runtime-state evidence of
active P1 commits (the pair-branch telemetry wiring still does not publish its
separate manual/epoch diagnostic snapshot).

For the first successful replacement, GDB recorded a pin for old pair
`source_revision=1`, `generation=5`, `authority_session=1`, map sequence 165,
with historical Runtime state
`delta=-0.0077726461716244137` and port
`{u_w=0.003254899959716619, u_delta=-0.10882708163942315}`.  At prepare, it
re-read and certified the latest Runtime state rather than treating the pin's
history as an immutable install predicate:

| point | retained delta | previous final port |
| --- | ---: | --- |
| pin history | -0.0077726461716244137 | `{0.003254899959716619, -0.10882708163942315}` |
| prepare expected | -0.023335451423984764 | `{0.010489187715407708, -0.18442507274493483}` |
| finalize live | -0.023335451423984764 | `{0.010489187715407708, -0.18442507274493483}` |

Thus the Runtime changed after the historical pin, but the new-owner prepare
step certified the latest state, and the final exact-bit predicate still held.
The debugger reached `H2_FINALIZE_CAS_SUCCESS`: it installed
`source_revision=2`, `generation=6`, `authority_session=1`, map sequence 167,
with prepared range starting `w=4.7244765782270788` and seam
`w=5.160740816946233`.  A later replacement also reached a CAS success
(`source_revision=3`, `generation=9`, map sequence 190).  This is a real
authority replacement, not reuse of the old tube or a retry gate.

The live command log then reported a new active range
`path_w_start=4.724, path_w_end=8.424`, followed by phase/path use past the
old endpoint (`w=7.974 > 5.707`) while `d_goal=1.07 m`, with
`final_cmd_source=VEL_MATCH_GOVERNOR`, `fallback_reason=none`, positive valid
candidate counts, and `fallback_hold_pos=0`.  It subsequently reached the
goal (`distance=0.176`).  Hence the successful replacement continued physical
matched/tube guidance across old-path exhaustion before terminal arrival; it
was not silently left on the old baseline path.

### Episode statistics and scope

| Observation | Count/result |
| --- | --- |
| `POINT_PHASE_V2 INIT` | 1 |
| C2 `connector_success` | 2 |
| GDB stage / prepare / finalize observations | 4 / 4 / 4 |
| GDB successful H2 CAS publications | 4 (including bootstrap and replacements) |
| `replan not installed` | 4 early individual switch attempts (`reject_worse_new`); later CAS and new active ranges prove they did not prevent the successful replacement |
| `all_candidates_path_end_clamped` | 0 |
| `CERTIFICATE_DENIED`, `CURRENT_OUTSIDE`, P2a mailbox stage/consume | 0 / 0 / 0 (not exercised) |
| `GOVERNOR_INVALID_HOLD` | 6, all pre-initial-install `guidance_invalid` entries; no path-end HOLD after H2 success |
| Explicit `tracking=` telemetry | not published in this episode; no S4 numerical claim |

This validates the runtime-rebase mechanism for the observed private episode:
historical pin identity remains strict, latest Runtime state is re-certified at
prepare, and only prepare-to-finalize bit drift is CAS-rejected.  It does not
turn the P2a no-witness path, tracking S4 bound, every replan policy outcome,
or all future H2 timing cases into PASS.  No new gate, mode, reason, parameter,
or tube-geometry/filter change was introduced by this acceptance run.

## Active-pair diagnostics supplemental read-only analysis

The separate private diagnostics episode retained at
`/tmp/active_pair_diagnostics_20260817/evidence/active_pair_diagnostics.bag`
confirms that the pair-owned control publication path is no longer silent: it
contains 6 manual 83-field and 6 epoch 50-field messages.  These are event
snapshots, not 50 Hz samples.  The existing timer publishes them only when a
completed epoch agrees with the latest command snapshot; raw/cloud evidence is
published independently and has 35 rows in the same active window.

The only tracking scalar in both schemas is current
`RuntimeExecutionStatus::tracking_error_norm`; there is no separate current or
max-tracking field.  Across the six snapshots it has p50/p95/recorded-max
0.05448/0.06789/0.06789 m.  Among the three strict selected NORMAL snapshots
it is 0.05079/0.05448/0.05448 m.  Accordingly S4 `<0.05 m` remains **NOT PASS**:
the values themselves exceed the threshold and the six event samples cannot
prove a continuous-time maximum.  This supplemental observation changes no
prior A5, H2, P2, or physical-closed-loop acceptance conclusion.

The final snapshot at `1786968550.868336678` is a concrete P2a-r1 shape:
source revision 3, selected/valid false, Runtime `CERTIFICATE_DENIED`, all
current tube containment facts true, retained delta -0.02457, tracking 0.06051
m within its 0.15 m Runtime bound, and zero final port.  Its current
`executable` bit is not telemetry, but source semantics preserve the already
prepared executable bit when both exact-PWL forward witness attempts fail.
Consequently P2a's classifier would require recovery for this output if the
captured pair remained live.  The bag has no mailbox ticket/stage/consume
telemetry, so neither an actual P2a stage nor a consume can be claimed.

Critically, this was not a P2b recovery episode: 32 `PositionCommand`s followed
over the next 0.631 seconds and the log remained
`VEL_MATCH_GOVERNOR`/`fallback_reason=none` until the goal.  Source review shows
that an unselected matched output leaves the precomputed baseline lifted
guidance and zero-port governor anchor in place.  The continued motion is
therefore legacy baseline-governor fallback on the current path owner, not a
certified matched-port command and not an authorised physical recovery.
