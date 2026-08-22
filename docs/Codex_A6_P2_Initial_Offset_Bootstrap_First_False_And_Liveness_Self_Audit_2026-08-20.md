# A6-P2 Initial Offset Bootstrap First-False and Liveness: Self Audit

Date: 2026-08-20  
Workspace: `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

## Result

A6-P2 is complete and stops here.  The initial active offset bootstrap is now
observably live: a certified same-owner Path--Tube Pair commits, its first
command is selected by Runtime, and a later command commits a nonzero retained
offset.  This stage does **not** claim dynamic proof of a replacement H2
commit/consume: Run 4 reached the H2 branch and attempted one point-mode stage,
but that stage returned `stage_success=0`.

## Scoped implementation

Only the A6-P2 whitelist was used for this stage's code changes:

- `gvf_manager.{h,cpp}` and `gvf_switch_policy_test.cpp`;
- `phase_offset_matched_adapter.{h,cpp}` and
  `phase_offset_matched_adapter_test.cpp`;
- this audit and the external handoff.

The manager's per-call bootstrap result keeps its outer result, existing
`PathTubePairStageFailure`, captured/current/seam phase, authority session, and
map observation sequence.  It logs a real attempted failure or a `COMMITTED`
result, without becoming authority, a gate, a latch, or a mailbox.

The committed-before-first-command lifecycle gap is covered without changing
Runtime, Tube, planner, C2, launch configuration, ROS schemas/topics,
parameters, or threads:

1. Point and closed replans capture exactly one Pair.  Either executed Runtime
   authority or that exact Pair's pending activation requires the existing H2
   handoff path.
2. A direct neutral planner install rejects the pending Pair.
3. The adapter repeats that rejection under `runtime_command_mutex_`, closing
   the check-to-retire race against a timer final CAS.
4. Generation/session one-shot lifecycle messages are evidence only.  The
   nonzero message records the pre-step retained delta and the selected
   projection's `next_delta`; it does not affect selection, authority, or the
   command.

The two deterministic regressions are:

- `GvfTimerBootstrap.PendingActivationPairRequiresH2AndSurvivesNeutralPlannerAttempt`:
  a committed but unexecuted Pair forces H2, survives neutral retirement, then
  executes and continues to require H2.
- `PhaseOffsetMatchedAdapterTest.PendingActivationPairCannotBeNeutralRetiredBeforeFirstCommand`:
  the matching Pair cannot be neutral-retired before its first selected update;
  the later normal continuous command has nonzero `projection.next_delta`.

## Verification

Final focused checks passed:

```text
catkin_make -j2 gvf_switch_policy_test phase_offset_matched_adapter_test  PASS
gvf_switch_policy_test                                                    48/48 PASS
phase_offset_matched_adapter_test                                         82/82 PASS
git diff --check                                                          PASS
```

The primary-agent final review then reran the required regression expression
from the workspace `build/` directory and the complete workspace build against
the final lifecycle-log sources:

```text
required CTest regex                                                       10/10 PASS
catkin_make -j2 (including final formation_planning relink)                PASS
git diff --check                                                          PASS
```

Static review of the new lifecycle paths found no added retry loop, sleep,
parameter tuning, message/topic/launch change, or alternate Pair publication.
`stagePathTubePair()` still has only its two production manager call sites
(bootstrap and H2), and the H2 pending/completed/consume path was not bypassed.
The workspace already contained extensive user-owned tracked and untracked
work outside this stage; it was preserved and not modified as A6-P2 work.

## Run 4: same-configuration isolated acceptance

Evidence directory:
`/tmp/a6_p2_20260820_acceptance_4`

Run 4 used a new private master at `127.0.0.1:12884`, a new `ROS_HOME`, and the
unchanged launch/override:

```text
phase_offset_esdf_tube_single.launch
phase_offset_manual_observe_only:=false
goal: (8, 0, 1), existing callback internal target: (8, 0, 2)
```

The finalized bag is
`evidence/run4_acceptance.bag` (57.206 s, 6,186 messages).  Its decisive
`/rosout` records are:

```text
COMMITTED pair_generation=1 session=3 owner_w=[4.821036776,8.326349570]
command_capture=pending_activation pair_generation=1 session=3
command_activation=selected_runtime_executed pair_generation=1 session=3 delta=0
command_activation=selected_nonzero_delta pair_generation=1 session=3
  retained_delta_before=0 retained_delta_after=-0.000384455
command_activation=not_selected_or_invalid pair_generation=1 session=3
  certificate_denied=1 fatal_control_failure=0
command_activation=selected_nonzero_delta pair_generation=2 session=3
  retained_delta_before=-0.005596105 retained_delta_after=-0.007772646
replan_branch=H2 mode=point reason=executed_authority pair_generation=2
  session=3 stage_success=0
POINT_GOAL REACHED distance=0.196
```

The certificate-denial line is an observed transient Runtime denial for a
short-horizon joint-port-polygon witness; it explicitly reports
`fatal_control_failure=0`.  It is not a claim that the H2 replacement
succeeded.  In particular, Run 4 dynamically proves the branch selection and
one stage attempt only; it does not prove H2 replacement CAS, completed
mailbox publication, mirror installation, or consume.  The existing
deterministic H2 mailbox/retry/consume regressions remain in the green 48-test
manager suite.

The bag contains 34 raw-candidate diagnostics.  Offline field checks found all
34 with `current_sample_found=1` and `current_sample_complete=1`.  It contains
one goal at `(8,0,1)`, 224 position commands, and 5,696 odometry messages.
There is no `all_candidates_path_end_clamped` log.  The terminal distance and
the existing `+1.0` goal-z mapping are consistent with the internal `(8,0,2)`
target.

Only the Run 4 recorder PGID and launcher PGID were sent SIGINT.  Final
`processes_final_check.txt` and `port_final_check.txt` are both empty, proving
no task process and no listener on port 12884 remained.

## Stop boundary

No Tube V2, recenter, M4 authority refactor, mailbox/pin cleanup, planner/C2
change, or multi-UAV work was started.  A later independently authorized stage
would be needed to address any H2 replacement-stage failure; it must not be
inferred from this successful initial-bootstrap result.
