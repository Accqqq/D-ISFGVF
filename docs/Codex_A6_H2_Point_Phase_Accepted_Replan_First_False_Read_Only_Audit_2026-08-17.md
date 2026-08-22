# A6/H2 point-phase accepted replan first-false — read-only audit

## Findings first

The point-phase policy accepted the candidate at `phase_w=4.453`, but the
H2 transaction did not stage a new frontend.  This is **not** evidence that
the policy rejected the replan, that A5/Filter geometry failed, or that the
remaining old-path headroom was below the H2 minimum.

The exact 11764 transcript records:

```text
[GVF][POINT_PHASE_V2] replan not installed; keep current frontend and phase_w=4.453
[GVF][SWITCH] accept=1 reason=accept_collision&timout J_old=0.000 J_new=0.000
```

The same episode configured `min_certified_forward_w=0.400`.  The installed
old path ended at `w=5.706`; consequently the scalar certified-forward
precheck only required `w >= 4.853` and had approximately `1.253 m` of
remaining old-path range.  It is not the terminal `all_candidates_path_end_clamped`
condition seen later at `w=5.692`.

The earlier, separate one-run nonterminal selector audit provides relevant
but deliberately non-identical corroboration.  At `w=4.450337`, it found
18 exact old-owner/profile seam samples over `[4.863005, 5.706521]`, and
observed C2 successes for its first three attempted seams.  That replan also
did not install.  It proves that nearby phase values can have certified seams
and viable C2 connectors; it does **not** prove the identity or outcome of
the 11764 `w=4.453` call.

## Exact classification boundary

`stageFutureSeamPathTubeTransaction()` has no per-return diagnostic.  In the
11764 transcript there is neither a `connector_success` nor a
`connector_failed` line for the accepted `w=4.453` replan.  Since
`buildPhaseV2C2Frontend()` can return before either of those logs, this
absence does not distinguish a pre-C2 transaction return from an early C2
precondition return.

The available record therefore cannot uniquely identify the first false
among these existing branches:

1. no valid captured pair pin;
2. captured old-pair/session invariant or occupied manager handoff slot;
3. no certified seam candidates for this exact immutable pair;
4. a silent early C2 precondition or a later C2 construction failure;
5. `stagePathTubePair()` (including its existing pair/lease authority,
   new-owner, prepared-epoch, or local Runtime dry-run checks);
6. candidate-pair/session validation or a slot race before pending publication.

`preparePathTubePairCommit`, final pair CAS, and frontend consumption cannot
be the first false for this occurrence unless staging first succeeds and
publishes its pending handoff.  The current transcript has no proof that this
happened.

No repair is authorized by this audit.  In particular, do not reduce the
0.4 forward requirement, reuse a tube for a different owner, alter C2/A5/
Filter geometry, add a retry/gate/state/reason, or change a parameter.

## Evidence and source examined

- private episode evidence:
  `/tmp/h2_successful_epoch_refresh_first_false_20260817/evidence`;
- launch transcript:
  `roslaunch.stdout.log`, lines 350--358;
- prior selector evidence:
  `docs/Codex_A6_H2_Nonterminal_Seam_Counter_Temporary_Measurement_Read_Only_Audit_2026-08-17.md`;
- current call chain:
  `gvf_manager.cpp`: `stageFutureSeamPathTubeTransaction`,
  `certifiedFutureSeamCandidates`, `buildPhaseV2C2Frontend`, and
  `prepareAndCommitPendingPathTubeHandoff`;
- adapter call chain:
  `captureAndAcquirePathTubePairPin`, `stagePathTubePair`,
  `preparePathTubePairCommit`, and
  `finalizePreparedPathTubePairCommit`.

The only valid next diagnostic action is the one-run, temporary measurement
specified in
`Codex_A6_H2_Point_Phase_Accepted_Replan_First_False_Temporary_Measurement_Execution_Spec_2026-08-17.md`.

## Read-only integrity

This audit did not modify source, launch/configuration, parameters, runtime
processes, bags, or product binaries.  `git diff --check` passes at audit
time; the workspace remains user-owned and dirty outside these new documents.
