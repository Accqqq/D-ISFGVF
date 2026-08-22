# A6/H2 nonterminal seam-counter temporary measurement — execution specification

```text
DOCUMENT_ROLE=DEDICATED_CURRENT_STAGE_EXECUTION_SPECIFICATION
DATE=2026-08-17
STAGE=A6_H2_NONTERMINAL_CERTIFIED_FUTURE_SEAM_CAUSE_MEASUREMENT
STATUS=TEMPORARY_INSTRUMENTATION / AUTHORIZED_BY_USER_2026-08-17
IMPLEMENTATION_AUTHORIZED=YES__ONE_REVERSIBLE_PRIVATE_EVIDENCE_EPISODE_ONLY
AUTO_ADVANCE=false
```

## Objective

Classify the *first elimination layer* in
`gvf_manager::certifiedFutureSeamCandidates` for the already observed
nonterminal empty-selector calls near `captured_w` = 3.3757, 4.4238, and
5.4196. The prior temporary trace reported only
`no_certified_future_seam`; it did not expose whether structural validation,
the certified-end precheck, phase threshold, certified interval, owner
equivalence, or `TubeFilter::query` first emptied the seam set.

The terminal case is already proven separately: at `w=5.68946` the old path
ends at `5.707`, so no old sample can satisfy the required `w >= 6.08946`.
This stage does not revisit or relax that result.

## Whitelist

Temporary source edit:

- `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`

Permanent documentation only:

- this execution specification
- one post-run read-only audit document

No header is authorized or needed: the counter and emission helper can be
local to `gvf_manager.cpp`'s anonymous namespace and have no public ABI.

## Baseline and preconditions

Before the forward patch, record:

```text
git status --short
git diff --check
sha256sum gvf_manager.cpp
readelf Build-ID: libbspline_gvf.so,
                  libphase_offset_matched_adapter.so,
                  formation_planning
rg TEMP_H2_SEAM_COUNTER src/swarm_planner/bspline_traj   # must be empty
```

Required baseline SHA-256 for `gvf_manager.cpp`:

```text
078ae5378b49b7942b12ecef927f928f70fda0b0f5642d386691e650fdb07c18
```

If the SHA differs, a user-owned change overlaps the sole whitelist file, an
existing master occupies the chosen private port, the instrumented build fails,
or an owned process cannot be proven private, stop the run, reverse any patch,
and report. Do not expand the whitelist or repeat the episode.

## Exact temporary instrumentation

Add a fixed `TEMP_H2_SEAM_COUNTER` stderr line only in the low-frequency
`certifiedFutureSeamCandidates` selector path. It must execute after the
existing calculations and preserve every original condition, evaluation order,
return value, mutex boundary, allocation, and ownership operation.

The line must contain:

```text
captured_w
min_certified_forward_w
minimum_future_seam_w
old_owner_start_w, old_owner_end_w
active_profile_sample_first_w, active_profile_sample_last_w
active_certified_segment_start_w, active_certified_segment_end_w
old_full_sample_count, old_full_sample_first_w, old_full_sample_last_w
structural_precheck_pass
certified_end_precheck_pass
ordered_sample_validation_pass
phase_threshold_survivors
certified_interval_survivors
owner_equivalence_survivors
tube_query_survivors
first_surviving_w, last_surviving_w
result_count
```

For early structural failure, fields without a valid source object must use a
fixed non-semantic sentinel (for example `nan` / `0` as appropriate) in the
temporary line only. The implementation must not introduce a persistent
diagnostic topic/schema, ROS parameter, enum/reason, state, gate, retry, or
latch. It must not print from the 50 Hz command governor or from the normal
empty pending/completed mailbox paths.

The counter semantics are cumulative over the exact existing filters:

1. `structural_precheck_pass` is the conjunction already used before reading
   the profile interval.
2. `certified_end_precheck_pass` is the existing
   `certified_end >= minimum_future_seam_w - eps` condition.
3. `ordered_sample_validation_pass` is zero only on the original immediate
   return for an invalid/non-increasing old sample.
4. Each survivor count is incremented only after the corresponding original
   filter has passed, in the original loop order; no filter is moved or
   duplicated for control decisions.
5. `result_count`, `first_surviving_w`, and `last_surviving_w` reflect exactly
   the existing returned vector.

This is a counterfactual observation of an existing pure selection function,
not a replacement selector. It may not add a fallback seam or modify H2, C2,
A5 geometry, Filter, map evidence, Runtime, governor, or physical command.

## One private episode

After one successful instrumented build, run **exactly one** active private
episode:

- use a fresh loopback ROS master and private `ROS_HOME`/`ROS_LOG_DIR`;
- never attach to, list-kill, or alter user-owned ROS processes;
- only override `phase_offset_manual_observe_only:=false`;
- retain the current approved map, launch values, and goal `(8,0,1)`;
- bag existing diagnostics, `/particle0/path`, `/position_cmd`, `/sim/odom`,
  `/move_base_simple/goal`, and `/rosout`;
- save instrumented Build-IDs, `/proc/<owned-planner>/maps`, PID ownership,
  port checks, parameter dump/hash, stdout, bag checksum/info, and transcript;
- hard upper bound: 70 seconds after the one goal publication, no second goal
  and no rerun even if the three nonterminal captures do not recur.

The run is evidence only. Stop sooner only for an unsafe task-owned launch
failure, missing planner, bag-recorder failure, or private-master collision;
in all cases proceed directly to rollback.

## Mandatory unconditional rollback

Immediately after the single run—or after any pre-run/run failure—apply the
precise reverse patch. Then, before any interpretation:

```text
rg TEMP_H2_SEAM_COUNTER src/swarm_planner/bspline_traj   # no output
sha256sum gvf_manager.cpp                                # exact baseline above
catkin_make --pkg bspline_race -j2
readelf restored Build-IDs
```

Run the focused restored-product checks:

```text
phase_offset_matched_adapter_test
phase_offset_tube_epoch_diagnostics_test
phase_offset_tube_epoch_integration_test
gvf_switch_policy_test
continuous_phase_path_test
git diff --check
git status --short
```

An already stale incompatible test artifact is to be reported, not rebuilt or
fixed outside this specification. The final audit must show the forward and
reverse evidence, exact source SHA restoration, absence of the marker from
product source, build/test results, process/port cleanup, and a strict
evidence boundary.

## Decision boundary and stop

After rollback, classify only the first observed selector elimination layer.
It is acceptable for the result to be `NOT OBSERVED` if the one episode does
not reproduce a nonterminal call. Do not implement a repair, tune a parameter,
add a gate/retry, or change any H2/A5/C2/Filter rule in this stage. Any repair
requires a new dedicated execution specification and fresh user authorization.
