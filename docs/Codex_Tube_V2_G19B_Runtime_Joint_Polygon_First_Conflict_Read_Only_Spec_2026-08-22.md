# Tube V2 G19B Runtime joint-polygon first-conflict read-only spec

    DOCUMENT_ROLE=READ_ONLY_PARALLEL_ROOT_CAUSE_SPEC
    DOCUMENT_STATUS=ACTIVE
    DATE=2026-08-22
    PLAN_OWNER=PRIMARY_CODEX_AGENT
    EXECUTOR=LUNA_MAX_SINGLE_AGENT
    SUBAGENT_CREATION_ALLOWED=false
    REPOSITORY_EDIT_ALLOWED=false
    NEW_ROS_RUN_ALLOWED=false
    NEW_TEST_HARNESS_ALLOWED=false

## 1. Purpose and priority

G19B is parallel evidence analysis only.  It must not delay, modify or compete
with G19.  G19 owns the earlier exact-current `[0,0]` H2 failure.  G19B owns
the independent later Runtime failure already observed both live and during
H2 staging:

    short-horizon exact-port witness unavailable
    first_failure_step=4
    joint port feasible polygon is empty

The product remains `PARTIALLY_FIXED`.  G19B must identify which existing
PortProjector constraints first make the step-4 polygon empty, or state the
exact missing runtime field which prevents that determination.

## 2. Frozen evidence

Use only:

    /dev/shm/tube_v2_g18_dynamic_20260822_194602/

Primary files include:

    test_gvf.launch.log
    test_gvf.bag
    derived/live_runtime_denials.tsv
    derived/h2_staging_dry_run.tsv
    derived/pair_timeline.tsv
    derived/final_report.md

Relevant known staging tuple:

    current_w=10.688202612
    seam_w=11.108793243
    retained_delta=-0.090206323
    previous_final_port.u_w=0.043454594
    previous_final_port.u_delta=-0.180320547
    runtime_mode=5
    first_failure_step=4

Also analyze the four live denials at generations 6, 7, 10 and 14.  They
prove the failure is not exclusive to the new H2 owner.

## 3. Hard boundaries

- Do not edit any repository file.
- Do not run ROS launch, simulator, planner or new dynamic test.
- Do not create a test executable, script in the repository, fixture, harness
  or package.
- Temporary read-only extraction commands and files under a new `/dev/shm`
  evidence directory are allowed.
- Do not change parameters, limits, margins, authority, horizon, Runtime,
  Builder, Projector, launch or map.
- Do not propose recenter, effective-base cancellation, forced delta zero,
  widened Tube, changed rate, retry, gate, cache, state machine or controller.
- Do not open subagents.

## 4. Required reconstruction

First enumerate the bag topics and message definitions already recorded.
Extract, when present, for each of the four live denials and the H2 staging
denial:

- current phase, retained delta, delta ref;
- raw and previous/final `u_w`, `u_delta`;
- base `w_dot`, base tangent speed, `||r_w||`, curvature;
- current PWL lower/upper bounds and slopes;
- certified profile domain and current cell;
- `dt`, `u_w/u_delta` absolute and slew limits;
- phase-dot and tangent-speed lower limits;
- future step count and tube refresh period;
- Runtime mode, selected bit and invalid reason.

Use the unchanged production formulas in `phase_offset_runtime.cpp` and
`port_projector.cpp` to reconstruct the feasible-set sequence from root through
step 4.  Do not assume the generic invalid reason identifies the conflicting
planes.

For each step compute or bound:

    w_lower = max(-u_w_abs_max,
                  previous_u_w - u_w_rate_max*dt,
                  phase_dot_min - base_w_dot,
                  (tangent_speed_min-base_tangent_speed)/||r_w||)
    w_upper = min(+u_w_abs_max,
                  previous_u_w + u_w_rate_max*dt)

    delta_lower = max(-u_delta_abs_max,
                      previous_u_delta-u_delta_rate_max*dt,
                      regularity lower when applicable)
    delta_upper = min(+u_delta_abs_max,
                      previous_u_delta+u_delta_rate_max*dt,
                      regularity upper when applicable)

Then account for the exact current/terminal PWL cell and any crossed-knot
half-planes.  Identify the first empty intersection and name the smallest
conflict set from these labels:

    local_w_lower / local_w_upper
    local_delta_lower / local_delta_upper
    pwl_terminal_w_lower / pwl_terminal_w_upper
    pwl_terminal_lower / pwl_terminal_upper
    pwl_crossed_lower / pwl_crossed_upper

If exact dynamic fields are absent, compute only rigorous ranges and do not
invent values.

## 5. Questions that must be answered

1. Is step 4 empty because retained delta plus the previous negative
   `u_delta` cannot brake before the narrow certified lower boundary?
2. Is it instead caused by nonnegative phase/tangent progress and `u_w` slew?
3. Does a PWL boundary slope couple `u_w` and `u_delta` into the conflict?
4. Does the first conflict occur before the C2 seam in the staging rollout?
5. Why can a later live cycle become valid again after the earlier denial,
   given that failed Runtime steps do not mutate `previous_final_port_`?
6. Is the H2 staging denial the same numerical conflict class as generation
   14's preceding live denial, or merely the same generic error string?
7. Which single additional transaction-local diagnostic field/set would be
   minimally sufficient if the existing bag cannot prove the conflict?

## 6. Output and classification

Write evidence only under:

    /dev/shm/tube_v2_g19b_readonly_<timestamp>/

Return one of:

- `DELTA_SLEW_VS_LOWER_PWL_BOUND_PROVEN`
- `W_PROGRESS_OR_TANGENT_SLEW_CONFLICT_PROVEN`
- `COUPLED_PWL_SLOPE_CONFLICT_PROVEN`
- `MULTIPLE_DISTINCT_POLYGON_CONFLICTS_PROVEN`
- `EXISTING_EVIDENCE_INSUFFICIENT`

The report must cite exact log/bag times, formulas and reconstructed numeric
intervals.  If insufficient, state the smallest missing field list, capped at
one concise transaction-local log line.  Do not propose or implement a fix.
