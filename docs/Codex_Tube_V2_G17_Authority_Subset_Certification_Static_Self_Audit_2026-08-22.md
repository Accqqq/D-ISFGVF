# Tube V2 G17 authority-subset certification static self-audit

    DOCUMENT_ROLE=PRODUCT_CORRECTION_STATIC_SELF_AUDIT
    DOCUMENT_STATUS=STATIC_IMPLEMENTATION_COMPLETE_DYNAMIC_NOT_RUN
    DATE=2026-08-22
    EXECUTOR=LUNA_MAX_SINGLE_AGENT
    PRIMARY_CLASSIFICATION=G17_AUTHORITY_SUBSET_IMPLEMENTED
    COMPLETION_CLASSIFICATION=PARTIALLY_FIXED_PENDING_UNCHANGED_G1_G5_DYNAMIC_ACCEPTANCE
    STATIC_BASELINE_EXCEPTION=PREEXISTING_BASELINE_CONTRACT_FAILURE
    ROS_DYNAMIC_TEST_RUN=false

## 1. Authorized result

G17 now keeps the environmental Tube and the controller authority request as
separate values:

```text
unchanged Builder raw/environment I_geo
-> unchanged TubeFilter output
-> post-filter certification copy intersected with I_authority
-> existing SurfaceValidator
-> existing Runtime dry-run and atomic Pair
```

No new ROS parameter, message, state owner, gate, cache, retry, timer,
watchdog, module, recovery controller, or launch setting was added.

## 2. Mandatory preflight

Evidence root:

```text
/tmp/tube_v2_g17_20260822_160715/
/tmp/tube_v2_g17_r1_r2_20260822_171814/
```

The required preflight passed before product edits:

```text
Runtime           35/35
Adapter           83/83
GVF switch        49/49
CertifiedBuilder  12/12
Integration       10/10
git diff --check  clean
```

The only production recenter spelling was the pre-existing Runtime
`force_recenter` lifecycle.  No G16 E/M6 recenter product code remained.

## 3. Product changes

Product files changed:

- `certified_tube_builder.h/.cpp`;
- `tube_epoch_types.h`;
- `tube_epoch_manager.cpp`;
- `phase_offset_matched_adapter.h/.cpp`.

Existing `TubeBounds` is reused as immutable transaction input through:

```text
adapter command-lock capture
-> TubeBuildRequest
-> TubeEpochUpdateInput
-> CertifiedTubeBuildInput
-> CertifiedTubeBuilder
```

For ESDF, the adapter computes exactly:

```text
A = abs(configured manual amplitude)
S_lower = min(-A, retained_delta, 0)
S_upper = max(+A, retained_delta, 0)
request.lower = S_lower - existing interior_margin
request.upper = S_upper + existing interior_margin
```

Nonfinite values, negative interior margin, invalid ordering, nonzero
derivative fields, or a request not containing zero fail closed.  The formula
does not read accepted/preflight amplitude, `fixed_delta_max`, `max_offset`, or
`search_extent`.

The Builder and Filter still produce full raw/environment evidence.  The
CertifiedBuilder makes one local post-filter copy and performs knotwise
intersection with the authority request.  The generalized existing narrowing
helper recomputes only filtered endpoints, exact local PWL slopes, and
`min_width`; raw/environment fields, `raw_build_samples`, current anchor,
snapshot provenance, and truncation evidence remain unchanged.  The first
Validator attempt and every inward retry start from this authority candidate,
never from raw `I_geo`.

The final R1 review correction makes the ESDF zero-baseline helper contract
explicit: zero-capacity and failed-validation outcomes set only the filtered
certification interval and its PWL slopes to zero.  Per-sample `raw_*`,
`environment_*`, their Builder/Filter evidence flags, `raw_build_samples`,
snapshot provenance, and truncation evidence remain unchanged.  The existing
FIXED zero-capacity behavior remains unchanged.

The final R2 review correction captures `task_generation` and the effective
`authority_session` in the same `runtime_command_mutex_` critical section as
retained delta, authority request, Runtime snapshot, and staged source
revision.  The lock-free request, candidate Pair, transaction, and staged
epoch propagate those captured values without reloading live atomics.  The
existing final session/pin/CAS checks remain unchanged and reject a task reset
that occurs while the expensive owner/Tube build is in progress.

FIXED and NONE behavior is unchanged.  Candidate markers continue to read
`raw_build_samples`; certified markers read only the validated narrow profile.

## 4. Focused proof and cost

The G17 focused Builder suite passed 20/20.  Its deterministic open-space
comparison recorded:

```text
raw queries          864
narrow queries        99
raw geometry cells   181
narrow geometry cells 11
raw v-splits           85
narrow v-splits         0
```

The same test retains raw samples near `[-3,+3]` while the certified profile
is within `[-0.10,+0.10]`.  `fixed_delta_max=0.04` does not clamp either the
ESDF request or raw geometry.

G16 D0 remains intact:

```text
expected validator queries 1152
actual validator queries   1152
inward attempts before      101
inward attempts after         0
```

Tests also cover invalid requests, retained negative/positive offsets, one
interior-margin expansion, accepted-amplitude independence, obstacle evidence
outside versus inside authority, no widening on inward retry, raw/certified
marker separation, FIXED preservation, and Epoch propagation.
The final review tests additionally cover broad raw/environment preservation
for exact-zero authority, in-authority UNKNOWN, and Validator query-terminal
fallbacks; the production adapter's raw diagnostics and Candidate marker stay
broad while certified display stays closed.  A deterministic H2 concurrency
test pauses owner evaluation after the command-lock capture, resets the task,
then proves the staged Pair kept the old captured task/session and the existing
commit boundary rejected it.  Both final review tests passed 20/20 repeated
runs.

## 5. Static verification

All G17 and integration-relevant direct suites except the independently
classified pre-existing baseline test passed:

```text
total direct tests 457
passed             456
failed               1
```

Key exact counts:

```text
CertifiedBuilder  20/20
EpochManager      55/55
Adapter           87/87
Tube markers      14/14
Integration       10/10
GVF switch        49/49
Runtime           35/35
TubeBuilder         9/9
TubeFilter          9/9
TubeCrossSection    7/7
```

`catkin_make -j2` completed successfully.  Tracked `git diff --check` and
per-file checks for all ten G17-edited files are clean.  The final review
changed only `certified_tube_builder.cpp`, its focused test,
`phase_offset_matched_adapter.cpp`, and its focused test; the other six G17
files retained their preceding G17 hashes.  Final source/binary hashes and the
forbidden-change audit are under the evidence roots.

## 6. Pre-existing SurfaceValidator baseline exception

The sole failing direct test is:

```text
TubeSurfaceValidatorTest.ExactCurrentAnchorSampledCoverOmitsOnlyFixedHalfVoxel
```

Classification:

```text
PREEXISTING_BASELINE_CONTRACT_FAILURE
```

G17 mandatory preflight did not include the full 22-test SurfaceValidator
suite.  After a fresh test rebuild, the failure reproduced 5/5.  The pre-G17
and current hashes are bit-identical for:

```text
tube_surface_validator.h       577bb4bb...9cff9c4
tube_surface_validator.cpp     986ca3b3...e221a9
tube_surface_validator_test.cpp ab9b98ba...3a6ebd88
```

No G17 symbol is present in those files.  The fixture gives all world points
at `x=0` a fixed clearance of `0.425`.  The exact anchor requests `0.400001`
and passes, but the existing validator subsequently checks the nondegenerate
adjacent cell at the same endpoint with a larger cover request (observed
maximum `0.535001`), so it correctly reports insufficient clearance.  Changing
that outcome would require editing the frozen SurfaceValidator product or
altering the historical fixture contract; neither was authorized or done.

Evidence:

```text
/tmp/tube_v2_g17_20260822_160715/preexisting_surface_baseline/
```

This exception does not use the new authority request path and does not
invalidate G17's raw-versus-certified semantics or its cost reduction.  It
does prevent claiming an unqualified all-green workspace static baseline.

## 7. Frozen-file audit

Hashes remained at their pre-G17 values for TubeCrossSection, TubeBuilder,
TubeFilter, SurfaceValidator, Runtime, marker production, launch files, map,
and CMake.  No query/depth/attempt limit, safety margin, rate, horizon,
default, or launch value was changed.  No candidate-to-certified promotion or
raw marker contraction was introduced.

## 8. Remaining acceptance

No ROS dynamic test was started.  G17 cannot be labelled `FIXED` until the
primary agent reviews this implementation and the unchanged separate
`simulator.launch` plus user `test_gvf.launch` G1--G5 sequence proves:

- broad raw Candidate remains visible;
- narrow certified Tube produces real ADD messages;
- a nonzero Pair commits and executes;
- H2 replacement commits or fails directly and safely;
- neutral navigation does not wait for Tube;
- all five goals reach without stable path-end HOLD.
