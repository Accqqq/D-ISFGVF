# PhaseOffsetSwarm B1E G3R E0 Invalid-Fixture Correction Execution Specification

Date: 2026-08-21

Owner: Codex root supervisor

Executor: Luna Max only

Repository:

```text
/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
```

Expected identity:

```text
branch = main
HEAD = 9a0e97560b8ddf15ac1ed1581030ed4b911f6e43
```

Execution policy:

```text
AUTO_ADVANCE = false
LUNA_SUBAGENTS = forbidden
PRODUCTION_SOURCE_EDITS = forbidden
TUBE_EDITS = forbidden
NEW_PRODUCTION_GATES = forbidden
E1_THROUGH_E8 = forbidden
```

This specification authorizes correction of the B1E diagnostic fixture only.
It does not authorize a navigation, planner, simulator, SO3, Tube, Swarm
Intent, allocator, CBF, threshold, gain, or control-chain change.

## 1. Invalid E0 evidence

The first G3 E0 attempt is retained read-only at:

```text
/tmp/codex_b1e_e0_20260821T084929Z_1051605
```

It completed a full receiver-monotonic observation:

```text
observation_duration_s = 60.04740496404702
odom = 6005 samples at about 100 Hz
local_map = 600 samples at about 10 Hz
PositionCommand = 319 samples at about 50 Hz
SO3Command = 6005 samples at about 100 Hz
```

The run is invalid, not a production regression, for two fixture reasons:

1. `rosgraph.Master.getSystemState()` already returns the three system-state
   arrays.  The diagnostic incorrectly unpacked it as `(code, message,
   value)`, so a valid system-state list was mistaken for an error code and
   graph audit raised an exception.
2. The test used the earliest retained odometry sample after its process
   started as the pre-goal state.  With the intentionally preserved E0
   `start_at_hover=false` startup, the plant had already entered its startup
   transient before the rostest process subscribed.  The reported
   `initial_position_error_m=0.8341462588291859` therefore did not represent
   the latest state immediately before the single goal publication.

The run also showed that the protected planner interpreted the published
`(5,-3.5,1)` click as a terminal command near `(5,-3.5,2)`.  This is diagnostic
evidence, not authorization to alter the protected `+1.0` goal-height
semantics or the frozen B1 acceptance goal.

The run's process exit was additionally reported by rostest as "did not
generate test results" because the executable emitted JSON but did not create
the JUnit XML path supplied by rostest.  Result-file reporting must be fixed
without changing diagnostic pass/fail semantics.

## 2. Mandatory executor constraints

Luna Max must execute personally and must not:

- start or delegate to any sub-agent;
- edit this specification;
- edit any file except the one path in Section 4;
- build any package;
- start roscore, roslaunch, rostest, or any ROS runtime during G3R-G1;
- connect to or query a user or Tube V2 ROS master;
- modify any launch, `.test`, CMake, package manifest, production source,
  configuration, map, manual-map file, Tube file, or self-audit;
- add a production readiness gate, watchdog, command timeout, fallback,
  retry, terminal controller, feature flag, runtime mode, or behavior change;
- change the existing 25 s readiness bound, 15 s subscriber bound, 60 s
  observation, 45 s arrival bound, 3 s stable window, 5 s drift window, or
  any XY/Z/speed/drift threshold;
- run E0 again until a separate root release;
- run E1 through E8 under this specification.

## 3. Required reading

Read completely before editing:

1. `AGENTS.md`;
2. the main B1E execution specification;
3. this G3R correction specification;
4. the original E0 `B1E_METRICS` and rostest logs in Section 1;
5. `/opt/ros/noetic/lib/python3/dist-packages/rosgraph/masterapi.py`, limited
   to the `Master.getSystemState` contract;
6. the current B1E diagnostic Python file.

## 4. Complete write whitelist

Luna Max may modify only:

```text
src/swarm_planner/bspline_traj/test/b1e_single_equivalence_diagnostic_test.py
```

Everything else is read-only.  In particular all 8 B1E launch files, all 9
B1E `.test` files, the main B1E specification, all B1 files, the protected
manual-map file, and all Tube V2 work are no-touch.

## 5. Required correction A: ROS master system-state contract

Replace the incorrect wrapper handling with the real `rosgraph.Master`
contract:

```python
publishers, subscribers, services = master.getSystemState()
```

Do not call the raw XML-RPC handle and do not inspect another master.  Preserve
the existing endpoint inversion, negative topology audit, cross-agent audit,
and manager union-endpoint audit.

Add a pure helper which validates that the returned object contains exactly
three list-like sections and returns them.  Add a synthetic test which passes
a representative direct three-section value and proves it is decoded without
looking for an XML-RPC status code.

## 6. Required correction B: pre-goal start-state semantics

The frozen requirement remains:

```text
latest physical position immediately before goal publication
distance to START < 0.02 m
```

This is the already authorized B1E initial-condition requirement.  It is not a
new threshold or production gate.

Use one total `READINESS_SECONDS=25.0` receiver-monotonic readiness interval.
Within that existing interval require all of the following from a single
locked snapshot:

1. at least one odometry sample;
2. at least one local-map sample;
3. finite odometry and local-map streams;
4. the latest odometry sample, `snapshot["odom"][-1]`, has 3-D distance to
   `START` strictly below `0.02 m`.

Do not use `snapshot["odom"][0]`.  Do not add a velocity threshold, dwell
window, consecutive-cycle count, retry, fallback, hover command, goal
republish, or control publication.

After readiness succeeds, take a new locked snapshot immediately before goal
publisher creation/publication and record:

```text
initial_physical_state
initial_position_error_m
initial_state_relative_to_goal_publish_s
```

The recorded state must be the latest pre-goal odometry sample.  Revalidate
the same `<0.02 m` condition from that sample.  If the startup transient does
not return to the frozen start within the existing 25 s interval, keep the run
invalid and stop after evidence; do not change the plant or threshold.

Add a pure synthetic case containing an early far sample and a later near
sample.  It must prove that the helper selects the later sample and does not
silently select the first record.

## 7. Required correction C: rostest JUnit result artifact

Keep exactly one stdout line beginning `B1E_METRICS `.  Do not put another
`B1E_METRICS` copy in JUnit `system-out`.

Recognize only the standard process argument:

```text
--gtest_output=xml:<absolute path>
```

Create a minimal, valid UTF-8 JUnit XML file at that exact path before process
exit.  It must contain one testsuite and one testcase:

- `failures=0` and no failure element when `run_valid=true`;
- `failures=1` with a bounded, secret-free failure message derived only from
  `infrastructure_failures` when `run_valid=false`;
- no complete environment dump, ROS parameter dump, logs, metrics JSON,
  tokens, paths outside the requested result path, or user data;
- XML creation failure must make the process fail and must be included in the
  one final metrics object's infrastructure failures when possible.

The process exit code remains:

```text
0 iff run_valid=true
1 otherwise
```

`strict_b1_pass=false` with `run_valid=true` must still produce a passing JUnit
testcase and exit zero.  Strict B1 outcome is diagnostic data, not the
rostest-process validity condition.

Implement XML serialization using Python standard library only.  Do not add a
test framework dependency, shell/process invocation, network operation, or
workspace output.  Add pure/synthetic checks for argument parsing and for the
pass/fail XML structure without writing into the workspace.

## 8. Static acceptance G3R-G1

After editing, Luna Max may run only static checks:

1. temporary-prefix `/usr/bin/python3 -m py_compile`;
2. import and all synthetic checks;
3. `xmllint` and `roslaunch --nodes` on the unchanged 17 B1E XML files;
4. searches proving the diagnostic publishes only one goal and no control;
5. searches proving no production gate/watchdog/fallback/mode was added;
6. Python mode remains `775`;
7. no B1E workspace pycache;
8. `git diff --check` and trailing-whitespace scan;
9. branch, HEAD, main B1E spec, this spec, 9 B1 hashes, and 15 protected
   dependency hashes;
10. exact whitelist proof showing only the diagnostic Python file changed in
    this correction.

Then report and stop.  Do not build or rerun E0 without root release.

## 9. Root-released E0 rerun G3R-G2

Only after static root acceptance, rerun E0 once on a new private master and a
new unique `/tmp/codex_b1e_e0r_*` directory.  Reuse all secret-safe,
port-isolation, process-group, owned-cleanup, and protected-hash requirements
from the main B1E specification.  The invalid port 49033 run is evidence only
and must not be overwritten or reused.

The rerun must prove:

- one original `B1E_METRICS` emit;
- one generated per-test JUnit XML;
- `rostest` exit zero exactly when `run_valid=true`;
- valid graph audit;
- initial latest pre-goal state within 0.02 m;
- full 60 s observation;
- post-goal four-stream progress and 80--130 Hz odometry;
- E0 parameters/manual flags/effective map defaults;
- protected manual-map hash unchanged;
- cleanup of only the new E0R PGIDs and port.

If `run_valid=false` again for a new reason, stop.  Do not run E1.

## 10. Final stop rule

This correction ends after a valid E0 rerun report.  E1 requires a separate
explicit root release.  No evidence in this stage authorizes a production
change, a goal-height change, a B1 acceptance revision, or a Tube/control
integration change.
