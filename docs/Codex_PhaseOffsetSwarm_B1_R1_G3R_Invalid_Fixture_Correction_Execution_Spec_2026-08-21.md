# PhaseOffsetSwarm B1-R1 G3R Invalid-Fixture Correction Execution Specification

Date: 2026-08-21

Owner: Codex root supervisor

Executor: the existing Luna Max agent only

Repository:

    /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws

Expected identity:

    branch = main
    HEAD = 9a0e97560b8ddf15ac1ed1581030ed4b911f6e43

AUTO_ADVANCE=false. This correction authorizes only a B1 test-fixture graph
decoder repair and one corrected single-agent rerun. It does not authorize a
three-agent run until a later explicit root release.

Luna Max must execute personally. It must not spawn, delegate to, consult,
request, message, or wait for a sub-agent, nested agent, other model, or
parallel worker.

## 1. Authority and scope

This specification is a narrow correction addendum to:

    docs/Codex_PhaseOffsetSwarm_B1_R1_Legacy_Goal_Height_Semantic_Alignment_And_Independent_Three_UAV_Execution_Spec_2026-08-21.md

Root-released parent specification SHA256:

    85e5750a72fb64758b891eed2355e4d6090454e28d402b51dc6605fe980356c7

Current authorized B1 test SHA256 after the accepted goal-height semantic
patch:

    0cdf539462995c1497cddff38c8b4a40b9f8910d89e6a75167444989ef10b679

This correction changes no goal, height semantic, threshold, gain, map, lane,
planner, plant, controller, Tube, Swarm Intent, allocator, CBF, production
source, or runtime mode.

## 2. Retained invalid-run evidence

The first B1-R1 G3 attempt is retained read-only at:

    /tmp/codex_b1_r1_g3_single_20260821T123917Z_AClluW

It is infrastructure-invalid and must not be counted as a B1 acceptance run.

Owned runtime identity:

    private master port = 20000
    master PID/PGID = 1120371/1120371
    rostest PID/PGID = 1120427/1120427

Both process groups were cleaned and port 20000 was released. The directory
must not be modified, overwritten, reused, or deleted.

The navigation evidence before graph audit passed:

    published goal = [5.0, -3.5, 1.0]
    effective goal = [5.0, -3.5, 2.0]
    goal_publish_count = 1
    arrival_s = 14.24424964201171
    final XY error = 6.366856901850678e-05 m
    final Z error relative to 2.0 m = 2.4801838282462185e-05 m
    final speed = 0.0004100215993532839 m/s
    drift = 9.604824450485396e-05 m
    odom rate = 100.00386193193805 Hz
    minimum path clearance = 0.5164661157073144 m
    physical clearance = 0.5086437777306633 m
    finite_data = true

The only runtime exception was the fixture's incorrect interpretation of
rosgraph.Master.getSystemState().

## 3. Exact invalidity causes

### 3.1 ROS master graph decoder

The current test incorrectly does:

    code, message, value = master.getSystemState()
    if code != 1:
        raise RuntimeError(...)

The rosgraph.Master wrapper already unwraps the XML-RPC status envelope. Its
public getSystemState() method returns the direct three-section state:

    publishers, subscribers, services

The valid publishers list was therefore misread as a status code, causing
graph_audit to throw after navigation acceptance had already succeeded.

This is a test-fixture decoding defect. It is not a ROS graph, planner,
controller, Tube, plant, SO3, goal-height, or navigation defect.

### 3.2 JUnit runner mode

The invalid run used:

    rostest --reuse-master --text bspline_race b1_independent_single.test

ROS Noetic warns that the overall result is not accurate in text mode, and
the requested standard JUnit artifact was not produced. For B1-R1 corrected
and later runs, this addendum overrides only the runner spelling:

    rostest --reuse-master bspline_race <test-file>

Do not use --text. Standard rostest must own the JUnit result. Do not add
custom JUnit serialization to the B1 Python test.

## 4. Complete write whitelist

Luna Max may modify only:

    src/swarm_planner/bspline_traj/test/b1_independent_navigation_test.py

Everything else in the workspace is read-only during G3R-G1 and G3R-G2.

Do not create the final B1-R1 self-audit during this correction. Do not edit
the parent R1 specification, this correction specification, any launch or
.test file, CMake, package.xml, map, config, simulator, planner, SO3, Tube,
Swarm Intent, allocator, CBF, or production source.

Runtime evidence may be created only under a new task-owned:

    /tmp/codex_b1_r1_g3r_single_<UTC timestamp>_<unique suffix>

## 5. Required correction A: validated direct system-state decoder

Add one pure helper adjacent to invert_endpoint_state:

    def decode_system_state(state):
        ...

Contract:

1. Accept only a list or tuple of length exactly three.
2. Interpret the three sections as publishers, subscribers, and services.
3. Require each section to be a list or tuple.
4. Require every entry to be a list or tuple of length exactly two.
5. Require the entry's node collection to be a list or tuple.
6. Return the three validated sections without changing endpoint or node
   values.
7. Raise RuntimeError with a bounded, non-secret diagnostic for malformed
   structure.
8. Do not look for code/message/value.
9. Do not call a raw XML-RPC handle.
10. Do not contact any additional master.
11. Do not add retry, fallback, alternate decoder, compatibility mode, gate,
    environment switch, or version detection.

Replace get_system_state with exactly one call to the current private master:

    master = rosgraph.Master(rospy.get_name())
    publishers, subscribers, services = decode_system_state(
        master.getSystemState())

Then preserve the existing invert_endpoint_state calls and all graph
acceptance logic unchanged.

The validation helper is fixture input validation only. It is not a runtime
navigation or controller gate.

## 6. Frozen semantic patch and acceptance logic

The accepted G1 goal-height patch must remain byte-for-byte equivalent in
behavior:

    LEGACY_GOAL_Z_OFFSET_M = 1.0
    published z = 1.0 m
    effective accepted z = 2.0 m

Do not change:

- scenario starts or published_goals;
- effective_goal;
- goal publication count or construction;
- stable_arrival;
- XY, Z, speed, stable-window, arrival-horizon, or drift thresholds;
- graph expected nodes/topics;
- parameter audit;
- PCD parsing or clearance;
- rate or timestamp checks;
- pair-distance logic;
- exception/process behavior;
- structured goal metrics.

## 7. No new gating

Do not add any new:

- ROS parameter;
- launch argument;
- enable flag;
- health/readiness state;
- watchdog;
- timeout beyond existing test bounds;
- retry or resend;
- fallback;
- controller mode;
- legacy-height mode;
- graph compatibility mode;
- hidden test bypass;
- environment-variable switch;
- production subscriber, publisher, service, or timer.

The decoder validates the documented return structure once. It does not
select between modes or silently accept alternate structures.

## 8. G3R-G1 static and synthetic validation

Before editing:

1. Reverify branch, HEAD, parent spec SHA, correction spec SHA, and current
   test SHA.
2. Recheck no build/test writer.
3. Confirm no B1-R1 owned runtime process remains.
4. Confirm the invalid run directory remains read-only.

After the one-file patch:

1. Show the exact minimal diff.
2. Read the rosgraph Master.getSystemState contract in:

       /opt/ros/noetic/lib/python3/dist-packages/rosgraph/masterapi.py

3. Run Python syntax validation using a temporary pycache outside workspace.
4. Import the module without starting a ROS node.
5. Synthetic valid direct-state case:

       [
           [["/topic_a", ["/node_a"]]],
           [["/topic_b", ["/node_b", "/node_c"]]],
           [["/service_a", ["/node_a"]]],
       ]

   must decode as the same three sections.
6. Synthetic valid empty case [[], [], []] must pass.
7. Reject:
   - a non-list/tuple outer object;
   - outer length other than three;
   - non-list section;
   - entry length other than two;
   - non-list node collection.
8. Re-run the full G1 goal semantic and stable_arrival synthetic cases.
9. Verify exactly one master.getSystemState call.
10. Verify no raw master handle or XML-RPC call.
11. Verify no code/message/value status handling remains in get_system_state.
12. Verify graph_audit logic after get_system_state is unchanged.
13. Verify exactly one goal Publisher construction and one goal publish call.
14. Verify no command/controller publisher.
15. Verify no new gate/parameter/mode/fallback/retry/watchdog.
16. Recompute all protected hashes; only the authorized test hash may change.
17. Run git diff --check, targeted trailing-whitespace checks, and cache scan.

Report G3R-G1 to root with no-sub-agent confirmation, exact diff, new test
hash, validation results, files written, and stop. Do not rerun yet.

## 9. G3R-G2 corrected single-agent rerun

Only after explicit root release:

1. Recheck identity, hashes, build writers, current listeners, and port
   ownership.
2. Select a new loopback port that is not 12914, not 20000, and not any
   pre-existing listener.
3. Create a new owned ROS_HOME and ROS_TEST_RESULTS_DIR.
4. Start the private master in an explicit owned process group.
5. Run without --text:

       rostest --reuse-master bspline_race b1_independent_single.test

6. Preserve runner stdout/stderr, ROS logs, one B1_METRICS source, and the
   standard JUnit result.
7. Require process exit zero and JUnit:
   - one test case;
   - zero errors;
   - zero failures.
8. Require graph_audit to complete and all graph assertions to pass.
9. Require every single-agent validity and acceptance item in parent R1
   Section 14.
10. Verify published/effective goal metrics and unchanged thresholds.
11. Clean only the owned master and rostest process groups.
12. Prove owned PIDs are gone and owned port is no longer listening.

Do not manually copy B1_METRICS into JUnit. Standard rostest output capture is
allowed. Count exactly one original metrics emission from the test process.

If infrastructure-invalid, preserve and report without another rerun. If
valid but failing, stop and report. If valid and passing, report full metrics,
JUnit path, run directory, port/PID/PGID, cleanup, hashes, and wait for a new
root release before any three-agent run.

## 10. Concurrent Tube and user-process protection

All parent R1 Section 10 protections remain mandatory:

- never use port 12914;
- never contact an existing ROS master;
- never signal a non-owned PID or PGID;
- never use pkill, killall, or process-name cleanup;
- never touch Tube V2 files, evidence, processes, or build outputs;
- never compete with catkin_make, make, ninja, cmake --build, or run_tests.py;
- never modify another /tmp/codex_* directory;
- clean only exact owned process groups and port.

## 11. Stop conditions

Stop and report if:

- branch, HEAD, parent spec, or correction spec changes;
- a non-whitelist edit is required;
- protected hashes change unexpectedly;
- a build is required;
- direct documented system-state decoding does not solve graph_audit;
- standard non-text rostest cannot create a valid JUnit result;
- a single-agent acceptance item fails after graph correction;
- passing appears to require a production change, threshold change, new gate,
  retry, fallback, Tube, Swarm Intent, allocator, CBF, or controller action;
- safe private-master ownership cannot be guaranteed.

## 12. Required conclusion and STOP

If the corrected rerun passes, conclude only:

    The first G3 run was infrastructure-invalid due to test-fixture graph
    decoding and text-mode JUnit suppression. The corrected test preserves
    published z=1, effective z=2, all thresholds, and all production behavior.
    The one-agent B1-R1 acceptance is valid and passing.

Then STOP before three-agent G4. Root must explicitly release G4 under the
parent R1 specification.
