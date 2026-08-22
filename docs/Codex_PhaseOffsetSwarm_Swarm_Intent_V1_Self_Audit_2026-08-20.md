# PhaseOffsetSwarm Swarm Intent V1 self-audit

Date: 2026-08-20 (Asia/Shanghai)

## Scope and timestamps

- Implementation start (first authorized Swarm Intent file created):
  `2026-08-20T23:10:53+08:00`.
- Final audit timestamp: `2026-08-20T23:40:00+08:00` (approximately; the
  final commands were run immediately before this report).
- Authorized scope was the isolated nominal Swarm Intent V1 layer and its
  ROS1 shadow wrapper.  No allocator, CBF, matched-port, Tube, governor, SO3,
  or control-chain integration was attempted.

## Baseline and worktree protection

Preflight recorded:

- branch: `main`;
- `HEAD`: `9a0e97560b8ddf15ac1ed1581030ed4b911f6e43`;
- `phase_offset_msgs/` and `phase_offset_swarm/` did not exist at the start;
- the worktree already contained 17 tracked modified files and many
  untracked user-owned Tube/single-UAV/prototype assets;
- initial tracked diff summary was 17 files, 7,512 insertions and 810
  deletions;
- existing `phase_offset_core/` and `phase_offset_navigation/` directories
  were user assets and were not modified;
- the initial build-process check was empty.

Final `git status --short` contains the pre-existing entries plus these stage
entries: `?? src/swarm_planner/phase_offset/phase_offset_msgs/`,
`?? src/swarm_planner/phase_offset/phase_offset_swarm/`, and this self-audit
document.  Final status remains on `main` at the same `HEAD`.  The only source
additions from this stage are the two authorized package directories and this
audit document.  Existing files outside the whitelist remain untouched.  No commit,
branch, stash, stage, reset, restore, clean, rebase, or push was performed.

## Files created by this stage

`phase_offset_msgs`:

- `src/swarm_planner/phase_offset/phase_offset_msgs/CMakeLists.txt`
- `src/swarm_planner/phase_offset/phase_offset_msgs/package.xml`
- `src/swarm_planner/phase_offset/phase_offset_msgs/msg/AgentState.msg`

`phase_offset_swarm`:

- `CMakeLists.txt`, `package.xml`, `README.md`;
- public headers: `agent_state.h`, `neighbor_state.h`,
  `swarm_parameters.h`, `swarm_output.h`, `neighbor_manager.h`,
  `elastic_interaction.h`, `conflict_friction.h`, `swarm_intent.h`,
  `pair_geometry.h`;
- core sources: `neighbor_manager.cpp`, `elastic_interaction.cpp`,
  `conflict_friction.cpp`, `swarm_intent.cpp`;
- ROS shadow source: `swarm_intent_shadow_node.cpp`;
- `config/swarm_intent_v1.yaml` and `launch/swarm_intent_shadow.launch`;
- tests: four pure C++ gtests, `swarm_intent_shadow.test`, and
  `swarm_intent_shadow_test.py`.

## Build and test evidence

The pure C++ hard gate passed before the ROS wrapper was created:

```text
catkin_make --pkg phase_offset_swarm                         PASS
catkin_make run_tests_phase_offset_swarm                     PASS (core-only)
```

Core XML results were 36/36 with zero failures/errors:

- NeighborManager: 10/10;
- ElasticInteraction: 7/7;
- ConflictFriction: 7/7;
- SwarmIntent aggregate: 12/12.

After the wrapper was added, the package and message build passed with the
system ROS Python (the workspace Miniconda Python lacked `netifaces`):

```text
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  --pkg phase_offset_msgs phase_offset_swarm                    PASS
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  run_tests_phase_offset_swarm                                   PASS
```

The final package test run contained the same 36/36 pure C++ tests plus the
registered ROS shadow acceptance 1/1, all with zero failures/errors.  The
`catkin_test_results` aggregate reports 74 tests because nested gtest suites
are counted twice by the tool; the XML truth is 36 pure C++ tests plus one
ROS test.

Existing `phase_offset_core` regression also passed:

```text
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  run_tests_phase_offset_core                                   PASS
```

Geometry: 15/15; PortProjector: 24/24; MatchedPort: 6/6 (45/45 total).

## Mathematical and numerical audit

- Separation uses `-k_sep * H(s_r) * n_ij`, so a close neighbor produces an
  outward intent; it is never scaled by beta.
- Cohesion uses the compact C2 bump `64*s^3*(1-s)^3`, points toward a distant
  organization neighbor, and is scaled only by the local clamped
  `beta_preview`.
- Conflict friction is exactly `k_conf * omega * dot_d * n_ij`; approaching
  pairs have `dot_d < 0` and are opposed, while separating pairs and common
  translation produce zero friction.
- `dot_d`, closing speed, TTC, and all distance decisions use predicted
  constant-velocity positions.  Zero distance uses the deterministic ID axis,
  preserving finite anti-symmetric pair contributions.
- Components are retained before saturation.  Only the complete raw sum is
  norm-saturated once to `g_max`; aggregate equality and direction-preserving
  saturation are covered by tests.
- Non-finite state input is rejected at the cache boundary.  Non-finite
  `beta_preview` fails closed to zero and finite beta is then clamped to
  `[0,1]`.  Outputs are finite except the permitted infinity sentinels for
  unavailable minimum distance/TTC diagnostics.
- No runtime feature flag, mode/state machine, fallback gate, hidden test
  switch, or second eligibility filter was added.  The only gates are the
  explicitly specified input validity/timestamp ordering, FRESH/STALE/LOST,
  distance hysteresis, positive closing/TTC selection, finite numerical
  handling, mandated odometry readiness, and local beta clamp.

## Communication and ROS shadow boundary

`AgentState.msg` contains exactly:

```text
std_msgs/Header header
uint16 robot_id
geometry_msgs/Point position_world
geometry_msgs/Vector3 velocity_world
```

No sequence/session, phase, offset, Tube, beta, intent, QP, ETA, branch,
priority, lease, or control field is broadcast.  `beta_preview` is a local
node parameter only and is never serialized into `AgentState`.

The shadow node subscribes to local `nav_msgs/Odometry` and shared
`AgentState`, then publishes only `geometry_msgs/Vector3Stamped` diagnostics:
`g_coord`, `g_sep`, `g_coh`, and `g_conf`.  It also republishes the local
four-field `AgentState`.  It does not advertise or call a PositionCommand,
velocity command, phase/offset command, allocator, CBF, matched-port,
governor, SO3, Tube, or `gvf_manager` interface.  The rostest checks both
topic names (`/position_cmd`, `/shadow/position_cmd`, `/cmd`,
`/shadow/cmd`) and published message types ending in `/PositionCommand`.

## ROS private-master evidence

The explicit acceptance run used:

- `ROS_MASTER_URI=http://127.0.0.1:12933`;
- `ROS_HOME=/tmp/codex_swarm_intent_shadow_reuse_20260820_233305/ros_home`;
- preflight port 12933 was free;
- `rostest --reuse-master --text phase_offset_swarm swarm_intent_shadow.test`;
- test result: 1/1 success, finite nonzero too-close approaching output;
- after teardown, `rosnode list` contained only `/rosout`, the master PID
  `891988` was gone, and port 12933 was gone.

The package-registered rostest was also run under a separate fresh
`ROS_HOME=/tmp/codex_swarm_intent_pkg_tests_20260820_233612/ros_home` and
passed 1/1.  Existing user-owned ROS masters (including Tube V2 port 12886)
were never contacted or terminated.

## Dependency and scope searches

The ROS-free core headers and four core `.cpp` files were searched for
`ros/`, `ros::`, `nav_msgs`, `geometry_msgs`, `phase_offset_msgs`,
`common_msgs`, `SDFMap`, `phase_offset_navigation`, `Tube`, `CBF`, `u_w`,
`u_delta`, and `PositionCommand`; no matches were found.  The core target
links only Eigen and the C++ standard library.  The ROS node's boundary search
found no `phase_offset_navigation`, `gvf_manager`, `PositionCommand`, CBF,
allocator, or matched-port dependency.  `AgentState.msg` was checked directly
against the four-field contract above.

`git diff --check` passed.  No stage-generated `__pycache__` or `.pyc` files
remain under either authorized package.  No historical prototype file was
added to CMake, and no path outside the whitelist was edited by this stage.

## Deviations and stop statement

The first registered rostest invocation used the workspace's Miniconda Python
and produced a missing-results artifact because that interpreter lacked
`netifaces`.  This was an environment-only test-runner issue; no source or
user-owned file was changed to bypass it.  Reconfiguring Catkin to use the
system ROS Python resolved it, and both the explicit private-master acceptance
and package-registered rostest then passed.

This stage is complete.  I explicitly stop before phase-offset allocation, CBF,
matched-port integration, Tube integration, multi-UAV controller activation,
and every control-chain change.
