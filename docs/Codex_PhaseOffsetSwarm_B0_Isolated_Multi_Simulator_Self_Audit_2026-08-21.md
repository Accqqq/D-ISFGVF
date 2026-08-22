# PhaseOffsetSwarm B0 Isolated Multi-Simulator Self-Audit

Status: G5 final regression and audit complete; B0 isolated multi-simulator
implementation is complete and released only for the B0 scope.

## Identity and scope

- Start timestamp: 2026-08-21T01:52:34+08:00 (first B0 whitelist artifact;
  continuation from accepted G0)
- Current audit timestamp: 2026-08-21T02:45:49+08:00
- Branch: `main`
- HEAD: `9a0e97560b8ddf15ac1ed1581030ed4b911f6e43`
- Scope: B0 isolated one/three-plant simulation only.
- No later swarm, navigation, planning, control, map, communication,
  allocator, CBF, Tube, or experiment work was started.

## Worktree evidence

The worktree was intentionally dirty at G0. Existing user changes outside the
whitelist were preserved. The initial tracked diff summary recorded by the
root supervisor was 17 files, 7,512 insertions, and 810 deletions. No files
were staged, committed, reset, restored, cleaned, or reverted.

Protected original SHA256 values (before and at this audit):

```text
src/uav_simulator/so3_quadrotor_simulator/src/quadrotor_simulator_so3.cpp
9a2452e2c329f3ef8f49faadb93e05e7f7f888d81dc977b040457009bfc65c92
src/uav_simulator/so3_quadrotor_simulator/launch/simulator.launch
d6339f7d6b0cde4beffa3b037ef28a2415ccd5fa1419f46a6b97df2c9d3134c6
src/uav_simulator/so3_quadrotor_simulator/src/dynamics/Quadrotor.cpp
0d5d0af994674ffefa87886a73bc580fc36606b34389346c9799b60d54d3281f
src/uav_simulator/so3_quadrotor_simulator/include/quadrotor_simulator/Quadrotor.h
86652b16839d503bc0fc7a21d8420926af5b7488383112f6acb89d76d02fb222
```

## Authorized B0 files

```text
src/uav_simulator/so3_quadrotor_simulator/CMakeLists.txt
src/uav_simulator/so3_quadrotor_simulator/package.xml
src/uav_simulator/so3_quadrotor_simulator/include/quadrotor_simulator/multi_quadrotor_simulator.h
src/uav_simulator/so3_quadrotor_simulator/src/multi_quadrotor_simulator_so3.cpp
src/uav_simulator/so3_quadrotor_simulator/launch/multi_simulator.launch
src/uav_simulator/so3_quadrotor_simulator/config/initial_states_1.yaml
src/uav_simulator/so3_quadrotor_simulator/config/initial_states_3.yaml
src/uav_simulator/so3_quadrotor_simulator/test/b0_multi_simulator_test.py
src/uav_simulator/so3_quadrotor_simulator/test/b0_multi_simulator_equivalence.test
src/uav_simulator/so3_quadrotor_simulator/test/b0_multi_simulator_three_agent.test
docs/Codex_PhaseOffsetSwarm_B0_Isolated_Multi_Simulator_Self_Audit_2026-08-21.md
```

## Static implementation findings

- The executable accepts exactly `num_agents=1` or `num_agents=3`.
- Startup requires an explicit `initial_states` array with exactly five fields
  per entry (`robot_id`, `x`, `y`, `z`, `yaw`), contiguous unique IDs, finite
  values, and positive `z`.
- Each agent owns its plant, command, disturbances, previous RPM control,
  messages, timestamps, publishers, and subscribers through a stable heap
  address.
- Topics are fixed to the `/uav_i` contract and frames are `world` plus
  `uav_i/base_link` by default.
- The launch has exactly one node and exactly six arguments: population,
  frame, rates, hover initialization, and state-file selector.
- The historical arbitrary-population layout and command cessation behavior
  were not migrated. Only the inherited per-motor non-finite RPM fallback is
  retained.
- The original single executable, launch, dynamics, and public plant header
  were not refactored or included by the new target.

## Static audits completed at G1

- Production multi header/source/launch/CMake/package contain no forbidden
  later-stage dependency or feature tokens from the execution specification.
- No controller, planner, map, visualization, or cross-agent topic appears in
  the new launch or production source.
- CMake keeps the original targets and link order, adds only the new target,
  and registers exactly the two B0 rostests.
- The equivalence test audits both node publication sets and all command,
  force, and moment subscriptions. `/rosout` is treated as normal ROS
  infrastructure and is filtered separately from the exact plant endpoint
  sets.
- The three-agent test freezes hover metrics at an explicit three-second
  checkpoint (rate, position/yaw error, altitude, and both frame IDs), then
  records per-stream before/after sample counts, timestamp progress, and finite
  tails for both command and disturbance phases. It also rejects any topic
  outside the expected plant/infrastructure graph.
- Python syntax check passed with:

  ```text
  PYTHONPYCACHEPREFIX=/tmp/codex_b0_pycache_20260821_01 python3 -m py_compile \
    src/uav_simulator/so3_quadrotor_simulator/test/b0_multi_simulator_test.py
  ```

  The generated cache was outside the workspace; the historical test cache
  was not touched.
- `git diff --check` passed for the tracked B0 package/CMake changes. The
  final G5 whitelist whitespace audit is recorded below.

## Runtime evidence

### G2 package build

Preflight at 2026-08-21T02:12:58+08:00 confirmed `main`, the expected HEAD,
the four protected hashes, and no concurrent build/test process. No ROS node,
master, or rostest was started.

The required command was run after sourcing `/opt/ros/noetic/setup.bash` and
`devel/setup.bash`:

```text
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 --pkg so3_quadrotor_simulator
```

The first pass ran from 02:12:58 to 02:13:12 and exited 1 because Eigen 3.3
requires an explicit `toRotationMatrix()` on the copied angle-axis product.
That was a whitelist-local compile defect; the original target still linked.
After the one-line header correction, the same command ran from 02:13:51 to
02:13:57 and exited 0. Build logs are retained at:

```text
/tmp/codex_b0_g2_build_20260821.log
/tmp/codex_b0_g2_build_retry_20260821.log
```

The retry log reports both targets:

```text
[ 85%] Built target quadrotor_simulator_so3
[100%] Linking .../devel/lib/so3_quadrotor_simulator/multi_quadrotor_simulator_so3
[100%] Built target multi_quadrotor_simulator_so3
```

Both generated link files reference the current workspace source object and
the shared `devel/lib/libquadrotor_dynamics.so`; they do not use the stale
historical multi binary:

```text
build/uav_simulator/so3_quadrotor_simulator/CMakeFiles/quadrotor_simulator_so3.dir/link.txt
build/uav_simulator/so3_quadrotor_simulator/CMakeFiles/multi_quadrotor_simulator_so3.dir/link.txt
```

Resulting target timestamps were 02:13:12 for the original executable and
02:13:57 for the new executable. No build process remained afterward.

### G3 single-agent equivalence

The first registered invocation at 02:17:16 exposed a test-environment import
issue (`quadrotor_msgs.msg` was an empty generated namespace after the
package-only build). The whitelist test harness now falls back to the checked-
in source-generated message package, uses an explicit `/usr/bin/python3`
shebang, and logs its interpreter. The 02:20:10 retry then exposed and fixed
one raw ROS master state-unpacking error. These were test-harness defects only;
no plant threshold, command waveform, rate, or protected source was changed.

The final registered target was run with:

```text
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  run_tests_so3_quadrotor_simulator_rostest_test_b0_multi_simulator_equivalence.test
```

Window: 2026-08-21T02:25:46+08:00--02:25:55+08:00, exit 0. XML:

```text
build/test_results/so3_quadrotor_simulator/rostest-test_b0_multi_simulator_equivalence.xml
```

The test log records `B0_TEST_PYTHON executable=/usr/bin/python3
version=3.8.10`. Registered metrics:

```text
matched_samples=689
max_position_error_m=0.002134229031906362
max_velocity_error_mps=0.000769259966021921
max_attitude_error_rad=0.0008959959697355549
original_odom_hz=99.99746494572223
multi_odom_hz=100.00000669936549
```

The XML reports one test, zero errors, and zero failures.

An independent acceptance used a fresh private master:

```text
run_dir=/tmp/codex_b0_multi_sim_equiv_20260821_0226
ROS_HOME=/tmp/codex_b0_multi_sim_equiv_20260821_0226/ros_home
ROS_MASTER_URI=http://127.0.0.1:60575
master PID/PGID=925967/925967
rostest PID/PGID=925999/925999
command=rostest --reuse-master so3_quadrotor_simulator b0_multi_simulator_equivalence.test
```

The private run launched the two current devel executables and the test from
02:26:38 through teardown at 02:26:47, exited 0, and recorded graph snapshots
only through the private URI. Its XML is under the run's ROS_HOME test-results
directory. Metrics:

```text
matched_samples=688
max_position_error_m=0.0057547234351889446
max_velocity_error_mps=0.0015672059995766876
max_attitude_error_rad=0.0005096767528360851
original_odom_hz=99.99331516337753
multi_odom_hz=100.00864801393413
```

The private graph contained exactly the original remapped odom/IMU/command
endpoints, the multi `/uav_0` plant endpoints, and the expected original and
multi force/moment subscriptions, plus ROS infrastructure. The final graph
after teardown contained only `/rosout`. Both owned process groups were empty
after cleanup and `PORT_FREE=1` proved port 60575 could be rebound.

### G4 three-agent isolation

The registered target was run after the final IMU-frame assertion was added:

```text
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  run_tests_so3_quadrotor_simulator_rostest_test_b0_multi_simulator_three_agent.test
```

Window: 2026-08-21T02:29:37+08:00--02:29:48+08:00, exit 0. XML:

```text
build/test_results/so3_quadrotor_simulator/rostest-test_b0_multi_simulator_three_agent.xml
```

The registered run logged `/usr/bin/python3` 3.8.10 and one test with zero
errors/failures. Frozen metrics were:

```text
hover_duration_s=3.000513500010129
each odom_hz=100.00104426833359
each position_error_m=3.7469678101365034e-06
each yaw_error_rad=0.0
each z_m=0.9999962530321899
each odom_frame_id=world, imu_frame_id=world
child_frame_ids=uav_0/base_link,uav_1/base_link,uav_2/base_link
command displacement=[10.341897576433265, 1.4941507799459686e-05,
                      1.4941507799459686e-05]
command phase: every stream growth=401, stamp_delta_s=4.009993314743042,
  phase_odom_hz=100.00001788139663, finite=true
disturbance displacement=[0.28897513197594266, 1.0618731543732274e-05]
disturbance phase: every stream growth=170, stamp_delta_s=1.6999926567077637,
  phase_odom_hz=99.99878337466812, finite=true
```

The independent private acceptance used:

```text
run_dir=/tmp/codex_b0_multi_sim_three_20260821_0231
ROS_HOME=/tmp/codex_b0_multi_sim_three_20260821_0231/ros_home
ROS_MASTER_URI=http://127.0.0.1:37263
master PID/PGID=931890/931890
rostest PID/PGID=931922/931922
command=rostest --reuse-master so3_quadrotor_simulator b0_multi_simulator_three_agent.test
```

Window: 2026-08-21T02:31:39+08:00--02:31:52+08:00, exit 0. XML:

```text
/tmp/codex_b0_multi_sim_three_20260821_0231/ros_home/test_results/so3_quadrotor_simulator/rostest-test_b0_multi_simulator_three_agent.xml
```

The test log again records `/usr/bin/python3` 3.8.10. Private metrics were:

```text
hover_duration_s=3.0003417370026
uav_0/uav_1 odom_hz=99.97787170627102; uav_2 odom_hz=99.97773666237768
position_error_m=[3.749269735120464e-06,3.749269735120464e-06,
                  3.772327867523906e-06]
yaw_error_rad=[0.0,0.0,0.0], z_m≈0.9999963, all odom/imu frame_id=world
command displacement=[10.240743355628616,1.4844130507007947e-05,
                      1.4844130507007947e-05]
command phase: growth=[400,400,400], stamp_delta_s=3.9994804859161377,
  phase_odom_hz=100.01266922020017, finite=true
disturbance displacement=[0.28432774518829407,1.0533568495563194e-05]
disturbance phase: growth=[170,170,170], stamp_delta_s=1.6998374462127686,
  phase_odom_hz=100.01095942672812, finite=true
```

Graph snapshots queried only through the private master showed exactly the 15
`/uav_0..2` plant topics (six publications and nine subscriptions), exactly
one multi-simulator node, no fourth-agent topic, no global coordination,
planner, map, or control endpoints, and the expected ROS logging services.
The final post-teardown graph contained only `/rosout`. Cleanup proved both
owned process groups empty and `PORT_FREE=1` for port 37263.

After G4, an unrelated user full-workspace `catkin_make -j2` appeared (PGID
928975). It was not queried beyond process-state observation and was not
signaled or modified. The later G5 build was started only after the build tree
was clear.

### G5 combined registered regressions

Both B0 registered rostests were rerun together with:

```text
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  run_tests_so3_quadrotor_simulator_rostest
```

Window: 2026-08-21T02:36:57+08:00--02:37:17+08:00, exit 0. Log:

```text
/tmp/codex_b0_g5_registered_20260821.log
```

The registered result XMLs were:

```text
build/test_results/so3_quadrotor_simulator/rostest-test_b0_multi_simulator_equivalence.xml
build/test_results/so3_quadrotor_simulator/rostest-test_b0_multi_simulator_three_agent.xml
```

Each XML contains one test, zero errors, and zero failures. The final
equivalence metrics were:

```text
matched_samples=689
max_position_error_m=0.007509810033232963
max_velocity_error_mps=0.0017820446520873143
max_attitude_error_rad=0.0003161525080339
original_odom_hz=100.00044443528233
multi_odom_hz=100.00466609898632
```

The final registered three-agent metrics remained within the frozen G4
contracts:

```text
hover_duration_s=3.0005188840150367
odom_hz≈99.997416
position_error_m=3.7033655533313947e-06
yaw_error_rad=0
z_m=0.9999962966
odom_frame_id=world
imu_frame_id=world
child frames=uav_0/base_link,uav_1/base_link,uav_2/base_link
command displacement=[10.332632028045227,1.4864658377078399e-05,
                      1.4864658377078399e-05]
command growth=[400,400,400], phase rate≈100 Hz, finite
disturbance displacement=[0.28901497841118645,1.0619933346056598e-05]
disturbance growth=[170,170,170], phase rate≈100 Hz, finite
```

### G5 original single-node smoke

The original executable was exercised directly on a fresh private master,
without using the multi launch or any later-stage node:

```text
run_dir=/tmp/codex_b0_single_smoke_20260821_0237
ROS_HOME=/tmp/codex_b0_single_smoke_20260821_0237/ros_home
ROS_MASTER_URI=http://127.0.0.1:39205
master PID/PGID=936208/936208
original node PID/PGID=936236/936236
window=2026-08-21T02:38:44+08:00--02:38:50+08:00
exit=0
```

The smoke produced 401 finite odometry/IMU samples at 100.0000535 Hz, with
position error `6.318e-06 m`, odometry `frame_id=world`, IMU `frame_id=world`,
and original odometry `child_frame_id=/quadrotor`, plus the original
odometry/IMU/command endpoints. Both process groups were empty after cleanup
and port 39205 was free.

### G5 full incremental workspace build

After the smoke cleanup, the requested incremental workspace build completed:

```text
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -j2
```

Window: 2026-08-21T02:39:12+08:00--02:39:14+08:00, exit 0. Log:

```text
/tmp/codex_b0_g5_full_build_20260821.log
```

Both `quadrotor_simulator_so3` and `multi_quadrotor_simulator_so3` built
successfully, and no build process remained afterward.

### Final G5 read-only audits

- The protected original hashes listed above were recomputed and matched
  exactly before and after the B0 work.
- Production multi-simulator header/source/launch/CMake/package searches found
  no forbidden later-stage dependency or feature token, no extra gate/mode,
  no controller/planner/map/communication endpoint, and no arbitrary-N or
  command-timeout behavior.
- The launch contains exactly one node and six arguments; CMake contains exactly
  two `add_rostest` registrations, one for each B0 test.
- The final tracked diff was 19 files, 7,539 insertions, and 813 deletions;
  this includes the pre-existing unrelated dirty worktree plus the two tracked
  B0 CMake/package edits. The remaining eight B0 implementation/test/config
  files and this audit are untracked whitelist additions. No files were staged,
  committed, reset, restored, cleaned, or reverted.
- `git diff --check` and the explicit trailing-whitespace scan over every B0
  whitelist file passed. The only simulator-test cache present is the preserved
  historical `src/uav_simulator/so3_quadrotor_simulator/test/__pycache__/multi_sim_tests.cpython-310.pyc`;
  stage-generated cache artifacts were cleaned, and that historical cache was
  left untouched.
- Final process inspection found no B0 build, test, ROS master, or simulator
  process owned by these runs; all private ports used by G3, G4, and G5 were
  free after teardown.

Equivalence metrics (minimum 300 matched samples; frozen tolerances) are
recorded in the G3 and G5 sections above. Three-agent hover/rate/frame,
per-stream phase-liveness, command-isolation, disturbance-isolation, private
master, process-group, port, original-smoke, and full-build evidence are all
recorded above. No deviation or blocker remains within B0.

## Stop declaration

This audit stops at B0. Work is explicitly stopped before B1, B2, or B3; Swarm
Intent execution; allocator or CBF integration; Tube construction; planner,
navigation, map, communication, or control-chain integration; and any
experiment/scenario work. None of those later-stage changes were authorized or
started by this work.
