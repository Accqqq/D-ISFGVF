# PhaseOffsetSwarm B1E G12 comparison and self-audit

Date: 2026-08-21

This is the root-released G12 comparison and final static self-audit for the
B1E original-to-B1 single-UAV diagnostic. The only workspace file written in
G12 is this document. The main B1E specification and the G3R correction
specification were read completely and remained read-only.

## Scope, authority, and identity

- Executor: Luna Max personally; no sub-agent was started or consulted.
- Branch: main.
- HEAD: 9a0e97560b8ddf15ac1ed1581030ed4b911f6e43.
- Main B1E specification SHA256:
  e9e3a1506514168b0fa8b949188f3848680c0f7046f24e55cdadd5333797b1de.
- G3R correction specification SHA256:
  4ebc14608de8a7a50b86d574e3feab7f770495dbc691fc0643ec22fbbbc3802d.
- Corrected diagnostic SHA256:
  70234fc756ee144de5be315fcf14214c37f635b5167492472ab8f3f3c354e777.
- No Tube, Swarm Intent, allocator, CBF, controller-chain, or production
  source was edited or connected. No B1 three-UAV run or production
  correction was started.
- The pre-existing dirty worktree was preserved. G12 did not stage, restore,
  format, or delete any user-owned file.

## G2 build and G3R correction provenance

The exact Section 21 build command was:

    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 --pkg so3_quadrotor_simulator map_generator so3_control bspline_race

The retained build log is
/tmp/codex_b1_g2_catkin_make_20260821.log. It records
BUILD_START=2026-08-21T14:42:00+08:00,
BUILD_END=2026-08-21T14:42:09+08:00, BUILD_EXIT_CODE=0, and successful
formation_planning, map_pub/local_sensing, SO3, original simulator, and
multi simulator targets. No build was run during G12.

The original invalid E0 attempt
/tmp/codex_b1e_e0_20260821T084929Z_1051605 was retained only as correction
history. It was invalid for fixture reasons, not a production regression:

1. The diagnostic unpacked the direct three-section return of
   rosgraph.Master.getSystemState() as an XML-RPC status tuple, so a valid
   graph state was reported as a graph exception.
2. It selected the earliest retained odometry sample instead of the latest
   sample immediately before the one goal publication. With
   start_at_hover=false, the startup transient produced the invalid
   initial_position_error_m=0.8341462588291859.
3. The old executable did not create the requested JUnit XML result.

G3R corrected only the diagnostic fixture: direct three-section decoding,
latest pre-goal state semantics using the existing readiness interval, and a
minimal standard JUnit artifact. It did not change thresholds, goal-height
semantics, plant, SO3, planner, or control behavior. The corrected diagnostic
hash above is unchanged across all nine valid runs.

## Nine-run causal matrix

All nine runs below are the valid, non-invalid-history runs. Each retained
non-latest diagnostic stdout contains exactly one B1E_METRICS line, with
run_valid=true, infrastructure_failures empty, one goal publication, and a
passing one-test JUnit artifact. Strict values are diagnostic outcomes, not
process-validity outcomes.

| Variant | Unique factor | run_valid | strict | Initial error (m) | Min XY error (m) | Min Z error (m) | Min 3D error (m) | Near-goal min speed (m/s) | Last PositionCommand receive time / z | Final physical z | Path / physical clearance (m) | Odom Hz | Run directory |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | ---: | --- |
| E0R | headless original-equivalent reference | true | false | 0.00769807938 | 4.74572825e-11 | 1.15983002e-05 | 0.764979257 | 6.40375039e-10 | 6.230120 / 2.000000000 | 1.999999917 | 0.524648188 / 0.492898726 | 99.998617589 | /tmp/codex_b1e_e0r_20260821T092531Z_1066475 |
| E1 | manual layer only | true | false | 0.0140417646 | 7.06452877e-12 | 0.000217136401 | 0.799111601 | 1.25170901e-10 | 6.232926 / 2.000000000 | 1.999999917 | 0.500673920 / 0.478669970 | 99.998819047 | /tmp/codex_b1e_e1_20260821T093208Z_1069445 |
| E2 | namespace/remap boundary only | true | false | 0.00984571048 | 3.10670151e-12 | 0.000127731968 | 0.675871808 | 1.40044233e-10 | 6.357603 / 2.000000000 | 1.999999918 | 0.532784989 / 0.472044183 | 99.998992235 | /tmp/codex_b1e_e2_20260821T093952Z_1071626 |
| E3 | SO3 loading topology only | true | false | 0.00968500511 | 2.45058087e-12 | 0.000133596020 | 0.733167533 | 1.43943315e-10 | 6.549839 / 2.000000000 | 1.999999917 | 0.533222363 / 0.522837748 | 100.000322150 | /tmp/codex_b1e_e3_20260821T095209Z_1074553 |
| E4 | frame metadata only | true | false | 0.0140789988 | 1.13114526e-11 | 1.58886362e-05 | 0.654382545 | 1.53188220e-10 | 6.498241 / 2.000000000 | 1.999999918 | 0.535770809 / 0.488046163 | 99.998694417 | /tmp/codex_b1e_e4_20260821T100329Z_1076506 |
| E5 | hover initialization only | true | false | 8.27406611e-08 | 2.39551966e-10 | 8.27618035e-08 | 0.636550609 | 5.02061761e-10 | 6.942682 / 2.000000000 | 1.999999917 | 0.533780373 / 0.523963152 | 99.999964237 | /tmp/codex_b1e_e5_20260821T101139Z_1078066 |
| E6 | single-to-multi plant implementation only | true | false | 8.83617503e-08 | 7.73376709e-11 | 8.82663953e-08 | 0.841181052 | 3.12945054e-10 | 6.222869 / 2.000000000 | 1.999999917 | 0.512919890 / 0.465510154 | 100.000446861 | /tmp/codex_b1e_e6_20260821T105522Z_1087114 |
| E7 | effective use_external_yaw only | true | false | 8.22540445e-08 | 1.26543564e-10 | 8.22693076e-08 | 0.701765007 | 8.61954799e-10 | 6.324768 / 2.000000000 | 1.999999917 | 0.518197487 / 0.489239673 | 100.000207443 | /tmp/codex_b1e_e7_20260821T110328Z_1089156 |
| E8 | common gain scale only | true | false | 8.29167235e-08 | 9.05160860e-11 | 8.29415508e-08 | 0.854257658 | 7.78489676e-10 | 13.523142 / 2.000000000 | 1.999999916 | 0.505644698 / 0.512828879 | 99.999979811 | /tmp/codex_b1e_e8_20260821T111425Z_1093344 |

The minimum XY, minimum Z, minimum 3D, and near-goal minimum-speed values
are separate minima. They are not a claim that one sample jointly achieved
all four conditions. In every row, strict_b1_pass=false with
strict_failure_components=[first_stable_window,xy_z_never_jointly_ok].

Per-run evidence also proves finite four-stream data, positive timestamp
progress for local map, odometry, PositionCommand, and SO3Command, and odom
rates within the required 80--130 Hz band. The nine per-test JUnit files each
contain one testcase, zero errors, zero failures, and no B1E_METRICS copy.

## Eight adjacent causal conclusions

Each adjacent conclusion below cites its retained adjacent-delta evidence.
Both variants in every pair were valid, so the named launch-contract factor
is isolated. No factor changed strict B1 status.

1. E0R to E1, manual layer:
   E0 manual enable/auto-load true versus E1 all manual flags false. Both
   strict values remained false, both terminal commands/final states remained
   near z=2, and all streams stayed live. Evidence:
   /tmp/codex_b1e_e1_20260821T093208Z_1069445/evidence/adjacent_delta_e0_to_e1.txt.

2. E1 to E2, namespace/remap:
   Global planner/map endpoints became the isolated /uav_0 contract. Both
   strict values remained false; the namespaced graph had no unexpected
   global nodes/endpoints or cross-agent edges. Evidence:
   /tmp/codex_b1e_e2_20260821T093952Z_1071626/evidence/adjacent_delta_e1_to_e2.txt.

3. E2 to E3, SO3 loading topology:
   Standalone /so3_control became one manager with exactly
   /so3_control_nodelet_0. Both strict values remained false; manager
   endpoint union and NodeletList were exact. Evidence:
   /tmp/codex_b1e_e3_20260821T095209Z_1074553/evidence/adjacent_delta_e2_to_e3.txt.

4. E3 to E4, frame metadata:
   Original odometry frame /simulator became world while manager topology,
   namespace, and gains remained fixed. Both strict values remained false and
   E4 reported world odom/local-map/path frames. Evidence:
   /tmp/codex_b1e_e4_20260821T100329Z_1076506/evidence/adjacent_delta_e3_to_e4.txt.

5. E4 to E5, hover initialization:
   start_at_hover changed false to true. Strict remained false, but startup
   readiness materially improved: initial error changed from
   0.014078998755084005 m to 8.274066110036671e-08 m and the initial
   near-start speed became approximately 1.62e-09 m/s. This is initialization
   behavior, not a new hold gate or strict-arrival cause. Evidence:
   /tmp/codex_b1e_e5_20260821T101139Z_1078066/evidence/adjacent_delta_e4_to_e5.txt.

6. E5 to E6, single-to-multi plant:
   The original single plant became protected B0 multi N=1 with the B1
   initial-state YAML. Strict remained false; E6 had the expected multi plant,
   no original single plant, valid manager graph, and the same terminal z≈2
   behavior. Evidence:
   /tmp/codex_b1e_e6_20260821T105522Z_1087114/evidence/adjacent_delta_e5_to_e6.txt.

7. E6 to E7, effective SO3 yaw parameter:
   The only production parameter delta was
   /so3_control_nodelet_0/use_external_yaw true to false. Strict remained
   false, with all other production parameters, graph endpoints, frames, and
   rates equivalent. Evidence:
   /tmp/codex_b1e_e7_20260821T110328Z_1089156/evidence/adjacent_delta_e6_to_e7.txt.

8. E7 to E8, common gain scale:
   The only production parameter deltas were GVF gain1 2.0 to 0.8 and gain2
   -2.2 to -0.88. The last PositionCommand time moved from
   6.3247683009831235 s to 13.523141996003687 s, but both terminal command
   z values, settled physical z values, and strict status remained unchanged.
   Evidence:
   /tmp/codex_b1e_e8_20260821T111425Z_1093344/evidence/adjacent_delta_e7_to_e8.txt.

The first material operational change in the matrix is E4 to E5 startup
readiness/hover initialization; it did not change strict acceptance. The
topology changes (namespace and manager), frame/hover/plant changes, the
effective yaw parameter, and gain scaling all preserve the same strict
false outcome. Thus no adjacent factor explains the strict result.

## Structure, graph, parameter, and JUnit conclusions

- All nine valid runs have one goal publication, a connected goal subscriber,
  complete 60 s observations, finite data, non-empty paths, and empty
  parameter-failure lists.
- All four observed streams have positive timestamp progress in all nine runs;
  every odometry rate is between 99.9986 and 100.0005 Hz.
- The namespaced E2--E8 graph audits report no unexpected global nodes or
  endpoints, no nonzero-agent nodes, no unexpected agent endpoints, no
  cross-agent edges, and no shared algorithm edges.
- E3--E8 manager audits report NodeletList exactly
  [/so3_control_nodelet_0], no nodelet mismatch/service error, and exact
  operational endpoint union
  [/uav_0/position_cmd, /uav_0/sim/imu, /uav_0/sim/odom,
  /uav_0/so3_cmd, /uav_0/motors, /uav_0/corrections].
- E7 to E8 runtime parameter comparison changes only the two GVF gains.
  Static roslaunch --dump-params comparison likewise changes only those two
  production keys after fixture expected_* metadata is excluded.
- The protected manual-map file hash is
  740b4f1da4d42cf75729cb0af1b0bc2f7f3d422905e30cfb5cd43f13a8fd434f.
  The retained E0 hash verification reports all 23 protected/B1 entries
  matched after E0; the current hash matches.
- Every per-test JUnit result is one testcase with zero errors/failures and no
  B1E_METRICS in the artifact or rostest system-out.

## Goal-height semantics and interpretation

The protected implementation's gvf_manager::goalCallback is the key
interpretation point. In
src/swarm_planner/bspline_traj/src/gvf_manager.cpp:

- lines 446--485 receive the marker goal and set
  marker.pose.position.z = msg->pose.position.z + 1.0;
- lines 487--492 construct goal_pt with
  msg->pose.position.z + 1.0;
- lines 544--550 assign that goal_pt to the active manager.

Therefore the diagnostic publishes the frozen goal (5,-3.5,1), while the
protected planner's effective terminal PositionCommand and settled physical
state are near z=2. The strict B1 metric remains correctly evaluated against
the published z=1 goal: XY can become tiny and speed can become tiny, but the
XY/Z joint condition never holds. The user has confirmed that this +1.0
behavior is present in the original navigation and should be preserved.

The user's independently observed original navigation is normal. The evidence
does not show a Tube, plant, SO3, GVF, or control-chain fault. It shows a
semantic difference between the frozen published-goal strict acceptance and
the protected planner's effective terminal goal, which explains visual
stopping versus strict B1 failure without changing thresholds or production
behavior.

## Final static audit

The following checks were run read-only during G12:

- Direct import of the corrected diagnostic and synthetic_checks:
  16 cases, all_pass=true. Every one of the nine retained metrics objects also
  reports the same 16 synthetic cases all passing.
- xmllint --noout passed for all 17 unchanged B1E XML files (8 launches and
  9 .test files).
- roslaunch --nodes passed for all 17 unchanged B1E XML files.
- Static publisher search found exactly one diagnostic goal Publisher creation
  and exactly one goal_publisher.publish(goal) call. No PositionCommand,
  SO3Command, motors, or corrections publisher exists in the diagnostic.
- Forbidden live node/topic attribute scan found no Tube, Swarm Intent,
  allocator, CBF, scenario, visualizer, or RViz endpoint. The only
  phase-offset entries are the required disabled phase_offset_mode and
  phase_offset/mode settings. The only fallback token in the diagnostic is
  the safe checked-in message-package import fallback, not a runtime control
  fallback or gate.
- Diagnostic Python mode remains 775.
- No B1E-named workspace pycache exists.
- git diff --check passed. Targeted B1E launch/test/diagnostic trailing
  whitespace scan found zero lines. Pre-existing unrelated worktree changes
  were not modified.
- A read-only process check found no catkin_make, cmake --build, make, ninja,
  or run_tests.py writer. Existing user-owned ROS processes and listening
  ports were observed read-only and were not queried through a ROS master,
  signaled, or cleaned.

### B1E implementation hashes (18)

    87e541c3192cef90283537d9a612a6d194803f20607d04572b3c3e9de2e712d7  src/swarm_planner/bspline_traj/launch/b1e_original_global_manual_on.launch
    d88df6c4a26bc6b594bb41fdda79a37658af40bf79534cbcaa6a41faec1a6c15  src/swarm_planner/bspline_traj/launch/b1e_original_global_manual_off.launch
    31ab04f5673258b9ee0c25ae40e227a106f923a473dc9a77e330877a51b34ffb  src/swarm_planner/bspline_traj/launch/b1e_namespaced_single_standalone_original_gains.launch
    1312e65aacbd10993cd41adf2e6052e4dfa08a0f3e06752aab55fdba19c6f6c9  src/swarm_planner/bspline_traj/launch/b1e_namespaced_single_manager_original_gains.launch
    0cc792ccb0a81b68d8165a7f6df1115832eccc2e30d4a2e4fa1355f05891c5b1  src/swarm_planner/bspline_traj/launch/b1e_namespaced_single_manager_world_frame_original_gains.launch
    4e65d82926cc3eef2da7c18c8a5a4e7cc8c3f6aafd8913a12ed6a6d750afd01a  src/swarm_planner/bspline_traj/launch/b1e_namespaced_single_manager_world_hover_original_gains.launch
    2aedd4a26ff4ee1328393213906745e08c952957853505e13df3bb1fee496529  src/swarm_planner/bspline_traj/launch/b1e_namespaced_multi_manager_original_gains.launch
    4385c4022d95e0ad8183b974177b6d34ee03378913d15f93deecbf5cf21dfc38  src/swarm_planner/bspline_traj/launch/b1e_namespaced_multi_manager_original_gains_b1_so3_params.launch
    70234fc756ee144de5be315fcf14214c37f635b5167492472ab8f3f3c354e777  src/swarm_planner/bspline_traj/test/b1e_single_equivalence_diagnostic_test.py
    1b9b293e04f5bb8820d9cb4d82670256bee072044e93224d4fc15d07392e3845  src/swarm_planner/bspline_traj/test/b1e_e0_original_global_manual_on.test
    3257980f33a66903ad224942a41c2dc615aa45f41829e6655608dafbc1cb3c3e  src/swarm_planner/bspline_traj/test/b1e_e1_original_global_manual_off.test
    e8e3165e6e16d1d0da9079aaaf297d1fb2d9ad1c60a0ec176400731b55524ad9  src/swarm_planner/bspline_traj/test/b1e_e2_namespaced_single_standalone.test
    cf344e36e4947a5bf07075c8c0f4a1727aa866e2303a34e7609cd8a339fa0992  src/swarm_planner/bspline_traj/test/b1e_e3_namespaced_single_manager.test
    0059c4f2bcc6caf5a1c12825b8ec3c79c7000e70b09187953d588b3fca76e860  src/swarm_planner/bspline_traj/test/b1e_e4_namespaced_single_manager_world_frame.test
    e086f048d0d9d020b3ca8101343420dd3da7221054b418622a481fa6a272dc21  src/swarm_planner/bspline_traj/test/b1e_e5_namespaced_single_manager_world_hover.test
    c8867933da90aef71698394f8d1b124332920b23c6114ddd999d12bd2a76a920  src/swarm_planner/bspline_traj/test/b1e_e6_namespaced_multi_manager_original_gains.test
    67b1b5d2252cb17ec5d94d83a97d63c82f545e767c3971a0da6b1b1dc71ff386  src/swarm_planner/bspline_traj/test/b1e_e7_namespaced_multi_manager_b1_so3_params.test
    dc0b243d0ca8551f3540d3c0a4f37295942f0e463d0a46f71502d4a1d4912a2b  src/swarm_planner/bspline_traj/test/b1e_e8_b1_scaled_gains.test

### Protected and B1 hashes (23)

    ce1ff4a3daca6efeef39507c14227afd806f94d55b85c200b8c8a5032b213fc5  AGENTS.md
    cc6ae11d739c62d20f937acc5267fa542857f0a89e4d57dc56f5ae8823985555  src/swarm_planner/bspline_traj/launch/test_gvf.launch
    4a454d6a997e875d50114b51d920a109638f84c8fc38cd2ef803d4d401f43314  src/swarm_planner/bspline_traj/src/gvf_manager.cpp
    d13475010de18dd3f7280b7aee31aad6b7431560cc248468608906de0c290788  src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
    f98805f3a8ff44386480dabdab1de80934dd0e6fc7076ad583172a4806f01880  src/uav_simulator/dynamic_map_generator/src/local_sensing.cpp
    609cab63db1209be0d0390931782eb702859a0d85964a7f3ff8394966ec23414  src/uav_simulator/dynamic_map_generator/resource/pillar.pcd
    b39a59b20383b90b26075a207a3df4c5e402b0955ee7df6bd60f6d61726fc661  src/uav_simulator/so3_control/src/so3_control_nodelet.cpp
    708f6922c6f541cf9d979bc6a12f7692abb69203d4bd79f066f4d33bee11df07  src/uav_simulator/so3_quadrotor_simulator/launch/multi_simulator.launch
    bee45213c89f203c8364ebd0e5da7b9e641375c846fdc2d08accf3541962c6e2  src/uav_simulator/so3_quadrotor_simulator/src/multi_quadrotor_simulator_so3.cpp
    7b639f072e7acb88426a14018e1c8ab93106c06bdd028f10eb9371cdb9e153c3  src/uav_simulator/so3_quadrotor_simulator/include/quadrotor_simulator/multi_quadrotor_simulator.h
    9a2452e2c329f3ef8f49faadb93e05e7f7f888d81dc977b040457009bfc65c92  src/uav_simulator/so3_quadrotor_simulator/src/quadrotor_simulator_so3.cpp
    d6339f7d6b0cde4beffa3b037ef28a2415ccd5fa1419f46a6b97df2c9d3134c6  src/uav_simulator/so3_quadrotor_simulator/launch/simulator.launch
    0d5d0af994674ffefa87886a73bc580fc36606b34389346c9799b60d54d3281f  src/uav_simulator/so3_quadrotor_simulator/src/dynamics/Quadrotor.cpp
    86652b16839d503bc0fc7a21d8420926af5b7488383112f6acb89d76d02fb222  src/uav_simulator/so3_quadrotor_simulator/include/quadrotor_simulator/Quadrotor.h
    740b4f1da4d42cf75729cb0af1b0bc2f7f3d422905e30cfb5cd43f13a8fd434f  src/swarm_planner/bspline_traj/config/manual_obstacles_test_gvf.txt
    12e706e8153140d5c46663a7c2af0ed3c3ea8adcb077a7617a248ae1b202410a  src/swarm_planner/bspline_traj/launch/b1_independent_agent.launch
    ee1e171610c0b2072a8fdf49e52de7d5483bc34c836ec724932a9c37f5d1eb4b  src/swarm_planner/bspline_traj/launch/b1_pillar_single_baseline.launch
    841e449cd1d7be5a1ccfc1e4112eec886480c0cf8c29ab971848c326f423259a  src/swarm_planner/bspline_traj/launch/b1_pillar_independent_3.launch
    7b75d77f567f6521c213c7ba5d893dfd74c88ddc36ab63f3d8116aeeb6853acb  src/swarm_planner/bspline_traj/config/phase_offset_swarm/b1/initial_states_1.yaml
    727c1c98efeb06aa133346e5cfbf4f126a0288762f2b487b099f6b2117cac668  src/swarm_planner/bspline_traj/config/phase_offset_swarm/b1/initial_states_3.yaml
    931bc55b6fe5673e3d8b6129494846780008df5c91fb55c933ceba18f7d5a044  src/swarm_planner/bspline_traj/test/b1_independent_navigation_test.py
    2f26d62d0ec9ef3734116cb6a4067efa389b4aa7af8559d5a734579a9fa0ed3e  src/swarm_planner/bspline_traj/test/b1_independent_single.test
    8d5f53de60df7d998c81362b3b6afe00aab69d860981e7e799338097580eae00  src/swarm_planner/bspline_traj/test/b1_independent_three.test

## Procedural deviation recorded honestly

The E7 run directory contains a file named
/tmp/codex_b1e_e7_20260821T110328Z_1089156/evidence/final_self_audit.txt.
This was a runtime evidence filename created during E7 before the G11
instruction explicitly prohibited a self-audit filename. It is outside the
workspace, did not change code, did not alter the E7 metrics, and did not
interact with Tube or control processes. No G12 self-audit was created before
this whitelisted document, and no other G12 workspace file was written.

## Final required conclusion

Outcome 9 of Section 25 is supported:

all implementations navigate normally but strict B1 acceptance differs from
visual stopping.

Every adjacent pair is valid and retains strict_b1_pass=false. The measured
terminal z≈2 behavior follows the existing protected +1.0 goal-height
semantics, while the frozen strict metric is evaluated against published z=1.
No adjacent manual, namespace, topology, frame, hover, plant, yaw, or gain
factor changes that strict state.

## STOP

This audit ends here. Do not perform a production correction, change the B1
acceptance goal or thresholds, continue B1 three-UAV navigation, or connect
Swarm Intent, Tube, allocator, or CBF to control. Any subsequent action
requires a new root-owned execution specification.
