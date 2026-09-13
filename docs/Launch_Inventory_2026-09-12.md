# Launch 清单与阶段归属（2026-09-12）

目的：回答"哪个 launch 现在还用、哪个只是历史阶段的验收资产"。

**2026-09-12 变更**：B 类 24 个历史 launch 中的 23 个已在
`src/swarm_planner/bspline_traj/launch/` 下删除（用户明确授权）。
该目录现在只剩 `test_gvf.launch` 与 `gvf.launch` 两个现役文件。
被删文件可从 commit `d3adc73` 完整恢复（该提交已推到 D-ISFGVF）。
下表 B 段保留，作为"这些文件当时属于哪个阶段"的索引。

复核方式（本文所有结论都可用这两条命令重算）：

```bash
cd /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
rg -w -l "<launch 文件名>" src        # 还有谁 include / 引用它
grep -n "add_rostest" src/swarm_planner/bspline_traj/CMakeLists.txt   # 无输出 = 未注册
```

---

## 1. 现在真正在跑的链路（只有三条）

```text
① 单机仿真
   so3_quadrotor_simulator/simulator.launch  +  bspline_race/test_gvf.launch

② 集群仿真（当前主线）
   phase_offset_sim.launch
     ├── phase_offset_world.launch     地图 + 集群可视化 + 两个 RViz
     └── phase_offset_swarm.launch     启动 swarm_orchestrator.py
           └── 生成 sim_b_child.launch
                 ├── so3_quadrotor_simulator/multi_simulator.launch
                 └── phase_offset_agent.launch            （每台机一份）
                       ├── bspline_race/test_gvf.launch    （逐机规划核心）
                       └── 邻居 runtime 节点（inline，enable_neighbor_transport 时）

③ 实机单机模板
   bspline_race/gvf.launch （机上那份已改成 /ov_msckf/odomimu）
```

关键点：**集群逐机也走 `test_gvf.launch`**，所以单机核心和集群核心是同一个文件，
差别只在传进去的参数（协同 backend / 邻居开关 / 幅度）。

---

## 2. Launch 索引表

### A. 现役（还在上面三条链路里，动之前要想清楚）

| launch | 阶段 | 谁驱动它 | 现在用吗 |
| --- | --- | --- | --- |
| `bspline_traj/launch/test_gvf.launch` | 全程核心 | ① 手动；② `phase_offset_agent.launch` include | **在**。单机与集群共用，tube/相位/协同参数都在这里 |
| `bspline_traj/launch/gvf.launch` | 实机单机 | 手动（机上那份） | **在**。实机集群 launch 的起点 |
| `phase_offset_sim_bringup/launch/phase_offset_sim.launch` | 当前集群 | 手动（一条命令入口） | **在** |
| `phase_offset_sim_bringup/launch/phase_offset_world.launch` | 当前集群 | `phase_offset_sim.launch` | **在**（地图 + RViz×2） |
| `phase_offset_sim_bringup/launch/phase_offset_swarm.launch` | 当前集群 | `phase_offset_sim.launch` | **在**（起 orchestrator） |
| `phase_offset_sim_bringup/launch/phase_offset_agent.launch` | 当前集群 | orchestrator 生成的子 launch | **在**（逐机） |
| `phase_offset_sim_bringup/launch/phase_offset_agent_view.launch` | 当前集群 | 手动（只看跟随视角） | **在**（可选） |
| `so3_quadrotor_simulator/launch/simulator.launch` | 仿真器 | ① 手动 | **在**（单机仿真器） |
| `so3_quadrotor_simulator/launch/multi_simulator.launch` | 仿真器 | ② 子 launch include | **在**（集群仿真器） |
| `phase_offset_swarm/launch/agent_state_neighbor_runtime.launch` | 集群数据面 | 测试；实机部署会用到 | **在**（实机要单起） |

补充：`phase_offset_sim_bringup` 与 `phase_offset_swarm` 这两个包在 CMake 里注册了
rostest（`sim_b_bringup.test`、`sim_c_bringup.test`、`swarm_intent_shadow.test`、
`agent_state_neighbor_transport.test`），也就是 A 类里的 bringup 结构本身是被
自动化测试锁住的；`bspline_traj` 则一个 `add_rostest` 都没有。

### B. 历史阶段资产 —— **已删除（2026-09-12）**，下表仅作阶段索引

| launch | 阶段 | 谁驱动它 | 现在用吗 |
| --- | --- | --- | --- |
| `b1_independent_agent.launch` | B1 | `b1_independent_navigation_test.py` | 不用 |
| `b1_pillar_independent_3.launch` | B1 | 同上 | 不用 |
| `b1_pillar_single_baseline.launch` | B1 | 同上 | 不用 |
| `b1e_original_global_manual_on.launch` | B1e 等价性 | `b1e_e0 .test` | 不用 |
| `b1e_original_global_manual_off.launch` | B1e 等价性 | `b1e_e1 .test` | 不用 |
| `b1e_namespaced_single_standalone_original_gains.launch` | B1e | `b1e_e2 .test` | 不用 |
| `b1e_namespaced_single_manager_original_gains.launch` | B1e | `b1e_e3 .test` | 不用 |
| `b1e_namespaced_single_manager_world_frame_original_gains.launch` | B1e | `b1e_e4 .test` | 不用 |
| `b1e_namespaced_single_manager_world_hover_original_gains.launch` | B1e | `b1e_e5 .test` | 不用 |
| `b1e_namespaced_multi_manager_original_gains.launch` | B1e | `b1e_e6 .test` | 不用 |
| `b1e_namespaced_multi_manager_original_gains_b1_so3_params.launch` | B1e | `b1e_e7 .test` | 不用 |
| `phase_offset_swarm_sim.launch` | 批次 swarm 总入口 | `batch2…batch10` 脚本 | 不用 |
| `phase_offset_swarm_3.launch` | 批次（3 机薄包装） | 手动 / 批次脚本 | 不用 |
| `phase_offset_swarm_7.launch` | 批次（7 机薄包装） | 手动 / 批次脚本 | 不用 |
| `phase_offset_single_sim.launch` | 批次单机入口 | `batch*` 脚本 | 不用 |
| `phase_offset_pillar_left_to_right_3.launch` | 批次场景 | 手动 | 不用 |
| `phase_offset_record.launch` | 批次录包 | 手动 | 不用 |
| `bspline_traj/launch/phase_offset_agent.launch` | 批次逐机（**与现役同名不同文件**） | `phase_offset_swarm_sim` / `phase_offset_single_sim` include | 不用 |
| `phase_offset_shadow_single.launch` | M2/M3 shadow | 手动 | 不用 |
| `phase_offset_active_zero_single.launch` | M3 zero-port | 手动 | 不用 |
| `phase_offset_manual_single.launch` | M3 manual-port | 手动 | 不用 |
| `phase_offset_esdf_tube_single.launch` | M4 tube 证据（`observe_only=true`） | 手动；docs 引用 51 处 | 不用（保留作证据） |
| `phase_offset_fixed_tube_single.launch` | M4 tube 证据 | 手动；docs 引用 6 处 | 不用（保留作证据） |
| `phase_offset_swarm/launch/swarm_intent_shadow.launch` | 集群 intent 影子（手动版） | 手动；注册的测试走 `test/swarm_intent_shadow.test`（节点定义在 .test 里，不 include 本文件） | 不用（调试备用） |

说明：B 类里的单机 M4 入口全部是 `observe_only=true`——**它们从来没有开过集群**，
只是用来观察 tube 几何；tube 参数现已并入 `test_gvf.launch` 默认值。

### C. 上游自带 / 与本课题无关

| launch | 归属 | 现在用吗 |
| --- | --- | --- |
| `uav_simulator/mockamap/launch/mockamap.launch` | 上游 mockamap | 不用 |
| `uav_simulator/mockamap/launch/maze2d.launch` | 上游 mockamap | 不用 |
| `uav_simulator/mockamap/launch/maze3d.launch` | 上游 mockamap | 不用 |
| `uav_simulator/mockamap/launch/perlin3d.launch` | 上游 mockamap | 不用 |
| `uav_simulator/mockamap/launch/post2d.launch` | 上游 mockamap | 不用 |
| `uav_simulator/dynamic_map_generator/launch/draw_map.launch` | 上游地图工具 | 不用 |
| `uav_simulator/dynamic_map_generator/launch/draw_map_3d.launch` | 上游地图工具 | 不用 |
| `uav_simulator/dynamic_map_generator/launch/rviz.launch` | 上游地图工具 | 不用 |
| `uav_simulator/Utils/drone_control/launch/px4ctrl.launch` | 上游 PX4 控制 | 不用 |
| `uav_simulator/so3_quadrotor_simulator/launch/run_rviz.launch` | 上游 RViz | 备用 |

---

## 3. 四个容易踩的点

1. **重名文件**：`phase_offset_agent.launch` 有两份——
   `bspline_traj/launch/`（历史）与 `phase_offset_sim_bringup/launch/`（现役）。
   只有后者是当前链路，按文件名搜索时会被混在一起。
2. **未注册 rostest**：`bspline_traj/CMakeLists.txt` 只有 `catkin_add_gtest`，
   没有任何 `add_rostest`。所以 `b1e_*.test` 与 `batch*.py` 全部是手动回归，
   `catkin_make run_tests` 不会碰它们——这也是它们"藏着但没坏"的原因。
3. **实机集群 launch 目前不存在**。25 个 `bspline_traj/launch` 里没有一个是
   实机集群入口；需要新建（实机单机链 + 集群定稿参数 + 邻居 runtime + UDP 桥 + 0.6 偏置）。
4. **删除需要授权**。`AGENTS.md` 要求 preserve 原链路；把 B/C 类归档属于一次
   需要 execution spec 的动作，且 `phase_offset_esdf_tube_single.launch` 在 docs 里
   被引用 51 次，直接删会让 M4 报告对不上号。建议归档到 `docs/legacy_launches/`
   并保留本索引，而不是删除。

---

## 4. 与实机部署的关系

实机要用的三个文件，全部在 A 类里：

| 用途 | 文件 |
| --- | --- |
| 逐机规划核心 | `bspline_traj/launch/test_gvf.launch` |
| 实机单机模板（要改成实机集群） | `bspline_traj/launch/gvf.launch` |
| 邻居 runtime | `phase_offset_swarm/launch/agent_state_neighbor_runtime.launch` |

外加当前尚缺的一项：UDP 邻居桥（用于把机上 via-UDP 的邻居状态灌进
`/phase_offset/agent_state`）。这项本轮只做了勘察与结论，**尚未落代码**。
