# Luna Max Handoff: PhaseOffsetSwarm Swarm Intent V1

Date: 2026-08-20

Workspace:

```text
/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
```

## 1. New-conversation objective

Implement and verify the isolated PhaseOffsetSwarm **Swarm Intent V1** stage, then continue to a ROS1 shadow wrapper that does not connect to the UAV control chain.

The stage computes only:

\[
(id_j,t_j,x_j,v_j),\ \beta_i
\longrightarrow
g_i^{coord},
\]

where:

\[
g_i^{coord}
=
g_i^{sep}
+\beta_i g_i^{coh,0}
+g_i^{conf}.
\]

It must not implement the phase-offset allocator, CBF, matched port, Tube internals, emergency controller, priority protocol, or any final UAV command.

## 2. User decisions already made

The user has explicitly confirmed:

1. The attached/current task specification is valid authorization for an isolated B2/B3 Swarm Intent stage even though production B0-B1 multi-instance simulation is not yet complete.
2. After the pure C++ library and tests pass, continue with a ROS1 shadow wrapper that only publishes/logs the nominal intent and does not connect to the control chain.
3. The executing Luna Max agent must work alone and must **not spawn sub-agents**.

Do not ask these questions again unless the workspace state contradicts the recorded preconditions.

## 3. Mandatory reading order

Before any code edit, read these files completely in this order:

1. Repository rules:

   ```text
   /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/AGENTS.md
   ```

2. Frozen detailed execution specification:

   ```text
   /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/docs/Codex_PhaseOffsetSwarm_Swarm_Intent_V1_Execution_Spec_2026-08-20.md
   ```

3. Architecture reference, especially the `phase_offset_msgs` and `phase_offset_swarm` boundaries:

   ```text
   /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/docs/PhaseOffsetSwarm_Code_Architecture_2026-08-08.md
   ```

4. Current proposal only for terminology and formulas; do not edit it:

   ```text
   /home/cxq/ISF-GVF/Paper/ICRA/PhaseOffsetSwarm_Latest_Detailed_Proposal_2026-08-20.md
   ```

5. Historical reference-only implementation, read-only and never connected directly to CMake:

   ```text
   /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/src/swarm_planner/bspline_traj/include/bspline_race/swarm_neighbor_model.h
   /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/src/swarm_planner/bspline_traj/src/swarm_neighbor_model.cpp
   /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/src/swarm_planner/bspline_traj/test/swarm_neighbor_model_test.cpp
   ```

The detailed execution specification is authoritative for file layout, interfaces, formulas, tests, build order, audits, and stop conditions. Do not substitute a shorter personal plan.

## 4. Current workspace state

At handoff preparation time:

```text
branch: main
HEAD: 9a0e97560b8ddf15ac1ed1581030ed4b911f6e43
```

The worktree is intentionally dirty. Another Codex/development process is modifying single-UAV Tube V2 and related files. All existing modifications and untracked files are user-owned.

No Swarm Intent production code has been written by the handing-off agent. Only planning/handoff documentation was added.

Before editing, Luna Max must record:

```bash
git branch --show-current
git rev-parse HEAD
git status --short
git diff --stat
```

If branch or `HEAD` differs, stop and report. If unrelated files continue changing, preserve them and restrict work to the authorized whitelist.

## 5. Authorized file whitelist

Luna Max may create or modify only:

```text
/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/src/swarm_planner/phase_offset/phase_offset_msgs/**
/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/src/swarm_planner/phase_offset/phase_offset_swarm/**
/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/docs/Codex_PhaseOffsetSwarm_Swarm_Intent_V1_Self_Audit_2026-08-20.md
```

Do not modify this handoff document or the frozen execution specification during implementation.

Explicitly forbidden existing paths include:

```text
phase_offset_core/**
phase_offset_navigation/**
bspline_traj/**
common_msgs/**
plan_env/**
uav_simulator/**
```

No proposal edits are authorized.

## 6. Frozen communication contract

High-frequency inter-UAV algorithm information is exactly:

```text
robot_id
timestamp
position_world
velocity_world
```

The ROS message must be:

```text
std_msgs/Header header
uint16 robot_id
geometry_msgs/Point position_world
geometry_msgs/Vector3 velocity_world
```

Do not add session, custom sequence, control mode, phase, offset, Tube, neighbor beta, intent, QP result, branch event, ETA, lease, or priority fields.

`beta_preview` is local-only input with default value `1.0`. It is not broadcast. V1 uses:

\[
g_i^{coh}=\beta_i g_i^{coh,0},
\]

not `min(beta_i,beta_j)`.

## 7. Required execution sequence

Follow the execution specification exactly. The condensed sequence is:

1. Record preconditions and verify package paths do not contain conflicting user work.
2. Create the pure C++14/Eigen `phase_offset_swarm_core` library and public contracts.
3. Implement Neighbor Manager with timestamp ordering, constant-velocity prediction, FRESH/STALE/LOST, self filtering, retention, and distance hysteresis.
4. Implement bounded C2 separation and weak cohesion using only local `beta_preview`.
5. Implement selective radial conflict friction using distance rate, closing speed, and TTC; never implement full velocity consensus.
6. Aggregate components and apply one final vector-norm saturation only.
7. Run and pass all pure C++ tests. This is a hard gate.
8. Create `phase_offset_msgs/AgentState.msg` and the ROS1 shadow node.
9. The shadow node may subscribe to local odometry and shared AgentState, then publish `g_coord`, `g_sep`, `g_coh`, and `g_conf` for visualization/logging only.
10. Add and run the ROS shadow test.
11. Run dependency searches, regression tests, `git diff --check`, whitelist verification, and final status capture.
12. Write the required self-audit and stop. Do not continue to allocator/CBF/control integration.

## 8. Required mathematical invariants

Use the exact formulas and kernels in the detailed execution specification. At minimum preserve:

- `n_ij=(predicted_position_j-position_i)/d_ij`;
- separation has a negative sign and points away from a close neighbor;
- cohesion has a positive sign and points toward a distant organization neighbor;
- separation is never multiplied by beta;
- cohesion uses only local beta;
- `dot_d=(v_j-v_i)^T n_ij`;
- `closing=max(0,-dot_d)`;
- conflict friction is `k_conf * omega * dot_d * n_ij`;
- approaching pairs are opposed;
- separating pairs and common translation produce zero friction;
- zero distance uses the deterministic ID-based axis specified in the execution plan;
- components are logged before saturation;
- only the complete `g_coord` sum is saturated;
- no output contains NaN or Inf except the permitted infinity sentinels for unavailable diagnostic minima.

## 9. ROS shadow boundary

The node is allowed to publish only diagnostic/shadow information. It must not publish or call:

```text
PositionCommand
velocity command
u_w
u_delta
phase rate
offset rate
allocator result
CBF result
matched-port command
governor/SO3 command
```

It must not include or depend on `phase_offset_navigation`, `gvf_manager`, Tube classes, CBF, or allocator code.

## 10. Completion evidence

The final response and self-audit must include:

- exact files created/modified;
- pure C++ build/test commands and results;
- ROS message/node build results;
- ROS shadow test result;
- `phase_offset_core` regression result;
- dependency-boundary search results;
- confirmation of the four-field communication contract;
- confirmation that `beta_preview` remains local-only;
- proof no control topic is published;
- `git diff --check` result;
- final `git status --short` and whitelist comparison;
- explicit stop before allocator/CBF/control integration.

Do not create commits, branches, tags, stashes, or pushes. Do not stage files.

## 11. Prompt to paste into the new Luna Max conversation

```text
请在 /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws 中执行 PhaseOffsetSwarm Swarm Intent V1。

先完整阅读：
1. /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/AGENTS.md
2. /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/docs/Luna_Max_PhaseOffsetSwarm_Swarm_Intent_V1_Handoff_2026-08-20.md
3. /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/docs/Codex_PhaseOffsetSwarm_Swarm_Intent_V1_Execution_Spec_2026-08-20.md
4. 交接文档列出的架构和参考文件。

用户已明确授权：即使 production B0-B1 尚未完成，也允许先实现完全隔离的 Swarm Intent；纯 C++ library/tests 全部通过后，继续实现不接控制链的 ROS1 shadow wrapper。

严格按详细 execution spec 执行。你必须独立完成，不要开启或委派任何子代理。只修改白名单内文件，保护当前并发 Tube V2 和所有用户已有改动。完成构建、测试、依赖边界检查、自审文档后停止；不要进入 allocator、CBF、matched port 或控制链集成。
```
