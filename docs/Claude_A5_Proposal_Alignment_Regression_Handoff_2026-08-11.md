# Claude 审查交接：A5 Phase-Offset Tube 与详细 Proposal 对齐、回归和回退决策

> 日期：2026-08-11  
> 仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
> 审查性质：先只读诊断，不修改代码  
> 用户当前判断：A5 可能再次被改坏，tube、重规划、相位运行和 UAV 偶发卡住同时出现异常  
> 当前未完成任务：A5-G2h 已中断，执行者确认尚未修改任何文件

## 0. 给 Claude 的直接任务

请完整审查当前仓库的 A5 实现，并回答：

1. 当前 A5 是否仍然符合详细 proposal 中的 phase-offset tube 定义和数据流？
2. 哪些代码是正确且应保留的，哪些只是为修复局部症状不断叠加的状态、门控和证书？
3. 当前 ESDF Candidate tube 几何是否已经正确，而真正失败的是 Runtime、Active ownership、端口选择、A6 continuation 或原 governor/C2？
4. UAV 偶发卡住、粉色参考点明显落后/领先、重规划成功但路径不安装，分别来自哪里？
5. 应该继续修 A5-G2h，先做 A6，还是选择性回退 A5 Runtime，仅保留可靠的 tube 几何？
6. 给出最小、可验证、不会继续增加门控的修复/回退方案。

第一轮只读。不要直接实施修复，不要修改参数来让 tube 变宽，不要新增状态机，不要把 A5 与 A6 混成一次大改。先给出证据、proposal 对齐矩阵和明确的保留/撤回清单。

## 1. 仓库与 Git 安全边界

可信基线：

```text
branch: main
HEAD: 9a0e97560b8ddf15ac1ed1581030ed4b911f6e43
trusted baseline label: 9a0e975
prototype stash: deepseek-phaseoffset-tracked-prototype-2026-08-08
```

必须遵守根目录 `AGENTS.md`：

- 所有现有改动均视为用户所有；
- 禁止 `git reset --hard`、`git restore .`、`git checkout -- .`、`git clean`；
- 禁止 pop/restore 整个 prototype stash；
- 禁止 commit、branch、tag、push；
- 没有新的执行规范时只允许只读审查；
- A5、A6、planner、swarm 必须保持阶段边界。

当前 tracked diff 相对可信 HEAD 涉及 11 个文件，约 `+691/-35`：

```text
src/swarm_planner/bspline_traj/CMakeLists.txt
src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
src/swarm_planner/bspline_traj/launch/test_gvf.launch
src/swarm_planner/bspline_traj/package.xml
src/swarm_planner/bspline_traj/src/gvf.cpp
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/test/gvf_lifted_visualization_test.cpp
src/swarm_planner/plan_env/CMakeLists.txt
src/swarm_planner/plan_env/include/plan_env/sdf_map.h
src/swarm_planner/plan_env/src/sdf_map.cpp
src/uav_simulator/so3_quadrotor_simulator/config/swarm_rviz.rviz
```

大量 phase-offset 文件仍是 untracked；不能因为 untracked 就视为可删除。

## 2. 必须先完整读取的资料

### 2.1 最高优先级：详细研究 Proposal

完整 3079 行原文：

```text
/home/cxq/.codex/attachments/42b890f7-a680-4a8d-8149-cfb2e9b41766/pasted-text.txt
```

标题：

```text
PhaseOffsetSwarm：面向异步增量路径管道的分布式多 UAV 集群协调
新版详细研究方案，2026-08-05
```

不要只读本交接的摘要；必须完整读取原文。

### 2.2 当前单机实施路线和代码架构

- `docs/PhaseOffsetSwarm_Single_First_Implementation_Plan_2026-08-07.md`
- `docs/PhaseOffsetSwarm_Code_Architecture_2026-08-08.md`

注意：主路线文档经过多轮局部改写，顶部阶段 metadata 与文末“当前只允许 A0”存在历史不一致。不要仅凭 metadata 判断当前真实阶段，应以当前代码、执行单和原始证据为准。

### 2.3 既有 Claude 理论审查

详细 proposal 的理论修订意见：

```text
/home/cxq/.codex/attachments/88ecc13d-5c4c-43ba-b0d1-b4dd3a5a97ee/pasted-text.txt
```

其中与当前 A5 最相关的是：

- 必须区分正速度安全可行集 `U+` 与允许零速的安全可行集 `U_safe`；
- fixed tube epoch 内边界至少分段 C1 且斜率有界；
- “当前横截面安全”不自动保证至少存在一个安全控制端口；
- 普通加权 QP 不保证严格的“先 offset、后 phase”优先级。

### 2.4 既有 Claude 对旧 A5 的实现审查

```text
/home/cxq/.codex/attachments/ef71c979-9a92-4cfe-b49e-04472b0e2b1b/pasted-text.txt
```

它指出旧 A5 的四个结构性偏差：

- 环境 tube 被 `max_offset=0.20` 人为钳死；
- inflated ESDF 后重复 erosion；
- `delta=0` 不安全被错误等价为横截面为空；
- global interval fallback 让一个窄点压窄整段 preview。

后续 G1–G2g 声称已修复这些几何问题，但需要重新核查实际源码，不要只采信汇报。

### 2.5 当前 A5-G2g 与未执行 G2h

- `docs/Terra_A5_G2g_PointCloud2_Continuous_Tube_Geometry_Prompt_2026-08-10.md`
- `docs/Terra_A5_G2h_Live_Exact_Port_Viability_And_Emergency_Semantics_Prompt_2026-08-10.md`

G2h 当前状态：执行者只完成只读设计和旧 bag 分类，尚未写入任何文件，现已中断。因此当前源码是 G2g 完成后的状态，不包含 G2h selector。

## 3. 详细 Proposal 的权威语义

### 3.1 总体数据流

Proposal 的目标不是“给路径画一个带很多显示门控的 ribbon”，而是：

```text
本机局部感知 / 局部占据地图或 ESDF
        ↓
本机 A* + B-spline 规划得到基础中心线 p(w)
        ↓
现有 C2 connector 形成 ContinuousPhasePath
        ↓
在每个 w 求 lifted differential state: p, p_w, p_ww
        ↓
求 T, N, N_w, curvature
        ↓
沿 N 构造局部非对称鲁棒横截面 [delta_lower(w), delta_upper(w)]
        ↓
沿 w 保守平滑，形成局部变化、连续的 tube surface
        ↓
候选 tube epoch 的当前安全和前视端口可行性检查
        ↓
安装 Active tube，保持 w 和 delta
        ↓
端口分配 / 投影得到同一个 final (u_w, u_delta)
        ↓
同一 final port 同时进入物理 matched 前馈、w_dot 和 delta_dot
        ↓
原 governor / SO3
```

Tube 不是另一条 B-spline，不需要第二套 tube C2 connector。Tube 应建立在规划出的基础轨迹及其 lifted state 上，再从一维中心线扩展成随 `w` 变化的二维带状安全集合。

### 3.2 基础路径与活动参考必须分开

基础中心线：

\[
p(w).
\]

活动参考：

\[
r(w,\delta)=p(w)+N(w)\delta.
\]

真实 UAV：

\[
x=r+e.
\]

因此：

- `delta` 是活动参考的横向内部坐标，不是 UAV 的真实横向位置；
- tube 约束的是鲁棒活动参考可用区域；
- 真实 UAV 安全还必须覆盖 tracking/定位/地图不确定性；
- tracking error 不能与 tube offset 混为同一状态。

### 3.3 2.5D 几何是批准后的 Proposal 修订

详细 proposal 原文按固定高度二维写：

\[
N=R_{\pi/2}T,
\qquad
r_w=v_p(1-\kappa\delta)T.
\]

但 A2.1 已证明真实规划路径存在约 `0.451 m` 高度变化。因此当前批准实现是重力参考 2.5D：

\[
e_z=(0,0,1)^T,
\qquad
N=\frac{e_z\times p_w}{\|e_z\times p_w\|},
\]

\[
N_w=
\frac{(I-NN^T)(e_z\times p_{ww})}{\|e_z\times p_w\|},
\]

\[
r=p+N\delta,
\qquad
r_w=p_w+N_w\delta.
\]

Claude 不应机械恢复固定高度公式；应把 2.5D 看作详细 proposal 的已批准保守推广。

### 3.4 Proposal 中的环境 tube

沿 `+N`、`-N` 从路径点到物理障碍边界的距离分别为：

\[
c^+(w),\qquad c^-(w).
\]

鲁棒半径：

\[
r_{eff}=r_{uav}+e_{map}+e_{loc}+e_{track}.
\]

障碍约束给出：

\[
\overline\delta^{obs}(w)=c^+(w)-r_{eff},
\]

\[
\underline\delta^{obs}(w)=-c^-(w)+r_{eff}.
\]

再与曲率正则区间相交：

\[
[\underline\delta,\overline\delta]
=
[\underline\delta^{obs},\overline\delta^{obs}]
\cap
[\underline\delta^{curv},\overline\delta^{curv}].
\]

关键语义：

- 左右边界独立，允许非对称；
- 环境 tube 不应被 controller 的 `delta_ctrl_max` 冒充；
- `interval_nonempty`、`contains_zero`、`contains_current_delta` 是三个不同事实；
- 合法横截面可以不包含零；
- 单侧安全 tube 不等于自动命令普通单机侧移；
- 中心线失去鲁棒安全时应触发中心线重规划，但不能把仍存在的单侧几何 corridor 谎报为空。

### 3.5 Tube boundary 必须保留局部函数

Proposal 需要：

\[
\underline\delta=\underline\delta(w),
\qquad
\overline\delta=\overline\delta(w),
\]

并在固定 tube epoch 内至少分段 C1、斜率有界。不能用整个 preview 的 global intersection 替代局部边界函数。

开阔—窄区—开阔应该显示局部收缩后重新展开，而不是整段都被最窄点压平。

### 3.6 Tube epoch hybrid update

新地图 observation 产生 Candidate tube。安装新 tube 时应保持：

\[
w^+=w^-,
\qquad
\delta^+=\delta^-.
\]

应区分：

1. `U+ != empty`：安装，rolling；
2. `U+ == empty` 且 `U_safe != empty`：安装最新安全约束，safety-priority，可允许 `w_dot -> 0`；
3. `U_safe == empty`：emergency/failsafe；
4. 当前地图信息不确定：fail-closed，但不应谎报“已证明 unsafe”；
5. 路径 revision 不连续：属于 A6 continuation，不应用 A5 gate 掩盖。

### 3.7 Matched port 是核心不变量

最终系统：

\[
\dot x=f_x^{ISF}+r_wu_w+Nu_\delta,
\]

\[
\dot w=f_w^{ISF}+u_w,
\qquad
\dot\delta=u_\delta.
\]

必须使用同一个 final port，才能得到：

\[
\dot e=-K_2q e_\perp-K_1\sigma(e_\parallel)\tau.
\]

任何幅值、slew、tube 或安全投影都可以改变 raw intent，但物理端与内部端不能使用不同的结果。

### 3.8 A6 continuation 的权威语义

基础路径接受新 revision 时应在 retained phase 匹配：

\[
p^+=p^-,\quad p_w^+=p_w^-,\quad p_{ww}^+=p_{ww}^-.
\]

并保持：

\[
w^+=w^-,\qquad\delta^+=\delta^-.
\]

由此得到：

\[
r^+=r^-,\qquad r_w^+=r_w^-,\qquad e^+=e^-.
\]

若当前有限前端、authoritative phase 和新路径安装不能保证这一点，A5 的 tracking、Active ownership、Certified Marker 和端口可行性都会被污染。

## 4. 当前源码实际数据流

### 4.1 当前模块

`phase_offset_core`：

- geometry；
- matched port；
- PortProjector。

`phase_offset_navigation`：

- `TubeCrossSectionSolver`；
- `TubeBuilder`；
- `TubeFilter`；
- `TubeSurfaceValidator`；
- `TubeEpochManager`；
- `TubeDynamicFeasibility`；
- `PhaseOffsetRuntime`。

`bspline_race integration`：

- PointCloud2/SDFMap snapshot bridge；
- Candidate/Active profile ownership bridge；
- Runtime 调用；
- Marker 与多个 Float64MultiArray diagnostics。

### 4.2 当前 ESDF/cloud Candidate 路径

当前 G2g 路径是：

```text
/sim/local_map PointCloud2
        ↓
SDFMap::cloudCallback
        ↓
build immutable CloudOccupancySnapshot
        ↓
adapter gets one snapshot and one ContinuousPhasePath evaluator
        ↓
TubeBuilder::buildCloudClearance
        ↓
adaptive lifted-state sampling
        ↓
center-clearance cross sections
        ↓
TubeFilter local C1 profile
        ↓
TubeSurfaceValidator continuous ribbon check
        ↓
Candidate profile
        ↓
TubeEpochManager decides Active ownership
        ↓
Runtime performs live current/next port check
```

这比旧“两个法向 ray 距离减半径”更接近 proposal。

### 4.3 当前 Marker

- `/formation_planning/phase_offset_manual/tube_candidate`：Candidate 几何，UNCERTIFIED；
- `/formation_planning/phase_offset_manual/tube`：Active 且当前执行条件满足的 Certified；
- `/formation_planning/phase_offset_manual/frame`：粉色球，位置为当前活动参考 `r(w,delta)`；
- 彩虹曲线 `/particle0/gvf/traj_vis`：当前有限 GVF frontend。

Candidate Marker 主要回答“最新环境/路径几何构造出了什么 tube”。Certified Marker 还混合了 Active ownership、current validation、tracking、exact port、gate/failure 等执行条件。用户看到 Certified 时有时无，不能直接推断 Candidate 几何本身时有时无。

## 5. 已经有较强证据支持、但范围有限的部分

G2g 自审核：

```text
/tmp/a5_g2g_acceptance_20260810/SELF_AUDIT.md
```

关键结果：

- build 与 completed-stage tests 通过；
- `666 tests, 0 failures/errors`；
- 41/41 due Candidate ADD frame 通过独立 snapshot surface audit；
- occupied-voxel-volume 最小 clearance `0.46688701787509995 m`；
- residual requirement `0.45 m`；
- 旧实现反例 `0.16529574541714323 m`；
- 新 raw point-to-ribbon 最小值 `0.6433672008824303 m`。

因此可暂时认为：

> 在那一次特定 local_sensing、pillar、显式 snapshot 参数和离线审计条件下，G2g Candidate ribbon 没有穿入其声明的 occupied voxel volume。

不能由此推出：

- A5 完整闭环正常；
- 当前 dedicated ESDF launch 等价于验收配置；
- 实际深度融合 PointCloud2 满足 complete observed-domain contract；
- A6 continuation 正常；
- Certified tube 应稳定显示；
- UAV 不会 HOLD；
- Runtime emergency 语义正确。

## 6. 当前高优先级异常和 Proposal 偏差

### P0-1：G2g 几何通过，但 A5 闭环实际上几乎没有运行

同一正式 ROS-2 中：

```text
epoch frames: 172
Candidate ADD/DELETE: 167/5
Certified ADD/DELETE: 2/170
Runtime emergency: 117
selected: 2
fatal/latch: 0
retained outside: 118
```

证据：

```text
/tmp/a5_g2g_acceptance_20260810/esdf/evidence/esdf_summary.json
```

这意味着 Candidate tube 几何常常存在，但 Active/Runtime/Certified 控制链几乎不可用。不能把 G2g 标记为“ESDF A5 工作正常”。

### P0-2：专用 ESDF launch 与正式 G2g 验收配置不一致

G2g 正式验收实际命令包含：

```text
phase_offset_tube_cloud_obstacle_set_complete:=true
phase_offset_tube_preincluded_map_uncertainty:=0.10
```

证据日志：

```text
/tmp/a5_g2g_acceptance_20260810/esdf/ros_home/log/
  1139e9d0-94c3-11f1-87cc-09f015e6f70e/
  roslaunch-cxq-B530E-WXXXX-2564075.log
```

但当前：

```text
src/swarm_planner/bspline_traj/launch/phase_offset_esdf_tube_single.launch
```

没有向 `test_gvf.launch` 传这两个参数，因此使用默认：

```text
cloud_obstacle_set_complete=false
preincluded_map_uncertainty=0.0
```

用户运行专用 ESDF launch 与所谓正式验收不是同一系统。请判断这是 launch 漏接、故意 fail-closed，还是 sim/real contract 没有被正确分层。

### P0-3：实际 PointCloud2 contract 仍需重新证明

当前 snapshot 只有在：

```text
cloud_obstacle_set_complete=true
```

时，才把 observed AABB 内未占据 voxel 当作 known free。

这对模拟 `local_sensing` 的完整局部障碍集合可能成立，但对真实“深度图融合后的局部占据障碍云”不自动成立。PointCloud2 类型相同不代表 free-space 语义相同。

Claude 必须追踪真实管道：

```text
depth image
  -> ray integration / fusion
  -> local occupied obstacle PointCloud2
  -> whether absence inside AABB proves free
  -> snapshot known/unknown contract
```

如果 fused cloud 只发布 occupied hits，而不提供 observed-free domain，当前 `complete=true` 可能产生不成立的 free-space certificate。

### P0-4：当前 Runtime 把一个 projected point 失败误写为整个端口集合为空

当前 `PhaseOffsetRuntime::complete()`：

1. 调用 `PortProjector::project()`，得到离 raw intent 最近的一个点；
2. 用该点计算 exact next `(w+,delta+)`；
3. 若该点不在 next bounds，返回 false；
4. U+ 和 U>=0 两次都失败时写：

```text
current U+ and U>=0 port sets are empty
```

但一个 nearest point 失败不证明整个二维 feasible polygon 与 nonlinear exact-next envelope 的交集为空。

这是 G2h 想修的核心，但 G2h 尚未实施。请判断是否需要完整 `ExactTubePortSelector`，还是可以更简单地把 exact next constraint 直接纳入唯一 PortProjector，避免继续增加模块和 diagnostics schema。

### P0-5：A6 revision/continuation 未完成，却已污染 A5 验收

G2g 数据中：

```text
candidate/active path revision mismatch frames: 54
emergency with revision mismatch: 42
```

用户还观察到：

- 粉色参考点从 UAV 前方变到后方；
- 重规划日志显示 Kino success，但新 frontend 可能不安装；
- UAV 偶发卡住；
- tube/Certified 在换路附近变化。

这些不能只归因于 tube gate。A6 必须验证：

- accepted path revision 的 `p,p_w,p_ww` continuity；
- `w` 不越出旧 frontend；
- `delta` 不重置；
- Candidate/Active profile 的 path ownership 原子切换；
- 切换处 `r,r_w,e` 无 jump。

### P0-6：原 manager 在 governor HOLD 前已经提交 phase

当前 `gvf_manager.cpp` 的顺序是：

1. 根据 `out.w_dot` 计算并提交新的 `phase_w_`；
2. 之后运行 velocity-matching governor；
3. governor 无有效候选时 `cmd_pos=当前 UAV 位置`，即 `GOVERNOR_INVALID_HOLD`。

这会造成 UAV 不动但 phase 已前进。闭合有限前端尤其可能越过 `path_end_w`，随后 C2 在旧路径上按越界 `phase_at_switch` 查询失败，形成：

```text
old frontend near/end
 -> phase keeps advancing
 -> governor candidates invalid / HOLD
 -> Kino replan succeeds
 -> C2 old-state query fails or connector unavailable
 -> new path not installed
 -> keep old frontend
 -> repeated HOLD/replan
```

A5 diff 没有改 C2 主体，但 A5 selected guidance 会改变 `w_dot` 和触发时序，因此可能更容易暴露该潜在问题。请明确这属于 A6/base control fix，而不是用更多 A5 gate 掩盖。

### P1-1：A5 验证 launch 把 tube、manual sinusoidal offset 和 100-cycle gate 混在一起

`phase_offset_fixed_tube_single.launch` 和 `phase_offset_esdf_tube_single.launch` 使用：

```text
phase_offset_mode=manual
manual_amplitude=0.10
profile_period=2.0
warmup_cycles=100
```

用户期望普通单机 `delta=0`；非零 offset 应仅用于明确的 A4 manual test 或未来 swarm intent。当前 launch 同时承担：

- tube 几何显示；
- manual offset profile；
- A3 zero-equivalence arming；
- Active/Certified control。

这使“我只是想看 tube”与“我要启动非零 matched control”无法清楚区分。请评估是否应保留一个纯 Candidate/diagnostic 的 tube launch，并把 manual active 控制作为独立测试入口。

### P1-2：Candidate 几何和 Certified 执行证书虽已分 topic，但用户语义仍不清楚

当前 Candidate Marker 只依赖几何 profile；Certified Marker 还要求：

- Active profile ownership；
- current geometry/bounds；
- retained delta inside；
- tracking within bound；
- current map safe；
- live projected/matched port valid；
- no emergency/fatal/latch。

所以 Certified 闪烁不等于 tube 几何消失。但 UI 名称和用户运行流程没有充分解释这一点。

请判断论文和系统真正需要哪几层可视化：

1. environment Candidate tube；
2. installed Active tube；
3. current executable/control certificate。

不要再让一个 Marker topic承担所有含义。

### P1-3：当前 state/mode/diagnostics 数量明显膨胀

当前 navigation 中同时存在：

- `TubeEpochState` 7 种；
- `RuntimeExecutionMode` 9 种；
- `CurrentSafetyStatus`；
- `TubeInstallDisposition`；
- `TubeEpochReason` 17 种；
- `ControlFailureReason`；
- dynamic feasibility class/reason/witness；
- legacy 83 字段、epoch 60 字段、raw 49 字段、cloud 18 字段 diagnostics；
- Candidate/Active/Certified/failure latch/100-cycle gate。

其中一些是必要的事实分类，但当前组合已超出用户可理解范围，并且没有换来稳定闭环。

请审查每个状态是否满足以下标准：

- 是否改变控制动作？
- 是否是不可由其他事实派生的权威状态？
- 是否属于 proposal 中的 hybrid mode？
- 是否只是 diagnostics label？
- 删除它是否改变安全性？

目标不是把所有状态压成一个 bool，而是删除重复 ownership、重复 gate 和重复证书。

### P1-4：代码规模偏离架构约束

当前主要文件行数：

```text
tube_builder.cpp               1076
tube_epoch_manager.cpp          727
port_projector.cpp              593
phase_offset_matched_adapter.cpp 727
tube_dynamic_feasibility.cpp    438
phase_offset_runtime.cpp        447
tube_filter.cpp                 357
tube_surface_validator.cpp      351
```

架构文档建议算法实现尽量低于约 500 行，adapter 只做转换和调用。请检查大文件是否包含重复的 legacy/raw/cloud 路径、diagnostics 拼装或多阶段兼容逻辑，是否应该选择性删除而不是继续拆出更多 helper。

### P1-5：legacy ESDF 和新 cloud clearance 两套语义并存

`TubeBuilderConfig` 仍保留：

- legacy `max_offset/ray_step/ErosionMargins`；
- 新 `TubeCrossSectionConfig`；
- fixed path；
- raw occupancy compatibility；
- cloud clearance path。

README 说明 legacy 路径只为兼容测试，但实际类型和 diagnostics 仍混合。请判断 production A5 是否应只有一个权威环境 tube construction path，避免后续维护者误接回旧逻辑。

### P1-6：通过单元测试不等于通过 operational acceptance

`666/666` 只能证明覆盖的断言通过。正式 ROS 中 `selected=2/172` 和 `emergency=117/172` 说明系统运行目标没有完成。审查时必须把：

- geometry safety；
- current execution viability；
- path continuation；
- actual task completion；
- visualization semantics；

分别验收，不能用总测试数替代。

## 7. 117 帧 Runtime emergency 的已知分类

G2g 正式 ESDF run 中：

```text
73 frames: WAITING / CURRENT_OFFSET_OUTSIDE
15 frames: explicit actual/reference map clearance unsafe
4 frames : FORWARD_HORIZON_SHORT
25 frames: revision-compatible, current-safe, dynamic infeasible
```

另有 42 个 emergency 与 path revision mismatch 重合。

正确理解：

- 73 帧不应通过自动改变普通单机 `delta=0` 来“修好”；
- 15 帧若地图证据正确，可能是真 emergency；
- 4 帧更像 observation horizon 不足，应是 waiting/replan；
- 25 帧才是 G2h exact-port/short-horizon viability 的核心候选；
- revision mismatch 必须交给 A6。

但请独立复算，不要直接采信这一分类。

## 8. 已知历史阶段与回退事实

### 8.1 不应恢复的 A5.2.3

A5.2.3 曾修改：

- Kino margin schedule；
- B-spline effective safe distance；
- planner clearance contract；
- C2/install hard validator；
- 48 字段 diagnostics。

这些修改被判断偏离 proposal、过度侵入 planner，已通过 A5-R0 选择性撤回。不要恢复。

### 8.2 A5.2.2 audit 已隔离

旧 clearance audit 文件可能仍保留为未跟踪参考，但已从 adapter、launch、CMake 和测试目标隔离。不要把旧 audit publisher 当成当前正式链路。

### 8.3 G2g 之后尚无 G2h 代码

当前 G2h 执行者状态：interrupted；确认零文件修改。不要寻找不存在的 exact selector 实现，也不要把 G2h prompt 当成已验收代码。

## 9. 建议 Claude 实际追踪的调用链

### 9.1 规划和 phase

```text
goal
 -> KinoAstar/B-spline
 -> point/closed phase-v2 frontend
 -> ContinuousPhasePath install/reject
 -> phase_w_ update
 -> calcLiftedGuidanceAtPhase
 -> velocity-matching governor
 -> PositionCommand
```

必须检查：

- `phase_w_` 在何时 commit；
- governor invalid/HOLD 时是否冻结；
- old path domain 和 `phase_at_switch`；
- rejected C2 后 retained frontend 是否仍可执行；
- RViz 彩虹 frontend 与实际 authority 是否一致。

### 9.2 A5 tube

```text
ContinuousPhasePathState
 -> ConvertContinuousPhasePathStateForActive
 -> preview/path_state_query
 -> cloud snapshot clearance query
 -> TubeBuilder::buildCloudClearance
 -> TubeFilter
 -> TubeSurfaceValidator
 -> TubeEpochManager::update/installActive
 -> PhaseOffsetRuntime::prepare/complete
 -> PortProjector
 -> MatchedPort
 -> manager selected override
```

### 9.3 地图

```text
local_sensing or actual depth fusion
 -> /sim/local_map or actual PointCloud2
 -> SDFMap::cloudCallback
 -> latest_cloud
 -> CloudOccupancySnapshot observed AABB/occupied voxels
 -> categorical/clearance query
```

确认：

- cloud 是否完整表达当前 local occupied set；
- AABB 内的缺失点是否可证明 free；
- one-voxel inflation 的真实几何语义；
- `preincluded_map_uncertainty=0.10` 是否与实现一致；
- manual/static/environment layers 是否进入同一 safety certificate；
- snapshot build 的 CPU/locking 是否影响 planner/control timing。

## 10. 建议的只读复现实验矩阵

先不要修改代码。相同地图、起点、目标至少比较：

### R0：原 baseline

```text
phase_offset_mode=disabled
phase_offset_manual_tube_source=none
```

目标：确认原 planner/C2/governor 是否仍会卡住。

### R1：manual/none

目标：确认 A4 matched port/gate 与 tube 无关时是否正常。

### R2：manual/fixed

目标：确认固定 tube 下 Runtime/PortProjector 是否稳定 selected，排除地图。

### R3：manual/ESDF Candidate-only 观察

保持普通单机 `delta=0`，不让 manual profile接管，只观察 Candidate 和 path revision。

### R4：正式 G2g 参数复现

当前正式 G2g 使用过：

```bash
roslaunch bspline_race test_gvf.launch \
  phase_offset_mode:=manual \
  phase_offset_manual_tube_source:=esdf \
  phase_offset_tube_cloud_obstacle_set_complete:=true \
  phase_offset_tube_preincluded_map_uncertainty:=0.10 \
  sdf_map_buffer_refresh_period:=3.0
```

不要误用当前 stale 的 `phase_offset_esdf_tube_single.launch` 作为等价证据。

每组必须保存：

- roslaunch 全日志；
- rosbag；
- path revision/C2 install events；
- governor command source/reason；
- PositionCommand 是否为当前位置 HOLD；
- Candidate、Active、Certified Marker actions；
- manual/epoch/cloud diagnostics；
- `phase_w_`、path start/end、reference `r`、actual UAV position。

## 11. 请 Claude 给出的最终交付格式

### 11.1 Findings-first 审查

按严重度列出：

```text
P0: 会破坏安全、基线、控制或 proposal 核心语义
P1: 会造成错误状态、闪烁、不可运行或错误验收
P2: 可维护性、diagnostics、launch 和文档问题
```

每条必须包含：

- 代码路径和行号；
- 实际行为；
- 对应 proposal 条款；
- 是否有 ROS/bag 证据；
- 建议保留、修改还是撤回；
- 属于 A5、A6 还是 baseline defect。

### 11.2 Proposal 对齐表

至少逐项评价：

| Proposal contract | 当前实现 | 结论 |
|---|---|---|
| planned path -> lifted state -> tube surface | | |
| asymmetric local bounds | | |
| margin accounting exactly once | | |
| interval/zero/current-delta separation | | |
| local C1 boundary function | | |
| Candidate vs Active epoch | | |
| U+ / U_safe / emergency | | |
| same final matched port | | |
| path/tube double continuation | | |
| original governor/SO3 preserved | | |
| simulated and real PointCloud2 contract | | |

### 11.3 最小决策方案

请在证据基础上选择并论证一种：

1. 保留 G2g geometry，最小修复 Runtime，然后做 A6；
2. 保留 G2g geometry，暂时退回 Candidate-only，先修 A6，再重新接 Runtime；
3. 选择性回退到更早的 A5-R1/R2 或 A5.2.1，再按 proposal 重建；
4. 证明当前结构可保留，但删除具体重复 gate/state。

不得只回答“继续调参数”或“再增加一个 gate”。

### 11.4 精确回退清单

如果建议回退，必须逐文件列出：

- 保留；
- 撤回；
- 暂时隔离；
- 重新实现；
- 不得触碰。

特别说明是否保留：

- `CloudOccupancySnapshot`；
- `TubeCrossSectionSolver`；
- `TubeSurfaceValidator`；
- `TubeFilter`；
- `TubeEpochManager`；
- `TubeDynamicFeasibility`；
- `PhaseOffsetRuntime`；
- Candidate/Certified marker helper；
- `gvf.cpp` Kernel 抽取；
- `gvf_manager.cpp` adapter 接入。

## 12. 当前建议的审查假设，不是预设结论

目前最符合证据的工作假设是：

1. G2g 的 Candidate continuous surface geometry 比早期 A5 正确，应优先保留；
2. A5 Runtime/epoch/port/Certified 语义仍然过复杂且 operationally failed；
3. 一部分 emergency 是真实地图/centerline mismatch；
4. 一部分 emergency 是单点投影造成的假空集；
5. 一部分异常来自 A6 revision/continuation；
6. UAV 卡住还涉及原 manager 的 phase commit 与 governor HOLD 顺序；
7. 当前专用 ESDF launch 没有复现正式 G2g 参数；
8. 实际深度融合 PointCloud2 的 observed-free contract 尚未闭合。

Claude 应独立推翻或确认这些假设。

## 13. 当前停止声明

- 本交接只新增这一份文档；
- 未修改任何源码、launch、CMake、参数或测试；
- 未启动 ROS；
- A5-G2h 已中断，零代码修改；
- 未进入 A6、planner integration、swarm 或 CBF；
- 当前应等待 Claude 完成只读审查和回退/修复决策。
