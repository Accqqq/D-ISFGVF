# A5-G2f：局部 Tube 几何、Filter 与显示语义简化执行规范

## 0. 授权与停止边界

本执行单只授权 **A5-G2f**。目标是把当前 ESDF/cloud tube 恢复为主
proposal 中的简单管道：

```text
规划 B 样条 p(w)
  -> lifted state (p,T,N,kappa)
  -> 沿 +/-N 查询不可变 PointCloud2 occupancy snapshot
  -> margin erosion
  -> 包含当前 w、retained delta 的局部连续 tube segment
  -> 当前周期 PortProjector
```

`AUTO_ADVANCE=false`。完成实现、构建、单测、隔离 ROS 验收、自审核和
报告后立即停止。禁止进入 A6、A5.3、多机、CBF、planner tuning 或新场景。

本阶段不得通过增加 gate、增加 profile 副本、调大 tracking bound、放宽地图
语义或调大 `boundary_slope_max` 让测试通过。

## 1. 执行前置条件

开始前只读记录并核对：

- branch 必须为 `main`；
- HEAD 必须为 `9a0e97560b8ddf15ac1ed1581030ed4b911f6e43`；
- stash `deepseek-phaseoffset-tracked-prototype-2026-08-08` 必须存在且不得
  restore/pop；
- 记录 `git status --short` 和 tracked diff；
- 当前 dirty worktree 全部视为用户内容，不得清理、覆盖、暂存或提交；
- 不得连接、终止或复用用户已有 ROS master/process。

任一前置条件不满足时停止并报告，不得自行修复仓库状态。

## 2. 已确认问题与正式输入证据

2026-08-10 隔离 ESDF/cloud 实测证据：

`/tmp/a5_esdf_visual_check/evidence/esdf_visual.bag`

测试事实：

- Candidate Marker：49 帧 ADD、80 帧 DELETE；
- Certified Marker：3 帧 ADD、126 帧 DELETE；
- 27/27 cloud snapshot 均
  `available=valid=usable=obstacle_set_complete=1`；
- 27/27 candidate raw build 均 `raw_complete=1`；
- sequence 8--24 出现 `filtered_complete=0`，持续约 1.70 s；
- Candidate DELETE 从 path revision 仍为 1 时已经开始，不能归因于 A6；
- 后续 revision 2/3/4 与旧 active profile 域失配属于 A6，不在本阶段修；
- 当前 Candidate Marker 要求 `profile.complete`，因此 filter 对整个 preview
  的一票否决直接造成可见 tube 消失；
- 当前 PointCloud2/local_sensing snapshot 管道本身有效，不得退回 SDFMap raw
  storage、自清除 seed 或 mutable occupancy 查询。

本阶段必须以这些事实为起点，禁止重新把问题描述为 RViz 假象、cloud topic
缺失或 100-cycle gate 未打开。

## 3. 主 proposal 语义

### 3.1 Tube 是几何走廊，不是状态机

基础路径与活动参考保持区分：

\[
r(w,\delta)=p(w)+N(w)\delta,
\qquad
\underline\delta(w)\le\delta\le\overline\delta(w).
\]

Tube 构建只回答：在当前不可变地图观测上，沿基础路径法向可得到什么局部
连续 offset 区间。

以下事实不得参与 Candidate 几何完整性：

- 100-cycle zero gate；
- `selected`；
- tracking 是否暂时超界；
- Runtime normal/recovery/emergency mode；
- failure latch；
- 旧 active profile 的 ownership；
- install-time dynamic classification；
- path revision continuation 是否已在 A6 完成。

### 3.2 安全证书不是 tube 本身

严格 obstacle certificate 表示：地图观测、margin erosion、当前 tracking ball
和当前可执行端口共同支持安全声明。它可以因为 tracking 或当前执行集合失效而
撤销，但不得使 Candidate 几何随之消失。

Candidate 是主要的 tube 几何显示；Certified 是额外的当前严格声明。不要为了
让 Certified 常亮而放宽安全条件，也不要让 Certified 的条件反向控制 Candidate。

### 3.3 只保留 candidate/active ownership

继续只允许：

```text
candidate_profile = 最新观测上包含当前 w 的局部连续 tube segment
active_profile    = 最近一次满足安装条件的 candidate
```

禁止新增 recovery/display/pending/shadow/fallback profile。

## 4. 必须实现的 Filter 修订

### 4.1 从“整个 preview 一票否决”改为“当前锚定的局部连续段”

当前 raw builder 已能在 UNKNOWN、OUT_OF_MAP、OCCUPIED 等位置把 requested
preview 截短为包含当前样本的 certified segment。Filter 必须采用相同的局部段
语义。

输入至少必须明确：

- `current_w`；
- `retained_delta`；
- raw profile；
- 配置的边界斜率/连续性限制。

输出必须满足：

1. 找到与 `current_w` 对应的锚点样本；
2. 锚点 raw interval 必须包含 `retained_delta`；
3. 从锚点向前、必要时向后构造最大可用的连续 filtered segment；
4. 某个远端 segment 发生 slope/dense-safety/interval 冲突时，在该处截短，
   不得把当前仍有效的局部段整体判为 incomplete；
5. 返回段的每个 filtered boundary 必须是 raw interval 的子集；
6. 返回段必须有序、finite、非空，并在锚点包含 retained delta；
7. 连续插值后的 dense samples 必须保持在 raw bounds 内；
8. 实际边界变化率不得超过配置值；
9. 不允许通过把配置默认值 `0.80` 调大来获得成功；
10. 如果连锚点或锚点相邻的最小可显示/可执行段都不可行，才返回
    `filtered_complete=false`。

算法必须确定性。样本数量当前很小，允许为正确性使用有界的区间搜索或候选段
搜索；禁止引入通用优化器、QP dependency 或 ROS dependency。

### 4.2 截短事实

复用现有 `TubeProfile` 的 requested/certified segment、truncated flag、
`first_truncated_w/reason` 语义。不要再创建第三份 profile。

若现有字段不能区分 raw truncation 与 filter truncation，可在现有 focused type
中追加一个最小 enum/field；不得修改旧 diagnostics 索引。确需 ROS diagnostics
时只能追加独立 payload 或在 epoch schema 末尾追加，且必须有 static_assert 和
严格列数测试。优先复用现有字段，避免 schema 扩张。

### 4.3 Candidate Marker

Candidate 三个 Marker ID：

- 只由最新 candidate local segment 是否 complete/finite、samples >= 2 决定；
- local segment 合格时三个 ADD；
- 当前锚点本身不可构建时三个 DELETE；
- 不得因远端 suffix 截短而 DELETE；
- 不依赖 gate、selected、tracking、Runtime mode、failure latch 或 Certified；
- namespace、ID、颜色可保持现状；
- 三个 ID 不得出现 mixed ADD/DELETE。

## 5. Active 安装与 Runtime：做减法，不增加门控

### 5.1 Active 安装

新的 local candidate 只有在以下条件满足时才替换 active：

- candidate local segment complete；
- 当前几何/bounds 有效；
- retained delta 在当前 bounds 内；
- ESDF/cloud 当前 reference/actual 没有明确 OCCUPIED/OUT_OF_MAP；
- candidate 的 forward certified length 满足现有
  `min_certified_forward_w`。

candidate 截短但 forward length 仍足够时允许安装；不足时保留旧 active。
UNKNOWN/UNAVAILABLE/incomplete 不得伪装成 permanent failure。

### 5.2 Install-time dynamic rollout 不得成为额外总 gate

主 proposal 的执行安全由每个高率周期的同一个 `PortProjector` 保证：

1. 先求当前/next-envelope 的 `U+`；
2. `U+` 为空时求 `U>=0` 回中端口；
3. 两者都为空或出现明确 map unsafe/math invalid 时才 emergency。

审查现有 install-time horizon rollout。若它只是重复 Runtime exact
PortProjector 并造成完整几何 candidate 被拒绝，则将它降为 diagnostics/precheck，
不得继续作为 Candidate 完整性或 Marker gate。不得复制 PortProjector 公式。

不要新增第四种普通 Runtime mode。保留：

```text
normal U+
nonnegative recenter U>=0
emergency/fatal
```

兼容 enum 可保留，但不得再组合新的长布尔门控链。

### 5.3 100-cycle gate

gate 只决定 phase-offset 是否接管控制：

```text
selected = zero_gate_open && runtime exact port executable && !fatal
```

gate 不参与 tube build、Candidate、active ownership 或 obstacle certificate。

## 6. Certified Marker 的最小语义审查

不得降低安全声明。只删除与证书无直接关系的偶然门控。

Certified 可以要求：

- active profile complete/obstacle-certified；
- 当前 geometry/bounds 有效；
- retained delta 当前在 active bounds 内；
- tracking within reserved bound；
- 当前地图事实不是明确/不确定 unsafe；
- 当前至少存在 exact `U>=0` 执行结果；
- 无 emergency/fatal/latch。

Certified 不得仅因为：

- zero gate 未开；
- control 尚未 selected；
- 最新 candidate 被远端 filter suffix 截短；
- install-time frozen facts 与当前值不完全相等；

而撤销。

若当前架构无法在不扩大范围的情况下给出“当前 exact U>=0”事实，则保持
Certified 保守 DELETE，并在报告中说明；不得新增一套 projector 或缓存证书。

## 7. A6 明确隔离

本阶段不得修改：

- 原 C2 connector；
- `gvf_manager.cpp` 的 path install/switch；
- `gvf.cpp` 的 phase advancement/governor；
- path revision 生成逻辑；
- `w,delta` continuation；
- 旧 frontend 越界/closed phase hold。

如果 ESDF ROS 测试出现：旧 active path revision 与新 base path revision 不同、
当前 phase 超出旧 active 域、reference jump 或 C2 unavailable，记录为 A6 blocker。
不得在 A5 添加 revision gate、phase clamp 或 reset delta 的 workaround。

## 8. 文件白名单

允许按最小必要范围修改：

```text
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_filter.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_filter_test.cpp

src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_manager.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp

src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp

src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_dynamic_feasibility.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_dynamic_feasibility.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_dynamic_feasibility_test.cpp

src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_tube_markers.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_tube_markers.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_markers_test.cpp
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_tube_epoch_diagnostics.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_tube_epoch_diagnostics.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_diagnostics_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_integration_test.cpp
src/swarm_planner/bspline_traj/CMakeLists.txt
```

`tube_dynamic_feasibility.*` 只有在将其从 hard gate 降为 focused
diagnostics/precheck 时才允许修改；不得扩大其公式或增加新约束。

禁止修改：

- `gvf_manager.*`、`gvf.*`；
- planner/Kino/B-spline optimizer；
- plan_env/SDFMap/cloud snapshot producer；
- launch 默认值和速度/地图/目标参数；
- phase_offset_core 数学；
- 原 C2/governor/SO3/simulator；
- swarm/CBF/messages；
- RViz 配置（现有 Candidate/Certified display 已存在）。

若正确实现必须修改白名单外文件，停止并报告，不得自行扩权。

## 9. 必须新增/修订的单元测试

至少覆盖：

1. raw profile 全部 complete，但远端 suffix 与 slope/dense safety 冲突：
   返回包含 current 的截短 complete segment，而不是整条失败；
2. current raw sample 不可行：candidate incomplete；
3. current interval 不包含 retained delta：candidate 不可安装；
4. 截短后的所有 filtered dense samples 均在 raw interval 内；
5. boundary slope 不超过原配置 `0.80`；
6. segment 有序、finite、至少两点并包含 current；
7. forward segment 足够时可安装 active；不足时保留旧 active；
8. incomplete/unknown candidate 不覆盖 active，不 latch；
9. explicit current OCCUPIED/OUT_OF_MAP 才撤销当前安全声明；
10. Candidate 在远端截短时三个 ADD；当前锚点失败时三个 DELETE；
11. Candidate 不受 gate/tracking/selected/emergency/failure latch 影响；
12. Certified 不受 gate/selected 或单纯 filter suffix truncation 影响；
13. Runtime 继续按 `U+ -> U>=0 -> emergency`，无新增 gate；
14. zero-port、matched residual、same-final-port 和 non-reversing invariants
    全部回归；
15. diagnostics schema 若未授权追加则严格保持 manual=83、epoch=60、
    raw=49、cloud=18。

不得删除、弱化或跳过现有测试来获得通过。

## 10. 构建与回归

依次执行并保存完整结果：

1. `catkin_make -j8`；
2. phase_offset_navigation 的 filter/epoch/runtime/dynamic focused tests；
3. phase_offset_core 全部回归；
4. bspline_race matched adapter/marker/epoch diagnostics/integration 回归；
5. `catkin_test_results --verbose`；
6. dependency-boundary 搜索；
7. `git diff --check`。

既有 `uav_utils` 缺 XML 可作为历史项报告，不得顺手修复。

## 11. 隔离 ROS 验收

运行前检查用户 ROS master/process。使用新的隔离端口和独立 `ROS_HOME`，只清理
本任务启动的进程。

### ROS-0 baseline

裸 `test_gvf.launch`：

- 实际参数必须仍为 disabled/none/refresh=3.0；
- manual Candidate/Certified/diagnostics 无 publisher；
- `/position_cmd` 唯一 publisher 仍为 `/formation_planning`；
- 原点到点目标正常到达。

### ROS-1 fixed

沿用现有 fixed 参数，不调参：

- Candidate 从有效 path 起连续 ADD；
- gate 前 emergency=0、selected=0；
- gate 后 normal/recenter 可 selected；
- tracking transient 可以 Certified DELETE，但 Candidate 不消失；
- no latch/fatal，matched residual 合格。

### ROS-2 ESDF/cloud snapshot

沿用 G2d/G2e 的 PointCloud2 occupancy snapshot 参数、原 pillar 地图和正式长目标，
不得为了避开障碍改地图/目标/速度/threshold。

必须录制：

```text
/formation_planning/phase_offset_manual/tube_candidate
/formation_planning/phase_offset_manual/tube
/formation_planning/phase_offset_manual/diagnostics
/formation_planning/phase_offset_manual/tube_epoch_diagnostics
/formation_planning/phase_offset_manual/tube_raw_candidate_diagnostics
/formation_planning/phase_offset_manual/tube_cloud_snapshot_diagnostics
/position_cmd
/sim/odom
/sim/local_map
/move_base_simple/goal
```

验收要求：

- raw storage access=0、self-free seed=0；
- cloud snapshot contract 持续 usable；
- margins 保持 full/preincluded/residual = `0.55/0.10/0.45`；
- 不再出现 `raw_complete=1 && filtered_complete=0` 仅因远端 suffix
  冲突而使整个 Candidate DELETE；
- 远端冲突应表现为 local segment 截短；
- Candidate/diagnostics/Marker 每帧一致，无 mixed action；
- Certified 可因真实 tracking/current exact execution/map evidence 撤销，
  但不得因 zero gate 或单纯远端 filter truncation 撤销；
- gate 后至少得到一个可核查的连续 selected normal/recenter 窗口；
- no permanent latch/fatal；
- emergency 只能对应 explicit current map unsafe、exact `U>=0` empty 或
  math invariant failure；
- 若失败由 path revision/reference jump/旧 active 域越界导致，明确归类 A6
  blocker，不得在本阶段绕改。

保存 bag、CSV、JSON、分析脚本及 SHA-256。分析必须按 rosbag message time 对齐，
严格断言 payload 列数和 Marker/diagnostics 一致性。

## 12. 最终自审核与报告

报告必须给出：

- 实际修改文件与每个文件的职责；
- filter 的确定性局部段算法和安全不变量；
- 是否删除/降级了任何 accidental gate；
- Candidate、Certified、active、Runtime 的最终语义；
- 单测数量与结果；
- ROS-0/1/2 原始证据路径和统计；
- path revision/A6 事件是否仍存在；
- dependency-boundary 搜索；
- `git diff --check`；
- 最终 `git status --short`；
- 无 staged/commit/tag/push；
- 未修改任何白名单外文件；
- 明确停止语句：未进入 A6/A5.3/多机。

如果 ROS-2 证明当前 raw corridor 在 current anchor 本身不安全，或正确修复必须
修改 planner/C2/地图，立即停止并报告。不得继续增加 gate 或调参。
