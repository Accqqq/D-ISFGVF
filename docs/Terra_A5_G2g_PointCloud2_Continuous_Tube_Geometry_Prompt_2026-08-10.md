# A5-G2g：PointCloud2 连续 Tube 几何与 Clearance 一致性执行规范

## 0. 授权与停止边界

本执行单只授权 **A5-G2g**。目标是修复当前 ESDF/cloud Candidate tube 的
几何安全定义：从不可变 PointCloud2 occupancy snapshot 构造并验证真正的连续
安全 ribbon，而不是只在离散 lifted-state knot 上沿 `+N/-N` 两条射线检查。

`AUTO_ADVANCE=false`。完成代码、构建、单测、隔离 ROS 验收、自审核和报告后
必须停止。

本阶段禁止处理：

- 当前剩余的 106 帧 live `U+ / U>=0` empty；
- Runtime slew-continuous port sequence；
- A6 path/C2/phase/tube continuation；
- planner/Kino/B-spline optimizer tuning；
- 多机、swarm、CBF；
- 速度、margin、地图、目标或 gate 参数调整。

如果连续几何通过后仍有 live port emergency，原样记录并留给独立 A5-G2h，
不得在本阶段顺手修 Runtime。

## 1. 执行前置条件

开始前只读记录并核对：

- branch=`main`；
- HEAD=`9a0e97560b8ddf15ac1ed1581030ed4b911f6e43`；
- stash `deepseek-phaseoffset-tracked-prototype-2026-08-08` 存在且不得
  restore/pop；
- 记录 `git status --short`、tracked diff 和白名单内 untracked 文件摘要；
- 当前 dirty worktree 全部视为用户内容；
- staged 必须为空；
- 不得连接、复用或终止用户 ROS master/process；
- ROS 验收必须使用新的隔离端口和独立 `ROS_HOME`。

任一前置条件异常时停止报告，不得清理、reset、restore、stash、commit、branch、
tag 或 push。

## 2. 已确认的正式问题证据

最新 ESDF/cloud 证据：

`/tmp/a5_g2f_semantics_20260810/esdf/evidence/esdf.bag`

A5-G2f 已确认：

- Candidate 317/317 ADD；
- cloud snapshot 66/66 usable；
- full/preincluded/residual margin=`0.55/0.10/0.45`；
- raw-storage/self-free 均为 0；
- 单机 retained delta/intent 语义已收紧，普通单机不自动侧移。

但主代理对 66 个 due Candidate ribbon 与同期 `/sim/local_map` 做了独立空间
审计：

- 没有原始 cloud point 直接落在 ribbon 三角形内部；
- 22/66 个 Candidate 的 ribbon 到 occupied cloud point 的最小距离小于
  `0.44 m`；
- 19/66 小于 `0.40 m`；
- 11/66 小于 `0.30 m`；
- 全局最小距离约 `0.1604 m`；
- 当前 residual effective radius 为 `0.45 m`。

因此 A5-G2f 只解决了 Marker/Candidate 连续显示，没有建立连续几何 clearance
证书。显示膨胀障碍时，tube 与障碍区域重叠是可复现的真实问题，不是 RViz
错觉，也不是 live port emergency 或 A6 revision 问题。

## 3. 根因与禁止继续使用的假设

当前 `TubeCrossSectionSolver`：

1. 在每个 path knot 上仅沿 `+N/-N` 两条一维射线查询 categorical occupancy；
2. 找到射线停止位置后，用
   `ray_distance - residual_effective_radius` 得到 offset bound；
3. `TubeFilter` 只验证 scalar bounds 的 Hermite/dense 插值关系；
4. 最终 Marker 把相邻边界点连接成三角 ribbon。

该逻辑不能发现：

- 位于法向射线旁边的 diagonal obstacle；
- 两个 `w` knots 之间的障碍；
- obstacle voxel 与 candidate center 的最近方向不是 `N` 的情况；
- 连接后的三角 ribbon 在 workspace 中切过障碍 clearance ball；
- observed AABB 边缘附近没有足够完整邻域支撑 clearance 声明。

禁止继续使用以下推理作为安全证书：

```text
两条法向射线无 occupied
  + 沿法向减去 radius
  = 整个连续 ribbon 有 Euclidean clearance
```

这在一般 PointCloud2 障碍几何下不成立。

## 4. 目标几何语义

基础路径 lifted state：

\[
p(w),\quad T(w),\quad N(w),\quad \kappa(w).
\]

Candidate center surface：

\[
r(w,\delta)=p(w)+N(w)\delta,
\qquad
\underline\delta(w)\le\delta\le\overline\delta(w).
\]

对 snapshot 已包含 `0.10 m` map inflation 的当前配置，Candidate 上每个中心位置
必须满足：

\[
d_{\mathrm{occupied\ voxel\ volume}}(r(w,\delta))
\ge \rho_{\mathrm{residual}}=0.45\ \mathrm{m}.
\]

距离必须针对 occupied voxel 的闭 AABB/体积，而不是只针对 voxel center；不得
因 voxel resolution 得到乐观 clearance。

Candidate complete 的含义改为：

> 当前不可变 snapshot 上，包含 `current_w` 的局部连续 surface segment 已得到
> observed-domain 和 Euclidean-clearance 的保守证书。

Candidate 仍不依赖：

- retained delta 是否在 corridor 中；
- gate/selected；
- tracking；
- active ownership；
- Certified；
- dynamic rollout/live port；
- path revision continuation。

## 5. PointCloud2 Snapshot Clearance Query

### 5.1 复用 immutable voxel snapshot

继续复用 `plan_env::CloudOccupancySnapshot`：

- snapshot 从融合后的 PointCloud2/local occupycloud 构建；
- local_sensing 与真实融合 occupancy cloud 走同一接口；
- snapshot 构建后不可变；
- tube build 不读取 mutable SDFMap raw log odds、ESDF buffer 或 self-free seed；
- `obstacle_set_complete` opt-in 和 observed AABB contract 保持。

### 5.2 新增 bounded clearance query

在 `plan_env` 中新增 focused clearance query，建议语义：

```text
queryCloudOccupancySnapshotClearance(snapshot, point, required_radius)
  -> status
  -> nearest occupied voxel-volume distance, capped if appropriate
  -> clearance_certified
```

实现可使用当前 occupancy grid 的确定性 bounded voxel neighborhood 搜索；不要求
引入 PCL KD-tree、全局 EDT 或新第三方 dependency。

要求：

1. invalid snapshot -> UNAVAILABLE；
2. point outside global map -> OUT_OF_MAP；
3. point outside observed AABB -> UNKNOWN；
4. point inside occupied voxel volume -> OCCUPIED，distance=0；
5. 查询 `required_radius` 所需的完整闭球/搜索 AABB 若越出 observed AABB，
   必须 UNKNOWN，不能把未观测空间当 free；
6. 距离计算到 occupied voxel closed AABB，不是 voxel center；
7. 若完整搜索邻域内没有 occupied，允许保守返回
   `distance >= required_radius`；
8. 所有 metadata、index arithmetic、overflow、NaN fail closed；
9. included inflation 只计算一次；
10. query 是 snapshot 纯只读操作，线程安全、确定性、无 ROS callback。

性能上搜索半径由现有 residual radius 和 snapshot resolution 决定，禁止新增
用户调参来控制正确性。

### 5.3 Integration bridge

在 bspline adapter bridge 中把 snapshot clearance query 转换为 navigation 的
抽象 `DistanceQuery` 或一个同等 focused Eigen/STL callback。

Navigation 不得依赖 `plan_env`、ROS、PointCloud2、SDFMap 或 PCL。

## 6. Cross-section 求解必须改为中心 clearance

### 6.1 不再使用 raw ray distance 减 radius

ESDF/cloud cross-section 对候选 offset：

\[
x(\delta)=p+N\delta
\]

直接查询 `x(delta)` 的最近 occupied-volume clearance。

`delta` 安全当且仅当：

- query status=KNOWN_FREE；
- signed/unsigned clearance finite；
- clearance >= residual effective radius；
- regularity condition 仍满足。

可以沿 `delta` 使用现有 `ray_step=0.05` 做确定性采样，并在 safe/unsafe、
safe/UNKNOWN、safe/OUT_OF_MAP 边界上做有界二分 refinement。禁止把 categorical
ray termination distance 再减一次 radius。

### 6.2 连通分量

同一 cross-section 可能存在多个 safe delta component。

- Candidate geometry 可以选择包含 `preferred_delta` 的 component；
- 若 preferred delta 不在任一 component，选择与 preferred delta 距离最小且
  deterministic tie-break 的非空 component，仅用于 Candidate 显示；
- active 安装仍由 TubeEpochManager 单独检查 retained delta containment；
- 不得自动改变正常单机的 delta intent；
- component 不得跨越 unsafe/UNKNOWN gap 合并。

### 6.3 Legacy 字段

可保留 `c_plus_raw/c_minus_raw`、ray termination 等 ABI/diagnostic 字段，但必须
明确它们是 compatibility facts，不能再驱动 clearance 证明。禁止 double erosion。

## 7. 沿 w 的自适应 lifted-state 采样

离散 cross-section 安全不足以证明连续 ribbon 安全。

### 7.1 Abstract path evaluator

Navigation 可新增纯 C++ `PathStateQuery` callback：

```text
bool evaluate(w, PathDifferentialState&)
```

Integration adapter 可使用已经存在的 semantic path/sample interface 绑定该
callback；不得修改 `gvf_manager.cpp`、原 C2 connector 或路径公式。

### 7.2 Adaptive refinement

对相邻 `w_i,w_{i+1}` 至少在以下情况插入 midpoint lifted-state sample：

- path center chord/actual midpoint deviation 不能由 snapshot resolution 保守覆盖；
- `N` 或 boundary workspace endpoint 的变化过大；
- surface cell 的空间 cover radius 过大；
- midpoint clearance 或 status 与 endpoints 不一致；
- 当前插值无法证明整段 clearance。

step/tolerance 必须由：

- snapshot resolution；
- existing boundary tolerance；
-现有 sample step 上限；

确定性派生。禁止新增“调到能过”的 launch 参数。

必须有：

- max subdivision depth；
- max total samples；
- overflow/limit exceeded fail closed；
- deterministic ordering/tie-break；
- current_w 精确锚点保留。

## 8. 连续 Ribbon Surface Validator

新增 focused `TubeSurfaceValidator`（或同等单一职责模块），输入：

- filtered local profile；
- abstract path evaluator；
- immutable clearance query；
- snapshot resolution/required residual clearance；
- current_w。

验证整个：

\[
(w,\delta)\mapsto p(w)+N(w)\delta
\]

局部面，而不只是左右边界线。

### 8.1 保守覆盖条件

对每个 surface cell 使用确定性 subdivision。若 cell 内所有位置到已查询
samples 的最大空间 cover radius 为 `h`，则已查询 samples 必须满足：

\[
d_{obs}\ge \rho_{residual}+h+\epsilon.
\]

利用 Euclidean distance 的 1-Lipschitz 性质，才能推出 cell 内所有中心点满足
`d_obs >= rho_residual`。

或者使用同等严格、可单测证明的 interval/conservative geometry 方法。禁止只用
固定几个视觉采样点后声称连续证书。

### 8.2 Failure handling

- 远端 surface cell 失败：截短到包含 current_w 的最近连续 certified segment；
- 当前锚点 surface/cross-section 失败：Candidate DELETE；
- unknown/out-of-map：截短或 fail closed；
- 不允许保留穿过 unsafe gap 的两段并用 Marker 三角形连接；
- validator 不修改 delta、active、Runtime 或 control command。

## 9. Base-path delta=0 一致性

正常单机无 manual/swarm intent 时：

\[
\delta_{ref}=0,\qquad \delta=0.
\]

本阶段不修改 planner，但必须输出并使用以下事实：

- Candidate local corridor 是否在 current/forward certified horizon 包含 0；
- retained delta 是否在 current/forward corridor 内；
- base centerline surface clearance 是否满足同一 snapshot predicate。

Active 安装必须继续要求当前 retained delta 在 corridor 内，并要求现有最小 forward
horizon 内 retained delta 连续包含。若 `delta=0` 不满足：

- Candidate 可显示安全几何 component；
- active 不安装或 current validation 撤销；
- Certified DELETE；
- Runtime 等待；
- 不得自动选择非零 delta；
- 记录为 base-path/tube clearance mismatch，供未来独立 planner integration 阶段
  使用。

禁止在本阶段修改 Kino margin schedule、B-spline lambda/safe_distance、final path
install、C2 cost 或 replan policy。

## 10. Candidate、Active、Certified 边界

### Candidate

- 只显示经过连续 surface clearance 验证的最新局部几何；
- 三个 Marker ID 同步 ADD/DELETE；
- 远端截短不删除当前安全段；
- 不依赖 delta containment/gate/tracking/selected/Runtime/dynamic/A6。

### Active

- 继续由 candidate/active 两个 ownership 表达；
- 安装要求 retained delta current/forward contained；
- 不满足时不自动侧移；
- 不新增 recovery/display/pending profile。

### Certified

- 继续保守；
- 必须基于新的 continuous surface certificate；
- 其余 tracking/current exact port 语义保持；
- 本阶段不得为解决 106 live emergency 修改 Certified 条件或 Runtime。

## 11. 文件白名单

允许按最小必要范围修改：

```text
src/swarm_planner/plan_env/include/plan_env/cloud_occupancy_snapshot.h
src/swarm_planner/plan_env/src/cloud_occupancy_snapshot.cpp
src/swarm_planner/plan_env/test/cloud_occupancy_snapshot_test.cpp
src/swarm_planner/plan_env/CMakeLists.txt

src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/distance_query.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_cross_section.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_cross_section.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_cross_section_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_builder.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_builder_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_filter.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_filter_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_manager.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/CMakeLists.txt
```

允许新增以下单一职责文件：

```text
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/path_state_query.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_surface_validator.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
```

Integration bridge：

```text
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_cloud_occupancy_query.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_cloud_occupancy_query.cpp
src/swarm_planner/bspline_traj/test/phase_offset_cloud_occupancy_query_test.cpp
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_integration_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_markers_test.cpp
src/swarm_planner/bspline_traj/CMakeLists.txt
```

如果需要 focused independent diagnostics，允许新增最多一个 header/source/test 和
一个独立 ROS topic；不得移动 manual=83、epoch=60、raw=49、cloud=18 的旧索引。
优先复用 `TubeBuildDiagnostics.min_safety_margin`、truncation 和现有 schema。

禁止修改：

- `gvf_manager.*`、`gvf.*`；
- Kino/path_searching/B-spline optimizer；
- phase_offset Runtime、PortProjector、dynamic feasibility；
- original C2/governor/SO3/simulator；
- launch 默认参数、margin、速度、地图或目标；
- swarm/CBF/messages/RViz；
- SDFMap mutable raw storage/ESDF 算法。

若正确实现必须修改白名单外文件，停止报告，不得扩权。

## 12. 必须新增的单元测试

### plan_env clearance query

1. invalid snapshot -> UNAVAILABLE；
2. outside map -> OUT_OF_MAP；
3. outside observed AABB -> UNKNOWN；
4. clearance ball 越出 observed AABB -> UNKNOWN；
5. point inside occupied voxel -> OCCUPIED/distance=0；
6. point-to-voxel-AABB distance，不是 center distance；
7. diagonal/corner nearest voxel；
8. no occupied within certified radius；
9. included inflation 不重复；
10. empty complete occupycloud 在完整 observed neighborhood 内为可认证 free；
11. NaN/index overflow/vector size mismatch fail closed。

### cross-section

1. diagonal obstacle 不在 `+N/-N` ray 上仍收缩 interval；
2. one-sided corridor；
3. multiple safe components 不跨 unsafe gap 合并；
4. preferred delta component deterministic selection；
5. preferred delta 不安全时 Candidate component 仍可输出，但不自动改变 intent；
6. UNKNOWN/OUT_OF_MAP 截短；
7. no double erosion；
8. curvature regularity 仍正确。

### adaptive path/surface

1. obstacle 位于两个 w knots 中间，endpoints 均安全但 surface 被拒绝/细分；
2. diagonal wall；
3. thin pillar；
4. curved path/normal rotation；
5. left/right boundary safe 但 interior surface unsafe；
6. observed-domain edge；
7. cover-radius/Lipschitz clearance 条件；
8. max depth/sample limit fail closed；
9. far suffix unsafe 截短，current local segment 保留；
10. current cell unsafe -> Candidate incomplete；
11. output profile ordered/finite、无 unsafe gap bridge；
12. every certified test sample clearance >= residual radius。

### ownership/integration

1. Candidate complete 与 retained delta containment 解耦；
2. normal single delta=0 不在 corridor -> Candidate ADD、active reject、Certified
   DELETE、无自动 lateral intent；
3. retained delta current/forward contained 才安装；
4. snapshot pointer/sequence 在一个 build 内一致；
5. raw-storage/self-free 仍为 0；
6. Marker 三 ID 同步；
7. fixed tube 行为不变；
8. diagnostics 旧列数严格不变。

不得删除或弱化现有测试。

## 13. 构建与回归

依次执行并保存结果：

1. `catkin_make -j8`；
2. plan_env snapshot/clearance tests；
3. navigation cross-section/builder/filter/surface/epoch tests；
4. phase_offset_core 全部回归；
5. bspline adapter/cloud/marker/epoch integration tests；
6. A5-G2f fixed/runtime/dynamic 既有回归；
7. `catkin_test_results --verbose`；
8. dependency-boundary search；
9. `git diff --check`；
10. staged/status/whitelist audit。

历史 `uav_utils -luav_utils` 聚合链接缺陷只报告，不得修复。

## 14. 隔离 ROS 验收

### ROS-0 baseline

- naked launch 仍 disabled/none/refresh=3.0；
- manual tube topics 无 publisher；
- `/position_cmd` 唯一 publisher `/formation_planning`；
- 原点到点达到。

### ROS-1 fixed

- Candidate 连续 ADD；
- fixed 不依赖 map clearance query；
- gate 后 selected；
- 0 emergency/latch/fatal；
- matched residual 合格。

### ROS-2 ESDF/cloud

沿用 A5-G2f 相同正式参数、pillar、目标、速度和 margin，不调参。必须录制原有
Candidate/Certified/manual/epoch/raw/cloud diagnostics、position_cmd、odom、
local_map、goal，并保存 bag/CSV/JSON/analyzer/SHA-256。

必须增加独立离线 surface-clearance audit：

1. 每个 due Candidate Marker 对齐同一 observation sequence/snapshot；
2. 使用与生产不同实现的审计器，从 bag PointCloud2/voxel geometry 计算 ribbon
   到 occupied voxel volume 的保守最小 clearance；
3. 按 marker triangle/surface cell 做 adaptive/subdivision 检查；
4. 断言所有 Candidate ADD 的 certified local surface：

\[
\min d_{occupied\ volume}\ge0.45-\epsilon
\]

其中 `epsilon` 只能覆盖数值/voxel boundary tolerance，不能取 0.01 以上来隐藏
0.16 m 一类违规；
5. 断言 occupied point/voxel 不在 ribbon interior；
6. 断言 observed-domain neighborhood 完整；
7. 报告 min/percentile、失败 frame/sequence/w/delta/workspace point；
8. 旧 bag 应被新审计器稳定检出约 0.160 m 的反例，作为 analyzer sanity check；
9. 新 bag 不得出现该反例。

其他验收：

- Candidate Marker/diagnostics 一致，无 mixed action；
- raw-storage/self-free=0；
- cloud usable；
- margin 仍 `0.55/0.10/0.45`；
- current delta=0 不在 corridor 时不自动侧移；
- no latch/fatal；
- 106 live exact-port emergency 是否仍存在只记录，不作为本阶段几何通过条件，
  也不得在本阶段修；
- path revision/A6 事件只记录。

若连续 clearance 导致当前真实场景没有足够 Candidate local segment，按事实停止并
报告 base planner/clearance mismatch，不得调小 0.45、换目标或放宽 validator。

## 15. 最终报告与停止

报告必须给出：

- 实际修改文件及职责；
- snapshot clearance query 的 voxel-volume 距离定义；
- cross-section component 算法；
- adaptive w/surface validator 及保守证明；
- no-double-erosion 说明；
- 单机 delta=0 语义；
- 单测/回归结果；
- ROS-0/1/2 原始证据；
- 旧 bag 反例与新 bag min clearance 对比；
- Candidate/Certified/action 统计；
- live port emergency 只读统计；
- dependency audit、diff check、status、staged=0；
- 无 commit/tag/push；
- 未修改白名单外文件；
- 明确停止：未进入 A5-G2h、A6、A5.3 或多机。

只有当新 ESDF bag 的连续 surface clearance 证据通过时，A5-G2g 才能标记完成。
