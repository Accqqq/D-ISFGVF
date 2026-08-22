# A5-G2d：Cloud Occupancy Snapshot 地图合同纠正执行规范

执行模型：`gpt-5.6-terra`

推理强度：`max`

日期：2026-08-10

`AUTO_ADVANCE=false`

本执行单只授权 A5-G2d。完成构建、测试、隔离 ROS 验收、自审核和报告后必须停止。禁止进入 A5-G2e、A5.3、A6 或任何多机阶段。

---

## 1. 本阶段结论与唯一目标

当前仿真和实机规划入口都是 obstacle/occupancy PointCloud2：

```text
simulation:
pillar.pcd -> local_sensing -> /sim/local_map

real flight:
depth/local mapping pipeline -> /drone_1_cloud_registered

both:
PointCloud2 -> SDFMap::cloudCallback
            -> occupancy_buffer_inflate_
            -> distance_buffer_all_
```

`SDFMap::cloudCallback` 不更新 raw log-odds `occupancy_buffer_`，也不调用 depth raycast。当前 A5-G2b 却因为 raw vector 已分配就强制注入 `RawOccupancyQuery`，导致 cloud mode 中几乎所有 cell 被判为 UNKNOWN。

本阶段唯一目标：

> 为 A5 ESDF-selected tube 建立一个来自当前 occupycloud 合同的、不可变且单次 build 一致的 inflated-occupancy snapshot；继续复用已经完成的非对称 `TubeCrossSectionSolver`，并用明确的 preincluded map inflation accounting 避免 double erosion。

本阶段不是 planner clearance tuning，不实现新 planner contract，不修改 A*/B-spline/C2，也不补完整 hybrid dynamic feasibility。

---

## 2. 执行前必须核对的仓库状态

必须先只读记录：

```bash
git branch --show-current
git rev-parse HEAD
git status --short
git diff --stat
git diff --check
git stash list
```

预期：

- branch：`main`；
- HEAD：`9a0e97560b8ddf15ac1ed1581030ed4b911f6e43`；
- stash 中存在：`deepseek-phaseoffset-tracked-prototype-2026-08-08`；
- 工作树长期 dirty，全部视为用户内容；
- 无 staged 内容。

若 branch、HEAD、stash 或 staged 状态不符，立即停止并报告。禁止 reset、restore、clean、stash pop、checkout 覆盖用户内容。

在修改前保存所有 tracked 文件的 SHA-256/mtime manifest，并保存 `git status --short` 与 tracked diff。完成后做精确前后比较。

---

## 3. 必须先读完的权威资料

在修改前完整读取与本阶段相关部分：

1. `AGENTS.md`；
2. `docs/PhaseOffsetSwarm_Single_First_Implementation_Plan_2026-08-07.md` 的 A5/A6；
3. `docs/PhaseOffsetSwarm_Code_Architecture_2026-08-08.md` 的 navigation、adapter 和每周期数据流；
4. `/home/cxq/ISF-GVF/Paper/ICRA/PhaseOffsetSwarm_Detailed_Proposal_2026-08-05.md`：
   - Section 9；
   - Section 10.1–10.6；
   - Section 13.5–13.7；
   - Section 17；
5. 当前实现：
   - `local_sensing.cpp`；
   - `SDFMap::initMap/cloudCallback/updateESDF3d/bufferRefreshCallback`；
   - Kino A* 和 B-spline 的地图 consumer；
   - `TubeCrossSectionSolver`；
   - `TubeBuilder`；
   - `TubeFilter`；
   - `TubeEpochManager`；
   - `PhaseOffsetRuntime`；
   - matched adapter、raw bridge、candidate diagnostics、Marker helper。

禁止把后来的 G2b raw-log-odds 假设当成 proposal 原文。

---

## 4. 必须保持的正确架构

以下实现必须保留：

- planned `ContinuousPhasePath` -> `PathDifferentialState`；
- `GeometryEvaluator` 产生 `p,N,N_w,curvature`；
- `+N/-N` 独立搜索；
- 非对称 interval；
- one-sided interval 可为几何有效；
- `lower > upper` 才是横截面为空；
- 正负 curvature 的不同单侧约束；
- `TubeFilter` 的保守 C1 子集；
- candidate/active profile 分离；
- active epoch 只在 material install 时变化；
- Candidate 与 Certified Marker 分 topic；
- Certified 的 failure-latch DELETE 语义；
- fixed tube；
- matched port、100-cycle zero-port gate；
- 裸 launch `disabled/none` baseline。

不要重写 G1/G2a/R1/R2 已验证的纯 C++ 数学和 ownership。

---

## 5. 明确禁止继续使用的生产地图合同

ESDF-selected cloud mode 不得再使用以下判断：

```cpp
rawOccupancyStorageReady(map)
```

vector 已分配不表示 cloud observation 已写入 raw occupancy。

生产 adapter 不得再由以下 raw layer 构造 tube：

```cpp
SDFMap::getOccupancy()
SDFMap::isUnknown()
occupancy_buffer_
RawOccupancySelfFreeSeed
```

本阶段结束时，cloud-mode production path 中不得存在 self-free seed。它不能把错误 backing 修成可观测地图。

旧 raw bridge 可以删除，或保留为未接线、明确 deprecated 的测试参考；但不得继续链接到 `phase_offset_matched_adapter` 的 production query 路径。优先删除生产接线和自欺性的 readiness 判断，避免两套合同同时活跃。

---

## 6. 新 snapshot 的职责和边界

新增一个单一职责、不可变的 cloud occupancy snapshot。命名可根据现有风格微调，例如：

```text
plan_env/cloud_occupancy_snapshot.h
plan_env/cloud_occupancy_snapshot.cpp
```

或等价清晰名称。

Snapshot 必须至少包含：

```cpp
struct CloudOccupancySnapshot {
  bool valid;
  uint64_t observation_sequence;
  ros::Time observation_stamp;
  Eigen::Vector3d map_min;
  Eigen::Vector3d map_max;
  Eigen::Vector3d observed_min;
  Eigen::Vector3d observed_max;
  Eigen::Vector3d grid_origin;
  Eigen::Vector3i voxel_count;
  double resolution;
  double included_map_inflation;
  std::vector<uint8_t> occupied;
};
```

字段可适配工程风格，但以下语义不可改变：

1. snapshot 是不可变值对象或 `shared_ptr<const ...>`；
2. 一次 tube build 的所有 query 必须来自同一个 snapshot；
3. snapshot sequence 来自真实 cloud observation 更新，不是 tube attempt count；
4. 首次有效 odom 和 cloud 前 snapshot invalid；
5. 空 PointCloud2 也是一次有效 observation；
6. local_sensing 的 observed domain 是当次 odom 中心的 axis-aligned local update box，与其发布逻辑一致；
7. snapshot 只表达 environment occupancy，不包含 tube margin、planner safe distance 或 controller limit；
8. snapshot 不允许保存 Runtime、TubeBuilder、publisher 或控制状态。

优先在 `SDFMap::cloudCallback` 中由当次 cloud 和当次 camera position 构建局部 snapshot，然后通过小锁原子替换 immutable pointer。不要让 TubeBuilder 在 8-thread `AsyncSpinner` 下直接遍历正在修改的 SDFMap vector。

新 snapshot 的构建不得改变原 planner 使用的 `occupancy_buffer_inflate_`/ESDF 行为。本阶段要保持单机 baseline；snapshot 是只读旁路证据，不是 planner map 重写。

若需要复用同一 inflation voxelization，应抽取小型纯 helper，避免复制两份公式；但禁止顺手重构整个 SDFMap。

---

## 7. Cloud snapshot 的查询语义

新增 ROS integration query，例如：

```text
phase_offset_cloud_occupancy_query.h/.cpp
```

它把 immutable snapshot 转为 navigation 的 categorical environment query。

查询顺序必须是：

1. snapshot invalid -> `UNAVAILABLE`；
2. point 非 finite 或全局 map 外 -> `OUT_OF_MAP`；
3. point 在全局 map 内、但在本次 observed AABB 外 -> `UNKNOWN`；
4. point 对应 snapshot inflated occupied voxel -> `OCCUPIED`；
5. observed AABB 内其余 voxel -> `KNOWN_FREE`。

这份 `KNOWN_FREE` 的依据是本阶段明确采用的 occupycloud 合同：

> 输入 cloud 表达当前 local observed box 内完整的环境障碍集合；box 内未占据位置作为 cloud-map 模式下的自由空间。

仿真 `local_sensing` 满足这个合同，因为它从完整 pillar PCD 中裁剪整个 AABB，而不是只生成视锥中的一部分射线。

实机 topic 若只是稀疏瞬时 hit cloud、没有完整 local occupancy-domain 语义，则不能直接启用该合同；应 fail closed 并由未来专用 producer 提供 observed mask。本阶段只正式验收原 pillar/local_sensing 管道。

---

## 8. Inflation 与 margin accounting

详细 proposal 的四项总鲁棒半径是：

```text
r_uav             = 0.25 m
e_map             = 0.10 m
e_loc             = 0.05 m
e_track           = 0.15 m
--------------------------------
r_eff_full        = 0.55 m
```

当前 cloud map：

```text
resolution              = 0.10 m
obstacles_inflation     = 0.099 m
ceil(0.099 / 0.10)      = 1 voxel
```

本阶段必须把这一个约 0.10 m voxel inflation 明确记作已经包含的 map uncertainty，不得再减完整 0.55 m。

生产 cross-section 使用：

```text
included_map_uncertainty = 0.10 m
residual_effective_radius
  = r_uav
  + (e_map - included_map_uncertainty)
  + e_loc
  + e_track
  = 0.45 m
```

代码必须显式表达 full、preincluded 和 residual 三个事实，禁止直接把默认 margin 魔改成 0.45 而不记录来源。

建议扩展 `RobustTubeMargins`：

```cpp
double preincluded_map_uncertainty = 0.0;
double fullEffectiveRadius() const;
double residualEffectiveRadius() const;
```

要求：

- `0 <= preincluded_map_uncertainty <= map_uncertainty`；
- snapshot 声明的 `included_map_inflation` 必须不小于配置的 preincluded 值，否则 query/config fail closed；
- `TubeCrossSectionSolver` 的 obstacle erosion 使用 residual radius；
- diagnostics 同时保留 full radius、preincluded radius 和 residual radius；
- 旧测试默认 `preincluded=0`，保持原 uninflated analytic contract；
- 新测试覆盖 `full=0.55, preincluded=0.10, residual=0.45`。

不要把以下值算作 map inflation：

- Kino A* margin；
- B-spline `safe_distance=0.4`；
- B-spline `lambda3=10`；
- collision threshold；
- fixed delta limit。

它们不修改 occupancy backing，不能参与 tube erosion subtraction。

旧六字段 0.70 m legacy erosion 不得进入新的 cloud snapshot production path。保留它只允许用于 legacy isolated tests，且必须明确不参与正式 ESDF-selected ROS 路径。

---

## 9. Adapter 与 epoch 接线

matched adapter 的 ESDF-selected update 必须改为：

```text
SDFMap 获取 immutable cloud occupancy snapshot
        ↓
snapshot sequence/stamp 作为地图 observation 身份
        ↓
make cloud occupancy query(snapshot)
        ↓
TubeEpochManager::update(... categorical query ...)
```

具体要求：

1. 禁止 `rawOccupancyStorageReady()`；
2. 禁止 self-free seed；
3. `map_observation_sequence` 必须使用 snapshot sequence，不得每次 tube attempt 自增冒充地图版本；
4. 同一个 epoch update 的 candidate build、current cross-section diagnostics 和 debug probe 必须使用同一 snapshot；
5. snapshot 没更新但 period 到期时可以做 equivalent refresh；不得伪造新 map revision；
6. source/path revision 与 map observation sequence 继续分开；
7. fixed/none 不获取 snapshot；
8. adapter 仍只做转换、调度和发布，不放入 cross-section 数学。

为保持既有 83/epoch/raw-diagnostic schema，不得移动或重解释旧索引。若必须增加 snapshot diagnostics，只允许追加到独立的新 topic/schema；优先复用现有 epoch/raw diagnostic 中尚能准确表达的字段。本阶段不得破坏现有数组长度断言。

原 `raw_cross_section_path_used` 字段可暂时保留数值兼容，但注释必须说明它现在表示 asymmetric categorical environment path，不再表示 SDFMap raw log-odds backing。

---

## 10. Path/tube 采样修复

当前 adapter 实际按 `preflight_sample_step_w` 采样，`tube.sample_step_w` 只做配置校验而未驱动 tube sampling。

本阶段授权修复为：

```cpp
tube_sample_step = min(0.10,
                       config.tube.sample_step_w,
                       config.preflight_sample_step_w);
```

或语义等价实现。

要求：

- tube sample 间隔不得大于 `tube.sample_step_w`；
- preflight 可以复用更密采样，但不能让更粗的 preflight 参数覆盖 tube 参数；
- 不修改 `ContinuousPhasePath` 数学；
- 添加 adapter 单测，证明改变 tube sample step 会改变 candidate preview sample density。

---

## 11. 当前阶段明确不做的 hybrid 内容

本阶段不得实现：

- proposal 10.6 完整 `U+` / `U>=0` 动态可行集；
- 新 QP；
- new CBF；
- Runtime 状态机重写；
- planner replan trigger 接线；
- one-sided tube 的自动中心线重规划；
- A6 continuation；
- terminal mode；
- swarm/neighbor。

`dynamic_feasibility_evaluated=false` 继续保持，并在最终报告中明确 A5 仍未完成。地图合同通过后，下一独立阶段才处理 hybrid feasibility 和 WAITING/SAFETY_PRIORITY 执行语义。

不得为让本阶段 ROS 通过而修改 `UNCERTIFIED_RECENTER`、failure latch 或 Runtime projection。

---

## 12. 建议白名单

只允许修改以下概念范围。执行前应列出精确路径；如需要白名单外文件，停止并报告，不得自行扩张。

### plan_env

- `src/swarm_planner/plan_env/CMakeLists.txt`
- `src/swarm_planner/plan_env/include/plan_env/sdf_map.h`
- `src/swarm_planner/plan_env/src/sdf_map.cpp`
- 新增单一职责 cloud snapshot header/source
- 新增对应测试

### phase_offset_navigation

- `include/phase_offset_navigation/tube_cross_section.h`
- `src/tube_cross_section.cpp`
- 对应 cross-section/builder/epoch tests，仅在 margin accounting 需要时修改
- `README.md` 仅更新地图合同说明

禁止修改 Geometry、MatchedPort、PortProjector、Runtime 和 TubeFilter，除非编译所需且先停止报告。

### bspline_race integration

- `CMakeLists.txt`
- `include/bspline_race/integration/phase_offset_matched_adapter.h`
- `src/integration/phase_offset_matched_adapter.cpp`
- 新增 cloud occupancy query header/source/test
- raw bridge header/source/test：只允许删除生产接线、标记 deprecated 或从 target 移除
- raw candidate diagnostics：只允许把 query 来源改成同一 snapshot，并保持 schema
- matched adapter/epoch integration tests
- `launch/test_gvf.launch`：只允许追加显式 cloud-contract/inflation-accounting 参数；原默认 `disabled/none/refresh=3.0` 不得改变

禁止修改：

- `gvf_manager.cpp` orchestration；
- planner/A*/B-spline/C2；
- `gvf.cpp`；
- `phase_offset_core`；
- `PhaseOffsetRuntime`；
- Marker helper；
- governor/SO3；
- map PCD；
- speed/K1/K2/saturation；
- messages/swarm/CBF。

如果 snapshot 接线确实无法在不修改 `gvf_manager.cpp` 的情况下完成，先停止并报告最小所需接口；不得直接编辑。

---

## 13. 必须添加的纯 C++/单元测试

至少覆盖：

### Snapshot

1. 无 odom -> invalid；
2. 无 cloud -> invalid；
3. 空 cloud + valid odom -> valid all-free observed AABB；
4. AABB 内 cloud point 经一格 inflation 后 occupied；
5. AABB 内非 occupied -> known free；
6. AABB 外、全局 map 内 -> unknown；
7. 全局 map 外 -> out of map；
8. sequence 每个 cloud observation 单调增加，包括 empty cloud；
9. 新 cloud 发布后旧 immutable snapshot 内容不变；
10. included inflation 和 resolution finite、准确；
11. 非 finite input fail closed。

### Margin accounting

1. `0.25+0.10+0.05+0.15=0.55`；
2. preincluded 0.10 -> residual 0.45；
3. preincluded <0、>map uncertainty、NaN/Inf -> config invalid；
4. preincluded=0 保持原 analytic raw test；
5. inflated clearance + residual erosion 与对应 raw clearance + full erosion 等价；
6. planner safe distance/lambda 不参与公式。

### Cross section

1. `+N/-N` 独立；
2. 非对称例子；
3. one-sided interval 保持 valid；
4. `lower>upper` 才 empty；
5. unknown 仅截断对应 ray；
6. curvature 正负符号回归；
7. open snapshot 不受 legacy `max_offset=0.20` 限制；
8. search extent 仍是 certification horizon，不是 controller limit。

### Adapter

1. ESDF-selected cloud mode 不访问 raw occupancy API；
2. 不创建 self-free seed；
3. snapshot invalid -> candidate incomplete/fail closed；
4. snapshot valid -> G1 asymmetric path；
5. epoch map observation sequence 等于 snapshot sequence；
6. 同 snapshot 重建不伪造 map revision；
7. fixed/none 不请求 snapshot；
8. tube sample step 真正控制 preview 密度；
9. candidate/active Marker helper输入语义不变；
10. failure/certificate 既有测试全部通过。

---

## 14. 构建与回归

必须运行：

```bash
catkin_make -j8
```

以及：

- 新 snapshot tests；
- 新 cloud query tests；
- tube_cross_section；
- tube_builder；
- tube_filter；
- tube_epoch_manager；
- runtime（确认未被破坏）；
- core 全回归；
- matched adapter；
- raw/candidate diagnostics（若仍保留）；
- tube marker；
- bspline_race 全部正式测试；
- plan_env 全部正式测试；
- `catkin_test_results --verbose`。

测试前清理本任务生成的旧 test XML，不能把历史 XML 当作新证据。不得删除用户文件。

既存 `uav_utils` 缺 XML 若仍出现，只记录，不越权修复。

---

## 15. ROS 验收

先检查现有 ROS master/进程，不得连接、停止或复用用户进程。所有测试使用隔离端口，并只清理本任务启动的进程。

### ROS-0 baseline

裸 `test_gvf.launch`：

- 实际参数必须为 `disabled/none/refresh=3.0`；
- phase-offset manual/candidate/certified/snapshot diagnostics 均无 publisher；
- `/position_cmd` 唯一 publisher 为 `/formation_planning`；
- 原 pillar 点到点目标正常到达；
- local_sensing 和 planner baseline 行为不变。

### ROS-1 fixed regression

显式 manual/fixed：

- 100-cycle gate 行为不变；
- Candidate 和 Certified 继续正确显示；
- failure/fallback/tube violation=0；
- matched residual 数值精度量级；
- 新 cloud snapshot 生产路径不应被 fixed mode 调用。

### ROS-2 cloud occupancy tube

显式 manual/ESDF-selected，并显式启用：

```text
cloud_obstacle_set_complete=true
preincluded_map_uncertainty=0.10
```

只使用原 `pillar.pcd`、原 `local_sensing`、空 manual obstacle 文件和未启用 static-preinflated layer。

必须记录原始 bag、CSV、JSON/report、命令、参数 dump 和分析器哈希。

必须证明：

1. local_sensing cloud messages 正常，包括 sequence/stamp；
2. raw `occupancy_buffer_` 仍可保持 UNKNOWN，而 candidate 仍由 cloud snapshot 成功构造；
3. `raw_storage_ready`/self-free 不再参与生产；
4. snapshot query 使用 inflated occupancy；
5. full/preincluded/residual margin 分别为 0.55/0.10/0.45；
6. candidate 至少出现稳定 ADD 窗口；
7. candidate 宽度在开阔横截面明显超过旧 legacy 0.40 m 总宽上限；
8. 左右 clearance 不被强制对称；
9. candidate/active actions 与 diagnostics 对齐；
10. snapshot sequence 只在 cloud observation 时变化，不随 tube attempt 虚增；
11. 同一 build 的所有 debug/diagnostic query 使用同一 sequence；
12. current path、actual tracking、matched residual、failure/fallback 均正常；
13. 不因 buffer refresh attempt 产生无依据的 ADD/DELETE 闪烁。

若原 pillar 实际几何使 retained delta 不满足 active install，可以接受 Candidate ADD + Certified DELETE，但必须证明：

- candidate 是完整、非对称、来自正确 snapshot 的几何 tube；
- Certified DELETE 的原因是 retained/current hybrid 条件，而不是 UNKNOWN raw backing；
- 不得通过调 planner 权重、目标、速度或 margin 绕过。

本阶段不要求 15 秒 certified rolling，也不宣称 A5 完成。

---

## 16. 依赖边界检查

必须搜索并报告：

```text
phase_offset_core: ROS / SDFMap / visualization / planner / swarm / CBF
phase_offset_navigation: ROS / SDFMap / plan_env / PointCloud2 / publisher
snapshot helper: Runtime / TubeBuilder / Port / planner / control
integration: PositionCommand / position_cmd / SO3 / swarm / neighbor / CBF
```

要求：

- navigation 继续纯 C++/Eigen；
- plan_env snapshot 不依赖 navigation；
- adapter 只翻译 snapshot 到 navigation query；
- 无 PositionCommand 发布逻辑进入 integration；
- 不新增 common_msgs 消息。

---

## 17. Stop conditions

出现以下任一情况立即停止：

1. 需要修改 whitelist 外文件；
2. 必须改变 planner/A*/B-spline/C2 才能让 candidate 出现；
3. 必须改变 map inflation、速度、K1/K2、saturation 或目标；
4. snapshot 无法在不改变 baseline planner map 行为的情况下实现；
5. real cloud contract 无法从仓库内证明；
6. ROS runtime 不可用；
7. candidate 仍因 raw UNKNOWN 失败；
8. margin accounting 无法给出唯一、可测试的 full/preincluded/residual；
9. existing user process 会被影响；
10. acceptance 需要完整 hybrid dynamic feasibility、A6 或 swarm。

停止时保留工作树，报告准确 blocker，不得扩大范围。

---

## 18. 最终自审核与报告格式

完成后必须：

1. `git diff --check`；
2. `git status --short`；
3. staged 内容必须为空；
4. branch/HEAD/stash 保持；
5. 前后 SHA/mtime manifest 精确比较；
6. 列出所有修改/新增/删除文件；
7. 证明无 whitelist 外修改；
8. 报告每个测试数量和结果；
9. 报告每个 ROS 端口、bag、CSV、JSON、分析器 SHA；
10. 报告 snapshot sequence、query status、margin accounting、Candidate/Certified 行为；
11. 明确说明未修改 planner、C2、Runtime hybrid、速度、地图参数、tube geometry、MatchedPort；
12. 明确停止语句。

最终报告必须以以下结论边界结束：

```text
A5-G2d 完成/blocked。
本阶段仅纠正 cloud occupycloud -> tube environment snapshot 地图合同。
未实现完整 tube hybrid dynamic feasibility。
未进入 A5-G2e、A5.3 或 A6。
A5 仍保持 blocked，等待独立下一阶段执行规范。
```

