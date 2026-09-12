# M4D：Section 走廊截面裁剪（柱阵解冻）

日期 2026-09-11。用户确认语义：在 tube 容许范围就按意图走，容不下就按最大边界走；
并明确选择方案 B：Section 路径不再逐 tick 验证可达包络一致性，改为直接把横向指令
裁进走廊截面。

## 1. 柱阵为什么停住（打点结论）

柱阵场景三台机停在 y≈13.8，逐 tick 的拒绝原因按顺序有三层：

1. 相位速率锁死。base_w_dot≈2.36，而 launch 的 upper_nu=2.0；横向权限只有
   u_w=±0.12，压不回窗口内。参考相位被永久冻在同一个 w（实测一直是 w=6.847），
   飞机既不前进也不收敛。论文用的窗口是 3.0。
2. 假的 RATE_INFEASIBLE（1 ULP）。收缩率判据拿向外取整的天花板斜率比向下取整的
   地板阈值：max_slope=0.083333333333333343 vs allow_slope=0.083333333333333329，
   只差 1 个 ULP，整条本来刚好可通的走廊被判成不可通行。旧几何评估器一直用
   boundary_tolerance 吸收这个取整误差，严格 PWL 核心继承了构造、没继承裕度。
3. 逐 tick 包络一致性过严。held-step 要求 δ 在每一步都不越出限速可达包络。实测 δ
   追一条正在移动/收窄的走廊时滞后只有 0.18 mm ~ 5.8 mm，而且上一刻 δ 仍在以
   −0.077 外移、速率限幅一 tick 只能改 0.024，物理上转不回来；走廊末端还会收成
   零宽（common=[0,0]）。判据把每个 tick 都否掉，参考相位不前进，飞机停住。

## 2. 本批改动

修复层（两类真 bug）：

- tube_viability.cpp（EvaluateStrictPwlCore）：收缩率比较加入取整裕度
  delta_rate + max(boundary_tolerance, 16 ULP)，与几何评估器口径一致；1 ULP 的伪
  RATE_INFEASIBLE 归零。
- launch：phase_offset_normal_preview_upper_nu 默认 2.0 改为 3.0（对齐论文）。

方案 B（Section 走廊截面裁剪）：

- phase_offset_allocator.cpp
  - 新增 SectionCorridorAt()：走廊截面（TubeViabilityKnot::geometric，即 tube 代表
    的横向安全集合）在给定 w 的线性插值。
  - allocate()：把横向速率窗口与“下一 tick 仍留在走廊截面内”的区间求交；走廊需求
    与横向窗口无交集时取窗口内最靠走廊的端点，并在与 slew 限幅无交集时取 slew
    边界上最靠需求的值，都不再丢 tick。
  - PreviewValidForAllocation()：接受 CURRENT_DELTA_OUTSIDE（走廊收窄后 δ 落到
    外面），由上面的恢复分支把它拉回。
  - FinalCommandValid：仅对被裁剪掉的那个硬边界放行，其余边界照旧。
- phase_offset_runtime.cpp（prepareSection）
  - 删除逐 tick TubeViability::checkHeldStep 一致性验证，改为直接推进
    next_w = w + dt*w_dot、next_delta = delta + dt*u_delta（由上游裁剪保证在走廊内）。
    相位速率窗口、切向速度下限、ZOH、provenance、slew/幅值等基本检查保留。
- tube_viability.cpp（EvaluateSection）：section_profile 在所有 feasible 状态下都
  借用（不再只限 FEASIBLE），否则 rate-degraded / δ 在外的 tick 会被身份闸门否掉。

保留未动：checkHeldStep 本体（V2 链路仍在用）。

## 3. 验收

- 单元测试：phase_offset_allocator_test 46/46、phase_offset_runtime_test 14/14、
  phase_offset_tube_viability_test 42/42、phase_offset_section_tube_test 32/32。
  迁移 1 条把旧 1-ULP 行为写死的用例（CriticalNonuniformRateStateMatchesV2：该状态
  实际 feasible，Section preview 应当借用 profile）。
- 柱阵 sim_b_formation_3.yaml + pillar.pcd：prepareSection 拒绝 0 次，
  final_cmd_source valid:hold 约 391:533，三台机从冻在 y≈13.8 变为穿越柱阵到
  y≈−7.1/−9.0/−9.2。
- 开放地图同一场景：三台机到达 y≈−9.99（目标 −10），与既有基线一致。

## 4. 柱阵停在离目标 1~3 m 处的真正原因：目标点在障碍膨胀里（已修）

柱阵里三台机一度停在离目标 1~3 m 处。根因不是 tube，而是**目标点选址**：把
`pillar.pcd` 按 sdf_map 参数（0.2 m 分辨率、0.099 m 膨胀）栅格化后，原来 y=-10.0
这一行只有 x=-1 是空的，x=0 和 x=+1 都落在柱体膨胀里（离柱面分别只有 0.362 m /
0.146 m）。目标点在膨胀里时，`computeShotTraj` 的末段直连永远不通过，
`kinodynamic_astar.cpp` 的 near_end 分支又没有父节点可退回，于是打印 `no path`
（柱阵 278 次、开放地图 0 次），前端路径到不了终点，飞机停在半路。

对照实验：只把目标行换成自由行 y=-9.0（其余不动），三台全部到达且 `no path` 归零，
直接证明 tube 这条链路是干净的。

**修法（方案 1：改目标行）**，按"三台目标点全部自由 + 距最近柱面净空最大"选行：

| 场景 | 原目标行 | 新目标行 | 最小净空 |
|---|---|---|---|
| sim_b_formation_3.yaml | y=-10.0（x=0,+1 在膨胀里） | y=-11.2 | 0.84 m |
| sim_b_formation_wide_3.yaml | y=-10.0（x=-5,0 在膨胀里） | y=-12.6 | 0.66 m |
| sim_b_single_lane.yaml | y=-10.0（x=0 在膨胀里） | y=-8.6 | 1.29 m |
| sim_b_open_3.yaml | 起点 x=-5 在膨胀里 | 整列 shift 到 x=-7.4/-6.4/-5.4 | y=-10 与 y=+2 都自由 |

`sim_b_conflict_3.yaml` 检查后本来就全自由，未改。

**复验**（柱阵 + 修好的 `sim_b_formation_3.yaml`）：`prepareSection` 拒绝 0 次、
`no path` 0 次，三台到达 y=-11.196 / -11.282 / -11.316（目标 -11.2）。

## 5. 相对冻结规格的偏离（记录）

1. 严格 PWL 核心的收缩率比较加入取整裕度（M3C 原为精确比较）。
2. Section 执行路径不再调用 checkHeldStep，改为走廊截面裁剪（M4C 曾把该类情形列为
   本批仍 fail-closed，本批按用户选择改为裁剪）。

## 6. 附：分布式共享点击（一次 RViz 点击 = 全队平移）

问题：本工程是 N 个独立 agent，每个 agent 的 launch 把 `/move_base_simple/goal`
remap 成自己的 `/uav_<id>/goal`，于是 RViz 的 2D Nav Goal 默认话题**没有任何订阅者**，
点一下没反应；即使对上话题也只能到一台。

参考 SUPER/SPH-planning：它们的 `plan_manager::goalCallback` 用
`goal_i = init_i + clicked + bias` 一次算出全队目标再广播——但 SPH 本身是**集中式**
的（一个 `swarm_planning_3d` 节点规划全队），不能照搬到分布式架构。

本批按分布式、无队长的做法落地：

- 新增 `scripts/clicked_goal_translator.py`，**每个 agent 各跑一份**；所有实例订阅
  **同一个**全局点击话题，各自用**只属于自己的** `initial_i` 本地套用同一条公式
  `goal_i = initial_i + bias + clicked`，再发到自己的 `/uav_<id>/goal`。没有协调节点、
  没有 fan-out、没有 leader，各实例可互换、算式完全相同。
- `bias` 是**一个全局共享常量**（场景自身的公共 start->goal 平移向量，由 orchestrator
  从场景算出并传给每个 agent），所以“点原点 = 复现场景目标”，点别处 = 整队再平移。
- 节点放在 agent launch 里 **remap group 之外**，这样它保留对全局点击话题的订阅；
  规划器对 `/uav_<id>/goal` 的订阅不变，自动场景目标照旧可用。

实测（`publish_goals:=false`，只往 `/move_base_simple/goal` 发**一次**点击 (0,0)）：

| | uav0 | uav1 | uav2 |
|---|---|---|---|
| 点击前 | (−1.000, 20.000) | (0.000, 20.000) | (1.000, 20.000) |
| 点击后 27 s | (−1.238, −11.124) | (0.218, −11.105) | (1.151, −11.318) |

三个 translator 实例在同一次点击里各自打出
`click 1 -> goal (-1.000, -11.200, 0.000)` / `(0.000, ...)` / `(1.000, ...)`，规划器
同时收到三条目标。三架按原队形一起到达 y≈−11.2。

单机基线不受影响：`test_gvf.launch` / `phase_offset_esdf_tube_single.launch` 不含
`phase_offset_agent.launch`，也没有这个共享点击话题。

## 7. 附：localsensing 与栅格对齐单机 baseline

对比两份工程（`local_update_range_*` 是**半范围**，`local_sensing.cpp` 注释与代码均如此：
`|point.x - pos.x| <= local_range.x()`）：

| 参数 | 单机 baseline | 本工程 swarm（改前） | 本工程 swarm（改后） |
|---|---|---|---|
| `local_update_range_x/y` | 4.0 / 4.0 | 6.0 / 6.0 | **4.0 / 4.0** |
| `local_update_range_z` | 2.5 | 2.5 | 2.5 |
| 感知体 | 8 × 8 × 5 m | 12 × 12 × 5 m | **8 × 8 × 5 m** |
| `sdf_map/resolution` | 0.1 | 0.20 | **0.10** |
| `gvf/local_update_range_*` | 3.0/3.0/2.05 | 3.0/3.0/2.05 | 同 |
| `obstacles_inflation` | 0.099 | 0.099 | 同 |

**分辨率不只是精度问题**：`SDFMap::inflatePoint` 是 `±ceil(inflation/resolution)` 格的
立方膨胀，所以 0.099 m 膨胀在 0.2 m 栅格上实际是 **±0.20 m**，在 0.1 m 上是 ±0.10 m。
按两种分辨率把柱阵栅格化后：

| 目标点 | res = 0.2 | res = 0.1 |
|---|---|---|
| (−1, −10) | free | free |
| (0, −10) | **OCC** | free |
| (1, −10) | **OCC** | free |
| (−5, −10) / (5, −10) | OCC / free | free / free |

所以第 4 节里"目标点落在膨胀里 → A\* no path"的**真正原因是 swarm 的 0.2 m 分辨率**，
不是目标行本身。改回 0.1 m 后：

- 目标行恢复成原来的 **y = −10.0**（30 m SPH 式平移）；
- 实测柱阵 `sim_b_formation_3.yaml`：三架到达 y = −10.005 / −9.870 / −10.095，
  `prepareSection` 拒绝 0 次、`no path` 8 次（0.2 m 时是 130~278 次、且有飞机停在半路）。

### 7.1 每个 agent 的地图 = 单机口径（方案 1）

`sdf_map` 的原点写死为 `(-size_x/2, -size_y/2, ground)`，**不会跟随飞行器**，所以
"飞多远"直接决定"地图必须多大"。参考 SPH-planning 自己的配置也遵循同一规则：
它起点 y=18、感知半范围 7.0，地图半高正好是 25 = 18 + 7。

据此把 swarm 拉回单机口径：

- 场景起点行 y = 20 → **y = 9.0**（三套 formation 场景的列在 0.1 m 栅格下都自由，
  最小净空 0.37 m），终点仍为 y = −10 → 走廊 **19 m**（SPH 自己是 18 m）；
- swarm `map_size` 60×80 → **20×30×2.5**，`map_resolution` 0.20 → **0.10**，
  `local_update_range` 6.0/6.0 → **4.0/4.0** —— 与单机 baseline 完全一致。

实测（柱阵 `sim_b_formation_3.yaml`）：

| | 改前（60×80 @0.2） | 改后（20×30 @0.1，单机口径） |
|---|---|---|
| 每 agent RSS | 2.0 GB | **0.43 GB**（4.7 倍↓） |
| 三架终点 | 有架停在 y≈10 | uav0 −9.97 / uav1 −9.99 / **uav2 −9.34** |

### 7.2 残留

起点现在落在柱阵内部（y=9，柱阵覆盖 ±13.7），因此仍有 `no_path`（124 次）并且
uav2 停在离目标 0.7 m 处。把 `map_size_y` 放到 40（~0.52 GB/架）**没有改变**这个数
（同样是 124 次、uav2 同样差 0.7 m），说明它来自"起点在柱阵里"，不是地图容量。
注意 `no_path` 都出现在目标发布之后（日志行 2263~4347，目标发布在行 ~913），不是
启动期的空目标导致。

要根治需要给 `sdf_map` 加**可配原点 / 跟随位姿滚动**：那样就能用一张小地图覆盖
"起点在柱阵外 + 长走廊"，同时保持单机量级的内存。
