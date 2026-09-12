# M4B：C2 前端映射锚点对齐（消除“向后搭桥”）

日期 2026-09-10。状态：已冻结待执行，用户已批准。
依据：`docs/Horizontal_Section_Tube_M4A_Report_2026-09-10.md` 第 6.8 节。

## 1. 问题

集群 3 机 30 m 编队航线上，相位在部分时刻被判“没有可行命令”并 hold，原因不是
走廊（preview 一直 FEASIBLE），而是**相位速率窗口为空**：连接段内部
`‖dp/dw‖` 塌到 0.28~0.47，而窗口上限只允许
`upper_nu + u_w_abs_max = 2.12`。

剖面对照实验（M4A 6.4）证明凹坑只由“w 跨度与连接段自身弧长不匹配”产生；
6.6 的容差+向上扩展把它从 0.28 缓解到 ≥0.47，三机从“冻在 y≈14.3、只飞
3~5 m”变成“2/3 台跑完 30 m”。残余停顿来自**第二个、独立成因**。

## 2. 根因（已实测）

接缝处旧路径切向与新样条切向**夹角恒为 0.0°**，但新样条的端点经常落在接缝
点的**后方**：`dot(p_new − p_old, t_old)` 中位数 −0.49 m，87 次里 59 次为负。
两端都要求向前切向、而净位移向后 ⟹ 五次 Hermite 只能在跨度内绕环，环内
`‖dp/dw‖` 必然塌陷。任何跨度都无法修复（判据层面已验证：安装全部失败）。

原因是 H2 future seam 的**映射锚点错位**：`buildPhaseC2Frontend` 把候选样条
映射到 `[future_switch_w, semantic_path_end_w]`，即让它的 w 起点等于“当前相位
前方的未来接缝”；但样条的**几何**起点是规划器锚点（≈飞机当前位置，对应
`phase_at_switch`）。两套 w 参考相差约 0.4 w，于是连接段要向后搭桥。

基线没有这个问题：`buildPhaseV2C2Frontend` 从 `phase_at_switch` 映射样条，
几何起点与 w 起点一致，端点始终有序向前。

## 3. 本批改动（唯一一项）

`src/swarm_planner/bspline_traj/src/gvf_manager.cpp` 的
`gvf_manager::buildPhaseC2Frontend`：

- 把候选样条的映射起点从 `future_switch_w` 改为“与样条几何锚点一致的相位”，
  即 `mapped_start_w = min(future_switch_w, phase_at_switch)`；
- 其余全部不变：prefix 区间（`[phase_at_switch, future_switch_w]` 的旧路径切片）、
  连接段区间 `[future_switch_w, join_w]`、样条使用区间
  `[join_w, semantic_path_end_w]`、边界误差检查、采样、避障、cost 与
  6.6 已验收的“自洽跨度（10% 容差 + 向上扩展到 2.5 w）”选择逻辑都不动。
- 在改动处写明注释：w 锚点必须跟随样条自身的几何起点，否则连接段会向后搭桥。

## 4. 不做

- 不改 `future_switch_w` 的选择（H2 的 handoff 语义、`plannerOnlyFutureSeam`）。
- 不改 `phase_at_switch`、phase 推进、prefix/consumer 事务。
- 不改 allocator、preview、tube 构建、`upper_nu` 或任何速率窗口参数。
- 不改基线数学（`buildGlobalPhaseSamples`、`makeMappedBspline` 内部）。
- 不新增门控、证书、地图 ID、reserve、worker。

## 5. 验收

1. 隔离构建 `formation_planning` exit 0。
2. 打点（临时、验后移除）：`dot(p_new − p_old, t_old)` 应变为非负；
   `connector_success > 0`、`connector_failed` 显著下降。
3. 隔离 3 机 `sim_b_formation_3`（起点 (0,20)、1 m 间距、30 m 航线）：
   三机应全部到达 y ≈ −10；`hold` 应集中在起飞/终点，而不是中段。
4. 回归：`ContinuousPhasePath.CompositeRetainsOldPrefixAndC2Join`、
   `GvfH2FutureSeam.*`、`PhaseOffsetSectionInput` 的接缝用例、M3C 的
   manager/adapter 用例。
5. 结果写入 M4A 报告新增小节。

## 6. 回滚

单文件、单处改动。回滚即恢复 `future_switch_w` 作为映射起点。
不使用 `git reset --hard` / `checkout --` / `restore .` / `clean`。
