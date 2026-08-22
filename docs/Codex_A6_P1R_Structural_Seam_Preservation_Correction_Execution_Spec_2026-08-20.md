# A6-P1R Structural-Seam Preservation Correction

日期：2026-08-20  
工作区：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
执行分工：主代理负责规格、白名单、独立审查与验收；
`gpt-5.6-terra / xhigh` 负责白名单内实现和首轮测试。  
`AUTO_ADVANCE=true`，但仅限本文件定义的 A6-P1R。完成、自审并停止后，
才允许由新的 A6-P2 规格授权 bootstrap first-false 工作。

## 1. 终审发现

A6-P1 已修复连续 off-grid `current_w` 未进入 owner-aligned preview 的问题，
但其 canonicalization 仍有一个未覆盖边缘：

```text
owner structural seam = ws
0 < abs(current_w - ws) <= TubeBuilder current-match tolerance (1e-9)
current_w != ws
```

当前 helper 会删除所有位于 current-anchor tolerance 内的 knot，再插入精确
`current_w`。如果被删除的是 immutable owner segment seam，最终相邻 cell 可能从
一段 owner 延伸到另一段 owner。`ContinuousPhasePath::cellBounds()` 的证明单位是
单个 owner segment；数值 domain epsilon 不是跨 segment 几何证明，不能用它把 seam
吞掉。

这不是已观察到的动态失败，也不授权修改 Tube 几何或放宽 tolerance；它是 A6-P1
终审发现的结构 partition 完整性缺口。

## 2. 修正后的 canonical precedence

owner-aligned preview 的等价类优先级必须是：

```text
immutable owner structural seam
  > exact required current phase
  > ordinary supplied planner knot / non-structural range-neighbour knot
```

具体合同：

1. current 与普通 supplied knot 接近时，仍保留精确 `required_current_w`；
2. current 与 preview start/end 接近、但该 boundary 不是 owner seam 时，仍保持
   A6-P1 的精确 current 语义；
3. current 与 owner seam 完全相等时，只保留该 exact seam/current knot；
4. current 与 owner seam 不完全相等、但位于 TubeBuilder current-match tolerance 内时，
   必须保留 exact seam，不得用 current 替换它；
5. 第 4 项中 TubeBuilder 可用其冻结的 nearest-current match 选择 seam；真实 current
   位于以 seam 分割的同一 owner cell 内，其几何/clearance 由现有连续 cell certificate
   与 SurfaceValidator 覆盖；不得另外放宽 Builder tolerance；
6. 不得同时保留间距小于等于 Builder 最小 cell epsilon 的 seam/current 两点，制造
   必然失败的退化 cell；
7. 所有保留 knot 的 state 仍只允许由同一个 immutable owner 精确 evaluate；
8. 最终 partition 必须严格单调，每个相邻 cell 都不得跨越任何位于 preview range
   内的 owner structural seam。

A6-P1 所称“唯一 current anchor”在本修正后定义为：

```text
唯一的 TubeBuilder current-match representative
```

通常它是精确 current；仅在上述结构 seam 等价类中允许它是 exact seam。

## 3. 必须冻结

不得修改：

- TubeBuilder、CertifiedTubeBuilder、Filter、SurfaceValidator、EpochManager；
- current-match tolerance、cell epsilon、owner domain epsilon；
- planner、C2、H2、Runtime、authority、map/snapshot provenance；
- Tube margin、safe distance、参数、launch、ROS schema、topic 或线程；
- ordinary off-grid current、near ordinary knot、near preview boundary 的 A6-P1 行为。

不得用 rounding、grid snap、扩大 epsilon、删除 certificate 或 fallback 绕过此问题。

## 4. 文件白名单

只允许修改：

```text
docs/Codex_A6_P1R_Structural_Seam_Preservation_Correction_Execution_Spec_2026-08-20.md
docs/Codex_A6_P1R_Structural_Seam_Preservation_Correction_Self_Audit_2026-08-20.md
docs/Codex_A6_Owner_Aligned_Current_Phase_Preview_Correction_Self_Audit_2026-08-20.md

src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp

/home/cxq/ISF-GVF/handoff.md
```

其余文件全部只读。若实现需要 header、Tube、manager、Runtime、launch 或参数改动，
停止并报告，不得扩展白名单。

## 5. 强制测试

在 `MakeCertifiedThreePieceOwner()` 或等价多段 certified owner 上新增至少：

1. `current_w = seam - d`，其中
   `kPreparedCoverageTolerance < d < kTubeCurrentPhaseAnchorTolerance`；
2. `current_w = seam + d`，使用同样范围的 `d`；
3. 两个用例都必须证明：
   - candidate raw/filter/complete；
   - current sample found/complete；
   - exact owner seam 保留；
   - 不要求 exact current knot 同时保留；
   - current-match representative 唯一；
   - partition 严格单调；
   - 任意相邻 cell 不跨 owner seam；
   - cell geometry certificate 路径没有退回 fixed inset；
4. 既有 exact seam、near supplied knot、near preview boundary、连续三个 off-grid
   current 和 prepared Pair 测试保持通过。

## 6. 文档事实修正

同步修正 A6-P1 自审：

- `/tmp/a6_p1_20260820_esdf_active_1` 是 A6-P1 实际启动产生的隔离 active capture，
  不得称为 supplied/existing/no-new-launch；
- observe-only 仍可明确标注为继承 M3-L1 的既有证据；
- goal 消息 `(8,0,1)` 经 `gvf_manager::goalCallback()` 的既有语义转换为内部
  `goal_pt.z = msg.z + 1.0`，所以 final odom `z≈2` 与内部目标一致；不得把它写成
  未解释的 1 m 测试差异。

不得为文档修正而修改 `goalCallback()`。

## 7. 验收

必须通过：

```text
catkin_make -j2 phase_offset_matched_adapter_test
./devel/lib/bspline_race/phase_offset_matched_adapter_test
ctest --output-on-failure -R '(_ctest_phase_offset_navigation_gtest_phase_offset_tube_(cross_section|builder|filter|surface_validator)_test|_ctest_phase_offset_navigation_gtest_phase_offset_(certified_tube_builder|runtime|tube_epoch_manager)_test|_ctest_bspline_race_gtest_(gvf_switch_policy|continuous_phase_path)_test)$'
catkin_make -j2
git diff --check
```

还必须静态确认：

- 两个 production helper call 仍显式传入 current；
- owner seam precedence 只位于 preview partition helper；
- 未修改 Tube/manager/Runtime/launch/header/参数；
- M3 unified candidate builder 入口未旁路。

本修正不要求重新 launch：它是离散 seam epsilon 边缘，单测比依赖时序恰好命中的
动态运行更确定。完成后写自审并停止，等待 A6-P2 新规格。
