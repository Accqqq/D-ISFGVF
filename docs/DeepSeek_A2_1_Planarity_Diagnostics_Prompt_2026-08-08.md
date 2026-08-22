# DeepSeek A2.1 平面性诊断与无效可视化修正执行单

```text
DOCUMENT_ROLE=STAGE_EXECUTION_SPEC
STAGE=A2.1
PREVIOUS_STAGE=A2
AUTO_ADVANCE=false
ALLOWED_TO_EXECUTE=A2.1_ONLY
```

## 1. 唯一目标

不进入 A3，不改变任何控制行为，只完成：

1. 量化真实 `ContinuousPhasePath` 全路径的高度范围、最大
   \(|p_{w,z}|\) 和最大 \(|p_{ww,z}|\)；
2. 记录最大值对应的 semantic phase；
3. 记录 Shadow 路径总采样数和无效采样数；
4. 无效时明确删除旧 RViz 候选路径和 T/N Marker，禁止显示陈旧几何；
5. 为是否满足固定高度 proposal 假设提供数据。

本阶段不改变 planar tolerance，不投影新的输入，不修正 C2，不进入 active。

## 2. 开始前阅读和自检

完整阅读根目录 `AGENTS.md`、总体计划、架构文档和 A2 执行单。

确认：

- branch=`main`、HEAD=`9a0e975`；
- A1 core 7/7；
- bspline_race 原64项和 A2 adapter 7项通过；
- A2 ROS disabled/shadow 都能到达；
- prototype stash 和旧 untracked 原型仍保留。

## 3. 文件白名单

只允许修改：

```text
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_shadow_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_shadow_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_shadow_adapter_test.cpp
```

禁止修改 manager、launch、CMake、package、phase_offset_core 和其他任何文件。

## 4. 诊断字段

在 ShadowDiagnostics 中增加等价字段：

```text
sample_count
invalid_sample_count
path_z_min
path_z_max
path_z_span
max_abs_p_w_z
max_abs_p_w_z_at_w
max_abs_p_ww_z
max_abs_p_ww_z_at_w
candidate_path_complete
```

扫描必须使用 adapter 收到的原始 `PathDifferentialState`，不能先修改或投影
输入后再统计。

扩展 `Float64MultiArray` 时必须更新字段顺序注释。不能新增自定义消息。

无效原因继续通过 throttle 日志输出；日志至少包含当前
`p_w.z`、`p_ww.z`、全路径最大值及对应 w。

## 5. Marker 行为

当前实现无效时发布空 MarkerArray，可能保留旧 frame。修正为：

- current geometry 无效时，对当前点、候选点、T、N 的既有 marker ID 发布
  `DELETE`，或使用明确且经过测试的 `DELETEALL`；
- candidate path 不完整时，不允许用一个 LINE_STRIP 跨越无效区间连接前后
  有效点；第一版可直接清空 candidate path 并标记
  `candidate_path_complete=false`；
- base path 可以继续显示全部 finite 的原始 p 样本；
- 所有发布内容必须 finite。

## 6. 单元测试

新增或扩展测试：

1. 正常平面样本统计值为零或预期值；
2. 人工设置不同 `p_w.z`，最大值和对应 w 正确；
3. 人工设置不同 `p_ww.z`，最大值和对应 w 正确；
4. path z span 计算正确；
5. 存在无效样本时 `invalid_sample_count` 正确，candidate path 不跨段连接；
6. current geometry 无效时生成删除旧 frame 的 Marker action；
7. 输入不被修改；
8. 原 A2 adapter 7项测试继续通过或在等价覆盖下增加通过项。

执行：

```bash
catkin_make -j8
catkin_make run_tests_phase_offset_core
catkin_make run_tests_bspline_race
catkin_test_results --verbose
```

## 7. ROS 数据采集

使用原 `pillar.pcd`、原 simulator 和 A2 shadow launch，设置：

```text
shadow_delta=0.40
```

使用与 A0/A2 同类的点到点目标完整运行一次，覆盖规划、飞行、重规划和末端
C2 段。

必须报告原始数值：

```text
path_z_min
path_z_max
path_z_span
max_abs_p_w_z 及其 w
max_abs_p_ww_z 及其 w
最大 invalid_sample_count / sample_count
current invalid 出现次数和 phase 范围
```

不得只写“残差较小”或“超过容差”。必须给出数值、单位和 phase。

确认 UAV 控制和到达仍不受影响，`/position_cmd` 仍只有原
formation_planning 发布者。

## 8. 严格禁止

1. 提高或降低 planar tolerance；
2. 修改 core 的 p_w/p_ww 检查；
3. 在 adapter 中把超限残差强行置零；
4. 修改 C2、B 样条、A*、SDF、manager、governor 或 simulator；
5. 实现 active、matched port、tube、CBF 或 swarm；
6. 修改 K1/K2 或速度限制；
7. 恢复旧 prototype；
8. 创建 commit；
9. 自动进入 A3。

## 9. 自审核和汇报

完成后汇报：

1. 修改的3个文件；
2. 新诊断字段和消息顺序；
3. Marker 删除与候选路径不跨段策略；
4. 单元测试及全部回归结果；
5. ROS 测得的全部原始平面性数值；
6. current invalid 的时间/phase 范围；
7. 到达结果和 `/position_cmd` 发布者；
8. 最终 Git 状态和禁止依赖审核；
9. 明确写出：`A2.1 完成后已停止，未进入 A3`。

如果数据表明路径并非固定高度，不自行修改理论或实现；保留数据并停止，由
后续审核决定是修正前端固定高度，还是正式扩展为 2.5D 几何。
