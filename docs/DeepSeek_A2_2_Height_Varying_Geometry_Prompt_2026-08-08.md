# DeepSeek A2.2 变高度 2.5D 几何与 Shadow 重验执行单

```text
DOCUMENT_ROLE=STAGE_EXECUTION_SPEC
STAGE=A2.2
PREVIOUS_STAGE=A2.1
AUTO_ADVANCE=false
ALLOWED_TO_EXECUTE=A2.2_ONLY
```

## 1. 唯一目标

A2.1 已证明原规划链输出的是允许高度变化的三维路径，而不是固定高度 XY
路径。本阶段只修订单机 phase-offset 几何契约，使其支持：

```text
三维变高度基础路径 p(w)
+ 重力参考的单个水平 offset delta
+ 原三维 ISF-GVF 切向
```

完成后重新运行 A2 Shadow，验证真实 `ContinuousPhasePath` 不再因为
`p_w.z` 或 `p_ww.z` 非零而无效。

本阶段仍是只读 Shadow，不进入 active，不实现 matched port、tube、CBF 或
swarm，不修改原规划、C2 和控制链。

## 2. 已确认的 A2.1 事实

开始前必须确认以下诊断结论仍作为本阶段输入，而不是待调容差的问题：

```text
path_z_min                 0.773360063 m
path_z_max                 1.224231978 m
path_z_span                0.450871915 m
max_abs_p_w_z              0.256489620 m/w
max_abs_p_ww_z             1.038905520 m/w^2
max invalid ratio          56/60
worst invalid ratio        49/49
current invalid count      14
```

这些值来自原始 `PathDifferentialState`。它们表明高度变化是实际路径几何，
不能作为数值噪声投影掉，也不能通过放大 planar tolerance 掩盖。

## 3. 开始前阅读与自检

完整阅读：

- 根目录 `AGENTS.md`；
- `docs/PhaseOffsetSwarm_Single_First_Implementation_Plan_2026-08-07.md`；
- `docs/PhaseOffsetSwarm_Code_Architecture_2026-08-08.md`；
- A1、A2 和 A2.1 三份执行单；
- `phase_offset_core` 当前 README、geometry 实现与测试；
- Shadow adapter 当前实现与测试。

执行并记录：

```bash
cd /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
git branch --show-current
git rev-parse HEAD
git status --short
git diff --stat
git diff --check
git stash list
```

必须确认：

- branch=`main`；
- HEAD=`9a0e975`；
- `deepseek-phaseoffset-tracked-prototype-2026-08-08` 仍存在；
- tracked diff 仍只有 A2 授权的 5 个文件；
- A2.1 没有新增 tracked 修改；
- A1 core 7/7、原 bspline_race 64 项和 adapter 11/11 的最近结果可审计；
- 原 launch 中 `circle_test/enable=false`、`auto_start=false` 仍保留。

任何前置条件不同，立即停止并汇报。

## 4. 文件白名单

只允许修改以下 8 个文件：

```text
src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/geometry_types.h
src/swarm_planner/phase_offset/phase_offset_core/src/geometry.cpp
src/swarm_planner/phase_offset/phase_offset_core/test/geometry_test.cpp
src/swarm_planner/phase_offset/phase_offset_core/README.md
src/swarm_planner/phase_offset/phase_offset_core/package.xml
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_shadow_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_shadow_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_shadow_adapter_test.cpp
```

不允许新增源码文件。若认为必须修改其他文件，停止并汇报，不得自行扩大
白名单。

尤其禁止修改：

- `gvf_manager.h/.cpp`；
- `gvf.h/.cpp`；
- `continuous_phase_path.*`；
- 任何 launch、CMake 或非白名单 package 文件；
- A*、B-spline optimizer、C2 connector、SDF、governor、simulator；
- `common_msgs`、旧 prototype 和未来 phase_offset package。

## 5. 正式几何契约

### 5.1 维度与能力边界

本阶段实现的是：

```text
gravity-referenced 2.5D phase-offset geometry
```

含义：

- 基础路径 `p(w)`、`p_w`、`p_ww` 保持完整三维；
- UAV 高度跟随原基础路径；
- 标量 `delta` 只产生水平横向 offset；
- 不实现二维法平面 offset；
- 不实现 Bishop/parallel-transport frame；
- 近似竖直、水平切向退化的路径段明确拒绝。

### 5.2 重力参考法向

定义：

\[
e_z=(0,0,1)^T,
\qquad
h=e_z\times p_w,
\qquad
s_h=\|h\|=\|(p_{w,x},p_{w,y})\|.
\]

要求：

\[
\|p_w\|>\varepsilon_T,
\qquad
s_h>\varepsilon_{xy}.
\]

水平左法向为：

\[
N=\frac{h}{s_h}
=\frac{(-p_{w,y},p_{w,x},0)^T}{s_h}.
\]

禁止再用 `abs(p_w.z)` 或 `abs(p_ww.z)` 判断路径无效。

### 5.3 法向导数与活动参考导数

定义：

\[
h_w=e_z\times p_{ww}.
\]

法向解析导数：

\[
N_w
=
\frac{(I-NN^T)h_w}{s_h}.
\]

同时可计算水平有符号曲率：

\[
\kappa_{xy}
=
\frac{p_{w,x}p_{ww,y}-p_{w,y}p_{ww,x}}{s_h^3},
\]

并验证等价关系：

\[
N_w=-\kappa_{xy}(p_{w,x},p_{w,y},0)^T.
\]

活动参考和固定 offset 下的 phase 导数为：

\[
r=p+N\delta,
\]

\[
r_w=p_w+N_w\delta.
\]

禁止继续使用会删除真实竖直导数的：

\[
r_w=(1-\kappa\delta)(p_{w,x},p_{w,y},0)^T.
\]

### 5.4 切向、误差与正则性

活动参考单位切向定义为：

\[
T=\frac{r_w}{\|r_w\|}.
\]

误差必须相对活动参考并沿活动切向分解：

\[
e=x-r,
\qquad
e_\parallel=T^Te,
\qquad
e_\perp=e-e_\parallel T.
\]

水平非折返裕度仍定义为：

\[
\gamma=1-\kappa_{xy}\delta.
\]

要求：

\[
\gamma\ge\mu>0,
\qquad
\|r_w\|>\varepsilon_T.
\]

必须验证：

\[
N^Tp_w=0,
\qquad
N^TN_w=0,
\qquad
N^Tr_w=0.
\]

当 `delta=0` 时必须严格退化为：

\[
r=p,
\qquad
r_w=p_w,
\qquad
T=\frac{p_w}{\|p_w\|}.
\]

当 `p_w.z=p_ww.z=0` 时，新实现必须退化为原二维公式。

## 6. 类型与参数要求

`GeometryParams` 不再包含或使用 `planar_tolerance`。应具有等价的：

```text
tangent_epsilon
horizontal_tangent_epsilon
regularity_margin
```

具体命名可小幅调整，但参数语义必须明确。

`PhaseOffsetGeometryState` 至少应能表达：

```text
p, p_w, p_ww
N, N_w
r, r_w
T                         # 活动参考三维单位切向
path_speed                # ||p_w||
horizontal_path_speed     # ||p_w_xy||
curvature                 # kappa_xy
regularity                # gamma
error, e_parallel, e_perp
w, delta, valid, invalid_reason
```

不要创建另一个混合全局类型头。

失败输出继续保证所有数值字段 finite、`valid=false` 且原因非空。

## 7. Shadow adapter 修订

Adapter 不再设置 `kShadowPlanarTolerance`，删除“固定高度 C2 只有有界 z
噪声”的错误注释。

A2.1 已有原始 z 诊断必须保留，作为路径特性记录，但不得再决定几何是否
有效。

在 `ShadowDiagnostics` 中增加等价字段：

```text
min_horizontal_path_speed
min_horizontal_path_speed_at_w
min_horizontal_to_total_speed_ratio
min_horizontal_to_total_speed_ratio_at_w
max_abs_candidate_z_offset
```

其中：

```text
horizontal_to_total_speed_ratio = ||p_w_xy|| / ||p_w||
candidate_z_offset = r.z - p.z
```

所有统计继续使用 adapter 收到的原始 `PathDifferentialState`。扩展
`Float64MultiArray` 时更新字段顺序注释，不新增自定义消息。

Marker 约定：

- 基础路径保留原三维高度；
- candidate path 每个样本满足 `r.z == p.z` 到数值精度；
- 当前 `T` 箭头允许有 z 分量；
- 当前 `N` 箭头必须水平；
- 无效删除策略继续沿用 A2.1，不能回退为空数组或陈旧 Marker。

## 8. phase_offset_core 单元测试

至少覆盖：

1. 原平面直线、圆、8 字和椭圆回归保持通过；
2. 斜直线 `p=(w,0,a w)` 被接受，`N=(0,1,0)`，`r_w=p_w`；
3. 变高度圆或螺旋曲线的解析 `N_w` 与数值导数一致；
4. 同一变高度曲线的解析 `r_w` 与 `r(w)` 数值导数一致；
5. 较大的 finite `p_w.z`、`p_ww.z` 不再单独导致拒绝；
6. `delta=0` 时 `r=p`、`r_w=p_w`、`T=p_w/||p_w||`；
7. `N.z=0`、`N^T p_w=0`、`N^T r_w=0`；
8. 固定高度输入退化为原二维公式；
9. `gamma < regularity_margin` 时继续拒绝；
10. 近似竖直或 `||p_w_xy||` 过小的路径明确拒绝；
11. 非 finite、零切向和非法参数继续安全拒绝且输出 finite；
12. 现有随机多样本微分一致性测试继续保留或等价增强。

不得通过放宽断言隐藏导数错误。

## 9. Shadow adapter 单元测试

保留 A2/A2.1 的全部等价覆盖，并至少增加：

1. 变高度样本 candidate path 完整，`invalid_sample_count=0`；
2. candidate 每个点的 z 与 base path 相同；
3. `max_abs_candidate_z_offset` 为数值精度量级；
4. 原始 z span、`p_w.z`、`p_ww.z` 诊断仍正确；
5. 最小水平速度、最小水平/总速度比例及对应 w 正确；
6. 斜路径当前 `T` 含正确 z 分量且单位化；
7. `N.z=0` 且与 `T` 正交；
8. 近似竖直 current/sample 继续触发无效和 DELETE；
9. Marker 构造不修改输入；
10. disabled、无效输入和候选不完整删除行为继续通过。

原 `ProjectsBoundedPlanarDerivativeResiduals` 测试不得原样保留；它应替换为
“真实三维导数被保留并正确进入活动切向”的测试。

## 10. 构建与测试

执行：

```bash
source /opt/ros/noetic/setup.bash
cd /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
catkin_make -j8
catkin_make run_tests_phase_offset_core
catkin_make run_tests_bspline_race
catkin_test_results --verbose
```

要求：

- 所有新增 core 测试通过；
- adapter 扩展测试全部通过；
- 原 bspline_race 回归不减少；
- `catkin_test_results` 只允许保留已经记录的无关
  `uav_utils` 缺失 gtest XML；
- 不因本阶段新增警告或失败。

## 11. ROS Shadow 重验

先检查是否已有 ROS master 和用户进程。不得连接、停止或复用用户拥有的
ROS/仿真进程；只清理由本阶段启动的进程。

使用原 `pillar.pcd`、原 simulator、原 A2 Shadow launch：

```text
shadow_delta=0.40
sample_step_w=0.10
```

使用 A2.1 同类点到点完整回程，覆盖规划、飞行、重规划和末端 C2 段。

必须报告：

```text
sample_count / invalid_sample_count
candidate_path_complete
path_z_min / path_z_max / path_z_span
max_abs_p_w_z 及其 w
max_abs_p_ww_z 及其 w
min_horizontal_path_speed 及其 w
min_horizontal_to_total_speed_ratio 及其 w
max_abs_candidate_z_offset
current invalid 次数、原因和 phase 范围
```

验收条件：

- 原 A2.1 的真实 z 变化仍被原样记录；
- 不再出现 `path tangent is outside the XY plane` 或
  `path second derivative is outside the XY plane`；
- 对水平切向非退化的完整路径快照，`invalid_sample_count=0` 且
  `candidate_path_complete=true`；
- `max_abs_candidate_z_offset` 为机器精度量级；
- 当前 T 可随路径升降，N 保持水平；
- UAV 仍正常到达；
- `/position_cmd` 唯一发布者仍是原 `/formation_planning`；
- Shadow 仍没有控制 publisher。

如果真实路径出现近似竖直段或水平切向退化，不得降低 epsilon、切换法向或
临时加入 Bishop frame；保留数据并停止汇报。

## 12. 依赖边界与自审核

执行：

```bash
rg -n "#include <ros|#include \"ros|ros::|SDFMap|ContinuousPhasePath|UniformBspline|gvf_manager|swarm|neighbor|CBF|robot_id" \
  src/swarm_planner/phase_offset/phase_offset_core

rg -n "PositionCommand|cmd_pub|phase_w_|progress_w_|GovernorCommand|SDFMap|CBF|Neighbor|Swarm|u_delta|u_w" \
  src/swarm_planner/bspline_traj/include/bspline_race/integration \
  src/swarm_planner/bspline_traj/src/integration

rg -n "planar_tolerance|fixed-height XY|bounded reparameterization noise" \
  src/swarm_planner/phase_offset/phase_offset_core \
  src/swarm_planner/bspline_traj/include/bspline_race/integration \
  src/swarm_planner/bspline_traj/src/integration

git diff --check
git status --short
git diff --stat
```

确认：

- core 仍只依赖 Eigen/STL；
- adapter 仍无控制写入；
- 只修改 8 个白名单文件；
- manager、launch、CMake、planner、C2 和控制链无变化；
- tracked diff 仍为 A2 的 5 个授权文件；
- prototype stash 和旧 untracked prototype 保持；
- 没有 commit、branch、tag 或 push。

## 13. 严格禁止

1. 修改 A*、B-spline、C2 或 z 规划边界；
2. 将真实 z 导数置零或提高 planar tolerance；
3. 把基础路径投影成固定高度；
4. 实现 Bishop/parallel-transport frame；
5. 把标量 `delta` 改成二维向量；
6. 实现 active、ISFReferenceKernel、matched port 或 PortProjector；
7. 实现 tube、ESDF、CBF、neighbor、swarm 或新消息；
8. 修改 manager、governor、PositionCommand、SO3 或 phase 更新；
9. 修改 K1/K2、速度、加速度、饱和或场景参数；
10. 恢复或连接旧 prototype；
11. 创建 commit；
12. 自动进入 A3。

## 14. 最终汇报格式

1. 开始 Git 状态、branch、HEAD 和 stash；
2. 修改的 8 个白名单文件中的实际子集；
3. 新 2.5D 几何公式和字段语义；
4. 平面退化一致性和变高度微分一致性测试；
5. core、adapter、原 bspline_race 和总测试结果；
6. ROS 全部原始诊断数值；
7. A2.1 invalid 与 A2.2 invalid 的原因对比；
8. UAV 到达和 `/position_cmd` 发布者；
9. 依赖搜索、`git diff --check` 和最终 status；
10. 未修改 planner/C2/control 的证明；
11. 明确写出：`A2.2 完成后已停止，未进入 A3`。

任何验收需要超出白名单或引入完整三维法平面时，停止并汇报，不得扩大
范围。
