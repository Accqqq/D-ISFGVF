# Codex A5G：有限前端可执行相位参数化与语义域修正执行单

```text
DOCUMENT_ROLE=IMPLEMENTATION_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=A5G
IMPLEMENTATION_AUTHORIZED=true
AUTO_ADVANCE=G1_TO_G2_TO_REGRESSION_TO_PRIVATE_ROS_ONLY
AUTHORIZATION_SCOPE=POINT_FRONTEND_ARCLENGTH_PARAMETERIZATION_AND_EXECUTABLE_SEMANTIC_DOMAIN
TUBE_GEOMETRY_FILTER_VALIDATOR_CHANGE_ALLOWED=false
PRODUCT_GATE_STATE_MODE_REASON_CERTIFICATE_LATCH_SCHEMA_CHANGE_ALLOWED=false
LAUNCH_CONFIG_PARAMETER_CHANGE_ALLOWED=false
MARGIN_LOOKAHEAD_BACK_SPEED_SATURATION_RATE_CHANGE_ALLOWED=false
AGENTS_MD_CHANGE_ALLOWED=false
```

> 日期：2026-08-13  
> 仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
> 前置阶段：A5F 已完成精确 Lipschitz PWL 内包络与 exact-PWL held-step 约束。  
> 本单不推翻 A5，也不修改 tube 横截面、Filter、SurfaceValidator、epoch 或 H2 原子 pair 语义。

## 0. 结论与执行顺序

当前动态阻塞不是 tube 覆盖不足。只读探针已经证明：

```text
raw=1 filtered=1 complete=1 active=1 valid=1 horizon=1
certified_forward=0.400
tracking≈0.00138
```

真正失败链为：

```text
Kino B-spline 静止数学端点
-> 线性 time-to-phase 映射把 |p_t|≈0 原样带入 |p_w|≈0
-> ISF base_w_dot 与 1/|p_w| 成正比并升至约 35.35
-> dt=0.02 的一步从 w=0.05 到约 w=0.756
-> exact-PWL 正确判定 U+ 与 U_safe 都空
-> initial finite frontend failed
```

把 bootstrap horizon 从 `0.40` 临时扩到既有 `lookahead_w=2.0` 后仍然失败，证明扩 tube
horizon 不是根修。严格执行：

```text
baseline + probe freeze
-> G1 normalized-arclength mapped B-spline
-> G2 executable semantic domain using existing endpoint margin
-> focused/current-source/H2/A5F regression
-> private-master dynamic ESDF without probe
-> only after Selected: physical S4 tracking
```

## 1. 冻结事实

### 1.1 原始失败

初始 point frontend 的 raw spline 数学起点为静止边界：

```text
w=0
||p_w||≈3.55e-6
curvature≈-2.898e10
delta=-0.05 regularity≈-1.449e9
```

当前 A4 preflight 扫描完整 owner，因此正确地拒绝该坏 semantic sample。

### 1.2 只跳过端点不是修复

临时只读探针排除 `w=0` 后，初始相位 `w=0.05` 仍有：

```text
||p_w||≈0.056
base_w_dot≈35.35
dt=0.02
next_w≈0.756
```

即使把 bootstrap 终点临时改为 `w=2.05`，projector 仍报告：

```text
current U+ and U_safe port sets are empty; offset certificate denied
```

因此不得在 preflight、Runtime 或 TubeFilter 中增加 skip/epsilon/gate，也不得扩大 horizon
掩盖相位参数化奇异。

### 1.3 尾端同样必须使用可执行域

当前特定采样未必命中 raw B-spline 的静止终点，但 Kino 轨迹具有终端静止约束；采样率与
duration 对齐时仍可能命中 `end_vel=0`。不能依赖一次运行中尾端恰好非零。

## 2. G1：normalized-arclength mapped B-spline

### 2.1 数学对象

对 manager 明确提供的严格内部时间区间
`[t_exec_start,t_exec_end]`，定义：

$$
s(t)=\int_{t_{exec,start}}^t\|p_\tau(\tau)\|d\tau,
\qquad S=s(t_{exec,end}),
$$

$$
c=\frac{S}{w_{exec,end}-w_{exec,start}},
\qquad
s(t(w))=c(w-w_{exec,start}).
$$

由链式法则：

$$
p_w=c\frac{p_t}{\|p_t\|},
$$

$$
p_{ww}=c^2\left(
\frac{p_{tt}}{\|p_t\|^2}
-\frac{p_t(p_t^T p_{tt})}{\|p_t\|^4}
\right).
$$

`state.vel` 继续保存 B-spline 的真实时间导数 `p_t`；不得把 `p_w` 冒充物理速度。

### 2.2 数值实现要求

- 使用确定性、单调的弧长表与 bracketed inversion；不得依赖 ROS 或运行时状态。
- 表必须包含 manager 传入的两个严格内部 time endpoints。
- 数值积分/反解精度必须由收敛测试证明，不得新增 launch/yaml 参数。
- 若提供的内部区间本身不正规（非有限、长度为零、内部仍有不可积分的停滞区间），factory
  返回空 evaluator，沿用现有 fail-closed；不得新增 reason/mode。
- `makeMappedBspline` 的外部 phase 坐标跨度保持调用者给出的 `[w0,w1]`，不重新定义系统中
  的 revision、phase 或 tube coordinate。
- 不修改 `UniformBspline::singleDeboor()`、`getDerivative()`，不伪造 raw endpoint 导数。

## 3. G2：已有 endpoint margin 定义完整可执行语义域

### 3.1 语义

raw planner B-spline 仍完整保留。point-mode 的 `ContinuousPhasePath`、
`full_path_samples`、tube profile 与 H2 pair 只拥有可执行语义域。

初始前端令 raw phase domain 为 `[w_raw_start,w_raw_end]`，复用现有：

```text
m = min(point_phase_endpoint_margin_w_,
        0.25 * (w_raw_end - w_raw_start))
```

并定义：

```text
w_exec_start = w_raw_start + m
w_exec_end   = w_raw_end - m
```

在 raw spline 上按弧长从两端各裁去同一个既有 `m`，求得严格内部
`t_exec_start/t_exec_end`，再由 G1 映射到 `[w_exec_start,w_exec_end]`。

这不是新阈值或新 gate；只是把已经用于 phase clamp 的 margin 提升为 owner 的真实 semantic
domain，使 preflight 扫描的“完整路径”与可物理执行路径一致。

### 3.2 初始相位与防止 double margin

- 初始 point phase 直接为 `continuous_path->startW()`。
- owner 已裁边后，`clampPointPhaseCandidate()` 对该 owner 只 clamp 到
  `[owner.startW(), owner.endW()]`；不得再额外加减 endpoint margin。
- raw planner domain 的旧调用若仍存在，可继续用现有 helper 形成 exec bounds；产品中必须明确
  区分 raw domain 与已裁过的 semantic owner domain，禁止隐式 double trim。

### 3.3 replan、C2 与 H2

- 新 mapped suffix 使用同一弧长参数化，并在 raw candidate 尾端按已有 margin 裁边。
- future seam/join 必须位于新 owner semantic domain 内。
- C2 connector 继续只在 seam/join 匹配 `p,p_w,p_ww`；不得改变 connector 理论。
- composite owner、`full_path_samples`、bootstrap/active profile、H2 transaction 的
  `semantic_path_end_w` 必须来自同一个 semantic owner endpoint。
- 不得把 raw 静止尾端通过 `path_end_w` 或 coverage 字段重新带回 tube/H2。
- 旧 owner 的已认证前缀与 revision/authority-session 原子提交规则保持不变。

## 4. 明确禁止的伪修

禁止：

1. 只删除 preflight 首/尾 sample；
2. 在 Runtime/adapter/filter 内按 `|p_w|` 跳点；
3. clamp `p_w`、clamp `base_w_dot` 或复制邻点导数；
4. 增加最小速度、最小导数、bootstrap horizon、preview gate 或 warm-up latch；
5. 修改 `K1/K2`、amplitude、margin 数值、lookahead、速度、rate、tracking bound；
6. 修改 A5F Filter/PWL、tube geometry、validator 或 certificate 逻辑；
7. 新增 gate/state/mode/reason/certificate/latch/ROS diagnostics schema；
8. 把 raw endpoint 声称为 C2 regular state。

## 5. 穷尽文件白名单

### 5.1 产品

```text
src/swarm_planner/bspline_traj/include/bspline_race/continuous_phase_path.h
src/swarm_planner/bspline_traj/src/continuous_phase_path.cpp
src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
```

`gvf_manager.h` 仅允许修改/增加无状态 helper 的签名或明确 raw-vs-semantic domain 的参数；不得
增加 product lifecycle 状态。

### 5.2 测试

```text
src/swarm_planner/bspline_traj/test/continuous_phase_path_test.cpp
src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_integration_test.cpp
```

后三项只在对应 owner/domain/H2 regression 确有需要时修改。

### 5.3 执行单与证据

```text
docs/Codex_A5G_Executable_Phase_Parameterization_Execution_Plan_2026-08-13.md
```

一次性探针、runner、bag、日志、自审与分析写入 `/tmp/a5g_*`，不得接入产品 CMake。

### 5.4 明确不在白名单

```text
AGENTS.md
CMakeLists.txt / package.xml
launch / yaml / messages
phase_offset_core / phase_offset_navigation 产品文件
tube_* 产品实现 / epoch manager
gvf.cpp / ISF kernel / governor / SO3 / simulator
planner / UniformBspline implementation / map
```

若正确实现要求修改白名单外产品文件，停止并报告，不能自行扩范围。

## 6. 必须测试

### 6.1 G1 pure path tests

1. 非静止普通 B-spline：mapped world point 与 raw B-spline 同轨迹；
2. 多个 interior `w` 上 `|p_w|≈c>0`；
3. `p_w,p_ww` 与 phase 数值差分一致；
4. 非均匀曲率路径的弧长 inversion 单调且端点准确；
5. 静止 raw start/end 被裁后，semantic 两端与内部的 `p_w,p_ww`、曲率有限；
6. 故意含 interior 停滞区间的坏输入 fail-closed，不伪造导数；
7. `state.vel` 与相应 raw spline `p_t` 一致。

### 6.2 G2 manager/domain tests

1. initial raw `[0,L]` 形成 semantic `[m,L-m]`；
2. 初始 phase 等于 semantic start，不再二次加 margin；
3. clamp 对 semantic owner 只 clamp 到 owner bounds；
4. preflight 获得的 full samples 不含 raw 静止端点；
5. 故意在 semantic interior 构造 regularity 失败仍被 preflight 拒绝，证明没有 skip；
6. initial `base_w_dot` 不再由 `|p_w|→0` 升至约 35，一步 `w_next` 留在已构建 profile 内；
7. C2 seam/join 的 `p,p_w,p_ww` 连续；
8. mapped suffix、composite owner、full samples、profile 与 H2 pair endpoint 同域；
9. reset/revision/authority-session/current-source 原子事务回归不退化。

### 6.3 A5F/H2 回归

至少重建并运行当前源码对应的：

```text
continuous_phase_path_test
gvf_switch_policy_test
phase_offset_port_projector_test
phase_offset_runtime_test
phase_offset_tube_filter_test
phase_offset_tube_surface_validator_test
phase_offset_tube_epoch_manager_test
phase_offset_matched_adapter_test
phase_offset_tube_epoch_integration_test
```

随后运行 current-CMake focused 16-binary suite。必须报告源码、二进制 SHA-256 与 exit code，
不得用旧 ABI artifact 冒充结果。

## 7. 动态验收

使用 task-owned private ROS master/ROS_HOME，launch、pillar map 与参数值保持不变；移除所有
`LD_PRELOAD`/bypass probe。

依次验收：

1. 不再出现 `initial finite frontend failed`；
2. 初始 semantic owner 不含 raw 静止端点；
3. initial `base_w_dot`、`next_w` 与 profile domain 有日志/只读证据；
4. Candidate -> Active -> Certified -> Selected；
5. path/tube revision pair 一致，C2/H2 handoff 不退化；
6. build/first-consume p95 保持 `<100 ms`；
7. 只有 Selected 后才计算 `/position_cmd -> /sim/odom` 的真实 tracking；
8. S4 要求 max tracking `<0.05 m`。若未达标，报告真实时序与命令链证据，不得把其他字段
   冒充 tracking，也不得回到 A5 geometry 加 gate。

## 8. 审计与停止边界

执行前后记录：

- `git status --short`、tracked diff/name-status；
- 白名单文件 SHA-256；
- `git diff --check`；
- build/test/ROS commands 与 exit codes；
- 修改文件与本单白名单逐项比对；
- `/tmp/a5g_*` 最终自审。

本单完成 private ROS 与 S4 结果后停止。emergency/replan bridge、多机、论文实验、参数调优与其他
阶段不属于本单。
