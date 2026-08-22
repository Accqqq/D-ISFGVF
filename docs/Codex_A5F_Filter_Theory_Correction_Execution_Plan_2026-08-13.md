# Codex A5F：Tube Filter 理论与精确一步约束修正执行单

```text
DOCUMENT_ROLE=IMPLEMENTATION_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=A5F
IMPLEMENTATION_AUTHORIZED=true
AUTO_ADVANCE=F1_TO_F2_TO_VALIDATION_ONLY
AUTHORIZATION_SCOPE=FILTER_FALSE_NEGATIVE_AND_EXACT_PWL_STEP_CONSTRAINTS
PRODUCT_GATE_STATE_MODE_REASON_CERTIFICATE_LATCH_SCHEMA_CHANGE_ALLOWED=false
LAUNCH_CONFIG_PARAMETER_CHANGE_ALLOWED=false
MARGIN_LOOKAHEAD_BACK_SPEED_SATURATION_RATE_CHANGE_ALLOWED=false
BOUNDARY_SLOPE_PARAMETER_CHANGE_ALLOWED=false
RAW_CROSS_SECTION_OR_SURFACE_CERTIFICATE_RELAXATION_ALLOWED=false
AGENTS_MD_CHANGE_ALLOWED=false
```

> 日期：2026-08-13  
> 仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
> 理论授权：用户已单独授权 Filter 理论修改，并要求连续完成 F1、F2 与验证。  
> 冻结证据：
> `/tmp/a5r_0b_20260812T004000Z_terra_max_final_v3/`。

## 0. 目标与执行顺序

本单修正两个已经分离的问题：

1. **F1：Filter 几何假阴性。** 当前 zero-tangent Hermite 与固定两段 repair budget
   会把“插值器没有构造成功”错误解释成“安全内管道不存在”，造成远端后缀假截断。
2. **F2：一步边界检查并不 exact。** 当前 `PortProjector` 用当前位置边界导数外推
   `w_next`；当一步跨越 piecewise boundary knot 时，这不是实际下一步边界。

严格顺序为：

```text
baseline + frozen-corpus oracle
-> F1 exact Lipschitz inner envelope
-> F1 focused regression + 46-cohort replay
-> F2 exact piecewise-linear sample-and-hold constraints
-> F2 focused regression
-> H2/current-source regression
-> private-master dynamic ESDF
-> S4 physical tracking
```

F1 验收通过后自动进入 F2；F2 验收通过后自动进入 private-master 动态验证。除此之外
不自动扩展到 emergency/replan bridge、多机、planner 参数、论文实验或其他阶段。

## 1. 已证实的事实

### 1.1 A5R-0B 观测

冻结 corpus 共 46 个 cohort：

- raw cross-section forward first exclusion：`0/46`；
- current Filter forward first exclusion：`45/46`；
- raw boundary slope 统计不能直接判定 filtered inner tube 是否存在；
- 保持现有配置，使用正确的 knot Lipschitz 内包络重算：
  - `L=0.80` 时 44/46 cohort 的几何内管道前向覆盖至少 `0.40 w`；
  - 即使沿用旧代码内部的 `0.80/1.5`，仍有 44/46 达到 `0.40 w`；
  - 剩余两个不足 `0.40 w` 的 cohort 同时已有
    `CURRENT_OFFSET_OUTSIDE` 事实，不能归咎于 Filter slope。

因此现有数据证明的是 **Filter 表示算法假阴性**，尚未证明柱子场与斜率受限 inner tube
理论不兼容。

### 1.2 当前实现的结构性错误

`tube_filter.cpp` 当前执行：

```text
knot_slope = boundary_slope_max / 1.5
-> ApplySlopeEnvelope
-> zero-tangent cubic Hermite DenseSafe
-> 每次压平一个失败 cell
-> kLocalRepairBudget = 2
-> 第三个失败 cell 令整个 range 失败并截断 suffix
```

反例：raw lower 在 cell 内为 `l(s)=s`，相同端点、两端零导数的 Hermite 为
`q(s)=3s^2-2s^3`。靠近起点时 `q(s)<l(s)`，所以 cubic 越出 raw-safe lower。
这是插值选择失败，不是安全 tube 不存在。

### 1.3 不采用普通 PCHIP 的原因

普通 PCHIP/shape-preserving Hermite 只控制相邻 endpoint values，不自动证明 cubic 始终位于
两条随 `w` 变化的 raw 直线之间。全局 C1 也不总能存在：若唯一可行 corridor 是
`lower=upper=|w|`，knot 处必然只有单侧导数。

本单理论对象因此是：

```text
continuous piecewise-linear safe inner tube
+ bounded one-sided derivatives
+ full TubeSurfaceValidator ribbon certificate
```

不伪造不存在的全局 C1。

## 2. F1 数学定义：精确 Lipschitz 安全内包络

给定严格递增 knots `w_0 < ... < w_n` 与 raw intervals

$$
I_k=[\ell_k,u_k],\qquad \ell_k\le u_k,
$$

其中 raw bounds 在相邻 knots 之间按线性插值解释。令
`L = boundary_slope_max`，定义

$$
\widehat\ell_k=\max_j\{\ell_j-L|w_k-w_j|\},
$$

$$
\widehat u_k=\min_j\{u_j+L|w_k-w_j|\}.
$$

它们分别是 raw lower 的最小 `L`-Lipschitz majorant 与 raw upper 的最大
`L`-Lipschitz minorant。range 的精确离散可行判据是

$$
\boxed{\widehat\ell_k\le\widehat u_k\quad\forall k.}
$$

不能使用下列错误判据：

```text
abs(raw boundary slope) <= L
```

### 2.1 线性时间传播

不得用全局反复 repair。对一个候选 range，使用确定性的 forward/backward passes：

```text
lower = raw_lower
forward:  lower[k] = max(lower[k], lower[k-1] - L*dw)
backward: lower[k] = max(lower[k], lower[k+1] - L*dw)

upper = raw_upper
forward:  upper[k] = min(upper[k], upper[k-1] + L*dw)
backward: upper[k] = min(upper[k], upper[k+1] + L*dw)
```

结果必须与上面的 all-pairs 数学定义在容差内一致。不得保留 `/1.5`、zero-tangent
Hermite、`kLocalRepairBudget` 或失败 cell flattening。

### 2.2 全 cell 安全证明

相邻 cell 内 raw 与 filtered 均为线性函数。若两个端点满足

$$
\ell_k\le\widehat\ell_k\le\widehat u_k\le u_k,
$$

则差函数也是线性的，故整个 cell 都满足

$$
\ell_{raw}(w)\le\widehat\ell(w)\le
\widehat u(w)\le u_{raw}(w).
$$

该解析事实取代 Filter 内的 dense 猜测式 cubic repair；它不取代
`TubeSurfaceValidator` 对世界坐标完整 ribbon 的 clearance certificate。

### 2.3 query 与 knot 导数

`TubeFilter::query()` 改为 piecewise-linear evaluation：

- cell interior 返回该 cell 的常斜率；
- interior knot 在系统既有非反向 phase 语义下返回右单侧导数；
- preview 最后一个 knot 返回左单侧导数；
- value 在 knot 连续；
- `abs(lower_w), abs(upper_w) <= L`；
- query 不得扩大 preview domain 或在 domain 外 clamp 成有效结果。

`TubeRawSample::lower_w/upper_w` 可继续保存 query 所需的单侧导数，避免新增 profile
schema。`dense_samples_per_segment` 为兼容已有 config/ABI 可保留，但不能再成为几何可行性
或截断门槛。

### 2.4 anchored range 选择

保留 Filter 的几何职责：从包含 current anchor 的 raw-connected profile 中优先选择最远
forward endpoint，同 endpoint 时保留更多直接相连的 history。每一个 candidate range 使用上面的
精确 envelope 判据。只有以下事实才允许截断：

- raw sample 不完整或 interval 空；
- phase ordering 非法；
- 该 anchored range 的精确 Lipschitz envelope 确实为空；
- 后续 `TubeSurfaceValidator` 对该 world-space cell fail-closed。

retained delta 是否在当前 interval 内仍由既有 manager/runtime 判断；不得塞回 Filter 成为新 gate。

## 3. F2 数学定义：真实 piecewise-linear 一步约束

F2 不新增 preview install gate。它只修正当前 high-rate projector 已经声称执行的下一步约束。

令一个 command 周期内 sample-and-hold：

$$
v=\dot w=f_w^{ISF}+u_w,\qquad q=u_\delta,
$$

$$
w^+=w+\Delta t\,v,\qquad
\delta^+=\delta+\Delta t\,q.
$$

对每个可能的 terminal PWL cell 有

$$
\ell(w)=m_\ell w+c_\ell,\qquad
u(w)=m_u w+c_u.
$$

terminal containment、terminal cell membership 与途中 crossed knots 都能写成
`(u_w,u_delta)` 的线性 half-planes。

### 3.1 terminal cell 约束

每个 candidate polygon 必须同时包含：

```text
cell_w0 <= w + dt*(base_w_dot + u_w) <= cell_w1
delta + dt*u_delta >= lower(w_next) + interior_margin
delta + dt*u_delta <= upper(w_next) - interior_margin
```

### 3.2 crossed-knot 约束

若正向一步跨越 knot `xi`，令 `d=xi-w`。在该 knot 检查线性 offset trajectory：

$$
dq\ge[\ell(\xi)+m-\delta]v,
$$

$$
dq\le[u(\xi)-m-\delta]v.
$$

由于 `v=base_w_dot+u_w`，它们仍是二维线性 half-planes。当前点已由 existing current
containment 检查；每个 crossed knot 与 terminal point 都安全，就能推出 trajectory 在每个
PWL sub-cell 内安全。

`v=0` 按 current containment 处理，不除以 `v`。负 phase 仍由既有 non-reversing limits 禁止，
不得为本单增加 reverse-path 分支。

### 3.3 projector 集成边界

允许在 pure core input 中加入 caller-owned 的通用二维线性 constraints；它们是数学输入，
不是 product gate/state/reason/schema。禁止新增 failure enum、runtime mode、epoch reason、latch、
certificate 或 ROS diagnostics 字段。

当 caller 未提供 exact PWL constraints 时，`PortProjector` 的现有非-tube/A4 行为必须保持。
当提供时：

- exact PWL terminal/crossed-knot constraints 取代当前错误的 current-tangent
  `exact_next_upper/lower` 外推；
- 保留既有 local amplitude/rate、positive/nonnegative phase、tangent-speed、regularity约束；
- 现有 §13.6 current one-sided invariant constraints可以保留，但不得以它替代 exact terminal/
  crossed-knot检查；
- 为每个可能 terminal cell 复用现有二维 polygon projector；
- 从所有可行 polygons 中按已有平方距离与 lexicographic tie-break 选全局最优；
- 投影后必须用 `TubeFilter::query(w_next)` 复核真实 next bounds，并验证 crossed knots；
- 所有尝试在 live Runtime state commit 前完成，失败不得改变 `delta_`、previous final port、
  profile lifecycle 或 H2 pair ownership。

正常 `U+` 失败后再尝试既有 `U_safe`，两者均空时沿用已有 certificate-denied 语义。不得新增
第三套 gate/mode/reason。

## 4. 穷尽文件白名单

### 4.1 执行单与理论记录

```text
docs/Codex_A5F_Filter_Theory_Correction_Execution_Plan_2026-08-13.md
```

最终证据、自审与一次性 replay 工具写入 `/tmp/a5f_*`，不把 corpus 分析脚本接入产品 CMake。

### 4.2 F1 产品与测试

```text
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_filter.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_filter_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp
```

后两项仅在新增 regression fixture 确有需要时修改；不得改其产品实现。

### 4.3 F2 产品与测试

```text
src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/port_types.h
src/swarm_planner/phase_offset/phase_offset_core/src/port_projector.cpp
src/swarm_planner/phase_offset/phase_offset_core/test/port_projector_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_integration_test.cpp
```

`phase_offset_runtime.h` 只允许增加无状态的 private/helper 数学接口；不得改变公开 product
status、mode 或 diagnostics schema。adapter/integration 文件仅允许测试修改。

### 4.4 明确不在白名单

禁止修改：

```text
AGENTS.md
CMakeLists.txt / package.xml
launch / yaml / rviz / message definitions
tube_cross_section.* / tube_builder.* / tube_surface_validator.cpp
tube_epoch_manager.cpp / tube_epoch_types.h
gvf_manager.cpp / gvf.cpp / path C2 geometry / H2 ownership product code
planner / map margin / governor / SO3 / simulator
```

若正确实现确实需要白名单外产品文件，停止该子步骤并报告；不得自行扩大范围。

## 5. F1 必须测试

至少覆盖：

1. constant、asymmetric 与 one-sided profiles；
2. 非均匀 `w`；
3. 连续超过三个收缩/展开 cells，不再因 repair budget 假截断；
4. 原 `boundary_slope_max=100` remote-suffix fixture 不再因固定 repair budget 截断；
5. all-pairs oracle 与 forward/backward envelope 一致；
6. 每个 knot 和 dense cell query 均满足 raw subset；
7. `abs(filtered slope)<=0.80`；
8. interior knot value 连续，右单侧导数确定；
9. `lower=upper=abs(w)` 类尖角 corridor 被表达为 PWL，不伪造全局 C1；
10. genuine exact-envelope empty 才按既有 `REGULARITY` reason 截断；
11. Filter 不检查 retained delta；
12. SurfaceValidator 仍验证完整 ribbon，unsafe fixture 不得变成 safe。

### 5.1 冻结 corpus replay

在 `/tmp/a5f_*` 编写独立 replay，读取 A5R-0B 的同 cohort `FILTER_INPUT`：

- 先以 all-pairs 公式形成 oracle；
- 再以 production F1 算法形成结果；
- 两者逐 knot 比较；
- 报告每 cohort raw endpoint、exact-envelope first exclusion、forward coverage；
- 单独报告 `retained_delta outside raw current interval`，但不把它算作 Filter failure；
- 预期 `L=0.80` 时至少 44/46 cohort 有 `>=0.40 w` 几何覆盖；
- 其余 cohort 必须提供 exact-envelope-empty witness，不能用 raw slope p90 替代。

## 6. F2 必须测试

至少覆盖：

1. 一步停留在同一 PWL cell；
2. 从 interior 跨一个 knot；
3. 一步跨多个 knots；
4. current phase 正好位于 knot，使用右单侧导数；
5. zero progress；
6. `U+` 非空；
7. `U+` 空而 `U_safe` 非空；
8. 两者均空；
9. terminal cell 枚举全局最优与 lexicographic tie-break；
10. dense time rollout 的每个时刻均位于 PWL bounds 内；
11. projection failure 不改变 Runtime retained delta、previous port、profile lifecycle；
12. no-tube/A4 compatibility；
13. current-source/H2 pair revision 不回归；
14. 当前所谓 `exact_next` 跨 knot 的负例必须在修改前复现、修改后通过真实 bounds 检查。

## 7. 构建、回归与动态验收

### 7.1 每阶段前后审计

记录到独立 `/tmp/a5f_*` evidence：

- `git status --short`；
- tracked diff/name-status；
- 白名单文件 SHA-256；
- `git diff --check`；
- build/test commands、exit codes 与 binary SHA-256；
- 不清理、不覆盖、不归属既有用户脏改动。

### 7.2 focused build/tests

顺序执行：

```text
phase_offset_tube_filter_test
phase_offset_tube_surface_validator_test
phase_offset_tube_builder_test
phase_offset_tube_epoch_manager_test
phase_offset_port_projector_test
phase_offset_runtime_test
phase_offset_matched_adapter_test
phase_offset_tube_epoch_integration_test
```

随后运行 current-CMake 16-binary focused suite。旧 ABI/non-current-CMake artifact 不得冒充
当前源码结果；若发现 ABI 不一致，正常重建授权包与 test target，不修改 CMake。

### 7.3 private-master 动态 ESDF

不得接入或终止用户默认 ROS master。只能使用 task-owned private master/ROS_HOME，并清理本任务
创建的进程。launch、参数与 pillar map 保持不变。

必须记录：

- raw/filter/validator forward coverage；
- build/first-consume timing；
- Candidate、Active、Certified、Selected；
- path/tube revision pair；
- `CURRENT_OFFSET_OUTSIDE`、`FORWARD_HORIZON_SHORT` 与 projector infeasibility 的原有分类；
- selected 同 revision 连续区间；
- `/position_cmd -> SO3 -> simulator` tracking error。

产品验收目标：

```text
Filter 不再制造 0.05--0.10 m 假短 horizon
fixed revision Candidate -> Active -> Certified -> Selected
H2 old/new pair 原子性不回归
first consume p95 <= 100 ms
动态 selected 同 revision连续 >= 3 s
S4 tracking max < 0.05 m（如实使用真实 tracking 数据）
```

若 Filter/F2 已通过而 tracking 未过，只报告真实物理 blocker；不得改 gain、速度、margin、
lookahead、slope 或加 gate 使其通过。

## 8. 立即停止当前子步骤的条件

下列任一项发生时停止当前子步骤、保留 evidence 并报告；若只影响 dynamic episode，可继续完成
所有不依赖它的静态回归，但不得伪报 PASS：

- 任一 filtered point/cell 超出 raw；
- SurfaceValidator 出现 false-positive；
- production envelope 与 all-pairs oracle 不一致；
- 需要调 `boundary_slope_max`、margin、lookahead、速度、rate 或 tracking bound；
- 需要新增 gate/state/mode/reason/certificate/latch/schema；
- F2 candidate 在 crossed knot 或 terminal true bounds 外；
- failed projection 改写 live Runtime/H2 ownership；
- H2 revision pair authority 回归；
- 需要修改白名单外产品文件。

## 9. 最终交付判定

最终报告必须分别给出：

```text
F1_FILTER_THEORY = PASS / FAIL
FROZEN_46_COHORT_REPLAY = PASS / FAIL
F2_EXACT_PWL_STEP = PASS / FAIL
FOCUSED_REGRESSION = PASS / FAIL
H2_REGRESSION = PASS / FAIL
DYNAMIC_ESDF = PASS / NOT_RUN / FAIL
S4_TRACKING = PASS / NOT_RUN / FAIL
NO_NEW_PRODUCT_GATES = PASS / FAIL
```

不得用“编译通过”“测试总数”或 marker 可见替代闭环结果，也不得因为动态环境不可用而把
未运行项写成 PASS。不得 commit、branch、tag、push、reset、restore、clean 或 stash-pop。

