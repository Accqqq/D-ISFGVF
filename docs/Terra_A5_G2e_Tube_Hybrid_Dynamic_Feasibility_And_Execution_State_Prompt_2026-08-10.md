# A5-G2e：Tube Hybrid 动态可行性与执行状态语义纠正执行规范

执行模型：`gpt-5.6-terra`

推理强度：`max`

日期：2026-08-10

`AUTO_ADVANCE=false`

本执行单只授权 A5-G2e。完成实现、构建、单测、隔离 ROS 验收、自审核和报告后必须停止。禁止进入 A5.3、A6 或任何多机阶段。

---

## 1. 已确认的问题与本阶段唯一目标

A5-G2d 已纠正 cloud occupancy snapshot 地图合同；不得再次修改地图语义。G2d 正式 ROS bag 中的永久 latch 也已只读定位，不是 tracking bound 直接触发：

```text
evidence:
  /tmp/a5_g2d_ros2/evidence/esdf_snapshot.bag
  /tmp/a5_g2d_ros2/evidence/esdf_snapshot.csv

last valid selected frame:
  stamp                  = 1786308773.1915176
  failure_latched        = 0
  tracking_norm/bound    = 0.05081 / 0.15
  tube lower/upper       = -0.48091 / 2.55
  lower_w/upper_w        = 0 / 0
  selected               = 1

first latched frame:
  stamp                  = 1786308773.1918855
  failure_latched        = 1
  tracking_norm/bound    = 0.04024 / 0.15
  tube lower/upper       = -0.45999 / 2.55
  lower_w/upper_w        = 0.577536 / 0
  retained delta         = -0.004873
  base_w_dot             = 1.790840
  previous u_delta       = -0.002423
  u_delta_rate_max       = 1.20
  dt                     = 0.02
  runtime mode           = FATAL_CONTROL_FAILURE
  final port             = (0, 0), invalid
```

在该帧，下边界不变条件要求：

```text
u_delta - lower_w * (base_w_dot + u_w) + gamma * h_lower >= 0
```

即使取当前 rate box 中最有利的端口，仍远不能满足该约束；当前 `PortProjector` 因 `joint port feasible polygon is empty` 返回 false。根因是：

> `TubeEpochManager` 只检查候选 tube 静态容纳、tracking 和前视横截面，没有在 install 前检查 proposal 10.6 要求的动态端口可行性。动态不可跟随的 candidate 被先安装为 active，下一高频周期才被 Runtime 当成内部控制 invariant 失败并永久 latch。

本阶段唯一目标：

> 在不改变 tube 几何、地图合同、规划器和控制参数的前提下，补齐候选 tube 的 hybrid 动态可行性检查，明确区分正向 rolling 可行、仅非反向 safety-priority 可行、emergency required、暂时等待和真正的内部 fatal invariant，并保证 transient map/tracking/tube rejection 不再错误地永久 latch。

本阶段不是参数调优；不得通过放宽 `tracking_error_bound`、增大端口限幅、降低速度或修改 planner 来使测试通过。

---

## 2. Proposal 权威语义

实现前必须完整读取：

1. `AGENTS.md`；
2. `docs/PhaseOffsetSwarm_Single_First_Implementation_Plan_2026-08-07.md` 的 A5/A6；
3. `docs/PhaseOffsetSwarm_Code_Architecture_2026-08-08.md` 的 navigation、Runtime、adapter 和每周期数据流；
4. `/home/cxq/ISF-GVF/Paper/ICRA/PhaseOffsetSwarm_Detailed_Proposal_2026-08-05.md`：
   - 10.4–10.6；
   - 13.3–13.7；
   - 14.1、14.5；
   - 17.1–17.2；
   - 21；
5. G2d 执行规范和当前实现/测试。

必须保持以下语义：

```text
candidate current state safe + U+ nonempty
    -> install new tube epoch, ROLLING

candidate current state safe + U+ empty + U>=0 nonempty
    -> activate latest safe bounds, SAFETY_PRIORITY

U>=0 empty, or latest observation proves current state unsafe
    -> EMERGENCY_REQUIRED / failsafe request

candidate incomplete, unavailable, unknown, or observation not yet sufficient
    -> WAITING_FOR_CANDIDATE, no false certificate
```

任何合法 tube epoch 更新都必须保留：

```text
w+     = w-
delta+ = delta-
```

禁止 reset、clip 或 project retained `w`/`delta` 以伪造 install 成功。

固定 tube epoch 内的端口约束必须继续同时包含：

- `u_w`、`u_delta` 幅值；
- 两端口的周期变化率；
- rolling 的正 phase 下界；
- rolling 的正物理切向速度下界；
- safety-priority 的非反向 phase 和非负物理切向速度；
- 当前/下一步 offset envelope；
- 上下边界前向不变约束；
- 下一步 regularity 冗余检查。

必须继续由同一个最终端口驱动：

- 内部 `w_dot`；
- 内部 `delta_dot`；
- physical matched feedforward。

---

## 3. 执行前仓库核对

先只读记录：

```bash
git branch --show-current
git rev-parse HEAD
git status --short
git diff --stat
git diff --check
git diff --cached --name-only
git stash list
```

预期：

- branch：`main`；
- HEAD：`9a0e97560b8ddf15ac1ed1581030ed4b911f6e43`；
- named stash：`deepseek-phaseoffset-tracked-prototype-2026-08-08` 存在；
- 无 staged 内容；
- 工作树长期 dirty，所有既有内容均视为用户所有。

若 branch、HEAD、stash 或 staged 前提不符，立即停止。禁止 reset、restore、clean、stash pop、覆盖用户内容。

修改前保存：

- `git status --short`；
- tracked diff；
- tracked 文件 SHA-256/mtime manifest；
- 本阶段白名单文件初始 SHA-256。

---

## 4. 必须保持且不得重做的已验证内容

以下内容已经完成，本阶段不得重写：

- planned B-spline -> `ContinuousPhasePath` -> `PathDifferentialState`；
- `GeometryEvaluator` 的 `p,T,N,N_w,kappa,r,r_w`；
- `+N/-N` 独立射线搜索和非对称横截面；
- one-sided tube 与 `lower > upper` 空横截面语义；
- cloud occupancy snapshot 和 observation sequence；
- full/preincluded/residual margin `0.55/0.10/0.45`；
- `TubeFilter` 的保守 filtered profile；
- candidate/active profile 分离；
- fixed tube；
- 100-cycle zero-port equivalence gate；
- matched cancellation；
- Candidate/Certified 两个 Marker topic 及现有几何点；
- 裸 launch `disabled/none/refresh=3.0` baseline。

不得重新引入 raw log-odds、self-free seed 或 A5.2.3 planner clearance contract。

---

## 5. 动态可行性检查的职责边界

新增一个纯 C++、单一职责的前视动态可行性模块，建议命名：

```text
phase_offset_navigation/tube_dynamic_feasibility.h
phase_offset_navigation/tube_dynamic_feasibility.cpp
```

名称可小幅调整，但不得把算法堆入 adapter、ROS callback 或 `gvf_manager.cpp`。

模块输入必须显式、不可读取 ROS/global state，至少包括：

- candidate filtered `TubeProfile`；
- 与 profile 对齐的 preview `PathDifferentialState`；
- current `w`、retained `delta`；
- current/previous final port；
- control `dt`；
- 当前可用的 base phase/tangent motion facts，或一个不依赖 ROS 的明确抽象；
- 现有 amplitude/rate/positive-margin/regularity/invariant/interior 配置；
- existing certified forward horizon。

输出至少包括：

```cpp
enum class TubeDynamicFeasibilityClass {
  NOT_EVALUATED,
  POSITIVE_FORWARD,
  NONNEGATIVE_ONLY,
  INFEASIBLE,
  INDETERMINATE,
};

struct TubeDynamicFeasibilityResult {
  bool evaluated;
  bool positive_feasible;
  bool nonnegative_feasible;
  TubeDynamicFeasibilityClass classification;
  double checked_start_w;
  double checked_end_w;
  double first_infeasible_w;
  std::size_t checked_sample_count;
  // finite numeric reason/stage enum for diagnostics
};
```

具体字段可适配风格，但语义不得模糊。

### 5.1 不允许的伪检查

以下均不构成合格动态可行性：

- 只检查当前 `delta` 在 candidate bounds 内；
- 只检查 `retainedOffsetForwardContained`；
- 只检查 nominal manual raw port；
- 只看 `lower_w/upper_w` 是否小于某个硬编码阈值；
- 只在安装后的下一控制周期等待 `PortProjector` 报错；
- 忽略 `u_w/u_delta` slew-rate；
- 用新增任意参数替代现有控制限制；
- 修改 tube 边界使不可行问题消失。

### 5.2 必须检查的两个集合

必须分别检查：

1. `U+`：使用现有 `phase_dot_min > 0` 和 `tangent_speed_min > 0`；
2. `U>=0`：只把上述两个下界降为零，其他 amplitude、slew、tube、invariant、regularity 约束完全保留。

不得把 `U+` 失败直接当 emergency；只有 `U>=0` 也为空时才是动态 emergency。

实现必须复用 `PortProjector` 的同一套半平面约束或抽取共享的小型纯 C++ 求解内核，不能在 navigation 再复制一套符号略有差异的 invariant 公式。

若需要让 `PortProjectionLimits` 支持零下界，只允许把合法性从严格正改为非负，并用回归证明正值配置和 tube-disabled A4 路径结果不变。

### 5.3 前视与边界导数

检查区间至少覆盖：

```text
[current_w,
 min(candidate.certified_segment_end_w,
     current_w + existing min_certified_forward_w)]
```

不得只检查 current point。必须覆盖：

- current point；
- predicted next point；
- 区间内所有 tube sample/knot；
- end point；
- 边界导数发生变化处的左右单侧条件，或等价的保守离散 reachability 条件。

采样只能使用已有 `dt`、profile sample/knot、tube sample step 和 certified horizon；不得新增一个为通过测试而调的 launch 参数。

动态证书必须是保守的充分条件：允许 false negative，不允许 false positive。若实现无法在现有数据下证明连续前视可行，必须返回 `INDETERMINATE`/拒绝安装，而不是把局部单点可行冒充完整证书。

必须新增一个复现 G2d latch 的单测：

```text
base_w_dot about 1.79
lower_w about 0.5775
u_delta_abs_max 0.25
u_delta_rate_max 1.20
dt 0.02
previous u_delta about -0.0024
```

预期：candidate 在 install 前被判为 `U+ empty` 且 `U>=0 empty`（或保守 INDETERMINATE），绝不能先安装后触发 PortProjector fatal。

同时测试：

- 宽固定 tube：`U+` 可行；
- 正速度不可行但非反向可行：`NONNEGATIVE_ONLY`；
- 当前点可行但下一个 knot/segment 不可行：前视拒绝；
- rate limit 导致不可行；
- amplitude limit 导致不可行；
- regularity 导致不可行；
- non-finite/缺失 preview：`INDETERMINATE`；
- `w`、`delta`、previous port 在检查中不被修改。

---

## 6. TubeEpochManager 的安装事务

当前错误顺序是：

```text
build candidate
-> static checks pass
-> install active
-> next control frame discovers no feasible port
-> fatal latch
```

必须改为：

```text
build candidate
-> static/current safety checks
-> dynamic U+/U>=0 preview check
-> classify
-> only then commit candidate/active epoch state
```

若 manager 缺少 base guidance/control facts，应使用小型纯接口或两阶段 prepare/commit transaction；不得把 `IsfReferenceKernel`、ROS、SDFMap 或 adapter pointer 放进 manager。

必须保证：

- dynamic result 属于本次 candidate sequence/path revision/map observation sequence；
- stale result 不能安装更新后的 candidate；
- `active_tube_epoch` 只在 material install 时增长；
- equivalent refresh 不产生假 epoch；
- rejected/incomplete/indeterminate candidate 不覆盖 active profile；
- 但一旦最新 observation 否定旧 certificate，旧 active 不得继续标记 certified 或作为未经认证的前进约束；
- `delta` 不因 install/reject/emergency 改变；
- `dynamic_feasibility_evaluated` 不再永久写死 false。

状态语义必须至少满足：

```text
ROLLING:
  latest installed profile current-safe
  U+ certified

SAFETY_PRIORITY:
  latest safe bounds installed
  U+ unavailable
  U>=0 certified

WAITING_FOR_CANDIDATE:
  latest evidence incomplete/unknown/unavailable/indeterminate
  no current certified control claim

EMERGENCY_REQUIRED (新增或等价明确状态):
  latest evidence proves current unsafe, or U>=0 empty
  no manual tube control selection

CONFIGURATION_ERROR:
  immutable bad configuration
```

不得继续把 tracking exceeded、candidate incomplete、map observation changed、temporary WAITING 或 `U+` empty 混进 adapter permanent failure latch。

---

## 7. Runtime 执行模式

Runtime 必须基于 epoch 的已认证分类执行：

### 7.1 ROLLING

- 使用现有正 `phase_dot_min`、正 `tangent_speed_min`；
- 允许 manual excursion；
- 高率周期仍复核当前 exact feasible set 和 exact next envelope；
- 使用同一个 final port 更新内部状态和 matched physical feedforward。

### 7.2 SAFETY_PRIORITY

- 使用 `phase_dot_min=0`、`tangent_speed_min=0` 的非反向约束；
- 其他 amplitude、rate、tube、invariant、regularity 约束不变；
- 不启动新的 manual excursion；
- nominal intent 只允许平滑回中/保持最新安全边界；
- 允许总 phase 和物理切向进度连续降至零；
- 不允许反向；
- 不允许离散 reset/clip `delta`。

### 7.3 WAITING / blocked

- 不把旧 active tube 当作当前地图 certificate；
- 不用未经认证旧 tube 继续 manual progression；
- 不修改 retained `delta` 或 previous final port；
- 不 permanent latch；
- diagnostics/Marker fail closed。

### 7.4 EMERGENCY_REQUIRED

- 不选择 phase-offset manual output；
- 不更新 `delta`/previous port；
- 发布明确 emergency-required 诊断；
- 不伪装为普通 fallback、tracking violation 或 certified tube；
- 本阶段不允许新增 PositionCommand publisher 或低层 hover override，因此报告必须明确：这里只产生 failsafe request/classification，不宣称已经执行低层急停。

### 7.5 真正 FATAL_CONTROL_FAILURE

永久 `failure_latched` 只保留给真正的不可恢复内部控制合同破坏，例如：

- gate 打开后 zero-port equivalence 失效；
- finite/valid 输入下 geometry 或 matched-port 数学 invariant 失败；
- manager/runtime 已认证相应 feasible set，但相同 snapshot/state 上共享 projector 仍报告空集；
- exact next-envelope 检查与刚完成的同状态证书自相矛盾。

以下不得 permanent latch：

- tracking bound 暂时超限；
- candidate incomplete；
- cloud snapshot 暂时 unavailable/unknown/out-of-map；
- source revision/map observation 更新；
- `U+` empty 但 `U>=0` nonempty；
- `U>=0` empty（这是 emergency-required，不是软件 invariant failure）；
- WAITING/REPLAN_REQUIRED/SAFETY_PRIORITY 状态本身。

---

## 8. Adapter 与 Marker/诊断语义

Adapter 只负责编排：

- 转换 path/state；
- 提供 guidance/control facts 给纯接口；
- 调用 manager/runtime；
- 发布 diagnostics 和两个 Marker topic；
- 返回 selected guidance。

禁止在 adapter 中实现 reachability、半平面公式或多段 rollout。

Candidate Marker：

- G2d 几何预览语义不变；
- 只表示完整 finite candidate geometry；
- 与动态认证无关。

Certified Marker：

- 几何点、ID、颜色和 namespace 不变；
- 只有当前 latest installed certificate 与执行状态一致时 ADD；
- ROLLING 的 U+ certificate 可 ADD；
- SAFETY_PRIORITY 若 current-safe 且 U>=0 certificate 有效，可 ADD；
- WAITING、EMERGENCY_REQUIRED、fatal latch、tracking unsafe、stale observation 全部三个 DELETE；
- 不得出现混合 ADD/DELETE。

Manual 83 字段 schema 不移动、不复用旧索引，保持严格 83。

Epoch diagnostics 现有 0–48 索引不移动。现有：

```text
kEpochDynamicFeasibilityEvaluated
```

必须开始发布真实值。若需要新增字段，只允许追加，并至少表达：

- positive feasible；
- nonnegative feasible；
- feasibility classification；
- checked start/end/first infeasible w；
- checked sample count；
- emergency required；
- transient blocked；
- genuine fatal invariant；
- finite runtime failure/reason enum。

更新 static_assert、field names、CSV/analyzer assertions 和 tests；禁止字符串塞入 Float64 数组，禁止改变旧字段含义。

必须给当前 `latchFailure(const std::string&)` 的 reason 建立可诊断的有限枚举或等价结构，不能继续丢弃 reason 后只发布一个 bool。

---

## 9. 文件白名单

只允许修改/新增以下范围中确有必要的文件：

### phase_offset_core（仅共享 projector 可行集语义）

```text
src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/port_types.h
src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/port_projector.h
src/swarm_planner/phase_offset/phase_offset_core/src/port_projector.cpp
src/swarm_planner/phase_offset/phase_offset_core/test/port_projector_test.cpp
```

若无需修改 core，优先不改。

### phase_offset_navigation

```text
src/swarm_planner/phase_offset/phase_offset_navigation/CMakeLists.txt
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_dynamic_feasibility.h   (new)
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_dynamic_feasibility.cpp                              (new)
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_dynamic_feasibility_test.cpp                         (new)
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_types.h
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_epoch_manager.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_epoch_manager_test.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp
```

### bspline_race thin integration

```text
src/swarm_planner/bspline_traj/CMakeLists.txt
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_tube_epoch_diagnostics.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_tube_epoch_diagnostics.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_diagnostics_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_tube_epoch_integration_test.cpp
```

只有在必须为动态可行性提供纯 guidance bound/fact helper 时，才允许对以下现有文件做小范围扩展：

```text
src/swarm_planner/bspline_traj/include/bspline_race/guidance/isf_reference_kernel.h
src/swarm_planner/bspline_traj/src/guidance/isf_reference_kernel.cpp
src/swarm_planner/bspline_traj/test/isf_reference_kernel_test.cpp
```

任何其他文件都不在白名单。若正确实现需要修改 `gvf_manager`、planner、C2、SDFMap、cloud snapshot、launch 参数、Marker helper 几何或 PositionCommand 发布，立即停止并报告，不得扩权。

---

## 10. 明确禁止事项

禁止修改：

- `SDFMap`、cloud snapshot、occupycloud contract；
- Kino A*、B-spline optimizer、planner weights/safe distance；
- C2 connector、path install、replan 逻辑；
- tube ray search、margin、width、filter geometry；
- `0.55/0.10/0.45` accounting；
- `tracking_error_bound=0.15`；
- `u_w/u_delta` amplitude 或 rate；
- `phase_dot_min/tangent_speed_min` 正常值；
- maximum velocity、K1/K2、governor、SO3；
- 100-cycle gate；
- Candidate/Certified Marker 几何；
- baseline launch defaults；
- swarm、neighbor、CBF、messages；
- low-level emergency/hover publisher。

禁止新增调参开关来绕过本阶段逻辑。该状态机和检查在 manual fixed/ESDF tube 启用时是合同本身，不是实验性 display toggle。

---

## 11. 单元测试与回归要求

至少新增/更新以下测试：

### core/projector

- positive limits 行为与当前完全一致；
- nonnegative lower margins 可表达 `U>=0`；
- tube-disabled A4 exact regression 不变；
- empty polygon 仍明确返回 invalid reason；
- 不改变 matched port 公式。

### dynamic feasibility helper

- G2d `lower_w=0.5775` 复现；
- fixed wide tube `POSITIVE_FORWARD`；
- `NONNEGATIVE_ONLY`；
- next-knot infeasible；
- slew infeasible；
- regularity infeasible；
- indeterminate inputs；
- no mutation/finite deterministic result。

### TubeEpochManager

- dynamically infeasible candidate never installs；
- positive candidate installs ROLLING；
- nonnegative-only latest safe candidate installs SAFETY_PRIORITY；
- WAITING does not overwrite active profile but invalidates current certificate；
- latest explicit unsafe produces EMERGENCY_REQUIRED；
- no `delta` reset；
- candidate/result sequence cannot be mixed；
- equivalent refresh/epoch counts unchanged；
- `dynamic_feasibility_evaluated` true only for actual evaluation。

### Runtime

- SAFETY_PRIORITY uses zero lower progress margins but all other limits；
- safety mode uses same final port internally/physically；
- WAITING and EMERGENCY do not mutate delta/previous port；
- tracking exceed does not set fatal；
- U+ empty does not set fatal；
- U>=0 empty does not set software failure latch；
- certified/projector contradiction does set genuine fatal；
- next-envelope/math invariant failure still fatal；
- fixed/manual existing tests stay green。

### Adapter/diagnostics/Marker

- transient WAITING/tracking/emergency leaves `failure_latched=0`；
- true zero-port/matched/projector contract breach latches；
- reason enum retained and published；
- manual 83 schema exact；
- epoch old 0–48 indices unchanged, appended schema exact；
- Candidate action depends only on finite complete candidate；
- Certified ADD/DELETE agrees with epoch dynamic certificate and runtime mode；
- fatal latch first Marker cycle is three DELETE。

运行 completed-stage regressions：

```text
phase_offset_core
phase_offset_navigation
bspline_race guidance/integration tests
G2d cloud snapshot/query tests
marker helper tests
```

---

## 12. 构建与 ROS 验收

先运行增量构建；测试结果目录必须清理当前任务产生的 stale XML 后再统计，不能把历史 XML 当本轮证据。

### ROS-0 baseline

裸 `test_gvf.launch`：

- actual params `disabled/none/refresh=3.0`；
- manual/candidate/certified/epoch diagnostics 无 publisher；
- `/position_cmd` 唯一 publisher 仍为 `/formation_planning`；
- 原 pillar 点到点到达；
- 不启动新 emergency publisher。

### ROS-1 fixed

显式 manual/fixed：

- gate 第 100 周期打开；
- dynamic feasibility evaluated；
- `U+` feasible；
- ROLLING；
- Candidate/Certified 正常可见；
- same final port residual；
- failure/emergency=0；
- 83 和 epoch schema 每帧严格一致。

### ROS-2 ESDF/cloud snapshot

沿用 G2d 的 pillar/local_sensing/PointCloud2 合同和参数；不得调速度、端口 limit、tracking bound、地图或目标来规避状态。

必须录制：

```text
/formation_planning/phase_offset_manual/diagnostics
/formation_planning/phase_offset_manual/tube_epoch_diagnostics
/formation_planning/phase_offset_manual/tube_raw_candidate_diagnostics
/formation_planning/phase_offset_manual/tube_cloud_snapshot_diagnostics
/formation_planning/phase_offset_manual/tube_candidate
/formation_planning/phase_offset_manual/tube
/position_cmd
odom/local_map/goal
```

验收重点不是强求全程永远 rolling，而是证明状态分类正确：

- 所有 active install 前 dynamic check 已完成；
- 不再出现“install 后下一帧 joint polygon empty -> permanent latch”；
- 若复现 G2d 陡峭边界，必须在 install 前得到 NONNEGATIVE_ONLY、INFEASIBLE 或 INDETERMINATE；
- `failure_latched` 保持 0，除非注入真正 invariant failure；
- emergency-required 与 software fatal 明确分开；
- WAITING/emergency 时 Certified 三 DELETE；
- Candidate 可继续显示完整未认证几何；
- rolling/safety-priority 窗口中 Marker 与 diagnostics 每帧一致；
- snapshot raw access/self-free 仍为 0；
- margin 仍为 `0.55/0.10/0.45`；
- no mixed Marker actions；
- no `w`/`delta` reset or jump。

保存原始 bag、严格 CSV、JSON report、分析脚本和 SHA-256。按 bag message time 对齐并报告最大配对间隔。

若 ROS 不可用或用户已有 ROS master/process 冲突，按 AGENTS.md 停止；不得附着或终止用户进程。只清理本任务启动的进程和端口。

---

## 13. 自审核与停止

最终必须执行并报告：

```bash
git diff --check
git status --short
git diff --cached --name-only
```

同时检查：

- 白名单外文件改动数为 0；
- G2d map/cloud 文件 SHA-256 与执行前一致；
- planner/C2/gvf_manager/launch/Marker geometry 未改；
- core/navigation 无 ROS、SDFMap、visualization、neighbor、swarm、CBF；
- integration 无 PositionCommand publisher；
- manual schema 83；
- epoch schema old indices stable；
- no staged/commit/tag/push；
- branch/HEAD/stash 保持；
- 用户进程未触碰。

报告中必须单独列出：

1. G2d latch 的精确旧触发路径；
2. 新 install transaction；
3. `U+`/`U>=0` 的实现和前视充分性边界；
4. WAITING/SAFETY_PRIORITY/EMERGENCY/FATAL 的区别；
5. 是否仍有低层 emergency override 未实现；
6. ROS 中每次 install 的 dynamic certificate；
7. 所有证据路径和 SHA-256；
8. 未修改项。

完成 A5-G2e 后立即停止。

不得进入 A5.3、A6、多机、CBF、planner tuning 或场景扩展。

