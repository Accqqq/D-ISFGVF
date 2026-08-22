# Codex A5R-1C Prepared Path Geometry Execution Plan

```
DOCUMENT_ROLE=IMPLEMENTATION_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=A5R-1C
IMPLEMENTATION_AUTHORIZED=true
AUTO_ADVANCE=false
AUTHORIZATION_SCOPE=ONE_EXACT_SURFACE_VALIDATOR_GEOMETRY_HOTSPOT_ONLY
```

> 日期：2026-08-12  
> 仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
> 证据根目录：`/tmp/a5r_1c_20260812T033000Z_prepared_geometry`  
> 本单只授权 SurfaceValidator 中同一 cell、同一 `w` 行的重复 path/profile/geometry
> 求值消除。它不授权第二热点、A5R-2/3、A6、A7，且不改变 tube 的几何、安全集合、
> 证书、运行时或控制语义。

## 1. Findings-first 与唯一目标

post-A5R-1B 只读复测位于
`/tmp/a5r_1c_readonly_20260811T182716Z`。54 次 instrumented build 的主要结果为：

| 项目 | mean / build | p95 |
| --- | ---: | ---: |
| complete build | 108.675 ms | 194.799 ms |
| SurfaceValidator inclusive | 54.094 ms | 146.701 ms |
| SurfaceValidator own exclusive | 48.687 ms | — |
| TubeFilter | 27.103 ms | 65.529 ms |
| Builder inclusive | 27.398 ms | 55.532 ms |

细粒度 edge probe 的 55 次 Validator 中，57,909 个 cell-`w` 行对
`TubeFilter::query`、`PathStateQuery` 和 `GeometryEvaluator::evaluate` 都各调用三次；
Validator 内 GeometryEvaluator 子调用约为 **39.296 ms/validation**，是当前已定位的单一
主热点。单独 hoist Filter/Path 的理论收益上限只有约 1.484 ms/validation，不足以作为独立
性能修复。

本单的唯一目标是：

1. 在 `phase_offset_core` 内以单一公式实现 prepared path geometry；
2. 同一 Validator cell 的同一 `w` 行只计算一次 path-only 几何；
3. 三个原有 `delta` 点仍依次执行全部 regularity、active-reference、tangent、error 与
   finite/fail-close 检查；
4. Validator 只接收其消费的 `r`、`regularity`、valid/reason，避免构造完整
   `PhaseOffsetGeometryState`；
5. 保持 profile、证书、clearance 点/半径/顺序、递归和失败语义输出等价。

明确拒绝“只算 `regularity+r`”的捷径。只读 extreme-finite corpus 中该捷径 200,000 个
case 出现 6,539 个 bool mismatch，会把原 full evaluator fail-close 的输入错误放行。

## 2. 核心思想与全局禁止项

以下内容完全冻结：

\[
\mathcal T=\{p(w)+\delta N(w):\delta\in[\delta_-(w),\delta_+(w)]\}
\]

- 2.5D 水平法向、曲率与 `regularity = 1-kappa*delta` 的定义；
- 非对称 bounds、clearance/erosion、unknown fail-close；
- adaptive preview、TubeFilter、C1 profile、完整 ribbon surface cover；
- immutable snapshot、Candidate/Active ownership、Runtime 与 matched port；
- margin、slope、lookahead/back、rate、速度、饱和与所有阈值。

禁止新增或修改任何 gate、state、mode、reason enum、certificate、latch、diagnostics schema、
ROS topic/field、参数或 launch。禁止跨 cell、递归节点、profile、validate 调用、snapshot 或
path revision 缓存。禁止 clearance cache、减少九个 clearance 点、改变 query cap、递归深度、
cover 公式或 child 顺序。禁止在 navigation 中复制 geometry 公式。

## 3. 穷尽文件白名单

允许修改的仓库文件只有：

1. `docs/Codex_A5R_1C_Prepared_Path_Geometry_Execution_Plan_2026-08-12.md`；
2. `src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/geometry.h`；
3. `src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/geometry_types.h`；
4. `src/swarm_planner/phase_offset/phase_offset_core/src/geometry.cpp`；
5. `src/swarm_planner/phase_offset/phase_offset_core/test/geometry_test.cpp`；
6. `src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp`；
7. `src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp`。

不改 CMake、`path_state_query.h`、Builder、Filter、epoch manager、runtime、adapter、planner/C2、
governor、simulator、SO3、launch 或参数。若实现必须越出白名单，立即停止并报告。

## 4. 获授权的精确实现边界

### 4.1 core prepared path geometry

在既有 geometry concept 内增加一个窄的 prepared path state 和 reference result；不得新建
第二套 geometry module。命名可按现有风格微调，但语义必须满足：

1. prepared path state 只保存当前 full evaluator 已计算的 path-only 事实：
   `p/p_w/p_ww/w`、`path_speed`、`horizontal_path_speed`、`N/N_w`、`curvature`，以及
   分阶段的 valid/reason；不得用一个提前返回的 `valid=false` 混合原 evaluator 在
   `position/delta` 前后的失败；
   reason 只允许在 `.cpp` 内保存稳定的 non-owning string literal（如 `const char*`），不得为
   prepared fast path 新增 reason enum、owning `std::string` 或逐点动态分配；含 Eigen fixed-size
   成员的公开结构必须遵守现有 alignment 规则；
2. path-only 运算与现有实现保持相同表达式和顺序；
3. 必须显式保持当前错误优先级：
   `params -> path.valid -> path finite -> position finite -> delta finite -> path speed ->`
   `horizontal path speed -> curvature/regularity -> active reference speed -> final finite`。
   prepared 过程可以预计算 path speed、horizontal speed、`N/N_w` 和 curvature，但这些属于
   `position/delta` **之后**的失败必须作为 delayed path-derived result 保存；Validator 不得因
   prepare 返回值直接跳过第一个 delta 的 point checks；
4. 既有 `GeometryEvaluator::evaluate()` 的 public 行为、错误优先级、invalid reason、有限值
   清理和完整输出逐字段保持不变。full evaluator 不得简单地在检查 position/delta 之前调用
   一个可能传播 delayed path-derived failure 的 prepare-and-return 路径；
5. reference evaluation 对每个 `delta` 仍按原顺序执行：
   - 若 params/path/path-finite 失败，先返回原有 pre-point reason；
   - 否则先执行 position/delta finite；
   - 再消费 prepared 中 delayed 的 path-speed/horizontal/curvature failure；
   - regularity finite 与 margin；
   - `r`、`r_w`；
   - active reference speed finite/threshold；
   - `T`、`error`、`e_parallel`、`e_perp`；
   - 与当前 `HasFiniteOutput` 等价的全部字段 finite 检查；
6. reference evaluation 只公开 Validator 实际需要的 `r`、`regularity`、valid/reason；
   省略输出字段不等于省略它们对应的计算或 fail-close 检查；
reference 无论 valid 或 invalid，其 `r`、`regularity` 和 reason 都必须与同输入 full
   evaluator 经现有 `Invalidate/EnsureFiniteOutput` 后的对应字段 bitwise 相同：regularity-margin
   失败保留已计算的有限 regularity；active-speed/final-finite 失败保留或清零 `r` 的方式也必须
   与 full 一致；
7. full evaluator 与 reference evaluator 必须复用 `.cpp` 中同一 path-prefix 和 terminal-tail
   实现，不能维护两份独立公式；
8. 不添加动态分配、线程、全局/static cache 或 ROS 依赖。

### 4.2 Validator cell-local row evaluation

`CollectCellPoints` 仍以当前顺序处理：

```text
w0: v0, vmid, v1
wmid: v0, vmid, v1
w1: v0, vmid, v1
```

每一行只允许：

1. 调用一次 `TubeFilter::query(profile, w, bounds)`；
2. 调用一次 immutable `PathStateQuery(w, path)`；
3. 准备一次 path geometry；即使 prepared state 记录了 delayed path-derived failure，也必须调用
   第一个 delta 的 reference evaluation，让其按原顺序先处理 position/delta；
4. 使用完全相同的表达式
   `delta = lower + clamped_v * (upper - lower)` 依次求三个 reference point；
5. 任一点失败时，在与当前实现相同的 `w` 记录同一 `TubeStopReason::REGULARITY` 并停止
   geometry collection。

`CellClearancePasses` 必须保持九个 point、requested radius、clearance query 顺序、全部失败后
继续扫描、min margin、query count/cap 和 first-failure 语义不变。不得跨行或跨 cell 保存
prepared state。

## 5. 输出等价与差分门

实现前必须从本单冻结的 **preimage source** 在证据目录中独立重建一份 baseline geometry
library/executable，冻结 compiler invocation、baseline executable、library、source 和 corpus 的
SHA-256。不能只信任当前 `devel/lib`，也不能用实现后的新 full evaluator 充当自己的历史
reference。所有 harness 与产物只写证据根目录。

### 5.1 full geometry pre/post corpus

同一个 external corpus 在 pre/post library 上分别执行，并逐字段序列化：

- return bool、`output.valid`、`invalid_reason`；
- `PhaseOffsetGeometryState` 的所有 scalar/vector IEEE-754 bit pattern；
- valid nominal、invalid path、NaN/Inf、极端 finite；
- tangent/horizontal tangent、regularity、active-reference-speed 阈值两侧；
- finite overflow/cancellation 与 lower/mid/upper delta。

pre/post 必须零差异。输出必须以 binary 或 IEEE-754 hex bits 序列化，不能依赖十进制文本
浮点 round-trip。不能只比较最终 fingerprint；报告同时给出 corpus 数、分类数和逐字段 diff
结果。

### 5.2 full vs prepared-reference differential

在同一 post 进程中，对固定 seed randomized + deterministic edge corpus：

- full evaluator 与 prepared-reference 的 bool、reason、`r`、`regularity` 在 valid 和 invalid
  case 中都 bitwise 相同；
- prepared path failure、第一/第二/第三 delta failure 都必须覆盖；
- 组合优先级必须覆盖：invalid params/path/path-finite 与 invalid position/delta；zero/invalid
  tangent 与 NaN/Inf delta；curvature/path-derived failure 与 invalid delta；regularity-margin、
  active-speed、final-finite failure 后 `r/regularity` 的保留/清零；
- 固定 collision cases 至少包含：invalid params + invalid path/position/delta；invalid path +
  NaN position/delta；nonfinite path + NaN position/delta；NaN position + NaN delta + degenerate
  tangent；finite position + NaN delta + zero/near-zero tangent；tangent/horizontal threshold 的
  `std::nextafter` 两侧；curvature numerator、speed-cubed、`N_w`、`r_w`、error/dot product 的
  finite overflow/cancellation；active speed 为零及阈值两侧；signed zero、NaN/Inf，以及
  `path.valid=false` 但部分字段有限的初始化输出；
- 不允许只测全部 valid 的圆路径。

任一 mismatch 立即停止，不得用容差放行。

### 5.3 Validator pre/post differential

固定递归 corpus 比较：

- return、`TubeSurfaceValidationResult` 全字段；
- profile samples/bounds/derivatives、complete/certified/termination、截断与 certified endpoints；
- clearance point、requested radius 和调用顺序 bitwise 相同；
- query_sample_count 与 query-cap/child/first-failure 行为相同。

PathStateQuery 次数允许按设计从同一行三次降为一次；测试必须证明重复同 `w` 的 immutable
query 返回 bitwise 相同 state，并明确断言无跨 cell memoization。除总数外还必须记录完整
path-call `w` 序列，证明每个 cell 的三行各自查询一次、递归 sibling 之间没有复用。现有
72-point fixture 应明确断言 `clearance calls == 72`、`path calls == 24`，并关联每个 path call
到随后同一行的三个 surface point。geometry 在一行第 1/2/3 个 delta 失败的 case 都要验证
同一 first reason/w 且该 cell 的 clearance 调用数为零。clearance 次数不得改变。

阶段 preflight 还必须只读审计生产 `PathStateQuery` binder：同一 immutable candidate 内重复
同 `w` 调用返回 bitwise 相同 state，并记录相关 source/binary hash；不能只凭测试 lambda 推断
生产契约。若生产 query 有可观察副作用或非确定性，本热点立即停止。

## 6. 性能、构建与 ROS 验收

1. 构建 `phase_offset_core` 和 `phase_offset_navigation`，运行两 package 的阶段单测及既有相关
   regressions；
2. 对相同固定 corpus 运行 pre/post CPU benchmark，分开报告 full evaluator、prepared-reference、
   Validator 总体；不得用 `/tmp` 预测 benchmark 代替获授权代码结果；
3. 以当前 ABI 重新编译任何 probe；instrumented 数据只作归因，不作 endpoint PASS；
4. 在私有 loopback ROS master、未 preload、未改 launch/算法参数下至少三次完整 endpoint
   episode。为满足本条的计时要求，唯一允许的 measurement-only prelaunch 参数是已有的
   `/formation_planning/phase_offset/measurement/enable=true` 和每次独立的
   `/formation_planning/phase_offset/measurement/tube_due_csv_path=/tmp/...`；它们只记录既有
   sidecar CSV，不改变 tube、控制、timer、publish rate 或安全语义。除此之外不得设置任何
   ROS 参数。报告 build p50/p95/max、request→current Candidate、request→usable install、
   100 ms misses、service rate、source-stale 和 load；
5. 用户默认 master/process 不得连接、终止或复用。存在用户 workload 时，只标 actual-load
   evidence；
6. 本子阶段性能 PASS 仍使用 A5R master 的 100 ms p95 / 10 Hz 条件。输出等价通过但性能未过
   时，结论必须为本热点 COMPLETE + overall NO-PASS，不得调参或顺手开始第二热点。

## 7. 审计与停止

- 保留 stage 前后 `git status --short`、tracked names、`git diff --check`；
- 因相关 package 可能 untracked，必须使用本单前冻结 SHA-256、重建 preimage 和完整 full diff
  审核，不得以空 Git diff 代替；
- 最终确认只有七个白名单文件发生本阶段变化；
- dependency-boundary search：`rg 'preparePath|evaluatePrepared|PreparedPathGeometry'`（按最终
  命名调整）只能命中 core geometry、SurfaceValidator 及各自测试；Builder/runtime/adapter 或
  其他生产调用出现新 API 立即停止；
- 保存 build/test/benchmark/ROS 原始数据、脚本、命令和 SHA-256；
- 不 commit/branch/tag/push，不 reset/restore/clean/stash；
- 完成报告和 self-audit 后停止本执行单，不自动修改 A5R-2/3、A6 或第二热点。
