# Codex A5R Tube Performance and Installability Recovery Execution Plan

```
DOCUMENT_ROLE=EXECUTION_PLAN_DRAFT
DOCUMENT_STATUS=DRAFT
IMPLEMENTATION_AUTHORIZED=false
AUTO_ADVANCE=false
STAGE=A5R
A5R_SCOPE=FIXED_PATH_TUBE_PERFORMANCE_AND_INSTALLABILITY
A6_SCOPE=PATH_TUBE_CONTINUATION_NOT_AUTHORIZED
A7_SCOPE=FULL_DYNAMIC_CLOSED_LOOP_NOT_AUTHORIZED
```

> 日期：2026-08-11
> 仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`
> 替代：`Codex_A5_Tube_Coverage_Corrective_Execution_Plan_2026-08-11.md`
> 撤回依赖：`Codex_A6_Revision_Continuation_Execution_Plan_2026-08-11.md`
> 本文件只重组证据、阶段和验收口径；**不授权任何源码、参数或 launch 修改**。

---

## 0. Findings-first 结论

A5 不能整体判定为“正确”或“推翻”。当前证据支持保留稳定安全契约，但不支持把
所有 Candidate 数值策略、安装策略与闭环语义冻结为已验证实现。

### 0.1 可冻结的长期契约

1. `ContinuousPhasePath` 提供精确 lifted-state 输入，offset 使用已批准的水平 2.5D
   几何；
2. local-sensing 仿真契约下，`full=0.55 → preincluded=0.10 → residual=0.45`
   的裕度只核算一次；
3. 正负法向 clearance 独立形成非对称安全 component，并与曲率正则区间求交；
4. filtered interval 必须包含于 raw-safe interval 内，不得通过放宽边界制造通过；
5. 一个 build 绑定不可变 path revision、map snapshot、preferred delta 与 provenance；
6. Candidate 与 Active/Certified 所有权分离，最新明确 unsafe 地图事实保持权威；
7. 禁止调小安全裕度、调大速度/斜率/管道尺寸或新增 gate/state/mode 掩盖失败。

### 0.2 只能暂存、必须重新验证的数值策略

- adaptive sampling 的细分阈值、深度和 status-transition 细分；
- cross-section component 选择；
- `continuous_inset = snapshot_resolution`；
- `boundary_slope_max / 1.5`、local repair 与 `(first,last)` 截断搜索；
- SurfaceValidator 的 cover 系数、递归策略和 query 上限；
- `min_certified_forward_w=0.4` 与当前 install policy；
- 柱子场景中斜率受限 tube 模型的适用性；
- OFFSET_OUTSIDE、HORIZON_SHORT、unsafe/unknown 与端口不可行时的 hybrid/replan
  处理是否完整符合 proposal。

在 A5R 测量与等价性能优化期间，上述数值输出作为**冻结基准 corpus**保留，目的仅是
防止性能改动顺带改变安全集合；这不等于宣告其理论选择已经通过。

---

## 1. 已确认事实与证据

### 1.1 当前 post-S5 未插桩构建耗时

| Episode | Builds | p50 | p95 | max | source-current finalized |
|---|---:|---:|---:|---:|---:|
| current run | 50 | 100.074 ms | 240.568 ms | 734.754 ms | 41/50 |
| repeat | 44 | 136.071 ms | 304.490 ms | 491.492 ms | 35/44 |

证据：

- `/tmp/codex_current_tube_perf_YEfsXo/evidence/tube_due_raw.csv`
- `/tmp/codex_current_tube_perf_YEfsXo/repeat/evidence/tube_due_raw.csv`
- `/tmp/codex_tube_hotspot_profile_20260811_223738/TUBE_PERFORMANCE_READONLY_AUDIT.md`

`source_current_finalized` 只表示 finalize 时请求路径仍为当前路径，**不等于** Candidate
可安装、`ROLLING`、Active 或 Certified。

### 1.2 安装性与失败类别

| Episode | Raw `ROLLING` | 主要非 ROLLING 事实 |
|---|---:|---|
| current run | 14/50 | 28 OFFSET_OUTSIDE；5 HORIZON_SHORT；3 certificate-denied |
| repeat | 15/44 | 24 OFFSET_OUTSIDE；5 HORIZON_SHORT |

这些类别可能包含真实环境/路径事实，A5R 不以“计数归零”为成功门，而以正确分类、
正确安装或正确 fail-closed/replan 行为为成功门。

### 1.3 函数级只读性能归因

42 次插桩 build 的 inclusive CPU 占比：

- `TubeSurfaceValidator::validate`：39.42%；
- builder/cross-section：36.86%；
- `TubeFilter::filter`：23.67%；
- cloud-clearance query：461,901 次，与上述阶段重叠，不得重复相加；
- finalize 平均 0.552 ms，diagnostics publish 平均 0.0096 ms，不是主因。

插桩只用于归因；性能门只使用未插桩运行。

### 1.4 已证伪结论

1. 中段路径前端并非恒定只有 0.98 m；该值只出现在终点附近；
2. 切换点 C2 不证明 connector 区间与旧路径相同，旧 tube 不能复用到 `join_w`；
3. 总 certified segment 包含 UAV 身后部分，不能替代发布时前向余量；
4. proposal 明确 `w` 不默认是严格弧长，不能未经证明用世界速度乘时间与 `w`
   长度比较；
5. 旧 T1 的 38 个有效 pair 中只有 3 个使用同一 map observation，不能支持
   “widespread model mismatch”结论。

---

## 2. 阶段边界

### A5R：固定路径/revision 下的 tube

A5R 只验证：

- 同一不可变路径与 snapshot 上的 Candidate 构建；
- Candidate 安装为 Active/ROLLING 的条件；
- Runtime 在固定 source revision 下的 tube 查询、端口与 active 控制；
- 构建性能、发布时效与失败分类。

revision churn 仅可用于测量 source-stale 压力，不得作为 A5R 已完成双 continuation 的证据。

### A6：路径/tube continuation

A6 需另发执行单，验证新路径、新 connector 与新 tube 的同版本候选、切换点
`w+=w-`、`delta+=delta-`、`r/r_w/e` 连续以及原子安装。安装前还必须验证当前
`delta` 在 new tube 内可行、new connector 上存在未来 `U+` / `U>=0` offset 演化；
最新地图若已经否定旧路径或旧 tube，不得为保持连续性继续使用旧证书。A6 必须复用
已有 base-path C2 connector，不得新增第二套 tube C2。A5R 期间只允许做 A6 只读设计，
不允许修改重叠源码。

### A7：完整动态闭环

频繁 A*/B-spline/C2 revision 下的完整点到点闭环属于 A7。A6 未完成前，不得用该动态
场景宣称 A5R 固定 revision 验收完成。

---

## 3. 全局禁止项

1. 禁止修改任何 margin、`boundary_slope_max`、lookahead/back、
   `min_certified_forward_w`、timer/publish rate、速度或饱和参数来制造通过；
2. A5R 主线禁止新增 gate/state/mode/reason/certificate/diagnostics schema；旧 T2
   真实截断标签已移出 A5R，只能由用户另行明确授权的独立 diagnostics-only 阶段覆盖
   本禁令；
3. 禁止把 `source_current_finalized` 当作安装或闭环成功；
4. 禁止把 OFFSET_OUTSIDE、HORIZON_SHORT、unsafe 或 unknown 的“消失”作为成功门；
5. 禁止跨独立 ROS 运行要求 `/position_cmd` bitwise 相同；
6. 禁止跨 map observation/path revision/preferred delta 做 T1 空间斜率结论；
7. 禁止把切换点 C2 当作区间几何相同或复用旧 profile 的授权；
8. 禁止同时实施 A5R 与 A6 的源码改动；
9. 禁止修改 planner/C2 主体、governor、simulator、SO3；
10. 禁止 reset/restore/clean/stash pop/commit/branch/tag/push。

---

## 4. 执行顺序

### A5R-0 — 只读基线闭合

#### A5R-0A：性能基线与热点归因

- 状态：**证据已采集，只读完成**；
- 权威审计：
  `/tmp/codex_tube_hotspot_profile_20260811_223738/TUBE_PERFORMANCE_READONLY_AUDIT.md`；
- 结论：性能是 source-stale/视觉时效问题的必要修复对象，但不能自动解决安装性失败；
- 本条不授权性能修改。

#### A5R-0B：正确 T1 与失败阶段定位

- 单一目标：在同一次 build、同一 path revision、同一 map snapshot、同一
  preferred delta 内，对相邻 raw samples 测量边界斜率，并记录零偏移/retained delta
  首次在哪一层被排除；
- 必须区分：raw cross-section、curvature/inset、TubeFilter、SurfaceValidator；
- component switch 必须单独分类，不能当连续导数；
- 现有 bag 不含完整 per-build raw profile，不能完成本条；
- 若需要 measurement-only 输出，必须先提交独立、白名单闭合的执行单，并优先使用
  sidecar CSV 或 `/tmp` 临时插桩；本草案不授权新增持久 ROS diagnostics 字段或修改
  既有 schema。

停止条件：得到可复核的 per-profile 样本 corpus 与方法说明；在此之前不得作 `(a)/(b)`
模型级结论。

### A5R-1 — 单热点、输出等价的性能优化

- 前置：A5R-0 完成，冻结 corpus、等价判据、端到端计时方法和文件白名单；
- 每个子阶段只允许优化一个实测热点；不得在同一子阶段同时修改 Validator、builder
  与 Filter；
- 每个热点必须另发 `IMPLEMENTATION_AUTHORIZED=true` 的执行单，穷尽列出文件与算法；
- 任何安全集合扩大、reason/state 变化或参数变化立即停止。

性能优化顺序由子执行单依据以下事实选择，不由本草案猜测具体实现：Validator 主导长尾，
builder/cross-section 主导稳定成本，Filter 为独立显著成本。

### A5R-2 — 固定 revision 的 A5 验证

- 使用已有固定路径 fixture、冻结输入 replay 或天然无 source revision 的可复现实验；
  禁止修改 planner/C2、launch 行为、sourceRevision 判定或 revision gate 来人为压制
  revision；
- observe-only 与 active 分成两套验收；
- 验证 Candidate、ROLLING/Active、current validation、Runtime port 与物理执行；
- 显式 unsafe/unknown 必须 fail-closed，不能为了“Active 连续”错误保留 Certified；
- 不在本阶段宣称 A6 continuation 或 A7 动态闭环通过。

### A5R-3 — 失败类别分支

| 事实 | 处理方向 |
|---|---|
| raw 当前截面已排除 retained delta/zero | 中心线净空、观测域或 component 事实；按 proposal §10.4 评估重规划 |
| raw 包含，但 inset/curvature 后排除 | 数值保守性/理论选择问题；需导师确认后另发几何执行单 |
| inset 后包含，但 Filter 后排除 | 斜率模型、repair 或截断策略问题；依正确 T1 决策 |
| Filter 后包含，但 Validator 后排除 | 表面证明保守性或实现问题；不得跳过 obstacle certificate |
| request 时可用、发布/安装前过期 | 性能/调度/版本时效问题 |
| explicit unsafe | 及时撤证并进入既有 safety/replan 语义 |
| indeterminate/unknown/incomplete | fail-closed 等待或重建，不得谎报 unsafe |
| U+ 为空但 U>=0 可行 | 按 proposal 的非倒退安全分支处理 |
| U>=0 也为空 | emergency/replan 语义，不能只做计数或静默 baseline 回退 |
| source-stale | 仅作为 A6/version 压力；不得复用错误 revision profile |

---

## 5. 硬验收定义

### 5.1 性能门

> **2026-08-12 计量端点修正：** 本节中把 timer-owned marker/83/50-field
> publication 当作 Candidate publication 或 usable install 的定义，已由
> `Codex_A5R_1D_Runtime_Ready_Timing_Acceptance_Correction_2026-08-12.md`
> 替代。100 ms 工程预算不变，但执行端点改为 source-current atomic exposure 与首个
> 50 Hz Runtime consume；10 Hz marker/diagnostics cadence 只单独报告。其余要求不变。

proposal §22.8 给出 10–20 Hz 建议频率；本计划把 10 Hz 串行实现对应的 100 ms
定义为**工程 deadline**，不是 proposal 的字面定理。

每个性能子阶段必须在私有 ROS master、未插桩条件下至少重复三次并报告：

1. build 内部 p50/p95/max；
2. request→Candidate publication：对所有完整 build 报端到端 p50/p95/max；
3. request→usable install：只对冻结 manager 结果本来可安装的 Candidate 报端到端
   p50/p95/max；真实 unsafe/unknown/incomplete 不得被计为性能安装失败；
4. 100 ms deadline miss 数与比例；
5. 实际 build/visible/usable-install rate；
6. timer inflight skip、source-stale 与调度延迟；
7. marker gap 仅作为显示节奏，不得冒充 request-to-marker latency；
8. 系统 CPU、load average 与并发 ROS 进程；存在竞争 formation_planning 时该运行只能
   作为实际负载证据，不能作为隔离性能 PASS。

性能 PASS 同时要求：固定 revision 下完整 build 的 request→Candidate publication
端到端 p95 不超过 100 ms，build/visible service rate 在活动非终端窗口持续达到至少
10 Hz，并且没有增长中的 backlog；对本来可安装的 Candidate，request→usable install
也必须满足相同 deadline。不得要求真实 unsafe/unknown Candidate 安装。若这些门与
固定路径实际前向余量仍不兼容，必须按实际 `delta w` 覆盖门收紧 deadline，而不是
宣称性能已解决覆盖。

### 5.2 输出等价门

在冻结 corpus 上：

- path/sample domain、reason/state、complete/certified/termination 状态完全相同；
- 若运算顺序不变，raw/filtered bounds 与导数要求 bitwise 相同；
- 若子执行单明确允许浮点重排，必须预先规定误差界，并证明优化后安全集合不比基准更宽；
- margins、query status、obstacle-certified 与 fail-closed 判据不得变化；
- 独立 ROS A/B 只做统计对比；bitwise 仅用于固定输入 replay、同进程 dual evaluation
  或函数级 fixture。

### 5.3 正确 T1 门

- 同一次 profile、同 path revision、同 snapshot、同 preferred delta；
- 对相邻 raw samples 报告 lower/upper 的 p50/p90/p99/max 与越限占比；
- 报告连续空间范围、component switch 与样本覆盖，不得事后发明 `(a)/(b)` cutoff；
- 若要作模型级结论，判定规则必须在看结果前由新执行单或用户/导师写明。

### 5.4 固定 revision A5 门

- 对每个可安装 Candidate，在 deadline 内正确安装；
- 对 explicit unsafe/unknown/incomplete，正确撤证或等待，零次错误 Certified；
- `source_current_finalized`、ROLLING、Active/current-valid、Certified 与 control-selected
  分层报告；
- 覆盖使用 `cert_end_w - w_publish` 与下一次 usable install 前的实际 `delta w`，
  零次 Selected/Certified 在边界外继续执行；
- terminal exception 必须预先定义，不能事后删除失败帧；
- observe-only 不改变控制；active 单独验证实际 matched reference、端口和物理命令；
- S4 tracking `<0.05` 当前仍为未通过/用户豁免，不得写为 PASS。若要修改标准，必须
  另发正式验收修订。

---

## 6. 与旧计划的迁移

| 旧条目 | A5R 处理 |
|---|---|
| A5 T1 跨帧斜率 | 撤回，由 A5R-0B 替代 |
| A5 T2 截断标签 | 移出 A5R；保留为需独立授权的 diagnostics-only 候选 |
| A5 T3 horizon/replan | 保留并并入 A5R-3 全类别 hybrid 语义 |
| A5 T4 命名 | 保留为独立低优先级候选 |
| A5 T5 / A6 prefix reuse | 撤回，错误几何前提 |
| A6 C2 事实透传 | 旧用途撤回；未来新 A6 是否需要事实接口由新设计决定 |

---

## 7. 每个获授权子阶段的共同交付物

1. 阶段前后 `git status --short`、tracked diff、冻结文件 SHA-256；
2. 明确且穷尽的文件白名单，验证无越界修改；
3. 适当 build、阶段单测、已完成阶段回归与 dependency-boundary search；
4. `git diff --check`；
5. 私有 ROS master，检查既有 master，不附着或终止用户进程；
6. 原始 CSV/bag/log、分析脚本、方法说明和 SHA-256；
7. 书面自审与显式停止声明；
8. 不 commit/branch/tag/push，不自动进入下一子阶段。

---

## 8. 当前停止点

本草案没有实现授权。当前仅确认：

- 旧 A5 corrective 已被替代；
- 旧 A6 已因错误前提撤回；
- A5R-0A 性能证据已完成；
- A5R-0B 正确 T1 需要新的 measurement-only 执行单；
- A5R-1 性能优化需要新的单热点白名单执行单；
- A6 可做只读设计，但不得与 A5R 并行改源码；
- A7 完整动态闭环尚未授权。

`AUTO_ADVANCE=false`：等待用户与审查者确认本草案后，再单独签发下一份执行单。
