# Codex A5R-1D Runtime-Ready Timing Acceptance Correction

```text
DOCUMENT_ROLE=ACCEPTANCE_CORRECTION_AND_MEASUREMENT_EXECUTION_PLAN
DOCUMENT_STATUS=APPROVED
IMPLEMENTATION_AUTHORIZED=true
AUTO_ADVANCE=true
STAGE=A5R-1D
PRODUCT_SOURCE_CHANGES_AUTHORIZED=false
PRODUCT_PARAMETER_CHANGES_AUTHORIZED=false
PRODUCT_DIAGNOSTICS_SCHEMA_CHANGES_AUTHORIZED=false
A5R_2_SOURCE_CHANGES_AUTHORIZED=false
A6_SOURCE_CHANGES_AUTHORIZED=false
```

> 日期：2026-08-12  
> 仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
> 修正范围：仅修正
> `Codex_A5R_Tube_Performance_And_Installability_Recovery_Execution_Plan_2026-08-11.md`
> §5.1 中 Candidate/usable-install 的计时端点；其余安全、几何、版本和回归要求不变。

## 0. 结论

A5R-1D **不修改产品代码或调度**。A5R-1C 的约 165 ms 指标把下一次 10 Hz
`publishManual()` 的 marker/50-field 外显等待算进了 Candidate/usable-install，不能代表
Runtime 实际何时可消费新 epoch。

本修正不放宽既有 100 ms 工程预算，也不新增任何运行 gate。它只把预算应用到正确的
执行链端点，并把可视化/诊断外显节奏单独报告。

## 1. 已确认的执行链

```text
TubeBuildRequest stamp
  -> TubeEpochManager::update()
       -> Candidate 判定
       -> 既有 Active 安装/刷新（若可安装）
  -> finalizeTubeEpoch()
       -> source-current 检查
       -> atomic latest_candidate_epoch_snapshot_
       -> atomic latest_epoch_snapshot_（Runtime-ready）
  -> 下一次 50 Hz update()
       -> matchingEpochForRequest()
       -> Runtime::prepare()/complete()
       -> atomic latest_control_snapshot_（first command consume）

独立外显链：
下一次 10 Hz timer -> publishManual() -> marker + 83/50-field diagnostics
```

`publishManual()` 不授权 Candidate、Active、Certified 或控制消费；控制链不等待 marker。

## 2. A5R-1C 旧指标的解释

三次有效 episode 的已有证据：

| 指标 | p50 | p95 |
|---|---:|---:|
| request -> raw timer diagnostic | 38.024 ms | 67.317 ms |
| raw diagnostic -> 50-field due | 99.923 ms | 112.070 ms |
| request -> 旧称 Candidate published | 137.710 ms | 164.983 ms |

同一 Candidate 的 50-field DUE 与下一次 RAW build 几乎同时出现：
`DUE - next RAW` 为 `n=166, p50=0.022 ms, p95=0.227 ms`。因此旧指标多计了约一个
10 Hz 外显周期。A5R-1C 的原 `NO-PASS` 记录保留，不追溯改写；它证明旧外显端点没有
满足 100 ms，不证明 Runtime-ready/first-consume 已失败。

## 3. 修正后的计量定义

### 3.1 Candidate atomic exposure

起点：冻结 `TubeBuildRequest::stamp`。  
终点：source-current 检查通过后，完成
`latest_candidate_epoch_snapshot_` 的原子写入。

source-stale build 保留在分母和报告中，但不得算作可用 Candidate。

### 3.2 Runtime-ready exposure

起点：同一冻结 request。  
终点：同一 build sequence 完成 `latest_epoch_snapshot_` 的原子写入。

只有既有 manager 结果本来可安装、source-current 且 epoch/request contract 匹配的行进入
usable-install 时效统计。unsafe、unknown、incomplete、short-horizon 和 source-stale
按其真实类别报告，不伪造成性能失败或安装成功。

### 3.3 First command consume

起点：同一冻结 request。  
终点：首个读取同一 build sequence、调用既有 `Runtime::prepare()/complete()` 并形成
`latest_control_snapshot_` 的 50 Hz `update()`。

`selected`、100-cycle warmup、observe-only 和 marker 状态都不得被当作 consume 标志。

### 3.4 External evidence cadence

marker、83/50-field diagnostics 的 request-to-publish 延迟和发布率继续完整报告，但它们
属于 10 Hz 外显证据链，不进入 Runtime 100 ms deadline 判定。不得为缩短该数值而：

- 提高 timer/publish rate；
- 把 marker 构造搬回 command callback；
- 新增 pending/gate/latch/state/reason/schema；
- 修改 tube 几何、margin、slope、lookahead、速度或饱和。

## 4. 白名单

允许：

- 本执行单；
- `/tmp` 下的 measurement-only fixture、脚本、CSV/JSON/Markdown 证据；
- 读取已有 build artifact、bag、CSV 和源码；
- 使用已有 adapter 测试的 test-scoped private inspection 方式构造 `/tmp` probe；
- 私有 loopback ROS master 上的只读/测量运行，但不得修改产品 topic/schema。

禁止修改任何产品源码、测试源码、CMake、launch、参数、`AGENTS.md` 或默认 ROS master。

## 5. 执行与验收

1. 对固定 semantic revision 的完整、source-current、可安装 Candidate，至少采集三组独立
   measurement-only 运行；
2. 报告 request -> build completion、Candidate atomic exposure、Runtime-ready exposure、
   first command consume 的 p50/p95/max 和 100 ms miss；
3. 保留既有 100 ms 工程预算，不新增第二个 deadline；
4. 报告 first-consume service rate，固定 revision 活动窗口不得形成 backlog；
5. 单独报告 marker/50-field cadence，不据此判定执行时效；
6. 运行现有 adapter/manager/runtime 回归、dependency search、`git diff --check`；
7. 输出自审，明确没有产品改动、没有新增门控或 schema。

若 `/tmp` fixture 只能证明确定性调用链、不能证明真实 ROS callback jitter，则结果必须拆成：

- `DETERMINISTIC_RUNTIME_CHAIN_PASS/FAIL`；
- `LIVE_CALLBACK_JITTER_NOT_DIRECTLY_MEASURED`。

不得用 nominal 50 Hz 推算值冒充 live p95。

## 6. 后续边界

A5R-1D 只纠正性能计量语义。A5R-2 仍需独立执行单验证固定 revision 下 Candidate ->
ROLLING/Active/current-valid -> Certified -> Selected、覆盖与 fail-closed。A5R-3 依据 A5R-2
真实失败类别决定是否需要后续代码执行单。corrected A6 仍禁止复用旧 path tube 到新
quintic connector。

