# M3-L1 CertifiedTubeBuilder Pipeline Consolidation — Self Audit

日期：2026-08-20  
工作区：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

## 结论

状态：`STATIC_PASS_DYNAMIC_BLOCKED_OUTSIDE_WHITELIST`。

M3-L1 已将 production candidate 的 Builder/Filter/SurfaceValidator 串联收敛到
`CertifiedTubeBuilder`。静态差分、纯 C++ 回归、neutral 与 ESDF observe-only
launch 都通过。ESDF active 的后续阻塞已定位到
`bspline_traj` adapter 的 owner-aligned preview 没有显式放入当前 `w`，不属于
M3-L1 白名单；本阶段没有修改 adapter、Runtime、planner、C2、launch 或安全参数。

这不是“所有 active Tube 都构建失败”：流中第一个 candidate 已经完整构建。它没有
形成 initial Pair 是预期的 warm-up 行为：bootstrap 明确要求 `zero_gate_open_`，而
该首帧的 `w=0.05` 位于刚安装 planner owner 的起点；随后约 100 个 50 Hz zero-port
等价周期才打开 gate。到 gate 打开时，seq2+ 已因确定性的 current-sample 遗漏而
无法构成可 bootstrap 的完整 epoch。当前日志没有公开 Pair stage/dry-run/CAS 的
first-false 枚举，但不应把首帧无 Pair 误判为其被拒绝。

## 实现核对

- 新增 `certified_tube_builder.h/.cpp`：纯 C++ 组合现有 `TubeBuilder`、
  `TubeFilter` 和 `TubeSurfaceValidator`；header 不反向依赖
  `tube_epoch_types.h`。
- M3 入口严格保持旧顺序：raw build → ESDF snapshot provenance → single filter
  → nonzero capacity 判断/zero-only collapse → single ESDF surface validation
  → validation-failure zero-only collapse。
- `TubeEpochManager` 只持有 `CertifiedTubeBuilder certified_builder_`，每个
  candidate 只调用一次 `certified_builder_.build(...)`；其后的 current-state、
  retained-delta、install/retain/reject/wait/epoch policy 未迁移。
- source scan：manager production code 不含 direct `TubeBuilder::build*`、
  `TubeFilter::filter`、`TubeSurfaceValidator::validate`、旧
  `HasNonzeroCapacity` 或 `CollapseToPlannerZeroBaseline`。`TubeFilter::query`
  保留为 immutable profile 查询 API。
- 没有新增 ROS schema、参数、线程或 map query；内部 Builder/Filter/Validator
  算法未复制、未改写。

## 静态与单元验证

以下已通过：

- `phase_offset_certified_tube_builder_test`：6/6；
- `phase_offset_tube_epoch_manager_test`：53/53（原 52 + M3 结构回归 1）；
- CrossSection 7/7、Builder 9/9、Filter 16/16、SurfaceValidator 11/11、
  Runtime 35/35、MatchedAdapter 75/75；
- `gvf_switch_policy_test` 41/41、`continuous_phase_path_test` 11/11；
- `catkin_make -j2`；
- `git diff --check`。

核心回归构建命令：

```bash
catkin_make --make-args -j2 \
  phase_offset_tube_builder_test phase_offset_tube_filter_test \
  phase_offset_tube_surface_validator_test phase_offset_tube_cross_section_test \
  phase_offset_runtime_test phase_offset_matched_adapter_test \
  gvf_switch_policy_test continuous_phase_path_test
```

定向测试还逐项覆盖：无效子配置 fail-closed、FIXED old/new 等价、ESDF open
certificate old/new 等价、snapshot provenance 保留、zero-only、surface-proof
failure 的 zero-only fallback、raw incomplete 不升级为 zero-only。

## 隔离 launch 验证

所有 launch 使用新的 loopback ROS master、专属 `ROS_HOME`/`ROS_LOG_DIR` 与
专属进程组；未连接、未终止用户既有 ROS master 或节点。完成后本阶段启动的进程均已
清理。

1. neutral：`127.0.0.1:12881`，一个 `(8,0,1)` goal；四次 accepted C2，
   `POINT_GOAL REACHED distance=0.193`，无 endpoint-clamped hold。
2. ESDF observe-only：`127.0.0.1:12882`；确认 `mode=manual`、
   `observe_only=True`、`tube_source=esdf`；五次 accepted C2，
   到达距离 `0.191`，无 endpoint-clamped hold。
3. ESDF active：只覆盖
   `phase_offset_manual_observe_only:=false`；stream run 位于
   `/tmp/m3_l1_20260820_esdf_active_2_stream`，到达距离 `0.183`、四次
   accepted C2、无 endpoint-clamped hold。

active 的关键 raw evidence（49-field schema）：

| candidate | epoch | reason | raw/filter/complete | samples | current sample |
| --- | --- | --- | --- | --- | --- |
| seq1 | `ROLLING` | `NONE` | `1/1/1` | 42 | found/complete `1/1` |
| seq2+ | `WAITING_FOR_CANDIDATE` | `CANDIDATE_INCOMPLETE` | `0/0/0` | 通常 43–47 | found/complete `0/0` |

seq1 的 `w=0.05` 正好是 planner knot，因此完整。这证明 M3 的 actual ESDF
raw/filter/validator 通路能构成完整 candidate。seq2 的 current
`w=0.398624098655...` 不在 candidate preview/profile 中；raw diagnostic 的
`invalid_count=1` 与 `current_sample_found=0` 对应 Builder 的
`"adaptive preview does not contain the current phase sample"` fail-closed
分支。

## 白名单外 blocker

`src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
中的 `BuildOwnerAlignedTubePreview(...)` 仅把区间端点、supplied planner knots 和
owner segment endpoints 放入 preview。其两个调用方传入的范围围住 current `w`，
但 helper 没有接收或插入 current `w`。而 `TubeBuilder::buildCloudClearance()`
以 `1e-9` 容差要求 preview 包含精确 current sample。因此 current phase 一旦在
两个 planner knot 之间，candidate 必然 fail closed；这解释 seq2+，且与 ESDF
cloud usability、安全裕度或 M3 builder pipeline 无关。

该修复需要 adapter 改动，违反 M3-L1 §6 白名单，故未实施。后续独立任务应：

1. 令 owner-aligned preview 显式接收、范围检查并插入 exact current `w`，然后由
   同一 immutable owner evaluate；
2. 普通 timer 路径传 `request.current_path.w`，prepared Pair 路径传
   `captured_w0`；
3. 添加 between-knot current-w 的 timer 与 prepared Pair 回归；
4. 重跑 adapter 回归及同一 isolated active launch，并分别观测 raw/cloud/epoch/
   manual 与 Pair bootstrap result。

不得以降低 margin、改 planner sampling、换目标/地图或扩大 Tube 容差来绕开该
精确 provenance 契约。

## 未完成边界

- M3-L2、M4、M6、M7 未开始。
- seq1 未形成 Pair 的直接原因是当时 warm-up zero gate 尚未打开，而不是完整
  candidate 被 stage/dry-run/CAS 拒绝。后续若仍有 Pair failure，Pair 仍会额外经过
  `stagePathTubePair`、runtime dry-run、latest-map predicate 与 CAS；当前日志未公开
  其 first-false 分类。
- 未把这个现象标注为 M6/recenter boundary；先修复上述 preview plumbing 并重测。
