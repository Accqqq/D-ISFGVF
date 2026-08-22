# A6-H2 上游路径 / 当前状态兼容性：只读审查与后续架构边界

```text
DOCUMENT_ROLE=EVIDENCE_FIRST_READ_ONLY_AUDIT
DOCUMENT_STATUS=FINDINGS_COMPLETE__NO_PRODUCT_CHANGE_AUTHORIZED
STAGE=POST_A6_H2_UPSTREAM_CURRENT_STATE_COMPATIBILITY
AUTO_ADVANCE=false
```

## 1. 结论

H2 没有把一条本可执行的 tube 错误地拦下；Filter 也不是本次 `HOLD` 的首因。
实际问题是：上游路径 / 手工零偏移执行律没有保证在其将要经过的整个区间内仍存在
`delta=0` 的稳健可行截面。旧 Active profile 在前方已经变成单侧
`[+0.125,+1.325]`；Runtime 的 retained delta 仍为 `0`。车辆走到该区间时，
`CURRENT_OFFSET_OUTSIDE` 是正确的 fail-closed 结果。

因 timer-H2 的新 owner 必须保留 `[captured_w0,future_seam]` 旧路径前缀，新的
C2 connector 无法改变 capture 时刻的同一世界点。故它不可能借由换 seam、CAS、pin、
profile 复用或 Filter 调整修复这个当前状态。强装新 pair 会绕过安全合同。

分类是混合的：

1. **上游 planner / tube / execution compatibility 合同缺失**；
2. **恢复桥接缺失**（不是 H2 原子提交缺失）；
3. **已配置净空模型存在冲突**，但本审查不授权调参；
4. 没有证据支持采样、坐标或 exact-PWL 查询 bug；
5. “map 从 51/49 更新到 57/54 才使旧证书失效”不是本次根因。

## 2. 可复核的动态证据

P0 已首先固定 H2 first-false 位于 fresh staged
`TubeEpochManager::update()`：

```text
/tmp/a6_h2_p0_20260816/run6/gdb.transcript.txt
captured_w0=3.4867096991473838
reason=CURRENT_OFFSET_OUTSIDE
candidate_raw/filtered/complete=true/true/true
retained_delta=0; retained_delta_current_inside=false
forward_horizon_sufficient=true
```

P0.5 的可复现 run3（old map 51 -> new map 57）进一步给出 capture 精确样本：

```text
/tmp/a6_h2_p05_20260816/run3/gdb.transcript.txt
captured_w0=3.5282144601532073; retained=0
owner p=(3.498540234,0.326778079,1.019138662)
actual =(3.480972919,0.319137539,1.037919705)
snapshot residual effective radius=0.45
reference clearance=0.426778079; actual clearance=0.419137539
raw=[+0.125,+1.325]; filtered=[+0.125,+0.864691]
```

因此 `0` 在 raw 几何已经被排除，Filter 只进一步收缩 upper bound，不能是 first
exclusion。reference / actual 都是已知自由点而非硬碰撞，但分别比 residual robust
clearance 少约 `0.0232 m` / `0.0309 m`。

为了检验“只因 map 更新才出错”的假设，另起完全隔离的 private ROS master `11705`，
未连接端口 `11311` 或用户进程。其运行目录为：

```text
/tmp/a6_h2_upstream_compat_20260816/run1
adapter Build-ID d6bb62f6f96ba4389ce8f6abe1e9072f87c7c558
adapter SHA256   e0cd24ef443d3c83706292f40caad665132c2b6c44cae26d75fad18a8380ef94
```

该 trace 不在 GDB 内调用 C++，而是按
`CloudOccupancySnapshot` 的公开 dense-vector / closed-voxel 算法读取两个 immutable
snapshot。`gdb.transcript.txt:97-103` 记录：

```text
old pair: revision=1, map=49
new candidate: revision=2, map=54
captured_w0=3.5399780711924902; retained=0

old map49  owner clearance=0.426380810 < 0.45
old map49  actual clearance=0.418468352 < 0.45
new map54  owner clearance=0.426380810 < 0.45
new map54  actual clearance=0.418468352 < 0.45

old Active nearest profile sample raw=[+0.125,+1.325]
new Candidate exact-current sample raw=[+0.125,+1.325]
```

两个 immutable snapshot 对同一 owner / actual 点的答案相同。这证明此复现中不是
`49 -> 54` 地图更新添加障碍；旧 Active profile 的前向部分本身已经不含 zero。
它与 run3 的 `51 -> 57` 现象相互印证。该运行只清理了任务自有进程；其
`processes_prestart/postshutdown.txt`、`ports_prestart/postshutdown.txt`、GDB transcript、
loaded-library maps、ROS logs 和 hash 均保留在上述目录。

## 3. 静态合同审计

### 3.1 H2 不可能修正 capture 当前点

timer-H2 在
`gvf_manager.cpp` 的 `buildPhaseV2C2Frontend()` 中构造：

```text
[captured_w0, future_seam_w]  appendSlice(old_path)  -- 世界几何逐点相同
[future_seam_w, join_w]       quintic Hermite
[join_w, path_end_w]          new mapped B-spline
```

timer 模式下 `prefix_start=phase_at_switch=captured_w0`。故 capture 点和所有 seam
前前缀都来自 old owner；future seam 只影响其后的连接器。prepared build 在
`phase_offset_matched_adapter.cpp:1295-1363` 也以新 owner 的 `captured_w0` 状态构建
current slice，并要求 retained delta 位于其中。把 old tube/sample/world geometry 套到新
connector 上既不正确，也不能解决当前点。

### 3.2 旧 profile 可以在前向处失去 zero，但安装只核验当下

`tube_epoch_manager.cpp:429-440` 对 Candidate 的当前 `w` 查询 PWL bounds，并以
retained delta 是否在 current interval 内决定 installability。它不宣称整个 certified
segment 都包含 zero。上面 old map49 的 active sample 已表明，profile 向前可以变为单侧，
而飞行中的 retained delta 仍是 zero。

`phase_offset_runtime.cpp:345-427` 的 manual preflight 仅对 `±amplitude` 做几何
regularity / 高度检查，不查询 tube bounds；随后 `:430-455` 生成与 tube 无关的正弦
`delta_ref`。它没有产生“沿当前 tube 可行分支的 delta reference”这一合同。因此，不能把
profile preflight 通过误读为 zero 或这条正弦在前向 tube 内可行的证明。

### 3.3 planner 净空与 tube robust 净空不是同一合同

本运行的有效参数和 tube 边界为：

```text
planning/safe_distance = 0.40
search/margin          = 0.50
tube residual radius   = 0.45 = 0.25 + (0.10-0.10) + 0.05 + 0.15
tube full radius       = 0.55 = 0.25 + 0.10 + 0.05 + 0.15
```

该数值来自 private launch log 和 `RobustTubeMargins::full/residualEffectiveRadius()`。
`KinodynamicAstar::search()` 初始化 start node 时没有先检验 start point 的 clearance
（`kinodynamic_astar.cpp:54-81`），只对后续 propagation samples 使用 mutable SDF 与
`search/margin`（`:251-276`）。而 `astaropt()` 对 replan 使用 old
`pm.last_traj(i0)` 作为 KINO start（`gvf_manager.cpp:6058-6085`），B-spline optimizer 的
距离 cost 使用 `planning/safe_distance`。所以“KINO 成功”不证明 capture state 满足
immutable snapshot 的 residual `0.45`，更不证明完整 tube robust `0.55` 合同。

这不是本单授权通过调 `safe_distance`、margin、速度或 tube margin 掩盖的 bug；它是后续
研究/架构决策必须显式统一的合同。

### 3.4 已存在 HOLD，缺少 tube-infeasible recovery bridge

Runtime 会把 current retained offset 不在 active PWL interval 的情况报告为
`WAITING_FOR_CANDIDATE`，或在 witness 不存在时报告 `CERTIFICATE_DENIED`
（`phase_offset_runtime.cpp:519-537, 878-891`）。当前 Adapter output 已携带这些既有
`runtime_execution` 与 `tube_epoch_status` 字段。

但 cmd callback 仅在 `matched_output.selected` 时替换 `out`；non-selected 后原基础
guidance 仍可能保持 valid 并继续被 governor 消费（`gvf_manager.cpp:1335-1465`）。现有
`makeGovernorInvalidHold()` 只会在 guidance / governor 本身无效或 path-end clamp 时发出
位置 HOLD（`:729-741, 1032-1034`）。没有从现有 `CURRENT_OFFSET_OUTSIDE` /
certificate denial 到现有 `REPLAN_TRAJ` 或物理 emergency 的明确桥接。历史
`switch_emergency` 注释块不是有效路径，不得当作现有实现。

这不是仅有的静态推断。第二个隔离 private run 的
`/tmp/a6_h2_upstream_compat_20260816/run3_runtime_waiting/gdb.transcript.txt:123-125`
在 Runtime 输出边界记录了实际消费链：

```text
current_w=2.529100040
current bounds=[+0.044317367,+1.360038422]
retained/output delta=0; retained_inside=false
Runtime mode=WAITING_FOR_CANDIDATE
requires_base_guidance=false; selected=false; valid=false
raw port=(0,0); matched_valid=false
```

紧随的同次 command log 仍是：

```text
final_cmd_source=VEL_MATCH_GOVERNOR
fallback_reason=none
fallback_hold_pos=0
```

故 Runtime 没有执行 matched port，也没有写入下一 `delta` / final port；而 manager
仍发布由原 base GVF 驱动的 position command。另一个隔离 run 在 current bounds 仍含 zero
但 bounded witness 失败时得到同样的 `CERTIFICATE_DENIED, selected=false, valid=false`
后继续 `VEL_MATCH_GOVERNOR`，证明该桥接缺口不限于本次 raw-outside case。运行的
Build-ID、SHA、maps、process cleanup evidence 与 GDB transcript 均保存在
`/tmp/a6_h2_upstream_compat_20260816/run2_runtime_port` 及 `run3_runtime_waiting`。

因此“直接 HOLD”也不是理论上充分的恢复：当前点仅证明失去 robust certificate，未证明
位置 HOLD 是可持续的安全恢复控制；它不能取代一个明确、经验证的 recovery contract。

## 4. 排除项

- **非 Filter bug**：raw lower 已为 `+0.125`，Filter 并非首个排除层。
- **非 H2 scheduler / C2 / CAS / pin bug**：C2 成功、old pair identity/pin 已在 P0 验证；
  fail 在 local staged manager current state。
- **非 profile query / 坐标 mismatch**：new current sample 的 `w` 与 capture 精确相等，
  `p/N` 来自 immutable new owner；old/new snapshot dense scan 与 manager clearance 一致。
- **非“必须复用旧 tube”**：old map49 同一点也无 0.45m residual clearance，复用不安全。
- **非单纯地图更新故障**：两个连续 immutable snapshot 对本次 current point 的结果相同。

## 5. 此后阶段的最小架构，不在本单实施

在导师/用户明确选择理论路线前，禁止改 A5 geometry、Filter、SurfaceValidator、margins、
速度、lookahead、rates、CAS/pin/session/runtime bit 或 current/seam contract，禁止增加
gate/mode/state/reason/enum/latch/diagnostic schema。

后续工作必须拆为两个独立授权阶段，不能再作为 A6/H2 “激活修补”。

### P1：路径—tube—执行可行性合同（先作设计/测量）

先用 immutable snapshot 和实际 execution law，测量每个被接受 path 从当前 state 到下一次
replacement 前的 feasible `delta` component、其 rate-limited 可达性及 zero reference 是否
可持续。必须在以下路线中作明确选择，不能悄悄调参数：

1. 将 planner 的 path acceptance 证明提升到 tube 的 robust zero-offset 合同；
2. 将执行律从与 tube 无关的双侧正弦改为经理论批准的、tube-aware feasible
   `delta*(w)` / port policy，并证明从 retained state 的可达性；
3. 若当前状态已经不满足上述合同，声明进入 recovery domain，而不是安装/执行一个伪证书。

P1 的验收不是“某个 Candidate complete”，而是：任何允许进入 normal execution 的 owner
都对实际 retained delta 和已有 port-rate limits 给出连续 executable witness。若选路线 1，
不能把 `.40/.45/.55` 的不一致靠参数改写隐藏；若选路线 2，则属于论文/控制模型修改，先获
研究授权。

候选文件白名单（仅在新的 P1 执行单、完成理论选择后）：

```text
src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h
src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
docs/<new P1 execution specification>
```

其中不允许改变 `TubeFilter`、cross-section 或 H2 transaction；Adapter 只能传递已有
immutable path/profile/snapshot，不得复用跨 owner 的几何。

### P2：当前 robust certificate 已失效时的 recovery bridge

P2 的入口只能消费已有 `TubeEpochStatus` / `RuntimeExecutionStatus`，不新增状态枚举或
诊断字段。它必须明确区分：

- 候选暂缺但 old current certificate 仍有效；
- retained state 已不在 current tube；
- categorical collision/unknown；
- 理论已经证明的物理 emergency command。

该阶段要把第二、三类交给一个已有 FSM replan / 经授权 physical recovery owner；不得让
non-selected silently fall back to legacy base GVF，也不得将 generic HOLD 误称为碰撞恢复。
对于 current unsafe start，recovery planner 必须有单独的起点合同；不能用 old C2 prefix 或
旧 certificate 伪造“当前安全”。

候选文件白名单（仅在单独 P2 执行单获准后）：

```text
src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
docs/<new P2 execution specification>
```

不得在 P2 修改 A5 geometry / Filter / margins / H2 CAS 或通过 timer 重试、新 gate、latch、
mode、reason 规避该问题。H2 完成后仍须原封不动回归测试；它只保证原子 ownership，不提供
失去当前 robust certificate 后的逃逸控制。

## 6. 当前停止点

本审查只增加本文件。未改产品源码、测试、参数、AGENTS.md 或启动文件；未接触用户 ROS
master / 用户进程。后续需要用户/导师先选择 P1 的理论路线，并单独授权 P2 recovery
architecture，才可实施任何代码修改。
