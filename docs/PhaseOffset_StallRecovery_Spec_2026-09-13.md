# 相位偏移层"永久悬停"的结构性根因与最小恢复方案（Spec）

日期：2026-09-13
状态：**Spec，未实施**

---

## 1. 问题：一个四步闭环

```
① 相位可以无限领先飞机（代码里没有任何"不得领先"的上限）
② tube 的扫描锚在【相位】上（相位 + 2.4 m），
   而"认证域/已知域"锚在【飞机】上（最近一次被接受的观测盒子）→ 两个锚会错开
③ 错开后 profile 变 PARTIAL；此时 δ ≠ 0，设计上不允许退回基准引导
   → out.valid = false → GOVERNOR_INVALID_HOLD
④ HOLD 只调用 requestRecenter() 并在注释里明确 "leave phase/Runtime untouched"
   → 状态一点不变 → 下一拍条件完全相同 → 永久自锁
```

代码位置：

| 步骤 | 位置 | 行为 |
| --- | --- | --- |
| ③ | `gvf_manager.cpp:2440-2446` | `retained_nonzero_section()` → `requestRecenter(); out.valid = false;` |
| ③ | `gvf_manager.cpp:2455-2461` | 同上（candidate 不存在时的另一条路径） |
| ③ | `tube_viability.cpp:1727` | profile 未覆盖 horizon → `preview_infeasible` |
| ④ | `gvf_manager.cpp:2519-2521` | `!out.valid → makeGovernorInvalidHold(pos, "guidance_invalid")` |

## 2. 证据

| 观测 | 数值 |
| --- | --- |
| `scan_path_left_known_domain`（有掉队的轮次） | 75 – 107 次 |
| 同上（全部到点的轮次） | 0 – 4 次 |
| 掉队机状态 | 指令位置 = 当前位置、速度 0，`reach end = 0`，两次采样（间隔 60 s）位置完全不变 |
| 30 机拒绝次数（修复前 → 修复后） | 275+150+99+… → 55+14+5+5+5 |

已做且**有效**的 4 个修复（保留）：
① 分配器边界同域比较；② 切向下限在已饱和 tick 上不再判死；
③ 分配器补上"切向速度下限"约束；④ 静态场景关闭无谓的缓冲擦除。

已做但**无效、已撤回**的 2 个尝试：
⑤ 认证域前缀截断（15 机 12/15、14/15、15/15）；⑥ 相位领先限幅 2.5 m（三次均 14/15，签名无变化）。
**结论：任何单点补丁都只能降低触发概率，不能断环。**

## 3. 范围（只允许碰这些）

只改 `gvf_manager.cpp` 中上面两处 `retain_nonzero_reference` 分支，以及为取 δ 所需的一个只读访问器。
**不碰**：SDF/地图层、tube 几何、ISF-GVF 律、相位口径、协同（SPH）参数。

**明确禁止（对应用户"不要臃肿保守"的要求）**：

- 不新增任何 accept / reject 判据，不新增门控；
- 不新增证书、地图 ID、安装链或校验层；
- **不使用** `phase_offset_recovery_owner` 那套多 tick 恢复事务（候选集、认证测度、deadline……）——
  那正是"臃肿"的来源，本方案显式排除；
- 灰度由**参数默认值**控制：默认 = 现有行为，集群链显式打开。

## 4. 方案（最小实现）

### 4.1 选用的做法：中心线回退 + δ 有界衰减

现在 δ ≈ 0 时本来就走这条路（`gvf_manager.cpp:2463+`）：

```cpp
section_for_command = nominal_bundle;
command_path = nominal_bundle->path;
out = pm.gvf_->calcLiftedGuidanceAtPhase(pos, phase_before, command_path);
```

本方案把**同一套回退**扩展到 δ ≠ 0，只是把参考横向偏移从 δ 有界地衰减到 0：

```
δ_eff = δ - sign(δ) * min(|δ|, u_delta_abs_max * dt)      // 每拍最多走 u_delta_abs_max
参考点 ref_pt += δ_eff * N                                  // 参考连续，不跳变
```

- 参考点**连续**（这正是当前那条注释担心的"jump"被消除的方式）；
- 横向运动由**已有的** `u_delta_abs_max` 限制，始终在 tube 内；
- δ 归零后自动落回 δ≈0 的正常分支 ✓ 恢复完成；
- 复用**已有**的中心线回退与接收侧的 recenter 弹簧（`delta_tracking_gain`），
  **不引入任何新组件**。

需要新增的唯一接口：从 adapter 只读地取当前保留 δ（`runtime_->retainedDelta()` 已有，只需暴露）。

### 4.2 明确否决的做法

| 做法 | 为什么否决 |
| --- | --- |
| 用 recovery owner 的多 tick 恢复事务 | 引入一整套新机器（候选/测度/期限）→ 就是"臃肿" |
| 相位倒退跟飞机 | 违反"相位不可倒退"契约 |
| 永久放宽某个审计判据 | 已验证：既有 8 个测试明文锁定，属契约 |
| 继续扩大感知盒 | 只是降低概率（8 m 实测收益递减），不治本 |

### 4.3 实施前必须补齐的一环（已核实）

只把 `out.valid = false` 换成"走中心线回退"是**不够**的，会留下一个状态错配：

- 回退引导评估在**中心线**上（δ = 0）；
- 但 `PhaseOffsetRuntime` 内部**仍然持有 δ ≠ 0**（`retainedDelta()`），
  而运行时对外**只有** `requestRecenter()` 这一个接口，且它要等**一次成功的 section tick** 才能生效；
  section 恰好就是失败的那个 → 请求永远无法兑现。

所以最小可行实现是**二选一**：

| | 做法 | 代价 |
| --- | --- | --- |
| **(a)** | 在运行时新增一个**有界 δ 衰减**入口：本拍把保留 δ 按 `u_delta_abs_max * dt` 向 0 收敛，并把回退引导按该 δ 抬起（参考连续、状态一致） | 运行时多一个 ~20 行的确定性函数，无新判据、无新证书 |
| **(b)** | 回退时**同时清零**运行时的保留 δ（参考跳变 ≤ ~1 m；实测横向速度指令 ≈ `k2·q(ρ)·ρ` ≈ 1.2 m/s，仍在 `cmd_vel_max` 与 tube 半宽 1.5 m 之内） | 更少代码，但有一次参考跳变（这正是原注释担心的点） |

推荐 **(a)**：它既消除自锁，又不引入参考跳变，且新增的是一个"确定性数值函数"，不是判据/门控/证书。
若希望代码量最小、且接受那次跳变，则选 **(b)**。

## 5. 验收标准

1. 30 机 ×1：到位 **≥ 29/30**；
2. **任意掉队机在 10 s 内恢复运动**（从"命令位置=当前位置、速度 0"恢复为有非零指令速度）——
   这是本方案的判别性指标（现状是永不恢复）；
3. 15 机 ×3：**全部 15/15**；
4. `phase_offset_navigation` + `bspline_traj` 契约测试 **保持全过**（当前 950 项）；
5. **新增 accept/reject 判据数 = 0**；`git diff` 只落在 §3 允许的文件内；
6. 默认参数下行为与现在**完全一致**（灰度安全）。

## 6. 回滚

新逻辑由参数 `phase_offset/recovery_from_hold_enable`（默认 `false`）控制；
置 `false` 即回到当前行为，无需回退代码。

---

## 7. 实测记录：方案 (b) 已试并撤回（2026-09-13）

按 (b) 实现（释放保留 δ + 走中心线回退）后实测：

| 场景 | 结果 |
| --- | --- |
| 15 机 ×3 | 13/15（差 1.6 / 4.0 m）、15/15、15/15 |
| **30 机 ×1** | **19/30 → 45 s 后 11/30**，且出现横向飞散：uav_29 到 x=−18.4、uav_23 到 (−10.4,−19.1)（已出地图窗口 x∈[−10,10]） |

结论：**释放 δ 会把 tube 提供的横向约束一起丢掉**，智能体被协同推着横向飞散。
代码里那句 "A nonzero Runtime reference has no safe nominal fallback" 的判断是**正确的**，
方案 (b) 不安全，已撤回；本轮保留的仍是 §2 的 4 个修复。

因此本问题的下一步只剩两条（都需要真正的设计改动，不再做单点补丁）：

- **(a)** 保留 δ，并把回退引导按 δ 抬起（需要在回退路径里具备 lifted-guidance 评估能力，
  而不是像 (b) 那样退回无偏移的中心线）；
- **(I)** 认证域实时化（当前位姿 + 感知半径），从源头消除"两个锚错开"。

---

## 8. 方案 (a) 的具体设计（待实施）

### 8.1 为什么 (a) 不是"回到臃肿"

(b) 失败的原因是**把 δ 丢了**：引导回到无偏移的中心线 → 横向约束随之消失 → 智能体横向飞散。
(a) 不丢 δ，做法是复用**工程里已有的模式**：

```cpp
// gvf.cpp 里现有（目前只用于向量场可视化）：
//   calcLiftedVisualizationVector(): eval_pos = pos - δ · N(w)
// (a) 把同一模式用于回退引导的求值：
//   calcLiftedGuidanceAtPhaseWithDelta(pos, w, path, delta)
//       -> 在 pos - delta·N(w) 处评估现有 ISF kernel，其余完全复用
```

| 对比项 | 当年那套臃肿 | 本方案 (a) |
| --- | --- | --- |
| 新增 accept/reject 判据 | 大量 | **0** |
| 新增证书 / 地图 ID / 安装校验 | 有 | **0** |
| 新增恢复事务机器（recovery owner：候选/测度/deadline） | 有 | **0**（明文排除） |
| 与既有实现的复用 | 重新造 | 复用现有 `pos − δ·N` + 现有 ISF kernel |
| 代码量 | 大 | 约 30 行（1 个求值重载 + 回退分支调用 + δ 有界衰减入口） |

要点：(a) **删掉一个拒绝（HOLD）**、**保留已有状态（δ）**，方向是"减少约束层"而不是增加。
唯一需要承认的新增是"一段控制层代码"，因此必须有对应测试。

### 8.2 改动清单（3 处，白名单）

| # | 文件 | 改动 |
| --- | --- | --- |
| 1 | `gvf.{h,cpp}` | 新增 `calcLiftedGuidanceAtPhaseWithDelta(pos, w, path_owner, delta)`：按 `pos − δ·N(w)` 求值，复用现有 ISF kernel；δ = 0 时必须与 `calcLiftedGuidanceAtPhase` **逐位一致**（可加断言测试） |
| 2 | `phase_offset_runtime.{h,cpp}` | 新增 `applyBoundedDeltaDecayNoFail(max_step)`：`δ ← δ − sign(δ)·min(|δ|, max_step)`（不判据、不证书；与 `commitV2NoFail` 同级别的确定性写入） |
| 3 | `gvf_manager.cpp` | 两处 `retain_nonzero_reference` 分支：开关打开时用 (1) 求值（δ 取衰减后的值）并同步 (2)，替代 `out.valid = false`；开关关闭时**逐位保持现行为** |

参数：`phase_offset/recovery_from_hold_enable`（默认 `false`），集群链 `true`。

### 8.3 验收

1. **δ = 0 等价性**：新增单元测试断言 `calcLiftedGuidanceAtPhaseWithDelta(..., 0.0)` 与
   `calcLiftedGuidanceAtPhase(...)` 的 v_cmd/w_dot/ref_pt 逐位相同（保护零偏移链路）；
2. `phase_offset_navigation` **950/950** + `bspline_race` 既有通过项保持通过；
3. 15 机 ×3：**全部 15/15**；
4. 30 机 ×1：**≥ 29/30**，且**无智能体飞出地图窗口**（|x| ≤ 10、|y| ≤ 25）——
   这是 (b) 失败的直接对照项；
5. 判别性指标：若有掉队机，**10 s 内恢复运动**（(b) 与现状都是"永不恢复"）；
6. **新增 accept/reject 判据数 = 0**；`git diff` 仅落在 §8.2 白名单内。

### 8.4 风险与回退

- 风险：回退期间的横向参考仍是"带 δ 的中心线"，若 δ 较大（>1 m）且回退期间横向权限耗尽，
  可能短暂贴住走廊边界 → 由现有 `u_delta_abs_max` 与 tube 裁剪约束，不会越界；
- 回退：参数置 `false` 即逐位回到当前行为，无需回退代码。
