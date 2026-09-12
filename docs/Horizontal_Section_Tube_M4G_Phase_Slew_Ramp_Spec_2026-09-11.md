# M4G：相位轴 slew 爬坡（"phase scalar slew interval is empty" 死锁）

日期 2026-09-11。状态：已执行。

## 1. 现象（用户 19:33 那次运行）

uav_2 冻在 `phase_w≈10.0`（起点前方 ~10 m），持续 `GOVERNOR_INVALID_HOLD
reason=guidance_invalid`，而**不是**相位窗口判据：

```
phase_offset_matched_adapter.cpp:529 allocator status=5 valid=0 feasible=0
  reason='phase scalar slew interval is empty'
```

## 2. 根因

`PhaseOffsetAllocator::allocate` 里相位轴的静态可行集与 slew 窗口**无交集**时，
`BuildSelectedScalar` 直接返回 false。语义上这是"请求的可达速率距离上一次已提交
速率超过一个 tick 的 slew 预算"。

关键问题在**失败 tick 不提交**：`previous_u_w` 永远停在旧值，于是静态集与 slew 窗口
下一 tick 依然无交集 —— 死锁，相位永久冻结。实测该判据在 uav_2 上连续出现 126 次。

横向轴早就按"取 slew 边界、不丢 tick"处理（M4D 的 transverse slew recovery），相位轴
漏了同款处理；M4C §5 明确把这一类列为"本批仍 fail-closed"。M4E 把 `u_w_abs_max`
从 0.12 提到 0.40 后，`f_w0 ∈ (upper_nu, upper_nu + u_w_abs_max]` 这一段从"分配器
裁剪（`phase_window_clipped`）"变成"静态集存在但够不到"，于是这条 fail-closed 被
真正触发。

## 3. 本批语义

与横向轴完全同构：静态可行集与 slew 窗口无交集时，**取 slew 窗口里最靠近静态集的
那个端点**（即朝可行集方向走满这一 tick 的预算），并置 `phase_window_clipped`，
在后续 tick 继续爬坡。不丢 tick、不冻结。

运行层同步放宽：M4E 的放行原先要求 `u_w` 精确落在幅值边界上，现在承认第二种合法
形式 —— "朝窗口方向、且位移不超过 `u_w_slew_rate*dt`"的爬坡点（仍要求基座速率确实
越窗、且标记来自分配器）。幅值/slew/切向速度/ZOH/provenance 判据不变。

## 4. 实现范围（白名单）

1. `src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_allocator.cpp`
   - 相位轴新增与横向轴同构的 slew 爬坡预裁剪。
2. `src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp`
   - `prepareSection` 的放行条件增加"朝窗口的 slew 爬坡"形态。
3. `src/swarm_planner/phase_offset/phase_offset_navigation/test/phase_offset_allocator_test.cpp`
   - 新增 `PhaseSlewRampTowardsAnUnreachableWindowStillSelects`。
4. `src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp`
   - 原"边界外修正必须被拒"的用例改为"**远离**窗口的修正被拒"，并新增"朝窗口爬坡
     被接受"的用例。

## 5. 验收

单元测试：

| 测试 | 结果 |
|---|---|
| `phase_offset_allocator_test`（含新增 1 条） | 47/47 |
| `phase_offset_runtime_test`（含新增爬坡用例） | 15/15 |
| `phase_offset_tube_viability_test` | 42/42 |
| `phase_offset_section_tube_test` | 32/32 |
| `phase_offset_section_input_test` | 35/35 |
| `phase_offset_matched_adapter_test` | 10/10 |

柱阵场景（`sim_b_formation_3.yaml`，目标行 -11.2，全程）：

| 指标 | 结果 |
|---|---|
| `phase scalar slew interval is empty` | **0 次** |
| `Section MatchedPort phase rate is outside the frozen limits` | **0 次** |
| `GOVERNOR_INVALID_HOLD` | 仅起飞前 3 次（`guidance_invalid`），飞行中 **0 次** |
| `fallback_reason=none` | 365 次 |
| `[REACHED]` | **3/3**（0.190 / 0.200 / 0.197 m） |
| 终点 | (-1.000, -11.200) / (0.000, -11.200) / (1.000, -11.200) |

至此 M4E（相位窗口饱和）+ M4F（曲率封顶）+ M4G（slew 爬坡）三层都不再让
planner-valid 的导航任务因相位/横向可行域问题进入持久 HOLD。
