# Codex A5R-1 Validator Fixed-Array Execution Plan

```
DOCUMENT_ROLE=IMPLEMENTATION_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=A5R-1
IMPLEMENTATION_AUTHORIZED=true
AUTO_ADVANCE=false
AUTHORIZATION_SCOPE=ONE_VALIDATOR_ALLOCATION_HOTSPOT_ONLY
```

> 日期：2026-08-12  
> 仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
> 前置：A5R-0A hotspot audit 和 A5R-0B frozen cohort 已完成；A5R-0B 的
> `/tmp/a5r_0b_20260812T004000Z_terra_max_final_v3/analysis/A5R_0B_METHOD_AND_RESULTS.md`
> 结论为 `GO`。本执行单仅授权一个输出等价的 Validator 内存分配优化，不授权 A5R-2/3、A6 或 A7。

## 1. 唯一目标与证据

`TubeSurfaceValidator::validate` 是 A5R-0A 中最大的 inclusive hotspot（42 次 build 平均
65.200 ms，39.42%）。在 `ValidateCell` 的每一次递归调用中，`CollectCellPoints` 都创建、
清空并 `push_back` 恰好九个 `SurfacePoint`。该九点 3×3 tessellation 的大小是编译期常量，
因此本阶段仅将这个局部动态容器改为固定大小的 `std::array<SurfacePoint, 9>`。

此变更只消除每个 cell 的局部堆分配；不以降低 query 数、放宽 certificate 或改变递归为性能手段。

## 2. 穷尽文件白名单

允许修改的仓库源文件只有：

1. `docs/Codex_A5R_1_Validator_Fixed_Array_Execution_Plan_2026-08-12.md`（本执行单）；
2. `src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp`；
3. `src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_surface_validator_test.cpp`。

`build/` 与 `devel/` 只能由本阶段的正常 `catkin_make --pkg phase_offset_navigation -j1`
生成，绝不手工编辑；一次性 benchmark、日志和哈希证据仅写到 `/tmp/a5r_1_*`。所有其他
仓库文件均为保护文件，特别是 `tube_surface_validator.h`、CMake、builder、cross-section、
Filter、epoch manager、runtime、adapter、launch、参数、planner、governor、simulator 与 SO3。

## 3. 允许的算法及不变量

允许的唯一代码算法为：

1. 保持原有 `w_values = {w0, midpoint, w1}` 外层遍历和
   `v_values = {v0, midpoint, v1}` 内层遍历；
2. 将每一个按该顺序得到的 `SurfacePoint` 写入固定索引 `3 * w_index + v_index` 的九元素数组；
3. 按同一索引顺序计算 center/radius，并按同一索引顺序调用 `ClearanceQuery`；
4. 保持在某个 clearance 点不安全后仍扫描该 cell 的其余点，保持
   `first_failure_reason`、`first_failure_w`、`min_clearance_margin` 与
   `query_sample_count` 的原有语义；
5. 保持 query-limit 检查、cover 公式、最大递归深度、子 cell 次序和 `&&` 的短路次序逐字等价。

明确禁止：跨 parent/sibling/cell 缓存 `SurfacePoint`、PathStateQuery 或 Geometry 结果；缓存、
合并或跳过任何 `ClearanceQuery`；假设不同 `requested_radius` 的 query 可以复用；重排遍历；
提前停止 cell 扫描；改变 query limit、参数、margin、cover、递归、reason/state/certificate 或
diagnostics。A5R-0B 所见 `query_sample_count` 最高约 27,792，不能把 limit 当作优化杠杆。

## 4. 输出等价判据

本阶段不允许任何安全集合扩大。对固定输入，优化前后必须保持：

1. 3×3 PathStateQuery、Geometry 和 ClearanceQuery 调用的数量及行优先顺序相同；
2. 每次 ClearanceQuery 的点和 `requested_radius` 相同；
3. `query_sample_count`、`first_failure_reason/w`、`min_clearance_margin`、
   `complete/current_anchor_valid/truncated/limit_exceeded`、certified 范围和 profile 截断完全相同；
4. raw/filtered bounds、margins、status、obstacle certification 与 fail-closed 判据不变。

`tube_surface_validator_test` 新增一个递归成功 fixture，记录 PathStateQuery 与
ClearanceQuery 序列，断言固定的 72 次调用、前缀顺序、每次几何点的相位对应关系以及原有
certificate 输出。它是固定输入的同进程等价检查；不把不同 ROS run 的命令作 bitwise 比较。
此外，变更前后的同一 `/tmp` standalone fixture 将记录 observable-result bit fingerprint；
该 fixture 的输入、运行次数和二进制 SHA-256 都写入证据目录。

## 5. 验证和性能测量

1. 记录 stage 前 status、tracked diff、`git diff --check`、白名单 source 与现有 library/test
   SHA-256；先运行现有 Validator 测试二进制作为基线。
2. 在 `/tmp/a5r_1_*` 编译一个不修改仓库的固定输入 benchmark；它只调用 Validator，输出
   完整结果 fingerprint、每轮 query count 和 `CLOCK_PROCESS_CPUTIME_ID` 耗时。先以变更前
   `devel/lib/libphase_offset_navigation.so` 运行，再以重建后的库运行相同二进制和相同输入。
3. 正常增量构建 `catkin_make --pkg phase_offset_navigation -j1`，运行
   `phase_offset_tube_surface_validator_test`，以及 phase-offset navigation 的相关既有回归
   （cross-section、builder、Filter、epoch manager）。
4. 只在不存在用户 ROS master 冲突且能拥有 loopback master 时，沿 A5R-0A 的未插桩方法以
   未改参数的 `phase_offset_esdf_tube_single.launch` 重复三次；报告 `buildTubeEpoch` 的
   p50/p95/max、deadline miss、build rate、source-stale 和系统负载。若本阶段无法安全取得
   三个私有 episode，则只报告 fixture CPU 指标，不能宣称端到端性能 PASS。
5. 比较冻结 A5R-0B 证据的 schema/provenance，而不是伪造不存在的 per-clearance replay；
   任何 result/state/reason/bound/query-count 不等价立即停止并保留 worktree。

## 6. 停止条件和交付

若需要白名单外文件、跨 cell 缓存、递归/参数改动，或任一等价性检查失败，立即停止并报告。
完成时只交付该单热点的 diff、build/test/fixture 证据、可重复的 CPU 数据、前后 status/diff
审计和剩余风险；不得自动进入其他 hotspot、A5R-2/3、A6 或 A7，也不得 commit、branch、tag
或 push。
