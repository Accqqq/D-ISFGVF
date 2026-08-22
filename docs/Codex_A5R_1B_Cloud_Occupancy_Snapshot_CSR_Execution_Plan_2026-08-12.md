# Codex A5R-1B Cloud-Occupancy Snapshot CSR Execution Plan

```
DOCUMENT_ROLE=IMPLEMENTATION_EXECUTION_SPEC
DOCUMENT_STATUS=ACTIVE
STAGE=A5R-1B
IMPLEMENTATION_AUTHORIZED=true
AUTO_ADVANCE=false
AUTHORIZATION_SCOPE=ONE_EXACT_SNAPSHOT_CLEARANCE_CSR_ACCELERATION_ONLY
```

> 日期：2026-08-12  
> 仓库：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`  
> 前置：A5R-1B 的只读 `LD_PRELOAD` probe 已在私有 loopback ROS master 上完成。该单仅授权
> `CloudOccupancySnapshot` clearance dense-empty-voxel scan 的一个输出等价优化；不授权第二个
> hotspot、A5R-2/3、A6 或 A7。

## 1. 唯一目标与实测依据

只读证据根目录为
`/tmp/a5r_1b_20260812T013000Z_snapshot_sparse/evidence/episode_1`。它使用未修改的
`phase_offset_esdf_tube_single.launch`、`manual_observe_only=true`、新的 loopback-only ROS
master 和独立 `ROS_HOME`；用户拥有的默认 master `formation_planning` 未被连接或终止。

probe 在原函数调用之外计数，原函数的 `CLOCK_THREAD_CPUTIME_ID` 时间单独记录：

| 项目 | 结果 |
| --- | --- |
| valid snapshots | 514 / 514；每个 1,500,000 voxel |
| 全图 occupied density | p50 1.823%，mean 2.065%，max 3.527% |
| 原 snapshot build CPU | p50 15.402 ms，p95 24.594 ms |
| clearance calls / valid bboxes | 371,882 / 369,788（99.44%） |
| dense bbox cells | p50 1,728，mean 2,219，p95 4,096，p99 7,220，max 13,800 |
| bbox occupied cells | p50 108，mean 191.5，p95 756 |
| bbox density | p50 4.86%，mean 10.30%；主半径 0.45 的 empty bbox 为 29.3% |
| 原 clearance CPU | p50 4.81 µs，p95 16.06 µs |
| 全部有效 bbox | 820,604,391 dense checks，对应 70,806,127 occupied candidates（11.6×） |

这些数支持跳过空 voxel，但不支持扫描全局 occupied list：全图约 27k--53k occupied cell，
反而大于典型 1.7k--2.2k bbox。唯一获授权的算法是按 `(x,y)` column 的 CSR 精确索引，再在
column 内按 address/z 范围迭代。

## 2. 穷尽文件白名单

允许修改的仓库文件只有：

1. `docs/Codex_A5R_1B_Cloud_Occupancy_Snapshot_CSR_Execution_Plan_2026-08-12.md`（本执行单）；
2. `src/swarm_planner/plan_env/include/plan_env/cloud_occupancy_snapshot.h`；
3. `src/swarm_planner/plan_env/src/cloud_occupancy_snapshot.cpp`；
4. `src/swarm_planner/plan_env/test/cloud_occupancy_snapshot_test.cpp`。

`build/` 和 `devel/` 只可由普通 `catkin_make --pkg plan_env -j1` 生成，绝不手工编辑。所有
probe、benchmark、日志、binary copies、hash 和分析只写到
`/tmp/a5r_1b_20260812T013000Z_snapshot_sparse`。不改 CMake、launch、参数、geometry、margin、
timer、gate、mode、reason、schema、planner、governor、simulator、SO3、Builder、Filter、runtime
或 adapter。

## 3. 获授权的精确数据结构和算法

在 `CloudOccupancySnapshot` 增加一个 optional immutable CSR acceleration：header 只前向声明
其 type，并私有持有 `shared_ptr<const ...>`；完整 index type、offset/address storage 和唯一构造点
都在 `.cpp`。外部代码不能创建、编辑或向另一个 snapshot 注入 index。

1. 原 `occupied` dense byte vector 永远保留，并且仍是唯一真值；`!= 0U` 保持为 occupied。
2. builder 在原有 cloud rounding、XY inflation、一个 z voxel 和 dense writes 全部结束后，按
   原 address order `(x, y, z)` 单次遍历 dense `occupied`，构造：
   - 长度为 `voxel_count.x() * voxel_count.y() + 1` 的 prefix offsets；
   - 每个 nonzero dense voxel 的 `std::size_t` address，column 内严格 z/address 递增；
   - builder observation sequence、voxel dimensions 和 dense size 的 immutable binding metadata。
     正常 value-copy 同时复制相同 dense truth 和 private shared index，故 SDFMap 的 const snapshot
     copy 保持可用；手工 snapshot 的 private index 一律为 null。
3. clearance query 保留原有 metadata、map、observed closed-ball、grid closed-ball、index-range、
   `kMaxClearanceVoxelChecks` 以及所有 fail-closed 判定的原始次序。仅在这些全部通过后：
   - 如果 private CSR absent、binding metadata 或尺寸/头尾 prefix 的 O(1) 结构检查不成立，逐字
     使用原 dense `x/y/z` loop；手工构造的旧 snapshot 因此继续合法，绝不变为 `UNAVAILABLE`；
   - 如果 CSR 可用，按相同 `x`、`y`、递增 address/z 次序访问各 column，在 `[first_z,last_z]`
     中的 occupied address 才调用原 closed-voxel-volume distance；
   - 所访问 column 必须有 bounds、nondecreasing offsets、address 属于该 column/z range、address
     小于 dense size、且 dense byte nonzero 的 guard。任何不可信 index 事实都不得产生
     `KNOWN_FREE` 或 certificate：改走完整 dense fallback（若原 dense contract 本身无法满足，仍
     返回现有 `UNAVAILABLE`）。
4. `cloudOccupancySnapshotConsistent()` 继续只验证原 metadata 和 dense size。它不得每 query
   O(columns) 或 O(occupied) 全验 CSR；private index 缺席不是 snapshot invalid。builder 后修改
   dense vector 本来就违反 `CloudOccupancySnapshot` 的 immutable-value contract，不能把这种修改
   当作可由公开 index 伪造/掩盖的语义。
5. indexed 和 dense path 必须保持同一 occupied candidate 的数值访问顺序；不因 `nearest == 0`
   提前退出，仍保留任何随后 non-finite distance 的 fail-closed 行为。

明确禁止：KD tree、ESDF、近似 distance、cross-snapshot cache、降低/缓存 ClearanceQuery、改变
radius/bbox/margin、改变 domain 或 closed-volume distance、删除 dense truth、将 public 手工 snapshot
因无索引标成 UNAVAILABLE，或将全局 occupied address list 当作 query scan。

## 4. 输出等价和 differential 判据

新增测试必须提供一个固定 corpus 和一个固定 seed 的 randomized dense-vs-CSR differential：

1. 每一对 snapshot 共享相同 dense truth；reference copy 清除/禁用 CSR，强制旧 dense loop；
   indexed copy 使用 CSR。对每个 query 逐字段比较 `status`、`clearance_certified`，以及
   `nearest_occupied_voxel_volume_distance` 的 IEEE-754 bit pattern。
2. corpus 覆盖 empty CSR、空 bbox、命中 voxel、closed voxel face/edge/corner、多个 column、
   capped certified free、`UNKNOWN`、`OUT_OF_MAP`、`UNAVAILABLE`、boundary radius 和
   metadata failure。
3. randomized corpus 覆盖不同 small valid dimensions、random occupied layout、多个 point/radius，
   并包含 byte 值 `2`、`255` 等 nonzero occupied truth；CSR 对这些值仍必须与 dense 等价。
4. 手工/legacy snapshot（private CSR 为 null）必须保持
   `cloudOccupancySnapshotConsistent()==true` 且走 dense fallback；opaque binding-mismatch copy 也必须
   无法改变 dense result。builder snapshot 的 empty cloud 仍生成可用 empty CSR，并与 dense
   reference bitwise 等价。测试不得 reset、replace 或构造 private index；dense reference 以新建
   snapshot 显式复制公开 metadata 和 dense truth 获得。

不以跨 ROS run 的 floating telemetry 做 bitwise equivalence。这里的 bitwise 判据仅针对同一
进程、同一 snapshot/query 的 clearance result 字段。

## 5. 验证、成本和审计

1. 记录 stage 前 status、tracked names、`git diff --check`，并以冻结 hash 审计三个 untracked
   source preimage 与 `devel/lib/libplan_env.so`。A5R-1 用户变更完全保留。
2. 在 `/tmp` 保留当前 external probe、编译命令、owned-run command、CSV 和本阶段的 read-only
   analysis；不得把 profiler 造成的 wall time 误报为性能结果。
3. 建立前后使用相同固定 input/corpus 的 external CPU benchmark，并记录 build CPU、clearance
   CPU、index build额外 CPU 和 bytes（offset/address vectors）。再以相同 private ROS 方法记录
   post-change 原函数 CPU；所有 runtime episode 只用私有 loopback master。
4. 执行 `catkin_make --pkg plan_env -j1`、
   `devel/lib/plan_env/cloud_occupancy_snapshot_test`，和与此 package 可安全运行的既有 regression。
   再运行三个无 preload、未改参数的 owned ROS episode；若受用户默认 workload 影响，结果只能
   表述为 actual-load evidence，不可虚称隔离性能 PASS。
5. 最后运行 whitelist/preimage audit、`git diff --check`、status、hash manifest 和书面 self-audit。
   任何 output differential、未授权文件、build-regression 或 safety-contract 差异都立即停止。

## 6. 交付和停止

完成时只交付该 snapshot CSR single-hotspot diff、dense-vs-index evidence、build/query CPU 与 memory
开销、private ROS evidence、审计和剩余风险。不得 commit、branch、tag、push 或自动进入任何其他
优化/研究阶段。
