# 水平横截面 Tube：M0 预检与监督报告

日期：2026-09-09

依据：[M0 执行规格](Horizontal_Section_Tube_M0_Execution_Spec_2026-09-09.md)；[总体计划](Horizontal_Section_Tube_Refactor_Plan_2026-09-09.md)。

## 1. 本批次实际完成范围

本批次执行基线预检、地图源对比、路径/normal/接续/速率分配的隔离重编译测试，并由指定的 Luna 子代理编写一个独立诊断程序。生产 `src/`、原 `build/`、原 `devel/` 和 launch 均未修改，没有启动 ROS master/仿真、创建分支或提交。

这不是“完整 tube 重构完成”的报告。M0 的基线及定向能力检查已执行；完整场景回放、最终局部地图数据接口/锁闭包、代码替换规格和性能预算仍需在功能批次放行前冻结。

## 2. 基线与归档

- 当前 branch：`pro_review_current_20260903`。
- 当前 HEAD：`20d44c94d4165e35621cc30269e3fa0b2a5f10ff`。
- 新 HEAD 是之前未提交源修改的 checkpoint；与主计划编写时的 src 指纹相同。
- tracked diff 为空，SHA256 为 `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`。
- rg 可见 src 清单指纹：`3658d5a20a004b7bf2cb6f7edf5739b8505332c44c6432fcfd2c1830ca9a4efc`。
- 本轮产物目录：`.horizontal_section_refactor/m0_20260909_01/`。
- 已归档 `src/`、AGENTS.md 和计划/自审文本：`source_entry.tar.gz`，SHA256 为 `1a624ea058e5fcc03bc1c812a10d4bb18db55bd96c7a857c735f0fa3cdd92683`。
- `entry_manifest.json` 保存工具链、关键源文件指纹和原 devel smoke 输出；`map_baseline_comparison.json` 保存地图基准哈希。

本次硬件为 i5-12400，12 个逻辑 CPU，约 31 GiB RAM；原 CMakeCache 是 RelWithDebInfo。本轮独立编译使用 C++14、O2、g、必要时 `-ffp-contract=off`，不与原 devel 混合链接。当前运行负载不作为性能验收标准。

## 3. 重要发现一：现有测试产物不能直接当源码基线

主会话先对原 devel 中四组测试做 smoke：均退出139；路径测试程序仅列14项，而当前源文件有20项。路径测试二进制时间为2026-09-05，相关路径动态库时间为2026-09-09。

随后完全使用当前源码，在本轮产物目录重新编译所需对象和测试，四组均通过。因此这四次 smoke 崩溃不能用来证明当前源数学错误，首先应归为旧测试产物/新库混用风险。未具体对每次崩溃做 ABI backtrace，故不声称已逐个证明唯一根因。

| 测试 | 原 devel smoke | 当前源码独立编译 | XML |
|---|---|---|---|
| continuous_phase_path | 14项，断言失败后139 | 20/20通过 | `path_tests.xml` |
| continuous_phase_normal_frame | 首项139 | 6/6通过 | `normal_tests.xml` |
| recovery_continuation_provider | 首项139 | 7/7通过 | `recovery_continuation_provider_test.xml` |
| phase_offset_allocator | 首项139 | 38/38通过 | `phase_offset_allocator_test.xml` |

四组总71项。它们是当前接口的定向基线，不是 formation_planning 全套测试或新 tube 验收。

监督决定：不让 Luna 根据旧二进制崩溃修复原算法；原 build/devel 暂不覆盖，后续集成必须同源完整构建。

## 4. 重要发现二：baseline 地图可以作为明确恢复源

主会话独立哈希复核，用户 B 工作区的下列四个文件均与 `9a0e975` 完全相同：

1. `src/swarm_planner/plan_env/src/sdf_map.cpp`。
2. `src/swarm_planner/plan_env/include/plan_env/sdf_map.h`。
3. `src/swarm_planner/plan_env/CMakeLists.txt`。
4. `src/uav_simulator/dynamic_map_generator/src/local_sensing.cpp`。

详细哈希见 `map_baseline_comparison.json`。其中地图 cpp baseline SHA256 为 `185d1e67c1203ceac7eefe655d1ee5618f2f33b92ebd704ddd1454355dda284f`，头文件为 `04dd598de7081aff0977a4e1ba9d2c809515b83659be3c77ff01f72fc090a463`。

当前相对9a0e975：sdf_map.cpp 增1251/删4行，头文件增127/删1行，CMake增22行，local_sensing增37/删4行。新增部分主要是 cloud snapshot、capture/support identity、同步与生产者契约，而非必须保留的新地图算法。

但 B 的 test_gvf.launch 与9a/当前不一致，其中 circle enable/auto_start 为true，局部地图范围也不同。绝不能整个复制 B launch 来“恢复地图”。当前用户保护的 false/false 和现有启动配置保留。

### 4.1 可直接恢复与需要成组处理的范围

- 地图 cpp/h 是明确的配对恢复来源。
- CMake 参考 B 的原 map target，再按新局部接口增量接入，不机械整包替换。
- local_sensing 的恢复取决于局部已知域与消息元信息接口；不得静默丢掉数据语义。
- baseline 原 getters 和 backing 是 mutable 且没有一致快照锁；恢复原数学行为不等于新并发 builder 可以无锁逐点读。
- 仍保留原 inflated occupancy、ESDF、manual/static 层和实际有效 ceiling/boundary。不能从原始点云另起一套遗漏这些层的地图。

### 4.2 地图源独立编译结果

主会话分别独立编译 B 和 W 的地图核心及原 manual map 测试，各运行以下两项：

- ObstacleCallbackMarksManualAndInflatedOccupancy。
- BoundaryUsesTwoPointsBeforeConstrainingMap。

结果 B 2/2、W 2/2通过，XML分别为 `baseline_map_tests.xml`、`current_map_tests.xml`。这支持配对恢复的可编译性和两项原行为一致性，不代表所有地图层已回归通过。

另外3项原测试使用固定 `/tmp` 文件名并主动删除/覆盖文件，本轮未执行；后续在批准的独立测试fixture中调整临时文件归属后再完整运行。第一次手工构建因本机缺少 pcl_common.pc 失败；改用已安装PCL头路径和库名后编译成功，未安装或改动环境。

## 5. 安全参数账本：确认值，不擅自调参

以下为当前 test_gvf.launch 默认展开与源码读取，不声称是某个正在运行 ROS 节点的实际参数；本轮未连接用户 ROS master。

| 项目 | 当前值/语义 | 证据与处理 |
|---|---|---|
| map resolution | 0.1 m | test_gvf.launch:327/436 |
| map size | 20×30×2.5 m | launch:95–97，不是独立证明已知域的标记 |
| map local update half-range | 6,6,2.5 m | launch:99–101，不能抄成B的4/4 |
| sdf/gvf inflation | 配置0.099 m | 对cloud XY用ceil(0.099/0.1)=1体素，Z现有代码一层；不是球半径的精确欧氏腐蚀 |
| production tube epsilon | planning/safe_distance=0.4 | adapter:660–704；查询所针对的是已有膨胀表示 |
| minimum_reference_speed | 代码默认1e-8 | tube_cross_section.h:60、tube_certificate_v2.h:75及core geometry类型；当前无对应launch覆盖 |
| nominal half width | 1.0 m | adapter实际nominal owner；旧max_offset=.20不直接作为V2几何上限 |
| raw_uav/map/loc margins | .25/.10/.05 | 不再相加进入生产epsilon，不能据此推断实际机体安全预算已验证 |
| raw_tracking_error_bound | .15 m | 不加入epsilon，但仍用于运行时/跟踪预算，见adapter:214与gvf_manager:1834；不能误删为纯诊断 |
| normal preview upper_nu | 2.0 | launch:48；论文示例为3.0 |
| normal preview horizon / spacing | 1.0 / .25 | launch现有参数；不同于几何lookahead=2/sample_step=.10 |
| cloud_obstacle_set_complete | 默认false | 部分专用launch另设true，不能将它视为所有输入自动完整 |

论文 epsilon=.55、m_r=.20 与当前值不同。迁移几何公平对比先固定同一体素表示、同一0.4 residual clearance与实际源配置；是否切换论文参数属于显式实验配置，不在恢复地图时暗改。

目前尚无本轮独立物理误差测量证明“已膨胀体素+0.4”与论文“原障碍+.55”相等。不能宣布物理安全预算账本已经完全闭合。

## 6. 已知域：有限静态仿真假设与真实观测分开

当前 map_publisher 将PCD作为静态障碍点真值发布；local_sensing 对局部轴对齐范围做完整裁剪。baseline 的点云stamp为发布时间，W新增版本在若干完整性条件满足时复用对应odom stamp。

首版静态仿真可明确采用“指定有限世界内，PCD就是全部环境障碍”的实验契约，然后验证局部裁剪完整及坐标/范围一致；不需要证明PCD之外真实世界为空。但必须显式选择该契约，不能将点云缺点、receiver的map_size或任意传感器自动解释为已知自由空间。

新局部view需说明本次数据对应的区域中心/有效域。只恢复旧stamp，同时声称旧receiver最新camera_pos就是准确采样域，会重新引入域边缘漏障碍风险。这个薄接口需在功能批次规格中确定，不再使用长support编号链解决。

本轮未运行真实感知或场景回放，unknown/遮挡真机能力不在完成声明内。

## 7. 路径能力与接续：主会话复核

### 7.1 现有可复用能力

- `ContinuousPhasePath::appendSlice`（continuous_phase_path.cpp:2774附近）复用同一不可变 evaluator，而不是重拟合前缀。
- `buildPhaseV2C2Frontend` 在 gvf_manager.cpp:3344–3380 已有 quintic 接续及旧前缀复制。
- `installPlannerOnlyFrontendV2` 在约3851行已检查 copied_prefix_expired。后续是复用/简化检查，不需发明完整新共同前缀系统。
- `cellBounds` 返回 q_min、速度、加速度、jerk、N_w 幅值界；MakeMappedBsplineCertificate可汇总一个查询范围内多个内部ArcLengthMap单元。
- normal frame 点值提供 N 和 N_w，没有直接提供一般非零offset所需的 N_ww 点值。
- recovery_continuation_provider.cpp:20–31 明确 exact r_ww 仅在delta=0时提供；其c2_continuous字段在require_full_jet=false时不构成一般active-reference C2证明。

### 7.2 可用于新算法的数学方向

设横向未归一化向量v=e_z×p_w，q=inf||v||>0，A= sup||v'||，J>=sup||v''||。则可使用：

`||N_w|| <= A/q`，`||N_ww|| <= J/q+3(A/q)^2`。

后者来自归一化向量二阶导数的幅值界，不需要先新增N_ww点值。它可用于平面方向二阶偏离的保守估计；实际数值舍入及legacy源界可靠性仍需在新模块中按冻结容差验证，不能把本轮有限采样当成连续证明。

因此不必为了横截面新建完整V2证书体系，也不必改变实际p(w)反解。应缓存每个待检查路径段的界，并区别“内部反解表格节点”和“必须改变执行几何/约束的分段边界”。不能让1024个内部表格点自动决定tube截面数量。

### 7.3 Luna probe 初次结果与主会话审核

Luna按单文件白名单新增 `path_geometry_probe.cpp`，没有修改生产代码。固定输入为直线、7点三次B-spline以及[4.2,4.8]共同前缀。

初次运行：直线legacy/V2均20/20；B-spline legacy 30/30、V2 0/30；slice legacy 6/6、V2 0/6。现有V2接口在这些固定输入/网格上返回失败，不等于所有B-spline都失败，也没有授权子代理修补旧V2。

非零delta=.3、w_dot=1、delta_dot=.1的21点共同前缀比较通过，r/r_dot无报告失配。幅值界采样初版在64×machine-epsilon尺度容差下无报告失败，不构成严格零原始超界或连续证明。

主会话逐行审核后要求返修：补上规格要求的legacy/V2查询独立计时；CSV保持固定列数，RESULT移出表格；公开比较容差并报告原始超界量，避免宽松判定掩盖数值差异。最终复验结果在本报告第10节填写。

## 8. 地图替换的调用闭包与批次风险

成对恢复地图后，以下真实消费者必须同步处理：

- `R/src/gvf_manager.cpp` 的 acceptedStateVisibilityV2 / captureAuthoritativeSDFMapV2 调用（1610、1668、1768、3622附近）。
- `R/src/integration/phase_offset_cloud_occupancy_query.cpp/.h`：capture/free-ball旧bridge。
- `R/src/integration/phase_offset_environment_evidence_query.cpp/.h`：map旧uninfated evidence接口。
- `R/src/integration/phase_offset_tube_epoch_diagnostics.cpp/.h`：旧snapshot DTO消费者。
- `R/CMakeLists.txt` 的旧query、worker、adapter/diagnostics链接；`E/CMakeLists.txt` 的旧snapshot/evidence源。
- adapter、runtime、allocator、viability和recovery中的旧profile/provenance绑定需在新bundle接入时一起适配。

这里R为bspline_traj、E为plan_env。以上不是全部函数替换清单，不将rg命中表当成已证明闭合的精确代码白名单。

重要批次风险：M1不能仅覆盖map cpp/h后就宣称“原formation_planning已恢复”，因为现有manager/adapter仍引用被删API。原计划允许隔离中间态，但完整关闭模式导航验收必须在消费者断开或替换后进行。

建议下一功能批次先明确“地图薄接口/离线几何准备”和“地图+消费者成组切换”的依赖关系；不生成虚假capture/support ID兼容旧接口，也不在主工作区部署半替换状态。最终顺序与文件级规格由主会话冻结，Luna不得自行把阶段扩成整仓重写。

## 9. M0 当前放行状态

| 项目 | 状态 |
|---|---|
| 当前源基线/归档 | 已确认 |
| B地图与9a0e975一致性 | 已确认，主会话独立哈希复核 |
| 当前路径/normal/recovery/allocator定向源码基线 | 71/71通过 |
| B/W地图两个无临时文件用例 | 分别2/2通过，共4项 |
| 原devel可信性 | 不可直接沿用，必须重构建相应测试 |
| 共同前缀与局部几何界的固定输入probe | 监督返修后主会话独立重编译复验通过；不等于连续证明 |
| 物理安全余量等价/实测跟踪预算 | 尚未验证；不暗改配置 |
| 局部已知域数据接口与锁闭包 | 尚未冻结 |
| 完整formation_planning/场景回放与性能预算 | 尚未执行/冻结 |
| M1/M2生产代码执行规格 | 未放行，禁止直接换图后让旧消费者失配 |

## 10. 最终复验与收尾

Luna按两项格式/计时要求及原始误差公开要求完成返修。主会话随后使用 `-Wall -Wextra -Wpedantic` 独立重编译为 `path_geometry_probe_reviewed` 并运行，退出0；只有原core头文件的 `__int128` GNU扩展警告，没有因probe新增代码产生的编译错误。

主会话复验结果：

| 路径 | cell数 | legacy界可用 | V2界可用 | 点查询失败 | 原始界超出次数 | 原始最大超出量 |
|---|---:|---:|---:|---:|---:|---:|
| 水平直线 | 20 | 20 | 20 | 0 | 0 | 0 |
| 固定B-spline | 30 | 30 | 0 | 0 | 0 | 0 |
| copied prefix | 6 | 6 | 0 | 0 | 0 | 0 |

共同前缀21点的r/r_dot及原p、p_w、p_ww、N、N_w比较通过。CSV经独立检查为16列、56行数据，无混入RESULT行。完整stdout和退出码保存为 `supervisor_probe_run.json`。

本次单次legacy界查询总耗时：直线0.206 ms、B-spline0.410 ms、slice0.082 ms；只包path API，不包含21点核对和I/O。这支持“可以低成本复用局部界”的定向判断，不是新tube构建性能，V2失败的快速返回也不能作为算法速度对照。

probe源码SHA256：`f937f95f5c28de20305181e5e2498b27ff0ef4ff00f6ab27cdb3838f3570eb2d`。

本轮累计执行75个现有源测试实例通过（71个路径/执行相关，B/W各2个地图用例），另加上述独立probe；没有删除或放宽原测试断言。

Luna当前停止等待新规格。主会话未放行生产地图替换；下一批必须先处理第8节的成组依赖切换及第9节尚未冻结条件。这个阶段边界用于避免用户再次得到“地图已经覆盖、旧消费者还在调用”的半成品，不表示整体重构完成。
