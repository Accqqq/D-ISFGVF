# A5.2 修订版：Baseline 恢复、Tube 显示证书与合格 ESDF 因果诊断执行单

工作目录：

    /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws

阶段控制：

    AUTO_ADVANCE=false

本执行单与执行模型无关，Terra、DeepSeek 或其他执行者均必须逐条遵守。

本修订版完整取代此前同名 A5.2 执行单、旧 A5.2 实现和旧 A5.2 验收结论。
当前可信代码起点是：旧 A5.2 代码已精确撤回，工作区恢复到 A5.1。

本轮只执行 A5.2。完成确定性修复、构建、测试、合格隔离 ROS 诊断、自审核和
汇报后立即停止。禁止进入 A5.3、A6 或任何多机阶段。

## 1. 当前可信结论

开始前必须接受以下只读审计结论，不得重新引用旧 A5.2 汇报覆盖它们。

### 1.1 已确认的产品缺陷

1. failure latch 后仍可能发布 tube ADD：

   - Runtime 可保留已有 complete profile；
   - Marker 当前只检查 tube_profile.complete；
   - 历史 A bag 中 failure latch 发生于 4.798988 s；
   - latch 后 Marker IDs 0、1、2 仍为 ADD；
   - 0.583598 s 后才首次全部 DELETE。

   这不符合安全可视化语义。

2. 当前裸 test_gvf.launch 默认：

       phase_offset_mode=manual
       phase_offset_manual_tube_source=esdf

   裸 launch 因此会创建 matched adapter、重建 ESDF tube、发布额外诊断与
   Marker，并可能在 gate 后替换 guidance。它不能作为原单机 baseline。

3. 历史 CSV 导出链存在枚举别名错误：

   - 表头 84 列；
   - 数据行 83 列；
   - kManualLegacyDiagnosticCount 与 kTubeSource 数值相同；
   - 导出器把 sentinel 和真实字段都写入表头；
   - 从 kTubeSource 开始标签整体错位。

   旧 CSV 和由该 CSV 派生的统计不得用于验收。

### 1.2 已确认不合格的旧 A/B

按最宽松的有效条件 gate && !failure && profile_complete && ready：

- A 连续窗口为 3.001844–4.797263 s，共 1.795419 s；
- B 连续窗口为 7.241767–7.900962 s，共 0.659195 s；
- 两组窗口内 source revision 仍变化；
- 两组均远不足 15 s。

因此旧 A/B 不能证明 buffer refresh 是原因，也不能证明需要 snapshot。

### 1.3 旧 B ready 抖动的准确含义

历史 B bag 在 4.202010、4.321829、4.461397、4.621548 s 出现 ready 抖动，
但同一时刻：

    raw_complete=0
    filtered_complete=0
    profile_complete=0
    Marker IDs 0/1/2 全为 DELETE

它只能证明 readiness/query 状态抖动，不能证明已经可见的 tube 闪烁。

### 1.4 当前不能声称的结论

本阶段开始时禁止声称：

- 3 秒 buffer refresh 已被证明是 tube 闪烁主因；
- refresh=0 已被证明能修复问题；
- ESDF snapshot/coherence 已被证明必须实施；
- 旧 A5.2 已完成；
- A5 已完成；
- 可以进入 A6。

## 2. 本阶段唯一目标

A5.2 只完成以下三组工作。

### 2.1 确定性修复

1. 恢复裸 test_gvf.launch 的 baseline 默认值：

       phase_offset_mode=disabled
       phase_offset_manual_tube_source=none

2. 修复 Marker 安全语义：

   - geometry/profile 完整但 failure latched 时，三个 tube Marker 必须立即 DELETE；
   - gate 未开、source 未评估或未 ready、profile 不完整、当前安全条件不成立时，
     三个 Marker 必须 DELETE；
   - 禁止继续把几何 complete profile 显示成当前有效安全 tube。

3. 修复诊断证据链：

   - 不移动现有 diagnostics payload 索引；
   - 明确 sentinel/alias 不是独立 payload 字段；
   - 所有导出必须验证 header count == row count；
   - 旧 /tmp/a5_2_export_csv.py 不得复用。

### 2.2 只读语义诊断补全

补充最小诊断，以区分：

- readiness 尚未评估；
- readiness 已评估但 source 不可用；
- manual preflight 几何失败；
- tube path geometry/regularity 失败；
- unavailable、out-of-map、unknown、occupied；
- insufficient clearance；
- path/source revision 改变；
- tracking bound failure；
- 当前周期 tube 是否具备显示证书。

这些诊断不得改变控制、tube 数学、ESDF 查询结果或 failure 行为。

### 2.3 合格的 refresh A/B

在完全相同的静态 pillar 场景下，对：

    buffer_refresh_period=3.0

和：

    buffer_refresh_period=0.0

进行新的隔离 A/B。

只有满足本执行单定义的连续有效窗口和原始证据完整性要求，A/B 才可用于因果
判断。

## 3. Tube 三层语义必须分开

本阶段必须在代码、测试和汇报中区分以下对象。

### 3.1 几何 profile

TubeProfile.complete 只表示：

- 当前 preview 的离散几何样本完整；
- raw/filtered offset 边界可用；
- 相应 source-specific 构造条件成立。

它不是当前控制周期的完整安全证书。

### 3.2 当前周期显示证书

本阶段允许增加一个纯派生的 tube display certificate。它只决定 diagnostics
和 Marker ADD/DELETE，不改变控制选择、delta、端口或 failure latch。

### 3.3 未来 lifted tube snapshot

路径 revision、ESDF snapshot revision、candidate/installed tube epoch 的原子
绑定不属于 A5.2。

本阶段禁止创建 SafeLiftedTubeSnapshot、ActiveTubeEpoch、MapSnapshot 或等价
持久对象，也禁止把当前 rebuild_count/tube_revision 宣称为论文 tube epoch。

若合格 A/B 证明同一稳定路径下仍存在一致性问题，只能报告建议 A5.3，不能在
本轮实现。

## 4. 开始前完整检查

完整读取：

- 根目录 AGENTS.md；
- 总路线与代码架构；
- A5、A5.1 和本修订版 A5.2 执行单；
- 旧 A5.2 只读审计结论；
- phase_offset_navigation 全部当前源码和测试；
- matched adapter 当前源码和测试；
- test_gvf.launch；
- gvf_manager 中 adapter 创建和 cmdCallback 接入；
- SDFMap 中 buffer refresh、occupancy、ESDF update 和 getDistance；
- formation_planning.cpp 的 spinner；
- 当前 simulator.launch 与 swarm_rviz.rviz。

执行并保存原始输出：

    cd /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
    git branch --show-current
    git rev-parse HEAD
    git status --short
    git diff --stat
    git diff --check
    git diff --cached --stat
    git stash list
    catkin_test_results --verbose
    wc -l \
      src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp \
      src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp \
      src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
    git diff --numstat 9a0e975 -- \
      src/swarm_planner/bspline_traj/src/gvf_manager.cpp

必须确认：

- branch 为 main；
- HEAD 为 9a0e97560b8ddf15ac1ed1581030ed4b911f6e43；
- stash deepseek-phaseoffset-tracked-prototype-2026-08-08 存在；
- 无 staged 内容；
- 旧 A5.2 诊断字段和 launch refresh arg 已不在当前源码；
- A5.1 ribbon 与原 RViz 四个 display 仍存在；
- phase_offset_matched_adapter.cpp 当前约 486 行；
- phase_offset_runtime.cpp 当前约 488 行；
- tube_builder.cpp 当前约 332 行；
- gvf_manager.cpp 相对基线仍为约 +134/-0；
- circle_test/enable=false；
- circle_test/auto_start=false；
- 当前测试只保留既存 uav_utils 缺 XML。

任一前提不符时立即停止。

## 5. 文件白名单

只允许修改以下 10 个文件：

    src/swarm_planner/bspline_traj/launch/test_gvf.launch
    src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_types.h
    src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h
    src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp
    src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
    src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_builder_test.cpp
    src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp
    src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
    src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
    src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp

不允许新增 repository 文件。

ROS bag、CSV、分析脚本和日志只能作为 /tmp 下的本轮测试产物，不得写入源码树。

特别禁止修改：

- plan_env/sdf_map.*；
- formation_planning.cpp 或 spinner；
- gvf_manager.*；
- swarm_rviz.rviz；
- A5.1 ribbon 几何、颜色和 topic；
- fixed/ESDF wrapper launch；
- phase_offset_core；
- TubeFilter、PortProjector、MatchedPort；
- planner、A*、B-spline、C2、governor、PositionCommand、SO3、simulator；
- package.xml、CMakeLists.txt；
- common_msgs；
- prototype 文件；
- 路线文档；
- A5.3/A6 文件。

需要白名单外修改时立即停止，不得扩大范围。

## 6. Baseline launch 修复

在 test_gvf.launch 中只允许对默认与 refresh 参数做以下修改。

### 6.1 恢复默认模式

必须改为：

    <arg name="phase_offset_mode" default="disabled" />
    <arg name="phase_offset_manual_tube_source" default="none" />

要求：

- 不删除 manual/fixed/esdf 显式参数能力；
- A5.1 和 A5 验证仍通过命令行显式开启；
- 不改变任何 planner、速度、增益、tube 或安全参数；
- circle 两项继续为 false。

### 6.2 暴露 refresh 参数

增加：

    <arg name="sdf_map_buffer_refresh_period" default="3.0" />

把现有硬编码替换为：

    <param name="sdf_map/buffer_refresh_period"
           value="$(arg sdf_map_buffer_refresh_period)" />

要求：

- 默认仍为 3.0，与可信 baseline 数值一致；
- 0.0 只用于本阶段静态 A/B；
- SDFMap 当前在 period <= 0 时不创建 buffer timer，必须在汇报中引用源码证据；
- 不把 0.0 写入 wrapper 默认；
- 不把 0.0 宣称为动态地图最终方案。

## 7. Diagnostics payload 兼容规则

现有 payload 的所有实际字段索引必须完全不变。

特别注意：

    kManualLegacyDiagnosticCount

是旧字段数量 sentinel，不是独立 payload 字段。

    kTubeSource = kManualLegacyDiagnosticCount

表示 kTubeSource 复用该数值作为第一个 A5 字段。

任何字段名表、CSV header 或分析字典必须：

1. 只为 kTubeSource 写一个真实列名；
2. 排除 kManualLegacyDiagnosticCount；
3. 排除最终 kManualDiagnosticCount；
4. 保证字段名数量精确等于每行 data 数量；
5. 数量不等时立即退出非零，禁止补零、截断或继续统计。

旧 /tmp/a5_2_export_csv.py 明确禁止使用。

## 8. 允许追加的 diagnostics

新字段只能依次追加在当前 kManualDiagnosticCount 之前，不得插入旧字段中间。

至少追加：

    kTubeReadinessEvaluated
    kTubeDisplayCertified
    kPreflightFirstInvalidW
    kPreflightFirstInvalidSide
    kTubeFirstInvalidW
    kTubeFirstInvalidSide
    kTubeFirstStopReason
    kTubeInsufficientClearanceCount
    kTubeMinWidth
    kTubeMinSafetyMargin

稳定编码：

- side=-1：negative；
- side=0：base、bilateral 或无 side；
- side=+1：positive；
- stop reason 使用 TubeStopReason 的现有枚举整数值；
- 没有失败样本时 first-invalid 数值填 0；
- first-invalid 必须结合 invalid count 解释；
- 所有发布值必须 finite。

除上述追加外，禁止改变消息长度语义、旧字段顺序或旧字段数值。

## 9. Readiness evaluated

在 TubeRuntimeStatus 增加：

    bool readiness_evaluated = false;

语义必须为：

- gate 未打开时 false；
- preflight 未完成且尚未进入 ESDF query 时 false；
- geometry 尚未形成时 false；
- 只有当前 reference 与 actual position 的 ESDF 查询确实执行后才为 true；
- evaluated=false、ready=false 表示尚未评估；
- evaluated=true、ready=false 表示已查询但当前 source 条件不满足；
- fixed tube 进入固定边界检查后可同时设 evaluated=true、ready=true；
- 不改变 gate、selected、failure 或 fallback 行为。

## 10. 首个失败诊断

### 10.1 Manual preflight

ManualPreflightResult 追加：

    first_invalid_w
    first_invalid_side

首次 geometry 或高度一致性失败时记录：

- 原始 path w；
- 当前 bilateral sign；
- 原 invalid_reason；
- 后续失败不得覆盖第一个失败样本；
- 不改变 amplitude reduction 搜索。

### 10.2 Tube builder

TubeBuildDiagnostics 追加：

    first_invalid_w
    first_invalid_side
    first_stop_reason

规则：

- invalid path、regularity、base query failure：side=0；
- positive ray 使 sample uncertified：side=+1；
- negative ray 使 sample uncertified：side=-1；
- 只记录第一个导致 sample/profile incomplete 的失败；
- obstacle 或 insufficient-clearance 作为已认证射线终点时，不得误记为 profile
  incomplete；
- 继续累计 unavailable、out-of-map、unknown、occupied、
  insufficient-clearance；
- min width 和 min safety margin 必须 finite；
- 不改变 ray trace、erosion、regularity 或 profile 接受公式。

## 11. Tube display certificate

必须在 adapter 中使用一个唯一的纯派生判定，供 diagnostics 和 Marker 共用。
禁止在两处复制不同条件。

其语义至少等价于：

    mode is manual
    && zero gate is open
    && geometry is valid
    && tube source is not NONE
    && profile is complete
    && readiness has been evaluated
    && source is ready
    && current delta is inside
    && tracking is within bound
    && failure is not latched
    && (source is FIXED || obstacle certification is true)

可以用小型 helper 表达，但不得增加控制状态。

要求：

- kTubeDisplayCertified 使用该唯一 helper；
- Marker 使用同一个 helper；
- certificate=true 时 IDs 0/1/2 必须全部 ADD；
- certificate=false 时 IDs 0/1/2 必须全部 DELETE；
- 禁止混合 ADD/DELETE；
- failure latch 生效后的第一次 Marker 发布就必须全部 DELETE；
- 不允许保留上一帧 ribbon；
- 不改变 base path、active path 或 frame Marker 语义；
- 不改变 Runtime 的 selected/fallback/control 行为。

## 12. 单元测试

### 12.1 Tube builder

至少覆盖：

- UNKNOWN 首失败 w、side、reason；
- OUT_OF_MAP、UNAVAILABLE、INVALID_PATH、REGULARITY 可区分；
- base occupied/insufficient clearance 的语义正确；
- ray 上 certified obstacle/clearance 截断不会被误报为 profile incomplete；
- insufficient clearance count 正确；
- min width/min safety margin finite；
- 原 sentinel、erosion、regularity 和 conservative tests 全部保持。

### 12.2 Runtime

至少覆盖：

- gate closed：readiness_evaluated=false；
- gate open 且真正执行 ESDF query：evaluated=true；
- query success：ready=true；
- query failure：evaluated=true、ready=false；
- fixed source 的 evaluated/ready；
- preflight 首个失败 w/side；
- failure latch、delta、previous final port、projection、MatchedPort 行为不变。

### 12.3 Adapter 与 Marker

至少覆盖：

- 裸配置 disabled/none 的既有行为；
- 旧 diagnostics 索引逐项不移位；
- kTubeSource 与 sentinel alias 关系显式测试；
- 新字段只追加；
- diagnostics 数组长度等于 kManualDiagnosticCount；
- 所有数值 finite；
- complete profile + failure latch => IDs 0/1/2 全 DELETE；
- complete profile + gate closed => 全 DELETE；
- complete profile + readiness not evaluated => 全 DELETE；
- complete profile + source not ready => 全 DELETE；
- ESDF complete 但未 obstacle certified => 全 DELETE；
- 合格 fixed tube => 三个 ADD；
- 合格 ESDF tube => 三个 ADD；
- kTubeDisplayCertified 与 Marker action 始终一致；
- A5.1 ribbon 三角形数量与边界几何测试保持；
- adapter 仍无 PositionCommand publisher。

## 13. 构建与回归

执行：

    catkin_make -j8
    catkin_make run_tests_phase_offset_core
    catkin_make run_tests_phase_offset_navigation
    catkin_make run_tests_bspline_race
    catkin_test_results --verbose

只允许既存 uav_utils 缺 gtest XML。

执行：

    git diff --check

必须通过。

静态解析：

    source devel/setup.bash
    roslaunch --nodes bspline_race test_gvf.launch
    roslaunch --nodes bspline_race test_gvf.launch \
      phase_offset_mode:=manual \
      phase_offset_manual_tube_source:=fixed
    roslaunch --nodes bspline_race test_gvf.launch \
      phase_offset_mode:=manual \
      phase_offset_manual_tube_source:=esdf \
      sdf_map_buffer_refresh_period:=0.0

不得新增额外 simulator、controller、map publisher 或第二个 RViz。

## 14. 隔离 ROS 规则

开始前检查现有 ROS master 与进程。

- 不得连接用户已有 master；
- 不得终止用户进程；
- 每组使用新的 master 端口；
- 每组使用新的 ROS_LOG_DIR；
- 只清理本轮启动的进程；
- A、B 必须分别干净启动；
- 禁止在同一进程中动态切换 refresh 参数充当 A/B。

保留：

- 原 pillar.pcd；
- 原 simulator、local sensing、planner 和 SO3；
- 原速度、增益、tracking bound、erosion 和 tube 参数；
- circle 两项 false。

## 15. ROS-0：裸 baseline 回归

使用分离式启动：

    roslaunch so3_quadrotor_simulator simulator.launch
    roslaunch bspline_race test_gvf.launch

必须验证：

- phase_offset_mode 实际为 disabled；
- tube source 实际为 none；
- 不发布 phase_offset_manual diagnostics/Marker；
- 不创建 manual matched runtime；
- /position_cmd 唯一 publisher 仍为 /formation_planning；
- 原 planner、ISF-GVF、governor 和 SO3 正常；
- 原 pillar 点到点目标正常到达；
- circle 两项仍为 false。

该回归失败时立即停止，不得进行 A/B。

## 16. ROS-1：显式 fixed tube 回归

显式启动：

    roslaunch bspline_race test_gvf.launch \
      phase_offset_mode:=manual \
      phase_offset_manual_tube_source:=fixed

必须验证：

- 原 simulator RViz 中仍只有一套窗口；
- IDs 0/1/2 在 display certificate=true 时全部 ADD；
- certificate=false 时全部 DELETE；
- fixed tube 不宣称 obstacle_certified；
- matched residual、delta 连续积分、正 phase 和正切向回归不变；
- /position_cmd publisher 不变；
- 无 failure、fallback 或 tube violation。

## 17. ROS-2：合格 ESDF A/B

### 17.1 场景选择

使用相同初始状态、相同目标、相同参数和相同 pillar.pcd。

必须选择一个不修改速度、增益、planner 或安全参数即可获得长时间稳定路径
revision 的点到点路线。可以通过更长的安全起终点距离获得观察时间，但不得：

- 更换地图；
- 添加狭窄通道；
- 启用 circle/figure-eight；
- 降低速度使测试通过；
- 修改 tracking bound；
- 关闭 C2 或 planner；
- 拼接多个不同 revision 窗口凑时长。

若无法获得合格连续窗口，按停止条件报告，不能放宽窗口定义。

### 17.2 合格观察窗口

每组必须存在至少一个连续不少于 15.0 s 的窗口。整个窗口内必须满足：

- zero gate 已打开；
- failure latch 始终为 0；
- current geometry valid；
- manual preflight complete；
- tracking error 始终不超过 0.15 m；
- semantic/source revision 保持完全不变；
- 没有 C2/path install 事件；
- UAV 处于有效点到点飞行过程，不是目标发布前或到达后；
- readiness 每周期均已实际评估。

窗口内允许 source_ready、profile_complete、display certificate 发生变化，因为这些
正是 A/B 的观测变量。

一旦 source revision 改变，当前窗口立即作废并从零重新计时。禁止跨 revision
拼接。

### 17.3 A 组

    roslaunch bspline_race test_gvf.launch \
      phase_offset_mode:=manual \
      phase_offset_manual_tube_source:=esdf \
      sdf_map_buffer_refresh_period:=3.0

### 17.4 B 组

    roslaunch bspline_race test_gvf.launch \
      phase_offset_mode:=manual \
      phase_offset_manual_tube_source:=esdf \
      sdf_map_buffer_refresh_period:=0.0

### 17.5 必须记录

至少记录：

- diagnostics 原始 Float64MultiArray；
- tube MarkerArray；
- odometry；
- position command；
- source revision；
- tube revision；
- readiness evaluated/ready；
- raw/filtered/profile complete；
- obstacle certified；
- display certified；
- first invalid w/side/reason；
- unavailable/out-of-map/unknown/occupied/insufficient counts；
- min width/min safety margin；
- current bounds；
- reference/actual distance；
- tracking error/bound；
- failure、selected、fallback；
- 每次 Marker ADD/DELETE 时间；
- 当前 UAV 速度。

注意：bufferRefreshCallback 在估算速度大于 1.0 m/s 时会提前返回，因此不能只凭
每 3 秒近似周期就声称回调实际执行。A/B 结论必须基于两组差异，而不是只看
周期。

## 18. 原始证据与导出完整性

rosbag 是唯一原始事实来源。

允许在 /tmp 创建本轮专用分析脚本，但必须：

1. 明确列出唯一 payload 字段表；
2. 排除 kManualLegacyDiagnosticCount sentinel；
3. 排除 kManualDiagnosticCount；
4. 只保留一次 kTubeSource；
5. 每条消息先断言 len(data) 等于预期字段数；
6. CSV 写入前断言 len(header) == len(row)；
7. 任一断言失败立即退出非零；
8. 不得读取或导入旧 /tmp/a5_2_export_csv.py；
9. 最终汇报给出分析脚本路径和 sha256；
10. 保留原始 bag，不得只提交 CSV。

Marker 与 diagnostics 对齐必须使用消息时间戳，并报告允许的最大配对时间差。
不能把相距数百毫秒的状态当作同一周期。

## 19. A/B 结果判定

### 19.1 可以认定静态 refresh 是主要触发因素

只有同时满足：

1. A、B 均有合格的至少 15 s 连续窗口；
2. A 在同一稳定 source revision 下发生至少两次 display-certified
   ADD→DELETE→ADD；
3. 每次 DELETE 都有 readiness/profile 诊断对应；
4. A 的事件具有与 refresh 配置一致的重复性；
5. B 在相同条件下事件消失或显著下降为 0；
6. 两组没有 failure、tracking violation、collision 或 publisher 变化；
7. B 的 bounds、clearance 和 obstacle certification 仍满足 A5 安全语义。

此时只允许结论：

    对当前静态 pillar 单机仿真，显式 refresh=0 可作为隔离验证配置。

不得推广为动态地图最终方案。

### 19.2 两组都稳定

结论只能是：

    本轮未复现用户观察到的可见 tube 闪烁。

不得声称 refresh 已修复问题，也不得进入 A5.3。

### 19.3 两组都发生同 revision 不可用

如果 A、B 均在合格窗口内出现 display-certified 中断：

- 按 first failure 和计数分类；
- 报告是否为 unknown/out-of-map/insufficient/geometry；
- 报告可能需要 A5.3 coherence；
- 禁止本轮实现 snapshot；
- 禁止直接把 readiness 抖动称为可见 tube 闪烁，必须有 Marker
  ADD→DELETE 证据。

### 19.4 无法取得合格窗口

若因 source revision、C2 或 tracking blocker 无法获得 15 s：

- A/B 判定为无效；
- 保留确定性修复和测试结果；
- 明确记录哪个条件最先终止窗口；
- A5 继续 blocked；
- 禁止通过拼接窗口、放宽 bound 或修改速度补验。

## 20. 确定性修复验收

无论 A/B 结果如何，A5.2 的确定性修复只有在以下全部成立时才算通过：

1. 裸 test_gvf.launch 为 disabled/none；
2. 裸 baseline 不发布 manual topics；
3. refresh 默认仍为 3.0；
4. 显式 manual/fixed/esdf 仍可用；
5. failure latch 后首个 Marker 周期即三个 DELETE；
6. kTubeDisplayCertified 与 Marker action 始终一致；
7. 没有混合 ADD/DELETE；
8. diagnostics 旧索引未移动；
9. header/row 数量检查通过；
10. build 和所有阶段回归通过；
11. /position_cmd publisher 未改变；
12. 没有修改安全、控制、C2 或地图算法。

## 21. 行数与依赖边界

要求：

- phase_offset_matched_adapter.cpp 不超过 500 行；
- phase_offset_runtime.cpp 不超过 500 行；
- tube_builder.cpp 不超过 380 行；
- 不得压缩为不可读单行规避限制；
- 合理实现若超过上限，停止并报告需要新的拆分授权；
- navigation 继续只依赖 Eigen/STL 和 phase_offset_core；
- navigation 不得依赖 ROS、SDFMap、plan_env、manager、neighbor、swarm 或 CBF；
- core 继续纯 C++14/Eigen。

## 22. 严格禁止

1. 修改 SDFMap 或 AsyncSpinner；
2. 实现 ESDF snapshot/coherence；
3. 创建 active tube epoch 或 lifted tube snapshot；
4. 把旧 profile 强制留在 RViz；
5. 把 UNKNOWN 当作自由空间；
6. 改大 unobserved threshold；
7. 减小 reference clearance；
8. 放宽 0.15 m tracking bound；
9. 修改 tube max offset、ray step、erosion 或 regularity；
10. 修改速度、增益、planner 或 C2；
11. reset/clip delta；
12. 修改 final port 一致性；
13. 修复 A5 tracking blocker；
14. 使用旧错位 CSV 形成结论；
15. 跨 source revision 拼接有效窗口；
16. 修改 RViz topic 或新开第二个 RViz；
17. 进入 A5.3、A6、swarm、CBF 或多机阶段；
18. commit、tag 或 push。

## 23. 停止条件

出现以下任一情况立即停止并报告：

- branch、HEAD、stash 或起始 diff 不符；
- 需要白名单外修改；
- 裸 baseline 无法恢复；
- failure 后 Marker 不能在首周期 DELETE；
- diagnostics 需要移动旧索引；
- header/row 数量无法严格一致；
- 合格窗口不足 15 s；
- source revision 在候选窗口内变化；
- tracking error 超过 0.15 m；
- 需要修改速度、地图、margin、C2 或控制才能完成 A/B；
- B 组仍有同 revision 可见 tube 中断；
- 新回归失败；
- ROS runtime 不可用；
- 行数或依赖边界无法保持。

停止不等于擅自进入 A5.3 或 A6。

## 24. 自审核

结束前执行：

    rg -n "phase_offset_mode|phase_offset_manual_tube_source|sdf_map_buffer_refresh_period|buffer_refresh_period" \
      src/swarm_planner/bspline_traj/launch/test_gvf.launch

    rg -n "snapshot|MapSnapshot|ActiveTube|SafeLiftedTube|Continuation|neighbor|swarm|CBF" \
      src/swarm_planner/phase_offset \
      src/swarm_planner/bspline_traj/include/bspline_race/integration \
      src/swarm_planner/bspline_traj/src/integration

    rg -n "PositionCommand|cmd_pub|publishGovernorPositionCommand|SO3" \
      src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp

    wc -l \
      src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp \
      src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp \
      src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp

    git diff --numstat 9a0e975 -- \
      src/swarm_planner/bspline_traj/src/gvf_manager.cpp

    git diff --check
    git status --short
    git diff --stat
    git diff --cached --stat
    git stash list

要求：

- 只修改 10 个白名单文件；
- 无新增 repository 文件；
- plan_env、spinner、manager、RViz、core、TubeFilter diff 未变化；
- launch 默认 disabled/none/refresh=3.0；
- circle 两项仍为 false；
- 安全、速度、tracking、C2 参数未变化；
- 无 staged、commit、tag 或 push；
- 只清理本轮 ROS 进程；
- 隔离 master 端口释放。

## 25. 最终汇报

最终汇报必须包括：

1. 实际修改文件；
2. 修复前后的 launch 默认值；
3. tube display certificate 的唯一条件；
4. failure latch 到首个 DELETE 的测试与 ROS 证据；
5. 新 diagnostics 的完整索引表；
6. sentinel/alias 的唯一导出规则；
7. header count、row count 和断言结果；
8. 原始 bag、分析脚本路径及 sha256；
9. A、B 精确命令；
10. 每组合格窗口起止时间、长度和 source revision；
11. 每次 display ADD/DELETE transition；
12. first failure w/side/reason 和分类计数；
13. tracking、clearance、bounds、publisher 结果；
14. 是否真正复现可见 tube 闪烁；
15. refresh=0 是否在合格条件下改变结果；
16. build/test 回归；
17. 行数、依赖、白名单、Git、stash 自审核；
18. 明确说明旧 A5.2 实现与旧 CSV 未被采信；
19. 明确说明没有实现 snapshot、continuation、A5.3 或 A6；
20. 明确说明 A5 tracking blocker 仍保留。

若某项因停止条件未执行，必须在对应条目中明确写“未执行”及触发停止的原始
证据，禁止虚构 A/B 窗口或用旧 bag 补齐。

只有全部确定性修复、回归和合格 A/B 均实际完成时，最终一行才允许为：

    A5.2 修订版完成后已停止；A5 继续 blocked，未进入 A5.3 或 A6。

若触发任一停止条件，最终一行必须改为：

    A5.2 修订版按停止条件终止；A5 继续 blocked，未进入 A5.3 或 A6。
