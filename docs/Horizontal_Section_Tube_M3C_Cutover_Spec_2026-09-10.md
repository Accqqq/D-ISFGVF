# M3C：实际地图与规划/执行成组切换

日期：2026-09-10。状态：R2已由主会话冻结，依据用户继续执行授权，经Astra medium完整只读审核后放行本规格A–D配套实现。Luna是唯一编码代理；主会话监督、审核和独立验收。8.3–8.5来自执行者核查建议，由主会话逐条审核修订后生效，执行者不得自行更改规格。

## 1. 本批终点

完成后生产MANUAL/coordination链为：原planner路径及C2 → 原地图一致局部读取 → SectionBuilder同步构建 → immutable path/profile组合 → Section Preview/Allocator → Runtime Section单拍 → 原governor/SO3，发布成功后w/delta同事务提交。

旧tube timer/worker/map-ID安装机制从生产调用退出，baseline SDFMap cpp/h成对恢复核心算法，仅补局部读取与线程一致性。不能在当前主工作区交付一个只拆旧链、尚未接新链的状态。构建中间态不运行ROS。

本批不是再做独立数学模块：M1A/M2/M2B/C/D/M3A/B已验收的模块全部复用。旧tube来源ESDF选择映射到新的原inflated occupancy section后端，不再ray/freeball，不同时生产两种tube。ACTIVE零port行为和关闭phase-offset的原导航保持。

## 2. 固定执行模型

唯一delta与previous_u仍在PhaseOffsetRuntime，唯一w仍在manager。一个task generation用于新目标/reset淘汰旧候选，一个pending命令标识防止错命令发布；不新增map/support/configuration身份序列或第二状态owner。

SectionPathBundle为薄integration值（优先放已有phase_offset_section_input.h/.cpp，不另造package）：shared_ptr<const ContinuousPhasePath> path、shared_ptr<const ContinuousPhaseNormalFrame> frame、shared_ptr<const SectionTubeProfile> profile、frame_id、task generation、构建参数副本、构建时局部reference/obstacle域。候选与当前组合不可变，profile来自同path，不能事后贴新revision。

提交检查只负责task/源path仍适用、最新w/delta在可用范围、路径替换时仍在真实共同前缀。几何已在构建完成，不在安装时再跑整条tube证明。地图静态前提和真实改变的处理见第5节。

## 3. 规划内同步构建

移除phaseOffsetTubeTimerCallback中的几何捕获与worker调度，取消专用tube timer/thread。FSMCallback已有planning计时入口按tube_update_period进行同路径局部刷新，规划/C2/地图capture/Section build均在规划线程同步，不能在cmdCallback/runtime锁中计算。普通odom重复和同内容cloud不因消息序号拒绝候选。

初始路径：保留原planner install/acquisition；在同一次FSM更新对已确定path构建候选。零delta时新tube失败不能撤销原路径或永久HOLD，仅禁横向协调并继续原导航。

重规划：保留已有future seam + appendSlice真实共同前缀；获取当前source path和w快照后构建candidate path，不锁控制线程。同步捕获局部地图/构建candidate profile，最后只在短提交区检查generation、source owner、最新相位仍在prefix内；若越过prefix则不采用该候选，不清delta、不硬跳reference。回零/重试见第7节。

C清理返修澄清：不得将FSM的point/closed统一相位REPLAN分支当成V2证书代码整体删除，也不得以测试局部实现替代被删生产seam选择。恢复原候选评分、真实B-spline弧长映射、C2/结构breakpoint选择与闭合目标记录，只替换旧binding/worker安装消费者。有真实copied prefix的中性安装允许同任务同source、flags不变、相位单调推进且仍在prefix内；无prefix保持精确phase元组校验。中性安装必须在frontend→handoff→phase→adapter task→runtime序列下确认delta为零；成功时一起撤销旧pending并退役旧Section组合/hand-off，锁外回收，不改delta或增加“bundle必须为空/候选同身份”条件。非零只走Section路径发布切换。EXEC_TRAJ近终点时非零delta保持receive_goal和控制/原回中重规划，不因XY距离独自切WAIT_TARGET；零delta保持原到达阈值，不新增delta容差或清零跳变。

地图ROI必须局部而非全图：待构建实际phase区间端点/单元p_ww界给出每坐标路径包围，再按half_width仅扩XY（N_z=0），按clearance取得halo；不得只用离散点极值。优先限定现有local_update_range内前向段和已知域，不把请求范围扩大到任务全图。

参数保持：clearance读取planning/safe_distance（现默认.4），half_width读取现phase_offset/tube/environment_search_extent（现默认3，不偷偷改为离线默认1），minimum_reference_speed保持现值；Section细分/工作budget用已验收默认并记录结果。不得为了成功调预算/缩短profile却不报PARTIAL，不调SPH或控制增益。原raw_tracking_error_bound继续用于执行tracking，不再重复加到几何净空。

## 4. 真实命令接线

matched adapter的MANUAL生产分支改消费当前/候选Section bundle，不调用worker、V2 admission、TubeExecutionGuardV2或finite reserve。复用M3A preview/allocator、M3B prepareSection与MatchedPort，g_des仍按原SPH/D1B/回中公式解析，不改协调数学。

Adapter更新流程：按当前bundle提供实际p/N，读取Runtime当前delta/previous，算geometry与原IsfReferenceKernel，Section preview→allocator→prepareSection。所有失败按第7节返回真实状态；成功只创建私有prepared值和pending bundle/执行reference query，不改变runtime或manager。

governor保持原速度匹配/PositionCommand算法，不逆向从PositionCommand推port、不新增直接SPH物理叠加。matched同final_u已用于reference推进和物理速度意图；governor作为原低层命令整形可能使实际物理轨迹不精确等于数学意图，这属于tracking界/仿真验证，不能宣称PositionCommand与port数值相等。

governor使用该pending bundle实际path和pending.nextDelta的immutable executed reference query，允许查询范围不得越过已构建profile有效段（原基路径末端/terminal另处理）。同一cmdCallback的input_dt、nextW、nextDelta与pending一致；governor返回invalid/hold或取消pending时不提交runtime步。

发布事务沿用manager frontend→handoff→phase、adapter task_publication→runtime锁序。发布前核对pending/current task、当前路径/共同prefix、Runtime.validateSectionBeforePublish，随后实际publish；成功后Runtime.commitSectionNoFail、manager phaseNoFail及bundle切换一起提交，无late failure。失败publish不推进状态。旧authority snapshot/map binding/proof ID比较不再参与。

新目标/reset不提交旧pending，非零offset不得以普通tube刷新为由清零。真实任务reset沿原约定重置，终点不能因旧pending cancellation直接阻止原到达命令；补terminal cancellation回归。

## 5. baseline地图恢复与局部读取

只读baseline B=/home/cxq/Sim_demo/New_ISFGVF/gvf_ws，sdf_map.cpp/h核心恢复对应9a0e975；不复制B launch。删除map instance/support/configuration identity、全图mask、cloud snapshots的生产生成及capture/evidence API。旧证书专用cpp退出plan_env生产target，不用假返回值stub维持引用。

新增/保留最少的地图owned recursive mutex（无ID）；所有map写入口、公开可独立调用的reset/setOccupied/setOccupancy/updateESDF、组合trilinear/occupancy/getDistance/nearestfree/可视化/ready读在同一锁域。纯初始化后固定坐标换算不必每次加锁；嵌套调用允许recursive避免改原算法。

map方法readLocalObstacleView(request)由map内部填known域和3层buffer借用，锁内调用M1A buildLocalObstacleView局部扫描、返回自持有盒，锁外Section geometry。调用方不能传假的known=true或以空occupancy推断已知。

首版只接受明确opt-in的完整静态local_sensing输入。复用实际publisher namespace的range/frame/odom流以及256条stamp→center历史；cloud非零stamp代表原local_sensing完整性声明，同流精确stamp匹配源中心。不得读取全局mock_map建立旁路地图，不能拿receiver最新中心替代source中心。

known域取source闭盒∩baseline实际receiver严格开盒∩有效map边界，然后扣原inflation stencil+完整体素影响（XY(inf_step+1)*resolution，Z2*resolution）并向内对齐完整native体素边界；这保证已有inflated层完整，不是再次膨胀。builder自己的clearance仍单独保持。

数值验收澄清（C子批主会话审核补充，不改变上述几何合同）：必须保留实际相交面的开闭属性；精确可表示的source闭边界、闭map边界和精确halo运算不能无条件nextafter后再ceil/floor，导致额外丢整格。必要算术误差应有对应向内包围，receiver实际严格开边界仍不能伪作闭边界。最小反例为origin=0、resolution=1、map=[0,8]³、source/receiver中心均4、source range=3、receiver range=4、inflation=0：应得known XY=[2,6]、Z=[3,5]，不得误报UNKNOWN；source/receiver range均3的严格开边界负例继续保留。由Luna在既有白名单map实现和capture测试内修正，不能通过更改halo/clearance/预算绕过。

完整空云允许更新known域，但保持baseline空云不写occupancy/ESDF；未声明/不匹配/非finite/未知输入不能成为已知空闲。保留static/manual覆盖层和原ceiling/边界行为，不改变点云采样密度/规划器参数。

reset/gradualReset/depth清图在同锁使未来known声明失效，下一完整云恢复。已发布的immutable view是固定静态世界观测，在同静态环境下不因重复消息或缓存清理序号失效；真正手动障碍/有效边界/静态来源配置变化必须显式通知规划端撤销受影响bundle，不能假装“所有地图永远不变”。此通知不使用地图ID；具体最小机制待第8节审核冻结。

## 6. 构建与退役策略

先保存所有修改源和测试清单，再迁manager/adapter消费者，最后恢复map和移除旧API；同一规格分子批但最后必须完整编译后才能运行。

生产目标不链接旧tube_runtime_v2、tube_v2_diagnostics、cloud_occupancy_query、旧marker证书重载、旧raw/epoch诊断或map cloud snapshot/evidence生产实现。数学回归仍可保留legacy离线target；对于依赖已移除生产V2 API的旧测试，不得随便删断言：逐项登记被替代行为和新的测试位置，再退出旧专属target。源码历史文件可以保留只读但不得export到生产。

这一步的精确文件白名单和旧测试映射必须根据Luna的直接consumer闭包表冻结，不能以目录通配让执行者自由删文件。

## 7. 失败处理必须可执行

零delta且原planner可用：tube/preview/横向capacity缺失只禁coordination，沿原nominal导航，不以证书失败持久HOLD。UNKNOWN不伪造ZERO_ONLY。

非零delta：优先保留已提交路径/reference和仍适用的旧profile，以原g_des=-gain*delta*N请求平滑回中；新候选不兼容不能直接归零或切新path。若正常preview或单拍不可行，需要明确有界回收/重规划处理，不能把“继续HOLD并不停重试”写成完成。已经移除finite reserve后，何种最小回收策略覆盖此情况必须在第8节明确，不能临时重新添加旧reserve。

真实tracking超限/几何碰撞与单纯tube构建失败分开报告。原physical emergency处理保持；不能因无可行新tube而谎称安全，也不能无视非零活动reference连续性。

## 8. 编码前仍须冻结的关键项

1. 精确生产修改白名单、旧测试到新测试映射（Luna只读核查）。
2. 非零delta处new K不兼容、原NORMAL单拍无解时的最小有界回收行为，不用reserve/全新状态机、不瞬移、不无限HOLD。
3. 真地图改变撤销旧静态bundle的最小通知/锁语义，不用新地图ID，不能让callback在map锁内反向取adapter锁。

这三项已按8.1–8.6由主会话收敛并经Astra审核；以下R2是执行依据。

### 8.1 非零退化的首版精确边界

本批不新增恢复求解器，不复制I伪造FEASIBLE K，不重引finite reserve。新候选不可采用时保持旧bundle与delta，以既有NORMAL preview和allocator在旧bundle上请求原回中g_des，同时向FSM发起重规划请求。上一拍向外port受slew限制时允许逐步回收，不能强制每拍delta*u_delta<0。

若旧bundle本拍候选也未通过，报告`selected_step_unavailable`而非`physical_infeasible`，因为一条候选失败不代表整个约束集无解；原emergency控制保持当前位置，w/delta均不提交，FSM仍能工作并尝试生成与保留reference相容的候选。不得清零/切基线伪装恢复，不宣称此状态有安全可执行port。

为了避免把无限HOLD算成功，隔离验收将此作为明确失败场景：有限测试观察窗口内必须恢复并到达，否则此场景验收失败并报告阻断，不能宣布全切换完成。本批不承诺对任意几何/端点/拥堵必能回零；如果正常单机/可控非零切换例频繁进入该情况，回到主会话补最小回收设计而不是执行者擅自放宽速率。

### 8.2 静态观测有效性的同步边界

在map局部读取wrapper中使用一个共享的静态环境有效标记及短事务互斥量。它不是map ID：无序号、不比较token地址、不挂proof string，不随普通消息递增。

真实manual/边界/来源配置改变在同一`environment_change_mutex`下将此前捕获的标记置invalid，再修改地图；结束后为之后的新capture建立有效标记。仅普通cloud重复与缓存reset不使已捕获静态世界观测无效（reset仍清掉未来known声明）。源变化/未知输入不能凭新标记假造完整域。

新bundle携带这个map-owned标记及共享mutex。发布短事务先取得environment_change_mutex，再按原manager/adapter锁顺序做最后valid检查→publish→commit；map真实变更回调按environment_change_mutex→map_data_mutex，不反取manager/adapter锁。普通cloud/ESDF只map_data_mutex，不跨入发布锁。这样正常地图重计算不阻塞发布，真正环境变化与发布有确定顺序。

capture只取`map_data_mutex`，复制不可变局部盒并保留当时map-owned `EnvironmentValidityState` 的shared_ptr；capture不直接读取非atomic的valid bool。发布前在`environment_change_mutex`下读取该state的valid，若与真实变更竞争而持有旧state，会看到invalid并拒绝。任何需要重新分配的旧bundle回收在短提交锁外进行，不能在publish锁内析构大数组。

真实变更持有environment_change_mutex→map_data_mutex，在同一个map临界区内完成旧state失效、地图修改、known决策和新state安装；capture只能见完整变更前或变更后状态。state的valid bool只在environment mutex内读写，map mutex保护shared_ptr安装/复制；无需atomic、序号、ID或地址比较。新state有效仅表示环境假设未被后续修改，不代表known域完整；来源/配置变化须known=false，直到匹配完整观察恢复。

map header所需最小wrapper `LocalObstacleCapture`（含`LocalObstacleView`、map-owned `EnvironmentValidityState` shared_ptr及共享environment mutex）只在plan_env/integration层，不传到navigation/core。局部读取不返回V2 capture对象。物理安全依赖固定静态完整观测声明，真机动态障碍不在本批范围。

### 8.3 精确修改白名单（编码批次候选）

以下路径相对于 `W=/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`，是本批唯一可以修改的源码、测试、构建和启动文件。列表不是目录通配；若实现需要列表外路径，Luna必须停止并报告，不得自行扩展。

规格与验收记录本身允许主会话维护：`docs/Horizontal_Section_Tube_M3C_Cutover_Spec_2026-09-10.md`、`docs/Horizontal_Section_Tube_M3C_Report_2026-09-10.md`。

生产接线与值接口：

- `src/swarm_planner/bspline_traj/CMakeLists.txt`
- `src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h`
- `src/swarm_planner/bspline_traj/src/gvf_manager.cpp`
- `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h`
- `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
- `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_section_input.h`
- `src/swarm_planner/bspline_traj/src/integration/phase_offset_section_input.cpp`
- `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_executed_reference_query.h`
- `src/swarm_planner/bspline_traj/src/integration/phase_offset_executed_reference_query.cpp`
- `src/swarm_planner/phase_offset/phase_offset_navigation/CMakeLists.txt`
- `src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h`
- `src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp`
- `src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_viability.h`
- `src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_viability.cpp`
- `src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_allocator.h`
- `src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_allocator.cpp`
- `src/swarm_planner/plan_env/CMakeLists.txt`
- `src/swarm_planner/plan_env/include/plan_env/sdf_map.h`
- `src/swarm_planner/plan_env/src/sdf_map.cpp`
- `src/swarm_planner/plan_env/include/plan_env/local_obstacle_view.h`
- `src/swarm_planner/plan_env/src/local_obstacle_view.cpp`

`LocalObstacleCapture`（含`EnvironmentValidityState` shared_ptr，不含可伪造ID）应在已有 `local_obstacle_view.h` 的 plan_env/integration边界内增加；本批不新建第二个地图/证书包，不把capture wrapper传入 `phase_offset_core` 或 navigation数学层。Section bundle只携带path/frame/profile及构建时局部盒和该map-owned有效state引用。

允许新增唯一实际地图回归文件 `src/swarm_planner/plan_env/test/local_obstacle_capture_test.cpp`，在既有plan_env CMake注册，覆盖真实SDFMap而不是再造模拟地图算法。

另允许在既有 `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_tube_markers.h`、`src/swarm_planner/bspline_traj/src/integration/phase_offset_tube_markers.cpp` 及 `src/swarm_planner/bspline_traj/test/phase_offset_tube_markers_test.cpp` 增加Section可视化入口（仅path+profile→左右边界线）；旧V2 marker调用从生产退出，可留纯离线函数。不得把旧marker的mapID认证复用到新入口。构建marker在规划线程，控制线程不遍历路径采样。

启动参数接线（只删除失效tube/证书参数或改为Section等价字段；保留 `test_gvf.launch` 的 `circle_test/enable=false` 与 `circle_test/auto_start=false`）：

- `src/swarm_planner/bspline_traj/launch/test_gvf.launch`
- `src/swarm_planner/bspline_traj/launch/phase_offset_esdf_tube_single.launch`
- `src/swarm_planner/bspline_traj/launch/phase_offset_fixed_tube_single.launch`
- `src/swarm_planner/phase_offset/phase_offset_sim_bringup/launch/phase_offset_agent.launch`
- `src/swarm_planner/phase_offset/phase_offset_sim_bringup/launch/phase_offset_swarm.launch`
- `src/swarm_planner/phase_offset/phase_offset_sim_bringup/scripts/swarm_orchestrator.py`

测试迁移（只保留下面列出的测试源可改动）：

- `src/swarm_planner/bspline_traj/test/gvf_switch_policy_test.cpp`
- `src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp`
- `src/swarm_planner/bspline_traj/test/phase_offset_cloud_occupancy_query_test.cpp`
- `src/swarm_planner/bspline_traj/test/phase_offset_clearance_audit_test.cpp`
- `src/swarm_planner/bspline_traj/test/phase_offset_executed_reference_query_test.cpp`
- `src/swarm_planner/bspline_traj/test/phase_offset_section_input_test.cpp`
- `src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp`
- `src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_viability_test.cpp`
- `src/swarm_planner/phase_offset/phase_offset_navigation/test/phase_offset_allocator_test.cpp`
- `src/swarm_planner/phase_offset/phase_offset_navigation/test/section_tube_test.cpp`
- `src/swarm_planner/plan_env/test/cloud_occupancy_snapshot_test.cpp`
- `src/swarm_planner/plan_env/test/local_obstacle_view_test.cpp`
- `src/swarm_planner/plan_env/test/manual_map_layer_test.cpp`

旧worker、cloud snapshot/evidence、raw/epoch诊断的独立库从生产target退出；有历史价值且可独立编译的函数/测试可在既有CMake的非导出test-only target保留。对无法在恢复后的SDFMap API上编译的旧evidence target，按8.4迁移其有效地图断言后停止注册，不加stub保留不存在的API。除上述白名单外，不修改旧头/源。`phase_offset_core`、原governor/SO3、SPH消息/算法、simulator感知实现和其他launch保持只读。

兼容源码隔离边界：本批首先必须消除实际生产调用和实例（worker线程、V2 admission/reserve/authority与SDFMap证书API），而非为满足字符串搜索把所有历史值类型也重写一遍。Navigation已共用的viability/allocator/runtime文件若旧离线入口尚需旧V2数学链接，可暂保留这些不被新adapter/manager调用的兼容定义；报告列明待M5剥离的残留，不能宣传源码中完全没有V2。这不允许生产创建任何V2证书/身份/worker或以旧入口fallback。禁止为一次纯类型清理新增多个V3/legacy头文件。

### 8.4 旧测试到新测试的固定映射

| 旧目标/测试范围 | M3C处理与新覆盖位置 |
|---|---|
| `gvf_switch_policy_test` 的 planner/governor/C2、initial-acquisition、phase-commit/publication断言 | 保留原断言；V2 handoff/Stage6用Section bundle、共同前缀和terminal cancellation重写，不能删掉发布失败不提交语义。 |
| `phase_offset_matched_adapter_test` 的 `S2B` worker、admission、reserve、binding、map-ID/epoch用例 | 退出生产target；有价值的 pending publication、failed publish、reset/goal override、terminal cancellation改测Section pending bundle；worker/证书专属断言移至test-only或退役并登记。 |
| `phase_offset_cloud_occupancy_query_test` 的legacy snapshot状态 | 保留为test-only snapshot回归；V2 bridge/capture/free-ball改测 `SDFMap::readLocalObstacleView`、完整/部分/unknown域、静态/手动层和锁边界。 |
| `phase_offset_clearance_audit_test` 的几何、margin、tracking和失败分类 | 保留；capture/V2 provenance部分改用LocalObstacleView/Section输入，不以空occupancy或假ID替代unknown。 |
| `plan_env/cloud_occupancy_snapshot_test` 的空云、reset、occupancy、旧内容隔离和并发语义 | 基线语义保留在test-only或迁入map/local-view测试；所有SDFMap V2 capture、support/map-ID测试改到局部读取和environment-validity并发测试。 |
| `local_obstacle_view_test`、`manual_map_layer_test` | 保留并扩展完整/部分/unknown域、空云、reset后known失效、inflated/manual/static/ceiling/boundary层及map锁一致性。 |
| `phase_offset_navigation/test/runtime_test` | Section单拍、selected-u同一性、非零delta失败不提交、ABA/reset、发布后no-fail commit保留；旧V2 reserve/identity断言只在test-only legacy目标，不得成为生产入口。 |
| `tube_viability_test`、`phase_offset_allocator_test` | 保留Section与既有数学回归；旧V2身份覆盖只作隔离离线回归，Section分支继续验证非均匀knots、PARTIAL/terminal、错profile/w/delta/path/frame。 |
| `phase_offset_section_input_test`、`section_tube_test` | 作为生产Section构建/几何的主要单元和独立oracle覆盖，不复制V2证书fixture。 |
| `phase_offset_tube_runtime_v2_test`、`phase_offset_tube_v2_diagnostics_test`、`phase_offset_tube_epoch_integration_test`、`tube_runtime_v2_acceptance.py/.test` | 不得链接生产库；若保留，只能是明确test-only legacy目标并标注旧链；若无独立价值则退出测试注册。 |
| `phase_offset_tube_markers_test`、`phase_offset_raw_candidate_diagnostics_test`、`phase_offset_tube_epoch_diagnostics_test`、`phase_offset_recovery_continuation_provider_test`、`phase_offset_environment_evidence_query_test`、`plan_env/sdf_map_environment_evidence_test` | 不进入生产；保留的原始/evidence断言单独链接test-only实现，Section marker/诊断需另有明确字段后才可新增，不能把旧V2 marker标成Section active。 |

旧测试若因API退出而删除，提交记录必须在M3C报告中注明原测试名、被替代的Section/local-view断言和未覆盖的旧证书语义；不能以删目标代替迁移。

### 8.5 生产target闭包与锁顺序验收

实际运行调用闭包中允许的tube实现为SectionBuilder、Section Preview/Allocator、Runtime Section和MatchedPort；formation_planning/bspline_gvf不得创建或调用旧worker、TubeExecutionGuardV2、execution authority/recovery reserve、SDFMapCaptureV2、CloudOccupancySnapshot、free-ball/evidence或V2 marker。旧独立worker/diag库不链接生产adapter。Navigation共享文件内仅为离线兼容而保留的旧类型/函数，遵循8.3明确边界，不能被误作新生产要求。

发布事务固定为：地图真实变更 `environment_change_mutex → map_data_mutex`；局部capture只取 `map_data_mutex` 并复制view及共享`EnvironmentValidityState`指针（不读非atomic bool）；发布短事务先取 `environment_change_mutex`，在该锁下读取state.valid，再按现有 `frontend_apply_mutex_ → path_reference_handoff_mutex_ → authoritative_phase_mutex_ → adapter task_publication_mutex → runtime_command_mutex` 顺序完成valid检查、实际publish和所有no-fail commit。任何map回调不能反取manager/adapter锁，普通cloud/ESDF更新不能取得发布锁。析构旧bundle/大数组在短提交锁外执行。

本节已由主会话核对白名单/消费者与测试映射，并经Astra只读审核。本批按R2显式放行，map有效状态不得泄漏到navigation/core。

### 8.6 实现子批及可执行接口

本规格在一次授权中覆盖下列紧密配套子批，最后全链编译/回归前不运行仿真，不把中间无法编译状态当交付。实现者每个子批节点回报，主会话审核后继续同规格范围，不自行设计新阶段。

A. map/Section bundle值类型与capture接口：

- `EnvironmentValidityState`含`bool valid=true`，由同一map的shared environment mutex保护；`LocalObstacleCapture`含M1A view、shared validity与shared mutex。不能从public request传入标记或known域，map方法替换request中known字段。
- `SDFMap::readLocalObstacleView(const plan_env::LocalObstacleRequest&) const` 返回LocalObstacleCapture；读取失败返回view status UNKNOWN/INVALID而不是空free。
- SectionPathBundle包含path/frame/profile和capture有效状态，新增到existing section_input.h。可用`shared_ptr<const SectionPathBundle>`贯穿FSM/adapter。没有worldframe转换，保留现world约定。
- SDFMap现有可复制行为若测试依赖，可用无ID的copyable mutex wrapper（复制后独立mutex、独立validity、buffers复制）维持；禁止两个独立地图复制体意外共用invalid标记。复制操作只有源数据稳定时支持；不宣称并发copy-assignment安全。

B. adapter/manager消费者：

- matched adapter原类保留，但MANUAL constructor只初始化现Runtime，不创建worker、不要求tube_certificate_v2.complete或manual waveform amplitude/preflight字段；配置仅实际numeric policy/limits/Section参数。ACTIVE原零port比较保留。
- 提供`stageSectionBundle(bundle, source_path, copied_prefix_start, copied_prefix_end)`供FSM短事务暂存，及`captureSectionBundle()`供cmd读取已提交bundle。task/reset/current path适用性由既有generation和不可变path共同确认，不伪造旧TubeV2ExecutionBinding。
- `update`只对同一captured bundle计算M3A/M3B链，成功暂存prepared+candidate/source bundle和pending executed query；新pending类型不含V2字段。每周期只一个pending，失败publish仍保留当前权威bundle。
- `publishPendingPositionCommand`保留回调入口并迁移实现，validated pending身份是单次command事务标识，不是mapID；noFail后清pending、提交bundle，旧大对象延后锁外析构。取消pending的真实terminal override在delta==0时允许原goal publish；delta!=0不瞬移，走明确回中/重规划。
- manager的V2 handoff DTO迁为只含source/candidate path、copied prefix范围、frontend mirror、Section bundle和task generation；保持原C2构造位置和旧phase原子提交，不重写planner。FSM同路径刷新与candidate构建共用一个同步helper；cmdCallback只拿不可变结果，不调地图扫描/builder。
- `pendingExecutedReferenceQuery`用现有query类并持有本拍nextDelta、真实path；如需限制domain，给现query构造器增加可选有效域而不更改已有constructor语义。profile末端约束在新adapter构造该query时给出。

C. map核心恢复/旧目标退役：只有B中旧消费者已改后才成对恢复baseline map，补最小同步和A接口。停止旧tube timer创建/worker调用、旧full capture生产及marker事件链；按8.4迁移/隔离测试。普通关闭phaseoffset时不扫描local view/建tube、不检查Section有效性。

D. 隔离全构建、迁移测试、N=1验证。先对原单机导航与PhaseOffset观察模式通过再允许非零coordination测试。任何原导航回归不明失败，停止交付，不清用户工作树、不调安全参数绕过。

上述方法可按已有代码命名习惯在同一白名单类内实现等价私有helper；不得改变角色、所有权、失败语义或新增额外安全门槛。新类型不拥有可写w/delta，不增加新的mode选择配置维持两套生产后端。

### 8.7 最终审核约束与产物

Astra完整审核无新增硬blocking；下列三点必须落实并测试：cloud入口若发现来源/config改变，应先进入environment→map真实变更流程，不能持map再反取environment；map复制/重建仅限无存续capture/bundle或已退役生命周期，不能用复制重置validity绕过失效；发布从尚未持manager/adapter锁的外层取得environment锁，不能在旧深层publish函数中反向补锁。

Luna只按白名单用apply_patch编码，不写规格/架构文档，不新增白名单外文件。产物仅`.horizontal_section_refactor/m3c_20260910_01/executor/`；主会话使用同级supervisor目录，执行者不得并发写。源码与现有用户改动快照在supervisor/source_before.tgz。实现前后记录HEAD/branch/status/diff/源摘要；每个A–D子批给进度和变更清单，出现边界外依赖或冻结算法无法满足立即报告主会话，不自行扩大范围。

任何中间编译失配必须在同一配套实现中修齐，不能作为可运行版本交付；未经全构建/测试放行不启动ROS。禁止删除用户源码、reset/restore/clean、stage/commit或操作用户运行中的ROS进程。

## 9. 最终验收

原planner/map核心回归；全部已完成数学测试；新map有效占据/完整域/并发/empty cloud/reset/manual/static回归；真实FSM同步构建/命令50Hz延迟测试；重复静态cloud不饥饿；nonzero共同前缀切换/发布失败/reset/terminal取消；关闭phaseoffset零新开销；未知/部分覆盖诚实；无旧worker/map ID生产调用。

使用独立ROS master/namespace，只启动并回收本任务进程，不连接用户仿真。不直接run.sh。先N=1原导航到达，再可控横向输入、非零重规划；多机/论文效果另批，不能只凭单机通过声明论文规模实现。
