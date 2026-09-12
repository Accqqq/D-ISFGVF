# M3B：Section单拍推进与轻量提交

日期：2026-09-09。状态：R4冻结，主会话依据用户继续授权，经Astra medium对R3审核并合并数值边界修订后放行本批六文件。此前R1/R2是未执行草案，以下完整替代；Luna为唯一编码者，主会话冻结接口并验收。

## 1. 目的与边界

在现有PhaseOffsetRuntime中增加Section入口，继续使用原delta_和previous_final_port_，不新增第二状态owner。检查已选定的同一port在一个dt内的可行性，复用MatchedPort生成数学通道；发布前验证、发布成功后无失败提交。

不调用PortProjector::project或verify（verify内部仍会重投影），不调用TubeExecutionGuardV2，不生成reserve/未来回零证书，不加map/support/config IDs，不查询地图或几何builder。不修改core数学或controller/governor。

本批仍为运行端值接口接入，不是ROS主控制切换。manager仍拥有w，当前governor在matched后生成PositionCommand，最后实际命令/路径bundle的生产事务必须在后续manager/adapter成组切换验证。map baseline恢复和worker退出不在本批提前进行。

## 2. 六文件白名单

以W=/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws为根：

- src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_viability.h
- src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_viability.cpp
- src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_viability_test.cpp
- src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_runtime.h
- src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp
- src/swarm_planner/phase_offset/phase_offset_navigation/test/runtime_test.cpp

不再包括PortProjector/core文件。不改CMake/allocator/manager/adapter/map/launch或其他源码。主会话文档为本规格/M3B_Report，产物仅.horizontal_section_refactor/m3b_20260909_01/，Luna用executor，主会话用supervisor。旧源码及测试保留，不stage/commit，不操作ROS进程。

## 3. 单拍PWL检查

tube_viability.h增加TubeHeldStepResult：bool valid=false；double next_w/next_delta=0；size_t work_count=0；string reason。不含身份/证书、不能写控制状态。

TubeViability增加静态方法：
checkHeldStep(const TubeViabilityResult& preview, double current_w, double current_delta, double phase_rate, double u_delta, double dt, size_t max_work, TubeHeldStepResult& output)。

输入是M3A产生的有效Section PWL preview：SECTION_PWL、FEASIBLE、valid/feasible/rate_feasible/contraction_rate_feasible/current_delta_inside、section_profile非空，current_w/current_delta与preview.current_w/evaluated_delta精确匹配，policy.numericValid。借用profile及preview在调用期间不可变；不能把任意伪造knots当成认证事实。dt正有限、phase_rate非负且在policy.lower_nu/upper_nu内、abs(u_delta)<=preview.upper_u_delta、max_work正。

next_w=current_w+dt*phase_rate，next_delta=current_delta+dt*u_delta，严格有限且不倒退、域内，不clamp。这两个double结果是之后runtime提交的唯一期望端点；安全检查同时用现有区间helper包围精确乘加的终端位置，不能只按舍入后的next_w决定跨节点集合。可能触及的PWL节点和终端包围范围必须可保守判定，无法判定则失败，不外推、不clamp。内部节点时间和delta乘加使用向外包围，整体落在对应K内；终端固定dt，不反解时间，不以普通double单值越界判定替代区间检查。

同一给定port的仿射ZOH轨迹，只检查当前点、每个严格跨越的preview knot及终点；每个cell中delta与K边界之差仿射，因此不需要未来rollout/全图几何再验证。使用现有严格PWL envelope插值和数值helper；边界比较不使用任意增加的物理margin或1e-6宽容差。内部跨越时t=(knot_w-current_w)/phase_rate，delta_k=current_delta+t*u_delta；phase_rate=0时不除零，固定w检查delta两端。初/终点严格取0/dt，不用roundoff后的终点再反解t，避免终点舍入造成伪越界。

终端w包围区间必须完全在preview有效域内；终端delta包围区间必须在该w包围范围内各PWL边界的共同内区间内。可以检查w包围两端与其中可能触及的knots，因为边界分段仿射。内部knot时间包围与[0,dt]求交是合法的时间限制，不是状态clamp；交集为空则该knot本拍不会触及。实际提交next_w/next_delta仍是规定double结果，并也必须满足域/区间条件。不能扩大检查范围到未来若干拍。

新增单拍检查应在tube_viability.cpp复用现有非负乘加/区间数学以保守处理浮点误差，不复制投影算法；如涉及严格域/跨节点端点的数值约定无法保持，必须向主会话报告，不能现场扩成证书框架。

按实际遍历/检查节点计work；只扫描本拍相关区间，不扫描整条地图，不复制profile/整份preview。超预算/非法/越界返回false，output.valid=false，不给可提交的next状态；可保留work/reason。原legacy/V2/Section点查询行为不改。

本检查基于同一冻结K，不能声明未来滚动K或环境更新后的无限时域安全；后续adapter处理刷新适用性。几何I不重复再证，因为K已经是其内包络。

## 4. Runtime输入与准备

runtime.h包含已有phase_offset_allocator.h，以复用PhaseOffsetAllocatorBounds，不新增另一套limits。

RuntimeSectionPrepareInput字段：

- shared_ptr<const SectionTubeProfile> profile；
- const NormalPreviewResult* preview=nullptr（仅prepare借用）；
- phase_offset_core::MatchedPortInput matched（geometry、base_v_cmd、base_w_dot、final_port，唯一最终port）；
- phase_offset_core::PortCommand previous_u；
- PhaseOffsetAllocatorBounds limits；
- double dt=0；
- size_t max_work=0。

prepareSection(const RuntimeSectionPrepareInput&, RuntimeSectionPreparedStep&) 为只读/const方法。要求sectionConfigurationValid()、matched.geometry有效、delta==delta_、previous_u与previous_final_port_精确一致，preview.profile与profile.get相同，w/delta/path/frame与matched.geometry配套（沿用实际path/frame，不要地图ID）。不得改selected port或再调用allocator。

新增sectionConfigurationValid()const仅验证本入口实际使用的config_.tube.tracking_error_bound有限非负、config_.tube.minimum_reference_speed有限正、config_.manual.tangent_speed_min有限正。原configurationValid()/ValidManual行为不变，但新Section入口不受manual.amplitude/profile_period/preflight等无关旧参数门控；这些不是Section执行条件。测试须证明manual.amplitude=0造成旧configurationValid=false时，新Section仍可按有效的实际数值准备。

path/frame须非零且相等，不能两个默认零值充作绑定。保留既有真实物理条件：geometry.error有限且范数不超过config_.tube.tracking_error_bound；geometry.r_w有限且范数达到config_.tube.minimum_reference_speed；geometry.T有限，MatchedPort输出的T.dot(v_cmd)达到config_.manual.tangent_speed_min。这些均是当前旧执行链已使用的数值限制，不改默认值、不重新查询地图、不造tracking证书。真实初始acquisition如何回退是未来caller职责，本批不因拒绝返回永久HOLD模式。

标量limits为调用方从同一allocator事务提供的冻结值；校验有限、非负、上下界顺序及既有zoh_min/max/dt语义。检查final_port幅度、相对于previous_u的slew范围，phase_rate同时在limits与preview policy的允许范围内。slew使用与现有allocator一致的previous±rate*dt上下界，避免减法相消产生不同判定；不改变原参数或默认值。调用现有MatchedPort::evaluate一次，并使用其w_dot与delta_dot检查本拍。matched输入/输出不是ROS最终PositionCommand，本批不混淆二者。

调用checkHeldStep验证同一w/delta/最终port/dt；成功prepared保存matched输出、selected_u、next_w/next_delta、work_count。runtime不拥有w，不在prepare写delta/previous。

RuntimeSectionPreparedStep为可复制但数据私有的值类，仅PhaseOffsetRuntime可填入；公开const getters：valid(), nextW(), nextDelta(), selectedPort(), matchedOutput(), workCount(), reason()。内部另外保存producer Runtime地址、expected_revision、expected_delta、expected_previous_u。外部不能改next_delta/port绕过prepare。此值不持有preview深拷贝/地图证书，不额外复制几何profile；上层应持有原profile至发布事务结束。

## 5. 发布前验证与发布后提交

复用现有Runtime外部串行模型，不增加内部mutex/线程。新增private uint64_t section_state_revision_=0作为本地状态版本；不是map/profile身份，不传播到tube。每次真实runtime状态变更（reset、requestRecenter、所有legacy/V2/Section成功commit及legacy complete直接写delta/port）递增；它只使旧Section准备值失效，不改变旧V2/legacy接受规则或数值。溢出饱和到max，Section prepare拒绝继续，禁止回绕；旧接口不因这个新计数失败。

requestRecenter只有false→true才递增，重复同一意图不额外失效；reset代表新任务，哪怕数值相同仍递增。prepared只在创建它的Runtime对象生命周期内有效，caller不得销毁/原址重建或copy-assign替换Runtime后复用旧prepared。旧dryRun复制Runtime不修改原对象版本。

validateSectionBeforePublish(const RuntimeSectionPreparedStep&) const：检查private有效值来自this、版本一致且未饱和、expected_delta/previous与live精确相同，sectionConfigurationValid()有效。无几何/preview重算、不分配。

commitSectionNoFail(const RuntimeSectionPreparedStep&) noexcept：只在调用方已持有覆盖finalValidate→实际publish→commit的同一串行锁，且publish成功后调用。只写原delta_/previous_final_port_并递增本地版本；不改w、不分配、不重算。不将旧reserve用于新Section step。关于旧V2 marker，Section提交令v2_state_valid_=false即可，不在锁内销毁大型reserve对象；旧owner由后续切换/退役回收。此计数不替代manager任务generation/bundle适用性，后续caller负责。

可提供commitSection(prepared)便捷方法用于离线测试：先validate再no-fail，它本身不代表ROS已publish，头文件明确真实调用方须先publish成功。重复提交或reset后相同delta/port的ABA被版本拒绝。NoFail的调用前置条件是已验证且同锁连续，不能让它再次晚拒绝导致命令与状态分裂；外部绕过前置条件不属于支持用法。

不用另一层makeCommitToken/共享sealed副本，RuntimeSectionPreparedStep本身就是只读待提交值。不能先consume版本再publish；publish失败丢prepared，state不写。NoFail不clear巨大vector/reserve，也不增加诊断字符串gate。

## 6. 验收

保留M3A229适用回归和既有runtime两套宏目标测试。runtime Section测试放两个legacy/V2宏之外，两个目标都能运行，不引入CMake目标。

新增至少：
1. 单拍跨2个以上K cell，端点均可行但内部窄点越界必须拒绝；正常全拍通过；相关knot数/预算有界。验证当前点、内部knots、终点和零phase率的处理（如policy lower_nu>0，则零率应明确按policy拒绝而不是除零）。
2. PARTIAL实际valid_end内允许step，越出拒绝；真实末端无需外推。几何profile pointer及w/delta错配拒绝。
3. 非零delta与正/负u_delta、dt、幅度、slew、phase上下界，失败不改变Runtime；不snap delta=0。
4. 同一个final_port用于MatchedPort物理注入/w_dot/delta_dot及next_w/next_delta；残差与原MatchedPort一致，输入selected_u不被重投影。
5. prepare/validate不写状态，模拟publish失败不commit；publish成功后一次commit推进，重复commit、旧step、reset后ABA、另一Runtime实例step拒绝。用外部测试mutex演示锁序；本批不声称真实ROS锁闭包已接通。
6. 先Section准备后legacy/V2任一成功commit或requestRecenter使旧Section值失效；原接口原测试行为不变。epoch不可回绕，代码审核/可达边界测试不靠开放public setter。
7. 使用M3A真实preview+allocator输出接prepareSection的端到端离线用例，不能只手填valid标志。典型wide/narrow/跨节点/partial场景都记录失败原因。
8. sanitizer覆盖修改模块；diff --check、全部源码范围与指纹审计。Luna只apply_patch编码；主会话独立构建和验收。

## 7. 放行条件

Astra medium审核确认私有prepared、本地版本与同锁发布事务闭合；其唯一数值blocking已按R4终端和内部knot包围规则补齐。主会话现在明确放行本批，发现要改core/manager/adapter或重定义控制律时停止报告。完成本批后仍需manager/governor最终命令配套、真实地图同步读取及baseline恢复、旧worker/安装链退出；不以本批值测试冒充生产切换。
