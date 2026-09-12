# M3A：现有Preview/Allocator消费新SectionProfile

日期：2026-09-09。状态：R1冻结，主会话依据用户继续授权，经Astra medium只读审核后放行Luna本批6文件。用户授权继续运行链路成组切换，Luna是唯一编码代理。

## 1. 成组切换顺序

M2D只完成多段几何输入。现有生产preview/allocator仍硬依赖V2 profile和map/support IDs，因此先迁移这两个真实消费者，再迁manager/runtime及地图，最后退出旧worker/安装链。不得通过假V2 profile或固定ID=1绕过旧门控。

本批增加新SectionProfile直接入口，复用同一套PWL preview与分配数学，旧入口暂保留以保证成组切换前工程可构建。不是长期第二套算法，不能复制整个preview/allocator实现。本批单独完成仍不代表主控制已切换，也不恢复地图或启动ROS。

后续M3B必须落实：原runtime继续拥有delta/previous_final_port、manager继续拥有w；FSM规划内同步构建、控制无重几何；原publish-success后no-fail commit保持；map baseline恢复与旧消费者同时替换。具体文件/接口待本批数据契约落实后冻结，不以本文件授权未来任意改动。

## 2. 本批白名单

W=/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws。

- src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/tube_viability.h
- src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_viability.cpp
- src/swarm_planner/phase_offset/phase_offset_navigation/test/tube_viability_test.cpp
- src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/phase_offset_allocator.h
- src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_allocator.cpp
- src/swarm_planner/phase_offset/phase_offset_navigation/test/phase_offset_allocator_test.cpp

主会话可写本规格和M3A_Report，产物仅.horizontal_section_refactor/m3a_20260909_01/；Luna用executor子目录，主会话用supervisor。无新CMake目标、不修改core/math、runtime/adapter/manager/map/launch或其他测试文件，不stage/commit，不连接ROS。

## 3. 公共新契约

include section_tube.h，TubeViability增加 `evaluate(const SectionTubeProfile&, const TubeViabilityInput&, TubeViabilityResult&)` overload。

TubeViabilityProofKind新增SECTION_PWL枚举，不是新的证书体系，仅选择既有严格PWL数学查询语义。保留LEGACY/V2_LIVE_PWL原行为。

TubeViabilityResult增加 `const SectionTubeProfile* section_profile = nullptr; double evaluated_delta = 0;`。这是借用同一immutable profile的对象关联与真实查询状态，不是新序号；调用者必须持有profile直至所有preview/allocator查询完成，不得在mutable alias修改。SectionProfile自己没有path/frame身份，因此新入口使用input.path_revision/frame_revision填已有provenance相应字段（非零绑定），其余map/source/tube/profile/policy身份字段保持默认，不填假值。

NormalPreviewProductionPolicy增加 `numericValid()`，仅验证现有horizon/spacing/lower_nu/upper_nu/b_tight/b_open数值和immutable；原valid()仍保留旧ID要求供旧入口使用。新入口不要求policy_revision/configuration_identity/configuration_id，不改变任何数值参数。

TubeViabilityInput增加 `bool section_reaches_path_end = false;`。默认false保持完整horizon必须覆盖；true仅供未来adapter已确认profile.valid_end等于真实path终点时设置，并且代码必须要求profile.complete且状态COMPLETE/ZERO_ONLY，才允许horizon截到valid_end并标preview_truncated_after。PARTIAL即使设置true也拒绝，不能用失败终点冒充真实终点。当前w==有效终点时返回明确无正长preview，不伪造FEASIBLE；后续原terminal逻辑处理，不在此新增控制模式。

PhaseOffsetAllocatorInput增加 `const SectionTubeProfile* expected_section_profile = nullptr;`。当preview.proof_kind==SECTION_PWL时，要求指针与preview.section_profile相同非空，geometry.w==preview.current_w且geometry.delta==preview.evaluated_delta（已核对geometry_types.h存在这两个字段），existing expected_path_revision/frame_revision非零且等于preview及geometry对应字段。新分支不检查map/source/tube/profile revision与obstacle_contract_id；这些旧字段可为0/空。非section分支行为完全不变。

指针与path/frame字段只防止消费错profile/错状态，不新增生命周期ID。现有数学geometry有限性、幅度、slew、实际phase rate与ZOH duration检查保留。新代码不得读取ROS或地图。

## 4. 共用数学核

把当前EvaluateV2中通过profile/provenance校验后的merged-node/PWL递推、slopes、beta、rate查询部分提取为文件内部通用数学核心，由V2与Section两个入口调用。新Section不构造临时TubeProfileV2，不伪造cells/proofs/config/map字段。

可用内部轻量source视图持有有效范围、节点w列表的只读访问及evaluate(w,lower,upper)callback；不能复制完整V2对象或每tick重新构建几何。BuildMergedNodesV2可改为模板/内部view以同时支持两种真实knots，保持旧V2算术结果和工作计数语义。允许原函数名保留以减小改动，不把函数名V2误当新profile身份。

Section入口要求profile.usable、>=2有限严格递增knots、端点与valid_start/valid_end精确一致、所有lower<=0<=upper、无域外插值，COMPLETE/ZERO_ONLY/PARTIAL状态与布尔语义自洽。PARTIAL可供preview使用但只能在其真实覆盖内，不要求complete=true。预算max_work>0；验证、节点扫描、递推有界，不绕过预算。成功保留section_profile和evaluated_delta，失败清空不可用数据，不把unknown伪造zero-only。

所有真实profile knots都与preview网格/起终点合并，不回退legacy均匀采样丢失窄点。反向递推的横向可达宽度仍按upper_u_delta*dw/upper_nu；PWL内包络、双侧坡度、beta和rateIntervalAt的边界速率限制均复用原实现。现有带phase_rate的查询只验证给定值在policy范围内，实际边界限制仍按整个lower_nu/upper_nu范围计算，本批保持它，不声称已按单个固定rate收紧或完成整tick检查。queryEnvelope/queryRateInterval对SECTION_PWL使用同一严格PWL数值核，但不要求旧V2身份。不得另加free-ball、几何再认证、reserve rollout或控制模式。

Allocator的解析投影、标量限制、slew和最终selected_u所有字段不改变，唯一差异是新section入口的关联验证。不能以单元测试通过为理由调低minimum_reference_speed或放宽不等式。

## 5. 验收

1. 手工PWL渐缩/渐扩、非均匀knots、普通采样间的窄点：新入口保留全部几何节点，K始终在I内，解析可达范围符合原递推。
2. 相同几何/数值policy的合法V2输入与Section输入：merged knots、K、beta、rate interval及可比work结果一致（身份字段除外），不得用新算法冒充复用。
3. Section输入无map/source/tube/config IDs（全0/空）仍可得到正常preview并分配；错profile指针、错w/delta、错path/frame、无效几何/数值明确拒绝。
4. PARTIAL覆盖不足完整preview时明确不可行；覆盖足够可用；真实terminal标志才允许截短。标志不代表本批已验证生产者真实性，头文件说明未来caller前置条件。
5. 缺policy数值、坏knots、预算不足、NaN/颠倒域拒绝，不跨gap/外推；invalid preview不会把默认beta解释成窄通道。
6. 同一个合法geometry/g_des/previous_u/dt/bounds，旧preview与Section preview的分配结果selected_u一致；保留原38项allocator数学/旧身份断言，不删除旧入口回归。
7. 新入口在不同查询w、精确内部节点的右侧坡度及域边界使用严格PWL查询，不落回legacy容差分支。这些是点查询，不代表有限tick跨节点全程验证；完整runtime ZOH推进/commit留M3B，不能宣称已验证主控制。
8. catkin、两修改测试suite、原math/matched/allocator及M2D185适用回归，新增接口ASan/UBSan，diff --check、白名单/指纹核对。strict数值核的IEEE环境前置条件沿用原测试环境，不扩大形式化保证。

## 6. R1审核后冻结约束

Astra medium确认共用核心可在6文件内实现，主会话采纳其两项关键修订：点查询不冒充整tick ZOH；terminal flag必须完整profile且非PARTIAL。geometry.w/delta字段已核对。

SECTION分派须覆盖QueryEnvelopeV2Strict、QueryRateIntervalV2Strict和公开查询入口，rate查询的policy检查对SECTION改numericValid，旧V2仍valid。allocator身份检查的allocate前置、GeometryMatchesPreview和失败分类ProvenanceMatchesInput三处都按SECTION一致分派，不能只改中间一处。

caller须共同持有profile及构建时path/frame，不能事后给旧profile贴新revision；借用指针及revision检查只防错对象/错状态，不是证明profile来自某路径。不新增证明链。Section的ZERO_ONLY要求全knots为[0,0]；COMPLETE/ZERO_ONLY均complete，PARTIAL usable且不complete。不可分配的失败结果不得留下可误用的借用关联；诊断标志及reason诚实。

本批已冻结放行；发现需改变原数学/白名单外接口则停止报告，不自行扩展。实现前后记录branch/HEAD/status/diff/source指纹，主会话独立复验后交付，并在本批停止功能修改。
