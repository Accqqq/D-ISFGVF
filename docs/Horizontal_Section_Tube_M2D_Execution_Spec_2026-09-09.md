# M2D：多段路径接缝与横截面构建接入

日期：2026-09-09。状态：R1冻结，主会话依据用户继续授权，经Astra medium只读审核后放行Luna实施本批11文件。用户已明确Luna是唯一编码代理。

## 1. 本批结果与非目标

解除M2B/M2C整个path对象只能单segment的临时限制；支持真实旧prefix + quintic + mapped B样条的多段几何输入。保持原planner路径、相位映射、C2系数、安全距离和半宽。

必须同时解决实际path选段与builder的单元点值/导数界配套，不能只删除unsupported_multi_segment判断。此次不接地图同步、preview、manager/worker、不改变控制模式，不把几何接入完成当作非零offset handoff或ROS闭环通过。

## 2. 白名单

W=/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws，HEAD20d44c94d4165e35621cc30269e3fa0b2a5f10ff。

- src/swarm_planner/bspline_traj/src/continuous_phase_path.cpp
- src/swarm_planner/bspline_traj/test/continuous_phase_path_test.cpp
- src/swarm_planner/bspline_traj/include/bspline_race/continuous_phase_normal_frame.h
- src/swarm_planner/bspline_traj/src/continuous_phase_normal_frame.cpp
- src/swarm_planner/bspline_traj/test/continuous_phase_normal_frame_test.cpp
- src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_section_input.h
- src/swarm_planner/bspline_traj/src/integration/phase_offset_section_input.cpp
- src/swarm_planner/bspline_traj/test/phase_offset_section_input_test.cpp
- src/swarm_planner/phase_offset/phase_offset_navigation/include/phase_offset_navigation/section_tube.h
- src/swarm_planner/phase_offset/phase_offset_navigation/src/section_tube.cpp
- src/swarm_planner/phase_offset/phase_offset_navigation/test/section_tube_test.cpp

不改CMake/core/地图/manager/launch或其他文件。主会话可写本规格及M2D_Report；产物仅.horizontal_section_refactor/m2d_20260909_01/，executor与supervisor目录分离。保留已有全部修改，不stage/commit，不操作ROS。

## 3. 精确选段

ContinuousPhasePath::evaluate只将segment选择条件从带±kDomainEps改成精确w>=w0 && w<=w1；已有外层全路径clamp_to_domain语义不变。准确位于公共seam时保留原先左段优先，seam右侧哪怕nextafter也必须选右段；gap内返回false，不跨gap吸附。仅改选段，不改每段evaluator的point/导数公式。

cellBounds同样精确闭包含，跨seam区间拒绝，禁止证书容差把右区间解释为左段。旧tubeCellBoundsV2已精确，不改。

appendSegment不在本批改变接受规则，以免影响旧caller；新适配器自己拒绝任意gap/overlap。多段接缝运行语义由实际路径的这个共同查询入口统一，不能只在新adapter偷偷修选段而使execution看到另一条p(w)。

## 4. 单元配套查询

SectionBuildInput新增可选：

`std::function<bool(double w0, double w1, double w, SectionPathSample&)> cell_point_query;`

它提供闭cell所属同一几何段在w处的值，允许公共seam左右两个cell得到不同的浮点端点值。须同bounds_query的整段几何配套，且在cell内部与point_query表示的实际路径一致；公共seam可能有两侧数值差异，因此不能以单一w缓存跨侧复用。全局point_query仍保留为执行查询/兼容入口，仍为必需输入。

BuilderContext新增单元模式：有cell_point_query时以(w0,w1,w)为cache key查询；无此callback时完整保留旧w-key cache及point_query行为。只修改evaluateCell三次start/end/midpoint调用的上下文参数，budget、平面算法、正则性和PWL组装不改。callback失败不得转fallback全局查询，以免混入错段。缓存规模仍由max_cells限定。

左右cell各自完整验证自己的闭区间，包括两侧seam值；相邻节点仍按原assembleProfile求区间交集。因此实际执行的seam左值及其右侧极限都在共同节点允许范围的几何检查内，不靠加大clearance或端点误差容差替代双侧检查。

## 5. 复用法向公式

ContinuousPhaseNormalFrame新增public static `computeGeometry(const ContinuousPhasePathState&, Eigen::Vector3d& tangent, Eigen::Vector3d& tangent_w, Eigen::Vector3d& normal, Eigen::Vector3d& normal_w)`。

把representedNormalAt取得state之后的现有算术原样提取到此函数，保持运算顺序、q阈值和finite/正交检查。representedNormalAt调用它，新cell callback也调用它，不在adapter重新抄一份normal公式。不改normal query的身份字段或certifyCell。

## 6. 多段适配契约

- 校验相关请求范围完全连续覆盖，所有相关segment域有限且正长，边界精确相接；gap/overlap拒绝，不以epsilon填补。
- 对请求内或恰在请求边界的公共seam，双侧Segment::evaluate查询p、dp_dw、d2p_dw2，finite有效。沿用原frontend的数值C2接受上限：三者差的范数之和<=1e-6；大跳变拒绝，不以样点相近声明精确C2。这个门槛只拒绝不合适的路径接续，不用于几何安全放宽；几何由左右各自检查。
- 全局point_query仍走path->evaluate和现有normal frame，不能重拟合路径。
- bounds_query找到精确包含闭cell的唯一segment，再使用path->cellBounds（精确选段已修复），继续M2C分量优先/旧scalar回退规则。
  不另比较certificate.segment_identity：唯一精确包含检查和共同path查询入口已经保证选段一致，不增加重复身份门控。现有PathCellGeometryCertificate仅作为整段数学界数据来源，不把其身份传入新profile。
- cell_point_query同样以cell选择segment，w必须在cell闭范围内，再直接调用该已存Segment::evaluate(w)。用computeGeometry填N/N_w；有限校验后赋值，失败default。不复制Segment/Evaluator，不重建path，不创建另一个phase owner。
- 真segment的w0/w1合并为structural_breakpoints，保留请求范围内边界，排序去重；不枚举旧1024内部proof节点。cell跨多个segment明确失败，由初始分区/细分保证正确分段。
- 仅一个segment时不必设置cell_point_query，保留旧单段缓存性能；两种模式几何语义一致。
- 请求若从公共seam开始，全局实际值可能属于前段，按第8节补充单点检查，不扩大请求范围、不忽略前段闭端点。

注意：已有C2数值误差<=1e-6不等于活动参考严格C2，后续运行handoff仍需原连续性/当前状态检查。本批只证明所给piecewise参考带的几何内包含和局部regularity，不宣称控制跨缝已验收。

## 7. 验收

1. path精确seam仍左值；nextafter(seam,+inf)及seam+5e-9取右段；gap中点不吸附；跨seam cellBounds拒绝，右cell取得右证书。原单段point/parameterization/C2回归不变。
2. computeGeometry与旧normal query固定点/升降/近竖直退化行为一致，不改变阈值和算术公式。
3. builder synthetic双段在seam使用可区别的左右值，point缓存不可混淆；cell callback失败不回退，预算/前缀语义保留。原无callback全部28测试通过。
4. 真实makeQuinticHermite+makeMappedBspline+appendSlice生成连续多段path，在默认clearance=.4/halfwidth1完整构建；逐输出cell检查实际path与原AABB距离、域、参考速度，并检查seam双侧值。包含升降及水平例。
5. 1e-7级数值接缝（在既有门槛内）双侧几何安全才允许；一侧碰撞/另一侧安全不得COMPLETE。不能用误差容差把危险侧忽略。
6. 大接缝跳变、gap、overlap、坏point/bounds/缺normal能力明确失败；不虚构ZERO_ONLY。
7. 单segment旧场景profile/宽度一致；M2C平面零厚度修复保留。新请求起终点恰在seam、slice、短prefix、budget终止有明确测试。
8. 所有175原测试语义保留；只将原“一律拒绝多段”的临时断言替换成对应合理多段成功与非法多段拒绝断言。新path/normal/adapter/builder跑ASan/UBSan。记录幅度/覆盖/point查询次数，不把缩短返回范围当速度改进。

## 8. R1冻结：实际首末端点覆盖

有cell_point_query时，在assembleProfile成功后、最终状态分类前，处理已返回profile的valid_start及valid_end（含PARTIAL真实尾端，不能拿请求w_end代替）。各自取global point_query(w)与accepted首/尾cell的cell_point_query(w0,w1,w)，验证两者有限和现有样本/法向规则。按全部5个向量p/p_w/p_ww/N/N_w逐分量精确相等比较（+0/-0可相等），不能用1e-6误差阈值跳过几何检查。

相同时不再做几何；不同时对global样本计算单点安全截面：用已有domain planes、closestSeedPointToBox(global.p,global.p)、原clearance分离平面；复用平面区间求交，span=0且二阶bounds=0，不调用要求正cell长度/正速度下界的整cell校验。再applyRegularity(global.p_w,global.N_w,minimum_reference_speed)求零连通单点速度区间，与对应首/末knot区间求交。只缩这两个节点，其他已验证cell和内部seam共享交集不改。两侧公共内部seam各有闭cell验证，故不对每个内部节点重复全局检查。

额外global查询按实际调用增加point_queries，额外逐障碍检查计入同max_obstacle_checks。无cell_point_query时不执行新首末逻辑，原单段计数/缓存/结果保持。

global/cell查询失败、样本退化、plane/regularity失败、交集不含0或额外预算耗尽时，立即返回明确失败（理由endpoint_*），usable=false、complete=false、knots清空、valid_start/valid_end重置，保留计数；不得进入“有accepted就PARTIAL可用”的收尾分支。零宽也必须通过单点速度和几何条件，不伪造ZERO_ONLY。成功后照原profile最终状态分类，可因节点真实收缩得到ZERO_ONLY。

新增验收：请求恰seam起点，global左值与右端点近似但不相等且左值碰撞/退化，必须失败；可行微差则成功且首节点同时满足两侧；PARTIAL尾端使用valid_end；首末不一致补查预算耗尽不可用；只p_w/N_w不同的位置相同样本不能漏检。原正常内部seam不得额外逐障碍重复检查。

## 9. 审核及交付

Astra medium只读审核结论：按第8节补齐后，本批几何安全契约闭合，无新增关键blocking；不代表活动参考控制跨缝已验收。主会话采纳其完整sample精确比较及失败清空/真实valid_end要求，不增加ID/独立worker/重优化。

实现前后记录branch/HEAD/status/diff及source摘要，保留M2C改动。编译测试、sanitizer、依赖检查、diff --check、白名单核对与报告由主会话独立完成。执行者发现必须改11文件外或改变冻结数学时停止报告。完成本批后停止，不自动进入地图/运行切换。
