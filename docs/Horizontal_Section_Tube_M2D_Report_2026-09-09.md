# M2D：多段接缝与截面构建监督记录

日期：2026-09-09。状态：本有界批次实现与主会话独立验收完成。依据M2D R1冻结规格，Luna是唯一编码者，主会话负责规格、监督和独立验证。Astra medium只读审核，不改文件。

## 1. 本批范围

多段真实路径到新section builder的几何接入，不是主控制切换。原map/worker/安装链、manager、preview仍未迁移。只在规格11文件内修改，原安全距离/半宽/控制参数和C2系数不变。

- path evaluate/cellBounds选段精确闭包含，seam本点左优先，右侧nextafter不再被左段1e-8容差捕获。gap不吸附。
- normal公式提取computeGeometry供普通查询和cell查询复用，运算顺序/阈值不变。
- SectionBuildInput可选cell_point_query，cache按(w0,w1,w)隔离两侧端点；无callback保留旧单段w缓存。
- 多段相关域必须连续无gap/overlap，数值C2沿用原frontend 1e-6阈值。两侧各自闭cell验证、共享PWL节点取交，不以接缝误差容差放宽几何。
- 实际首/valid_end端点若与cell样本不同，复用零span单点平面/regularity检查并只缩边界节点；失败清空不可用，不能落入PARTIAL可用收尾。

## 2. 审核与返修

Astra确认首末单点补查闭合，关键要求为全部5向量精确比较、global样本校验、预算计费、实际valid_end以及失败立即清空返回。生产代码首轮cache/端点/bridge审核无关键blocking。

主会话要求adapter finitePathState加入vel有限性，与global path实际拒绝规则一致；去掉重复segment_identity比较，精确唯一包含+共同path入口已保证点/界配套，不增加身份门控。

原composite测试位移与端点速度不匹配，Luna测得连接中段水平速度约0.00695，旧界未能完成构建。这个值为正，不能称速度为零、实际碰撞或已经证明逆向。主会话要求保留失败证据，另构造位移/速度配套的正测，不调预算或安全参数。法向变化fixture必须有canonical p_ww和N_w配套，且global/cell内部一致，仅在允许的端点存在差异。

## 3. 独立验证（中间结果）

产物位于.horizontal_section_refactor/m2d_20260909_01/。八包catkin配置通过，未写原build/devel或操作ROS进程。主会话生产目标编译通过；path/normal/builder以-Wall -Wextra -Werror通过sanitizer对象编译。

中间普通测试通过：path25、normal7、adapter25、旧builder28、Geometry28、Matched6、Allocator38、LocalObstacleView23，共180项。新增builder端点预算/部分尾端/cache失败测试仍在补齐，因此不把这些结果当最终验收。

## 4. 未完成与停止条件

本批最终测试、sanitizer、源码指纹核对均已完成，在本批范围停止功能改动。新tube仍未接preview/主控制，旧worker/安装链未退出。后续需要真实地图同步/已知域与生产消费者成组切换，不能把当前几何接入误报为整体完成。

## 5. 最终独立验收

普通构建：Path25、Normal7、SectionInput26、SectionTube32、Geometry28、Matched6、Allocator38、LocalObstacleView23，共185项通过。前四组另以ASan/UBSan运行共90项通过，detect_leaks与halt_on_error启用；不是新增90项不同测试。新源码及tests以-Wall -Wextra -Werror编译，历史path测试沿用gnu++14的hexfloat支持。原依赖警告未通过修改旧数学来消除。

主要最终结果：

- 使用真实appendSlice旧前缀、makeQuinticHermite和makeMappedBspline构成3段路径；水平及升降两例均以默认clearance=.4、half_width=1完整覆盖w[0,4]，并通过实际path与seam双侧几何/参考速度oracle。这是固定输入测试，不是planner实飞回放。
- 1e-7数值C2差异在旧1e-6门槛内可接入，左右端点单独验证；请求从seam开始且实际左值越出有效域时，即使右值可行也明确拒绝并清空结果。没有收紧C2门槛来回避问题。
- cell/global内部一致，缓存两侧点各自保留；callback失败不回退global错段查询。
- 额外端点障碍预算耗尽返回不可用BUDGET_EXCEEDED，不误标可用PARTIAL。
- partial尾端检查访问相位恰为[0,.5]而非请求w_end=1；尾节点确实因N_w约束收缩，测试不再仅断言valid_end。
- 原单段与M2C零厚度水平修复保持，精确seam仍左优先而nextafter右侧选择右段，gap不吸附。

XML：path_final.xml、normal_final.xml、adapter_final.xml、builder_final.xml；san文件为path_sanitized_reviewed.xml、normal_sanitized_reviewed.xml、adapter_sanitized_final.xml、builder_sanitized_final.xml。四份既有回归XML也在本批目录。

## 6. 最终范围审计

进入时src摘要：e118af5ace61ee09dd42c9ebb285de0b79560afdf043b7006038c51b7486f188；冻结后src摘要：27751547465a2b7c8b76c31c16e258bc93edd6c51708aa18c6dbfcaca7ce8b2e。

将11白名单摘要替换为进入值后，恢复的整体摘要与进入值完全一致；证据为supervisor_whitelist_before.txt、supervisor_whitelist_final.sha256、supervisor_normalized_after.sha256。其余rg可见src内容未改变。git diff --check通过；HEAD仍20d44c94d4165e35621cc30269e3fa0b2a5f10ff，branch仍pro_review_current_20260903，未stage/commit，原用户修改保留。

本批没有新增map/support ID、worker、安装证书或主控制gate。既有PathCellGeometryCertificate仅作为数学界来源；曾出现的冗余segment_identity比较已删除。section_tube中的support是障碍盒支持函数，不是地图support身份。

限制：几何安全不等于严格活动参考C2或离散控制handoff已验证；既有1e-6数值接续误差未被当作精确连续证明。没有测新旧全链同输入速度差异，没有运行ROS或宣称论文多机效果完成。
