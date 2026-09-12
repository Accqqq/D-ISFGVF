# M2C：真实路径逐坐标加速度界

日期：2026-09-09。主会话依据用户继续执行计划的授权冻结本有界批次。Luna编码，主会话监督独立验收。不是地图或运行切换授权。

## 1. 目标

修复M2B已明确记录的范数转分量局限：固定高度弯曲路径不应因为将全向加速度范数填入z界而拒绝零厚度水平参考域。复用现有路径的数学界计算，输出逐坐标abs(p_ww)整段上界，优先供section适配使用。

不改变任何实际路径点值、相位映射、五次C2系数、segment选择、normal query、旧scalar界计算/接受规则。多段seam修复与地图/worker切换在后续成组规格完成，本批仍拒绝多segment owner。不得声称最终路径接入已完成。

## 2. 白名单

W=/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws；HEAD20d44c94d4165e35621cc30269e3fa0b2a5f10ff。

1. src/swarm_planner/phase_offset/phase_offset_core/include/phase_offset_core/path_state.h：仅在legacy PathCellGeometryCertificate增加下述可选数据字段/注释，其他类型和验证规则不改。
2. src/swarm_planner/bspline_traj/src/continuous_phase_path.cpp：仅界生产辅助函数和对应字段传播，不改point evaluators、path选择、参数化。
3. src/swarm_planner/bspline_traj/test/continuous_phase_path_test.cpp：新增界测试，保留全部原测试。
4. src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_section_input.h：更新分量能力说明。
5. src/swarm_planner/bspline_traj/src/integration/phase_offset_section_input.cpp：优先使用分量字段，其他逻辑不变。
6. src/swarm_planner/bspline_traj/test/phase_offset_section_input_test.cpp：更新已经被修复的平面局限测试并补无能力回退/坏字段测试，原语义断言保留。

主会话文档为本规格及Horizontal_Section_Tube_M2C_Report_2026-09-09.md；产物仅.horizontal_section_refactor/m2c_20260909_01/（executor子目录由Luna使用）。不改CMake、map、manager、M2 builder、launch、其他core数学，不stage/commit或操作ROS进程。

## 3. 数据契约

PathCellGeometryCertificate增加 `Eigen::Vector3d sup_abs_p_ww = Eigen::Vector3d::Zero(); bool component_acceleration_bound_complete = false;`。默认false表示未提供，默认零不表示零加速度。旧complete checker不要求新字段，不改变旧生产消费者语义。

DifferentialBounds内部携带相同分量数据和能力位。MakeCertificate复制可选字段；新界失败时清空能力位但保留原scalar产物，不改变旧路径能力。所有新增分量需有限、非负。

section adapter：能力true时检查分量有限非负，再使用componentwise min(sup_abs_p_ww, (Axy,Axy,A))，两个都是整段上界，取min仍是上界。能力true但非法字段时bounds callback失败且输出default，不能悄悄退回。能力false时保留M2B原范数回退。N_ww界算法不变，不加新身份。

## 4. 计算方法

仅计算必要的p_ww分量，不先增加无人消费的p_www分量或一套V3证书。

复用现有多项式区间限制/幂基转Bernstein或B样条局部控制点收集。新helper对每个坐标的已收集控制点取max(abs(component))，沿用现有legacy UpperBound数值膨胀约定；明确这不是新的严格区间证明系统。不得用离散路径采样估计上界。

- Quintic：已存五次系数求二阶导的幂系数，经现有RestrictPowerVector/VectorPowerToBernstein得到查询区间控制点，逐坐标取绝对上界再除h_segment²。除法/乘法新合成用现有outwardUpperRatio/Product或保守包围的正分母，避免h²先向上舍入导致商界下偏；不可改原scalar公式。源二阶幂系数该坐标全为0时保留精确0；其他情形不能凭采样恒定判零。32子区间退路对每坐标取max，所有子区间能力均true才置aggregate能力true。
- Mapped B-spline：复用实际所选enclosed_t0/t1；对dp_dt和d2p_dt2收集同一时间范围的控制点分量界D1_i、D2_i。已有M1=max|t_w|、M2=max|t_ww|，新界 `D2_i*M1² + D1_i*M2`，非负乘法/加法向上包围并检查溢出。不得遗漏第二项。原导数控制点分量全0时该项精确0；固定高度z的两项均0，保持精确0。
- Linear fallback：同一t0/t1的D2_i*|t_w|²，t_ww=0；覆盖已有linear fallback入口，不以“默认走弧长”跳过。

对零判定：仅信已表示多项式/导数控制点结构性全零；极小非零不得阈值归零。原控点全等z使导数控点z精确零是结构依据；不改点值来制造该条件。数值溢出/无法提供界只使可选能力false。单slice复用原immutable evaluator携带能力。

## 5. 验收

所有原path20/normal6/M1A23/M2 28/core28/matched6/allocator38及M2B非被修复局限的断言保留通过。

新增/更新测试至少包括：

1. 固定高度mapped B样条、平面弯曲quintic：能力true且z界精确0，水平分量非零。
2. 升降mapped/quintic：逐cell逐分量密集p_ww oracle包含；有z加速度时不得错误置0。
3. linear fallback与appendSlice传递能力；32子分割路径需验证分量聚合不丢失。
4. 极小非零坐标导数不被置0，有限数/溢出能力失败语义诚实。
5. 旧点值不变：同一保存的实际path evaluator查询与新增界查询前后p/dp/d2一致；禁止“换模型使界成立”。原参数化、C2与normal回归必须通过。
6. M2B两个原平面局限用例改成已修复用例：零厚度z=1域下构建COMPLETE/usable，默认clearance=.4、half_width=1，原始域不调宽；逐输出cell独立检查参考点域/原盒距离/正则性，不能仅assertComplete。
7. 构造只提供旧scalar证书的合法synthetic path，能力false时适配回退仍保守；能力true但NaN/负分量时callback明确失败/default。
8. 新平面/升降例报告有效覆盖和宽度，不宣称最大截面。多段拒绝回归保留，不能以此批验收掩盖多段接入未完成。

测试容差不得抹掉minimum_reference_speed门槛；采样只作独立交叉验证，不作为连续证明来源。sanitizer覆盖新增逻辑，新增adapter/test严格warning，旧依赖警告不能靠改原数学消除。

## 6. 放行与停止

执行前后保存branch/HEAD/status/diff/源指纹；编译测试用隔离目录，不写原build/devel。主会话审核后冻结源码，独立构建+测试+ASan/UBSan、diff --check、范围与指纹核对。

主会话自审：此批只增加被新section消费的数学值，旧checker不受新能力位门控；未知能力不冒充0；mapped链式法则保留t_ww项；planar修复不调地图厚度或净空。发现需要改point evaluator、旧scalar安全语义或白名单外文件时停止报告，由主会话修规格。
