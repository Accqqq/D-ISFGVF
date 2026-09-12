# M2B：真实路径与局部视图值适配执行规格

日期：2026-09-09

状态：主会话依据用户“那接着做这个吧”的继续接入授权冻结此有限批次；执行者不得自动进入地图恢复或运行时切换。用户最近对baseline差异的询问不构成回退相位参数化授权。

## 1. 本批交付和边界

将现有不可变 ContinuousPhasePath、M1A LocalObstacleView 转成 M2 SectionBuildInput，建立可复用且离线验收的薄接口。使用真实路径实现构造B样条、C2及共同前缀用例，不声称这些固定输入是实际飞行回放。

不修改planner、相位映射、C2公式、SDFMap、manager、旧worker、preview、控制状态和安全参数。不把新profile塞入V2身份接口。实际地图读取同步/已知域生产者、运行时切换仍需成组规格。本批的地图适配仅接受已经完整捕获的M1A值，不伪装成已经连接SDFMap。

## 2. 白名单

根：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`。

- 新增 `src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_section_input.h`
- 新增 `src/swarm_planner/bspline_traj/src/integration/phase_offset_section_input.cpp`
- 新增 `src/swarm_planner/bspline_traj/test/phase_offset_section_input_test.cpp`
- 修改 `src/swarm_planner/bspline_traj/CMakeLists.txt`：仅增加上述库和gtest，链接existing continuous_phase_path、phase_offset_navigation及测试所需plan_env，不改其他目标。

主会话文档：本规格和 `docs/Horizontal_Section_Tube_M2B_Report_2026-09-09.md`。产物仅 `.horizontal_section_refactor/m2b_20260909_01/`。保留已有全部改动，不stage/commit、建分支或连接ROS。HEAD预期20d44c94d4165e35621cc30269e3fa0b2a5f10ff，branch pro_review_current_20260903。

## 3. 冻结接口

namespace FLAG_Race；头文件依赖path、section_tube、local_obstacle_view及Eigen/STL，不直接包含SDFMap/ROS。

`bool makeSectionBuildInput(const std::shared_ptr<const ContinuousPhasePath>& path, double w_start, double w_end, const plan_env::LocalObstacleView& view, phase_offset_navigation::SectionBuildInput& output, std::string& reason);`

成功表示输入适配成功，不代表tube安全或构建完整。失败重置output，reason非空。要求view.status==VALID，frame_id非空、分辨率有效、盒有限且min<=max；occupied_voxels与obstacles.size一致，复制全部障碍盒，不裁剪/再膨胀/过滤。不比较map/support ID。路径与view同世界坐标系是调用方前置条件，本接口不做TF转换，也不凭非空frame证明相同坐标系。

路径非空，范围有限且严格在startW/endW内，w_end>w_start。校验请求范围被segment连续覆盖，gap/overlap明确拒绝；包含真实segment边界并排序去重，仅保留请求范围内断点。禁止枚举certificateBreakpointsV2的内部证明分区。

callbacks按值捕获shared_ptr<const path>，不得捕获栈引用。调用方不得通过另一个mutable alias修改该path；不复制重采样路径。point_query对请求域外严格失败，使用path->evaluate(w,state,false)和现有ContinuousPhaseNormalFrame::query取得同一路径的p/p_w/p_ww/N/N_w。不创建额外持久身份。每次失败重置sample。

bounds_query只接受请求域内、完整位于一个真实segment闭区间内的非零区间（w0>=segment.w0且w1<=segment.w1，允许等于端点），先自行检查精确边界，不依赖旧cellBounds的容差来跨越接缝。使用path->cellBounds得到整段数学界；不调用tubeCellBoundsV2、不传递certificate身份到新接口。无整段界则明确失败，不由采样拟合补齐。

R2最终接缝审核修订（覆盖本规格中所有多段正测要求）：由于ContinuousPhasePath::evaluate() 在 seam 附近使用 1e-8 容差选段，本批只接受 path->segments().size()==1 的路径对象，其他情况立即重置output并返回明确的unsupported multi-segment原因。即使请求落在多段对象的某个单段内部也拒绝，不裁剪/重新创建路径owner来绕过实际执行。唯一segment内按闭区间精确检查请求和callback域，仍调用原path->evaluate和path->cellBounds。删除不可达的双侧seam一致性检查，不加入未来兼容框架。真实单段B样条、单段五次Hermite、单段来源appendSlice产生的单段对象作为正测；多段共同前缀和C2拼接（包括精确相接和1e-7不一致）均作为拒绝负测。此批不宣称多段C2或共同前缀执行接通，生产切换前必须另行冻结接缝修复规格。

## 4. 导数界与明确限制

设 q=inf_horizontal_p_w_norm，v=inf_p_w_norm，Axy=sup_horizontal_p_ww_norm，A=sup_p_ww_norm，J=sup_p_www_norm。

首批使用可信范数界的分量上界：

- horizontal_speed_lower=q，path_speed_lower=v。
- abs_p_ww=(Axy,Axy,A)。
- abs_N_ww=(B,B,0)，B=J/q+3(Axy/q)^2。

N永远水平，因此N_ww,z=0；p_z不一定恒定，不得照抄测速样例将p_ww,z设0。使用existing outwardUpperRatio/Product及有限nextafter上包围求和，保留真正零值，拒绝溢出/无效界，不启用fast-math。Axy==0证明整段水平速度向量恒定，此时N_ww精确为0，不因纯竖直jerk削减横向容量。

这是安全但可能偏松的适配，不能宣称完成逐坐标紧界。尤其水平弯曲但固定高度的路径，A用于z界可能导致零厚度reference_domain下构建不成功；必须有测试和报告说明，禁止调宽已知域/调小clearance掩盖。后续生产切换前需在真实路径界生产者中提供可信逐坐标界，以恢复此类平面域能力。本批不修改数学生产者、不以抽查z相同推导z导数恒零。

## 5. 验收

至少覆盖：

1. 空path、非法范围、非VALID视图、坏盒、障碍计数不一致，失败output无可用回调。
2. M1A真实网格生成的视图经过适配保留全部盒/域，不再膨胀；unknown不得变available。
3. 适配后销毁调用方变量/修改原view不影响输出；不允许测试修改已捕获路径。
4. 真实mapped B样条（含升降）点值与实际path/normal查询一致；w域外不外推。
5. 单段五次Hermite及单段来源appendSlice正测，严格域检查；任何多segment路径对象拒绝，包括精确C2、坏接缝、gap/overlap以及仅请求多段对象中单段的情况。
6. 无bounds的合法点路径：输入可适配，bounds失败，builder不伪造ZERO_ONLY。
7. 固定种子升降B样条/C2，整段界与密集采样p_ww比较；N_ww用独立解析路径或数值差分交叉检查（不是证明来源）。
8. 水平导数恒定、竖直高阶非零的C2：N_ww界精确0。
9. 在完整三维已知盒内构建实际B样条及C2 tube，非空完整结果逐输出cell采样参考点，独立检查对原障碍AABB的距离、域、参考速度，不复用plane逻辑。
10. 平面弯曲路径零厚度域的松z界限制显式记录，不能伪造成功；本批不以这种局限为运行接入已完成。

先隔离直接C++14构建和运行，再ASan/UBSan；验证CMake目标接线，原path/normal及M1A/M2回归。源码使用apply_patch。实现前后记录status/diff/hash，diff --check与白名单核对。不得写原build/devel，不连接用户ROS。

## 6. 主会话自审与后续阻断项

- 已将测速平面特例与一般升降路径区分；明确承认范数转分量的宽度局限。
- 已把地图值转换与真实SDFMap同步读取区分，空云不自动证明free。
- baseline cloudCallback按接收方camera_pos再过滤，空云直接返回；已知域不能直接套用最新odom范围。生产切换需要明确生产者采样中心与接收裁剪域交集、膨胀及clearance halo，以及reset后域失效处理。
- 当前local_sensing另有全图遍历完整性检查；移除旧身份链时应一并核查其成本，不将它算入本批已测builder性能。
- Sol和主会话审核发现左右闭cell在C2接缝可能使用不同函数，而且path->evaluate带1e-8选段容差。R1端点一致检查不足，R2最终版收缩为单segment owner，任何多段对象拒绝。一般真实C2接入仍需后续修正可证明的接缝误差处理，不通过放宽容差解决。
- 本批不改变已有执行行为，也不承诺完整M3交付。后续成组规格未冻结时禁止触碰地图/控制消费者。
