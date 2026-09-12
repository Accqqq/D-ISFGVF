# M2C：真实路径逐坐标加速度界验收

日期：2026-09-09。依据M2C执行规格；主会话监督Luna编码、Sol只读数学审核、主会话独立编译与测试。状态：本有界批次完成并通过最终验收。

## 1. 实际改进

现有PathCellGeometryCertificate增加可选sup_abs_p_ww及能力位，quintic、弧长mapped B样条、linear fallback分别生产整段逐坐标加速度上界。section适配器在能力可用时取分量界与原范数界的逐分量最小值；能力缺失保留旧保守回退，能力声明存在但数值非法时拒绝。

固定高度曲线的z导数由原多项式/导数控制点的结构零提供精确0，避免M2B用全向范数误加z余量。升降路径仍使用真实三维p(w)，没有压平轨迹或限制planner高度。安全距离、搜索半宽、控制参数、旧scalar公式、point evaluator、相位映射、C2系数和segment选择均未修改。

N_ww公式不变；本批不消除所有保守性，不增加无人消费的高阶DTO，不引入V3身份/证书体系。

## 2. 实现与监督

- Quintic从二阶幂系数在查询区间的Bernstein控制点取逐坐标界，除h两次以避免h²先舍入造成下偏。
- mapped保留D2_i*|t_w|²+D1_i*|t_ww|两项，逐次乘法向上包围。linear的第二项为0。
- 32子区间退路逐坐标取max，能力位按所有子区间AND；可选字段不反过来门控旧scalar certificate。
- 初稿只检查变换后全零，可能把下溢当精确零。主会话与Sol均发现并要求源系数结构零guard，非零源变换成全零只使可选能力false。
- 新underflow测试初稿碰到旧jerk范数自身下溢，使scalar先失败。改用x(w)=w+w³加denorm级z扰动，仅修改fixture，不改旧范数数学，最终验证scalar有效、component能力false。
- slice初稿查过长区间，源路径同区间也缺旧scalar界；改为source/slice同一局部[2.3,2.8]比较，不放宽旧接受规则。
- 临时MQDB/MQC诊断输出已全部删除。无永久diagnostic字段。

## 3. 主会话独立验证

隔离产物：`.horizontal_section_refactor/m2c_20260909_01/`；配置只overlay /opt/ros/noetic。原build/devel、ROS会话未使用。

| 测试 | 通过 |
|---|---:|
| ContinuousPhasePath（含4项新增） | 24/24 |
| Section input（含3项新增及平面旧断言更新） | 22/22 |
| 原Geometry/Matched/Allocator | 28+6+38 |
| 原Normal frame/SectionTube/LocalObstacleView | 6+28+23 |

普通构建合计175项通过，不把旧20路径与新24路径重复计数。Path24与adapter22另在ASan/UBSan下通过，detect_leaks和halt_on_error开启。新路径/适配器代码及测试通过-Wall -Wextra -Werror编译；历史path测试包含hexfloat扩展，测试编译使用gnu++14，未修改这些旧fixture。

XML包含：continuous_phase_path_test_reviewed.xml、path_sanitized_reviewed.xml、section_input_reviewed.xml、section_input_sanitized_reviewed.xml及六份原回归结果。

验证零厚度水平B样条和quintic在默认clearance=.4、half_width=1下COMPLETE，真实升降mapped/quintic仍COMPLETE。独立oracle逐cell检查域、原盒距离、参考速度；密集差分仅为交叉测试，不作为连续界证明来源。

## 4. 范围与未完成项

仅M2C白名单6源码文件修改。主会话用进入时6文件摘要替换退出时对应摘要，恢复整体src指纹33e2379af7fb95e21859630ef554e5ae03fad24fd8948002a077a8fdfd5e9389，与进入时一致；其他rg可见源码未改变。HEAD仍20d44c94d4165e35621cc30269e3fa0b2a5f10ff；未stage/commit，用户改动保留。

本批仍只接受单segment owner，多段C2接缝还未处理；没有接真实SDFMap同步读取/已知域、preview和执行，没有退出旧worker/安装链，没有恢复baseline地图，没有ROS闭环验收。那些是下一份成组规格的工作，不把本次数学值改进称为总体重构完成。

## 5. 最终冻结结果

最终记录格式修复后再次独立catkin22/22、ASan/UBSan22/22通过；path24/24普通与san通过。175项普通测试和46项san配置执行均通过，san测试不另算新增用例。最终XML均存本批目录。

默认clearance=.4、half_width=1的零厚度域测试：FlatMapped覆盖w[4,7]全部3个相位单位，min全宽1.018378米；PlanarQuintic覆盖w[0,1.5]全部1.5个相位单位，min全宽1.016695米。相位长度不是米，不由此推断全场景性能或最大截面。这两例在M2B因z界偏松失败，现在已通过；升降路径回归仍通过。

最终SHA256：

- path_state.h：52fc8f570bcdb23a21d42351dfde26572e7ed0b3fe56a2aaa1fde9dafbd01b95
- continuous_phase_path.cpp：8baf6995a55f6ec5bfa80090763efedd51c9f9242c85ad25145e82a6f0eccc5c
- continuous_phase_path_test.cpp：3c70e8b63ce25e34f638d9e15876531aad050156afc7b841fe7a3d36f215dca7
- phase_offset_section_input.h：93417b39cb1a774f7df9e76a5dcdff7f5e896616265e2ca78821e7642b9a84a2
- phase_offset_section_input.cpp：b050f3d38f32479974c59dde35206ddbf5df3104ea8dd24e379adbd978b11efc
- phase_offset_section_input_test.cpp：c8eca469a02a2fe8fac7f3525095d379960b67ee57e14144eb5a87141330f24f

最终diff --check通过；Luna冻结源码，主会话在本批完成后停止功能编辑，不自动扩展后续阶段。一次Luna中间构建误用了本批supervisor_build，主会话要求其改回executor，并从最终源码重新独立构建；未写原工作区build/devel。所有临时源码诊断已移除。过程中代理曾遭429限流，未以更换模型或削弱验收绕过。
