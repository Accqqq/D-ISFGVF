# M2：离线水平截面 Builder 实现与监督验收

日期：2026-09-09

依据：[M2 执行规格](Horizontal_Section_Tube_M2_Execution_Spec_2026-09-09.md)。

## 1. 本批结论

离线截面builder已实现，并通过主会话独立构建、数值测试和sanitizer检查。编码由用户指定的Luna / max子代理完成；主会话冻结算法/白名单，逐段审查并要求返修，最后独立验收。

交付内容是围绕给定路径生成水平左右截面及连续PWL几何profile的纯数学模块，不是完整ROS tube替换。原SDFMap、planner/path、manager、adapter、preview、SPH及实际执行链均未切换。没有启动仿真或连接用户ROS进程。

## 2. 文件范围

以 `src/swarm_planner/phase_offset/phase_offset_navigation` 为根：

- 新增 `include/phase_offset_navigation/section_tube.h`：输入环境、真实路径查询/整段导数界、配置及profile接口。
- 新增 `src/section_tube.cpp`：同步构建、分离平面、截面解析、正则性、细分和PWL查询。
- 新增 `test/section_tube_test.cpp`：28项离线用例及独立几何oracle。
- `CMakeLists.txt` 只增加9行，新src加入原库并注册/链接 `phase_offset_section_tube_test`。

M1A的4文件保持原样，旧navigation/core代码、原build/devel、launch、baseline、SUPER、论文附件均未修改。没有新增worker、map/support/configuration身份链或实际控制模式。

## 3. 已实现的算法

### 3.1 数据契约

环境输入为已知完整局部障碍盒、reference_domain和obstacle_region；空障碍列表不能自行证明空间已知。reference_domain加clearance必须落在完整obstacle_region内。

路径输入提供p、p_w、p_ww、规范水平N及N_w，并提供整段速度下界、逐坐标abs_p_ww/abs_N_ww。所有数据在构建期间不变、同坐标系。点值抽查用于发现明显矛盾，不会把采样拟合当成连续导数界。

navigation不依赖plan_env/ROS；后续由薄bridge将M1A视图及实际路径转换为此接口。

### 3.2 几何截面

1. 按结构断点和max_step_w形成有限初始单元。
2. 用局部弦段与障碍AABB的最近点方向生成分离平面，按完整盒支持函数扣除剩余clearance。
3. 对每个plane、每个cell端点，解：

   `alpha*delta + c0 + c1*abs(delta) <= beta`。

   其中c0、c1来自逐坐标二阶界与cell跨度平方/8。正负半轴分别线性求交，直接获得非对称区间，不查询三维安全球，也不做横向ray步进。
4. 该条件约束整段常数offset矩形域。相邻cell边界取区间交集，形成PWL节点；插值后的区间仍在对应cell已验证矩形域中。

域的z方向若真实为常量且相应二阶界为0，支持零厚度水平参考域，不因为横向弯曲而误加z方向运动余量。

### 3.3 正则性与结果

使用anchor处的一元二次不等式和整段变化界限制r_w的最小范数，保留含0的连通分支；弯道内外两侧不强制相同宽度。

N_w整段恒0时，r_w=p_w与delta无关，可信速度下界足够则保留整个几何区间。真正几何零宽或half_width=0不会为了不存在的横向容量递归到底。

一般正则性证明成功得到[0,0]也直接接受，不再要求另一条较弱的path_speed_lower重复证明。只有一般证明不足时，才使用明确的已有零路径速度证据作为有限分辨率退路。

完整覆盖返回COMPLETE或ZERO_ONLY；后段未完成/预算终止但已有连续前缀时返回PARTIAL，保留失败原因和实际valid_end，不越过失败cell或外推查询。

## 4. 主会话审核和修正记录

| 问题 | 修正/验收 |
|---|---|
| 存在无用的占位计算函数和假span | 删除stub，只保留显式相位跨度的真实求交 |
| 正侧使用alpha+2*c1再减c1，浮点下不等价 | 改成直接alpha+c1，负侧alpha-c1；进一步简化为零连通半轴求交 |
| 向内取整helper却命名outward | 改名澄清上下界含义 |
| plane余量仅取abs(sum)，忽略大项相消 | 按dot各项绝对量级处理数值内缩，不改物理clearance |
| halo普通加减可能吞掉极小正clearance | 使用有限TwoSum残差比较；修正测试中真实余量不足的手写ROI，未放宽检查 |
| 接受cell数与初始cell数相等才算完整 | 改为全部初始区间处理成功且精确覆盖终点，允许正常细分 |
| 预算前缀仍挂BUDGET状态，usable语义不一致 | 几何前缀统一PARTIAL，budget保留在failure_reason |
| 起点结构断点被拒绝、近似端点拼接可能跨gap | 支持起终点断点，生成并要求精确共享cell端点 |
| bounds缺少样点矛盾检查 | 核对速度下界、p_ww分量、N/N_w规范公式；不把抽查称为证明 |
| 零路径证据与非零充分条件混在一起 | 明确完整几何检查后的零证据；seed/query/budget失败不能伪装ZERO_ONLY |
| 常normal但p_w变化时被anchor误差压零 | 加入r_w=p_w的精确常normal化简，实测零/宽域特例 |
| 一般证明已通过[0,0]还被另一条松下界拒绝 | 删除重复门槛，新增两种zero来源且松速度下界的回归 |
| Oracle仅全局21点、薄障碍断言允许COMPLETE | 改为每个knot区间21w×11delta及显式delta=0；严格验证前缀在不安全相位前结束 |
| “零长度seed”测试并未实际覆盖零方向 | 用精确闭合多项式与远处盒构造真零seed，要求细分后完整且通过距离oracle |
| 缺近重根与真实曲线绕障验收 | 补helix近重根、正负曲率分支、根弦穿盒但圆弧绕开的严格用例 |

这些修正没有改变原控制律、安全参数或地图；均在本批白名单内完成。Luna的初次“通过”自报未直接作为最终验收。

## 5. 主会话独立验证

产物目录：`.horizontal_section_refactor/m2_20260909_01/`。

### 5.1 构建

独立catkin配置仅包含phase_offset_core和phase_offset_navigation，CMAKE_PREFIX_PATH仅为`/opt/ros/noetic`，build/devel/install均在本批产物目录，不使用旧工作区devel。

新navigation库及 `phase_offset_section_tube_test` 构建通过。原有Eigen3导出变量和core GNU __int128警告没有顺手修改。

主会话另从最新两份新cpp/test独立编译ASan+UBSan二进制，使用：C++14、O1、`-ffp-contract=off`、`-Wall -Wextra -Wpedantic -Werror`、`-fno-omit-frame-pointer`、`-fsanitize=address,undefined`，只链接Eigen/STL/gtest。该构建通过，没有因新模块warning而放松-Werror。

### 5.2 测试结果

| 检查 | 独立结果 | 产物 |
|---|---|---|
| 新截面builder catkin测试 | 28/28通过 | `section_tube_catkin_reviewed.xml` |
| 新模块ASan/UBSan，detect_leaks及halt_on_error开启 | 28/28通过，无报告的sanitizer错误 | `section_tube_sanitized_reviewed.xml` |
| 原GeometryEvaluator | 28/28通过 | `geometry_final.xml` |
| 原MatchedPort | 6/6通过 | `matched_final.xml` |
| 原PhaseOffsetAllocator | 38/38通过 | `allocator_final.xml` |

合计28项新用例、72项原回归；sanitizer是新用例的另一种运行配置，不将其重复计为额外28项新用例。全部stdout和退出码记录在 `supervisor_*_final.json`。

### 5.3 代表性验收

- 空旷直线达到设定左右宽度；单侧/双侧墙按明确clearance产生相应非对称/对称边界。
- 水平z=1零厚度参考域支持弯曲路径。
- 薄障碍位于节点之间仍被发现，返回前缀严格止于首个中心线净空不足相位前。
- 单位圆根弦穿过中心小盒、真实圆弧绕开时，有限细分恢复COMPLETE，并逐cell检查距离/速度。
- 精确闭合多项式p0=p1的种子可处理，未以“零长度”除零或跳过障碍。
- 正/负曲率的不同正则性分支、helix近重根、常normal但非零加速度、松path_speed_lower下的已证明零区间均有回归。
- 预算停止和query失败只保留真实连续前缀，不跨gap、不外推。

距离oracle直接计算参考点到原始障碍盒的距离，不复用plane生成逻辑；每个输出knot区间检查21个w、11个均匀delta及显式0，并检查domain及r_w最小速度。有限采样仅为交叉验证，连续性根据仍是算法的h²界及调用方提供的真实整段导数界。

## 6. 不应夸大的结果

- 本批为零偏移可行分支的安全内近似，不求全局最大截面；某些真实非零分支可行而零分支不可行的情况不由本版处理。
- 测试使用解析路径及明确给定的界，真实B-spline/C2路径的bridge、逐坐标界供给和局部地图生产者契约仍需后续接入验证。
- 物理安全距离仍相对于输入障碍盒；没有把论文0.55与原inflated-map上0.4的语义差异悄悄合并。
- 输入要求正常IEEE最近舍入、不可变同帧数据；不宣称fast-math/FTZ或任意不可信callback下的形式化保证。
- 当前算法会逐cell检查输入障碍盒，工作量由显式预算限制；没有证明大场景/多机性能达标，也没有把测试总耗时当成真实tube更新耗时。
- 未接preview/SPH、未修改轨迹安装、未运行ROS导航场景，因此不宣称论文提前收缩、集群变形或最终避碰实验完成。

## 7. 修改边界与指纹

branch仍为 `pro_review_current_20260903`，HEAD仍为 `20d44c94d4165e35621cc30269e3fa0b2a5f10ff`。

排除本批4文件后的rg可见src指纹，前后均为：

`bfb44f14126d650041aaf9fadc964f12f0b6aff1ab9860c18bba575e472cae88`。

最终代码SHA256：

- section_tube.h：`d15d403e69a8ef0eebb6b665df204b5522ef15efbda7a232770eae629d8b779a`。
- section_tube.cpp：`540581697b3e0f61611de0353f361999e9f08f6bceea85e55d533f5eea48e2b0`。
- section_tube_test.cpp：`d56027535fdb2a279c6a775abd4f51a2f1e32f0776570f0911b8889b4561532f`。
- navigation/CMakeLists.txt：`7f5e1064e01956390380b5e850558eb0a0e0bfacdd6745d52564e44a8475e6a7`。

git diff --check及新增文件单独空白检查通过。未stage/commit，原有M1A及用户未跟踪文件保留。

## 8. 停止点与下一步

本批离线builder完成，Luna已停止等待下一份有界规格。下一步将真实路径/局部地图接到此接口，并完成preview和运行时的成组替换设计；只在配套接口和回归成立后恢复baseline地图并退出旧tube链。不得直接把新profile塞进旧V2接口或用假ID维持旧安装机制。
