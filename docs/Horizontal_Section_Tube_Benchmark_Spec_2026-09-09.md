# 新截面 Tube 独立构建速度测量规格

日期：2026-09-09

用户只要求测当前新builder，不与SUPER或旧V2作速度比较，不修改/优化算法，不继续运行时接入。

## 1. 写入范围

Luna仅编码 `.horizontal_section_refactor/benchmark_20260909_01/section_tube_benchmark.cpp`，必要的测试输出/二进制/对象也只能在该目录。主会话写本规格与 `docs/Horizontal_Section_Tube_Benchmark_Report_2026-09-09.md`，负责独立运行和审核。

所有生产src、CMake、launch、baseline/SUPER、原build/devel和既有M0/M1A/M2产物只读。不得调参数让结果变完整，不运行ROS仿真，不改用户进程/CPU频率。

## 2. 测量边界

- Release C++14，O3、DNDEBUG、ffp-contract=off，无sanitizer，无fast-math/native特殊优化。
- 直接编译当前section_tube.cpp，不使用旧V2，不引用旧devel。
- 每次wall计时仅包 `auto profile=buildSectionTube(input,config)`，包含函数内部缓存构建/清理、分离平面、几何/正则性和PWL装配。
- 输入生成、路径构造、地图提取、输出profile后续校验/析构、CSV和stdout均在计时外。
- 每场景10次warmup、200次正式测量；每次调用新的builder上下文，不跨build复用内部结果。
- 输出每次原始wall_ns，并统计mean/P50/P95/P99/max（nearest-rank百分位）。不删离群值。
- 另做20次带callback计时的诊断构建：记录point-query、bounds-query和instrumented total；剩余时间只能称为“几何构建及其他开销”，不能冒充单独的障碍检查时间。主统计采用不插计时器的callback版本。

## 3. 固定参数与输入

config保持当前SectionBuildConfig默认：clearance=.4，half_width=1，mr=1e-8，max_step=.1，min_step=.0001，max_depth=8，max_cells=2048，max_obstacle_checks=200000，max_obstacles=20000。

场景：

1. straight，p=(w,0,1)，w∈[0,2]，N=(0,1,0)，p_w=(1,0,0)，二阶界0，速度下界1。障碍数0/64/256/1024/4096/12000。
2. arc，p=(2*sin(w/2),2*(1-cos(w/2)),1)，w∈[0,2]。p_w=(cos(w/2),sin(w/2),0)，p_ww=(-.5*sin(w/2),.5*cos(w/2),0)，N=(-sin(w/2),cos(w/2),0)，N_w=(-.5*cos(w/2),-.5*sin(w/2),0)。abs_p_ww=(.5,.5,0)，abs_N_ww=(.25,.25,0)，速度下界1。障碍数0/256/4096。
3. mapped_bspline：当前真实ContinuousPhasePath::makeMappedBspline求值，7控制点 `(0,0,1),(.4,0,1),(.8,.2,1),(1.2,.5,1),(1.6,.9,1),(2,1.1,1),(2.4,1.2,1)`，三次、interval=.2，真实t_range映射w∈[4,7]。障碍数0/256/4096。
4. blocked_straight：同straight，单盒x∈[1,1.1]、y∈[-.1,.1]、z∈[.8,1.2]。这是故意验证局部失败耗时，不能按完整成功统计。

第3类路径在计时前构造。point callback调用真实path.evaluate(w,false)，再按规范cross-product公式计算N/N_w；不是解析直线代替真实路径。bounds callback调用当前legacy cellBounds，取q、速度下界、A_xy和J；abs_p_ww=(A,A,0)，abs_N_ww=(B,B,0)，B=J/q+3*(A_xy/q)^2。固定控制点z全为1，因而这里z界0有输入结构依据；不能把这适配器宣称为已完成所有真实路径类型的生产bridge。查询失败按原样返回，不修旧path/证明源码。

第3类只采用真实ContinuousPhasePath::segments()边界（本例4和7），不导入certificateBreakpointsV2的内部弧长表/ULP证明节点。整段legacy界能够覆盖内部表单元，新builder按默认max_step_w自行划分；测量适配器不能偷偷重新引入旧证明网格。

墙体盒生成确定、互不重复，边长0.1；顺序i：side=i%2，x_index=(i/2)%40，z_order=(i/80)%20，layer=i/1600。x_min=-1+.1*x_index；z顺序从中心高度向两侧展开（索引9,10,8,11,...,0,19，z_min=.1*该索引）。正墙y_min=wall_y+.1*layer，负墙y_max=-wall_y-.1*layer。straight的wall_y=1.2；arc/B-spline为2.5。

墙体数量增加主要用于稳定净空下的输入规模测量，不代表真实观测分布；不使用重复同一盒伪造数量。

reference_domain统一x[-2,5]、y[-2,3]、z[0,2]；obstacle_region统一x[-4,7]、y[-5,5]、z[-1,3]，明确available=true。这是合成已知静态局部域，不从稀疏点云推断已知自由。

## 4. 每次测量必须附带的质量字段

场景/路径类型、box_count、w_start/w_end、status、usable、complete、valid_start/end、coverage_ratio、knots数量、最小/相位加权平均width、visited_cells、obstacle_checks、point_queries、bounds_queries、max_depth、failure_reason。

逐次验证返回结果状态/几何与同场景基准结果一致，失败/部分结果也保留。密集盒可能触发默认budget；不能上调budget伪装完整构建，也不能将短prefix的耗时当完整tube耗时。

每场景首个输出在计时外做有限安全交叉验证（实际profile有效域，每个knot cell若干w及delta，检查原box距离、reference_domain和min speed）；这不是新连续证明，不计入build耗时。

## 5. 产物与监督

程序输出raw.csv、summary.csv、diagnostics.csv或同等明确文件，默认只在指定artifact目录落盘。stdout每完成一个场景打印简短统计并flush，方便主会话监督。

主会话在全部编译结束后运行（避免自身编译干扰）；可仅对本任务benchmark进程设置CPU affinity，记录CPU、频率策略、负载和实际亲和性。不能暂停用户进程或调整系统调度/频率。

报告明确：单线程、固定输入warm benchmark；不含planner/地图捕获/ROS/preview/控制/可视化，不是完整在线性能。只回答当前实现的测量结果，不比较SUPER，也不在测量中优化代码。
