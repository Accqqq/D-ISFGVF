# M2：离线水平截面 Builder 执行规格

日期：2026-09-09

本批由主会话依据用户对下一批横截面builder的确认冻结。Luna / max负责编码，主会话监督、源码审核与独立验收。不自动切换运行时，不替换地图。

## 1. 范围与基线

W=`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`；N=`W/src/swarm_planner/phase_offset/phase_offset_navigation`。

基线HEAD=`20d44c94d4165e35621cc30269e3fa0b2a5f10ff`，branch=`pro_review_current_20260903`；M1A的4文件及此前文档/产物均保留。

代码白名单仅为：

1. 新增 `N/include/phase_offset_navigation/section_tube.h`。
2. 新增 `N/src/section_tube.cpp`。
3. 新增 `N/test/section_tube_test.cpp`。
4. 修改 `N/CMakeLists.txt`：仅将新cpp加至现有production库，注册/链接新gtest。不改旧源列表、legacy测试、flags或dependency。

主会话文档：本文件和 `W/docs/Horizontal_Section_Tube_M2_Report_2026-09-09.md`。构建/测试/清单产物：`W/.horizontal_section_refactor/m2_20260909_01/`。

禁止修改M1A接口、原path/normal/core、manager/adapter、任何launch、原build/devel、baseline/SUPER；不复制整个SUPER、不增worker/ID/证书、不做新轨迹优化，不创建git分支或提交。

## 2. 本批真正交付的能力

纯Eigen/STL的同步builder：输入不可变的路径查询/几何界及局部障碍盒，生成沿相位的左右非对称安全区间及连续分段线性profile。使用局部种子与分离平面思想，不依赖旧V2 builder、free-ball query或地图实现。

本批先处理原路径零偏移可行的连通分支，保证有效区间包含0。不声称覆盖零偏移已不安全但别的非零分支仍可延续的所有情况；此类场景返回有限能力失败，不等同于物理无解。此限制与原计划的首版零分支凸内近似一致。

不包含真实ROS路径bridge、preview、SPH或路径安装。M1A view由后续bridge复制为本批纯值环境，navigation不能反向依赖plan_env。

## 3. 固定输入/输出接口

namespace `phase_offset_navigation`，类型可按下述英文名直接实现。

`SectionBox`：闭AABB，Eigen::Vector3d min/max。

`SectionEnvironment`：

- `bool available=false`，来自上层明确的已知完整局部view状态；不是从空obstacles推断。
- `reference_domain`：参考点可检查域，可以零厚度。
- `obstacle_region`：障碍集合完整的域，包含reference_domain所需clearance邻域。
- `vector<SectionBox, Eigen::aligned_allocator<SectionBox>> obstacles`：已膨胀封闭体素/障碍盒，可重叠。

`SectionPathSample`：p、p_w、p_ww、N、N_w，全部Vector3d。

`SectionCellBounds`：

- `double horizontal_speed_lower`、`path_speed_lower`：整段有效非负下界。
- `Vector3d abs_p_ww`、`abs_N_ww`：整段逐坐标二阶导数绝对值上界，非负有限。
- `bool valid=false`。

horizontal_speed_lower必须大于原core的水平normal能力阈值1e-8；0不是足以证明整个cell可定义N的下界。minimum_reference_speed同样必须严格正。已有当前路径producer满足该阈值，本批不改原core参数。

几何界与点值必须来自同一不变真实路径；调用方负责提供可信整段界，本批不把有限样点拟合成界。点值必须满足horizontal N定义（规范cross-product方向），N_z=0、unit N、p_w与N正交，N_w与N正交；校验尺度相关误差限1e-8，拒绝明显不一致/水平切向退化。

校验不只检查正交：在sample处用q=e_z×p_w、q_w=e_z×p_ww核对N=q/||q||及N_w=(q_w-N*(N.dot(q_w)))/||q||，防止传入相反normal或与真实N导数不一致的N_w。拒绝明显低估样点p_ww、horizontal speed的bounds，但不把这项抽查称为界的证明。

逐坐标界是必要的：若p_z常数，则abs_p_ww.z=0；水平N恒有abs_N_ww.z=0。不能用非零全方向范数替代已知为零的z方向，让零厚度水平参考域永远检查失败。后续真实path bridge需提供/验证这些值，不能将M0单次采样结果当作该能力已全面交付。

`SectionBuildConfig`：

- `clearance=0.4`：相对于输入障碍盒的剩余安全距离；不再次计算地图膨胀/机体半径。
- `half_width=1.0`，允许为0（真实zero-only请求），不得为负。
- `minimum_reference_speed=1e-8`；测试可显式用论文0.2，不能暗改生产参数。
- `max_step_w=0.1`、`min_step_w=1e-4`、`max_depth=8`。
- `max_cells=2048`（所有访问的初始/细分单元）、`max_obstacle_checks=200000`（所有seed/box检查）、`max_obstacles=20000`。
- 只允许正常有限配置；max_depth>=0，所有容量>0，min_step<=max_step。

`SectionBuildInput`：w_start/w_end、structural_breakpoints、std::function<bool(double,SectionPathSample&)> point_query、std::function<bool(double,double,SectionCellBounds&)> bounds_query、SectionEnvironment。范围由调用方显式给出，w_end>w_start；断点必须有限严格递增且在范围内，可包含端点。

`SectionTubeKnot`：w、lower、upper。

`SectionTubeStatus`：COMPLETE、PARTIAL、ZERO_ONLY、INVALID_INPUT、UNKNOWN_DOMAIN、GEOMETRY_UNAVAILABLE、FRAME_DEGENERATE、SEED_BLOCKED、REGULARITY_LIMIT、BUDGET_EXCEEDED。

`SectionTubeProfile`：

- status默认INVALID_INPUT、`bool usable=false`、`bool complete=false`。
- `valid_start/valid_end`（失败时不伪造范围）、knots。
- visited_cells、point_queries、bounds_queries、obstacle_checks、max_depth_reached。
- first_failure_w、failure_reason（构建未完成的原因与profile是否可用分开）。
- `bool evaluate(double w,double& lower,double& upper) const`：只在usable有效闭域内插值，不clamp到域内、不跨空隙。非有限输入拒绝。

函数 `SectionTubeProfile buildSectionTube(const SectionBuildInput&, const SectionBuildConfig& = SectionBuildConfig());`

输出仅值数据，不能承载map epoch、support ID、逐query证书或控制状态。节点按w排序、区间包含0。

## 4. 几何方法

### 4.1 全局输入校验与域条件

构建前检查callback存在、环境available、盒子有序有限、config有效、障碍数量和结构断点合法。

available=false返回UNKNOWN_DOMAIN。reference_domain是query域，不是free域；必须避开所有输入obstacles。障碍盒允许退化点/面表示，但必须通过严格分离平面，不因零厚度跳过。

reference_domain扩展clearance必须在obstacle_region中，否则INVALID_INPUT/UNKNOWN_DOMAIN（固定采用INVALID_INPUT表示矛盾完整域契约），不能使用为更小clearance捕获的局部障碍列表。使用有限步骤保守数值判断；恒等和精确边界不能被粗大tolerance放宽。不能从obstacle_region的裁剪盒再额外减一次clearance约束reference_domain；M1A已经形成适用的reference_domain。

### 4.2 初始分段与缓存

先按structural_breakpoints分段，再按max_step_w均匀细分各段，保证真实结构断点进入网格。不强迫使用path内部1024个弧长查表节点。

每cell取w0、w1和midpoint点值，一次bounds_query；点查询按w缓存，已查过的不重复算。构建入口同步执行，无ROS线程依赖。

索引循环与节点预分配有界，初始网格数量若已超过max_cells，构建前返回BUDGET_EXCEEDED，而不是先生成巨大vector。

### 4.3 种子与障碍分离平面

种子线段s(t)=p0+t(p1-p0)，t∈[0,1]。

对每个障碍盒求近点方向：收集线段坐标与盒各min/max平面的t交点及0/1，排序去重。每个t小区间内，point-to-box squared distance是一个二次式；检查端点及驻点，得到最近seed点s和盒点o。平行轴/零长种子单独处理；数值溢出/不确定不能被当作空障碍。

若seed与盒相交，或无法得到非零有限分离方向，该cell尝试失败；允许按4.6细分，因为弦碰撞不等于真实曲线路径碰撞。

取a=(o-s)/||o-s||；安全平面：

`a.dot(x) <= min_{v in box} a.dot(v) - clearance * a.norm()`。

box的min支持值逐坐标取对应min/max端点计算，不把体素当零半径点，不重复添加半对角线。a.norm用于实际计算值，不能无条件假设浮点归一化后严格等于1。

最近点算法即使保守也不能靠它单独声明安全：必须核对seed两个端点和后续连续参考带在这个平面安全侧。平面本身通过盒支持函数保证与整个盒分离。所有obstacles都需覆盖；本批不实现会漏约束的去重/截断。

障碍平面允许一个尺度相关的数值向内余量（例如64*machine-epsilon乘dot/product量级），记录于注释，不把它当可调物理安全参数，也不引入大固定厘米级余量。域平面不额外膨胀，不对已内缩reference_domain再收缩机体半径。

### 4.4 同时求区间并检查连续相位

每cell先使用常数offset区间[L,U]，再装配PWL profile。这是局部安全内近似，不求最大自由空间。

对一个plane(a,b)，点端点j=0,1处设：

`alpha_j=a.dot(N_j)`，`beta_j=b-a.dot(p_j)`；

`c0=abs(a).dot(abs_p_ww)*(w1-w0)^2/8`；

`c1=abs(a).dot(abs_N_ww)*(w1-w0)^2/8`。

因为常数delta下 `|(a.dot(r))''| <= abs(a).dot(abs_p_ww)+|delta|*abs(a).dot(abs_N_ww)`，要求两个端点均满足：

`alpha_j*delta+c0+c1*abs(delta) <= beta_j`。

分别在[-half_width,0]与[0,half_width]解线性不等式即可，不做横向射线采样。零系数时检查常数侧，不能除以近零数造成NaN；若系数很小但非零，可用长双精度比值或安全饱和求交，不能将其随意置零丢约束。

六个reference_domain轴向边界也作为plane加入。对水平恒高度路径，其z二阶界和N_z严格为0，零厚度reference域可通过零等式约束。

delta=0在任何plane连续条件下不成立，该cell不能返回非零连通tube；走4.6细分。不把失败结果伪装成[0,0]。

注意：上面的h²界对整个cell成立的前提是提供的分段二阶上界有效，且一阶导数在cell内连续；不连续处必须有结构断点。本批测试使用解析光滑路径，后续真实bridge不能跳过这个契约。

### 4.5 非对称正则性

在当前由plane得到的[L,U]上，使用anchor=midpoint的p_w、N_w。

设D=max(|L|,|U|)，A=||abs_p_ww||，B2=||abs_N_ww||，E=(A+D*B2)*(w1-w0)/2。

要求对区间内所有delta：

`||p_w(anchor)+N_w(anchor)*delta|| >= minimum_reference_speed+E`。

这通过一元二次不等式求解；只保留含0的连通分支，不能跨过奇异gap。不要用|delta|对称上界代替这一非对称求交。必须处理零二次项、近重根/判别式、两根同侧、已有区间限制。

精确常normal特例：若sample的N_w=0且整段abs_N_ww=0，则整段N_w恒为0，r_w=p_w与delta无关。此时只要path_speed_lower>=minimum_reference_speed，保留整个plane区间，不因p_w方向/速度变化的anchor误差把本来安全的横向容量压成0。这是原正则条件的直接化简，不增加新控制律。

一般anchor+E二次条件本身也是完整速度证明：若它成功且输出区间为[0,0]，直接接受ZERO_ONLY，不再要求path_speed_lower额外证明同一事实。显式half_width=0也可使用一般条件，不能因调用方提供合法但较松的path_speed_lower而跳过更强的已有几何证明。

若这个充分界暂时无法证明非零分支，但path_speed_lower已证明整段零偏移速度>=minimum_reference_speed，且所有plane证明零偏移安全，则可返回真实ZERO_ONLY区间；不得从query failure推导ZERO_ONLY。为争取非零容量可先按4.6预算细分，达到有限深度后才保留已证明的zero-only。

例外：half_width=0的显式请求，或几何域已真实只剩[0,0]且零偏移速度下界足够时，直接接受zero-only，不为不存在的横向容量强制细分。禁止复刻旧方案“已证明零路径但仍递归到底”的开销。

zero-only退路只适用于已经完成所有obstacle/domain连续检查、仅非零正则性充分条件不足的cell；seed失败、未完成障碍枚举、callback失败、预算失败绝不能借此转成zero-only。

这个Lipschitz正则性界是保守的，测试同时报告弯道内侧损失与外侧保留。它不是旧几何安全球查询，也不要求两侧宽度相同。

### 4.6 有界细分与失败归因

cell因seed、plane连续界或regularity无法通过时，在depth<max_depth且半宽>=min_step_w并且预算允许时二分；按w递增处理左右子cell。点值/bounds能力失败可按同样预算细分，达到限制后明确失败。

所有访问cell和obstacle检查都计入预算，callback调用次数由cell/缓存有界。不对预算失败再递归，不使用无限retry。

完整覆盖的判定使用所有初始段处理成功、无终止失败、覆盖精确到w_end；不得比较“接受cell数量等于初始cell数量”，细分会改变数量。结构断点允许包含起终点；生成的相邻cell端点必须精确共享，不通过容差跨过gap。

第一个不可接受cell之前已经完成的连续前缀可返回PARTIAL（usable=true、complete=false、真实valid_end）。从首cell就失败时usable=false、knots空。不能跳过一个失败cell继续拼后段。精确ZERO_ONLY也必须提供真实连续范围；部分zero-only结果仍是PARTIAL并保留失败原因。

### 4.7 PWL装配

每个接受cell的[L,U]对整个cell都安全。共享节点取相邻cell区间交集；首尾用本cell区间。两个端点区间均为该cell区间子集，因此线性插值的任意offset仍在该cell安全区间内。

装配检查节点严格递增、finite、lower<=0<=upper、无gap。complete全域且所有节点均[0,0]时status=ZERO_ONLY；complete有非零宽时COMPLETE；因失败截短为PARTIAL。

明确HALF_WIDTH=0或几何零宽域与非零几何域的regularity证明不足不同：后者先按已设预算尝试细分，不以立即回零掩盖可用空间。细分预算耗尽时保留已完成前缀，不反过来覆盖已生成child结果；zero-only仅在对应cell几何完整且有速度下界证明时授予。

不在本批加入preview、平滑器、profile安装或动态状态所有权。

## 5. 固定离线验收

至少以下测试类别，使用固定解析路径和独立几何oracle；必须有显式main以适配本机catkin gtest。

1. 空间充足直线：得到名义左右宽度，COMPLETE。
2. 单侧墙：相对clearance的解析左右非对称边界正确。
3. 双墙：解析截面正确，不重复侵蚀reference_domain/机体余量。
4. 搜索半宽0和仅横向域宽0：真实ZERO_ONLY，不能等同UNKNOWN。
5. 未知/错误环境、缺失callback、无效box/config/断点：明确失败，不残留可用profile。
6. 水平弯道：N规范方向、内侧正则性限制、外侧仍可展开；变化曲率cell细分有限。
7. 水平z=1零厚度reference_domain：曲线路径也能构建（逐坐标界z=0）。
8. 薄障碍夹在普通节点之间：不得漏检或跨gap插值。
9. 弦穿障碍但曲线绕开：合理细分后至少保留真实连续可用域，不把所有弦失败称为path碰撞。
10. 不同平面/相邻段切换：PWL节点交集和整段安全，左右不强制对称。
11. 水平切向退化、normal不一致、regularity能力不足：失败归因正确。
12. 初始网格超预算、细分耗尽、obstacle数/检查数超预算：有界退出，无伪造完整结果。
13. 后段确实不可行：PARTIAL的valid_end准确，evaluate不能外推/跨gap。
14. 线段零长/平行面/盒面接触的数值路径：不除零、不NaN、不跳过障碍。
15. 相同输入重复构建结果确定，点值缓存避免同w重复查询。
16. 用独立point-to-box距离对所有输出cell的至少21个w、每w至少11个delta检查clearance；这只是数值交叉验证，不替代4.4的连续数学条件。
17. 二次regularity特殊情况（N_w=0、两根、近重根、根在已有区间外）和输出区间最小速度验证。

解析直墙边界允许1e-7 m量级误差；不能用该tolerance抵扣clearance验证的实质违例。弯道允许充分界造成的可解释内侧宽度损失，但空地图直线不能莫名窄于搜索上限。

复杂场景oracle直接计算参考点到原盒距离，不调用待测plane生成器判断自己是否安全。测试环境的obstacle_region要真正覆盖reference_domain+clearance，不能为使测试过而把冲突域标available。

## 6. 构建与监督

Luna先direct C++14/Eigen/gtest测试，再ASan/UBSan，输出只在本批产物目录。主会话审核最近点算法、区间符号、cell连续界、zero-only/partial语义与测试oracle，必要时原白名单内返修。

主会话另外在独立catkin build/devel中构建新target及原navigation库；不链接主工作区旧devel。旧core数学与M1A源指纹必须保持不变。

交付只说明离线builder通过，不声称真实路径bridge、runtime安装、地图恢复、完整导航或论文多机效果完成。若规格有数学矛盾或需要改原接口/算法，Luna应立即报告，不自增另一后端或新证书框架。
