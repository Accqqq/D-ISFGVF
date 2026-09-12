# 水平横截面 Tube 重构详细计划

日期：2026-09-09

版本：R1（本会话自审修订版）

状态：设计计划，待用户确认；不是已授权的代码执行规格

关联自审：[自审记录](Horizontal_Section_Tube_Refactor_Self_Review_2026-09-09.md)

## 0. 本轮授权与结论

用户本轮要求“先做详细计划文档，然后自己审核一遍”。本轮只允许新增本计划及自审记录，不修改代码、配置、论文、构建产物，不启动仿真，不创建分支、提交或操作用户进程。

本计划收敛此前讨论，采用以下架构：

> 保留原 planner、地图核心、ISF-GVF 和物理执行链。先有原规划路径，再参考 SUPER 的局部种子和分离平面思想构建左右安全截面。截面构建在规划更新流程内同步完成；控制发布独立运行。Preview 和 SPH 调整活动参考，matched 注入保持原数学结构。

不构建完整三维体积 tube；不引入垂直 offset；不导入 SUPER 的整套轨迹优化与 backup 系统；不保留独立 tube worker 和逐帧地图身份认证体系。

本计划可以用于评审和分阶段准备。实施前必须完成 M0 的基线、安全距离账本、路径几何界能力和精确文件清单，并由当前主会话形成经用户批准的阶段执行规格。未完成的事项不能由执行者自行猜测。

## 1. 已确定的用户意图与不可变边界

### 1.1 Tube 的含义

对每架 UAV，保留论文定义：

\[
r(w,\delta)=p(w)+N(w)\delta,\qquad
N(w)=\frac{e_z\times p_w(w)}{\|e_z\times p_w(w)\|}.
\]

每个相位的物理横截面是一条水平线段：

\[
C(w)=\{p(w)+N(w)\delta:\delta\in I(w)\},\qquad
I(w)=[\underline\delta(w),\overline\delta(w)].
\]

沿相位排列这些截面形成参考带状 tube：

\[
\mathcal T=\bigcup_w C(w)=\pi_x(\mathcal M).
\]

左右边界允许不对称。高度随原路径的 p_z(w) 变化，不由 delta 调整。这里“横截面”不是具有两个自由度的二维截面，也不是三维圆管。此前讨论中将 tube 改成三维多面体并集的建议不采用。

### 1.2 保留与不做

必须保留：

- 原 planner 的目标处理、搜索、B-spline 优化及路径参数化的名义输出。
- 原基路径 C2 接续能力；活动参考的连续性另外验证，不能由此直接推定。
- 原 ISF-GVF、governor、SO3 与模拟器物理模型。
- phase-offset 几何、matched tangent injection 的数学关系。
- SPH 邻居通信与协调公式；本次不顺带调整增益或替换集群组织方法。
- 原有真正的安全限制、速率限制与物理切向要求。移除的是无必要的身份绑定，不是为了成功而删除安全约束。
- `test_gvf.launch` 中 `gvf/circle_test/enable=false` 与 `auto_start=false`。

明确不做：

- 在走廊内再加一轮 SUPER 轨迹优化。
- 增加垂直偏移或新 QP/CBF 控制框架。
- 改写整个地图、规划器、phase_offset_core 或邻居处理模块。
- 把 delta 每次直接截断或清零来绕过接入问题。
- 将所有未知空间直接标记为空闲。
- 为新实现新增第三套 V3 证书、身份和安装状态机。

## 2. 已核对的工程基线与证据

路径约定：

- W：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`，当前主工作区。
- B：`/home/cxq/Sim_demo/New_ISFGVF/gvf_ws`，用户指定的干净单机行为参考，只读。
- S：`/home/cxq/ISF-GVF/related_work/SUPER`，方法参考，只读。
- 论文：`/home/cxq/.codex/attachments/df6421bf-63cc-4dd7-bb54-4f76eda6696e/pasted-text.txt`，只读。

本轮开始时 W：

- branch：`pro_review_current_20260903`。
- HEAD：`133a6cbed3401a44a7c1f864e4002a56a79f81a5`。
- 工作树存在大量已修改和未跟踪源文件；HEAD 不是完整实验基线。
- `git diff --binary` 的 SHA256：`a09903b563347753ce23870f936c7e83c23dbeec826836d4ff66dbf7377b35d7`。
- `rg --files -0 src | sort -z | xargs -0 sha256sum | sha256sum`：`3658d5a20a004b7bf2cb6f7edf5739b8505332c44c6432fcfd2c1830ca9a4efc`。此指纹覆盖 rg 可见的 src 文件，不替代 M0 完整归档。
- AGENTS.md 还指定历史单机提交 `9a0e975`。必须比较该提交与 B，不得声称两者完全相同。

### 2.1 源码事实

| 事实 | 核对位置（相对 W，SUPER 项相对 S） | 设计含义 |
|---|---|---|
| 生产采用 V2 builder | `phase_offset_navigation/CMakeLists.txt` 的 production target；实际位于 `src/swarm_planner/phase_offset/` 下 | 不能把所有旧 tube 文件都当作当前热路径 |
| 旧 builder/validator 主要仍在 legacy 测试库中 | 同上 | 后续移除生产依赖与删除历史测试是两件事 |
| 生产 epsilon 来自 `planning/safe_distance` | `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp` | 不能说 launch 中所有 margin 当前全部相加 |
| 安装比较 latest/captured support 编号 | 同一 adapter 的 admission/incumbent 检查 | 重复观测可能令已完成结果被拒绝 |
| map cloudCallback 仍构建旧 CloudOccupancySnapshot | `src/swarm_planner/plan_env/src/sdf_map.cpp` | 删除旧生产消费者后也要去掉无用生产成本 |
| manager 使用无参数全图 capture | `src/swarm_planner/bspline_traj/src/gvf_manager.cpp` 的 buildCurrentTubeRequestV2 | 新接口必须使用局部范围，不能只提供裁剪接口却不调用 |
| 当前 formation_planning 使用 AsyncSpinner(8) | `src/swarm_planner/bspline_traj/src/formation_planning.cpp` | 已有并发，不可把“同步 tube”误写为全进程单线程 |
| SUPER 在 generateExpTraj 内直接调用 SearchPolytopeOnPath | `super_planner/src/super_core/super_planner.cpp` | 走廊同步属于规划流水线 |
| SUPER 的重规划与命令发布是分离的回调 | `super_planner/include/ros_interface/ros1/fsm_ros1.hpp`，`Apps/fsm_node_ros1.cpp` | 控制不必等待规划完成；仍有提交锁及超时检查 |

### 2.2 已有运行记录的使用边界

已有 `/tmp/disfgvf_pinned_20260909/runtime.log` 记录：request 12 构建成功，当前相位在返回区间内，但 support 27→30 导致 admission 拒绝。对应记录明确 preview 尚未计算。它支持“构建失败与接入失败必须分开”，不证明当前所有失败都来自同一原因。

已有 `/tmp/disfgvf_anchor_hull_20260909/after/profile.csv` 中 A 样例约 304 ms、27965 次 free-ball 查询；B 的历史 8 秒问题经过局部修复降至约 29 ms，但有效覆盖很短。旧记录不是本轮性能实测，不可把旧缺陷或某个样例的加速比直接写成新方案效果。

## 3. 目标数据流与职责

```text
局部地图读取 ───────────────┐
                          ↓
候选 p(w) + 接续几何 → 横截面 Builder → 不可变 PathTubeBundle
                                              ↓ 小范围提交检查
                                    当前执行 bundle
                                              ↓
                                     I(w) 查询 / K(w) preview
                                              ↓
邻机消息 → SPH g_des ───────────────→ 速率分配 / matched GVF
                                              ↓
                                    原 governor / SO3
```

### 3.1 分层

- plan_env：原地图行为、局部占据和已知域读取；不知道 tube 是否安装。
- phase_offset_core：原纯数学层；不增加 ROS、地图、线程或机器人 ID。
- phase_offset_navigation：截面构建、连续参考带检查、preview、速率可行性。
- bspline_race integration：把实际路径和局部地图转换为值输入；规划流水线编排和 bundle 提交。
- phase_offset_swarm：只提供协调意图；不拥有 w、delta，不发布最终物理命令。

### 3.2 最小数据契约（语义固定，类型命名可在 M0 整理）

`LocalObstacleView`：

- 世界坐标 frame、局部查询 ROI、有效已知域、分辨率。
- 封闭占据体素 AABB 或等价且保守的几何表示。
- 地图数据语义与安全账本标识；一个可选语义 map revision，不绑定逐帧消息序号。
- 不可变所有权；构建期间不从 mutable SDFMap 逐点读取。
- 数据获取失败、域外、未知与占据分开返回。

`PathGeometryView`：

- path revision 和不可变路径所有权。
- 有效相位范围、结构断点。
- p、p_w、p_ww、N、N_w 的实际执行一致查询。
- 为连续检查提供所需的局部导数/方向界；高级能力不存在时明确返回 unsupported。
- 不允许换一套弧长反解或插值算法，使 builder 和实际执行看到不同路径。

`SectionProfile`：

- 有效范围、排序节点 `(w, lower, upper)`，约定为连续分段线性边界。
- 内近似几何检查结果和边界导数查询；未知/未完成区间不插值跨越。
- 路径版本、地图局部有效条件、配置值随对象保存。
- 原因码、截面/平面/细分计数、各阶段耗时；无逐查询证明身份对象。

`PathTubeBundle`（ROS integration 所有）：

- 不可变的配套路径、normal frame、SectionProfile。
- 一个 generation 防止目标 reset 或候选替换后的旧 bundle 提交。
- 路径切换接口所需的有限前缀/接续条件。
- 不包含可写的 current w、delta 或 SPH 状态。

`PreviewResult`：

- current w、适用 profile 引用、本周期策略与计算状态。
- K(w)、必要的单侧边界斜率、b_pre、beta、当前状态是否可接入。
- invalid 时不将默认 beta=0 冒充已计算结果。

## 4. 安全距离与地图：先统一语义，再谈不保守

### 4.1 首版约束

首版几何迁移以原有效 inflated occupancy 为表示基准，完整保留 manual/static/ceiling/boundary 等实际有效层。第一轮固定输入新旧对比使用相同的封闭体素和相同 residual epsilon，不通过改变地图表示或降低距离赢得“变宽/变快”。

当前 launch 的 `planning/safe_distance=0.4` 可作为旧配置固定输入对比值，但不自动等于论文中的机体物理净空。论文表中的 epsilon=0.55、m_r=0.20 以及当前不同代码默认必须在 M0 列出，不能混用。

M0 产出安全距离账本：

| 项目 | 必须记录 |
|---|---|
| 原始障碍表示 | 点、体素还是预膨胀地图，各层是否一致 |
| 体素体积 | h、边界约定；不能把体素当零半径点 |
| 原膨胀 | XY/Z stencil、预膨胀层来源；各向异性不靠一个标量猜测抵扣 |
| 机体/误差预算 | UAV 尺寸、定位误差、跟踪误差、地图不确定性，以及哪些已经覆盖 |
| tube residual epsilon | 相对于所选表示还要求多大距离 |
| planner 距离含义 | ESDF 查询对象、插值、soft cost 与实际碰撞阈值 |
| 论文报告距离 | 相对于真实障碍的独立测量，不使用 tube 内部估计替代 |

在账本定稿前，不允许宣称“0.4 与 0.55 等价”，也不允许机械计算 `epsilon_new=epsilon_old-0.1`。如果统一后的物理预算需要修改 planner 安全配置，提交单独决定，不隐含授权。

### 4.2 没有垂直 offset，但仍要避免实体碰撞

查询包含参考带及其机体/误差安全邻域的三维局部障碍；求解变量仍只有 delta。上下障碍通过局部几何约束限制可行性，不生成垂直运动计划。

若原地图已充分膨胀，可直接查膨胀表示并添加剩余预算；不能只截取 z=p_z 的零厚度点云后忽略机体高度。

### 4.3 已知域

第一阶段实验选用户现有静态仿真地图，但必须确认局部发布者确实输出指定域内完整障碍集合。已知自由域来自生产者契约或真实 raycast 观测，不来自点云极值、非空点云或随意设置 complete=true。

ROI 的获取范围覆盖整个待构建参考带及安全 halo，并且落在真实已知域。外侧未获取障碍不能影响声明安全的带；既要查询 halo，也要把参考安全邻域限制在已知域内部。

真实传感器的遮挡/未知适配不在首版“已知静态域”验收范围内。接口必须明确返回未知，后续适配需要单独验收，不允许把仿真假设用于真机。

### 4.4 并发和 map revision

- 删除的是复杂 authoritative/support 协议，不是所有线程同步。
- 原有多线程环境下，局部快照读取与地图写入必须遵循一个简单一致的同步规则。
- 锁内只完成必要的一致数据捕获/提交；平面生成、路径几何计算和截面构建在锁外。
- 不能在关闭 phase-offset 时仍生成旧快照、全图 support 数组或 tube 专用索引。
- 语义 revision 表示相关占据/已知域/几何配置变化；重复同内容观测不使 tube 自动失效。
- 配置和 frame 运行期间固定。重初始化时整体废弃相关 view/bundle，不在线维持配置代数认证。

首版局部更新有效性采用“在完整变更覆盖可证明时用 ROI 变化判断，否则重新检查受影响参考带”的简单规则；不要求先实现复杂 dirty-block 基础设施。若无法得到无遗漏的变更信息，不得因为 map ID 看似不变而跳过检查。

## 5. 横截面几何算法

### 5.1 选择的首版路线

采用 SUPER 的“局部种子段 + 障碍分离平面 + 重叠/连续处理”思想，但直接输出横向区间，不建立完整三维走廊消息或体积优化目标。

首版建议使用可测试的 seed-separation 内核，而不是导入整个 CIRI 依赖链。它不是声称与 SUPER/CIRI 等价或最大化横向宽度。完整 CIRI/MVIE 可作为离线参考；只有首版截面质量不满足预定验收时，再由主会话提出是否引入的变更，不由执行者临时叠加后端。

几何内核仍是 Eigen/STL。若实际复用 S 的源码，保留该文件版权、LGPL/MIT 等适用条款及引用，并评审最小依赖闭包；不能移除版权后当作原创。

### 5.2 种子与障碍平面

1. 沿实际候选路径划分局部段，先在 B-spline、相位映射、C2 接续的结构断点处分段。
2. 以短弦段 `[p(w0),p(w1)]` 为种子。节点密度同时受物理长度、方向变化及结构断点约束，不只由固定 Delta w 决定。
3. 对局部可能影响参考带的障碍体素构造分离平面，约束参考点与障碍保持 residual epsilon。
4. 截面与相邻段复用局部障碍数据；只做不会遗漏约束的冗余消除。

对封闭体素盒 V 与种子线段 S0：求最近点对 s∈S0、o∈V。若距离 d>0，取单位法向 n=(o-s)/d。候选平面：

\[
n^T x\le \min_{v\in V}n^Tv-\varepsilon_{res}.
\]

在最近点对正确、d 满足 epsilon 且整个种子验证位于安全侧时，该平面隔开种子与 V 的安全膨胀。对中心 c、半边长向量 h 的盒：

\[
\min_{v\in V}n^Tv=n^Tc-|n|^Th.
\]

此处显式考虑体素体积，不再额外加一次同一体素的半对角线。若输入使用点球近似，则必须在账本中记录其覆盖半径，不能与盒支持函数双重计入。

最近点求解必须覆盖退化线段、端点最小值、线段与盒相交、切触和浮点容差。无法获得有效分离平面时允许细分种子；弦段碰撞不直接等价于真实弯曲路径碰撞。

为避免首版静默过窄：保留每个限制面的来源和左右活跃约束；固定场景与独立横向净空 oracle 比较。该方法给出连通安全内近似，不声称恢复全部自由空间。

### 5.3 截面解析求交

初始限制为 `[-delta_search_max,+delta_search_max]`。delta_search_max 是应用允许的搜索上限，不是安全距离。

对安全平面 `a^T x <= b`，令 alpha=a^T N(w)、beta=b-a^T p(w)：

- alpha>0：更新 upper=min(upper,beta/alpha)。
- alpha<0：更新 lower=max(lower,beta/alpha)。
- alpha 接近 0：不得直接除法；根据尺度相关容差检查是否整条搜索区间被排除。模糊区用有界 alpha、beta 判断最坏值，不能一律当作无限宽。

求交后得到环境允许区间。无障碍时结果受搜索范围及真实已知域约束，不能无限大。

符号约定：论文中由真实 robust free space 定义的区间记为 I_paper(w)，实际 builder 输出记为 I_geom(w)，需满足 I_geom(w)⊆I_paper(w)。本文接口里的 I 指 I_geom。由于采用局部凸内近似，不声称 I_geom=I_paper 或全局最大宽；论文实现说明和实验必须披露这一点，不能修改论文定义来掩盖近似损失。

若用多个局部凸区域提供截面候选，只合并真正相交/接触且已验证安全的区间，再保留从初始化零偏移分支连续延续的分量。禁止取不相连区间的凸包。首版单种子局部凸内近似可能无法表达某些零偏移已不可行但非零分支仍存在的情形，应报近似能力限制，不能声称物理空间不存在。

### 5.4 正则性

在节点处精确处理一元二次条件：

\[
q(\delta)=\|N_w\|^2\delta^2+2p_w^TN_w\delta+\|p_w\|^2-m_r^2\ge0.
\]

处理二次项接近零、判别式近零、双根、零偏移本身不满足等情况。与环境区间相交后保持连续分支，不以统一 `|delta|` 最坏界无理由同时削掉弯道两侧。

节点通过仍不足以声明连续正则；整段最小速度需要第 5.5 节的局部几何界支持。水平切向退化、终端停车或不支持的段明确报 `FRAME_DEGENERATE` / `REGULARITY_LIMIT`，不靠无穷细分“证明成功”。原正常终端到达逻辑仍然有效，论文正向 phase 条件适用于 regular guidance 域，不覆盖停止后的终端状态。

### 5.5 截面之间的连续安全性

这是新 builder 的组成部分，不放到“安装”时再做一遍。

每个小相位段输出线性 lower(w)、upper(w)。对同一凸安全约束域，r 对 delta 仿射，因此只需检查两条边界曲线：

\[
r_-(w)=p(w)+N(w)lower(w),\quad
r_+(w)=p(w)+N(w)upper(w).
\]

对每个平面、每条边界定义 f(w)=a^T r_±(w)-b。采用以下有依据的局部方法，而不是仅查端点：

- 在具有二阶界的光滑单元，若 |f''|<=M，则端点线性插值加 M(Delta w)^2/8 可上界整段 f；上界<=0 才接受。
- 其中 `r_±''=p_ww+N_ww delta_±+2N_w delta_±'`。N_ww 能力不能由现有点值接口默认推定。
- 没有该二阶能力但存在有效区间/方向包围时，使用该方向包围；复用已有正确数值界的数学部分是允许的，不复用 V2 证书身份结构。
- 没有可信连续界时，该段仅为调试采样结果，不进入 production-safe profile。可以按预算细分，但不能将有限密采样标注为连续证明。

对正则性同样使用整段 p_w、N_w 的有界几何，在 delta 区间上验证速度下界。缓存每个路径小段的几何界，多个平面/截面复用；禁止每次地图查询重新运行路径弧长反解认证。

在连接断点处分段、检查共同端点。单元内部必要条件不通过时：细分/保守缩小边界/缩短有效前缀，三者均有显式计数和预算。不得跨过失败单元连接后面的 profile。

M0 必须查明真实 `ContinuousPhasePath` 与 normal frame 的界能力；若需要给真实参数化新增局部导数界，这是列入白名单的有限工作，不允许换掉名义 p(w) 以便证明。

### 5.6 预算与结果定义

几何预算覆盖：最大节点数、局部障碍数量、平面数量、细分深度/节点数及总耗时。具体值通过固定场景 M0 基线和 M2 压测后冻结，不从旧 V2 的巨大预算直接继承。

结果至少区分：`COMPLETE`、`PARTIAL`、`ZERO_ONLY`、`UNKNOWN_DOMAIN`、`CENTERLINE_CLEARANCE`、`FRAME_DEGENERATE`、`REGULARITY_LIMIT`、`GEOMETRY_UNSUPPORTED`、`BUDGET_EXCEEDED`。

- ZERO_ONLY 必须证明零偏移在其声明区间内可行；不能将 UNKNOWN 或未完成转换成安全的 [0,0]。
- PARTIAL 必须返回实际连续有效范围，并由使用方检查是否足够。
- “返回一个很短区间”不得作为整条路径或在线任务成功的验收。

## 6. Preview、离散执行与 SPH

### 6.1 几何和速率明确分离

保留论文反向递推：

\[
K_H=I_H,\qquad K_k=I_k\cap[K_{k+1}\oplus[-d_k,d_k]],
\qquad d_k=\overline u_\delta\frac{w_{k+1}-w_k}{\overline\nu}.
\]

geometry 采用自适应节点时，preview 合并实际几何节点、结构断点与所需的 preview 网格，不能跨过一个未检查的窄点。反向遍历为 O(H)；选取连续分段光滑内包络，验证整段处于 I 内并满足向内收缩斜率限制。节点内包含不自动证明连续包络内包含。

不得把原始几何 I 直接覆盖为 K；可视化和日志分别保留两者。b_pre、beta 按论文计算，beta 只调节规定的密度吸引分支，不随意缩放整个 SPH 或安全排斥。

### 6.2 时间与相位单位

Delta w 不是固定米数，也不天然等于秒；由实际 p_w 与 phase rate 解释物理前视距离/时间。

论文示例 `H=20, Delta w=0.1, upper_nu=3, upper_u_delta=0.25` 对应最短预览穿越时间约 0.667 s、该时间内最大横向位移约 0.167 m。窄 K 有可能是预期速率约束，不能全部归因于几何保守。

M0 对照实际 launch、运行时读取值及论文表，产出一份有效参数清单。未经明确评审不改变 upper_nu、m_r、安全距离或 SPH 增益。

### 6.3 不能只实现连续时间边界等式

论文的边界切向条件需映射到实际 50 Hz/ZOH 执行：

- 速率分配使用最终实际 dt 或已验证的 dt 上界。
- 在选定 w_dot、delta_dot 后，检查整个本周期跨越的 K 分段，而非仅下一端点。
- 对每个线性 K 单元与 ZOH 状态段，可解析检查时间子区间端点；跨节点必须切分。
- 保留原幅值、slew、phase 正向及物理切向约束。它们交集为空时明确报告，不能宣称论文的一次 clip 必然满足所有工程限制。
- 所有最终执行限制完成后，同一组 `(u_w,u_delta)` 用于参考状态更新和 matched 物理通道。
- 无实际控制命令成功发布时，不提前更新权威 phase/offset 状态；保留现有正确的单周期发布/提交事务语义，不因删除 tube 安装层而误删。

### 6.4 在线更新 K 的时间变化

滑动 preview 和地图更新会改变 K。不能依靠固定 K(w) 的不变性结论，直接证明任意时变 K(w,t) 都可行。

首版处理：每周期生成值结果；新包络接受前验证当前 delta 与下一控制段可行性。不能直接接入时继续使用仍有效的旧包络并有界回收，或进入明确恢复/安全停止。不得瞬间把 delta 裁进新 K，也不得无限保持已经不适用的旧 K。

## 7. 同步规划与轻量提交

### 7.1 调度

同一规划更新入口串行完成：候选路径 → 原接续几何 → 局部地图 view → section profile → candidate bundle。

地图相关更新和前视范围不足也进入这一串行规划/几何刷新入口：路径没变时复用路径与已验证前缀，只刷新受影响截面。更新请求可合并为一个 pending 事件，不新增独立 tube worker、request-completion 队列或第二套构建调度器。

控制 50 Hz 继续使用当前不可变 bundle，不持有规划锁等待重计算。路径/目标更新通过一个 generation 判断候选是否仍适用。规划内部同步不消除“规划期间机器人在移动”的事实。

### 7.2 有用覆盖与超时

需要覆盖当前状态、preview 和下一次刷新所需时间。建议使用物理含义明确的条件：

\[
\frac{w_{end}-w_{now}}{\overline\nu}
\ge T_{preview,min}+T_{refresh,budget}+T_{switch,budget}.
\]

它是足够覆盖的工程检查，不额外腐蚀横向宽度。取值来自实际策略与预算，不能随意叠加多个同义 slack。

规划完成后在最新 w/delta 上做轻量检查，而不是使用开始构建时的旧状态。必要时缩短、提前刷新或拒绝该次更新；控制不能等待超时构建结束才知道已到旧路径末端。

上式用于普通巡航。剩余路径已经覆盖真实终点且原终端停止过程安全可执行时，应采用原 terminal 契约，不强求终点后还有额外 preview 长度。否则固定前视长度会使接近终点的有效候选永远无法采用。该例外不允许在中途失败/未知区间冒充终点。

### 7.3 提交只负责适用性，不重复证明

一次短事务检查：

1. task generation 未被 reset/新目标替换；路径与 profile 配套。
2. 当前及下一控制段、需要的前视范围仍被覆盖。
3. 当前活动参考能连续接入候选，当前 offset 可执行。
4. 无真实影响候选使用区域的环境变化；有变化则在规划入口完成局部重检，而不是仅比较逐帧 support ID。

通过后整体替换 bundle。锁内不构建截面、不重新扫描整条 tube、不运行 CIRI。

已知静态几何且有效域没有失效时，不需要安装几何再验证。动态/新发现障碍与未观测域仍需真实检查；本计划不声称静态测试即可提供动态障碍时间安全保证。

### 7.4 非零 offset 的路径切换

仅“基路径 C2”不足以证明活动参考接续。切换前后至少检查：

\[
r^- = r^+,\qquad
r_w^-\dot w^-+N^-\dot\delta^-
=r_w^+\dot w^++N^+\dot\delta^+.
\]

首选复用现有连续路径的共同前缀：在实际 handoff 时段 p、N 及所需导数一致，当前 delta 和速率连续，后续新路径在前方接续。这使非零 offset 不必每次重规划都回零。

共同前缀必须按当前实际 w 和最迟切换时刻覆盖检查，不能只比较两个路径“前几个点相同”。候选构建期间旧执行状态继续前进时，提交处重新检查是否仍位于共同前缀；已越过则该候选不再具有此前的接续理由。

若不能建立这种连续关系：复用已经验证的 recovery continuation；其 source/target active-reference jet 必须实际匹配。若只具备一阶能力，不标注 active-reference C2。需更高阶保证时单独验证 N_ww、offset 速率导数及实际时间参数化。

最后的有限退路是在仍安全的旧 tube 中平滑回零，再进行原路径接续；它不能成为每次重规划的默认策略，否则可能压制论文中的集群横向行为。回零不可达时不能瞬移，进入明确重规划或原安全停止。

## 8. 失败处理与原单机兼容

| 事件 | 横向能力 | 原导航处理 |
|---|---|---|
| phase-offset 关闭 | 无 tube 构建/快照开销 | 完全走原单机链 |
| 截面非零且 preview 可行 | 正常协调 | 正常前进 |
| 原路径具备 tube 所需净空但仅 [0,0] 可行 | 不授予非零偏移 | delta 已为零时保留原导航 |
| 原 planner 可执行，但额外 tube 净空条件无法确认 | 不宣称有安全 tube | 已在零偏移时可按原 baseline 权限运行，并明确标注非论文 tube-certified 模式 |
| 当前 delta 非零，新 K 不包含它 | 不直接接入/不清零 | 在旧安全域有界恢复或使用有效接续 |
| UNKNOWN、几何能力缺失、预算耗尽 | 未知，不等于物理碰撞 | 不因认证失败永久 HOLD；也不得虚构安全的非零执行 |
| 新障碍侵入实际执行区域 | 旧 tube 不再适用 | 使用原安全停止/重规划机制 |
| 当前路径末端覆盖不足 | 触发刷新/接续 | 不静默夹持 w，也不无限沿过期路径执行 |
| 终端到达/水平切向退化 | 停止 transverse 协调或进入明确 terminal | 保留原到达与停止语义 |

“tube 条件比 baseline 严格”与“baseline 本身碰撞”必须分开。原导航降级不等于保持论文安全声明。每次退化记录原因、开始结束时间和权限模式。

初始 acquisition 同样受此约束：若真实 UAV 尚未进入选定机体/跟踪误差预算覆盖的参考邻域，即使 delta=0 且 I 已算出，也不能宣称物理机体已获得 tube 安全保证。先按原 acquisition 导航，满足跟踪预算和当前可行性后再授予协调权限。误差超预算的监测和处置复用原执行接口，不改低层控制律。

不新增庞大状态机：执行只需表达 nominal/coordination、必要的 recovery、原安全停止；构建结果和接入原因以数据状态表示，不为每个 proof gate 新建模式。

## 9. 文件改动范围与退役策略

以下是候选清单，不是本轮编辑许可。M0 必须解析真实 include/link 闭包、逐项确定增改删和测试替代，再冻结具体路径；未列出或未确认的文件不得自行扩大修改。

约定 P=`W/src/swarm_planner`，N=`P/phase_offset/phase_offset_navigation`，R=`P/bspline_traj`，E=`P/plan_env`。

### 9.1 核心修改候选

| 具体文件 | 允许的目标行为 | 不允许的附带改动 |
|---|---|---|
| `E/src/sdf_map.cpp`, `E/include/plan_env/sdf_map.h` | 恢复 baseline 数值行为、局部读取与简单同步、移出 tube 证书逻辑 | 改原膨胀/清图规则来让测试成功 |
| `E/CMakeLists.txt` | 去除旧生产快照依赖，接入新局部接口 | 重排无关包依赖 |
| `R/src/continuous_phase_path.cpp`, `R/include/bspline_race/continuous_phase_path.h` | 实际几何局部界、结构断点接口，退役 proof-only 耦合 | 修改名义 p(w)、弧长反解结果和原 C2 算法 |
| `R/src/continuous_phase_normal_frame.cpp`, `R/include/bspline_race/continuous_phase_normal_frame.h` | 必要的局部界/导数能力，保持 normal 定义 | 偷换 N(w) 或增加垂直自由度 |
| `R/src/gvf_manager.cpp`, `R/include/bspline_race/gvf_manager.h` | 同步规划编排、配套 bundle、移除 tube 专用 timer/request 逻辑 | 塞入几何算法、改 GVF/规划增益 |
| `R/src/integration/phase_offset_matched_adapter.cpp`, 对应 include 头 | 精简接入与状态绑定，接新 profile | 改 matched 注入关系、跳过发布成功提交语义 |
| `R/src/integration/phase_offset_recovery_continuation_provider.cpp`, 对应 include 头 | 验证/适配 active-reference 连续接入 | 未验证就宣布任意非零 delta C2 |
| `N/src/tube_viability.cpp`, 对应 include 头 | 保留 preview 算法，移除 V2 provenance 耦合 | 去掉向内收缩限制 |
| `N/src/phase_offset_allocator.cpp`, 对应 include 头 | 同一数学分配接新 preview，保留 ZOH/速率约束 | 绕过 infeasible、额外直接加 SPH 速度 |
| `N/src/phase_offset_runtime.cpp`, 对应 include 头 | 适配 bundle 和单一状态所有权 | 添加第二个 w/delta owner |
| `N/src/phase_offset_recovery_owner.cpp`, 对应 include 头及 `recovery_prepared_step.h` | 去除旧证书依赖，保留必要恢复 | 一律立即回零 |
| `R/src/integration/phase_offset_tube_markers.cpp`, 对应 include 头 | 横向参考带、I/K、实际采用状态显示 | 画不存在的三维容量或把 candidate 标为 active |
| `N/CMakeLists.txt`, `R/CMakeLists.txt` | 新唯一生产后端与测试迁移 | 新旧后端在生产互相 fallback |

上表“对应 include 头”具体位于现有 `include/phase_offset_navigation/` 或 `include/bspline_race/integration/` 下，文件名与 .cpp 同名。冻结清单时必须展开成具体路径，不将此表作为通配授权。

### 9.2 建议新增的有限模块

- `E/include/plan_env/local_obstacle_view.h`、`E/src/local_obstacle_view.cpp`：地图侧值对象/局部提取。若已有轻量接口能满足则复用，不重复新增。
- `N/include/phase_offset_navigation/section_tube.h`、`N/src/section_tube.cpp`：新截面 builder、局部平面与连续检查；纯 Eigen/STL。
- `R/include/bspline_race/integration/phase_offset_section_tube_bridge.h`、对应 src：地图/路径适配和 bundle 类型，不拥有 worker。

只为当前实现需要的职责建文件；不能照着旧 V2 再拆出十几个证书/authority 类。类型、函数与预算字段在 M0 形成编译级接口草案后冻结。

### 9.3 退出生产路径的旧组件

- `R/src/integration/phase_offset_tube_runtime_v2.cpp` 及头：独立 worker、cohort/pinned request 机制。
- `R/src/integration/phase_offset_tube_v2_diagnostics.cpp` 及头：其专属事件链。
- `R/src/integration/phase_offset_cloud_occupancy_query.cpp` 及头：V2 capture/free-ball bridge。
- `N/src/tube_certificate_v2.cpp`、`tube_execution_v2.cpp` 及相应头、`tube_profile_v2.h`：旧几何证书/执行绑定。
- `E/src/cloud_occupancy_snapshot.cpp`、`sdf_map_environment_evidence.cpp` 及相应头中的旧专属能力：确认全部生产消费者退出后再退役。
- `tube_builder`、`certified_tube_builder`、`tube_surface_validator`、`tube_epoch_manager` 等旧 legacy 链：不得重新接入新生产流程；历史测试去留单独登记。
- adapter、allocator、runtime 中残留的旧字段也要同步迁移，不能用伪造全为 1 的 ID 维持旧接口。

旧模块先在完整原状态归档后脱离生产；是否保留为非导出离线回归 target，以还能提供独立价值为准。删除测试必须列出原断言对应的新行为测试，不能以删测试代替修复。

### 9.4 受保护与条件范围

- `phase_offset_core` 的原数学代码、`gvf.cpp`、`bspline_opt_3d.cpp`、搜索器、governor、SO3、SPH 消息/算法默认只读。
- `phase_offset_core/path_state.h` 的 proof-only 类型如需清除依赖，先证明不影响数学类型并单独列入执行清单；不全包授权。
- `R/launch/test_gvf.launch` 只在集成阶段删除无效旧 tube 参数或增加明确新字段；不改用户 circle 设置和安全增益。
- `src/uav_simulator/dynamic_map_generator/src/local_sensing.cpp` 仅允许经 M0 比较后恢复/迁出 tube 专属契约；不得顺手改变仿真点云分布或感知范围。
- B、S、论文附件、其他 worktree、旧 frozen plan 和用户进程全部保持只读。

## 10. 分阶段实施与放行条件

### M0：基线、能力和规格冻结（先做，未通过不改功能代码）

输入：本计划经用户确认后的明确阶段授权。

工作：

1. 记录 branch/HEAD/status、完整 tracked/untracked 源状态、配置、编译环境和已有测试结果；保护 prototype stash。
2. 比较 B、`9a0e975`、W 的地图和执行差异，分类为原功能、必要多机适配、tube 侵入、未知差异。
3. 保存可复现的路径/地图固定输入、实际有效参数和安全距离账本。
4. 核对真实路径几何界及 normal 高阶能力；决定第 5.5 节的可实现界方法，列明误差和数值容差。
5. 画出生产依赖闭包、现有状态所有权和锁顺序，冻结具体文件增改删清单及接口。
6. 选定隔离开发位置；若需新 worktree/分支，先取得用户明确授权。不能仅用 HEAD 创建副本而漏掉未提交修改。
7. 冻结场景、硬件/负载、误差阈值、CPU/时延预算和回滚产物位置。

输出：可执行 M1/M2 规格、`baseline_manifest`、`clearance_ledger`、`dependency_and_lock_map`、固定输入测试数据说明。输出路径在该阶段执行规格内明确，不由实现者到处生成临时计划。

放行：关键语义无 TBD；无法提供真实路径连续界时不得先宣传新 tube 安全。

### M1：单机基线隔离与地图薄接口

目标：phase-offset 关闭时恢复原行为与轻量运行；建立新 builder 可使用的局部数据接口。

先断开旧生产调用，再恢复地图接口和依赖；不要先覆盖 sdf_map.cpp 导致全工程失配。该阶段可在隔离候选版本中暂不提供新 coordination 生产功能，但不能把这种中间态部署成已完成主版本。

测试：相同输入下 effective occupancy、ESDF、manual/static/boundary、cloud/depth、空点云/清图等回归；关闭 phase-offset 的原导航及到达测试；局部 view 一致性和并发读取测试。

验收：关闭模式没有旧 snapshot/support/tube worker 开销；原行为差异逐项解释，不能用“测试通过”掩盖地图语义变更。新增适配成本单独计时。

### M2：离线横截面 builder

目标：不接 ROS 控制，完成单一新几何内核及固定输入回放。

工作：种子局部化、体素/安全平面、截面解析、非对称正则性、连续带检查、有限预算和原因码。

验收：第 11 节 G 系列全部通过；独立几何 oracle 无安全漏检；直墙解析场景宽度误差在冻结容差内；薄障碍与曲线接缝无遗漏；性能在相同输入和有效覆盖质量下评估。

不以截面 marker 好看作为通过。若保守内近似在预定急弯/通道用例里失效，回到算法设计审议，不能临时加旧 free-ball 后端兜底。

### M3：Preview 与单机闭环接入

目标：将新 profile 接入现有数学执行，去除 tube 专属 worker/安装链。

工作：规划内同步构建、current/candidate bundle、短事务替换；同路径刷新、非零 offset 的路径接续；preview、ZOH、matched selected-u 一致性；明确失败处理。

验收：第 11 节 P/H 系列通过；有界延迟注入不阻塞 50 Hz 控制；重复静态点云不导致接入饥饿；新目标/reset 不采用旧候选；构建失败不虚构 zero-only。

该阶段前必须有 M1 的原单机回归和 M2 的几何通过。不是同时重写 SPH 的阶段。

### M4：多机与论文效果验证

顺序：N=1 可控横向输入 → N=2 窄通道/错位 → N=5、10 → 批准后按资源扩大至论文规模。

保持 SPH 公式与原控制参数，在一致地图和初始条件下测量。先证明行为机制，再报告规模与统计效果；不能直接从单机通过跳到“50 UAV 已实现”。

验收：第 12 节实验、距离/误差/时间数据与失败归因完整。达不到预期则报告几何、控制、通信或拥堵原因，不通过放宽安全距离完成指标。

### M5：清理与交付

确认生产仅有一个 tube 构建实现；移除无消费者字段/参数/旧 marker 和测试胶水。保留有价值的历史证据，不在源码目录复制多个快照版本。

输出最终文件清单、完整测试报告、性能分解、已知限制、回滚说明和“未授权范围未修改”核对。论文正文修改另行授权。

每阶段需主会话审核并获得后续执行授权；`AUTO_ADVANCE=false`。计划文档本身不授权自动走完整条路线。

## 11. 验收测试矩阵

### 11.1 地图与原基线

| ID | 场景 | 必须成立 |
|---|---|---|
| B01 | phase-offset 关闭，同输入重放 | 原地图/规划/执行行为保持；无 tube 专属计算 |
| B02 | manual、static、ceiling、map boundary | 所有有效障碍层进入局部 view，无漏层/幻影层 |
| B03 | cloud/depth、空观测、清图、域外 | 保留原地图行为；unknown 不变 free |
| B04 | 并发更新与读取、目标 reset | 无数据竞争/半帧视图/悬空引用，锁不覆盖重几何 |
| B05 | 同内容静态点云重复、无关 ROI 变化 | 不因消息编号更新废弃相同有效几何 |

### 11.2 几何

| ID | 场景 | 必须成立 |
|---|---|---|
| G01 | 无障碍直线、有限已知域 | 截面由搜索上限和已知域决定；非强制固定小宽度 |
| G02 | 单侧平墙 | 左右非对称，解析边界可独立计算 |
| G03 | 双墙渐缩渐扩 | I 跟随空间；没有把 preview 收缩写回 I |
| G04 | 稀疏节点间薄障碍 | 连续检查发现，不跨 gap 插值 |
| G05 | 急弯、弦穿障碍但曲线绕开 | 局部细分或合理缩段；不直接判整路径碰撞 |
| G06 | 曲率正负、近奇异和外弯宽侧 | 二次正则性正确选分支，不对称误剪 |
| G07 | 多区域交集、接缝、相离区间 | 不取跨障碍凸包；共同端点和段间连续 |
| G08 | 地面/天花板靠近机体，z 随路径改变 | 考虑实体安全邻域，无垂直 offset |
| G09 | seed/voxel 切触、alpha≈0、退化线段 | 数值处理稳定且保守，不用大 tolerance 放宽安全 |
| G10 | 未知域、ROI/halo 边界 | 未观测不当空闲，完整输入才能声明安全 |
| G11 | 中心线不足净空、zero-only、无高阶界 | 三种状态明确区分；不把失败伪装成 [0,0] |
| G12 | 预算耗尽、极短 prefix | 有界退出；有效范围真实；不算任务成功 |

独立 oracle 不能调用待测平面 builder 得到“真值”。平墙/圆弯使用解析值，体素场景使用独立 segment/box 或连续曲线距离检查；高密采样只作补充可视证据。

### 11.3 Preview 与执行

| ID | 场景 | 必须成立 |
|---|---|---|
| P01 | 手工可算的宽→窄区间序列 | 反向递推与解析结果一致 |
| P02 | 非均匀网格、窄点位于普通采样间 | 合并几何节点，连续 K 在 I 内 |
| P03 | 不同 upper_nu/u_delta、零宽域 | 收缩速度可解释；无虚假“总可行” |
| P04 | 当前 delta 位于边界、ZOH 跨节点、dt 抖动 | 整个 tick 可行；slew 交集为空明确失败 |
| P05 | 新 K 突然收缩、滚动 preview | 不裁状态，不套用静态包络结论 |
| P06 | invalid preview | beta 标记未计算，不能用默认零解释狭窄 |
| H01 | 相同路径 tube 刷新 | p、N、delta 不跳，当前段可执行 |
| H02 | 非零 delta 路径替换、共同前缀 | 活动参考位置/速度连续；不能只检查 p 的 C2 |
| H03 | 无共同前缀、回零不可达 | 不瞬移、不无限重试，明确安全处置 |
| H04 | 人为延迟构建、旧路径末端 | 控制线程不等待几何；超时和覆盖不足及时处理 |
| H05 | 新目标/reset/配置重初始化 | 老 bundle 不提交；无第二套状态 owner |
| H06 | 发布失败、selected-u 限幅 | 无提前状态提交；两路使用同一最终 u |
| H07 | tube unavailable 但零偏移原导航有效 | 不因缺少横向认证永久 HOLD，权限模式诚实 |
| H08 | 原路径实际碰撞或跟踪误差超预算 | 不以 baseline fallback 名义继续宣称安全 |
| H09 | 终点不足固定 preview 长度 | 正确进入真实 terminal，不因覆盖公式阻止到达 |
| H10 | 初始 UAV 远离参考、候选完成时已越过共同前缀 | acquisition 不伪装已获 tube 保证；不错误接续 |

现有 `manual_map_layer_test`、`continuous_phase_path_test`、`continuous_phase_normal_frame_test`、`gvf_switch_policy_test`、`phase_offset_matched_adapter_test`、`phase_offset_recovery_continuation_provider_test`、`phase_offset_recovery_owner_test`、`phase_offset_allocator_test`、`runtime_test` 应逐项映射到新接口并保留数学/安全断言。新增建议为 `local_obstacle_view_test`、`section_tube_test`、`section_tube_integration_test`；不先搭建无实际测试内容的框架。

## 12. 论文效果与性能验收

### 12.1 论文效果

| 要验证的效果 | 对照/输入 | 指标与通过含义 |
|---|---|---|
| 横截面物理正确 | 平墙、收缩通道、随机体素 | 解析场景边界符合容差；复杂场景内近似安全且记录宽度损失 |
| 提前回收偏移 | 同路径/同输入，有 preview 与仅当前 I | 提前回收时间、峰值横向速率、包络越界；预期提前量不伪造 |
| tube-aware SPH | 相同随机种子，beta 调节与不调节对照 | 密度、横向展开、通过顺序/拥堵、机间距离 |
| matched 属性 | 相同几何/协调输入，matched 与直接叠加工作空间协调速度对照 | `e_dot-(f_x^a-r_w f_w^a)` 残差和实际跟踪误差 |
| 路径接续 | 非零 delta 期间重复重规划 | 活动参考跳变、恢复占空比、到达率 |
| 扩展性 | 固定环境密度与 N 分组 | 每 UAV 及总 CPU、控制 jitter、最小距离、成功率 |

离散导数残差需区分数学核误差、采样差分误差和低层执行误差。碰撞距离从独立地图/仿真真值测量，不能只报告 planner ESDF 或 tube 估计。

SPH 不是严格的无碰撞证明；tube 安全也不保证机间安全。狭窄通道的纵向排队、phase 速率饱和和 stale neighbor 需要实验验证。失败不得归零统计。首版静态地图成功不代表真机感知、动态障碍或论文 50 UAV 指标已经达成。

### 12.2 性能

必须分解：局部地图捕获、障碍筛选、分离平面、截面求交、连续检查/路径界、preview/分配、提交等待、控制回调时间。统计 P50/P95/P99/max、控制 deadline miss、过期/部分 profile 比例和有效覆盖长度。

50 Hz 对应 20 ms 控制周期；这是控制 deadline，不要求规划和完整几何构建都在同一 20 ms 内。正常 steady-state 的 preview+分配新增耗时建议目标 P95<=1 ms，单 UAV tube 更新建议目标 P95<=20 ms；它们是待 M0 冻结的工程目标，不是已实测承诺。真正硬条件是构建不阻塞控制、有效覆盖满足第 7.2 节、整个命令回调在已冻结测试负载下满足 deadline。

对照必须相同地图语义、安全余量、路径、搜索半宽和最低有效覆盖；缩小 ROI 而遗漏障碍、缩短返回区间、少做安全检查不算性能提升。关闭调试日志和 sanitizer 后测 release 性能，sanitizer 用于正确性回归。也记录开关 marker 的开销，避免将可视化时间算成几何缺陷。

## 13. 构建、验证、回滚与停止规则

本轮不执行下列构建命令。经批准的候选工作区可使用现有 ROS Noetic/catkin 工具，示例：

```bash
source /opt/ros/noetic/setup.bash
# 在已批准的候选 workspace 下运行，不 source 多个混杂 overlay
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON
catkin_make run_tests
catkin_test_results build/test_results
git diff --check
git status --short
```

实际目标名以该阶段 CMake 导出的名字为准。全套历史测试中既有失败要在 M0 记录并解释，不能把未运行、编译失败或跳过算通过。数学/几何新模块运行适用的 ASan/UBSan；并发适配做锁序/竞争检查，不能因为全 ROS TSan 噪声多而忽略自己新增的共享状态。

运行测试使用独立 ROS master、命名空间和日志位置；不得连接/停止用户仿真。不得运行仓库 `run.sh` 来代替隔离测试，该脚本会启动外部设备/工作区相关程序。

回滚：实现开始前保存完整未提交状态和构建/参数基线；每阶段保留可复现检查点。失败时停止使用候选版本，通过已验证的原工作区/产物恢复运行。文件级撤销只对本任务明确拥有且有精确备份的修改执行；不使用 reset --hard、checkout --、restore .、git clean、覆盖整个目录或 stash pop。

必须停止并报告：安全账本无法对齐、真实路径连续界缺失、需要改变原 planner 数学、文件清单越界、活动参考接续无法满足、需要新的感知/动态安全系统、基线出现无法解释的退化、用户工作区发生并发变更。

阶段完成不等于自动授权下一阶段；本轮完成仅意味着计划和自审文档交付。

## 14. 自审后修订摘要与待冻结项

本会话自审重点修正：

1. 不再把完整 CIRI/三维走廊作为用户横截面需求的必要前置。
2. 连续检查明确依赖真实路径界，不能把离散采样包装成证明。
3. 正则性节点解析之外仍有整段条件；N_ww 不是现有接口自动具备。
4. 规划同步不等于控制同步；已有 AsyncSpinner 下仍需短锁和 generation。
5. 原 p 的 C2 不等于非零 offset 活动参考 C2；首选共同前缀，避免每次重规划都回零。
6. map support ID 可删除，地图一致读取、未知空间和真实变化检查不能删除。
7. ZERO_ONLY 必须真实可行，baseline 退化不能冒充论文 tube 安全。
8. 动态 K 和离散 tick 的可行性列入验收，不只引用连续时间 clip。
9. 不改变安全距离来掩盖几何或性能问题，不保证 SUPER-style 内近似最大宽。
10. 新算法通过前，旧方案先隔离而非在主工作区破坏性清除。
11. 巡航前视覆盖公式不用于要求真实终点之后仍有路径；initial acquisition 与跟踪预算明确区分。
12. builder 输出为论文可行集的内近似，质量验收不错误要求所有复杂场景恢复最大宽度。

M0 待冻结项：安全距离账本；真实路径连续界实现与误差；地图已知域来源；精确增改删清单及锁顺序；硬件/负载/性能和数值容差；现有非零 offset handoff 的实证能力。这些是有明确输出的前置审计，不授权开发者临时扩大算法。

本计划自审结论：架构与用户最新意图一致，可供确认和启动经授权的 M0；尚不能把本文直接交给执行者自动完成 M1–M5。详细发现、已处理风险和验证边界见关联自审记录。
