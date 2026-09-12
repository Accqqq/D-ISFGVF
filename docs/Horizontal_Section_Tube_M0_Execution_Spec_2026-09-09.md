# 水平横截面 Tube：M0 执行规格

日期：2026-09-09

授权依据：用户要求按照已讨论计划执行，并明确编码交给刚建立的 `gpt-5.6-luna / max` 子代理，主会话监督调度。

## 1. 本批次边界

本批次只执行计划 M0：基线、地图语义/依赖、路径几何/接续能力以及隔离测试核对。主会话拥有设计、规格、评审与验收；Luna 执行具体代码任务，不自行改变架构或扩大文件范围。

当前源基线 HEAD 为 `20d44c94d4165e35621cc30269e3fa0b2a5f10ff`，branch 为 `pro_review_current_20260903`。旧计划 HEAD 后的 checkpoint 已提交之前的未提交源修改；src 指纹仍为 `3658d5a20a004b7bf2cb6f7edf5739b8505332c44c6432fcfd2c1830ca9a4efc`，不是功能漂移。tracked diff 为空；已有未跟踪文档/测试缓存归用户所有。

## 2. 精确写入范围

主会话文档：

- 本文件。
- `docs/Horizontal_Section_Tube_M0_Report_2026-09-09.md`。

M0 诊断产物目录：

- `.horizontal_section_refactor/m0_20260909_01/`。
- 仅允许源状态归档、清单、编译对象、测试二进制、测试输出和下述诊断程序；不允许在此藏入生产实现替代品。

Luna 的编码文件仅为：

- `.horizontal_section_refactor/m0_20260909_01/path_geometry_probe.cpp`。

不编辑 `src/`、原 `build/`、原 `devel/`、launch、baseline、SUPER、其他 worktree、旧计划和用户运行会话；不创建 git 分支、提交或 stage，不重置工作树。

## 3. 只读核对任务

Luna：比较用户 baseline、历史 9a0e975 与当前地图文件、配置和调用闭包，报告可成对恢复文件、有效占据层、膨胀/epsilon/m_r/跟踪误差读取、局部读取能力/线程风险、已知自由域来源。只报告事实，不另写架构。

主会话：核对真实参数化的几何界、共同前缀接续与锁顺序，隔离编译既有源测试，汇总差异和阶段放行条件。

## 4. 诊断程序的固定行为（主会话下发后才编码）

诊断程序不使用 SDFMap、tube V2 builder 或 ROS master；只使用当前源的 ContinuousPhasePath、ContinuousPhaseNormalFrame 和原 UniformBspline。

固定测试输入：

1. 水平直线的 quintic Hermite，w∈[0,2]，p=(w,0,1)，一阶导数=(1,0,0)，二阶导数=0。
2. 七个控制点的三次 B-spline：`(0,0,1),(.4,0,1),(.8,.2,1),(1.2,.5,1),(1.6,.9,1),(2,1.1,1),(2.4,1.2,1)`，knot interval=0.2，实际 spline.t_range 映射至 w∈[4,7]。
3. 用 appendSlice 从输入2复制 [4.2,4.8]，逐点比较旧/新 p、p_w、p_ww、N、N_w，在 delta=0.3、w_dot=1、delta_dot=0.1 下比较 r 和 r_dot；不得只比较端点。

输出每个 cell 的 `w0,w1,legacy_bounds_ok,v2_bounds_ok,q_min,A_xy,A_all,J,N_w_bound,N_ww_derived_bound,point_samples_ok`；普通网格步长0.1，域端点明确处理。记录 cell 查询时间，仅为诊断，不做性能达标承诺。

当 legacy cellBounds complete、q_min>0 且所有上界有限时，可报告推导值：

`B1=A_xy/q_min`，`B2=J/q_min+3*(A_xy/q_min)^2`。

这是 unit-normal 二阶导数的候选保守幅值界；不宣称从已有接口取得 N_ww 点值。M0 程序不建立新安全证明，也不把有限采样通过当作连续认证。

每个 cell 取21个内部/边界样点，核对点值在提供的速度/加速度/normal一阶幅值界内，并分别打印 query failure 与 containment failure。源码查询拒绝的 cell 保留记录，不绕过或把零填充当成功。

程序非零退出条件：直线基本几何错误、copied-prefix 不一致、查询成功的 cell 中发现幅值界违例。非直线 cell query 无法提供界另计数并以报告字段标记，不靠更改源码让返回值变 true。

输出可重复，使用 std::setprecision(17)；不引入新库、不做环境修改、不写数据到源码目录。

## 5. 编译与验证

在诊断目录以当前源码重新编译所需 path、normal frame、UniformBspline 对象和既有 path/normal 测试，避免原 devel 测试与库混用。必要 ROS 编译/链接依赖来自 `/opt/ros/noetic`；不启动 roscore。

当前 devel smoke 的四个测试退出139，且 path 测试binary列14项、当前源码列20项。它仅作为 stale-artifact 线索，未经隔离重编译不能归因于源码，更不授权修复原算法。

验收：准确记录编译参数/源指纹、测试数量/退出码、probe局部界可用比例和共同前缀结果、地图差异与缺口；主会话复查 probe 源码与运行结果。发现源级既有缺陷不擅自修改。

## 6. 停止与阶段放行

M0 报告需明确哪些事实已确认、哪些需要用户决定或后续规格。原 src 指纹和 tracked diff 必须保持不变。

M1/M2 功能修改不在本文件白名单内；必须在 M0 放行条件满足后形成新的精确执行规格，不能借“按照计划”越过尚未确认的安全语义或路径能力问题。
