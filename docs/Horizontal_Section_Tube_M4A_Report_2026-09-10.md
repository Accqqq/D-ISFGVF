# M4A 报告：集群仿真接入水平横截面 Tube

日期 2026-09-10。执行：主会话（Luna 连续两次 stream 断开，零改动退出，按既有
授权由主会话接管编码，仍由主会话独立构建/验收）。
规格：`docs/Horizontal_Section_Tube_M4A_Swarm_Cutover_Spec_2026-09-10.md`。
产物目录：`.horizontal_section_refactor/m4a_20260910_01/supervisor/`。

## 1. 结论

集群仿真已经能真正跑起来并进到新 tube：3 机同时起飞、各自建横截面 tube、
邻居状态平面与 SPH 通道都有数据。**尚未完成**的是集群意图驱动 δ 的最后一段：
agent 侧 `beta` 仍 `valid=false`，因此 provider 的 `g_coord` 无效、δ 恒 0。

## 2. 起不来的直接原因（不是逻辑问题）

隔离 devel 里有两个可执行从来没编译过，roslaunch 报
`cannot launch node of type ...`：

| 缺失可执行 | 影响 |
| --- | --- |
| `so3_quadrotor_simulator/multi_quadrotor_simulator_so3` | 没有任何 `/uav_N/sim/odom`，规划/感知全空 |
| `phase_offset_swarm/agent_state_neighbor_runtime_node` | 无 `AgentState`、无 `g_coord`，集群面全空 |

两者已在 `full_build` 补编译（`m4a_build_msim.log`、`m4a_build_swarm.log`）。

## 3. 代码/配置改动（白名单见规格第 3 节）

- 集群默认改为 SPH-ready：`manual` + `sph` + `observe_only=false` +
  neighbor transport + SPH provider + `cloud_obstacle_set_complete=true` +
  六个 preview 值（否则集群里连 tube 都不会建）。
- 逐机 tube 话题隔离：`/phase_offset_section_tube` 在 agent launch 内
  remap 成 `/uav_N/phase_offset_section_tube`。单机话题名保持不变。
- 共享地图默认 20×30 → 40×60，容纳 `(0, 20)` 起始点。
- 新场景 `sim_b_formation_3.yaml`（起始 `(±1, 20, 1)` / `(0, 20, 1)`，
  间隔 1 m，终点按 SPH-planning 的整体平移方式向南 30 m）并设为默认。
- `rviz/phase_offset_sim_b.rviz` 三机 tube 显示与单机
  `swarm_rviz.rviz` 字段完全一致（三机都打开）。
- `gvf_manager.cpp` 相对名尝试已回退，最终无行为改动。

## 4. 实测证据（隔离 master，场景 `sim_b_formation_3`）

命令：`supervisor/m4a_interface_run.sh 11542 sim_b_formation_3.yaml 30 30`
（任务自有 master，未连接用户 ROS 会话）。

| 观测项 | 结果 |
| --- | --- |
| `/phase_offset/agent_state` | 1802 条，来自 robot 0/1/2 |
| `/uav_N/phase_offset_section_tube` | 每机 30/30 条非空，`points=[27,27,156]`，`types=[4,4,11]`（左右 LINE_STRIP + TRIANGLE_LIST 填充） |
| 三机位置 | x≈-1.18 / 0.04 / 1.00，y≈12.2 / 13.1 / 15.2（从 y=20 出发向南） |
| `/uav_N/phase_offset/sph_beta` | 在流，但 `valid=false` |
| `/uav_N/phase_offset/g_coord` | 在流，但 `valid=false`、`norm=0` |

对照（`sim_b_conflict_3`，1.2 m 间距）结果一致：三机 tube 均建成，
`g_coord` 通道有数据但 valid=false。

## 5. 未完成项与下一步

1. **β 合法性的根因已定位（见第 6 节）**：preview 本身完全可行、β=1.0，
   失败的是 allocator 的相位速率窗口。打通后 δ 才会随邻居变化。
2. `planner_path` 话题（`/uav_N/particle0/path`）采样期间无消息，
   横向偏移统计因此为空；需要确认它在集群下的发布时机再补 δ 量化证据。
3. M3C 审计遗留：扫描构建器补专门单测；报告记录“生产已从逐 box+细分
   builder 换成扫描构建器”这一对冻结规格的偏离。

## 6. β 断点定位（2026-09-10 复盘）

临时诊断（已全部回退，`grep -c known-domain-debug/sph-beta-debug/gvf-hold-debug`
均为 0）在隔离集群上的结论：

1. `allocator_evaluated=1`，allocator **确实在跑**；零门已打开
   （`zero_gate_open=1 warmup=100 failure_latched=0 equivalent=1`）。
2. preview **不是**瓶颈：
   `preview(valid=1 feasible=1 rate=1 contraction=1 inside=1
   status=1(FEASIBLE) beta=1.0000)`。
3. 真正的失败在 allocator：
   `alloc(status=5 valid=0 feasible=0
   reason='phase scalar admissible interval is empty')`。
   `phase_offset_allocator.cpp:457-466` 先算
   `[lower_nu - f_w0, upper_nu - f_w0]` 再与
   `[-u_w_abs_max, +u_w_abs_max]` 求交；实测
   `base_f_w0 = 4.2~4.7`，而 `lower_nu=0.02`、`upper_nu=2.0`、
   `u_w_abs_max=0.12`，于是 `[-4.65,-2.67] ∩ [-0.12,0.12] = ∅`。
4. 失败后 `runtime_execution.mode=WAITING_FOR_CANDIDATE`，
   `publish_sph_beta` 只接受 NORMAL 步，于是 β 发 `valid=false`，
   provider 把 `g_coord` 标 invalid，δ 恒 0。**β 无效是症状不是原因。**
5. 这是自锁：区间空 → hold → 落后参考更多 → `f_w0` 更大（4.2→4.7）→
   区间更空，因此约 4–5 s 后永久 hold。
6. 对照组（同场景同地图，`phase_offset_mode=disabled` +
   `coordination_backend=disabled`）：三机全程有效飞行，最后一条有效命令
   t=20.3 s，终点 odom 正好落在各自目标 `y=-10`。所以这是
   manual+Section 命令路径引入的，不是场景或 planner 本身。
7. 参数口径：`upper_nu` 现为 2.0，而
   `docs/Horizontal_Section_Tube_M0_Report_2026-09-09.md:92` 记录论文示例为
   3.0；`docs/Horizontal_Section_Tube_Refactor_Plan_2026-09-09.md:335`
   明确写“未经明确评审不改变 upper_nu”。因此该项**不自行修改**，
   待用户/评审决定后另行执行。

### 6.1 `f_w0` 为什么是 4.2~4.7（相位更新率来源）

公式（`guidance/isf_reference_kernel.cpp:96`，`gvf.cpp` 同式）：

```
w_dot = k1 * (alpha + sigma(e_parallel)) / ‖dp/dw‖
sigma = tanh(e_parallel / progress_delta)      # progress_delta = 0.3
```

隔离实测（`sim_b_formation_3`，1 Hz 采样）：

| 阶段 | `‖dp/dw‖` | `e_parallel` | `w_dot` | 物理速率 `w_dot*‖p_w‖` |
| --- | --- | --- | --- | --- |
| 健康 (t≈1–3 s) | 1.000 | ≈0 | 1.9–2.0 | 1.9–2.0 |
| 首次失败 (t≈4.04 s) | 0.846 | ≈0 | 2.31 | 1.96 |
| 锁死 (t≥5 s) | 0.846–0.85 | +0.55 | 4.2–4.6 | 3.9 |

关键点：

1. **入口不是追赶项**。replan/C2 之后相位路径不再是弧长参数化
   （`‖dp/dw‖≈0.85`），此时零沿迹误差的标称相位率就已经是
   `k1/0.85 ≈ 2.36`，已经超过窗口上限
   `upper_nu + u_w_abs_max = 2.0 + 0.12 = 2.12` → 区间空 → hold。
   只要路径非弧长参数化，`upper_nu=2.0` 就会被顶穿。
2. **锁死放大**。hold 后相位冻结在 `w=7.45`，而飞机停在相位参考点
   前方 0.556 m，于是 `sigma=tanh(0.556/0.3)=0.95`，
   `w_dot = 2*(0.995+0.95)/0.85 = 4.59`。这是 4.6 的直接来源。
3. **单位修正**：`nu` 是 w 坐标下的速率，物理速度 = `nu * ‖dp/dw‖`。
   `‖p_w‖=0.85` 时 `upper_nu=2.0` 只等于物理 1.7 m/s，窗口比表面更紧。
   论文的 3.0 覆盖标称 `2.36`（`3.0+0.12 > 2.36`），2.0 不覆盖。

### 6.2 参数调优路线实测否决

把 `phase_offset_normal_preview_upper_nu` 临时覆盖为论文值 3.0
（`M4A_EXTRA_ARGS` 覆盖，不改文件），同场景重跑：

- 参数确实生效（三个 agent 的 `phase_offset/normal_preview/upper_nu` 都是 3.0）；
- **仍然卡死**：最后一条有效命令 t≈4.1 s，其后 `GOVERNOR_INVALID_HOLD
  reason=guidance_invalid` 持续到结束；
- 失败时刻度 `‖dp/dw‖ = 0.539~0.647`、`f_w0 = 6.0~7.2`。

也就是说刻度不是 0.85 这种"小偏差"，而是会在不同前端之间从 1.000 掉到
0.54。要覆盖到 0.54 需要 `upper_nu >= 3.6`，而那时自锁已经把 `f_w0`
放大到 7.2 —— 固定 w 单位的上限只能追着刻度跑。结论：
**不要再靠调 `upper_nu` 解决**，要从坐标口径上解决。

### 6.3 根因：C2 连接段的局部刻度塌陷（已定位）

与基线 `/home/cxq/Sim_demo/New_ISFGVF/gvf_ws` 对照后确认：

- `buildGlobalPhaseSamples` 两侧**完全相同**（`scale = (end_w - anchor_w) /
  future_length`）。基线多一道 `required_install_span_w` 拒绝门，但它只拒绝
  过短区间，不改变刻度。
- 平均刻度没有问题：安装时实测 `installed_len / installed_span_w ≈ 1.000`
  （6 次安装分别为 1.003/0.999/0.998/1.003/1.000/1.001）。
- 问题在**局部**。每次安装时沿约 5.1 w 的前端均匀采样 12 点的 `‖dp/dw‖`：

```
1.085  1.077  0.276~0.416  0.938~0.972  1.085 1.085 ... 1.085
  ↑       ↑         ↑            ↑            ↑
 前缀(旧路径切片)  C2 连接段     过渡      映射样条
```

前端由三段拼成：前缀 + `c2_quintic` 连接段 + `mapped_bspline`。前缀和映射
样条都是弧长映射（常数 1.085），**只有连接段塌到 0.28~0.42**。原因：连接段
被固定分配 `join_delta_w`（0.5~1.0 搜索常数，实测恒为 1.000）个 w，但它的
物理长度只有约 0.3~0.4 m（五次 Hermite 连接两个本来就挨得很近的点），于是
局部"米每 w"只有 0.28。

后果链条：

1. 相位一旦走进连接段，`f_w0 = k1*(alpha+sigma)/0.28 ≈ 7`；
2. 任何以 w 为单位的窗口（2.12 或 3.12）都容不下；
3. allocator 判无解 → hold → 相位冻结在连接段里 → 出不来；
4. 冻结时实测相位只在前端 `frac = 0.105~0.111`（远不是末端），
   局部 `r_w = 0.851~0.915`，正是连接段所在区域。

每次 replan 都会在当前相位前方约 0.4 w 处新建一个连接段，所以相位一重建就
很快撞上它，这就是"t≈3.9 s 突然崩"的来源。

为什么基线不崩：基线没有 PhaseOffset allocator。那边 `w_dot` 只用于推进
相位，物理速度由 `v_cmd` 独立给出，连接段的局部小刻度只是让参考点走得慢，
不会判定"没有可行命令"。把这个局部小刻度变成致命的是 PhaseOffset 的
"以 w 为单位的速度窗口"这一假设。

### 6.4 判定实验：凹坑来自 w 跨度与弧长不匹配（方案一成立）

在隔离构建里临时把 C2 连接段的搜索范围放大到 `h ∈ [0.5, 2.5]`（步长 0.25），
对每个候选测**连接段自身的弧长 L** 和它内部的 `‖dp/dw‖` 剖面，共 630 条记录：

| `L / h` | `min ‖dp/dw‖` | `mean ‖dp/dw‖` | 判读 |
| --- | --- | --- | --- |
| 0.674 | 0.352 | 0.676 | 深凹坑（就是线上那种塌陷） |
| 0.877 | 0.730 | 0.879 | 中等凹坑 |
| 0.993 | 0.987 | 0.993 | 无凹坑 |
| 1.007 | 1.000 | 1.006 | 无凹坑 |
| 1.325 | 1.008 | 1.326 | 无凹坑（整段偏快） |

结论：

1. 凹坑**只**在 `L/h < 1`（连接段被塞进过宽的 w 跨度）时出现，深度基本等于
   `1 - L/h`；当 `L/h ≈ 1` 时 `min ≈ mean ≈ 1`，凹坑完全消失。
   所以**方案一确实能治根因**，我之前担心的"C2 二阶约束必然产生 S 形低速段"
   不成立。
2. 但必须用**不动点**实现，不能靠现在的 0.25 步长离散搜索：一致点通常不在
   网格上。实验里加硬过滤（`|L/h - 1| <= 0.03`）后，64 次安装因找不到候选而
   失败（`connector_failed`），飞机仍然卡住 —— 这验证了"要解 `h = L(h)`"。
3. 次要项：健康段本身是 1.085 m/w（比 1 高 8%），窗口 `[0.02, 2.0]` 换算成
   物理速度是 `[0.02, 2.17]` m/s，对 2.0 m/s 巡航只剩 9% 余量；任何
   `e_parallel >~ 0.1 m` 的瞬态都会让 `w_dot` 越过 2.12。所以窗口余量
   （论文的 3.0）与连接段自洽是**两件都要做**的事。

### 6.5 连接段自洽修复：已实现、已实测，本配置下无可行解（已回退）

按指示只改"连接段自洽"这一项：给 C2 候选加过滤（连接段自身弧长不得小于
它的 w 跨度，容差 5%），并在调好的区间无解时沿几何阶梯向下扩展
（0.5 → 0.05）。隔离实测结果：

- 过滤本身工作正常：每次安装尝试 14 个候选，其中 7 个被判"被压扁"
  （`L/span < 0.95`）并被拒绝 —— 正是原来那种 `L/h = 0.67~0.88` 的候选。
- 通过自洽过滤的另外 7 个候选，**全部被现有折角门拒绝**
  （`tangent_dot < -0.2`，相邻采样切向反向）。
- 曾怀疑折角来自采样混叠（连接段 w 跨度小于采样步 0.05），于是加了
  "跨连接段时改用连续路径细分检查"的版本重测：折角在连续路径上依然存在，
  说明它是几何的，不是采样伪影。
- 结论：开启自洽过滤后 `connector_success = 0`、`connector_failed = 12/次`，
  安装全部失败。该改动已整体回退；回退后实测
  `connector_success=3 / connector_failed=0`，行为复原。

机理：自洽要求 w 跨度≈连接段物理长度，而该 seam 上新样条起点与旧路径状态
之间**弦长与切向都不一致**。跨度小到不被压扁时，五阶 Hermite 要在一小段 w
里塞下更长的弦、同时匹配两端一阶/二阶导 → 打结（切向反向）；跨度大到能
平滑转弯 → 必然被压扁、`|dp/dw|` 塌陷。所以瓶颈在 **seam 本身**，不在
"给连接段多少 w"。

下一步候选：(a) seam 选点同时考虑位置与切向，使新旧路径在 seam 处几乎同向，
连接段几乎不用转弯（最小、最贴合根因）；(b) 连接段改到弧长域构造后再映射
到 w，使自洽成为构造保证。

### 6.6 方向修正后：集群真的飞起来了（连接段改动保留）

6.4 的判定标准是对的，但 6.5 把它**实现反了方向**：6.4 的数据显示 `L/h`
随跨度增大向 1 收敛（h=0.5 时 0.46~1.67，h=2.5 时收敛到 0.92~1.21），所以
被压扁的几何，自洽跨度在**上方**。修正为向上扩展后（上限 2.5 w）实测：

| 容差 | 结果 |
| --- | --- |
| 5%（严格自洽） | 上/下两个方向都无解：向下会让连接段打结（折角门拒绝），向上要到 h≈4 才够，而前端总跨度约 5.1 w，会被连接段吃光。 |
| 10%（允许轻微压扁） | 安装恢复：`connector_success` 44~114 次、`connector_failed` 26 次；连接段比例落在 0.90~0.95。 |

飞行效果（隔离 3 机，30 m 编队航线，k1=1.2）：

- 改动前：三机在 y≈14.3 冻结（只飞了 3~5 m），永久 hold。
- 改动后：三机从 y≈9.3 一路飞到 **-6.1 / -6.2 / -9.99**，其中一台**到达目标**
  （目标 y=-10），走廊 tube 全程正常发布（165~190/190）。

所以"连接段自洽"这一项本身是有效的，前提是配合一个可承受的残余压扁容差
（k1=1.2 时 `w_dot = 1.2*(1+sigma)/ratio`，ratio≥0.90 仍有窗口余量）。

残余问题（下一步）：仍有 agent 在离目标 2~4 m 处重新 hold，`sph_beta` 仍为
invalid、`g_coord` 仍为 0（δ 仍不随邻居变化），说明第二个瓶颈还在——窗口余量
（`upper_nu`）与任务末端相位域二者之一或两者。

代码状态：改动保留在 `buildPhaseC2Frontend`（连接段长度测量 + 10% 容差 +
向上扩展 + `join_span_ratio` 日志）；所有临时诊断已移除，干净重编，二进制内
无 debug 字符串。

### 6.7 残余停顿的定位：判据用错了统计量

对 6.6 之后仍会停顿的时刻做打点（临时 `[end-hold-debug]`，已回退）：

| 观测 | 值 |
| --- | --- |
| 局部刻度 `‖dp/dw‖` | **0.462 ~ 0.479**（仍被压扁，只是不再是 0.28） |
| 基础相位速率 `f_w0` | 4.2 ~ 4.3（= `k1(1+sigma)/r_w`） |
| allocator | `status=5`，`phase scalar admissible interval is empty` |
| preview | `status=1`（FEASIBLE，走廊没问题） |
| `delta` | -0.324 / +0.071（**已经出现非零横向偏移**） |
| 相位所在前端 | `[22.2, 27.3]` 内某点 |

所以我们面对的仍是**同一个机理**（局部刻度塌陷 → 相位速率需求超窗口 →
判定无解 → 冻结），只是从 0.28 缓解到了 0.47，还没回到 1.085。

关键修正：我设的 10% 容差作用在**平均值** `L/span` 上，而致命的是
**最小值** `min ‖dp/dw‖`。6.4 的表里正好有对照：

| `L/span`（平均） | `min ‖dp/dw‖` |
| --- | --- |
| 0.877 | 0.730 |
| 0.674 | 0.352 |
| 0.993 | 0.987 |

最小值掉得比平均快得多，所以"允许 10% 平均压扁"实际放行了 50%+ 的局部塌陷。
另外，压扁还会通过 **prefix（旧路径切片）继承**：相位一旦停在被压扁的区域
里，之后每个新前端都会把这一段拷进来，于是永久卡住。

正确的判据应该是"沿**整条已安装前端（含前缀）**的 `min ‖dp/dw‖ >= 1 - tol`"，
而不是连接段的平均 `L/span`。更彻底的做法是把连接段构造在**弧长域**（先做
弧长 C2 连接，再映射到 w），这样 `r_w` 处处等于周围路径的刻度（1.085），
既不压扁也不打结，也不需要容差与跨度搜索。

### 6.8 弧长域构造被否决 + 真因是"向后搭桥"

**弧长域构造这条路不成立。**给定接缝两端六个 w 域条件（`p`、`dp/dw`、
`d²p/dw²`）与跨度 h，五次 Hermite 是唯一的：常数尺度映射与现构造逐字等价；
尺度随 w 变化才能消除压扁，但那样 `p(w)` 变成十次多项式，而 cell certificate
按六系数五次写死（`MakeQuinticCertificate` / `MakeQuinticV2Certificate`），
要支持十次就得泛化整条已验收的证书链。所以**压扁不是参数化问题，是边界
条件与跨度本身的问题**。

接着打了接缝对齐的点（87 次安装，`[seam-align-debug]`，已回退）：

| 量 | 结果 |
| --- | --- |
| 接缝处旧路径切向 vs 新样条切向夹角 | **恒为 0.0°** |
| 沿旧切向的有符号位移 `along = dot(p_new - p_old, t_old)` | 中位数 **-0.49 m**，范围 [-1.97, +0.16]，**87 次里 59 次为负** |
| 弦长 / 跨度 | 中位数 0.98，最小 0.024 |

结论：切向是对齐的，但**新样条的端点经常落在接缝点的后方**——连接段要"向后
搭桥"，而两端都要求向前切向，于是必然在跨度内绕环，局部 `‖dp/dw‖` 因此塌陷。
根源是 M3C 引入的 "future seam"：接缝取在当前相位**前方**，而新规划是从飞机
当前位置开始的，两者方向关系倒挂。基线没有这个问题——基线的 C2 连接段从
`phase_at_switch`（当前相位点）开始，新规划也从飞机当前位置开始，端点始终
有序向前。

按这个发现把判据换成"最小局部刻度 + 禁止向后搭桥"后实测：**安装全部失败**
（`connector_success=0`、`connector_failed=12/次`，三机退回冻在 y≈14.3），
说明判据层面已经救不了，必须改 seam/锚点本身。该实验已回退，代码恢复到
6.6 版本（平均比例 10% 容差 + 向上扩展）。

恢复到 6.6 版本后的最新成绩（3 机，30 m 航线）：`connector_success=101`、
`failed=25`，**两台到达目标（y = -9.99 / -9.97）**，第三台在 y = 0.76。

### 6.9 M4B：映射锚点对齐后，三机全部跑完并点亮集群意图

按 `docs/Horizontal_Section_Tube_M4B_Seam_Anchor_Align_Spec_2026-09-10.md`
只改一处：`buildPhaseC2Frontend` 中把候选样条的映射起点从 `future_switch_w`
改为与样条几何锚点一致的相位
（`mapped_start_w = min(future_switch_w, phase_at_switch)`）。

隔离 3 机、`sim_b_formation_3`（起点 (0,20)、1 m 间距、30 m 航线、k1=1.2）：

| 观测 | 结果 |
| --- | --- |
| 连接段安装 | `connector_success=60`、`connector_failed=4` |
| 三机终点 | **全部 y = -9.99（目标 -10）** |
| 走廊 tube | 全程发布（168/168 等） |
| `sph_beta` | **valid 854/1504（57%）** |
| `g_coord` | **valid 341/602（57%）**，`max ‖g_coord‖ = 1.44~1.50`（非零） |

也就是说：飞行连续、β 有效、集群意图的幅值真的起来（1.4~1.5，贴着
`g_max = 1.5` 饱和）。

C2 回归（隔离构建）：

| 目标 | 结果 |
| --- | --- |
| `continuous_phase_path_test` | 25/25 passed |
| `phase_offset_section_input_test` | 32/32 passed |
| `gvf_switch_policy_test` | 34 passed + 4 skipped（隔离 master 专用用例） |

剩余未闭环：

1. `δ` 的横向偏移**量化证据还没拿到**：采样器里"相对规划路径的横向偏移"
   统计没有落盘（`planner_path` 有 16~17 条、`odom` 3000+ 条），采样器本身
   要修；"δ 随邻居变化并被走廊限制"这一条因此仍无数据。
2. `β` 只有 57% 的样本 valid，需要确认剩下的是否集中在起飞/终点等正常阶段。
3. "有无邻居"的对照实验（验证 δ 由集群意图驱动）尚未做。

### 6.10 集群意图确实让轨迹偏移（有/无邻居对照）

场景：同一套起点列 (0,20)、同一条 30 m 南向直线航线、同样的 SPH 风格平移
终点，只改横向间距。路径本身是直线（起点 x 与终点 x 相同），因此最终
`x` 相对起点列的偏移就是执行层的横向偏移。

| 场景 | uav0 终态 x | uav1 | uav2 | 相对各自路径的偏移 |
| --- | --- | --- | --- | --- |
| 1 m 间距（邻居在 r_comm/r_conf 内） | -1.303 | -0.000 | +1.297 | 外侧两台各 **向外 0.30 m** |
| 5 m 间距（无邻居，对照） | -5.065 | 0.000 | +5.065 | 各 **向内 0.065 m** |

两次都跑完全程（y = -9.99），走廊 tube 全程发布。结论：

1. 有邻居时，外侧两台被横向推开 0.30 m，中间那台因对称保持 0 —— 这正是
   分离/协调意图的方向特征；
2. 无邻居时只剩 0.065 m，而且方向相反（向内），是普通跟踪误差量级
   （与实测 `e_perp ≈ 0.035 m` 同量级）；
3. 0.30 m 远小于走廊半宽 1.5 m，说明偏移**被走廊限制住**。

遗留疑点（下一步）：对照组 `max ‖g_coord‖` 同样显示 1.5，说明 provider 在
5 m 间距下仍会输出饱和意图——要么邻居半径没有真正起到门限作用，要么
`g_coord` 还有别的来源。轨迹层面的效果差异（0.30 vs 0.065）是真实的，但这个
幅值口径需要再查一次（例如打印邻居计数与 `g_sep/g_coh/g_conf` 分量）。

另外，采样器里"相对规划路径的横向偏移"统计仍未落盘（本轮的横向证据来自
终态 x 对比，不依赖该统计）；采样脚本本身仍需修。

### 6.11 换成 pillar 地图后 tube 失效：已知域被画在起飞点

按用户要求把 `phase_offset_world.launch` 的默认地图换成 `pillar.pcd`
（实测 `/mock_map` 点数 410 → 144640，frame=world），结果：

| 地图 | 三机终点 | tube 剖面 | β / g_coord |
| --- | --- | --- | --- |
| `phase_offset_open.pcd`（410 点） | 全部 y ≈ -9.99 | 正常 | valid 57% / max 1.5 |
| `pillar.pcd`（144640 点） | **停在 y ≈ 13** | **38 次 unavailable（status=2 UNKNOWN_DOMAIN）** | 全 0 |

诊断（临时打点已回退）：

- 已知域**确实在建立**（20 s 内 132 次 commit），不是"从没建立"；
- 但建立出来的盒子是 `min=(-6.6,14.4,0.39) max=(4.4,25.4,1.99)`，
  而打印出来的 `source=(-1.00,20.00,1.00) camera=(-1.00,20.00,1.00)`
  —— **源中心与地图自身的 odom 都停在起飞点 y=20**，此时飞机已飞出约 6 m；
- `first_depth=0`（不是深度保护）；CPU 12 核、负载 3.7（不是算力饱和）。

也就是说：地图端用来构造"已知域"的自身里程计几乎不动（飞机飞了 6 m，它只动了
0.02 m），于是已知域盒子停在起飞点附近，而 tube 的 ROI 建在飞机当前位置附近
—— 两者对不上，`readLocalObstacleView` 报 UNKNOWN_DOMAIN，剖面建不出来，
β 发不出，δ 不动，飞机停在柱阵边缘。

尚未定论的是"为什么地图自身 odom 会冻住"：候选是（a）稠密点云（单帧 23040 点）
的 cloud 回调长时间持有 `map_data_mutex_`，而 `odomCallback` 取同一把锁，被
饿死；（b）odom 订阅自身出了问题。这个需要用"cloud 回调与 odom 回调的调用
次数/持锁时长"再打一次点才能定论。

另外记录一次施工事故与修复：回退诊断打点时脚本把 `commitKnownStaticObservationLocked`
复制成了两份（其中一份被截断），已删除重复副本；随后 `formation_planning`
编译通过，并用 `phase_offset_open.pcd` 复跑确认三机重新到达 y ≈ -9.99。

### 6.12 柱阵地图的真实失败原因（6.11 的结论被推翻）

6.11 里"地图自身 odom 冻结"是**读样本错的产物**：我只看了启动瞬间的头几条
commit（那时飞机确实在 y=20）。改成统计计数后，稳态数据是：

| 计数器（单个 agent 进程，同一个稳态时刻） | 值 |
| --- | --- |
| `odomCallback` 调用次数 | 3527（约 150/s，消息年龄 `age=0.000`） |
| `camera`（地图自身 odom） | `(0.21,13.79,0.98)` 等，**紧跟飞机当前位置** |
| `cloudCallback` 平均耗时 | 9.7~11.6 ms（最长 21 ms） |
| 已知域 commit / invalidate | 351 / 13 |
| 视图请求 / UNKNOWN | 309 / 6（**98% 有效**） |

所以：odom 没冻、没有锁饥饿、已知域也不是瓶颈。柱阵下 `local obstacle view
unavailable status=2` 只是全run 约 18 次的瞬态。

真正的原因在 hold 那一刻（临时 `[hold-debug]`，已回退）分成两类：

| 出现次数 | 现象 |
| --- | --- |
| ~56 | `preview=2 (RATE_INFEASIBLE)` 且 allocator 未运行 → **走廊本身速率不可行**（柱阵把走廊压窄） |
| ~27 | `alloc=5 'phase scalar admissible interval is empty'`，`r_w≈0.97`、`f_w0≈2.31` → **追赶项把相位速率顶上窗口**（σ≈0.87） |

两类都是"速率"问题，但和 6.5/6.6 修的那个不同：这里 `r_w` 是健康的（0.97），
所以不是局部刻度塌陷，而是①走廊速率可行性与②窗口余量。实测把
`upper_nu` 覆盖为 3.0 **无效**（仍停在 y≈13），说明 RATE_INFEASIBLE 那一类
不是单靠 `upper_nu` 能解决的。

结论：柱阵场景需要单独一批处理（走廊速率可行性 + 追赶项窗口），在这之前
开放空间（open 地图）的编队效果是可用且已验证的。

### 6.13 clearance（residual epsilon）对照实验：它不是瓶颈

同一柱阵场景、同一场景文件，只把 `phase_offset/tube/cross_section/planner_safe_distance`
预置成 0.4 / 0.25 / 0.10（启动前 `rosparam set`，该参数在任何 launch 里都
没被显式设置，C++ 默认 0.4），每档跑一遍：

| ε | uav0 终点 y | uav1 | uav2 | 走廊宽度（均值/最小/最大，m） |
| --- | --- | --- | --- | --- |
| 0.40 | 13.83 | 13.81 | 12.74 | 1.45 / 0.00 / 2.80（uav0） |
| 0.25 | 13.83 | 13.59 | 12.80 | 1.46 / 0.00 / 2.80（uav0） |
| 0.10 | 13.53 | 13.58 | 12.78 | 1.36 / 0.00 / 2.80（uav0） |

三个 ε 下**结论完全一样**：三机都停在 y≈12.7~13.8，`g_coord` 全 0，走廊宽度
也几乎不变（均值 1.36~2.05 m、最大 2.80 m）。所以：

1. **clearance 不是控制走廊宽度的量**，"clearance 把走廊吃光"这个假设被本
   实验否定；
2. 更重要的修正：我之前引用的"走廊被挤到 ±1 cm"其实是 preview 的
   **reachable δ 区间**（`delta_reach`），**不是走廊宽度**；实测走廊平均
   1.4~2.0 m，只是在个别 cross-section 上降到 0；
3. 因此柱阵下的停机更可能出在 preview 的可行性/可达性计算本身（此前打点
   到 `max_inward_contraction_slope == allowed_inward_contraction_slope ==
   0.1250` 且 `boundary_tolerance = 0`，是"恰好等于"的边界情形），而不是
   地图把空间吃光。

## 7. 卫生

- 临时诊断打印（`sdf_map.cpp` 的 `known-domain-debug`）已全部回退，
  `grep -c known-domain-debug` = 0。
- `sim_b_bringup_test.py` 5/5 通过（含新增的 SPH-ready 默认值断言、
  1.2 m 冲突场景断言、SPH 风格终点断言）；`git diff --check` 干净。
- 未提交、未建分支、未触碰用户 ROS 会话；不改动不在白名单内的用户文件。
