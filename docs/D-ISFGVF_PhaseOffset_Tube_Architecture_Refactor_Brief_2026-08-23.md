# D-ISFGVF Phase-Offset / Path-Tube 架构重构审计 Brief

**日期：2026-08-23**
**用途：给 Codex 主窗口作为长期上下文、给 Luna Max 做只读证据审计、给 Sol 做独立复核，并据此生成分阶段重构计划。**

---

## 0. 这次任务不是“修一个 Tube bug”

当前问题不能再按“看到一个 first-false 就继续增加一个 gate / certificate / retry / fallback”的方式修补。

本轮工作的目标是重新建立清晰的职责边界：

\[
p_i(w)
\rightarrow
\mathcal I_i(w)
\rightarrow
\mathcal K_i
\rightarrow
g_i^{\mathrm{des}}
\rightarrow
u_i^\star=[u_{w,i}^\star,u_{\delta,i}^\star]^\top
\rightarrow
\text{matched injection}.
\]

其中：

- Planner 负责生成可执行的三维局部路径 \(p_i(w)\)。
- Path-Tube 几何层只回答“沿路径横向哪里是几何可行的”。
- Preview feasibility 只回答“在横向速率约束下未来是否还能持续位于 Tube 内”。
- Allocator / CBF 只回答“当前控制周期执行什么 \(u_w,u_\delta\)”。
- Matched injection 负责把 phase-offset coordination 注入 nominal ISF-GVF，同时保持 nominal tracking-error dynamics。
- Tube/coordination 子系统不得因为“没有非零横向余量”而否决一个仍然安全、可执行的 planner centerline path。

**核心原则：**

> 对于和 Tube 使用同一 robust free-space contract 的有效 planner path，\(\delta=0\) 应当是 geometric Path-Tube 的基线可行状态；非零 transverse capacity 由局部自由空间决定。Preview 或 allocator 的不可行只能限制 phase-offset coordination，不能自动升级成 planner path failure。

## 0.1 本轮执行策略：架构一次冻结，代码分批落地

这次不再采用“先修一个 Stage，再到下一 Stage 重新设计接口”的方式。Luna Max 与 Sol 的只读审计必须覆盖最终 Section II--IV 对应的**完整目标架构**，主窗口随后一次性冻结所有跨层接口、ownership、状态机和 failure semantics。

冻结后的目标链条是：

\[
p_i(w)
\rightarrow
(r_i,N_i,r_{w,i})
\rightarrow
\mathcal I_i(w)
\rightarrow
\mathcal K_i
\rightarrow
g_i^{\mathrm{des}}
\rightarrow
u_i^\star
\rightarrow
D\Xi_i u_i^\star.
\]

**“一起改”指统一成一次 architecture refactor，不是一次性无检查地改完所有源码。** 在同一个重构分支中，接口先冻结，再按依赖关系分 Batch A--D 落地；每个 Batch 都必须可编译、可测试、可回滚。除非后续出现新的源码证据证明 architecture freeze 有错误，否则不得在实施中重新引入旧 contract 或继续追加临时 gate/certificate。

本轮允许删除或重写与最终理论冲突的临时 internal contract；必须保持的是：

1. 原 planner 的正常路径生成与 replanning 能力；
2. nominal ISFGVF 在 phase-offset coordination 关闭时的原有导航行为；
3. 明确的 current-state safety 约束；
4. matched injection 对 nominal tracking-error dynamics 的保持。

验收优先级固定为：

\[
\text{Base planner + ISFGVF navigation liveness}
>
\text{Geometric Tube correctness}
>
\text{Smooth phase-offset coordination}.
\]

Tube certificate completeness 不得再拥有高于基础导航存活性的优先级。

---

# 1. 冻结后的论文理论边界

下面是本轮代码必须对齐的理论，不允许再用旧 proposal、旧 manual profile、旧二维模型反向定义 production behavior。

## 1.1 Planner 与 swarm coordination 的职责分离

每台 UAV 独立维护 planner 生成的三维路径

\[
p_i:\mathbb R\rightarrow\mathbb R^3,
\qquad
p_i(w_i)=[p_{x,i},p_{y,i},p_{z,i}]^\top.
\]

Planner 负责：

- 3D free-space exploration；
- KinoA*；
- B-spline path geometry；
- static-obstacle avoidance；
- local replanning；
- 上升、下降、斜飞等三维几何。

Swarm layer 不重新规划完整三维路径，只增加一个标量横向自由度：

\[
r_i(w_i,\delta_i)=p_i(w_i)+N_i(w_i)\delta_i.
\]

其中：

- \(w_i\)：沿本机局部路径的纵向 progression / speed redistribution；
- \(\delta_i\in\mathbb R\)：一个标量 transverse degree of freedom，用于横向展开、压缩、错位通过、恢复；
- 在 \(n=3\) 时，\(N_i(w)\) 是三维路径二维 normal plane 中连续选择的一条单位方向，不是把物理运动限制到水平面。

## 1.2 Geometric Path-Tube

定义

\[
T_i(w)=\frac{p_{w,i}(w)}{\|p_{w,i}(w)\|},
\qquad
N_i^\top T_i=0,
\qquad
\|N_i\|=1.
\]

相位偏移参考：

\[
r_i(w,\delta)=p_i(w)+N_i(w)\delta.
\]

\[
r_{w,i}=p_{w,i}+N_{w,i}\delta,
\qquad
r_{\delta,i}=N_i.
\]

实际采用的正则性条件是：

\[
\|r_{w,i}(w,\delta)\|\ge m_r>0.
\]

而不是把 planar 条件 \(1-\kappa\delta\) 当成 general 3D production hard gate。

在给定 \(w\) 时，raw transverse cross-section 是

\[
\mathcal I_i^{\mathrm{raw}}(w)
=
\{\delta\mid r_i(w,\delta)\in\mathcal F_i^{\mathrm{rob}},\ \|r_{w,i}\|\ge m_r\}.
\]

如果 raw cross-section 断开：

- 正常滚动阶段保留与**当前 \(\delta_i\)** 一致的 connected component；
- 初始化时当前 \(\delta_i=0\)，因此自然优先选包含 \(0\) 的 component；
- 不允许跨 unsafe gap 做凸化。

最终：

\[
\mathcal I_i(w)=
[\underline\delta_i(w),\overline\delta_i(w)].
\]

## 1.3 Zero-only Tube 是正常退化状态

如果路径中心线安全但局部没有可认证的横向余量，允许

\[
\mathcal I_i(w)=[0,0].
\]

这代表：

- planner centerline 继续可用；
- phase-offset coordination 当前没有横向 authority；
- \(\delta_i\) 应连续恢复到 0；
- \(u_{\delta,i}\) 最终恢复到 0；
- 必要时通过 \(u_{w,i}\) 减速，为横向恢复创造时间；
- **不能因为 zero-only 就直接阻断 planner handoff 或导航。**

## 1.4 Preview feasibility 与 geometric Tube 分离

Tube 几何成立不等于横向状态在有限速率下可以瞬间跟随。

有限横向速率对应单独的 preview feasibility：

\[
\Delta_\delta=
\bar u_\delta\frac{\Delta s}{\bar v_s},
\qquad
\mathcal B_\delta=[-\Delta_\delta,\Delta_\delta],
\]

\[
\mathcal K_{i,H}=\mathcal I_{i,H},
\]

\[
\mathcal K_{i,k}
=
\mathcal I_{i,k}
\cap
(\mathcal K_{i,k+1}\oplus\mathcal B_\delta).
\]

因此：

- \(\mathcal I\) 失败：几何层问题；
- \(\mathcal K\) 失败：动态预览不可行；
- QP infeasible：当前执行层问题；
- 这三种状态不能再使用同一个“Tube failed”语义。

## 1.5 连续横向展开、压缩与恢复

正常行为应是：

### 开阔区域

\[
\beta_i\rightarrow1,
\]

弱 cohesion 可以发挥作用，allocator 根据 \(g_i^{\mathrm{des}}\) 让 UAV 产生适度横向展开/组织：

\[
|\delta_i|>0.
\]

### 通道逐渐变窄

移动 Tube 边界开始收缩：

\[
\underline\delta_i(w),\ \overline\delta_i(w)
\rightarrow0.
\]

QP 同时调节：

\[
u_\delta
\quad\text{和}\quad
u_w,
\]

产生

\[
\text{transverse compression}
+
\text{longitudinal redistribution}.
\]

### Tube 退化到 centerline

\[
\mathcal I_i=[0,0]
\]

时，系统连续趋向

\[
\delta_i\rightarrow0,
\qquad
u_{\delta,i}\rightarrow0,
\]

而不是跳变或 HOLD。

### 再次开阔

Tube 横向容量重新增大，coordination 可以再次连续展开。

## 1.6 IV-A：Two-variable constrained allocator

定义

\[
J_i=[r_{w,i},N_i]=\pi_xD\Xi_i,
\]

\[
u_i=
\begin{bmatrix}
u_{w,i}\\u_{\delta,i}\end{bmatrix}.
\]

\[
J_iu_i=r_{w,i}u_{w,i}+N_iu_{\delta,i}.
\]

推荐目标：

\[
 u_i^\star=
 \arg\min_{u_i}
 \|J_iu_i-g_i^{\mathrm{des}}\|_{R_g}^2
 +\|S_iu_i\|_{R_u}^2
 +\|S_i(u_i-u_i^{\mathrm{prev}})\|_{R_\Delta}^2,
\]

\[
S_i=\operatorname{diag}(\|r_{w,i}\|,1).
\]

约束至少包括：

### phase progress

\[
\underline\nu
\le
f_{w,i}^0+u_{w,i}
\le
\overline\nu.
\]

### rolling-mode physical forward speed

\[
v_{\parallel,i}^0+\|r_{w,i}\|u_{w,i}\ge v_{\min}>0.
\]

### transverse rate

\[
|u_{\delta,i}|\le\bar u_\delta.
\]

### moving Tube boundary constraints

\[
(\partial_w\bar\delta_i)(f_{w,i}^0+u_{w,i})-u_{\delta,i}
\ge
-\gamma_T(\bar\delta_i-\delta_i),
\]

\[
u_{\delta,i}
-(\partial_w\underline\delta_i)(f_{w,i}^0+u_{w,i})
\ge
-\gamma_T(\delta_i-\underline\delta_i).
\]

### pairwise velocity-level safety

\[
2q_{ij}^\top
\left(
f_{x,i}^0+J_iu_i-\hat v_j
\right)
\ge
-\gamma_sh_{ij}+\rho_{ij}.
\]

## 1.7 III-B 中 nominal coordination 与 hard safety 分离

最终 coordination intent：

\[
g_i^{\mathrm{coord}}
=
g_i^{\mathrm{sep}}
+
\beta_i g_i^{\mathrm{coh},0}
+
g_i^{\mathrm{damp}}.
\]

之后

\[
g_i^{\mathrm{des}}
=
\operatorname{sat}(g_i^{\mathrm{coord}})
-k_{\mathrm{rec}}\delta_iN_i.
\]

Pairwise safety 不应该再额外以一个独立 soft `g_safe` 和 nominal coordination 叠加后作为唯一 safety 手段；真正的 safety 由 allocator 的 pairwise velocity-level constraint 承担。

## 1.8 IV-B：Matched phase-offset injection

\[
\Phi_i=x_i-r_i(w_i,\delta_i),
\]

\[
G_i=D\Phi_i=[I_n,-r_{w,i},-N_i].
\]

\[
\mathcal B_i=D\Xi_i=
\begin{bmatrix}
r_{w,i}&N_i\\
1&0\\
0&1
\end{bmatrix}.
\]

\[
G_i\mathcal B_i=0.
\]

因此匹配注入：

\[
\dot x_i
=f_{x,i}^0+r_{w,i}u_{w,i}^\star+N_i u_{\delta,i}^\star,
\]

\[
\dot w_i=f_{w,i}^0+u_{w,i}^\star,
\]

\[
\dot\delta_i=u_{\delta,i}^\star,
\]

保持

\[
\dot e_i
=f_{x,i}^0-r_{w,i}f_{w,i}^0
=F_i(e_i,w_i).
\]

---

# 2. 当前代码初审发现：需要 Luna Max 独立验证的疑似架构偏差

以下不是让实现模型直接照着改，而是“高优先级待验证 hypotheses”。Luna Max 必须重新读源码、测试、launch 和调用链，对每一条给出精确证据；Sol 必须再次独立复核。

## H1. Cross-section connected component 固定围绕 \(\delta=0\)

疑似位置：

- `src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_cross_section.cpp`
- `ExpandFromZero(...)`

疑似行为：

- production ray expansion 始终从 \(\delta=0\) 开始；
- 注释明确围绕 “component containing delta = 0”；
- 与最终理论“滚动阶段保留包含当前 \(\delta_i\) 的 connected component，初始化才用 0”不一致。

需要验证：

1. 当前 retained delta 非零时，是否仍重新只构造 zero-connected interval；
2. 如果 zero-connected component 和 current-delta component 不同，哪个层会发生丢失/拒绝；
3. 是否存在旧 active profile 能暂时掩盖该问题。

## H2. `authority_request` 把 manual/control intent 混入 geometric Tube

疑似位置：

- `phase_offset_matched_adapter.cpp::makeAuthorityRequest(...)`
- `certified_tube_builder.cpp`

疑似行为：

- manual amplitude、retained delta、interior margin 被组合成 authority interval；
- CertifiedTubeBuilder 再与环境 Tube 相交；
- 几何 Tube 因控制侧“想要多大 offset”而变化。

最终要求：

\[
\mathcal I(w)
\text{ 只能由 path geometry + robust free space + regularity 决定。}
\]

## H3. Adaptive sampling 的几何细分尺度严重放大了 normal variation

疑似位置：

- `tube_builder.cpp::AdaptiveNeedsSubdivision(...)`

重点验证：

- `normal_deviation * cross_section.search_extent` 是否使用 `environment_search_extent`；
- launch 中 `environment_search_extent` 是否约 3.0 m，而实际 `max_offset` 约 0.2 m；
- 这是否导致正常弯曲路径在 adaptive samples / recursion depth 上大量耗尽；
- failure 是否最终传播成 Candidate incomplete。

## H4. Path-cell certificate 不成立时采用整 voxel `continuous_inset`

疑似位置：

- `tube_builder.cpp::buildCloudClearance(...)`

重点验证：

- fallback `continuous_inset = snapshot_resolution`；
- 对窄 Tube 是否会把大量 nonzero interval 压成 zero-only；
- 是否存在与后续 SurfaceValidator cover 的重复保守性。

## H5. SurfaceValidator 远强于论文 geometric Tube contract

疑似位置：

- `tube_surface_validator.cpp`

重点验证：

- 2D \((w,v)\) ribbon surface；
- 3x3 samples；
- cover radius；
- `requested_clearance = required_clearance + cover_radius + epsilon`；
- subdivision depth / query budget；
- 全宽失败后的后果。

需要区分：

- 这是一个可以保留的 conservative geometric validator；还是
- 已经被错误地赋予“planner path / handoff owner”的权限。

重点不是单纯删掉 validator，而是明确其失败语义只能是：

> 缩窄或撤销 nonzero offset capacity。

不能自动变成 planner navigation failure。

## H6. CertifiedTubeBuilder 的 inward search 与 zero-only fallback 语义混乱

疑似位置：

- `certified_tube_builder.cpp`

重点验证：

- BOTH_SIDED / POSITIVE_ONLY / NEGATIVE_ONLY；
- 0.5、0.25、0.125... 缩窄重试；
- 最终 `CollapseToPlannerZeroBaseline(...)`；
- `ZERO_ONLY_PLANNER_BASELINE` 在后续哪些地方仍被视为“不够好”。

## H7. Zero-only Tube 不能成为正常 PathTubePair replacement

疑似位置：

- `phase_offset_matched_adapter.cpp`
- PathTubePair / staged epoch / H2 handoff 相关逻辑

重点验证：

- 是否硬要求 `OFFSET_CERTIFIED`；
- `ZERO_ONLY_PLANNER_BASELINE` 是否被拒绝；
- 是否造成 `Tube no capacity -> replacement failed -> HOLD`。

最终要求：

- zero-only 是正常退化状态；
- 如果新 planner path 安全且 \(\delta=0\) baseline 可用，Tube 不得阻止 path ownership transition；
- 如果当前 \(\delta\neq0\)，应先执行连续 recenter transition，而不是直接硬切。

## H8. 3D normal 仍是旧的 horizontal normal

疑似位置：

- `phase_offset_core/src/geometry.cpp`

疑似实现：

\[
N=\frac{e_z\times p_w}{\|e_z\times p_w\|}.
\]

风险：

- 接近垂直路径时 normal 退化；
- planner 可以正常上升/下降，但 Tube geometry invalid；
- 与论文“在 3D normal plane 中连续选择一个方向”不一致。

Luna Max 要特别研究：

- 当前代码中有没有已有的 parallel transport / reference-axis fallback / frame continuity helper 可复用；
- 如何实现连续、确定性、无符号翻转的 \(N(w)\)；
- 不能只用“若垂直就换另一个固定轴”而忽略跨 sample 的连续性。

## H9. Production regularity 仍以 planar \(1-\kappa\delta\) 为主要 hard gate

疑似位置：

- `geometry.cpp`
- `tube_cross_section.cpp`
- `tube_surface_validator.cpp`

最终应对齐：

\[
\|p_w+N_w\delta\|\ge m_r.
\]

需要判断：

- 哪些 planar regularity checks 可以删除；
- 哪些可以保留为特定 horizontal-normal implementation 的辅助 sufficient condition；
- 不允许 auxiliary condition 继续错误拒绝 general 3D regular reference。

## H10. Future exact-port viability 被提升成 Tube / handoff 的存在性 gate

疑似位置：

- `phase_offset_runtime.cpp`
- `port_projector.cpp`
- `phase_offset_matched_adapter.cpp` staging dry-run / future witness

已观察到的典型冲突：

- old \(\delta\neq0\)；
- old \(u_\delta\) 有较大负值；
- new Tube 要求快速向中心收缩；
- slew constraint 与 new Tube future lower bound 在若干 future steps 内无交集；
- `joint port feasible polygon is empty`；
- 该 dynamic transition failure 被升级成 replacement/handoff failure。

需要验证：

- exact future witness 当前到底负责什么；
- 它是否在几何层、epoch install、PathTubePair handoff、Runtime mode 上重复 gate；
- 如何把它降级成 preview/transition feasibility，而不是 geometric Tube existence。

## H11. 真正接近论文 IV-A 的 allocator/CBF 已经存在但没有进入 production

疑似位置：

- `bspline_traj/src/phase_offset_allocator.cpp`
- `bspline_traj/src/phase_offset_cbf_constraints.cpp`

需要验证：

- production `gvf_manager` / adapter / runtime 是否真正调用；
- 还是只在 test/docs 中存在；
- 当前 production 是否仍主要是 manual sinusoidal profile + PortProjector + future witness。

## H12. 现有 allocator 中可能存在量纲错误和 fallback bug

需要重点核对：

### duplicate static delta constraint

类似

\[
u_\delta\ge\underline\delta-\delta
\]

左边是 m/s，右边是 m，量纲错误。

若已有 moving Tube CBF，则这类 duplicate `delta_lower/delta_upper` constraint 应当删除或改成具有 dt 的离散预测约束，不能继续混用。

### slew relaxation bug

检查 safety fallback 放宽 `u_delta` slew 时，是否错误使用 `u_w_slew_rate`。

## H13. `g_safe` 仍被作为 soft world-frame intent 叠加到 `g_des`

疑似位置：

- `swarm_neighbor_model.cpp`

最终理论要求：

\[
g^{coord}=g^{sep}+\beta g^{coh,0}+g^{damp}
\]

pairwise safety 由 allocator 的 hard velocity-level constraint 单独处理。

需要验证：

- soft safety repulsion 是否仍在 production；
- 是否与 pairwise CBF 重复；
- 是否影响论文实验可解释性。

## H14. Candidate / Certified RViz marker 语义混入 runtime success

需要追调用链确认：

- Candidate marker 实际代表什么；
- Certified marker 是否要求 active/current/runtime/port/matched mode 全部成功；
- 如果是，则不能再用“Certified marker 数量”直接衡量 geometric Tube construction success。

重构后应至少有明确的可观测层级：

1. geometric candidate interval；
2. active geometric Tube；
3. preview-feasible set / transition state；
4. allocator feasible/infeasible；
5. planner path ownership / navigation state。

---

# 3. 本轮重构必须建立的不可破坏 invariants

这些 invariants 比某个函数或类更重要。主窗口后续写 execution plan 时必须逐条给测试。

## INV-1 Planner authority invariant

在下列条件成立时：

- planner 当前 path 自身有效；
- 与 Tube 使用同一 robust clearance contract 时 \(\delta=0\) 可行；
- 当前实际状态没有明确 collision/safety violation；

则：

> Tube 子系统不能仅因为“nonzero offset capacity 不存在”“preview 不可行”“old port 无法立即兼容 new profile”而把 planner path 变成 HOLD / invalid。

## INV-2 Geometric Tube purity

\[
\mathcal I(w)
\]

只依赖：

- 当前 planner path geometry；
- chosen continuous normal field；
- robust free-space observation；
- reference regularity。

不得依赖：

- manual amplitude；
- swarm desired intent；
- previous port；
- previous slew state；
- future exact port witness。

## INV-3 Current-component continuity

当 \(\delta_i\neq0\) 时，新建 Tube 的 connected branch selection 必须优先保持 current-offset continuity。

初始化时才自然退化为 zero-connected branch。

## INV-4 Zero-only is valid

\[
[0,0]
\]

是合法 geometric Tube state，不是 software failure。

## INV-5 Preview is not geometry

Preview infeasible 只限制当前可执行 phase-offset behavior，不撤销几何 Tube。

## INV-6 Smooth recenter transition

当新路径 / 新 Tube 暂时不包含 retained \(\delta\) 时：

1. 不直接硬切到新 reference；
2. 不因为 old port/new Tube slew conflict 直接 HOLD；
3. 保留当前安全 owner；
4. 生成 recenter objective \(\delta\to0\) 或向新 Tube overlap 区间恢复；
5. 必要时降低 phase progression；
6. 达到 handoff-compatible state 后再切换 owner。

## INV-7 3D normal continuity

\(N(w)\) 必须：

- \(N^TT=0\)；
- \(\|N\|=1\)；
- 对正常 3D path 连续；
- 不因接近垂直 segment 直接失效；
- 不出现 sample-to-sample arbitrary sign flip；
- 能给出一致的 \(N_w\) 或等价可用于 \(r_w\) 的连续导数信息。

## INV-8 Matched dynamics invariant

任何 feasible coordination port \(u^\star\) 进入 matched injection 后，不得改变 nominal tracking-error dynamics。

## INV-9 Safety scope invariant

- nominal coordination intent 不等于 hard safety；
- pairwise velocity-level safety 在 allocator 中单独约束；
- communication/stale prediction uncertainty 需要明确 margin；
- 不夸大为完整 nonlinear multirotor collision-proof guarantee。

---

# 4. 推荐的目标 production 架构

建议最终 production pipeline 收敛成下面五层。

## Layer A — Planner path owner

输出：

- current semantic path；
- current \(w\)；
- path state query；
- path revision；
- planner-valid / planner-invalid。

Tube 不拥有 planner validity。

## Layer B — Geometric Path-Tube builder

输入：

- path；
- current \(\delta\)；
- map/free-space snapshot；
- robust margin；
- continuous normal state。

输出：

- PWL \(\underline\delta(w),\overline\delta(w)\)；
- zero-only / nonzero capacity；
- geometric diagnostics；
- current connected branch id/continuity facts。

不接受：

- manual amplitude；
- previous port；
- slew constraints；
- swarm desired motion。

## Layer C — Preview / transition feasibility

输入：

- geometric Tube；
- current \(w,\delta\)；
- rate bounds；
- candidate new path/tube；
- optional current handoff target interval。

输出：

- preview feasible set \(\mathcal K\)；
- normal rolling / recenter / wait-for-overlap / safety-dominated；
- 不修改 geometric Tube 本身。

## Layer D — Distributed allocator

输入：

- \(J=[r_w,N]\)；
- \(g^{des}\)；
- \(\mathcal I\) / PWL slopes；
- phase/tangent/rate bounds；
- pairwise safety constraints；
- previous port 仅作为 smoothness regularization / slew bound。

输出：

\[
u^\star=[u_w^\star,u_\delta^\star].
\]

## Layer E — Matched ISF-GVF injection

只负责执行：

\[
\dot x=f_x^0+r_wu_w^\star+Nu_\delta^\star,
\]

\[
\dot w=f_w^0+u_w^\star,
\qquad
\dot\delta=u_\delta^\star.
\]

---

# 5. 统一架构重构与分批落地

本轮必须先完成**完整 architecture freeze**，再开始任何 production refactor。不能出现“Batch A 做完以后才决定 Batch B 的接口”这种边做边改架构的情况。

## Phase A — Read-only audit + architecture freeze

这一阶段不修改 production code。

Luna Max 与 Sol 必须一次性覆盖：

- Planner / semantic path owner；
- 3D phase-offset geometry；
- geometric Tube；
- preview feasibility；
- current-
  \(\delta\) connected component；
- zero-only baseline；
- recenter / handoff；
- two-variable allocator；
- moving Tube CBF；
- pairwise CBF；
- matched injection；
- Runtime / RViz / telemetry；
- thread / epoch / snapshot ownership。

主窗口在这一阶段结束时必须冻结以下接口，不允许留到编码时再临时决定：

1. `PhaseOffsetGeometryState`：
   \(p,p_w,p_{ww},N,N_w,r,r_w,T,\delta\) 的定义与 validity contract；
2. `GeometricTubeProfile`：
   只包含 path/free-space/regularity 得到的 \(\mathcal I(w)\)，不包含 manual、previous port、future DFS；
3. `PreviewFeasibility`：
   \(\mathcal K\)、rate-limited preview 与 transition feasibility；
4. `PhaseOffsetAllocatorInput/Output`：
   \(J,g^{des},u^{prev}\)、progress、Tube、slew、pairwise constraints；
5. `HandoffState`：
   至少区分 `ROLLING / RECENTERING_FOR_HANDOFF / HANDOFF_READY / PLANNER_ONLY`；
6. `MatchedInjection`：
   \(u_w,u_\delta\) 在 physical/internal coordinates 中的唯一注入位置；
7. failure semantics：
   `GEOMETRIC_TUBE_UNAVAILABLE`、`ZERO_ONLY`、`PREVIEW_INFEASIBLE`、`ALLOCATOR_INFEASIBLE`、`CURRENT_STATE_UNSAFE`、`PLANNER_INVALID` 不得互相冒充。

Phase A 最终输出必须是一份统一 ADR / execution plan，而不是若干互相独立的修复建议。

## Batch A — Final 3D Geometry + Geometric Tube

这一批一次性统一 Section III-A 的基础几何和 Tube contract。

主要修改范围：

- `phase_offset_core/geometry.*`
- `tube_cross_section.*`
- `tube_builder.*`
- 与 geometric Tube 直接相关的数据结构和 tests。

必须同时完成：

- 3D normal-plane 中连续的 \(N(w)\) 选择，处理 near-vertical path 与 sign continuity；
- \(N_w\) 与
  \[
  r_w=p_w+N_w\delta
  \]
  的统一实现；
- production regularity 改为
  \[
  \|r_w\|\ge m_r;
  \]
- raw cross-section 以 current \(\delta\) 的 connected component 为主，初始化才自然等价于 zero-connected；
- geometric Tube 不再读取 manual amplitude、previous port、slew state、future DFS、swarm intent；
- adaptive sampling / continuous cover 的尺度以**实际 admissible transverse range**为依据，而不是固定 3 m search extent 放大 normal variation；
- valid planner centerline 在相同 robust clearance contract 下至少允许 `ZERO_ONLY` baseline。

Batch A 的验收重点不是 swarm coordination，而是：

> 打开 Tube 以后，单机 planner + nominal ISFGVF 仍能沿 centerline 正常导航；开阔区域能生成合理非零 Tube，狭窄区域可自然退化到 zero-only。

## Batch B — Lifecycle + Preview + Continuous Recenter / Handoff

这一批统一解决当前最严重的 `Tube -> planner veto -> HOLD` 问题。

主要修改范围：

- `tube_epoch_manager.*`
- `phase_offset_runtime.*`
- `phase_offset_matched_adapter.*`
- `gvf_manager` / PathTubePair / owner handoff 相关代码。

必须同时完成：

- geometric Tube、preview feasible、runtime control feasible 分层；
- `ZERO_ONLY` 成为正常可执行的 planner baseline；
- future exact-port incompatibility 不再被解释为 new geometric Tube invalid；
- `STAGING_DRY_RUN` 不得再直接 veto 一个 planner-valid successor path；
- old path 上 \(\delta\neq0\) 时实现显式 transition owner：
  \[
  \delta\rightarrow\delta_{\mathrm{handoff}},\qquad
  u_\delta\rightarrow u_{\delta,\mathrm{handoff}},
  \]
  必要时通过 \(u_w\) 降低纵向 progression；
- successor Tube 与 current state 出现 overlap 后再进入 `HANDOFF_READY`；
- planner path replacement 与 Tube coordination authority 解耦，但 current-state explicit unsafe 仍必须 fail-safe；
- Certified/RViz 不再用 runtime port success 冒充 geometric Tube existence。

Batch B 的关键验收：

> 开启 Tube 后，单机 point-to-point/replan 的到达率不得低于 Tube 关闭时；new Tube 暂时接不住旧 port 时出现的是平滑 recenter / slowing，而不是长期 36/0/36 HOLD。

## Batch C — Final Distributed Allocator + CBF + Matched Injection

这一批把 Section III-B、IV-A、IV-B 的最终算法真正接入 production。

主要修改范围：

- `phase_offset_allocator.*`
- `phase_offset_cbf_constraints.*`
- `swarm_neighbor_model.*`
- allocator/runtime/adapter/gvf integration。

必须先修现有实现中的已知疑点：

- 删除量纲错误的 static `delta_lower/delta_upper` rate constraints，统一使用 moving Tube CBF；
- `u_delta` slew relaxation 不得错误使用 `u_w_slew_rate`；
- objective 对齐
  \[
  \|Ju-g^{des}\|_{R_g}^2+
  \|Su\|_{R_u}^2+
  \|S(u-u^{prev})\|_{R_\Delta}^2;
  \]
- nominal `g_des` 中去除与 hard pairwise CBF 重复承担 safety 的旧 soft `g_safe`；
- progress、Tube、actuation、slew、pairwise constraints 全部在 two-variable allocator 内有明确 ownership；
- matched injection 只通过
  \[
  D\Xi_i u_i^\star
  \]
  注入，并验证
  \[
  G_iD\Xi_i=0.
  \]

Batch C 的动态验收必须看到：

\[
\text{continuous lateral expansion}
\rightarrow
\text{continuous compression}
\rightarrow
\text{continuous recovery},
\]

而不是直接横跳、人工正弦偏移或 Tube failure 导致 HOLD。

## Batch D — Cleanup + RViz / telemetry + paper experiments

只有 A--C 的 production behavior 通过后才做清理。

包括：

- manual sinusoidal profile 降级为 test/diagnostic，不再作为 production coordination source；
- 删除已经失去 owner 的旧 certificate/gate/bypass；
- 对仍有价值的 conservative proof 重命名并放回正确层；
- RViz 至少分别展示：geometric Tube、preview-feasible region、active offset reference、handoff/recenter state；
- logging 对齐论文实验：\(\delta\)、Tube width、\(\beta\)、\(u_w\)、\(u_\delta\)、pairwise distance、QP time、mode、handoff state；
- 完成 open -> narrow -> open、zero-only、replan at nonzero \(\delta\)、near-vertical 3D path 等回归场景。

## 每个 Batch 的统一实施规则

虽然属于同一次架构重构，但每个 Batch 仍必须：

1. 在同一个 refactor branch 上实现；
2. 先完成该 Batch 的 tests，再修改下一 Batch；
3. build + unit tests + relevant ROS smoke test；
4. 做 diff self-review；
5. 高风险 Batch B/C 完成后可开一个 Sol mini-audit，只读审核 diff；
6. 保留可回滚 commit；
7. 不允许多个 coding agent 并行修改同一组 production files。

# 6. 不允许的修法

后续任何 agent 都不得：

1. 为了让某一个日志通过，继续新增一个全局 bypass；
2. 通过扩大 numerical tolerance 掩盖 architecture conflict；
3. 直接删除 safety checks 而不重新归属到正确层；
4. 让 Tube builder 读取 manual amplitude 或 previous port；
5. 让 geometric Tube success 依赖未来多步 exact-port DFS；
6. 让 zero-only Tube 自动触发 planner invalid；
7. 在没有连续性设计时简单“垂直路径换一个固定 axis”解决 3D normal；
8. 同时让多个 coding agent 修改同一批 production 文件；
9. 在 Luna Max / Sol 审计完成前直接开始大范围 implementation；
10. 用文档里的旧 proposal 取代这里冻结的最终理论。

---

# 7. Luna Max 只读审计任务

Luna Max 的职责是**找证据和建立完整调用链**，不是写代码。

必须独立阅读：

### Core geometry / Tube

- `src/swarm_planner/phase_offset/phase_offset_core/src/geometry.cpp`
- `src/swarm_planner/phase_offset/phase_offset_core/src/port_projector.cpp`
- `src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_cross_section.cpp`
- `src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_builder.cpp`
- `src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_filter.cpp`
- `src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_surface_validator.cpp`
- `src/swarm_planner/phase_offset/phase_offset_navigation/src/certified_tube_builder.cpp`
- `src/swarm_planner/phase_offset/phase_offset_navigation/src/tube_epoch_manager.cpp`
- `src/swarm_planner/phase_offset/phase_offset_navigation/src/phase_offset_runtime.cpp`

### Integration / handoff

- `src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp`
- 相关 `.h`
- `gvf_manager` 及调用 adapter/runtime 的代码
- PathTubePair / staged epoch / authority handoff 相关代码

### Swarm algorithm

- `phase_offset_allocator.cpp/.h`
- `phase_offset_cbf_constraints.cpp/.h`
- `swarm_neighbor_model.cpp/.h`
- debug message / topic definitions

### Planner / map / launch

- planner safe-distance 的实际约束语义；
- cloud occupancy snapshot / clearance query；
- `test_gvf.launch`；
- `phase_offset_esdf_tube_single.launch`；
- 所有 phase_offset 参数默认值和覆盖链。

### Tests

至少检查：

- geometry tests；
- tube_cross_section tests；
- tube_builder tests；
- tube_surface_validator tests；
- certified_tube_builder tests；
- tube_epoch_manager tests；
- runtime tests；
- allocator tests；
- matched adapter tests。

Luna 输出必须包含：

1. end-to-end production call graph；
2. 每个 hypothesis H1–H14：`CONFIRMED / PARTIAL / FALSE / UNKNOWN`；
3. 对每个 confirmed finding 提供：
   - 文件；
   - 函数/类；
   - 精确代码行为；
   - 上游输入；
   - 下游影响；
   - 为什么会降低 Tube success 或影响 navigation；
4. 额外发现，特别是我们初审漏掉的 owner/state-machine 问题；
5. 当前 unit tests 哪些是在“保护旧错误语义”；
6. 现有代码中可以保留复用的模块；
7. 推荐最小架构切口，而不是逐日志 patch；
8. 不修改任何源码。

---

# 8. Sol 独立复核任务

Sol 不允许只阅读 Luna 报告后点头。必须重新查看对应源码，并执行 red-team audit。

Sol 要回答：

1. Luna 的每个 `CONFIRMED` 是否真的成立；
2. 是否把安全保守设计误判成架构 bug；
3. 哪些条件虽然过严，但在当前 map contract 下仍有必要；
4. 哪些修改可能让 planner path 与 Tube 使用不同安全 margin，从而破坏 “zero baseline” 假设；
5. 3D continuous normal 的实现提议是否数学上连续、工程上可实现；
6. current-delta component selection 是否会错误选择 disconnected unsafe branch；
7. recenter handoff 有没有 deadlock / liveness 问题；
8. QP moving Tube constraint 的符号、单位、base \(f_w^0\) 接口是否一致；
9. matched injection 是否真的保持当前 nominal error dynamics；
10. allocator 接入后是否与 existing governor / command saturation 冲突；
11. 哪些旧 tests 应改，哪些必须保留；
12. 是否还有任何 Tube -> planner veto 的隐藏路径；
13. 是否有线程/epoch/snapshot ownership 的竞态风险；
14. 给出最终 P0/P1/P2 优先级。

Sol 输出格式：

- `Accepted findings`
- `Rejected / overstated findings`
- `Missing findings`
- `Safety concerns`
- `Recommended architecture changes`
- `Implementation ordering corrections`
- `Tests required before coding`

Sol 同样**不得修改源码**。

---

# 9. 当前主窗口（Orchestrator）的职责

主窗口是唯一 architectural owner，不是另一个独立意见模型，也不是只负责把几个 agent 的建议拼起来。

必须按下面顺序工作：

1. 完整读取本 Brief；
2. 启动 Luna Max 做**全仓库、全目标架构**的只读 evidence audit；
3. Luna 完成后，把本 Brief + Luna 报告交给 Sol；
4. Sol 必须独立重新看源码做 red-team review；
5. 主窗口综合 Brief、Luna、Sol、当前 tests/logs，先产出一份 **Unified Architecture Freeze**；
6. Architecture Freeze 必须一次性冻结：
   - layer ownership；
   - core data structures / interfaces；
   - geometric Tube / preview / allocator / handoff / matched injection contract；
   - zero-only semantics；
   - failure-state taxonomy；
   - old contract -> new contract migration matrix；
   - 哪些旧模块删除、重写、保留；
7. 在 Architecture Freeze 基础上再给出 Batch A--D 的 authoritative execution plan；
8. 用户明确同意前，不修改 production code；
9. 用户同意后，在同一个 refactor branch 上按 Batch A -> B -> C -> D 落地；
10. 每个 Batch 都必须 build、unit test、ROS smoke test、diff self-review，并有独立 rollback commit；
11. Batch B/C 完成后优先使用 Sol 做只读 mini-audit；
12. 除非出现新的源码证据证明 Architecture Freeze 有错误，否则实施阶段不得重新引入旧 gate/certificate 或临时改变跨层接口。

主窗口最终 execution plan 必须为每个 Batch 列出：

- files touched；
- interfaces changed / removed / introduced；
- exact semantic changes；
- ownership changes；
- invariants restored；
- tests added/changed；
- expected ROS/runtime behavior；
- acceptance criteria；
- rollback point；
- explicit non-goals。

还必须单独给出一张 **old -> new responsibility migration table**，例如：

- old Runtime future DFS 的哪些部分迁移到 Preview/Transition；
- old Certified display gate 的哪些部分变成 geometric visualization；
- old manual profile 的哪些部分只保留在 test；
- old PortProjector 的哪些约束进入最终 allocator；
- old PathTubePair authority 中哪些 planner veto 被删除；
- current-state explicit unsafe 由哪一层继续拥有 fail-safe authority。

最重要的 review 问题不是“某个旧测试还能不能过”，而是：

> 这个测试保护的是最终 II--IV 的正确 contract，还是在保护已经确认错误的临时实现？

# 10. 最终 acceptance scenarios

至少建立下面 8 组测试场景。

## A. Straight open corridor

预期：

- nonzero Tube；
- \(\delta\) 可平滑展开；
- planner 正常推进。

## B. Open -> narrow -> open

预期：

- Tube 连续收缩；
- \(\delta\) 连续压缩；
- \(u_w\) 必要时降低；
- narrow 部分可退化到 near-zero / zero-only；
- 出通道后平滑恢复展开；
- 无不必要 HOLD。

## C. Zero-only but valid planner path

预期：

- geometric Tube classification = zero-only；
- planner centerline 继续导航；
- \(\delta\to0\)；
- 不因“没有 nonzero capacity”切 HOLD。

## D. Replan while \(\delta\neq0\)

预期：

- successor path/tube 若不立即包含 retained \(\delta\)，进入 recenter transition；
- old safe owner 暂时保留；
- 不把 old port 直接硬塞到 new profile；
- handoff ready 后再切换。

## E. Near-vertical 3D path

预期：

- normal field 不退化；
- Tube 可构建；
- \(N\) 无随机 sign flip；
- \(r_w\) regularity 正常。

## F. Disconnected transverse free space

预期：

- 不跨 unsafe gap convexify；
- 当前 \(\delta\) 所在 component 被连续保留；
- 初始化时自然用 zero component。

## G. Dynamic preview temporarily infeasible

预期：

- geometric Tube 保持有效；
- allocator/transition 降速或 recenter；
- 不把 preview failure 改写成 Tube geometry failure。

## H. Pairwise conflict in bottleneck

预期：

- separation / Tube-aware cohesion / radial damping 形成 nominal intent；
- pairwise CBF 单独 enforce safety；
- Tube compression + longitudinal redistribution；
- matched tracking dynamics 保持。

---

# 11. 期望最终得到的代码语义

如果这个重构完成，运行时应该可以用一句话解释：

> Planner 决定“中心路径往哪走”，Path-Tube 决定“横向还能动多少”，preview 决定“是否来得及横向调整”，QP 决定“这一拍纵向和横向各动多少”，matched port 确保这些 swarm coordination 不破坏 nominal ISF-GVF tracking dynamics。

也就是说：

\[
\boxed{
\text{Tube loss of width}
\neq
\text{planner path loss}
}
\]

而应该是：

\[
\boxed{
\text{Tube narrows}
\Rightarrow
\text{continuous compression / recentering}
}
\]

只有 planner 本身无安全路径、当前物理状态明确 unsafe、或 safety-level constraints 确实无法满足时，才允许进入真正的 stop / replan / emergency behavior。
