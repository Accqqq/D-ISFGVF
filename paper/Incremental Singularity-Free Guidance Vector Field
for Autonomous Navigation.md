\documentclass[letterpaper,journal]{IEEEtran}
\usepackage{amsmath,amsfonts}
\usepackage{algorithmic}
\usepackage{algorithm}
\usepackage{array}
\usepackage[caption=false,font=normalsize,labelfont=sf,textfont=sf]{subfig}
\usepackage{textcomp}
\usepackage{stfloats}
\usepackage{url}
\usepackage{verbatim}
\usepackage{graphicx}
\hyphenation{op-tical net-works semi-conduc-tor IEEE-Xplore}
\usepackage[style=numeric,sorting=none]{biblatex}
\addbibresource{sn-bibliography.bib}
\usepackage{wrapfig}
\usepackage{multirow}%
\usepackage{mathrsfs}%
\usepackage{manyfoot}%
\usepackage{booktabs}%
\usepackage{listings}%
\usepackage{makecell}
\usepackage{color}
\usepackage{fontawesome}
\usepackage{xcolor}
\usepackage{epstopdf}
\usepackage{siunitx}
\usepackage{amsthm}
\newtheorem{remark}{Remark}
\epstopdfDeclareGraphicsRule{.tif}{png}{.png}{convert #1 \OutputFile}

\begin{document}

\title{Incremental Singularity-Free Guidance Vector Field for Autonomous Navigation}

\author{Xinqing Chen

\thanks{Manuscript received 4 December 2024. This research was funded in part by the National Natural Science Foundation of China under Grant 62173234, in part by the Shenzhen Science and Technology Program under Grant 20220818095816035, and in part by the Guangdong Major Project of Basic and Applied Basic Research under Grant 2023B1111050010.(Corresponding author: Yu Hu.)}% <-this % stops a space
\thanks{Bo Zhang is with the College of Mechatronics and Control Engineering, Shenzhen University, Shenzhen 518060, China, the Shenzhen City Joint Laboratory of Autonomous Unmanned Systems and Intelligent Manipulation, Shenzhen 518060, China and the Guangdong Provincial Laboratory of Artificial Intelligence and Digital Economy (Shenzhen), Shenzhen 518060, China. (email: zhangbo@szu.edu.cn)}
\thanks{Xiao Bowen, Jinshi Qiu and Sicheng Tan are with the College of Mechatronics and Control Engineering, Shenzhen University, Shenzhen 518060, China, the Shenzhen City Joint Laboratory of Autonomous Unmanned Systems and Intelligent Manipulation, Shenzhen 518060, China. (email:2310295054@email.szu.edu.cn; 2110296027@email.szu.edu.cn; 2200292010@email.szu.edu.cn)}
\thanks{Yu Hu is with Guangdong Laboratory of Artificial Intelligence and Digital Economy (SZ), Shenzhen, China. (email:huyu@gml.ac.cn)}}


\markboth{Journal of \LaTeX\ Class Files,~Vol.~14, No.~4, December~2024}%
{Shell \MakeLowercase{\textit{et al.}}:Incremental singularity-free guidance vector field for autonomous navigation}

\maketitle


\begin{abstract}
Autonomous navigation becomes challenging when the reference path is repeatedly adjusted during execution and may exhibit complex geometric structures such as closed, self-intersecting, looping, or revisiting segments. Under such conditions, conventional guidance methods may suffer from branch ambiguity, discontinuous guidance, or local degeneration. This paper proposes an incremental singularity-free guidance vector field (ISF-GVF) in a lifted state space for autonomous navigation under replanning and complex path geometries. The proposed field is progressively constructed and updated around the currently valid path as replanning proceeds, rather than being generated once for a fixed reference path. By explicitly incorporating path phase into the system state, the lifted-state formulation provides a singularity-free mechanism that separates physically overlapping but semantically distinct path branches and yields a nonvanishing guidance field with branch distinguishability and local nondegeneracy. The physical-space component of the lifted guidance law is further extracted as a unified geometric guidance interface, through which the proposed method serves as a unified guidance layer for different motion-constrained robotic platforms. The main analytical properties of the proposed ISF-GVF are established, including local regularity, exact error dynamics, singularity-free property, forward completeness, and asymptotic convergence to the reference path. The effectiveness of the proposed method is validated through RViz simulations and real-world platform experiments on heterogeneous motion-constrained robotic platforms, including a quadrotor UAV and a differential-drive ground robot.
\end{abstract}


\begin{IEEEkeywords}
Incremental singularity-free guidance vector field, lifted-state formulation, motion-constrained robotic platforms, complex path geometries, autonomous navigation.
\end{IEEEkeywords}

\section{Introduction}
\label{sec1}

\IEEEPARstart{A}{utonomous} navigation in real environments often requires a robot to follow a desired geometric path rather than a strictly time-parameterized trajectory. Such path-following tasks arise in aerial inspection, traffic monitoring, marine navigation, ground-robot patrol, and other robotic applications where the robot is expected to converge to and move along a specified spatial curve \cite{Nelson2007VectorField,Park2007NonlinearGuidance,Aguiar2007TrajectoryTracking}. Compared with trajectory tracking, path following treats the reference as a geometric object and does not require the robot to track a prescribed time history exactly \cite{Aguiar2007TrajectoryTracking,Lapierre2003NonlinearPathFollowing}. This distinction is important for robots operating under disturbances, local avoidance requirements, and platform-specific motion constraints.

Among different path-following approaches, vector-field-based and guiding-vector-field methods provide a direct geometric mechanism for navigation. In these methods, a vector field is constructed around the desired path so that the robot is guided toward the path and then propagated along it. Classical vector-field guidance has been used for miniature air vehicles \cite{Nelson2007VectorField}, nonlinear guidance laws have been developed for aircraft path following \cite{Park2007NonlinearGuidance}, and guiding-vector-field formulations have been extended to nonholonomic mobile robots and three-dimensional paths \cite{Kapitaniuk2018Guiding,Yao2020Path3D}. More recent extensions further improve vector-field guidance by considering moving geometric references, switched field structures, and inverse-kinematics-based shaping of the guidance behavior \cite{Kapitaniuk2017MovingPath,Basak2024SwitchedVectorField,Zhou2025InverseKinematicsGVF}. These methods are attractive because the vector field can be interpreted as an outer-loop guidance signal, while the robot-specific inner-loop controller or execution layer tracks the generated reference.

However, conventional vector-field guided path-following methods are usually defined directly in the physical workspace. This leads to fundamental difficulties when the desired path is closed, self-intersecting, looping, or revisiting. For such paths, a single physical position may correspond to multiple path phases and tangent directions, and the guidance direction may become ambiguous near overlapping branches. Existing work has shown that conventional vector-field path-following algorithms may contain singular points where the vector field vanishes \cite{Kapitaniuk2018Guiding,Yao2020Path3D}. More importantly, singularity-free GVF studies have shown that vector fields of the same dimension as the physical path cannot generally guarantee global convergence to simple closed or self-intersecting paths \cite{Yao2020SingularityElimination,Yao2021SingularityFree,Yao2022Topological}. These results indicate that the difficulty is not merely caused by poor gain tuning, but is related to the topology of the desired path.

To address this topological obstruction, singularity-free guiding-vector-field methods lift or transform the desired path into a higher-dimensional space. In this way, a closed or self-intersecting physical path can be represented as a nonself-intersecting lifted path, and a singularity-free guiding vector field can be constructed in the lifted space \cite{Yao2020SingularityElimination,Yao2021SingularityFree}. Further topological analysis has characterized the obstruction and the domain of attraction of vector-field guided path following on manifolds \cite{Yao2022Topological,Yao2023Domain}. Recent homotopy-based formulations also indicate that topological transformations remain an active route for constructing nonsingular guiding vector fields \cite{Chen2023HomotopyGVF}. These works provide an important theoretical basis for handling complex path geometries. Nevertheless, existing singularity-free GVF formulations mainly focus on constructing a singularity-free field for a given desired path. They do not explicitly formulate the lifted vector field as a reusable guidance interface for different motion-constrained robotic platforms.

Another related line of work introduces a path parameter, virtual target, or phase variable whose evolution is regulated together with the robot motion. Early path-parameter methods avoid purely time-based tracking by allowing a reference point to move along the path according to designed phase dynamics \cite{Lapierre2003NonlinearPathFollowing,Lapierre2006Nonsingular,Aguiar2007TrajectoryTracking}. Predictive path-following and contouring-control methods further include the path progress as an optimization variable and handle constraints within an MPC framework \cite{Faulwasser2016OutputPathFollowing,Greeff2018ModelPredictivePathFollowing,Liniger2015AutonomousRacing}. Recent MPCC-based flight and NMPC-based marine guidance studies continue this trend by combining path-progress optimization with platform constraints and safety requirements \cite{Krinner2024MPCCpp,Bejarano2022USVGuidance}. These methods recognize that geometric path following should not be reduced to rigid trajectory tracking. However, most of them assume that the active path branch is already well defined, or that branch consistency is implicitly handled by the optimization state. For closed, self-intersecting, looping, or revisiting paths, the phase variable must not only describe path progress, but also preserve branch-consistent guidance semantics.

The platform-realization issue is also essential in practical navigation systems. A quadrotor UAV can interpret a geometric guidance vector as a high-level position, velocity, or yaw reference, whereas a differential-drive ground robot must convert the same geometric intention into admissible forward and angular velocity commands under nonholonomic constraints. Existing path-following studies often rely on cascaded architectures in which an outer-loop guidance module generates geometric references and an inner-loop controller realizes them on a specific platform \cite{Nelson2007VectorField,Akhtar2012QuadrotorPathFollowing,Greeff2018ModelPredictivePathFollowing}. For nonholonomic mobile robots, the guidance output must be further mapped to feasible velocity commands \cite{Kapitaniuk2018Guiding}. Hence, a guidance method intended for different motion-constrained robotic platforms should not be designed as a platform-specific low-level controller. Instead, it should generate a platform-agnostic geometric guidance output that can be interpreted through different execution interfaces.

Motivated by these observations, this paper proposes an incremental singularity-free guidance vector field (ISF-GVF) formulated in a lifted state space. The key idea is to explicitly incorporate the path phase into the system state, so that physically overlapping but semantically distinct path branches become separable in the lifted space. The proposed field combines tangential advancement, transversal contraction, and phase correction to generate a nonvanishing lifted-space guidance vector field. Its physical-space component is extracted as a unified geometric guidance output, which can be mapped to different platform-specific execution interfaces. When the reference path is updated, the guidance field is reconstructed around the currently valid path while the internal phase state is retained. In this sense, the proposed method acts as an upper-layer guidance mechanism between the planner and the execution controller, rather than as a new replanner or a universal low-level controller.

It is useful to clarify the terminology used in this paper. The term complex path geometries refers to reference paths that may be closed, self-intersecting, looping, or revisiting. These paths are challenging because the same physical position may correspond to different path phases and different tangent directions. The term lifted state refers to the augmented state \((x,w)\), where \(x\) is the physical position and \(w\) is the internal path phase. The term unified guidance layer means that the proposed method generates a platform-agnostic geometric guidance output, rather than a platform-specific low-level control input. Therefore, the focus of this paper is the construction and realization of a reusable guidance mechanism for different motion-constrained robotic platforms.


The main contributions of this paper are summarized as follows:
\begin{itemize}
    \item We propose an incremental singularity-free guidance vector field for autonomous navigation under complex path geometries. The proposed method is designed as an upper-layer guidance mechanism rather than a platform-specific controller, and is suitable for closed, self-intersecting, looping, and revisiting reference paths.

    \item We develop a lifted-state guidance formulation that explicitly incorporates path phase into the system state. This formulation separates physically overlapping but semantically distinct path branches, removes branch ambiguity, and yields a nonvanishing guidance field with local nondegeneracy.

    \item We establish the main analytical properties of the proposed ISF-GVF, including local regularity, update-compatible smoothness, exact error dynamics, singularity-free property, forward completeness, global asymptotic convergence, and bounded-set convergence-rate estimates under additional sector conditions.

    \item We extract the physical-space component of the lifted guidance law as a unified geometric guidance interface for different motion-constrained robotic platforms. The same guidance layer is realized on heterogeneous platforms, including a quadrotor UAV and a differential-drive ground robot, through platform-specific execution interfaces.
\end{itemize}

The remainder of this paper is organized as follows. Section~II formulates the problem and presents the overall system architecture. Section~III develops the proposed ISF-GVF in the lifted state space, including its construction, singularity-free mechanism, incremental update process, and unified geometric guidance output. Section~IV analyzes the main theoretical properties of the proposed method. Section~V presents the guidance-to-control realization on motion-constrained robotic platforms. Section~VI provides experimental validation through RViz simulations and real-world platform experiments, and Section~VII concludes the paper.


\iffalse
\section{Problem Formulation and System Architecture}\label{sec2}

\subsection{Problem Statement}\label{subsec21}
\iffalse
本文研究一种面向路径在线更新与复杂轨迹几何的增量式无奇异引导向量场方法。不同于针对单条固定参考路径离线设计的传统引导方法，本文关注的是：当参考路径可能因局部绕障、重规划或任务变化而持续更新，且轨迹可能包含闭合、自交、回环或重访结构时，如何构造一种能够持续工作、保持明确引导语义并可供不同受运动学约束机器人平台复用的统一引导层。
\fi
This paper addresses an incremental singularity-free guidance vector field for online-updated paths and geometrically complex trajectories. Unlike conventional guidance laws designed offline for a single fixed reference path, the focus here is to construct a unified guidance layer that remains operational when the reference path is repeatedly updated by local replanning, obstacle avoidance, or task changes, and when the path may contain closed, self-intersecting, looping, or revisiting structures.

\iffalse
现有引导方法通常默认两项条件成立：其一，参考路径在一段时间内保持不变，因此引导场可围绕该路径一次性构造。其二，轨迹几何足够简单，因此物理空间中的当前位置对应唯一且连续的推进方向。然而在实际机器人系统中，这两项条件往往难以同时满足。路径可能因在线建图和局部重规划而持续变化，复杂闭合轨迹则可能使同一物理位置对应多个不同路径相位及切向方向，从而引发分支歧义、引导突变或局部退化。
\fi
Many existing guidance methods implicitly rely on two assumptions. First, the reference path remains unchanged over a sufficiently long interval, so the guidance field can be constructed once around that path. Second, the path geometry is simple enough that each physical location corresponds to a unique and continuous advancing direction. In practical robotic systems, both assumptions may fail simultaneously. Online mapping and local replanning continuously update the path, while complex closed trajectories may cause a single physical location to correspond to multiple path phases and tangent directions, thereby inducing branch ambiguity, guidance discontinuity, or local degeneration.

\iffalse
因此，本文的核心问题是：在路径持续变化且参考轨迹可能具有复杂闭合结构的条件下，构造一种增量式、无奇异的引导向量场，并使其作为受运动学约束机器人平台的统一几何引导层。本文中，“增量式”指引导场能够围绕当前有效路径持续更新，并通过误差反馈实时修正相位推进；“无奇异”指引导场在提升状态空间中处处非零，并在复杂轨迹条件下保持分支可区分和局部导航语义不退化；“统一几何引导层”指其输出是平台无关的几何引导结果，而非某一平台的最终控制量。
\fi
The problem considered in this paper is therefore to construct an incremental and singularity-free guidance vector field that can serve as a unified geometric guidance layer for motion-constrained robotic platforms under path updates and complex closed-path geometries. Here, “incremental” means that the guidance field can be continuously updated around the current path and can adapt its phase evolution through error feedback. “Singularity-free” means that the lifted vector field is nonvanishing over the lifted state space and preserves branch distinguishability and local navigation semantics under complex path geometries. “Unified geometric guidance layer” means that the output is a platform-agnostic geometric guidance signal rather than a universal executable control input.


\subsection{Path Representation and Platform-Agnostic Requirement} 
\iffalse
尽管不同机器人平台在工作空间维数、运动学约束和控制接口方面存在显著差异，其前端模块输出的路径都可统一抽象为一条连续参数曲线
p: ℝ→ℝn
其中 $n$ 由平台工作空间维数决定。对引导层而言，关键不在于路径由何种规划器生成，而在于路径能否被表示为连续的参数化几何对象。
\fi
Although robotic platforms differ in workspace dimension, kinematic constraints, and low-level control interfaces, the paths generated by their front-end modules can all be abstracted as continuous parametric curves:
\[
p:\mathbb{R}\rightarrow\mathbb{R}^{n},
\]

where $n$ is determined by the workspace dimension of the platform. From the viewpoint of the guidance layer, the key issue is not which planner generates the path, but whether the path can be represented as a continuous parameterized geometric object.

\iffalse
基于这一认识，本文将整体系统划分为前端路径生成模块、引导层和控制执行层。前端负责生成当前可用路径；引导层依据当前位置、参考路径与路径相位之间的几何关系输出统一几何引导；控制层则进一步将该引导结果映射为平台可执行命令。这里所说的平台无关，并不是说所有机器人共享同一控制输入，而是说它们共享同一套上层几何语义。
\fi
Based on this observation, the overall system is decomposed into a front-end path-generation module, a guidance layer, and an execution controller. The front end provides the current path, the guidance layer generates a unified geometric guidance signal from the geometric relation among the robot position, the reference path, and the path phase, and the control layer maps this guidance signal to platform-executable commands. Hence, platform agnosticism here refers to a common upper-layer geometric semantics rather than a shared final control input.


\subsection{Necessity of a Singularity-Free Guidance Layer} 
\iffalse
对于闭合、自交、回环或重访轨迹，同一物理位置可能对应多个路径相位，而这些相位通常具有不同切向方向。因此，仅凭当前位置往往无法唯一判断系统所属的路径分支，也无法可靠决定当前应沿哪个方向继续推进。若仍仅在物理空间内定义单值推进方向，则容易在分支区域产生错误切换、在自交附近引发方向突变，并导致局部导航语义退化。
\fi
For closed, self-intersecting, looping, or revisiting trajectories, a single physical location may correspond to multiple path phases, and these phases generally induce different tangent directions. Therefore, the current position alone is often insufficient to determine which branch of the path is active or which direction should be followed. If a single-valued advancing direction is still defined only in physical space, incorrect branch switching, abrupt direction changes near self-intersections, and local degradation of navigation semantics may occur.

\iffalse
路径在线更新会进一步放大上述问题。若引导层没有显式建模路径相位，则路径更新后系统通常只能在新路径上重新选取与当前运动状态并不一致的引导点，从而引入额外对齐过程甚至控制突变。
\fi
Online path updates further aggravate this issue. Without an explicit phase model, the guidance layer typically has to reselect a guidance point on the updated path, which may be inconsistent with the current motion state and may introduce additional transients or even control discontinuities.

\iffalse
因此，本文不再尝试仅在物理空间中构造传统单值引导场，而是引入同时编码位置信息与路径进度信息的提升状态表示。这样，物理空间中重合但路径语义不同的分支可在扩展状态空间中被区分，路径更新前后的引导进度也获得了连续延续的基础。本文中的“无奇异”既体现为提升空间中的向量场处处非零，也体现为复杂轨迹条件下分支可区分且局部语义不退化。
\fi
For this reason, the proposed method does not rely on a conventional single-valued guidance field defined only in physical space. Instead, it introduces a lifted-state representation that jointly encodes position and path progress. In the lifted space, branches that overlap in physical space but correspond to different path semantics become distinguishable, and phase continuity across path updates can be explicitly maintained. Accordingly, the singularity-free property in this paper refers not only to the nonvanishing nature of the lifted vector field, but also to branch distinguishability and local semantic nondegeneracy under complex path geometries.


\subsection{Hierarchical System Architecture} 
\iffalse
基于上述问题定义，本文采用三层架构组织导航系统：前端路径生成模块、中间引导层以及下游控制执行层。前端根据环境和任务信息生成当前路径；中间层围绕该路径构造增量式无奇异引导向量场，并输出统一几何引导结果；控制层结合具体平台约束将其转换为可执行命令。
\fi
Based on the above formulation, the navigation system is organized into three layers: a front-end path-generation module, a middle guidance layer, and a downstream execution controller. The front end generates the current path from task and environment information. The middle layer constructs an incremental singularity-free guidance vector field around that path and outputs a unified geometric guidance signal. The control layer then converts this signal into executable commands subject to the constraints of the specific platform.

\iffalse
本文的核心贡献集中于中间引导层，而非前端规划器或某一特定平台的底层控制器。在线建图与重规划仅提供路径持续变化的应用背景，而四旋翼与差速平台的控制实现则用于说明同一引导层如何通过不同接口在异构平台上落地执行。
\fi
The main contribution of this paper lies in the middle guidance layer rather than in the front-end planner or in a platform-specific low-level controller. Online mapping and replanning only provide an application setting in which the path changes over time, whereas the quadrotor and differential-drive realizations are used to demonstrate how the same guidance layer can be executed through different interfaces on heterogeneous robotic platforms.
\fi
\section{Problem Formulation and System Architecture}
\label{sec2}

\subsection{Problem Statement}
\label{subsec21}

This paper considers the guidance problem for autonomous navigation on different motion-constrained robotic platforms under complex path geometries. The reference path provided by the planning layer may be closed, self-intersecting, looping, or revisiting. In such cases, the same physical position may correspond to different path phases and different tangent directions. Therefore, a guidance law defined only in the physical workspace may lose branch distinguishability and may generate ambiguous or degenerate guidance directions.

The objective of this paper is to construct an upper-layer guidance mechanism that remains well defined under such complex path geometries and can be realized on different robotic platforms. The proposed method is not intended to replace the front-end planner or the platform-specific low-level controller. Instead, it provides a reusable geometric guidance layer between the planning layer and the execution layer. The guidance layer receives the currently valid reference path and the estimated robot state, constructs a lifted-state guidance vector field, and outputs a platform-agnostic geometric guidance signal.

In this work, the term \emph{incremental} means that the guidance field can be reconstructed around the currently valid path segment when the front-end planner updates the reference path. The term \emph{singularity-free} means that the lifted guidance field is nonvanishing and preserves branch distinguishability for complex path geometries. The term \emph{unified guidance layer} means that the same guidance formulation can be connected to different motion-constrained robotic platforms through different execution interfaces.

\subsection{Path Representation and Guidance Objective}
\label{subsec22}

Let the current reference path be represented by a continuous parametric curve
\begin{equation}
p_k:\mathbb{R}\rightarrow\mathbb{R}^{n},
\label{eq:path_representation_sec2}
\end{equation}
where \(k\) denotes the current path index or update instance, \(w\in\mathbb{R}\) is the path phase, and \(n\) is the dimension of the physical workspace. For a quadrotor UAV, \(n\) may correspond to the two- or three-dimensional position space depending on the task setting. For a ground robot, \(n=2\) is used for planar navigation.

The path \(p_k(w)\) is treated as a geometric object rather than a time-indexed trajectory. Therefore, the robot is not required to track a preassigned point at a prescribed time. Instead, the guidance objective is to drive the robot toward the currently active path branch and propagate it along the path with consistent phase evolution.

To encode both the physical position and the path phase, the proposed method introduces the lifted state
\begin{equation}
z=(x,w)\in\mathbb{R}^{n}\times\mathbb{R},
\label{eq:lifted_state_sec2}
\end{equation}
where \(x\in\mathbb{R}^{n}\) is the robot position and \(w\) is the internal path phase. The use of \(w\) is essential for complex path geometries. If two different phases \(w_1\neq w_2\) satisfy \(p_k(w_1)=p_k(w_2)\), then the two corresponding path branches overlap in physical space but remain distinguishable in the lifted state space through their different phase coordinates.

The guidance problem is therefore formulated as the design of a lifted vector field that drives \((x,w)\) toward the lifted path associated with \(p_k(w)\), while generating a physical-space guidance output that can be interpreted by different platform execution layers.

\subsection{Layered System Architecture}
\label{subsec23}

\begin{figure*}[!t]
    \centering
    \includegraphics[width=0.95\textwidth]{Cxq_pic1.png}
    \caption{Overall architecture of the proposed ISF-GVF framework. The front-end planning layer provides the currently valid path, the ISF-GVF guidance layer constructs the lifted-state guidance field and outputs the geometric guidance signal \(v_g\), and the motion-constrained platform layer maps this signal to platform-executable commands.}
    \label{fig:overall_architecture}
\end{figure*}

The overall architecture of the proposed framework is shown in Fig.~\ref{fig:overall_architecture}. The system is organized into three layers: the front-end planning layer, the ISF-GVF guidance layer, and the motion-constrained platform layer.

The front-end planning layer provides the currently valid path according to the navigation task, online mapping information, and local replanning results. In this paper, the front end is used to supply a feasible geometric reference for the guidance layer. The proposed method does not depend on a specific planner and does not require the front-end module to solve the guidance problem. When a new path segment is available, the guidance layer updates the reference path from \(p_k\) to \(p_{k+1}\).

The ISF-GVF guidance layer is the main focus of this paper. As illustrated in the middle part of Fig.~\ref{fig:overall_architecture}, this layer first represents the currently valid path as a phase-parameterized curve \(p_k(w)\). It then constructs the lifted state \(z=(x,w)\), decomposes the local tracking error into tangential and transversal components, and synthesizes the ISF-GVF. The resulting lifted guidance vector consists of the physical-space guidance component \(v_g\) and the internal phase rate \(\dot w\). The phase rate is used internally to update the lifted state, whereas \(v_g\) is passed to the platform execution layer as the unified geometric guidance output.

The motion-constrained platform layer maps the guidance output to commands compatible with the corresponding robot. For a quadrotor UAV, \(v_g\) can be converted into a bounded high-level position or velocity reference and executed through the flight-control interface. For a differential-drive ground robot, the same geometric guidance information must be converted into admissible linear and angular velocity commands under the nonholonomic constraint. The state-estimation and perception modules provide the robot state and environmental information required by the planning and guidance layers.

This architecture separates the geometric guidance problem from both path generation and low-level execution. The front-end planner determines the currently valid path, the ISF-GVF layer determines how the robot should move relative to that path, and the platform layer determines how the guidance output is executed under platform-specific constraints.

\subsection{Scope of This Work}
\label{subsec24}

The contribution of this paper lies in the ISF-GVF guidance layer. The proposed method assumes that the front-end planning layer can provide a continuous parametric path or a path segment that can be reparameterized as \(p_k(w)\). The method also assumes that the platform layer can track or approximate the generated geometric guidance output through its own execution interface.

Under this separation, the proposed framework does not aim to design a new mapping algorithm, a new obstacle-detection module, or a universal low-level controller. Instead, it addresses the intermediate guidance problem: how to construct a phase-consistent, singularity-free, and platform-agnostic geometric guidance field for complex path geometries. This formulation allows the same ISF-GVF guidance layer to be used on heterogeneous motion-constrained robotic platforms while leaving platform-specific execution details to the corresponding control layer.


\section{Incremental Singularity-Free Guidance Vector Field}
\label{sec3}

This section develops the ISF-GVF guidance layer shown in Fig.~\ref{fig:overall_architecture}. The objective is to construct a lifted-state vector field that provides phase-consistent guidance for complex path geometries and outputs a platform-agnostic geometric guidance signal for the downstream execution layer.

On an inter-update interval, the currently valid reference path is denoted by \(p_k(w)\). For notational simplicity, the subscript \(k\) is omitted in this section, and the path is written as \(p(w)\). When a new path segment becomes available, the guidance field is reconstructed around the updated path, as described in Section~\ref{subsec34}.

\subsection{Lifted-State Representation and Error Decomposition}
\label{sec31}

To construct the guidance field, the robot position and the path phase are represented jointly by the lifted state
\begin{equation}
z=(x,w)\in\mathbb{R}^{n}\times\mathbb{R},
\label{eq:lifted_state}
\end{equation}
where \(x\in\mathbb{R}^{n}\) is the physical position and \(w\in\mathbb{R}\) is the internal phase variable. The corresponding lifted-path set is defined as
\begin{equation}
\Gamma:=\{(p(w),w)\mid w\in\mathbb{R}\}.
\label{eq:lifted_path_set}
\end{equation}

The set \(\Gamma\) describes the ideal consistency relation between the robot position and the path phase. Instead of defining guidance solely in the physical space, the proposed method designs a vector field in the lifted space so that the state \((x,w)\) is driven toward \(\Gamma\) while being propagated along it.

The phase coordinate is essential for complex path geometries. If there exist \(w_1\neq w_2\) such that
\begin{equation}
p(w_1)=p(w_2),
\label{eq:phase_ambiguity}
\end{equation}
then the two cases are indistinguishable in physical space but correspond to different lifted states, namely \((p(w_1),w_1)\) and \((p(w_2),w_2)\). Therefore, the lifted-state representation separates physically overlapping but semantically distinct path branches.

Given the current state \((x,w)\), define the phase-dependent tracking error as
\begin{equation}
e=x-p(w).
\label{eq:tracking_error_def}
\end{equation}
This error is measured with respect to the path point associated with the current phase, rather than with respect to the nearest point on the whole path. This definition is suitable for branch-consistent guidance because the active path branch is determined by \(w\).

When \(p'(w)\neq 0\), define the unit tangent vector as
\begin{equation}
\tau(w)=\frac{p'(w)}{\|p'(w)\|}.
\label{eq:unit_tangent_def}
\end{equation}
The tracking error is decomposed into tangential and transversal components:
\begin{equation}
e_{\parallel}=\tau(w)^{\top}e,\qquad
e_{\perp}=\left(I-\tau(w)\tau(w)^{\top}\right)e,
\label{eq:error_decomposition}
\end{equation}
so that
\begin{equation}
e=e_{\parallel}\tau(w)+e_{\perp}.
\label{eq:error_reconstruction}
\end{equation}
Further define
\begin{equation}
\rho=\|e_{\perp}\|.
\label{eq:rho_definition}
\end{equation}

Here, \(e_{\parallel}\) measures the longitudinal mismatch between the robot and the current phase point, whereas \(e_{\perp}\) measures the lateral deviation from the current path branch. In the proposed ISF-GVF, \(e_{\perp}\) is used to construct the transversal contraction term, while \(e_{\parallel}\) is used to regulate the phase evolution. This decomposition provides the geometric basis for the guidance-field construction.

\subsection{Construction of the ISF-GVF}
\label{subsec32}

Based on the lifted-state representation, the proposed incremental singularity-free guidance vector field is defined as
\begin{equation}
\chi_{\mathrm{ISF}}(x,w)=
\begin{bmatrix}
\dot{x}\\[2mm]
\dot{w}
\end{bmatrix}
=
\begin{bmatrix}
K_1\alpha(\rho^2)\tau(w)-K_2q_r(\rho)e_{\perp}\\[2mm]
\dfrac{K_1\alpha(\rho^2)+K_1\sigma(e_{\parallel})}{\|p'(w)\|}
\end{bmatrix}
\label{eq:isf_gvf}
\end{equation}
In \eqref{eq:isf_gvf}, \(K_1>0\) and \(K_2>0\) are the tangential-advancement and lateral-contraction gains, respectively, \(\alpha : [0,\infty)\to(0,1]\) is a progress-gating function defined with respect to the squared transversal error \(\rho^2\), \(\sigma:\mathbb{R}\to\mathbb{R}\) is a bounded odd function used to regulate the phase-advancement speed according to the tangential error, and
\begin{equation}
q_r(\rho) =
\begin{cases}
\dfrac{\tanh(\rho/r)}{\rho}, & \rho > 0, \\[2mm]
\dfrac{1}{r}, & \rho = 0,
\end{cases}
\qquad r > 0
\label{eq:lateral_contraction_profile}
\end{equation}
is the lateral-contraction profile.

The use of \(\alpha(\rho^2)\), rather than a direct dependence on \(\rho=\|e_{\perp}\|\), is adopted to avoid nonsmooth radial dependence at \(e_{\perp}=0\), which is useful for the update-regularity analysis in Section~IV.

The first $n$ components of $\chi_{\mathrm{ISF}}$ define the physical-space guidance law, whereas the last component governs the evolution of the internal phase variable.The term $K_1\alpha(\rho^2)\tau(w)$ advances the state along the path tangent at the current phase point. The gating function $\alpha(\rho^2)$ modulates this advancement according to the squared transversal deviation, so that tangential propagation is preserved near the path and suppressed when the state is far from the path. The term $-K_2 q_r(\rho)e_{\perp}$ is the lateral-contraction term and drives the state toward the current path branch. Since
\begin{equation}
q_r(\rho)\rho = \tanh(\rho/r),
\label{eq:contraction_boundedness}
\end{equation}
this term remains bounded far from the path while remaining continuous and well defined near the path.

The phase-update law is given by the second component of $\chi_{\mathrm{ISF}}$. The nominal term $K_1\alpha(\rho^2)/\|p'(w)\|$ advances the phase consistently with the tangential propagation in physical space, while the correction term $K_1\sigma(e_{\parallel})/\|p'(w)\|$ adjusts the phase speed according to the tangential mismatch. As a result, the proposed field regulates both the geometric deviation from the path and the consistency between the current phase and the actual progression of the robot.

Because the phase evolution is determined by the current error state rather than by a prescribed clock, the resulting field is not a static law defined once for a fixed path, but a dynamic guidance layer that remains applicable under path updates. The proposed ISF-GVF therefore integrates tangential advancement, lateral contraction, and phase correction into a unified lifted-space vector field.

\subsection{Geometric Interpretation and Singularity-Free Mechanism}
\label{subsec33}

The singularity-free property of the proposed ISF-GVF should be understood as a structural consequence of the lifted-state formulation and the vector-field construction, rather than as a purely numerical regularization. As illustrated in Fig.~\ref{fig:singularity_free_mechanism}, its mechanism consists of two complementary aspects: branch distinguishability in the lifted state space and nonvanishing local guidance in the physical-space component.

\begin{figure*}[!t]
    \centering
    \includegraphics[width=0.95\textwidth]{Cxq_pic2.png}
    \caption{Geometric interpretation of the proposed singularity-free mechanism. 
    (a) In physical space, two different phases may correspond to the same point, i.e., \(p(w_1)=p(w_2)\) with \(w_1\neq w_2\), leading to ambiguous tangent directions. 
    (b) In the lifted state space, phase-distinct states are separated by the phase coordinate. 
    (c) The ISF-GVF physical component combines tangential advancement and transversal contraction. Since \(e_{\perp}\) is orthogonal to \(\tau(w)\) and \(K_1\alpha(\rho^2)>0\), the two components cannot cancel each other, yielding a nonvanishing guidance vector.}
    \label{fig:singularity_free_mechanism}
\end{figure*}

First, the lifted-state representation removes branch ambiguity by explicitly incorporating the path phase into the system state. For closed, self-intersecting, looping, or revisiting trajectories, a single physical position may correspond to multiple path phases and tangent directions. If guidance were defined only as a single-valued field in physical space, the active branch near such overlapping regions could become ambiguous. In contrast, the proposed formulation represents physically overlapping but phase-distinct path points as different lifted states. Therefore, the active branch is encoded directly by the phase variable, which provides the geometric basis for branch-consistent guidance.

Second, the vector-field structure prevents local degeneration of the guidance semantics. In the physical-space component of the proposed ISF-GVF, the tangential-advancement term \(K_1\alpha(\rho)\tau(w)\) and the transversal-contraction term \(-K_2q_r(\rho)e_{\perp}\) act along orthogonal geometric directions because \(e_{\perp}\) is constructed to be orthogonal to \(\tau(w)\). Since \(K_1>0\) and \(\alpha(\rho)\in(0,1]\), the tangential component remains nonzero and cannot be canceled by the transversal component. This structural separation is the geometric reason behind the nonvanishing property established later in Section~IV-C.

Accordingly, the singularity-free property in this paper has a stronger meaning than the pointwise nonzero condition alone. It also includes branch distinguishability and local nondegeneracy under closed, self-intersecting, looping, or revisiting path geometries. This interpretation clarifies why both the lifted-state representation and the ISF-GVF construction are essential to the proposed method, and provides the geometric basis for the theoretical analysis presented in Section~IV.


\subsection{Incremental Update with \(C^2\)-Compatible Stitching}
\label{subsec34}

The incremental aspect of the proposed method refers to the update of the guidance field when the currently valid reference path changes. In the architecture of Fig.~\ref{fig:overall_architecture}, the front-end planning layer may provide a new path segment during navigation. The ISF-GVF layer then reconstructs the guidance field around the updated path while retaining the internal phase state.

Suppose that a path update occurs at time \(t_k\), and the reference curve changes from \(p_k(w)\) to \(p_{k+1}(w)\). The phase state is not reinitialized, but is continued as
\begin{equation}
w(t_k^+)=w(t_k^-).
\label{eq:phase_continuity_update}
\end{equation}
This phase continuation preserves the accumulated guidance progress and avoids restarting the guidance process after a path update. However, phase continuation alone does not guarantee smooth guidance-vector switching under an arbitrary hard path replacement, because the path-dependent quantities may change discontinuously at the retained phase.

To obtain a smooth incremental update, the accepted new path is connected to the previous path through a \(C^2\)-compatible stitching segment. Specifically, the stitching segment is constructed to match the position, first derivative, and second derivative of the previous and updated paths at the connection boundaries. The currently valid path used by the ISF-GVF after the update is therefore the stitched path \(\tilde p_{k+1}\), rather than a direct hard replacement of \(p_k\) by \(p_{k+1}\). The corresponding regularity result is analyzed in Section~\ref{subsec:update_regularity}.

The resulting incremental guidance update is summarized in Algorithm~\ref{alg:isfgvf_construction}.

\begin{algorithm}[!t]
\caption{Incremental ISF-GVF Construction}
\label{alg:isfgvf_construction}
\begin{algorithmic}[1]
\STATE \textbf{Input:} state \(x\), phase \(w\), path pair \((p_k,p_{k+1})\)
\STATE \textbf{Parameters:} gains \(K_1,K_2\), bandwidth \(r\)
\STATE \textbf{Output:} lifted guidance vector \(\chi_{\mathrm{ISF}}=(v_g,\dot w)\)

\STATE \textbf{// Path update and phase continuation}
\IF{\(p_{k+1}\) is available at \(t_k\)}
    \STATE Construct a \(C^2\)-compatible stitched path \(\tilde p_{k+1}\)
    \STATE \(p \leftarrow \tilde p_{k+1}\)
    \STATE \(w(t_k^+) \leftarrow w(t_k^-)\)
\ELSE
    \STATE \(p \leftarrow p_k\)
\ENDIF

\STATE \textbf{// Local path geometry}
\STATE \(p_w \leftarrow p(w)\)
\STATE \(\tau \leftarrow p'(w)/\|p'(w)\|\)

\STATE \textbf{// Error decomposition}
\STATE \(e \leftarrow x-p_w\)
\STATE \(e_{\parallel} \leftarrow \tau^{\top}e\)
\STATE \(e_{\perp} \leftarrow e-e_{\parallel}\tau\)
\STATE \(\rho \leftarrow \|e_{\perp}\|\)

\STATE \textbf{// ISF-GVF synthesis}
\STATE \(v_g \leftarrow K_1\alpha(\rho^2)\tau-K_2q_r(\rho)e_{\perp}\)
\STATE \(\dot w \leftarrow
\dfrac{K_1\alpha(\rho^2)+K_1\sigma(e_{\parallel})}{\|p'(w)\|}\)
\STATE \(\chi_{\mathrm{ISF}} \leftarrow (v_g,\dot w)\)
\end{algorithmic}
\end{algorithm}

\begin{figure}[!t]
    \centering
    \includegraphics[width=\linewidth]{Cxq_pic3.png}
    \caption{Incremental update of the proposed ISF-GVF under a path change. 
    (a) The guidance field is constructed around the current reference path \(p_k(w)\). 
    (b) When the reference path is updated to \(p_{k+1}(w)\), the guidance field is reconstructed around the updated path while the internal phase state is retained, i.e., \(w(t_k^+)=w(t_k^-)\). 
    The faded path and vectors denote the previous path and its associated guidance field before the update.}
    \label{fig:incremental_update}
\end{figure}

As illustrated in Fig.~\ref{fig:incremental_update}, the path update changes the geometric reference used by the guidance layer, but it does not reset the internal phase. A direct reinitialization of the phase may select a guidance point that is inconsistent with the current robot state, which can introduce unnecessary transients in the guidance output. Therefore, the proposed update preserves the internal phase and reconstructs the ISF-GVF on the stitched currently valid path.

The retained phase state provides phase continuity across path updates, while the \(C^2\)-compatible stitched path provides geometric compatibility between the previous and updated references. After the update, the quantities \(p(w)\), \(\tau(w)\), \(e_{\parallel}\), \(e_{\perp}\), and \(\rho\) are recomputed with respect to the stitched currently valid path, and the ISF-GVF is synthesized accordingly. In this way, the guidance field adapts to the updated reference without performing a hard replacement of the path geometry.

It should be emphasized that phase continuation does not require \(w\) to increase monotonically. Since \(w\) is a closed-loop internal state of the guidance layer, its evolution is regulated by the tangential mismatch through the correction term \(\sigma(e_{\parallel})\). When the robot state and the current phase point are inconsistent, the phase evolution may slow down or temporarily reverse to recover phase consistency.

Accordingly, the incremental update mechanism consists of two operations: the currently valid path is updated through a \(C^2\)-compatible stitched reference, and the internal phase state is preserved across the update. This allows the guidance layer to adapt to path changes without restarting the guidance process, while avoiding geometry-induced discontinuities associated with direct hard path replacement.



\subsection{Unified Geometric Guidance Output}
\label{subsec35}

Although the proposed ISF-GVF is defined in the lifted state space, practical robotic platforms primarily use its physical-space component. Accordingly, the unified geometric guidance output is defined as
\begin{equation}
v_g = K_1\alpha(\rho^2)\tau(w) - K_2q_r(\rho)e_{\perp}.
\label{eq:unified_guidance_output}
\end{equation}

This vector is exactly the physical-space part of the lifted guidance field. It simultaneously encodes the intention to advance along the path and the intention to contract toward the current path branch, and therefore provides a compact geometric representation of the relation among the current robot state, the reference path, and the current phase.

The quantity $v_g$ is not the final control input of any specific platform. Instead, it serves as a platform-agnostic upper-layer guidance interface. For a quadrotor, $v_g$ can be mapped to a velocity reference. For a differential-drive robot, it must be further converted into heading and speed commands consistent with the nonholonomic constraint. More generally, for other motion-constrained platforms, $v_g$ should be interpreted through the corresponding execution interface. In this sense, platform dependence is confined to the control layer, whereas the guidance layer itself remains unchanged.

This separation is important for two reasons. First, it allows the proposed method to be reused across heterogeneous robotic platforms without redesigning the guidance logic for each platform separately. Second, it ensures that the main contribution of this paper remains at the level of geometric guidance rather than being diluted by platform-specific execution details. The proposed ISF-GVF should therefore be understood not merely as a path-following formula, but as a unified guidance layer that can be interfaced with different motion-constrained robotic systems.

The meaning of $v_g$ also depends on the structural properties established above. If the guidance field were not incremental, $v_g$ could vary discontinuously under path updates. If the lifted-state formulation did not resolve branch ambiguity, $v_g$ could lose a unique geometric meaning near closed or self-intersecting path regions. It is precisely because the proposed construction preserves phase continuity and branch-consistent guidance that $v_g$ can be stably defined as a unified geometric guidance output. This output also provides the interface through which the proposed lifted-space guidance law can be analyzed and deployed on motion-constrained robotic platforms. The main properties of the proposed ISF-GVF are analyzed in the next section.

\iffalse
IV. Theoretical Analysis of the ISF-GVF
本章对所提出的增量式无奇异引导向量场进行理论分析。分析对象为定义在提升状态空间中的ISF-GVF及其诱导的理想引导律。理论分析的目的在于从严格的数学层面说明：所提引导场在定义域内是良定义的，在结构上是无奇异的，在误差动力学上具有清晰的分解形式，并且在理想引导条件下能够保证误差渐近收敛到零误差集合，进而使机器人在物理空间中渐近收敛到参考路径集合。
IV-A. Analytical Setup and Ideal Guidance Law
为了便于后续严格分析，首先对理想ISF-GVF引导律及相关记号作统一说明。设参考路径为参数化曲线
p: ℝ→ℝn
其中p(w)表示与路径进度参数w∈ℝ对应的路径点。引入提升状态
z = (x,w) ∈ ℝn × ℝ = ℝn+1
其中x∈ ℝn为机器人在物理空间中的位置，w∈ℝ为内部进度变量。相应地，参考路径在扩展状态空间的表示记为
Γ ={(p(w),w)∣w∈ℝ}⊂ℝn+1
定义跟踪误差为
e = x - p(w)
当p'(w)≠0时，单位切向量定义为
τ(w) = p'(w)‖p'(w)‖
据此，将误差分解为纵向误差与横向误差：
 e∥=τ(w)⊤e, e⊥=e − e∥τ(w)
从而有
 e=e∥τ(w)+e⊥,  τ(w)⊤e⊥=0
进一步记
ρ = ‖e⊥‖
定义收敛剖面函数
qr(ρ) =tanh(ρ/r)ρ,ρ > 0,1r,ρ = 0， r>0
在此基础上，理想ISF-GVF引导律写为
 x = K1α(ρ) t(w)−K2qr(ρ)e⊥#(1)
w = K1α(ρ) + K1σ( e∥)‖p'(w)‖#(2)
相应地，定义提升空间中的ISF-GVF可表示
 χISF(x,w) =xw=K1α(ρ)τ(w)−K2qr(ρ)e⊥K1α(ρ) + K1σ( e∥)‖p'(w)‖#(3)
这一理想引导律正对应前一章给出的统一结构：其物理空间分量由切向推进与横向收缩组成，相位分量由名义推进与纵向误差反馈组成。后续所有理论分析都围绕这一统一形式展开。
IV-B. Assumptions
为保证后续理论分析成立，作如下假设。
假设1：路径p∈C2(ℝ,ℝn)，且存在常数 0 < m < M < ∞ 使得
m ≤ ‖p'(w)‖ ≤ M,  ∀w∈ℝ#(4)
假设2：函数α:[0,∞)→(0,1] 局部Lipschitz，且满足
α(0) = 1#(5)
假设3：函数σ:ℝ→ℝ 为奇函数、严格单调、局部Lipschitz，且有界，并满足
σ(s)s > 0,  ∀s≠0#(6)
假设4：设计参数满足
K1 > 0, K2 >0, r >0#(7)
这些假设分别保证了路径切向量的良定义性、门控函数与相位修正函数的正则性，以及后续收敛分析所需的结构条件。
IV-C. Preliminary Results
引理1：提升路径
 Γ = {(p(w),w)∣w∈ℝ}
是不自交的。
证明：
若存在w1,w2∈ℝ使得
(p(w1),w1)=(p(w2),w2)#(8)
则由最后一个坐标立即得到
w1=w2#(9)
因此，Γ不可能发生自交。证毕。
引理2：在假设1-4下，ISF-GVF χISF(x,w)关于(x,w)在ℝn+1上局部Lipschitz。
证明：
由假设1以及‖p'(w)‖≥m>0可知，单位切向量
τ(w) = p'(w)‖p'(w)‖#(10)
关于w局部Lipschitz。
又由于
e = x − p(w)#(11)
可知e关于(x,w) 局部Lipschitz，因此e∥、e⊥以及ρ = ‖e⊥‖也关于(x,w) 局部Lipschitz。
下面证明映射qr(ρ)e⊥关于e⊥局部Lipschitz。定义标量函数
ϕ(s)=tanh(s/r)s,s > 0,1r,s = 0.#(12)
由于tanh(∙)为解析函数，且当s→0时有
tanh(s/r) =sr+o(s)#(13)
固ϕ(s)在s = 0处可连续延拓，并且在任意有界区间上具有有界导数，从而ϕ在[0,∞)上局部Lipschitz。
进一步定义径向映射
β(e⊥) = ϕ(‖e⊥‖)e⊥#(14)
由于ϕ(‖e⊥‖)局部Lipschitz，且e⊥↦‖e⊥‖为Lipschitz映射，可知β(e⊥)关于e⊥局部Lipschitz。注意到β(e⊥) = qr(ρ)e⊥，故横向收缩项也是局部Lipschitz的。
再结合假设2与假设3中 α(⋅)、σ(∙)的局部Lipschitz性，以及假设1中‖p'(w)‖≥m>0，可知式(3)所定义的ISF-GVF关于(x,w)局部Lipschitz。证毕。
IV-D. Singularity-Free Property
在本文中，所谓“无奇异”主要是指所构造的ISF-GVF在整个定义域上处处非零。其良定义性与局部正则性则由前述引理进一步保证。由引理2可知其良定义性与局部Lipschitz性已经成立，下面进一步证明该向量场处处非零。
定理1：在假设1-4下，定义在提升空间中的ISF-GVF对任意(x,w)∈ℝn+1满足
χISF(x,w)≠0#(15)
证明：
采用反证法。设存在(x,w)∈ℝn+1使得
χISF(x,w)= 0#(16)
则其前n个分量必满足
K1α(ρ)τ(w)−K2qr(ρ)e⊥=0#(17)
对式(17)左乘τ(w)⊤，得到
K1α(‖e⊥‖)τ(w)⊤τ(w) − K2qr(‖e⊥‖)τ(w)⊤e⊥ = 0#(18)
由单位切向量定义以及误差正交分解可知
τ(w)⊤τ(w) = 1,   τ(w)⊤e⊥ = 0#(19)
将式(19)代入(18)，得到
K1α(ρ) = 0#(20)
然而，由假设2与假设4可知
K1>0,    α(ρ)∈ (0,1]#(21)
由此式(20)不可能成立，矛盾。
故不存在(x,w)∈ℝn+1使得χISF(x,w) = 0，即所构造的ISF-GVF在整个定义域上处处非零。证毕。
IV-E. Exact Error Dynamics
定理2：沿理想ISF-GVF引导律(1)-(2)的轨迹，误差e = x − p(w)满足
e = −K2qr(ρ) e⊥ − K1σ( e∥)τ(w)#(22)
证明：
由
e = x − p(w)#(23)
对时间求导，得
 e = x − p'(w)w#(24)
又由于
p'(w) = ||p'(w)||τ(w)#(25)
将式(1)、式(2)、及式(25)代入式(24)，可得
e = (K1α(ρ)τ(w) − K2qr(ρ)e⊥) − ||p'(w)||τ(w)∙K1α(ρ) + K1σ( e∥)‖p'(w)‖#(26)
化简后，切向推进项K1α(‖e⊥‖)τ(w)精确抵消，从而得到
e = − K2qr(ρ)e⊥ − K1σ( e∥)τ(w)#(27)
证毕。
IV-F. Forward Completeness
引理3：在假设1-4下，理想ISF-GVF引导律(1) - (2)的任意解对所有t≥0都存在，即系统前向完备。
证明：
由α(ρ)∈(0,1]可得
 ||K1α(ρ)τ(w)||≤K1#(28)
另一方面，
||K2qr(ρ)e⊥|| = K2tanh(ρ/r)≤K2#(29)
故
||x||≤K1+K2#(30)
因此，x全局有界。
再由σ(∙)有界，设
‖σ‖∞ ≤ σ#(31)
结合假设1中‖p'(w)‖ ≥ m > 0，有
|w| ≤ K1+K1σm#(32)
故w同样全局有界。
于是系统状态导数(x,w)全局有界。设系统最大存在区间为[0, Tmax)。若Tmax< ∞，则由式(30) - (32)可知，(x(t),w(t))在[0, Tmax)上为Lipschitz轨迹，因此必然有界，并且存在极限点。另一方面，由引理2可知向量场χISF(x,w)局部Lipschitz，因此根据常微分方程解的延拓定理，该解可从该极限点继续延拓到 Tmax之后，这与 Tmax为最大存在时间相矛盾。
因此，必有
 Tmax=+∞#(33)
故系统前向完备。证毕。
IV-G. Global Asymptotic Convergence
定理3：在假设1-4下，理想ISF-GVF引导律(1) - (2)的零误差集合全局渐进稳定。特别地，
e(t) → 0,  t → ∞#(34)
证明：
取Lyapunov函数
V(e) = 12‖e‖2 = 12‖e⊥‖2+e∥2#(35)
沿系统轨迹求导，并利用定理2中的误差动力学，得到
V = e⊤e=e⊤(−K2qr(ρ) e⊥ − K1σ( e∥) τ(w)) #(36)
再由误差分解
e=e∥τ(w)+e⊥#(37)
可知
e⊤e⊥ = ‖e⊥‖2，  e⊤ τ(w) =  e∥#(38)
将式(38)代入式(36),得到
V = −K2qr(ρ) ‖e⊥‖2 − K1σ( e∥)  e∥ ≤ 0#(39)
因此，V(t)单调不增且下界有界，从而收敛到某个有限极限V∞ ≥ 0。又由于V(e)=12‖e‖2 ，故可进一步推出e(t)有界。
接下来，由定理2可知
e = − K2qr(ρ)e⊥ − K1σ( e∥)τ(w)#(40)
其中，第一项满足
qr(ρ) ‖e⊥‖=tanh(ρ/r) ≤ 1#(41)
故其有界，第二项由于σ(∙)有界且||τ(w)|| = 1，亦有界。因此，e有界。
下面证明V(t)一致连续。由引理3，w有界，因此w(t)在[0, ∞)上为Lipschitz函数。又由假设1，p∈C2且 ‖p'(w)‖≥m≥0，故τ(w)关于w局部Lipschitz。由于w(t)有界导数，且e(t)已知有界，可知(e(t),w(t))始终落在某个紧集内。
由于式(40)右端关于(e,w)的局部Lipschitz，且(e(t),w(t))始终落在某个紧集内，因此该右端在该紧集上为Lipschitz映射，从而e(t)一致连续。又因为e(t)有界，且
V(t) = e(t)⊤e(t)#(42)
由于V(t)收敛，故
0∞−V(t) dt = V(0) − V∞ < ∞#(43)
根据Barbalat引理，可知
V(t) → 0,  t → ∞#(44)
由式(39)可知
 K2qr(ρ)‖e⊥‖2 + K1σ( e∥) e∥ → 0#(45)
由于上式两项均非负，因此它们分别趋于零。
首先，
qr(ρ)‖e⊥‖2 =‖e⊥‖tanh(ρ/r)#(46)
由于tanh(ρ/r)>0当且仅当ρ>0，故式(46)趋于零当且仅当
‖e⊥‖→0
因此，
e⊥(t) → 0#(47)
其次，由假设3可知，σ(∙)为严格单调奇函数，且满足
σ(s)s>0,  ∀s≠0#(48)
因此，σ( e∥) e∥=0当且仅当 e∥=0。结合式(45)可得
 e∥(t) → 0#(49)
再由式(47)已经得到
e⊥(t) → 0#(50)
注意到总误差满足正交分解
e(t)=e∥(t)τ(w(t))+e⊥(t)#(51)
且||τ(w(t))||=1，于是有
‖e(t)‖≤|e∥(t)|+‖e⊥(t)‖#(52)
由式(49)和式(50)可进一步推出
e(t)→0,t→∞#(53)
因此，零误差集合对理想ISF-GVF引导律是全局渐近稳定的。证毕。
IV-H. Convergence to the Physical Path Set
上述结论在提升空间误差e = x − p(w)的意义下建立的。下面进一步说明，该结果可直接推出机器人在物理空间中对原始路径集合的渐近收敛。
推论1：在假设1-4下，理想ISF-GVF引导律(1)-(2)生成的物理轨迹x(t)全局渐近收敛到路径集合
P ={p(w)∣w∈ℝ}⊂ℝn
也即
dist(x(t),P) → 0,  t → ∞#(54)
证明：
对任意t≥0，由路径集合的定义可知，点p(w(t))∈P。因此，
dist(x(t),P) ≤ ‖x(t)−p(w(t))‖ = ‖e(t)‖#(55)
由定理3已知‖e(t)‖→ 0，故有
dist(x(t),P) → 0,  t → ∞#(56)
证毕。
\fi


\section{Theoretical Analysis of the ISF-GVF}
\label{sec4}

This section analyzes the ideal ISF-GVF defined in \eqref{eq:isf_gvf} for a currently valid reference path. The objective is to establish its regularity, singularity-free property, exact error dynamics, forward completeness, and asymptotic convergence to the reference path under the ideal lifted dynamics.

It should be clarified that the following analysis is conducted on an inter-update interval, during which the reference path is treated as fixed and the lifted state evolves according to \eqref{eq:isf_gvf}. When replanning updates the reference path, the guidance field is reconstructed around the newly valid path as described in Section~III-D, while the internal phase state is retained. Therefore, the results in this section characterize the ideal guidance-layer properties of the proposed ISF-GVF on each currently valid path. The effects of platform-specific execution interfaces are addressed separately in Section~V.

\subsection{Assumptions}
\label{subsec41}

The following assumptions are imposed throughout the analysis.

\textbf{Assumption 1:}
The reference path $p\in C^2(\mathbb{R},\mathbb{R}^n)$, and there exist constants $m,M,L_p>0$ such that
\begin{equation}
0<m\le \|p'(w)\|\le M,\qquad \|p''(w)\|\le L_p,\qquad \forall w\in\mathbb{R}.
\label{eq:assumption_path}
\end{equation}

\textbf{Assumption 2:}
The function $\alpha:[0,\infty)\to(0,1]$ is locally Lipschitz and satisfies
\begin{equation}
\alpha(0)=1.
\label{eq:assumption_alpha}
\end{equation}

\textbf{Assumption 3:}
The function $\sigma:\mathbb{R}\to\mathbb{R}$ is bounded, odd, strictly increasing, and locally Lipschitz, and satisfies
\begin{equation}
\sigma(s)s>0,\qquad \forall s\neq 0.
\label{eq:assumption_sigma}
\end{equation}

\textbf{Assumption 4:}
The design parameters satisfy
\begin{equation}
K_1>0,\qquad K_2>0,\qquad r>0.
\label{eq:assumption_parameters}
\end{equation}

\textbf{Assumption 5:}
For the smoothness analysis across incremental path updates, the accepted updated path is connected to the previous path through a \(C^2\)-compatible stitching segment. The stitched path matches the position, first derivative, and second derivative of the previous and updated paths at the stitching boundaries. If the reconstructed vector field itself is required to be \(C^2\) with respect to \((x,w)\), the stitched path is further assumed to be \(C^3\)-compatible. The progress-gating function is selected as a smooth function of \(\rho^2=\|e_{\perp}\|^2\).

Assumption~1 guarantees that the unit tangent vector is well defined and globally regular. Assumptions~2 and~3 specify the regularity and sign properties required of the gating and phase-correction functions. Assumption~4 gives the basic design constraints. Assumption~5 is only used for the regularity analysis across incremental path updates. The local Lipschitz, singularity-free, forward-completeness, and asymptotic-convergence results below rely on Assumptions~1--4, whereas the update-regularity result additionally uses Assumption~5.

\subsection{Preliminary Results}
\label{subsec42}

\begin{lemma}
The lifted-path set
\begin{equation}
\Gamma=\{(p(w),w)\mid w\in\mathbb{R}\}
\label{eq:gamma_sec4}
\end{equation}
is non-self-intersecting.
\end{lemma}

\begin{proof}
If $(p(w_1),w_1)=(p(w_2),w_2)$, then the last coordinate immediately yields $w_1=w_2$. Hence, $\Gamma$ cannot self-intersect.
\end{proof}

\begin{lemma}
Under Assumptions~1--4, the vector field $\chi_{\mathrm{ISF}}(x,w)$ defined in \eqref{eq:isf_gvf} is locally Lipschitz on $\mathbb{R}^{n+1}$.
\end{lemma}

\begin{proof}
By Assumption~1 and the lower bound $\|p'(w)\|\ge m>0$, the unit tangent vector
\begin{equation}
\tau(w)=\frac{p'(w)}{\|p'(w)\|}
\label{eq:tau_sec4}
\end{equation}
is locally Lipschitz in $w$. Since $e=x-p(w)$, the quantities $e_{\parallel}$, $e_{\perp}$, and $\rho=\|e_{\perp}\|$ are locally Lipschitz in $(x,w)$. Moreover, the radial mapping
\begin{equation}
\beta(e_{\perp})=q_r(\rho)e_{\perp}
\label{eq:beta_sec4}
\end{equation}
is locally Lipschitz because the extension of $q_r(\rho)$ at $\rho=0$ is continuous and smooth on bounded sets. Combining these facts with the local Lipschitz continuity of $\alpha$ and $\sigma$, and using again $\|p'(w)\|\ge m>0$, shows that the right-hand side of \eqref{eq:isf_gvf} is locally Lipschitz in $(x,w)$.
\end{proof}

\subsection{Regularity Under Incremental Path Updates}
\label{subsec:update_regularity}

We next characterize the regularity of the proposed incremental update. The phase-continuation condition in \eqref{eq:phase_continuity_update} preserves the internal progress state, but it does not by itself guarantee smoothness of the guidance vector under an arbitrary hard replacement of the reference path. The reason is that the path-dependent quantities \(p(w)\), \(p'(w)\), and \(p''(w)\) may differ before and after the update.

Under Assumption~5, the accepted updated path is connected to the previous path by a \(C^2\)-compatible stitching segment. Let \(w_k=w(t_k^-)=w(t_k^+)\) be the retained phase on the previous path, and let \(w_j\) be the selected joining phase on the updated path. The stitching segment \(q(s)\), \(s\in[0,1]\), is constructed to satisfy
\begin{equation}
q(0)=p_k(w_k),\quad
q'(0)=p_k'(w_k),\quad
q''(0)=p_k''(w_k),
\label{eq:c2_stitch_start}
\end{equation}
and
\begin{equation}
q(1)=p_{k+1}(w_j),\quad
q'(1)=p_{k+1}'(w_j),\quad
q''(1)=p_{k+1}''(w_j).
\label{eq:c2_stitch_end}
\end{equation}
Such a segment can be realized by a quintic polynomial because the six boundary conditions in \eqref{eq:c2_stitch_start} and \eqref{eq:c2_stitch_end} uniquely determine the six polynomial coefficients.

\begin{lemma}
Under Assumption~5, the stitched path used after an incremental update is \(C^2\)-compatible at the connection boundaries.
\end{lemma}

\begin{proof}
The stitching segment satisfies the same position, first derivative, and second derivative as the previous path at its starting boundary by \eqref{eq:c2_stitch_start}. It also satisfies the same position, first derivative, and second derivative as the updated path at its ending boundary by \eqref{eq:c2_stitch_end}. Hence, the concatenated reference path has continuous position, tangent, and second-order path geometry across the stitching boundaries. Therefore, the incremental path update is \(C^2\)-compatible.
\end{proof}

\begin{remark}
The \(C^2\)-compatible stitching condition guarantees second-order continuity at the path-geometry level. Since the local tangent, transversal error, and guidance vector are reconstructed from the stitched path, this avoids geometry-induced discontinuities caused by direct hard path replacement. If the vector field \(\chi_{\mathrm{ISF}}\) itself is required to be \(C^2\) with respect to \((x,w)\), the stronger \(C^3\)-compatible condition in Assumption~5 is needed. Without such stitching or derivative matching, an instantaneous replacement of \(p_k\) by \(p_{k+1}\) generally yields only a piecewise-smooth vector field.
\end{remark}

\subsection{Singularity-Free Property}
\label{subsec43}

\begin{theorem}
Under Assumptions~1--4,
\begin{equation}
\chi_{\mathrm{ISF}}(x,w)\neq 0,\qquad \forall (x,w)\in\mathbb{R}^{n+1}.
\label{eq:nonvanishing_field}
\end{equation}
\end{theorem}

\begin{proof}
Assume, for contradiction, that there exists $(x,w)\in\mathbb{R}^{n+1}$ such that $\chi_{\mathrm{ISF}}(x,w)=0$. Then the first $n$ components of \eqref{eq:isf_gvf} satisfy
\begin{equation}
K_1\alpha(\rho)\tau(w)-K_2q_r(\rho)e_{\perp}=0.
\label{eq:zero_first_components}
\end{equation}
Taking the inner product of \eqref{eq:zero_first_components} with $\tau(w)$ and using $\tau(w)^{\top}e_{\perp}=0$ gives
\begin{equation}
K_1\alpha(\rho)=0.
\label{eq:contradiction_step}
\end{equation}
This contradicts Assumptions~2 and~4, since $\alpha(\rho)\in(0,1]$ and $K_1>0$. Hence, $\chi_{\mathrm{ISF}}$ is nonvanishing on the entire lifted state space.
\end{proof}

\subsection{Exact Error Dynamics}
\label{subsec44}

\begin{theorem}
Along the trajectories of the ideal ISF-GVF \eqref{eq:isf_gvf}, the tracking error $e=x-p(w)$ satisfies
\begin{equation}
\dot e=-K_2q_r(\rho)e_{\perp}-K_1\sigma(e_{\parallel})\tau(w).
\label{eq:exact_error_dynamics}
\end{equation}
\end{theorem}

\begin{proof}
Differentiating $e=x-p(w)$ yields
\begin{equation}
\dot e=\dot x-p'(w)\dot w.
\label{eq:error_derivative_sec4}
\end{equation}
Using $p'(w)=\|p'(w)\|\tau(w)$ together with \eqref{eq:isf_gvf}, the tangential-advancement term $K_1\alpha(\rho)\tau(w)$ cancels exactly, which gives \eqref{eq:exact_error_dynamics}.
\end{proof}

\subsection{Forward Completeness}
\label{subsec45}

\begin{lemma}
Under Assumptions~1--4, every solution of \eqref{eq:isf_gvf} exists for all $t\ge 0$.
\end{lemma}

\begin{proof}
Since $\alpha(\rho)\in(0,1]$ and
\begin{equation}
q_r(\rho)\rho=\tanh(\rho/r)\le 1,
\label{eq:qr_bound_sec4}
\end{equation}
the physical-space dynamics satisfy
\begin{equation}
\|\dot x\|\le K_1+K_2.
\label{eq:xdot_bound_sec4}
\end{equation}
Let $\bar\sigma=\|\sigma\|_{\infty}$. Then, using $\|p'(w)\|\ge m$,
\begin{equation}
|\dot w|\le \frac{K_1(1+\bar\sigma)}{m}.
\label{eq:wdot_bound_sec4}
\end{equation}
Hence, the state derivative $(\dot x,\dot w)$ is globally bounded. By Lemma~2, the vector field is locally Lipschitz. Standard continuation results for ordinary differential equations therefore exclude finite-time blow-up, and the system is forward complete.
\end{proof}

\subsection{Global Asymptotic Convergence}
\label{subsec46}

\begin{theorem}
Under Assumptions~1--4, the zero-error set of \eqref{eq:isf_gvf} is globally asymptotically stable. In particular,
\begin{equation}
e(t)\to 0,\qquad t\to\infty.
\label{eq:error_convergence_sec4}
\end{equation}
\end{theorem}

\begin{proof}
Consider the Lyapunov function
\begin{equation}
V(e)=\frac{1}{2}\|e\|^2.
\label{eq:lyapunov_sec4}
\end{equation}
Along the trajectories of \eqref{eq:isf_gvf}, using \eqref{eq:exact_error_dynamics} and the orthogonal decomposition in \eqref{eq:error_decomposition}, one obtains
\begin{equation}
\dot V=-K_2q_r(\rho)\|e_{\perp}\|^2-K_1\sigma(e_{\parallel})e_{\parallel}\le 0.
\label{eq:lyapunov_derivative_sec4}
\end{equation}
Thus, $V(t)$ is nonincreasing and bounded below, which implies that $e(t)$ remains bounded.

Next, \eqref{eq:exact_error_dynamics} and the boundedness of $\sigma$ imply that $\dot e(t)$ is bounded. To invoke Barbalat's lemma, it remains to establish the uniform continuity of $\dot V(t)$. By Assumption~1, $\tau(w)$ is globally Lipschitz because $p\in C^2$, $\|p'(w)\|\ge m>0$, and $\|p''(w)\|\le L_p$. Since $e(t)$ is bounded and $\dot w(t)$ is bounded by \eqref{eq:wdot_bound_sec4}, the quantities $e_{\parallel}(t)$, $e_{\perp}(t)$, and $\rho(t)$ evolve in bounded sets. Together with the Lipschitz properties of $\alpha$, $\sigma$, and $q_r$ on bounded sets, this implies that the right-hand side of \eqref{eq:exact_error_dynamics} is uniformly continuous in time. Hence, $\dot V(t)=e(t)^{\top}\dot e(t)$ is uniformly continuous.

Since $V(t)$ converges and $\dot V(t)$ is uniformly continuous, Barbalat's lemma yields
\begin{equation}
\dot V(t)\to 0,\qquad t\to\infty.
\label{eq:vdot_convergence_sec4}
\end{equation}
From \eqref{eq:lyapunov_derivative_sec4}, both terms
\begin{equation}
K_2q_r(\rho)\|e_{\perp}\|^2,\qquad K_1\sigma(e_{\parallel})e_{\parallel}
\label{eq:nonnegative_terms_sec4}
\end{equation}
are nonnegative and their sum converges to zero. Therefore, each term must converge to zero. Since
\begin{equation}
q_r(\rho)\|e_{\perp}\|^2=\|e_{\perp}\|\tanh(\rho/r),
\label{eq:perp_term_identity_sec4}
\end{equation}
the first term vanishes if and only if $\|e_{\perp}\|=0$. By Assumption~3, the second term vanishes if and only if $e_{\parallel}=0$. Hence,
\begin{equation}
e_{\perp}(t)\to 0,\qquad e_{\parallel}(t)\to 0.
\label{eq:error_components_convergence_sec4}
\end{equation}
Combining \eqref{eq:error_components_convergence_sec4} with \eqref{eq:error_decomposition} yields \eqref{eq:error_convergence_sec4}.
\end{proof}

\subsection{Convergence Rate Bounds in a Working Tube}
\label{subsec:convergence_rate_bounds}

The result above establishes global asymptotic convergence under Assumptions~1--4. A stronger exponential estimate can be obtained on a bounded working tube when the contraction profiles satisfy additional sector bounds. This distinction is important because the proposed ISF-GVF should not be interpreted as globally exponentially convergent under the basic assumptions alone.

Consider a bounded error set
\begin{equation}
\mathcal{E}_{E}:=
\{(e_{\perp},e_{\parallel})\mid
\|e_{\perp}\|\le E,\ |e_{\parallel}|\le E\},
\label{eq:bounded_working_tube}
\end{equation}
where \(E>0\). Since \(V(t)\) is nonincreasing from \eqref{eq:lyapunov_derivative_sec4}, an initial condition satisfying \(\|e(0)\|\le E\) remains in such a bounded set.

For \(\rho\in[0,E]\), define
\begin{equation}
q_{\min}(E)=\frac{\tanh(E/r)}{E},
\qquad
q_{\max}(E)=\frac{1}{r}.
\label{eq:q_bounds_working_tube}
\end{equation}
Then
\begin{equation}
q_{\min}(E)\le q_r(\rho)\le q_{\max}(E),
\qquad \rho\in[0,E],
\label{eq:qr_sector_bound}
\end{equation}
where the lower bound follows from the monotonic decrease of
\(\tanh(\rho/r)/\rho\) on \((0,\infty)\), and the upper bound follows from
\(\lim_{\rho\to0}q_r(\rho)=1/r\).

Assume further that on \(|s|\le E\), the phase-correction function satisfies the sector bounds
\begin{equation}
c_{\sigma}(E)s^2
\le
\sigma(s)s
\le
C_{\sigma}(E)s^2,
\qquad
c_{\sigma}(E)>0,\ C_{\sigma}(E)>0 .
\label{eq:sigma_sector_bounds}
\end{equation}

\begin{theorem}
Under Assumptions~1--4 and the sector condition \eqref{eq:sigma_sector_bounds}, the ideal ISF-GVF admits exponential convergence bounds on the bounded working tube \(\mathcal{E}_{E}\). In particular,
\begin{equation}
\|e(0)\|e^{-\Lambda(E)t}
\le
\|e(t)\|
\le
\|e(0)\|e^{-\lambda(E)t},
\label{eq:error_two_sided_bound}
\end{equation}
where
\begin{equation}
\lambda(E)=
\min\{K_2q_{\min}(E),K_1c_{\sigma}(E)\},
\label{eq:lambda_working_tube}
\end{equation}
and
\begin{equation}
\Lambda(E)=
\max\{K_2q_{\max}(E),K_1C_{\sigma}(E)\}.
\label{eq:Lambda_working_tube}
\end{equation}
\end{theorem}

\begin{proof}
From \eqref{eq:lyapunov_derivative_sec4},
\begin{equation}
\dot V
=
-K_2q_r(\rho)\|e_{\perp}\|^2
-
K_1\sigma(e_{\parallel})e_{\parallel}.
\label{eq:vdot_rate_bound_start}
\end{equation}
Using \eqref{eq:qr_sector_bound} and the lower sector bound in \eqref{eq:sigma_sector_bounds}, one obtains
\begin{equation}
\dot V
\le
-K_2q_{\min}(E)\|e_{\perp}\|^2
-
K_1c_{\sigma}(E)e_{\parallel}^2 .
\label{eq:vdot_upper_rate}
\end{equation}
Since
\begin{equation}
V=\frac{1}{2}\left(\|e_{\perp}\|^2+e_{\parallel}^2\right),
\end{equation}
it follows that
\begin{equation}
\dot V\le -2\lambda(E)V.
\label{eq:comparison_upper_rate}
\end{equation}
By the comparison principle,
\begin{equation}
V(t)\le V(0)e^{-2\lambda(E)t}.
\label{eq:V_upper_rate}
\end{equation}
Equivalently,
\begin{equation}
\|e(t)\|\le \|e(0)\|e^{-\lambda(E)t}.
\label{eq:error_upper_rate}
\end{equation}

Similarly, using the upper bounds in \eqref{eq:qr_sector_bound} and \eqref{eq:sigma_sector_bounds}, one obtains
\begin{equation}
\dot V
\ge
-K_2q_{\max}(E)\|e_{\perp}\|^2
-
K_1C_{\sigma}(E)e_{\parallel}^2
\ge
-2\Lambda(E)V.
\label{eq:comparison_lower_rate}
\end{equation}
Again by comparison,
\begin{equation}
V(t)\ge V(0)e^{-2\Lambda(E)t},
\label{eq:V_lower_rate}
\end{equation}
which yields the lower bound in \eqref{eq:error_two_sided_bound}. This completes the proof.
\end{proof}

\begin{remark}
The exponential bounds in \eqref{eq:error_two_sided_bound} are bounded-set estimates. They do not imply global exponential convergence under Assumptions~1--4 alone. In fact, \(q_r(\rho)=\tanh(\rho/r)/\rho\) has no global positive lower bound as \(\rho\to\infty\). Therefore, the basic convergence result of the proposed ISF-GVF is global asymptotic convergence, while exponential rate bounds are obtained on bounded working tubes under additional sector conditions.
\end{remark}

\subsection{Convergence to the Physical Path Set}
\label{subsec47}

Define the physical path set by
\begin{equation}
P:=\{p(w)\mid w\in\mathbb{R}\}\subset\mathbb{R}^n.
\label{eq:physical_path_set_sec4}
\end{equation}

\begin{corollary}
Under Assumptions~1--4, the physical trajectory $x(t)$ generated by \eqref{eq:isf_gvf} converges globally asymptotically to the path set $P$.
\end{corollary}

\begin{proof}
For every $t\ge 0$, one has $p(w(t))\in P$. Therefore,
\begin{equation}
\operatorname{dist}(x(t),P)\le \|x(t)-p(w(t))\|=\|e(t)\|.
\label{eq:distance_to_path_sec4}
\end{equation}
The claim follows immediately from \eqref{eq:error_convergence_sec4}.
\end{proof}


\section{Guidance-to-Control Realization for Motion-Constrained Robotic Platforms}
\label{sec5}

This section describes how the unified geometric guidance output of the proposed ISF-GVF is realized on different motion-constrained robotic platforms. The objective is not to design a universal low-level controller for all platforms, but to show how the same upper-layer guidance signal can be connected to platform-specific execution interfaces. In this sense, the proposed method provides a unified guidance layer rather than a universal platform controller.

The same ISF-GVF guidance layer is considered for two representative platforms: a quadrotor UAV and a differential-drive ground robot. These platforms have different motion constraints, sensing configurations, and command interfaces. The quadrotor can execute a bounded high-level motion reference through its flight-control interface, whereas the differential-drive robot must convert the guidance information into admissible linear and angular velocity commands under the nonholonomic constraint. This contrast is used to demonstrate the cross-platform deployability of the proposed guidance layer.

\subsection{Guidance-to-Control Interface}
\label{subsec51}

The lifted vector field \(\chi_{\mathrm{ISF}}\) consists of the physical-space guidance component \(v_g\) and the internal phase rate \(\dot{w}\). In implementation, \(v_g\) is exposed as the external geometric guidance signal, whereas \(\dot{w}\) is used internally to update the phase state. The platform-specific execution interface then maps \(v_g\), together with the local path geometry when necessary, to executable commands that satisfy the motion and command constraints of each robot.

The unified geometric guidance output is given by
\begin{equation}
v_g = K_1\alpha(\rho^2)\tau(w) - K_2q_r(\rho)e_{\perp}.
\label{eq:vg_sec5}
\end{equation}
This vector is not the final executable control input of a specific robotic platform. Instead, it represents the desired geometric motion direction and path-convergence intention generated by the ISF-GVF guidance layer. For a quadrotor UAV, this signal can be converted into a bounded high-level motion reference and executed through the flight-control interface. For a differential-drive ground robot, the same signal must be interpreted through a planar nonholonomic execution interface and mapped to admissible forward and angular velocity commands. More generally, for other motion-constrained robotic platforms, \(v_g\) should be interpreted through the corresponding platform-specific execution interface.

The convergence results in Section~IV are established for the ideal lifted dynamics in which the physical state follows the ISF-GVF guidance velocity exactly. Practical robotic platforms may introduce realization errors due to actuation limits, command discretization, low-level tracking errors, or platform-specific kinematic constraints. Therefore, the role of this section is to clarify the guidance-to-command realization on different platforms, while the experimental results in Section~VI validate that the same guidance layer can be deployed on both aerial and ground robotic systems.

\begin{remark}
The proposed ISF-GVF is a guidance-layer method. The ideal convergence analysis in Section~IV applies to the lifted dynamics \(\dot{x}=v_g\) and \(\dot{w}=\chi_w(x,w)\), where \(\chi_w\) denotes the phase component of \(\chi_{\mathrm{ISF}}\). When \(v_g\) is realized through a platform-specific execution interface, the actual motion may differ from the ideal guidance velocity due to saturation, discretization, low-level tracking errors, model mismatch, or kinematic constraints. Under bounded realization errors, the ideal asymptotic convergence should be interpreted as practical convergence to a neighborhood determined by the execution error. The platform realizations below therefore aim to preserve the geometric meaning of \(v_g\), rather than to prove global stability of the complete platform dynamics.
\end{remark}

\subsection{Quadrotor UAV Realization}
\label{subsec52}

 

\subsection{Differential-Drive Ground Robot Realization}
\label{subsec53}

For the differential-drive ground robot, the physical-space guidance output \(v_g\) cannot be directly executed as an arbitrary planar velocity because of the nonholonomic constraint. The vehicle execution layer therefore converts the ISF-GVF guidance information into admissible linear and angular velocity commands.

The planar kinematics of the ground robot are modeled as
\begin{equation}
\dot x=v\cos\psi,\qquad
\dot y=v\sin\psi,\qquad
\dot\psi=\omega,
\label{eq:dd_kinematics_sec5}
\end{equation}
where \(v\) and \(\omega\) are the executable linear and angular velocity commands. Instead of directly applying \(v_g\), the proposed implementation constructs a short-horizon reference from the currently valid path and the internal phase state. Each reference point contains the phase-consistent position, heading, linear velocity, and angular velocity:
\begin{equation}
\mathcal{H}
=
\{(x_i^{r},y_i^{r},\psi_i^{r},v_i^{r},\omega_i^{r})\}_{i=0}^{N-1}.
\label{eq:mpc_reference_horizon}
\end{equation}

The reference heading is determined by the tangent direction of the current path branch,
\begin{equation}
\psi_i^{r}=\operatorname{atan2}(\tau_y(w_i),\tau_x(w_i)).
\label{eq:dd_reference_heading}
\end{equation}
The linear velocity reference is obtained from the tangential component of the ISF-GVF guidance output,
\begin{equation}
v_i^{r}=\max\{0,\, v_g^{\top}\tau(w_i)\},
\label{eq:dd_reference_velocity}
\end{equation}
and the angular velocity reference is generated from the local curvature information as
\begin{equation}
\omega_i^{r}=\kappa(w_i)v_i^{r}.
\label{eq:dd_reference_yaw_rate}
\end{equation}

A short-horizon MPC tracking layer then tracks the reference sequence \(\mathcal{H}\) under the differential-drive kinematics and command constraints, and outputs the executable command \((v,\omega)\). This MPC layer is used only as a constrained execution interface. It does not redefine the path phase, perform branch selection, or replace the ISF-GVF guidance law. The lifted ISF-GVF remains responsible for the phase-continuous guidance semantics, while the MPC layer converts the resulting reference into commands compatible with the ground robot.

\subsection{Discussion on Cross-Platform Realization}
\label{subsec54}

The quadrotor and ground-robot realizations show that the proposed ISF-GVF acts as a reusable upper-layer guidance mechanism for different motion-constrained robotic platforms rather than as a platform-specific controller. For the quadrotor UAV, the guidance output is converted into a bounded high-level motion reference. For the differential-drive ground robot, the same lifted guidance information is converted into a phase-consistent short-horizon reference and tracked by a constrained vehicle execution layer. In both cases, the platform-dependent components appear only after the unified guidance output has been generated.

This separation preserves the main structure of the proposed method. The lifted-state ISF-GVF remains responsible for path-phase evolution, branch-consistent guidance, and generation of the physical-space guidance output. The platform-specific execution layer only maps this output to commands compatible with the corresponding robotic platform. Therefore, the same guidance formulation can be deployed on heterogeneous motion-constrained robotic platforms without redesigning the core guidance law.

The cross-platform results should be interpreted at the guidance-layer level. The theoretical analysis in Section~IV establishes the ideal properties of the ISF-GVF, while the platform realizations in this section specify how its output is executed on representative motion-constrained robotic platforms. The experimental validation in Section~VI further demonstrates that this separation is practically feasible for both aerial and ground robotic systems.



\section{Experimental Validation}
\label{sec6}

This section validates the proposed ISF-GVF framework on heterogeneous robotic platforms, including a quadrotor UAV and a differential-drive ground robot. Following the experimental organization commonly used in robotic navigation studies, the validation is divided into RViz simulation results and real-world platform experiments. The RViz simulations are used to examine the geometric guidance behavior under controlled obstacle configurations, while the real-world experiments are used to verify physical deployability on the robotic platforms.

For both platforms, the ISF-GVF guidance layer generates the same lifted guidance output, consisting of the physical-space guidance vector \(v_g\) and the internal phase rate \(\dot w\). The platform-dependent part appears only after this guidance output has been generated. The UAV converts \(v_g\) into a bounded high-level motion reference, whereas the ground robot converts the lifted guidance information into a phase-consistent short-horizon reference that is tracked by a constrained vehicle execution layer. Therefore, the experiments are intended to validate the deployability of the proposed guidance layer rather than to compare low-level controllers.

\subsection{Experimental Platforms and Tasks}
\label{subsec61}

The experimental validation is conducted on two robotic platforms with different motion and command characteristics, as shown in Fig.~\ref{fig:experimental_platforms}. The first platform is a quadrotor UAV equipped with an RK3588 onboard computer, an omnidirectional camera, and an STM32F405RGT6 flight controller. The second platform is a differential-drive ground robot based on a Scout 2.0 mobile platform, equipped with an onboard computer, a RoboSense LiDAR, and an SBG IMU. These two platforms differ significantly in sensing configuration, motion constraints, and executable command interfaces, which makes them suitable for evaluating the cross-platform deployability of the proposed guidance layer.

The UAV executes the guidance output through its onboard flight-control interface, where the physical-space guidance vector is converted into a bounded high-level motion reference. The ground robot realizes the same guidance output through a nonholonomic vehicle execution layer, where a phase-consistent short-horizon reference is tracked and converted into admissible linear and angular velocity commands.

\begin{figure}[!t]
    \centering
    \includegraphics[width=0.68\linewidth]{Cxq_pic4.png}\\[1mm]
    \includegraphics[width=0.68\linewidth]{Cxq_pic5.png}
    \caption{Experimental robotic platforms used for validation. The upper image shows the quadrotor UAV platform, and the lower image shows the differential-drive ground robot platform.}
    \label{fig:experimental_platforms}
\end{figure}

Two types of tasks are considered on both platforms. The first type is closed-trajectory acquisition and tracking, where the robot is initialized away from a closed reference curve and is required to converge to and track the trajectory. This task evaluates whether the lifted-state formulation can maintain branch-consistent guidance on closed and self-intersecting path geometries. The second type is point-to-point navigation, where the robot moves from an initial position to a target region while following the currently valid reference path. This task evaluates whether the proposed ISF-GVF can provide coherent guidance during goal-directed navigation and path updates.

The same ISF-GVF formulation is used for both platforms, while the numerical gains are selected according to the admissible speed range and execution bandwidth of each platform. The main parameters used for guidance-layer validation are summarized in Table~\ref{tab:guidance_parameters}. These parameters include the ISF-GVF gains and the path-update settings, rather than low-level controller gains.

\begin{table}[!ht]
\centering
\caption{Main Parameters Used for Guidance-Layer Validation}
\label{tab:guidance_parameters}
\begin{tabular}{c l c c}
\toprule
Parameter & Description & UAV & Ground robot \\
\midrule
\multicolumn{4}{l}{\textit{ISF-GVF}} \\
\(K_1\) & Tangential-advancement gain & 2.0 & 1.0 \\
\(K_2\) & Transversal-contraction gain & 2.2 & 1.0 \\
\(r\) & Contraction bandwidth & 0.15 & 0.50 \\
\multicolumn{4}{l}{\textit{Planning}} \\
\(T_p\) & Replanning interval & 0.2 s & 0.2 s \\
\(d_{\mathrm{safe}}\) & Obstacle safety distance & 0.35 & 0.35 \\
\bottomrule
\end{tabular}
\end{table}

In all experiments, the same ISF-GVF construction is used at the guidance layer, while platform-dependent operations, such as command saturation, high-level reference generation, or constrained vehicle tracking, are handled only in the execution layer described in Section~V.



\subsection{RViz Simulation Results}
\label{subsec62}

\subsubsection{Closed-Trajectory Acquisition and Tracking}

RViz simulations are first conducted to evaluate the geometric behavior of the proposed ISF-GVF in closed-trajectory acquisition and tracking tasks. Two representative closed trajectories are considered: a circular trajectory and a figure-eight trajectory. The circular trajectory is used to evaluate closed-path acquisition and sustained tracking, whereas the figure-eight trajectory is used to examine branch-consistent guidance near a self-intersection.

The simulations are performed in obstacle-populated environments. When obstacles lie on or near the nominal closed reference curve, the currently valid path may locally deviate from the nominal trajectory. In this case, the proposed ISF-GVF is reconstructed around the updated path while retaining the internal phase state, allowing the robot to avoid obstacles and subsequently reacquire the closed trajectory.

\begin{figure}[!t]
    \centering
    \includegraphics[width=\linewidth]{Cxq_pic8.png}
    \caption{RViz closed-trajectory simulation results on heterogeneous robotic platforms.
    (a)--(d) Quadrotor UAV results, and (e)--(h) differential-drive ground robot results.
    The red curves denote reference trajectories, the green curves denote executed trajectories, and occupied cells represent obstacles.}
    \label{fig:closed_tracking_rviz}
\end{figure}

Fig.~\ref{fig:closed_tracking_rviz} shows the RViz simulation results of closed-trajectory acquisition and tracking on the quadrotor UAV and the differential-drive ground robot. For each platform, the circular cases evaluate closed-path acquisition from different off-path initial states, while the figure-eight cases evaluate branch-consistent guidance near the self-intersection. The results from \(x_1(0)\) and \(x_2(0)\) indicate that the proposed guidance layer does not require the robot to be initialized close to the reference curve.

In the figure-eight cases, the simulated trajectories pass through the crossing region along the intended branch. This behavior indicates that the lifted phase state preserves the branch information required for continuous guidance, whereas a purely physical-space guidance description may become ambiguous near the self-intersection.

The results also show local obstacle-avoidance behavior around the nominal closed reference curve. When obstacles lie on or near the reference trajectory, the executed trajectory may locally deviate from the nominal curve and then reacquire the closed trajectory after the obstacle is avoided. This behavior is consistent with the incremental update mechanism, where the ISF-GVF is reconstructed around the currently valid path while preserving the internal phase state.

\subsubsection{Point-to-Point Navigation Under Path Updates}

Point-to-point navigation simulations are further conducted to evaluate the proposed ISF-GVF under goal-directed navigation with path updates. In these simulations, the robot moves from an initial state to a target region while the reference path is updated according to the current obstacle distribution and planning result. The purpose is to verify that the guidance layer remains coherent when the currently valid path changes, rather than only tracking a fixed reference trajectory.

\begin{figure*}[!t]
    \centering
    \includegraphics[width=0.98\textwidth]{Cxq_pic9.png}\\[1mm]
    \includegraphics[width=0.98\textwidth]{Cxq_pic10.png}
    \caption{RViz point-to-point navigation simulation results under path updates on heterogeneous robotic platforms.
    A1--A4 show the quadrotor UAV results, and B1--B4 show the differential-drive ground robot results.
    A1--A2 and B1--B2 show global navigation views at different stages, while A3--A4 and B3--B4 show zoomed-in views of the local guidance behavior near obstacles.
    The green curves denote locally updated reference paths, and the red arrows indicate local ISF-GVF guidance/vector-field information.}
    \label{fig:point_to_point_rviz}
\end{figure*}

Fig.~\ref{fig:point_to_point_rviz} shows the RViz simulation results of point-to-point navigation for both platforms.
In A1--A2 and B1--B2, the robots move through obstacle-populated environments toward the target region while following the currently valid reference paths.
The executed trajectories show that the proposed guidance layer can provide coherent motion guidance during goal-directed navigation.

The zoomed-in views in A3--A4 and B3--B4 further illustrate the local behavior of the proposed ISF-GVF near obstacles.
When the reference path is locally adjusted by the planning layer, the guidance field is reconstructed around the updated path while retaining the internal phase state.
As a result, the guidance process continues without restarting the path phase, which is consistent with the incremental update mechanism described in Section~III-D.

For the UAV, the physical-space guidance output is converted into a bounded high-level motion reference and executed through the flight-control interface.
For the differential-drive ground robot, the lifted guidance information is converted into a phase-consistent short-horizon reference and tracked by the constrained vehicle execution layer.
The results indicate that the same ISF-GVF guidance formulation can support point-to-point navigation after being mapped through different platform execution interfaces.

Together, the RViz simulation results show that the proposed lifted-state guidance formulation can support closed-trajectory acquisition from different initial states, local obstacle avoidance with trajectory reacquisition, branch-consistent guidance on self-intersecting reference paths, and coherent point-to-point navigation under path updates.



\subsection{Real-World Platform Experiments}
\label{subsec63}

Real-world experiments are further conducted on the quadrotor UAV and the differential-drive ground robot to validate the deployability of the proposed ISF-GVF framework on physical robotic platforms. Different from the RViz simulations in Section~VI-B, which are used to examine the geometric guidance behavior under controlled obstacle configurations, the real-world experiments evaluate whether the same guidance layer can be executed through the physical sensing, state-estimation, and control interfaces of the two platforms.

% 后面补实机图
% \begin{figure*}[!t]
%     \centering
%     \includegraphics[width=0.95\textwidth]{Cxq_pic11.png}
%     \caption{Real-world experimental results of the proposed ISF-GVF framework on heterogeneous robotic platforms.
%     A1--A4 show the quadrotor UAV experiments, and B1--B4 show the differential-drive ground robot experiments.
%     The snapshots and trajectory visualizations show that the same guidance layer can be deployed through different platform execution interfaces.}
%     \label{fig:real_world_experiments}
% \end{figure*}

Fig.~\ref{fig:real_world_experiments} shows the real-world experimental results on the quadrotor UAV and the differential-drive ground robot. For the quadrotor UAV, the physical-space guidance output is converted into a bounded high-level motion reference and executed through the onboard flight-control interface. For the differential-drive ground robot, the lifted guidance information is converted into a phase-consistent short-horizon reference and tracked by the nonholonomic vehicle execution layer.

The real-world experiments are not intended to repeat every closed-trajectory and point-to-point case shown in the RViz simulations. Instead, they verify that the proposed guidance layer can operate with real sensing feedback, platform-specific command interfaces, and practical execution constraints. The results show that the ISF-GVF guidance output can be consistently interpreted and executed on both aerial and ground robotic platforms.

These physical experiments further support the separation between the guidance layer and the execution layer. The ISF-GVF remains responsible for phase-continuous guidance, branch-consistent path interpretation, and generation of the unified geometric guidance output, while the platform-dependent execution modules convert this output into commands compatible with the corresponding robot. This confirms that the proposed method can be used as a reusable upper-layer guidance mechanism for heterogeneous motion-constrained robotic platforms.



\subsection{Cross-Platform Discussion}
\label{subsec64}

The simulation and real-world results demonstrate the role of the proposed ISF-GVF as a unified guidance layer rather than a platform-specific controller. In the quadrotor UAV experiments, the physical-space guidance output is converted into a bounded high-level motion reference and executed through the flight-control interface. In the ground-robot experiments, the same lifted guidance information is converted into a phase-consistent short-horizon reference and executed through a constrained vehicle tracking layer.

The RViz closed-trajectory simulations validate three geometric properties of the proposed method. First, the robot can acquire and track circular closed trajectories from different off-path initial states. Second, the robot can locally deviate from the nominal closed curve when obstacles lie on or near the reference trajectory and then reacquire the trajectory after avoidance. Third, the figure-eight cases show that the lifted-state formulation maintains branch-consistent guidance near self-intersections.

The RViz point-to-point simulations further validate the incremental guidance behavior under path updates. When the reference path is locally adjusted by the planning layer, the guidance field is reconstructed around the currently valid path while the internal phase state is retained. As a result, the guidance process does not need to restart after replanning, and the robot can continue toward the target region under the updated reference path.

The real-world platform experiments complement the RViz simulations by verifying physical deployability. These experiments show that the proposed guidance output can be interpreted and executed through different sensing, state-estimation, and control interfaces. Therefore, the observed performance is not tied to a single robot model or a single execution interface.

Overall, the experimental results support the main claim that the proposed ISF-GVF provides an incremental, singularity-free, and platform-agnostic guidance layer for autonomous navigation under both closed-trajectory and goal-directed navigation tasks.


\section{Conclusion}
\label{sec7}

This paper proposed an incremental singularity-free guidance vector field for autonomous navigation under path updates and complex path geometries. The proposed method formulates the guidance problem in a lifted state space by explicitly incorporating the path phase into the system state. This formulation separates physically overlapping but semantically distinct path branches and provides a basis for singularity-free and branch-consistent guidance on closed, self-intersecting, looping, and revisiting trajectories.

The ISF-GVF is constructed by combining tangential advancement, transversal contraction, and phase correction in the lifted state space. When the reference path is updated by replanning, the guidance field is reconstructed around the currently valid path while the internal phase state is retained. This incremental update mechanism allows the guidance process to continue without reinitializing the path phase. The physical-space component of the lifted guidance field is further extracted as a unified geometric guidance output, which can be interpreted by different motion-constrained robotic platforms through platform-specific execution interfaces.

The main analytical properties of the proposed method were established, including local regularity, update-compatible smoothness, exact error dynamics, singularity-free property, forward completeness, and asymptotic convergence to the reference path under the ideal lifted dynamics. In addition, bounded-set convergence-rate estimates were derived under sector-bounded contraction profiles.These results clarify the theoretical role of the ISF-GVF as a guidance-layer mechanism rather than a platform-specific controller.

Experimental validation was conducted on heterogeneous robotic platforms, including a quadrotor UAV and a differential-drive ground robot. RViz simulations demonstrated closed-trajectory acquisition from different off-path initial states, local obstacle avoidance with trajectory reacquisition, branch-consistent guidance near self-intersections, and coherent point-to-point navigation under path updates. Real-world platform experiments further verified that the same guidance layer can be deployed through different sensing, state-estimation, and execution interfaces on physical robotic systems.

Overall, the results show that the proposed ISF-GVF provides an incremental, singularity-free, and platform-agnostic guidance layer for autonomous navigation. Future work will focus on extending the framework to more general dynamic environments, incorporating explicit robustness analysis for platform execution errors, and validating the method on additional robotic platforms with different motion constraints.

\section*{Acknowledgment}
The authors gratefully acknowledge Dr. Ruocheng Li of Beijing Institute of Technology, China. His insightful revision suggestions during the manuscript review improved the work's quality and rigor.

\printbibliography
\vfill
\end{document}



