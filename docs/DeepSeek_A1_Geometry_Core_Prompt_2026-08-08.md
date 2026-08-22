# 给 DeepSeek 的 A1 几何核心任务提示词

将下面横线之间的内容完整发送给 DeepSeek。不要只说“继续 A1”。

```text
DOCUMENT_ROLE=STAGE_EXECUTION_SPEC
STAGE=A1
PREVIOUS_STAGE=A0
AUTO_ADVANCE=false
ALLOWED_TO_EXECUTE=A1_ONLY
```

本文件是当前唯一执行授权。总体计划和架构文档只提供背景，不授权执行 A2 或任何后续阶段。

---

你现在只执行 PhaseOffsetSwarm 实施计划中的 **A1：独立 phase–offset 几何核心**。A1 完成、自审核并汇报后必须停止，禁止进入 A2。

工作区：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

开始前完整阅读：

- `docs/PhaseOffsetSwarm_Single_First_Implementation_Plan_2026-08-07.md`
- `docs/PhaseOffsetSwarm_Code_Architecture_2026-08-08.md`

## 一、A1 唯一目标

新建一个不依赖 ROS、不依赖原 `bspline_race`、不依赖地图和多机代码的纯数学 catkin package：

```text
src/swarm_planner/phase_offset/phase_offset_core
```

本阶段只实现和测试：

\[
p,p_w,p_{ww},x,\delta
\longrightarrow
T,N,\kappa,r,r_w,e_\parallel,e_\perp.
\]

不实现 matched port、端口投影、tube、continuation、swarm、CBF 和 ROS 控制接入。

## 二、开始前状态检查

执行并记录：

```bash
cd /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
git branch --show-current
git rev-parse --short HEAD
git status --short
git stash list
git diff -- src/swarm_planner/bspline_traj/launch/test_gvf.launch
```

必须确认：

- branch 为 `main`；
- HEAD 为 `9a0e975`；
- `stash@{0}` 为 `deepseek-phaseoffset-tracked-prototype-2026-08-08`；
- tracked 修改只有 `test_gvf.launch` 的两个 circle 参数；
- A0 未被破坏。

若不一致，停止并汇报。

开始前记录旧原型的校验值，保证本轮不修改它们：

```bash
sha256sum \
  src/swarm_planner/bspline_traj/include/bspline_race/phase_offset_geometry.h \
  src/swarm_planner/bspline_traj/include/bspline_race/phase_offset_types.h \
  src/swarm_planner/bspline_traj/src/phase_offset_geometry.cpp
```

## 三、只创建以下目录和文件

```text
src/swarm_planner/phase_offset/phase_offset_core/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/phase_offset_core/
│   ├── path_state.h
│   ├── geometry_types.h
│   └── geometry.h
├── src/
│   └── geometry.cpp
└── test/
    └── geometry_test.cpp
```

本轮不要提前创建空的 `matched_port.*`、`port_projector.*`、`continuation.*` 或 navigation/swarm 目录。

## 四、依赖约束

`phase_offset_core` 只允许依赖：

- Eigen3；
- C++14 标准库；
- catkin/gtest 构建基础。

include 和 src 中禁止出现：

- `ros/ros.h`、`ros/time.h`；
- `common_msgs`、`geometry_msgs`、`nav_msgs`；
- `plan_env`、`SDFMap`；
- `bspline_race`、`ContinuousPhasePath`；
- `gvf_manager`、publisher、subscriber、topic；
- `Swarm`、`Neighbor`、`CBF`、`Tube`、robot ID。

不要修改原 `common_msgs`、`bspline_traj/CMakeLists.txt` 或工作区其他 package。

## 五、数据类型

### 5.1 path_state.h

至少定义：

```cpp
namespace phase_offset_core {

struct PathDifferentialState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_ww = Eigen::Vector3d::Zero();
  double w = 0.0;
  bool valid = false;
};

}  // namespace phase_offset_core
```

核心包不包含 B 样条参数、path epoch 和 ROS 时间戳。

### 5.2 geometry_types.h

建议拆分：

```cpp
struct GeometryParams {
  double tangent_epsilon = 1e-8;
  double regularity_margin = 0.1;
  double planar_tolerance = 1e-6;
  bool require_planar_xy = true;
};
```

```cpp
struct PhaseOffsetGeometryState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_ww = Eigen::Vector3d::Zero();
  Eigen::Vector3d T = Eigen::Vector3d::Zero();
  Eigen::Vector3d N = Eigen::Vector3d::Zero();
  Eigen::Vector3d r = Eigen::Vector3d::Zero();
  Eigen::Vector3d r_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d error = Eigen::Vector3d::Zero();
  Eigen::Vector3d e_perp = Eigen::Vector3d::Zero();
  double w = 0.0;
  double delta = 0.0;
  double path_speed = 0.0;
  double curvature = 0.0;
  double regularity = 0.0;
  double e_parallel = 0.0;
  bool valid = false;
  std::string invalid_reason;
};
```

可以根据实现需要小幅调整字段，但不能加入 tube、swarm、CBF 和控制模式状态。

## 六、几何公式

当前第一版固定为二维 XY、固定高度。令

\[
v_p=\sqrt{p_{w,x}^2+p_{w,y}^2}.
\]

要求

\[
v_p>\varepsilon_T,
\qquad
|p_{w,z}|\le\varepsilon_z
\]

时定义

\[
T=\frac{p_w}{v_p},
\qquad
N=(-T_y,T_x,0)^T.
\]

平面有符号曲率：

\[
\kappa
=
\frac{p_{w,x}p_{ww,y}-p_{w,y}p_{ww,x}}
{v_p^3}.
\]

活动参考：

\[
r=p+N\delta.
\]

固定 \(\delta\) 时：

\[
r_w=(1-\kappa\delta)p_w.
\]

正则性：

\[
\gamma=1-\kappa\delta.
\]

有效条件至少包括：

\[
\gamma\ge\mu>0.
\]

相对活动参考误差：

\[
e=x-r,
\qquad
e_\parallel=T^Te,
\qquad
e_\perp=e-e_\parallel T.
\]

实现中必须检查所有输入和输出为 finite。任何无效情况返回 `valid=false` 和明确 `invalid_reason`，不能产生 NaN 后继续运行。

## 七、GeometryEvaluator 接口

建议：

```cpp
class GeometryEvaluator {
 public:
  explicit GeometryEvaluator(const GeometryParams& params);

  bool evaluate(const PathDifferentialState& path,
                const Eigen::Vector3d& position,
                double delta,
                PhaseOffsetGeometryState& output) const;

 private:
  GeometryParams params_;
};
```

允许采用等价的无状态 free function，但 API 必须简洁、可测试、无 ROS 副作用。

## 八、必须完成的单元测试

至少覆盖：

### 8.1 直线

- \(p_w\) 为常向量；
- \(\kappa=0\)；
- \(r=p+N\delta\)；
- \(r_w=p_w\)；
- \(T^TN=0\)；
- 误差分解重构正确。

### 8.2 圆

使用解析圆路径，验证：

- 曲率符号和大小；
- \(T,N\) 正交且为单位向量；
- \(r_w=(1-\kappa\delta)p_w\)；
- 解析 \(r_w\) 与中心差分数值导数一致；
- 正 offset 对应当前 N 方向。

### 8.3 8 字局部段

在非退化相位采样，验证：

- 几何量 finite；
- \(T^TN\approx0\)；
- 解析 \(r_w\) 与数值导数一致；
- 自交位置的不同 phase 仍由输入 \(w\) 区分，不在 core 中做最近点选支。

### 8.4 零 offset 退化

多个路径和相位下验证：

\[
\delta=0\Rightarrow r=p,\quad r_w=p_w.
\]

### 8.5 无效输入

至少验证：

- `path.valid=false`；
- 零或过小切向；
- 非 finite 输入；
- 超出 planar tolerance 的 \(p_{w,z}\)；
- \(1-\kappa\delta<\mu\)。

所有无效情况必须返回 false、`valid=false` 和非空原因。

### 8.6 随机/多采样一致性

在一组有效圆或一般平面参数曲线上多点采样，验证正交性、finite 和数值导数误差上界，避免只针对单个相位写死。

## 九、CMake 与测试

只创建一个 library：

```text
phase_offset_core
```

只创建一个测试目标：

```text
phase_offset_geometry_test
```

不要修改 `bspline_gvf` target，不要链接 `plan_env`、`common_msgs` 或 `bspline_race`。

构建和测试：

```bash
source /opt/ros/noetic/setup.bash
cd /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
catkin_make -j8
catkin_make run_tests_phase_offset_core
catkin_test_results --verbose
```

同时重新运行原包测试，确认新 package 没有破坏 A0：

```bash
catkin_make run_tests_bspline_race
catkin_test_results --verbose
```

A1 不启动 ROS simulator，不做 RViz 飞行测试。

## 十、自审核

完成后自行检查并修复仅限 A1 范围内的问题：

```bash
git status --short
git diff --name-only
git diff --check
find src/swarm_planner/phase_offset/phase_offset_core -maxdepth 4 -type f | sort
rg -n "ros/|ros::|common_msgs|geometry_msgs|nav_msgs|plan_env|SDFMap|bspline_race|gvf_manager|Swarm|Neighbor|CBF|Tube" \
  src/swarm_planner/phase_offset/phase_offset_core/include \
  src/swarm_planner/phase_offset/phase_offset_core/src
```

预期：

- tracked diff 仍只有 `test_gvf.launch` 两行；
- A1 所有内容都位于新 `phase_offset_core` package；
- core include/src 搜索不到上述禁止依赖；
- 原 untracked prototype 文件未修改。

重新计算旧原型校验值，与开始前对比：

```bash
sha256sum \
  src/swarm_planner/bspline_traj/include/bspline_race/phase_offset_geometry.h \
  src/swarm_planner/bspline_traj/include/bspline_race/phase_offset_types.h \
  src/swarm_planner/bspline_traj/src/phase_offset_geometry.cpp
```

## 十一、严格禁止

1. 修改任何已有 tracked 源码、CMake、package.xml 或 launch；
2. 修改或移动旧 untracked phase-offset 原型；
3. 从 stash 恢复文件；
4. 接入 `gvf_manager`、`gvf` 或 `bspline_gvf`；
5. 创建 adapter、runtime、navigation、msgs 或 swarm package；
6. 实现 matched port、allocator、tube、continuation、CBF 或 neighbor；
7. 添加 publisher、subscriber、timer、node 或 launch；
8. 修改 K1/K2、C2、SDF、A*、B 样条或 simulator；
9. 创建 Git commit；
10. 自动继续 A2。

## 十二、最终汇报格式

完成后按以下格式汇报：

1. 开始时 branch、HEAD、stash 和 tracked diff；
2. A1 新增文件列表；
3. package 依赖；
4. 核心 API；
5. 实现的公式和有效性条件；
6. 单元测试列表与结果；
7. 全量编译结果；
8. 原 `bspline_race` 回归测试结果；
9. 禁止依赖搜索结果；
10. 旧原型校验值前后对比；
11. 最终 `git status --short`；
12. 自审核发现和修复的问题；
13. 明确写出：`A1 完成后已停止，未进入 A2`。

若 A1 范围内无法解决，保留现场并汇报，不扩大任务范围。

---
