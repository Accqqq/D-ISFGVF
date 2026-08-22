# DeepSeek A2 Shadow 接入执行单

```text
DOCUMENT_ROLE=STAGE_EXECUTION_SPEC
STAGE=A2
PREVIOUS_STAGE=A1
AUTO_ADVANCE=false
ALLOWED_TO_EXECUTE=A2_ONLY
```

## 1. 唯一目标

只把已经通过 A1 的 `phase_offset_core` 以只读 Shadow 方式接入原
`bspline_race`。

原 ISF-GVF、semantic phase、PositionCommand、command governor、B 样条、
C2 connector 和 SO3 控制继续作为唯一实际控制链。

A2 只完成：

1. 将当前 `ContinuousPhasePathState` 转换为
   `phase_offset_core::PathDifferentialState`；
2. 在原 semantic phase \(w\) 上计算固定测试 offset
   \(\delta_{shadow}\) 对应的候选参考；
3. 只读发布基础路径、shadow 候选参考、当前 \(T,N\) 和几何诊断；
4. 验证开启 Shadow 不改变原单机控制和到达结果。

候选参考定义为：

\[
r^{shadow}(w)=p(w)+N(w)\delta_{shadow}.
\]

它不是实际控制参考，也不宣称满足安全 tube。

A2 不实现 matched port、\(u_w\)、\(u_\delta\)、tube、allocator、CBF、
continuation 或 swarm。

## 2. 开始前必须阅读

完整阅读：

- `/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/AGENTS.md`
- `docs/PhaseOffsetSwarm_Single_First_Implementation_Plan_2026-08-07.md`
- `docs/PhaseOffsetSwarm_Code_Architecture_2026-08-08.md`

总体计划和架构文档只提供背景，本文件是本轮唯一执行授权。

## 3. 开始前自检

执行：

```bash
cd /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
git branch --show-current
git rev-parse --short HEAD
git status --short
git diff --check
git stash list
catkin_make run_tests_phase_offset_core
```

必须确认：

- branch=`main`；
- HEAD=`9a0e975`；
- prototype stash 仍存在；
- A1 geometry test 为 7/7；
- tracked diff 在开始 A2 前只有 `test_gvf.launch` 的两个 circle 参数；
- A1 package 不含 ROS、地图和原工程依赖。

若不满足，停止并汇报。

## 4. 文件白名单

### 4.1 允许新增

```text
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_shadow_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_shadow_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_shadow_adapter_test.cpp
src/swarm_planner/bspline_traj/launch/phase_offset_shadow_single.launch
```

### 4.2 允许修改

```text
src/swarm_planner/bspline_traj/CMakeLists.txt
src/swarm_planner/bspline_traj/package.xml
src/swarm_planner/bspline_traj/include/bspline_race/gvf_manager.h
src/swarm_planner/bspline_traj/src/gvf_manager.cpp
src/swarm_planner/bspline_traj/launch/test_gvf.launch
```

如果认为需要修改其他文件，立即停止并汇报，不得扩大白名单。

禁止修改 `phase_offset_core`、旧 untracked prototype、`gvf.h/.cpp`、
`continuous_phase_path.*`、SDF、A*、B 样条、C2、simulator 和 common_msgs。

## 5. Shadow 模式参数

在 `test_gvf.launch` 中只增加以下参数入口，并保持默认 disabled：

```text
phase_offset/mode=disabled
phase_offset/shadow_delta=0.0
phase_offset/shadow_sample_step_w=0.10
phase_offset/shadow_publish_rate=5.0
phase_offset/frame_id=world
```

要求：

- `disabled`：不构造 Shadow publisher，不计算几何，不采样路径，不改变
  原 ROS graph 和控制流程；
- `shadow`：只读计算和发布；
- `active` 或未知字符串：A2 中不得启用，明确报错并安全退回 disabled；
- 原 `circle_test/enable=false` 和 `auto_start=false` 必须保留。

新增 `phase_offset_shadow_single.launch` 只包装原 `test_gvf.launch`，把 mode
设置为 shadow，并允许从命令行设置 `shadow_delta`。它不启动 simulator、地图
或第二套 RViz。

## 6. Adapter 架构

Adapter 位于 `bspline_race/integration`，可以依赖：

- `phase_offset_core`；
- `ContinuousPhasePathState` 的头文件；
- roscpp、std_msgs、visualization_msgs；
- Eigen。

Adapter 禁止依赖：

- `gvf_manager`；
- SDFMap；
- common_msgs 新消息；
- swarm、neighbor、CBF、tube；
- PositionCommand 和 SO3 command。

建议接口至少包含：

```cpp
enum class PhaseOffsetShadowMode {
  DISABLED = 0,
  SHADOW = 1
};

struct ShadowConfig {
  PhaseOffsetShadowMode mode = PhaseOffsetShadowMode::DISABLED;
  double delta = 0.0;
  double sample_step_w = 0.10;
  double publish_rate = 5.0;
  std::string frame_id = "world";
};

struct ShadowUpdateInput {
  phase_offset_core::PathDifferentialState current_path;
  std::vector<phase_offset_core::PathDifferentialState,
              Eigen::aligned_allocator<
                  phase_offset_core::PathDifferentialState>> sampled_path;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  ros::Time stamp;
};

struct ShadowDiagnostics {
  bool valid = false;
  double w = 0.0;
  double delta = 0.0;
  double curvature = 0.0;
  double regularity = 0.0;
  double e_parallel = 0.0;
  double e_perp_norm = 0.0;
  std::string invalid_reason;
};
```

具体命名可以小幅调整，但必须保持只读和单一职责。

Adapter 应提供一个明确转换函数，把：

```text
ContinuousPhasePathState + w
```

转换为：

```text
PathDifferentialState
```

转换必须逐项保持：

```text
p -> p
dp_dw -> p_w
d2p_dw2 -> p_ww
w -> w
valid -> valid
```

不得重新投影相位，不得使用全路径最近点。

## 7. Shadow 发布内容

使用 private namespace 发布，推荐：

```text
~phase_offset_shadow/base_path
~phase_offset_shadow/candidate_path
~phase_offset_shadow/frame
~phase_offset_shadow/diagnostics
```

发布内容：

- 基础路径 \(p(w)\)：蓝色 LINE_STRIP；
- shadow 候选参考 \(r^{shadow}(w)\)：紫色或洋红 LINE_STRIP；
- 当前基础参考点 \(p(w)\)；
- 当前候选参考点 \(r^{shadow}(w)\)；
- 当前切向 \(T\)：绿色箭头；
- 当前法向 \(N\)：橙色箭头；
- 标准消息诊断：\(w,\delta_{shadow},\kappa,\gamma,e_\parallel,
  \|e_\perp\|,valid\)。

不新增自定义消息。可以使用 `std_msgs::Float64MultiArray`，但必须在代码
注释或 README 中写清字段顺序。

若当前几何无效：

- 不发布非 finite Marker；
- 诊断 valid=0；
- 使用 throttle warning；
- 不影响原控制。

## 8. manager 接入限制

`gvf_manager` 只做以下工作：

1. 读取 Shadow 参数；
2. mode=shadow 时构造一个 adapter；
3. 从当前 `ContinuousPhasePath` 在原 semantic phase 上 evaluate；
4. 按配置步长 sample 当前路径；
5. 转换输入并调用 adapter；
6. 不读取 adapter 输出用于控制。

调用 Shadow 时必须使用原控制本周期使用的 semantic phase，不能重新寻找
最近点，不能创建第二个 phase 状态。

建议在现有 command callback 中以只读方式调用，并由 adapter 自身按
`shadow_publish_rate` 节流。禁止创建新的控制 timer 或 subscriber。

Shadow 调用不得修改或引用为非常量输出：

- `phase_w_`；
- `progress_w_`；
- `out.v_cmd`、`out.w_dot`；
- `GovernorCommandResult`；
- `PositionCommand`；
- command governor 内部状态；
- B 样条和 C2 path。

目标：A2 对 `gvf_manager.cpp` 的新增尽量控制在约 100 行以内，绝对不能
把 Marker 构造和几何公式写入 manager。

## 9. CMake 与 package

`bspline_race` 增加对 `phase_offset_core` 的正式依赖。

新增独立 library：

```text
phase_offset_shadow_adapter
```

不要把 adapter 源码直接堆进 `bspline_gvf` 的 source list。`bspline_gvf`
可以链接该小库。

新增测试目标：

```text
phase_offset_shadow_adapter_test
```

该测试只链接必要 library，不链接 simulator 或 swarm prototype。

## 10. 单元测试

至少测试：

1. `ContinuousPhasePathState + w` 转换逐项一致；
2. 固定直线样本在 shadow delta 下得到正确候选路径；
3. Marker 中基础路径和候选路径点数、位置正确；
4. 当前 \(T,N,p,r\) Marker 均 finite；
5. 无效路径不会产生非 finite Marker；
6. disabled mode 不发布、不计算；
7. Adapter API 不包含 PositionCommand、phase 写回或 governor 输出。

构建与测试：

```bash
source /opt/ros/noetic/setup.bash
cd /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
catkin_make -j8
catkin_make run_tests_phase_offset_core
catkin_make run_tests_bspline_race
catkin_test_results --verbose
```

要求：

- A1 geometry 7/7 保持通过；
- 原 bspline_race 64 项保持通过；
- 新 Shadow adapter 测试通过；
- 无新增编译警告。

## 11. ROS 运行验收

A2 是第一次允许启动 launch 观察 Shadow。

### 11.1 disabled 回归

启动原：

```bash
roslaunch so3_quadrotor_simulator simulator.launch
roslaunch bspline_race test_gvf.launch
```

确认：

- 不存在 `phase_offset_shadow` publisher/topic；
- 原 `pillar.pcd`；
- 用户手工发布目标后仍能规划、避障并到达；
- 不自动进入圆轨迹。

### 11.2 shadow 运行

重新启动原 simulator，并启动：

```bash
roslaunch bspline_race phase_offset_shadow_single.launch \
  shadow_delta:=0.40
```

用户在 RViz 发布与 A0 相同或同类安全目标。验证：

- UAV 仍由原 ISF-GVF 控制并到达；
- RViz 同时显示基础路径和 shadow 候选参考；
- 正 offset 位于 N 指向的一侧；
- T、N 单位正交且方向随路径变化；
- 自交/重复访问时使用当前 semantic phase，不切换到另一分支；
- candidate path 穿过障碍也只能视为诊断，因为 A2 尚无 tube；
- `/position_cmd` 仍只有原 formation_planning 控制源；
- Shadow 没有控制 publisher。

记录 disabled 和 shadow 两次运行的：

- 目标；
- 是否到达；
- 最终误差；
- 节点/topic；
- `PositionCommand` publisher；
- Shadow topic 频率；
- 关键 RViz 截图。

不要求两次开放环轨迹逐采样完全相同，但必须从代码结构和运行结果证明
Shadow 没有写入控制链。

## 12. 自审核

完成后执行：

```bash
git status --short
git diff --check
git diff --stat
rg -n "PositionCommand|cmd_pub|phase_w_|progress_w_|GovernorCommand|SDFMap|CBF|Neighbor|Swarm|u_delta|u_w" \
  src/swarm_planner/bspline_traj/include/bspline_race/integration \
  src/swarm_planner/bspline_traj/src/integration
```

Adapter 中不应出现 PositionCommand、控制 publisher、phase 写回、SDF、CBF、
neighbor、swarm 和端口命令。

检查 manager 新增行数、所有变更是否在白名单内、A1 package 是否未修改、旧
prototype 校验值是否保持。

## 13. 严格禁止

1. 修改 `phase_offset_core`；
2. 实现 matched port、端口投影或 active mode；
3. 实现 tube、ESDF 查询、CBF、neighbor 或 swarm；
4. 修改 `gvf.h/.cpp` 和 ISF 公式；
5. 修改 phase 更新、governor、PositionCommand 和 SO3；
6. 修改 A*、B 样条、C2、SDF 和 simulator；
7. 修改 K1/K2、最大速度和规划参数；
8. 恢复 stash 或接入旧 prototype；
9. 新建 navigation、msgs 或 swarm package；
10. 创建 commit；
11. 自动进入 A3。

## 14. 最终汇报格式

1. 开始状态和 A1 回归；
2. 新增和修改文件；
3. Adapter API 和依赖；
4. manager 接入位置与新增行数；
5. 参数及默认 disabled 行为；
6. 单元测试结果；
7. A1 和原 bspline_race 回归结果；
8. disabled ROS 结果；
9. shadow ROS 结果与截图路径；
10. `/position_cmd` 控制源检查；
11. 最终 Git 状态；
12. 自审核发现及修正；
13. 明确写出：`A2 完成后已停止，未进入 A3`。

任何验收需要越过白名单或实现后续功能时，停止并汇报，不得扩大范围。
