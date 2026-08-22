# DeepSeek A5.1 原 Simulator RViz Tube 可视化修复执行单（修订版）

工作目录：

`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

阶段控制：

`AUTO_ADVANCE=false`

本修订版完整取代此前“新建专用 RViz config、由 A5 wrapper 自动再启动一个
RViz”的 A5.1 方案。此前方案不符合用户的实际启动方式，禁止继续实施或恢复。

本轮只执行 A5.1 可视化修复。完成测试、隔离 ROS 验收、自审核和汇报后立即
停止。禁止修改 A5 控制/安全算法，禁止修复 A5 tracking blocker，禁止进入 A6。

## 1. 用户实际启动方式

用户分别启动：

```bash
roslaunch so3_quadrotor_simulator simulator.launch
roslaunch bspline_race test_gvf.launch ...
```

`simulator.launch` 已经启动原有主 RViz，并加载：

```text
src/uav_simulator/so3_quadrotor_simulator/config/swarm_rviz.rviz
```

因此正确方案是把 A5 显示接入这个原有 RViz，不得创建第二套 RViz config，
不得由 fixed/ESDF wrapper 再启动一个 RViz。

## 2. 当前事实

开始前必须核实：

1. 原 `simulator.launch` 加载 `swarm_rviz.rviz`；
2. `swarm_rviz.rviz` 已有一个用户添加的 `MarkerArray` display，topic 为
   `/formation_planning/phase_offset_manual/tube`，名称目前可能仍为通用
   `MarkerArray`；
3. `test_gvf.launch` 的 `phase_offset_manual_tube_source` 默认值为 `none`；
4. 所以裸启动 `test_gvf.launch` 时不会产生可见 tube，RViz display 只会收到
   DELETE 或没有完整 profile；
5. 当前 adapter 对 tube 只发布 ID 0/1 两条 `LINE_STRIP`；
6. fixed tube 的实际 lateral bounds 为约 `[-0.04,0.04] m`，两条线在全局视角
   下很难辨认；
7. 当前工作区没有 `config/phase_offset_tube.rviz`，两个 A5 wrapper 没有
   `launch_rviz`，adapter 没有 ID 2 ribbon；不得把此前未完成会话的文字汇报
   当作已经落盘的修改。

## 3. A5 状态保持

A5 已按停止条件终止：

- fixed tube 构建、投影和 ROS 验收通过；
- ESDF-ready profile、动态窄化、联合投影和 clearance 已出现；
- C2 更新后 tracking error 达到约 `0.176349849 m`，超过 `0.15 m` bound；
- Runtime 正确锁存 failure；
- A5 未完成 ESDF 动态验收；
- A6 continuation 尚未开始。

A5.1 只能修显示，不能改变以上结论。

## 4. 唯一目标

1. 在原 `swarm_rviz.rviz` 中清晰接入 A5 base、active、frame 和 tube topics；
2. 保留用户原 simulator 的 RViz、地图、视角、工具和其他 displays；
3. 在真实 lower/upper 边界之间增加半透明 `TRIANGLE_LIST` ribbon；
4. 保留 orange lower、cyan upper 两条真实边界；
5. invalid/incomplete 时 ID 0、1、2 全部发布 DELETE；
6. 用用户的分离式启动方式分别验证 fixed 与 ESDF tube；
7. 不改变 tube 几何、控制、安全、C2 或 tracking bound。

## 5. 开始前检查

完整读取：

- 根目录 `AGENTS.md`；
- A5 执行单；
- A5 最终停止汇报；
- 本 A5.1 修订版执行单；
- `simulator.launch`；
- `test_gvf.launch`；
- `swarm_rviz.rviz`；
- `phase_offset_matched_adapter.*` 及测试；
- fixed/ESDF wrapper launch，只读确认其当前行为。

执行并记录：

```bash
cd /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
git branch --show-current
git rev-parse HEAD
git status --short
git diff --stat
git diff --check
git stash list
git diff --cached --stat
wc -l \
  src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp \
  src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
git diff --numstat 9a0e975 -- \
  src/swarm_planner/bspline_traj/src/gvf_manager.cpp
rg -n "rviz|swarm_rviz" \
  src/uav_simulator/so3_quadrotor_simulator/launch/simulator.launch
rg -n -C 8 "phase_offset_manual/tube" \
  src/uav_simulator/so3_quadrotor_simulator/config/swarm_rviz.rviz
rg -n "phase_offset_mode|phase_offset_manual_tube_source" \
  src/swarm_planner/bspline_traj/launch/test_gvf.launch
```

必须保持：

- branch `main`；
- HEAD `9a0e97560b8ddf15ac1ed1581030ed4b911f6e43`；
- prototype stash `deepseek-phaseoffset-tracked-prototype-2026-08-08` 存在；
- adapter 开始时 442 行；
- manager 相对基线仍为 `+134/-0`；
- 无 staged 内容、commit、tag 或 push；
- `gvf/circle_test/enable=false`；
- `gvf/circle_test/auto_start=false`。

任一前置条件不符时停止。不要自行恢复、reset、clean 或覆盖用户文件。

## 6. 文件白名单

只允许修改以下 3 个既有文件：

```text
src/uav_simulator/so3_quadrotor_simulator/config/swarm_rviz.rviz
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
```

不允许新增任何文件。

特别禁止修改：

- `simulator.launch`、`run_rviz.launch`、`fpv.rviz`；
- `test_gvf.launch`；
- `phase_offset_fixed_tube_single.launch`；
- `phase_offset_esdf_tube_single.launch`；
- `package.xml`、`CMakeLists.txt`；
- `phase_offset_matched_adapter.h`；
- `gvf_manager.*`；
- `phase_offset_core`、`phase_offset_navigation`；
- Runtime、TubeBuilder、TubeFilter、PortProjector、MatchedPort；
- Shadow、A3 Active、ISFReferenceKernel；
- planner、B-spline、C2、SDFMap、governor、simulator、SO3；
- `common_msgs`、prototype 文件和 A6 文件。

需要白名单外修改时立即停止并报告。

## 7. 原 Simulator RViz 的最小修改

只对 `swarm_rviz.rviz` 的 displays 列表做最小文本 patch。禁止启动 RViz 后按
Save 覆盖整个配置，因为那会改写窗口布局、视角、dock state 和用户设置。

必须保留：

- Fixed Frame `world`；
- 原 `/mock_map` display；
- 原 odometry、trajectory、path、grid、tools；
- 原视角、Window Geometry 和其他 display 的 Enabled 状态；
- 用户已经添加的正确 tube topic。

在原 displays 中提供以下 4 项，默认启用：

```text
A5 Base Path
  Class: rviz/Marker
  Marker Topic: /formation_planning/phase_offset_manual/base_path

A5 Active Path
  Class: rviz/Marker
  Marker Topic: /formation_planning/phase_offset_manual/active_path

A5 Frame
  Class: rviz/MarkerArray
  Marker Topic: /formation_planning/phase_offset_manual/frame

A5 Tube
  Class: rviz/MarkerArray
  Marker Topic: /formation_planning/phase_offset_manual/tube
```

如果 tube display 已存在，只允许原地改名为 `A5 Tube` 并保留正确 topic，禁止
重复添加第二个相同 topic。

禁止出现：

```text
/phase_offset_swarm/vis/tubes
```

禁止创建：

```text
src/swarm_planner/bspline_traj/config/phase_offset_tube.rviz
```

禁止增加 `/phase_offset_tube_rviz` 节点或自动启动第三个 RViz。

## 8. Tube Marker 结构

继续在原 topic 发布同一个 `visualization_msgs/MarkerArray`：

```text
/formation_planning/phase_offset_manual/tube
```

namespace 保持：

```text
phase_offset_manual_tube
```

固定 ID：

- ID 0：lower boundary，`LINE_STRIP`，orange；
- ID 1：upper boundary，`LINE_STRIP`，cyan；
- ID 2：tube ribbon，`TRIANGLE_LIST`，半透明蓝绿或浅蓝。

不得改变 lower/upper 的真实几何位置：

\[
q_{\mathrm{lower}}=p+N\underline\delta,
\qquad
q_{\mathrm{upper}}=p+N\overline\delta.
\]

必须使用 `tube_profile.samples` 中已经过滤完成的 `filtered_lower` 和
`filtered_upper`，不得重新计算、放大、平移或平滑 bounds。

ribbon 对每个相邻 sample pair 生成两个三角形：

```text
lower_i, upper_i, upper_i+1
lower_i, upper_i+1, lower_i+1
```

要求：

- header stamp/frame 与边界一致；
- `pose.orientation.w=1`；
- ribbon alpha 在 `0.15–0.25`；
- lower/upper line width 可设为 `0.05–0.07 m`；
- 所有边界点和 ribbon 点必须 finite；
- ribbon 只能是只读可视化，不是碰撞体或安全证书；
- 不创建三维圆管，不改变实际 lateral bound。

## 9. 完整性与 DELETE 语义

只有同时满足下列条件时才发布三个 ADD Marker：

- `tube_profile.complete=true`；
- samples 数量至少为 2；
- 每个 sample 的 `p`、`N`、`filtered_lower`、`filtered_upper` finite；
- 每个 lower/upper point finite；
- ribbon 能完整生成 `6*(N-1)` 个 finite points。

否则必须在同一 namespace、同一周期发布：

- ID 0 DELETE；
- ID 1 DELETE；
- ID 2 DELETE。

禁止残留上一周期 ribbon，禁止发布残缺 `TRIANGLE_LIST`。base、active、frame
的既有 DELETE 行为保持不变。

## 10. 单元测试

只扩展 `phase_offset_matched_adapter_test.cpp`，不得修改 Runtime 输出或伪造
tube 数学。

至少覆盖：

1. valid fixed profile 产生恰好 3 个 tube Marker；
2. ID 固定为 0、1、2；
3. ID 0/1 为 `LINE_STRIP`；
4. ID 2 为 `TRIANGLE_LIST`；
5. N 个 samples 产生 `6*(N-1)` 个 ribbon points；
6. lower、upper、ribbon 所有 point finite；
7. lower/upper point 与 `p+N*filtered_bound` 一致；
8. ribbon 顶点只使用相邻 lower/upper 边界点；
9. ribbon alpha 大于 0 且小于 1；
10. incomplete、少于 2 samples、non-finite 任一种情况都对 0/1/2 发布 DELETE；
11. 原 diagnostics、matched residual、projection、DistanceQuery 测试保持；
12. adapter 仍无 control publisher。

## 11. 构建与回归

执行：

```bash
catkin_make -j8
catkin_make run_tests_phase_offset_core
catkin_make run_tests_phase_offset_navigation
catkin_make run_tests_bspline_race
catkin_test_results --verbose
```

只允许已知无关的 `uav_utils` 缺 gtest XML，不允许本阶段新增失败。

静态检查原启动链：

```bash
source devel/setup.bash
roslaunch --nodes so3_quadrotor_simulator simulator.launch
roslaunch --nodes bspline_race test_gvf.launch \
  phase_offset_mode:=manual \
  phase_offset_manual_tube_source:=fixed
```

要求：

- simulator 仍只列出原有 `/rviz` 和 `/rviz1`；
- 不出现 `/phase_offset_tube_rviz`；
- `test_gvf.launch` 不启动 simulator、controller、map_pub 或 RViz；
- `/position_cmd` 最终发布者仍只有 `/formation_planning`。

## 12. 隔离 ROS 验收原则

先检查用户现有 ROS master 和进程。不得连接、终止或干扰用户的默认 ROS 和
长期运行进程。

使用新的隔离端口，例如 `11331`。只清理本轮启动的进程。

所有验收都必须采用用户的分离式启动顺序，而不是 A5 wrapper：

1. 启动原 `simulator.launch`；
2. 确认原 `swarm_rviz.rviz` 已加载；
3. 在第二个进程启动带显式 tube 参数的 `test_gvf.launch`。

## 13. Fixed Tube 验收

隔离环境中启动：

```bash
roslaunch so3_quadrotor_simulator simulator.launch
```

然后启动：

```bash
roslaunch bspline_race test_gvf.launch \
  phase_offset_mode:=manual \
  phase_offset_manual_tube_source:=fixed \
  phase_offset_tube_fixed_delta_max:=0.04
```

发布一个此前安全通过的单目标：

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
"{header: {frame_id: world}, pose: {position: {x: 8.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"
```

验收：

- 用户原主 RViz 中出现 `A5 Base Path`、`A5 Active Path`、`A5 Frame`、
  `A5 Tube`；
- A5 Tube Status 为 Ok，topic 正确；
- MarkerArray 含 IDs 0、1、2 且 action=ADD；
- orange/cyan 两边界和半透明 ribbon 可见；
- ribbon 严格位于两边界之间；
- fixed 总宽仍约 `0.08 m`；
- tube 随有效 preview 更新；
- 无额外 RViz 窗口由 A5.1 启动；
- fixed projection、matched residual 和到达行为不变；
- `/position_cmd` 唯一发布者仍为 `/formation_planning`。

## 14. ESDF Tube 验收

使用新的干净隔离运行，仍先启动原 simulator，然后启动：

```bash
roslaunch bspline_race test_gvf.launch \
  phase_offset_mode:=manual \
  phase_offset_manual_tube_source:=esdf
```

使用 A5 中未触发 blocker 的单目标，例如 `(-6,3,1)`，只验收显示：

- 原 `/mock_map` 仍可见；
- obstacle-certified profile 后 IDs 0、1、2 action=ADD；
- ribbon 宽度随 ESDF filtered bounds 变化；
- 窄化处两边界和 ribbon 同步；
- profile invalid/incomplete 时三个 ID 全部 DELETE；
- 不要求越过已知 A5 continuation blocker。

如果运行触发 `0.15 m` tracking failure，必须按 A5 原行为停止并记录。禁止修改
bound、C2、速度、增益、地图或 margin。

## 15. 给用户的启动说明

最终汇报必须明确说明：裸启动

```bash
roslaunch bspline_race test_gvf.launch
```

时 `phase_offset_manual_tube_source=none`，不会显示 tube。这是保留基线的预期
行为，不是 RViz 故障。

Fixed tube 使用：

```bash
source devel/setup.bash
roslaunch so3_quadrotor_simulator simulator.launch
```

新终端：

```bash
source devel/setup.bash
roslaunch bspline_race test_gvf.launch \
  phase_offset_mode:=manual \
  phase_offset_manual_tube_source:=fixed \
  phase_offset_tube_fixed_delta_max:=0.04
```

ESDF tube 只把 `fixed` 改为 `esdf`。

## 16. 自审核

执行：

```bash
rg -n "/phase_offset_swarm/vis/tubes|phase_offset_tube.rviz|phase_offset_tube_rviz" \
  src/uav_simulator/so3_quadrotor_simulator/config/swarm_rviz.rviz \
  src/swarm_planner/bspline_traj/launch \
  src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp

rg -n "PositionCommand|cmd_pub|publishGovernorPositionCommand|SO3" \
  src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp

rg -n "A5 Base Path|A5 Active Path|A5 Frame|A5 Tube|phase_offset_manual" \
  src/uav_simulator/so3_quadrotor_simulator/config/swarm_rviz.rviz

wc -l \
  src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp

git diff --numstat 9a0e975 -- \
  src/swarm_planner/bspline_traj/src/gvf_manager.cpp

git diff -- \
  src/uav_simulator/so3_quadrotor_simulator/config/swarm_rviz.rviz

git diff --check
git status --short
git diff --stat
git diff --cached --stat
git stash list
```

要求：

- adapter 不超过 500 行；
- manager 仍为 `+134/-0`；
- A5 control/runtime/core/navigation diff 未变化；
- A5.1 只修改 3 个白名单文件；
- `swarm_rviz.rviz` diff 只包含四个 A5 displays 的最小修改，保留用户原有
  layout、view、map 和其他 displays；
- 未新增 RViz config、launch node 或 dependency；
- circle 两项仍为 false；
- 无 staged、commit、tag 或 push；
- 隔离 ROS 端口释放。

## 17. 严格禁止

1. 新建专用 RViz config；
2. 自动启动额外 RViz；
3. 修改 simulator 或 test_gvf launch；
4. 修改 tube 数学、安全 margin 或 profile；
5. 修改 tracking bound；
6. 修改 Runtime failure 逻辑；
7. 修改 PortProjector、MatchedPort 或 C2；
8. 修改 phase、delta 或 path update；
9. 修改 manager 或控制链；
10. 让裸 `test_gvf.launch` 默认启用 tube；
11. 把 ribbon 当作碰撞证书；
12. 修复 A5 tracking blocker；
13. 进入 A6；
14. commit、tag 或 push。

## 18. 停止条件

出现以下任一情况立即停止：

- 需要白名单外修改；
- 原 simulator RViz 无法通过最小 display patch 接入；
- RViz 显示需要改变 tube 几何；
- valid profile 无法生成完整 ribbon；
- 验收需要修改 tracking/C2/control；
- 会连接或终止用户进程；
- 出现 A5 新回归失败。

## 19. 最终汇报

必须包含：

1. 实际修改的 3 个文件；
2. 原 `swarm_rviz.rviz` 中四个 A5 displays 和 topics；
3. 明确说明没有新增 config、没有自动启动额外 RViz；
4. IDs 0/1/2、类型、颜色和 DELETE 结果；
5. fixed 与 ESDF 的实际可视化表现；
6. 用户分离式启动使用的完整命令；
7. build/test 结果；
8. `/position_cmd` publisher 未变；
9. adapter/manager 行数；
10. dependency、白名单、Git、stash 审核；
11. 明确写出：
    `裸启动 test_gvf.launch 时 tube_source=none，不显示 tube 是预期行为。`
12. 明确写出：
    `本轮只修复原 simulator RViz 的 tube 可视化，未修改 A5 安全/控制逻辑。`
13. 明确写出：
    `A5 仍因 tracking-bound stop condition 未完成。`
14. 最后一行：
    `A5.1 完成后已停止，未进入 A6。`
