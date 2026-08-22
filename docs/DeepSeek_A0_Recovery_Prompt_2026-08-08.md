# 给 DeepSeek 的 A0 恢复提示词

将下面横线之间的内容完整复制给 DeepSeek。

---

你现在只执行 PhaseOffsetSwarm 实施计划中的 **A0：恢复并验证原单机基线**。禁止自动进入 A1，禁止实现任何 phase-offset 或多机功能。

工作区：`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws`

开始前完整阅读：

- `docs/PhaseOffsetSwarm_Single_First_Implementation_Plan_2026-08-07.md`
- `docs/PhaseOffsetSwarm_Code_Architecture_2026-08-08.md`

## 唯一目标

将此前对已有文件的 phase-offset/多机修改可逆隔离，恢复原单机源码和构建链；保留用户原有的两个 circle 开关；重新编译并验证原 `simulator.launch + test_gvf.launch`。

可信基线：

- branch：`main`
- commit：`9a0e975 feat: integrate unified phase GVF navigation`

## 1. 执行前检查

运行并记录：

```bash
cd /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
git branch --show-current
git rev-parse --short HEAD
git status --short
git diff --stat
git diff --name-only
git stash list
```

若 branch 不是 `main` 或 HEAD 不是 `9a0e975`，立即停止，不自行切换或重置。

## 2. 可逆隔离 tracked 修改

只 stash tracked 修改，不包含 untracked 文件：

```bash
git stash push -m "deepseek-phaseoffset-tracked-prototype-2026-08-08"
git stash list
git stash show --stat stash@{0}
git status --short
```

禁止给 stash 命令增加 `-u`。确认新源码、测试、地图、配置和文档仍作为 untracked 文件保留。

禁止使用：

- `git reset --hard`
- `git checkout -- .`
- `git restore .`
- `git clean`
- `git stash pop`

## 3. 只重新应用用户修改

只修改 `src/swarm_planner/bspline_traj/launch/test_gvf.launch` 中：

```xml
<param name="gvf/circle_test/enable" value="false" />
<param name="gvf/circle_test/auto_start" value="false" />
```

除此之外，本轮不得修改任何 tracked 源码、CMake、package.xml 或 launch。

修改后运行：

```bash
git diff --name-only
git diff -- src/swarm_planner/bspline_traj/launch/test_gvf.launch
git diff --check
```

预期 tracked diff 只能是 `test_gvf.launch` 的两个参数值。

## 4. 确认新原型没有进入基线构建

检查以下文件恢复为基线，不再引用此次新增的 phase-offset、tube、neighbor、CBF 和 multi simulator target：

- `src/swarm_planner/bspline_traj/CMakeLists.txt`
- `src/swarm_planner/common_msgs/CMakeLists.txt`
- `src/uav_simulator/so3_quadrotor_simulator/CMakeLists.txt`
- 原 `test_gvf.launch`
- 原 `simulator.launch`

不要删除任何 untracked 文件。它们只是暂时不参与构建。

## 5. 干净编译

```bash
source /opt/ros/noetic/setup.bash
cd /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws
catkin_make clean
catkin_make -j8
source devel/setup.bash
```

编译失败时只定位基线问题，不接回 phase-offset 文件，不修改算法绕过错误，不处理无关历史缺陷。

## 6. 基线测试

```bash
catkin_make run_tests_bspline_race
catkin_test_results --verbose
```

只汇报 `bspline_race` 测试。已知的无关 `uav_utils-test` 问题不要修改。

## 7. ROS 原单机验证

终端 1：

```bash
source /opt/ros/noetic/setup.bash
source /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/devel/setup.bash
roslaunch so3_quadrotor_simulator simulator.launch
```

终端 2：

```bash
source /opt/ros/noetic/setup.bash
source /home/cxq/ISF-GVF/New_ISFGVF/gvf_ws/devel/setup.bash
roslaunch bspline_race test_gvf.launch
```

验证：

1. 地图仍是原 `map_generator/resource/pillar.pcd`；
2. `/sim/odom` 约 100 Hz；
3. `/sim/local_map` 正常；
4. formation_planning、SDF、A*/B 样条和 SO3 节点存活；
5. 不启动 multi simulator；
6. 不启动 PhaseOffsetSwarm 新 timer、邻居通信和机间 CBF；
7. circle 开关为 false，不自动进入闭合轨迹；
8. 用户在 RViz 手工发布目标后，原单机能规划、避障并到达。

不要改变地图、初始位置和代码来自动生成目标。如果无法与用户交互，完成启动、节点和 topic 检查后，明确等待用户在 RViz 点击目标；收到目标后再记录结果。

## 8. 最终自审核

运行：

```bash
git status --short
git diff --name-only
git diff --check
rosnode list
rostopic list
```

结束测试时清理本轮启动的 ROS 进程。若启动前发现已有 roscore，停止并询问，不误连用户现有 master。

必须确认：

- tracked 修改只有两个 circle 参数；
- untracked DeepSeek 原型仍保留；
- 命名 stash 存在；
- 原 CMake 没有编译新模块；
- 没有修改 K1/K2、最大速度、C2、SDF、A* 或 simulator；
- 没有创建 `phase_offset_core`；
- 没有进入 A1。

## 9. 禁止事项

本轮禁止：

1. 新建任何 phase-offset package；
2. 把 untracked 新文件接入 CMake；
3. 修改 `gvf_manager`、`gvf`、SDF、A*、B 样条和 C2 connector；
4. 修改单机 simulator 源码；
5. 修改 K1、K2、最大速度或规划速度；
6. 加入 shadow、active、tube、allocator、CBF 或 neighbor；
7. 删除、移动或提交现有原型；
8. 创建 Git commit；
9. 自动继续 A1；
10. 大规模格式化或修改无关代码。

## 10. 最终汇报格式

完成后只汇报：

1. 开始时 branch、HEAD 和状态摘要；
2. stash 名称与编号；
3. stash 保存的文件统计；
4. 回退后 tracked 修改；
5. 两个 circle 参数最终值；
6. 编译命令和结果；
7. `bspline_race` 测试结果；
8. ROS 节点、topic 和地图检查；
9. 单机点到点结果，或等待用户点击目标的状态；
10. 最终 `git status --short`；
11. 自审核发现的问题；
12. 明确写出：`A0 完成后已停止，未进入 A1`。

任何一步失败时保留现场并汇报，不扩大范围“顺便修复”。

---
