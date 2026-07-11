# 闭合轨迹绕障候选选择设计

## 背景与问题

闭合轨迹模式会为多个前视距离调用 KinoA*。当前实现将 KinoA* 实际终点到各自目标点的距离 `end_to_goal_dist` 作为 partial 路径的主要比较量。不同候选的目标点不同，因此这个距离不能横向表示哪条路径沿闭合轨迹推进得更远。靠近当前位置的短目标更容易得到较小的终点误差，可能导致系统接受仅有 `0.2–0.5 m` 的短路径；路径很快耗尽后，governor 进入无效保持，表现为无人机在障碍物前停顿。

现有的障碍推远逻辑只找到参考轨迹上的第一个占用点，并把期望前视距离推到“障碍起点 + 余量”。它没有确定障碍物在参考轨迹方向上的结束位置，也没有检查 KinoA* 的实际终点是否已经越过障碍物。

## 目标

- 正常 TRACK 状态继续优先短前视距离，避免点到点 KinoA* 对闭合轨迹抄近道。
- 检测到参考轨迹前方障碍时，使用实际规划结果判断候选是否已经越过障碍。
- 优先选择刚好越过障碍的候选；没有候选越障时，选择实际推进最远的有效 partial。
- 不增加参考轨迹偏离代价。
- 不拒绝 partial 路径。
- 不改变碰撞检测、定时重规划、半程强制接受、governor 或新旧轨迹切换策略。

## 非目标

- 不修改 KinoA* 的搜索代价或启发函数。
- 不要求规划路径贴合闭合参考轨迹。
- 不扩大正常状态的全局前视距离。
- 不引入新的飞行安全门或路径拒绝条件。

## 方案选择

### 方案一：根据候选目标前视距离判断

实现最简单，但远目标的 KinoA* 可能只返回障碍前的 partial。目标在障碍后不等于实际路径已越障，因此不采用。

### 方案二：根据实际终点到目标点的欧氏距离判断

这是当前行为。由于不同候选目标不同，误差不可直接比较，短目标天然占优，因此不采用。

### 方案三：根据实际终点在闭合参考轨迹上的前向相位判断

先确定参考轨迹上的障碍结束相位，再把每条 KinoA* 的实际终点投影到当前相位前方的局部区间。投影相位只用于判断候选在障碍前还是障碍后，不进入 KinoA* 代价。该方案能同时保留闭合轨迹方向约束和点到点绕障自由度，因此采用。

## 详细设计

### 1. 障碍区间检测

仅在 `goal_push_past_obstacle=true`、SDF 地图可用且存在前视候选时启用。

从当前 `closed_ref_w_` 开始，以 `goal_obstacle_check_step_w` 沿闭合参考轨迹向前扫描到最大候选前视距离：

1. 第一个膨胀占用采样点记为 `obstacle_start_delta_w`。
2. 发现障碍后继续扫描。
3. 连续 3 个在地图内且无膨胀占用的采样点，表示离开障碍区间。
4. 连续自由段的第一个采样点记为 `obstacle_end_delta_w`。
5. 越障门槛为：

   ```text
   bypass_delta_w = obstacle_end_delta_w + goal_obstacle_pass_margin_w
   ```

6. `bypass_delta_w` 限制在最大候选前视距离内。

地图外采样不作为自由证据，也不打断已经累计的地图内连续自由计数。若扫描窗口内只发现障碍起点但找不到障碍结束，则本轮不启用“已经越障”判断，继续保留正常候选和 partial fallback，避免因不完整地图拒绝所有结果。

### 2. 实际终点的局部前向投影

对每条 `planKinoToGoal()` 返回的有效候选，取：

```cpp
candidate_samples.point_set.back()
```

作为实际规划终点。调用现有局部投影能力，在以下区间寻找闭合参考轨迹上的最近点：

```text
[closed_ref_w_, closed_ref_w_ + lookahead_max_w]
```

不允许向后投影，也不进行整条闭合轨迹全局投影。得到：

```text
candidate_end_delta_w = projected_end_w - closed_ref_w_
```

前向局部窗口能够避免 8 字轨迹交叉处把终点投影到另一圈或后向分支。

### 3. 越障判定

当本轮成功确定 `bypass_delta_w` 时：

```text
passed_obstacle = candidate_end_delta_w >= bypass_delta_w - tolerance
```

容差使用一个障碍检测采样步长：

```text
tolerance = goal_obstacle_check_step_w
```

这样不新增独立调参项，同时覆盖离散扫描和局部投影的量化误差。

### 4. 候选选择规则

正常模式保持现有 full-success 打分和短期望前视行为。

绕障模式下，对所有有效 KinoA* 结果统一应用以下顺序，不因结果是 full 或 partial 而改变“是否越障”的第一优先级：

1. 已越过 `bypass_delta_w` 的候选优先于未越过候选。
2. 多个候选均越障时，选择 `candidate_end_delta_w` 最小的候选，即刚刚越过障碍，减少远目标抄近路。
3. 没有候选越障时，选择 `candidate_end_delta_w` 最大的候选，即实际推进最远的有效结果。
4. 实际推进量相同时，优先 `end_to_goal_dist` 更小者。
5. 仍相同时，优先目标前视距离更小者，保证结果稳定。

任何有效 partial 都可以成为 fallback；本设计不新增拒绝条件。

### 5. 状态与切换

绕障选择仅在本次 `selectClosedGoalCandidate()` 调用内生效，不引入跨周期 BYPASS 状态机。每次重规划都根据最新地图重新扫描：

- 前方没有完整障碍区间时，使用正常 TRACK 选择。
- 前方存在完整障碍区间时，使用绕障排序。
- 无人机越过障碍后，前向扫描不再看到该障碍，自动恢复正常短前视行为。

该设计不修改 `shouldAcceptCandidate()`。旧轨迹碰撞、旧轨迹半程、短轨迹耗尽等现有强制接受语义保持不变。

## 可测试接口

在 `gvf_manager.h` 中增加一个只包含标量的候选描述和静态比较函数，供选择代码与 gtest 共用：

```cpp
struct ClosedGoalCandidateProgress {
    bool valid;
    bool passed_obstacle;
    double end_delta_w;
    double end_to_goal_dist;
    double lookahead;
};

static bool preferClosedGoalBypassCandidate(
    const ClosedGoalCandidateProgress& lhs,
    const ClosedGoalCandidateProgress& rhs,
    bool bypass_mode);
```

比较函数不依赖 ROS、地图或规划器，便于覆盖所有排序边界。

## 日志

扩展 `[GVF][CLOSED_GOAL]` 日志，至少包含：

- `obstacle_start_delta_w`
- `obstacle_end_delta_w`
- `bypass_delta_w`
- `bypass_mode`
- `selected_end_delta_w`
- `selected_passed_obstacle`
- 每个有效候选的实际 `end_delta_w`

保留现有目标前视距离、终点误差、full/partial 等字段，便于用 rosbag 对照判断停顿是否消失。

## 测试与验收

### 单元测试

- 越障候选优先于推进更远但未越障的候选。
- 多个候选均越障时选择刚越障者。
- 都未越障时选择实际推进最远者。
- 推进量相同时使用终点误差和前视距离稳定打破平局。
- 非绕障模式不改变现有 full-success 分数选择。

### 构建验证

运行 `bspline_race` 包的测试，要求全部通过并无编译错误。

### 仿真验收

使用 `test_gvf.launch` 和相同障碍布局复测：

- `[GVF][CLOSED_GOAL]` 显示绕障时优先选择实际终点越过 `bypass_delta_w` 的路径。
- 若没有候选越障，选择的 partial 具有本轮最大的 `end_delta_w`，不再因近目标误差较小选择 `0.2–0.5 m` 短路径。
- 越过障碍后恢复 `goal_prefer_lookahead_w` 附近的短前视选择。
- 闭合轨迹方向保持，且不因全局远目标产生明显抄近路。
