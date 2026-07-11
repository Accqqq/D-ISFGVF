# 闭合轨迹渐进式候选选择设计

## 背景

现有闭合轨迹绕障逻辑存在两个已由 rosbag 和 ROS 日志确认的问题：

1. 当障碍物只影响实际 B 样条边缘、但闭合参考线采样点仍为空闲时，`bypass_mode` 不会启用。KinoA* 在膨胀障碍边界附近只能返回短 partial，而正常 fallback 使用 `end_to_goal_dist` 比较不同目标的 partial，近目标天然占优，可能连续发布 `0.15–0.35 m` 的短轨迹。governor 很快耗尽轨迹，表现为障碍物前明显停顿。
2. 当参考线障碍区间较长时，现有逻辑把期望目标直接推到“障碍末端 + 0.8 m”，并可夹到 `3.0 m` 最大前视。日志已记录 `goal_dist_xy=4.091 m`、最终 B 样条 `path_w_len=5.778 m` 的轨迹被接受发布，表现为瞬时超长轨迹。

## 目标

- 正常 TRACK 状态继续保持约 `1.0 m` 的短前视，避免闭合轨迹抄近路。
- 障碍边缘未触发参考线占用判断时，仍能按 KinoA* 的实际前进能力选择 partial。
- 避免为了单次越过整个障碍而直接选择 `3.0 m` 目标和 `5–6 m` 绕行路径。
- 采用连续重规划逐段绕障，使新轨迹在旧轨迹耗尽前提供足够前进量。
- 不增加参考轨迹偏离代价。
- 不拒绝 partial 路径。
- 不修改点到点导航、碰撞检测、定时重规划、governor 或新旧轨迹切换策略。

## 非目标

- 不修改 KinoA* 搜索代价、启发函数或地图膨胀逻辑。
- 本轮不修复 KinoA* `REACH_HORIZON` 状态丢失和搜索 margin 持久降低问题。
- 本轮不增加参考线横向走廊扫描；实际候选排序不再依赖参考线障碍检测是否成功，因此先解决已确认的主因。
- 不要求单条轨迹一次越过完整障碍区间。

## 方案比较

### 方案一：仅把 partial fallback 改为实际前进最大

可以缓解短轨迹耗尽，但保留“障碍末端 + 余量”的强推后，超长轨迹仍会出现，因此不采用。

### 方案二：渐进式候选窗口与统一实际进度排序

取消障碍末端对目标前视的直接强推。TRACK 状态只在受限的渐进窗口内选择候选，并根据实际前进量和 Kino 几何长度统一比较 full 与 partial。该方案同时覆盖参考线漏检、短 partial 和超长绕行，因此采用。

### 方案三：增加参考线横向走廊和多障碍区间扫描

能提高障碍检测完整性，但对地图噪声更敏感，并且不能单独修复不同目标 partial 不可比的问题。作为后续增强，不纳入本轮。

## 详细设计

### 1. 候选度量

为每个有效 KinoA* 候选计算并保存：

```text
lookahead_w       候选目标前视距离
end_delta_w       实际 Kino 终点投影得到的闭合参考前向进度
kino_path_length  Kino 采样点折线总长度
end_to_goal_dist  实际终点到该候选目标的距离
full_success      end_to_goal_dist 是否小于现有 full-success 容差
```

`end_delta_w` 继续使用现有前向局部投影 `[closed_ref_w_, closed_ref_w_ + lookahead_max_w]`，不允许向后或全局投影。`kino_path_length` 仅用于候选之间选择，不进入 KinoA* 代价函数。

### 2. 渐进候选窗口

TRACK 状态使用：

```text
progressive_max_lookahead_w = min(lookahead_max_w, goal_prefer_lookahead_w + 0.75 m)
```

当前参数下即 `min(3.0, 1.0 + 0.75) = 1.75 m`。只比较 `0.5–1.75 m` 范围内的候选，避免单周期直接跳到 `3.0 m`。

RECOVER 状态保留完整 `lookahead_min_w–lookahead_max_w` 范围，避免严重偏离闭合参考时恢复能力不足。

新增参数：

```text
gvf/circle_test/progressive_lookahead_extra_w = 0.75
```

### 3. 最低有效前进量

根据实际水平速度计算本轮希望获得的最低前进量：

```text
required_progress_w = clamp(odom_speed_xy * 1.0 s, 0.6 m, 0.8 m)
```

实际速度约 `0.8 m/s` 时，要求候选提供约 `0.8 m` 的实际前进量，可覆盖约两个 `0.5 s` 重规划周期。飞机已经减速时下限保持 `0.6 m`，避免阈值随停车降到零而再次选中极短路径。

新增参数：

```text
gvf/circle_test/progressive_min_progress_w = 0.6
gvf/circle_test/progressive_max_progress_w = 0.8
gvf/circle_test/progressive_progress_time = 1.0
```

### 4. 统一选择规则

正常 TRACK 和障碍边缘漏检场景使用同一规则，不再以 `bypass_mode` 决定是否按实际进度比较。

先筛选 `end_delta_w >= required_progress_w` 的“进度充足”候选：

1. 前视距离更小者优先，确保逐段推进。
2. 前视距离相同时，`kino_path_length` 更短者优先。
3. 再以 `end_to_goal_dist` 和候选索引稳定打破平局。

如果没有候选达到最低进度：

1. `end_delta_w` 最大者优先，选择本轮实际能推进最远的 full 或 partial。
2. 进度相同时，`kino_path_length` 更短者优先。
3. 再以较小前视和较小目标误差稳定打破平局。

该 fallback 不拒绝任何有限、有效的 partial；如果所有候选确实只能前进很短，系统仍保留当前安全减速行为。

### 5. 障碍推远逻辑

保留参考线障碍扫描和相关日志，用于说明前方是否存在障碍区间，但不再执行：

```text
desired_lookahead = obstacle_end_delta_w + pass_margin
```

`passed_obstacle` 继续记录为诊断字段，但不再拥有最高选择优先级。绕障依靠每次重规划重新计算候选并逐段前进，而不是要求一次跨过整个障碍。

现有 `goal_push_past_obstacle` 参数保留，避免 launch 接口突然失效；它只控制障碍扫描和诊断，不直接改变候选目标。

### 6. 作用域

修改限制在闭合轨迹候选生成与纯标量比较 helper：

- `gvf_manager::selectClosedGoalCandidate()`
- `gvf_manager` 中闭合候选比较结构和静态函数
- `test_gvf.launch` 中新增渐进选择参数
- `gvf_switch_policy_test.cpp` 中新增单元测试

以下代码保持不变：

- `shouldAcceptCandidate()`
- `checkCollision()`
- FSM 定时/碰撞重规划触发
- governor 命令生成
- 点到点 `astaropt` 目标选择

## 日志

扩展 `[GVF][CLOSED_GOAL]`，增加：

```text
selection_mode
progressive_max_lookahead_w
required_progress_w
selected_kino_path_length
selected_progress_sufficient
tried_kino_path_lengths
```

保留现有障碍区间、实际终点进度、full/partial 和目标误差字段，便于继续用 rosbag 对照。

## 测试

### 单元测试

- `bypass_mode=false` 且所有结果为 partial 时，`0.8 m` 实际进度候选优先于 `0.15 m` 近目标候选。
- 进度均充足时，较小前视优先，防止 `3.0 m` 候选压过 `1.5 m` 渐进候选。
- 前视相同时，较短 Kino 几何路径优先。
- 无候选达到最低进度时，选择实际进度最大者。
- TRACK 只允许渐进窗口；RECOVER 保留完整候选窗口。
- 非有限进度或路径长度不能成为有效候选。

### 构建验证

```bash
catkin_make --pkg bspline_race --make-args gvf_switch_policy_test
./devel/lib/bspline_race/gvf_switch_policy_test
catkin_make --pkg bspline_race
```

### 仿真验收

- 障碍边缘场景不再连续发布 `0.15–0.35 m` 的近目标 partial；若存在可前进候选，应选择至少约 `0.6–0.8 m` 的实际进度。
- TRACK 绕障目标不再单周期跳到 `3.0 m`。
- 不再出现由障碍末端强推导致的 `4–6 m` 瞬时轨迹。
- 正常无遮挡时继续选择 `goal_prefer_lookahead_w≈1.0 m`。
- 点到点导航、碰撞重规划与现有换轨理由保持不变。

