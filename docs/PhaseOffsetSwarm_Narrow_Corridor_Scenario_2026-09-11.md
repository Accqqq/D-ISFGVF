# 狭窄通道场景（1.0 m 走廊）

日期 2026-09-11。状态：已执行并实机验证（3 机与 15 机）。

## 1. 地图

| 文件 | 几何（实测） |
|---|---|
| **`phase_offset_corridor_single_wide.pcd`** | x∈[-10, 28]（长 38 m），墙板 y∈[-1,-0.5] 与 [0.5,1] → **通道净宽 1.0 m**，高 3 m |
| `phase_offset_corridor_two_wide.pcd` | 同上，墙板 y∈[-1.5,-1] 与 [1,1.5] → 净宽 2.0 m |

两张都由 `bspline_traj/scripts/generate_phase_offset_maps.py` 生成（脚本头注明
`corridor_two_wide = 2.0 m`、`corridor_single_wide = 1.0 m`），改宽度重生成即可。
（上游那些 `narrow.pcd` / `door*.pcd` 是"墙缝/门洞"，`corridor.pcd` 其实是 20 m 宽，
都不是这条通道。）

## 2. 队形：line（沿飞行轴的纵队）

1.0 m 通道放不下圆盘，所以 `circular_formation_offsets()` 新增第三种形状：

```
line : N 台沿飞行轴（formation_axis）排成一列，间距 = formation_spacing
```

配套新增 `formation_axis`（`x|y`，默认 y）与 `formation_goal_translation_x`
（沿 x 飞时的目标平移）。走廊沿 **x**，所以用 `formation_axis:=x`；此时目标只沿 x
平移，y 保持编队原值。

## 3. 运行

地图只传给 world（agent 的 SDF 从 `/mock_map` 订阅）。注意目录名是
`dynamic_map_generator`，但 **ROS 包名是 `map_generator`**，所以要用
`$(rospack find map_generator)`（并确保当前 shell 已 `source devel/setup.bash`，
否则 rospack 连工作空间里的包都找不到）：

```bash
MAP=$(rospack find map_generator)/resource/phase_offset_corridor_single_wide.pcd
# 或者直接用绝对路径：
# MAP=$HOME/ISF-GVF/New_ISFGVF/gvf_ws/src/uav_simulator/dynamic_map_generator/resource/phase_offset_corridor_single_wide.pcd
roslaunch phase_offset_sim_bringup phase_offset_world.launch map_file:=$MAP

# 3 机验证
roslaunch phase_offset_sim_bringup phase_offset_swarm.launch \
    agent_count:=3 formation_shape:=line formation_axis:=x \
    formation_spacing:=1.5 formation_goal_translation_x:=12 \
    init_y:=0 map_size_x:=30 publish_goals:=true

# 15 机纵队（间距 1.2 m）
roslaunch phase_offset_sim_bringup phase_offset_swarm.launch \
    agent_count:=15 formation_shape:=line formation_axis:=x \
    formation_spacing:=1.2 formation_goal_translation_x:=12 \
    init_x:=1.0 init_y:=0 map_size_x:=50 publish_goals:=true
```

要点：`init_y` 必须是 0（默认 20 会把编队放到走廊外面），`map_size_x` 要覆盖
纵队长度 + 飞行距离（20×50 的默认地图只有 x∈[-10,10]）。

## 4. 验收

| 场景 | 结果 |
|---|---|
| 3 机、间距 1.5 m、飞行 12 m | 3/3 `REACHED` + `WAIT_TARGET`；横向偏差最大 **0.023 m**；相位/slew 拒绝 0 |
| 15 机、间距 1.2 m、飞行 12 m | **15/15** `REACHED` + `WAIT_TARGET`；到点后最大 \|y\| = **0.20 m**（通道半宽 0.5 m），纵队间距 1.2 m；相位/slew 拒绝 0 |

也就是说：通道把集群意图横向完全夹住（1 m 宽只剩 ±0.5 m，膨胀后约 ±0.4 m），
15 台只能保持纵向队列通过——正是想要看到的"意图被通道约束"的效果。

## 5. 踩到的坑（下次直接用）

纵队**必须整列落在墙的 x 范围内**（这条走廊是 x∈[-10,28]）。第一次 15 机跑时纵队
尾部一台 init x=-10.5，落在入口外侧；它从墙外侧绕行后无法再穿墙进入走廊，卡在
y=1.45 的墙外（`no_valid_candidate`）。把编队整体移到 `init_x:=1.0`（纵队
x∈[-7.4, 9.4]，全在墙内）后 15/15 全部到点。
