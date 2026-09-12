# 长隧道地图（墙里一条可参数化的窄隧道）

日期 2026-09-11。状态：已执行；1.2 m 版本通过，1.0 m 版本暴露规划器上限。

## 1. 为什么要自己生成

上游 `narrow.pcd` 的"通道"只是 0.34 m 厚的一道墙缝，谈不上"长"。要"更长、更窄"
就得自己造：墙铺满整张地图宽度，中间开一个**长 L、宽 w 的矩形隧道**，除了隧道
无路可走。

## 2. 生成器

`src/swarm_planner/bspline_traj/scripts/generate_phase_offset_tunnel_maps.py`

```bash
python3 src/swarm_planner/bspline_traj/scripts/generate_phase_offset_tunnel_maps.py \
        --outdir src/uav_simulator/dynamic_map_generator/resource
```

已在 `resource/` 生成（墙 x∈[−15,15] 铺满地图、z 0…3 m）：

| 文件 | 隧道宽 | 隧道长 |
|---|---|---|
| `phase_offset_tunnel_w12_l6.pcd` | 1.2 m | 6 m（25 544 点） |
| `phase_offset_tunnel_w10_l10.pcd` | 1.0 m | 10 m（30 628 点） |

改 `variants` 里的 `(文件名, 宽, 长)` 即可生成任意组合。隧道内部已校验无点
（`points inside the tunnel: 0`）。

> **采样密度必须 ≥ 地图分辨率**（2026-09-11 修正）。最初的生成器用
> `step=0.2 m`、`zstep=0.5 m`：地图 0.1 m 分辨率下，一个体素只有落进点才算占用，
> 而 `obstacles_inflation=0.099` 只膨胀 1 个体素，于是墙面在 z 方向每 0.5 m 留一条
> **0.3 m 高的水平缝**（z ∈ (0.1,0.4)、(0.6,0.9)、(1.1,1.4)…），飞机高度一偏就
> 从墙里穿过去。现在两张隧道图和两张走廊图都改成 **0.1 m × 0.1 m** 采样：
>
> | 地图 | 修正前点数 | 修正后 | 水密性检测（可穿过的列数） |
> |---|---|---|---|
> | tunnel_w12_l6 | 2 912 | 25 544 | 626 → **0** |
> | tunnel_w10_l10 | 3 472 | 30 628 | 606 → **0** |
> | corridor_single/two_wide | 5 348 | 47 244 | 576 → **0** |
>
> 检测口径：0.1 m 体素化 + 1 体素膨胀后，在墙体 x 范围内（排除隧道本身
> |x| ≤ w/2）沿 y 逐列扫描，统计"整列无占用"的位置。

## 3. 运行（起点/终点都要整块落在墙外）

墙沿 y 占 `[−L/2, +L/2]`，所以编队圆盘（半径 3.6 m）必须整体在墙外：

```bash
MAP=.../phase_offset_tunnel_w12_l6.pcd
roslaunch phase_offset_sim_bringup phase_offset_world.launch map_file:=$MAP

roslaunch phase_offset_sim_bringup phase_offset_swarm.launch \
    agent_count:=15 formation_shape:=disk formation_spacing:=1.5 \
    init_y:=8.0 formation_goal_translation_y:=-16.0 \
    local_update_range_x:=7.0 local_update_range_y:=7.0 \
    publish_goals:=true
```

要点：

- `init_y` 必须 ≥ 墙半长 + 圆盘半径（L=6 → ≥ 6.6；L=10 → ≥ 8.6），否则**部分目标
  会落在墙体内部**，那几台永远到不了（我第一次就踩了这个坑）。
- `local_update_range_*` 现已可从 swarm launch 传下去（默认 4.0）。SPH-planning 用
  的是 7.0；单开口地图建议放大，否则贴着墙、离口较远的机器看不到开口。

## 4. 结果

| 隧道 | 结果 |
|---|---|
| **1.2 m × 6 m** | **15/15** `REACHED` + `WAIT_TARGET`，0 拒绝；过隧道时（同时 6 台在洞内）横向散布 **0.79 m**；tube 半宽最小降到 0.15–0.25 m（开阔段 1.45 m），即 preview 把横向意图压到洞宽 |
| 1.0 m × 10 m | 12/15 通过；**3 台卡在墙北面 x≈±7 处**（见下） |

加密后的复验（1.2 m × 6 m，逐 2 s 采样整段飞行）：**没有任何一台出现在墙体内部**
（判据 `|y| ≤ 3 且 |x| > 0.75`，共 14 次采样、0 次命中），全部经由隧道通过并到点。

## 5. 1.0 m 版本为什么卡：是规划器上限，不是 tube

卡住三台的日志特征：

```
[GVF][COLL] ... 561 次      # 自己规划出的轨迹被判碰撞
reason=near_end 550 次
[POINT_PHASE_V2] replan not installed 277 次
→ final_cmd_source=GOVERNOR_INVALID_HOLD fallback_reason=no_valid_candidate
```

原因链：`sdf_map/obstacles_inflation = 0.099 m` 在 0.1 m 分辨率下占 1 个体素 →
1.0 m 隧道实际只剩 **0.8 m** 自由宽度；而 B 样条优化器的
`planning/safe_distance = 0.4` 要求离两边各 0.4 m，正好把 0.8 m 通道卡死，优化后
的轨迹一膨胀就判碰撞 → 新前端装不进去 → 无候选 hold。

要真正跑到 1.0 m 或更窄，需要动这两个参数之一（目前都写死在 launch 里，没有做成
arg）：

- `planning/safe_distance`：0.4 → 0.25 左右（规划器的避障偏好，不影响 tube 的
  `section_clearance`）；
- 或把 `sdf_map/resolution` 降到 0.05 m（`obstacles_inflation` 随之减半到 0.05 m），
  代价是地图内存/CPU 翻几倍。

tube/preview 一侧没有问题：它在洞口已经能报出 0.05–0.25 m 的横截面半宽，压缩行为
正常。需要的话我把这两个参数也接成 launch arg，再跑 1.0 m / 0.8 m 版本。

## 6. 当前决定（2026-09-11）

先用 **1.2 m × 6 m** 这条隧道做实验，**不改 `planning/safe_distance`**：

- `planning/safe_distance` 保持 **0.4**（`test_gvf.launch` 第 411 行，本批未改动，
  `git diff` 中没有任何修改其数值的行）；
- tube 自己的走廊壁 clearance 是独立参数 `phase_offset_tube_section_clearance = 0.15`，
  也只作用于 tube 截面，与规划器避障偏好无关；
- 1.0 m / 0.8 m 的隧道暂不追求（那是规划器优化器在 0.8 m 自由宽度下的上限问题）。
