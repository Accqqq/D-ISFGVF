# 单开口闸门场景（外围散开 → 唯一通道 → preview 压凝聚）

日期 2026-09-11。状态：已执行并实机验证（15 机）。

## 1. 地图：`narrow.pcd`

实测几何：

| 量 | 值 |
|---|---|
| 墙沿 x | −15.01 … 15.62（30.6 m，**覆盖整张 20 m 宽的地图**） |
| 墙厚（y） | 0.34 m（y∈[−0.16, 0.18]） |
| 墙高（z） | −0.5 … 5.5（覆盖飞行高度） |
| **唯一开口** | x ≈ [−1.15, 0.83]，**净宽 2.0 m** |

因为墙在 x 方向铺满整张地图（`map_size_x=20` → x∈[−10,10]），**除了这个 2 m 的口，
南北之间没有别的通路** —— 正是"只有一个通道过去"。

（对比：`door1/2/3.pcd` 是"一排多扇门"的墙，`phase_offset_corridor_*.pcd` 是整条
长廊、起点必须在长廊内，都不符合这个用法。）

## 2. 场景：外围散开 → 穿口 → 南侧重新聚拢

用默认的 `disk` 编队（15 台、间距 1.5 m、半径 3.62 m）：

- 起点：编队质心在墙**北侧** `(0, +5)` → x∈[−3.6,3.6]、y∈[1.4,8.6]，完全散开；
- 终点：同一编队整体南移 10 m → 质心 `(0, −5)`，y∈[−8.6,−1.4]，全在墙南侧；
- 于是 15 台必须从 7.2 m 宽的散布收拢到 2 m 的口里排队通过，再在南侧展开。

```bash
MAP=$HOME/ISF-GVF/New_ISFGVF/gvf_ws/src/uav_simulator/dynamic_map_generator/resource/narrow.pcd
roslaunch phase_offset_sim_bringup phase_offset_world.launch map_file:=$MAP

roslaunch phase_offset_sim_bringup phase_offset_swarm.launch \
    agent_count:=15 formation_shape:=disk formation_spacing:=1.5 \
    init_y:=5.0 formation_goal_translation_y:=-10.0 \
    publish_goals:=true
```

（`init_x` 默认 0；地图沿用 20 × 50，z 2.5。）

## 3. 验收

| 指标 | 结果 |
|---|---|
| `[REACHED]` / `EXEC_TRAJ → WAIT_TARGET` | **15 / 15** |
| 相位窗口 / slew 拒绝 | 0 |
| 通过口时的横向散布 | 采样到 3 台同时在口内，x∈[−0.02, 1.21] → **散布 1.23 m ≤ 2.0 m 口宽** |
| 出口后 | 终点重新聚成圆盘：y∈[−8.0,−2.0]，x∈[−3.0,3.0] |
| preview 的走廊横截面 | 全程最小半宽低至 **0.05 / 0.15 / 0.21 / 0.23 m**（开阔段是 1.45 m） |

最后一行就是"用 preview 降低集群凝聚"的直接证据：过口前 preview 看到的走廊横截面
已经收窄到 0.2 m 量级（2 m 口 − 膨胀 − clearance），分配器只能把横向意图 δ 裁到这个
边界，编队被迫收拢成队列；过口后横截面恢复到 1.45 m，编队重新展开。

## 4. 想更狠一点

2 m 口对 15 台来说还不算极限。真要压到 1.0–1.2 m，用
`bspline_traj/scripts/generate_phase_offset_maps.py` 的方式生成一张"墙 + 单口"的
PCD（改口宽即可），其余命令不变。需要的话我来加这个生成器。
