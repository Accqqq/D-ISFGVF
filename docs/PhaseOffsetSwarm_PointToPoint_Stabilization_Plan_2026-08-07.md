# PhaseOffsetSwarm 三机点到点稳定化计划

> 日期：2026-08-07  
> 当前唯一目标：只使用原工程的 `pillar.pcd`，让三架 UAV 从地图左侧稳定飞到右侧，完成各自局部规划、静态避障、分布式集群组织和 matched port 闭环。完成以前不再继续狭窄通道、闭合轨迹和增量换路。

## 1. 当前判断

现有批次 2–10 代码可以保留作为实验原型，但不能把当前状态认定为“全部完成”。

已有结果暴露出以下问题：

1. `open_hex` 三机点到点全功能模式出现 98 个 `EMERGENCY` 样本，最小机间距离只有 0.617 m；
2. 理论鲁棒距离约为 0.925 m，但已有 CBF 压力测试的物理最小间距只有 0.69 m，说明低层跟踪与 governor 滞后尚未被正确闭合；
3. 双宽走廊最小间距 0.137 m，并出现 94 个小于 0.6 m 的采样，不能称为安全通过；
4. `batch10_scenarios.py` 的场景成功条件主要是节点启动和 odom 存在，固定等待后就返回成功，没有严格检查每架 UAV 是否到达目标并稳定停止；
5. 当前场景目标发布器没有等待订阅连接，只发布一次，可能发生目标漏收；
6. 七机仅完成 shadow 聚合，没有完成七机 ACTIVE 点到点稳定飞行；
7. 8 字任务通过关闭重规划规避了 C2 connector 曲率问题，因此不属于完整闭环完成。

因此从现在开始执行功能冻结，不再扩展新场景。

## 2. 功能冻结范围

以下代码保留，但暂时不继续调试、不作为当前验收内容：

- 狭窄通道专用的 tube 压缩、channel beta 和 conflict event 调度；
- DeepSeek 新生成的 split-merge/open/corridor 地图；
- 单宽/双宽狭窄通道；
- circle 和 figure-eight；
- 飞行中 C2 增量重规划；
- conflict/channel event；
- 七机 ACTIVE 控制；
- ablation 和论文统计；
- 新的理论功能或消息类型。

禁止为了让点到点通过而继续增加新的控制模式、额外启发式或场景专用补丁。

当前只允许修改：

- 点到点专用 launch/YAML/test；
- goal 可靠发送与接收诊断；
- phase-offset/swarm/allocator/CBF 的通用缺陷；
- governor、velocity feedforward 和 SO3 跟踪参数；
- 点到点 TERMINAL 行为；
- 调试与统计字段。

## 3. 固定的最小系统配置

第一版只使用：

- UAV 数量：3；
- 环境：原工程 `map_generator/resource/pillar.pcd`；
- 每机独立 odom、local sensing、local map、planner 和 SO3 controller；
- 每机独立 point-to-point B 样条基础路径；
- `point_phase_v2=true`；
- `phase_offset=true`；
- `swarm=true`；
- P2P-0/P2P-1 使用固定 offset 区间；P2P-2 开启原地图上的 ESDF tube，防止横向 offset 离开安全走廊；
- CBF 作为安全保障；
- velocity feedforward 开启；
- 周期重规划关闭或设置为足够大的间隔；
- 不启用 circle/figure-eight；
- 不在飞行中改变目标；
- 不使用新生成地图、狭窄通道或闭合轨迹。

推荐初始位置：

| UAV | 起点 |
|---|---|
| 0 | (-5.0, -4.75, 1.0) |
| 1 | (-5.0, -3.50, 1.0) |
| 2 | (-5.0, -2.25, 1.0) |

推荐目标：

| UAV | 终点 |
|---|---|
| 0 | (5.0, -4.75, 1.0) |
| 1 | (5.0, -3.50, 1.0) |
| 2 | (5.0, -2.25, 1.0) |

这组任务由 `pillar.pcd` 在飞行高度的占用投影检查得到，具有以下特点：

- 起点和终点均位于膨胀障碍外；
- 相邻 UAV 间距为 1.25 m，处于 1.55 m 组织邻域内，并高于物理安全距离；
- UAV 0 与 UAV 2 不直接成为组织邻居，二者影响通过中间 UAV 逐跳传播；
- 三条基础路径总体从左向右，但会分别绕过不同柱子；
- 不要求固定队形，也不要求三架 UAV 收敛到同一个点；
- 能同时检查原局部规划器避障、弹性集群保持和 phase-offset 注入。

正常飞行速度只通过 ISF-GVF 的 `K1/K2`（当前代码中的
`gvf_gain1/gvf_gain2`）调整，不通过修改最大速度上限实现。

要求：

- `gvf/cmd/vel_max`、`gvf/cmd/tangent_vel_max` 保持原值；
- allocator 的 phase/tangent speed max 保持原值；
- 前端规划器的 `planning/max_vel` 和搜索速度上限保持原值；
- 第一轮只对 `K1/K2` 使用同一个比例系数，保持二者原有比例；
- 当前 `K1=2.0、K2=-2.2` 时，可以先用比例系数 `0.4`，即
  `K1=0.8、K2=-0.88`；
- 稳定后再逐步把比例系数提高到 `0.5、0.6 ... 1.0`。

最大速度参数仅作为异常情况下的安全饱和边界，不能作为正常速度调节器。

## 4. 必须新增的专用入口

新增以下文件，避免继续复用批次 10 的大场景脚本：

1. `launch/phase_offset_pillar_left_to_right_3.launch`
2. `config/initial_states_pillar_left_to_right_3.yaml`
3. `test/pillar_left_to_right_stability_test.py`

专用 launch 必须显式写出所有关键开关，不能依赖容易变化的默认值。
同时增加 `gvf_gain1/gvf_gain2`（或统一的 `gvf_gain_scale`）参数，并将其传给
三架 UAV 的 formation_planning；禁止在该 launch 中覆盖任何 max velocity 参数。

建议固定：

```text
num_agents=3
scenario=none
map_file=$(find map_generator)/resource/pillar.pcd
enable_phase_offset=true
enable_swarm=true
enable_tube=true
enable_cbf_safety=true
closed_shape=none
plan_interval=100.0
use_velocity_feedforward=true
gvf_gain1=0.8
gvf_gain2=-0.88
```

## 5. 目标可靠发送

这一阶段默认由用户分别向 `/uav_0/goal`、`/uav_1/goal`、`/uav_2/goal` 手工发布目标。自动测试脚本使用同一组坐标发送目标。

修正手工/自动 goal 发送流程：

1. 每个 goal publisher 必须等待对应 topic 至少有一个订阅者；
2. 连接成功后只发送一次正式目标，避免重复触发路径初始化；
3. 超时未建立连接时明确失败退出；
4. 发布后至少等待管理器输出对应 path epoch/goal received 诊断；
5. 测试必须记录每架 UAV 的 goal receive count；
6. 每架 UAV 在一次场景中必须且只能安装一次初始目标。

如果当前没有 goal acknowledgment，应在调试消息中增加只读字段，而不是周期重复发布目标。

## 6. 分阶段执行

### P2P-0：三机独立导航基线

使用原 `pillar.pcd`，关闭 phase-offset、swarm、tube 和机间 CBF，仅保留各机原局部规划器的静态避障。

验证：

- 三架 UAV 均收到自己的目标；
- 三个局部规划器均成功生成路径；
- 三条路径均绕开原地图柱子；
- 三架 UAV 均到达各自终点；
- 话题和命令完全隔离；
- 无 NaN、无进程退出、无错误重规划。

这一阶段失败时，不允许调试集群控制。

### P2P-1：phase-offset 几何但无邻居作用

开启 phase-offset，保持 swarm 关闭，先使用固定 offset 区间。

验证：

- `delta=0` 时与 P2P-0 基本等价；
- matched residual 接近机器精度；
- TERMINAL 模式能让 UAV 到达并停在终点；
- 不出现无原因的 phase 反向、EMERGENCY 或持续振荡。

### P2P-2：三机 ACTIVE 集群闭环

开启 swarm、ESDF tube 和 CBF。

重点检查：

- 每架 UAV 只聚合本机有效邻居；
- `g_swarm` 经过 allocator 生成 `u_w/u_delta`；
- 物理速度只通过 matched port 注入；
- CBF 不允许被 fallback 模式静默丢弃；
- 三机到达不同终点并稳定停止；
- 终端阶段不能继续被 cohesion 拉离目标。
- 三架 UAV 均不得因 offset 进入柱子膨胀区。

### P2P-3：可重复性

完整场景连续运行至少 10 次。

可以加入：

- goal 发布时间 0–1 s 随机抖动；
- SwarmState 发送时刻轻微抖动；
- 不同 ROS 进程启动顺序；
- 小于 5 cm 的初始位置扰动。

10 次必须全部满足验收标准。

## 7. 正式验收标准

一次运行必须同时满足：

### 任务完成

- 三架 UAV 全部在 45 s 内到达自己的目标；
- 最终水平目标误差不超过 0.25 m；
- 最终高度误差不超过 0.10 m；
- 最终速度不超过 0.15 m/s；
- 上述状态连续保持至少 3 s；
- 稳定后 5 s 内位置漂移不超过 0.10 m。

### 安全

- 小于 0.6 m 的机间距离采样数必须为 0；
- 实测最小机间距离必须大于 0.80 m；
- 设计目标为大于 0.95 m；
- 不允许通过修改统计阈值隐藏接近事件。
- 每架 UAV 到静态障碍物的最小距离必须大于规划器膨胀安全边界；
- 静态障碍碰撞采样数必须为 0。

### 分配器与控制模式

- `EMERGENCY` 样本为 0；
- 正常飞行只允许 `ROLLING`；
- 终端只允许 `TERMINAL`；
- 不允许长时间 `SAFETY_PRIORITY`；
- allocator 不可行次数为 0；
- matched residual 小于 `1e-8`；
- 无 phase 反向；
- 无 delta 突跳。

### 跟踪与通信

- `e_perp` 均值小于 0.15 m；
- `e_perp` 最大值小于 0.50 m；
- odom 频率约 100 Hz；
- SwarmState 频率位于 20–30 Hz；
- 无 stale 邻居造成的错误控制；
- 每机 goal 只接收一次；
- 每机只订阅自己的 odom 和 local map。

### 可重复性

- 连续 10 次全部通过；
- 不能只展示最好的一次；
- 输出每次运行的 JSON 和总汇总 JSON。

## 8. 测试脚本必须真正判断成功

`point_to_point_stability_test.py` 不能只检查节点和 odom 存在。

必须自动计算：

- 每机是否收到目标；
- 路径是否安装成功；
- 到达时间；
- 最终目标误差；
- 最终速度和漂移；
- 最小机间距离；
- 小于安全距离的采样数；
- ROLLING/TERMINAL/SAFETY_PRIORITY/EMERGENCY 数量；
- allocator 不可行次数；
- matched residual；
- e_perp；
- 通信频率；
- 进程是否存活；
- 测试结束后是否存在残留 ROS 进程。

任一硬指标不满足，脚本退出码必须非零。

## 9. 调试顺序

若 P2P-2 不稳定，按以下顺序排查，不要同时改大量参数：

1. goal 是否可靠接收且只初始化一次；
2. 三条基础路径和 phase 是否正确；
3. 没有集群作用时是否能稳定到达；
4. `g_swarm` 每条邻居贡献是否合理；
5. `u_w/u_delta` 是否限幅、连续且方向正确；
6. matched port 是否使用最终 allocator 输出；
7. governor 是否真实执行 velocity feedforward；
8. SO3 实际速度与承诺速度误差；
9. CBF 不可行的具体约束组合；
10. TERMINAL 阶段是否仍有非必要 cohesion/offset 命令。

每次只修改一个问题，并重新执行至少三次完整场景。

## 10. 当前阶段明确禁止

在三机使用原 `pillar.pcd` 从左到右连续 10 次全部通过以前，禁止：

- 调试走廊；
- 切换到 DeepSeek 新生成的障碍场；
- 调试 circle/figure-eight；
- 修改 C2 connector；
- 运行七机 ACTIVE；
- 扩展新消息或新控制模式；
- 继续做消融；
- 宣称批次 2–10 全部完成；
- 为某一个场景硬编码 robot ID、位置或优先级。

## 11. 阶段交付物

完成后只提交以下结果：

1. `pillar.pcd` 左到右专用 launch/YAML/test；
2. 为稳定点到点所需的最小通用修复；
3. 10 次独立运行 JSON；
4. 汇总 JSON；
5. 一份 rosbag；
6. 一张 RViz 截图；
7. 三机轨迹图；
8. 最小距离、目标误差、速度、模式计数和 matched residual；
9. 仍然存在的问题。

在这一里程碑通过以后，下一步优先考虑在同一 `pillar.pcd` 上加入一次异步目标更新或增量换路，而不是直接返回狭窄通道和闭合 8 字。
