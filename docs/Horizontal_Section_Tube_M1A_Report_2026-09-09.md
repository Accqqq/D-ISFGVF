# M1A：局部障碍视图实现与监督验收

日期：2026-09-09

依据：[本批执行规格](Horizontal_Section_Tube_M1A_Execution_Spec_2026-09-09.md)。

## 1. 结果和边界

M1A 完成：局部障碍读取内核、离线测试及 plan_env 的有限 CMake 接线已实现。编码由用户指定的已有 Luna / max 子代理完成；主会话编写规格、逐段审核、要求返修，并独立编译和测试。

尚未恢复/替换 SDFMap，也未切换当前 tube 后端、manager、adapter 或执行逻辑。新模块现在提供可复用的数据接口，不自动在控制周期运行。不能把本批结果称为完整 tube 重构或论文效果已实现。

## 2. 实际代码修改

以工作区 `New_ISFGVF/gvf_ws` 为根：

| 文件 | 修改 |
|---|---|
| `src/swarm_planner/plan_env/include/plan_env/local_obstacle_view.h` | 新增只依赖 Eigen/STL 的输入/输出与函数接口 |
| `src/swarm_planner/plan_env/src/local_obstacle_view.cpp` | 新增局部域裁剪、闭体素读取、覆盖层并集与预算处理 |
| `src/swarm_planner/plan_env/test/local_obstacle_view_test.cpp` | 新增23项离线测试 |
| `src/swarm_planner/plan_env/CMakeLists.txt` | 仅增11行，添加新cpp及新gtest，不删除旧源/测试 |

其他生产源文件未修改；baseline、SUPER、原build/devel、launch和用户ROS进程未动。未创建分支、commit或stage。

## 3. 提供的行为

- 输入为调用方保证稳定一致的栅格buffer借用；函数不伪造coherent标志，也不添加map/support/configuration编号链。
- 三个实际有效占据层取并集，单个体素只输出一次，附简单层mask。
- 只扫描请求区域加clearance halo后的局部候选范围，闭体素与ROI的面/边/角接触不遗漏。
- 请求超出map/已知域时返回真实裁剪后的reference_domain，不将原请求全部标记可用。
- 明确已知域缺失时返回UNKNOWN_DOMAIN；空占据列表不自动等于未知空间free。
- 预算超限、无效输入、无可用域均不携带半截可用几何结果。
- 输出拥有障碍盒，不继续借用输入；后续改源buffer不会更改返回值。
- `VALID`只说明本次局部障碍信息完整，`reference_domain`是可查询范围，不是free-space声明。后续builder必须根据obstacles计算真正安全截面。

本批不推断机体物理半径或修改原安全距离；clearance由后续已确定语义的调用者显式提供。运行时一致读取与生产者已知域来源仍在成组切换批次落实。

## 4. 主会话审查及返修

| 审查发现 | 处理 |
|---|---|
| 初版将long double称为exact，极小clearance可能丢失 | 改为有限步骤的方向包围，clearance=0保持恒等，非零极小值专门测试 |
| 只对最终乘加结果做舍入，不能覆盖origin与乘积相消误差 | 乘法和加法分别保守包围；增加index=999999、origin=-100000、resolution=.1的真实相消反例 |
| nextafter修复while没有上限 | 移除无界循环，最多一步补正后仍不可表达就明确失败 |
| 单轴裁剪可能误缩未裁剪的零厚度参考轴 | 按实际被裁剪的面处理；x裁剪而z=1的水平参考面保持不变 |
| 巨大origin/极小resolution可能被向外ULP“造出”有效grid | 先检查未扩张的原始步长/整体跨度可表示，失败返回INVALID_INPUT |
| 给所有free格计算世界盒，并按大候选数预留输出空间 | 先检查层mask，free格直接跳过；初始reserve不超过256 |
| 测试二进制直接编译通过，但catkin目标没有main | 新test文件按同包风格提供main，实际catkin重新构建通过 |
| VALID/reference_domain易被理解为无碰撞 | 补齐头文件语义说明；不将读取成功冒充tube构建成功 |

修改均限原4文件白名单；没有因发现问题扩大到原地图或控制算法。

## 5. 独立验证结果

### 5.1 真实 CMake/catkin 构建

在 `.horizontal_section_refactor/m1a_20260909_01/catkin_build` 独立配置，只包含common_msgs和plan_env，CMAKE_PREFIX_PATH仅为`/opt/ros/noetic`，输出至本批catkin_devel，不引用原工作区devel。

主会话完成：

1. common_msgs C++消息生成。
2. plan_env完整库构建，包含新local_obstacle_view.cpp。
3. 新local_obstacle_view_test目标构建。
4. 既有manual_map_layer_test目标构建。

新测试目标第一次构建暴露缺main，已由Luna在白名单内修正。首次配置早于目标添加，曾出现target不存在；重新配置后解决。这些过程问题没有被记为成功测试。

系统原有VTK缺失工具引用、PCL可选功能以及旧源未使用函数/符号比较warning存在，没有为此修改系统或无关源码。

### 5.2 测试

| 路径 | 主会话结果 | 产物 |
|---|---|---|
| 隔离catkin新测试 | 23/23通过 | `local_obstacle_view_test_reviewed.xml` |
| 主会话重新编译ASan+UBSan，泄漏检测与halt-on-error开启 | 23/23通过，无报告的sanitizer错误 | `sanitized_reviewed.xml` |
| 同一新plan_env库的原manual obstacle/boundary回归 | 2/2通过 | `manual_map_layer_test_reviewed.xml` |

Luna自己的direct与sanitizer测试也通过，但主会话未仅依靠其自报结果验收。主会话运行stdout和退出码保存为`supervisor_*.json`。

新测试包括层并集、闭接触、小ROI局部遍历、已知域裁剪、clearance halo、坏buffer/维度/预算、输出独立性、确定性，以及审查增加的极小余量、单面裁剪、乘加相消和不可表示网格。

本批没有运行会写固定/tmp文件的另外3项旧manual测试，也没有运行ROS导航场景；不把2项回归说成全部地图回归。

测试执行时间不作为新tube性能数据。尚未测量真实地图流下的锁等待、几何构建耗时或多机规模。

## 6. 文件范围与指纹

基线branch仍为`pro_review_current_20260903`，HEAD仍为`20d44c94d4165e35621cc30269e3fa0b2a5f10ff`。

排除本批4个白名单文件后的rg可见src指纹，前后均为：

`a2af8d1c55c9f9fd83ff42f52cf3776f03acc4e5e5ff34ee862412f818fd6e03`。

最终代码SHA256：

- header：`8533ea3e43ed64d27bd8e842213781a7e4ccd049d0b926c902c8f322d5740e4a`。
- cpp：`d52f90f40f40ae500f051fa4f9b1792a808eab2fbdf43d761a8dff6c3fac3e52`。
- test：`e5fb43a629117292c37bb26aaef45d0ef7def0249bcb1db683dd376b28316cf9`。
- CMake：`d33bc395095f23c2dda2f0b3ba05b8a5747e21e77f6a8eb04a96e3715d3d948a`。

`git diff --check`通过；新增未跟踪文件另作no-index空白检查。原用户未跟踪文档/缓存与M0产物保留。

## 7. 停止点与下一批

本批已完成并停止，Luna等待下一份有界规格。下一步是基于该局部障碍契约实现和离线验证横截面builder；其通过后，才成组恢复baseline地图并替换旧消费者。仍需在切换批次中完成局部已知域来源、共享buffer同步、活动参考接续和控制deadline验收。

没有将当前map替换成半成品，也没有启动第二套生产tube后端。
