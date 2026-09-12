# M1A：局部障碍视图接口与成组替换边界

日期：2026-09-09

状态：本会话依据用户“可以，继续冻结接口及成组替换规格”的授权，冻结本批有限实现。编码由已有 gpt-5.6-luna / max 子代理执行，主会话审核和复验。不得自动进入 M1B/M2/M3。

## 1. 本批目标与顺序修正

实现可独立测试的局部障碍读取内核，使新横截面 builder 有明确的地图输入契约。暂不替换原 SDFMap、manager 或 adapter，不切换运行时后端。

M0 已证明：直接覆盖地图 cpp/h 会令旧 manager/query/adapter 对不存在的 API 继续引用。为保证交付点可构建，把“地图薄接口准备”和“原地图恢复+消费者切换”分开：

1. M1A：本文件的局部数据内核及测试。
2. 新截面 builder 的离线实现/验证；依赖已冻结的数据语义，不依赖运行时安装机制。
3. M1B 与 M3 的成组切换：恢复 baseline 地图，补最少的一致读取入口，同时替换全部旧消费者并接入新 builder；完整构建和关闭模式导航通过后才能视为地图恢复交付。

这只澄清原 M1/M2/M3 的依赖顺序，不增加算法目标，也不建立第二条长期生产后端。本批并不宣称地图已恢复或新 tube 已投入执行。

基线：HEAD `20d44c94d4165e35621cc30269e3fa0b2a5f10ff`，branch `pro_review_current_20260903`。现有未跟踪文档和 M0 产物保留。

## 2. 代码白名单

以 W=`/home/cxq/ISF-GVF/New_ISFGVF/gvf_ws` 为根：

- 新增 `src/swarm_planner/plan_env/include/plan_env/local_obstacle_view.h`。
- 新增 `src/swarm_planner/plan_env/src/local_obstacle_view.cpp`。
- 新增 `src/swarm_planner/plan_env/test/local_obstacle_view_test.cpp`。
- 修改 `src/swarm_planner/plan_env/CMakeLists.txt`：仅把新 cpp 加入现有 plan_env target，并注册/链接新 gtest；不删除其他源/测试，不更改编译旗标。

主会话可写本规格及 `docs/Horizontal_Section_Tube_M1A_Report_2026-09-09.md`；编译/测试/清单产物仅在 `.horizontal_section_refactor/m1a_20260909_01/`。

其他 src、baseline、SUPER、原 build/devel、launch、旧计划均不修改。不创建分支、提交、stage，不连接用户 ROS 会话。

## 3. 接口约定

头文件仅依赖 Eigen/STL，不 include ROS、SDFMap、phase_offset_core 或 V2 snapshot/certificate。

namespace：`plan_env`。

`LocalObstacleBox`：`Eigen::Vector3d min/max`，表示世界坐标的闭 AABB。

`LocalObstacleGridView`：只读借用输入，不拥有缓冲区。

- `origin`、`voxel_count`、`resolution`。
- `map_bounds`、`frame_id`。
- `const std::vector<char>* inflated`：必需，有效基础占据层。
- `const std::vector<char>* manual`、`static_layer`：可选有效覆盖层。

输入不读 raw log-odds/ESDF、不重新膨胀；非零字节均保守视为占据。调用者必须传入所有实际有效覆盖层，不能因为 mp 的某个 enable 关闭而丢失实际 getter 仍会使用的 overlay。ceiling若已经在有效占据层中自然保留；不根据参数无条件生成不存在的天花板。

`LocalObstacleRequest`：

- `reference_region`：参考带所需区域，允许平面/线/点退化盒（min<=max）。
- `clearance`：相对于输入已膨胀体素还需保留的非负余量。本批不规定新的机体预算，调用方显式传值。
- `known_region`、`known_region_valid`（默认false）：调用方明确给出的完整已知域，不由零值占据推断。
- `max_voxel_checks`、`max_occupied_voxels`：默认200000、100000；必须为正，先计算遍历工作量再分配和扫描。

`LocalObstacleViewStatus`：`VALID`、`INVALID_INPUT`、`UNKNOWN_DOMAIN`、`OUT_OF_DOMAIN`、`BUDGET_EXCEEDED`。

`LocalObstacleVoxel`：原始整数 `index`、完整世界坐标 `bounds`、`layer_mask`。bit0=inflated，bit1=manual，bit2=static；重叠层只输出一个体素。

`LocalObstacleView`：

- `status` 默认INVALID_INPUT、`frame_id`、`resolution`。
- `obstacle_region`：本次取得完整障碍信息的局部区域。
- `reference_domain`：本视图允许声明可检查的参考位置域；新 builder 必须把它作为边界约束。
- `request_clipped`：原请求是否因map/known边界被缩小。
- `visited_voxels`、`occupied_voxels`（后者若与vector长度重复可只留vector长度查询）。
- `std::vector<LocalObstacleVoxel, Eigen::aligned_allocator<LocalObstacleVoxel>> obstacles`。

函数：

```cpp
LocalObstacleView buildLocalObstacleView(
    const LocalObstacleGridView& grid,
    const LocalObstacleRequest& request);
```

输出拥有所需障碍盒，返回后不再借用输入buffer。integration后续用 `shared_ptr<const LocalObstacleView>` 持有即可，不新增map/support/configuration ID或逐查询身份。

## 4. 域、裁剪与体素规则

1. 校验所有坐标/分辨率有限，resolution>0，count各轴>0，frame非空；维度乘积size_t安全。inflated长度恰好等于体素数，非空overlay指针同样校验。拒绝无效map盒和明显超出grid物理范围的map边界。
2. known_region_valid=false返回UNKNOWN_DOMAIN，无障碍安全结果；known盒非有限/颠倒是INVALID_INPUT。known域可以超出map，取交集；交集无有效区域返回OUT_OF_DOMAIN。
3. reference_region加clearance形成所需obstacle ROI，与grid/map/known有效域相交。允许裁剪，不因名义半宽伸出已知域而拒绝整个可用截面。
4. 返回reference_domain是原reference_region与本次obstacle ROI内缩clearance后的交集。完全覆盖原请求时可原样保留reference_region，避免因无意义浮点往返令零厚度参考带变空。部分裁剪时边界必须向安全内侧取整；空域返回OUT_OF_DOMAIN。
5. 域计算必须确保reference_domain内每点的clearance邻域落在obstacle_region中。处理clearance=0的恒等情况，数值溢出/不能可靠表示时明确失败，不用大tolerance拓宽安全域。
   long double不能被当作精确实数；非零极小clearance也不能消失。采用有限步骤的方向舍入/包围，不允许无界nextafter修复循环，也不引入多精度或浮点身份认证框架。只对真正被裁剪的面做内缩，不因x方向裁剪把未裁剪的零厚度y/z参考域误缩为空。request_clipped必须反映实际reference域缩小。
6. 枚举每个与闭obstacle_region相交/接触的封闭体素，包含网格面两侧邻格。可以使用保守的一格索引padding，但不能漏掉只以表面相接的占据体素；不能只检查体素中心是否在ROI内。
   padding只扩展候选枚举范围；输出前以完整闭体素盒与obstacle_region相交过滤，不把明显不相交的padding格当作局部障碍返回。向外浮点包围产生的ULP级额外覆盖允许保守保留。独立oracle在一般小数网格上验证“不漏真相交”，精确集合相等测试使用可精确表示的网格。
7. 体素box按native origin + index*resolution生成，返回完整盒而非ROI裁剪后的碎片。浮点盒边界采用向外包围；不要额外把同一体素再扩大半对角线。
8. x/y/z确定性顺序，x-major地址与原SDFMap一致。只扫描局部索引范围，不创建全图support mask或全图索引。
9. 遍历量超过budget，在扫描前失败。障碍输出量超过budget时清空输出并返回BUDGET_EXCEEDED；不得把截断障碍列表标为VALID。失败输出不携带可被误用的有效reference_domain/obstacles。
10. 输出中的空障碍列表仅在明确已知域与完整枚举成立时表示局部没有占据，绝不从普通传感器点云为空推导已知空闲。

输入是稳定的一致读视图这一点属于调用方前置条件：函数自身不获取SDFMap锁，也不通过一个“coherent=true”字段伪装同步证明。头文件必须明确禁止输入缓冲区在调用期间被修改。并发集成由第6节的成组切换规格负责；本批不声称已经完成运行时同步。

## 5. 固定验收测试

至少以下独立gtest，全部离线、无ROS master、无固定/tmp文件写入：

1. 已知无障碍网格：VALID、空列表、reference域正确。
2. known_region_valid=false：UNKNOWN_DOMAIN，不能伪造free。
3. 单占据体素：正确native index、闭盒体积和mask。
4. manual/static/inflated覆盖层并集与重叠去重，非零负char同样占据。
5. ROI恰落网格面/边/角：接触的相邻占据格不遗漏。
6. 非零origin、非整数resolution，整数地址与世界盒一致。
7. 超大网格、很小ROI：visited仅为局部范围；不得依赖遍历完整网格。
8. 请求超出已知域但中心部分可用：裁剪、reference_domain收缩正确，原请求未被伪装全部覆盖。
9. 请求完全域外、clearance导致域为空：OUT_OF_DOMAIN。
10. clearance halo内但reference_region外的障碍仍被返回。
11. map边界小于grid整数覆盖范围：reference_domain不扩展到grid多出来的体素区。
12. 输入非有限、负/零resolution、维数非法、尺寸乘积溢出、buffer长度错误、颠倒盒：INVALID_INPUT，不访问越界buffer。
13. voxel预算/obstacle预算失败：无半截安全结果。
14. 退化reference盒（点/线/水平面）、clearance=0/正数：合理保留或保守明确拒绝，不出现NaN。
15. 输出独立性：返回后改输入buffer不改变已返回结果。
16. 多次相同输入得到相同有序几何结果，不添加计时/序号作为几何有效性条件。
17. 非零极小clearance、大origin、仅单轴裁剪而其他轴为零厚度的参考带：不丢失余量、不无界修复、不伪装完整域。

独立测试oracle对小网格穷举闭盒相交（与实现索引算法独立），核对没有遗漏；随机种子固定。大网格的性能测试只记录visited，不以机器耗时脆弱断言通过。

## 6. 后续成组替换的已冻结边界

- baseline sdf_map.cpp/h成对恢复，不复制B的launch，不改变原occupancy/ESDF/manual/static算法和参数。
- 恢复后增加的地图侧内容只允许服务一致局部读取的最小锁/入口；不加地图身份和证书链。
- stable `LocalObstacleGridView` 的生成和buildLocalObstacleView调用必须与原map写者同步，借用buffer不越过锁生命周期；只允许必要局部扫描在锁内，后续几何构建在锁外。
- known域来自首版完整静态局部云契约或真实free观测，生产者中心/范围不可用receiver最新位置无条件替代。首版契约的具体消息/读取入口仍需在切换批次中落实，不在本次伪造。
- 新section builder和adapter完成前，原map调用者不删除、不替换成假ID stub。
- 切换时必须成组处理manager、cloud/evidence query、adapter、runtime、preview、allocator和CMake闭包，保留命令发布成功后的状态提交及原路径接续。
- 单独本批新增local view不改变当前运行时行为，关闭phase-offset的原路径也不额外计算local view。

## 7. 构建与交付

Luna只按白名单编码；发现接口无法满足规则、必须改SDFMap或增加外部依赖时停止报告。主会话可修订本规格，不允许Luna自写新方案。

先用C++14/Eigen/gtest直接编译新源码和新测试到本批产物目录，再执行ASan/UBSan。之后对plan_env的实际CMake目标做独立构建配置验证，不写原build/devel。保留其他existing测试声明，检查git diff --check和文件白名单。

交付：新局部读取模块、测试、CMake有限接线、主会话审核记录及结果。不得称“tube重构完成”或“baseline地图已替换”。
