# M2B：真实单段路径与局部视图适配监督记录

日期：2026-09-09

状态：R2有限批次已通过主会话独立验收。本文不是生产切换完成报告。

## 1. 本批实际范围

依据 `Horizontal_Section_Tube_M2B_Execution_Spec_2026-09-09.md` R2最终版。Luna负责主要C++实现，主会话冻结规格、审核返修、独立构建验证。主会话曾补入测试localBox前置声明，以修复并行编辑期间暴露的编译问题；其余功能编码仍委托Luna。

新增integration的makeSectionBuildInput，将现有不可变单segment ContinuousPhasePath和已完整捕获的LocalObstacleView转为SectionBuildInput。实际路径查询仍用原evaluate与normal frame，导数界仍来自原cellBounds；不重拟合、不改参数化、不改C2公式，不带入旧map/support安装身份。

这只是地图值适配，不是SDFMap捕获入口。新模块未接manager/preview/控制。旧worker、旧安装链、旧地图仍在生产中。

## 2. 为什么不能把此批当M3完成

1. 原ContinuousPhasePath的多段evaluate使用1e-8选段容差，在seam后极窄区间仍可能选左段并clamp；单纯端点C2容差检查不足以与右段连续界配套。R1精确端点检查方案被审核否决，R2仅接受整个path对象只有一个segment，多段对象无论请求范围如何均拒绝。此限制是中间交付，不能替代最终多段C2/共同前缀接入。
2. 现有界为范数界，abs_p_ww=(Axy,Axy,A)。水平弯曲而高度恒定时z界仍可能偏松，不能谎称逐坐标紧界。abs_N_ww=(B,B,0)，B=J/q+3(Axy/q)^2；Axy=0时normal恒定，B精确0。
3. 已知域仍要求调用方提供，不能从零占据或空云推断已知空闲。真实地图读取同步、生产者范围与接收裁剪范围、halo账本尚待成组接入。

## 3. 监督返修

- 回调初稿捕获整个Segment，会复制Evaluator及弧长表；改为只保留shared path owner和两个域端点。
- 障碍集合初稿发生额外复制；改为move，保留所有盒且不再膨胀。
- 去掉重复证书完整性验证和未使用helper；头文件明确同frame、不可变alias、成功不等于tube安全的前置条件。
- 测试初稿缺真实M1A生成视图、默认参数mapped tube的独立oracle、逐分量导数交叉验证；要求补齐。
- 速度断言曾用1e-8最低门槛减1e-8容差，退化为空断言；要求收紧至1e-12。
- 平面零厚度测试曾允许成功/失败均通过；要求固定失败预期，不掩盖已知能力限制。
- 原测试localBox声明顺序错误已暴露并修正，不能把编辑中断导致的编译失败解释为路径数学失败。

## 4. 独立验证记录

隔离产物：`.horizontal_section_refactor/m2b_20260909_01/`；没有写原build/devel、没有启动ROS或操作用户进程。

八包catkin隔离配置通过；重新编译原path、normal、M1A、M2及数学/allocator回归。已有回归结果：path20、normal6、LocalObstacleView23、SectionTube28、Geometry28、MatchedPort6、Allocator38，共149项通过。XML均存本批产物目录。

M2B中间版本曾16/16、17/17通过。断点恢复后，主会话从现有源码独立完成catkin 19/19和ASan/UBSan 19/19（detect_leaks与halt_on_error启用），新adapter/test以-Wall -Wextra -Werror编译通过。结果为section_input_catkin_interim.xml和section_input_sanitized_interim.xml。仍有平面用例固定失败断言、mapped逐分量导数oracle收紧等验收缺项，这些通过不作为最终完成依据。

Luna曾返回503鉴权/配额错误，主会话已重试其未完成任务；不擅自换编码模型，不把服务故障说成数学失败。源码与测试产物均保留。

## 5. 修改范围

本批代码白名单为phase_offset_section_input.h/.cpp、phase_offset_section_input_test.cpp和bspline_traj/CMakeLists.txt。CMake目前只新增21行接线；M1A/M2原改动与用户未跟踪文件保留。

HEAD仍为20d44c94d4165e35621cc30269e3fa0b2a5f10ff，branch为pro_review_current_20260903。test_gvf.launch的circle enable/auto_start仍为false。原section_tube.cpp指纹仍为540581697b3e0f61611de0353f361999e9f08f6bceea85e55d533f5eea48e2b0。

## 6. 下一步边界

本批R2单segment适配验收完成并在此停止功能改动，不自动进入下一阶段。仍需新规格落实一般C2接缝、逐坐标紧界及真实地图同步/已知域，然后才能成组退出旧worker/安装链并接preview和执行。本文不声称ROS闭环、多机或论文效果已验证。

## 7. 最终冻结验收

Luna恢复后完成剩余修订。主会话再次独立构建：catkin 19/19；新adapter/test以C++14、O1、-ffp-contract=off、-Wall -Wextra -Werror、ASan+UBSan编译，链接主会话独立instrument的path/normal/B样条/M1A/M2对象，19/19通过，detect_leaks及halt_on_error均启用。未报告sanitizer错误。XML为 `section_input_catkin_reviewed.xml`、`section_input_sanitized_reviewed.xml`。

最终检查内容：mapped及quintic逐坐标p_ww检查；N_w中心差分交叉验证N_ww（采样仅为测试，不是连续证明）；真实M1A网格视图保留原盒；默认.4 clearance/1半宽mapped完整tube通过独立域、距离、参考速度oracle；gap/overlap/bad seam多段对象拒绝；平面零厚度例明确不可用、不完整、非ZERO_ONLY；callbacks先填candidate再校验成功赋值。既有149项回归也通过。

冻结文件SHA256：

- phase_offset_section_input.h：061507d91903d251400a93cac8a48320f8060c99580bde81ae081ac214401e74
- phase_offset_section_input.cpp：d5b792db94c2aa5782f31990e9ebe685f0968b173872a604ff4110b6eca5fb79
- phase_offset_section_input_test.cpp：b06fd4c733f4461f11423adfb45c78490d709ce7e2a3108c782f9fedd79e197c
- bspline_traj/CMakeLists.txt：e18f6a75602f486481b173aa82739d38bc4082b9a6aab3959f75d178be00d49f

所有rg可见src冻结指纹：33e2379af7fb95e21859630ef554e5ae03fad24fd8948002a077a8fdfd5e9389。移除本批3新增文件并将白名单CMake摘要替换为进入时摘要后，恢复得到344df5943c539a47eb377551d38cba6dea2f8346d3fee21385d4980dbf735312，与进入时完全一致；证明其余rg可见源码内容未改变。git diff --check通过，HEAD/branch未变，未stage/commit，原用户未跟踪文件保留。
