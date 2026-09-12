# M3B 单拍执行监督记录

日期2026-09-09。状态：R4本批实现与主会话独立验收完成。Luna唯一编码，主会话规格/监督/独立验证，Astra medium只读审核。

## 实际边界

在TubeViability检查选定port的一拍PWL轨迹；在现有Runtime用同一delta_/previous_final_port_进行准备与提交。不是ROS主控制切换，没有恢复baseline地图、接实际读取或退出worker/安装链。最终governor命令配套仍需后续成组切换。

不改PortProjector/core、不重投影、不调用V2 guard/reserve，不新增地图ID/证书。R1/R2未执行草案已由R4替代，白名单仅runtime/viability各h/cpp/test。

## 监督返修

- 现有PortProjector::verify内部调用project，不能作为轻量验证；改为同一K上的本拍当前/跨节点/终点检查。
- 私有prepared即待提交值，不新增sealed副本。提交前必须同锁validate→publish→noFail，本地revision防重复/ABA，不是地图身份。
- 初稿IntersectHeld清空result却传入(common,envelope,common)，导致alias输入被清；改局部结果后赋值，避免可行例误拒。
- 舍入next_w==knot不代表精确终点；仅terminal区间为该节点singleton时省交点检查。
- prepared初稿保存仅prepare借用的preview裸指针且重复校验，已要求删除，私有结果只存matched/port/next/旧状态与版本。
- 新Section配置只检查实际使用的tracking/min-reference/tangent数值，不继承旧manual amplitude/period/preflight门控。
- 所有旧成功状态提交与reset使旧Section值失效；手动preflight仅更新无关元数据不应失效，只有真正改变回收状态才失效。
- 节点定位计入budget，跨节点用已知index直接取K，不每节点从头查；不扫描地图或未来轨迹。

## 已有独立验证

隔离目录.horizontal_section_refactor/m3b_20260909_01/，原build/devel与ROS会话未改。进入时源码摘要7143940f208dd71d341278c35b34bfc9800e236b86b7efbc62bcfd8cd17c702f，HEAD20d44c94d4165e35621cc30269e3fa0b2a5f10ff。

基线Runtime V2 5/5、legacy37/37通过。首轮实现后V2宏9/9、legacy宏41/41（均含4项新Section测试）、viability40/40通过，仅中间结果，不当作最终验收。生产runtime/viability ASan对象已独立编译，最终测试覆盖与san运行待冻结后完成。

## 收尾验收与最终结果

真实allocator两拍非零delta回收、prepare后输入销毁生命周期、旧提交失效、同锁模拟发布失败/成功、幅度slew/tracking/tangent、PARTIAL覆盖与rounded-knot/alias守卫回归均已完成。两拍使用相同1相位单位测试预览，第二拍w取第一拍nextW而不重置；原先2单位预览超过测试profile剩余覆盖的fixture问题没有靠放宽生产参数解决。

精确舍入回归：current=.1、dt=.7、rate=1，DecodeExact确认exact终点比rounded knot大2^-55；wide profile正常通过且work29，对照精确terminal为21。临时替换旧skip条件后29变27，恢复正确源码，锁定跨节点保护真实执行。非singleton终点正常成功同时覆盖alias修复，不以明显越界例冒充该守卫测试。

主会话最终普通构建测试：

| 目标 | 结果 |
|---|---:|
| Runtime生产宏 | 14/14（6旧/兼容与8Section） |
| Runtime legacy宏 | 46/46（38旧/兼容与相同8Section） |
| TubeViability | 42/42 |
| Allocator | 44/44 |
| Geometry / Matched / LocalObstacleView | 28/6/23 |
| Path / Normal / SectionInput / SectionTube | 25/7/26/32 |

合计293次普通测试执行，扣除Runtime两宏重复8项Section测试为285个不同用例；不混算测试数。ASan/UBSan独立运行TubeViability42和Runtime生产宏14，共56项通过，无报告错误；legacy完整37旧测试与新增兼容测试在普通构建执行，未宣称legacy整套也经过本批sanitizer。detect_leaks/halt_on_error启用。旧源码pragma/宏分支unused告警保留，新增实现严格warning编译；未为此改core数学。

最终结果位于.horizontal_section_refactor/m3b_20260909_01/的*_final.xml、*_regression.xml、runtime_sanitized_final.xml、viability_sanitized_final.xml；Luna记录位于executor/*_final.*。初版日志不覆盖成最终成功，源码所有改动由Luna完成。Luna中间使用supervisor构建目录的分工错误已提醒，主会话独立从冻结源码重建验收；不写原工作区build/devel。

## 修改范围与停止点

最终src摘要f800b25e992fead531de22dc707d10bf2cd482c575b999037876e41e4f2643d0。将六白名单摘要替换为进入值后恢复7143940f208dd71d341278c35b34bfc9800e236b86b7efbc62bcfd8cd17c702f，与进入时一致，证明其余rg可见源码不变。证据为supervisor_whitelist_before.txt、supervisor_whitelist_final.sha256、supervisor_normalized_after.sha256。git diff --check通过，HEAD20d44c94d4165e35621cc30269e3fa0b2a5f10ff与原分支不变，未stage/commit，用户修改保留。

Luna冻结并停止本批编辑，主会话完成验收。在此停止本批功能修改，不自动扩展manager/map。当前Section Runtime入口仍未由实际cmdCallback调用；外部同锁发布事务测试是接口示范，不是ROS运行验证。后续必须完成baseline地图恢复、真实局部读取、manager/governor最终命令配套及旧worker/安装链退出，不能把本批称为整体完成。
