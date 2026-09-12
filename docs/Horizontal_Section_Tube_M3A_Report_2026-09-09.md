# M3A：Preview/Allocator消费者迁移监督记录

日期：2026-09-09。状态：本批已实现并通过主会话独立验收。Luna唯一编码，主会话规格/监督/独立验证，Astra medium只读审核。

## 1. 本批位置

M2D多段几何已验收。M3A让现有preview/allocator直接消费SectionTubeProfile，共用原严格PWL数学核，消除新入口对map/source/tube/profile身份链的依赖，不把新profile转换为假V2对象。旧入口在manager/runtime切换前保留原行为。单独本批不代表主控制切换完成。

## 2. 审核修订

Astra确认6文件内可提取共用PWL核心，要求三类细节：所有strict envelope/rate查询都按SECTION使用numericValid；allocator前置/关联/失败分类三处一致分派；借用profile和构建时path/frame由caller共同持有，不事后给旧profile贴新revision。

现有带phase_rate的rate查询仅校验该rate落在policy范围，实际限制仍按lower_nu/upper_nu整个区间计算，并无dt全程检查。主会话因此修正规格：本批点查询不冒充整tick ZOH；完整runtime推进/发布后commit留下一批。

终端截短必须profile.complete且状态COMPLETE/ZERO_ONLY，PARTIAL即使设置terminal flag也拒绝；真实path终点相等是未来adapter前置条件。本批不造新的终端控制模式。

## 3. 成组切换前只读核查存档

这些是下一批的事实输入，不是已实施内容：

- manager权威w在authoritative_phase_mutex_内，runtime持有delta_/previous_final_port_；不得创建第二owner。
- cmdCallback约2368–2452准备phase token并在发布成功后提交w，adapter publishPendingPositionCommand约3353–3411在task_publication→runtime锁内发布后commit。必须保留发布成功才改变状态。
- manager已存在frontend_apply→handoff→phase锁序；stageFutureSeamPathReferenceHandoffV2在重建/地图捕获时不持manager锁，最后短锁提交。新的同步几何应放FSM规划/C2流程，不能进高频命令回调或runtime锁内。
- 旧adapter update约3421–3698长期持runtime锁计算preview/allocator/准备证据；新切换要避免在此加地图扫描/tube构建。
- 旧worker唯一builder调用在phase_offset_tube_runtime_v2.cpp约682。worker内部state mutex不跨build，但adapter joinTubeWorker持worker_state锁跨join；退出时必须保证不在主控制关键锁等待。
- local_sensing保留首次静态完整地图，发布局部盒，完整云用实际sensing_odom_stamp；SDFMap已有256条odom历史及生产者range/frame匹配。可提取必要的source中心/range，不保留旧IDs/full support mask。
- baseline cloudCallback忽略stamp，以receiver当前中心严格<再裁剪，XY膨胀ceil(inflation/resolution)、Z固定1；因此known域须对应source盒和实际receiver裁剪盒交集及膨胀/格宽halo，不是点云极值/最新odom盒。
- 完整空云可给known域依据，但baseline空云不写occupancy/ESDF这一行为应保持；未验证空云不能当free。
- reset/gradualReset/depth clearing等移除地图内容后，未来capture的known声明必须失效，不能沿用半帧信息。所有地图写者和组合读者需同一锁闭包，不能只锁cloud+新view；actual AsyncSpinner为8线程。
- 单独恢复baseline sdf_map.cpp/h会破坏旧manager/query消费者，需成组退出旧API。地图核心算法、manual/static层、planner/ISF/governor/SO3不在本批改动。
- 一项待后续确认的terminal旧分支疑点：force_goal_position取消pending时publishPendingPositionCommand可能因cancel flag直接返回false而未到deactivate/local_publish分支；未据此修改现工程，未来terminal测试应覆盖。

## 4. 验证与停止点

修改前HEAD20d44c94d4165e35621cc30269e3fa0b2a5f10ff，src摘要27751547465a2b7c8b76c31c16e258bc93edd6c51708aa18c6dbfcaca7ce8b2e。六文件白名单和before记录在.horizontal_section_refactor/m3a_20260909_01/。最终测试/指纹见下节。

未恢复地图、未退出worker/安装链、未接新profile到ROS控制。不得把本批编译/单测当ROS或论文效果验证。

## 5. 最终实现与独立验收

V2和Section直接入口共用StrictPwlSourceView、合并节点、向内PWL插值、反向递推、坡度、beta和rate数学核心。临时#if0旧body已删除，未保留第二份算法。新Section不构造V2对象、不填假ID；policy只需numericValid；allocator只核对借用对象、真实w/delta和构建路径/frame配套，不检查map/source/tube/profile ID或policy ID。旧入口在生产切换前保留原要求。

主会话要求的额外修正：校验profile已花work传入核心而非清零；早拒绝也保留work/max_work；Section不混入legacy profile源；Section诊断source字符串不作为gate；不可行preview清空借用后，非零delta不误归STALE；旧V2终点truncated标志保留。

| 独立普通测试 | 数量 |
|---|---:|
| TubeViability（31旧＋7新） | 38/38 |
| Allocator（38旧＋6新） | 44/44 |
| Geometry、MatchedPort、LocalObstacleView | 28+6+23 |
| Path、Normal、SectionInput、SectionTube | 25+7+26+32 |

合计229项普通测试通过。Viability38与Allocator44另在ASan/UBSan下共82项通过，detect_leaks与halt_on_error开启，不将san运行重复计为新增测试。旧#pragma STDC FP_CONTRACT被GCC忽略是原警告；明确使用-ffp-contract=off，strict编译仅豁免该既有unknown-pragmas警告，其余新增代码/测试-Wall -Wextra -Werror通过。

新测试验证：无map IDs正常计算/分配；非均匀真实节点不遗漏；Section与V2相同几何K/beta一致；旧与新相同geometry/limits最终selected_u一致；零宽合法/假零宽拒绝；PARTIAL有足够覆盖可用、覆盖不足不可行、不能冒充terminal；query严格域/节点语义；错对象/w/delta/path/frame及错入口拒绝；空诊断字符串不阻挡；非零delta不可行正确分类；validation工作计数不丢失。

原非均匀临界fixture {.0,.35,1.4,2.0}、区间{[-1,1],[-.2,.2],[-.7,.8],[-.4,.4]}触及严格收缩rate条件；保留CriticalNonuniformRateStateMatchesV2对照，确认两入口结果一致而非删掉失败记录。另用有数值余量的非均匀例验证FEASIBLE，未改变生产参数或原数学。

最终XML：phase_offset_tube_viability_test_final.xml、phase_offset_allocator_test_final.xml、viability_sanitized_final.xml、allocator_sanitized_final.xml及七份*_regression.xml。

## 6. 最终范围与停止声明

冻结后src摘要7143940f208dd71d341278c35b34bfc9800e236b86b7efbc62bcfd8cd17c702f。替换六文件摘要为进入值后，整体摘要恢复27751547465a2b7c8b76c31c16e258bc93edd6c51708aa18c6dbfcaca7ce8b2e，说明其余rg可见src未变。完整文件摘要存supervisor_whitelist_final.sha256，归一化核对文件supervisor_normalized_after.sha256。git diff --check通过，HEAD/branch未变，未stage/commit，用户改动保留。

Luna已冻结并停止编辑，主会话完成独立验收。本批在此停止功能修改，不自动扩展map/runtime。下一批仍须真实地图同步/known域、runtime整tick ZOH和发布后提交、manager规划内同步构建及旧worker/安装链退出；不能用本批通过宣称这些已完成。
