# M3B R3 自审

## 结论

R3比R2更小：不改PortProjector，不复制project，不引入完整V2 admission/reserve。只在现有TubeViability中增加单拍PWL检查，在Runtime中增加Section prepare/commit值接口。

## 已处理风险

- 选定port不会被重新投影或悄悄替换。
- 一个dt跨多个PWL cell时逐个检查节点和终点。
- token使用本地单调epoch防重复提交/ABA；发布到commit必须由caller同一串行域覆盖。
- governor最终PositionCommand仍明确列为非本批目标。
- PARTIAL不冒充终点；Section不依赖地图ID。

## 仍明确留后续

manager/adapter最终命令配套、地图同步/known域、baseline地图恢复、旧worker/安装链退出、ROS验证及论文多机效果均未授权本批处理。若实现必须修改这些文件，Luna必须停止并报告。

## 放行

R4已合并Astra的终端/内部knot浮点包围修订，主会话显式放行。仅要求本拍的有界数值检查，不新增未来reserve。保留旧tracking/tangent/min-reference数值限制，不增加默认margin。验收必须保留原runtime/V2测试，并新增Section单拍、失败不提交、重复提交和ZOH跨节点测试。
