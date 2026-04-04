# Memory Index

这个目录用于存放项目的正式记忆文档。

## 文件说明

- `LONG_TERM_MEMORY.md`
  - 长期稳定信息
  - 项目身份、核心架构、关键函数锚点、有效参数、长期约束
- `ITERATION_MEMORY.md`
  - 当前迭代状态
  - 当前启用的行为、最近修改、当前风险、下一步任务

## 当前约定

- 根目录里旧的记忆文档已删除。
- 以后项目记忆只维护在 `memory/` 目录下。
- `src/swarm_planner/bspline_traj/gvf_manager_fsm_notes.md` 保留为专项说明，不作为正式总记忆入口。

## 使用原则

- 判断“项目本身是什么、核心结构是什么”时，优先看 `LONG_TERM_MEMORY.md`
- 判断“现在这一版代码到底处于什么状态”时，优先看 `ITERATION_MEMORY.md`
- 如果两者冲突：
  - 先以当前代码为准
  - 再更新 `ITERATION_MEMORY.md`
  - 若属于长期稳定结论，再同步 `LONG_TERM_MEMORY.md`
