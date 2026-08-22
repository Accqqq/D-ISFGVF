# Codex A6/H2 Runtime rebase：专项执行规范

```text
DOCUMENT_ROLE=DEDICATED_CURRENT_STAGE_EXECUTION_SPECIFICATION
DOCUMENT_STATUS=AUTHORIZED_BY_USER_2026-08-17__INDEPENDENT_REVIEW_REVISED_GO
STAGE=A6_H2_RUNTIME_REBASE_AT_FINAL_COMMAND_BOUNDARY
IMPLEMENTATION_AUTHORIZED=YES__MINIMAL_ADAPTER_ONLY
AUTO_ADVANCE=false
SUPERSEDES=NONE
```

## 1. 目标和边界

本阶段只处理一个已动态定位的 H2 replacement 失败：新 C2 owner 和全新 tube/profile
已成功 staged，但 final commit 仍要求 Runtime 的 `delta` 和 previous final port 与
**开始构建时**逐 bit 相同。正常 selected command 在 staging 的 35--40 ms 中会合法更新
这两个值，因此 transaction 被拒绝，`replan not installed`，旧局部 path 随后耗尽。

目标是在 stage 和 final command boundary 分别对**已完整构造的、不可变的新 owner/profile**
使用当时最新的 Runtime state 作既有 Runtime exact-PWL witness 检查；只有所有既有身份、
profile coverage、geometry、frozen provenance 和 latest categorical safety 合同仍成立时，才以
final-boundary state 作为 final CAS 的 expected state 安装 pair。

这不是 A5 tube 几何、Filter、SurfaceValidator、margin、slope、lookahead、速度、rate 或
C2 几何的修改；也不是 timer retry、提前 replan、复用旧 tube/sample，或把 `selected=false`
改成可执行。它不解决 retained state 已不在 new profile 内的情况；那种情况必须继续拒绝。

## 2. 冻结动态证据

唯一可用于本阶段的 active private-run 事实来自
`/tmp/p1_active_dynamic_20260817/evidence/` 与
`Codex_P1_Dynamic_Acceptance_Read_Only_2026-08-17.md`：

```text
bootstrap build/dry-run/stage        true / true / true
P1 normal Runtime commits            observed after gate opened
replacement C2                       success
stagePathTubePair capture            exact pair/pin/session/generation/map/profile match
staging Runtime delta                -0.0037596066975834628
staging Runtime port                 (0.0014675412986572556, -0.074337806486138358)
final live Runtime delta             -0.035817306589090242
final live Runtime port              (0.016437002918046038, -0.21862238886092472)
final `finish`                       false
then                                  `replan not installed`
terminal result                       `all_candidates_path_end_clamped`, goal still about 2.37 m away
```

因此这一次的 first false 是 Runtime bit mismatch，不是 A5 build、C2、seam
enumeration、pin/pair/session/map provenance 或 path-end replan trigger。它也不能泛化为
“所有 H2 failure 都是此原因”：只有符合本规范所有 rebase predicate 的 transaction 才能尝试。

此前 P0/P0.5 的真实 `CURRENT_OFFSET_OUTSIDE` 结论继续有效。若 latest current retained
delta 在 new profile 的 exact-PWL interval 外、reference/actual snapshot clearance 不足、
或 dry-run 无 witness，本规范要求拒绝，绝不能 rebase 安装。

## 3. 正确的线性化语义

`stagePathTubePair()` 的 capture 仍然必须在 C2/tube work 之前取得 exact old pair、
authority session 和 exclusive pin lease。它保持：

```text
old pair pointer/generation/session/profile/snapshot identity
new owner and every new profile sample belong to the new owner
old pair is pinned through final CAS
```

pin capture 内的 `retained_delta` 与 `previous_final_port` 是 capture 历史证据，**不是**
future command 的永久 lease。它们不得在 stage/prepare/final CAS 中作为 stale expected
Runtime state 使用；pin 的 pair/generation/session/profile/snapshot identity 合同不变。

本规范的“pair+runtime atomic commit”有严格且有限的含义：pair 的 single authority CAS
只能在 Runtime 仍逐 bit 等于本次 final rebase capture 时成功。CAS 不写入、回退、重置或
伪造 Runtime；成功 boundary 前后 Runtime bits 必须相同。下一正常 command 自己依既有
`complete()` 合同更新 Runtime。任何方案若要在 handoff CAS 中另行写 Runtime，属于新的
控制状态转换，必须 STOP 并另写架构执行单。

## 4. 已获 review GO 的候选机制

### R0 — staging 不变且不可绕过

继续用 capture-time取得的 exact pair/session/pin 作为 H2 base authority，但在 stage 的
Runtime mutex 内读取**当下** live Runtime bits，用其构建 new owner 全量 sampled path/new
profile并执行首次 local Runtime dry-run。已有 staged `TubeEpochManager` current admissibility、
new frozen snapshot provenance 和 profile coverage 均不变。stage 时 `CURRENT_OFFSET_OUTSIDE` /
map unsafe / no witness 仍不形成 pending handoff。

不得重新 build 或修改 staged profile；不得把 old profile/raw sample/clearance/witness
拷贝、裁剪、映射到 new connector。

### R1 — command-boundary latest capture

`prepareAndCommitPendingPathTubeHandoff()` 已取得本 command 的 immutable phase tuple
`w_live` 后，Adapter 在其短 Runtime lock 中只在以下全部成立时复制一个 local Runtime
snapshot 和 latest expected bits：

1. `live_pair == transaction.expected_pair`（exact pointer）；
2. live pair 的 generation/session/path/profile/snapshot 全部仍与 pin capture 相同，且
   `ActiveLeaseMatchesCapture()` 对原 lease 仍为真；
3. `authority_session` 未变，new candidate 的 complete owner/profile provenance 自洽；
4. `captured_w0 <= w_live < future_seam_w`，且 candidate profile 完整覆盖该 exact query
   phase 和既有 certified forward-horizon contract；
5. Runtime 的 latest `retained_delta`、previous final port 和 command phase 都是有限值。

这里不把“same revision”当作 pair equality，不允许 timer refresh、map/profile swap、session
retirement 或 pin 失效被视为 equivalent。

### R2 — lock 外的 exact Runtime revalidation

对 R1 capture 的 immutable candidate 和 local Runtime copy，在所有 manager/Runtime locks
外完成且不写 live state：

1. 验证已 prepared new owner/profile 的既有 structural coverage、owner geometry、frozen
   snapshot provenance、`captured_w0 <= w_live < future_seam_w` 和 certified horizon；不得
   修改或重建 profile，也不得重读独立 timer manager slot。
2. 使用同一 new owner/profile/snapshot 和 R1 Runtime copy，执行已有 non-mutating
   Runtime `dryRun()` continuous exact-PWL witness。dry-run 无效、domain/profile/owner mismatch
   或 `U+ -> U_safe` 皆不能成功，均 discard。
3. 保留现有 latest-snapshot categorical check：latest observation 若对 current reference 或
   actual 明确 `OCCUPIED` / `OUT_OF_MAP`，即使 frozen candidate 通过，也必须 discard。

独立审查否决了在本阶段抽取或改动 `TubeEpochManager`：stage 已用既有 manager 对 stage-time
latest Runtime 完成 current admissibility；prepare 只对 immutable prepared profile 做已有 Adapter
structural checks、exact Runtime dry-run 和 latest categorical veto。不得在 Adapter 重写或放宽
manager predicate。

### R3 — final exact CAS

在 R2 成功后，保留现有 lock ordering（handoff → phase → Adapter Runtime/CAS），重读并要求：

```text
manager phase tuple     exactly equals the command capture used by R1/R2
live pair               == transaction.expected_pair (exact pointer)
pin lease/capture       still exact and active
authority session       unchanged
Runtime delta/port      bitwise equal to R1's latest capture
candidate identity      unchanged and fully new-owner-owned
```

只有该 predicate 和 single pair CAS 都成功，才发布 new pair 并将现有 completed mailbox 交给
FSM。Runtime 不在此处写入；candidate 保留其 construction-time fields 作为历史 provenance，
而 R1 latest bits 只存在 commit preparation 的 private expected predicate 中。

## 5. 明确拒绝语义

下列任一情况均丢弃这一次 pending handoff，保持 old pair 为唯一 authority，不 partial
install、不 path-only publish、不 retry loop：

- `w_live >= future_seam_w`、profile/domain/horizon 不覆盖；
- pair/generation/session/map/profile/snapshot/pin lease 任何 identity 变化；
- R2 prepared-profile/domain/current exact-PWL witness 不通过，包括 retained delta outside、
  owner/coverage mismatch 或 latest categorical unsafe；
- Runtime dry-run/witness 失败；
- rebase 到 final CAS 之间 Runtime bits、phase 或 pair 发生变化；
- CAS race、pending/completed slot 不可用。

失败只释放 task transaction pin 并交回现有 replan policy；不得添加 timer retry、phase hold、
warm-up/lead gate、new mode/reason/enum/latch/parameter，或以放宽 `current_w < future_seam_w`
规避失败。

## 6. 专属白名单（review 后才生效）

```text
src/swarm_planner/bspline_traj/include/bspline_race/integration/phase_offset_matched_adapter.h
src/swarm_planner/bspline_traj/src/integration/phase_offset_matched_adapter.cpp
src/swarm_planner/bspline_traj/test/phase_offset_matched_adapter_test.cpp
docs/Codex_A6_H2_Runtime_Rebase_Execution_Spec_2026-08-17.md
docs/Codex_A6_H2_Runtime_Rebase_Self_Audit_2026-08-17.md
```

若实现需要 `TubeEpochManager`、GVF manager、CMake、launch/config、Runtime、P1 witness、P2
mailbox/recovery owner、H2 C2 builder、AGENTS 或不在上表的文件，停止。

## 7. 必须新增的回归

1. **runtime drift, safe rebase:** install exact old pair, pin it, stage a genuinely distinct
   C2 new owner/full new profile, then合法地执行 normal Runtime step 使 delta/port 与 stage
   capture bits 不同。若 new profile 在 latest `w` 精确包含该 state 且 all current checks and
   dry-run pass，final rebase must install one new pair. Assert old/new profile pointer differ and
   every connector-interior sample comes from new owner.
2. **three-time bit matrix:** separately alter Runtime bits before stage, between stage and prepare,
   and between prepare and finalize. The first two are rebased and may pass only when the existing
   profile/witness checks pass; the last must fail final CAS with zero publication.
3. **unsafe/domain matrix:** latest retained delta outside exact-PWL bounds; phase at seam; invalid
   profile coverage; and latest categorical unsafe each reject with no publication.
4. **identity matrix:** changed old pair/profile/map/snapshot/generation/session/lease rejects;
   `refreshPairFromTimerEpoch()` keeps its historical Runtime-bit rejection unchanged.
5. **100-cycle zero-pollution:** rejected stage/prepare/finalize across an interleaved 100-cycle
   Runtime sequence does not mutate old pair, candidate profile, live Runtime or publish a path-only
   frontend. A successful pair CAS itself leaves Runtime bits bit-identical until ordinary next
   `complete()`.
6. Re-run current-source focused regression: `phase_offset_matched_adapter_test`,
   `phase_offset_runtime_test`, `phase_offset_tube_epoch_manager_test`,
   `phase_offset_tube_epoch_integration_test`, `continuous_phase_path_test`,
   `gvf_switch_policy_test`, plus `git diff --check` and whitelist audit.

Only after all source tests pass, one task-owned private ROS episode may use the already-authorised
single override `phase_offset_manual_observe_only:=false`. It must demonstrate at least one real
replacement H2 install before old-path exhaustion with goal still distant, and no continuing
`all_candidates_path_end_clamped` HOLD. It must record build IDs, hashes, parameters, bag/logs and
cleanup. A failed episode is evidence, not permission to tune, retry or add a gate.

## 8. Review and stop gates

Independent review completed with the following binding revision: do not touch TubeEpochManager or
GVF manager. Stage retains exact pair/session/pin, reads latest Runtime bits under the existing
Runtime mutex for its prepared build/first dry-run, prepare repeats that latest-bit capture plus
exact `w_live` dry-run, and finalize accepts only those preparation bits. Any later change rejects
without writing Runtime. This is the only implementation authorised by this specification.

Any implementation finding that this adapter-only route weakens a current safety predicate, needs a
new state/recovery action, or cannot preserve the final exact CAS causes `STOP__NO_PRODUCT_CHANGE`.
