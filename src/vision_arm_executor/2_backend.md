# ESTUN 使用的 NX 视觉后端边界

本文只说明当前 ESTUN 运行链路及其复用代码。它不是旧 CR5 行为说明，也不把
`install/` 中可能滞后的构建副本当作源码。

## 1. 代码归属

ESTUN 没有复制一份 NX 协议和任务执行器：

```text
NX TCP :8888
  -> vision_arm_executor/nx_gateway.py
  -> JSON-lines RPC :17881
  -> vision_arm_executor_estun/node.py::EstunVisionExecutor
  -> vision_arm_executor/executor.py::VisionExecutor
  -> ICP / AprilTag 共享后端
  -> vision_arm_executor_estun/robot.py::EstunRobot
  -> estun_codroid_bridge -> Codroid WebSocket
```

- `nx_commands.py`、`nx_gateway.py`：共享的 NX 报文分类和转发。
- `executor.py`、`station_store.py`、`teaching.py`：共享的示教状态机、执行和落盘。
- `vision_arm_executor_estun/node.py`：继承共享 executor，使用
  `data/vision_arm_estun` 和 ESTUN 元数据。
- `vision_arm_executor_estun/robot.py`：唯一的 ESTUN 机械臂运动适配边界。

因此这次协议修复必须改共享文件才会被 ESTUN 使用；ESTUN 的关节运动、IK、
Codroid 调用仍只在 ESTUN 包中实现。

## 2. 唯一且不依赖运行状态的报文分类

| NX 报文 | 固定含义 |
|---|---|
| `mechanical_arm_command`，无 `mapid/poseid`，command 1..6 | 裸机械臂控制 |
| `mechanical_arm_command`，带完整 `mapid+poseid` | 当前站点示教步骤 |
| `type1` + `mapid+poseid` | 开始示教 |
| `type2` + `mapid+poseid` | 完成匹配的活动示教，然后查询；空闲时只查询 |
| `type3` + `mapid+poseid` | 删除整个 UUID 点位 |
| `type3` + `mapid+poseid+command` | 只删除该点位内一个 command 点组 |
| 无 type、只有 `mapid+poseid` | 生产执行该点位的全部已保存点组 |

分类不再读取 `health.teaching_active` 决定一条裸 command 是控制还是示教。因此：

```json
{"type":"mechanical_arm_command","command":1}
```

永远是 `robot_reset`，不会偶发串成示教。示教录点必须同时携带完整站点身份：

```json
{"type":"mechanical_arm_command","mapid":"M","poseid":"P","command":1}
```

只带 `mapid` 或只带 `poseid` 会拒绝；未知的非空 `type` 也会拒绝，不允许回落到
生产执行。

## 3. type1 和 AprilTag 默认参数

视觉示教的推荐请求是：

```json
{"type":"type1","mapid":"M","poseid":"P","point_type":0}
```

`params` 不是必填。AprilTag 缺省值为：

```json
{"tag_id":0,"tag_offset_xyz_mm":[0,0,0]}
```

若调用方显式提供了 `params.tag_id` 或 `params.tag_offset_xyz_mm`，保留调用方的值，
不会用缺省值覆盖。相同字段也兼容放在顶层。

`point_type=0` 是 AprilTag，`point_type=1` 是 ICP；省略 `point_type` 是普通关节点组，
不是自动猜测视觉类型。

## 4. pose、command、Ref 和多点

`poseid` 是站点 UUID，不需要 `group_id`。同一个 `mapid/poseid` 的 manifest 内，
点组以持久化字段 `command=1,2,3...` 区分；`type1` 未指定内部
`task_command` 时，存储层选择下一个未使用编号。

这里存在两个都叫 command 的数字域，必须按报文和状态区分：

1. manifest 顶层 `command`：站点内的点组编号；
2. 活动示教期间 `mechanical_arm_command.command`：当前点组内的 Ref/Work 录制步骤。

这是当前已有实现的命名重叠，不是让网关猜测的依据。网关只按“是否有完整
`mapid+poseid`”确定控制或示教。

视觉点组的录制规则已经是 Ref + 多 Work：

| point_type | command=1 | command=2 | command=3/4/5/... |
|---|---|---|---|
| ICP | 保存 Ref 点云和 Ref 位姿 | 保存 W1 | 继续保存 W2/W3/W4/... |
| AprilTag | 定位并保存 Tag Ref、观察位 | 保存 W1 | 继续保存 W2/W3/W4/... |
| 省略（普通关节） | 保存 W1 | 保存 W2 | 继续保存 W3/W4/W5/... |

每个 Work 都直接相对同一个 Ref，不做链式累积：

```text
ICP: Ref -> W1, Ref -> W2, Ref -> W3
Tag: Tag -> W1, Tag -> W2, Tag -> W3
```

不是 `Ref -> W1 -> W2 -> W3`。

ICP 保存 `T_A_to_B = inv(T_base_flange_ref) * T_base_flange_work`；生产时 ICP
重新对齐 Ref，再从纠正后的 Ref 分别执行全部 Work。AprilTag 保存每个
`T_tag_to_flange_work`；生产时重新检测当前 Tag，再分别计算每个目标。

## 5. type2 完成并查询

```json
{"type":"type2","mapid":"M","poseid":"P"}
```

- 匹配的示教处于活动状态：发送内部 finish（command 0），退出拖拽并保留已保存
  Ref/Work，直接调用一次 ESTUN `reset_robot()`，然后查询站点。复位不是重新发送
  NX command=1，因此不会串成 Ref 录制。
- 当前没有示教：直接查询。
- 活动示教属于另一个 map/pose：明确失败，不能结束别的站点。

只有“活动示教变为完成”的这一次 type2 会复位；完成后重复 type2 只是查询，不会
重复复位。

`type2` 不需要 `point_type`，也不会进入 `vision_point_teach` 的录点分支。因此不会再
出现查询请求报 `point_type is required`。

## 6. type3 删除

删除整个 UUID 点位：

```json
{"type":"type3","mapid":"M","poseid":"P"}
```

只删除一个编号：

```json
{"type":"type3","mapid":"M","poseid":"P","command":2}
```

若目标正处于示教，顺序固定为：退出拖拽并恢复使能、清除活动示教状态、删除
manifest 记录和对应 record 目录。无 command 的 type3 不得落入生产执行。

## 7. 生产入口

生产请求只有这一种隐式形式：

```json
{"mapid":"M","poseid":"P"}
```

它按 manifest 顺序执行该 UUID 点位的所有 command 点组和每组所有 Work。带
`type1/type2/type3/mechanical_arm_command` 的报文都不能进入生产入口。

所有后端的 AprilTag 定位都不再设置 spread 阈值。spread 仍会计算并写入
日志/指标，但只用于诊断，不参与成功/失败判断。

AprilTag 示教与生产执行的新旧帧边界不同：

- 示教的 command 1 先确认机械臂连续停稳 `teaching_stable_sec`（默认
  2 秒），再等待目标 Tag 的下一张新检测帧。
- 新的 `type1` 是绝对切换命令：若另一个点组仍处于示教状态，后端先安全
  退出旧拖动状态。已有至少一个 Work 的旧组保留已落盘数据；没有 Work 的
  不完整旧组会被清理，随后直接进入新 `mapid/poseid/point_type` 的示教，
  不再返回 `another teaching point is active`。
- 生产执行到达观察位后，先静置 `apriltag_post_observation_settle_sec`
  （默认 2 秒），再等待下一张新 Tag 帧。中心未检出时，沿相机
  画面水平方向左右各搜索 `apriltag_search_lateral_mm`（默认 20 mm）；
  每次移动到位后都重新稳定 2 秒。
- 两条路径都不使用 `max_detection_age_sec` 作为成功/失败门控，且都不会
  把停稳前的旧 TF 当成新检测结果。普通 AprilTag 定位等待新帧的上限为
  `apriltag_detection_timeout_sec`（默认 5 秒）；生产执行的中心/左/右搜索
  单个位置只等待 `apriltag_search_detection_timeout_sec`（默认 2 秒）。三个
  位置均无目标 ID 时回到中心观察位并结束该次执行。

## 8. 已兼容与尚未明确的边界

已经兼容：

- 旧 `schema_version=2` 的 `label=P1/P2/...` 可按数字映射为 command 1/2/...；
- 旧 ICP `icp_a/icp_b` 可只读归一化为 Ref + W1，并可查询、删除和执行；
- 删除旧 ICP 时同时清理旧记录中声明的 `record_dir`。

不能无损推断、因此没有伪造：

- 旧 AprilTag 条目只有观察位和 XYZ offset，没有新多点链路所需的完整
  `T_tag_to_flange_work`。它可以被 type2 查询、被 type3 删除，但不能自动转换为
  新格式执行；需要重新示教。
- 顶层点组 command 与组内录制 command 目前同名。代码行为已经确定，但协议若要
  消除命名歧义，需要前后端另行约定新字段；本次不擅自改线协议。

