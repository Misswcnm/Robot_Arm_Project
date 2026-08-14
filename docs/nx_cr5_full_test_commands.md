# NX/CR5 后端全流程测试命令

本文档用于在不启动前端的情况下，用 `printf | nc` 模拟小车 `application`
向本机 NX 兼容网关发送 TCP JSON。测试端口统一为 `8888`。

当前协议基线：

- `command=1`：空闲时 CR5 控制器复位；ICP 示教中记录 A。
- `command=2`：空闲时 CR5 上使能；ICP 示教中记录 B、保存并结束示教。
- `command=3`：CR5 下使能。
- `command=4`：开始拖拽，成功后应为 `RobotMode=6`。
- `command=5`：结束拖拽并恢复使能；ICP 示教中表示取消本次未完成示教。
- `command=6`：清除报警。
- `type1 + mapid + poseid`：开始一次 ICP 示教并进入拖拽。
- `type2 + mapid + poseid`：查询已经完成保存的 ICP 点。
- 只有 `mapid + poseid`：模拟小车到达导航点，按 `nx_routes.json` 执行生产任务。

### application 到 8888 的实际映射

| application 操作 | application 接收的主要字段 | 发到 8888 的 JSON | 本地含义 |
|---|---|---|---|
| `mechanical_arm/send_command` | `command` | `{"type":"mechanical_arm_command","command":N}` | 数字命令 1～6 |
| `tcp/send_demo_point` | `mapId`、`pointId` | `{"type":"type1","mapid":"...","poseid":"..."}` | 开始 ICP 示教 |
| `mechanical_arm/send_demo_point` | `mapid`、`poseid`、可选 `type` | 默认 `{"type":"demo_point_recorded","mapid":"...","poseid":"..."}` | 网关同样按开始 ICP 示教处理 |
| 小车到达导航点 | `mapid`、`poseid`、可选 `label` | `{"mapid":"...","poseid":"...","label":"..."}` | 按路由执行 ICP 或 AprilTag |

`execution_enable/disable` 和显式视觉 action 只用于本地直接测试。application 的
正式导航任务只发送 `mapid/poseid/label`，网关自动在单次任务范围内管理运动许可。

application 的 `mechanical_arm/get_demo_points` 当前扫描的是小车本地旧示教文件，
不会向 8888 发送 `type2`。本文中的 `type2` 是对本机新 ICP 站点文件的直接查询测试。

## 0. 测试前准备

机械臂周围清场，确认急停、示教器和网线正常。执行 `command=4` 前必须托住机械臂，
因为成功后机械臂会进入可手动拖动状态。执行 ICP、AprilTag 真实任务时机械臂会运动。

文档中的命令全部直接写明本机地址 `127.0.0.1 8888`，不依赖 shell 环境变量，
复制到任何新终端都能使用。在小车工控机上执行时，只把每条命令末尾的
`127.0.0.1` 替换成 `192.168.2.103`，端口仍为 `8888`。

所有一次性控制台请求都使用 `nc -N`。8888 为兼容小车而保留 TCP 长连接；
`-N` 会在 `printf` 输入结束后关闭客户端写方向，使网关发送完整 JSON 后立即断开，
避免普通 `nc -w 10` 在动作完成后仍空等 10 秒。

以下示例统一使用：

```text
mapid  = test-map
poseid = test-station
```

整套程序必须使用重新构建后的代码启动：

```bash
cd ~/Robot_Arm_Project
./scripts/local_robot_arm.sh build
./scripts/local_robot_arm.sh all
```

另开终端检查端口：

```bash
nc -vz 127.0.0.1 8888
```

预期：

```text
Connection to ... 8888 port [tcp/*] succeeded!
```

本机还可以直接检查内部 RPC；这条不是小车业务协议，只用于诊断执行器：

```bash
printf '%s\n' \
'{"request_id":"local-health-1","action":"health","params":{},"timeout_sec":2,"dry_run":true}' \
| nc -N -w 3 127.0.0.1 17881
```

预期 `status="succeeded"`，并能看到 `icp_teaching_active` 和
`execution_enabled` 等状态。

## 1. 只读状态检查

### 1.1 小车后端发送的 JSON

```json
{"type":"arm_status","timeout_sec":5,"dry_run":true}
```

### 1.2 控制台测试

```bash
printf '%s\n' \
'{"type":"arm_status","timeout_sec":5,"dry_run":true}' \
| nc -N -w 8 127.0.0.1 8888
```

通过标准：

- `status="succeeded"`；
- `metrics.pose` 是 6 个数；
- `metrics.robot_mode` 不是 `null`。

常见模式：

- `4`：未使能；
- `5`：已使能；
- `6`：拖拽/回拖。

## 2. 数字命令 1～6

数字命令对应 application 的通用 HTTP 接口
`mechanical_arm/send_command`。例如 HTTP body `{"command":4}` 最终会发送
TCP JSON `{"type":"mechanical_arm_command","command":4}`。

为避免状态互相影响，推荐按本节顺序测试。

### 2.1 command=6：清除报警

后端 JSON：

```json
{"type":"mechanical_arm_command","command":6}
```

控制台：

```bash
printf '%s\n' \
'{"type":"mechanical_arm_command","command":6}' \
| nc -N -w 15 127.0.0.1 8888
```

通过标准：`status="succeeded"`、`qingchu="success"`。

### 2.2 command=2：上使能

后端 JSON：

```json
{"type":"mechanical_arm_command","command":2}
```

控制台：

```bash
printf '%s\n' \
'{"type":"mechanical_arm_command","command":2}' \
| nc -N -w 40 127.0.0.1 8888
```

通过标准：`status="succeeded"`、`shangdian="success"`；随后
`arm_status` 应得到 `robot_mode=5`。如果发送前已经是模式 5，应直接返回
`metrics.already_enabled=true`，不得再次发送 `EnableRobot/SpeedFactor/User/Tool`。

### 2.3 command=4：开始拖拽

先托住机械臂，再执行：

```bash
printf '%s\n' \
'{"type":"mechanical_arm_command","command":4}' \
| nc -N -w 40 127.0.0.1 8888
```

通过标准：

- `action="robot_start_drag"`；
- `tuozhuai="success"`；
- `metrics.robot_mode=6`；
- `metrics.dragging=true`。

响应中仍保留旧字段 `zidong="success"`，仅用于兼容旧接收端，不再表示自动模式。

### 2.4 command=5：结束拖拽

```bash
printf '%s\n' \
'{"type":"mechanical_arm_command","command":5}' \
| nc -N -w 40 127.0.0.1 8888
```

通过标准：

- `action="robot_stop_drag"`；
- `quxiaotuozhuai="success"`；
- `metrics.robot_mode=5`；
- `metrics.dragging=false`。

响应中仍保留旧字段 `shoudong="success"`，仅用于兼容旧接收端。

### 2.5 command=3：下使能

```bash
printf '%s\n' \
'{"type":"mechanical_arm_command","command":3}' \
| nc -N -w 20 127.0.0.1 8888
```

通过标准：`status="succeeded"`、`xiadian="success"`；随后
`arm_status` 应得到 `robot_mode=4`。

继续后续测试前重新上使能：

```bash
printf '%s\n' \
'{"type":"mechanical_arm_command","command":2}' \
| nc -N -w 40 127.0.0.1 8888
```

### 2.6 command=1：控制器复位

必须确认当前没有 ICP 示教会话。后端 JSON：

```json
{"type":"mechanical_arm_command","command":1}
```

控制台：

```bash
printf '%s\n' \
'{"type":"mechanical_arm_command","command":1}' \
| nc -N -w 30 127.0.0.1 8888
```

通过标准：`status="succeeded"`、`fuwei="success"`。

这个命令是 CR5 `ResetRobot`，不是移动到某个固定 Home 点。复位后如需继续测试，
重新发送 `command=2`。

## 3. ICP 完整示教：保存 P1

`nx_routes.json` 的 `operation_mode` 必须为 `1`。当前默认配置就是 `1`。

不要把 `type1` 和 `command` 放进同一个 JSON。以下三步是三条独立请求。

### 3.1 开始 test-map/test-station 示教

小车后端 JSON：

```json
{"type":"type1","mapid":"test-map","poseid":"test-station"}
```

控制台：

```bash
printf '%s\n' \
'{"type":"type1","mapid":"test-map","poseid":"test-station"}' \
| nc -N -w 40 127.0.0.1 8888
```

通过标准：

- `action="vision_icp_teach"`；
- `status="succeeded"`；
- `metrics.icp_teaching_active=true`；
- `metrics.icp_teaching_phase="awaiting_a"`；
- `metrics.icp_label="P1"`，如果该站点已有点则可能是下一个 `Pn`；
- `metrics.robot_mode=6`。

此时 `command=4` 是幂等检查，不会重新创建会话：

```bash
printf '%s\n' \
'{"type":"mechanical_arm_command","command":4}' \
| nc -N -w 15 127.0.0.1 8888
```

预期 `teaching_command="drag_already_active"`。

### 3.2 拖到 A，记录模板点云和 A 位姿

将机械臂拖到 ICP 模板位置 A，然后执行：

```bash
printf '%s\n' \
'{"type":"mechanical_arm_command","command":1,"params":{"frames":5}}' \
| nc -N -w 90 127.0.0.1 8888
```

通过标准：

- `status="succeeded"`；
- `teaching_command="record_a"`；
- `metrics.icp_teaching_phase="awaiting_b"`；
- `metrics.record_id` 以 `icp_a-` 开头；
- `artifacts` 中同时有 `.npz` 点云和 `.json` 元数据。

示教会话激活时，`command=1` 不执行复位。

### 3.3 拖到 B，记录戳点并完成保存

将机械臂拖到对应戳点 B，然后执行：

```bash
printf '%s\n' \
'{"type":"mechanical_arm_command","command":2}' \
| nc -N -w 60 127.0.0.1 8888
```

通过标准：

- `status="succeeded"`；
- `teaching_command="record_b"`；
- `metrics.saved_label="P1"` 或实际分配的 `Pn`；
- `metrics.icp_teaching_active=false`；
- `metrics.icp_teaching_phase="idle"`；
- `metrics.robot_mode=5`；
- `artifacts` 中包含站点文件
  `data/vision_arm/tasks/test-map/test-station/test-map_test-station.json`。

示教会话激活时，`command=2` 不执行普通上使能，而是完成 B 点和整个示教事务。

### 3.4 查询刚保存的点

小车后端 JSON：

```json
{"type":"type2","mapid":"test-map","poseid":"test-station"}
```

控制台：

```bash
printf '%s\n' \
'{"type":"type2","mapid":"test-map","poseid":"test-station"}' \
| nc -N -w 15 127.0.0.1 8888
```

通过标准：`status="succeeded"`，`metrics.points` 中存在刚保存的 `P1/Pn`，
并且同时具有 `a_record_id` 和 `b_record_id`。

本机直接查看落盘文件：

```bash
python3 -m json.tool \
data/vision_arm/tasks/test-map/test-station/test-map_test-station.json
```

## 4. ICP 示教取消测试

先开始一个新的示教；已有 P1 时通常会自动分配 P2：

```bash
printf '%s\n' \
'{"type":"type1","mapid":"test-map","poseid":"test-station"}' \
| nc -N -w 40 127.0.0.1 8888
```

不记录或只记录 A，然后发送 `command=5`：

```bash
printf '%s\n' \
'{"type":"mechanical_arm_command","command":5}' \
| nc -N -w 40 127.0.0.1 8888
```

通过标准：

- `status="succeeded"`；
- `teaching_command="abort"`；
- `quxiaotuozhuai="success"`；
- `metrics.icp_teaching_active=false`；
- `metrics.robot_mode=5`；
- 再次执行 `type2` 时，未完成的 P2/Pn 不应出现在 `metrics.points` 中。

## 5. ICP 生产流程

### 5.1 只校验数据，不运动

后端显式 JSON：

```json
{"type":"vision_icp_align_and_move_b","mapid":"test-map","poseid":"test-station","params":{"max_iters":30},"timeout_sec":200,"dry_run":true}
```

控制台：

```bash
printf '%s\n' \
'{"type":"vision_icp_align_and_move_b","mapid":"test-map","poseid":"test-station","params":{"max_iters":30},"timeout_sec":200,"dry_run":true}' \
| nc -N -w 220 127.0.0.1 8888
```

通过标准：`status="done"`、`metrics.dry_run=true`、`metrics.point_count>=1`。
该请求只检查站点记录、点云文件和标定校验和，不驱动机械臂。

只选择某些标签：

```bash
printf '%s\n' \
'{"type":"vision_icp_align_and_move_b","mapid":"test-map","poseid":"test-station","label":"P1","params":{"max_iters":30},"timeout_sec":200,"dry_run":true}' \
| nc -N -w 220 127.0.0.1 8888
```

### 5.2 真实 ICP 对齐并移动到 B

危险：以下命令会真实运动。确认相机点云正常、工作空间无人员、A/B 示教正确后执行。

先开放生产运动许可：

```bash
printf '%s\n' \
'{"type":"execution_enable"}' \
| nc -N -w 10 127.0.0.1 8888
```

通过标准：`status="succeeded"`、`action="execution_enable"`。

执行：

```bash
printf '%s\n' \
'{"type":"vision_icp_align_and_move_b","mapid":"test-map","poseid":"test-station","label":"P1","params":{"max_iters":30},"timeout_sec":200,"dry_run":false}' \
| nc -N -w 240 127.0.0.1 8888
```

通过标准：

- `status="done"`；
- `metrics.converged=true`；
- `metrics.labels` 包含 `P1`；
- `metrics.points` 中具有 ICP 收敛指标和 `T_target`。

测试完成立即关闭生产运动许可：

```bash
printf '%s\n' \
'{"type":"execution_disable"}' \
| nc -N -w 10 127.0.0.1 8888
```

### 5.3 完全模拟小车到达导航点

application 到达点位时没有 `type`，只发送：

```json
{"mapid":"test-map","poseid":"test-station","label":"P1"}
```

控制台：

```bash
printf '%s\n' \
'{"mapid":"test-map","poseid":"test-station","label":"P1"}' \
| nc -N -w 240 127.0.0.1 8888
```

当前 `nx_routes.json` 的默认 `dry_run=true`，所以默认只校验、不运动。需要真实执行时，
只需给该站点增加 `"dry_run":false` 路由并重启网关。网关会自动在任务开始前开放
运动许可，并在成功或失败后关闭。

## 6. AprilTag 正式定位加戳点

该流程不是手动验证 demo，不需要拖拽、手动触碰或分两次调用。正式 action
`apriltag_locate_and_touch` 会在一次任务中完成：

```text
识别 tag36h11:0 → 计算目标法兰位姿 → 移动到预接近点 → 移动到戳点 → 停在戳点
```

当前 Tag 尺寸配置为 `0.152m`。

### 6.1 定位加戳点 dry-run

```bash
printf '%s\n' \
'{"type":"apriltag_locate_and_touch","params":{},"timeout_sec":60,"dry_run":true}' \
| nc -N -w 75 127.0.0.1 8888
```

通过标准：

- `status="done"`；
- `action="apriltag_locate_and_touch"`；
- `metrics.dry_run=true`；
- `metrics.tag_id=0`；
- `metrics.frames` 大于 0；
- 不发送任何机械臂运动命令。

### 6.2 真实定位加戳点

危险：以下命令会直接移动机械臂到 AprilTag 戳点。确认 Tag、TCP、手眼标定、
工作空间和周围环境正确后执行。

```bash
printf '%s\n' \
'{"type":"execution_enable"}' \
| nc -N -w 10 127.0.0.1 8888
```

```bash
printf '%s\n' \
'{"type":"apriltag_locate_and_touch","params":{},"timeout_sec":120,"dry_run":false}' \
| nc -N -w 140 127.0.0.1 8888
```

通过标准：

- `status="done"`；
- `action="apriltag_locate_and_touch"`；
- `message="AprilTag located and touch point completed"`；
- 机械臂依次到达预接近点和戳点，最后停在戳点。

完成后：

```bash
printf '%s\n' \
'{"type":"execution_disable"}' \
| nc -N -w 10 127.0.0.1 8888
```

## 7. application 的 AprilTag 导航点 JSON

当前全局 `operation_mode=1`，所以普通 `mapid+poseid` 默认执行 ICP。若要让某个导航点
执行 AprilTag，在 `src/vision_arm_executor/config/nx_routes.json` 的 `routes`
中增加站点覆盖，例如：

```json
{
  "mapid": "test-map",
  "poseid": "tag-station",
  "mode": 2,
  "dry_run": true,
  "timeout_sec": 120
}
```

重启网关或整套程序后，小车后端发送：

```json
{"mapid":"test-map","poseid":"tag-station"}
```

控制台模拟：

```bash
printf '%s\n' \
'{"mapid":"test-map","poseid":"tag-station"}' \
| nc -N -w 140 127.0.0.1 8888
```

先保持路由 `dry_run=true` 验证。真实执行时将该路由改成 `false`、重启网关并清场；
application 不需要额外发送运动许可命令。

## 8. 预期失败测试

这些测试用于确认后端会拒绝错误流程，而不是误动作。

### 8.1 非法数字命令

```bash
printf '%s\n' \
'{"type":"mechanical_arm_command","command":7}' \
| nc -N -w 10 127.0.0.1 8888
```

预期：`status="failed"` 或 `type="error"`，
`error_code="unsupported_legacy_command"`。

### 8.2 未开始示教就显式记录 A

先确认不存在活动示教，随后：

```bash
printf '%s\n' \
'{"type":"vision_icp_teach","command":1,"timeout_sec":30,"dry_run":false}' \
| nc -N -w 40 127.0.0.1 8888
```

预期：`status="failed"`，提示先发送 `0/start`。

注意：不要用普通 `mechanical_arm_command=1` 做这个失败测试；示教空闲时它表示控制器复位。

### 8.3 未开放运动许可就执行真实 ICP

先发送：

```bash
printf '%s\n' \
'{"type":"execution_disable"}' \
| nc -N -w 10 127.0.0.1 8888
```

再发送：

```bash
printf '%s\n' \
'{"type":"vision_icp_align_and_move_b","mapid":"test-map","poseid":"test-station","label":"P1","timeout_sec":30,"dry_run":false}' \
| nc -N -w 45 127.0.0.1 8888
```

预期：`status="failed"`，消息中包含 `execution_disabled`，机械臂不运动。

### 8.4 不存在的 ICP 站点

```bash
printf '%s\n' \
'{"type":"vision_icp_align_and_move_b","mapid":"missing-map","poseid":"missing-station","timeout_sec":30,"dry_run":true}' \
| nc -N -w 45 127.0.0.1 8888
```

预期：`status="failed"`，提示站点记录或模板不存在。

### 8.5 AprilTag 不可见

移走或遮挡 Tag 后执行正式流程的 dry-run：

```bash
printf '%s\n' \
'{"type":"apriltag_locate_and_touch","params":{},"timeout_sec":30,"dry_run":true}' \
| nc -N -w 45 127.0.0.1 8888
```

预期：`status="failed"`，提示尚未检测到 Tag 或检测已过期，机械臂不运动。

## 9. 汇总问题时需要保存的信息

每个失败问题请同时记录：

1. 执行的完整 `printf | nc` 命令；
2. `nc` 收到的完整 JSON；
3. NX 网关日志中的 `NX request` 和 `NX result`；
4. CR5 驱动对应的 `tcp send cmd`、`tcp recv feedback`；
5. 失败前后的 `arm_status`；
6. 是否处于 ICP 示教、A/B 哪个阶段；
7. 当时机械臂实际 `RobotMode`；
8. 是否执行过 `execution_enable`；
9. 对应 `mapid/poseid/label`；
10. 卡住的大致秒数。

建议按下面格式汇总：

```text
测试编号：
发送 JSON：
开始时间：
收到响应：
耗时：
RobotMode：
机械臂实际表现：
NX request/result 日志：
CR5 send/recv 日志：
是否可重复：
```

## 10. 测试结束后的安全收尾

如果 ICP 示教仍处于活动状态，先取消：

```bash
printf '%s\n' \
'{"type":"mechanical_arm_command","command":5}' \
| nc -N -w 40 127.0.0.1 8888
```

关闭生产运动许可：

```bash
printf '%s\n' \
'{"type":"execution_disable"}' \
| nc -N -w 10 127.0.0.1 8888
```

需要机械臂下使能时：

```bash
printf '%s\n' \
'{"type":"mechanical_arm_command","command":3}' \
| nc -N -w 20 127.0.0.1 8888
```

最后重新执行 `arm_status`，确认状态符合现场要求。
