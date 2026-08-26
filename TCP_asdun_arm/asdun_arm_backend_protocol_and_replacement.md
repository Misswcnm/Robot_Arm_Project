# Asdun/Estun 机械臂后端协议与 API 替换评估

## 1. 先给结论

`TCP_asdun_arm` 不是单纯的“机械臂驱动”，而是一个运行在旧 NX 上的完整业务程序。它同时承担：

1. 对小车后端提供 `TCP 8888` 服务；
2. 把小车消息解释成复位、上/下电、模式切换、示教准备或巡检任务；
3. 通过 Codroid SDK 的 WebSocket 接口控制埃斯顿机械臂；
4. 管理示教文件、RealSense、热成像、补光灯和 YOLO；
5. 把任务完成状态和图片发回小车。

因此，当前本地 `vision_arm_executor + nx_gateway` **不能无条件直接替换整个 Asdun 程序**：

- 可以替换“小车后端连接的 `192.168.2.103:8888` 入口”；
- 可以把 ICP 和 AprilTag 业务接到同一个入口；
- 不能直接替换埃斯顿 Codroid SDK，因为本地项目控制的是 CR5/Dobot，机器人厂家协议不同；
- 也尚未等价实现旧程序的关节点示教文件、`type2` 查询、逐点拍照、图片二进制回传和安全事件。

如果目标只是让现有小车后端调用本地的 ICP、AprilTag，推荐保留 `application/` 不动，让本地 `nx_gateway` 模拟旧 NX 的 `8888` 服务，再配置 `mapid + poseid + label -> action` 路由。若目标是整机完全替换旧 Asdun 巡检能力，则还需要补齐第 10 节列出的兼容项。

## 2. 本文依据的实际运行代码

当前 [`CMakeLists.txt`](../TCP_asdun_arm/CodroidApi/CMakeLists.txt) 真正编译的只有：

- `src/RobotDemo.cpp`
- `src/WebSocketClient.cpp`

`MachineController.cpp`、`RobotTask.cpp`、`EstunController.cpp`、`main.cpp` 等模块化文件在 CMake 中被注释，没有进入当前 `RobotControl`。所以本文以 [`RobotDemo.cpp`](../TCP_asdun_arm/CodroidApi/src/RobotDemo.cpp) 为准；那些同名模块只能视为未启用的重构草稿。

本文是静态代码审查结果，未在埃斯顿真机和原 NX 环境上启动该二进制。

## 3. 实际通信分层

```text
前端
  │ HTTP operation + JSON
  ▼
小车 application / Master_node.py
  │ 长连接 TCP，JSON（没有长度头）
  │ 目标：旧 NX 192.168.2.103:8888
  ▼
Asdun RobotControl / RobotDemo.cpp
  ├─ WebSocket JSON → 埃斯顿控制器 192.168.2.5:9000
  ├─ RealSense → 可见光图片
  ├─ 热成像 RTSP/SDK → 温度和红外图片
  ├─ DO16 → 补光灯
  └─ YOLOv5 → 检测结果图片
```

这里有两套完全不同的 JSON 协议：

- 小车到 NX：项目自定义的业务 JSON，例如 `{"command":2}`、`{"mapid":"m1","poseid":"p1"}`；
- NX 到埃斯顿控制器：Codroid SDK WebSocket JSON，例如 `type=common, action=mov`。

不能把小车发到 `8888` 的 JSON 直接发给埃斯顿的 `9000`。

## 4. 启动时会自动做什么

`RobotControl` 启动后不是只监听端口，它会立即：

1. 创建 `0.0.0.0:8888` TCP 服务；
2. 连接埃斯顿控制器 `192.168.2.5:9000`；
3. 发送上电 `SwitchOn`；
4. 切换自动模式 `ToAuto`；
5. 等待 2 秒；
6. 使用 `movJ` 移动到硬编码关节复位点 `[86, 23, -112, -176, -85, 0]`；
7. 启动任务接收、状态发送、拍照、YOLO、图片发送和安全监控线程。

这意味着直接启动旧程序会让机械臂运动，不能把它当成只读通信服务。

[`runDemo.sh`](../TCP_asdun_arm/CodroidApi/runDemo.sh) 只启动 `build/RobotControl`，**不会启动**示教文件监控脚本 `auto_process.py`。需要旧示教流程时，两个进程都必须运行。

## 5. 小车发给 Asdun：收到什么、做什么、返回什么

### 5.1 总表

| 收到的 TCP JSON | 实际动作 | 返回小车 | 是否立即返回 |
|---|---|---|---|
| 含 `command`，值为 1～6 | 执行机器人状态命令 | 命令专用 JSON | 否，分离线程执行后返回 |
| `type="type1"` + `mapid` + `poseid` | 记录当前示教归属并创建目录 | **无 TCP 回包** | 不适用 |
| `type="type2"` + `mapid` + `poseid` | 查询已解析的关节点 JSON | 点位数组或 `error` | 是 |
| 没有上述类型，含 `mapid` + `poseid` | 执行该导航点的关节序列并拍照 | 成功时 `status="done"`，失败通常无回包 | 任务全部结束后 |
| 未知 `command` | 只写日志 | **无回包** | 不适用 |
| 无法解析的 JSON | 只写错误日志并断开本次接收循环 | **无结构化失败回包** | 不适用 |

任何含 `command` 字段的消息都会优先进入命令分支，`type` 在该分支中没有实际判定作用。小车通常仍发送 `type="mechanical_arm_command"`，这是应用层约定，不是 Asdun 的必要条件。

### 5.2 命令 1～6

#### 命令 1：复位

收到：

```json
{"type":"mechanical_arm_command","command":1}
```

执行：调用 `resetRobot()`，本质是 `movJ` 到固定关节点 `[86,23,-112,-176,-85,0]`，并循环查询当前关节角，直到每个关节误差不大于 `0.01`。

返回：

```json
{"command":1,"fuwei":"success"}
```

或：

```json
{"command":1,"fuwei":"failed"}
```

注意：它不是控制器原生的 Reset，也不是 `goHome()`。

#### 命令 2：上电/上使能

执行：Codroid `sendUserCommand(SwitchOn)`。

成功返回：

```json
{"command":2,"shangdian":"success"}
```

失败返回中的字段存在原代码拼写错误：

```json
{"command":2,"sahngdian":"failed"}
```

#### 命令 3：下电/下使能

执行：Codroid `sendUserCommand(SwitchOff)`。

返回：

```json
{"command":3,"xiadian":"success"}
```

或 `xiadian="failed"`。

#### 命令 4：自动模式

执行：Codroid `sendUserCommand(ToAuto)`。

返回：

```json
{"command":4,"zidong":"success"}
```

或 `zidong="failed"`。

#### 命令 5：手动模式

执行：Codroid `sendUserCommand(ToReady)`。

返回：

```json
{"command":5,"shoudong":"success"}
```

或 `shoudong="failed"`。

#### 命令 6：清除报警

执行：Codroid `sendUserCommand(ClearWarning)`，其枚举数值为 `501`。

返回：

```json
{"command":6,"qingchu":"success"}
```

或 `qingchu="failed"`。

这里容易混淆的两组数字含义不同：

- 小车协议的 `command=1..6` 是 `RobotDemo.cpp` 自己定义的业务编号；
- Codroid 的 `SwitchOn=1`、`SwitchOff=2`、`ToReady=3`、`ToAuto=5`、`ClearWarning=501` 是厂家控制器命令值。

`RobotDemo.cpp` 完成了两者之间的映射，并不是简单地把小车的 1～6 原样转发给机器人。

### 5.3 `type1`：示教准备，不是 GetPose 录点

收到：

```json
{"type":"type1","mapid":"map_1","poseid":"nav_1"}
```

Asdun 只做两件事：

1. 写入 `current_task.json`，记录当前 `mapid`、`poseid` 和时间戳；
2. 创建 `src/<mapid>/<poseid>/` 目录。

它此时不会调用 `getJointPosition()`，不会记录当前位姿，也不会向小车返回“示教完成”。真实示教依赖另一个人工流程：

```text
小车发 type1
  → NX 只记录当前 mapid/poseid
  → 操作员在埃斯顿/Codroid 示教界面完成示教并导出 .crp
  → .crp 出现在 /home/nvidia/Downloads
  → auto_process.py 发现新文件
  → jiexi.py 提取 type=points 的 children
  → 保存到 src/<mapid>/<poseid>/<mapid>_<poseid>.json
```

生成的点位数据格式为：

```json
[
  {
    "label": "point_1",
    "joints": {
      "jntpos1": 1.0,
      "jntpos2": 2.0,
      "jntpos3": 3.0,
      "jntpos4": 4.0,
      "jntpos5": 5.0,
      "jntpos6": 6.0
    }
  }
]
```

这个流程存在关联风险：`auto_process.py` 只读取当时最新的 `current_task.json`，因此连续发送多个 `type1` 后再导出 `.crp`，可能把示教文件归到错误的 `mapid/poseid`。

### 5.4 `type2`：读取示教结果

收到：

```json
{"type":"type2","mapid":"map_1","poseid":"nav_1"}
```

执行：读取：

```text
src/map_1/nav_1/map_1_nav_1.json
```

成功返回：直接返回上一节所示的 JSON 数组，末尾加换行，不包装 `type` 或 `status`。

文件不存在时返回：

```json
{"error":"json file not found"}
```

读取异常时返回：

```json
{"error":"exception while reading json"}
```

### 5.5 正常巡检任务：到点后运动并拍照

可见光任务示例：

```json
{"mapid":"map_1","poseid":"nav_1","label":"point_1,point_2","led":"1"}
```

热成像任务示例：

```json
{"mapid":"map_1","poseid":"nav_1","label":"point_1","rc":"1"}
```

执行流程：

1. 读取 `src/<mapid>/<poseid>/<mapid>_<poseid>.json`；
2. 如果有逗号分隔的 `label`，按请求中的 label 顺序选择点；缺失的 label 被跳过；
3. 如果没有 `label`，按 label 的字符串字典序执行所有点，而不是严格按 JSON 文件顺序；
4. 切换自动模式；
5. 每个点调用 `movJ(point, 30, 30)`；
6. 轮询当前关节角判断到位；
7. `rc="1"` 时拍热成像，否则拍 RealSense 可见光；
8. 可见光且 `led="1"` 时通过 DO16 开补光灯，拍照后总会尝试关闭 DO16；
9. 所有点完成后移动到固定复位点；
10. 返回完成状态。

成功返回：

```json
{"mapid":"map_1","poseid":"nav_1","status":"done"}
```

关键失败语义：点位文件不存在、点位为空、运动失败、拍照失败等路径大多只在 NX 控制台写日志，**没有稳定的 TCP `failed` 回包**。小车后端因此只能等到自己的 200 秒超时。

## 6. Asdun 主动发给小车的消息

### 6.1 文本 JSON 状态

所有普通状态都是 UTF-8 JSON，末尾加 `\n`。包括：

- 命令 1～6 的结果；
- `type2` 点位数组或错误；
- 巡检任务的 `status="done"`；
- 安全状态变化。

安全报警：

```json
{
  "type":"safety_alert",
  "error":true,
  "message":"机器人处于错误状态",
  "status_flag":32768,
  "timestamp":1750000000000
}
```

安全恢复：

```json
{
  "type":"safety_recovery",
  "error":false,
  "message":"机器人错误状态已清除",
  "status_flag":0,
  "timestamp":1750000000000
}
```

安全线程每 500 ms 查询状态标志，第 15 位作为错误位。

### 6.2 图片协议

图片和 JSON 状态复用同一个 TCP 连接。发送格式是：

```text
一行 JSON 元数据 + "\n"
紧接着 filesize 字节的原始二进制图片
```

元数据示例：

```json
{
  "type":"raw",
  "mapid":"map_1",
  "poseid":"nav_1",
  "filename":"capture_120000.png",
  "filesize":123456
}
```

`type` 可能为：

- `raw`：RealSense 原图；
- `result`：YOLO 结果图；
- `rc`：正常热成像；
- `error_rc`：最高温度超过 50 ℃ 的热成像。

二进制数据后没有额外结束符，接收端必须严格按 `filesize` 读取，再继续解析下一行 JSON。

## 7. 小车 application 如何触发这些行为

小车的 `master_node_subscriber.py` 创建 TCP 客户端，默认连接旧 NX `192.168.2.103:8888`。前端 HTTP 请求由 `Master_node.py` 转换为上述 TCP JSON。

| 前端/HTTP operation | 小车发往 8888 的核心 JSON | Asdun 行为 |
|---|---|---|
| `tcp/send_demo_point` | `{"type":"type1","mapid":...,"poseid":...}` | 正确进入示教准备 |
| `tcp/send_demo_point_direct` | 默认 `type="demo_point_recorded"` | **默认不兼容**；Asdun 会把它当普通执行任务 |
| `mechanical_arm/send_demo_point` | 默认 `type="demo_point_recorded"` | **默认不兼容**；只有前端显式传 `type1` 才进入示教准备 |
| `mechanical_arm/send_command` | `{"type":"mechanical_arm_command","command":1..6}` | 执行命令 1～6 |
| `mechanical_arm/send_manual` | 最终发送命令 5 | 切换埃斯顿手动模式 |
| `mechanical_arm/send_auto` | 最终发送命令 4 | 切换埃斯顿自动模式 |
| `mechanical_arm/send_clear_alarm` | 最终发送命令 6 | 清除警告 |
| `mechanical_arm/send_enable` | 最终发送命令 2 | 上电 |
| `mechanical_arm/send_disable` | 最终发送命令 3 | 下电 |
| 导航到点触发 | `{"mapid":...,"poseid":...,"label":...,"rc":...,"led":...}` | 关节序列 + 拍照 + 回 `done` |
| `mechanical_arm/check_connection` | 不发送业务命令，只检查/重连 TCP | 只能证明 8888 TCP 可连，不能证明机器人可运动 |

`mechanical_arm/get_demo_points` 目前主要查询小车本地保存目录，不等同于向 Asdun 发送 `type2`。因此旧代码中同时存在“NX 端点位文件”和“小车端点位副本”两套数据路径。

小车收到 `status="done"` 后才唤醒等待中的导航点机械臂任务。命令接口的 HTTP 成功通常只代表 TCP `sendall()` 成功，并不代表机械臂已经执行成功；异步命令结果主要由接收线程打印/通用处理。

## 8. Asdun 调用了哪些厂家机械臂 API

| Asdun 业务 | Codroid C++ 调用 | 发到控制器的 action/data | 返回判定 |
|---|---|---|---|
| 测试连接/读模式 | `getRobotState()` | `common/getparam`, `Robot/Control/state` | 外层 WebSocket `code=200` 且内层 `data.code=0` |
| 上电 | `sendUserCommand(SwitchOn)` | `common/setparam`, `Robot/Control/command=1` | 同上 |
| 下电 | `sendUserCommand(SwitchOff)` | `Robot/Control/command=2` | 同上 |
| 手动模式 | `sendUserCommand(ToReady)` | `Robot/Control/command=3` | 同上 |
| 自动模式 | `sendUserCommand(ToAuto)` | `Robot/Control/command=5` | 同上 |
| 清除警告 | `sendUserCommand(ClearWarning)` | `Robot/Control/command=501` | 同上 |
| 关节运动 | `movJ()` | `common/mov`, `type=movj`, `target.type=apos` | 内层 `data.code=0` |
| 查询当前关节角 | `getJointPosition()` | `common/getCurAPos` | 返回 jntpos1～jntpos6 |
| 读错误标志 | `getRobotStateFlag()` | `common/getRobotStates` | 读取 `statusFlag` |
| 补光灯 | `setDO(16, 0/1)` | `common/setDO` | 内层 `data.code=0` |

Codroid 请求通用封装为：

```json
{"id":1,"type":"common","action":"...","data":{}}
```

控制器响应必须带相同 `id`；外层 `code=200` 才被 SDK 视为通信成功。之后各 API 还会检查响应 `data.code` 是否为 0。SDK 内部区分 `Timeout`、`QueueFull`、`NetError` 和 `RequestFailed`，但 `RobotDemo.cpp` 对小车基本只压缩为布尔成功/失败。

## 9. 与本地 `vision_arm_executor + nx_gateway` 的逐项对比

### 9.1 已经能替换的部分

本地 `nx_gateway.py` 可以监听 `192.168.2.103:8888`，让未改动的小车 application 仍作为 TCP 客户端连接。它已经支持：

- 旧 `mechanical_arm_command` 入口；
- `mapid/poseid/label` 路由；
- `type1` 和 `demo_point_recorded` 的示教路由；
- 明确动作：`arm_status`、ICP A/B 录制与对齐、AprilTag 定位/验证/抓取；
- 任务互斥、超时、失败回包、执行许可和认证；
- 成功执行任务后向旧 application 返回 `status="done"`。

当前 [`nx_routes.json`](../src/vision_arm_executor/config/nx_routes.json) 的 `teaching_routes` 和 `routes` 都还是空数组，所以**显式 action 可以调用，但普通导航点消息还没有业务映射，不能投入自动联动**。

### 9.2 不能直接等价替换的部分

| 能力 | 旧 Asdun | 当前本地实现 | 是否等价 |
|---|---|---|---|
| TCP 8888 入口 | 有 | 有 | 基本等价 |
| 命令 2/3/6 | 埃斯顿上/下电、清警告 | CR5/Dobot 对应控制动作 | 业务名称相同，硬件实现不同 |
| 命令 1 | `movJ` 到固定关节点 | 当前是机器人 Reset 动作 | **不等价** |
| 命令 4/5 | 控制器自动/手动模式 | 当前映射为本地执行许可开/关 | **不等价** |
| 命令返回 JSON | `fuwei/shangdian/...` | 统一 `type=response` | 严格协议不等价；现 application 通常不等待该回包 |
| `type1` 示教 | 关联外部 `.crp` 文件 | 路由到 ICP/视觉动作 | **不是同一种示教** |
| GetPose 录点 | 旧程序没有 | 可由 CR5 后端新增/规范化 | 尚未作为旧协议功能提供 |
| `type2` 点位查询 | 返回关节点数组 | 未实现 | 不等价 |
| 普通巡检 | 读取关节序列、逐点 movJ、拍照 | 路由到 ICP/AprilTag 动作 | 业务按路由替代，不是原样等价 |
| 图片回传 | 元数据行 + 二进制图片 | RPC 只返回 artifact 路径 | 不等价 |
| 热成像/DO16/YOLO | 内置 | 未集成 | 不等价 |
| 安全异步事件 | `safety_alert/recovery` | 任务安全检查和结构化失败 | 消息协议不等价 |
| 失败语义 | 经常无回包，依赖小车超时 | 明确 `failed/error_code` | 本地更规范，但不是字节级兼容 |

### 9.3 “直接替换 API”要分三种理解

1. **替换 `application -> NX:8888`：可以作为入口替换，但需要路由和兼容补齐。** 小车 IP、端口和 application 可以不改。
2. **用 CR5/Dobot API 直接替换 Codroid API：不可以。** `SwitchOn/ToAuto/mov/getCurAPos` 是埃斯顿协议，本地 ROS2 service/dashboard 是 Dobot 协议，必须经过后端适配器，不能改个函数名就替换。
3. **用 ICP/AprilTag 替换旧关节点巡检业务：可以，但这是业务升级，不是 API 等价替换。** 必须明确每个 `mapid/poseid/label` 应触发 ICP 录制、ICP 对齐、AprilTag 定位还是抓取。

## 10. 推荐的落地替换方式

保持小车 `application/` 原封不动，在本地实现一个明确的“旧 NX 兼容层”，分两类业务：

### 10.1 视觉业务路由

在 `nx_routes.json` 中配置实际地图点位，例如：

```json
{
  "schema_version": 1,
  "defaults": {"timeout_sec": 200, "dry_run": false},
  "teaching_routes": [
    {"mapid":"map_1","poseid":"icp_a","action":"vision_icp_record_a"},
    {"mapid":"map_1","poseid":"icp_b","action":"vision_icp_record_b",
     "params":{"manual_positioned":true}}
  ],
  "routes": [
    {"mapid":"map_1","poseid":"icp_work","action":"vision_icp_align"},
    {"mapid":"map_1","poseid":"tag_locate","action":"apriltag_locate"},
    {"mapid":"map_1","poseid":"tag_pick","action":"apriltag_pick"}
  ]
}
```

示例只说明映射方式；真实 `mapid/poseid`、是否移动、是否抓取和 `dry_run` 必须现场确认后再写入生产配置。

### 10.2 如果还要完整保留旧巡检能力

需要继续实现：

1. 明确的 `record_joint_pose`：调用 CR5 `GetPose` 或读取关节角，返回并持久化结构化点位；
2. `list/get_demo_points`：替代旧 `type2` 和小车本地文件扫描；
3. 关节序列执行后端，并定义速度、坐标系、User/Tool、到位容差和超时；
4. RealSense/热成像/补光灯/YOLO 的统一相机后端；
5. artifact 下载或旧式图片流兼容；
6. `robot_auto`、`robot_manual` 和“执行许可”分成不同动作，不能继续混用命令 4/5；
7. 把命令 1 明确为 `move_home` 或 `controller_reset`，不能都叫“复位”；
8. 对所有失败路径返回结构化 `failed`，并让小车不再只能等 200 秒超时。

## 11. 原 Asdun 实现中需要注意的工程风险

- TCP 接收每次只 `recv(1024)` 一次就直接 `json::parse`，不支持 JSON 分包、粘包或超过 1024 字节的请求；小车端又不加换行和长度头，协议依赖一次 `sendall` 恰好对应一次 `recv`，并不可靠。
- 只保存一个客户端 socket，状态、图片、命令结果和安全事件全部复用该连接。
- 发送状态时如果客户端断开，发送线程会进入阻塞式 `accept()`。
- 多数执行失败没有网络失败回包。
- 命令用 detached thread 并发执行，没有统一的机械臂运动互斥；复位、模式切换与巡检任务可能竞争同一机器人。
- `isMovementComplete()` 内部已经循环等待约 20 秒，任务层外面又套了一层循环，最坏情况下可能造成远超注释“10 秒”的等待。
- 可见光拍照失败后，任务仍循环等待 `isImageCaptured()`，没有超时。
- 启动程序自动上电、自动模式并移动，运维风险较高。
- 源码和构建脚本硬编码 NX 用户目录、ARM64 RealSense 库路径、机器人/相机 IP；热成像连接信息也硬编码在源码中，生产部署应改为环境变量或受控配置，并轮换已经进入源码历史的凭据。
- `type1` 没有请求 ID 和示教完成确认，`.crp` 与 map/pose 的关联容易被后续请求覆盖。

## 12. 建议的验收边界

在宣布“可以替换旧 Asdun”前，至少逐项验证：

1. 小车能持续连接本地 `192.168.2.103:8888`；
2. 命令 1～6 的真实语义已经逐项确认，而不仅是收到成功 JSON；
3. 每个生产 `mapid/poseid/label` 都有唯一命中路由；
4. ICP A、B 的录制与生产对齐不会被普通导航到点误触发；
5. AprilTag 定位与抓取分成两步，定位缓存过期和机械臂漂移会拒绝抓取；
6. 执行成功才返回 `status="done"`，失败返回明确 `error_code`；
7. 小车断线重连、重复请求、任务超时和急停恢复均通过测试；
8. 如前端仍需要旧巡检图片，已经提供图片下载接口或兼容二进制流。

完成这些以后，本地服务可以替换旧 NX 的后端入口；在此之前，只能说 ICP/AprilTag 的独立 RPC 和 8888 网络通道已经可用，不能说整个旧 Asdun 业务已经等价替换。
