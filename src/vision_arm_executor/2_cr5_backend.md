# CR5 视觉机械臂后端协议

## 1. 点位类型

| `point_type` | 点位 |
|---:|---|
| `0` | AprilTag |
| `1` | ICP |

`point_type` 使用独立字段；`label` 使用 `P1/P2/...`

```text
mapid
└── poseid
    ├── P1  point_type=1  ICP
    ├── P2  point_type=0  AprilTag
    └── P3  point_type=1  ICP
```

## 2. 示教触发

### 2.1 ICP
```json
{"type":"type1","mapid":"","poseid":"","point_type":1}
```

```text
开始示教
└── {"type":"type1","mapid":"","poseid":"","point_type":1}
    └── 本地自动生成未使用的点位编号（P1/P2/...），进入拖拽状态
        ├── command=1：记录 A 点云和 A 法兰位姿
        ├── command=2：记录 B 法兰位姿并保存
        └── command=5：取消示教，不保存
```

记录 A：

```json
{"type":"mechanical_arm_command","command":1}
```

记录 B：

```json
{"type":"mechanical_arm_command","command":2}
```

### 2.2 AprilTag
```json
{
      "type":"type1",
      "mapid":"",
      "poseid":"",
      "point_type":0,
      "params":{
        "tag_id":7,
        "tag_offset_xyz_mm":[0,0,0]
      }
}
```
```text
开始示教
└── {
      "type":"type1",
      "mapid":"",
      "poseid":"",
      "point_type":0,
      "params":{
        "tag_id":7,
        "tag_offset_xyz_mm":[0,0,0]
      }
    }
    └── 本地自动生成未使用的点位编号（P1/P2/...），进入拖拽
        ├── 人工将机械臂拖到 AprilTag 观察位置
        ├── command=1
        │   ├── GetPose 记录当前法兰观察位姿
        │   ├── 保存 tag_id
        │   ├── 保存 Tag 坐标系下的戳点偏移
        │   ├── 保存点位
        │   └── 结束拖拽并恢复使能
        └── command=5：取消示教，不保存
```

记录 AprilTag 戳点：

```json
{"type":"mechanical_arm_command","command":1}
```

`tag_offset_xyz_mm` 使用 AprilTag 坐标系，单位为毫米；戳 Tag 中心时配置为
`[0,0,0]`。

`tag_id` 必填。相机视野中可以同时出现多个 AprilTag；生产执行时本地只使用
该点示教记录中保存的 ID，其他 ID 不参与本次定位。

## 3. 查询点位

请求：

```json
{"type":"type2","mapid":"","poseid":""}
```

返回：

```json
{
  "status": "succeeded",
  "metrics": {
    "mapid": "",
    "poseid": "",
    "points": [
      {"label": "P1", "point_type": 1},
      {"label": "P2", "point_type": 0}
    ]
  }
}
```

## 4. 任务执行触发

application 发送给本地后端：

```json
{
  "mapid": "",
  "poseid": ""
}
```

```text
收到 mapid/poseid
└── 读取该站点完整点位清单
    ├── P1：读取本地点位的 point_type
    │   ├── 1 → 加载 ICP A/B → MovJ回示教A → ICP回正 → 执行相对A→B
    │   └── 0 → 到观察位 → 识别 Tag → 相对偏移戳点
    ├── P2：按保存的 point_type 执行
    ├── ...
    └── 全部点完成 → 返回一次任务完成
```

```text
目标针尖位置 = T_base_tag_current × [offset_x, offset_y, offset_z, 1]
目标法兰姿态 = observe_flange_pose 中记录的姿态
目标法兰位置 = 目标针尖位置 - 法兰姿态旋转后的 TCP 针尖偏移
```

生产请求不发送 `label` 或 `point_type`。`point_type` 只在示教时传入并保存
到点位记录；生产时由本地逐点读取。不再通过 `nx_routes.json` 选择 ICP
或 AprilTag。

## 5. 本地数据格式

站点清单：

```json
[
  {
    "schema_version": 2,
    "mapid": "",
    "poseid": "",
    "label": "P1",
    "point_type": 1,
    "icp_a": {"record_id": "icp_a-..."},
    "icp_b": {"record_id": "icp_b-..."}
  },
  {
    "schema_version": 2,
    "mapid": "",
    "poseid": "",
    "label": "P2",
    "point_type": 0,
    "apriltag": {
      "tag_id": 7,
      "observe_flange_pose": [230.7, 411.6, 376.6, 177.5, 69.1, -112.1],
      "tag_offset_xyz_mm": [0.0, 0.0, 0.0]
    }
  }
]
```

AprilTag 点位直接保存在站点清单中，不额外生成矩阵记录文件。

## 6. 本地存储路径

```text
data/vision_arm/
├── tasks/
│   └── <mapid>/
│       └── <poseid>/
│           └── <mapid>_<poseid>.json
└── records/
    └── <mapid>_<poseid>_<label>_<point_type>/
        ├── icp_a.json
        ├── icp_a.npz
        └── icp_b.json
```

## 7. 前后端指令汇总

### 7.1 对接约定

| 项目 | 约定 |
|---|---|
| application → NX | TCP `8888`，UTF-8 JSON，一次请求等待一次最终响应 |
| 前端 → application | HTTP，使用下表中的 `mechanical_arm/*` 接口 |
| `type` | 示教兼容 `1/"1"/"type1"`；查询兼容 `2/"2"/"type2"` |
| 站点标识 | `mapid + poseid` |
| 点位编号 | 本地自动生成 `P1/P2/...`，前端不分配 |
| `point_type` | 仅开始示教时发送：`0=AprilTag`，`1=ICP` |
| `tag_id` | AprilTag 示教必填 |
| 生产执行 | 只发送 `mapid + poseid` |
| 成功判断 | 普通请求 `status=succeeded`；生产任务 `status=done` |
| 失败判断 | `status=failed/timeout/cancelled`，读取 `error_code/message` |

文档中的 `mapid:""`、`poseid:""` 是动态值占位，实际发送时必须填入当前
地图和导航站点 ID。

下一条示教指令必须等上一条返回成功后再发送。
TCP JSON 使用换行符结尾；整站生产任务统一使用 `300s` 超时。

### 7.2 ICP 示教

开始示教并进入拖拽：

```json
{"type":1,"mapid":"","poseid":"","point_type":1}
```

记录 A，记录后继续拖拽：

```json
{"type":"mechanical_arm_command","command":1}
```

记录 B，随后停止拖拽、恢复使能并保存：

```json
{"type":"mechanical_arm_command","command":2}
```

### 7.3 AprilTag 示教

开始示教并进入拖拽：

```json
{
  "type":1,
  "mapid":"",
  "poseid":"",
  "point_type":0,
  "params":{
    "tag_id":7,
    "tag_offset_xyz_mm":[0,0,0]
  }
}
```

记录法兰观察位姿，随后停止拖拽、恢复使能并保存：

```json
{"type":"mechanical_arm_command","command":1}
```

`tag_offset_xyz_mm` 省略时使用 `[0,0,0]`。

### 7.4 取消示教

ICP 和 AprilTag 共用：

```json
{"type":"mechanical_arm_command","command":5}
```

停止拖拽、恢复使能，不保存当前未完成点位。

### 7.5 查询站点点位

```json
{"type":2,"mapid":"","poseid":""}
```

成功响应中的 `metrics.points` 返回该站点全部点位及其 `label/point_type`。
这是只读查询，不得触发 ICP 或机械臂运动。

### 7.6 生产执行

```json
{"mapid":"","poseid":""}
```

本地按站点清单顺序执行全部 `P1/P2/...`。前端不得发送 `label`、
`point_type` 或 `tag_id`。全部点成功后返回：

ICP 点固定执行顺序为：MovJ 到保存的绝对 A 法兰位姿，执行 ICP 视觉回正，
再将保存的 `T_A_to_B = inv(T_base_flange_A) × T_base_flange_B`
应用到回正后的当前法兰位姿。ICP 残差一次达标即判定收敛。

```json
{"type":"response","status":"done","mapid":"","poseid":""}
```

### 7.7 机械臂状态

```json
{"type":"arm_status","timeout_sec":5,"dry_run":true}
```

成功响应中的 `metrics.robot_mode` 为机械臂模式，
`metrics.pose` 为当前 `[x,y,z,rx,ry,rz]`。

### 7.8 空闲状态机械臂命令

以下命令只在没有示教会话时使用：

| `command` | 动作 | 成功响应兼容字段 |
|---:|---|---|
| `1` | 复位 | `fuwei=success` |
| `2` | 使能 | `shangdian=success` |
| `3` | 下使能 | `xiadian=success` |
| `4` | 开始拖拽 | `tuozhuai=success` |
| `5` | 取消拖拽 | `quxiaotuozhuai=success` |
| `6` | 清除报警 | `qingchu=success` |

统一格式：

```json
{"type":"mechanical_arm_command","command":1}
```

示教会话中 `command=1/2/5` 由示教状态机处理，不执行表中的空闲动作：

| 示教状态 | `command=1` | `command=2` | `command=5` |
|---|---|---|---|
| ICP | 记录 A | 记录 B并保存 | 取消且不保存 |
| AprilTag | 记录观察位并保存 | 不允许 | 取消且不保存 |

示教期间不要发送 `command=3/6`。

### 7.9 通用响应

成功：

```json
{
  "type":"response",
  "status":"succeeded",
  "action":"vision_point_teach",
  "message":"",
  "error_code":"",
  "metrics":{},
  "artifacts":[]
}
```

失败：

```json
{
  "type":"response",
  "status":"failed",
  "action":"",
  "message":"失败原因",
  "error_code":"错误码",
  "metrics":{},
  "artifacts":[]
}
```

请求格式错误时 `type` 可能为 `error`。前后端统一以 `status` 为准，不以
`type` 判断业务是否成功。

### 7.10 application HTTP 映射

| 前端操作 | HTTP operation | 请求体 | application 发给 NX |
|---|---|---|---|
| 开始示教 | `mechanical_arm/send_demo_point` | `mapid/poseid/point_type/params` | `type=1` |
| 记录或控制 | `mechanical_arm/send_command` | `command` | `type=mechanical_arm_command` |
| 查询站点点位 | `mechanical_arm/get_demo_points` | `mapid/poseid` | `type=2` |
| 生产执行 | 小车到站自动触发 | 无前端机械臂请求 | 仅 `mapid/poseid` |

三个 HTTP 接口均等待 NX 最终响应后再返回。前端不直接连接 `8888`。

## 8. 小车端测试指令

### 8.1 NX 状态

```bash
printf '%s\n' '{"type":"arm_status","timeout_sec":5,"dry_run":true}' | nc -N -w 8 192.168.2.20 8888
```

### 8.2 application 连接

```bash
curl -sS --max-time 8 -X POST -H 'Content-Type: application/json' -d '{}' http://127.0.0.1:1819/robot/mechanical_arm/check_connection
```

### 8.3 查询点位

```bash
curl -sS --max-time 20 -X POST -H 'Content-Type: application/json' -d '{"mapid":"test01","poseid":"test01"}' http://127.0.0.1:1819/robot/mechanical_arm/get_demo_points
```

### 8.4 ICP 示教

开始：

```bash
curl -sS --max-time 70 -X POST -H 'Content-Type: application/json' -d '{"mapid":"test01","poseid":"test01","point_type":1}' http://127.0.0.1:1819/robot/mechanical_arm/send_demo_point
```

记录 A：

```bash
curl -sS --max-time 100 -X POST -H 'Content-Type: application/json' -d '{"command":1}' http://127.0.0.1:1819/robot/mechanical_arm/send_command
```

记录 B：

```bash
curl -sS --max-time 100 -X POST -H 'Content-Type: application/json' -d '{"command":2}' http://127.0.0.1:1819/robot/mechanical_arm/send_command
```

### 8.5 AprilTag 示教

开始：

```bash
curl -sS --max-time 70 -X POST -H 'Content-Type: application/json' -d '{"mapid":"test01","poseid":"test01","point_type":0,"params":{"tag_id":0,"tag_offset_xyz_mm":[0,0,0]}}' http://127.0.0.1:1819/robot/mechanical_arm/send_demo_point
```

记录：

```bash
curl -sS --max-time 100 -X POST -H 'Content-Type: application/json' -d '{"command":1}' http://127.0.0.1:1819/robot/mechanical_arm/send_command
```

### 8.6 取消示教

```bash
curl -sS --max-time 40 -X POST -H 'Content-Type: application/json' -d '{"command":5}' http://127.0.0.1:1819/robot/mechanical_arm/send_command
```

### 8.7 生产执行

```bash
printf '%s\n' '{"mapid":"test01","poseid":"test01"}' | nc -N -w 320 192.168.2.20 8888
```

## 9. application 改动

接口地址不变。

| 原接口/链路 | 原行为 | 当前行为 |
|---|---|---|
| `mechanical_arm/send_demo_point` | 透传旧 `type`，发送即成功 | 接收 `mapid/poseid/point_type/params`，转为 `type=1`，等待 NX 结果 |
| `mechanical_arm/get_demo_points` | 扫描小车本地旧 JSON | 向 NX 发送 `type=2`，返回站点点位 |
| `mechanical_arm/send_command` | 异步发送；`4/5=自动/手动` | 发送 `mechanical_arm_command`；`4/5=开始/取消拖拽`；等待 NX 结果 |
| `send_manual/auto/clear_alarm/enable/disable` | 分别发送旧命令 | 兼容保留，统一转到 `send_command` |
| 导航到站执行 | 发送 `mapid/poseid/rc/led/label` | 只发送 `mapid/poseid`，等待最终状态 |
| NX 地址 | 固定 `192.168.2.103:8888` | 可配置，默认 `192.168.2.20:8888` |
| `MechanicalArmAction` | 缺少服务即启动失败 | 服务可选，不影响 HTTP/TCP/动作话题 |

TCP 一行一个 JSON；生产超时 `300s`；`iot_bridge.cpp` 未修改。
