# ESTUN 接口边界与开发约定

本文用于固定两套接口的真实边界，避免继续根据字段名称猜测动作：

- **ESdunTCP**：`TCP_asdun_arm/CodroidApi/src/RobotDemo.cpp` 暴露给 NX 车辆的原生 TCP JSON 协议。
- **ros2estun**：`vision_arm_executor/nx_gateway.py` 接收 NX TCP JSON，再通过本机 JSON RPC 调用 `vision_arm_executor_estun`。
- 本文不比较 ESTUN 控制器的 Codroid WebSocket 原始协议。

当前 ros2estun 调用链为：

```text
NX 车辆 TCP :8888
  -> vision_arm_executor/nx_gateway.py
  -> JSON RPC 127.0.0.1:17881
  -> vision_arm_executor_estun
  -> /estun_codroid/* ROS 2 service
  -> estun_codroid_bridge(C++)
  -> Codroid WebSocket 192.168.2.5:9000
```

## 总原则

1. 一个请求必须只分类为一个确定动作，不能因字段缺失、当前示教状态或旧任务状态串入另一条链路。
2. 新命令到达时，以新命令的显式语义为准；旧示教任务应安全结束或取消，不能长期阻塞新命令。
3. ESdunTCP 已支持、但 ros2estun 尚未支持的字段或动作，必须明确返回失败，禁止静默忽略后继续执行。
4. 文档中的“当前实现”只描述代码现状；“目标约定”才是后续修改必须满足的行为。
5. `mapid + poseid` 表示一个站点；站点内的每个小点使用独立 `command` 编号。

## ESdunTCP 当前接口

### 1. 开始传统示教

```json
{"type":"type1","mapid":"M","poseid":"P"}
```

行为：

- 写入当前任务信息并创建 `mapid/poseid` 目录。
- 传统 `.crp` 文件的监听、解析和 JSON 导出由 `TCP_asdun_arm/CodroidApi/auto_process.py` 完成。
- 最终生成 `{mapid}_{poseid}.json`。
- `RobotDemo.cpp` 的 `type1` 分支没有稳定、统一的即时成功响应。

### 2. 查询传统示教点

```json
{"type":"type2","mapid":"M","poseid":"P"}
```

行为：

- 读取站点下的 `{mapid}_{poseid}.json`。
- 成功时直接返回原始 JSON 点位数组。
- 文件不存在时返回：

```json
{"error":"json file not found"}
```

- 读取异常时返回：

```json
{"error":"exception while reading json"}
```

### 3. 执行生产任务

```json
{"mapid":"M","poseid":"P","label":"P1,P2","rc":"0","led":"1"}
```

字段语义：

- `label`：选择点位并决定执行顺序；逗号分隔时按输入顺序执行。
- `rc=1`：热成像链路；其他值进入普通相机链路。
- `led`：普通相机链路中的补光灯控制。
- 执行记录的 APos 点位、采集图像或热成像数据，完成后复位机械臂。

成功响应：

```json
{"mapid":"M","poseid":"P","status":"done"}
```

当前原生代码的部分运动失败、点位缺失分支只复位或继续，失败响应并不完全统一。这是 ESdunTCP 自身的现状，不应照搬成 ros2estun 的静默失败。

### 4. 裸机械臂命令

ESdunTCP 只要收到包含 `command` 的 JSON，就优先进入机械臂命令分支。它没有区分“无站点控制命令”和“带站点的小点编号”。

| command | ESdunTCP 动作 | 主要响应字段 |
| --- | --- | --- |
| 1 | 复位 | `fuwei` |
| 2 | 上电 | `shangdian` |
| 3 | 下电 | `xiadian` |
| 4 | 自动模式 | `zidong` |
| 5 | 手动/Ready 模式 | `shoudong` |
| 6 | 清错 | `qingchu` |

ESdunTCP 没有实现 `type3` 删除协议。

## ros2estun 当前接口

### 1. 开始站点示教

普通点示教
```json
{"type":"type1","mapid":"M","poseid":"P"}
```
AprilTag示教
```json
{"type":"type1","mapid":"M","poseid":"P","point_type":0}
```

后续将tag_offset_xyz_mm删掉，预留"tag_id":0 接口，后续任务可能拓展为多tag任务，可显示指定"tag_id"。

ICP示教
```json
{"type":"type1","mapid":"M","poseid":"P","point_type":1}
```
存储逻辑

- 省略 `point_type`：普通示教。
- `params`、`tag_id`、`tag_offset_xyz_mm` 都不是普通 `type1` 的必填字段；显式提供时仍按兼容参数处理。
- `type1` 只负责开始示教，禁止携带 `command`。

### 2. 记录站点内小点

```json
{"type":"mechanical_arm_command","mapid":"M","poseid":"P","command":1}
```

只要 `mapid`、`poseid` 完整存在，该请求就是站点示教命令，不得解释成裸机械臂控制。

- AprilTag/ICP：`command=1` 为 Ref，`command=2` 为 Work1，`command=3/4/5...` 为后续 Work。
- 普通示教：`command=1/2/3...` 分别记录站点内第 1/2/3... 个小点。
- `command=0`：结束并保存当前示教。
- `command=-1`：取消当前示教。

因此下面两个请求语义完全不同：

```json
{"type":"mechanical_arm_command","command":1}
```

表示机械臂复位。

```json
{"type":"mechanical_arm_command","mapid":"M","poseid":"P","command":1}
```

表示记录站点的小点 1/Ref，绝不能串到复位或站点执行。

### 3. 完成并查询站点

```json
{"type":"type2","mapid":"M","poseid":"P"}
```

当前行为：

- 若同一站点仍处于示教状态，先执行结束保存。
- 随后查询该站点的点位清单。
- 返回 ros2estun 统一响应包装，不是 ESdunTCP 的原始 JSON 数组。
- `type2` 是查询/完成协议，禁止携带 `command`，不得触发示教记录或站点执行。

### 4. 删除站点或小点

删除整个站点，poseid为uuid，唯一，一个pose下的不同点poseid是不同的：

```json
{"type":"type3","mapid":"M","poseid":"P"}
```

删除指定小点，command是与前端共同约定：

```json
{"type":"type3","mapid":"M","poseid":"P","command":3}
```

这是 ros2estun 扩展能力，ESdunTCP 没有对应实现。
### 5. 执行整个视觉站点

```json
{"mapid":"M","poseid":"P"}
```

当前行为：按站点 manifest 顺序执行所有已保存组。AprilTag 与 ICP 的 Ref/Work 多点关系由 manifest 和各组记录决定，不依赖 ESdunTCP 的传统 `.crp` 点位文件。

### 6. 裸机械臂命令

只有不带 `mapid`、`poseid` 的命令才进入机械臂控制：

| command | ros2estun 当前动作 |
| --- | --- |
| 1 | `robot_reset` |
| 2 | `robot_enable` |
| 3 | `robot_disable` |
| 4 | `robot_start_drag` |
| 5 | `robot_stop_drag` |
| 6 | `robot_clear_error` |

## 已确认的接口差异

| 能力 | ESdunTCP | ros2estun 当前实现 | 结论 |
| --- | --- | --- | --- |
| `type1` 传统示教 | `.crp` 监听并导出关节点 JSON | 内部普通/AprilTag/ICP 示教 | 名称相同，数据模型不同 |
| 站点小点编号 | 没有独立语义，`command` 总是机械臂命令 | 带站点的 `command` 是 Ref/Work/普通小点编号 | ros2estun 扩展 |
| `type2` | 返回原始点位数组 | 结束当前示教并返回统一查询响应 | 响应格式不兼容 |
| `type3` | 未实现 | 删除站点或指定小点 | ros2estun 扩展 |
| 无 `type` 的站点执行 | 支持 | 支持 | 基础入口一致，内部数据不同 |
| `label` 选点与排序 | 支持 | 未实现 | 必须明确失败，不能忽略 |
| `rc` 热成像 | 支持 | 未实现 | 必须明确失败，不能忽略 |
| `led` 补光灯 | 支持 | NX 生产链路未实现 | 必须明确失败，不能忽略 |
| 普通/热成像采图 | 支持 | 未按 ESdunTCP 语义实现 | 必须明确失败 |
| command 4 | 自动模式 | 开始拖动 | 严重语义冲突 |
| command 5 | 手动/Ready | 停止拖动 | 严重语义冲突 |
| 统一结构化失败 | 不完整 | 基本具备 | ros2estun 应继续统一 |

## ros2estun 尚未实现的 ESdunTCP 能力

以下能力不能宣称已经兼容：

1. `.crp` 文件监听、解析和传统关节点 JSON 导出。
2. `label` 指定部分点位及执行顺序。
3. `rc=1` 热成像任务。
4. `led` 控制普通相机补光灯的完整生产链路。
5. ESdunTCP 的普通相机/热成像采图及文件返回约定。
6. `type2` 原始数组响应格式。
7. ESdunTCP command 4 自动模式和 command 5 手动/Ready 模式。

当前 `nx_gateway.py` 不会把顶层 `label`、`rc`、`led` 传给执行器，站点仍可能继续执行全部 manifest。这属于尚未修复的协议缺陷，不是兼容行为。

## 失败返回约定

目标约定：任何已识别但未实现的 ESdunTCP 字段或动作都必须在机械臂动作前失败。例如：

```json
{
  "type":"error",
  "status":"failed",
  "error_code":"unsupported_feature",
  "message":"ros2estun does not implement ESdunTCP field: label"
}
```

必须遵守：

- 不支持 `label` 时，不得改成执行全部点。
- 不支持 `rc` 时，不得默认为普通相机。
- 不支持 `led` 时，不得忽略后继续执行。
- command 4/5 在协议版本或消息类型未明确前，不得猜测为自动/手动或开始/停止拖动。
- 缺少 `mapid` 或 `poseid` 时直接返回 `invalid_request`，不得降级到其他动作。
- 未知 `type` 或未知 `command` 必须返回失败，不能只打印日志。

注意：`unsupported_feature` 是目标约定；当前网关解析失败仍主要使用 `invalid_request`，需要后续实现专门的错误码。

## 后续修改边界

修改顺序必须是：

1. 先为每一种 JSON 输入固定解析测试，确认“一条请求只对应一个动作”。
2. 再补齐未实现字段的前置拒绝和 `unsupported_feature` 响应。
3. 明确 command 4/5 的协议版本或独立消息类型后，才能修改动作映射。
4. 最后再做执行器内部重构；不得以重构为理由改变已经固定的 NX 请求语义。

建议将代码职责收敛成三层：

```text
协议解析层：只校验字段并确定唯一请求类型
兼容适配层：明确 ESdunTCP、视觉扩展及响应格式差异
执行器层：只执行已经确定的动作，不再猜测协议含义
```

每次协议修改至少验证：

- 裸 `command=1` 只复位一次。
- 带完整站点的 `command=1/2/3/4/5` 只记录对应小点。
- `type2` 只完成/查询。
- `type3` 只删除，并且活动示教不会使其串到执行。
- `label`、`rc`、`led` 在未实现时明确失败，且机械臂不运动。
- 同一 TCP 请求只产生一个最终响应；车辆重连重发不会排队造成重复运动。
