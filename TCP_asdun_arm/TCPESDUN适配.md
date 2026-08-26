# TCP ASDUN（ESTUN/埃斯顿）适配说明

一句话：`TCP_asdun_arm` 是跑在旧 NX 上的机械臂巡检程序 —— **TCP 8888 收小车 JSON → 解析 → 控制埃斯顿机械臂走关节点 + 拍照/热成像/YOLO → 回图片和状态**。你要做的，是把主项目的 ICP / AprilTag 视觉伺服能力，接到这个 8888 入口上。

---

## 0. 运行入口：`RobotControl` 在哪

`runDemo.sh` 里的 `./RobotControl` 是**编译产物，不是源码文件**：

| 项 | 位置 |
|---|---|
| 真正参与编译的主程序源码 | [CodroidApi/src/RobotDemo.cpp](CodroidApi/src/RobotDemo.cpp) |
| 编译产物二进制 | `CodroidApi/build/RobotControl`（**aarch64 ELF**，只能在 NX 上跑，本机 x86 跑不了） |
| 编译脚本 | `compileDemo.sh`（cmake + make） |

`runDemo.sh` 逻辑就两行：`cd build && ./RobotControl`。

---

## 1. 目录一览

```text
TCP_asdun_arm/
├── CodroidApi/                 ← 核心代码（唯一需要动的）
│   ├── src/                    ← C++ 源码 + 运行期数据目录
│   ├── include/                ← 头文件（SDK API + 类型 + json 单头库）
│   ├── lib/                    ← 大华/流转换 SDK 的 .so
│   ├── build/                  ← 编译产物（含 RobotControl）
│   ├── CMakeLists.txt          ← 决定“哪些文件参与编译”
│   ├── compileDemo.sh / runDemo.sh
│   ├── auto_process.py         ← .crp 示教文件监控（独立进程）
│   ├── jiexi.py                ← .crp 解析脚本
│   ├── current_task.json / processed_files.json  ← 运行期状态
├── build/                      ← 顶层残留 build（旧目录，忽略）
├── picture/ yolo_results/      ← 拍照 / YOLO 输出目录
├── *.pdf                       ← Codroid SDK 使用手册（参考）
├── asdun_arm_backend_protocol_and_replacement.md  ← 旧协议逐项对照（权威）
├── DEVELOPMENT.md              ← ESTUN 接口边界约定
├── 鎏鑫研发.md                  ← 你在写的协议解析流程草稿
└── 新建 文本文档.txt             ← NX 部署/启动笔记
```

---

## 2. 代码文件逐个说明

> 标记：✅ **在用**（编译进 RobotControl） / ❌ **未用**（CMake 注释掉或根本没被引用）

### 2.1 真正参与编译的（CMakeLists 里 `add_executable` 只有这两个）

| 文件 | 干什么 |
|---|---|
| ✅ [src/RobotDemo.cpp](CodroidApi/src/RobotDemo.cpp) | **主程序**。TCP 8888 监听 + JSON 字段解析 + 机器人状态机 + 示教(type1/type2/type3) + 巡检执行 + 拍照/YOLO/热成像 + 图片/状态回传 |
| ✅ [src/WebSocketClient.cpp](CodroidApi/src/WebSocketClient.cpp) | boost.beast WebSocket 客户端（被 CodroidApi 调用，连埃斯顿 9000） |

### 2.2 被 RobotDemo 间接引用的头文件

| 文件 | 干什么 |
|---|---|
| ✅ [include/CodroidApi.h](CodroidApi/include/CodroidApi.h) | **埃斯顿 Codroid SDK 的 C++ 封装**：getRobotState / sendUserCommand(上电/自动/清错) / movJ / getJointPosition / setDO 等，底层发 WebSocket JSON |
| ✅ [include/WebSocketClient.h](CodroidApi/include/WebSocketClient.h) | WebSocket 客户端类声明（对应 2.1） |
| ✅ [include/Define.h](CodroidApi/include/Define.h) | `c2` 命名空间的类型/枚举：Point、APos、ResponseCode、UserCommand 等 |
| ✅ [include/Request.h](CodroidApi/include/Request.h) / [RequestData.h](CodroidApi/include/RequestData.h) | 构造 WebSocket 请求 JSON |
| ✅ [include/thermal_manager.h](CodroidApi/include/thermal_manager.h) | 大华热成像相机封装（RTSP 抓帧 + 温度） |
| ✅ [include/nlohmann/json.hpp](CodroidApi/include/nlohmann/json.hpp) | JSON 单头库（本地副本，非系统库） |
| ⚠️ [include/dhnetsdk.h](CodroidApi/include/dhnetsdk.h) | 大华 SDK 头文件（8MB，热成像用；但源码里热成像实际走 RTSP，此头可能只被部分路径用到） |

### 2.3 未编译的“模块化重构草稿”（CMake 里整段被注释掉）

这几个是把 RobotDemo 拆成多文件的版本，**代码在但没进二进制**，只能当参考：

| 文件 | 干什么（重构目标） |
|---|---|
| ❌ [src/main.cpp](CodroidApi/src/main.cpp) | 模块化版的 main（对应 RobotDemo 的 main） |
| ❌ [src/EstunController.cpp](CodroidApi/src/EstunController.cpp) + .h | 埃斯顿控制类（独立出来） |
| ❌ [src/MachineController.cpp](CodroidApi/src/MachineController.cpp) + .h | TCP 8888 服务类（独立出来） |
| ❌ [src/RobotTask.cpp](CodroidApi/src/RobotTask.cpp) + .h | 主任务线程 taskThread（独立出来） |
| ❌ [src/CameraController.cpp](CodroidApi/src/CameraController.cpp) + .h | RealSense 拍照类（独立出来） |
| ❌ [src/GlobalDefines.cpp](CodroidApi/src/GlobalDefines.cpp) + .h | 全局变量/状态枚举定义 |

### 2.4 你最近在写的“模拟版主程序”（⚠️ 关键，还没进 CMake）

| 文件 | 干什么 |
|---|---|
| 🔶 [src/mockrobot.cpp](CodroidApi/src/mockrobot.cpp) | **RobotDemo 的 Mock 版**：结构几乎一样，但用 `MockEstunController` 代替真机（不连 9000，全打印模拟）、Mock 拍照。**TCP 8888 解析 + type1/type2/type3 分发骨架已搭好，但每个分支动作是空的**（switch 里只有注释） |

> 这就是你融入 ICP/AprilTag 的落点 —— 见第 6 节。

### 2.5 未编译的早期视觉草稿

| 文件 | 干什么 |
|---|---|
| ❌ [src/arm_vision.cpp](CodroidApi/src/arm_vision.cpp) | 早期“视觉引导机械臂”demo（RealSense + 移动 + 拍照） |
| ❌ [src/takephoto.cpp](CodroidApi/src/takephoto.cpp) | 独立 RealSense 拍照 demo |
| ❌ [src/vision.py](CodroidApi/src/vision.py) | 独立 RealSense 工具：鼠标点选取 3D 坐标（调试用） |
| ❌ src/arm_vision.zip / jiexi.zip | 备份压缩包，忽略 |

### 2.6 Python 脚本（独立进程，不随 RobotControl 启动）

| 文件 | 干什么 | 谁调它 |
|---|---|---|
| 🔶 [auto_process.py](CodroidApi/auto_process.py) | 监控 `/home/nvidia/Downloads` 的新 `.crp` 示教文件 → 调 jiexi.py 解析 → 存成点位 JSON | 需手动 `python3 auto_process.py` 单独跑 |
| 🔶 [jiexi.py](CodroidApi/jiexi.py) | 解析 `.crp`：递归找 `type=points` 的 children → 提取 `label + joints` → 输出 `[{"label","joints":{jntpos1..6}}]` | 被 auto_process.py 调用 |

### 2.7 运行期数据文件

| 文件/目录 | 干什么 |
|---|---|
| `current_task.json` | type1 时由 RobotDemo 写入，记录当前 mapid/poseid，供 auto_process.py 关联示教文件 |
| `processed_files.json` | auto_process.py 记录已处理的 .crp 文件列表 |
| `src/<mapid>/<poseid>/` | 点位 JSON（`<mapid>_<poseid>.json`）+ 图片（raw/detect/rc 子目录） |
| `src/<uuid>/*` | 历史运行生成的 mapid/poseid 数据目录 |

---

## 3. RobotControl 启动后自动做什么（不能当只读服务）

`main()` 一启动就**会让机械臂运动**：

```text
1. TCP 0.0.0.0:8888 监听
2. 连埃斯顿 192.168.2.5:9000（Codroid WebSocket）
3. 上电 SwitchOn
4. 切自动模式 ToAuto
5. sleep 2s
6. movJ 到硬编码复位点 [86,23,-112,-176,-85,0]
7. 起 6 个线程（见下）
```

热成像 RTSP 地址硬编码在 main 里：`rtsp://admin:...@192.168.2.108:554/...`。

---

## 4. 多线程状态机

| 线程 | 职责 |
|---|---|
| `positionReceiverThread` | TCP 8888 accept + recv，把 JSON 字符串推进 `g_positionQueue` |
| `taskThread` | **核心**：从队列取 JSON → 解析 type/mapid/poseid/label/rc/led → 分发（type1 示教 / type2 查询 / 普通巡检执行） |
| `statusSenderThread` | 把 `g_statusQueue` 里的状态 JSON 发给小车 |
| `imageSenderThread` | 把 `g_imageQueue` 里的图片（元数据行 + 二进制）发给小车 |
| `detectThread` | 调 YOLOv5 对拍照结果推理，结果图回送 |
| `safetyMonitorThread` | 每 500ms 查错误标志位，主动推 `safety_alert/recovery` |

`RobotState` 状态机：`IDLE → INITIALIZING → MOVING → IMAGING → (RESETTING/FAULT) → IDLE`。

---

## 5. TCP 8888 协议（小车 → NX，收到的做什么）

| 收到的 JSON | 动作 | 返回 |
|---|---|---|
| `{"command":1..6}` | 机械臂命令（复位/上电/下电/自动/手动/清错） | `{"command":N,"xxx":"success/failed"}` |
| `{"type":"type1","mapid","poseid"}` | 示教准备：写 current_task.json + 建目录（**不录点**，录点靠外部 .crp） | 无回包 |
| `{"type":"type2","mapid","poseid"}` | 读 `<mapid>_<poseid>.json` 点位数组 | 点位数组 或 `{"error":"json file not found"}` |
| `{"mapid","poseid","label","rc","led"}` | 巡检：读点位 → 逐点 movJ → 拍照/热成像 → 复位 | 成功 `{"mapid","poseid","status":"done"}`，失败常无回包 |

command 1~6 映射（**小车编号 ≠ 控制器命令值**）：

| 小车 command | 动作 | Codroid 底层 |
|---|---|---|
| 1 | 复位（movJ 到固定点） | movJ |
| 2 | 上电 | SwitchOn=1 |
| 3 | 下电 | SwitchOff=2 |
| 4 | 自动模式 | ToAuto=5 |
| 5 | 手动/Ready | ToReady=3 |
| 6 | 清错 | ClearWarning=501 |

---

## 6. 融入你的 ICP / AprilTag 项目 —— 落点

### 6.1 别直接改 RobotDemo，改 mockrobot.cpp

- `RobotDemo.cpp` 是**真机老业务**（关节点巡检 + 拍照），跟埃斯顿绑定，别动它。
- `mockrobot.cpp` 是你已经在写的**协议适配骨架**：TCP 8888 + type1/type2/type3 分发已经搭好，**但动作分支是空的**。这正是把主项目能力塞进去的地方。

### 6.2 具体接法（mockrobot.cpp 里的空分支）

`mockrobot.cpp` 的 `taskThread()` 里，`type1` 的 `switch(command)` / `point_type` 分支目前全是空注释。要填成：

| 分支 | 应调用的主项目能力 |
|---|---|
| `type1` 无 command、无 point_type | 普通点示教（开始拖拽）→ CR5 `StartDrag` |
| `type1` + `point_type:0` | AprilTag 示教 → AprilTag 定位/录 Ref |
| `type1` + `point_type:1` | ICP 示教 → ICP 录模板 |
| `type1` + `command:1` | 录 Ref 点（普通/apriltag/icp 按 point_type 区分） |
| `type1` + `command:2..` | 录 Work 点 |
| `type2` | 查询点位 → 读 manifest |
| `type3` | 删除站点/小点 |
| 无 type 的 `{"mapid","poseid"}` | 执行整个视觉站点（ICP 对齐 / AprilTag 定位 / 抓取） |

### 6.3 三条硬约束（来自 CLAUDE.md / DEVELOPMENT.md，别踩）

1. **厂家协议不能混**：埃斯顿走 Codroid WebSocket(9000)，你的 CR5/Dobot 走 ROS2 dashboard(29999)。8888 入口可以复用，但 9000↔CR5 之间必须隔一层适配器，不能改个函数名就替换。
2. **command 语义冲突**：老协议 command 4/5 是“自动/手动模式”，新协议是“开始/停止拖拽”。带 `mapid+poseid` 的 command 是“站点小点编号”，裸 command 才是机械臂命令 —— 解析时必须先分这两类。
3. **未实现的字段要显式失败**：`label`/`rc`/`led` 你的 ICP/AprilTag 流程不支持时，必须回 `unsupported_feature`，**不能静默忽略继续动机械臂**。

### 6.4 建议的三层结构

```text
协议解析层  只校验字段，确定唯一请求类型（type1/type2/type3/裸command/站点执行）
兼容适配层  明确 ESdunTCP 老协议 ↔ 你的视觉扩展 的字段/响应差异
执行器层    只执行已确定动作（CR5 拖拽/录点/ICP/AprilTag/抓取），不再猜协议
```
