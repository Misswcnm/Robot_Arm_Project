# CR5 C++ TCP/IP 操作手册

本文供现场开发、调试和维护人员使用。所有机械臂操作均通过本目录下的 `TCP-IP-CR-CPP-V3` C++ API 完成，不使用 `nc` 或手工发送字符串。

## 1. SDK 与端口结构

当前使用的 SDK：

```text
src/TCP-IP-CR-CPP-V3/
└── CppDemo/
    ├── api/
    │   ├── DobotClient.*    # TCP 连接、发送、同步接收
    │   ├── Dashboard.*      # 状态、使能、清错、参数、拖拽
    │   ├── DobotMove.*      # MovJ、MovL、JointMovJ、ServoP 等运动
    │   ├── Feedback.*       # 30004 后台接收和解析
    │   └── FeedbackData.h   # 实时状态数据结构
    └── CR5BasicExample.cpp  # 本工程提供的只读连接示例
```

V3 SDK 必须建立三条连接：

| C++ 类 | 端口 | 用途 |
|---|---:|---|
| `Dobot::CDashboard` | `29999` | 使能、清错、状态查询、参数设置、拖拽 |
| `Dobot::CDobotMove` | `30003` | `MovJ`、`MovL`、`JointMovJ`、点动和伺服运动 |
| `Dobot::CFeedback` | `30004` | 接收并解析实时二进制反馈 |

当前 CR5 地址为 `192.168.1.6`。如控制柜 IP 已修改，只改程序配置，不要把 IP 分散硬编码到多个业务文件。

## 2. 安全要求

1. 新程序第一次运行时只允许查询状态，禁止直接执行运动。
2. 工作区必须无人、无障碍物，工具和线缆安装牢固，物理急停可触达。
3. 新点位必须先在示教器中低速验证；程序首次执行建议 `SpeedFactor(15)`。
4. 同一时间只运行一个控制进程。C++ SDK、ROS 2 驱动和其他 TCP 客户端不能同时控制机械臂。
5. API 返回成功只表示控制器接收了命令，不表示机械臂已经到位。

## 3. 编译只读示例

```bash
cd /home/ylx/Robot_Arm_Project/src/TCP-IP-CR-CPP-V3/CppDemo
cmake -S . -B build
cmake --build build --target CR5BasicExample -j"$(nproc)"
```

运行：

```bash
./build/CR5BasicExample 192.168.1.6
```

该示例只执行：

- 连接 29999、30003、30004；
- `RobotMode()`；
- `GetPose()`；
- `GetAngle()`；
- `GetErrorID()`；
- 输出一帧 30004 实时状态。

它不会清错、使能、拖拽或运动，适合作为现场第一步联通验证。

不要直接运行原有 `DobotTcpDemo` 进行首次测试。原 Demo 在连接后会使能机械臂，并在两个示例点之间无限循环执行 `MovL`。

## 4. C++ 连接封装

业务代码应把三条连接封装为一个对象，并在任一连接失败时终止初始化：

```cpp
#include "api/Dashboard.h"
#include "api/DobotClient.h"
#include "api/DobotMove.h"
#include "api/Feedback.h"

#include <string>

class CR5Driver
{
public:
    explicit CR5Driver(std::string ip) : ip_(std::move(ip)) {}

    bool connect()
    {
        if (!Dobot::CDobotClient::InitNet()) {
            return false;
        }

        const bool dashboard_ok = dashboard_.Connect(ip_, 29999);
        const bool move_ok = move_.Connect(ip_, 30003);
        const bool feedback_ok = feedback_.Connect(ip_, 30004);
        return dashboard_ok && move_ok && feedback_ok;
    }

    void disconnect()
    {
        feedback_.Disconnect();
        move_.Disconnect();
        dashboard_.Disconnect();
        Dobot::CDobotClient::UinitNet();
    }

    Dobot::CDashboard& dashboard() { return dashboard_; }
    Dobot::CDobotMove& move() { return move_; }
    Dobot::CFeedback& feedback() { return feedback_; }

private:
    std::string ip_;
    Dobot::CDashboard dashboard_;
    Dobot::CDobotMove move_;
    Dobot::CFeedback feedback_;
};
```

生产代码还应实现：

- RAII 析构断连；
- 连接状态检查和有限次数重连；
- 指令串行化，禁止多个线程同时操作同一个控制通道；
- 每条命令的超时、错误码和日志；
- 运动前状态检查和运动后的到位确认。

## 5. API 返回值判断

`CDashboard` 和 `CDobotMove` 的接口返回控制器原始字符串，例如：

```text
0,{5},RobotMode();
0,{},EnableRobot();
-2,{not tcp mode or system is starting},SpeedFactor(15);
```

可以统一提取第一个逗号前的错误码：

```cpp
#include <string>

bool commandAccepted(const std::string& reply, int* error_id = nullptr)
{
    const std::size_t comma = reply.find(',');
    if (comma == std::string::npos) {
        return false;
    }

    try {
        const int id = std::stoi(reply.substr(0, comma));
        if (error_id != nullptr) {
            *error_id = id;
        }
        return id == 0;
    } catch (...) {
        return false;
    }
}
```

规则：

- 错误码 `0`：命令被控制器接受。
- 非 `0`：立即停止当前流程，不得继续发送依赖该命令的操作。
- 空字符串：接收超时或连接中断。
- `device does not connected!!!`：该 API 客户端尚未连接或已经断开。
- 运动 API 返回成功仍需读取实时反馈判断完成。

## 6. 状态和位姿查询

### 6.1 通过 Dashboard 查询

```cpp
const std::string mode_reply = robot.dashboard().RobotMode();
const std::string pose_reply = robot.dashboard().GetPose();
const std::string joints_reply = robot.dashboard().GetAngle();
const std::string errors_reply = robot.dashboard().GetErrorID();
```

`GetPose()` 返回 `{x,y,z,rx,ry,rz}`：

- `x/y/z`：mm；
- `rx/ry/rz`：deg。

`GetAngle()` 返回 `{J1,J2,J3,J4,J5,J6}`，单位为 deg。

当前 CR5 优先使用无参数 `GetPose()`。只有确认控制器固件支持时，才使用：

```cpp
robot.dashboard().GetPose(user_index, tool_index);
```

### 6.2 通过 30004 实时反馈读取

`CFeedback::Connect()` 成功后会启动后台接收线程。必须等到收到一帧有效数据：

```cpp
#include <chrono>
#include <thread>

bool waitForFeedback(Dobot::CFeedback& feedback,
                     std::chrono::milliseconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (feedback.IsDataHasRead()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}
```

读取一份快照：

```cpp
if (!waitForFeedback(robot.feedback(), std::chrono::seconds(3))) {
    throw std::runtime_error("30004 feedback timeout");
}

const Dobot::CFeedbackData state = robot.feedback().GetFeedbackData();

const long mode = state.RobotMode;
const bool enabled = state.EnableStatus != 0;
const bool running = state.RunningStatus != 0;
const bool queue_running = state.RunQueuedCmd != 0;
const bool error = state.ErrorStatus != 0;

double x = state.ToolVectorActual[0];
double y = state.ToolVectorActual[1];
double z = state.ToolVectorActual[2];
double rx = state.ToolVectorActual[3];
double ry = state.ToolVectorActual[4];
double rz = state.ToolVectorActual[5];
```

常用字段：

| 字段 | 含义 |
|---|---|
| `RobotMode` | 机器人模式 |
| `EnableStatus` | 使能状态 |
| `RunningStatus` | 运动状态 |
| `RunQueuedCmd` | 队列正在运行 |
| `ErrorStatus` | 报警状态 |
| `DragStatus` | 拖拽状态 |
| `QActual[6]` | 六轴实际关节角 |
| `ToolVectorActual[6]` | 实际 TCP 位姿 |
| `DigitalInputs/Outputs` | 数字量输入/输出 |

V3 SDK 的模式转换位于 `Feedback.cpp::ConvertRobotMode()`：

| 值 | V3 含义 |
|---:|---|
| 1 | 初始化 |
| 2 | 抱闸松开 |
| 3 | 本体电源状态 |
| 4 | 未使能 |
| 5 | 已使能且空闲 |
| 6 | 拖拽 |
| 7 | 正在运行 |
| 8 | 轨迹记录 |
| 9 | 报警 |
| 10 | 暂停 |
| 11 | 点动 |

不要把 V4 协议中可能出现的其他模式定义直接套到 V3 SDK 上，应以当前控制器固件和实时反馈为准。

## 7. 标准初始化流程

先读取状态：

```cpp
const Dobot::CFeedbackData state = robot.feedback().GetFeedbackData();
```

如果已经使能且空闲，不要重复执行下使能—上使能，只设置作业参数：

```cpp
commandAccepted(robot.dashboard().SpeedFactor(15));
commandAccepted(robot.dashboard().SetCollisionLevel(5));
commandAccepted(robot.dashboard().User(0));
commandAccepted(robot.dashboard().Tool(0));
```

如果未使能，按顺序执行：

```cpp
if (!commandAccepted(robot.dashboard().ClearError())) {
    throw std::runtime_error("ClearError failed");
}

if (!commandAccepted(robot.dashboard().EnableRobot())) {
    throw std::runtime_error("EnableRobot failed");
}

if (!commandAccepted(robot.dashboard().SpeedFactor(15)) ||
    !commandAccepted(robot.dashboard().SetCollisionLevel(5)) ||
    !commandAccepted(robot.dashboard().User(0)) ||
    !commandAccepted(robot.dashboard().Tool(0))) {
    throw std::runtime_error("robot setup failed");
}
```

随后重新读取反馈，必须确认：

```cpp
state.EnableStatus != 0
state.ErrorStatus == 0
state.RobotMode == 5
```

任一步失败都不能继续执行运动。

## 8. 笛卡尔与关节运动

### 8.1 定义点位

笛卡尔点位：

```cpp
Dobot::CDescartesPoint target{};
target.x = x_mm;
target.y = y_mm;
target.z = z_mm;
target.rx = rx_deg;
target.ry = ry_deg;
target.rz = rz_deg;
```

关节点位：

```cpp
Dobot::CJointPoint target_joint{};
target_joint.j1 = j1_deg;
target_joint.j2 = j2_deg;
target_joint.j3 = j3_deg;
target_joint.j4 = j4_deg;
target_joint.j5 = j5_deg;
target_joint.j6 = j6_deg;
```

### 8.2 MovJ：笛卡尔目标的关节运动

```cpp
const std::string reply = robot.move().MovJ(target);
if (!commandAccepted(reply)) {
    throw std::runtime_error("MovJ rejected: " + reply);
}
```

SDK 会通过 30003 发送：

```text
MovJ(x,y,z,rx,ry,rz)
```

### 8.3 MovL：笛卡尔直线运动

```cpp
const std::string reply = robot.move().MovL(target);
if (!commandAccepted(reply)) {
    throw std::runtime_error("MovL rejected: " + reply);
}
```

`MovL` 的终点可达不代表中间直线路径安全，首次必须低速验证整条路径。

### 8.4 JointMovJ：关节目标运动

```cpp
const std::string reply = robot.move().JointMovJ(target_joint);
if (!commandAccepted(reply)) {
    throw std::runtime_error("JointMovJ rejected: " + reply);
}
```

关节运动的末端路径不是直线，必须检查机械臂完整扫掠空间。

### 8.5 可选参数

V3 SDK 的模板重载会把附加参数追加到运动命令中。例如：

```cpp
robot.move().MovJ(target, "User=0", "Tool=0", "SpeedJ=20", "AccJ=20");
robot.move().MovL(target, "User=0", "Tool=0", "SpeedL=20", "AccL=20");
```

不同固件对可选参数名称和范围可能存在差异。基础现场流程优先通过独立 Dashboard API 设置全局参数，确认协议版本后再使用模板参数。

## 9. 等待运动完成

运动 API 返回成功只表示命令进入控制器队列。推荐根据 30004 反馈进行有限超时等待：

```cpp
bool waitMotionFinished(Dobot::CFeedback& feedback,
                        std::chrono::milliseconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    bool observed_running = false;

    while (std::chrono::steady_clock::now() < deadline) {
        const Dobot::CFeedbackData state = feedback.GetFeedbackData();

        if (state.ErrorStatus != 0 || state.RobotMode == 9) {
            return false;
        }

        if (state.RunningStatus != 0 || state.RunQueuedCmd != 0 ||
            state.RobotMode == 7) {
            observed_running = true;
        }

        if (observed_running && state.RunningStatus == 0 &&
            state.RunQueuedCmd == 0 && state.RobotMode == 5) {
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}
```

运动完成后再比较 `ToolVectorActual` 与目标点。位置误差和姿态误差应分别设置阈值，不能把角度和毫米直接混为一个距离。

SDK 也提供 `CDobotMove::Sync()`，但生产代码仍应设置业务级总超时，并监控 `ErrorStatus`；不能无限等待。

## 10. 拖拽操作

进入拖拽前确认机械臂已使能、空闲且无报警：

```cpp
if (!commandAccepted(robot.dashboard().StartDrag())) {
    throw std::runtime_error("StartDrag failed");
}
```

等待反馈满足：

```cpp
state.DragStatus != 0
state.RobotMode == 6
```

拖拽期间读取当前法兰/TCP 位姿：

```cpp
const Dobot::CFeedbackData state = robot.feedback().GetFeedbackData();
const double* pose = state.ToolVectorActual;
```

退出拖拽：

```cpp
if (!commandAccepted(robot.dashboard().StopDrag())) {
    throw std::runtime_error("StopDrag failed");
}
```

随后读取反馈：

- 如果已回到模式 5，不要重复使能；
- 如果为模式 4，再调用 `EnableRobot()`；
- 恢复 `SpeedFactor`、`SetCollisionLevel`、`User` 和 `Tool`；
- 最终确认模式 5 且无报警。

## 11. 停止、暂停和故障恢复

V3 API 的相关接口：

```cpp
robot.dashboard().Pause();          // 暂停队列
robot.dashboard().Continue();       // 继续队列
robot.dashboard().ResetRobot();     // 停止/复位机器人运动
robot.dashboard().EmergencyStop();  // 软件急停
```

出现危险时优先使用物理急停，软件接口不能替代现场安全回路。

报警处理顺序：

```cpp
const std::string errors = robot.dashboard().GetErrorID();
// 先记录并分析错误，排除实际原因后再清错。
const std::string clear_reply = robot.dashboard().ClearError();
```

完整流程：

1. 停止运动，必要时按物理急停。
2. 检查碰撞、关节超限、工具、负载、线缆和控制模式。
3. 调用 `GetErrorID()` 并保存原始返回。
4. 使用 `alarm_controller.json` 和 `alarm_servo.json` 查询错误含义。
5. 排除原因后调用 `ClearError()`。
6. 根据实际模式决定是否重新使能。
7. 重新设置安全参数和坐标系。
8. 确认反馈为已使能、空闲、无报警后恢复作业。

禁止在故障原因未知时循环发送清错和使能。

## 12. 下使能和关闭连接

只有在机械臂停止且处于安全姿态后才能下使能：

```cpp
const std::string reply = robot.dashboard().DisableRobot();
if (!commandAccepted(reply)) {
    throw std::runtime_error("DisableRobot failed: " + reply);
}
```

正常完成一个动作或一个任务后不需要反复下使能再使能。结束作业、维护或安全流程明确要求时才下使能。

退出程序前按以下顺序断连：

```cpp
robot.feedback().Disconnect();
robot.move().Disconnect();
robot.dashboard().Disconnect();
Dobot::CDobotClient::UinitNet();
```

## 13. 建议的业务类边界

生产程序不要让业务代码直接散落调用三个 SDK 对象。建议封装以下接口：

```cpp
class IRobotArm
{
public:
    virtual ~IRobotArm() = default;
    virtual bool connect() = 0;
    virtual bool initialize() = 0;
    virtual bool getPose(Dobot::CDescartesPoint& pose) = 0;
    virtual bool moveJ(const Dobot::CDescartesPoint& pose) = 0;
    virtual bool moveL(const Dobot::CDescartesPoint& pose) = 0;
    virtual bool startDrag() = 0;
    virtual bool stopDrag() = 0;
    virtual bool stop() = 0;
};
```

实现层负责三端口、错误码、反馈线程、超时和安全状态；业务层只调用明确的机械臂动作。这样后续更换 ROS 2 驱动或其他机械臂时，不需要修改上层流程。

## 14. 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| 29999 连接失败 | IP、网线、控制柜或 TCP 模式异常 | 检查控制柜启动、网段和远程控制模式 |
| 29999 成功、30003 失败 | V3 运动端口未开放或被其他客户端占用 | 停止其他驱动，确认控制器 V3 TCP 模式 |
| 30004 无反馈 | 反馈端口未连接、控制器未就绪或协议不匹配 | 检查 `Connect()` 返回值和固件协议版本 |
| API 返回空字符串 | 等待应答超时或连接断开 | 查询 `IsConnected()`，记录并执行有限重连 |
| `-2` | 不在 TCP 模式或控制柜仍在启动 | 等待启动完成并切换 TCP 控制模式 |
| 运动端口不能继续执行 | 队列暂停或报警堵塞 | 查询报警；排障清错后按需调用 `Continue()` |
| 指令成功但机械臂仍在运动 | 指令仅完成入队 | 监控 30004 的运行状态和队列标志 |
| 状态或应答错配 | 多个线程/进程同时控制 | 每个通道串行调用，只保留一个控制进程 |

## 15. 代码依据

- 连接实现：`CppDemo/api/DobotClient.*`
- 状态与使能：`CppDemo/api/Dashboard.*`
- 运动接口：`CppDemo/api/DobotMove.*`
- 实时反馈：`CppDemo/api/Feedback.*`
- 反馈字段：`CppDemo/api/FeedbackData.h`
- 官方示例：`CppDemo/DobotTcpDemo.*`
- 安全只读示例：`CppDemo/CR5BasicExample.cpp`

当前目录是 V3 SDK，控制器应为 V3 系列 3.5.5.0 及以上兼容版本。机械臂固件和协议不一致时，不得混用 V3 与 V4 的端口、状态枚举或命令格式。
