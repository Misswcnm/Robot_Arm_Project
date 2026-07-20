# 借鉴 Cartographer 的实时 ICP 伺服架构

本文目标不是把 SLAM 全部搬到机械臂上，而是借鉴 Cartographer 的几个关键工程模式，解决当前 ICP 伺服最危险的问题：ICP 算完时，机器人状态已经变了，控制命令却还按旧状态下发。

> 当前工程说明（2026-07-20）：`continuous_icp_servo` 已改为“低频 ICP 覆盖基座系绝对目标、高频 ServoP 重复该目标”。笛卡尔反馈只使用 `GetPose()`，不使用 `ToolVectorActual`。由于控制量是绝对目标而不是重复积分的相对误差，当前模式不做状态外推，也不做文中建议的 ICP 小步拆分；本文对应内容仅保留为相对误差伺服方案的理论参考。

## 1. 从 Cartographer 借鉴什么

### 1.1 传感器消息先按时间排队

Cartographer 的 `OrderedMultiQueue` 不是“谁先回调就先处理谁”，而是为每个传感器维护队列，然后每次取所有队列里时间最早的数据。`DispatchAllBefore(mapping_time)` 会把当前激光帧之前的 IMU、里程计等状态先处理完，再处理这帧点云。

对机械臂 ICP 伺服的启发：

- 点云、`ToolVectorActual`、`JointState` 都必须带时间戳进入缓存。
- ICP 处理某帧点云时，要拿这帧点云时间附近的工具位姿，而不是拿“现在最新位姿”硬配。
- 如果点云时间戳太旧，或者该时间附近没有机器人状态，直接丢弃这帧，不进入控制。

### 1.2 用状态外推器补齐时间差

Cartographer 的 `PoseExtrapolator` 维护最近位姿、IMU、里程计，并根据速度把状态外推到指定时间。它不是等待所有东西完美同步，而是在可控时间窗口内预测。

对机械臂 ICP 伺服的启发：

- 高频反馈流维护 `latest_tool`、`latest_joint` 和短时间历史队列。
- 控制器要能查询 `tool_at(t_cloud)` 和 `tool_at(t_now + servo_latency)`。
- 外推只允许很短，例如 50 到 150 ms；超过窗口就认为状态不可用。

### 1.3 重计算放后台，控制线程只做轻计算

Cartographer 用 `Task` 和 `ThreadPool` 把投影、匹配、建图、回环分成任务，并用依赖关系保证顺序。线程池线程还主动降低 nice 等级，避免抢占前台实时任务。

对机械臂 ICP 伺服的启发：

- ROS 回调只接收消息和更新缓存。
- ICP 线程低频运行，例如 2 到 6 Hz。
- ServoP 控制定时器高频运行，例如 20 到 50 Hz，只读取最新估计结果、做限幅、发命令。
- 任何 PCL、ICP、文件保存、日志统计，都不能阻塞控制定时器和机器人状态回调。

## 2. 推荐总架构

```
PointCloud 回调  ──> 点云缓存 ──> ICP worker 2~6Hz ──> IcpMeasurement
ToolVector 回调  ─┐
JointState 回调  ─┴─> RobotStateBuffer ──> StateEstimator ──> ServoState
命令/键盘回调    ──> 模式状态
Servo timer 20~50Hz ───────────────────────────────> ServoP 发布
```

线程划分：

- `MultiThreadedExecutor`：至少 4 个线程。
- 独立 callback group：点云、`ToolVectorActual`、`JointState`、命令、服务、servo timer。ssh
- 独立 ICP worker：低优先级后台线程，永远不直接发控制命令。
- ServoP timer：只做状态读取、目标生成、限幅和异步发送。

## 3. 时间戳策略

实时性第一原则：不用“刚算好的 ICP”，而用“时间关系正确的 ICP”。

每一帧点云进入 ICP 前：

1. 读取点云时间 `t_cloud`。
2. 从 `RobotStateBuffer` 查询 `T_base_tool(t_cloud)`。
3. 如果机器人状态缺失，丢弃该点云。
4. 如果点云年龄 `now - t_cloud > max_cloud_age`，丢弃该点云。
5. ICP 输出必须携带 `t_cloud`、`seq`、质量指标和计算耗时。

ServoP 控制周期：

1. 查询当前估计工具位姿 `T_now`。
2. 查询最近 ICP 测量 `icp.seq`。
3. 如果 `now - icp.stamp > max_icp_age`，不使用该 ICP。
4. 如果 `icp.seq` 已经消费过，只继续跟踪平滑目标，不能重复积分同一个误差。
5. 如果 `ToolVectorActual` 或 `JointState` 过期，立刻停止发 ServoP。

推荐阈值：

| 项目 | 建议值 |
| --- | --- |
| `ToolVectorActual` 最大年龄 | 0.20 s |
| `JointState` 最大年龄 | 0.20 s |
| 点云最大年龄 | 0.30 s |
| ICP 结果最大年龄 | 0.50 s |
| 状态外推最大时间 | 0.10 s |
| ServoP 频率 | 20 到 50 Hz |
| ICP 频率 | 2 到 6 Hz |

## 4. 状态估计和滤波

### 4.1 机械臂状态缓存

建议维护两个环形队列：

```cpp
struct TimedToolPose {
  rclcpp::Time stamp;
  Eigen::Isometry3d T_base_tool;
};

struct TimedJointPose {
  rclcpp::Time stamp;
  std::array<double, 6> q_deg;
};
```

缓存长度不用大，1 到 3 秒足够。查询时按时间插值：

- 平移：线性插值。
- 姿态：四元数 slerp。
- 如果目标时间晚于最新反馈，只允许短时间常速度外推。

### 4.2 ICP 作为低频测量

ICP 输出不是控制命令，而是一个测量：

```cpp
struct IcpMeasurement {
  uint64_t seq;
  rclcpp::Time stamp;
  Eigen::Isometry3d T_cam_cur_to_ref;
  double rmse_mm;
  double overlap;
  int inliers;
  double solve_ms;
  bool valid;
};
```

质量门控：

- `overlap` 太低，拒绝。
- `rmse` 太大，拒绝。
- 平移或旋转突变太大，拒绝。
- 连续 N 次质量差，停止 ServoP。

### 4.3 误差滤波

将 ICP 误差转为工具坐标修正：

```
T_tool_cur_to_ref = X_tool_cam * T_cam_cur_to_ref * X_cam_tool
e = log(T_tool_cur_to_ref)
```

其中 `e` 是 6 维误差，前三维为平移，后三维为旋转。不要把 ICP 原始误差直接全量下发，必须滤波：

```
e_filtered = alpha * e + (1 - alpha) * e_last
e_step = clamp(e_filtered, max_step_xyz, max_step_rpy)
```

推荐初值：

- `alpha = 0.2 ~ 0.4`
- 单周期平移步长 `0.2 ~ 1.0 mm`
- 单周期旋转步长 `0.03 ~ 0.15 deg`
- 连续 3 到 5 次 ICP 坏帧后停机

如果后续要升级，可以把它做成 EKF：

- 预测量：`ToolVectorActual` 推出来的当前位姿和速度。
- 观测量：ICP 给出的相对位姿误差。
- 协方差：由 `rmse`、`overlap`、`inliers` 动态调整，质量越差，测量权重越低。

## 5. 控制发布原则

ServoP 只应该跟踪平滑目标，不应该跟踪 raw ICP。

控制周期逻辑：

```text
read latest robot state
if robot state stale: stop

read latest filtered correction
if correction stale: hold or stop

target = current_tool_pose * small_delta
target = rate_limit(target)
send ServoP(target)
```

必须限制：

- 位置单周期步长。
- 姿态单周期步长。
- 速度上限。
- 加速度或步长变化率。
- 工作空间范围。
- 关节限位。
- 连续异常次数。

停止条件：

- 机器人反馈过期。
- 点云过期。
- ICP 结果过期。
- ICP 质量连续失败。
- 目标超出工作空间。
- ServoP 服务不可用或连续返回异常。

## 6. 当前工程迁移清单

### 6.1 已经应该保留的做法

- 使用 `MultiThreadedExecutor`。
- 订阅 `ToolVectorActual` 维护 `latest_tool`。
- 订阅 `/joint_states_robot` 维护 `latest_joint`。
- 控制路径不再调用 `GetAngle`。
- ICP 和 ServoP 分频：ICP 低频，ServoP 高频。
- 键盘/命令、点云、机器人状态、服务分 callback group。

### 6.2 还需要补齐的做法

1. 把 `latest_tool` 升级成 `TimedRingBuffer<ToolPose>`。
2. 把 `latest_joint` 升级成 `TimedRingBuffer<JointPose>`。
3. ICP worker 处理点云前，按点云时间查询工具位姿。
4. ICP result 增加 `cloud_stamp`、`robot_pose_stamp`、`solve_ms`、`latency_ms`。
5. Servo timer 只消费时间有效的 ICP，不重复积分旧 `seq`。
6. 增加 `StateEstimator`，用反馈流估计当前位姿和速度。
7. 增加平滑目标生成器，ServoP 跟踪目标生成器输出。
8. 后台日志统计控制延迟、ICP 延迟、反馈年龄、丢帧原因。

## 7. 建议类划分

```text
RobotStateBuffer
  addTool(stamp, pose)
  addJoint(stamp, q)
  lookupTool(stamp)
  latestTool()

StateEstimator
  predictTool(stamp)
  updateFromToolFeedback()
  updateFromIcpMeasurement()

IcpWorker
  consumeLatestPointCloud()
  queryRobotStateAtCloudTime()
  publishIcpMeasurement()

ServoController
  readEstimatedState()
  readFilteredCorrection()
  generateRateLimitedTarget()
  sendServoP()
```

核心边界：`IcpWorker` 不发 ServoP，`ServoController` 不跑 ICP。

## 8. 实验验证方案

### 8.1 静态延迟实验

机械臂不动，记录：

- 点云时间戳。
- `ToolVectorActual` 最新时间。
- ICP 开始和结束时间。
- ServoP 发送时间。

合格标准：

- ServoP 控制周期内不出现阻塞。
- `ToolVectorActual age < 0.2s`。
- ICP 延迟被记录，过期结果不会进入控制。

### 8.2 小偏移回收实验

先录模板，再用 JointMovJ 做小偏移，例如 5 到 10 mm 等效位移。

观察：

- ICP 误差是否逐步减小。
- ServoP 是否单向平滑收敛。
- 是否出现来回抖动。

如果抖动：

- 降低 `alpha`。
- 降低 `max_step_mm`。
- 增大 ICP 质量门槛。
- 检查 `T_cam_cur_to_ref` 到工具坐标修正的方向。

### 8.3 动态过期实验

故意降低 ICP 频率或增加 ICP 计算负载。

合格标准：

- ICP 过期时控制器停止使用旧修正。
- 控制器不重复积分旧 ICP。
- 机器人状态回调仍然稳定更新。

## 9. Cartographer 源码对应关系

本次 review 主要参考这些位置：

| Cartographer 文件 | 借鉴点 | 机械臂对应 |
| --- | --- | --- |
| `sensor/internal/ordered_multi_queue.cc` | 多传感器按时间顺序分发，`DispatchAllBefore()` 先处理目标时间之前的数据 | 点云进入 ICP 前，先查询点云时间附近的工具位姿 |
| `sensor/internal/sensor_collator.cc` | 每个传感器单独入队，统一排序后回调 | 点云、ToolVector、JointState 分 callback group，统一进状态缓存 |
| `extrapolator/pose_extrapolator.cc` | 最近位姿队列、速度估计、短时间外推 | `RobotStateBuffer + StateEstimator`，允许短时间预测，不允许长期补偿 |
| `mapping/map_builder.cc` | 投影、匹配、建图、回环拆成任务，用依赖关系保证顺序 | ICP worker 和 ServoP timer 解耦，ICP 不直接控制机械臂 |
| `common/task.cc` / `common/thread_pool.cc` | 后台任务调度，重计算不堵塞前台 | PCL/ICP/日志放后台，机器人反馈和控制发布优先 |

其中最关键的一句是：Cartographer 在处理当前激光帧前，会调用类似 `DispatchAllBefore(cur_laser_time)` 的逻辑，把该时间之前的状态先更新完。机械臂 ICP 也应该这样：先用点云时间找机器人状态，再计算 ICP，最后高频控制器只消费有效、未过期的测量。

## 10. 一句话结论

平滑 ICP 伺服不能写成“点云来了就 ICP，ICP 完了就 ServoP”。应写成 Cartographer 式的实时系统：传感器按时间进入缓存，状态可插值/外推，ICP 是低频测量，控制是高频平滑跟踪，任何过期数据都只能丢弃，不能补发到机械臂上。
