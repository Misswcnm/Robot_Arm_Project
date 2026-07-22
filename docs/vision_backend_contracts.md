# 视觉机械臂后端边界

小车协议和 `vision_arm_executor` RPC 是稳定的外部接口。原 ICP、AprilTag
交互程序只作为算法实现，由 `vision_arm_executor.backends` 中的适配器隔离。

## IcpBackend

- `robot_state()`：返回统一的 `RobotState`，不初始化或使能机器人。
- `initialize_robot()`：执行 ICP CR5 的标准安全初始化，进程内幂等。
- `record_reference(frames, template_path)`：录制并保存 A 点模板。
- `flange_transform()`：读取 User0/Tool0 下的法兰齐次变换。
- `align(...)`：加载模板、运行闭环 ICP，并通过回调报告进度和运动安全检查。
- `move_relative(delta, label, motion_guard)`：执行经过安全回调批准的 A→B 相对运动。
- `reset_robot/enable_robot/disable_robot/clear_error`：控制器状态操作。

## AprilTagBackend

- `identity()`：返回 Tag ID、frame 和两份标定文件路径。
- `robot_state()`：返回与 ICP 相同结构的 `RobotState`。
- `locate()`：返回 `AprilTagLocation`，包含目标、定位快照和身份元数据。
- `validate_manual()`：仅在拖拽状态记录人工真值。
- `prepare_pick(localization)`：初始化机器人并恢复持久化定位快照。
- `localization_drift(localization)`：计算定位后机械臂的平移和旋转漂移。
- `pick(safety_check)`：仅在该调用期间打开 demo 内部运动许可，并保证退出时关闭。
- `disable_motion()`：无条件关闭 AprilTag 内部运动许可。

## 所有权

- Backend 负责 demo 类、其内部字段以及 ROS2 机械臂调用。
- VisionExecutor 负责请求状态、互斥、超时、取消、运动许可、安全边界和数据版本。
- NX gateway 只负责旧 TCP JSON 与 RPC action 的协议转换。
- `application/` 不依赖或导入任何本地算法模块。

内部仍保留 ICP 和 AprilTag 两个 CR5 实现：AprilTag 额外提供夹爪和拖拽操作，ICP
额外提供关节兜底。它们通过统一 `RobotState` 契约暴露给执行器，避免在没有完整硬件
回归的情况下强行合并运动实现。
