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
- `start_drag()`：进入 ICP 示教拖拽，确认 RobotMode 已进入 BACKDRIVE。
- `record_reference_teaching(...)`：在拖拽状态记录 A 点云和 A 法兰位姿。
- `flange_transform_teaching()`：在拖拽状态读取 B 戳点法兰位姿。
- `stop_drag_and_enable()`：退出拖拽并恢复 Enable、User0、Tool0 和运动安全参数。
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

## 生产模式与示教命令

- `nx_routes.json operation_mode=1`：所有未单独配置的生产点执行
  `vision_icp_align_and_move_b`。
- `nx_routes.json operation_mode=2`：所有未单独配置的生产点执行
  `apriltag_locate_and_touch`，在同一任务内完成定位、预接近和戳点。
- `routes[]` 中的 `mode=1/2` 可以按 `mapid + poseid + label` 覆盖全局模式。
- `vision_icp_teach command=0/start`：必须携带 `mapid + poseid`，进入拖拽并
  开启一次进程内示教会话；`label` 省略时自动分配下一个 `P` 序号。
- `vision_icp_teach command=1/record_a`：记录 A 点云模板和法兰位姿。
- `vision_icp_teach command=2/record_b`：记录 B 戳点位姿，依次完成
  `StopDrag → EnableRobot → RobotMode=5`，再保存 A→B 和完整站点记录到
  `tasks/<mapid>/<poseid>/<mapid>_<poseid>.json`，随后自动结束本次示教。
- `vision_icp_teach command=3/finish`：仅用于中途取消或异常恢复；退出拖拽、
  恢复使能，不写入一个不完整的示教点。
- `vision_icp_teach command=4/status`：读取示教会话和机器人状态。
- 旧 `type1 + mapid + poseid` 等价于 `command=0/start`。会话有效期间，
  `mechanical_arm_command=1/2` 分别重解释为记录 A、记录 B 并完成；空闲时
  仍保持旧复位/使能语义。旧 `type2` 只查询当前站点记录，禁止落入生产动作。
- 生产请求按 `mapid + poseid` 加载站点文件；`label` 为空按文件顺序执行所有
  点，`label=P3,P1` 按请求顺序只执行 P3、P1。缺文件或缺标签直接失败，绝不
  回退到最后一次全局示教数据。

本机交互入口为 `ros2 run vision_arm_executor icp_teach`。本机命令与远程接口
调用同一个 RPC 状态机，避免产生两套不同的示教数据格式。

## Asdun 命令 1～6 的 CR5 映射

与旧 `RobotDemo.cpp` 一致，只要非显式视觉 action 的 JSON 含有 `command=1..6`，
网关就优先按机械臂命令解析，不要求 `type=mechanical_arm_command`。

| command | 旧 Asdun 动作 | 当前 CR5 动作 | 兼容结果字段 |
|---|---|---|---|
| 1 | 移动到旧 Asdun 固定复位关节点 | 调用 CR5 `ResetRobot` 控制器复位；不套用另一台机械臂的关节值 | `fuwei` |
| 2 | 上电 | `EnableRobot`，并重新设置速度、碰撞等级、User0/Tool0 | `shangdian` |
| 3 | 下电 | `DisableRobot`，同时撤销生产运动许可 | `xiadian` |
| 4 | 自动模式 | 进入 CR5 拖拽/回拖模式并确认 `RobotMode=6` | `tuozhuai`（兼容别名 `zidong`） |
| 5 | 手动模式 | `StopDrag → EnableRobot` 并确认 `RobotMode=5`；ICP 示教中则安全取消未完成会话 | `quxiaotuozhuai`（兼容别名 `shoudong`） |
| 6 | 清除报警 | `ClearError` | `qingchu` |

CR5 没有直接等价于埃斯顿 `ToAuto/ToReady` 的控制器状态，因此 4/5 被明确改成
便于现场验证的拖拽/退出拖拽。application 的路由生产任务由网关在单次任务范围内
自动开放和关闭运动许可；显式许可消息仅供本地直接测试。命令 1 也不能直接复用旧 Asdun 的
`[86,23,-112,-176,-85,0]`，否则会把另一台机械臂、另一安装环境的关节复位点
应用到 CR5。若现场需要“移动到 CR5 Home”，应单独标定 Home 并增加明确的
`move_home` 动作。

`type1` 本身就是 `vision_icp_teach command=start`，不要再给同一个 JSON 增加
`command` 字段。规范流程是三条独立消息：`type1 + mapid + poseid` 开始示教，
随后 `mechanical_arm_command=1` 记录 A，最后 `mechanical_arm_command=2`
记录 B、保存并结束。示教中发送 `command=5` 可中止、不保存未完成点。

## 所有权

- Backend 负责 demo 类、其内部字段以及 ROS2 机械臂调用。
- VisionExecutor 负责请求状态、互斥、超时、取消、运动许可、安全边界和数据版本。
- NX gateway 只负责旧 TCP JSON 与 RPC action 的协议转换。
- `application/` 不依赖或导入任何本地算法模块。

内部仍保留 ICP 和 AprilTag 两个 CR5 实现：AprilTag 额外提供夹爪和拖拽操作，ICP
额外提供关节兜底。它们通过统一 `RobotState` 契约暴露给执行器，避免在没有完整硬件
回归的情况下强行合并运动实现。
