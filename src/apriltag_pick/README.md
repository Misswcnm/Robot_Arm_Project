# D455 + AprilTag + CR5 定位抓取

坐标链：

`T_base_tag = T_base_tool × T_tool_camera(手眼) × T_camera_tag(AprilTag)`

其中 `T_camera_tag` 必须来自去畸变后的 color rectified image。
`apriltag_ros` 的 PnP 使用 `CameraInfo.P` 且不使用畸变参数，不能直接喂
D455 `color/image_raw`。

目标针尖：

`T_base_tcp = T_base_tag × T_tag_tcp`

控制器目标原点：

`p_base_tool = p_base_tcp - R_base_tool × p_tcp_offset_tool`

## 先配置

1. 在 `config/tags.yaml` 填写 Tag 黑色方框外沿真实边长（米）和 ID。
2. 在 `config/pick.yaml` 设置同一个 `tag_frame`。
3. 设置 `tag_to_tcp.xyz_mm`，表示 Tag 中心到针尖接触点的偏移。
   工具姿态保持拖拽结束时的当前姿态；TCP 枢轴标定不包含姿态信息。
4. `tcp_calibration_path` 指向针尖枢轴标定结果，程序会自动补偿
   `tcp_offset_tool_mm`。
5. 首次保持 `execute_enabled: false` 和 `gripper.enabled: false`。

## 构建和启动

```bash
cd ~/Robot_Arm_Project
colcon build --symlink-install --packages-select apriltag_ros apriltag_pick
source install/setup.bash

# 终端 1：CR5（按项目现有方式启动）
ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py

# 终端 2：D455
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=camera enable_color:=true enable_depth:=true

# 终端 3：去畸变 + 检测 + 前台交互控制（推荐，输入方式与 icp_servoing 一致）
ros2 run apriltag_pick interactive
```

也可以拆成两个前台进程：

```bash
# 终端 3：先把 D455 color/image_raw 去畸变
ros2 run image_proc rectify_node --ros-args \
  -r image:=/camera/camera/color/image_raw \
  -r camera_info:=/camera/camera/color/camera_info \
  -r image_rect:=/camera/camera/color/image_rect \
  -p qos_overrides./camera/camera/color/image_raw.subscription.reliability:=best_effort \
  -p qos_overrides./camera/camera/color/image_raw.subscription.durability:=volatile \
  -p qos_overrides./camera/camera/color/camera_info.subscription.reliability:=best_effort \
  -p qos_overrides./camera/camera/color/camera_info.subscription.durability:=volatile

# 终端 4：只启动 AprilTag 检测
ros2 run apriltag_ros apriltag_node --ros-args \
  -r image_rect:=/camera/camera/color/image_rect \
  -r camera_info:=/camera/camera/color/camera_info \
  --params-file "$(ros2 pkg prefix apriltag_pick)/share/apriltag_pick/config/tags.yaml"

# 终端 5：像 icp_servoing 一样前台运行交互控制
ros2 run apriltag_pick pick_node --ros-args \
  --params-file "$(ros2 pkg prefix apriltag_pick)/share/apriltag_pick/config/pick.yaml"
```

控制终端输入：

- `l`：只计算并打印基座系 Tag/TCP 坐标，不运动。
- `t`：进入拖拽模式，手动移动相机寻找 Tag；拖拽中按 `l` 查看定位，
  按 `2` 退出拖拽并重新使能机械臂。
- `p`：一次定位并锁定基座目标 → 沿针轴预戳 → 戳点 → 沿针轴退回。
  靠近后即使相机丢失或遮挡 Tag，也不会中断已锁定的戳点流程。
- `q`：退出。

先连续执行 `l`，用尺或已知点验证基座坐标，尤其确认 Z 和
`tag_to_tcp` 姿态。验证完成后才把 `execute_enabled` 改成 `true`。

程序遵循本项目 CR5 约束：不调用 `PowerOn`，初始化为
`ClearError → DisableRobot → EnableRobot → SpeedFactor → SetCollisionLevel(5)`，
运动只发旧固件支持的 `MovJ(x,y,z,rx,ry,rz)`。
