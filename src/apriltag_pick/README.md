# D455 + AprilTag + CR5 定位抓取

坐标链：

`T_base_tag = T_base_flange × T_flange_camera(手眼) × T_camera_tag(AprilTag)`

本项目中的 `apriltag_ros` 已针对 D455 原始彩色图修改：PnP 直接使用
`color/image_raw` 的角点和 `CameraInfo.K + CameraInfo.D`，在求解中补偿
镜头畸变，不需要额外启动 `image_proc` 或 rectified 图像话题。若图像与
CameraInfo 分辨率不一致，该帧会被拒绝。

目标针尖：

`T_base_tcp = T_base_tag × T_tag_tcp`

控制器目标原点：

`p_base_flange = p_base_tcp - R_base_flange × p_flange_tip`

## 先配置

1. 在 `config/tags.yaml` 填写 Tag 黑色方框外沿真实边长（米）和 ID。
2. 在 `config/pick.yaml` 设置同一个 `tag_frame`。
3. 设置 `tag_to_tcp.xyz_mm`，表示 Tag 中心到针尖接触点的偏移。
   工具姿态保持拖拽结束时的当前姿态；TCP 枢轴标定不包含姿态信息。
4. `tcp_calibration_path` 指向针尖枢轴标定结果，程序会自动补偿
   `tcp_offset_flange_mm`（旧文件的 `tcp_offset_tool_mm` 仍兼容读取）。
5. 首次保持 `execute_enabled: false` 和 `gripper.enabled: false`。

手眼结果保存在 `scripts/handeye_calib_runs/<本次目录>/`；所有功能包统一读取
`scripts/active_handeye_calibration.json`。该文件只保存一个相对于 `scripts/`
的结果地址，例如：

```json
"handeye_calib_runs/handeye_20260713_171741/handeye_chessboard_result.json"
```

切换标定时只修改这一行，然后重启相关节点。

手眼标定必须声明 `T_base_flange(User0)` 与 `T_flange_camera`；TCP 标定必须
声明 `translation_from_flange_origin_to_physical_tip`。`Tool0` 不属于手眼外参定义。

## 构建和启动

```bash
cd ~/Robot_Arm_Project
colcon build --symlink-install --packages-select apriltag_ros apriltag_pick
source install/setup.bash

# 终端 1：CR5（按项目现有方式启动）

ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
# 终端 2：D455
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=camera enable_color:=true enable_depth:=true

# 终端 3：原始图像检测 + 前台交互控制（推荐）
ros2 run apriltag_pick interactive
```

也可以拆成两个前台进程：

```bash
# 终端 3：只启动 AprilTag 检测
ros2 run apriltag_ros apriltag_node --ros-args \
  -r image_rect:=/camera/camera/color/image_raw \
  -r camera_info:=/camera/camera/color/camera_info \
  --params-file "$(ros2 pkg prefix apriltag_pick)/share/apriltag_pick/config/tags.yaml"

# 终端 4：像 icp_servoing 一样前台运行交互控制
ros2 run apriltag_pick pick_node --ros-args \
  --params-file "$(ros2 pkg prefix apriltag_pick)/share/apriltag_pick/config/pick.yaml"
```

控制终端输入：

- `l`：定位并缓存基座系Tag、针尖目标和法兰目标，不运动；同时把原图、带四角编号/
  绿色边框/红色中心点的验证图、同帧PnP三维坐标轴/立方体图，以及对应
  像素/CameraInfo/PnP重投影JSON保存到
  `scripts/apriltag_validation_runs/`。
- `d`：进入拖拽模式。
- `v`：拖拽中把针尖放到Tag中心后，记录最近一次 `l` 对应的 `tip_plus`
  真值并输出 `预测−真值`；不会自动退出拖拽。
- `t`：退出拖拽并重新使能机械臂。
- `p`：使用最近一次 `l` 的缓存目标完成预接近和抓取，不会重新识别Tag；
  靠近后即使相机丢失或遮挡也不会改变缓存目标。
- `q`：退出。

真值验证顺序为 `l → d → 移动针尖到中心 → v → t`；抓取顺序为
`l → p`。执行 `p` 需要 `execute_enabled: true`。

每次 `l` 会额外生成 `locate_*_pnp3d.jpg`：X/Y/Z轴分别为红/绿/蓝，
紫色线框为从Tag平面沿 `+Z_tag` 投影的三维立方体，青色圆为PnP回投的四角。
对应JSON的 `pnp3d` 字段保存同帧 `T_camera_tag`、rvec、tvec、每角点误差和
重投影RMS。`tag_size_m` 必须与 `tags.yaml` 中的实际黑色外框边长一致。

程序不调用 `PowerOn`。CR5 已处于 mode=5 时只重设速度、碰撞等级和
User0/Tool0，不执行 Disable/Enable；仅非使能状态恢复时才执行
`ClearError → DisableRobot → EnableRobot`。运动使用旧固件支持的
`MovJ(x,y,z,rx,ry,rz)`。
