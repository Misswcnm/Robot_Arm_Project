# YOLO11n-seg + D455 RGB-D + CR5 抓取演示

数据链：`RGB → YOLO11n-seg mask → aligned depth → mask 内深度过滤 → 点云
中心/PCA方向 → T_base_camera → TCP 补偿后的 CR5 抓取`。

默认改为精确的已知物体模板链：`RGB → OpenCV ORB 特征匹配 + RANSAC 单应性
→ 目标四边形 mask → aligned depth → 点云/PCA/抓取`。请用 D455 在实际抓取
视角拍摄一张**只包含目标、纹理清晰且无大面积背景**的 PNG/JPG，保存为
`templates/target.png`（或在 YAML 的 `template_path` 指定）。`s` 会明确报告
匹配点或 RANSAC 内点不足，避免把错误结果送去抓取。纯色、反光严重或外观变化大
的目标不适合 ORB 模板；此时将 `detector_mode` 改回 `yolo11_seg`。

该 demo 默认 `execute_enabled: false`，`s` 只定位。它不自动下载权重；先安装
Ultralytics 并将权重放到配置中的 `model_path`：

```bash
python3 -m pip install ultralytics
mkdir -p ~/Robot_Arm_Project/models
# 将 yolo11n-seg.pt 放到 ~/Robot_Arm_Project/models/yolo11n-seg.pt
cd ~/Robot_Arm_Project
colcon build --packages-select yolo_seg_pick --symlink-install
source install/setup.bash
ros2 run yolo_seg_pick yolo_seg_pick
```

抓取前另开终端启动夹爪步进电机底层节点：

```bash
source ~/Robot_Arm_Project/install/setup.bash
ros2 run step_motor motor_node
```

`p` 的顺序是：发布一次 `/motor_control` 张开命令 → 机械臂预抓取/下探 →
发布一次闭合命令 → 抬升。默认命令与资料一致：`id=1 speed=200 mode=2
angle=30000 sub_divide=32`，张开 `dir=0`、闭合 `dir=1`；可在 YAML 的
`step_gripper` 下调整。

Realsense 必须发布对齐到彩色相机的深度：

```bash
ros2 launch realsense2_camera rs_launch.py camera_name:=camera \
  enable_color:=true enable_depth:=true align_depth.enable:=true
```

控制：`t` 进入拖拽移动相机寻找目标，`2` 安全退出拖拽并重新使能；`s` 运行
一次分割并打印目标中心/PCA（拖拽时也可使用）；`c` 显示当前 YOLO 模型可识别
的类别与 `target_class` 填法；`p` 对最近一次结果执行预抓取、下探、夹爪闭合、
抬升（仅 `execute_enabled=true` 时）；`q` 退出。

每次 `s` 还会发布下列 RViz 话题（frame 是 D455 彩色光学坐标系）：

- `/yolo_seg_pick/annotated_image`：mask、边界、类别和中心点叠加图；在 RViz 添加 `Image`。
- `/yolo_seg_pick/target_cloud`：mask 深度过滤后的彩色相机点云（单位米）；添加 `PointCloud2`。
- `/yolo_seg_pick/target_center`、`/yolo_seg_pick/pca_direction`：3D 中心球和 PCA 主方向箭头；添加两个 `Marker`。

RViz 的 `Fixed Frame` 设为图像 header 的彩色光学 frame（常见为
`camera_color_optical_frame`）。如要与机械臂基坐标同时显示，必须由 D455/手眼
发布对应的 TF；本 demo 仅用手眼矩阵进行抓取计算，不会伪造 TF。

PCA 方向只作为目标长轴诊断打印。实际抓取姿态暂保持当前 Tool0 姿态，避免在未
定义夹爪工具姿态的情况下把 PCA 图像方向直接误用为 CR5 欧拉角。
