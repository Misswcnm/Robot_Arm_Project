# continuous_icp_servo

C++ 连续 ICP 视觉伺服包。低频 ICP 生成并覆盖“最新绝对目标”，高频 `ServoP` 持续重复该目标；新视觉结果到达时直接替换旧目标，不生成手工中间点。

## 结构

```text
PointCloud2 callback
  -> 保存最新点云，5 mm 体素下采样

GetPose timer, 默认 30 Hz
  -> 唯一的笛卡尔反馈来源，不订阅 ToolVectorActual
  -> 保存短时间位姿历史，ICP选取与点云时间最近的GetPose

ICP thread, 默认 6 Hz
  -> current 点云与 ref 模板做 20/10/5 mm 金字塔点到点 ICP
  -> 10 mm残差内追加点到面精修（与icp_servoing一致）
  -> 计算绝对 T_target_base，只覆盖最新目标，不发运动命令
  -> 模板点云/空间索引不可变共享，循环中不再整套复制
  -> 空闲时不刷屏；开启连续伺服后打印 ICP 状态

ServoP timer, 默认 20 Hz
  -> 读取最新 T_target_base 和当前 GetPose
  -> 高频重复发送同一个绝对目标，直到低频ICP覆盖为新目标
  -> 100%目标，不做1/2 mm分步积分或比例补偿
  -> 同时用GetPose打印实际位置/姿态残差
  -> 同一时刻只允许一条ROS服务请求在途，避免队列堆积
  -> 不附带任何可选参数，避免当前 CR5/V4 驱动返回 -20000 参数数量错误
```

## 构建

```bash
cd ~/Robot_Arm_Project
colcon build --packages-select continuous_icp_servo --symlink-install
source install/setup.bash
```

## 运行

先启动机械臂和相机：

```bash
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
```

再启动连续伺服节点。脚本使用 `ros2 run` 而不是 `ros2 launch`，这样键盘输入会直接传给节点：

```bash
bash scripts/run_continuous_icp_servo.sh
```

按键：

| 键 | 功能 |
| --- | --- |
| `r` | 录制 ref 模板，5 帧融合并建立 ICP 金字塔 |
| `1`/`2`/`3` | 小/中/大负向 `JointMovJ` 关节偏移，让当前位置离开 ref |
| `4`/`5`/`6` | 小/中/大正向 `JointMovJ` 关节偏移，让当前位置离开 ref |
| `s` | 开启/关闭连续 `ServoP` 伺服 |
| `x` | 停止伺服并发送 `Stop()` |
| `q` | 退出 |

如果必须用 `ros2 launch`，键盘 stdin 可能不进入节点，可改用 topic 发命令：

```bash
ros2 topic pub --once /continuous_icp_servo/command std_msgs/msg/String "{data: r}"
ros2 topic pub --once /continuous_icp_servo/command std_msgs/msg/String "{data: 4}"
ros2 topic pub --once /continuous_icp_servo/command std_msgs/msg/String "{data: s}"
```

## 安全默认值

- `speed_percent=10`
- 碰撞等级 `5`
- 启动时强制 `User(0)/Tool(0)`，手眼链固定为 `T_base_flange × T_flange_camera`
- ICP 结果超过 `1.0 s` 未更新则停；时间戳使用点云采集时刻，慢计算不会伪装成新结果
- `GetPose` 超过 `0.25 s` 未更新则不发命令，连续 `0.5 s` 无效自动 `Stop()`
- ICP质量门控与 `icp_servoing` 一致：`inliers>=500`、`overlap>=0.03`、`rmse<=60mm`、`|t|<=500mm`、`|r|<=60deg`
- 上述平移/旋转阈值用于拒绝明显错误的ICP，不是ServoP输出限幅
- 高频循环重复的是缓存的绝对目标，不会把同一帧 ICP 误差重复积分
- `GetPose/ServoP/MovJ/Stop` 共用29999 Dashboard socket，节点保证所有请求严格串行
- 上一个 `ServoP` 服务调用未返回时跳过当前周期；首个非零返回立即关闭伺服并 `Stop()`
- `1-6` 偏移键使用 `JointMovJ`，不使用 `ServoP`，避免 ServoP 在当前 CR5 上触发 ERROR
- `1-6` 偏移键读取缓存的 `/joint_states_robot`，不在现场控制路径里临时调用 `GetAngle`
- 点云、JointState、命令和 service 分离 callback group，并固定使用4线程 `MultiThreadedExecutor`

关键参数：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `icp_hz` | 6 Hz | 低频视觉目标更新频率；实际频率不超过ICP计算能力 |
| `servo_hz` | 20 Hz | 高频绝对目标发送频率 |
| `getpose_hz` | 30 Hz | 法兰反馈采样频率 |
| `pose_sync_tolerance_s` | 0.10 s | 点云与GetPose允许的最大时间差 |
| `point_to_plane_max_points` | 80000 | 与icp_servoing一致；点到面精修最多处理点数，越大越准但越慢 |
