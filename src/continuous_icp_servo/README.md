# continuous_icp_servo

C++ 连续 ICP 视觉伺服实验包。目标是用低频 ICP 更新视觉误差，用高频 `ServoP` 发送小步笛卡尔目标，避免 `MovJ/MovL` 队列命令积压旧目标。

## 结构

```text
PointCloud2 callback
  -> 保存最新点云，5 mm 体素下采样

ICP thread, 默认 6 Hz
  -> current 点云与 ref 模板做 20/10/5 mm 金字塔 ICP
  -> 只更新最新 T_icp，不发运动命令
  -> 空闲时不刷屏；开启连续伺服后打印 ICP 状态

ServoP timer, 默认 20 Hz
  -> 读取最新 T_icp 和当前 ToolVectorActual
  -> 每个 ICP 结果最多消费一次，限幅 1 mm / 0.15 deg
  -> 发送 ServoP(x,y,z,rx,ry,rz)
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
- ICP 结果超过 `2.0 s` 未更新则停
- ICP 质量门控默认要求 `overlap>=0.12`、`rmse<=25mm`、`|r|<=10deg`
- 同一帧 ICP 结果只发一次 ServoP，避免低频 ICP 被高频 ServoP 重复积分导致漂移
- 上一个 `ServoP` 服务调用未返回时跳过当前周期，避免请求堆积
- `1-6` 偏移键使用 `JointMovJ`，不使用 `ServoP`，避免 ServoP 在当前 CR5 上触发 ERROR
- 初始化和 `GetAngle -> JointMovJ` 使用 ROS2 response 回调链，不用 `future.wait_for()` 阻塞等待
