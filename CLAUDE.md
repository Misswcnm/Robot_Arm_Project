# Robot Arm Project — CR5 机械臂 + RealSense D455

## 硬件
- 机械臂: Dobot CR5, IP 192.168.1.6
- 相机: RealSense D455, eye-in-hand (装在 J5 末端)
- 棋盘格: 18×24cm, 9×12 格, 每格 20mm, 内角点 8×11

## CR5 初始化序列 (关键!)

```python
ClearError → DisableRobot → EnableRobot → SpeedFactor → SetCollisionLevel(5)
```
**⚠️ 绝不调用 PowerOn!** CR5 的 PowerOn 会下电 (mode 5→3→4)

## CR5 运动指令
- ✅ JointMovJ (mode=True) — 唯一可靠的运动方式
- ✅ MovJ 笛卡尔位置参数格式 — `MovJ(x,y,z,rx,ry,rz)` 实测可接收
- ❌ MovL — CR5 固件不支持 (res=0 但不动, 还会触发 ERROR)
- ❌ RequestControl — 返回 -10000 (CR5 不支持)

### MovJ TCP格式实测规范 (CR5旧固件)

测试脚本:
```bash
python3 scripts/test_movj_tcp_formats.py --host 192.168.1.6 --current --execute
```

实测结论:
```text
✅ MovJ(x,y,z,rx,ry,rz)                         -> 0
✅ MovJ(x,y,z,rx,ry,rz,0,0,20,20,0)             -> 0
✅ JointMovJ(j1,j2,j3,j4,j5,j6)                 -> 0

❌ MovJ(pose={x,y,z,rx,ry,rz})                  -> -30001
❌ MovJ(pose={...},user=0,tool=0,a=20,v=20,cp=0)-> -30001
❌ MovJ(x,y,z,rx,ry,rz,a=20,v=20,cp=0)          -> -1
❌ JointMovJ(j1,j2,j3,j4,j5,j6,a=20,v=20,cp=0)  -> -1
❌ MovJ(joint={j1,j2,j3,j4,j5,j6},...)          -> -30001
```

因此 `dobot_msgs_v4/srv/MovJ` 的 `mode=False` 必须拼成:
```text
MovJ(x,y,z,rx,ry,rz)
```

不要拼成 V4 文档里的:
```text
MovJ(pose={x,y,z,rx,ry,rz},user=0,tool=0,a=20,v=20,cp=0)
```

这台控制器会对 `pose={...}` 返回 `-30001`，对命名速度参数 `a/v/cp` 返回 `-1`。主流程中 `movj_pose()` 不传 `user/tool/a/v/cp`，速度使用全局 `SpeedFactor` 控制。

### GetPose 与 User/Tool 实测规范（CR5旧固件）

这台控制器不能可靠使用文档中的带参形式：

```text
GetPose(user = 0, tool = 0)
```

项目统一使用三条全局指令，再读取无参数 GetPose：

```text
User(0)
Tool(0)
GetPose()
```

TCP 端口实测：

```text
User(0) -> 0
Tool(0) -> 0
GetPose() -> 0,{x,y,z,rx,ry,rz}

User(1) -> -1       # 用户坐标系1未配置，不可使用
Tool(1) -> 0        # 当前返回位姿与Tool0相同，但项目仍只使用Tool0
Tool(5) -> 0        # 返回完全不同的位姿，属于另一套工具坐标系
```

标准测试命令：

```bash
printf 'User(0)\nTool(0)\nGetPose()\n' | timeout 5s nc -w 4 192.168.1.6 29999
```

测试前必须停止 ROS 驱动，避免两个客户端同时访问 29999 端口。ROS 服务
`GetPose.srv` 虽然有 `user/tool` 字段，但当前 `parseTool.cpp` 故意生成
`GetPose()`；坐标系由前置的 `User(0)` 和 `Tool(0)` 锁定。

所有手眼、TCP、ICP 模板和 AprilTag 真值位姿必须在同一个 User0/Tool0
约定下采集。标定 JSON 必须记录 `coordinate_frames.user_index=0` 和
`coordinate_frames.tool_index=0`；坐标系未知的旧标定不得用于自动运动。

## 旋转约定 (重要!)

CR5 的 `rx,ry,rz` 是 **XYZ intrinsic Euler**:
```python
R = Rot.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
# 不是 'zyx'! (zyx 会导致 ~100mm 误差)
```
经 `test_convention.py` 遍历 96 种约定验证, `xyz +++` RMS=2.6mm, `zyx +++` RMS=108mm.

## 手眼标定 AX=XB

正确推导 (棋盘格固定, eye-in-hand):
```
G_i = T_tool0_in_base_i    (来自 User(0)/Tool(0) 后的 GetPose())
C_i = T_camera_board_i     (来自 solvePnP)
X   = T_camera_in_tool     (手眼矩阵)

G_i @ X @ C_i = G_j @ X @ C_j  (棋盘格不变)
→ G_j⁻¹ @ G_i @ X = X @ C_j @ C_i⁻¹
→ A = G_j⁻¹ @ G_i     (机器人运动)
→ B = C_j @ C_i⁻¹     (相机运动)
→ A @ X = X @ B
```

## 关键脚本

| 脚本 | 功能 |
|------|------|
| `scripts/handeye_chessboard.py` | 棋盘格手眼标定 (16 位姿 + 质量过滤 + RobotMode 等停) |
| `scripts/verify_calib.py` | 标定精度验证 (8 位姿 + 离群剔除) |
| `scripts/test_convention.py` | 旋转约定测试 (96 种) |
| `scripts/icp_servoing.py` | ICP 模板匹配 + 视觉伺服 |
| `scripts/move_robot.py` | 交互式机械臂移动 (按键 j1..j6) |

## 调试中的关键错误

1. `parseTool.cpp` MovJ/MovL 用了 V4 命名参数 (`pose={}` / `joint={}`) → CR5 返回 -30001
   → 修复: 改用位置参数 `MovJ(x,y,z,rx,ry,rz)` 和 `JointMovJ(j1,j2,...)`

2. `cr5_fk()` URDF rpy 参数 roll/yaw 互换 → FK 全错
   → 修复: J2/J5/J6 的 `_urdf_T` rpy 参数修正

3. `cr5_fk()` 输出单位是 **米**, 未乘 1000 → 所有数值缩小 1000 倍
   → 修复: `T[:3,3] *= 1000`

4. GetPose rx/ry/rz 旋转顺序用错 (`zyx` 应为 `xyz`)
   → 标定 RMS 从 108mm 降到 2.6mm

5. AX=XB 的 A/B 构造对调了相机和机器人
   → 旧: A=C⁻¹C, B=G⁻¹G → 修复: A=G⁻¹G, B=C·C⁻¹

6. PowerOn 让 CR5 下电 → 跳过

7. ICP 需要 T_init 的平移单位转换: 手眼链路(mm) ÷1000 → ICP内部(m); ICP输出(m) ×1000 → 补偿链路(mm)

8. RobotMode 解析必须保留在 `get_mode()` 内并做回归测试:
   `robot_return="{5}" → 5`。拖拽状态链为
   `mode 5 → StartDrag → mode 6 → StopDrag → EnableRobot → mode 5`；
   仅 mode 5 可发送 MovJ，禁止用高频 RobotMode 轮询替代服务结果检查。

9. 枢轴 TCP 标定只提供针尖平移 `tcp_offset_tool_mm`，不提供针尖姿态。
   AprilTag 戳点时保持拖拽选定的当前工具姿态；Tag 仅决定接触点位置。
   预戳/退刀沿标定的“工具原点→针尖”轴线，禁止固定沿基座 Z 运动。

10. AprilTag 一次定位不能强制在短时间内攒满固定帧数。只校验最新帧时效，
    再对时间窗内已有的至多 N 帧做融合；检测帧率低时也必须允许单帧定位。

11. AprilTag 当前使用 D455 `color/image_raw`，PnP 必须同时使用同分辨率
    `CameraInfo.K` 和 `CameraInfo.D` 做畸变建模。不得把 raw 图与只适用于
    rect 图的零畸变/P 矩阵逻辑混用；图像和 CameraInfo 尺寸不一致时拒绝估计。

12. 多份手眼文件不能只按“最新/ICP正在用”直接替换。相机未移动时，
    不同手眼矩阵应接近；若差异达到厘米/数十度，必须先做交叉验证或现场
    已知点验证。`X_camera_in_tool.matrix` 是权威字段；显示用 `rpy_deg`
    也必须按 CR5 的 `xyz` 约定导出，避免把 `zyx` 显示误当成标定约定。

13. 眼在手上的相机靠近目标后会丢失/遮挡 AprilTag。物体静止时按 `p`
    只在运动前定位一次并锁定基座系目标；预戳、接触和退刀不得要求二次识别。

## 安全
- `SetCollisionLevel(5)` 必须在 init_robot 中调用 (最高灵敏度碰撞检测)
- 关节限制: J2/J3 ±160°, J5 ±180°
- 速度建议 15-30%

## 环境
- Ubuntu 22.04, ROS2 Humble
- numpy 2.2.6 (与系统 cv2 不兼容, 需 `pip install opencv-python-headless`)
- DoBot 驱动: `cr_robot_ros2` 包, launch: `dobot_bringup_ros2.launch.py`
