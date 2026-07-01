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
- ❌ MovL — CR5 固件不支持 (res=0 但不动, 还会触发 ERROR)
- ❌ RequestControl — 返回 -10000 (CR5 不支持)

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
G_i = T_tool_in_base_i     (来自 ToolVectorActual)
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

1. `parseTool.cpp` MovJ/MovL 用了 V4 命名参数 (`joint={}`) → CR5 返回 -30001
   → 修复: 改用位置参数 `MovJ(j1,j2,...)` 和 `JointMovJ(j1,j2,...)`

2. `cr5_fk()` URDF rpy 参数 roll/yaw 互换 → FK 全错
   → 修复: J2/J5/J6 的 `_urdf_T` rpy 参数修正

3. `cr5_fk()` 输出单位是 **米**, 未乘 1000 → 所有数值缩小 1000 倍
   → 修复: `T[:3,3] *= 1000`

4. ToolVectorActual rx/ry/rz 旋转顺序用错 (`zyx` 应为 `xyz`)
   → 标定 RMS 从 108mm 降到 2.6mm

5. AX=XB 的 A/B 构造对调了相机和机器人
   → 旧: A=C⁻¹C, B=G⁻¹G → 修复: A=G⁻¹G, B=C·C⁻¹

6. PowerOn 让 CR5 下电 → 跳过

7. ICP 需要 T_init 的平移单位转换: 手眼链路(mm) ÷1000 → ICP内部(m); ICP输出(m) ×1000 → 补偿链路(mm)

## 安全
- `SetCollisionLevel(5)` 必须在 init_robot 中调用 (最高灵敏度碰撞检测)
- 关节限制: J2/J3 ±160°, J5 ±180°
- 速度建议 15-30%

## 环境
- Ubuntu 22.04, ROS2 Humble
- numpy 2.2.6 (与系统 cv2 不兼容, 需 `pip install opencv-python-headless`)
- DoBot 驱动: `cr_robot_ros2` 包, launch: `dobot_bringup_ros2.launch.py`
