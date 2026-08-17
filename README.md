# CR5 手眼标定 + ICP 视觉伺服

复现记录见 [`docs/icp_servoing_reproduce.md`](docs/icp_servoing_reproduce.md)。

## 启动

```bash
# 终端1: 机械臂驱动
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py

# 终端2: 相机 (D455)
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true

# 终端3: ICP伺服
cd ~/Robot_Arm_Project && source install/setup.bash
bash scripts/run_icp_servo.sh
```

## 项目结构

```
src/icp_servoing/          # ICP视觉伺服核心模块
├── robot.py               # CR5控制: init/movj/movl/get_tool/wait_stop/recover
├── servoing.py            # VisualServo: record_template/step/align
├── icp.py                 # 多尺度ICP (20→10→5mm)
├── pointcloud.py          # 点云处理: voxel_down/fuse_frames/cloud_to_xyz
├── handeye.py             # 加载标定矩阵 X
└── main.py                # CLI入口

scripts/
├── run_icp_servo.sh       # ICP伺服启动脚本
├── handeye_chessboard.py  # 棋盘格手眼标定
├── verify_calib.py        # 标定精度验证
├── test_convention.py     # ToolVectorActual旋转约定测试(96种)
├── icp_accuracy_test.py   # ICP补偿精度自动化测试
├── move_robot.py          # 交互式机械臂移动
├── handeye_chessboard_result.json  # 标定结果
└── calib_middle_image/     # 标定中间产物(棋盘格检测可视化)
```

## 手眼标定流程

```bash
python3 scripts/handeye_chessboard.py
```

1. 棋盘格(8×11内角, 20mm格)固定在视野内
2. 16个位姿自动采集: 小幅关节偏移(±6°~±15°)
3. 质量过滤: 重投影<0.5px / 角点完整 / 距离150-2000mm
4. AX=XB 求解 → `handeye_chessboard_result.json`

```bash
# 验证精度
python3 scripts/verify_calib.py
# 输出: 8帧棋盘格世界坐标一致性 RMS (<3mm=优秀)
```

## ICP 伺服流程

按 `r` 录制模板 → 按 `1`~`6` 偏移 → 按 `m` 单步补偿 → 按 `a` 闭环对齐

```bash
bash scripts/run_icp_servo.sh
```

| 键 | 功能 |
|----|------|
| `r` | 录制模板(5帧融合→5mm压缩→预建3层KDTree) |
| `m` | 单步: 采点云→ICP→70%补偿→MovL |
| `a` | 闭环迭代(最多15次)→|t|<10mm & |r|<1°收敛，ERROR自动恢复 |
| `1`/`2`/`3` | 反向偏移(小/中/大) |
| `4`/`5`/`6` | 正向偏移(小/中/大) |

**补偿公式**: `T_target = T_cur × (X·T_icp·X⁻¹)⁻¹ × 0.7`

## 关键约定

| 项目 | 约定 |
|------|------|
| ToolVectorActual RPY | `Rot.from_euler('xyz', [rx,ry,rz])`  intrinsic XYZ |
| 手眼矩阵 X | `T_camera_in_tool` (4×4, mm) |
| AX=XB 构造 | `A=G_j⁻¹·G_i` (机器人), `B=C_j·C_i⁻¹` (相机) |
| ICP 内部单位 | 米 (m), 链路层毫米 (mm), T_init 平移需 ÷1000 |
| CR5 初始化 | `ClearError→DisableRobot→EnableRobot→SpeedFactor→SetCollision(5)`  **不调PowerOn** |
| CR5 运动 | JointMovJ (可靠), MovL (固件兼容性待验证) |
| 停止检测 | ToolVectorActual 连续3帧 Δ<0.3mm |



## ICP Pyramid 加速 (25s → 0.5s)

### 问题: 模板 93万点 × 3级多尺度 × KDTree每次重建 = 单次ICP 25s+

### Pyramid 方案

**录制时 (record_template, 只做一次):**

```
5帧融合 → 5mm压缩(~80K点)
   ↓ 预建3层 cKDTree, 存为 self._ref_pyramid
┌─────────────────────────────────────────────────┐
│ L0: 20mm体素 → ~5K点  + cKDTree  dmax=100mm   │
│ L1: 10mm体素 → ~20K点 + cKDTree  dmax=50mm    │
│ L2:  5mm体素 → ~80K点 + cKDTree  dmax=25mm    │
└─────────────────────────────────────────────────┘
```

**每次 ICP (step), 3级逐级传递:**

```
T_init (初值, 米)
   ↓
┌─ L0 (20mm): 当前扫描→20mm下采样 → T_acc变换 → tree.query(workers=-1) ≈0.01s
│   在 ~5K模板点中找最近邻 (dmax=100mm) → SVD求R,t → 更新 T_acc
│   ↓
├─ L1 (10mm): 当前扫描→10mm下采样 → T_acc变换 → tree.query() ≈0.02s
│   在 ~20K模板点中找最近邻 (dmax=50mm) → SVD求R,t → 更新 T_acc
│   ↓
└─ L2 (5mm):  当前扫描→5mm下采样 → T_acc变换 → tree.query() ≈0.1s
    在 ~80K模板点中找最近邻 (dmax=25mm) → SVD求R,t → 最终 T_icp (米)
   ↓
T_icp × 1000 → T_icp_mm (毫米) → 手眼补偿链路
```

**每级只做 1 次配准 (不做迭代)**: 粗→中→精逐级传递，累积的 `T_acc` 已经足够精确。

**关键**: `cKDTree(query)` 用预建树直接查找，不重复构建；`tree.query(workers=-1)` 多线程；当前扫描每次按体素下采样后再变换。

## 安全

- `SetCollisionLevel(5)` 每步初始化必调
- 关节限制: J2/J3 ±160°, J5 ±180°
- 速度 15%, 补偿比例 70%
- ERROR后自动完整恢复(含碰撞检测重设)
