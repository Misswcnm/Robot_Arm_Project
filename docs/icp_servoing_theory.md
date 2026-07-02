# ICP 视觉伺服：原理与工程实现

## 问题定义

机械臂末端装有 RGB-D 相机（D455，eye-in-hand）。录制一帧模板点云后，机械臂被移动，通过 ICP 匹配当前点云与模板，驱动机械臂回到模板位姿。

**核心链路**：`ICP(点云) → 手眼变换 → 工具帧位移 → 雅可比反解 → 关节移动`

---

## 1. 数学推导

### 1.1 符号定义

| 符号 | 含义 | 来源 |
|------|------|------|
| `G_i` = `T_base_tool_i` | 工具坐标系在基座下的位姿（4×4, mm） | ToolVectorActual |
| `C_i` = `T_camera_board_i` | 棋盘格在相机下的位姿（4×4, mm） | solvePnP |
| `X` = `T_camera_in_tool` | 手眼矩阵：相机在工具坐标系下（4×4, mm） | 手眼标定 |

### 1.2 手眼标定 AX=XB

棋盘格固定在世界中，对于任意两个机械臂位姿 i, j：

```
G_i · X · C_i = G_j · X · C_j      (棋盘格不变)

⇒  G_j⁻¹ · G_i · X = X · C_j · C_i⁻¹

⇒  A · X = X · B

   A = G_j⁻¹ · G_i     (机器人运动)
   B = C_j · C_i⁻¹     (相机运动)
```

### 1.3 ICP→机械臂补偿

模板录制时：`T_ref = T_base_tool_ref`，`T_camera_ref = T_ref · X`

当前位姿：`T_cur = T_base_tool_cur`，`T_camera_cur = T_cur · X`

机器人初值（相机应如何移动才能对齐模板）：

```
T_init = inv(T_camera_ref) · T_camera_cur    (mm, 相机帧)
```

ICP 精修（m←mm 转换后）：

```
T_icp = multi_scale_icp(current_scan, reference_map, init=T_init)   (m)

T_delta = X · T_icp_mm · X⁻¹               (相机帧→工具帧, mm)
T_correction = inv(T_delta)                  (反向补偿)

T_target = T_cur · T_correction_partial      (部分补偿, 工具帧)
```

雅可比反解：

```
Δx = [T_target[:3,3] - T_cur[:3,3];  rotation_vector(R_target · R_curᵀ)]  (基座帧)
Δθ = J⁺(θ_cur) · Δx                  (关节增量)
θ_target = θ_cur + Δθ
```

---

## 2. ICP 金字塔加速

模板 93 万点 × 多尺度 × KDTree 重建 = 单次 ICP 25s。Pyramid 方案压缩到 <0.5s。

### 2.1 录制时预建（一次）

```
5帧点云融合 → 5mm体素压缩(~80K点)
   ↓ 对同一点集, 三级体素下采样 + cKDTree
┌─────────────────────────────────────────┐
│ L0: 20mm体素 → ~5K点 + cKDTree    dmax=100mm  │
│ L1: 10mm体素 → ~20K点 + cKDTree   dmax=50mm   │
│ L2:  5mm体素 → ~80K点 + cKDTree   dmax=25mm   │
└─────────────────────────────────────────┘
```

### 2.2 每次 ICP 的逐级传递

```
T_init (初值, m)
   ↓
L0: 当前扫描→20mm下采样 → T_acc变换 → cKDTree.query(workers=-1)
    在5K模板点中找 d<100mm 的最近邻 → SVD求R,t → 更新T_acc
   ↓
L1: 当前扫描→10mm下采样 → T_acc变换 → cKDTree.query()
    在20K模板点中找 d<50mm 的最近邻 → SVD修正 → 更新T_acc
   ↓
L2: 当前扫描→5mm下采样 → T_acc变换 → cKDTree.query()
    在80K模板点中找 d<25mm 的最近邻 → SVD精修 → 最终T_icp
   ↓
T_icp × 1000 → 补偿链路 (mm)
```

**关键优化**：
- `cKDTree`（C++）替代 `KDTree`（Python），预建后永久复用
- 每级只做 1 次配准（不迭代），粗→中→精逐级传递已足够
- `workers=-1` 多线程查询

---

## 3. 闭环控制策略

### 3.1 动态补偿比例

| 步数 | 比例 | 逻辑 |
|------|------|------|
| step 1-2 | 70% | 大步快走 |
| step 3-5 | 50% | 中步修正 |
| step 6+   | 20% | 临门一脚 |

### 3.2 单步限幅

```
|平移| ≤ 30mm/步    (防过冲)
```

### 3.3 收敛判据

```
|t| < 10mm 且 |r| < 1°   →  闭环收敛
```

### 3.4 质量门控

ICP 不满足以下任一条件时禁止移动：

- RMSE < 30mm
- overlap > 20%
- inliers > 50
- |t_icp| < 300mm
- |r_icp| < 45°

---

## 4. 工具链坐标系约定

| 项目 | 约定 |
|------|------|
| ToolVectorActual RPY | `Rot.from_euler('xyz', [rx,ry,rz])` — **XYZ intrinsic** |
| AX=XB A/B构造 | `A = G_j⁻¹·G_i`, `B = C_j·C_i⁻¹` |
| 手眼矩阵 X | `T_camera_in_tool` (4×4, mm) |
| ICP单位 | 内部 **米(m)**，手眼链路 **毫米(mm)** |
| T_init 转换 | `T_init_icp[:3,3] = T_init_mm[:3,3] / 1000` |

---

## 5. 硬件安全

```
初始化: ClearError → DisableRobot → EnableRobot → SpeedFactor → SetCollisionLevel(5)
        ⚠️ 绝不调 PowerOn (CR5会下电)
运动:   JointMovJ (关节移动, CR5可靠)
停止:   wait_tool_stable (ToolVectorActual seq-based, 5帧 Δ<0.3mm)
恢复:   ClearError→DisableRobot→EnableRobot→SpeedFactor→SetCollisionLevel(5)
```

---

## 6. 复现步骤

```bash
# 1. 启动
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py        # 机械臂
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true  # 相机

# 2. 手眼标定 (首次)
python3 scripts/handeye_chessboard.py
python3 scripts/verify_calib.py   # 验证 RMS < 3mm

# 3. ICP伺服
bash scripts/run_icp_servo.sh
# r:录制模板 → 1-6:偏移 → m:单步补偿 → a:闭环对齐
```

## 7. 项目结构

```
src/icp_servoing/
├── main.py         # CLI入口 (RobotNode + PCNode 分离)
├── robot.py        # CR5控制 (movj/wait_tool_stable/recover)
├── servoing.py     # 闭环核心 (step/align, 金字塔ICP)
├── pointcloud.py   # 点云处理 (voxel_down, fuse_frames)
├── handeye.py      # 加载标定矩阵
└── icp.py          # 多尺度ICP

scripts/
├── handeye_chessboard.py  # 棋盘格标定
├── verify_calib.py        # 精度验证
└── handeye_chessboard_result.json  # 标定结果
```
