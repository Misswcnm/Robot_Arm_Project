# smolVLA + MoveIt 联合控制 CR5 机械臂 — 整合方案

## 0. 一句话结论

本仓库已经具备 **MoveIt ↔ 真实 CR5 的桥接链路**（MoveIt 规划 → FollowJointTrajectory action → 驱动 `ServoJ` 下发）。
smolVLA 只需要再补一层 **推理循环 + 动作空间映射**：把模型输出的相对末端动作转成绝对目标，
交给 MoveIt 规划执行（离散）或直接走 `ServoP` 伺服（实时）。

---

## 1. 现有架构速览

| 模块 | 包 / 文件 | 作用 |
|------|-----------|------|
| 驱动 | `dobot_bringup_v4`（包名 `cr_robot_ros2`） | TCP 29999 控制 + 30004 反馈，暴露 ROS2 服务 |
| 消息定义 | `dobot_msgs_v4` | 100+ 个 `.srv`（MovJ/MovL/ServoJ/ServoP/GetPose/…） |
| MoveIt 配置 | `cr5_moveit` | URDF/SRDF + KDL 运动学，**硬件用 mock**（纯规划） |
| 关节状态桥 | `dobot_moveit/dobot_moveit/joint_states.py` | `/joint_states_robot` → `/joint_states`（MoveIt 读这个） |
| 轨迹执行桥 | `dobot_moveit/dobot_moveit/action_move_server.py` | `FollowJointTrajectory` action → `ServoJ` 流式下发 |
| 测试客户端 | `servo_action/servo_action/action_move_client.py` | 发 action 验证链路 |

### 关键事实

- 驱动发布话题：`/joint_states_robot`（`joint1`~`joint6`）、`/dobot_msgs_v4/msg/RobotStatus`、`/dobot_msgs_v4/msg/ToolVectorActual`。
- 驱动服务（控制用）：`MovJ`（`mode=true` → `JointMovJ`，`mode=false` → `MovJ`）、`MovL`、`ServoJ`、`ServoP`、`GetPose`、`GetAngle`、`RobotMode` 等。
- **`cr5_moveit/config/cr5_robot.ros2_control.xacro` 用的是 `mock_components/GenericSystem`**，MoveIt 自己不动真机；
  真机执行靠上面的 `action_move_server.py` 转发 `ServoJ`。

### 控制链路（真实机械臂）

```
smolVLA(推理) ──ΔEE(相对末端)──> 动作映射 ──绝对位姿──> MoveIt(规划/IK)
                                                            │
                                                            ▼ JointTrajectory
                                   /cr5_group_controller/follow_joint_trajectory (action)
                                                            │
                                   action_move_server.py  (rad→deg)
                                                            │
                                   驱动 /dobot_bringup_ros2/srv/ServoJ ──TCP──> CR5
```

---

## 2. 两种控制模式

### 模式 A：离散规划（推荐先跑通）

smolVLA 输出相对末端位移 → 叠加到当前 `GetPose` 得到**绝对目标位姿** →
MoveIt `set_pose_target()` / `compute_cartesian_path()` → 规划 → 已有 bridge 执行。

- 优点：带避障/关节限位校验，复用现有 `action_move_server.py`，改动最小。
- 缺点：每步一次规划，闭环频率低（Hz 级），不适合快速伺服。

### 模式 B：实时伺服（VLA 闭环，后续目标）

smolVLA 输出小增量 → 转绝对末端位姿 → 直接调 `ServoP`（或 `ServoJ`）按推理频率下发，
跳过 MoveIt 在线规划，只保留 URDF 做 FK/碰撞监控。

- 优点：闭环频率高，符合 VLA 连续控制范式。
- 缺点：**必须先现场验证旧固件对 `ServoP/ServoJ` 的支持**（CLAUDE.md 只实测过 MovJ/JointMovJ，
  `ServoP/ServoJ` 未验证，且 `ServoJ` 有 `t∈[0.004,3600]` 参数约束）。

---

## 3. 落地步骤

### 3.1 启动顺序

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
export IP_address=192.168.1.6
export DOBOT_TYPE=cr5

# ① 驱动
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py

# ② MoveIt + 桥（joint_states.py + action_move_server.py）
ros2 launch dobot_moveit dobot_moveit.launch.py

# ③ 你的 smolVLA 推理节点（需自写）
ros2 run <your_pkg> smolvla_agent
```

初始化序列（服务调用，务必遵守，见 CLAUDE.md）：

```text
ClearError → DisableRobot → EnableRobot → SpeedFactor(15~30) → SetCollisionLevel(5) → User(0) → Tool(0)
# 绝不调用 PowerOn（CR5 会下电）
```

### 3.2 核心 ROS2 接口速查

| 用途 | 服务/话题 | 请求字段 | 单位 |
|------|-----------|----------|------|
| 读当前末端位姿 | `/dobot_bringup_ros2/srv/GetPose` | `{user:0, tool:0}` | mm / deg, 顺序 `[x,y,z,rx,ry,rz]` |
| 读关节角 | `/dobot_bringup_ros2/srv/GetAngle` | `{}` | deg |
| 关节伺服 | `/dobot_bringup_ros2/srv/ServoJ` | `{a..f, param_value:["t=0.05"]}` | deg |
| 笛卡尔伺服 | `/dobot_bringup_ros2/srv/ServoP` | `{a..f, param_value:[...]}` | mm / deg |
| 点到点 | `/dobot_bringup_ros2/srv/MovJ` | `{mode, a..f, param_value:[]}` | mode=true→关节 |
| 查模式 | `/dobot_bringup_ros2/srv/RobotMode` | `{}` | 见手册表 |
| 关节状态 | `/joint_states_robot` | — | rad（话题） |
| 末端位姿反馈 | `/dobot_msgs_v4/msg/ToolVectorActual` | — | mm / deg |

### 3.3 smolVLA 推理节点骨架（要点）

```python
# 每个控制周期
obs_img  = 取相机 RGB（D455 color/image_raw，rect 后）
text     = 自然语言指令
action   = vla.infer(obs_img, text)        # 7维: [Δx,Δy,Δz,Δrx,Δry,Δrz, gripper]，归一化 [-1,1]

# 反归一化回物理量（读 checkpoint 的 normalization 配置）
dx,dy,dz,drx,dry,drz,grip = denormalize(action)

# 当前位姿(基座系, mm/deg) = GetPose()
tgt = curr + [dx,dy,dz,drx,dry,drz]

# 模式A: 交给 MoveIt
moveit.set_pose_target(tgt) ; plan_and_execute()
# 模式B: 直接 ServoP(tgt)（t 参数给一个周期时长）
```

---

## 4. smolVLA 动作空间映射（易错点）

1. **坐标系**：VLA 输出的相对位移通常在**基座系**；而你的相机是 eye-in-hand（末端系）。
   先确认所用 checkpoint 的动作/观测帧约定。若输出在末端/相机系，必须用当前 `GetPose` 的
   `rx,ry,rz`（**CR5 是 `xyz` intrinsic Euler**，不是 `zyx`）把增量旋到基座系再叠加。
2. **归一化**：反归一化必须与训练集一致（LeRobot 风格通常按数据集统计量做 delta 归一化），
   用错范围会导致动作爆炸或不动。
3. **旋转单位**：模型输出角度通常弧度或归一化；落到 `ServoP`/`MovJ` 要转成**度**。
4. **末端 vs 工具**：所有目标都必须在 `User(0)/Tool(0)` 约定下（与手眼/TCP 标定一致），
   否则差出一个手眼矩阵（厘米级误差）。
5. **抓取**：CR5 无内置夹爪服务，`gripper` 维需映射到你实际的 DO/ToolDO/AO 或 Modbus 接口。

---

## 5. 联调前必须验证/修改的点

- [ ] **action 命名空间对齐**：`action_move_server.py` 注册在
  `/{DOBOT_TYPE}_group_controller/follow_joint_trajectory`（= `/cr5_group_controller/follow_joint_trajectory`）。
  而 `cr5_moveit/config/moveit_controllers.yaml` 写的是 `action_ns: follow_joint_trajectory`。
  MoveIt 找不到 action 时，把 `action_ns` 改成完整路径 `cr5_group_controller/follow_joint_trajectory`。
- [ ] **`ServoJ`/`ServoP` 实测**：旧固件只验证过 MovJ/JointMovJ 位置参数格式；
  `ServoP` 需低速验证（`MovL` 已知不支持，勿依赖）。
- [ ] **关节限位**：J2/J3 ±160°、J5 ±180°；MoveIt `joint_limits.yaml` 已含，但 bridge 直接走 `ServoJ`
  时绕过 MoveIt，需自己在映射层限幅。
- [ ] **单实例**：联调时只跑一个驱动实例，不要同时用 `nc`/TCP 客户端直连 29999，否则应答错配。
- [ ] **完成判定**：`ServoJ`/`MovJ` 返回 `res=0` 只表示接收，到位要靠 `RobotMode` 7→5 + `GetPose` 复核。

---

## 6. 参考文件

- 驱动服务与手册：`dobot_bringup_v4/`、`CR5_ROS2_操作手册.md`
- 轨迹执行桥：`dobot_moveit/dobot_moveit/action_move_server.py`
- 关节状态桥：`dobot_moveit/dobot_moveit/joint_states.py`
- MoveIt 配置：`cr5_moveit/config/`（`moveit_controllers.yaml`、`kinematics.yaml`、`joint_limits.yaml`）
- 协议文档：`V4新增指令/Dobot TCP_IP二次开发接口文档_V4.6.5_20251015_cn.pdf`
