# TCP 标定方案：拖拽固定点法

## 目标

给新买的夹爪或工具求 TCP 偏移，即求出工具坐标系下真正工作点的位置：

```text
p_tcp_tool = [x, y, z] mm
```

标定后，机械臂的末端控制、ICP 伺服、抓取点规划都应该使用这个 TCP，而不是默认法兰中心。

## 方法

采用固定尖点 pivot calibration。

准备一个固定不动的尖点，例如：

- 桌面上固定一根针。
- 夹爪尖端轻触同一个小孔。
- 工具尖端抵住固定锥孔中心。

拖拽机械臂，让工具尖端始终接触同一个固定点，但改变末端姿态。每记录一次，得到当前法兰/工具坐标。脚本按当前 Dobot 反馈数据验证通过的 RPY 约定计算旋转矩阵：

```text
R = Rz(rz) * Ry(ry) * Rx(rx)
```

也就是：

```text
T_base_tool_i = [R_i, t_i]
```

因为真实 TCP 尖端在基坐标系下固定不动，所以有：

```text
R_i * p_tcp_tool + t_i = p_fixed_base
```

把 N 组姿态叠起来：

```text
[R_i  -I] [p_tcp_tool ] = -t_i
          [p_fixed_base]
```

用最小二乘解出：

```text
p_tcp_tool
p_fixed_base
```

其中真正要写入工具坐标的是 `p_tcp_tool`。

## 操作流程

1. 启动 Dobot 驱动，确认 `/dobot_msgs_v4/msg/ToolVectorActual` 有数据。
2. 固定一个尖点，保证标定过程中它不移动。
3. 运行脚本：

```bash
cd ~/Robot_Arm_Project
source install/setup.bash
python3 scripts/tcp_calibration/tcp_pivot_calibrator.py
```

脚本会在进入拖拽和每次 `r` 记录前强制调用控制器的 `User(0)` 与
`Tool(0)`。任一调用失败时不会采样或进入拖拽；保存的 JSON 也会写明
`coordinate_frames.user_index=0` 和 `tool_index=0`。

4. 输入 `d` 进入拖拽模式（此时会先锁定 User(0)/Tool(0)）。
5. 拖动机械臂，让夹爪尖端轻触固定尖点。
6. 保持接触不动，输入 `r` 记录一组姿态。
7. 换一个姿态继续轻触同一点，再输入 `r`。
8. 至少记录 8 组，推荐 12 到 20 组。
9. 输入 `s` 解算。
10. 输入 `w` 保存样本和结果。
11. 输入 `e` 退出拖拽，输入 `q` 退出脚本。

## 姿态采集要求

好数据比数量更重要。

- TCP 尖端必须始终顶在同一个固定点。
- 每次记录前停稳 1 秒。
- 姿态要分散，不能只在一个方向附近小幅晃动。
- 推荐 roll/pitch/yaw 至少有 20 到 40 度变化。
- 不要用力压弯工具或固定针。
- 如果夹爪有弹性，接触力要轻且一致。

## 结果判断

脚本会输出残差：

```text
mean residual
max residual
```

经验判断：

| 残差 | 评价 |
| --- | --- |
| mean < 1 mm | 很好 |
| 1 到 2 mm | 可用 |
| 2 到 5 mm | 需要检查接触是否稳定 |
| > 5 mm | 数据大概率有错误，重新采集 |

如果残差很大：

- 固定尖点可能动了。
- 工具尖端没有一直接触同一点。
- 姿态变化太小，方程病态。
- ToolVectorActual 的姿态约定和实际控制器不一致。

已有 json 可以用诊断脚本重新分析：

```bash
python3 scripts/tcp_calibration/tcp_analyze_json.py scripts/tcp_calibration/tcp_calibration_xxx.json
```

如果想临时去掉某几条样本再看结果：

```bash
python3 scripts/tcp_calibration/tcp_analyze_json.py scripts/tcp_calibration/tcp_calibration_xxx.json --drop 3
python3 scripts/tcp_calibration/tcp_analyze_json.py scripts/tcp_calibration/tcp_calibration_xxx.json --drop 3 7
```

## 和机械臂法兰坐标的关系

脚本使用 `GetPose()` 的法兰位姿，输出法兰坐标系下的 TCP 偏移：

```text
tcp_offset_flange_mm = [x, y, z]
```

如果 Dobot 工具坐标设置需要：

```text
X, Y, Z, Rx, Ry, Rz
```

则先填：

```text
X = tcp_offset_flange_mm.x
Y = tcp_offset_flange_mm.y
Z = tcp_offset_flange_mm.z
Rx = 0
Ry = 0
Rz = 0
```

夹爪姿态轴是否需要额外旋转，单独做工具姿态定义，不要和 pivot 求出的 TCP 平移混在一起。

## 安全注意

- 拖拽前先降低速度和碰撞等级。
- 标定固定点附近不要放相机、线缆、手。
- 记录点时只轻触，不要用机械臂压尖点。
- 结果写入控制器前，先用小范围运动验证。
