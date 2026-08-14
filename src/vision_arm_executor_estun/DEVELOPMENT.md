# ESTUN 视觉机械臂开发进度

## 已完成

- C++ Bridge 直连 ESTUN 控制柜 `192.168.2.5:9000`，提供状态、位姿、关节、
  上下电、清错、MovJ/MovL、停止和 DO 服务。
- Executor 复用 CR5 的 TCP `8888` JSON 协议，前端和小车后端无需更改。
- 手眼标定：拖拽机械臂采集法兰位姿和棋盘图像，按
  `T_base_target = T_base_flange × T_flange_camera × T_camera_target` 解算。
- TCP 标定：保持工具尖端接触同一固定点，改变法兰姿态，通过枢轴法解算尖端相对
  法兰的偏移。

## 当前标定

```text
手眼: handeye_20260810_103630/handeye_result.json
TCP:  tcp_20260810_152943/tcp_result.json
```

- 手眼：10 帧/45 组运动对，平移 RMS `1.46 mm`、旋转 RMS `0.15°`，
  重投影均值 `0.13 px`。
- TCP：7 个姿态，残差均值 `2.08 mm`、最大 `3.43 mm`，
  法兰到尖端偏移 `[-19.05, -12.65, 210.52] mm`。
- 两个 `active.json` 均只保存当前结果文件路径；历史结果保留在 `runs/`。

以上指标说明标定内部一致性良好，正式作业前仍需进行一次实际定位/触碰验证。

## 下一步

1. 启动正式后端，验证机械臂状态、D455 点云、AprilTag 检测和 TCP `8888`。
2. ICP：完成 A 模板与 B 作业位示教，验证导航偏差下的回正及 A→B 执行误差。
3. AprilTag：按 `tag_id` 和相对偏移示教，验证重复定位与实际触碰误差。
4. 验证同一 `mapid/poseid` 下 ICP、AprilTag 多点按保存顺序全部执行。
5. 完成夹爪 DO 配置、急停/超时/越界保护后，再接入小车正式任务。

启动和标定命令见 [README.md](README.md)。
