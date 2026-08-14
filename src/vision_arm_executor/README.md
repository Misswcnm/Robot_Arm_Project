# Vision Arm Executor

## 启动命令

构建：

```bash
cd ~/Robot_Arm_Project
./scripts/local_robot_arm.sh build
```

一键启动完整机械臂视觉栈：

```bash
cd ~/Robot_Arm_Project
./scripts/local_robot_arm.sh all
```

单独启动本地视觉执行器：

```bash
cd ~/Robot_Arm_Project
./scripts/local_robot_arm.sh vision
```

单独启动 NX TCP 兼容网关：

```bash
cd ~/Robot_Arm_Project
./scripts/local_robot_arm.sh nx-gateway
```

## 项目结构

```text
src/vision_arm_executor/
├── README.md
├── 1.md
├── 2_cr5_backend.md
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── vision_arm_executor
├── config/
│   ├── executor.yaml
│   └── nx_routes.json
├── vision_arm_executor/
│   ├── __init__.py
│   ├── node.py
│   ├── server.py
│   ├── protocol.py
│   ├── nx_commands.py
│   ├── nx_gateway.py
│   ├── executor.py
│   ├── teaching.py
│   ├── station_store.py
│   ├── store.py
│   ├── backends.py
│   └── teach_cli.py
└── test/
    ├── test_backends.py
    ├── test_executor.py
    ├── test_nx_gateway.py
    ├── test_protocol.py
    ├── test_server.py
    └── test_station_store.py
```
