#!/usr/bin/env python3
"""ICP Visual Servoing — entry point.
RobotNode(服务+ToolVectorActual) + ServoNode(点云) 分离, 避免点云阻塞服务调用."""
import sys, os, json, time
import numpy as np
from scipy.spatial.transform import Rotation as Rot
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from icp_servoing.robot import CR5Robot
from icp_servoing.handeye import load_X
from icp_servoing.servoing import VisualServo
from icp_servoing.pointcloud import cloud_to_xyz

PERTURB = {
    '1': {'label': '小-', 'delta': [-5,  0,  0, -3,  0,  0]},
    '2': {'label': '中-', 'delta': [-10, -5,  0, -5, -3,  0]},
    '3': {'label': '大-', 'delta': [-15, -8,  0, -8, -5, -3]},
    '4': {'label': '小+', 'delta': [ 5,  0,  0,  3,  0,  0]},
    '5': {'label': '中+', 'delta': [ 10,  5,  0,  5,  3,  0]},
    '6': {'label': '大+', 'delta': [ 15,  8,  0,  8,  5,  3]},
}


def tool_to_matrix(tool):
    R = Rot.from_euler('xyz', tool[3:6], degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tool[:3]
    return T


def pose_error(T_cur, T_target):
    dp = T_target[:3, 3] - T_cur[:3, 3]
    dR = T_target[:3, :3] @ T_cur[:3, :3].T
    dr = Rot.from_matrix(dR).as_rotvec()
    return dp, dr


def apply_relative_move_once(robot, T_target, label):
    from icp_servoing.servoing import _compute_jacobian

    tool = robot.get_tool()
    if not tool:
        print(f'  ❌ {label}: ToolVectorActual失败')
        return False, None
    T_cur = tool_to_matrix(tool)
    j_now = robot.get_joints()
    if not j_now:
        print(f'  ❌ {label}: GetAngle失败')
        return False, T_cur

    dp, drot = pose_error(T_cur, T_target)
    # dR = R_target @ R_cur.T 已经是base系旋转误差, 不能再左乘R_cur。
    cart = np.hstack([dp, drot])
    J = _compute_jacobian(j_now)
    try:
        dtheta = np.degrees(np.linalg.pinv(J, rcond=1e-3) @ cart)
    except Exception:
        print(f'  ❌ {label}: Jacobian奇异')
        return False, T_cur

    target_j = [j_now[i] + dtheta[i] for i in range(6)]
    print(f'  {label}: Δp=[{dp[0]:.1f} {dp[1]:.1f} {dp[2]:.1f}]mm '
          f'|dp|={np.linalg.norm(dp):.1f}mm |dr|={np.degrees(np.linalg.norm(drot)):.2f}°')
    ok = robot.movj(target_j)
    if not ok:
        print(f'  ❌ {label}: JointMovJ失败')
        return False, T_cur
    return True, T_cur


class PointCloudNode(Node):
    """只订阅点云, 轻量回调: 保存最新numpy + seq"""
    def __init__(self):
        super().__init__('servo_pointcloud')
        self.latest_pc = None
        self._pc_seq = -1
        self._sub = self.create_subscription(
            PointCloud2, '/camera/camera/depth/color/points', self._cb, 10)
    def _cb(self, msg):
        self.latest_pc = cloud_to_xyz(msg)
        self._pc_seq += 1
    def wait_fresh(self, timeout: float = 2.0):
        """等新点云帧 (seq号大于当前)"""
        t0 = time.time()
        target = self._pc_seq + 1
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self._pc_seq >= target:
                return True
        return False


def main():
    rclpy.init()

    # RobotNode: 只服务+ToolVectorActual, 不订阅点云 → _call不被点云阻塞
    robot_node = Node('cr5_robot_control')
    # PCNode: 只订阅点云, 轻量回调
    pc_node = PointCloudNode()

    X = load_X()
    robot = CR5Robot(robot_node, speed=15)
    servo = VisualServo(robot, X, compensation_ratio=0.7)
    servo._pc_node = pc_node  # 让 servo 能读到点云

    print('\n' + '=' * 55)
    print('  ICP Visual Servoing')
    print('  r=模板  m=单步  a=闭环对齐  1-6=偏移  h=home  H=记录home')
    print('  t=拖拽  g=ICP回归→拖拽位姿  q=退出')
    print('=' * 55)
    drag_joints = None     # 拖拽点B的关节(备查)
    T_tool_A = None         # 模板点A的工具位姿(4x4)
    T_tool_B = None         # 拖拽点B的工具位姿(4x4)
    delta_A2B = None        # 相对变换: T_B = T_A @ delta_A2B

    robot.init()

    # Check camera
    for _ in range(10):
        rclpy.spin_once(pc_node, timeout_sec=0.1)
        if pc_node.latest_pc is not None and len(pc_node.latest_pc) > 500:
            print(f'✅ 相机OK ({len(pc_node.latest_pc)}点)')
            break
    else:
        print('⚠️  未收到点云!')

    try:
        while rclpy.ok():
            raw = input('\n🎯 ').strip()
            c = raw.lower()
            if c == 'q': break
            elif c == 'r':
                servo.record_template(n_frames=5)
                # 记录模板点A的位姿
                tool = robot.get_tool()
                if tool:
                    T_tool_A = tool_to_matrix(tool)
                    delta_A2B = None
                    print(f'  点A已记录: xyz=[{tool[0]:.0f} {tool[1]:.0f} {tool[2]:.0f}]')
            elif c == 'm':
                servo.step()
            elif c == 'a':
                servo.align(max_iters=30)
            elif raw == 'H':  # case-sensitive (与h区分)
                j = robot.get_joints()
                if j:
                    path = os.path.expanduser('~/Robot_Arm_Project/scripts/home_pose.json')
                    with open(path, 'w') as f: json.dump(j, f)
                    print(f'✅ home已记录: J=[{j[0]:.1f} {j[1]:.1f} {j[2]:.1f} {j[3]:.1f} {j[4]:.1f} {j[5]:.1f}]')
            elif c == 'h':
                path = os.path.expanduser('~/Robot_Arm_Project/scripts/home_pose.json')
                if os.path.exists(path):
                    with open(path) as f: home_j = json.load(f)
                    print(f'▶ home: J=[{home_j[0]:.1f} {home_j[1]:.1f} ...]')
                    robot.movj(home_j)
                else:
                    print('⚠ scripts/home_pose.json 不存在')
            elif c == 't':
                print('🖐 进入拖拽模式...')
                if not robot.start_drag():
                    print('  ❌ 拖拽模式启动失败')
                    continue
                print('  拖拽中… 1=记录位姿  2=退出拖拽')
                while True:
                    sub = input('  🖐 ').strip()
                    if sub == '1':
                        time.sleep(0.3)
                        j = robot.get_joints()
                        tool = robot.get_tool()
                        if tool and T_tool_A is not None:
                            drag_joints = j
                            T_tool_B = tool_to_matrix(tool)
                            delta_A2B = np.linalg.inv(T_tool_A) @ T_tool_B
                            d_mm = np.linalg.norm(delta_A2B[:3,3])
                            dr_deg = np.degrees(np.linalg.norm(Rot.from_matrix(delta_A2B[:3,:3]).as_rotvec()))
                            print(f'  ✅ 点B已记录: xyz=[{tool[0]:.0f} {tool[1]:.0f} {tool[2]:.0f}]')
                            print(f'     Δ(A→B): {d_mm:.0f}mm  {dr_deg:.1f}°')
                        elif j:
                            drag_joints = j
                            print(f'  ⚠ 无点A(ToolVectorActual), 仅存关节')
                        print('  继续拖拽… 1=记录位姿  2=退出拖拽')
                    elif sub == '2':
                        robot.stop_drag()
                        if robot.enable():
                            print('  已退出拖拽, 已使能')
                        else:
                            print('  ⚠ 已退出拖拽, 但使能失败')
                        break
                    else:
                        print('  1=记录位姿  2=退出拖拽')
            elif c == 'g':
                if delta_A2B is None:
                    print('⚠ 未记录点B (先r录模板, 拖拽后按2)')
                else:
                    print('▶ ICP回归模板...')
                    if not servo.align(max_iters=30):
                        print('❌ ICP未收敛，不执行相对戳点')
                        continue
                    print('▶ 应用相对变换 A→B...')
                    tool = robot.get_tool()
                    if tool:
                        T_cur = tool_to_matrix(tool)
                        T_target = T_cur @ delta_A2B
                        ok, _ = apply_relative_move_once(robot, T_target, '相对戳点1')
                        if ok:
                            # 一次Jacobian是局部线性近似；到达后重新读实际位姿再补算一次。
                            apply_relative_move_once(robot, T_target, '相对戳点2')
                            after = robot.get_tool()
                            if after:
                                T_after = tool_to_matrix(after)
                                dp_final, dr_final = pose_error(T_after, T_target)
                                print(f'✅ 相对戳点完成: 剩余 |dp|={np.linalg.norm(dp_final):.1f}mm '
                                      f'|dr|={np.degrees(np.linalg.norm(dr_final)):.2f}°')
            elif c in PERTURB:
                p = PERTURB[c]
                j = robot.get_joints()
                if j:
                    target = [j[i] + p['delta'][i] for i in range(6)]
                    print(f'  {p["label"]}: ΔJ={p["delta"]}')
                    if robot.movj(target):
                        new_j = robot.get_joints()
                        if new_j:
                            dd = [new_j[i]-j[i] for i in range(6)]
                            print(f'  实际ΔJ=[{dd[0]:+.2f} {dd[1]:+.2f} {dd[2]:+.2f} '
                                  f'{dd[3]:+.2f} {dd[4]:+.2f} {dd[5]:+.2f}]°')
                    else:
                        print('  ❌ 移动失败')
            else:
                print('r=模板 m=单步 a=对齐 1-6=偏移 h=home q=退出')
    except KeyboardInterrupt: pass

    robot_node.destroy_node()
    pc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
