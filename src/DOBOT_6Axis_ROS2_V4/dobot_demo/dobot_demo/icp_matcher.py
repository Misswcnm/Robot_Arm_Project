#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ICP Point Cloud Matcher — 点云模板匹配节点

功能：
  1. 订阅 RealSense 点云话题，缓存最新一帧为 pnow
  2. 首次收到自动保存为模板 ptemp，或通过 /save_template 服务手动保存
  3. 自动/手动触发 ICP 匹配，计算 pnow 到 ptemp 的 R, t
  4. 结果发布为 PoseStamped，供机械臂后续使用

用法：
  ros2 run dobot_demo icp_matcher
  ros2 service call /save_template std_srvs/srv/Trigger       # 保存当前帧为模板
  ros2 service call /toggle_continuous std_srvs/srv/SetBool "data: true"
"""

import time
import numpy as np
from scipy.spatial import KDTree

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, SetBool

from scipy.spatial.transform import Rotation as R


# ---------------------------------------------------------------------------
#  工具函数：点云转换 & 下采样
# ---------------------------------------------------------------------------

def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """
    将 sensor_msgs/PointCloud2 消息转为 numpy (N, 3) 数组。
    跳过 NaN / Inf 点，只保留有效的 x, y, z。
    """
    points = []
    for p in pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True):
        x, y, z = float(p[0]), float(p[1]), float(p[2])
        # 再过滤一次 Inf（skip_nans 不一定处理 Inf）
        if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
            points.append([x, y, z])
    if not points:
        return np.empty((0, 3), dtype=np.float64)
    return np.array(points, dtype=np.float64)


def voxel_downsample(points: np.ndarray, voxel_size: float = 0.005) -> np.ndarray:
    """
    体素下采样：每个 voxel_size 的立方体内只保留一个点。
    大幅减少点数，加速 ICP 最近邻搜索。
    """
    if len(points) == 0:
        return points
    if voxel_size <= 0:
        return points
    # 计算每个点所在的体素索引
    voxel_indices = np.floor(points / voxel_size).astype(np.int64)
    # 去重：每个体素只取第一个点
    _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)
    return points[unique_idx]


# ---------------------------------------------------------------------------
#  ICP 核心算法（Point-to-Point，Kabsch-Umeyama）
# ---------------------------------------------------------------------------

def icp_step(source: np.ndarray,
             target: np.ndarray,
             max_dist: float = 0.05) -> tuple:
    """
    单步 ICP：对 source 的每个点找到 target 中最近邻，
    然后 SVD 求解最优刚性变换 (R, t) 使 source -> target。

    参数
    ----
    source : (N, 3) 待对齐的点云
    target : (M, 3) 目标点云（模板）
    max_dist : 最近邻最大距离阈值（米），超出则丢弃该匹配对

    返回
    ----
    R      : (3, 3) 旋转矩阵；匹配点太少时返回 None
    t      : (3,)   平移向量
    error  : float  匹配后的 RMSE
    """
    tree = KDTree(target)
    dists, indices = tree.query(source, distance_upper_bound=max_dist)

    valid = dists < max_dist
    n_valid = int(np.sum(valid))
    if n_valid < 10:
        return None, None, float('inf')

    src_matched = source[valid]          # (K, 3)
    tgt_matched = target[indices[valid]] # (K, 3)

    # --- 质心 ---
    centroid_src = np.mean(src_matched, axis=0)
    centroid_tgt = np.mean(tgt_matched, axis=0)

    # --- 去质心 ---
    src_c = src_matched - centroid_src
    tgt_c = tgt_matched - centroid_tgt

    # --- 互协方差矩阵 + SVD ---
    H = src_c.T @ tgt_c                  # (3, 3)
    U, S, Vt = np.linalg.svd(H)

    R_mat = Vt.T @ U.T
    # 反射校正 (确保 det(R) = +1)
    if np.linalg.det(R_mat) < 0.0:
        Vt[2, :] *= -1.0
        R_mat = Vt.T @ U.T

    t_vec = centroid_tgt - R_mat @ centroid_src

    # --- RMSE ---
    src_transformed = (R_mat @ src_c.T).T + centroid_tgt
    err = np.sqrt(np.mean(np.sum((src_transformed - tgt_c) ** 2, axis=1)))

    return R_mat, t_vec, err


def icp_full(source: np.ndarray,
             target: np.ndarray,
             max_iter: int = 30,
             max_dist: float = 0.05,
             tolerance: float = 1e-6) -> dict:
    """
    完整迭代 ICP。

    返回
    ----
    {
        'R':      (3, 3)  累积旋转矩阵,
        't':      (3,)    累积平移向量,
        'error':  float   最终 RMSE,
        'iters':  int     实际迭代次数,
        'converged': bool 是否收敛,
        'history': list   每步 RMSE
    }
    """
    R_accum = np.eye(3, dtype=np.float64)
    t_accum = np.zeros(3, dtype=np.float64)
    src_cur = source.copy()
    history = []

    for i in range(max_iter):
        R_i, t_i, error = icp_step(src_cur, target, max_dist)

        if R_i is None:
            return {'R': R_accum, 't': t_accum, 'error': float('inf'),
                    'iters': i, 'converged': False, 'history': history}

        history.append(error)

        # 累积变换：先旋转，再平移
        # p_target = R_accum * p_source + t_accum
        # R_accum 更新为 R_i @ R_accum
        # t_accum 更新为 R_i @ t_accum + t_i
        t_accum = R_i @ t_accum + t_i
        R_accum = R_i @ R_accum

        # 更新 source 供下一轮迭代
        src_cur = (R_i @ src_cur.T).T + t_i

        # 收敛判断
        if i > 0 and abs(history[-1] - history[-2]) < tolerance:
            return {'R': R_accum, 't': t_accum, 'error': error,
                    'iters': i + 1, 'converged': True, 'history': history}

    return {'R': R_accum, 't': t_accum, 'error': history[-1] if history else float('inf'),
            'iters': max_iter, 'converged': False, 'history': history}


# ---------------------------------------------------------------------------
#  辅助：格式化输出
# ---------------------------------------------------------------------------

def fmt_matrix(R_mat: np.ndarray, precision: int = 6) -> str:
    """格式化 3x3 矩阵为字符串。"""
    lines = []
    for row in R_mat:
        lines.append('  [' + '  '.join(f'{v: .{precision}f}' for v in row) + ']')
    return '\n'.join(lines)


def format_result(R_mat: np.ndarray, t_vec: np.ndarray) -> str:
    """将 R, t 格式化为可读字符串：旋转矩阵 + 欧拉角 (deg) + 平移 (m)。"""
    # 欧拉角 (内旋 ZYX)
    r = R.from_matrix(R_mat)
    euler_rad = r.as_euler('zyx', degrees=False)
    euler_deg = np.degrees(euler_rad)

    lines = []
    lines.append('─' * 55)
    lines.append('  Rotation Matrix R (3×3):')
    lines.append(fmt_matrix(R_mat))
    lines.append(f'  Euler ZYX (deg):  roll={euler_deg[2]:.4f}  pitch={euler_deg[1]:.4f}  yaw={euler_deg[0]:.4f}')
    lines.append(f'  Translation t (m):  x={t_vec[0]:.6f}  y={t_vec[1]:.6f}  z={t_vec[2]:.6f}')
    lines.append(f'  Translation t (mm): x={t_vec[0]*1000:.3f}  y={t_vec[1]*1000:.3f}  z={t_vec[2]*1000:.3f}')
    lines.append('─' * 55)
    return '\n'.join(lines)


# ---------------------------------------------------------------------------
#  ROS2 节点
# ---------------------------------------------------------------------------

class IcpMatcher(Node):
    """ICP 点云匹配节点。"""

    def __init__(self):
        super().__init__('icp_matcher')

        # ---- 参数 ----
        self.declare_parameter('pointcloud_topic', '/camera/camera/depth/color/points')
        self.declare_parameter('voxel_size', 0.005)        # 下采样体素大小 (m)
        self.declare_parameter('max_icp_iter', 30)         # ICP 最大迭代次数
        self.declare_parameter('max_dist', 0.05)           # 最近邻距离阈值 (m)
        self.declare_parameter('icp_tolerance', 1e-6)      # 收敛阈值

        self._voxel_size = self.get_parameter('voxel_size').value
        self._max_iter = self.get_parameter('max_icp_iter').value
        self._max_dist = self.get_parameter('max_dist').value
        self._tolerance = self.get_parameter('icp_tolerance').value
        self._topic = self.get_parameter('pointcloud_topic').value

        # ---- 状态 ----
        self._ptemp = None        # 模板点云 (N, 3)
        self._pnow = None         # 当前最新点云 (N, 3)
        self._pnow_raw = None     # 当前点云（下采样前）
        self._continuous = True   # 是否连续匹配
        self._first_saved = False

        # ---- 订阅 ----
        self._sub = self.create_subscription(
            PointCloud2, self._topic, self._cloud_callback, 10)

        # ---- 发布 ----
        self._pose_pub = self.create_publisher(PoseStamped, '/icp_result', 10)

        # ---- 服务 ----
        self._srv_save = self.create_service(
            Trigger, '/save_template', self._save_callback)
        self._srv_toggle = self.create_service(
            SetBool, '/toggle_continuous', self._toggle_callback)

        self.get_logger().info(
            f'ICP Matcher started\n'
            f'  Subscribe  : {self._topic}\n'
            f'  Voxel size : {self._voxel_size:.3f} m\n'
            f'  Max ICP    : {self._max_iter} iters\n'
            f'  Max dist   : {self._max_dist:.3f} m\n'
            f'  Continuous : {self._continuous}')

    # ------------------------------------------------------------------
    #  点云回调
    # ------------------------------------------------------------------

    def _cloud_callback(self, msg: PointCloud2):
        """接收点云，下采样后缓存，并触发 ICP。"""
        # 原始点云
        raw = pointcloud2_to_xyz(msg)
        if len(raw) == 0:
            return

        self._pnow_raw = raw
        # 下采样
        self._pnow = voxel_downsample(raw, self._voxel_size)

        # 首次自动保存模板
        if self._ptemp is None and not self._first_saved:
            self._save_template_internal()
            self._first_saved = True
            return

        # 连续匹配
        if self._continuous and self._ptemp is not None:
            self._run_icp_and_publish()

    # ------------------------------------------------------------------
    #  ICP 执行 & 结果发布
    # ------------------------------------------------------------------

    def _run_icp_and_publish(self):
        """用当前 pnow 对 ptemp 做 ICP，打印并发布结果。"""
        if self._ptemp is None or self._pnow is None:
            return

        t0 = time.perf_counter()
        result = icp_full(self._pnow, self._ptemp,
                          max_iter=self._max_iter,
                          max_dist=self._max_dist,
                          tolerance=self._tolerance)
        elapsed_ms = (time.perf_counter() - t0) * 1000

        if result['R'] is None:
            self.get_logger().warn('ICP failed: insufficient inliers')
            return

        R_mat, t_vec = result['R'], result['t']

        # 终端输出
        n_pts = len(self._pnow)
        tpl_pts = len(self._ptemp)
        info = (
            f'ICP | pts={n_pts}→{tpl_pts} | '
            f'iters={result["iters"]} | error={result["error"]*1000:.3f} mm | '
            f'{elapsed_ms:.1f} ms'
            + (' ✓' if result['converged'] else ' ✗')
        )
        self.get_logger().info(info)
        self.get_logger().info('\n' + format_result(R_mat, t_vec))

        # 发布 PoseStamped
        self._publish_pose(R_mat, t_vec)

    def _publish_pose(self, R_mat: np.ndarray, t_vec: np.ndarray):
        """将 R, t 发布为 geometry_msgs/PoseStamped。"""
        r = R.from_matrix(R_mat)
        quat = r.as_quat()  # [x, y, z, w] (scipy 默认)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'camera_depth_optical_frame'
        pose_msg.pose.position.x = float(t_vec[0])
        pose_msg.pose.position.y = float(t_vec[1])
        pose_msg.pose.position.z = float(t_vec[2])
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])

        self._pose_pub.publish(pose_msg)

    # ------------------------------------------------------------------
    #  内部方法
    # ------------------------------------------------------------------

    def _save_template_internal(self):
        """将 pnow 保存为模板 ptem。"""
        self._ptemp = self._pnow.copy()
        n = len(self._ptemp)
        self.get_logger().info(f'✅ Template saved: {n} points (after {self._voxel_size:.3f}m voxel)')

    # ------------------------------------------------------------------
    #  服务回调
    # ------------------------------------------------------------------

    def _save_callback(self, request, response):
        """手动保存模板服务。"""
        if self._pnow is None:
            response.success = False
            response.message = 'No point cloud received yet'
        else:
            self._save_template_internal()
            response.success = True
            response.message = f'Template saved: {len(self._ptemp)} points'
        return response

    def _toggle_callback(self, request, response):
        """开关连续匹配。"""
        self._continuous = request.data
        state = 'ON' if self._continuous else 'OFF'
        self.get_logger().info(f'Continuous matching: {state}')
        response.success = True
        response.message = f'Continuous matching is now {state}'
        return response


# ---------------------------------------------------------------------------
#  入口
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = IcpMatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
