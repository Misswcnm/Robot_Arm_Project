import os, rclpy, time
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from .executor import VisionExecutor
from .server import JsonLineServer
from apriltag_pick.pick_node import AprilTagPickNode

class ExecutorNode(Node):
 def __init__(self):
  super(ExecutorNode,self).__init__('vision_arm_executor')
  defaults={'rpc_host':'127.0.0.1','rpc_port':17881,'data_dir':'~/Robot_Arm_Project/data/vision_arm','handeye_path':'~/Robot_Arm_Project/scripts/active_handeye_calibration.json','tcp_calibration_path':'','robot_speed':15,'max_message_bytes':65536,'socket_timeout_sec':5.0,'task_ttl_sec':3600.,'apriltag_cache_ttl_sec':60.,'max_move_translation_mm':300.,'max_move_rotation_deg':45.,'workspace_min_xyz_mm':[-850.,-850.,20.],'workspace_max_xyz_mm':[850.,850.,1200.],'icp_frames':5,'icp_max_iters':30}
  self.cfg={k:self.declare_parameter(k,v).value for k,v in defaults.items()}; self.latest_pc=None; self.create_subscription(PointCloud2,'/camera/camera/depth/color/points',self._pc,10)
  self.pc_seq=0; self.tag_node=AprilTagPickNode()
  self.executor=VisionExecutor(self,self.cfg,tag_factory=lambda unused:self.tag_node); self.server=JsonLineServer(self.cfg['rpc_host'],self.cfg['rpc_port'],self.executor.submit,self.cfg['socket_timeout_sec'],self.cfg['max_message_bytes']); self.server.start(); self.get_logger().info('vision RPC listening on %s:%s; execution disabled'%(self.cfg['rpc_host'],self.cfg['rpc_port']))
 def _pc(self,msg):
  from icp_servoing.pointcloud import cloud_to_xyz
  self.latest_pc=cloud_to_xyz(msg); self.pc_seq+=1
 def wait_fresh(self, timeout=2.0):
  target=self.pc_seq+1; deadline=time.time()+timeout
  while time.time()<deadline:
   rclpy.spin_once(self,timeout_sec=.02)
   if self.pc_seq>=target:return True
  return False
 def destroy_node(self): self.server.close(); self.tag_node.destroy_node(); return super(ExecutorNode,self).destroy_node()
def main(args=None):
 rclpy.init(args=args); node=ExecutorNode(); executor=MultiThreadedExecutor(num_threads=4); executor.add_node(node); executor.add_node(node.tag_node)
 try:executor.spin()
 finally:executor.shutdown(); node.destroy_node(); rclpy.shutdown()
