#!/usr/bin/env python
#coding=utf-8

import sys
import psutil
import os
import commands
import time
import thread
import threading
import glob
import tarfile
import collections
import tornado.httpclient
from tornado.httpclient import HTTPRequest
import uuid
import base64
from enum import Enum
from PIL import Image

import sys
import os
sys.path.append(os.environ.get('DROBOT_DKDLL_DIR') + '/python2.7')
import base64
from Crypto import Random
from Crypto.PublicKey import RSA
from Crypto.Hash import SHA
from Crypto.Signature import PKCS1_v1_5 as PKCS1_signature
from Crypto.Cipher import PKCS1_v1_5 as PKCS1_cipher

from functools import partial
from socket import error

import tornado.gen
from tornado.concurrent import run_on_executor
from tornado.ioloop import IOLoop
from tornado.web import RequestHandler,HTTPError
from tornado.web import Application
from tornado.httpserver import HTTPServer
from tornado.websocket import WebSocketHandler
from tornado.concurrent import run_on_executor
import cv2
# from torchvision import transforms
import jsonschema

from datetime import datetime

from concurrent.futures import ThreadPoolExecutor

from rosbridge_library.util import json, bson
from concurrent.futures import ThreadPoolExecutor
import rospy
import math
import actionlib
import common_service.srv
from common_service.msg import *
import std_msgs.msg._Int8
from std_msgs.msg import Bool,UInt32,UInt16
from dr_nav_services.srv import *
# from map_manager.srv import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Point32, Vector3, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan
from drobot_manager_console.msg import LaserScanGrid,RobotPoseGrid
from xiaolv_msgs.msg import Velocities, Pelco , Output ,Pelco_velocity,Pelco_switch,Chassis_state_switch,Brake
import xiaolv_msgs.srv
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from actionlib_msgs.msg import *
from common_service.srv import *
from cartographer_ros_msgs.srv import SaveRosmap,RealTimeMap
from common_service.msg import mapInfo
import drobot_manager_console.srv as console_service
import drobot_manager_console.msg as console_msg
from perception_msgs.msg import CameraEvent

from std_srvs.srv import Empty ,Trigger

# roslaunch用于启动launch文件
import roslaunch
import master_node.mqtt_client_CM as MqttCM
import master_node.global_var
import master_node.rosbag_recorder as BagRecorder
import master_node.master_node_subscriber as MasterNodeSubscriber
import master_node.master_node_launch as MasterNodeLaunch
import master_node.master_node_ping as MasterNodePing
from master_node.master_node_state import MasterNodeState as MasterNodeState
import master_node.master_node_errorcode as MasterNodeErrorCode
import master_node.coordinates as helper
import master_node.paramschema as MasterNdeSchema
import xml.dom.minidom
import platform
#from master_node.master_node_errorcode import ErrorCode as MasterNodeErrorCode

class MasterNode:

    service_lookup_table = {}
    run_time_dir = ''
    run_map_dir = ''
    stop_move_flag = True
    start_navigation_flag_ = False

    authorized_state = bool(1-master_node.global_var.use_authorized_state)
    def __del__(self):
        # self.wiper_thread = False
        self.MqttCM_client_.client_.disconnect()
        self.MqttCM_client_.client_.loop_stop()
        self.timer_running_  = False
        self.wiper_thread_func_state = False
        rospy.loginfo("~~~~~~~~~~~~~~~~~~~~~Master~~~~~~~~~~~~~~~~~~~~~~~~" )
        self.default_bag_recoder.stopRosbagRecord()
        # self.master_node_state.state_operation == MasterNodeState.state_type.Idle
        pass





    def __init__(self,uuid):
        print("Master_node.py xc *********************0001************************")
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        #add service function area

        self.master_node_subscriber = MasterNodeSubscriber.MasterNodeSubscriber(self)
        print("Master_node.py xc *********************0002************************")
        self.master_node_state = MasterNodeState(uuid)
        print("Master_node.py xc *********************0003************************")
        self.master_node_ping = MasterNodePing.MasterNodePing()
        print("Master_node.py xc *********************0004************************")
        self.bag_recoder = BagRecorder.BagRecorder()
        print("Master_node.py xc *********************0005************************")
        self.default_bag_recoder = BagRecorder.BagRecorder()
        print("Master_node.py xc *********************0006************************")
        self.timer_running_  = True
        default_record = rospy.get_param('~default_record',False)
        default_location_record = rospy.get_param('~default_location_record', False)
        print("Master_node.py xc *********************0007************************")
        #进行默认录包
        if default_record:
          self.default_bag_recoder.startRosbagRecord(BagRecorder.BagRecorder.RecordType.RunningAckermann,"__default")
        if default_location_record:
          self.default_bag_recoder.startRosbagRecord(BagRecorder.BagRecorder.RecordType.Localization,"__default_location")
        print("Master_node.py xc *********************0008************************")
        #进行http回调函数的注册
        # 建图功能
        self.service_lookup_table['mapping/scan_map_png'] = self.scan_map_png
        self.service_lookup_table['mapping/scan_map_grid'] = self.scan_map_grid
        self.service_lookup_table['mapping/start_scan_map'] = self.start_scan_map
        self.service_lookup_table['mapping/restart_scan_map'] = self.restart_scan_map
        self.service_lookup_table['mapping/stop_scan_map'] = self.stop_scan_map
        self.service_lookup_table['mapping/stop_calibration_laser'] = self.stop_calibration_laser
        self.service_lookup_table['mapping/cancel_scan_map'] = self.cancel_scan_map
        self.service_lookup_table['mapping/record_mapping_pose'] = self.record_mapping_pose

        #function for manage maps & initial pose
        self.service_lookup_table['map_manage/get_map_list'] = self.get_map_list
        self.service_lookup_table['map_manage/maps_pngs'] = self.maps_pngs
        self.service_lookup_table['map_manage/compress_map_pngs'] = self.compress_map_pngs
        self.service_lookup_table['map_manage/tile_maps_pngs'] = self.tile_maps_pngs
        self.service_lookup_table['map_manage/get_map'] = self.get_map
        self.service_lookup_table['map_manage/load_map'] = self.load_map
        self.service_lookup_table['map_manage/rename_map'] = self.rename_map
        self.service_lookup_table['map_manage/get_map_info'] = self.get_map_info
        self.service_lookup_table['map_manage/current_map_info'] = self.current_map_info
        self.service_lookup_table['map_manage/delete_map'] = self.delete_map
        self.service_lookup_table['map_manage/download_map'] =self.download_map
        self.service_lookup_table['map_manage/upload_map'] =self.upload_map
        self.service_lookup_table['map_manage/edit_map'] = self.edit_map
        #function for initialize the pose
        # 地图编辑，虚拟墙管理
        #function for obstacle operation
        self.service_lookup_table['navigate/add_virtual_obstacles'] = self.add_virtual_obstacles
        self.service_lookup_table['navigate/get_virtual_obstacles'] = self.get_virtual_obstacles

        #function for follow_target
        self.service_lookup_table['follow_target/start_follow_people'] = self.start_follow_people
        self.service_lookup_table['follow_target/start_follow_car'] = self.start_follow_car
        self.service_lookup_table['follow_target/stop_follow_target'] = self.stop_follow_target

        #function for navigation
        self.service_lookup_table['navigate/start_navigation'] = self.start_navigation
        self.service_lookup_table['navigate/cancel_navigation'] = self.cancel_navigation
        self.service_lookup_table['navigate/move_to'] = self.move_to
        self.service_lookup_table['navigate/pause_move_to'] = self.pause_move_to
        self.service_lookup_table['navigate/resume_move_to'] = self.resume_move_to
        self.service_lookup_table['navigate/cancel_move_to'] = self.cancel_move_to
        self.service_lookup_table['navigate/follow_path'] = self.follow_path
        self.service_lookup_table['navigate/clean_area'] = self.clean_area
        self.service_lookup_table['navigate/cancel_follow_path'] = self.cancel_follow_path
        self.service_lookup_table['navigate/get_navigator_status'] = self.get_navigator_status
        self.service_lookup_table['navigate/get_realtime_path'] = self.get_realtime_path
        self.service_lookup_table['navigate/get_time_task'] = self.get_time_task
        self.service_lookup_table['navigate/add_time_task'] = self.add_time_task
        self.service_lookup_table['navigate/delete_time_task'] = self.delete_time_task
        self.service_lookup_table['navigate/beat_button'] = self.beat_button
        self.service_lookup_table['navigate/set_auto_mode'] = self.set_auto_mode

        #function for path manage
        self.service_lookup_table['path_manage/start_record_path'] = self.start_record_path
        self.service_lookup_table['path_manage/save_record_path'] = self.save_record_path
        self.service_lookup_table['path_manage/cancel_record_path'] = self.cancel_record_path
        self.service_lookup_table['path_manage/get_record_status'] = self.get_record_status
        self.service_lookup_table['path_manage/get_path_list'] = self.get_path_list
        self.service_lookup_table['path_manage/delete_path'] = self.delete_path
        self.service_lookup_table['path_manage/get_path'] = self.get_path
        self.service_lookup_table['path_manage/get_manual_path'] = self.get_manual_path
        self.service_lookup_table['path_manage/generate_path'] = self.generate_path
        self.service_lookup_table['path_manage/update_path'] = self.update_path
        self.service_lookup_table['path_manage/verify_path_line'] = self.verify_path_line
        self.service_lookup_table['task_manager/follow_task_progress'] = self.follow_task_progress
        #function for pose manage
        # 点管理
        self.service_lookup_table['pose_manage/add_pose'] = self.add_pose
        self.service_lookup_table['pose_manage/add_cur_pose'] = self.add_cur_pose
        self.service_lookup_table['pose_manage/edit_pose'] = self.edit_pose
        self.service_lookup_table['pose_manage/get_pose'] = self.get_pose
        self.service_lookup_table['pose_manage/get_pose_from_map'] = self.get_pose_from_map
        self.service_lookup_table['pose_manage/delete_pose'] = self.delete_pose
        self.service_lookup_table['pose_manage/initialize_customized'] = self.initialize_customized
        self.service_lookup_table['pose_manage/bind_image'] = self.bind_image
        #function for task manage
        #创建任务并添加动作
        self.service_lookup_table['task_manager/save_task_queue']=self.save_task_queue
        self.service_lookup_table['task_manager/get_task_queue']=self.get_task_queue
        self.service_lookup_table['task_manager/update_task_queue']=self.update_task_queue
        self.service_lookup_table['task_manager/delete_task_queue']=self.delete_task_queue
        # 跑任务
        self.service_lookup_table['task_manager/start_task_queue']=self.start_task_queue
        self.service_lookup_table['task_manager/stop_task_queue']=self.stop_task_queue
        self.service_lookup_table['task_manager/pause_task_queue']=self.pause_task_queue
        self.service_lookup_table['task_manager/resume_task_queue']=self.resume_task_queue
        self.service_lookup_table['task_manager/is_task_queue_finished']=self.is_task_queue_finished
        self.service_lookup_table['task_manager/get_task_status']=self.get_task_status
        self.service_lookup_table['task_manager/get_autocharge_status']=self.get_autocharge_status
        self.service_lookup_table['task_manager/set_autocharge_status']=self.set_autocharge_status
        self.service_lookup_table['task_manager/get_task_report'] = self.get_task_report
        self.service_lookup_table['task_manager/get_report_image'] = self.get_report_image
        self.service_lookup_table['task_manager/generate_pdf_report'] = self.generate_pdf_report
        self.service_lookup_table['task_manager/download_report'] = self.download_report

        self.service_lookup_table['track_manage/add_track_graph']=self.add_track_graph
        self.service_lookup_table['track_manage/get_track_graph']=self.get_track_graph
        self.service_lookup_table['track_manage/restore_default_track']=self.restore_default_track

        self.service_lookup_table['image_manage/get_image']=self.get_image
        self.service_lookup_table['image_manage/get_image_lists']=self.get_image_lists
        self.service_lookup_table['image_manage/delete_image'] = self.delete_image
        self.service_lookup_table['image_manage/delete_images'] = self.delete_images
        self.service_lookup_table['image_manage/upload_image'] = self.upload_image
        self.service_lookup_table['image_manage/query_images'] = self.query_images
        self.service_lookup_table['image_manage/get_image_file'] = self.get_image_file

        self.service_lookup_table['parking_manager/add_parkings']=self.add_parkings
        self.service_lookup_table['parking_manager/delete_parkings']=self.delete_parkings
        self.service_lookup_table['parking_manager/update_parking'] = self.update_parking
        self.service_lookup_table['parking_manager/query_parkings'] = self.query_parkings
        # self.service_lookup_table['parking_manager/reload_parkings'] = self.reload_parkings

        self.service_lookup_table['area_manager/add_area']=self.add_area
        self.service_lookup_table['area_manager/delete_area']=self.delete_area
        # self.service_lookup_table['area_manager/update_area'] = self.update_area
        self.service_lookup_table['area_manager/query_areas'] = self.query_areas

        # TCP通信端点
        self.service_lookup_table['tcp/send_demo_point_direct'] = self.send_demo_point_tcp_direct
        self.service_lookup_table['tcp/send_demo_point'] = self.send_demo_point_tcp
        self.service_lookup_table['mechanical_arm/send_demo_point'] = self.send_demo_point_via_arm_controller
        self.service_lookup_table['mechanical_arm/get_demo_points'] = self.get_mechanical_arm_points_from_demo_files
        self.service_lookup_table['mechanical_arm/send_command'] = self.send_mechanical_arm_command
        self.service_lookup_table['mechanical_arm/send_manual'] = self.send_mechanical_arm_manual
        self.service_lookup_table['mechanical_arm/send_auto'] = self.send_mechanical_arm_auto
        self.service_lookup_table['mechanical_arm/send_clear_alarm'] = self.send_mechanical_arm_clear_alarm
        self.service_lookup_table['mechanical_arm/send_enable'] = self.send_mechanical_arm_enable
        self.service_lookup_table['mechanical_arm/send_disable'] = self.send_mechanical_arm_disable
        self.service_lookup_table['mechanical_arm/check_connection'] = self.check_mechanical_arm_connection

        self.service_lookup_table['param/update_param']=self.update_param
        self.service_lookup_table['param/get_param']=self.get_param
        self.service_lookup_table['param/update_system_param']=self.update_system_param
        self.service_lookup_table['param/get_system_param']=self.get_system_param
        self.service_lookup_table['param/reset_param']=self.reset_param
        self.service_lookup_table['param/upload_laser_param']=self.upload_laser_param
        self.service_lookup_table['param/get_devices_param'] = self.get_devices_param
        self.service_lookup_table['param/update_devices_param'] = self.update_devices_param

        self.service_lookup_table['lane_detection/start_lane_detection'] = self.start_lane_detection
        self.service_lookup_table['lane_detection/cancel_lane_detection'] = self.cancel_lane_detection
        self.service_lookup_table['lane_detection/save_lane_detection'] = self.save_lane_detection

        self.service_lookup_table['user_manage/verify_user'] = self.verify_user
        self.service_lookup_table['user_manage/get_users'] = self.get_users
        self.service_lookup_table['user_manage/update_passwd'] = self.update_passwd
        self.service_lookup_table['user_manage/add_user'] = self.add_user
        self.service_lookup_table['user_manage/delete_user'] = self.delete_user

        self.service_lookup_table['sound_manage/query_sound'] = self.query_sound
        self.service_lookup_table['sound_manage/play_sound'] = self.play_sound
        self.service_lookup_table['sound_manage/stop_sound'] = self.stop_sound
        self.service_lookup_table['sound_manage/upload_sound'] = self.upload_sound
        self.service_lookup_table['sound_manage/delete_sound'] = self.delete_sound

        #关于录像机和摄像机接口
        self.service_lookup_table['video_manager/get_serial_num'] = self.get_serial_num
        #function for control command
        # 移动控制
        # 录包管理&&log管理
        # TODO 开始录包,结束录包,包列表,删除包,清空列表,上传包
        self.service_lookup_table['cmd/get_robot_state'] = self.get_robot_state
        self.service_lookup_table['cmd/change_robot_state'] = self.change_robot_state
        self.service_lookup_table['cmd/start_record_bag'] = self.start_record_bag
        self.service_lookup_table['cmd/stop_record_bag'] = self.stop_record_bag
        self.service_lookup_table['cmd/get_bag_list'] = self.get_bag_list
        self.service_lookup_table['cmd/download_bag'] = self.download_bag
        self.service_lookup_table['cmd/delete_bag'] = self.delete_bag
        self.service_lookup_table['cmd/is_bag_recording'] = self.is_bag_recording
        self.service_lookup_table['cmd/move'] = self.move
        self.service_lookup_table['cmd/move_brake'] = self.move_brake
        self.service_lookup_table['cmd/get_move_brake'] = self.get_move_brake
        self.service_lookup_table['cmd/ptz_location_move'] = self.ptz_location_move
        self.service_lookup_table['cmd/ptz_direction_move'] = self.ptz_direction_move
        self.service_lookup_table['cmd/ptz_switch'] = self.ptz_switch
        self.service_lookup_table['cmd/light_control'] = self.light_control
        self.service_lookup_table['cmd/switch_control'] = self.switch_control
        self.service_lookup_table['cmd/rotate_move'] = self.rotate_move
        self.service_lookup_table['cmd/linear_move'] = self.linear_move
        self.service_lookup_table['cmd/show_led_text'] = self.show_led_text
        self.service_lookup_table['cmd/get_led_text'] = self.get_led_text
        self.service_lookup_table['cmd/check_move_finished'] = self.check_move_finished
        self.service_lookup_table['cmd/stop_move'] = self.stop_move
        self.service_lookup_table['cmd/get_product_version'] = self.get_product_version
        self.service_lookup_table['cmd/srs_opreation'] = self.srs_opreation
        self.service_lookup_table['cmd/get_system_info'] = self.get_system_info
        self.service_lookup_table['cmd/traffic_light_switch'] = self.traffic_light_switch
        self.service_lookup_table['cmd/calibrate_scan'] = self.calibrate_scan
        self.service_lookup_table['cmd/calibrate_lslidar32'] = self.calibrate_lslidar_32
        self.service_lookup_table['cmd/calibrate_lslidar16'] = self.calibrate_lslidar_16
        self.service_lookup_table['cmd/clean_devices_control'] = self.clean_devices_control #扫地机
        self.service_lookup_table['cmd/floor_washer_control'] = self.floor_washer_control #洗地机
        #顶升
        self.service_lookup_table['cmd/lift_ctrl'] = self.lift_ctrl
        self.service_lookup_table['sensor_data/light_state'] = self.get_light_state
        self.service_lookup_table['sensor_data/switch_state'] = self.get_switch_state
        self.service_lookup_table['sensor_data/raw_velocity'] = self.get_raw_velocity
        # 激光数据
        self.service_lookup_table['sensor_data/raw_scan'] = self.get_raw_scan
        self.service_lookup_table['sensor_data/scan_grid'] = self.get_sensor_data_scan
        self.service_lookup_table['sensor_data/get_scan_grid_sum'] = self.get_scan_grid_sum
        self.service_lookup_table['sensor_data/get_scan_grid_list'] = self.get_scan_grid_list

        self.service_lookup_table['sensor_data/raw_odom'] = self.get_raw_odom
        self.service_lookup_table['sensor_data/raw_ultrasonic'] = self.get_raw_ultrasonic
        self.service_lookup_table['sensor_data/raw_irsensor'] = self.get_raw_irsensor
        self.service_lookup_table['sensor_data/raw_imu'] = self.get_raw_imu
        self.service_lookup_table['sensor_data/bumper'] = self.get_bumper
        self.service_lookup_table['sensor_data/chassis_state'] = self.get_chassis_state
        self.service_lookup_table['sensor_data/ptz_state'] = self.get_ptz_state
        # 位置信息
        self.service_lookup_table['sensor_data/robot_pose_grid'] = self.get_robot_pose
        self.service_lookup_table['sensor_data/robot_footprint'] = self.get_robot_footprint
        
        # 里程信息
        self.service_lookup_table['sensor_data/get_mileage_info'] = self.get_mileage_info
        
        # 获取监控信息
        self.service_lookup_table['sensor_data/battery'] = self.get_battery
        self.service_lookup_table['sensor_data/meteorology'] = self.get_meteorology
        self.service_lookup_table['sensor_data/gps_data'] = self.get_gps_data
        self.service_lookup_table['sensor_data/grid_gps_data'] = self.get_grid_gps_data
        self.service_lookup_table['sensor_data/traffic_light_state'] = self.get_traffic_light_state
        self.service_lookup_table['sensor_data/dongle_state'] = self.get_dongle_state_cb
        # 获取所有信息
        self.service_lookup_table['sensor_data/robot_status'] = self.get_robot_status
        # 获取notification
        self.service_lookup_table['notice/get_localization_notice'] = self.get_localization_notice
        self.service_lookup_table['notice/get_nav_notice'] = self.get_nav_notice
        #function for debug
        # 调试数据
        self.service_lookup_table['getMemCache'] = self.getMemCache
        self.service_lookup_table['module_check'] = self.module_check
        #扫地机状态
        self.service_lookup_table['washing_machine_state'] = self.washing_machine_state

        #场景功能
        self.service_lookup_table['scene_setting/rain_mode'] = self.rain_mode_setting #下雨天开关小激光

        #获取系统信息
        self.service_lookup_table['system_info/get_current_time'] = self.getCurrentTime #获取工控机内部时间

        #设置系统信息
        self.service_lookup_table['system_setting/sync_current_time'] = self.syncCurrentTime #同步设置工控机时间

        #返回路径信息
        self.service_lookup_table['drobot_path_manager/get_planned_path'] = self.getPlannedPath

        #返回清洁区域路径信息
        self.service_lookup_table['drobot_path_manager/get_clean_area_path'] = self.getCleanPath

        #场景模式
        self.service_lookup_table['drobot_scene_manager/get_scene'] = self.getScene
        self.service_lookup_table['drobot_scene_manager/add_scene'] = self.addScene
        self.service_lookup_table['drobot_scene_manager/delete_scene'] = self.deleteScene
        # 环境变量
        env_dist = os.environ
        self.run_time_dir = env_dist.get('DROBOT_RUNTIME_DIR')
        self.run_map_dir = env_dist.get('DROBOT_MAP_DIR')
        self.home_dir = env_dist.get('DROBOT_SYSTEM_HOME')
        self.workspace_dir = env_dist.get('DROBOT_SYSTEM_HOME')+'/WORKSPACE'

        #ProductInfo载入参数服务器
        self.load_product_info()

        #异常事件
        self.service_lookup_table['exception_event_manage/get_events_info'] = self.get_events_info

        ######
        # 初始化状态机
        self.master_node_state.state_initialized = False
        self.master_node_state.state_operation = MasterNodeState.state_operation.Idle

        # 发布的topic
        self.obstacles_pub_ = rospy.Publisher("/virtual_obstacles_changed", UInt16, queue_size=10)
        #?
        self.graph_pub_ = rospy.Publisher("/reset_nav_topo_graph", std_msgs.msg.Empty, queue_size=10)
        #遥控
        self.cmd_pub = rospy.Publisher("/key_vel", Twist, queue_size = 10)
        self.pleco_cmd_pub = rospy.Publisher("/cmd_pelco", Pelco, queue_size = 10)
        self.pleco_vel_cmd_pub = rospy.Publisher("/cmd_pelco_velocity", Pelco_velocity, queue_size = 10)
        self.pleco_switch_pub = rospy.Publisher("/cmd_pelco_switch", Pelco_switch, queue_size = 10)
        self.chassis_switch_pub = rospy.Publisher("/cmd_chassis_switch", Chassis_state_switch, queue_size = 10)
        self.cmd_brake_pub = rospy.Publisher("/cmd_brake", Brake, queue_size = 10)
        self.mapping_record_pub = rospy.Publisher("/recordPose", common_service.msg.RecordPose, queue_size = 10)
        self.lift_ctrl_pub = rospy.Publisher("/cmd_lift_platform_switch", std_msgs.msg.Int8, queue_size= 10)
        self.helmet_pub = rospy.Publisher("/camera_helmet_event", CameraEvent, queue_size= 10)
        self.beat_pub = rospy.Publisher("/drobot_switch_task_start", std_msgs.msg.Empty, queue_size= 10)
        self.set_auto_mode_pub = rospy.Publisher("/set_drive_mode", std_msgs.msg.Int8, queue_size= 10)

        #洗地机控制
        self.clean_switch_pub = rospy.Publisher("/cmd_clean", std_msgs.msg.Int8MultiArray, queue_size=10)
        self.floor_washer_pub = rospy.Publisher("/cmd_clean_expand", std_msgs.msg.Int16, queue_size=10)
        if master_node.global_var.use_authorized_state:
          self.authorized_state_sub_ = rospy.Subscriber("/dr110_authorized_state",xiaolv_msgs.msg.Authorized_state,self.checkAuthorized,queue_size = 10 )
        # 启动设备
        self.master_node_state.defaultLoad()
        print("Master_node.py xc *********************0008************************")
        self.master_node_state.changeState(MasterNodeState.state_type.RunningTask)
        print("Master_node.py xc *********************0009************************")

        #定时器
        self.BovaIP = rospy.get_param('~BovaIP', '192.168.1.30')
        self.Follow_speed_level = rospy.get_param('/navigation/follow/speed_level', 0)
        self.Nav_speed_level = rospy.get_param('/navigation/nav/speed_level', 0)
        self.connect_to_CM = rospy.get_param('~connect_to_CM_remote', False)
        self.connect_to_CM_byMqtt = rospy.get_param('~connect_to_CM_byMqtt', False)
        self.connect_to_Bova = rospy.get_param('~connect_to_Bova', False)
        self.start_navigation_flag_ = False
        rospy.loginfo("\033[32m [Master_node.py ] connect to CM %d, MQTT %d, Bova %d \033[0m", self.connect_to_CM, self.connect_to_CM_byMqtt, self.connect_to_Bova)

        self.Bova_token = ""
        if self.connect_to_Bova:
            timer = threading.Timer(1, self.get_amera_helmet)
            timer.start()

        if self.connect_to_CM:
            self.stopPointsList = []
            self.send_CM_request()

        if self.connect_to_CM_byMqtt:
            self.MqttCM_client_ = MqttCM.BaseMqtt()
            self.send_CM_byMqtt()

        rospy.set_param('/wiper_thread_state', False)
        self.cmd_brake_state = False
        self.wiper_thread_func_state = True
        thread.start_new_thread(self.wiper_thread_func,())
        self.taskStartTimeStamp_ = int(time.time() * 1000)
        self.taskStartTime_ = datetime.utcnow().strftime('%Y%m%d%H%M%S%f')[:-3]
        self.latest_index_ = 0
        self.latest_progress = 0
        try:
            get_maps_data_srv = rospy.ServiceProxy("/map_manager_node/map_maps_data", console_service.getMapsData)
            try:
                get_maps_data_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                rospy.loginfo("[Master_node.py ] map maps data service timeout")
            resp  = get_maps_data_srv("")
        except rospy.ServiceException as e:
            rospy.loginfo("[Master_node.py ] map maps data service failed to call")

        if not resp.success:
            rospy.loginfo("[Master_node.py ] failed to get map data")
        else:
            self.maps_info = resp.mapsInfo
        #thread.start_new_thread(self.protector_lidar_process,())
        # self.master_node_state.state_initialized = True
        print("Master_node.py xc *********************000init_end************************")

    ###################################GUADIAN FUNCTION AREA####################################
    def createThreading(self, loopTime, FuncName):
        timer = threading.Timer(loopTime, FuncName)
        if(self.timer_running_ ):
            timer.start()
        else:
            timer.cancel()

    #ProductInfo载入参数服务器
    def load_product_info(self):
        product_info_url = self.run_time_dir + '/sys/ProductInfo'
        try:
            with open(product_info_url, 'r') as info_file:
                info_content_msg = json.load(info_file)
                if info_content_msg.has_key('DrobotModuleSN'):
                    rospy.set_param('/DrobotModuleSN', info_content_msg['DrobotModuleSN'])
                if info_content_msg.has_key('orbbec'):
                    if info_content_msg['orbbec'].has_key('ob_camera_01') and info_content_msg['orbbec']['ob_camera_01'].has_key('serial_number'):
                        rospy.set_param('/orbbec/ob_camera_01/serial_number', info_content_msg['orbbec']['ob_camera_01']['serial_number'])
                    if info_content_msg['orbbec'].has_key('ob_camera_02') and info_content_msg['orbbec']['ob_camera_02'].has_key('serial_number'):
                        rospy.set_param('/orbbec/ob_camera_02/serial_number', info_content_msg['orbbec']['ob_camera_02']['serial_number'])
                    if info_content_msg['orbbec'].has_key('ob_camera_03') and info_content_msg['orbbec']['ob_camera_03'].has_key('serial_number'):
                        rospy.set_param('/orbbec/ob_camera_03/serial_number', info_content_msg['orbbec']['ob_camera_03']['serial_number'])
                    if info_content_msg['orbbec'].has_key('ob_camera_04') and info_content_msg['orbbec']['ob_camera_04'].has_key('serial_number'):
                        rospy.set_param('/orbbec/ob_camera_04/serial_number', info_content_msg['orbbec']['ob_camera_04']['serial_number'])
                    if info_content_msg['orbbec'].has_key('ob_camera_05') and info_content_msg['orbbec']['ob_camera_05'].has_key('serial_number'):
                        rospy.set_param('/orbbec/ob_camera_05/serial_number', info_content_msg['orbbec']['ob_camera_05']['serial_number'])
                    if info_content_msg['orbbec'].has_key('ob_camera_06') and info_content_msg['orbbec']['ob_camera_06'].has_key('serial_number'):
                        rospy.set_param('/orbbec/ob_camera_06/serial_number', info_content_msg['orbbec']['ob_camera_06']['serial_number'])
        except IOError:
            rospy.loginfo("[Master_node.py ] no found %s", product_info_url)
            return
        except Exception:
            rospy.loginfo("[Master_node.py ] productinfo json format error")
            return
    #线程函数：保护激光线程
    def protector_lidar_process(self):
        #获取目前命名空间的参数
        # protect = rospy.get_param('~protect',True)
        while (not rospy.is_shutdown()):
            rospy.sleep(1)
            #检测导航程序是否异常死亡,检测到异常死亡时停止机器人移动
            # rospy.loginfo("[Master_node.py] checking rslidar !!!!!!!!!!!!!!!!!!!!!!")
            if True and self.master_node_ping.ping_lidar3d() != 'alive':
                rospy.logerr("[Master_node.py] rslidar is dead for some reason")
                #极光驱动崩溃停止运行
                twist = Twist()
                twist.linear.x = 0
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.z = 0
                self.cmd_pub.publish(twist)
                # self.stop_other_task()
                #self.master_node_state.restartLidarDevice()
            # else:
            #     rospy.loginfo("[Master_node.py] rslidar is alive")

    ###################################NOTIFICATION FUNCTION AREA####################################
    #停止工作
    def stop_other_task(self):
        # a= common_service.srv.Empty()
        rospy.logdebug("[Master_node.py] stop all task")
        stop_task_srv = rospy.ServiceProxy("/drobot_task_manager/stop_task",common_service.srv.Empty)
        stop_task_srv()
        rospy.sleep(0.1)

    def checkAuthorized(self,data):
      if data.authorized_state != self.authorized_state:
          self.authorized_state = data.authorized_state

    ###################################END NOTIFICATION FUNCTION AREA####################################
    #根据url中关键字,调用回调函数
    def parse_operation(self,operation):
        rospy.logdebug("[Master_node.py] receive operation %s",operation)
        # print("Master_node.py parse_operation xc *********************0002************************", operation)
        return self.service_lookup_table.has_key(operation)
        pass
    def authorized_state_cb(self,request_handler):
        request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.DEVICE_LOCKED))
        return
    def getMemCache(self,request_handler):
        request_handler.write("aaaa\n")
        request_handler.flush()
        time.sleep(5)
        request_handler.write("bbbb\n")
        request_handler.flush()
        time.sleep(5)
        request_handler.write("cccc\n")
        request_handler.flush()
        time.sleep(5)
        request_handler.write("dddd\n")
        request_handler.flush()
        time.sleep(5)
        request_handler.write("eeee\n")
        request_handler.flush()
        time.sleep(5)
        pass

    def module_check(self,request_handler):
        modules = {}
        # modules["GridPoseConverter"] = self.master_node_ping.ping_scan_to_cloud_converter_node()
        modules["Devices"] = self.master_node_ping.ping_drobot_device_node()
        modules["Localization"] = self.master_node_ping.ping_localization()
        modules["Navigator"] = self.master_node_ping.ping_bt_navigator_node()
        modules["Mapping"] = self.master_node_ping.ping_cartographer_node()
        modules["MappingGrid"] = self.master_node_ping.ping_cartographer_occupancy_grid_node()
        modules["Console"] = self.master_node_ping.ping_drobot_manager_console()
        request_handler.write(MasterNodeErrorCode.SuccessedData(modules))
        pass

    def get_amera_helmet(self):
        if rospy.is_shutdown():
            return
        current_time = self.default_bag_recoder.getCurrentTime()
        suid = ''.join(str(uuid.uuid4()).split('-'))
        camera_envent = CameraEvent()
        file_path = self.run_time_dir + "/task_log/camera_event/camera_helmet_event"
        if  not os.path.exists(file_path):#如果路径不存在,创建路径
            os.makedirs(file_path)
        event = {}
        event["HelmetImageURL"] = file_path
        event["ipAddress"] = self.BovaIP
        event["eventType"] = "helmet"
        event["eventState"] = "active"
        event["dateTime"] = current_time[0: 8]
        event["UID"] = suid
        if self.Bova_token == "":
            try:
                url = 'http://' + self.BovaIP + '/api/login'
                basic = {}
                basic['Authorization']= 'Basic ' + base64.b64encode('admin:Drobot0109')
                http_login = tornado.httpclient.HTTPClient()
                response = http_login.fetch(url,method='GET',headers = basic)
                self.Bova_token = json.loads(response.body)["token"]
                rospy.logerr("[Master_node.py] Bova save jpg threading %s",self.Bova_token)
            except Exception as e:
                # rospy.logerr("[Master_node.py] Bova helmet event login HTTP exception: %s",e)
                return
        try:
            url = "http://" + self.BovaIP + "/api/intelligent/user/result?access_token=" + self.Bova_token
            http_result = tornado.httpclient.HTTPClient()
            response = http_result.fetch(url,method='POST',body='{"id": "GSF_ID_SVP_RESULT","op": "G0C0S0"}')
        except Exception as e:
            rospy.logerr("[Master_node.py] Bova helmet event HTTP exception: %s",e)
            return
        if json.loads(response.body)["data"]["personcnt"] > 0:
            if response.code == 200:
                event["CaptureResult"] = json.loads(response.body)["data"]
                if "nohelmetcnt" not in event["CaptureResult"] and "helmetcnt" in event["CaptureResult"]:
                    event["CaptureResult"]["nohelmetcnt"] = event["CaptureResult"]["personcnt"] - event["CaptureResult"]["helmetcnt"]
                try:
                    url = "http://" + self.BovaIP + "/api/channel/snap1?access_token=" + self.Bova_token
                    http_snap1 = tornado.httpclient.HTTPClient()
                    response_snap1 = http_snap1.fetch(url)
                except Exception as e:
                    rospy.logerr("[Master_node.py] Bova helmet event HTTP photo request exception: %s",e)
                    return
                if response_snap1.code == 200:
                    file_name = file_path + "/latest_camera_helmet.jpg"
                    camera_envent.local_url = file_name
                    with open( file_name , "wb") as conf:
                        conf.write(response_snap1.body)
                else:
                    rospy.logerr("[Master_node.py] Bova helmet event HTTP photo request status_code: %d", response_snap1.code)
                    return
            else:
                rospy.logerr("[Master_node.py] Bova helmet event HTTP request status_code: %d", response.code)
                return
            camera_envent.event_content = json.dumps(event)
            self.helmet_pub.publish(camera_envent)
        timer = threading.Timer(2, self.get_amera_helmet)
        timer.start()
        pass

    def send_CM_request(self):
        self.createThreading(5, self.send_CM_request)
        if rospy.is_shutdown():
            return
        if(self.start_navigation_flag_ == False):
            self.latest_index_ = 0
            self.latest_progress = 0
            self.stopPointsList = []
            return
        is_finished = False
        finished_last_point = False
        http_login = tornado.httpclient.HTTPClient()
        get_key_url = 'http://' + '36.137.231.230:81/v1/gateway/public' + '/auth/getKey'
        response = http_login.fetch(get_key_url, method='GET')
        resp_body = json.loads(response.body)
        public_key = "-----BEGIN PUBLIC KEY-----\n" +resp_body['data']+"\n-----END PUBLIC KEY-----"
        rsa_key = RSA.importKey(public_key)
        cipher = PKCS1_cipher.new(rsa_key)
        pass_word = "W@OLCQWJTIQR"
        encrypt_text = base64.b64encode(cipher.encrypt(bytes(pass_word.encode("utf8"))))
        encrypt_text.decode('utf-8')
        # print(encrypt_text.decode('utf-8'))
        get_token_url = 'http://' + '36.137.231.230:81/v1/gateway/public' + '/auth/getToken'
        request_body_dic = {}
        request_body_dic["username"] = "patroltask"
        request_body_dic["password"] = encrypt_text.decode('utf-8')
        request_body = json.dumps(request_body_dic)
        # print(request_body)
        # token_response = http_login.fetch(get_token_url, method='POST', body = request_body)

        header_dict = {
            'Content-Type': 'application/json',
            'charset' : 'UTF-8'
        }
        request_kwargs = {
            'validate_cert': False,
            'headers': header_dict,
            'body': request_body
        }
        request = tornado.httpclient.HTTPRequest(url = get_token_url, method='POST', **request_kwargs)
        http_client = tornado.httpclient.HTTPClient()
        token_response = http_client.fetch(request)
        token_resp_json = json.loads(token_response.body)
        report_task_header_dict = {
            'Content-Type': 'application/json',
            'Authorization' : token_resp_json['data']['accessToken']
        }
        report_task_url = 'http://' + '36.137.231.230:81/v1/gateway/public' + '/cloudapi/patrol/task/report'

        #任务详情
        try:
            task_status_req = console_service.GetTaskStatusRequest()
            get_task_status_srv = rospy.ServiceProxy("/drobot_task_manager/get_task_status",console_service.GetTaskStatus)
            resp = get_task_status_srv(task_status_req)
        except rospy.ServiceException as servexc:
            return
        if not resp.success:
            return

        task_status = {}
        current_task = json.loads(resp.current_task)

        if current_task.get('param') is None:
            return

        task_status["currentTask"] = current_task

        tasks = json.loads(resp.task)
        task_status["task"] = tasks

        length = len(task_status["task"]["tasks"])
        subtask_index = task_status["currentTask"]["current_task"]

        #进度
        progress = int(float(subtask_index)/length*1e2)
        if self.master_node_subscriber.check_reviced('bt_navigator_progress'):
            remaining_mileage = float(self.master_node_subscriber.submsg_table['bt_navigator_progress']['remaining_mileage'])
            total_mileage = float(self.master_node_subscriber.submsg_table['bt_navigator_progress']['total_mileage'])
            finished_progress = 1 - (remaining_mileage/total_mileage)

        if subtask_index == 1:
            progress = int(progress * finished_progress)
        elif subtask_index == 0:
            progress = 0
        else:
            progress = int(float(1)/length*1e2*finished_progress) + int(float(subtask_index - 1)/length*1e2)

        if subtask_index == 1 and self.latest_index_ == 0:
            self.taskStartTimeStamp_ = int(time.time() * 1000)
            self.taskStartTime_ = datetime.utcnow().strftime('%Y%m%d%H%M%S%f')[:-3]
        if resp.status == 0:
            is_finished = True

        status = 1
        endTime = ""
        if is_finished:
            if self.latest_index_ != 0:
                finished_last_point = True
            if finished_last_point == False:
                self.stopPointsList = []
            self.start_navigation_flag_ = False
            status = 2 # 任务状态 （1-执行中；2-已完成；3-待出发；4-已取消）
            endTime = int(time.time() * 1000)
            progress = 100
        if progress < self.latest_progress:
            progress = self.latest_progress

        self.latest_progress = progress
        #导航状态判断
        taskType = 1
        if self.master_node_subscriber.submsg_table['bt_navigator_status']['nav_status_type'] == "navigation":
            taskType = 1
        elif self.master_node_subscriber.submsg_table['bt_navigator_status']['nav_status_type'] == "follow_path":
            taskType = 2

        # dt = datetime.utcnow().strftime('%Y%m%d%H%M%S%f')[:-3]
        report_task_request_body_dic = {}
        report_task_request_body_dic["vehicleId"] = 'SMRPatrol001'
        report_task_request_body_dic["taskId"] = 'ZZP01' + self.taskStartTime_
        report_task_request_body_dic["taskType"] = taskType
        report_task_request_body_dic["startTime"] = self.taskStartTimeStamp_
        report_task_request_body_dic["endTime"] = endTime
        report_task_request_body_dic["progress"] = progress
        report_task_request_body_dic["status"] = status
        # report_task_request_body_dic["lineId"] = task_status['currentTask']['param']['point_name']
        report_task_request_body_dic["lineId"] = 'XLSM001'
        report_task_request_body_dic["stopPoints"] = []
        stopPointsList = {}
        longitude = int(114.31*1e7)
        latitude = int(30.52*1e7)
        if self.master_node_subscriber.check_reviced('gps'):
            longitude = int(self.master_node_subscriber.submsg_table['gps']['longitude'] * 1e7)
            latitude = int(self.master_node_subscriber.submsg_table['gps']['latitude'] * 1e7)
        if finished_last_point == True:
            stopPointsList= {}
            stopPointsList["stopTime"] = int(time.time() * 1000)
            stopPointsList["pos"] = {}
            stopPointsList["pos"]["longitude"] = longitude
            stopPointsList["pos"]["latitude"] = latitude
            self.stopPointsList.append(stopPointsList)
        if subtask_index != 0:
            stopPointsList[subtask_index - 1] = {}
            if subtask_index != self.latest_index_ and self.latest_index_ != 0:
                stopPointsList[subtask_index - 1]["stopTime"] = int(time.time() * 1000)
                stopPointsList[subtask_index - 1]["pos"] = {}
                stopPointsList[subtask_index - 1]["pos"]["longitude"] = longitude
                stopPointsList[subtask_index - 1]["pos"]["latitude"] = latitude
                self.stopPointsList.append(stopPointsList[subtask_index - 1])
            else:
                stopPointsList[subtask_index - 1]["stopTime"] = ""
                stopPointsList[subtask_index - 1]["pos"] = {}
                stopPointsList[subtask_index - 1]["pos"]["longitude"] = longitude
                stopPointsList[subtask_index - 1]["pos"]["latitude"] = latitude
        report_task_request_body_dic["stopPoints"] = self.stopPointsList
        report_task_request_body = json.dumps(report_task_request_body_dic)
        report_task_request_kwargs = {
            'headers': report_task_header_dict,
            'body': report_task_request_body
        }
        print(report_task_request_body)
        # report_task_request = tornado.httpclient.HTTPRequest(url=report_task_url, method='POST', **report_task_request_kwargs)
        # report_task_http_client = tornado.httpclient.HTTPClient()
        # report_task_response = report_task_http_client.fetch(report_task_request)
        self.latest_index_ = subtask_index
        pass

    def send_CM_byMqtt(self):
        self.createThreading(5, self.send_CM_byMqtt)
        ###档位
        transmission = 0
        linear_vel = 0
        if self.master_node_subscriber.check_reviced('velocity'):
            linear_vel = self.master_node_subscriber.submsg_table['velocity']['device_data']['linear_vel']
        if -0.05 < linear_vel < 0.05:
            transmission = 0 #N档
        elif linear_vel > 0.05:
            transmission = 3 #D挡
        elif linear_vel < -0.05:
            transmission = 4 #R挡
        else:
            transmission = 1 #P挡

        ###驾驶模式
        driveMode = 0 # 1:自动驾驶 2:人工驾驶
        if(self.start_navigation_flag_):
            driveMode = 1
        else:
            driveMode = 2

        ###车辆的灯光状态
        current_light_status = {}
        current_light_status['hazardSignal'] = 0 #双闪灯
        current_light_status['fogLight'] = 0 #雾灯
        angular_vel = 0
        if self.master_node_subscriber.check_reviced('velocity'):
            angular_vel = self.master_node_subscriber.submsg_table['velocity']['device_data']['angular_rotate']
        current_light_status['parkingLights'] = 0 #刹车灯
        current_light_status['rightTurnSignal'] = 0 #右转灯
        current_light_status['leftTurnSignal'] = 0 #左转灯
        if -0.15 < angular_vel < 0.15:
            current_light_status['parkingLights'] = 1
        elif angular_vel < -0.15:
            current_light_status['rightTurnSignal'] = 1
        elif angular_vel > 0.15:
            current_light_status['leftTurnSignal'] = 1
        else:
            pass

        ##车辆的有关里程
        try:
            srv_msg = console_service.MileageInfoRequest()
            service = rospy.ServiceProxy("/get_mileage_info", console_service.MileageInfo)
            try:
                service.wait_for_service(1.0)
            except rospy.ROSException as servexc:
                # request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            res = service(srv_msg)
            mileage = res.mileage
            autoMileage = res.autoMileage
            totalMileage = res.totalMileage
            totalAutoMileage = res.totalAutoMileage
        except rospy.ServiceException as servexc:
            # request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return

        report_task_request_body_dic = {}
        report_task_request_body_dic["vehicleId"] = 'SMRPatrol001'
        report_task_request_body_dic['timestamp'] = int(time.time() * 1000)
        report_task_request_body_dic['pos'] = {}
        if self.master_node_subscriber.check_reviced('gps'):
            report_task_request_body_dic['pos']['longitude'] = self.master_node_subscriber.submsg_table['gps']['longitude'] #经度
            report_task_request_body_dic['pos']['latitude'] = self.master_node_subscriber.submsg_table['gps']['latitude']  #纬度
            report_task_request_body_dic['pos']['elevation'] = self.master_node_subscriber.submsg_table['gps']['altitude'] #高度
        else:
            report_task_request_body_dic['pos']['longitude'] = 0.0
            report_task_request_body_dic['pos']['latitude'] = 0.0
            report_task_request_body_dic['pos']['elevation'] = 0.0
        report_task_request_body_dic["transmission"] = transmission #0: N 档；1: P 档；3: D 挡；4: R 档；
        report_task_request_body_dic["speed"] = linear_vel*3.6 #单位: km/h
        report_task_request_body_dic["heading"] = 0        #[0..36000]正北方向顺时针旋转至与车辆当前车头指向方向重合所转过的角度，单位: 10e-2 degree

        if self.master_node_subscriber.check_reviced('battery'):
            report_task_request_body_dic["dumpEnergy"] = self.master_node_subscriber.submsg_table['battery']['battery_capacity_percentage']
        else:
            report_task_request_body_dic["dumpEnergy"] = 100.0 #[0..100]剩余（电量/油量）百分比
        report_task_request_body_dic["driveMode"] = driveMode      #1、自动驾驶 2、人工驾驶 3、远程操控
        report_task_request_body_dic["mileage"] = mileage        #行驶小计里程 单位: km
        report_task_request_body_dic["autoMileage"] = autoMileage    #自动驾驶小计里程 单位: km
        report_task_request_body_dic["totalMileage"] = totalMileage   #车辆行驶总里程 单位: km
        report_task_request_body_dic["totalAutoMileage"] = totalAutoMileage#自动驾驶总里程 单位: km
        report_task_request_body_dic["lights"] = {}
        report_task_request_body_dic["lights"]['hazardSignal'] = current_light_status['hazardSignal']
        report_task_request_body_dic["lights"]['fogLight']  = current_light_status['fogLight']
        report_task_request_body_dic["lights"]['parkingLights'] = current_light_status['parkingLights']
        report_task_request_body_dic["lights"]['rightTurnSignal'] = current_light_status['rightTurnSignal']
        report_task_request_body_dic["lights"]['leftTurnSignal'] = current_light_status['leftTurnSignal']
        report_task_request_body_dic["events"] = [] #V2X 事件信息
        report_task_request_body_dic["alerts"] = [] #预警事件
        # mqtt_cmd_data = {report_task_request_body_dic}
        json_data = json.dumps(report_task_request_body_dic, ensure_ascii = False)
        print(json_data)
        self.MqttCM_client_.client_.publish(topic="prod/v1/vpub/vehicle/realtime/SMRPatrol001", payload=json_data, qos=0)
        pass

    def get_mileage_info(self,request_handler):
        mileage_info = {}
        try:
            srv_msg = console_service.MileageInfoRequest()
            service = rospy.ServiceProxy("/get_mileage_info", console_service.MileageInfo)
            try:
                service.wait_for_service(3.0)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            res = service(srv_msg)
            mileage_info["daily_mileage"] = round(res.mileage,2)
            mileage_info["total_mileage"] = round(res.totalMileage,2)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        request_handler.write(MasterNodeErrorCode.SuccessedData(mileage_info))
        pass

    def get_light_state(self,request_handler):
        light_state_msg = {}
        light_state_msg['light_switch'] = {}
        light_state_msg['light_switch']['High_light'] = False
        light_state_msg['light_switch']['Low_light'] = False
        light_state_msg['light_switch']['Warning_light'] = False
        light_state_msg['light_switch']['UV_light'] = False
        light_state_msg['light_switch']['Lock_light'] = False
        try:
            service = rospy.ServiceProxy("/drobot_chassis_manager/light_get", console_service.GetLight)
            try:
                service.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            request =console_service.GetLightRequest()
            request.lightType.type = console_msg.LightType.HIGH_LIGHT
            resp  = service(request)
            light_state_msg['light_switch']['High_light'] = resp.value
            request.lightType.type = console_msg.LightType.LOW_LIGHT
            resp  = service(request)
            light_state_msg['light_switch']['Low_light'] = resp.value
            request.lightType.type = console_msg.LightType.WARNING_LIGHT
            resp  = service(request)
            light_state_msg['light_switch']['Warning_light'] = resp.value
            request.lightType.type = console_msg.LightType.UV_LIGHT
            resp  = service(request)
            light_state_msg['light_switch']['UV_light'] = resp.value
            request.lightType.type = console_msg.LightType.LOCK_LIGHT
            resp  = service(request)
            light_state_msg['light_switch']['Lock_light'] = resp.value
        except rospy.ServiceException as e:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        request_handler.write(MasterNodeErrorCode.SuccessedData(light_state_msg))
        pass
    def get_switch_state(self,request_handler):
        switch_state_msg = {}
        switch_state_msg['switchs'] = {}
        try:
            service = rospy.ServiceProxy("/drobot_chassis_manager/switch_get", console_service.GetSwitch)
            try:
                service.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp  = service()
            for index in range(len(resp.values)):
              key = 'switch_bit_' + str(index)
              switch_state_msg['switchs'][key] = resp.values[index]
        except rospy.ServiceException as e:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        request_handler.write(MasterNodeErrorCode.SuccessedData(switch_state_msg))
        pass
    def get_raw_velocity(self,request_handler):
        if not self.master_node_subscriber.check_reviced('velocity'):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NOT_RECIVED_MSG))
            return
        request_handler.write(MasterNodeErrorCode.SuccessedData(self.master_node_subscriber.submsg_table['velocity']))
        pass

    def get_raw_scan(self,request_handler):
        if not self.master_node_subscriber.check_reviced('raw_scan'):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NOT_RECIVED_MSG))
            return
        request_handler.write(MasterNodeErrorCode.SuccessedData(self.master_node_subscriber.submsg_table['raw_scan']))
        pass

    def get_sensor_data_scan(self,request_handler):
        if not self.master_node_subscriber.check_reviced('scan_grid'):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NOT_RECIVED_MSG))
            return
        request_handler.write(MasterNodeErrorCode.SuccessedData(self.master_node_subscriber.submsg_table['scan_grid']))
        pass

    def get_scan_grid_sum(self,request_handler):
        request_handler.write(MasterNodeErrorCode.SuccessedData(rospy.get_param('/drobot_manager_console_node/ErdScan_grid_sum',0)))
        pass

    def get_scan_grid_list(self,request_handler):
        request_handler.write(MasterNodeErrorCode.SuccessedData(self.master_node_subscriber.sensor_scan_grid_list))
        pass

    def get_raw_odom(self,request_handler):
        if not self.master_node_subscriber.check_reviced('raw_odom'):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NOT_RECIVED_MSG))
            return
        request_handler.write(MasterNodeErrorCode.SuccessedData(self.master_node_subscriber.submsg_table['raw_odom']))
        pass

    def get_raw_ultrasonic(self,request_handler):
        ultrasonics=[]
        for index in range(0,self.master_node_subscriber.ultrasound_size):
          ultrasound_table_name = "ultrasound"+str(index)
          if self.master_node_subscriber.check_reviced(ultrasound_table_name):
            ultrasonics.append(self.master_node_subscriber.submsg_table[ultrasound_table_name])
          else:
            ultrasonic_msg={}
            ultrasonic_msg['frameid']=""
            ultrasonic_msg['originX']=0
            ultrasonic_msg['originY']=0
            ultrasonic_msg['angle']=0
            ultrasonic_msg['range']=0
            ultrasonics.append(ultrasonic_msg)
        request_handler.write(MasterNodeErrorCode.SuccessedData(ultrasonics))
        pass

    def get_raw_irsensor(self,request_handler):
        irsensors=[]
        for index in range(0,self.master_node_subscriber.irsensor_size):
          irsensors_table_name = "fallprevention"+str(index)
          if self.master_node_subscriber.check_reviced(irsensors_table_name):
            irsensors.append(self.master_node_subscriber.submsg_table[irsensors_table_name])
          else:
            irsensor_msg={}
            irsensor_msg['frameid']=""
            irsensor_msg['originX']=0
            irsensor_msg['originY']=0
            irsensor_msg['angle']=0
            irsensor_msg['range']=0
            irsensors.append(irsensor_msg)
        request_handler.write(MasterNodeErrorCode.SuccessedData(irsensors))
        pass
    def get_raw_imu(self,request_handler):
        if not self.master_node_subscriber.check_reviced('raw_imu'):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NOT_RECIVED_MSG))
            return
        request_handler.write(MasterNodeErrorCode.SuccessedData(self.master_node_subscriber.submsg_table['raw_imu']))
        pass

    def get_bumper(self,request_handler):
        if not self.master_node_subscriber.check_reviced('bumper'):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NOT_RECIVED_MSG))
            return
        request_handler.write(MasterNodeErrorCode.SuccessedData(self.master_node_subscriber.submsg_table['bumper']))
        pass

    def get_chassis_state(self,request_handler):
        if not self.master_node_subscriber.check_reviced('chassis_state'):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NOT_RECIVED_MSG))
            return
        request_handler.write(MasterNodeErrorCode.SuccessedData(self.master_node_subscriber.submsg_table['chassis_state']))
        pass

    def get_ptz_state(self,request_handler):
        if not self.master_node_subscriber.check_reviced('ptz_state'):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NOT_RECIVED_MSG))
            return
        self.master_node_subscriber.submsg_table['ptz_state']['wiper'] = rospy.get_param('/wiper_thread_state',False)
        request_handler.write(MasterNodeErrorCode.SuccessedData(self.master_node_subscriber.submsg_table['ptz_state']))
        pass

    def get_robot_pose(self,request_handler):
        if not self.master_node_subscriber.check_reviced('robot_pose_grid'):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NOT_RECIVED_MSG))
            return
        request_handler.write(MasterNodeErrorCode.SuccessedData(self.master_node_subscriber.submsg_table['robot_pose_grid']))
        pass

    def get_robot_footprint(self,request_handler):
        # if not self.master_node_subscriber.check_reviced('footprint'):
        #     request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NOT_RECIVED_MSG))
        #     return
        footprint_msg={}
        footprint_msg['baselink_points'] = []
        baselink_footprints = rospy.get_param('/bt_navigator_node/device/footprints', {})
        footprint_msg['baselink_points'] = baselink_footprints[0]["points"]
        request_handler.write(MasterNodeErrorCode.SuccessedData(footprint_msg))
        pass

    def get_battery(self,request_handler):
        if not self.master_node_subscriber.check_reviced('battery'):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NOT_RECIVED_MSG))
            return
        request_handler.write(MasterNodeErrorCode.SuccessedData(self.master_node_subscriber.submsg_table['battery']))
        pass

    def get_meteorology(self,request_handler):
        if not self.master_node_subscriber.check_reviced('meteorological'):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NOT_RECIVED_MSG))
            return
        request_handler.write(MasterNodeErrorCode.SuccessedData(self.master_node_subscriber.submsg_table['meteorological']))
        pass
    def get_gps_data(self,request_handler):
        if not self.master_node_subscriber.check_reviced('gps'):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NOT_RECIVED_MSG))
            return
        request_handler.write(MasterNodeErrorCode.SuccessedData(self.master_node_subscriber.submsg_table['gps']))
        pass
    def get_grid_gps_data(self,request_handler):
        if not self.master_node_subscriber.check_reviced('gps_grid'):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NOT_RECIVED_MSG))
            return
        request_handler.write(MasterNodeErrorCode.SuccessedData(self.master_node_subscriber.submsg_table['gps_grid']))
        pass
    def get_traffic_light_state(self,request_handler):
        if not self.master_node_subscriber.check_reviced('traffic_light_state'):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NOT_RECIVED_MSG))
            return
        request_handler.write(MasterNodeErrorCode.SuccessedData(self.master_node_subscriber.submsg_table['traffic_light_state']))
        pass
    def get_dongle_state_cb(self,request_handler):
        if not self.master_node_subscriber.check_reviced('dongle_state'):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NOT_RECIVED_MSG))
            return
        request_handler.write(MasterNodeErrorCode.SuccessedData(self.master_node_subscriber.submsg_table['dongle_state']))
        pass
    def get_robot_status(self,request_handler):
        if not self.master_node_subscriber.check_reviced('battery'):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NOT_RECIVED_MSG))
            return
        result_msg={}
        result_msg["battery_status"]={}
        result_msg["battery_status"]['battery_voltage'] = self.master_node_subscriber.submsg_table['battery']['battery_voltage']
        result_msg["battery_status"]['battery_current'] = self.master_node_subscriber.submsg_table['battery']['battery_current']
        result_msg["battery_status"]['battery_capacity_percentage'] = self.master_node_subscriber.submsg_table['battery']['battery_capacity_percentage']
        result_msg["battery_status"]['battery_temperature'] = self.master_node_subscriber.submsg_table['battery']['battery_temperature']
        result_msg["battery_status"]['battery_power'] = self.master_node_subscriber.submsg_table['battery']['battery_power']
        result_msg["battery_status"]['battery_24voltage'] = self.master_node_subscriber.submsg_table['battery']['battery_24voltage']
        result_msg["battery_status"]['battery_24current'] = self.master_node_subscriber.submsg_table['battery']['battery_24current']
        result_msg["battery_status"]['battery_12voltage'] = self.master_node_subscriber.submsg_table['battery']['battery_12voltage']
        result_msg["battery_status"]['battery_12current'] = self.master_node_subscriber.submsg_table['battery']['battery_12current']
        result_msg["driving_motor_status"]={}
        result_msg["driving_motor_status"]['motor_state'] = -1
        result_msg["driving_motor_status"]['motor_temperature']=-1
        result_msg["driving_motor_status"]['motor_current']=-1
        result_msg["driving_motor_status"]['motor_speed']=-1
        result_msg["driving_motor_status"]['motor_brakes']=-1
        result_msg["turning_motor_status"]={}
        result_msg["turning_motor_status"]['motor_state'] = -1
        result_msg["turning_motor_status"]['motor_temperature']=-1
        result_msg["turning_motor_status"]['motor_current']=-1
        result_msg["turning_motor_status"]['motor_speed']=-1
        result_msg["turning_motor_status"]['motor_brakes']=-1
        result_msg["TPZ_status"]=0
        result_msg["camera_status"]=0
        result_msg["TI_status"]=0
        result_msg["vidio_status"]=0
        result_msg["navigator_status"]=0
        result_msg["laser_status"]=0
        result_msg["protector_status"]=0
        result_msg["drop_status"]=0
        result_msg["ultrasonic_status"]=0
        request_handler.write(MasterNodeErrorCode.SuccessedData(result_msg))
        pass

    def get_serial_num(self, request_handler):
        result_msg = {}
        try:
            add_param_srv = rospy.ServiceProxy('/drobot_video_node/get_SerialNum',common_service.srv.Empty)
            try:
                add_param_srv.wait_for_service(1)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            res = add_param_srv()
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        result_msg['serial_num'] = res.message
        request_handler.write(MasterNodeErrorCode.SuccessedData(result_msg))
        pass


    def get_localization_notice(self, request_handler):
        if self.master_node_state.checkState([MasterNodeState.state_type.RunningTask, MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.SuccessedData(self.master_node_subscriber.submsg_table['localization_status'].update(self.master_node_subscriber.submsg_table['localization_score'])))
        else:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
        pass

    def get_nav_notice(self, request_handler):
        if self.master_node_state.checkState([MasterNodeState.state_type.RunningTask]):
            request_handler.write(MasterNodeErrorCode.SuccessedData(self.master_node_subscriber.submsg_table['last_navigation_notice']))
        else:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
        pass

    def washing_machine_state(self,request_handler):
        if not self.master_node_subscriber.check_reviced('clean_state') :
            request_handler.write(MasterNodeErrorCode.SuccessedData(self.master_node_subscriber.submsg_table['clean_state']))
            return
        request_handler.write(MasterNodeErrorCode.SuccessedData(self.master_node_subscriber.submsg_table['clean_state']))
        pass

    def rain_mode_setting(self, request_handler):
        stop_laser = request_handler.get_query_argument('stop_laser','')
        if not stop_laser:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        if stop_laser == 'true':
            try:
                add_param_req = String_reqRequest()
                add_param_req.str = '[{"isArray": false, "namespace": "/scene_setting/rain_mode", "type": "bool", "value": "true"}]'
                add_param_srv = rospy.ServiceProxy('/param_manager_node/update_param',String_req)
                try:
                    add_param_srv.wait_for_service(3)
                except rospy.ROSException as servexc:
                    request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                    return
                add_param_srv(add_param_req)
            except rospy.ServiceException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
                return
            rospy.set_param('/scene_setting/rain_mode', True)
            request_handler.write(MasterNodeErrorCode.SuccessedData(str('stop_laser : ') + stop_laser))
        else:
            try:
                add_param_req = String_reqRequest()
                add_param_req.str = '[{"isArray": false, "namespace": "/scene_setting/rain_mode", "type": "bool", "value": "false"}]'
                add_param_srv = rospy.ServiceProxy('/param_manager_node/update_param',String_req)
                try:
                    add_param_srv.wait_for_service(3)
                except rospy.ROSException as servexc:
                    request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                    return
                add_param_srv(add_param_req)
            except rospy.ServiceException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
                return
            rospy.set_param('/scene_setting/rain_mode', False)
            request_handler.write(MasterNodeErrorCode.SuccessedData(str('stop_laser : ') + stop_laser))
        return
    #Get Realtime map
    #获取实时建图png图片 即将废弃
    def scan_map_png(self,request_handler):
        if (self.master_node_state.state_operation != MasterNodeState.state_type.Mapping) and \
           (self.master_node_state.state_operation != MasterNodeState.state_type.RestartMapping):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        if not self.master_node_subscriber.check_reviced('sparse_map'):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NOT_RECIVED_MSG))
            return
        request_handler.write(self.master_node_subscriber.submsg_table['sparse_map'])
        pass

    #Get Realtime map
    def scan_map_grid(self,request_handler):
        if (self.master_node_state.state_operation != MasterNodeState.state_type.Mapping) and \
           (self.master_node_state.state_operation != MasterNodeState.state_type.RestartMapping):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        if not self.master_node_subscriber.check_reviced('sparse_map'):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NOT_RECIVED_MSG))
            return
        retdata = {}
        retdata['mapInfo'] = {}
        retdata['mapInfo']['width'] = self.master_node_subscriber.submsg_table['sparse_map']['mapInfo']['gridWidth']
        retdata['mapInfo']['height'] = self.master_node_subscriber.submsg_table['sparse_map']['mapInfo']['gridHeight']
        retdata['mapData'] = self.master_node_subscriber.submsg_table['sparse_map']['data']
        retdata['trajectory'] = self.master_node_subscriber.submsg_table['sparse_map']['trajectory']
        request_handler.write(MasterNodeErrorCode.SuccessedData(retdata))
        pass
    #开始建图
    def start_scan_map(self,request_handler):
        print("Master_node.py start_scan_map() xc *********************0004************************")
        # if self.master_node_state.state_operation != MasterNodeState.state_type.Idle:
        #     request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
        #     return
        self.master_node_state.state_initialized = False
        if self.master_node_state.state_initialized :
            rospy.loginfo("[Master_node.py] state_initialized: True")
        else :
            rospy.loginfo("[Master_node.py] state_initialized: False")
        #切换至建图状态
        self.master_node_subscriber.submsg_recived_table['sparse_map'] = False
        self.master_node_state.changeState(MasterNodeState.state_type.Mapping)
        request_handler.write(MasterNodeErrorCode.Successed())
        self.default_bag_recoder.stopRosbagRecord()
        self.default_bag_recoder.startRosbagRecord(BagRecorder.BagRecorder.RecordType.Mapping3D,"__default_mapping")
        rospy.loginfo("[Master_node.py] state_operation: %s", self.master_node_state.state_operation.value)
        return
        pass

    def restart_scan_map(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        map_info={}
        map_info['mapdirectory'] = self.master_node_subscriber.submsg_table['map_info'].directory
        map_info['mapId'] = self.master_node_subscriber.submsg_table['map_info'].mapId
        restart_map_pbstream_file = map_info['mapdirectory']+map_info['mapId']+".pbstream"
        self.master_node_subscriber.submsg_recived_table['sparse_map'] = False
        if not os.path.exists(restart_map_pbstream_file) :
          return request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.MAP_VERSION_ERROR))
        restart_map_args=[]
        restart_map_args.append("load_state_filename:="+restart_map_pbstream_file)
        if not self.master_node_state.changeState(MasterNodeState.state_type.RestartMapping,restart_map_args):
          return request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
        request_handler.write(MasterNodeErrorCode.Successed())
        return
    #Cancel scan map
    def cancel_scan_map(self,request_handler):
        if (self.master_node_state.state_operation != MasterNodeState.state_type.Mapping) and \
          (self.master_node_state.state_operation != MasterNodeState.state_type.RestartMapping):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        self.master_node_state.changeState(MasterNodeState.state_type.Idle)
        request_handler.write(MasterNodeErrorCode.Successed())
        self.default_bag_recoder.stopRosbagRecord()
        self.default_bag_recoder.startRosbagRecord(BagRecorder.BagRecorder.RecordType.RunningAckermann,"__default")
        rospy.loginfo("[Master_node.py] state_operation: %s",self.master_node_state.state_operation.value)
        pass
    def record_mapping_pose(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Mapping]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        pose_name = request_handler.get_query_argument('pose_name','')
        if not pose_name:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        msgs = common_service.msg.RecordPose()
        msgs.name = pose_name
        msgs.type = 2
        self.mapping_record_pub.publish(msgs)
        request_handler.write(MasterNodeErrorCode.Successed())
        return
        pass
    #Save Map
    def stop_scan_map(self,request_handler):
        if (self.master_node_state.state_operation != MasterNodeState.state_type.Mapping) and \
            (self.master_node_state.state_operation != MasterNodeState.state_type.RestartMapping):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        special_Characters=['~','#','%','&','*','{','}','\\',':','<','>','?','/','+','|','"']
        map_name = request_handler.get_query_argument('map_name','default_map')
        for character in special_Characters:
            ret=map_name.find(character)
            if ret != -1:
              request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_ILLEGAL_ERROR))
              return
        rotation_angle =0.0
        if self.master_node_state.state_operation == MasterNodeState.state_type.Mapping:
        ##重名判断
          try:
              get_maps_srv = rospy.ServiceProxy("/map_manager_node/map_getMaplist", StrArray_resp)
              resp_map  = get_maps_srv()
          except rospy.ServiceException as e:
              request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
              return
          for get_map_name in resp_map.strlist :
            if get_map_name == map_name:
              request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.MAP_IS_EXISTED))
              return
          rotation_angle = float(request_handler.get_query_argument('rotation_angle','0.0'))
          # rotation_angle = float(rotation_angle)
        rospy.loginfo("[Master_node.py] Tring to save map as :name: %s,rotation_angle:%f",map_name,rotation_angle)
        try:
            get_costmap_srv = rospy.ServiceProxy("Savemap", SaveRosmap)
            resp_costmap  = get_costmap_srv(rotation_angle,map_name)
        except rospy.ServiceException as e:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SAVEMAPPING_FAILED))
            return
        #TODO shutdown slam node
        if resp_costmap.status.code != 0:
            request_handler.write(MasterNodeErrorCode.FailedData(MasterNodeErrorCode.SERVICE_RETURN_FALSE,resp_costmap.status.code))
        self.master_node_state.changeState(MasterNodeState.state_type.Idle)#切换至Idle状态
        request_handler.write(MasterNodeErrorCode.Successed())
        self.default_bag_recoder.stopRosbagRecord()
        self.default_bag_recoder.startRosbagRecord(BagRecorder.BagRecorder.RecordType.RunningAckermann,"__default")
        rospy.loginfo("[Master_node.py] state_operation: %s",self.master_node_state.state_operation.value)

    # save calibration laser
    def stop_calibration_laser(self,request_handler):
        if (self.master_node_state.state_operation != MasterNodeState.state_type.Mapping) and \
            (self.master_node_state.state_operation != MasterNodeState.state_type.RestartMapping):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        try:
            get_costmap_srv = rospy.ServiceProxy("SaveCalibrationLaserTF", Trigger)
            resp_costmap  = get_costmap_srv()
        except rospy.ServiceException as e:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_RETURN_FALSE))
            return
        #TODO shutdown slam node
        if not resp_costmap.success:
            request_handler.write(MasterNodeErrorCode.FailedMsg(MasterNodeErrorCode.SERVICE_RETURN_FALSE,resp_costmap.message))
            return
        self.master_node_state.restartCalibratedLidarTF()
        self.master_node_state.changeState(MasterNodeState.state_type.Idle)#切换至Idle状态
        request_handler.write(MasterNodeErrorCode.Successed())
        rospy.loginfo("[Master_node.py] state_operation: %s",self.master_node_state.state_operation.value)

    ##################################MAPMANAGE SURFACE FUNTION##################################
    #Get all maps name
    def get_map_list(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RestartMapping,MasterNodeState.state_type.Mapping,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
        # if self.master_node_state.state_operation != MasterNodeState.state_type.Idle and self.master_node_state.state_operation != MasterNodeState.state_type.RunningTask:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        try:
            get_maps_srv = rospy.ServiceProxy("/map_manager_node/map_getMaplist", StrArray_resp)
            try:
                get_maps_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp  = get_maps_srv()
        except rospy.ServiceException as e:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        else:
            maps = {}
            maps['maps'] = []
            for map_name in resp.strlist :
                item ={}
                item["mapname"] = map_name
                maps['maps'].append(item)
            request_handler.write(MasterNodeErrorCode.SuccessedData(maps))
            return
        pass
    def compress_map_pngs(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        map_name = request_handler.get_query_argument('map_name',"")
        compress_width = request_handler.get_query_argument('compress_width', 128)
        # resize_scale = request_handler.get_query_argument('resize_scale',1)

        for map_info in self.maps_info:
            # if map_info.mapname == map_name:
            #     start = time.time()
            #     map_file_path = map_info.directory + map_info.mapId+'.png'
            #     im = Image.open(map_file_path)
            #     rospy.loginfo("[python image] E-open-time")
            #     width, height = im.size
            #     # width = int(width / float(resize_scale))
            #     # height = int(height / float(resize_scale))
            #     height = height * int(compress_width) / width
            #     width = int(compress_width)
            #     # im = transforms.Resize([width, height])(im)
            #     # im.thumbnail((width,height))
            #     im = im.resize((width, height))
            #     rospy.loginfo("[python image] F-save-time")
            #     im.save(map_file_path + '_new.png', quality = 95)
            #     with open(map_file_path + '_new.png','r') as pic:
            #         pics = pic.read()
            #     request_handler.write(pics)
            #     request_handler.set_header("Content-type","image/png")
            #     print("time:{:.4}".format((time.time() - start) * 1000))
            #     return

            #######OPENCV#############
            if map_info.mapname == map_name:
                map_file_path = map_info.directory + map_info.mapId+'.png'
                if(os.path.exists(map_file_path + '_new.png')):
                    print('map_file_path' + str(map_file_path))
                    with open(map_file_path + '_new.png','rb') as pic:
                        pics = pic.read()
                    request_handler.write(pics)
                    request_handler.set_header("Content-type","image/png")
                    return
                else:
                    im = cv2.imread(map_file_path)
                    rospy.loginfo("[python image] E-open-time")
                    size = im.shape
                    height = size[0]
                    width = size[1]
                    height = height * int(compress_width) / width
                    width = int(compress_width)
                    im = cv2.resize(im, (width, height), interpolation = cv2.INTER_LINEAR)
                    rospy.loginfo("[python image] F-save-time")
                    cv2.imwrite(map_file_path + '_new.png', im)
                    with open(map_file_path + '_new.png','r') as pic:
                        pics = pic.read()
                    request_handler.write(pics)
                    request_handler.set_header("Content-type","image/png")
                    return
        request_handler.write(MasterNodeErrorCode.FailedData(MasterNodeErrorCode.SERVICE_RETURN_FALSE,resp.message))
        return
        pass
    #Get appointed map's png file
    def maps_pngs(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        map_name = request_handler.get_query_argument('map_name')
        try:
            get_maps_data_srv = rospy.ServiceProxy("/map_manager_node/map_maps_data", console_service.getMapsData)
            try:
                get_maps_data_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp  = get_maps_data_srv(map_name)
        except rospy.ServiceException as e:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        else:
            maps = {}
            maps['maps'] = []
            for map_info in resp.mapsInfo:
                if map_info.mapname == map_name:
                    map_file_path = map_info.directory +map_info.mapId+'.png'
                    with open(map_file_path,'r') as pic:
                        pics = pic.read()
                    request_handler.write(pics)
                    request_handler.set_header("Content-Type", "application/octet-stream")
                    request_handler.set_header("Content-type","image/png")
                    return
            request_handler.write(MasterNodeErrorCode.FailedData(MasterNodeErrorCode.SERVICE_RETURN_FALSE,resp.message))
            return
        pass


    #Get tile map png
    def tile_maps_pngs(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        map_name = request_handler.get_query_argument('map_name')
        x = request_handler.get_query_argument('x')
        y = request_handler.get_query_argument('y')
        z = request_handler.get_query_argument('z')
        tile_index = z+"_"+y+"_"+x

        for map_info in self.maps_info:
          if map_info.mapname == map_name:
            map_file_path = map_info.directory +'tiles/' + tile_index+'.png'
            if not os.path.exists(map_file_path):
              request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.OPEN_FILE_FAILED))
              return
            with open(map_file_path,'r') as pic:
                pics = pic.read()
            request_handler.write(pics)
            request_handler.set_header("Content-Type", "application/octet-stream")
            request_handler.set_header("Content-type","image/png")
            return
        request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.OPEN_FILE_FAILED))
        return

    #Get current map Png file
    def get_map(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        runMapJson = self.run_time_dir + '/run_map.json'
        with open(runMapJson,'r') as load_f:
            load_dict = json.load(load_f)
            current_mapname = load_dict["id"]
        map_file_path = self.run_map_dir + '/' + current_mapname + "/" + current_mapname + '.png'
        with open(map_file_path,'r') as pic:
            pics = pic.read()
        request_handler.write(pics)
        request_handler.set_header("Content-type","image/png")
        return
        pass

    #change to the appointed map
    def load_map(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        map_name = request_handler.get_query_argument('map_name')
        if not map_name:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        try:
            change_maps_srv = rospy.ServiceProxy("/map_manager_node/map_change", String_req)
            change_map_resp  = change_maps_srv(map_name)
            clear_tasks_srv = rospy.ServiceProxy("/drobot_task_manager/clear_current_tasks", common_service.srv.Empty)
            clear_tasks_resp  = clear_tasks_srv()
        except rospy.ServiceException as e:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not change_map_resp.success or not clear_tasks_resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(change_map_resp))
            return
        else :
            request_handler.write(MasterNodeErrorCode.Successed())
            return
        pass


    def rename_map(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        origin_map_name = request_handler.get_query_argument('origin_map_name',"")
        new_map_name = request_handler.get_query_argument('new_map_name',"")
        if origin_map_name == "" or new_map_name == "":
          request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
          return
        try:
            rename_map_srv = rospy.ServiceProxy("/map_manager_node/map_rename", console_service.renameMap)
            try:
                rename_map_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp  = rename_map_srv(origin_map_name,new_map_name)
        except rospy.ServiceException as e:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        return

    #get all map's information
    def get_map_info(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.Mapping,MasterNodeState.state_type.RestartMapping,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        map_name = request_handler.get_query_argument('map_name',"")
        try:
            get_maps_data_srv = rospy.ServiceProxy("/map_manager_node/map_maps_data", console_service.getMapsData)
            try:
                get_maps_data_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp  = get_maps_data_srv(map_name)
        except rospy.ServiceException as e:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        else:
            maps = {}
            maps['maps'] = []
            self.maps_info = resp.mapsInfo
            for index in range(len(resp.mapsInfo)):
                item ={}
                item["mapname"] = resp.mapsInfo[index].mapname
                item["mapId"] = resp.mapsInfo[index].mapId
                item["gridWidth"] = resp.mapsInfo[index].gridWidth
                item["gridHeight"] = resp.mapsInfo[index].gridHeight
                item["originX"] = resp.mapsInfo[index].originX
                item["originY"] = resp.mapsInfo[index].originY
                item["resolution"] = resp.mapsInfo[index].resolution
                item["pngMD5"] = resp.mapsInfo[index].pngMD5
                item["maxZoom"] = resp.mapsInfo[index].max_zoom
                item["minBoxSize"] = resp.mapsInfo[index].min_box_size
                maps['maps'].append(item)
            request_handler.write(MasterNodeErrorCode.SuccessedData(maps))
            return
        pass

    def get_events_info(self,request_handler):

        page_num = request_handler.get_query_argument('page_num',"")
        page_size = request_handler.get_query_argument('page_size',"")

        #Executed ROS service
        try:
            get_events_data_srv = rospy.ServiceProxy("/drobot_event_manager/get_exception_event", console_service.GetExcpEvent)
            try:
                get_events_data_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp  = get_events_data_srv(int(page_num), int(page_size))
        except rospy.ServiceException as e:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return

        #return GET/POST result
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
        else:
            ret_total = {}
            num_block = {}
            data = []
            for index in range(len(resp.row_value)):
                string = resp.column_name[index % len(resp.column_name)]
                num_block[string] = resp.row_value[index]
                if (index + 1) % len(resp.column_name) == 0:
                    data.append(num_block)
                    num_block = {}
                else:
                    continue
            ret_total['data'] = data
            ret_total['total_num'] = resp.total_num

            request_handler.write(MasterNodeErrorCode.SuccessedData(ret_total))
        return

    def current_map_info(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization,MasterNodeState.state_type.Mapping]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        if not self.master_node_subscriber.check_reviced('map_info'):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NO_RUNNING_MAP))
            return
        if self.master_node_subscriber.submsg_table['map_info'].mapname == "scanning_map":
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NO_RUNNING_MAP))
            return
        map_info = {}
        map_info['gridWidth'] = self.master_node_subscriber.submsg_table['map_info'].gridWidth
        map_info['gridHeight'] = self.master_node_subscriber.submsg_table['map_info'].gridHeight
        map_info['name'] = self.master_node_subscriber.submsg_table['map_info'].mapname
        map_info['mapId'] = self.master_node_subscriber.submsg_table['map_info'].mapId
        map_info['originX'] = self.master_node_subscriber.submsg_table['map_info'].originX
        map_info['originY'] = self.master_node_subscriber.submsg_table['map_info'].originY
        map_info['resolution'] = self.master_node_subscriber.submsg_table['map_info'].resolution
        map_info['pngMD5'] = self.master_node_subscriber.submsg_table['map_info'].pngMD5
        map_info["maxZoom"] = self.master_node_subscriber.submsg_table['map_info'].max_zoom
        map_info["minBoxSize"] = self.master_node_subscriber.submsg_table['map_info'].min_box_size
        request_handler.write(MasterNodeErrorCode.SuccessedData(map_info))
        return
    #delete the appointed map
    def delete_map(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        map_name = request_handler.get_query_argument('map_name')
        try:
            delete_maps_srv = rospy.ServiceProxy("/map_manager_node/map_delete", String_req)
            resp  = delete_maps_srv(map_name)
        except rospy.ServiceException as e:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        else :
            request_handler.write(MasterNodeErrorCode.Successed())
            return
        pass
    def download_map(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        map_name = request_handler.get_query_argument('map_name')
        try:
            download_maps_srv = rospy.ServiceProxy("/map_manager_node/map_download", console_service.packageMap)
            resp  = download_maps_srv(map_name)
        except rospy.ServiceException as e:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        else:
            request_handler.write(resp.package_file)
            request_handler.set_header("Access-Control-Allow-Origin", "*")
            request_handler.set_header("Access-Control-Allow-Methods", "PUT, POST, GET, OPTIONS, DELETE")
            request_handler.set_header("Content-Type", "application/octet-stream")
            disp = "attachment;filename=" + map_name+".zip";
            request_handler.set_header("Content-Disposition", disp)
            return
    def upload_map(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        map_name = request_handler.get_query_argument('map_name')
        if not len(request_handler.request.body) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        if not map_name:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        try:
            upload_maps_srv = rospy.ServiceProxy("/map_manager_node/map_upload", console_service.uploadMap)
            resp  = upload_maps_srv(map_name,request_handler.request.body)
        except rospy.ServiceException as e:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        return
        pass
    def edit_map(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        if not self.master_node_subscriber.check_reviced('map_info'):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NO_RUNNING_MAP))
            return
        map_name = self.master_node_subscriber.submsg_table['map_info'].mapname
        operation_type=request_handler.get_query_argument('operation_type','')
        if not operation_type:
          request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
          return
        if operation_type != "restore":
          if not len(request_handler.request.body) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        try:
            obstacle = request_handler.request.body
            edit_maps_srv = rospy.ServiceProxy("/map_manager_node/map_edit", console_service.editMap)
            resp  = edit_maps_srv(map_name,operation_type,obstacle)
        except rospy.ServiceException as e:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        return

    #跟随目标模块
    def start_follow_people(self,request_handler):
        none = []
        self.master_node_state.changeState(MasterNodeState.state_type.FollowTarget, none, "people")
        request_handler.write(MasterNodeErrorCode.Successed())
        return

    def start_follow_car(self,request_handler):
        none = []
        self.master_node_state.changeState(MasterNodeState.state_type.FollowTarget, none, "car")
        request_handler.write(MasterNodeErrorCode.Successed())
        return

    def stop_follow_target(self,request_handler):
        if (self.master_node_state.state_operation != MasterNodeState.state_type.FollowTarget):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        self.master_node_state.changeState(MasterNodeState.state_type.Idle)
        request_handler.write(MasterNodeErrorCode.Successed())
        return
    ################################## POSE INIITIALIZE SURFACE FUNTION##################################


# 通过传入点的信息初始化
    def check_initialze(self):
        time.sleep(0.5)
        self.master_node_state.state_initialized = False
        while not self.master_node_state.state_initialized:
            self.master_node_state.state_initialized = self.master_node_subscriber.submsg_table['localization_status']['initialized']
            status = self.master_node_subscriber.submsg_table['localization_status']['status']
            if status == "laser check failed":
                rospy.loginfo("check laser failed")
                break
            time.sleep(0.5)
        if self.master_node_state.state_initialized:
            rospy.loginfo("[Master_node.py] current initializa status is True")
        else:
            rospy.loginfo("[Master_node.py] current initializa status is False")
        if self.stop_move_flag == False:
            rospy.loginfo("[Master_node.py] stop rotate")
            self.stop_move_flag = True

##################################Funtion for Obstacle opretation##################################
#add a obstacles
    def add_virtual_obstacles(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        if len(request_handler.request.body) > 0:
            virtual_obstacles_msg = json.loads(request_handler.request.body)
            if not 'obstacles' in virtual_obstacles_msg.keys():
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
                return
            if not self.master_node_subscriber.submsg_table['map_info']:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NO_RUNNING_MAP))
                return
            map_info = {}
            map_info['mapname'] = self.master_node_subscriber.submsg_table['map_info'].mapname
            map_info['originX'] = self.master_node_subscriber.submsg_table['map_info'].originX
            map_info['originY'] = self.master_node_subscriber.submsg_table['map_info'].originY
            map_info['resolution'] = self.master_node_subscriber.submsg_table['map_info'].resolution
            map_info['gridWidth'] = self.master_node_subscriber.submsg_table['map_info'].gridWidth
            map_info['gridHeight'] = self.master_node_subscriber.submsg_table['map_info'].gridHeight
            map_info['mapdirectory'] = self.master_node_subscriber.submsg_table['map_info'].directory
            if virtual_obstacles_msg['obstacles'].has_key('circles'):
              for circle in virtual_obstacles_msg['obstacles']['circles']:
                pose = helper.Coordinates.GridToPose(helper.Point(circle['center']['x'],circle['center']['y'],0),\
                                                    map_info['originX'],map_info['originY'],map_info['resolution'])
                circle['center']['x'] = pose.position.x
                circle['center']['y'] = pose.position.y
                circle['radius'] = circle['radius']*map_info['resolution']
            if virtual_obstacles_msg['obstacles'].has_key('lines'):
              for line in virtual_obstacles_msg['obstacles']['lines']:
                startPose = helper.Coordinates.GridToPose(helper.Point(line['start']['x'],line['start']['y'],0),\
                                                    map_info['originX'],map_info['originY'],map_info['resolution'])
                endPose = helper.Coordinates.GridToPose(helper.Point(line['end']['x'],line['end']['y'],0),\
                                                    map_info['originX'],map_info['originY'],map_info['resolution'])
                line['start']['x'] = startPose.position.x
                line['start']['y'] = startPose.position.y
                line['end']['x'] = endPose.position.x
                line['end']['y'] = endPose.position.y
            if virtual_obstacles_msg['obstacles'].has_key('polygons'):
              for polygon in virtual_obstacles_msg['obstacles']['polygons']:
                for point in polygon:
                  pose = helper.Coordinates.GridToPose(helper.Point(point['x'],point['y'],0),\
                                                    map_info['originX'],map_info['originY'],map_info['resolution'])
                  point['x'] = pose.position.x
                  point['y'] = pose.position.y
            if virtual_obstacles_msg['obstacles'].has_key('polylines'):
              for polyline in virtual_obstacles_msg['obstacles']['polylines']:
                for point in polyline:
                  pose = helper.Coordinates.GridToPose(helper.Point(point['x'],point['y'],0),\
                                                    map_info['originX'],map_info['originY'],map_info['resolution'])
                  point['x'] = pose.position.x
                  point['y'] = pose.position.y
            if virtual_obstacles_msg['obstacles'].has_key('rectangles'):
              for rectangle in virtual_obstacles_msg['obstacles']['rectangles']:
                  startPose = helper.Coordinates.GridToPose(helper.Point(rectangle['start']['x'],rectangle['start']['y'],0),\
                                                      map_info['originX'],map_info['originY'],map_info['resolution'])
                  endPose = helper.Coordinates.GridToPose(helper.Point(rectangle['end']['x'],rectangle['end']['y'],0),\
                                                      map_info['originX'],map_info['originY'],map_info['resolution'])
                  rectangle['start']['x'] = startPose.position.x
                  rectangle['start']['y'] = startPose.position.y
                  rectangle['end']['x'] = endPose.position.x
                  rectangle['end']['y'] = endPose.position.y
            if virtual_obstacles_msg.has_key('ramps') and virtual_obstacles_msg['ramps'].has_key('polygons'):
              for polygon in virtual_obstacles_msg['ramps']['polygons']:
                area = polygon['area']
                for point in area:
                  pose = helper.Coordinates.GridToPose(helper.Point(point['x'],point['y'],0),\
                                                    map_info['originX'],map_info['originY'],map_info['resolution'])
                  point['x'] = pose.position.x
                  point['y'] = pose.position.y
            if virtual_obstacles_msg.has_key('slowzones') and virtual_obstacles_msg['slowzones'].has_key('polygons'):
              for polygon in virtual_obstacles_msg['slowzones']['polygons']:
                area = polygon['area']
                for point in area:
                  pose = helper.Coordinates.GridToPose(helper.Point(point['x'],point['y'],0),\
                                                    map_info['originX'],map_info['originY'],map_info['resolution'])
                  point['x'] = pose.position.x
                  point['y'] = pose.position.y
            if virtual_obstacles_msg.has_key('crossroads') and virtual_obstacles_msg['crossroads'].has_key('polygons'):
              for polygon in virtual_obstacles_msg['crossroads']['polygons']:
                area = polygon['area']
                for point in area:
                  pose = helper.Coordinates.GridToPose(helper.Point(point['x'],point['y'],0),\
                                                    map_info['originX'],map_info['originY'],map_info['resolution'])
                  point['x'] = pose.position.x
                  point['y'] = pose.position.y
            if virtual_obstacles_msg.has_key('noobstacles') and virtual_obstacles_msg['noobstacles'].has_key('polygons'):
              for polygon in virtual_obstacles_msg['noobstacles']['polygons']:
                area = polygon['area']
                for point in area:
                  pose = helper.Coordinates.GridToPose(helper.Point(point['x'],point['y'],0),\
                                                    map_info['originX'],map_info['originY'],map_info['resolution'])
                  point['x'] = pose.position.x
                  point['y'] = pose.position.y
            if virtual_obstacles_msg.has_key('force_avoid_obstacle_zone') and virtual_obstacles_msg['force_avoid_obstacle_zone'].has_key('polygons'):
              for polygon in virtual_obstacles_msg['force_avoid_obstacle_zone']['polygons']:
                area = polygon['area']
                for point in area:
                  pose = helper.Coordinates.GridToPose(helper.Point(point['x'],point['y'],0),\
                                                    map_info['originX'],map_info['originY'],map_info['resolution'])
                  point['x'] = pose.position.x
                  point['y'] = pose.position.y
            if virtual_obstacles_msg.has_key('ban_camera_zone') and virtual_obstacles_msg['ban_camera_zone'].has_key('polygons'):
              for polygon in virtual_obstacles_msg['ban_camera_zone']['polygons']:
                area = polygon['area']
                for point in area:
                  pose = helper.Coordinates.GridToPose(helper.Point(point['x'],point['y'],0),\
                                                    map_info['originX'],map_info['originY'],map_info['resolution'])
                  point['x'] = pose.position.x
                  point['y'] = pose.position.y
            if virtual_obstacles_msg.has_key('ban_ultra_zone') and virtual_obstacles_msg['ban_ultra_zone'].has_key('polygons'):
              for polygon in virtual_obstacles_msg['ban_ultra_zone']['polygons']:
                area = polygon['area']
                for point in area:
                  pose = helper.Coordinates.GridToPose(helper.Point(point['x'],point['y'],0),\
                                                    map_info['originX'],map_info['originY'],map_info['resolution'])
                  point['x'] = pose.position.x
                  point['y'] = pose.position.y
            obstacle_path = map_info['mapdirectory']
            obstacles =obstacle_path+"virobstacles.json"
            if not os.path.isdir(obstacle_path):
                os.makedirs(obstacles)
            try:
                with open(obstacles, 'w') as f:
                    json.dump(virtual_obstacles_msg, f)
                    # self.obstacles_pub_.publish(1)
                    request_handler.write(MasterNodeErrorCode.Successed())
                    return
            except IOError:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.OPEN_FILE_FAILED))
                return
        else:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_BODY_IS_EMPTY))
            return
        pass

    #get all obstacles information
    def get_virtual_obstacles(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        if not self.master_node_subscriber.submsg_table['map_info']:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NO_RUNNING_MAP))
            return
        map_info = {}
        map_info['mapname'] = self.master_node_subscriber.submsg_table['map_info'].mapname
        map_info['originX'] = self.master_node_subscriber.submsg_table['map_info'].originX
        map_info['originY'] = self.master_node_subscriber.submsg_table['map_info'].originY
        map_info['resolution'] = self.master_node_subscriber.submsg_table['map_info'].resolution
        map_info['gridWidth'] = self.master_node_subscriber.submsg_table['map_info'].gridWidth
        map_info['gridHeight'] = self.master_node_subscriber.submsg_table['map_info'].gridHeight
        map_info['mapdirectory'] = self.master_node_subscriber.submsg_table['map_info'].directory
        obstacle_path = map_info['mapdirectory']
        obstacles =obstacle_path+"virobstacles.json"
        virtual_obstacles_msg={}
        if not os.path.isfile(obstacles):
            virtual_obstacles_msg = {
              'obstacles': {'circles': [],'lines': [],'polygons': [],'polylines': [],'rectangles': []},
              'ramps': {'polygons': []},
              'ban_camera_zone': {'polygons': []},
              'ban_ultra_zone': {'polygons': []},
              'slowzones': {'polygons': []},
              'crossroads': {'polygons': []},
              'noobstacles': {'polygons': []},
              'force_avoid_obstacle_zone': {'polygons': []}
            }
            request_handler.write(MasterNodeErrorCode.SuccessedData(virtual_obstacles_msg))
            return
        else:
            with open(obstacles, 'r') as f:
                virtual_obstacles_msg= json.load(f)
                if virtual_obstacles_msg['obstacles'].has_key('circles'):
                  for circle in virtual_obstacles_msg['obstacles']['circles']:
                    center_pose = geometry_msgs.msg.Pose()
                    center_pose.position.x=circle['center']['x']
                    center_pose.position.y=circle['center']['y']
                    pose = helper.Coordinates.PoseToGrid(center_pose,map_info['originX'],map_info['originY'],map_info['resolution'])
                    circle['center']['x'] = pose.x
                    circle['center']['y'] = pose.y
                    circle['radius'] = circle['radius']/map_info['resolution']
                if virtual_obstacles_msg['obstacles'].has_key('lines'):
                  for line in virtual_obstacles_msg['obstacles']['lines']:
                    start_pose = geometry_msgs.msg.Pose()
                    end_pose = geometry_msgs.msg.Pose()
                    start_pose.position.x = line['start']['x']
                    start_pose.position.y = line['start']['y']
                    end_pose.position.x = line['end']['x']
                    end_pose.position.y = line['end']['y']
                    startPose = helper.Coordinates.PoseToGrid(start_pose,map_info['originX'],map_info['originY'],map_info['resolution'])
                    endPose = helper.Coordinates.PoseToGrid(end_pose,map_info['originX'],map_info['originY'],map_info['resolution'])
                    line['start']['x'] = startPose.x
                    line['start']['y'] = startPose.y
                    line['end']['x'] = endPose.x
                    line['end']['y'] = endPose.y
                if virtual_obstacles_msg['obstacles'].has_key('polygons'):
                  for polygon in virtual_obstacles_msg['obstacles']['polygons']:
                    for point in polygon:
                      worldpose = geometry_msgs.msg.Pose()
                      worldpose.position.x = point['x']
                      worldpose.position.y = point['y']
                      pose = helper.Coordinates.PoseToGrid(worldpose,map_info['originX'],map_info['originY'],map_info['resolution'])
                      point['x'] = pose.x
                      point['y'] = pose.y
                if virtual_obstacles_msg['obstacles'].has_key('polylines'):
                  for polyline in virtual_obstacles_msg['obstacles']['polylines']:
                    for point in polyline:
                      worldpose = geometry_msgs.msg.Pose()
                      worldpose.position.x = point['x']
                      worldpose.position.y = point['y']
                      pose = helper.Coordinates.PoseToGrid(worldpose,map_info['originX'],map_info['originY'],map_info['resolution'])
                      point['x'] = pose.x
                      point['y'] = pose.y
                if virtual_obstacles_msg['obstacles'].has_key('rectangles'):
                  for rectangle in virtual_obstacles_msg['obstacles']['rectangles']:
                      start_pose = geometry_msgs.msg.Pose()
                      end_pose = geometry_msgs.msg.Pose()
                      start_pose.position.x = rectangle['start']['x']
                      start_pose.position.y = rectangle['start']['y']
                      end_pose.position.x = rectangle['end']['x']
                      end_pose.position.y = rectangle['end']['y']
                      startPose = helper.Coordinates.PoseToGrid(start_pose,map_info['originX'],map_info['originY'],map_info['resolution'])
                      endPose = helper.Coordinates.PoseToGrid(end_pose,map_info['originX'],map_info['originY'],map_info['resolution'])
                      rectangle['start']['x'] = startPose.x
                      rectangle['start']['y'] = startPose.y
                      rectangle['end']['x'] = endPose.x
                      rectangle['end']['y'] = endPose.y
                if virtual_obstacles_msg.has_key('ramps') and virtual_obstacles_msg['ramps'].has_key('polygons'):
                  for polygon in virtual_obstacles_msg['ramps']['polygons']:
                    area = polygon['area']
                    for point in area:
                      worldpose = geometry_msgs.msg.Pose()
                      worldpose.position.x = point['x']
                      worldpose.position.y = point['y']
                      pose = helper.Coordinates.PoseToGrid(worldpose,map_info['originX'],map_info['originY'],map_info['resolution'])
                      point['x'] = pose.x
                      point['y'] = pose.y
                if virtual_obstacles_msg.has_key('slowzones') and virtual_obstacles_msg['slowzones'].has_key('polygons'):
                  for polygon in virtual_obstacles_msg['slowzones']['polygons']:
                    area = polygon['area']
                    for point in area:
                      worldpose = geometry_msgs.msg.Pose()
                      worldpose.position.x = point['x']
                      worldpose.position.y = point['y']
                      pose = helper.Coordinates.PoseToGrid(worldpose,map_info['originX'],map_info['originY'],map_info['resolution'])
                      point['x'] = pose.x
                      point['y'] = pose.y
                if virtual_obstacles_msg.has_key('crossroads') and virtual_obstacles_msg['crossroads'].has_key('polygons'):
                  for polygon in virtual_obstacles_msg['crossroads']['polygons']:
                    area = polygon['area']
                    for point in area:
                      worldpose = geometry_msgs.msg.Pose()
                      worldpose.position.x = point['x']
                      worldpose.position.y = point['y']
                      pose = helper.Coordinates.PoseToGrid(worldpose,map_info['originX'],map_info['originY'],map_info['resolution'])
                      point['x'] = pose.x
                      point['y'] = pose.y
                if virtual_obstacles_msg.has_key('noobstacles') and virtual_obstacles_msg['noobstacles'].has_key('polygons'):
                  for polygon in virtual_obstacles_msg['noobstacles']['polygons']:
                    area = polygon['area']
                    for point in area:
                      worldpose = geometry_msgs.msg.Pose()
                      worldpose.position.x = point['x']
                      worldpose.position.y = point['y']
                      pose = helper.Coordinates.PoseToGrid(worldpose,map_info['originX'],map_info['originY'],map_info['resolution'])
                      point['x'] = pose.x
                      point['y'] = pose.y
                if virtual_obstacles_msg.has_key('force_avoid_obstacle_zone') and virtual_obstacles_msg['force_avoid_obstacle_zone'].has_key('polygons'):
                  for polygon in virtual_obstacles_msg['force_avoid_obstacle_zone']['polygons']:
                    area = polygon['area']
                    for point in area:
                      worldpose = geometry_msgs.msg.Pose()
                      worldpose.position.x = point['x']
                      worldpose.position.y = point['y']
                      pose = helper.Coordinates.PoseToGrid(worldpose,map_info['originX'],map_info['originY'],map_info['resolution'])
                      point['x'] = pose.x
                      point['y'] = pose.y
                if virtual_obstacles_msg.has_key('ban_camera_zone') and virtual_obstacles_msg['ban_camera_zone'].has_key('polygons'):
                  for polygon in virtual_obstacles_msg['ban_camera_zone']['polygons']:
                    area = polygon['area']
                    for point in area:
                      worldpose = geometry_msgs.msg.Pose()
                      worldpose.position.x = point['x']
                      worldpose.position.y = point['y']
                      pose = helper.Coordinates.PoseToGrid(worldpose,map_info['originX'],map_info['originY'],map_info['resolution'])
                      point['x'] = pose.x
                      point['y'] = pose.y
                if virtual_obstacles_msg.has_key('ban_ultra_zone') and virtual_obstacles_msg['ban_ultra_zone'].has_key('polygons'):
                  for polygon in virtual_obstacles_msg['ban_ultra_zone']['polygons']:
                    area = polygon['area']
                    for point in area:
                      worldpose = geometry_msgs.msg.Pose()
                      worldpose.position.x = point['x']
                      worldpose.position.y = point['y']
                      pose = helper.Coordinates.PoseToGrid(worldpose,map_info['originX'],map_info['originY'],map_info['resolution'])
                      point['x'] = pose.x
                      point['y'] = pose.y
                request_handler.write(MasterNodeErrorCode.SuccessedData(virtual_obstacles_msg))
                return
        pass


################################## NAVIGATION SURFACE FUNCTION##################################
#start navigation
    def start_navigation(self, request_handler):
        self.master_node_state.changeState(MasterNodeState.state_type.RunningTask)
        #新建一个线程
        thread.start_new_thread(self.protector_navigation_process,())
        request_handler.write(MasterNodeErrorCode.Successed())
        self.start_navigation_flag_  = True
        return
    #线程函数：保护导航过程
    def protector_navigation_process(self):
        #获取目前命名空间的参数
        protect = rospy.get_param('~protect',True)
        while self.master_node_state.state_operation == MasterNodeState.state_operation.RunningTask and ( not rospy.is_shutdown() ) and protect:
            rospy.sleep(1)
            #检测导航程序是否异常死亡,检测到异常死亡时停止机器人移动
            if True and self.master_node_ping.ping_bt_navigator_node() != 'alive':
                rospy.logerr("[Master_node.py] navigation node is error stoped")
                #导航程序崩溃,机器停止
                twist = Twist()
                twist.linear.x = 0
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.z = 0
                self.cmd_pub.publish(twist)
                self.stop_other_task()
                self.master_node_state.changeState(MasterNodeState.state_type.Idle)
                break
    #取消导航
    def cancel_navigation(self, request_handler):
        #判断状态机状态
        if self.master_node_state.state_operation != MasterNodeState.state_type.RunningTask:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        self.stop_other_task()
        self.master_node_state.changeState(MasterNodeState.state_type.Idle)
        request_handler.write(MasterNodeErrorCode.Successed())
        return
    #移动？？
    def move_to(self, request_handler):
        if self.master_node_state.state_operation != MasterNodeState.state_type.RunningTask:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        if not self.master_node_subscriber.submsg_table['localization_status']['initialized']:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.WAITING_INITIALIZED))
            return
        pos_name = request_handler.get_query_argument('pose_name','')
        navigation_goal = console_service.NavToPointRequest()
        if not pos_name:
            if not len(request_handler.request.body) > 0:
                request_handler.write(MasterNodeErrorCode.FailedData(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
            goal_pose_msg = json.loads(request_handler.request.body)
            if not 'position' in goal_pose_msg.keys() and not "type" in goal_pose_msg.keys():
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
                return
            position = goal_pose_msg['position'] #获取position的结构
            if ( not  'point' in position.keys()  ) or ( not 'angle' in position.keys() ):
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
                return
            point = position['point']
            if (not 'x' in point.keys() )  or (not 'y' in point.keys() ):
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
                return
            navigation_goal.point.pose_name = "__DEFAULT"
            navigation_goal.point.gridX = point['x']
            navigation_goal.point.gridY = point['y']
            navigation_goal.point.angle = position['angle']
            navigation_goal.point.point_type.type = goal_pose_msg['type']
        else:
            navigation_goal.point.pose_name = pos_name
        try:
            navigation_pose_srv = rospy.ServiceProxy("/drobot_task_manager/start_navigation",console_service.NavToPoint)
            resp = navigation_pose_srv(navigation_goal)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.FailedData(MasterNodeErrorCode.SERVICE_RETURN_FALSE,resp.message))
            return
        else:
            self.master_node_subscriber.submsg_recived_table['path'] = False
            self.master_node_subscriber.submsg_table['path'] = {}
            request_handler.write(MasterNodeErrorCode.Successed())
            self.start_navigation_flag_ = True
            return
        pass
    #停止移动
    def pause_move_to(self,request_handler):
        if self.master_node_state.state_operation != MasterNodeState.state_type.RunningTask:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        try:
            pause_task_srv = rospy.ServiceProxy("/drobot_task_manager/pause_task",common_service.srv.Empty)
            try:
                #等待服务端准备完成
                pause_task_srv.wait_for_service(1.0)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            res = pause_task_srv()
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        return
    #继续移动
    def resume_move_to(self,request_handler):
        if self.master_node_state.state_operation != MasterNodeState.state_type.RunningTask:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        try:
            resume_task_srv = rospy.ServiceProxy("/drobot_task_manager/resume_task",common_service.srv.Empty)
            try:
                resume_task_srv.wait_for_service(1.0)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            res = resume_task_srv()
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        return
    #取消移动
    def cancel_move_to(self,request_handler):
        if self.master_node_state.state_operation != MasterNodeState.state_type.RunningTask:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        try:
            stop_task_srv = rospy.ServiceProxy("/drobot_task_manager/stop_task",common_service.srv.Empty)
            try:
                stop_task_srv.wait_for_service(1.0)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            res = stop_task_srv()
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        return

    #跟随路径
    def follow_path(self, request_handler):
        if self.master_node_state.state_operation != MasterNodeState.state_type.RunningTask:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        if not self.master_node_subscriber.submsg_table['localization_status']['initialized']:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.WAITING_INITIALIZED))
            return
        path_name = request_handler.get_query_argument('path_name')
        result_msg = {}
        if not path_name :
            request_handler.write(MasterNodeErrorCode.FailedData(MasterNodeErrorCode.PARAM_FORMAT_ERROR,\
            "Failed: Incorrect parameters. No path name was passed"))
            return
        else:
            #now we have parameters in right form first try to get path
            try:
                #？？
                fix_path_goal = String_reqRequest()
                fix_path_goal.str = path_name
                fix_path_srv = rospy.ServiceProxy("/drobot_task_manager/start_follow_path",String_req)
                try:
                    fix_path_srv.wait_for_service(1.0)
                except rospy.ROSException as servexc:
                    request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                    return
                res = fix_path_srv(fix_path_goal)
            except rospy.ServiceException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
                return
            if False == res.success:
                request_handler.write(MasterNodeErrorCode.FailedData(MasterNodeErrorCode.SERVICE_RETURN_FALSE,res.message))
                return
                #we get path now call follow path service
                # self.master_node_subscriber.submsg_table['bt_navigator_status'] = {}
                # self.current_following_path = fix_path_goal
                # self.following_path_flag = True
                # for pause_pose in self.pause_points:
                #     pause_pose['paused'] = False
#                    self.start_play_path = True
#                    self.play_sound(path_name)
            self.master_node_subscriber.submsg_recived_table['path'] = False
            self.master_node_subscriber.submsg_table['path'] = {}
            request_handler.write(MasterNodeErrorCode.Successed())
            self.start_navigation_flag_ = True
            return
        pass
    #清除区
    def clean_area(self, request_handler):
        if self.master_node_state.state_operation != MasterNodeState.state_type.RunningTask:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        area_name = request_handler.get_query_argument('area_name','')
        try:
            srv_msg = String_reqRequest()
            srv_msg.str = area_name
            service = rospy.ServiceProxy("/drobot_task_manager/start_clean_area",String_req)
            try:
                service.wait_for_service(1.0)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            res = service(srv_msg)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if False == res.success:
            request_handler.write(MasterNodeErrorCode.FailedData(MasterNodeErrorCode.SERVICE_RETURN_FALSE,res.message))
            return
        self.master_node_subscriber.submsg_recived_table['path'] = False
        self.master_node_subscriber.submsg_table['path'] = {}
        request_handler.write(MasterNodeErrorCode.Successed())
        return
        pass
    #清除路径
    def cancel_follow_path(self, request_handler):
        if self.master_node_state.state_operation != MasterNodeState.state_type.RunningTask:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        try:
            stop_task_srv = rospy.ServiceProxy("/drobot_task_manager/stop_task",common_service.srv.Empty)
            try:
                stop_task_srv.wait_for_service(1.0)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            res = stop_task_srv()
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        return
        pass
    #获得导航状态
    def get_navigator_status(self,request_handler):
        if self.master_node_state.state_operation != MasterNodeState.state_type.RunningTask:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        if not self.master_node_subscriber.check_reviced('bt_navigator_status'):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NOT_RECIVED_MSG))
            return
        status_bit = self.master_node_subscriber.submsg_table['bt_navigator_status']
        request_handler.write(MasterNodeErrorCode.SuccessedData(status_bit))
        return
        pass
    #获得实时路径
    def get_realtime_path(self,request_handler):
        if not self.master_node_subscriber.check_reviced('path'):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NOT_RECIVED_MSG))
            return
        request_handler.write(MasterNodeErrorCode.SuccessedData(self.master_node_subscriber.submsg_table['path']))
        pass

    def get_time_task(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.RunningTask]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        # try:
        #     get_time_task_req = String_respRequest()
        #     get_time_task_srv = rospy.ServiceProxy('/drobot_schedule_manager/query_schedule',String_resp)
        #     resp = get_time_task_srv(get_time_task_req)
        # except rospy.ServiceException as servexc:
        #     request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
        #     return
        # if not resp.success:
        #     request_handler.write(MasterNodeErrorCode.GetReturn(resp))
        #     return
        # time_task = {}
        # time_task['scheduleList'] = []
        # time_task_msg = json.loads(resp.str)
        # if len(time_task_msg) > 0:
        #     for schedule in time_task_msg:
        schedule_file = self.run_time_dir + '/schedule.json'
        time_task = {}
        time_task['scheduleList'] = []
        try:
            with open(schedule_file,'r') as load_f:
                schedule_josn = json.load(load_f)
        except IOError:
            request_handler.write(MasterNodeErrorCode.SuccessedData(time_task))
            return
        if len(schedule_josn["schedule"]) > 0:
            for schedule in schedule_josn["schedule"]:
                item ={}
                item["id"] = schedule["id"]
                item["name"] = schedule["name"]
                item["cronString"] = schedule["cronstring"]
                item["type"] = schedule["type"]
                item["param"] = {}
                item["param"]["mapName"] = schedule["param"]["mapName"]
                item["param"]["mapId"] = schedule["param"]["mapId"]
                item["param"]["name"] = schedule["param"]["name"]
                item["sw"] = schedule["switch"]
                time_task['scheduleList'].append(item)
        request_handler.write(MasterNodeErrorCode.SuccessedData(time_task))
        pass

    def add_time_task(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.RunningTask]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        if len(request_handler.request.body) <= 0:
            request_handler.write(MasterNodeErrorCode.FailedData(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
        time_task_msg = json.loads(request_handler.request.body)
        try:
            add_task_req = console_service.AddTaskScheduleRequest()
            add_task_req.name = time_task_msg["name"]
            add_task_req.task_name = time_task_msg["taskName"]
            add_task_req.cron = time_task_msg["cronString"]
            add_task_req.repeat = time_task_msg["repeat"]
            add_task_req.force = time_task_msg["force"]
            # rospy.logerr("[Master_node.py] --------------------------------------------------------------")
            # print("add_task_req",add_task_req)

            add_time_task_srv = rospy.ServiceProxy('/drobot_schedule_manager/add_task_schedule',console_service.AddTaskSchedule)
            resp = add_time_task_srv(add_task_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        pass

    def delete_time_task(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.RunningTask]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        time_task_id = request_handler.get_query_argument('id','')
        if time_task_id== "":
            request_handler.write(MasterNodeErrorCode.FailedData(MasterNodeErrorCode.PARAM_IS_EMPTY))
        rospy.logerr("[Master_node.py] --------------------------------------------------------------")
        try:
            del_task_req = String_reqRequest()
            del_task_req.str = time_task_id
            del_time_task_srv = rospy.ServiceProxy('/drobot_schedule_manager/remove_schedule',String_req)
            resp = del_time_task_srv(del_task_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        pass

    def beat_button(self,request_handler):
        beat_cnt = 0
        while beat_cnt < 10:
            beat_cnt = beat_cnt + 1
            self.beat_pub.publish()
            rospy.sleep(0.05)
        self.beat_pub.publish()
        request_handler.write(MasterNodeErrorCode.Successed())
        pass

    def set_auto_mode(self,request_handler):
        target_mode = request_handler.get_query_argument('mode', '-1')
        rospy.loginfo("get mode ")
        target_mode = int(target_mode)
        rospy.loginfo("set mode to %d", target_mode)
        if target_mode != 0 and target_mode != 1:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_INVALID_ERROR))
            return

        set_auto_mode_cnt = 0
        auto_mode_msg = std_msgs.msg.Int8()
        auto_mode_msg.data = target_mode
        while set_auto_mode_cnt < 10:
            set_auto_mode_cnt = set_auto_mode_cnt + 1
            self.set_auto_mode_pub.publish(auto_mode_msg)
            rospy.sleep(0.05)
        self.set_auto_mode_pub.publish(auto_mode_msg)
        request_handler.write(MasterNodeErrorCode.Successed())
        pass


##################################FUNCTION OF PATH MANAGE##################################
#start record path
    def start_record_path(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.RunningTask]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        try:
            record_path_srv = rospy.ServiceProxy("/drobot_path_manager/start_record",common_service.srv.Empty)
            # record_path_srv = rospy.ServiceProxy("path_record_node/startRecord",Empty)
            try:
                record_path_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = record_path_srv(EmptyRequest())
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        return

#save record path
    def save_record_path(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.RunningTask]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        record_path_name = request_handler.get_query_argument("path_name")
        if not record_path_name:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        else:
            try:
                save_path_req = String_reqRequest()
                save_path_req.str = record_path_name
                save_path_srv = rospy.ServiceProxy('/drobot_path_manager/stop_record',String_req)
                resp = save_path_srv(save_path_req)
            except rospy.ServiceException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
                return
            if not resp.success:
                request_handler.write(MasterNodeErrorCode.GetReturn(resp))
                return
        request_handler.write(MasterNodeErrorCode.Successed())
        return
        pass

    #cancle and not save record path
    def cancel_record_path(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.RunningTask]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        try:
            record_path_srv = rospy.ServiceProxy("/drobot_path_manager/cancel_record",common_service.srv.Empty)
            try:
                record_path_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = record_path_srv(EmptyRequest())
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        return
    def get_record_status(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.RunningTask]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        try:
            service = rospy.ServiceProxy("/drobot_path_manager/path_record_status",common_service.srv.Int_resp)
            try:
                service.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = service(common_service.srv.Int_respRequest())
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        record_status={}
        record_status["status"]=resp.num
        request_handler.write(MasterNodeErrorCode.SuccessedData(record_status))
        return
    #delete path
    def delete_path(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        path_name = request_handler.get_query_arguments("path_name")
        if not path_name:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        else:
            path_name = path_name[0]
            try:
                delete_path_req = String_reqRequest()
                delete_path_req.str = path_name
                delete_path_srv = rospy.ServiceProxy('/drobot_path_manager/delete_path',String_req)
                try:
                    delete_path_srv.wait_for_service(3)
                except rospy.ROSException as servexc:
                    request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                    return
                resp = delete_path_srv(delete_path_req)
            except rospy.ServiceException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
                return
            if not resp.success:
                request_handler.write(MasterNodeErrorCode.GetReturn(resp))
                return
            request_handler.write(MasterNodeErrorCode.Successed())
        return
        pass

    def generate_path(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        try:
            add_path_req = String_reqRequest()
            add_path_req.str = request_handler.request.body
            add_path_srv = rospy.ServiceProxy('/drobot_path_manager/generate_path',String_req)
            try:
                add_path_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = add_path_srv(add_path_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
    def update_path(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        try:
            add_path_req = String_reqRequest()
            add_path_req.str = request_handler.request.body
            add_path_srv = rospy.ServiceProxy('/drobot_path_manager/update_path',String_req)
            try:
                add_path_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = add_path_srv(add_path_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
    def verify_path_line(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        try:
            check_line_req = String_reqRequest()
            check_line_req.str = request_handler.request.body
            check_line_srv = rospy.ServiceProxy('/drobot_path_manager/check_line',String_req)
            try:
                check_line_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = check_line_srv(check_line_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())

    def follow_task_progress(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        if not self.master_node_subscriber.check_reviced('follow_path_progress'):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NOT_RECIVED_MSG))
            return
        request_handler.write(MasterNodeErrorCode.SuccessedData(self.master_node_subscriber.submsg_table['follow_path_progress']))
        pass

    def get_path_list(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        try:
            get_path_req = console_service.GetPathsRequest()
            get_path_srv = rospy.ServiceProxy('/drobot_path_manager/get_path',console_service.GetPaths)
            try:
                get_path_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = get_path_srv(get_path_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        retPath={}
        retPath["paths"]=[]
        for path in resp.paths:
            pathJson={}
            pathJson["name"] = path.path_name
            pathJson["mapname"] = path.map_name
            pathJson["filename"] = path.path_file
            pathJson["pointCount"] = 0
            pathJson["createtime"] = ""
            retPath["paths"].append(pathJson)
            pass
        request_handler.write(MasterNodeErrorCode.SuccessedData(retPath))
        return
        pass

    #get path point
    def get_path(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        path_name = request_handler.get_query_argument('path_name')
        if not path_name:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        else:
            try:
                get_path_req = console_service.GetPathDataRequest()
                get_path_req.path_name = path_name
                get_path_srv = rospy.ServiceProxy('/drobot_path_manager/get_path_data',console_service.GetPathData)
                try:
                    get_path_srv.wait_for_service(3)
                except rospy.ROSException as servexc:
                    request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                    return
                resp = get_path_srv(get_path_req)
            except rospy.ServiceException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
                return
            if not resp.success:
                request_handler.write(MasterNodeErrorCode.GetReturn(resp))
                return
            if not self.master_node_subscriber.submsg_table['map_info']:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.NO_RUNNING_MAP))
                return
            else:
                map_info = {}
                map_info['gridWidth'] = self.master_node_subscriber.submsg_table['map_info'].gridWidth
                map_info['gridHeight'] = self.master_node_subscriber.submsg_table['map_info'].gridHeight
                originX = self.master_node_subscriber.submsg_table['map_info'].originX
                originY = self.master_node_subscriber.submsg_table['map_info'].originY
                resolution = self.master_node_subscriber.submsg_table['map_info'].resolution
                data = []
                rospy.loginfo("hello from get path map info")
                for pose_stamped in resp.path.poses:
                    point ={}
                    point['x'] = round((pose_stamped.pose.position.x - originX) / resolution, 2)
                    point['y'] = round((pose_stamped.pose.position.y - originY) / resolution, 2)
                    data.append(point)
                path = {}
                path['filename'] = path_name
                path['mapName'] = self.master_node_subscriber.submsg_table['map_info'].mapname
                path['name'] = path_name
                path['pointCount'] = len(data)
                return_msg={}
                return_msg['mapInfo'] = map_info
                return_msg['data'] = data
                return_msg['path'] = path
                request_handler.write(MasterNodeErrorCode.SuccessedData(return_msg))
                return
        pass
    def get_manual_path(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        try:
            get_path_req = String_respRequest()
            get_path_srv = rospy.ServiceProxy('/drobot_path_manager/manual_path_get',String_resp)
            try:
                get_path_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = get_path_srv()
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        path_json=json.loads(resp.str)
        request_handler.write(MasterNodeErrorCode.SuccessedData(path_json))
    def add_pose(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        if not len(request_handler.request.body) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        pose_msg = json.loads(request_handler.request.body)
        if not 'position' in pose_msg.keys() and not 'name' in pose_msg.keys() and not "type" in pose_msg.keys():
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
            return
        position = pose_msg['position']
        if ( not  'point' in position.keys()  ) or ( not 'angle' in position.keys() ):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
            return
        point = position['point']
        if (not 'x' in point.keys() )  or (not 'y' in point.keys() ):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
            return
        if " " in pose_msg['name']:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_ILLEGAL_ERROR))
            return
        try:
            add_pose_req = console_service.AddPoseRequest()
            add_pose_req.pose_name = pose_msg['name']
            add_pose_req.gridX = point["x"]
            add_pose_req.gridY = point["y"]
            add_pose_req.angle = position["angle"]
            add_pose_req.point_type.type = pose_msg['type']
            if not 'connect_id' in pose_msg.keys():
                add_pose_req.connect_id = ""
            else:
                add_pose_req.connect_id = pose_msg["connect_id"]
            rospy.loginfo("[Master_node.py ] add connect id %s", add_pose_req.connect_id)
            edit_pose_srv = rospy.ServiceProxy("/drobot_pose_manager/add_pose",console_service.AddPose)
            try:
                edit_pose_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = edit_pose_srv(add_pose_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        else:
            request_handler.write(MasterNodeErrorCode.Successed())
            return
        pass
        request_handler.write(MasterNodeErrorCode.Successed())
        pass
    def add_cur_pose(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        pose_name = request_handler.get_query_argument('pose_name','')
        if " " in pose_name:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_ILLEGAL_ERROR))
            return
        pose_type = request_handler.get_query_argument('pose_type',2)
        connect_id   = request_handler.get_query_argument('connect_id', '')
        result_msg = {}
        if not pose_name:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        else:
            try:
                add_pose_req = console_service.AddPoseRequest()
                add_pose_req.pose_name = pose_name
                if not pose_type:
                    add_pose_req.point_type.type = 2
                else :
                    add_pose_req.point_type.type = int(pose_type)
                add_pose_req.connect_id = connect_id
                add_pose_srv = rospy.ServiceProxy("/drobot_pose_manager/add_cur_pose",console_service.AddPose)
                try:
                    add_pose_srv.wait_for_service(3)
                except rospy.ROSException as servexc:
                    request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                    return
                resp = add_pose_srv(add_pose_req)
            except rospy.ServiceException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
                return
            if not resp.success:
                request_handler.write(MasterNodeErrorCode.GetReturn(resp))
                return
            request_handler.write(MasterNodeErrorCode.Successed())
        return
        pass

#******接口需要变动*****
    def get_pose(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        rospy.loginfo("get pose ")
        pose_name = request_handler.get_query_argument('pose_name','')
        pose_infos = {}
        try:
            get_pose_req = console_service.GetPosesRequest()
            get_pose_req.pose_name = pose_name
            get_pose_srv = rospy.ServiceProxy("/drobot_pose_manager/get_pose",console_service.GetPoses)
            rospy.loginfo("hello from get pose2")
            try:
                get_pose_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = get_pose_srv(get_pose_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        rospy.loginfo("hello from get pose3")
        pose_infos=MasterNodeErrorCode.GetReturn(resp)
        if not resp.success:
            pose_infos['PoseList'] = []
            request_handler.write(pose_infos)
        else:
            pose_infos["PoseList"] =[]
            map_info = {}
            map_info['gridWidth'] = self.master_node_subscriber.submsg_table['map_info'].gridWidth
            map_info['gridHeight'] = self.master_node_subscriber.submsg_table['map_info'].gridHeight
            originX = self.master_node_subscriber.submsg_table['map_info'].originX
            originY = self.master_node_subscriber.submsg_table['map_info'].originY
            resolution = self.master_node_subscriber.submsg_table['map_info'].resolution
            pose_infos["mapInfo"]=map_info
            for pose_info in resp.poses:
                point={}
                point['name'] = pose_info.pose_name
                point['poseId'] = pose_info.id
                point['connect_id'] = pose_info.connect_id
                point["type"] = pose_info.point_type.type
                point['gridX'] = int(round((pose_info.pose.position.x - originX) / resolution))
                point['gridY'] = int(round((pose_info.pose.position.y - originY) / resolution))
                angle = euler_from_quaternion([pose_info.pose.orientation.x, pose_info.pose.orientation.y, pose_info.pose.orientation.z, pose_info.pose.orientation.w])
                point['angle'] = angle[2]
                point['world'] = {}
                point['world']['orientation'] = {}
                point['world']['orientation']['x'] = pose_info.pose.orientation.x
                point['world']['orientation']['y'] = pose_info.pose.orientation.y
                point['world']['orientation']['z'] = pose_info.pose.orientation.z
                point['world']['orientation']['w'] = pose_info.pose.orientation.w
                point['world']['position'] = {}
                point['world']['position']['x'] = pose_info.pose.position.x
                point['world']['position']['y'] = pose_info.pose.position.y
                point['world']['position']['z'] = pose_info.pose.position.z
                point['image'] = {}
                point['image']['name'] = pose_info.bindImage.image_name
                point['image']['height'] = pose_info.bindImage.height
                point['image']['width'] = pose_info.bindImage.width
                pose_infos['PoseList'].append(point)
            pose_infos["successed"] = True
            data_json = {}
            data_json["PoseList"] = pose_infos['PoseList']
            data_json["mapInfo"] = map_info
            pose_infos["data"] = data_json
            request_handler.write(pose_infos)
        return

    def get_pose_from_map(self, request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        map_name = request_handler.get_query_argument('map_id','')
        try:
            get_pose_from_map_req = common_service.srv.String_req_respRequest()
            get_pose_from_map_req.str_req = map_name
            get_pose_from_map_srv = rospy.ServiceProxy("/drobot_pose_manager/get_pose_from_map", common_service.srv.String_req_resp)
            try:
                get_pose_from_map_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = get_pose_from_map_srv(get_pose_from_map_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        pose_infos = MasterNodeErrorCode.GetReturn(resp)
        pose_info_array_json = json.loads(resp.str_resp)
        if not resp.success:
            pose_infos['PoseList'] = []
            request_handler.write(pose_infos)
        else:
            pose_infos["PoseList"] =[]
            for pose_info_json in pose_info_array_json["poses"]:
                point={}
                point['name'] = pose_info_json["name"]
                point['poseId'] = pose_info_json["Id"]
                if not 'connect_id' in pose_info_json.keys():
                    point['connect_id'] = ""
                else:
                    point['connect_id'] = pose_info_json["connect_id"]
                point["type"] = pose_info_json["type"]
                point['gridX'] = pose_info_json["gridPose"]["gridX"]
                point['gridY'] = pose_info_json["gridPose"]["gridY"]
                point['angle'] = pose_info_json["gridPose"]["angle"]
                point['world'] = {}
                point['world']['orientation'] = {}
                point['world']['orientation']['x'] = pose_info_json["worldPose"]["orientation"]["x"]
                point['world']['orientation']['y'] = pose_info_json["worldPose"]["orientation"]["y"]
                point['world']['orientation']['z'] = pose_info_json["worldPose"]["orientation"]["z"]
                point['world']['orientation']['w'] = pose_info_json["worldPose"]["orientation"]["w"]
                point['world']['position'] = {}
                point['world']['position']['x'] = pose_info_json["worldPose"]["position"]["x"]
                point['world']['position']['y'] = pose_info_json["worldPose"]["position"]["y"]
                point['world']['position']['z'] = pose_info_json["worldPose"]["position"]["z"]
                point['image'] = {}
                point['image']['name'] = pose_info_json["image"]["name"]
                point['image']['height'] = pose_info_json["image"]["height"]
                point['image']['width'] = pose_info_json["image"]["width"]
                pose_infos['PoseList'].append(point)
            pose_infos["successed"] = True
            data_json = {}
            data_json["PoseList"] = pose_infos['PoseList']
            request_handler.write(pose_infos)
        return

    def edit_pose(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        pose_name = request_handler.get_query_argument('pose_name','')
        if not pose_name:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        if " " in pose_name:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_ILLEGAL_ERROR))
            return
        if not len(request_handler.request.body) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        pose_msg = json.loads(request_handler.request.body)
        if not 'position' in pose_msg.keys():
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
            return
        position = pose_msg['position']
        if ( not  'point' in position.keys()  ) or ( not 'angle' in position.keys() ):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
            return
        point = position['point']
        if (not 'x' in point.keys() )  or (not 'y' in point.keys() ):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
            return
        try:
            add_pose_req = console_service.AddPoseRequest()
            add_pose_req.pose_name = pose_name
            add_pose_req.gridX = point["x"]
            add_pose_req.gridY = point["y"]
            add_pose_req.angle = position["angle"]
            edit_pose_srv = rospy.ServiceProxy("/drobot_pose_manager/edit_pose",console_service.AddPose)
            try:
                edit_pose_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = edit_pose_srv(add_pose_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())

    def delete_pose(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        pose_name = request_handler.get_query_argument('pose_name')
        if not pose_name:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        else:
            try:
                delete_pose_req = String_reqRequest()
                delete_pose_req.str = pose_name
                delete_pose_srv = rospy.ServiceProxy("/drobot_pose_manager/delete_pose",String_req)
                try:
                    delete_pose_srv.wait_for_service(3)
                except rospy.ROSException as servexc:
                    request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                    return
                resp = delete_pose_srv(delete_pose_req)
            except rospy.ServiceException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
                return
            if not resp.success:
                request_handler.write(MasterNodeErrorCode.GetReturn(resp))
                return
            else:
                request_handler.write(MasterNodeErrorCode.Successed())
                return
        pass


# 通过传入点的信息初始化
    def initialize_customized(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        pos_name = request_handler.get_query_argument('point_name','')
        turning = request_handler.get_query_argument('turning','false')
        init_pose_srv = rospy.ServiceProxy("/drobot_pose_manager/initialize_pose", console_service.InitialPose)
        service_request_msg = console_service.InitialPoseRequest()
        if turning == 'true':
            rospy.loginfo("[Master_node.py] Turning is True")
            turning = True
        else:
            rospy.loginfo("[Master_node.py] Turning is False")
            turning = False
        if not pos_name:
            if not len(request_handler.request.body) > 0:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
                return
            goal_pose_msg = json.loads(request_handler.request.body)
            if not 'point' in goal_pose_msg.keys():
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
                return
            point = goal_pose_msg['point']
            if ( not  'position' in point.keys()  ) or ( not 'angle' in point.keys() ):
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
                return
            position = point['position']
            if (not 'x' in position.keys() )  or (not 'y' in position.keys() ):
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
                return
            originX = self.master_node_subscriber.submsg_table['map_info'].originX
            originY = self.master_node_subscriber.submsg_table['map_info'].originY
            resolution = self.master_node_subscriber.submsg_table['map_info'].resolution
            pose = helper.Coordinates.GridToPose(helper.Point(position['x'],position['y'],point['angle']),\
                                                 originX,originY,resolution)
            service_request_msg = console_service.InitialPoseRequest('empty',turning,True,pose)
        else:
            temp_p = Pose(Point(0,0,0),Quaternion(*quaternion_from_euler(0,0,0)))
            service_request_msg = console_service.InitialPoseRequest(pos_name,turning,False,temp_p)
        try:
            resp = init_pose_srv.call(service_request_msg)
        except rospy.ServiceException, e:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        if turning:
            #创建旋转进程
            self.stop_move_flag = False
            thread.start_new_thread(self.rotate,(360,0.3))
            # thread.start_new_thread(self.check_initialze,())
            request_handler.write(MasterNodeErrorCode.Successed())
            return
            pass
        self.master_node_state.state_initialized = True
        if self.master_node_state.state_initialized :
            rospy.loginfo("[Master_node.py] state_initialized: True")
        else :
            rospy.loginfo("[Master_node.py] state_initialized: False")
        request_handler.write(MasterNodeErrorCode.Successed())
        return
        pass

    def bind_image(self,request_handler):
        pose_name = request_handler.get_query_argument('point_name')
        image_name = request_handler.get_query_argument('image_name',"")
        try:
            service = rospy.ServiceProxy("/drobot_pose_manager/pose_image_bind",console_service.BindPoseImage)
            try:
                service.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = service(pose_name,image_name)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        else:
            request_handler.write(MasterNodeErrorCode.Successed())
            return
        pass
    def save_task_queue(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        if len(request_handler.request.body) < 0:
            request_handler.write(MasterNodeErrorCode.FailedData(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        try:
            task_json = request_handler.request.body
            add_task_req = String_reqRequest()
            add_task_req.str = task_json
            new_add_task_srv = rospy.ServiceProxy("/drobot_task_manager/new_add_task",String_req)
            resp = new_add_task_srv(add_task_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        else:
            # 保存成功后，检查任务中的机械臂动作（不影响响应）
            print("=== TASK SAVED SUCCESSFULLY ===")
            mechanical_arm_count = 0
            
            try:
                import json
                task_data = json.loads(task_json)
                
                if 'tasks' in task_data:
                    print("Total tasks in queue: %d" % len(task_data['tasks']))
                    for task_index, task in enumerate(task_data['tasks']):
                        task_name = task.get('name', 'Unknown')
                        task_type = task.get('type', 'Unknown')
                        print("Task[%d] name: %s, type: %s" % (task_index, task_name, task_type))
                        
                        if 'actions' in task:
                            print("Task[%d] has %d actions" % (task_index, len(task['actions'])))
                            for action_index, action in enumerate(task['actions']):
                                # 打印每个动作的type，方便调试
                                action_type = action.get('type')
                                print("Task[%d]Action[%d] type: %s" % (task_index, action_index, action_type))
                                
                                if action_type == 'MECHANICAL_ARM':
                                    mechanical_arm_count += 1
                                    print("*** FOUND MECHANICAL ARM ACTION ***")
                                    print("Task[%d]Action[%d] is MECHANICAL_ARM" % (task_index, action_index))
                                    print("Task name: %s" % task_name)
                                    print("Task type: %s" % task_type)
                                    print("Action type: %s" % action.get('type'))
                                    print("Point ID: %s" % action.get('param', {}).get('pointId'))
                                    print("Action time: %s" % action.get('actionTime'))
                                    print("*** MECHANICAL ARM ACTION CHECK COMPLETE ***")
                
            except Exception as e:
                # 安全地处理异常信息，避免编码问题
                print("Error checking mechanical arm actions: %s" % str(e))
            
            # 无论是否有异常，都显示最终结果
            if mechanical_arm_count > 0:
                print("=== TASK SAVED WITH %d MECHANICAL ARM ACTIONS ===" % mechanical_arm_count)
            else:
                print("=== TASK SAVED WITH NO MECHANICAL ARM ACTIONS ===")
            
            request_handler.write(MasterNodeErrorCode.Successed())
            return
        pass
    def get_task_queue(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        task_name = request_handler.get_query_argument('task_name',"")
        try:
            get_task_req = console_service.GetTasksRequest()
            get_task_req.task_name = task_name
            get_task_srv = rospy.ServiceProxy("/drobot_task_manager/get_task",console_service.GetTasks)
            resp = get_task_srv(get_task_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        else:
            tasksJson = json.loads(resp.tasks)
            request_handler.write(MasterNodeErrorCode.SuccessedData(tasksJson))
            return
        pass

    def update_task_queue(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        task_name = request_handler.get_query_argument('task_name',"")
        if not task_name:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        if not len(request_handler.request.body) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        try:
            task_json = request_handler.request.body
            update_task_req = String_reqRequest()
            update_task_req.str = task_json
            get_task_srv = rospy.ServiceProxy("/drobot_task_manager/modify_task",String_req)
            resp = get_task_srv(update_task_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        else:
            request_handler.write(MasterNodeErrorCode.Successed())
            return
        pass

    def delete_task_queue(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        task_name = request_handler.get_query_argument('task_name')
        if not task_name:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        else:
            try:
                delete_task_req = String_reqRequest()
                delete_task_req.str = task_name
                delete_task_srv = rospy.ServiceProxy("/drobot_task_manager/delete_task",String_req)
                try:
                    delete_task_srv.wait_for_service(3)
                except rospy.ROSException as servexc:
                    request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                    return
                resp = delete_task_srv(delete_task_req)
            except rospy.ServiceException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
                return
            if not resp.success:
                request_handler.write(MasterNodeErrorCode.GetReturn(resp))
                return
            else:
                request_handler.write(MasterNodeErrorCode.Successed())
                return
        pass

    def start_task_queue(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.RunningTask]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        if not self.master_node_subscriber.submsg_table['localization_status']['initialized']:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.WAITING_INITIALIZED))
            return
        if len(request_handler.request.body) < 0:
            request_handler.write(MasterNodeErrorCode.FailedData(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        try:
            task_json = json.loads(request_handler.request.body)
            loop = False
            task_name = task_json['name']
            loop_time = task_json['loop_time']
            loop = task_json['loop']
            if(loop_time > 1):
                loop = True
            start_task_req = console_service.StartTaskRequest()
            start_task_req.task_name = task_name
            start_task_req.loop = loop
            start_task_req.looptime = loop_time
            start_task_srv = rospy.ServiceProxy("/drobot_task_manager/start_task",console_service.StartTask)
            resp = start_task_srv(start_task_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        else:
            self.master_node_subscriber.submsg_recived_table['path'] = False
            self.master_node_subscriber.submsg_table['path'] = {}
            request_handler.write(MasterNodeErrorCode.Successed())
            self.start_navigation_flag_ = True
            return
        pass

    def stop_task_queue(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.RunningTask]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        result_msg ={}
        try:
            cancel_task_req = EmptyRequest()
            cancel_task_srv = rospy.ServiceProxy("/drobot_task_manager/stop_task",common_service.srv.Empty)
            try:
                cancel_task_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = cancel_task_srv(cancel_task_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        else:
            request_handler.write(MasterNodeErrorCode.Successed())
            self.start_navigation_flag_ = False
            return
        pass

    def pause_task_queue(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.RunningTask]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        result_msg = {}
        try:
            pause_cur_task_req = EmptyRequest()
            pause_cur_task_srv = rospy.ServiceProxy("drobot_task_manager/pause_task",common_service.srv.Empty)
            try:
                pause_cur_task_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = pause_cur_task_srv(pause_cur_task_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        else:
            request_handler.write(MasterNodeErrorCode.Successed())
            return
        pass

    def resume_task_queue(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.RunningTask]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        result_msg = {}
        try:
            resume_cur_task_req = EmptyRequest()
            resume_cur_task_srv = rospy.ServiceProxy("/drobot_task_manager/resume_task",common_service.srv.Empty)
            try:
                resume_cur_task_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = resume_cur_task_srv(resume_cur_task_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        else:
            request_handler.write(MasterNodeErrorCode.Successed())
            return
        return
        pass

    def is_task_queue_finished(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.RunningTask]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        try:
            task_status_req = console_service.GetTaskStatusRequest()
            get_task_status_srv = rospy.ServiceProxy("/drobot_task_manager/get_task_status",console_service.GetTaskStatus)
            resp = get_task_status_srv(task_status_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        else:
            finished = False
            if resp.status == 0:
                finished = True
            request_handler.write(MasterNodeErrorCode.SuccessedData(finished))
            return
        return

    def get_task_status(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.RunningTask]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        try:
            task_status_req = console_service.GetTaskStatusRequest()
            get_task_status_srv = rospy.ServiceProxy("/drobot_task_manager/get_task_status",console_service.GetTaskStatus)
            resp = get_task_status_srv(task_status_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        task_status = {}
        task_status["statusCode"]  = resp.status
        task_status["statusMessage"]  = resp.message
        task_status["remainingLoopTime"]  = resp.remaining_loop_time
        current_task = json.loads(resp.current_task)
        task_status["currentTask"] = current_task
        tasks=json.loads(resp.task)
        task_status["task"] = tasks
        if resp.status == 0:
            task_status["finished"] = True
        else :
            task_status["finished"] = False

        if self.master_node_subscriber.submsg_recived_table['chassis_state'] == False:
            self.master_node_subscriber.submsg_table['sellButtonFlag'] = 0
        task_status["sellButtonFlag"] = self.master_node_subscriber.submsg_table['sellButtonFlag']
        request_handler.write(MasterNodeErrorCode.SuccessedData(task_status))
        return


    def get_autocharge_status(self,request_handler):
        # if not self.master_node_state.checkState([MasterNodeState.state_type.RunningTask]):
        #     request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
        #     return
        try:
            get_autochrage_req = console_service.GetChargeTaskRequest()
            get_autochrage_srv = rospy.ServiceProxy('/drobot_task_manager/get_charge_task',console_service.GetChargeTask)
            try:
                get_autochrage_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = get_autochrage_srv(get_autochrage_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        autotask_json = {}
        autotask_json['AutoChargeSwitch'] = resp.sw
        autotask_json['ChargeState'] = resp.state
        request_handler.write(MasterNodeErrorCode.SuccessedData(autotask_json))
        pass
    def set_autocharge_status(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.RunningTask]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        if not len(request_handler.request.body) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        params = json.loads(request_handler.request.body)
        AutoChargeSwitch = params["AutoChargeSwitch"]
        try:
            set_charge_req = console_service.SetChargeTaskRequest()
            set_charge_req.sw = AutoChargeSwitch
            set_charge_srv = rospy.ServiceProxy('/drobot_task_manager/set_charge_task',console_service.SetChargeTask)
            try:
                set_charge_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = set_charge_srv(set_charge_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        pass

    def get_task_report(self,request_handler):
        start_time = request_handler.get_query_argument('start_time',0)
        end_time = request_handler.get_query_argument('end_time',0)
        current_page = request_handler.get_query_argument('current_page',1) #设置日期
        current_page = int(current_page)
        page_size = request_handler.get_query_argument('page_size',30) #设置日期
        page_size = int(page_size)
        try:
            req = console_service.GetTaskReportRequest()
            req.start_time = int(start_time)
            req.end_time = int(end_time)
            req.current_page = current_page
            req.page_size = page_size
            srv = rospy.ServiceProxy('/drobot_task_manager/get_task_report',console_service.GetTaskReport)
            try:
                srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = srv.call(req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        task_json=json.loads(resp.reports_json)
        request_handler.write(MasterNodeErrorCode.SuccessedData(task_json))
        return
    def get_report_image(self,request_handler):
        task_id = request_handler.get_query_argument('id',"") #设置日期
        if task_id == "":
          request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_BODY_IS_EMPTY))
          return
        image_file_path = self.run_time_dir + "/task_log/snapshot/" + task_id+ ".png"
        try:
          with open(image_file_path,'r') as pic:
              pics = pic.read()
          request_handler.write(pics)
          request_handler.set_header("Access-Control-Allow-Origin", "*")
          request_handler.set_header("Access-Control-Allow-Methods", "PUT, POST, GET, OPTIONS, DELETE")
          request_handler.set_header("Content-type","image/png")
        except IOError:
            raise HTTPError(404)
            return
        return
    def generate_pdf_report(self,request_handler):
        task_id = request_handler.get_query_argument('id',"") #设置日期
        if task_id == "":
          request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_BODY_IS_EMPTY))
          return
        try:
            req = String_reqRequest()
            req.str = task_id
            srv = rospy.ServiceProxy('/drobot_task_manager/pdf_generate_report', String_req)
            try:
                srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = srv.call(req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        return
    def download_report(self,request_handler):
        task_id = request_handler.get_query_argument('task_id',"") #设置日期
        if task_id == "":
           request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_BODY_IS_EMPTY))
           return
        path = self.run_time_dir + "/task_log/pdf/"
        report_name = task_id+ "*.pdf"
        pdf_files = glob.glob(os.path.join(path,report_name))
        try:
            tar=tarfile.open(task_id + '.tar.gz','w')
            for file in pdf_files:
                filename = os.path.basename(file)
                tar.add(file,arcname=filename)
            with open(task_id + '.tar.gz','r') as file:
                data = file.read()
                request_handler.write(data)
            tar.close
            os.remove(task_id + '.tar.gz')
        except IOError:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.OPEN_FILE_FAILED))
            return
        return


    def add_track_graph(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        if not len(request_handler.request.body) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        try:

            add_track_req = String_reqRequest()
            add_track_req.str = request_handler.request.body
            add_track_srv = rospy.ServiceProxy('/drobot_track_manager/add_graph',String_req)
            try:
                add_track_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = add_track_srv(add_track_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        self.graph_pub_.publish()
        request_handler.write(MasterNodeErrorCode.Successed())

    def get_track_graph(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        try:
            filled_graph = request_handler.get_query_argument('filled', "")
            get_track_req = String_respRequest()
            get_track_srv = None
            if filled_graph == "true":
                get_track_srv = rospy.ServiceProxy('/drobot_track_manager/get_filled_graph',String_resp)
            else:
                get_track_srv = rospy.ServiceProxy('/drobot_track_manager/get_graph',String_resp)
            try:
                get_track_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = get_track_srv()
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        graph_json=json.loads(resp.str)
        request_handler.write(MasterNodeErrorCode.SuccessedData(graph_json))

    def restore_default_track(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        try:
            restore_default_track = String_respRequest()
            restore_default_srv = rospy.ServiceProxy('/drobot_track_manager/restore_default_track',String_resp)
            try:
                restore_default_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = restore_default_srv()
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        request_handler.write(MasterNodeErrorCode.GetReturn(resp))
        return


    def get_image(self,request_handler):
        image_name = request_handler.get_query_argument('image_name')
        try:
            download_image_srv = rospy.ServiceProxy("/drobot_image_manager/get_image", console_service.GetImage)
            resp  = download_image_srv(image_name)
        except rospy.ServiceException, e:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        else:
            request_handler.write(resp.image_file)
            request_handler.set_header("Access-Control-Allow-Origin", "*")
            request_handler.set_header("Access-Control-Allow-Methods", "PUT, POST, GET, OPTIONS, DELETE")
            request_handler.set_header("Content-type","image/png")
            return
    def get_image_lists(self,request_handler):
        image_name = request_handler.get_query_argument('image_name',"")
        try:
            image_lists_srv = rospy.ServiceProxy("/drobot_image_manager/get_image_lists", console_service.GetImageList)
            resp  = image_lists_srv(image_name)
        except rospy.ServiceException, e:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        images = {}
        images['images'] = []
        for imagemsg in resp.imageLists :
            item ={}
            item["name"] = imagemsg.image_name
            item["height"] = imagemsg.height
            item["width"] = imagemsg.width
            images['images'].append(item)
        request_handler.write(MasterNodeErrorCode.SuccessedData(images))
    def delete_image(self,request_handler):
        image_name = request_handler.get_query_argument('image_name')
        try:
            service = rospy.ServiceProxy("/drobot_image_manager/delete_image", String_req)
            resp  = service(image_name)
        except rospy.ServiceException, e:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
    
    def delete_images(self,request_handler):
        # 获取要删除的图片ID列表
        try:
            import json
            import os
            # 从POST请求的body中获取数据
            request_data = json.loads(request_handler.request.body)
            images_data = request_data.get('images', [])
            
            if not images_data:
                print("[DELETE_IMAGES] 图片数据列表为空")
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAMETER_ERROR))
                return
            
            # 构建图片存储基础路径
            base_path = "/root/Drobot_Navigation_Module/WORKSPACE/src/application/master_node"
            
            # 逐个删除图片
            success_count = 0
            failed_count = 0
            
            for image_data in images_data:
                try:
                    image_id = image_data.get('id', '')
                    image_type = image_data.get('type', '')
                    
                    print("[DELETE_IMAGES] 开始删除图片ID: {} (类型: {})".format(image_id, image_type))
                    # 直接删除文件系统中的图片文件
                    file_deleted = self._delete_image_file_by_type(base_path, image_id, image_type)
                    
                    if file_deleted:
                        success_count += 1
                        print("[DELETE_IMAGES] 成功删除文件: {} (类型: {})".format(image_id, image_type))
                    else:
                        failed_count += 1
                        print("[DELETE_IMAGES] 文件删除失败: {} (类型: {})".format(image_id, image_type))
                        
                except Exception as e:
                    failed_count += 1
                    print("[DELETE_IMAGES] 删除文件异常: {}".format(str(e)))
                    continue
            
            # 返回删除结果
            result = {
                "success_count": success_count,
                "failed_count": failed_count,
                "total_count": len(images_data)
            }
            request_handler.write(MasterNodeErrorCode.SuccessedData(result))
            
        except Exception as e:
            print("[DELETE_IMAGES] 删除图片总体异常: {}".format(str(e)))
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAMETER_ERROR))
            return
    
    def _delete_image_file(self, base_path, image_id):
        """
        删除文件系统中的图片文件
        支持删除raw和result文件夹中的同名文件
        """
        try:
            import os
            import glob
            
            if not os.path.exists(base_path):
                print("[_DELETE_IMAGE_FILE] 基础路径不存在: {}".format(base_path))
                return False
            
            # 在所有可能的路径中查找并删除图片文件
            deleted_files = []
            total_found = 0
            
            print("[_DELETE_IMAGE_FILE] 开始查找图片ID: {}".format(image_id))
            
            # 遍历所有地图和点位查找图片
            for map_dir in os.listdir(base_path):
                map_path = os.path.join(base_path, map_dir)
                if not os.path.isdir(map_path):
                    continue
                
                for pose_dir in os.listdir(map_path):
                    pose_path = os.path.join(map_path, pose_dir)
                    if not os.path.isdir(pose_path):
                        continue
                    
                    # 遍历type文件夹 (raw, result)
                    for type_dir in os.listdir(pose_path):
                        type_path = os.path.join(pose_path, type_dir)
                        if not os.path.isdir(type_path):
                            continue
                        
                        # 遍历日期文件夹
                        for date_dir in os.listdir(type_path):
                            date_path = os.path.join(type_path, date_dir)
                            if not os.path.isdir(date_path):
                                continue
                            
                            # 查找匹配的图片文件
                            # 支持多种文件格式
                            for ext in ['*.jpg', '*.jpeg', '*.png', '*.bmp']:
                                pattern = os.path.join(date_path, ext)
                                for file_path in glob.glob(pattern):
                                    filename = os.path.basename(file_path)
                                    # 更精确的匹配：检查文件名是否包含图片ID
                                    # 支持多种命名格式：image_id_123.jpg, 123_image.jpg, image_123_result.jpg 等
                                    if (image_id in filename or 
                                        filename.startswith(image_id + '_') or 
                                        filename.startswith(image_id + '.') or
                                        '_' + image_id + '_' in filename or
                                        '_' + image_id + '.' in filename):
                                        total_found += 1
                                        try:
                                            os.remove(file_path)
                                            deleted_files.append(file_path)
                                            print("[_DELETE_IMAGE_FILE] 删除文件: {} (类型: {})".format(file_path, type_dir))
                                        except OSError as e:
                                            print("[_DELETE_IMAGE_FILE] 删除文件失败: {} - {}".format(file_path, str(e)))
            
            print("[_DELETE_IMAGE_FILE] 图片ID: {} - 找到文件: {}, 成功删除: {}".format(image_id, total_found, len(deleted_files)))
            return len(deleted_files) > 0
            
        except Exception as e:
            print("[_DELETE_IMAGE_FILE] 删除图片文件异常: {}".format(str(e)))
            return False
    
    def _delete_image_file_by_type(self, base_path, image_id, image_type):
        """
        根据图片类型删除文件系统中的图片文件
        """
        try:
            import os
            import glob
            
            if not os.path.exists(base_path):
                print("[_DELETE_IMAGE_FILE_BY_TYPE] 基础路径不存在: {}".format(base_path))
                return False
            
            deleted_files = []
            total_found = 0
            
            print("[_DELETE_IMAGE_FILE_BY_TYPE] 开始查找图片ID: {} (类型: {})".format(image_id, image_type))
            
            # 遍历所有地图和点位查找图片
            for map_dir in os.listdir(base_path):
                map_path = os.path.join(base_path, map_dir)
                if not os.path.isdir(map_path):
                    continue
                
                for pose_dir in os.listdir(map_path):
                    pose_path = os.path.join(map_path, pose_dir)
                    if not os.path.isdir(pose_path):
                        continue
                    
                    # 如果指定了图片类型，只在该类型文件夹中查找
                    if image_type:
                        type_folders = [image_type]
                    else:
                        # 如果没有指定类型，遍历所有类型文件夹
                        type_folders = ['raw', 'result', 'rc']
                    
                    for type_dir in type_folders:
                        type_path = os.path.join(pose_path, type_dir)
                        if not os.path.isdir(type_path):
                            continue
                        
                        # 遍历日期文件夹
                        for date_dir in os.listdir(type_path):
                            date_path = os.path.join(type_path, date_dir)
                            if not os.path.isdir(date_path):
                                continue
                            
                            # 查找匹配的图片文件
                            for ext in ['*.jpg', '*.jpeg', '*.png', '*.bmp']:
                                pattern = os.path.join(date_path, ext)
                                for file_path in glob.glob(pattern):
                                    filename = os.path.basename(file_path)
                                    file_without_ext = os.path.splitext(filename)[0]
                                    
                                    # 检查文件名是否精确匹配图片ID
                                    # 支持带扩展名和不带扩展名的匹配
                                    image_id_without_ext = os.path.splitext(image_id)[0]
                                    
                                    if (file_without_ext == image_id_without_ext or 
                                        file_without_ext == image_id or
                                        file_without_ext.startswith(image_id_without_ext + '_') or 
                                        file_without_ext.endswith('_' + image_id_without_ext) or
                                        '_' + image_id_without_ext + '_' in file_without_ext):
                                        total_found += 1
                                        try:
                                            os.remove(file_path)
                                            deleted_files.append(file_path)
                                            print("[_DELETE_IMAGE_FILE_BY_TYPE] 删除文件: {} (类型: {})".format(file_path, type_dir))
                                        except OSError as e:
                                            print("[_DELETE_IMAGE_FILE_BY_TYPE] 删除文件失败: {} - {}".format(file_path, str(e)))
            
            print("[_DELETE_IMAGE_FILE_BY_TYPE] 图片ID: {} - 找到文件: {}, 成功删除: {}".format(image_id, total_found, len(deleted_files)))
            return len(deleted_files) > 0
            
        except Exception as e:
            print("[_DELETE_IMAGE_FILE_BY_TYPE] 删除图片文件异常: {}".format(str(e)))
            return False
    
    def upload_image(self,request_handler):
        if not len(request_handler.request.files) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        if len(request_handler.request.files.get("file")) == 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        file = request_handler.request.files.get("file")[0]
        filename = file.get("filename")
        filebody = file.get("body")
        if filename == "" or filebody=="":
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        try:
            service = rospy.ServiceProxy("/drobot_image_manager/upload_image", console_service.UploadImage)
            resp  = service(filename,filebody)
        except rospy.ServiceException, e:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        return

    def query_images(self, request_handler):
        """
        查询图片接口 - 支持按日期、地图ID、点位ID查询图片
        """
        try:
            # 解析请求参数
            if not len(request_handler.request.body) > 0:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
                return
            
            params = json.loads(request_handler.request.body)
            print("[QUERY_IMAGES] 原始请求参数: {}".format(params))
            
            start_date = params.get('startDate', '')
            end_date = params.get('endDate', '')
            map_id = params.get('mapId', '')
            pose_id = params.get('poseId', '')
            image_type = params.get('imageType', '')
            
            print("[QUERY_IMAGES] 解析后的参数:")
            print("  startDate: '{}'".format(start_date))
            print("  endDate: '{}'".format(end_date))
            print("  mapId: '{}'".format(map_id))
            print("  poseId: '{}'".format(pose_id))
            print("  imageType: '{}'".format(image_type))
            print("[QUERY_IMAGES] 图片类型过滤条件: '{}'".format(image_type))
            print("[QUERY_IMAGES] 图片类型过滤条件类型: {}, 长度: {}".format(type(image_type), len(str(image_type))))
            print("[QUERY_IMAGES] 图片类型过滤条件repr: {}".format(repr(image_type)))
            
            # 验证必须参数：如果选择了点位或图片类型，必须指定地图ID
            if (pose_id or image_type) and not map_id:
                print("[QUERY_IMAGES] 错误：选择点位或图片类型时必须指定地图ID")
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
                return
            
            # 构建图片存储基础路径 - 使用运行时目录
            base_path ="/root/Drobot_Navigation_Module/WORKSPACE/src/application/master_node"
            # 必须指定地图ID才能查询
            if not map_id:
                print("[QUERY_IMAGES] 错误：必须指定地图ID才能查询图片")
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
                return
            
            # 查询指定地图的图片
            map_path = os.path.join(base_path, map_id)
            if not os.path.exists(map_path):
                print("[QUERY_IMAGES] 错误：地图路径不存在: {}".format(map_path))
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
                return
            
            images = []
            
            # 如果指定了点位ID，查询该点位下的图片
            if pose_id:
                pose_path = os.path.join(map_path, pose_id)
                if os.path.exists(pose_path):
                    images.extend(self._scan_images_in_path(pose_path, start_date, end_date, image_type, map_id, pose_id))
                else:
                    print("[QUERY_IMAGES] 错误：点位路径不存在: {}".format(pose_path))
                    request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
                    return
            else:
                # 扫描该地图下所有点位的图片
                for pose_dir in os.listdir(map_path):
                    pose_path = os.path.join(map_path, pose_dir)
                    if os.path.isdir(pose_path):
                        images.extend(self._scan_images_in_path(pose_path, start_date, end_date, image_type, map_id, pose_dir))
            
            # 去重：基于图片完整路径去重（因为不同目录下可能有同名文件）
            unique_images = []
            seen_paths = set()
            for image in images:
                # 使用完整路径作为唯一标识
                image_path = image.get('path', '')
                if image_path not in seen_paths:
                    seen_paths.add(image_path)
                    unique_images.append(image)
            
            # 按创建时间排序
            unique_images.sort(key=lambda x: x.get('createTime', ''), reverse=True)
            
            result = {
                'images': unique_images,
                'total': len(unique_images)
            }
            
            print("[QUERY_IMAGES] 原始找到 {} 张图片，去重后 {} 张图片".format(len(images), len(unique_images)))
            
            # 显示每张图片的详细信息
            for i, img in enumerate(unique_images):
                print("[QUERY_IMAGES] 图片 {}: 名称={}, 类型={}, 路径={}".format(i+1, img.get('name'), img.get('type'), img.get('path')))
            request_handler.write(MasterNodeErrorCode.SuccessedData(result))
            
        except Exception as e:
            print("[QUERY_IMAGES] 查询图片时发生错误: {}".format(str(e)))
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SYSTEM_ERROR))
    
    def _scan_images_in_path(self, path, start_date, end_date, image_type='', map_id='', pose_id=''):
        """
        扫描指定路径下的图片文件
        路径结构: .../mapid/poseid/result|raw/YYYY-MM-DD/
        """
        images = []
        
        try:
            print("[_SCAN_IMAGES] 开始扫描路径: {}".format(path))
            print("[_SCAN_IMAGES] 路径是否存在: {}".format(os.path.exists(path)))
            if os.path.exists(path):
                print("[_SCAN_IMAGES] 路径下的所有目录: {}".format(os.listdir(path)))
            else:
                print("[_SCAN_IMAGES] 路径不存在，无法扫描")
                return images
            
            # 检查路径下是否有类型目录（raw, result等）
            type_dirs = []
            date_dirs = []
            
            for item in os.listdir(path):
                item_path = os.path.join(path, item)
                if os.path.isdir(item_path):
                    # 检查是否是日期格式（YYYY-MM-DD）
                    import re
                    if re.match(r'\d{4}-\d{2}-\d{2}', item):
                        date_dirs.append(item)
                    elif item in ['raw', 'result', 'rc', 'error_rc']:  # 明确的类型目录
                        type_dirs.append(item)
            
            print("[_SCAN_IMAGES] 类型目录: {}".format(type_dirs))
            print("[_SCAN_IMAGES] 日期目录: {}".format(date_dirs))
            
            # 如果指定了图片类型，检查该类型是否存在
            if image_type and image_type not in type_dirs:
                print("[_SCAN_IMAGES] 警告：指定的图片类型 '{}' 不存在，可用类型: {}".format(image_type, type_dirs))
                return images
            
            # 如果路径下直接有类型目录，说明这是点位路径，需要遍历类型目录
            if type_dirs:
                print("[_SCAN_IMAGES] 检测到点位路径，遍历类型目录")
                for type_dir in type_dirs:
                    type_path = os.path.join(path, type_dir)
                    if not os.path.isdir(type_path):
                        print("[_SCAN_IMAGES] 跳过非目录: {}".format(type_dir))
                        continue
                
                    # 如果指定了图片类型，只处理对应的类型
                    print("[_SCAN_IMAGES] 检查类型目录: '{}' vs 过滤条件: '{}'".format(type_dir, image_type))
                    print("[_SCAN_IMAGES] 类型目录长度: {}, 过滤条件长度: {}".format(len(type_dir), len(image_type)))
                    print("[_SCAN_IMAGES] 类型目录类型: {}, 过滤条件类型: {}".format(type(type_dir), type(image_type)))
                    print("[_SCAN_IMAGES] 类型目录repr: {}, 过滤条件repr: {}".format(repr(type_dir), repr(image_type)))
                    print("[_SCAN_IMAGES] 字符串比较结果: {}".format(type_dir == image_type))
                    
                    # 修复逻辑：如果指定了图片类型，只处理匹配的类型；否则处理所有类型
                    if image_type:
                        # 指定了图片类型，只处理匹配的类型
                        if type_dir == image_type:
                            print("[_SCAN_IMAGES] 处理匹配的类型目录 '{}'，过滤条件 '{}'".format(type_dir, image_type))
                        else:
                            print("[_SCAN_IMAGES] 跳过类型目录 '{}'，因为不匹配过滤条件 '{}'".format(type_dir, image_type))
                            continue
                    else:
                        # 没有指定图片类型，处理所有类型
                        print("[_SCAN_IMAGES] 处理类型目录 '{}'，无过滤条件".format(type_dir))
                
                    # 遍历日期文件夹
                    for date_dir in os.listdir(type_path):
                        date_path = os.path.join(type_path, date_dir)
                        if not os.path.isdir(date_path):
                            continue
                        
                        # 检查日期过滤条件
                        if start_date and date_dir < start_date:
                            continue
                        if end_date and date_dir > end_date:
                            continue
                        
                        # 扫描该日期文件夹下的图片文件
                        for filename in os.listdir(date_path):
                            if filename.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp', '.gif')):
                                file_path = os.path.join(date_path, filename)
                                file_stat = os.stat(file_path)
                                
                                # 构建图片信息
                                image_info = {
                                    'id': os.path.basename(file_path),
                                    'name': filename,
                                    'path': file_path,
                                    'createTime': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(file_stat.st_ctime)),
                                    'size': file_stat.st_size,
                                    'type': type_dir,
                                    'date': date_dir,
                                    'mapId': map_id,  # 使用传入的地图ID
                                    'poseId': pose_id  # 使用传入的点位ID
                                }
                                
                                print("[_SCAN_IMAGES] 找到图片: {} - 类型: {} - 路径: {}".format(filename, type_dir, file_path))
                                
                                images.append(image_info)
            
            # 如果路径下直接是日期目录，说明这是类型路径，直接扫描日期目录
            elif date_dirs:
                print("[_SCAN_IMAGES] 检测到类型路径，直接扫描日期目录")
                for date_dir in date_dirs:
                    date_path = os.path.join(path, date_dir)
                    if not os.path.isdir(date_path):
                        continue
                    
                    # 检查日期过滤条件
                    if start_date and date_dir < start_date:
                        continue
                    if end_date and date_dir > end_date:
                        continue
                    
                    # 扫描该日期文件夹下的图片文件
                    for filename in os.listdir(date_path):
                        if filename.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp', '.gif')):
                            file_path = os.path.join(date_path, filename)
                            file_stat = os.stat(file_path)
                            
                            # 构建图片信息
                            image_info = {
                                'id': os.path.basename(file_path),
                                'name': filename,
                                'path': file_path,
                                'createTime': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(file_stat.st_ctime)),
                                'size': file_stat.st_size,
                                'type': 'unknown',  # 无法确定类型
                                'date': date_dir,
                                'mapId': map_id,  # 使用传入的地图ID
                                'poseId': pose_id  # 使用传入的点位ID
                            }
                            
                            images.append(image_info)
                            
        except Exception as e:
            print("[_SCAN_IMAGES] 扫描路径 {} 时发生错误: {}".format(path, str(e)))
        
        return images

    def get_image_file(self, request_handler):
        """
        获取图片文件 - 通过图片ID返回图片文件
        """
        try:
            image_id = request_handler.get_query_argument('imageId', '')
            image_type = request_handler.get_query_argument('imageType', '')
            if not image_id:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
                return
            
            print("[GET_IMAGE_FILE] 请求图片ID: {}, 图片类型: {}".format(image_id, image_type))
            
            # 构建图片存储基础路径 - 与query_images保持一致
            base_path = "/root/Drobot_Navigation_Module/WORKSPACE/src/application/master_node"
            
            # 在所有可能的路径中查找图片
            image_found = False
            image_path = ""
            raw_image_path = ""  # 专门记录raw类型的图片路径
            
            # 遍历所有地图和点位查找图片
            if os.path.exists(base_path):
                for map_dir in os.listdir(base_path):
                    map_path = os.path.join(base_path, map_dir)
                    if not os.path.isdir(map_path):
                        continue
                    
                    for pose_dir in os.listdir(map_path):
                        pose_path = os.path.join(map_path, pose_dir)
                        if not os.path.isdir(pose_path):
                            continue
                        
                        # 遍历type文件夹
                        for type_dir in os.listdir(pose_path):
                            type_path = os.path.join(pose_path, type_dir)
                            if not os.path.isdir(type_path):
                                continue
                            
                            # 遍历日期文件夹
                            for date_dir in os.listdir(type_path):
                                date_path = os.path.join(type_path, date_dir)
                                if not os.path.isdir(date_path):
                                    continue
                                
                                # 查找匹配的图片文件
                                for filename in os.listdir(date_path):
                                    if filename == image_id or filename.startswith(image_id.split('.')[0]):
                                        current_path = os.path.join(date_path, filename)
                                        print("[GET_IMAGE_FILE] 找到匹配图片: {} - 类型: {} - 路径: {}".format(filename, type_dir, current_path))
                                        
                                        # 如果指定了图片类型，只返回匹配的类型
                                        if image_type:
                                            if type_dir == image_type:
                                                print("[GET_IMAGE_FILE] 找到指定类型的图片: {} - 类型: {}".format(current_path, type_dir))
                                                image_path = current_path
                                                image_found = True
                                                break
                                        else:
                                            # 没有指定类型，优先返回raw类型
                                            if type_dir == 'raw':
                                                print("[GET_IMAGE_FILE] 记录raw类型图片: {}".format(current_path))
                                                raw_image_path = current_path
                                                image_found = True
                                                break
                                            elif not image_found:
                                                print("[GET_IMAGE_FILE] 记录其他类型图片: {} - 类型: {}".format(current_path, type_dir))
                                                image_path = current_path
                                                image_found = True
                                
                                if image_found:
                                    break
                            if image_found:
                                break
                        if image_found:
                            break
                    if image_found:
                        break
            
            # 优先使用raw类型的图片
            if raw_image_path:
                image_path = raw_image_path
                print("[GET_IMAGE_FILE] 最终选择raw类型图片: {}".format(image_path))
            elif image_path:
                print("[GET_IMAGE_FILE] 使用其他类型图片: {}".format(image_path))
            
            if not image_found or not os.path.exists(image_path):
                print("[GET_IMAGE_FILE] 图片未找到: {}".format(image_id))
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.FILE_NOT_EXISTED))
                return
            
            print("[GET_IMAGE_FILE] 找到图片: {}".format(image_path))
            
            # 读取图片文件
            with open(image_path, 'rb') as f:
                image_data = f.read()
            
            # 设置响应头
            request_handler.set_header("Content-Type", "image/jpeg")
            request_handler.set_header("Content-Length", str(len(image_data)))
            request_handler.set_header("Access-Control-Allow-Origin", "*")
            request_handler.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            
            # 返回图片数据
            request_handler.write(image_data)
            
        except Exception as e:
            print("[GET_IMAGE_FILE] 获取图片时发生错误: {}".format(str(e)))
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SYSTEM_ERROR))

    def add_parkings(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        if not len(request_handler.request.body) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        params = json.loads(request_handler.request.body)
        # try:
        #   jsonschema.validators.validate(instance=params, schema=MasterNdeSchema.schema_add_parkings)
        # except jsonschema.exceptions.ValidationError as e:
        #   request_handler.write(MasterNodeErrorCode.FailedMsg(MasterNodeErrorCode.PARAM_FORMAT_ERROR,e.message))
        #   return
        req = console_service.AddParkingsRequest()
        if not params.has_key('path_name') or not len(params['path_name']) > 0:
            try:
                for parking in params["parkings"]:
                    req.id.append(parking["id"])
                    req.height.append(parking["height"])
                    req.width.append(parking["width"])
                    req.gridX.append(parking["gridX"])
                    req.gridY.append(parking["gridY"])
                    req.angle.append(parking["angle"])
                srv = rospy.ServiceProxy('/drobot_parking_manager/add_parkings',console_service.AddParkings)
                try:
                    srv.wait_for_service(3)
                except rospy.ROSException as servexc:
                    request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                    return
                resp = srv(req)
            except rospy.ServiceException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
                return
        #按路径添加停车位
        else:
            path_name = params['path_name']
            print("[py] path_name = "+ str(path_name))
            try:
                get_path_req = console_service.GetPathDataRequest()
                get_path_req.path_name = path_name
                get_path_srv = rospy.ServiceProxy('/drobot_path_manager/get_path_data',console_service.GetPathData)
                try:
                    get_path_srv.wait_for_service(3)
                except rospy.ROSException as servexc:
                    request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                    return
                resp = get_path_srv(get_path_req)
            except rospy.ServiceException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
                return
            if not resp.success:
                request_handler.write(MasterNodeErrorCode.GetReturn(resp))
                return
            for pose_stamped in resp.path.poses:
                addpoint = geometry_msgs.msg.Point32()
                addpoint.x = round(pose_stamped.pose.position.x, 2)
                addpoint.y = round(pose_stamped.pose.position.y, 2)
                req.polygon.points.append(addpoint)
            req.id.append(params['id'])
            srv = rospy.ServiceProxy('/drobot_parking_manager/add_parkings',console_service.AddParkings)
            try:
                srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = srv(req)
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())


    def delete_parkings(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        if not len(request_handler.request.body) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        params = json.loads(request_handler.request.body)
        try:
          jsonschema.validators.validate(instance=params, schema=MasterNdeSchema.schema_delete_parkings)
        except jsonschema.exceptions.ValidationError as e:
          request_handler.write(MasterNodeErrorCode.FailedMsg(MasterNodeErrorCode.PARAM_FORMAT_ERROR,e.message))
          return
        try:
            req = console_service.DeleteParkingsRequest()
            for parking in params["id"]:
              req.id.append(parking)
            srv = rospy.ServiceProxy('/drobot_parking_manager/delete_parkings',console_service.DeleteParkings)
            try:
                srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = srv(req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
    def update_parking(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        if not len(request_handler.request.body) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        params = json.loads(request_handler.request.body)
        try:
          jsonschema.validators.validate(instance=params, schema=MasterNdeSchema.schema_update_parkings)
        except jsonschema.exceptions.ValidationError as e:
          request_handler.write(MasterNodeErrorCode.FailedMsg(MasterNodeErrorCode.PARAM_FORMAT_ERROR,e.message))
          return
        try:
            req = console_service.UpdateParkingRequest()
            req.parking.id = params["id"]
            req.parking.height = params["height"]
            req.parking.width = params["width"]
            req.parking.gridX = params["gridX"]
            req.parking.gridY = params["gridY"]
            req.parking.angle = params["angle"]
            srv = rospy.ServiceProxy('/drobot_parking_manager/update_parking',console_service.UpdateParking)
            try:
                srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = srv(req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
    def query_parkings(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        try:
            req = console_service.GetParkingsRequest()
            srv = rospy.ServiceProxy('/drobot_parking_manager/query_parkings',console_service.GetParkings)
            try:
                srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = srv(req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        parkings_data_json = {}
        parkings_data_json["parkings"] = []
        originX = self.master_node_subscriber.submsg_table['map_info'].originX
        originY = self.master_node_subscriber.submsg_table['map_info'].originY
        resolution = self.master_node_subscriber.submsg_table['map_info'].resolution
        for parking in resp.parkings:
          parkings_data={}
          parkings_data["id"] = parking.id
          parkings_data["createAt"] = parking.createAt
          parkings_data["modifyAt"] = parking.modifyAt
          parkings_data["gridX"] = parking.gridX
          parkings_data["gridY"] = parking.gridY
          parkings_data["angle"] = parking.angle
          parkings_data["height"] = parking.height
          parkings_data["width"] = parking.width
          parkings_data["polygon"] = []
          for point in parking.polygon:
            addpoint = geometry_msgs.msg.Point32()
            addpoint.x = round((point.x - originX) / resolution, 2)
            addpoint.y = round((point.y - originY) / resolution, 2)
            polygon_data = {
              "x": addpoint.x,
              "y": addpoint.y
            }
            parkings_data["polygon"].append(polygon_data)
          parkings_data_json["parkings"].append(parkings_data)
        request_handler.write(MasterNodeErrorCode.SuccessedData(parkings_data_json))

    def add_area(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        if not len(request_handler.request.body) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        params = json.loads(request_handler.request.body)
        try:
          jsonschema.validators.validate(instance=params, schema=MasterNdeSchema.schema_add_area)
        except jsonschema.exceptions.ValidationError as e:
          request_handler.write(MasterNodeErrorCode.FailedMsg(MasterNodeErrorCode.PARAM_FORMAT_ERROR,e.message))
          return
        try:
            req = console_service.AddAreaRequest()
            req.area.name = params["name"]
            full_cover_method = 0
            if params.has_key('full_cover_method'):
                full_cover_method = params['full_cover_method']
            req.area.full_cover_method = full_cover_method
            originX = self.master_node_subscriber.submsg_table['map_info'].originX
            originY = self.master_node_subscriber.submsg_table['map_info'].originY
            resolution = self.master_node_subscriber.submsg_table['map_info'].resolution
            if not params.has_key('path_name') or not len(params['path_name']) > 0:
                for point in params["polygon"]:
                    addpoint = geometry_msgs.msg.Point32()
                    addpoint.x = round(point["x"] * resolution + originX, 2)
                    addpoint.y = round(point["y"] * resolution + originY, 2)
                    req.area.polygon.points.append(addpoint)
            #按路径添加区域
            else:
                path_name = params['path_name']
                try:
                    get_path_req = console_service.GetPathDataRequest()
                    get_path_req.path_name = path_name
                    get_path_srv = rospy.ServiceProxy('/drobot_path_manager/get_path_data',console_service.GetPathData)
                    try:
                        get_path_srv.wait_for_service(3)
                    except rospy.ROSException as servexc:
                        request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                        return
                    resp = get_path_srv(get_path_req)
                except rospy.ServiceException as servexc:
                    request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
                    return
                if not resp.success:
                    request_handler.write(MasterNodeErrorCode.GetReturn(resp))
                    return
                for pose_stamped in resp.path.poses:
                    angle = euler_from_quaternion([pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w])
                    addpoint = geometry_msgs.msg.Point32()
                    addpoint.x = round(pose_stamped.pose.position.x, 2)
                    addpoint.y = round(pose_stamped.pose.position.y, 2)
                    addpoint.z = angle[2]
                    req.area.polygon.points.append(addpoint)
            srv = rospy.ServiceProxy('/drobot_area_manager/add_areas',console_service.AddArea)
            try:
                srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = srv(req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
    def delete_area(self,request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        area_name = request_handler.get_query_argument('name')
        try:
            req = console_service.DeleteAreaRequest()
            req.areaName = area_name
            service = rospy.ServiceProxy("/drobot_area_manager/delete_areas", console_service.DeleteArea)
            resp  = service(req)
        except rospy.ServiceException as e:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
    def query_areas(self, request_handler):
        if not self.master_node_state.checkState([MasterNodeState.state_type.Idle,MasterNodeState.state_type.RunningTask,MasterNodeState.state_type.Localization]):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        area_name = request_handler.get_query_argument('name','')
        try:
            req = console_service.QueryAreasRequest()
            req.areas = area_name
            srv = rospy.ServiceProxy('/drobot_area_manager/query_areas',console_service.QueryAreas)
            try:
                srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = srv(req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        area_data_json = {}
        area_data_json["areas"] = []
        originX = self.master_node_subscriber.submsg_table['map_info'].originX
        originY = self.master_node_subscriber.submsg_table['map_info'].originY
        resolution = self.master_node_subscriber.submsg_table['map_info'].resolution
        for area in resp.areas:
          area_data={}
          area_data["name"] = area.name
          area_data["full_cover_method"] = area.full_cover_method
          area_data["polygon"] = []
          for point in area.polygon.points :
            addpoint = geometry_msgs.msg.Point32()
            addpoint.x = round((point.x - originX) / resolution, 2)
            addpoint.y = round((point.y - originY) / resolution, 2)
            polygon_data = {
              "x": addpoint.x,
              "y": addpoint.y
            }
            area_data["polygon"].append(polygon_data)
          area_data_json["areas"].append(area_data)
        request_handler.write(MasterNodeErrorCode.SuccessedData(area_data_json))
    def update_param(self,request_handler):
        if not len(request_handler.request.body) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        params = json.loads(request_handler.request.body)
        params_data = params["params"]
        try:
            add_param_req = String_reqRequest()
            add_param_req.str = json.dumps(params_data)
            add_param_srv = rospy.ServiceProxy('/param_manager_node/update_param',String_req)
            try:
                add_param_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = add_param_srv(add_param_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.FailedData(MasterNodeErrorCode.SERVICE_RETURN_FALSE,resp.message))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        pass
    def get_param(self,request_handler):
        try:
            get_param_req = String_respRequest()
            get_param_srv = rospy.ServiceProxy('/param_manager_node/get_param',String_resp)
            try:
                get_param_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = get_param_srv()
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.FailedData(MasterNodeErrorCode.SERVICE_RETURN_FALSE,resp.message))
            return
        graph_json=json.loads(resp.str)
        request_handler.write(MasterNodeErrorCode.SuccessedData(graph_json))
        pass
    def update_system_param(self,request_handler):
        if not len(request_handler.request.body) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        params = json.loads(request_handler.request.body)
        params_data = params["params"]
        try:
            add_param_req = String_reqRequest()
            add_param_req.str = json.dumps(params_data)
            add_param_srv = rospy.ServiceProxy('/param_manager_node/system_update_param',String_req)
            try:
                add_param_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = add_param_srv(add_param_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.FailedData(MasterNodeErrorCode.SERVICE_RETURN_FALSE,resp.message))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        pass
    def get_system_param(self,request_handler):
        try:
            get_param_req = String_respRequest()
            get_param_srv = rospy.ServiceProxy('/param_manager_node/get_system_param',String_resp)
            try:
                get_param_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = get_param_srv()
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.FailedData(MasterNodeErrorCode.SERVICE_RETURN_FALSE,resp.message))
            return
        graph_json=json.loads(resp.str)
        request_handler.write(MasterNodeErrorCode.SuccessedData(graph_json))
        pass
    def reset_param(self,request_handler):
        reset_type = request_handler.get_query_argument('type')
        try:
            req_msg = String_reqRequest()
            req_msg.str = reset_type
            service = rospy.ServiceProxy('/param_manager_node/reset_param',String_req)
            try:
                service.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = service(req_msg)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:

            request_handler.write(MasterNodeErrorCode.FailedData(MasterNodeErrorCode.SERVICE_RETURN_FALSE,resp.message))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        pass
    def upload_laser_param(self,request_handler):
        if request_handler.request.method == 'OPTIONS' :
          request_handler.set_header("Content-Type", "text/plain")
          request_handler.set_header("Access-Control-Allow-Origin", "*")
          request_handler.set_header("Access-Control-Allow-Methods", "POST")
          request_handler.set_header("Content-Type", "application/octet-stream")
          request_handler.write(MasterNodeErrorCode.Successed())
          return
        if len(request_handler.request.files.get("file")) == 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        file = request_handler.request.files.get("file")[0]
        filename = file.get("filename")
        filebody = file.get("body")
        laser_file = self.run_time_dir+"/sys/scan_correct.yaml.new"
        f = open(laser_file, 'w')
        f.write(filebody)
        f.close()
        request_handler.write(MasterNodeErrorCode.Successed())
        return

    # def rain_mode_handle(self):
    #     rain_mode_state = rospy.get_param('/scene_setting/rain_mode', '')
    #     if rain_mode_state:
    #         rospy.loginfo("rain_mode_handle true")
    #         rospy.set_param('/navigation/follow/speed_level', 0)
    #         rospy.set_param('/navigation/nav/speed_level', 0)
    #     if not rain_mode_state:
    #         rospy.loginfo("rain_mode_handle false")
    #         if rospy.get_param('/navigation/follow/speed_level') == 0:
    #             rospy.set_param('/navigation/follow/speed_level', self.Follow_speed_level)
    #         if rospy.get_param('/navigation/nav/speed_level') == 0:
    #             rospy.set_param('/navigation/nav/speed_level', self.Nav_speed_level)
    #     return

    def start_lane_detection(self,request_handler):
        # if  self.master_node_state.state_operation != MasterNodeState.state_type.RunningTask:
        #     request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
        #     return
        # if (self.master_node_ping.ping_sick_lane_detection() == 'alive') or (self.master_node_ping.ping_sick_tim571() == 'alive'):
        #     self.master_node_launch.launch_lane_detection.shutdown()
        # self.master_node_launch.launch_lane_detection.start()
        # request_handler.write(MasterNodeErrorCode.Successed())
        return


    def cancel_lane_detection(self,request_handler):
        # if  self.master_node_state.state_operation != MasterNodeState.state_type.RunningTask:
        #     request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
        #     return
        # if (self.master_node_ping.ping_sick_lane_detection() == 'alive') or (self.master_node_ping.ping_sick_tim571() == 'alive'):
        #     self.master_node_launch.launch_lane_detection.shutdown()
        # request_handler.write(MasterNodeErrorCode.Successed())
        return
    def save_lane_detection(self,request_handler):
        if  self.master_node_state.state_operation != MasterNodeState.state_type.RunningTask:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        # result_msg = {}
        # result_msg['data'] =''
        # result_msg['errorCode'] = ''
        # if len(request_handler.request.body) > 0:
        #     save_command = json.loads(request_handler.request.body)
        #     if ('saveMode' in save_command.keys()) and ('filterSwitch' in save_command.keys()):
        #         saveMode = save_command['saveMode']
        #         filterSwitch = save_command['filterSwitch']
        #         if saveMode == "renovate":
        #             saveMode = "ref"
        #         elif saveMode == "add":
        #             saveMode = "add"
        #         else:
        #             request_handler.write(MasterNodeErrorCode.FailedData(MasterNodeErrorCode.PARAM_ILLEGAL_ERROR))
        #             return
        #         try:
        #             saveCostmap = rospy.ServiceProxy("/sick_lane_detection/save_obj_costmap", laneSave)
        #             resp  = saveCostmap(saveMode,"s",filterSwitch)
        #         except rospy.ServiceException as servexc:
        #             request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
        #             return
        #         if not resp.success:
        #             request_handler.write(MasterNodeErrorCode.FailedData(MasterNodeErrorCode.SERVICE_RETURN_FALSE,resp.message))
        #             return
        #     else:
        #         request_handler.write(MasterNodeErrorCode.FailedData(MasterNodeErrorCode.PARAM_ILLEGAL_ERROR))
        #         return
        # else:
        #     request_handler.write(MasterNodeErrorCode.FailedData(MasterNodeErrorCode.PARAM_IS_EMPTY))
        #     return
        # request_handler.write(json.dumps(result_msg))
        # if (self.master_node_ping.ping_sick_lane_detection() == 'alive') or (self.master_node_ping.ping_sick_tim571() == 'alive'):
        #     self.master_node_launch.launch_lane_detection.shutdown()
        request_handler.write(MasterNodeErrorCode.Successed())
        return

    def get_devices_param(self,request_handler):
        try:
            get_param_req = String_respRequest()
            get_param_srv = rospy.ServiceProxy('/param_manager_node/get_devices_param',String_resp)
            resp = get_param_srv()
            try:
                get_param_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = get_param_srv()
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.FailedData(MasterNodeErrorCode.SERVICE_RETURN_FALSE,resp.message))
            return
        graph_json=json.loads(resp.str)
        request_handler.write(MasterNodeErrorCode.SuccessedData(graph_json))
        pass

    def update_devices_param(self,request_handler):
        if not len(request_handler.request.body) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        params = json.loads(request_handler.request.body)
        params_data = params["params"]
        try:
            add_param_req = String_reqRequest()
            add_param_req.str = json.dumps(params_data)
            add_param_srv = rospy.ServiceProxy('/param_manager_node/devices_update_param',String_req)
            try:
                add_param_srv.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = add_param_srv(add_param_req)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.FailedData(MasterNodeErrorCode.SERVICE_RETURN_FALSE,resp.message))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        pass

    def verify_user(self,request_handler):
        user_name = request_handler.get_query_argument('username','')
        pass_word = request_handler.get_query_argument('passwd','')
        if (user_name == '' or pass_word == ''):
          request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
          return
        try:
            req_msg = console_service.UserOpreateRequest()
            req_msg.user.user_name = user_name
            req_msg.user.pass_word = pass_word
            service = rospy.ServiceProxy('/drobot_user_manager/verify_user',console_service.UserOpreate)
            try:
                service.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = service(req_msg)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        return
    def get_users(self,request_handler):
        try:
            req_msg = common_service.srv.String_respRequest()
            service = rospy.ServiceProxy('/drobot_user_manager/all_users', common_service.srv.String_resp)
            try:
                service.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = service(req_msg)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        users = json.loads(resp.str)
        request_handler.write(MasterNodeErrorCode.SuccessedData(users))
        return
    def update_passwd(self,request_handler):
        user_name = request_handler.get_query_argument('username','')
        pass_word = request_handler.get_query_argument('passwd','')
        new_pass_word = request_handler.get_query_argument('new_passwd','')
        if (user_name == '' or pass_word == '' or new_pass_word == ''):
          request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
          return
        try:
            req_msg = console_service.UserOpreateRequest()
            req_msg.user.user_name = user_name
            req_msg.user.pass_word = pass_word
            service = rospy.ServiceProxy('/drobot_user_manager/verify_user',console_service.UserOpreate)
            try:
                service.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = service(req_msg)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        # 验证登录成功,更新密码
        try:
            req_msg = console_service.UserOpreateRequest()
            req_msg.user.user_name = user_name
            req_msg.user.pass_word = new_pass_word
            service = rospy.ServiceProxy('/drobot_user_manager/update_pssswd_user',console_service.UserOpreate)
            try:
                service.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = service(req_msg)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        return

    def add_user(self,request_handler):
        user_name = request_handler.get_query_argument('username','')
        pass_word = request_handler.get_query_argument('passwd','')
        if (user_name == '' or pass_word == ''):
          request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
          return
        try:
            req_msg = console_service.UserOpreateRequest()
            req_msg.user.user_name = user_name
            req_msg.user.pass_word = pass_word
            service = rospy.ServiceProxy('/drobot_user_manager/add_user',console_service.UserOpreate)
            try:
                service.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = service(req_msg)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        return

    def delete_user(self,request_handler):
        user_name = request_handler.get_query_argument('username','')
        if (user_name == ''):
          request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
          return
        try:
            req_msg = console_service.UserOpreateRequest()
            req_msg.user.user_name = user_name
            service = rospy.ServiceProxy('/drobot_user_manager/delete_user',console_service.UserOpreate)
            try:
                service.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = service(req_msg)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        return

    def query_sound(self,request_handler):
        try:
            service = rospy.ServiceProxy('/drobot_sound_manager/query_sounds',console_service.GetSoundsStatus)
            try:
                service.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = service()
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        status = []
        for statu in resp.status:
          temp = {}
          temp["name"] = statu.name
          temp["state"] = statu.state
          status.append(temp)
        request_handler.write(MasterNodeErrorCode.SuccessedData(status))
        return

    def play_sound(self,request_handler):
        sound_name = request_handler.get_query_argument("sound","")
        loop = request_handler.get_query_argument("loop", "false")
        if loop == "False" or loop == "false":
          loop = False
        else: loop = True
        try:
            srv = console_service.PlaySoundRequest()
            srv.sound = sound_name
            srv.loop = loop
            service = rospy.ServiceProxy('/drobot_sound_manager/play_sounds',console_service.PlaySound)
            try:
                service.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = service(srv)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        return

    def stop_sound(self,request_handler):
        sound_name = request_handler.get_query_argument("sound","")
        stop_all = request_handler.get_query_argument("stop_all", "false")
        if stop_all == "False" or stop_all == "false":
          stop_all = False
        else: stop_all = True
        try:
            srv = console_service.StopSoundRequest()
            srv.sound = sound_name
            srv.stop_all = stop_all
            service = rospy.ServiceProxy('/drobot_sound_manager/stop_sounds',console_service.StopSound)
            try:
                service.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = service(srv)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        return

    def upload_sound(self, request_handler):
        request_handler.set_header("Content-Type", "text/plain")
        request_handler.set_header("Access-Control-Allow-Origin", "*")
        request_handler.set_header("Access-Control-Allow-Methods", "PUT, POST, GET, OPTIONS, DELETE")
        if request_handler.request.method == 'OPTIONS' :
            request_handler.write(MasterNodeErrorCode.Successed())
            return

        if len(request_handler.request.files) == 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        file = request_handler.request.files.get("file")[0]
        sound_name = file.get("filename")
        filebody = file.get("body")

        sound_file_dir = self.run_time_dir+"/sound/default/" + sound_name
        f = open(sound_file_dir, 'w')
        f.write(filebody)
        f.close()
        try:
            req_msg = String_reqRequest()
            req_msg.str = sound_name
            service = rospy.ServiceProxy('/drobot_sound_manager/add_sounds',String_req)
            service(req_msg)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        return

    def delete_sound(self, request_handler):
        sound_name = request_handler.get_query_argument("sound_name","")
        if (sound_name == ''):
          request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
          return
        try:
            srv = console_service.DeleteSoundRequest()
            srv.sound_name = sound_name
            service = rospy.ServiceProxy('/drobot_sound_manager/delete_sounds',console_service.DeleteSound)
            try:
                service.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = service(srv)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.Successed())
        return

    def is_bag_recording(self,request_handler):
        recorder_status={}
        bag_recoder_status,bag_name = self.bag_recoder.CheckRecordingActive()
        if bag_recoder_status:
          recorder_status["state"] = bag_recoder_status
          recorder_status["name"] = bag_name
        else:
          recorder_status["state"] = bag_recoder_status
          recorder_status["name"] = ""
        request_handler.write(MasterNodeErrorCode.SuccessedData(recorder_status))
        return

    def start_record_bag(self,request_handler):
        bag_recoder_status,bag_name = self.bag_recoder.CheckRecordingActive()
        if bag_recoder_status:
          request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.RECORDBAG_IS_RUNNING))
          return
        bag_name = request_handler.get_query_argument("bag_name","")
        record_type = request_handler.get_query_argument("record_type","")
        recordType = BagRecorder.BagRecorder.RecordType.Default
        if record_type == "default":
          recordType = BagRecorder.BagRecorder.RecordType.Default
        elif record_type == "mapping":
          recordType = BagRecorder.BagRecorder.RecordType.Mapping
        elif record_type == "mapping2":
          recordType = BagRecorder.BagRecorder.RecordType.Mapping3D
        elif record_type == "running":
          recordType = BagRecorder.BagRecorder.RecordType.RunningDiff
        elif record_type == "running2":
          recordType = BagRecorder.BagRecorder.RecordType.RunningAckermann
        elif record_type == "localization":
          recordType = BagRecorder.BagRecorder.RecordType.Localization
        else:
          recordType = BagRecorder.BagRecorder.RecordType.Default
        self.bag_recoder.startRosbagRecord(recordType,bag_name)
        request_handler.write(MasterNodeErrorCode.Successed())
        return
    def stop_record_bag(self,request_handler):
        bag_recoder_status,bag_name = self.bag_recoder.CheckRecordingActive()
        if not bag_recoder_status:
          request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.RECORDBAG_IS_SHUTDOWN))
          return
        self.bag_recoder.stopRosbagRecord()
        request_handler.write(MasterNodeErrorCode.Successed())
    def get_bag_list(self,request_handler):
        bag_list={}
        bag_list["baglist"] = self.bag_recoder.getBagList()
        request_handler.write(MasterNodeErrorCode.SuccessedData(bag_list))

    def delete_bag(self,request_handler):
        bag_name = request_handler.get_query_argument('bag_name','')
        if not bag_name:
          request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
          return
        bag_file_path = self.bag_recoder.bag_directory +"/"+bag_name
        if(os.path.exists(bag_file_path)):
          os.remove(bag_file_path)
          request_handler.write(MasterNodeErrorCode.Successed())
          return
        else:
          request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.OPEN_FILE_FAILED))
          return
    def download_bag(self,request_handler):
        bag_name = request_handler.get_query_argument('bag_name','')
        if not bag_name:
          request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
          return
        bag_file_path = self.bag_recoder.bag_directory +"/"+bag_name
        try:
          with open(bag_file_path,'r') as bagreader:
            if '16.04' in platform.platform():
                (status, bagfile) = commands.getstatusoutput('cd '+self.bag_recoder.bag_directory+';tar -zcf - '+ bag_name+' |openssl des3 -salt -k drobot180109 ')
            else:
                (status, bagfile) = commands.getstatusoutput('cd '+self.bag_recoder.bag_directory+';tar -zcf - '+ bag_name+' |openssl des3 -pbkdf2 -salt -k drobot180109')
        except IOError:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.OPEN_FILE_FAILED))
            return
        request_handler.write(bagfile)
        request_handler.set_header("Access-Control-Allow-Origin", "*")
        request_handler.set_header("Access-Control-Allow-Methods", "PUT, POST, GET, OPTIONS, DELETE")
        request_handler.set_header("Content-Type", "application/octet-stream")
        disp = "attachment;filename=" + bag_name+".drobot"
        request_handler.set_header("Content-Disposition", disp)

    def move(self,request_handler):
        print("Master_node.py move() xc *********************0004************************")
        if len(request_handler.request.body) > 0:
            move_command = json.loads(request_handler.request.body)
            if 'speed' in move_command.keys():
                speed = move_command['speed']
                linearSpeed = 0.0
                angularSpeed = 0.0
                twist = Twist()
                if 'linearSpeed' in speed.keys():
                    linearSpeed = speed['linearSpeed']
                if 'angularSpeed' in speed.keys():
                    angularSpeed = speed['angularSpeed']
                twist.linear.x = linearSpeed
                twist.angular.z = angularSpeed
                if twist.linear.x > 1.2:
                    twist.linear.x = 1.2
                if twist.linear.x < -0.6:
                    twist.linear.x = -0.6
                if twist.angular.z > 0.6:
                    twist.angular.z = 0.6
                if twist.angular.z < -0.6:
                    twist.angular.z = -0.6
#                print twist
                self.cmd_pub.publish(twist)
                request_handler.write(MasterNodeErrorCode.Successed())
                return
            else:
              request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
              return
        else:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_BODY_IS_EMPTY))
            return
    def move_brake(self,request_handler):
        if len(request_handler.request.body) > 0:
            move_brake_command = json.loads(request_handler.request.body)
            if 'brake' in move_brake_command.keys():
              brake = move_brake_command['brake']
              if isinstance(brake, bool):
                if brake:
                  self.stop_move_flag = True
                  twist = Twist()
                  twist.linear.x ,twist.linear.y ,twist.linear.z, twist.angular.z = 0,0,0,0
                  self.cmd_pub.publish(twist)
                self.cmd_brake_pub.publish(brake)
                self.cmd_brake_state = brake
              request_handler.write(MasterNodeErrorCode.Successed())
            else:
              request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
              return
        else:
          request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_BODY_IS_EMPTY))
          return
    def get_move_brake(self,request_handler):
          brake_state = {"brake_state": self.cmd_brake_state}
          request_handler.write(MasterNodeErrorCode.SuccessedData(brake_state))
          return
    def ptz_switch(self,request_handler):
        if len(request_handler.request.body) > 0:
            switch_command = json.loads(request_handler.request.body)
            switch = Pelco_switch()
            if 'wiper' in switch_command.keys():
              wiper_switch = switch_command['wiper']
              if wiper_switch:
                switch.wiper_onoff=1
              else:
                switch.wiper_onoff=2
            if 'lighting' in switch_command.keys():
              lighting_switch = switch_command['lighting']
              if lighting_switch:
                switch.lighting_onoff=1
              else:
                switch.lighting_onoff=2
            if 'thermal' in switch_command.keys():
              thermal_switch = switch_command['thermal']
              if thermal_switch:
                switch.thermal_onoff=1
              else:
                switch.thermal_onoff=2
            self.pleco_switch_pub.publish(switch)
            request_handler.write(MasterNodeErrorCode.Successed())
            return
        else:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_BODY_IS_EMPTY))
            return
    def ptz_location_move(self, request_handler):
        if len(request_handler.request.body) > 0:
            move_command = json.loads(request_handler.request.body)
            if 'location' in move_command.keys():
                location = move_command['location']
                yaw = 0.0
                pitch = 0.0
                twist = Pelco()
                if 'yaw' in location.keys():
                    yaw = location['yaw']
                if 'pitch' in location.keys():
                    pitch = location['pitch']
                twist.yaw = yaw
                twist.pitch = pitch
                # twist.wiper_state = wiper_state
                if twist.pitch < 0 and twist.pitch >= -90:
                    twist.pitch+=360
                if twist.yaw < 0:
                    twist.yaw = 0
                if twist.yaw >= 360:
                    twist.yaw = 360
                if twist.pitch < 0:
                    twist.pitch = 0
                if twist.pitch >= 360:
                    twist.pitch = 360
                if twist.pitch < 270 and twist.pitch > 180:
                    twist.pitch = 270
                if twist.pitch <= 180 and twist.pitch > 90:
                    twist.pitch = 90
                self.pleco_cmd_pub.publish(twist)
                request_handler.write(MasterNodeErrorCode.Successed())
                return
            else:
              request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
              return
        else:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_BODY_IS_EMPTY))
            return
    def wiper_thread_func(self):
        latest_wiper_state = 0
        while not rospy.is_shutdown() and self.wiper_thread_func_state:
          switch = Pelco_switch()
          if rospy.get_param('/wiper_thread_state',False) :
            switch.wiper_onoff=1
            latest_wiper_state = 1
          elif latest_wiper_state == 1:
            switch.wiper_onoff=2
            latest_wiper_state=2
          self.pleco_switch_pub.publish(switch)
          time.sleep(1)
    def ptz_direction_move(self, request_handler):
        if len(request_handler.request.body) > 0:
            move_command = json.loads(request_handler.request.body)
            if 'direction' in move_command.keys():
                direction_json = move_command['direction']
                direction = 0
                speed = 0
                twist = Pelco_velocity()
                switch = Pelco_switch()
                # twist.pelco_stop=False
                if 'direction' in direction_json.keys():
                    direction = direction_json['direction']
                if 'speed' in direction_json.keys():
                    speed = direction_json['speed']
                if speed > 64:
                    speed = 64
                if speed < 0:
                    speed = 0
                if direction < 5:
                  twist.state_move = direction+1
                  twist.velocity = speed
                elif direction == 5:
                    if speed == 0:
                      rospy.set_param('/wiper_thread_state',False)
                    else:
                      rospy.set_param('/wiper_thread_state',True)
                elif direction == 6:
                    if speed == 0:
                      switch.lighting_onoff=2
                    else:
                      switch.lighting_onoff=1
                elif direction == 7:
                    if speed == 0:
                      switch.thermal_onoff=2
                    else:
                      switch.thermal_onoff=1
                elif direction == 8:
                    if speed == 1:
                      switch.zoom_plusminus=1
                    if speed == 2:
                      switch.zoom_plusminus=2
                    if speed == 0:
                      switch.zoom_plusminus=3
                if direction<5:
                  self.pleco_vel_cmd_pub.publish(twist)
                else :
                  self.pleco_switch_pub.publish(switch)
                request_handler.write(MasterNodeErrorCode.Successed())
                return
            else:
              request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
              return
        else:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_BODY_IS_EMPTY))
            return
        pass

    def get_robot_state(self, request_handler):
        result_msg = {}
        data = {}
        data['state_initialized'] = self.master_node_state.state_initialized
        data['state_operation'] = self.master_node_state.state_operation.value
        data['data_return'] = request_handler.request.body
        request_handler.write(MasterNodeErrorCode.SuccessedData(data))
        return
        pass

    def change_robot_state(self,request_handler):
        state = request_handler.get_query_argument('state',"")
        if state == "":
          request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
          return
        if state == "Idle":
          robot_state = MasterNodeState.state_type.Idle
        elif state == "Mapping":
          robot_state = MasterNodeState.state_type.Mapping
        elif state == "RunningTask":
          robot_state = MasterNodeState.state_type.RunningTask
        elif state == "RestartMapping":
          robot_state = MasterNodeState.state_type.RestartMapping
        elif state == "Localization":
          robot_state = MasterNodeState.state_type.Localization
        elif state == "Sleep":
          robot_state = MasterNodeState.state_type.Sleep
        elif state == "FollowTarget":
          robot_state = MasterNodeState.state_type.FollowTarget
        else:
          request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
          return
        if not self.master_node_state.changeState(robot_state):
          request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.FAILED_CHANGE_STATE))
          return
        request_handler.write(MasterNodeErrorCode.Successed())
        return

    def light_control(self, request_handler):
        if len(request_handler.request.body) > 0:
            light_command = json.loads(request_handler.request.body)
            if 'light_switch' in light_command.keys():
              service = rospy.ServiceProxy("/drobot_chassis_manager/light_control", console_service.SetLight)
              try:
                  service.wait_for_service(3)
              except rospy.ROSException as servexc:
                  request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                  return
              light_control_msg = console_service.SetLightRequest()
              try:
                if light_command['light_switch'].has_key('High_light'):
                  light_control_msg.lightType.type = console_msg.LightType.HIGH_LIGHT
                  light_control_msg.value = light_command['light_switch']['High_light']
                  resp  = service(light_control_msg)
                if light_command['light_switch'].has_key('Low_light'):
                  light_control_msg.lightType.type = console_msg.LightType.LOW_LIGHT
                  light_control_msg.value = light_command['light_switch']['Low_light']
                  resp  = service(light_control_msg)
                if light_command['light_switch'].has_key('Warning_light'):
                  light_control_msg.lightType.type = console_msg.LightType.WARNING_LIGHT
                  light_control_msg.value = light_command['light_switch']['Warning_light']
                  resp  = service(light_control_msg)
                if light_command['light_switch'].has_key('UV_light'):
                  light_control_msg.lightType.type = console_msg.LightType.UV_LIGHT
                  light_control_msg.value = light_command['light_switch']['UV_light']
                  resp  = service(light_control_msg)
                if light_command['light_switch'].has_key('Lock_light'):
                  light_control_msg.lightType.type = console_msg.LightType.LOCK_LIGHT
                  light_control_msg.value = light_command['light_switch']['Lock_light']
                  resp  = service(light_control_msg)
              except rospy.ServiceException, e:
                  request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
                  return
              if not resp.success:
                  request_handler.write(MasterNodeErrorCode.GetReturn(resp))
                  return
              request_handler.write(MasterNodeErrorCode.Successed())
            else:
              request_handler.write(MasterNodeErrorCode.PARAM_FORMAT_ERROR)
        else:
            request_handler.set_header("Access-Control-Allow-Origin", "*")
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
        return
    def show_led_text(self,request_handler):
        if not len(request_handler.request.body) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        led_command = json.loads(request_handler.request.body)
        if ( 'color' in led_command.keys() ) and ( 'text' in  led_command.keys() ):
          service = rospy.ServiceProxy("/drobot_chassis_manager/led_set", console_service.SetLedDisplay)
          try:
              service.wait_for_service(3)
          except rospy.ROSException as servexc:
              request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
              return
          led_control_msg = console_service.SetLedDisplayRequest()
          led_control_msg.color = led_command['color']
          led_control_msg.display_char = led_command['text']
          try:
              resp  = service(led_control_msg)
          except rospy.ServiceException, e:
              request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
              return
          if not resp.success:
              request_handler.write(MasterNodeErrorCode.GetReturn(resp))
              return
          request_handler.write(MasterNodeErrorCode.Successed())
          return
        else:
          request_handler.write(MasterNodeErrorCode.PARAM_FORMAT_ERROR)
          return

    def get_led_text(self,request_handler):
        service = rospy.ServiceProxy("/drobot_chassis_manager/led_get", console_service.GetLedDisplay)
        try:
            service.wait_for_service(3)
        except rospy.ROSException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
            return
        try:
          resp  = service()
        except rospy.ServiceException, e:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        led_text = {}
        led_text['color'] = resp.color
        led_text['text'] = resp.display_char
        request_handler.write(MasterNodeErrorCode.SuccessedData(led_text))
        return


    #开关控制
    def switch_control(self,request_handler):
        if len(request_handler.request.body) > 0:
            switch_command = json.loads(request_handler.request.body)
            if 'switchs' in switch_command.keys():
              service = rospy.ServiceProxy("/drobot_chassis_manager/switch_control", console_service.SetSwitch)
              try:
                  service.wait_for_service(3)
              except rospy.ROSException as servexc:
                  request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                  return
              try:
                for switch in switch_command['switchs']:
                  switch_control_msg = console_service.SetSwitchRequest()
                  switch_control_msg.sw = switch['switch_bit']
                  switch_control_msg.value = switch['value']
                  resp  = service(switch_control_msg)
              except rospy.ServiceException as e:
                  request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
                  return
              if not resp.success:
                  request_handler.write(MasterNodeErrorCode.GetReturn(resp))
                  return
              request_handler.write(MasterNodeErrorCode.Successed())
            else:
              request_handler.write(MasterNodeErrorCode.PARAM_FORMAT_ERROR)
        else:
          request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
        return


    #直线移动
    def linear_move(self, request_handler):
        if not len(request_handler.request.body) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        control_msgs = json.loads(request_handler.request.body)
        if not self.stop_move_flag :
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.ROTATING_ERROR))
            return
        if (not 'distance' in control_msgs.keys()) or (not 'speed' in control_msgs.keys()) :
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
            return
        distance = control_msgs['distance']
        distance = math.fabs(distance)
        speed = control_msgs['speed']
        if (speed > 0.6):
            speed = 0.6
        if  speed < -0.6:
            speed = -0.6
        self.stop_move_flag = False
        thread.start_new_thread(self.linearControl,(distance,speed))
        request_handler.write(MasterNodeErrorCode.Successed())
        return


    #直线控制
    def linearControl(self,distance,speed):
        twist_msg = Twist()
        twist_msg.linear.x = speed
        initial_distance = distance
        initial_coordinate={}
        initial_coordinate["x"]=0
        initial_coordinate["y"]=0
        if self.master_node_subscriber.check_reviced('raw_odom'):
            initial_coordinate["x"] = self.master_node_subscriber.submsg_table['raw_odom']['pose']['position']['x']
            initial_coordinate["y"] = self.master_node_subscriber.submsg_table['raw_odom']['pose']['position']['y']
        else:
            return False
        current_distance=0
        while not self.stop_move_flag:
            distance_x = self.master_node_subscriber.submsg_table['raw_odom']['pose']['position']['x'] - initial_coordinate["x"]
            distance_y = self.master_node_subscriber.submsg_table['raw_odom']['pose']['position']['y'] - initial_coordinate["y"]
            current_distance = math.sqrt(distance_x*distance_x + distance_y*distance_y)
            if current_distance > initial_distance:
                twist_msg.linear.x = 0
                self.cmd_pub.publish(twist_msg)
                self.stop_move_flag = True
            else:
                self.cmd_pub.publish(twist_msg)
            time.sleep(0.2)
        #用于停止信号发送
        twist_msg.linear.x = 0
        self.cmd_pub.publish(twist_msg)
        return True
        pass


    #旋转
    def rotate(self,angle,speed):
        twist_msg = Twist()
        twist_msg.angular.z = speed
        initial_angular = 0
        if self.master_node_subscriber.check_reviced('raw_odom'):
            oritation = self.master_node_subscriber.submsg_table['raw_odom']['pose']['orientation']
            yaw_angle = euler_from_quaternion([oritation['x'], oritation['y'], oritation['z'],oritation['w']])
            initial_angular = yaw_angle[2]
        else:
            return False
        self.cmd_pub.publish(twist_msg)
        rotate_angle=0
        while not self.stop_move_flag:
            oritation = self.master_node_subscriber.submsg_table['raw_odom']['pose']['orientation']
            yaw_angle = euler_from_quaternion([oritation['x'], oritation['y'], oritation['z'],oritation['w']])
            current_angular = yaw_angle[2]
            diff=initial_angular-current_angular
            diff=math.atan2(math.sin(diff),math.cos(diff))
            rotate_angle=rotate_angle+diff
            initial_angular = current_angular
            if math.fabs(rotate_angle*57.295780) > angle :
                twist_msg.angular.z = 0
                self.cmd_pub.publish(twist_msg)
                self.stop_move_flag = True
            else :
              self.cmd_pub.publish(twist_msg)
            time.sleep(0.2)
            pass
        twist_msg.angular.z = 0
        self.cmd_pub.publish(twist_msg)
        return True
        pass


    #旋转移动
    def rotate_move(self, request_handler):
        if not len(request_handler.request.body) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        rotate_msgs = json.loads(request_handler.request.body)
        if not self.stop_move_flag  :
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.ROTATING_ERROR))
            return
        if (not 'angle' in rotate_msgs.keys()) or (not 'speed' in rotate_msgs.keys()) :
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
            return
        angle = rotate_msgs['angle']
        angle = math.fabs(angle)
        speed = rotate_msgs['speed']
        if angle > 360 :
            angle = 360
        if (speed > 0.5):
            speed = 0.5
        if  speed < -0.5:
            speed = -0.5
        if abs(speed) < 0.1:
          if speed> 0:
            speed = 0.1
          else:
            speed = -0.1
        self.stop_move_flag = False
        thread.start_new_thread(self.rotate,(angle,speed))
        request_handler.write(MasterNodeErrorCode.Successed())
        return
        pass


    #检查移动完成
    def check_move_finished(self, request_handler):
        request_handler.write(MasterNodeErrorCode.SuccessedData(self.stop_move_flag))
        return

    #停止移动
    def stop_move(self, request_handler):
        self.stop_move_flag = True
        request_handler.write(MasterNodeErrorCode.Successed())
        return

    #获得产品描述
    def get_product_version(self,request_handler):
        product_file=self.workspace_dir+"/version.json"
        version_msg={}
        try:
            with open(product_file, 'r') as f:
                version_msg = json.load(f)
                # print(version_msg)
        except IOError:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.OPEN_FILE_FAILED))
            return
        request_handler.write(MasterNodeErrorCode.SuccessedData(version_msg))
        return
    def srs_opreation(self,request_handler):
        op = request_handler.get_query_argument('op',"")
        if op == "restart" :
          srs_server_dir = os.environ.get("SrsServer")
          srs_server_dir = "/root/MediaServer/srs"
          (status, bagfile) = commands.getstatusoutput('cd '+srs_server_dir+';./etc/init.d/srs restart')
          request_handler.write(MasterNodeErrorCode.Successed())
          return

    #获得系统信息
    def get_system_info(self,request_handler):
        (status, hd_serial) = commands.getstatusoutput('lsblk --nodeps -no serial /dev/sda')
        info = {}
        info["hd_serial"] = hd_serial
        st = os.statvfs("/")
        info["free_disk"] = st.f_bavail * st.f_frsize / 1024 // 1024
        info["total_disk"] = st.f_blocks * st.f_frsize  / 1024 // 1024
        info["cpu_utilization"] = psutil.cpu_percent()
        info["cpu_temperature"] = round(psutil.sensors_temperatures()['coretemp'][0].current,1)
        mem = psutil.virtual_memory()
        # print mem
        info["memery_used_percent"] = round(mem.percent)
        info["memery_total"] = round(mem.total / 1024 / 1024)
        info["memery_available"] = round(mem.available / 1024 / 1024)
        info["memery_buffer_cache"] = round((mem.buffers+mem.cached) /1024 /1024)
        request_handler.write(MasterNodeErrorCode.SuccessedData(info))
        return
    def traffic_light_switch(self,request_handler):
        op = request_handler.get_query_argument('op',"")
        srv_msg = xiaolv_msgs.srv.Traffic_light_switchRequest()
        if op == "true":
          srv_msg.light_switch = True
        elif op == "false" :
          srv_msg.light_switch = False
        try:
          service = rospy.ServiceProxy('/cmd_traffic_light', xiaolv_msgs.srv.Traffic_light_switch)
          service.wait_for_service(3)
          resp = service(srv_msg)
        except rospy.ROSException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
            return
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        ret_msg = {}
        ret_msg["successed"] = resp.success
        ret_msg["errorCode"] = 0
        ret_msg["msg"] = resp.message
        ret_msg["data"] = {}
        request_handler.write(ret_msg)
        return

    #扫地机(东风类机器)
    def clean_devices_control(self, request_handler):
        if not len(request_handler.request.body) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        switch_command = json.loads(request_handler.request.body)

        clean_msg = std_msgs.msg.Int8MultiArray()
        clean_msg.data = [0, 0]
        if 'sweep_switch' in switch_command.keys():
            sweep_switch = switch_command['sweep_switch']
            if sweep_switch:
                clean_msg.data[0] = 1
            else:
                clean_msg.data[0] = 2

        if 'watering_switch' in switch_command.keys():
            watering_switch = switch_command['watering_switch']
            if watering_switch:
                clean_msg.data[1] = 1
            else:
                clean_msg.data[1] = 2
        self.clean_switch_pub.publish(clean_msg)

        request_handler.write(MasterNodeErrorCode.Successed())
        return

    #洗地机(洁驰类机器)
    def floor_washer_control(self, request_handler):
        if not len(request_handler.request.body) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        switch_command = json.loads(request_handler.request.body)
        washing_msg = std_msgs.msg.Int16()
        washing_msg.data = 0x00
        #吸盘(uptake_switch) 刮盘(scrape_switch) 刷盘(brush_switch) 喷水(watering_switch)
        if 'uptake_switch' in switch_command.keys():
            device_switch = switch_command['uptake_switch']['switch'] #吸盘控制
            speed_level = switch_command['uptake_switch']['speed'] #吸盘速度
            if device_switch:
                if speed_level == 1:
                    washing_msg.data |= 0x10 #低速
                if speed_level == 2:
                    washing_msg.data |= 0x20 #中速
                if speed_level == 3:
                    washing_msg.data |= 0x30 #高速
            else:
                washing_msg.data |= 0x00 #关闭刷盘

        if 'scrape_switch' in switch_command.keys(): #刮水控制
            device_switch = switch_command['scrape_switch']
            if device_switch:
                washing_msg.data |= 0x08 #开启刮盘
            else:
                washing_msg.data |= 0x00

        if 'brush_switch' in switch_command.keys(): #刷盘控制
            device_switch = switch_command['brush_switch']['switch']
            speed_level = switch_command['brush_switch']['speed'] #刷盘速度
            if device_switch:
                if speed_level == 1:
                    washing_msg.data |= 0x44 #低速
                if speed_level == 2:
                    washing_msg.data |= 0x84 #中速
                if speed_level == 3:
                    washing_msg.data |= 0xC4 #高速
            else:
                washing_msg.data |= 0x00

        if 'spout_switch' in switch_command.keys(): #喷水控制
            device_switch = switch_command['spout_switch']
            if device_switch:
                washing_msg.data |= 0x02
            else:
                washing_msg.data |= 0x00
        self.floor_washer_pub.publish(washing_msg)

        request_handler.write(MasterNodeErrorCode.Successed())
        return

    #顶升
    def lift_ctrl(self,request_handler):
        lift_state = request_handler.get_query_argument('lift_state','')
        lift_msg = std_msgs.msg.Int8()
        lift_state = int (lift_state)
        if (lift_state == None or(lift_state != 1 and lift_state != 0)):
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return
        lift_msg.data = lift_state
        self.lift_ctrl_pub.publish(lift_msg)
        request_handler.write(MasterNodeErrorCode.Successed())
        return

    def calibrate_scan(self, request_handler):
        scan_id = request_handler.get_query_argument("scan_id", "")
        x_offset = request_handler.get_query_argument('x_offset', "0")
        y_offset = request_handler.get_query_argument('y_offset', "0")
        yaw_offset = request_handler.get_query_argument('yaw_offset', "0")
        x_offset = float(x_offset)
        y_offset = float(y_offset)
        yaw_offset = float(yaw_offset)
        tf_after_calibration = {}
        valid_scan_id = set(["laser01", "laser02", "laser03", "laser04", "lslidar_c32"])
        if scan_id not in valid_scan_id:
            request_handler.write(MasterNodeErrorCode.FailedMsg(MasterNodeErrorCode.PARAM_INVALID_ERROR, "scan_id must be part of valid laser set"))
            return
        # 假定scan_id一定是 laser0x的形式
        calibrated_scan_launch_file = self.run_time_dir + "/public/calibrated_" + scan_id + "_tf.launch"
        if not os.path.exists(calibrated_scan_launch_file):
            # 激光未做过矫正，
            # 假设tf从launch文件中读取
            trans = None
            rot = None
            try:
                time_now = rospy.Time.now()
                (trans, rot) = self.master_node_subscriber.TfListener.lookupTransform('/base_link', scan_id, time_now)
            except:
                request_handler.write(MasterNodeErrorCode.FailedMsg(MasterNodeErrorCode.NOT_RECIVED_MSG, "no " + scan_id + " frame exist"))
                return

            trans[0] = trans[0] + x_offset
            trans[1] = trans[1] + y_offset
            euler_angle = euler_from_quaternion(rot)
            corrected_yaw = euler_angle[2] + yaw_offset
            tf_transform = []
            tf_transform.append(trans[0])
            tf_transform.append(trans[1])
            tf_transform.append(trans[2])
            tf_transform.append(corrected_yaw)
            tf_transform.append(euler_angle[1])
            tf_transform.append(euler_angle[0])
            tf_after_calibration['calibrated_tf'] = tf_transform
            doc = xml.dom.minidom.Document()
            launch_doc = doc.createElement("launch")
            doc.appendChild(launch_doc)
            node_doc = doc.createElement("node")
            node_doc.setAttribute('pkg', "tf2_ros")
            node_doc.setAttribute('type', "static_transform_publisher")
            node_name_attri = "base_link_to_" + scan_id
            node_doc.setAttribute('name', node_name_attri)
            tf_transform_str = ' '.join([str(item) for item in tf_transform])
            tf_transform_str = tf_transform_str + ' base_link ' + scan_id
            node_doc.setAttribute('args', tf_transform_str)
            launch_doc.appendChild(node_doc)
            fp = open(calibrated_scan_launch_file, "w")
            doc.writexml(fp, addindent='\t', newl='\n', encoding='utf-8')
            fp.flush()
            fp.close()
        else:
            # 激光已矫正，将launch文件中的x,y, yaw移动偏移量
            dom_doc = xml.dom.minidom.parse(calibrated_scan_launch_file)
            root_doc = dom_doc.documentElement
            nodes_doc = root_doc.getElementsByTagName('node')
            target_node_name = "base_link_to_" + scan_id
            args_str = nodes_doc[0].getAttribute('args').split(' ')
            args_str_suffix = args_str[6:]
            args_str = args_str[0:6]
            args_float = [float(item) for item in args_str]
            args_float[0] = args_float[0] + x_offset
            args_float[1] = args_float[1] + y_offset
            args_float[3] = args_float[3] + yaw_offset
            tf_after_calibration['calibrated_tf'] = args_float
            tf_transform_str = [str(item) for item in args_float]
            tf_transform_str.extend(args_str_suffix)
            tf_transform_str = ' '.join(tf_transform_str)
            nodes_doc[0].setAttribute('args', tf_transform_str)
            fp = open(calibrated_scan_launch_file, "w")
            dom_doc.writexml(fp, encoding='utf-8')
            fp.flush()
            fp.close()
        rospy.sleep(1)
        if(scan_id == "laser01"):
            self.master_node_state.restartCalibratedLaser01TF()
        elif(scan_id == "laser02"):
            self.master_node_state.restartCalibratedLaser02TF()
        elif(scan_id == "laser03"):
            self.master_node_state.restartCalibratedLaser03TF()
        elif(scan_id == "laser04"):
            self.master_node_state.restartCalibratedLaser04TF()
        elif(scan_id == "lslidar_c32"):
            self.master_node_state.restartCalibratedLSLidar32TF()
        request_handler.write(MasterNodeErrorCode.SuccessedData(tf_after_calibration))

        return

    def calibrate_lslidar_32(self, request_handler):
        # if self.master_node_state.state_operation != MasterNodeState.state_type.Idle:
        #     request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
        #     return
        rospy.loginfo("[Master_node.py] calibrate lslidar c32: True")
        self.master_node_state.master_node_launch.launch_calibrate_lslidar_c32.start()

        calibration_cnt = 0

        while (not rospy.is_shutdown() and calibration_cnt <= 3):
            rospy.sleep(1)
            calibration_cnt = calibration_cnt + 1
            rospy.loginfo("[Master_node.py] checking rslidar c32 !!!!!!!!!!!!!!!!!!!!!!")

        self.master_node_state.restartCalibratedLSLidar32TF()
        request_handler.write(MasterNodeErrorCode.Successed())

        return

    def calibrate_lslidar_16(self, request_handler):
        # if self.master_node_state.state_operation != MasterNodeState.state_type.Idle:
        #     request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
        #     return
        calibration_cnt = 0
        rospy.loginfo("[Master_node.py] calibrate lslidar c16: True")
        self.master_node_state.master_node_launch.launch_calibrate_lslidar_c16.start()
        while (not rospy.is_shutdown() and calibration_cnt < 3):
            rospy.sleep(1)
            calibration_cnt = calibration_cnt + 1
            rospy.loginfo("[Master_node.py] checking rslidar c16 !!!!!!!!!!!!!!!!!!!!")
        self.master_node_state.restartCalibratedLidarTF()
        request_handler.write(MasterNodeErrorCode.Successed())

        return

    def getCurrentTime(self, request_handler):
        now_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        request_handler.write(MasterNodeErrorCode.SuccessedData(now_time))
        return

    def syncCurrentTime(self, request_handler):
        if not len(request_handler.request.body) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        #限制当前状态Idle，否则状态不匹配
        if self.master_node_state.state_operation != MasterNodeState.state_type.Idle:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.STATUS_NOT_MATCHED))
            return

        request_command = json.loads(request_handler.request.body)

        if 'time' in request_command.keys():
            receive_time = request_command['time']
            print("receive_time = " + receive_time)
            #判断输入的时间参数格式，不符合则退出
            try:
                time.strptime(receive_time, "%Y-%m-%d %H:%M:%S")
            except:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
                return
            #设置系统时间
            cmd = 'date -s "{0}";hwclock -w'.format(receive_time)  # 设置时间并写入bios
            os.system(cmd)
            pass
        else:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_FORMAT_ERROR))
            return

        #正确时返回
        now_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        request_handler.write(MasterNodeErrorCode.SuccessedData(now_time))
        return

    def getPlannedPath(self, request_handler):
        if not len(request_handler.request.body) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        path_command = json.loads(request_handler.request.body)
        if "start" in path_command.keys() and "goal" in  path_command.keys():
            try:
                get_path_srv = rospy.ServiceProxy("/bt_navigator_node/get_planned_path", dr_nav_services.srv.GetPlannedPath)
                try:
                    get_path_srv.wait_for_service(3)
                except rospy.ROSException as servexc:
                    request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                    return
                get_planned_path_msg = dr_nav_services.srv.GetPlannedPathRequest()
                get_planned_path_msg.start.position.x = path_command['start']['x']
                get_planned_path_msg.start.position.y = path_command['start']['y']
                get_planned_path_msg.start.position.z = 0.0
                get_planned_path_msg.start.orientation.x = 0.0
                get_planned_path_msg.start.orientation.y = 0.0
                get_planned_path_msg.start.orientation.z = 0.0
                get_planned_path_msg.start.orientation.w = 1.0
                get_planned_path_msg.goal.position.x = path_command['goal']['x']
                get_planned_path_msg.goal.position.y = path_command['goal']['y']
                get_planned_path_msg.goal.position.z = 0.0
                get_planned_path_msg.goal.orientation.x = 0.0
                get_planned_path_msg.goal.orientation.y = 0.0
                get_planned_path_msg.goal.orientation.z = 0.0
                get_planned_path_msg.goal.orientation.w = 1.0
                try:
                    resp  = get_path_srv(get_planned_path_msg)
                except rospy.ServiceException as e:
                    request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
                    return
            except rospy.ServiceException as e:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
                return

            if not resp.success:
                request_handler.write(MasterNodeErrorCode.GetReturn(resp))
                return

            resp_json = {}
            resp_json["path_pose"] = []
            for path_point in resp.path_data:
                point_json = {}
                point_json['x'] = round(path_point.x, 2)
                point_json['y'] = round(path_point.y, 2)
                resp_json["path_pose"].append(point_json)
            resp_json["path_len"]  = resp.path_length
            return request_handler.write(MasterNodeErrorCode.SuccessedData(resp_json))
        else:
            request_handler.write(MasterNodeErrorCode.PARAM_FORMAT_ERROR)
            return

    def getCleanPath(self, request_handler):
        if not len(request_handler.request.body) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        path_command = json.loads(request_handler.request.body)
        if "area_name" in path_command.keys():
            try:
                get_clean_path_srv = rospy.ServiceProxy("/drobot_path_manager/get_clean_area_path", common_service.srv.CleanPath)
                try:
                    get_clean_path_srv.wait_for_service(3)
                except rospy.ROSException as servexc:
                    request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                    return
                get_clean_path_msg = common_service.srv.CleanPathRequest()
                get_clean_path_msg.area_name = path_command['area_name']
                try:
                    resp = get_clean_path_srv(get_clean_path_msg)
                except rospy.ServiceException as e:
                    request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
                    return
            except rospy.ServiceException as e:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
                return

            if not resp.success:
                request_handler.write(MasterNodeErrorCode.GetReturn(resp))
                return

            resp_json = {}
            resp_json["path_pose"] = []
            originX = self.master_node_subscriber.submsg_table['map_info'].originX
            originY = self.master_node_subscriber.submsg_table['map_info'].originY
            resolution = self.master_node_subscriber.submsg_table['map_info'].resolution
            pointstamped = PoseStamped()
            grid_poses=[]
            for point in resp.area_path_data:
                pointstamped.pose.position.x = round(point.x, 2)
                pointstamped.pose.position.y = round(point.y, 2)
                grid_point = helper.Coordinates.PoseToGrid(pointstamped.pose,originX,originY,resolution)
                grid_poses.append(grid_point)
            unique_grids = helper.Coordinates.UniqueGrid(grid_poses)

            for grid in unique_grids:
                point_json = {}
                point_json['x'] = grid.x
                point_json['y'] = grid.y
                resp_json["path_pose"].append(point_json)
            return request_handler.write(MasterNodeErrorCode.SuccessedData(resp_json))
        else:
            request_handler.write(MasterNodeErrorCode.PARAM_FORMAT_ERROR)
            return

    def getScene(self,request_handler):
        user_name = request_handler.get_query_argument('scene_name','')
        try:
            req_msg = console_service.GetSceneRequest()
            req_msg.scene_name = user_name
            service = rospy.ServiceProxy('/drobot_scene_manager/get_scene',console_service.GetScene)
            try:
                service.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = service(req_msg)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        sceneJson = json.loads(resp.scene_value)
        request_handler.write(MasterNodeErrorCode.SuccessedData(sceneJson))
        return

    def addScene(self,request_handler):
        if not len(request_handler.request.body) > 0:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.PARAM_IS_EMPTY))
            return
        str = json.loads(request_handler.request.body)
        try:
            req_msg = String_reqRequest()
            req_msg.str = request_handler.request.body
            service = rospy.ServiceProxy('/drobot_scene_manager/add_scene',String_req)
            try:
                service.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = service(req_msg)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.GetReturn(resp))
        return

    def deleteScene(self,request_handler):
        scene_name = request_handler.get_query_argument('scene_name','')
        try:
            req_msg = String_reqRequest()
            req_msg.str = scene_name
            service = rospy.ServiceProxy('/drobot_scene_manager/delete_scene',String_req)
            try:
                service.wait_for_service(3)
            except rospy.ROSException as servexc:
                request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_TIMEOUT_ERROR))
                return
            resp = service(req_msg)
        except rospy.ServiceException as servexc:
            request_handler.write(MasterNodeErrorCode.Failed(MasterNodeErrorCode.SERVICE_FAILED_CALLED))
            return
        if not resp.success:
            request_handler.write(MasterNodeErrorCode.GetReturn(resp))
            return
        request_handler.write(MasterNodeErrorCode.GetReturn(resp))
        return

#### web socket applications####

##################################END HANDLER FUNCTION##################################



#tornado.websocket

    #websocket推送所有消息
    def send_websocket_notice(self, client):

        websocket_notice = {}
        websocket_notice['nav_status'] = []
        if self.master_node_subscriber.check_reviced('navigation_notice'):
            # websocket_notice['navigate_status'] = 'navigation_notice'
            last_send_stamp = WebSocket.client_map[client.client_id]
            # rospy.loginfo("sending websocket client %d stamp %lf", client.client_id, WebSocket.client_map[client.client_id])
            send_nav_notice_size = 0
            for nav_notice in self.master_node_subscriber.submsg_table['navigation_notice']['nav_status']:
                if nav_notice['stamp'] > WebSocket.client_map[client.client_id]:
                    send_nav_notice_size = send_nav_notice_size + 1
                    websocket_notice['nav_status'].append(nav_notice)
                    last_send_stamp = nav_notice['stamp']

            # rospy.loginfo("sending websocket size %d to client %d last send stamp %lf", send_nav_notice_size, client.client_id, last_send_stamp)
            WebSocket.client_map[client.client_id] = last_send_stamp
            # 尝试清除 navigation_notice 队列中已被所有websocket用户发送过的数据
            oldest_send_time = time.time()
            for id,stamp_time in WebSocket.client_map.items():
                # rospy.loginfo(" client id %d send client time %lf", id, stamp_time)
                if stamp_time < oldest_send_time:
                    oldest_send_time = stamp_time

            for nav_notice in self.master_node_subscriber.submsg_table['navigation_notice']['nav_status']:
                if nav_notice['stamp'] < oldest_send_time:
                    self.master_node_subscriber.submsg_table['navigation_notice']['nav_status'].pop(0)
                else:
                    break
            # rospy.loginfo("nav notice size %d", len(self.master_node_subscriber.submsg_table['navigation_notice']['nav_status']))

        # websocket_notice['device_status'] = 'device_notice'
        websocket_notice.update(self.master_node_subscriber.device_notice)
        if self.master_node_subscriber.check_reviced('auto_mode'):
            websocket_notice['auto_mode'] = self.master_node_subscriber.submsg_table['auto_mode']
        if self.master_node_subscriber.check_reviced('move_flag'):
            websocket_notice.update(self.master_node_subscriber.submsg_table['move_flag'])
        if self.master_node_subscriber.check_reviced('velocity'):
            websocket_notice['angular_rotate'] = self.master_node_subscriber.submsg_table['velocity']['device_data']['angular_rotate']
        websocket_notice['rain_mode'] = rospy.get_param('/scene_setting/rain_mode', False)
        # websocket_notice['localization_status'] = 'localization_notice'
        if self.master_node_subscriber.check_reviced('localization_status'):
            websocket_notice.update(self.master_node_subscriber.submsg_table['localization_status'])
        if self.master_node_subscriber.check_reviced('localization_score'):
            websocket_notice.update(self.master_node_subscriber.submsg_table['localization_score'])

        if self.master_node_subscriber.check_reviced('robot_pose_grid'):
            websocket_notice['worldPose'] = self.master_node_subscriber.submsg_table['robot_pose_grid']['worldPose']
            websocket_notice['gridPosition'] = self.master_node_subscriber.submsg_table['robot_pose_grid']['nav_status_data']['gridPosition']

        cur_time = str(rospy.Time.now().secs) + str('.') + str(rospy.Time.now().nsecs)
        if self.master_node_subscriber.check_reviced('sensor_exception_event') and float(cur_time) < self.master_node_subscriber.cur_time_table['latest_laser_exception_time'] + 3:
            websocket_notice['sensor_exception_event'] = self.master_node_subscriber.submsg_table['sensor_exception_event']
        
        if self.master_node_subscriber.check_reviced('chassis_exception_event'):
            websocket_notice['chassis_exception_event'] = self.master_node_subscriber.submsg_table['chassis_exception_event']

        websocket_notice["date"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        client.write_message(MasterNodeErrorCode.SuccessedData(websocket_notice))
        pass

    #websocket推送所有消息
    def send_websocket_notice2(self, client):

        websocket_notice = {}
        websocket_notice['nav_notice'] = []
        websocket_notice['device_notice'] = {}
        websocket_notice['loc_notice'] = {}
        if self.master_node_subscriber.check_reviced('navigation_notice'):
            # websocket_notice['navigate_status'] = 'navigation_notice'
            websocket_notice['nav_notice'].update(self.master_node_subscriber.submsg_table['navigation_notice'])
            self.master_node_subscriber.submsg_table['navigation_notice']['nav_status'] = []

        # websocket_notice['device_status'] = 'device_notice'
        websocket_notice['device_notice'].update(self.master_node_subscriber.device_notice)
        if self.master_node_subscriber.check_reviced('move_flag'):
            websocket_notice['device_notice'].update(self.master_node_subscriber.submsg_table['move_flag'])
        if self.master_node_subscriber.check_reviced('velocity'):
            websocket_notice['device_notice']['angular_rotate'] = self.master_node_subscriber.submsg_table['velocity']['device_data']['angular_rotate']

        # websocket_notice['localization_status'] = 'localization_notice'
        if self.master_node_subscriber.check_reviced('localization_status'):
            websocket_notice['loc_notice']['status'] = self.master_node_subscriber.submsg_table['localization_status']
        if self.master_node_subscriber.check_reviced('localization_score'):
            websocket_notice['loc_notice']['score'] = self.master_node_subscriber.submsg_table['localization_score']

        if self.master_node_subscriber.check_reviced('robot_pose_grid'):
            websocket_notice['loc_notice']['worldPose'] = self.master_node_subscriber.submsg_table['robot_pose_grid']['worldPose']
            websocket_notice['loc_notice']['gridPosition'] = self.master_node_subscriber.submsg_table['robot_pose_grid']['nav_status_data']['gridPosition']
        websocket_notice["date"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        client.write_message(MasterNodeErrorCode.SuccessedData(websocket_notice))
        pass

    #websocket推送所有激光
    def send_scan_grid(self,client, req_scan_id):
        scan_grid_notice = {}
        if self.master_node_subscriber.check_reviced('scan_grid'):
            scan_grid_notice['scan_grid_3d'] = {}
            scan_grid_notice['scan_grid_3d'].update(self.master_node_subscriber.submsg_table['scan_grid'])

        if self.master_node_subscriber.check_reviced('scan_grid00'):
            scan_grid_notice['scan_grid00'] = {}
            scan_grid_notice['scan_grid00'].update(self.master_node_subscriber.submsg_table['scan_grid00'])

        if self.master_node_subscriber.check_reviced('scan_grid01') and u'scan_grid01' in req_scan_id:
            scan_grid_notice['scan_grid01'] = {}
            scan_grid_notice['scan_grid01'].update(self.master_node_subscriber.submsg_table['scan_grid01'])

        if self.master_node_subscriber.check_reviced('scan_grid02') and u'scan_grid02' in req_scan_id:
            scan_grid_notice['scan_grid02'] = {}
            scan_grid_notice['scan_grid02'].update(self.master_node_subscriber.submsg_table['scan_grid02'])

        if self.master_node_subscriber.check_reviced('scan_grid03') and u'scan_grid03' in req_scan_id:
            scan_grid_notice['scan_grid03'] = {}
            scan_grid_notice['scan_grid03'].update(self.master_node_subscriber.submsg_table['scan_grid03'])

        if self.master_node_subscriber.check_reviced('scan_grid04') and u'scan_grid04' in req_scan_id:
            scan_grid_notice['scan_grid04'] = {}
            scan_grid_notice['scan_grid04'].update(self.master_node_subscriber.submsg_table['scan_grid04'])

        if self.master_node_subscriber.check_reviced('scan_grid05') and u'scan_grid05' in req_scan_id:
            scan_grid_notice['scan_grid05'] = {}
            scan_grid_notice['scan_grid05'].update(self.master_node_subscriber.submsg_table['scan_grid05'])

        if self.master_node_subscriber.check_reviced('scan_grid_lslidar_c32') and u'scan_grid_lslidar_c32' in req_scan_id:
            scan_grid_notice['scan_grid_lslidar_c32'] = {}
            scan_grid_notice['scan_grid_lslidar_c32'].update(self.master_node_subscriber.submsg_table['scan_grid_lslidar_c32'])

        client.write_message(MasterNodeErrorCode.SuccessedData(scan_grid_notice))

    # TCP通信方法
    def send_demo_point_tcp_direct(self, client):
        """
        直接通过TCP发送示教录点数据到NX
        """
        print("[TCP直接发送] 开始处理示教录点数据")
        
        try:
            # 获取请求数据
            request_body = client.request.body
            print("[TCP直接发送] 请求体长度: {}".format(len(request_body) if request_body else 0))
            
            if not request_body:
                print("[TCP直接发送] 请求体为空")
                client.write(MasterNodeErrorCode.ErrorData("请求体为空"))
                return
                
            request_data = json.loads(request_body.decode('utf-8'))
            print("[TCP直接发送] 解析的请求数据: {}".format(request_data))
            
            # 提取参数
            map_id = request_data.get('mapid', 'unknown_map')
            point_id = request_data.get('poseid', '')
            point_type = request_data.get('type', 'demo_point_recorded')
            timestamp = request_data.get('timestamp', '')
            sequence = request_data.get('sequence', 0)
            
            # 确保poseid不为空
            if not point_id or point_id == '' or point_id.strip() == '':
                point_id = 'demo_point_' + str(int(time.time()))
                print("[TCP直接发送] poseid为空，生成默认ID: {}".format(point_id))
            
            print("[TCP直接发送] 收到示教录点数据: mapid={}, poseid={}, type={}".format(map_id, point_id, point_type))
            
            # 构造TCP消息 - 使用前端发送的type字段
            tcp_message = {
                'type': point_type,  # 使用前端发送的type值
                'mapid': map_id,
                'poseid': point_id
            }
            
            print("[TCP直接发送] 构造的TCP消息: {}".format(tcp_message))
            
            # 真正的TCP发送到NX
            print("[TCP直接发送] 示教录点数据接收成功，开始发送到NX")
            
            # 检查机械臂控制器
            if not hasattr(self.master_node_subscriber, 'mechanical_arm_controller'):
                print("[TCP直接发送] 机械臂控制器未初始化")
                client.write(MasterNodeErrorCode.ErrorData("机械臂控制器未初始化"))
                return
                
            # 获取机械臂控制器
            arm_controller = self.master_node_subscriber.mechanical_arm_controller
            print("[TCP直接发送] 机械臂控制器状态: {}".format(arm_controller.get_controller_status()))
            
            # 设置当前消息信息（用于关联返回的数据）
            arm_controller.set_current_message_info(map_id, point_id, timestamp)
            
            # 通过机械臂控制器发送TCP消息
            try:
                success = arm_controller._send_message(tcp_message)
                if success:
                    print("[TCP直接发送] 示教录点数据发送到NX成功")
                    
                    # 创建文件夹结构
                    self._create_demo_point_folders(map_id, point_id)
                    
                    response_data = {
                        "message": "TCP数据发送成功", 
                        "data": tcp_message,
                        "status": "real_success"
                    }
                    client.write(MasterNodeErrorCode.SuccessedData(response_data))
                else:
                    print("[TCP直接发送] 示教录点数据发送到NX失败")
                    client.write(MasterNodeErrorCode.ErrorData("TCP数据发送失败"))
            except Exception as tcp_error:
                print("[TCP直接发送] TCP发送异常: {}".format(str(tcp_error)))
                client.write(MasterNodeErrorCode.ErrorData("TCP发送异常: {}".format(str(tcp_error))))
                
        except json.JSONDecodeError as e:
            print("[TCP直接发送] JSON解析错误: {}".format(str(e)))
            client.write(MasterNodeErrorCode.ErrorData("JSON解析错误: {}".format(str(e))))
        except Exception as e:
            print("[TCP直接发送] 处理示教录点数据时发生错误: {}".format(str(e)))
            import traceback
            traceback.print_exc()
            client.write(MasterNodeErrorCode.ErrorData("处理示教录点数据时发生错误: {}".format(str(e))))

    def send_demo_point_tcp(self, client):
        """
        通过TCP发送示教录点数据到NX（兼容接口）
        """
        try:
            # 获取请求数据
            request_data = json.loads(client.request.body.decode('utf-8'))
            
            # 提取参数
            map_id = request_data.get('mapId', 'unknown_map')
            point_id = request_data.get('pointId', '')
            point_type = request_data.get('type', 'demo_point_recorded')
            timestamp = request_data.get('timestamp', '')
            
            # 确保pointId不为空
            if not point_id or point_id == '':
                point_id = 'demo_point_' + str(int(time.time()))
                print("[TCP发送] pointId为空，生成默认ID: {}".format(point_id))
            
            print("[TCP发送] 收到示教录点数据: mapId={}, pointId={}, type={}".format(map_id, point_id, point_type))
            
            # 构造TCP消息 - 添加type字段并放在最前面
            tcp_message = {
                'type': 'type1',
                'mapid': map_id,
                'poseid': point_id
            }
            
            # 真正的TCP发送到NX
            print("[TCP发送] 示教录点数据接收成功，开始发送到NX")
            print("[TCP发送] TCP消息内容: {}".format(tcp_message))
            
            # 检查机械臂控制器
            if not hasattr(self.master_node_subscriber, 'mechanical_arm_controller'):
                print("[TCP发送] 机械臂控制器未初始化")
                client.write(MasterNodeErrorCode.ErrorData("机械臂控制器未初始化"))
                return
                
            # 获取机械臂控制器
            arm_controller = self.master_node_subscriber.mechanical_arm_controller
            print("[TCP发送] 机械臂控制器状态: {}".format(arm_controller.get_controller_status()))
            
            # 通过机械臂控制器发送TCP消息
            try:
                success = arm_controller._send_message(tcp_message)
                if success:
                    print("[TCP发送] 示教录点数据发送到NX成功")
                    client.write(MasterNodeErrorCode.SuccessedData({
                        "message": "TCP数据发送成功", 
                        "data": tcp_message,
                        "status": "real_success"
                    }))
                else:
                    print("[TCP发送] 示教录点数据发送到NX失败")
                    client.write(MasterNodeErrorCode.ErrorData("TCP数据发送失败"))
            except Exception as tcp_error:
                print("[TCP发送] TCP发送异常: {}".format(str(tcp_error)))
                client.write(MasterNodeErrorCode.ErrorData("TCP发送异常: {}".format(str(tcp_error))))
                
        except Exception as e:
            print("[TCP发送] 处理示教录点数据时发生错误: {}".format(str(e)))
            client.write_message(MasterNodeErrorCode.ErrorData("处理示教录点数据时发生错误: {}".format(str(e))))

    def send_demo_point_via_arm_controller(self, client):
        """
        通过机械臂控制器发送示教录点数据到NX
        """
        try:
            # 获取请求数据
            request_data = json.loads(client.request.body.decode('utf-8'))
            
            # 提取参数
            map_id = request_data.get('mapid', 'unknown_map')
            point_id = request_data.get('poseid', '')
            point_type = request_data.get('type', 'demo_point_recorded')
            timestamp = request_data.get('timestamp', '')
            
            # 确保poseid不为空
            if not point_id or point_id == '':
                point_id = 'demo_point_' + str(int(time.time()))
                print("[机械臂控制器] poseid为空，生成默认ID: {}".format(point_id))
            
            print("[机械臂控制器] 收到示教录点数据: mapid={}, poseid={}, type={}".format(map_id, point_id, point_type))
            
            # 构造TCP消息 - 使用前端发送的type字段
            tcp_message = {
                'type': point_type,  # 使用前端发送的type值
                'mapid': map_id,
                'poseid': point_id
            }
            
            # 真正的TCP发送到NX
            print("[机械臂控制器] 示教录点数据接收成功，开始发送到NX")
            print("[机械臂控制器] TCP消息内容: {}".format(tcp_message))
            
            # 检查机械臂控制器
            if not hasattr(self.master_node_subscriber, 'mechanical_arm_controller'):
                print("[机械臂控制器] 机械臂控制器未初始化")
                client.write(MasterNodeErrorCode.ErrorData("机械臂控制器未初始化"))
                return
                
            # 获取机械臂控制器
            arm_controller = self.master_node_subscriber.mechanical_arm_controller
            print("[机械臂控制器] 机械臂控制器状态: {}".format(arm_controller.get_controller_status()))
            
            # 通过机械臂控制器发送TCP消息
            try:
                success = arm_controller._send_message(tcp_message)
                if success:
                    print("[机械臂控制器] 示教录点数据发送到NX成功")
                    
                    # 创建文件夹结构
                    self._create_demo_point_folders(map_id, point_id)
                    
                    client.write(MasterNodeErrorCode.SuccessedData({
                        "message": "机械臂控制器数据发送成功", 
                        "data": tcp_message,
                        "status": "real_success"
                    }))
                else:
                    print("[机械臂控制器] 示教录点数据发送到NX失败")
                    client.write(MasterNodeErrorCode.ErrorData("机械臂控制器数据发送失败"))
            except Exception as tcp_error:
                print("[机械臂控制器] TCP发送异常: {}".format(str(tcp_error)))
                client.write(MasterNodeErrorCode.ErrorData("TCP发送异常: {}".format(str(tcp_error))))
                
        except Exception as e:
            print("[机械臂控制器] 处理示教录点数据时发生错误: {}".format(str(e)))
            client.write_message(MasterNodeErrorCode.ErrorData("处理示教录点数据时发生错误: {}".format(str(e))))

    def get_mechanical_arm_points_from_demo_files(self, client):
        """
        从示教文件中查询机械臂点name值
        查询路径: /root/Drobot_Navigation_Module/WORKSPACE/src/application/master_node/{map_id}/{pose_id}/*.json
        """
        try:
            import json
            import glob
            import os
            
            # 基础路径
            base_path = "/root/Drobot_Navigation_Module/WORKSPACE/src/application/master_node"
            
            # 获取请求参数
            request_data = json.loads(client.request.body.decode('utf-8'))
            map_id = request_data.get('map_id', '')
            pose_id = request_data.get('pose_id', '')
            
            print("[查询机械臂点] ========== 开始查询机械臂点 ==========")
            print("[查询机械臂点] 收到请求: map_id='{}', pose_id='{}'".format(map_id, pose_id))
            print("[查询机械臂点] map_id类型: {}, pose_id类型: {}".format(type(map_id), type(pose_id)))
            print("[查询机械臂点] map_id长度: {}, pose_id长度: {}".format(len(map_id) if map_id else 0, len(pose_id) if pose_id else 0))
            print("[查询机械臂点] map_id是否为空: {}, pose_id是否为空: {}".format(map_id == '', pose_id == ''))
            
            # 检查基础路径是否存在
            if not os.path.exists(base_path):
                print("[查询机械臂点] 基础路径不存在: {}".format(base_path))
                client.write(MasterNodeErrorCode.SuccessedData({
                    "mechanical_arm_points": [],
                    "total_count": 0,
                    "search_path": base_path,
                    "message": "示教文件目录不存在"
                }))
                return
            
            # 构造查询路径 - 修复查询策略
            search_paths = []
            
            if map_id and pose_id:
                # 只查询特定地图和点位的文件
                specific_path = os.path.join(base_path, map_id, pose_id, "*.json")
                search_paths.append(specific_path)  
                print("[查询机械臂点] 特定路径查询: {}".format(specific_path))
            elif map_id and not pose_id:
                # 有地图ID但没有点位ID，返回空结果
                print("[查询机械臂点] 有地图ID但点位ID为空，返回空结果")
                client.write(MasterNodeErrorCode.SuccessedData({
                    "mechanical_arm_points": [],
                    "total_count": 0
                }))
                return
            else:
                # 查询所有文件
                all_path = os.path.join(base_path, "*", "*", "*.json")
                search_paths.append(all_path)
                print("[查询机械臂点] 全路径查询: {}".format(all_path))
            
            # 根据查询策略获取文件
            json_files = []
            for search_path in search_paths:
                found_files = glob.glob(search_path)
                json_files.extend(found_files)
                print("[查询机械臂点] 路径 {} 找到 {} 个文件".format(search_path, len(found_files)))
            
            # 去重
            json_files = list(set(json_files))
            print("[查询机械臂点] 去重后总共找到 {} 个JSON文件".format(len(json_files)))
            
            # 如果没有找到文件，列出基础路径下的内容
            if len(json_files) == 0:
                print("[查询机械臂点] 未找到JSON文件，列出基础路径内容:")
                try:
                    for root, dirs, files in os.walk(base_path):
                        print("[查询机械臂点] 目录: {}, 文件: {}".format(root, files))
                        if len(files) > 0:  # 只显示前几个目录
                            break
                except Exception as e:
                    print("[查询机械臂点] 列出目录内容时出错: {}".format(str(e)))
            
            mechanical_arm_points = []
            
            print("[查询机械臂点] ========== 开始处理JSON文件 ==========")
            for i, json_file in enumerate(json_files):
                print("[查询机械臂点] 处理文件 {}/{}: {}".format(i+1, len(json_files), json_file))
                try:
                    # 兼容不同Python版本的文件读取方式
                    try:
                        # 尝试使用encoding参数（Python 3+）
                        with open(json_file, 'r', encoding='utf-8') as f:
                            data = json.load(f)
                    except TypeError:
                        # 如果不支持encoding参数，使用兼容方式
                        print("[查询机械臂点] 使用兼容模式读取文件: {}".format(json_file))
                        with open(json_file, 'r') as f:
                            data = json.load(f)
                    
                    print("[查询机械臂点] 文件内容结构: {}".format(list(data.keys()) if isinstance(data, dict) else type(data)))
                    
                    # 检查是否是机械臂点位数据文件
                    if 'points_data' in data and isinstance(data['points_data'], list):
                        print("[查询机械臂点] 文件包含points_data，数量: {}".format(len(data['points_data'])))
                        
                        # 检查文件的地图和点位ID是否匹配请求参数
                        file_map_id = data.get('map_id', '')
                        file_pose_id = data.get('pose_id', '')
                        print("[查询机械臂点] 文件地图ID: '{}', 文件点位ID: '{}'".format(file_map_id, file_pose_id))
                        
                        # 如果提供了map_id和pose_id，只处理匹配的文件
                        if map_id and pose_id:
                            if file_map_id != map_id or file_pose_id != pose_id:
                                print("[查询机械臂点] ✗ 文件不匹配请求参数，跳过")
                                continue
                        elif map_id:
                            if file_map_id != map_id:
                                print("[查询机械臂点] ✗ 文件地图ID不匹配，跳过")
                                continue
                        
                        for j, point in enumerate(data['points_data']):
                            print("[查询机械臂点] 处理点 {}/{}: {}".format(j+1, len(data['points_data']), point))
                            # 支持新格式的label字段和旧格式的name字段
                            point_name = None
                            if 'label' in point:
                                point_name = point['label']
                                print("[查询机械臂点] 使用label字段: {}".format(point_name))
                            elif 'name' in point:
                                point_name = point['name']
                                print("[查询机械臂点] 使用name字段: {}".format(point_name))
                            
                            if point_name:
                                point_info = {
                                    'name': point_name,
                                    'file_path': json_file,
                                    'map_id': file_map_id,
                                    'pose_id': file_pose_id,
                                    'timestamp': data.get('timestamp', '')
                                }
                                mechanical_arm_points.append(point_info)
                                print("[查询机械臂点] ✓ 找到机械臂点: {} (来自文件: {})".format(point_name, json_file))
                            else:
                                print("[查询机械臂点] ✗ 点数据缺少name或label字段: {}".format(point))
                                print("[查询机械臂点] 点数据包含的字段: {}".format(list(point.keys()) if isinstance(point, dict) else '非字典类型'))
                    else:
                        print("[查询机械臂点] ✗ 文件不包含有效的points_data: {}".format(json_file))
                        if isinstance(data, dict):
                            print("[查询机械臂点] 文件包含的字段: {}".format(list(data.keys())))
                    
                except Exception as file_error:
                    print("[查询机械臂点] ✗ 读取文件 {} 时发生错误: {}".format(json_file, str(file_error)))
                    continue
            
            print("[查询机械臂点] ========== 查询结果汇总 ==========")
            print("[查询机械臂点] 总共找到 {} 个机械臂点".format(len(mechanical_arm_points)))
            
            # 输出所有找到的机械臂点详情
            for i, point in enumerate(mechanical_arm_points):
                print("[查询机械臂点] 点 {}/{}: 名称='{}', 文件='{}', 地图='{}', 点位='{}', 时间='{}'".format(
                    i+1, len(mechanical_arm_points), 
                    point['name'], 
                    point['file_path'], 
                    point['map_id'], 
                    point['pose_id'], 
                    point['timestamp']
                ))
            
            # 按文件分组统计
            file_stats = {}
            for point in mechanical_arm_points:
                file_path = point['file_path']
                if file_path not in file_stats:
                    file_stats[file_path] = 0
                file_stats[file_path] += 1
            
            print("[查询机械臂点] 按文件分组统计:")
            for file_path, count in file_stats.items():
                print("[查询机械臂点]   {}: {} 个点".format(file_path, count))
            
            client.write(MasterNodeErrorCode.SuccessedData({
                "mechanical_arm_points": mechanical_arm_points,
                "total_count": len(mechanical_arm_points),
                "search_path": search_path
            }))
            
        except Exception as e:
            print("[查询机械臂点] 发生错误: {}".format(str(e)))
            client.write(MasterNodeErrorCode.ErrorData("查询机械臂点时发生错误: {}".format(str(e))))

    def send_mechanical_arm_command(self, client):
        """
        发送机械臂控制命令到NX
        支持的命令: 1=复位, 2=手动, 3=自动, 4=清除报警
        """
        try:
            # 获取请求数据
            request_data = json.loads(client.request.body.decode('utf-8'))
            
            # 提取参数
            command = request_data.get('command', 0)
            command_type = request_data.get('type', 'mechanical_arm_command')
            timestamp = request_data.get('timestamp', '')
            
            print("[机械臂控制] ========== 收到机械臂控制命令 ==========")
            print("[机械臂控制] 命令类型: {}, 命令值: {}, 时间戳: {}".format(command_type, command, timestamp))
            
            # 验证命令值
            if command not in [1, 2, 3, 4, 5, 6]:
                print("[机械臂控制] ✗ 无效的命令值: {}".format(command))
                client.write(MasterNodeErrorCode.ErrorData("无效的机械臂命令: {}".format(command)))
                return
            
            # 命令名称映射
            command_names = {
                1: "复位",
                2: "上使能", 
                3: "下使能",
                4: "自动模式",
                5: "手动模式",
                6: "清除报警"
            }
            
            command_name = command_names.get(command, "未知命令")
            print("[机械臂控制] 执行命令: {} ({})".format(command_name, command))
            
            # 构造TCP消息
            tcp_message = {
                'type': command_type,
                'command': command,
                'timestamp': timestamp
            }
            
            print("[机械臂控制] TCP消息内容: {}".format(tcp_message))
            
            # 通过机械臂控制器发送TCP消息
            if hasattr(self.master_node_subscriber, 'mechanical_arm_controller') and self.master_node_subscriber.mechanical_arm_controller:
                # 获取控制器状态用于日志记录
                controller_status = self.master_node_subscriber.mechanical_arm_controller.get_controller_status()
                print("[机械臂控制] 控制器状态: {}".format(controller_status))
                
                # 发送命令（内部会自动处理连接检查和重连）
                success = self.master_node_subscriber.mechanical_arm_controller.send_mechanical_arm_command(command)
                if success:
                    print("[机械臂控制] ✓ 命令发送成功: {}".format(command_name))
                    client.write(MasterNodeErrorCode.SuccessedData({
                        "message": "机械臂{}命令已发送".format(command_name),
                        "command": command,
                        "command_name": command_name,
                        "timestamp": timestamp
                    }))
                else:
                    print("[机械臂控制] ✗ 命令发送失败: {}".format(command_name))
                    client.write(MasterNodeErrorCode.ErrorData("机械臂TCP连接异常，无法发送{}命令".format(command_name)))
            else:
                print("[机械臂控制] ✗ 机械臂控制器未初始化")
                client.write(MasterNodeErrorCode.ErrorData("机械臂控制器未初始化"))
                
        except Exception as e:
            print("[机械臂控制] 发生错误: {}".format(str(e)))
            client.write(MasterNodeErrorCode.ErrorData("发送机械臂命令时发生错误: {}".format(str(e))))
                
    def check_mechanical_arm_connection(self, client):
        """
        检查机械臂TCP连接状态
        """
        try:
            print("[机械臂连接检查] ========== 检查机械臂TCP连接状态 ==========")
            
            # 检查机械臂控制器是否存在
            if not hasattr(self.master_node_subscriber, 'mechanical_arm_controller'):
                print("[机械臂连接检查] ✗ 机械臂控制器未初始化")
                client.write(MasterNodeErrorCode.ErrorData("机械臂控制器未初始化"))
                return
            
            # 获取机械臂控制器
            arm_controller = self.master_node_subscriber.mechanical_arm_controller
            
            # 检查连接状态
            connection_status = arm_controller.check_connection_status()
            
            print("[机械臂连接检查] 连接状态: {}".format(connection_status))
            
            if connection_status.get('connection_ok', False):
                print("[机械臂连接检查] ✓ TCP连接正常")
                client.write(MasterNodeErrorCode.SuccessedData({
                    "message": "机械臂TCP连接正常",
                    "status": connection_status,
                    "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                }))
            else:
                print("[机械臂连接检查] ✗ TCP连接异常")
                client.write(MasterNodeErrorCode.ErrorData("机械臂TCP连接异常，请检查NX设备状态"))
                
        except Exception as e:
            print("[机械臂连接检查] 发生错误: {}".format(str(e)))
            client.write(MasterNodeErrorCode.ErrorData("检查机械臂连接状态时发生错误: {}".format(str(e))))

    def send_mechanical_arm_manual(self, client):
        """
        发送机械臂手动模式命令到NX
        """
        try:
            # 获取请求数据
            request_data = json.loads(client.request.body.decode('utf-8'))
            
            # 提取参数
            command = request_data.get('command', 0)
            command_type = request_data.get('type', 'mechanical_arm_manual')
            timestamp = request_data.get('timestamp', '')
            
            print("[机械臂手动模式] ========== 收到机械臂手动模式命令 ==========")
            print("[机械臂手动模式] 命令类型: {}, 命令值: {}, 时间戳: {}".format(command_type, command, timestamp))
            
            # 验证命令值
            if command not in [1, 2, 3, 4, 5, 6]:
                print("[机械臂手动模式] ✗ 无效的命令值: {}".format(command))
                client.write(MasterNodeErrorCode.ErrorData("无效的机械臂命令: {}".format(command)))
                return
            
            # 命令名称映射
            command_names = {
                1: "复位",
                2: "上使能", 
                3: "下使能",
                4: "自动模式",
                5: "手动模式",
                6: "清除报警"
            }
            
            command_name = command_names.get(command, "未知命令")
            print("[机械臂手动模式] 执行命令: {} ({})".format(command_name, command))
            
            # 构造TCP消息
            tcp_message = {
                'type': command_type,
                'command': command,
                'timestamp': timestamp
            }
            
            print("[机械臂手动模式] TCP消息内容: {}".format(tcp_message))
            
            # 通过机械臂控制器发送TCP消息
            if hasattr(self.master_node_subscriber, 'mechanical_arm_controller') and self.master_node_subscriber.mechanical_arm_controller:
                # 检查TCP连接状态
                controller_status = self.master_node_subscriber.mechanical_arm_controller.get_controller_status()
                print("[机械臂手动模式] 控制器状态: {}".format(controller_status))
                
                if not controller_status.get('tcp_connected', False):
                    print("[机械臂手动模式] ✗ TCP连接未建立，尝试重新连接...")
                    # 尝试重新连接
                    reconnect_success = self.master_node_subscriber.mechanical_arm_controller._start_tcp_connection()
                    if not reconnect_success:
                        print("[机械臂手动模式] ✗ 重新连接失败")
                        client.write(MasterNodeErrorCode.ErrorData("机械臂TCP连接未建立，无法发送命令"))
                        return
                    else:
                        print("[机械臂手动模式] ✓ 重新连接成功")
                
                success = self.master_node_subscriber.mechanical_arm_controller.send_mechanical_arm_command(command)
                if success:
                    print("[机械臂手动模式] ✓ 命令发送成功: {}".format(command_name))
                    client.write(MasterNodeErrorCode.SuccessedData({
                        "message": "机械臂{}命令已发送".format(command_name),
                        "command": command,
                        "command_name": command_name,
                        "timestamp": timestamp
                    }))
                else:
                    print("[机械臂手动模式] ✗ 命令发送失败: {}".format(command_name))
                    client.write(MasterNodeErrorCode.ErrorData("机械臂TCP连接异常，无法发送{}命令".format(command_name)))
            else:
                print("[机械臂手动模式] ✗ 机械臂控制器未初始化")
                client.write(MasterNodeErrorCode.ErrorData("机械臂控制器未初始化"))
                
        except Exception as e:
            print("[机械臂手动模式] 发生错误: {}".format(str(e)))
            client.write(MasterNodeErrorCode.ErrorData("发送机械臂命令时发生错误: {}".format(str(e))))

    def send_mechanical_arm_auto(self, client):
        """
        发送机械臂自动模式命令到NX
        """
        try:
            # 获取请求数据
            request_data = json.loads(client.request.body.decode('utf-8'))
            
            # 提取参数
            command = request_data.get('command', 0)
            command_type = request_data.get('type', 'mechanical_arm_auto')
            timestamp = request_data.get('timestamp', '')
            
            print("[机械臂自动模式] ========== 收到机械臂自动模式命令 ==========")
            print("[机械臂自动模式] 命令类型: {}, 命令值: {}, 时间戳: {}".format(command_type, command, timestamp))
            
            # 验证命令值
            if command not in [1, 2, 3, 4, 5, 6]:
                print("[机械臂自动模式] ✗ 无效的命令值: {}".format(command))
                client.write(MasterNodeErrorCode.ErrorData("无效的机械臂命令: {}".format(command)))
                return
            
            # 命令名称映射
            command_names = {
                1: "复位",
                2: "上使能", 
                3: "下使能",
                4: "自动模式",
                5: "手动模式",
                6: "清除报警"
            }
            
            command_name = command_names.get(command, "未知命令")
            print("[机械臂自动模式] 执行命令: {} ({})".format(command_name, command))
            
            # 构造TCP消息
            tcp_message = {
                'type': command_type,
                'command': command,
                'timestamp': timestamp
            }
            
            print("[机械臂自动模式] TCP消息内容: {}".format(tcp_message))
            
            # 通过机械臂控制器发送TCP消息
            if hasattr(self.master_node_subscriber, 'mechanical_arm_controller') and self.master_node_subscriber.mechanical_arm_controller:
                # 获取控制器状态用于日志记录
                controller_status = self.master_node_subscriber.mechanical_arm_controller.get_controller_status()
                print("[机械臂自动模式] 控制器状态: {}".format(controller_status))
                
                # 发送命令（内部会自动处理连接检查和重连）
                success = self.master_node_subscriber.mechanical_arm_controller.send_mechanical_arm_command(command)
                if success:
                    print("[机械臂自动模式] ✓ 命令发送成功: {}".format(command_name))
                    client.write(MasterNodeErrorCode.SuccessedData({
                        "message": "机械臂{}命令已发送".format(command_name),
                        "command": command,
                        "command_name": command_name,
                        "timestamp": timestamp
                    }))
                else:
                    print("[机械臂自动模式] ✗ 命令发送失败: {}".format(command_name))
                    client.write(MasterNodeErrorCode.ErrorData("机械臂TCP连接异常，无法发送{}命令".format(command_name)))
            else:
                print("[机械臂自动模式] ✗ 机械臂控制器未初始化")
                client.write(MasterNodeErrorCode.ErrorData("机械臂控制器未初始化"))
                
        except Exception as e:
            print("[机械臂自动模式] 发生错误: {}".format(str(e)))
            client.write(MasterNodeErrorCode.ErrorData("发送机械臂命令时发生错误: {}".format(str(e))))

    def send_mechanical_arm_clear_alarm(self, client):
        """
        发送机械臂清除报警命令到NX
        """
        try:
            # 获取请求数据
            request_data = json.loads(client.request.body.decode('utf-8'))
            
            # 提取参数
            command = request_data.get('command', 0)
            command_type = request_data.get('type', 'mechanical_arm_clear_alarm')
            timestamp = request_data.get('timestamp', '')
            
            print("[机械臂清除报警] ========== 收到机械臂清除报警命令 ==========")
            print("[机械臂清除报警] 命令类型: {}, 命令值: {}, 时间戳: {}".format(command_type, command, timestamp))
            
            # 验证命令值
            if command not in [1, 2, 3, 4, 5, 6]:
                print("[机械臂清除报警] ✗ 无效的命令值: {}".format(command))
                client.write(MasterNodeErrorCode.ErrorData("无效的机械臂命令: {}".format(command)))
                return
            
            # 命令名称映射
            command_names = {
                1: "复位",
                2: "上使能", 
                3: "下使能",
                4: "自动模式",
                5: "手动模式",
                6: "清除报警"
            }
            
            command_name = command_names.get(command, "未知命令")
            print("[机械臂清除报警] 执行命令: {} ({})".format(command_name, command))
            
            # 构造TCP消息
            tcp_message = {
                'type': command_type,
                'command': command,
                'timestamp': timestamp
            }
            
            print("[机械臂清除报警] TCP消息内容: {}".format(tcp_message))
            
            # 通过机械臂控制器发送TCP消息
            if hasattr(self.master_node_subscriber, 'mechanical_arm_controller') and self.master_node_subscriber.mechanical_arm_controller:
                # 获取控制器状态用于日志记录
                controller_status = self.master_node_subscriber.mechanical_arm_controller.get_controller_status()
                print("[机械臂清除报警] 控制器状态: {}".format(controller_status))
                
                # 发送命令（内部会自动处理连接检查和重连）
                success = self.master_node_subscriber.mechanical_arm_controller.send_mechanical_arm_command(command)
                if success:
                    print("[机械臂清除报警] ✓ 命令发送成功: {}".format(command_name))
                    client.write(MasterNodeErrorCode.SuccessedData({
                        "message": "机械臂{}命令已发送".format(command_name),
                        "command": command,
                        "command_name": command_name,
                        "timestamp": timestamp
                    }))
                else:
                    print("[机械臂清除报警] ✗ 命令发送失败: {}".format(command_name))
                    client.write(MasterNodeErrorCode.ErrorData("机械臂TCP连接异常，无法发送{}命令".format(command_name)))
            else:
                print("[机械臂清除报警] ✗ 机械臂控制器未初始化")
                client.write(MasterNodeErrorCode.ErrorData("机械臂控制器未初始化"))
                
        except Exception as e:
            print("[机械臂清除报警] 发生错误: {}".format(str(e)))
            client.write(MasterNodeErrorCode.ErrorData("发送机械臂命令时发生错误: {}".format(str(e))))

    def send_mechanical_arm_enable(self, client):
        """
        发送机械臂上使能命令到NX
        """
        try:
            # 获取请求数据
            request_data = json.loads(client.request.body.decode('utf-8'))
            
            # 提取参数
            command = request_data.get('command', 0)
            command_type = request_data.get('type', 'mechanical_arm_enable')
            timestamp = request_data.get('timestamp', '')
            
            print("[机械臂上使能] ========== 收到机械臂上使能命令 ==========")
            print("[机械臂上使能] 命令类型: {}, 命令值: {}, 时间戳: {}".format(command_type, command, timestamp))
            
            # 验证命令值
            if command not in [1, 2, 3, 4, 5, 6]:
                print("[机械臂上使能] ✗ 无效的命令值: {}".format(command))
                client.write(MasterNodeErrorCode.ErrorData("无效的机械臂命令: {}".format(command)))
                return
            
            # 命令名称映射
            command_names = {
                1: "复位",
                2: "上使能", 
                3: "下使能",
                4: "自动模式",
                5: "手动模式",
                6: "清除报警"
            }
            
            command_name = command_names.get(command, "未知命令")
            print("[机械臂上使能] 执行命令: {} ({})".format(command_name, command))
            
            # 构造TCP消息
            tcp_message = {
                'type': command_type,
                'command': command,
                'timestamp': timestamp
            }
            
            print("[机械臂上使能] TCP消息内容: {}".format(tcp_message))
            
            # 通过机械臂控制器发送TCP消息
            if hasattr(self.master_node_subscriber, 'mechanical_arm_controller') and self.master_node_subscriber.mechanical_arm_controller:
                # 检查TCP连接状态
                controller_status = self.master_node_subscriber.mechanical_arm_controller.get_controller_status()
                print("[机械臂上使能] 控制器状态: {}".format(controller_status))
                
                if not controller_status.get('tcp_connected', False):
                    print("[机械臂上使能] ✗ TCP连接未建立，尝试重新连接...")
                    # 尝试重新连接
                    reconnect_success = self.master_node_subscriber.mechanical_arm_controller._start_tcp_connection()
                    if not reconnect_success:
                        print("[机械臂上使能] ✗ 重新连接失败")
                        client.write(MasterNodeErrorCode.ErrorData("机械臂TCP连接未建立，无法发送命令"))
                        return
                    else:
                        print("[机械臂上使能] ✓ 重新连接成功")
                
                success = self.master_node_subscriber.mechanical_arm_controller.send_mechanical_arm_command(command)
                if success:
                    print("[机械臂上使能] ✓ 命令发送成功: {}".format(command_name))
                    client.write(MasterNodeErrorCode.SuccessedData({
                        "message": "机械臂{}命令已发送".format(command_name),
                        "command": command,
                        "command_name": command_name,
                        "timestamp": timestamp
                    }))
                else:
                    print("[机械臂上使能] ✗ 命令发送失败: {}".format(command_name))
                    client.write(MasterNodeErrorCode.ErrorData("机械臂TCP连接异常，无法发送{}命令".format(command_name)))
            else:
                print("[机械臂上使能] ✗ 机械臂控制器未初始化")
                client.write(MasterNodeErrorCode.ErrorData("机械臂控制器未初始化"))
                
        except Exception as e:
            print("[机械臂上使能] 发生错误: {}".format(str(e)))
            client.write(MasterNodeErrorCode.ErrorData("发送机械臂命令时发生错误: {}".format(str(e))))

    def send_mechanical_arm_disable(self, client):
        """
        发送机械臂下使能命令到NX
        """
        try:
            # 获取请求数据
            request_data = json.loads(client.request.body.decode('utf-8'))
            
            # 提取参数
            command = request_data.get('command', 0)
            command_type = request_data.get('type', 'mechanical_arm_disable')
            timestamp = request_data.get('timestamp', '')
            
            print("[机械臂下使能] ========== 收到机械臂下使能命令 ==========")
            print("[机械臂下使能] 命令类型: {}, 命令值: {}, 时间戳: {}".format(command_type, command, timestamp))
            
            # 验证命令值
            if command not in [1, 2, 3, 4, 5, 6]:
                print("[机械臂下使能] ✗ 无效的命令值: {}".format(command))
                client.write(MasterNodeErrorCode.ErrorData("无效的机械臂命令: {}".format(command)))
                return
            
            # 命令名称映射
            command_names = {
                1: "复位",
                2: "上使能", 
                3: "下使能",
                4: "自动模式",
                5: "手动模式",
                6: "清除报警"
            }
            
            command_name = command_names.get(command, "未知命令")
            print("[机械臂下使能] 执行命令: {} ({})".format(command_name, command))
            
            # 构造TCP消息
            tcp_message = {
                'type': command_type,
                'command': command,
                'timestamp': timestamp
            }
            
            print("[机械臂下使能] TCP消息内容: {}".format(tcp_message))
            
            # 通过机械臂控制器发送TCP消息
            if hasattr(self.master_node_subscriber, 'mechanical_arm_controller') and self.master_node_subscriber.mechanical_arm_controller:
                # 检查TCP连接状态
                controller_status = self.master_node_subscriber.mechanical_arm_controller.get_controller_status()
                print("[机械臂下使能] 控制器状态: {}".format(controller_status))
                
                if not controller_status.get('tcp_connected', False):
                    print("[机械臂下使能] ✗ TCP连接未建立，尝试重新连接...")
                    # 尝试重新连接
                    reconnect_success = self.master_node_subscriber.mechanical_arm_controller._start_tcp_connection()
                    if not reconnect_success:
                        print("[机械臂下使能] ✗ 重新连接失败")
                        client.write(MasterNodeErrorCode.ErrorData("机械臂TCP连接未建立，无法发送命令"))
                        return
                    else:
                        print("[机械臂下使能] ✓ 重新连接成功")
                
                success = self.master_node_subscriber.mechanical_arm_controller.send_mechanical_arm_command(command)
                if success:
                    print("[机械臂下使能] ✓ 命令发送成功: {}".format(command_name))
                    client.write(MasterNodeErrorCode.SuccessedData({
                        "message": "机械臂{}命令已发送".format(command_name),
                        "command": command,
                        "command_name": command_name,
                        "timestamp": timestamp
                    }))
                else:
                    print("[机械臂下使能] ✗ 命令发送失败: {}".format(command_name))
                    client.write(MasterNodeErrorCode.ErrorData("机械臂TCP连接异常，无法发送{}命令".format(command_name)))
            else:
                print("[机械臂下使能] ✗ 机械臂控制器未初始化")
                client.write(MasterNodeErrorCode.ErrorData("机械臂控制器未初始化"))
                
        except Exception as e:
            print("[机械臂下使能] 发生错误: {}".format(str(e)))
            client.write(MasterNodeErrorCode.ErrorData("发送机械臂命令时发生错误: {}".format(str(e))))

    def _create_demo_point_folders(self, map_id, pose_id):
        """
        创建示教录点文件夹结构
        路径: /root/Drobot Navigation Module/WORKSPACE/src/application/masternode/{map_id}/{pose_id}
        
        Args:
            map_id: 地图ID
            pose_id: 点位ID
        """
        try:
            # 基础路径
            base_path = "/root/Drobot_Navigation_Module/WORKSPACE/src/application/master_node"
            
            # 创建mapid文件夹路径
            map_folder = os.path.join(base_path, map_id)
            
            # 创建poseid文件夹路径
            pose_folder = os.path.join(map_folder, pose_id)
            
            print("[文件夹创建] 开始创建示教录点文件夹结构")
            print("[文件夹创建] 基础路径: {}".format(base_path))
            print("[文件夹创建] 地图文件夹: {}".format(map_folder))
            print("[文件夹创建] 点位文件夹: {}".format(pose_folder))
            
            # 检查当前工作目录和权限
            import pwd
            import stat
            current_user = pwd.getpwuid(os.getuid()).pw_name
            print("[文件夹创建] 当前用户: {}".format(current_user))
            print("[文件夹创建] 当前工作目录: {}".format(os.getcwd()))
            
            # 检查/root目录是否存在和权限
            if os.path.exists("/root"):
                root_stat = os.stat("/root")
                print("[文件夹创建] /root目录存在，权限: {}".format(oct(root_stat.st_mode)))
            else:
                print("[文件夹创建] /root目录不存在")
            
            # 创建基础路径（如果不存在）
            if not os.path.exists(base_path):
                try:
                    os.makedirs(base_path)
                    print("[文件夹创建] 创建基础路径: {}".format(base_path))
                    # 验证是否真的创建成功
                    if os.path.exists(base_path):
                        print("[文件夹创建] 基础路径创建成功并验证存在")
                    else:
                        print("[文件夹创建] 警告：基础路径创建后验证不存在")
                except OSError as e:
                    print("[文件夹创建] 创建基础路径时发生错误: {}".format(str(e)))
                    if e.errno != 17:  # 17 = File exists
                        raise
                    print("[文件夹创建] 基础路径已存在: {}".format(base_path))
            else:
                print("[文件夹创建] 基础路径已存在: {}".format(base_path))
            
            # 创建mapid文件夹
            if not os.path.exists(map_folder):
                try:
                    os.makedirs(map_folder)
                    print("[文件夹创建] 创建地图文件夹: {}".format(map_folder))
                    # 验证是否真的创建成功
                    if os.path.exists(map_folder):
                        print("[文件夹创建] 地图文件夹创建成功并验证存在")
                    else:
                        print("[文件夹创建] 警告：地图文件夹创建后验证不存在")
                except OSError as e:
                    print("[文件夹创建] 创建地图文件夹时发生错误: {}".format(str(e)))
                    if e.errno != 17:  # 17 = File exists
                        raise
                    print("[文件夹创建] 地图文件夹已存在: {}".format(map_folder))
            else:
                print("[文件夹创建] 地图文件夹已存在: {}".format(map_folder))
            
            # 创建poseid文件夹
            if not os.path.exists(pose_folder):
                try:
                    os.makedirs(pose_folder)
                    print("[文件夹创建] 创建点位文件夹: {}".format(pose_folder))
                    # 验证是否真的创建成功
                    if os.path.exists(pose_folder):
                        print("[文件夹创建] 点位文件夹创建成功并验证存在")
                    else:
                        print("[文件夹创建] 警告：点位文件夹创建后验证不存在")
                except OSError as e:
                    print("[文件夹创建] 创建点位文件夹时发生错误: {}".format(str(e)))
                    if e.errno != 17:  # 17 = File exists
                        raise
                    print("[文件夹创建] 点位文件夹已存在: {}".format(pose_folder))
            else:
                print("[文件夹创建] 点位文件夹已存在: {}".format(pose_folder))
            
            # 最终验证整个路径结构
            print("[文件夹创建] 最终验证文件夹结构:")
            print("[文件夹创建] 基础路径存在: {}".format(os.path.exists(base_path)))
            print("[文件夹创建] 地图文件夹存在: {}".format(os.path.exists(map_folder)))
            print("[文件夹创建] 点位文件夹存在: {}".format(os.path.exists(pose_folder)))
            
            if os.path.exists(pose_folder):
                print("[文件夹创建] 示教录点文件夹结构创建完成")
                return True
            else:
                print("[文件夹创建] 示教录点文件夹结构创建失败")
                return False
            
        except Exception as e:
            print("[文件夹创建] 创建示教录点文件夹时发生错误: {}".format(str(e)))
            return False

class WebSocket(WebSocketHandler):
    clients = set()
    send_interval = 550
    # client_idx = 0
    client_idx = 0
    client_map = {}

    def initialize(self,mn):
        self.master_node = mn
        self.clients.add(self)
        cls = self.__class__
        # id = client_idx
        self.client_id = WebSocket.client_idx
        WebSocket.client_idx = WebSocket.client_idx + 1
        WebSocket.client_map[self.client_id] = time.time()

    def open(self):
        rospy.loginfo("WebSocket Client connected" )
        cls = self.__class__
        self.timer_loop = tornado.ioloop.PeriodicCallback(self.check_time,self.send_interval)
        self.timer_loop.start()

    def on_message(self, message):
        value = json.loads(message)
        if "send_interval" in value.keys():
            location_interval = value["send_interval"]
            self.send_interval = location_interval
            self.timer_loop.stop()
            self.timer_loop = tornado.ioloop.PeriodicCallback(self.check_time,self.send_interval)
            self.timer_loop.start()
        pass

    def check_time(self):
        if not self.clients:
            rospy.logwarn("no websocket initialized")
            self.on_close()
        if self in self.clients:
            # WebSocket.client_map[self.client_id] = time.time()
            # rospy.loginfo("sending id %d time %lf", self.client_id, WebSocket.client_map[self.client_id])
            self.master_node.send_websocket_notice(self)

    def on_close(self):
        self.clients.remove(self)
        self.timer_loop.stop()
        del WebSocket.client_map[self.client_id]
        # rospy.loginfo("WebSocket Client disconnected.")

    def check_origin(self, origin):
        return True
    pass

class SensorWebSocket(WebSocketHandler):
    clients = set()
    send_interval = 1000

    def initialize(self,mn):
        self.master_node = mn
        self.req_scan_id = [u"scan_grid", u"scan_grid00"]
        self.send_interval = 1000
        self.clients.add(self)
        self.param = {}
        self.param["req_scan_id"] = self.req_scan_id
        self.param["send_interval"] = self.send_interval

    def open(self):
        cls = self.__class__
        self.timer_loop = tornado.ioloop.PeriodicCallback(self.check_time,self.send_interval)
        self.timer_loop.start()
        rospy.loginfo("Sensor Client conneted")

    def on_message(self, message):
        value = json.loads(message)
        if "send_interval" in value.keys():
            send_interval = value["send_interval"]
            self.send_interval = send_interval
            self.param["send_interval"] = self.send_interval
            self.timer_loop.stop()
            self.timer_loop = tornado.ioloop.PeriodicCallback(self.check_time,self.send_interval)
            self.timer_loop.start()
        if "req_scan_id" in value.keys():
            self.req_scan_id = value["req_scan_id"]
            self.timer_loop.stop()
            self.timer_loop = tornado.ioloop.PeriodicCallback(self.check_time,self.send_interval)
            self.timer_loop.start()
            self.param["req_scan_id"] = self.req_scan_id
        self.write_message(self.param)
        pass

    def check_time(self):
        # rospy.loginfo("sensor client num %d", len(self.clients))
        if not self.clients:
            rospy.logwarn("no websocket initialized")
            self.on_close()
        if self in self.clients:
            self.master_node.send_scan_grid(self, self.req_scan_id)

    def check_origin(self,origin):
        return True

    def on_close(self):
        cls = self.__class__
        self.clients.remove(self)
        MasterNode.sensor_websocket_opened = False
        rospy.loginfo("Sensor Client disconnected.")
        self.timer_loop.stop()
    pass

class NoticeWebSocket(WebSocketHandler):
    clients = set()
    send_interval = 1000

    def initialize(self,mn):
        self.master_node = mn
        self.clients.add(self)

    def open(self):
        rospy.loginfo("NoticeWebSocket Client connected" )
        cls = self.__class__
        self.timer_loop = tornado.ioloop.PeriodicCallback(self.check_time,self.send_interval)
        self.timer_loop.start()

    def on_message(self, message):
        value = json.loads(message)
        if "send_interval" in value.keys():
            location_interval = value["send_interval"]
            self.send_interval = location_interval
            self.timer_loop.stop()
            self.timer_loop = tornado.ioloop.PeriodicCallback(self.check_time,self.send_interval)
            self.timer_loop.start()
        pass

    def check_time(self):
        if not self.clients:
            rospy.logwarn("no websocket initialized")
            self.on_close()
        if self in self.clients:
            self.master_node.send_websocket_notice2(self)

    def on_close(self):
        self.clients.remove(self)
        # rospy.loginfo("WebSocket Client disconnected.")
        self.timer_loop.stop()

    def check_origin(self, origin):
        return True
    pass


def shutdown_hook():
    IOLoop.current().stop()

class IndexHandler(RequestHandler):
    #线城池
    executor = ThreadPoolExecutor(15)
    def initialize(self,mn):
        self.master_node = mn

    # get和post请求调用的是同一个函数
    # get请求
    @tornado.gen.coroutine
    def get(self,operation):
        # print("Master_node.py get xc *********************0001************************", operation)
        self.set_header("Access-Control-Allow-Origin", "*")
        self.set_header("Access-Control-Allow-Methods", "PUT, POST, GET, OPTIONS, DELETE")
        self.set_header("Access-Control-Allow-Headers", "Content-Type, Authorization, X-Requested-With")
        if self.master_node.parse_operation(operation):
            #print("we have this operation")
            # self.master_node.service_lookup_table[operation](self)
            yield self.runoperation(operation)
        else :
            if operation != "":
                rospy.logwarn("[Master_node.py] Didn't found the operation:%s",operation)

    # post请求
    @tornado.gen.coroutine
    def post(self,operation):
        # print("Master_node.py post xc *********************0001************************", operation)
        self.set_header("Access-Control-Allow-Origin", "*")
        self.set_header("Access-Control-Allow-Methods", "PUT, POST, GET, OPTIONS, DELETE")
        self.set_header("Access-Control-Allow-Headers", "Content-Type, Authorization, X-Requested-With")
        if self.master_node.parse_operation(operation):
            #print("we have this operation")
            # self.master_node.service_lookup_table[operation](self)
            yield self.runoperation(operation)
        else :
            if operation != "":
                rospy.logwarn("[Master_node.py] Didn't found the operation:%s",operation)

    @tornado.gen.coroutine
    def options(self,operation):
        # print("Master_node.py options xc *********************0001************************", operation)
        self.set_header("Access-Control-Allow-Origin", "*")
        self.set_header("Access-Control-Allow-Methods", "PUT, POST, GET, OPTIONS, DELETE")
        self.set_header("Access-Control-Allow-Headers", "Content-Type, Authorization, X-Requested-With")
        self.set_header("Access-Control-Max-Age", "86400")  # 24小时缓存
        # OPTIONS请求不需要执行具体操作，直接返回成功
        self.write("")
        self.finish()

    @run_on_executor
    def runoperation(self,operation):
        # print("Master_node.py runoperation xc *********************0003************************", operation)
        op_type = operation.split('/')[0]
        if not self.master_node.authorized_state and op_type != "cmd" and op_type != "sensor_data":
          return self.master_node.authorized_state_cb(self)
        self.master_node.service_lookup_table[operation](self)

    def send_mechanical_arm_auto(self, client):
        """
        发送机械臂自动模式命令到NX
        """
        try:
            # 获取请求数据
            request_data = json.loads(client.request.body.decode('utf-8'))
            
            # 提取参数
            command = request_data.get('command', 0)
            command_type = request_data.get('type', 'mechanical_arm_auto')
            timestamp = request_data.get('timestamp', '')
            
            print("[机械臂自动模式] ========== 收到机械臂自动模式命令 ==========")
            print("[机械臂自动模式] 命令类型: {}, 命令值: {}, 时间戳: {}".format(command_type, command, timestamp))
            
            # 验证命令值
            if command not in [1, 2, 3, 4, 5, 6]:
                print("[机械臂自动模式] ✗ 无效的命令值: {}".format(command))
                client.write(MasterNodeErrorCode.ErrorData("无效的机械臂命令: {}".format(command)))
                return
            
            # 命令名称映射
            command_names = {
                1: "复位",
                2: "上使能", 
                3: "下使能",
                4: "自动模式",
                5: "手动模式",
                6: "清除报警"
            }
            
            command_name = command_names.get(command, "未知命令")
            print("[机械臂自动模式] 执行命令: {} ({})".format(command_name, command))
            
            # 构造TCP消息
            tcp_message = {
                'type': command_type,
                'command': command,
                'timestamp': timestamp
            }
            
            print("[机械臂自动模式] TCP消息内容: {}".format(tcp_message))
            
            # 通过机械臂控制器发送TCP消息
            if hasattr(self.master_node_subscriber, 'mechanical_arm_controller') and self.master_node_subscriber.mechanical_arm_controller:
                # 获取控制器状态用于日志记录
                controller_status = self.master_node_subscriber.mechanical_arm_controller.get_controller_status()
                print("[机械臂自动模式] 控制器状态: {}".format(controller_status))
                
                # 发送命令（内部会自动处理连接检查和重连）
                success = self.master_node_subscriber.mechanical_arm_controller.send_mechanical_arm_command(command)
                if success:
                    print("[机械臂自动模式] ✓ 命令发送成功: {}".format(command_name))
                    client.write(MasterNodeErrorCode.SuccessedData({
                        "message": "机械臂{}命令已发送".format(command_name),
                        "command": command,
                        "command_name": command_name,
                        "timestamp": timestamp
                    }))
                else:
                    print("[机械臂自动模式] ✗ 命令发送失败: {}".format(command_name))
                    client.write(MasterNodeErrorCode.ErrorData("机械臂TCP连接异常，无法发送{}命令".format(command_name)))
            else:
                print("[机械臂自动模式] ✗ 机械臂控制器未初始化")
                client.write(MasterNodeErrorCode.ErrorData("机械臂控制器未初始化"))
                
        except Exception as e:
            print("[机械臂自动模式] 发生错误: {}".format(str(e)))
            client.write(MasterNodeErrorCode.ErrorData("发送机械臂命令时发生错误: {}".format(str(e))))

    def send_mechanical_arm_clear_alarm(self, client):
        """
        发送机械臂清除报警命令到NX
        """
        try:
            # 获取请求数据
            request_data = json.loads(client.request.body.decode('utf-8'))
            
            # 提取参数
            command = request_data.get('command', 0)
            command_type = request_data.get('type', 'mechanical_arm_clear_alarm')
            timestamp = request_data.get('timestamp', '')
            
            print("[机械臂清除报警] ========== 收到机械臂清除报警命令 ==========")
            print("[机械臂清除报警] 命令类型: {}, 命令值: {}, 时间戳: {}".format(command_type, command, timestamp))
            
            # 验证命令值
            if command not in [1, 2, 3, 4, 5, 6]:
                print("[机械臂清除报警] ✗ 无效的命令值: {}".format(command))
                client.write(MasterNodeErrorCode.ErrorData("无效的机械臂命令: {}".format(command)))
                return
            
            # 命令名称映射
            command_names = {
                1: "复位",
                2: "上使能", 
                3: "下使能",
                4: "自动模式",
                5: "手动模式",
                6: "清除报警"
            }
            
            command_name = command_names.get(command, "未知命令")
            print("[机械臂清除报警] 执行命令: {} ({})".format(command_name, command))
            
            # 构造TCP消息
            tcp_message = {
                'type': command_type,
                'command': command,
                'timestamp': timestamp
            }
            
            print("[机械臂清除报警] TCP消息内容: {}".format(tcp_message))
            
            # 通过机械臂控制器发送TCP消息
            if hasattr(self.master_node_subscriber, 'mechanical_arm_controller') and self.master_node_subscriber.mechanical_arm_controller:
                # 获取控制器状态用于日志记录
                controller_status = self.master_node_subscriber.mechanical_arm_controller.get_controller_status()
                print("[机械臂清除报警] 控制器状态: {}".format(controller_status))
                
                # 发送命令（内部会自动处理连接检查和重连）
                success = self.master_node_subscriber.mechanical_arm_controller.send_mechanical_arm_command(command)
                if success:
                    print("[机械臂清除报警] ✓ 命令发送成功: {}".format(command_name))
                    client.write(MasterNodeErrorCode.SuccessedData({
                        "message": "机械臂{}命令已发送".format(command_name),
                        "command": command,
                        "command_name": command_name,
                        "timestamp": timestamp
                    }))
                else:
                    print("[机械臂清除报警] ✗ 命令发送失败: {}".format(command_name))
                    client.write(MasterNodeErrorCode.ErrorData("机械臂TCP连接异常，无法发送{}命令".format(command_name)))
            else:
                print("[机械臂清除报警] ✗ 机械臂控制器未初始化")
                client.write(MasterNodeErrorCode.ErrorData("机械臂控制器未初始化"))
                
        except Exception as e:
            print("[机械臂清除报警] 发生错误: {}".format(str(e)))
            client.write(MasterNodeErrorCode.ErrorData("发送机械臂命令时发生错误: {}".format(str(e))))
