#!/usr/bin/env python
#coding=utf-8

import rospy
import time
import math as math
import os
import json
import sys

def safe_json_loads(json_str):
    """
    安全地解析JSON字符串，处理编码问题
    
    Args:
        json_str: 要解析的JSON字符串
        
    Returns:
        解析后的JSON对象
    """
    try:
        if isinstance(json_str, str):
            # 确保字符串是UTF-8编码的
            json_str = json_str.encode('utf-8').decode('utf-8')
        return json.loads(json_str)
    except (UnicodeDecodeError, UnicodeEncodeError) as e:
        rospy.logerr("JSON编码错误: %s", safe_str(e))
        # 尝试使用latin-1编码作为备选
        try:
            if isinstance(json_str, str):
                json_str = json_str.encode('latin-1').decode('utf-8')
            return json.loads(json_str)
        except Exception as e2:
            rospy.logerr("JSON解析完全失败: %s", safe_str(e2))
            return None
    except Exception as e:
        rospy.logerr("JSON解析错误: %s", safe_str(e))
        return None

def safe_str(e):
    """
    安全地转换异常为字符串，避免编码错误
    
    Args:
        e: 异常对象
        
    Returns:
        str: 安全的字符串表示
    """
    try:
        return str(e)
    except UnicodeError:
        return repr(e)
import common_service.srv
from enum import Enum,unique
from xiaolv_msgs.msg import Chassis_odometry,Battery,Meteorological_sensor,Output,Input,Dr210_ultrasonic, Dr210_state,Pelco_state,Gps,Traffic_light_state,drobot_chassis_state,cleaning_state
import xiaolv_msgs
from drobot_manager_console.msg import LaserScanGrid,RobotPoseGrid
from nav_msgs.msg import Path, OccupancyGrid,Odometry
from common_service.msg import mapInfo,GPSGrid,LocalizationScore,CommonEvent
from common_service.srv import *
from drobot_manager_console.msg import NavNotice
from cartographer_ros_msgs.msg import SparseRealTimeMap
from sensor_msgs.msg import LaserScan ,Imu
from sensor_msgs.msg import Range as SensorRange
import std_msgs
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from tf import TransformListener
import tf
import tf2_ros
from geometry_msgs.msg import Quaternion,PolygonStamped
import dr_nav_services.msg as drobot_nav_service
from geometry_msgs.msg import PointStamped ,PoseStamped
import master_node.coordinates as helper
from datetime import datetime
import master_node.mechanical_arm_controller as mechanical_arm_controller
import json
import drobot_manager_console.srv as console_service
from drobot_manager_console.srv import MechanicalArmAction, MechanicalArmActionRequest, MechanicalArmActionResponse

class MasterNodeSubscriber():
    sub_table = {}
    submsg_table={}
    submsg_recived_table={}
    cur_time_table = {}
    ultrasound_size = 10
    ultrasound_map ={}
    irsensor_size = 4
    irsensor_map ={}
    # websocket是否打开的变量
    sensor_websocket_opened = False
    nav_websocket_opened = False
    device_websocket_opened = False
    task_state_received = False

    env_dist = os.environ
    run_workspace_dir = env_dist.get('DROBOT_RUNTIME_DIR')
    locationJson = run_workspace_dir + '/latest_position.json'
    drobot_param_dir = env_dist.get('DROBOT_PARAM_DIR')
    ultrasonic_frames=[]
    nav_notice = {}
    device_notice = {}
    sensor_scan_grid_list = []

    # 构造函数，定义类的时候会自动引用
    def __init__(self, mn):
        print("master_node_subscriber.py xc *********************0000************************")
        # 监听的topic
        self.master_node = mn
        self.TfListener = TransformListener()
        
        # 初始化机械臂控制器，使用正确的NX IP地址
        self.mechanical_arm_controller = mechanical_arm_controller.MechanicalArmController(nx_host="192.168.2.103", nx_port=8888)
        self.create_subscriber("velocity","wheel_car_like",Chassis_odometry,self.vel_cb) #1
        self.create_subscriber("raw_scan","scan", LaserScan, self.laser_scan_cb) #2
        self.create_subscriber("scan_grid","scan_grid",LaserScanGrid, self.laser_scan_grid_cb) #3
        self.create_subscriber_args("scan_grid00","scan_grid00",LaserScanGrid,self.laser_scan_grid2d_cb,"scan_grid00")
        print("master_node_subscriber.py xc *********************0001************************")

        sensor_check_file = self.drobot_param_dir + '/sensor_check.json'
        if os.path.exists(sensor_check_file):
            with open(sensor_check_file,'r') as load_f:
                sensor_check_json = json.load(load_f)
            if sensor_check_json.has_key("laser_sensors") and len(sensor_check_json["laser_sensors"]) > 0:
                for laser_sensor in sensor_check_json["laser_sensors"]:
                    if laser_sensor.has_key("publish_topic_name"):
                        self.sensor_scan_grid_list.append(laser_sensor["publish_topic_name"])
                        self.create_subscriber_args(laser_sensor["publish_topic_name"],laser_sensor["publish_topic_name"],LaserScanGrid,self.laser_scan_grid2d_cb,laser_sensor["publish_topic_name"])

        self.create_subscriber("robot_pose_grid","robot_pose_grid",RobotPoseGrid,self.nav_robot_pose_cb) #4
        self.create_subscriber("raw_odom","/odom_wheel",Odometry, self.raw_odom_cb) #5
        self.create_subscriber("bt_navigator_status","/bt_navigator_node/status", std_msgs.msg.UInt32, self.bt_navigator_status_cb) #6
        self.create_subscriber("follow_path_progress","/move_base/follow_base/status", drobot_nav_service.KeyValueArray, self.follow_path_progress_cb)
        self.create_subscriber("bt_navigator_progress","/bt_navigator_node/DWALocalPlanner/status", drobot_nav_service.KeyValueArray, self.bt_progress_cb)
        now_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.notice_time = datetime.strptime(now_time, r"%Y-%m-%d %H:%M:%S")
        self.create_subscriber("navigation_notice", "/navigation_notice", NavNotice, self.navigation_notice_cb)
        # self.create_subscriber("task_state","drobot_task_manager/task_state",TaskState,self.task_state_cb) #7
        self.create_subscriber("battery","/chassis_battery",Battery,self.battery_cb) #8
        self.create_subscriber("meteorological","/chassis_meteorological_sensor",Meteorological_sensor,self.meteorology_cb) #9
        self.create_subscriber("sparse_map","sparse_map",SparseRealTimeMap,self.SparseMapCallback,queuesize = 5) #10
        self.create_subscriber("map_info","/map_info",mapInfo,self.map_info_cb,queuesize = 2) #11
        # self.create_subscriber("footprint","/bt_navigator_node/global_costmap/footprint_master",PolygonStamped,self.footprint_cb,queuesize = 5) #12
        self.create_subscriber("path","/bt_navigator_node/DWALocalPlanner/fixpattern_path",Path,self.path_cb,1) #13
        self.create_subscriber("localization_status","/localization_bit", std_msgs.msg.Int8, self.localization_status_cb) #14
        self.create_subscriber("localization_score","/localization_score",LocalizationScore,self.localization_score_cb)
        self.create_subscriber("bumper","/chassis_bumper",Input,self.bumper_cb) #15
        self.create_subscriber("chassis_state","/drobot_chassis_state",drobot_chassis_state,self.chassis_state_cb) #16
        self.create_subscriber("ptz_state","/chassis_pelco_state",Pelco_state,self.ptz_state_cb) #17
        self.create_subscriber("raw_imu","/drobot_imu",Imu,self.imu_cb)
        self.create_subscriber("gps","/gps_pub",Gps,self.gps_cb)
        self.create_subscriber("gps_grid","/gps_grid",GPSGrid,self.gps_grid_cb)
        self.create_subscriber("traffic_light_state","/chassis_traffic_light_state",Traffic_light_state,self.traffic_light_state_cb,queuesize = 10)
        self.create_subscriber("dongle_state","/program_running_status", std_msgs.msg.Int8,self.dongle_state_cb,queuesize = 10)
        self.create_subscriber("move_flag","/move_flag",std_msgs.msg.Int8, self.move_flag_cb, queuesize = 10)
        self.create_subscriber("auto_mode","/drive_mode_status",std_msgs.msg.Int8, self.drive_mode_status_cb, queuesize = 10)
        self.create_subscriber("clean_state","/sweep_state",cleaning_state, self.cleaning_state_cb, queuesize = 5)
        self.create_subscriber("washer_devices_state","/cmd_clean_expand",std_msgs.msg.Int16, self.washer_devices_state_cb, queuesize = 5)
        
        # 创建机械臂动作话题订阅
        self.create_subscriber("mechanical_arm_action_request","/mechanical_arm/action_request",std_msgs.msg.String,self.mechanical_arm_action_cb)
        
        # 创建机械臂动作服务
        self.mechanical_arm_service = rospy.Service('/mechanical_arm/execute_action', MechanicalArmAction, self.handle_mechanical_arm_action)
        print("机械臂动作服务已注册: /mechanical_arm/execute_action")
        exception_event_arr = ["sensor_exception_event", "chassis_exception_event"]
        self.create_subscriber("sensor_exception_event","/sensor_exception_event",CommonEvent, self.sensor_exception_cb, queuesize = 5)
        self.create_subscriber("chassis_exception_event","/sensor_exception_event",CommonEvent, self.sensor_exception_cb, queuesize = 5)
        for index in range(0,self.ultrasound_size):
          ultrasound_table_name = "ultrasound"+str(index)
          ultrasound_topic_name = "/chassis_" + ultrasound_table_name
          self.ultrasound_map[ultrasound_topic_name] = ultrasound_table_name
          self.create_subscriber(ultrasound_table_name,ultrasound_topic_name,SensorRange,self.ultrasonic_cb,queuesize = 10)
        for index in range(0,self.irsensor_size):
          irsensor_table_name = "fallprevention"+str(index)
          irsensor_topic_name = "/chassis_" + irsensor_table_name
          self.irsensor_map[irsensor_topic_name] = irsensor_table_name
          self.create_subscriber(irsensor_table_name,irsensor_topic_name,SensorRange,self.irsensor_cb,queuesize = 10)

        self.device_notice["battery"] = 0.0
        self.device_notice["battery_voltage"] = 0.0
        self.device_notice["battery_current"] = 0.0
        self.device_notice["emergency_stop"] = False
        self.device_notice["bumper_front"] = False
        self.device_notice["bumper_back"] = False
        self.device_notice["current_speed"] = 0.0
        self.submsg_table['navigation_notice'] = {}
        self.submsg_table['last_navigation_notice'] = {}
        self.submsg_table['navigation_notice']["nav_status"] = []
        self.submsg_table['bt_navigator_status']['nav_status_type'] = []
        self.laser_scan_grid2d_msg = {}
        rospy.loginfo('[master_node_subscriber.py ] run master_node_subscriber')
    # def getMemCache(self):
    #     memsize={}
    #     for msgs in self.submsg_recived_table:
    #       memsize[msgs.key] = sys.getsizeof(msgs)
    #     print memsize
    def getRangeMsgGrid(self,sensor_range,frame_id):
        point_in = PoseStamped()
        origin_point = PoseStamped()
        out_point = PoseStamped()
        point_in.header.stamp = sensor_range.header.stamp
        point_in.header.frame_id = sensor_range.header.frame_id
        try:
          self.TfListener.waitForTransform(frame_id, sensor_range.header.frame_id, sensor_range.header.stamp,rospy.Duration(1.0))
          origin_point=self.TfListener.transformPose(frame_id, point_in)
          point_in.pose.position.x = sensor_range.range
          out_point=self.TfListener.transformPose(frame_id, point_in)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException,tf2_ros.TransformException) as ex:
          #rospy.logwarn_throttle(60,"unable look up transform of frame_id:%s - frame_id:%s",frame_id, sensor_range.header.frame_id)
          pass
        return origin_point,out_point
    def get_subscribe_msgs(self,table_name):
        if not self.check_reviced(table_name):
          raise Exception("Unrecived message")
        return self.submsg_recived_table[table_name]
    #该函数创建一个订阅者,同时初始化返回的消息
    def create_subscriber(self,table_name,topic_name,topic_type,topic_callback,queuesize=30):
        self.sub_table[table_name] = rospy.Subscriber(topic_name,topic_type,topic_callback,queue_size = queuesize )
        self.submsg_table[table_name] = {}
        self.submsg_recived_table[table_name] = False
        pass

    def create_subscriber_args(self,table_name,topic_name,topic_type,topic_callback,args,queuesize=30):
        self.sub_table[table_name] = rospy.Subscriber(topic_name,topic_type,topic_callback,(args),queue_size = queuesize )
        self.submsg_table[table_name] = {}
        self.submsg_recived_table[table_name] = False
        pass

    def check_reviced(self,table_name):
        if not table_name in self.submsg_recived_table:
          return False
        if self.submsg_recived_table[table_name] == None or self.submsg_recived_table[table_name] == False:
          return False
        return True
    def stop_sound(self,name):
        try:
            stopsound_req = StopSoundRequest()
            stopsound_req.sound = name
            stopsound_req.language = "eng"
            stop_sound_srv = rospy.ServiceProxy("/sound/stop_sound",StopSound)
            resp = stop_sound_srv(stopsound_req)
        except rospy.ServiceException as servexc:
            print("Service did not process request: " + str(servexc))
        except rospy.TypeError as typeexc:
            print("type error" + str(typeexc))
        except rospy.ROSSerializationException as serrialexc:
            print("serial error" + str(serrialexc))
        pass
    def battery_cb(self,data):
        if self.submsg_recived_table['battery'] == False:
            self.submsg_recived_table['battery'] = True
        battery_msg={}
        battery_msg['battery_voltage'] = round(float(data.battery_voltage), 2)
        battery_msg['battery_current'] = round(float(data.battery_current), 2)
        battery_msg['battery_capacity_percentage'] = round(float(data.battery_capacity_percentage), 2)
        battery_msg['battery_temperature'] = round(float(data.battery_temperature), 2)
        battery_msg['battery_power'] = 0
        battery_msg['battery_24voltage'] = 0
        battery_msg['battery_24current'] = 0
        battery_msg['battery_12voltage'] = 0
        battery_msg['battery_12current'] = 0
        self.device_notice["battery"] = round(float(data.battery_capacity_percentage), 2)
        self.device_notice["battery_voltage"] = round(float(data.battery_voltage), 2)
        self.device_notice["battery_current"] = round(float(data.battery_current), 2)
        self.device_notice["battery_temperature"] = round(float(data.battery_temperature), 2)
        self.submsg_table['battery'] = battery_msg
        pass
    def meteorology_cb(self,data):
        if self.submsg_recived_table['meteorological'] == False:
            self.submsg_recived_table['meteorological'] = True
        meteorology_msg = {}
        meteorology_msg['air_temperature'] = data.air_temperature
        meteorology_msg['air_hiumidity'] = data.air_hiumidity
        meteorology_msg['NO2'] = data.NO2
        meteorology_msg['CH4'] = data.CH4
        meteorology_msg['CO'] = data.CO
        meteorology_msg['pm1dot0'] = data.pm1dot0
        meteorology_msg['pm2dot5'] = data.pm2dot5
        meteorology_msg['pm10'] = data.pm10
        meteorology_msg['H2S'] = data.H2S
        meteorology_msg['SO2'] = data.SO2
        meteorology_msg['NO'] = data.NO
        meteorology_msg['enc_illumination'] = data.enc_illumination
        meteorology_msg['enc_noise'] = data.enc_noise
        self.submsg_table['meteorological'] = meteorology_msg
        pass
    def vel_cb(self, data):
        if self.submsg_recived_table['velocity'] == False:
            self.submsg_recived_table['velocity'] = True
        device_vel_msg = {}
        device_vel_msg['device_type'] = "motor_vel"
        device_vel_msg['device_id'] = "1"
        device_vel_msg['device_data'] = {}
        # linear_vel --- x轴速度   linear_y --- y轴速度    angular_rotate --- 角速度 左正右负
        device_vel_msg['device_data']['linear_vel'] = round(data.linear_vel, 2)
        self.device_notice["current_speed"] = round(data.linear_vel, 2)
        device_vel_msg['device_data']['linear_y'] = round(data.linear_y, 0)
        chassis_type =rospy.get_param("/drobot_device_node/ChassisType",4)
        if chassis_type == 4 or chassis_type == 7 or chassis_type == 15 or chassis_type == 17 or chassis_type == 20 or chassis_type == 21:
          device_vel_msg['device_data']['angular_rotate'] = round(data.odometry_rotate, 2)
        else :
          device_vel_msg['device_data']['angular_rotate'] = round(data.angular_rotate, 2)
        self.submsg_table['velocity'] = device_vel_msg
        # push_device_message(json.dumps(device_vel_msg))
        pass

    def laser_scan_grid_cb(self,data):
        if self.submsg_recived_table['scan_grid'] == False:
            self.submsg_recived_table['scan_grid'] = True

            self.laser_scan_grid_msg = {}
            self.laser_scan_grid_msg["sensor_type"] = "laser_scan_grid"
            self.laser_scan_grid_msg["sensor_id"] = "1"
            self.laser_scan_grid_msg["sensor_data"] = {}
            self.laser_scan_grid_msg["sensor_data"]['header'] = {}
            # self.laser_scan_grid_msg["sensor_data"]['header']['seq'] = data.header.seq
            stamp_time = str(data.header.stamp.secs) + str('.') + str(data.header.stamp.nsecs)
            self.laser_scan_grid_msg["sensor_data"]['header']['stamp'] = round(float(stamp_time), 2)
            self.laser_scan_grid_msg["sensor_data"]['header']['frame_id'] = data.header.frame_id
            # self.laser_scan_grid_msg["sensor_data"]['angle_min'] = round(data.angle_min, 2)
            # self.laser_scan_grid_msg['sensor_data']['angle_max'] = round(data.angle_max, 2)
            # self.laser_scan_grid_msg['sensor_data']['angle_increment'] = round(data.angle_increment, 2)
            # self.laser_scan_grid_msg['sensor_data']['time_increment'] = data.time_increment
            # self.laser_scan_grid_msg['sensor_data']['scan_time'] = data.scan_time
            # self.laser_scan_grid_msg['sensor_data']['range_min'] = round(data.range_min, 2)
            # self.laser_scan_grid_msg['sensor_data']['range_max'] = round(data.range_max, 2)
            self.laser_scan_grid_msg['sensor_data']['points'] = []
            # sample_index = 0
            # for point in data.points:
            #     if sample_index % 2 == 0:
            #         p = {}
            #         p['x'] = round(point.x, 2)
            #         p['y'] = round(point.y, 2)
            #         p['z'] = 0.0
            #         self.laser_scan_grid_msg['sensor_data']['points'].append(p)
            #     sample_index = sample_index + 1
            self.laser_scan_grid_msg['sensor_data']['gridpoint'] = []
            # sample_index = 0
            # for gridpoint in data.gridpoints:
            #     if sample_index % 2 == 0:
            #         p = {}
            #         p['x'] = int(gridpoint.x)
            #         p['y'] = int(gridpoint.y)
            #         self.laser_scan_grid_msg['sensor_data']['gridpoint'].append(p)
            #     sample_index = sample_index + 1
            self.laser_scan_grid_msg['sensor_data']['intensities'] = data.intensities
            self.laser_scan_grid_msg['sensor_data']['mapinfo'] = {}
            self.laser_scan_grid_msg['sensor_data']['mapinfo']['mapname'] = data.mapinfo.mapname
            self.laser_scan_grid_msg['sensor_data']['mapinfo']['gridWidth'] = data.mapinfo.gridWidth
            self.laser_scan_grid_msg['sensor_data']['mapinfo']['gridHeight'] = data.mapinfo.gridHeight
            self.laser_scan_grid_msg['sensor_data']['mapinfo']['originX'] = data.mapinfo.originX
            self.laser_scan_grid_msg['sensor_data']['mapinfo']['originY'] = data.mapinfo.originY
            self.laser_scan_grid_msg['sensor_data']['mapinfo']['resolution'] = data.mapinfo.resolution
            self.submsg_table['scan_grid'] = self.laser_scan_grid_msg

        else:
            stamp_time = str(data.header.stamp.secs) + str('.') + str(data.header.stamp.nsecs)
            if (float(stamp_time) - float(self.laser_scan_grid_msg['sensor_data']['header']['stamp'])) < 1.0:
                pass

            self.laser_scan_grid_msg["sensor_type"] = "laser_scan_grid"
            self.laser_scan_grid_msg["sensor_id"] = "1"
            # self.laser_scan_grid_msg["sensor_data"]['header']['seq'] = data.header.seq
            self.laser_scan_grid_msg["sensor_data"]['header']['stamp'] = round(float(stamp_time), 2)
            self.laser_scan_grid_msg["sensor_data"]['header']['frame_id'] = data.header.frame_id
            # self.laser_scan_grid_msg["sensor_data"]['angle_min'] = round(data.angle_min, 2)
            # self.laser_scan_grid_msg['sensor_data']['angle_max'] = round(data.angle_max, 2)
            # self.laser_scan_grid_msg['sensor_data']['angle_increment'] = round(data.angle_increment, 2)
            # self.laser_scan_grid_msg['sensor_data']['time_increment'] = data.time_increment
            # self.laser_scan_grid_msg['sensor_data']['scan_time'] = data.scan_time
            # self.laser_scan_grid_msg['sensor_data']['range_min'] = round(data.range_min, 2)
            # self.laser_scan_grid_msg['sensor_data']['range_max'] = round(data.range_max, 2)
            self.laser_scan_grid_msg['sensor_data']['points'] = []
            # sample_index = 0
            # for point in data.points:
            #     if sample_index % 2 == 0:
            #         p = {}
            #         p['x'] = round(point.x, 2)
            #         p['y'] = round(point.y, 2)
            #         p['z'] = 0
            #         self.laser_scan_grid_msg['sensor_data']['points'].append(p)
            #     sample_index = sample_index + 1
            self.laser_scan_grid_msg['sensor_data']['gridpoint'] = []
            # sample_index = 0
            # for gridpoint in data.gridpoints:
            #     if sample_index % 2 == 0:
            #         p = {}
            #         p['x'] = int(gridpoint.x)
            #         p['y'] = int(gridpoint.y)
            #         self.laser_scan_grid_msg['sensor_data']['gridpoint'].append(p)
            #     sample_index = sample_index + 1
            self.laser_scan_grid_msg['sensor_data']['intensities'] = data.intensities
            self.laser_scan_grid_msg['sensor_data']['mapinfo'] = {}
            self.laser_scan_grid_msg['sensor_data']['mapinfo']['mapname'] = data.mapinfo.mapname
            self.laser_scan_grid_msg['sensor_data']['mapinfo']['gridWidth'] = data.mapinfo.gridWidth
            self.laser_scan_grid_msg['sensor_data']['mapinfo']['gridHeight'] = data.mapinfo.gridHeight
            self.laser_scan_grid_msg['sensor_data']['mapinfo']['originX'] = data.mapinfo.originX
            self.laser_scan_grid_msg['sensor_data']['mapinfo']['originY'] = data.mapinfo.originY
            self.laser_scan_grid_msg['sensor_data']['mapinfo']['resolution'] = data.mapinfo.resolution
            self.submsg_table['scan_grid'] = self.laser_scan_grid_msg
        pass

    def laser_scan_grid2d_cb(self,data,name):
        sample_step = 5
        if name == "scan_grid00":
            sample_step = 2
        else:
            sample_step = 1
        if self.submsg_recived_table[name] == False:
            self.submsg_recived_table[name] = True

            self.laser_scan_grid2d_msg[name] = {}
            self.laser_scan_grid2d_msg[name]['header'] = {}
            # self.laser_scan_grid2d_msg[name]['header']['seq'] = data.header.seq
            self.laser_scan_grid2d_msg[name]['header']['stamp'] = {}
            self.laser_scan_grid2d_msg[name]['header']['stamp']['secs'] = data.header.stamp.secs
            # self.laser_scan_grid2d_msg[name]['header']['stamp']['nsecs'] = data.header.stamp.nsecs
            self.laser_scan_grid2d_msg[name]['header']['frame_id'] = data.header.frame_id
            # self.laser_scan_grid2d_msg[name]['angle_min'] = round(data.angle_min, 2)
            # self.laser_scan_grid2d_msg[name]['angle_max'] = round(data.angle_max, 2)
            # self.laser_scan_grid2d_msg[name]['angle_increment'] = round(data.angle_increment, 2)
            # self.laser_scan_grid2d_msg[name]['time_increment'] = data.time_increment
            # self.laser_scan_grid2d_msg[name]['scan_time'] = data.scan_time
            # self.laser_scan_grid2d_msg[name]['range_min'] = round(data.range_min, 2)
            # self.laser_scan_grid2d_msg[name]['range_max'] = round(data.range_max, 2)
            self.laser_scan_grid2d_msg[name]['points'] = []
            self.laser_scan_grid2d_msg[name]['gridpoint'] = []

            sample_index = 0
            for point in data.points:
                if sample_index % sample_step == 0:
                    p = {}
                    p['x'] = round(point.x, 2)
                    p['y'] = round(point.y, 2)
                    p['z'] = 0.0
                    self.laser_scan_grid2d_msg[name]['points'].append(p)
                sample_index = sample_index + 1
            self.laser_scan_grid2d_msg[name]['intensities'] = data.intensities
            if name == "scan_grid00":
                self.laser_scan_grid2d_msg[name]['mapinfo'] = {}
                self.laser_scan_grid2d_msg[name]['mapinfo']['mapname'] = data.mapinfo.mapname
                self.laser_scan_grid2d_msg[name]['mapinfo']['mapId'] = data.mapinfo.mapId
                # self.laser_scan_grid2d_msg[name]['mapinfo']['directory'] = data.mapinfo.directory
                # self.laser_scan_grid2d_msg[name]['mapinfo']['obstacleFileName'] = data.mapinfo.obstacleFileName
                self.laser_scan_grid2d_msg[name]['mapinfo']['gridWidth'] = data.mapinfo.gridWidth
                self.laser_scan_grid2d_msg[name]['mapinfo']['gridHeight'] = data.mapinfo.gridHeight
                self.laser_scan_grid2d_msg[name]['mapinfo']['originX'] = data.mapinfo.originX
                self.laser_scan_grid2d_msg[name]['mapinfo']['originY'] = data.mapinfo.originY
                self.laser_scan_grid2d_msg[name]['mapinfo']['resolution'] = data.mapinfo.resolution
                self.laser_scan_grid2d_msg[name]['mapinfo']['pngMD5'] = data.mapinfo.pngMD5
                self.laser_scan_grid2d_msg[name]['mapinfo']['max_zoom'] = data.mapinfo.max_zoom
                self.laser_scan_grid2d_msg[name]['mapinfo']['min_box_size'] = data.mapinfo.min_box_size
                # self.laser_scan_grid2d_msg[name]['mapinfo']['createAt'] = data.mapinfo.createAt
                self.laser_scan_grid2d_msg[name]['mapinfo']['modifyAt'] = data.mapinfo.modifyAt
            self.submsg_table[name] = self.laser_scan_grid2d_msg[name]
        else:
            if data.header.stamp.secs - self.laser_scan_grid2d_msg[name]['header']['stamp']['secs'] < 1.0:
                pass

            # self.laser_scan_grid2d_msg[name]['header']['seq'] = data.header.seq
            self.laser_scan_grid2d_msg[name]['header']['stamp']['secs'] = data.header.stamp.secs
            # self.laser_scan_grid2d_msg[name]['header']['stamp']['nsecs'] = data.header.stamp.nsecs
            self.laser_scan_grid2d_msg[name]['header']['frame_id'] = data.header.frame_id
            # self.laser_scan_grid2d_msg[name]['angle_min'] = round(data.angle_min, 2)
            # self.laser_scan_grid2d_msg[name]['angle_max'] = round(data.angle_max, 2)
            # self.laser_scan_grid2d_msg[name]['angle_increment'] = round(data.angle_increment, 2)
            # self.laser_scan_grid2d_msg[name]['time_increment'] = data.time_increment
            # self.laser_scan_grid2d_msg[name]['scan_time'] = data.scan_time
            # self.laser_scan_grid2d_msg[name]['range_min'] = round(data.range_min, 2)
            # self.laser_scan_grid2d_msg[name]['range_max'] = round(data.range_max, 2)
            self.laser_scan_grid2d_msg[name]['points'] = []
            self.laser_scan_grid2d_msg[name]['gridpoint'] = []
            sample_index = 0
            for point in data.points:
                if sample_index % sample_step == 0:
                    p = {}
                    p['x'] = round(point.x, 2)
                    p['y'] = round(point.y, 2)
                    p['z'] = 0.0
                    self.laser_scan_grid2d_msg[name]['points'].append(p)
                sample_index = sample_index + 1
            self.laser_scan_grid2d_msg[name]['intensities'] = data.intensities
            if name == "scan_grid00":
                self.laser_scan_grid2d_msg[name]['mapinfo']['mapname'] = data.mapinfo.mapname
                self.laser_scan_grid2d_msg[name]['mapinfo']['mapId'] = data.mapinfo.mapId
                # self.laser_scan_grid2d_msg[name]['mapinfo']['directory'] = data.mapinfo.directory
                # self.laser_scan_grid2d_msg[name]['mapinfo']['obstacleFileName'] = data.mapinfo.obstacleFileName
                self.laser_scan_grid2d_msg[name]['mapinfo']['gridWidth'] = data.mapinfo.gridWidth
                self.laser_scan_grid2d_msg[name]['mapinfo']['gridHeight'] = data.mapinfo.gridHeight
                self.laser_scan_grid2d_msg[name]['mapinfo']['originX'] = data.mapinfo.originX
                self.laser_scan_grid2d_msg[name]['mapinfo']['originY'] = data.mapinfo.originY
                self.laser_scan_grid2d_msg[name]['mapinfo']['resolution'] = data.mapinfo.resolution
                self.laser_scan_grid2d_msg[name]['mapinfo']['pngMD5'] = data.mapinfo.pngMD5
                self.laser_scan_grid2d_msg[name]['mapinfo']['max_zoom'] = data.mapinfo.max_zoom
                self.laser_scan_grid2d_msg[name]['mapinfo']['min_box_size'] = data.mapinfo.min_box_size
                # self.laser_scan_grid2d_msg[name]['mapinfo']['createAt'] = data.mapinfo.createAt
                self.laser_scan_grid2d_msg[name]['mapinfo']['modifyAt'] = data.mapinfo.modifyAt
            self.submsg_table[name] = self.laser_scan_grid2d_msg[name]
        pass

    def raw_odom_cb(self,data):
        if self.submsg_recived_table['raw_odom'] == False:
            self.submsg_recived_table['raw_odom'] = True
        odom_msg={}
        odom_msg['header'] = {}
        odom_msg['header']["frame_id"] = data.header.frame_id
        stamp_time = str(data.header.stamp.secs) + str('.') + str(data.header.stamp.nsecs)
        odom_msg['header']["stamp"] = float(stamp_time)
        odom_msg['pose']={}
        odom_msg['pose']['position']={}
        odom_msg['pose']['position']['x']= data.pose.pose.position.x
        odom_msg['pose']['position']['y']= data.pose.pose.position.y
        odom_msg['pose']['position']['z']= data.pose.pose.position.z
        odom_msg['pose']['orientation']={}
        odom_msg['pose']['orientation']['x']= data.pose.pose.orientation.x
        odom_msg['pose']['orientation']['y']= data.pose.pose.orientation.y
        odom_msg['pose']['orientation']['z']= data.pose.pose.orientation.z
        odom_msg['pose']['orientation']['w']= data.pose.pose.orientation.w
        odom_msg['twist']={}
        odom_msg['twist']['linear']={}
        odom_msg['twist']['linear']['x']=data.twist.twist.linear.x
        odom_msg['twist']['linear']['y']=data.twist.twist.linear.y
        odom_msg['twist']['linear']['z']=data.twist.twist.linear.z
        odom_msg['twist']['angular']={}
        odom_msg['twist']['angular']['x']=data.twist.twist.angular.x
        odom_msg['twist']['angular']['y']=data.twist.twist.angular.y
        odom_msg['twist']['angular']['z']=data.twist.twist.angular.z
        self.submsg_table['raw_odom'] = odom_msg
        pass
    def follow_path_progress_cb(self,data):
        if self.submsg_recived_table['follow_path_progress'] == False:
            self.submsg_recived_table['follow_path_progress'] = True
        follow_path_status = data.data
        follow_path_status_msg={}
        for data in follow_path_status:
            follow_path_status_msg[data.key] = data.value
        self.submsg_table['follow_path_progress'] = follow_path_status_msg
        pass
    def bt_progress_cb(self,data):
        if self.submsg_recived_table['bt_navigator_progress'] == False:
            self.submsg_recived_table['bt_navigator_progress'] = True
        bt_progress_status = data.data
        bt_progress_status_msg={}
        for data in bt_progress_status:
            bt_progress_status_msg[data.key] = data.value
        # print(bt_progress_status_msg)
        self.submsg_table['bt_navigator_progress'] = bt_progress_status_msg
        pass
    def navigation_notice_cb(self, data):
        if self.submsg_recived_table['navigation_notice'] == False:
            self.submsg_recived_table['navigation_notice'] = True
        navigation_notice = {}
        navigation_notice["task_name"] = data.task_name
        navigation_notice["notice_level"] = data.notice_level
        navigation_notice["nav_type"] = data.nav_type
        navigation_notice["target_name"] = data.target_name
        end_time = datetime.now()
        end_time = end_time.strftime("%Y-%m-%d %H:%M:%S")
        navigation_notice["date"] = end_time
        navigation_notice["stamp"] = round(time.time() ,2)
        # rospy.loginfo("len nav_status): %d", len(self.submsg_table['navigation_notice']["nav_status"]))
        # rospy.loginfo("navigation_notice message: %s", self.submsg_table['navigation_notice']["nav_status"][-1]["message"])
        # rospy.loginfo("data.message: %s",data.message)
        if navigation_notice["nav_type"] == "NAV_POSE":
            if(len(self.submsg_table['navigation_notice']["nav_status"]) >0 and
               (self.submsg_table['navigation_notice']["nav_status"][-1]["message"] == data.message) and
               (self.submsg_table['navigation_notice']["nav_status"][-1]["stamp"] + 5) >= navigation_notice["stamp"]):
                return
            else:
                navigation_notice["target_pose"] = {}
                navigation_notice["target_pose"]["position"] = {}
                navigation_notice["target_pose"]["position"]["x"] = round(data.pose.position.x ,2)
                navigation_notice["target_pose"]["position"]["y"] = round(data.pose.position.y ,2)
                navigation_notice["target_pose"]["position"]["z"] = round(data.pose.position.z ,2)
                navigation_notice["target_pose"]["orientation"] = {}
                navigation_notice["target_pose"]["orientation"]["x"] = round(data.pose.orientation.x ,2)
                navigation_notice["target_pose"]["orientation"]["y"] = round(data.pose.orientation.y ,2)
                navigation_notice["target_pose"]["orientation"]["z"] = round(data.pose.orientation.z ,2)
                navigation_notice["target_pose"]["orientation"]["w"] = round(data.pose.orientation.w ,2)
        else:
            self.nav_notice["target_pose"] = {}
        navigation_notice["message"] = data.message

        end_time = datetime.strptime(end_time, r"%Y-%m-%d %H:%M:%S")
        diff = end_time - self.notice_time
        # if diff.total_seconds() <= 0.5:
          # rospy.logerr("[Time difference of less than seconds]")
        self.submsg_table['navigation_notice']['nav_status'].append(navigation_notice)
        self.submsg_table['last_navigation_notice'] = navigation_notice
          # rospy.logerr("navigation_notice",self.submsg_table['navigation_notice'])
        # else:
          # rospy.logerr("[Time difference of greater than seconds]")
          # self.submsg_table['navigation_notice'] = {}
          # self.submsg_table['navigation_notice']["nav_status"] = []
          # self.submsg_table['navigation_notice']["nav_status"].append(navigation_notice)
          # self.notice_time = datetime.now()
          # self.notice_time = self.notice_time.strftime("%Y-%m-%d %H:%M:%S")
          # self.notice_time = datetime.strptime(self.notice_time, r"%Y-%m-%d %H:%M:%S")
          # rospy.logerr("navigation_notice",self.submsg_table['navigation_notice'])
        pass

    def bt_navigator_status_cb(self,data):
        if self.submsg_recived_table['bt_navigator_status'] == False:
            self.submsg_recived_table['bt_navigator_status'] = True
        navigator_status_msg = {}
        self.submsg_table['bt_navigator_status']={}
        self.submsg_table['bt_navigator_status']['nav_status_data'] = {}
        self.submsg_table['bt_navigator_status']['process']={}
        bt_navigator_status_msg = data.data
        status_type = bt_navigator_status_msg & 15
        if status_type == 0:
            self.submsg_table['bt_navigator_status']['nav_status_type'] = "idle"
        elif status_type == 1:
            self.submsg_table['bt_navigator_status']['nav_status_type'] = "paused"
        elif status_type == 3:
            self.submsg_table['bt_navigator_status']['nav_status_type'] = "follow_path"
            status_data = (bt_navigator_status_msg & 3840) >> 8  #analyze status data
            self.submsg_table['bt_navigator_status']['process']=self.submsg_table['follow_path_progress']
            if status_data == 0:
                self.submsg_table['bt_navigator_status']['nav_status_data']["finished"] = True
                self.submsg_table['bt_navigator_status']['nav_status_data']['information'] = "NO_PATH"
            elif status_data == 1:
                self.submsg_table['bt_navigator_status']['nav_status_data']["finished"] = True
                self.submsg_table['bt_navigator_status']['nav_status_data']['information'] = "FIXPATH_NOT_VALID"
            elif status_data == 2:
                self.submsg_table['bt_navigator_status']['nav_status_data']["finished"] = False
                self.submsg_table['bt_navigator_status']['nav_status_data']['information'] = "NAVIGATE_TO_START_POINT"
            elif status_data == 3:
                self.submsg_table['bt_navigator_status']['nav_status_data']["finished"] = False
                self.submsg_table['bt_navigator_status']['nav_status_data']['information'] = "FOLLOWING"
            elif status_data == 4:
                self.submsg_table['bt_navigator_status']['nav_status_data']["finished"] = False
                self.submsg_table['bt_navigator_status']['nav_status_data']['information'] = "OBSTACLE_WAIT"
            elif status_data == 5:
                self.submsg_table['bt_navigator_status']['nav_status_data']["finished"] = False
                self.submsg_table['bt_navigator_status']['nav_status_data']['information'] = "OBSTACLE_AVOID"
            elif status_data == 6:
                self.submsg_table['bt_navigator_status']['nav_status_data']["finished"] = False
                self.submsg_table['bt_navigator_status']['nav_status_data']['information'] = "PATH_UNSAFE"
            elif status_data == 7:
                self.submsg_table['bt_navigator_status']['nav_status_data']["finished"] = True
                self.submsg_table['bt_navigator_status']['nav_status_data']['information'] = "FIXPATH_REACHED"
                # 触发机械臂动作
                self._trigger_mechanical_arm_action("FIXPATH_REACHED")
            elif status_data == 8:
                self.submsg_table['bt_navigator_status']['nav_status_data']["finished"] = False
                self.submsg_table['bt_navigator_status']['nav_status_data']['information'] = "FIXPATH_GOAL_UNREACHED"
        elif status_type == 4:
            #navigation status
            self.submsg_table['bt_navigator_status']['nav_status_type'] = "navigation"  
            status_data = (bt_navigator_status_msg & 240) >> 4  #analyze status data
            self.submsg_table['bt_navigator_status']['nav_status_data'] = {}
            if status_data == 2:
                self.submsg_table['bt_navigator_status']['nav_status_data']["finished"] = False
                self.submsg_table['bt_navigator_status']['nav_status_data']['information'] = "GOAL_NOT_SAFE"
            elif status_data == 3:
                self.submsg_table['bt_navigator_status']['nav_status_data']["finished"] = False
                self.submsg_table['bt_navigator_status']['nav_status_data']['information'] = "PATH_NOT_SAFE"
            elif status_data == 4:
                self.submsg_table['bt_navigator_status']['nav_status_data']["finished"] = False
                self.submsg_table['bt_navigator_status']['nav_status_data']['information'] = "GOAL_UNREACHABLE"
            elif status_data == 5:
                self.submsg_table['bt_navigator_status']['nav_status_data']["finished"] = False
                self.submsg_table['bt_navigator_status']['nav_status_data']['information'] = "GOAL_RUNNING"
            elif status_data == 6:
                self.submsg_table['bt_navigator_status']['nav_status_data']["finished"] = False
                self.submsg_table['bt_navigator_status']['nav_status_data']['information'] = "GOAL_PLANNING"
            elif status_data == 7:
                self.submsg_table['bt_navigator_status']['nav_status_data']["finished"] = False
                self.submsg_table['bt_navigator_status']['nav_status_data']['information'] = "GOT_PATH_PLAN"
            elif status_data == 8:
                self.submsg_table['bt_navigator_status']['nav_status_data']["finished"] = True
                self.submsg_table['bt_navigator_status']['nav_status_data']['information'] = "FAILED_GET_PATH"
            elif status_data == 9:
                self.submsg_table['bt_navigator_status']['nav_status_data']["finished"] = True
                self.submsg_table['bt_navigator_status']['nav_status_data']['information'] = "GOAL_REACHED"
                # 触发机械臂动作
                self._trigger_mechanical_arm_action("GOAL_REACHED")
            elif status_data == 10:
                self.submsg_table['bt_navigator_status']['nav_status_data']["finished"] = True
                self.submsg_table['bt_navigator_status']['nav_status_data']['information'] = "GOAL_UNREACHED"
        else :
            self.submsg_table['bt_navigator_status']['nav_status_type'] = "UNKNOW"
    def map_info_cb(self,data):
        if data.mapname == "scanning_map":
          return
        if self.submsg_recived_table['map_info'] == False:
            self.submsg_recived_table['map_info'] = True
        map_info_msg = data
        self.submsg_table['map_info'] = map_info_msg
        #rospy.loginfo("received mapinfo: %s",data.mapname)
        pass

    def footprint_cb(self,data):
        if self.check_reviced('map_info') == False:
            return
        if self.submsg_recived_table['footprint'] == False:
            self.submsg_recived_table['footprint'] = True
        footprint_msg={}
        map_info = {}
        map_info['gridWidth'] = self.submsg_table['map_info'].gridWidth
        map_info['gridHeight'] = self.submsg_table['map_info'].gridHeight
        originX = self.submsg_table['map_info'].originX
        originY = self.submsg_table['map_info'].originY
        resolution = self.submsg_table['map_info'].resolution
        footprint_msg['mapInfo'] = map_info
        footprint_msg['world_points'] = []
        footprint_msg['grid_points'] = []
        for point in data.polygon.points:
            grid_point = {}
            grid_point['x'] = int(round((point.x - originX) / resolution))
            grid_point['y'] = int(round((point.y - originY) / resolution))
            point_msg={}
            point_msg['x']=point.x
            point_msg['y']=point.y
            point_msg['z']=point.z
            footprint_msg['grid_points'].append(grid_point)
            footprint_msg['world_points'].append(point_msg)
        self.submsg_table['footprint'] = footprint_msg
        pass

    def path_cb(self,data):
        if self.check_reviced('map_info') == False:
            return
        if self.submsg_recived_table['path'] == False:
            self.submsg_recived_table['path'] = True
        path_msg = {}
        map_info = {}
        map_info['gridWidth'] = self.submsg_table['map_info'].gridWidth
        map_info['gridHeight'] = self.submsg_table['map_info'].gridHeight
        map_info['originX'] = originX = self.submsg_table['map_info'].originX
        map_info['originY'] = originY = self.submsg_table['map_info'].originY
        map_info['resolution'] = resolution = self.submsg_table['map_info'].resolution
        path_msg['mapInfo'] = map_info
        path_msg["grid_points"]=[]
        grid_poses=[]
        for point in data.poses:
          grid_point = helper.Coordinates.PoseToFloatGrid(point.pose,originX,originY,resolution)
          grid_poses.append(grid_point)
        unique_grids = helper.Coordinates.UniqueGrid(grid_poses)
        for grid in unique_grids:
            tempgrid={}
            tempgrid["x"] = grid.x
            tempgrid["y"] = grid.y
            tempgrid["angle"] = grid.angle
            path_msg["grid_points"].append(tempgrid)
        self.submsg_table['path'] = path_msg
        pass

    def localization_status_cb(self,data):
        if self.submsg_recived_table['localization_status'] == False:
            self.submsg_recived_table['localization_status'] = True
        localization_status={}
        if data.data == 5 :
            localization_status['initialized'] = False
            localization_status['status'] = "switching_map"
        elif data.data == 4:
            localization_status['initialized'] = True
            localization_status['status'] = "out_of_map"
        elif data.data == 3 :
            localization_status['initialized'] = True
            localization_status['status'] = "normal"
        elif data.data == 2 :
            localization_status['initialized'] = True
            localization_status['status'] = "sensor_data_lost"
        elif data.data == 1 :
            localization_status['initialized'] = False
            localization_status['status'] = "initializing"
        elif data.data == 0 :
            localization_status['initialized'] = False
            localization_status['status'] = "uninitialized"
        else:
            localization_status['initialized'] = False
            localization_status['status'] = "laser_check_failed"
            # localization_status['errormsg'] = "laser check failed"
        self.submsg_table['localization_status'] = localization_status
        pass

    def localization_score_cb(self,data):
        if self.submsg_recived_table['localization_score'] == False:
            self.submsg_recived_table['localization_score'] = True
        localization_score = {}
        localization_score["localization_score"] = round(data.localization_score, 2)
        localization_score["avg_localization_score_on_map"] = round(data.avg_localization_score_on_map, 2)
        self.submsg_table["localization_score"] = localization_score
        pass

    def ultrasonic_cb(self,data):
        frame_id = data.header.frame_id
        if frame_id in self.ultrasound_map.keys():
          if self.submsg_recived_table[self.ultrasound_map[frame_id]] == False:
              self.submsg_recived_table[self.ultrasound_map[frame_id]] = True
        else:
          return
        origin,outgrid = self.getRangeMsgGrid(data,"base_link")
        ultrasonic_msg={}
        ultrasonic_msg = {}
        ultrasonic_msg['frameid']=data.header.frame_id
        ultrasonic_msg['originX']=float(origin.pose.position.x)
        ultrasonic_msg['originY']=float(origin.pose.position.y)
        angle = euler_from_quaternion([origin.pose.orientation.x, origin.pose.orientation.y, origin.pose.orientation.z, origin.pose.orientation.w])
        ultrasonic_msg['angle']=float(angle[2])
        ultrasonic_msg['range']=data.range
        self.submsg_table[self.ultrasound_map[frame_id]] = ultrasonic_msg

    def irsensor_cb(self,data):
        frame_id = data.header.frame_id
        if frame_id in self.irsensor_map.keys():
          if self.submsg_recived_table[self.irsensor_map[frame_id]] == False:
              self.submsg_recived_table[self.irsensor_map[frame_id]] = True
        else:
          return
        origin,outgrid = self.getRangeMsgGrid(data,"base_link")
        irsensor_msg={}
        irsensor_msg = {}
        irsensor_msg['frameid']=data.header.frame_id
        irsensor_msg['originX']=float(origin.pose.position.x)
        irsensor_msg['originY']=float(origin.pose.position.y)
        angle = euler_from_quaternion([origin.pose.orientation.x, origin.pose.orientation.y, origin.pose.orientation.z, origin.pose.orientation.w])
        irsensor_msg['angle']=float(angle[2])
        irsensor_msg['range']=data.range
        self.submsg_table[self.irsensor_map[frame_id]] = irsensor_msg
    def bumper_cb(self,data):
        if self.submsg_recived_table['bumper'] == False:
            self.submsg_recived_table['bumper'] = True
        bumper_data = data.g_state_input
        bumper_size = 3
        self.device_notice["emergency_stop"] = bool(data.g_state_input & 0b00000001)
        self.device_notice["bumper_front"] = bool(data.g_state_input & 0b00000010)
        self.device_notice["bumper_back"] = bool(data.g_state_input & 0b00000100)
        bumper_str = ""
        for i in range(0, bumper_size):
            if (1<<i) & bumper_data:
                bumper_str ="1" + bumper_str
            else :
                bumper_str ="0" + bumper_str
        self.submsg_table['bumper'] = bumper_str
        if self.device_notice["emergency_stop"] == True or self.device_notice["bumper_front"] == True\
            or self.device_notice["bumper_back"] == True:
            try:
                pause_task_srv = rospy.ServiceProxy("/drobot_task_manager/pause_task",common_service.srv.Empty)
                res = pause_task_srv()
            except rospy.ServiceException as servexc:
                return
    def chassis_state_cb(self,data):
        if self.submsg_recived_table['chassis_state'] == False:
            self.submsg_recived_table['chassis_state'] = True
            self.submsg_table['chassis_state'] = {}
            self.submsg_table['chassis_state']['E_stop'] = 0
            self.submsg_table['chassis_state']['Left_Motor'] = 0
            self.submsg_table['chassis_state']['Right_Motor'] = 0
            self.submsg_table['chassis_state']['Engine_Motor'] =  0
            self.submsg_table['chassis_state']['Steering_Motor'] =  0
            self.submsg_table['chassis_state']['Ultrasonic'] = 0
            self.submsg_table['chassis_state']['Infrared'] = 0
            self.submsg_table['chassis_state']['ChargingState'] = 0
            self.submsg_table['chassis_state']['ChargedState'] = 0
            self.submsg_table['chassis_state']['Laser'] = 0
        STATE_UNKNOW = 0x00
        STATE_OPEN = 0x01
        STATE_CLOSE = 0x02
        STATE_NO_ERROR = 0x70
        STATE_UNKNOW_ERROR = 0x71
        chassis_state_data = {}
        if data.state_switch_emergency_stop == STATE_OPEN:
          self.submsg_table['chassis_state']['E_stop'] =  1
        elif data.state_switch_emergency_stop == STATE_CLOSE:
          self.submsg_table['chassis_state']['E_stop'] =  0
        if data.state_motor_left == STATE_UNKNOW_ERROR:
          self.submsg_table['chassis_state']['Left_Motor'] =  1
        elif data.state_motor_left == STATE_NO_ERROR:
          self.submsg_table['chassis_state']['Left_Motor'] =  0
        if data.state_motor_right == STATE_UNKNOW_ERROR:
          self.submsg_table['chassis_state']['Right_Motor'] =  1
        elif data.state_motor_right == STATE_NO_ERROR:
          self.submsg_table['chassis_state']['Right_Motor'] =  0
        if data.state_motor_engine == STATE_UNKNOW_ERROR:
          self.submsg_table['chassis_state']['Engine_Motor'] =  1
        elif data.state_motor_engine == STATE_NO_ERROR:
          self.submsg_table['chassis_state']['Engine_Motor'] =  0
        if data.state_motor_steering == STATE_UNKNOW_ERROR:
          self.submsg_table['chassis_state']['Steering_Motor'] =  1
        elif data.state_motor_steering == STATE_NO_ERROR:
          self.submsg_table['chassis_state']['Steering_Motor'] =  0
        if data.state_sensor_ultrasonic_0 == STATE_UNKNOW_ERROR:
          self.submsg_table['chassis_state']['Ultrasonic'] =  self.submsg_table['chassis_state']['Ultrasonic'] | (1 << 0 )
        elif data.state_sensor_ultrasonic_0 == STATE_NO_ERROR:
          self.submsg_table['chassis_state']['Ultrasonic'] =  self.submsg_table['chassis_state']['Ultrasonic'] & (0 << 0 )
        if data.state_sensor_ultrasonic_1 == STATE_UNKNOW_ERROR:
          self.submsg_table['chassis_state']['Ultrasonic'] =  self.submsg_table['chassis_state']['Ultrasonic'] | (1 << 1 )
        elif data.state_sensor_ultrasonic_1 == STATE_NO_ERROR:
          self.submsg_table['chassis_state']['Ultrasonic'] =  self.submsg_table['chassis_state']['Ultrasonic'] & (0 << 1 )
        if data.state_sensor_ultrasonic_2 == STATE_UNKNOW_ERROR:
          self.submsg_table['chassis_state']['Ultrasonic'] =  self.submsg_table['chassis_state']['Ultrasonic'] | (1 << 2 )
        elif data.state_sensor_ultrasonic_2 == STATE_NO_ERROR:
          self.submsg_table['chassis_state']['Ultrasonic'] =  self.submsg_table['chassis_state']['Ultrasonic'] & (0 << 2 )
        if data.state_sensor_ultrasonic_3 == STATE_UNKNOW_ERROR:
          self.submsg_table['chassis_state']['Ultrasonic'] =  self.submsg_table['chassis_state']['Ultrasonic'] | (1 << 3 )
        elif data.state_sensor_ultrasonic_3 == STATE_NO_ERROR:
          self.submsg_table['chassis_state']['Ultrasonic'] =  self.submsg_table['chassis_state']['Ultrasonic'] & (0 << 3 )
        if data.state_sensor_ultrasonic_4 == STATE_UNKNOW_ERROR:
          self.submsg_table['chassis_state']['Ultrasonic'] =  self.submsg_table['chassis_state']['Ultrasonic'] | (1 << 4 )
        elif data.state_sensor_ultrasonic_4 == STATE_NO_ERROR:
          self.submsg_table['chassis_state']['Ultrasonic'] =  self.submsg_table['chassis_state']['Ultrasonic'] & (0 << 4 )
        if data.state_sensor_ultrasonic_5 == STATE_UNKNOW_ERROR:
          self.submsg_table['chassis_state']['Ultrasonic'] =  self.submsg_table['chassis_state']['Ultrasonic'] | (1 << 5 )
        elif data.state_sensor_ultrasonic_5 == STATE_NO_ERROR:
          self.submsg_table['chassis_state']['Ultrasonic'] =  self.submsg_table['chassis_state']['Ultrasonic'] & (0 << 5 )
        if data.state_sensor_ultrasonic_6 == STATE_UNKNOW_ERROR:
          self.submsg_table['chassis_state']['Ultrasonic'] =  self.submsg_table['chassis_state']['Ultrasonic'] | (1 << 6 )
        elif data.state_sensor_ultrasonic_6 == STATE_NO_ERROR:
          self.submsg_table['chassis_state']['Ultrasonic'] =  self.submsg_table['chassis_state']['Ultrasonic'] & (0 << 6 )
        if data.state_sensor_ultrasonic_7 == STATE_UNKNOW_ERROR:
          self.submsg_table['chassis_state']['Ultrasonic'] =  self.submsg_table['chassis_state']['Ultrasonic'] | (1 << 7 )
        elif data.state_sensor_ultrasonic_7 == STATE_NO_ERROR:
          self.submsg_table['chassis_state']['Ultrasonic'] =  self.submsg_table['chassis_state']['Ultrasonic'] & (0 << 7 )

        if data.state_sensor_ir_0 == STATE_UNKNOW_ERROR:
          self.submsg_table['chassis_state']['Infrared'] =  self.submsg_table['chassis_state']['Infrared'] | (1 << 0 )
        elif data.state_sensor_ir_0 == STATE_NO_ERROR:
          self.submsg_table['chassis_state']['Infrared'] =  self.submsg_table['chassis_state']['Infrared'] & (0 << 0 )
        if data.state_sensor_ir_1 == STATE_UNKNOW_ERROR:
          self.submsg_table['chassis_state']['Infrared'] =  self.submsg_table['chassis_state']['Infrared'] | (1 << 1 )
        elif data.state_sensor_ir_1 == STATE_NO_ERROR:
          self.submsg_table['chassis_state']['Infrared'] =  self.submsg_table['chassis_state']['Infrared'] & (0 << 1 )
        if data.state_sensor_ir_2 == STATE_UNKNOW_ERROR:
          self.submsg_table['chassis_state']['Infrared'] =  self.submsg_table['chassis_state']['Infrared'] | (1 << 2 )
        elif data.state_sensor_ir_2 == STATE_NO_ERROR:
          self.submsg_table['chassis_state']['Infrared'] =  self.submsg_table['chassis_state']['Infrared'] & (0 << 2 )
        if data.state_sensor_ir_3 == STATE_UNKNOW_ERROR:
          self.submsg_table['chassis_state']['Infrared'] =  self.submsg_table['chassis_state']['Infrared'] | (1 << 3 )
        elif data.state_sensor_ir_3 == STATE_NO_ERROR:
          self.submsg_table['chassis_state']['Infrared'] =  self.submsg_table['chassis_state']['Infrared'] & (0 << 3 )

        if data.state_event_charging == 0x01: self.submsg_table['chassis_state']['ChargingState'] = 1
        if data.state_event_charged == 0x01: self.submsg_table['chassis_state']['ChargedState'] = 1
        if self.submsg_recived_table['raw_scan'] == False:
          self.submsg_table['chassis_state']['Laser'] = 1
        else:
          now_time = rospy.Time.now()
          stamp_time = str(now_time.secs) + str('.') + str(now_time.nsecs)
          if float(stamp_time) - self.submsg_table['raw_scan']['header']['stamp'] > 1: #激光超时判断
            self.submsg_table['chassis_state']['Laser'] = 1
          else:
            self.submsg_table['chassis_state']['Laser'] = 0
        self.submsg_table['sellButtonFlag'] = data.sellButtonFlag
        # self.submsg_table['chassis_state'] = chassis_state_data
    def ptz_state_cb(self,data):
        if self.submsg_recived_table['ptz_state'] == False:
            self.submsg_recived_table['ptz_state'] = True
        ptz_state_msg={}
        ptz_state_msg['yaw'] = data.pelco_yaw_value
        if data.pelco_pitch_value >= 270 and data.pelco_pitch_value <=360:
            data.pelco_pitch_value -= 360
        ptz_state_msg['pitch'] = data.pelco_pitch_value
        ptz_state_msg['light'] = data.pelco_light_state
        self.submsg_table['ptz_state'] = ptz_state_msg

    def imu_cb(self,data):
        if self.submsg_recived_table['raw_imu'] == False:
            self.submsg_recived_table['raw_imu'] = True
        imu_msg = {}
        imu_msg['header'] = {}
        stamp_time = str(data.header.stamp.secs) + str('.') + str(data.header.stamp.nsecs)
        imu_msg['header']['stamp'] = float(stamp_time)
        imu_msg['header']['frame_id'] = data.header.frame_id
        imu_msg['orientation']={}
        imu_msg['orientation']['x']=data.orientation.x
        imu_msg['orientation']['y']=data.orientation.y
        imu_msg['orientation']['z']=data.orientation.z
        imu_msg['orientation']['w']=data.orientation.w
        imu_msg['angular_velocity']={}
        imu_msg['angular_velocity']['x']=data.angular_velocity.x
        imu_msg['angular_velocity']['y']=data.angular_velocity.y
        imu_msg['angular_velocity']['z']=data.angular_velocity.z
        imu_msg['linear_acceleration']={}
        imu_msg['linear_acceleration']['x']=data.linear_acceleration.x
        imu_msg['linear_acceleration']['y']=data.linear_acceleration.y
        imu_msg['linear_acceleration']['z']=data.linear_acceleration.z
        self.submsg_table['raw_imu'] = imu_msg
    def gps_cb(self,data):
        if self.submsg_recived_table['gps'] == False:
            self.submsg_recived_table['gps'] = True
        gps_msg={}
        gps_msg['latitude'] = data.Latitude
        gps_msg['longitude'] = data.Longitude
        gps_msg['altitude'] = data.Altitude
        gps_msg['quality'] = data.Quality
        self.submsg_table['gps'] = gps_msg
    def gps_grid_cb(self,data):
        if self.submsg_recived_table['gps_grid'] == False:
            self.submsg_recived_table['gps_grid'] = True
        grid_gps_msg={}
        grid_gps_msg['header'] = {}
        grid_gps_msg['header']["frame_id"] = data.header.frame_id
        stamp_time = str(data.header.stamp.secs) + str('.') + str(data.header.stamp.nsecs)
        grid_gps_msg['header']["stamp"] = float(stamp_time)
        grid_gps_msg['grid_point'] = {}
        grid_gps_msg['grid_point']['x'] = data.point_map.x
        grid_gps_msg['grid_point']['y'] = data.point_map.x
        grid_gps_msg['type'] = data.type
        self.submsg_table['gps_grid'] = grid_gps_msg
    def traffic_light_state_cb(self,data):
        if self.submsg_recived_table['traffic_light_state'] == False:
            self.submsg_recived_table['traffic_light_state'] = True
        msg={}
        msg['light_switch_state'] = data.light_switch_state
        msg['straight'] = data.straight
        msg['right'] = data.right
        msg['left'] = data.left
        self.submsg_table['traffic_light_state'] = msg

    def dongle_state_cb(self,data):
        if self.submsg_recived_table['dongle_state'] == False:
            self.submsg_recived_table['dongle_state'] = True
        msg={}
        if data.data == 0:
          msg['dongle_state'] = True
        else :
          msg['dongle_state'] = False
        self.submsg_table['dongle_state'] = msg

    def move_flag_cb(self,data):
        if self.submsg_recived_table['move_flag'] == False:
            self.submsg_recived_table['move_flag'] = True
        msg={}
        msg['move_flag'] = data.data
        self.submsg_table['move_flag'] = msg

    def drive_mode_status_cb(self, data):
        if self.submsg_recived_table['auto_mode'] == False:
            self.submsg_recived_table['auto_mode'] = True
        self.submsg_table['auto_mode'] = data.data
    def cleaning_state_cb(self, data):
        if self.submsg_recived_table['clean_state'] == False:
            self.submsg_recived_table['clean_state'] = True
        clean_sate_msg={}
        clean_sate_msg['charge'] = data.bms_charge_status      #扫地机充电状态 0:未知状态 1：充电完成   2:未充电  3:充电中
        clean_sate_msg['trash'] = data.trash_can_full_status #垃圾箱满状态指示 0:未知状态 1：垃圾箱已满   2:垃圾箱未满
        clean_sate_msg['clean_water'] = data.water_tank_empty_status #储水水箱空状态指示{警示} 0:未知状态 1：储水水箱已空   2:储水水箱未空
        clean_sate_msg['clean_water_level'] = data.water_level_status       #储水水箱液位状态{提示} 百分比0-100%
        clean_sate_msg['sewage_water'] = data.sewage_tank_full_status  #污水水箱满状态指示{预留} 0:未知状态 1：污水水箱已满   2:污水水箱未满
        clean_sate_msg['watering_swtich'] = data.water_switch_status   #当前洒水的状态：true: 表示当前正在洒水，false：未在洒水
        clean_sate_msg['sweep_switch'] = data.clean_switch_status      #当前清扫的状态：true: 表示当前正在清扫，false：未在清扫
        self.submsg_table['clean_state'] = clean_sate_msg
    
    def washer_devices_state_cb(self, data):
        if self.submsg_recived_table['clean_state'] == False:
            self.submsg_recived_table['clean_state'] = True
        clean_sate_msg={}
        clean_sate_msg['uptake_switch']={}
        clean_sate_msg['brush_switch']={}
        clean_sate_msg['uptake_switch']['switch'] = True if (data.data & 0x30) >> 4 else False
        clean_sate_msg['uptake_switch']['speed'] = 2 if not (data.data & 0x30) >> 4 else (data.data & 0x30) >> 4
        clean_sate_msg['brush_switch']['switch'] = True if (data.data & 0x04) >> 2 else False
        clean_sate_msg['brush_switch']['speed'] = 2 if not (data.data & 0xC0) >> 6 else (data.data & 0xC0) >> 6
        clean_sate_msg['scrape_switch'] = True if (data.data & 0x08) >> 3 else False
        clean_sate_msg['spout_switch'] = True if (data.data & 0x02) >> 1 else False
        self.submsg_table['clean_state']['washer']=clean_sate_msg

    def nav_robot_pose_cb(self,data):
        if self.submsg_recived_table['robot_pose_grid'] == False:
            self.submsg_recived_table['robot_pose_grid'] = True
        nav_robot_pose_msg={}
        nav_robot_pose_msg['nav_status_type'] = "robot_pose"
        nav_robot_pose_msg['robot_id'] = "1"
        nav_robot_pose_msg['nav_status_data'] = {}
        nav_robot_pose_msg['nav_status_data']['gridPosition'] = {}
        nav_robot_pose_msg['nav_status_data']['gridPosition']['x'] = data.grid_point.x
        nav_robot_pose_msg['nav_status_data']['gridPosition']['y'] = data.grid_point.y
        nav_robot_pose_msg['nav_status_data']['angle'] = data.grid_point.theta
        nav_robot_pose_msg['nav_status_data']['mapinfo'] = {}
        nav_robot_pose_msg['nav_status_data']['mapinfo']['mapname'] = data.map_info.mapname
        nav_robot_pose_msg['nav_status_data']['mapinfo']['gridWidth'] = data.map_info.gridWidth
        nav_robot_pose_msg['nav_status_data']['mapinfo']['gridHeight'] = data.map_info.gridHeight
        nav_robot_pose_msg['nav_status_data']['mapinfo']['originX'] = data.map_info.originX
        nav_robot_pose_msg['nav_status_data']['mapinfo']['originY'] = data.map_info.originY
        nav_robot_pose_msg['nav_status_data']['mapinfo']['resolution'] = data.map_info.resolution
        nav_robot_pose_msg['worldPose']= {}
        nav_robot_pose_msg['worldPose']['position'] = {}
        nav_robot_pose_msg['worldPose']['position']['x'] = round(data.point.x, 2)
        nav_robot_pose_msg['worldPose']['position']['y'] = round(data.point.y, 2)
        nav_robot_pose_msg['worldPose']['position']['theta'] = round(data.grid_point.theta, 2)
        self.submsg_table['robot_pose_grid'] = nav_robot_pose_msg

        location_robot_pose_msg = {}
        location_robot_pose_msg['latestPose'] = {}
        location_robot_pose_msg['latestPose']["position"] = {}
        location_robot_pose_msg['latestPose']['orientation'] = {}
        location_robot_pose_msg['latestPose']["position"]['x'] = data.point.x
        location_robot_pose_msg['latestPose']["position"]['y'] = data.point.y
        q_angle = quaternion_from_euler(0,0,data.point.theta)
        q = Quaternion(*q_angle)
        location_robot_pose_msg['latestPose']['orientation']['x'] = q.x
        location_robot_pose_msg['latestPose']['orientation']['y'] = q.y
        location_robot_pose_msg['latestPose']['orientation']['z'] = q.z
        location_robot_pose_msg['latestPose']['orientation']['w'] = q.w
        # with open(self.locationJson, 'w') as f:
            # json.dump(location_robot_pose_msg, f)
            #print(f.read())

        #push_nav_message(nav_robot_pose_msg)
        # robot_pose_grid_buffer = nav_robot_pose_msg
        pass

    # def task_state_cb(self,data):
    #     if self.submsg_recived_table['task_state'] == False:
    #         self.submsg_recived_table['task_state'] = True
    #     self.submsg_table['task_state'] = data
    #     self.task_state_received = True
    #     pass

    def SparseMapCallback(self,data):
        if self.submsg_recived_table['sparse_map'] == False:
            self.submsg_recived_table['sparse_map'] = True
        RealTime_msg = {}
        RealTime_msg['mapInfo'] = {}
        RealTime_msg['mapInfo']['gridWidth'] = data.gridWidth
        RealTime_msg['mapInfo']['gridHeight'] = data.gridHeight
        RealTime_msg['data']=[]
        RealTime_msg['data'] = data.data
        RealTime_msg['trajectory'] = []
        for trajectory in data.grid_trajectories:
          trajectory_msg ={}
          trajectory_msg['id'] = trajectory.trajectoryID
          trajectory_msg['type'] = trajectory.trajectory_type
          trajectory_msg['data'] = trajectory.grid_trajectory_data
          RealTime_msg['trajectory'].append(trajectory_msg)
        self.submsg_table['sparse_map'] = RealTime_msg
        # self.sparse_map_buffer = RealTime_msg
        pass

    def laser_scan_cb(self,data):
        if self.submsg_recived_table['raw_scan'] == False:
            self.submsg_recived_table['raw_scan'] = True
        laser_scan_msg = {}
        laser_scan_msg['header'] = {}
        stamp_time = str(data.header.stamp.secs) + str('.') + str(data.header.stamp.nsecs)
        laser_scan_msg['header']['stamp'] = float(stamp_time)
        laser_scan_msg['header']['frame_id'] = data.header.frame_id
        # laser_scan_msg['angle_min'] = data.angle_min
        # laser_scan_msg['angle_max'] = data.angle_max
        # laser_scan_msg['angle_increment'] = data.angle_increment
        # laser_scan_msg['time_increment'] = data.time_increment
        # laser_scan_msg['scan_time'] = data.scan_time
        # laser_scan_msg['range_min'] = data.range_min
        # laser_scan_msg['range_max'] = data.range_max
        filterrange = []
        for range in data.ranges:
            if math.isinf(range):
                range = data.range_max
            filterrange.append(range)
        laser_scan_msg['ranges'] = filterrange
        laser_scan_msg['intensities'] = data.intensities
        self.submsg_table['raw_scan'] = laser_scan_msg
        pass

    def sensor_exception_cb(self, data):
        now_time = rospy.Time.now()
        stamp_time = str(now_time.secs) + str('.') + str(now_time.nsecs)
        if data.identifier == 'SensorExceptionEvent':
            self.cur_time_table['latest_laser_exception_time'] = float(stamp_time)
            if self.submsg_recived_table['sensor_exception_event'] == False:
                self.submsg_recived_table['sensor_exception_event'] = True
            msg = {}
            msg['Level'] = data.level
            total_sensor_id = []
            # 使用安全的JSON解析函数处理编码问题
            value = safe_json_loads(data.event_content)
            if value is None:
                rospy.logerr("无法解析传感器异常事件内容")
                return
            for id in value:
                total_sensor_id.append(id['SensorID'].encode("ascii"))
            msg['Info'] = 'Laser disconnection, please check the laser !'
            msg['Sensor_id'] = total_sensor_id
            self.submsg_table['sensor_exception_event'] = msg
        if data.identifier == 'ChassisExceptionEvent':
            if self.submsg_recived_table['chassis_exception_event'] == False:
                self.submsg_recived_table['chassis_exception_event'] = True
            msg = {}    
            msg['Level'] = data.level
            msg['Info'] = data.event_content
            self.submsg_table['chassis_exception_event'] = msg

    def _trigger_mechanical_arm_action(self, status_type):
        """
        触发机械臂动作 - 已禁用
        注意：机械臂动作现在由任务管理器统一管理，此方法已禁用以避免重复触发
        """
        # 机械臂动作现在由任务管理器通过ROS话题统一管理
        # 此方法已禁用以避免重复触发
        print("=== 机械臂动作触发已禁用 ===")
        print("状态类型: {}".format(status_type))
        print("注意：机械臂动作现在由任务管理器统一管理，避免重复触发")
        return
    
    def _get_current_mechanical_arm_action(self, current_task):
        """
        获取当前任务的机械臂动作信息
        
        Args:
            current_task: 当前任务信息
            
        Returns:
            dict: 机械臂动作信息，如果没有则返回None
        """
        try:
            # 获取当前任务的所有动作
            actions = current_task.get('actions', [])
            
            # 查找机械臂动作
            for action in actions:
                if action.get('type') == 'MECHANICAL_ARM':
                    rospy.loginfo("找到机械臂动作: %s", action)
                    return action
            
            rospy.loginfo("当前任务点无机械臂动作")
            return None
            
        except Exception as e:
            rospy.logerr("获取机械臂动作信息时发生异常: %s", safe_str(e))
            return None
    
    def _get_current_task_from_manager(self):
        """
        直接从任务管理器获取当前任务信息
        
        Returns:
            dict: 当前任务信息，如果没有则返回None
        """
        try:
            print("尝试从任务管理器获取任务状态...")
            rospy.loginfo("尝试从任务管理器获取任务状态...")
            
            # 调用任务管理器服务获取任务状态
            task_status_req = console_service.GetTaskStatusRequest()
            get_task_status_srv = rospy.ServiceProxy("/drobot_task_manager/get_task_status", console_service.GetTaskStatus)
            
            try:
                get_task_status_srv.wait_for_service(3)
                print("任务管理器服务可用")
            except rospy.ROSException:
                print("任务管理器服务不可用")
                return None
            
            try:
                resp = get_task_status_srv(task_status_req)
                print("任务管理器服务调用成功")
                print("  resp.success: {}".format(resp.success))
                print("  resp.status: {}".format(resp.status))
                print("  resp.current_task: {}".format(resp.current_task))
                print("  resp.task: {}".format(resp.task))
            except Exception as e:
                print("任务管理器服务调用失败: {}".format(str(e)))
                return None
            
            if resp.success:  # 服务调用成功，即使status为0也要处理
                # 使用安全的JSON解析函数处理编码问题
                current_task = safe_json_loads(resp.current_task)
                tasks = safe_json_loads(resp.task)
                
                if current_task is None or tasks is None:
                    rospy.logerr("无法解析任务管理器返回的JSON数据")
                    return None
                
                # 构造任务信息 - 从tasks中提取点位信息
                
                # 从tasks中查找当前任务
                current_point = ''
                point_name = ''
                if 'tasks' in tasks and tasks['tasks']:
                    for task in tasks['tasks']:
                        if task.get('name') == 'NavigationTask':
                            param = task.get('param', {})
                            current_point = param.get('point_name', '')
                            point_name = param.get('point_name', '')
                            
                            # 获取动作列表
                            actions = task.get('actions', [])
                            break
                
                # 如果从tasks中没找到，再尝试从current_task中获取
                if not current_point:
                    current_point = current_task.get('param', {}).get('point_name', '')
                    if not current_point:
                        current_point = current_task.get('point_name', '')
                    if not current_point:
                        current_point = current_task.get('current_point', '')
                
                if not point_name:
                    point_name = current_task.get('param', {}).get('point_name', '')
                    if not point_name:
                        point_name = current_task.get('point_name', '')
                    if not point_name:
                        point_name = current_task.get('current_point', '')
                
                # 获取动作列表
                actions = []
                if 'tasks' in tasks and tasks['tasks']:
                    for task in tasks['tasks']:
                        if task.get('name') == 'NavigationTask':
                            actions = task.get('actions', [])
                            break
                
                task_info = {
                    'name': tasks.get('name', ''),
                    'current_point': current_point,
                    'point_name': point_name,
                    'actions': actions
                }
                
                # 安全地处理任务信息格式化，避免编码问题
                try:
                    safe_task_info = str(task_info).encode('utf-8').decode('utf-8')
                    print("从任务管理器获取到任务信息: {}".format(safe_task_info))
                    rospy.loginfo("从任务管理器获取到任务信息: %s", safe_task_info)
                except (UnicodeError, UnicodeDecodeError):
                    print("从任务管理器获取到任务信息: {}".format(repr(task_info)))
                    rospy.loginfo("从任务管理器获取到任务信息: %s", repr(task_info))
                
                # 同时更新master_node中的任务信息
                if hasattr(self.master_node, 'current_task_info'):
                    self.master_node.current_task_info = task_info
                    print("已更新master_node中的任务信息")
                    rospy.loginfo("已更新master_node中的任务信息")
                
                return task_info
            else:
                print("任务管理器返回失败或无运行任务")
                rospy.logwarn("任务管理器返回失败或无运行任务")
                return None
                
        except Exception as e:
            print("从任务管理器获取任务信息时发生异常: {}".format(safe_str(e)))
            rospy.logerr("从任务管理器获取任务信息时发生异常: %s", safe_str(e))
            return None
    
    
    def _get_point_uuid_from_list(self, point_name):
        """
        从点位列表中获取点位UUID（参考示教完成时的逻辑）
        
        Args:
            point_name: 点位名称
            
        Returns:
            str: 点位UUID，如果找不到则返回None
        """
        try:
            print("从点位列表查找UUID - 点位名称: {}".format(point_name))
            
            # 调用获取所有点位的服务
            get_poses_req = console_service.GetPosesRequest()
            get_poses_srv = rospy.ServiceProxy("/drobot_pose_manager/get_poses", console_service.GetPoses)
            
            try:
                get_poses_srv.wait_for_service(3)
            except rospy.ROSException:
                print("点位管理服务不可用")
                return None
            
            resp = get_poses_srv(get_poses_req)
            
            if resp.success and resp.poses:
                # 遍历所有点位查找匹配的点位
                for pose in resp.poses:
                    if hasattr(pose, 'name') and pose.name == point_name:
                        if hasattr(pose, 'poseId') and pose.poseId:
                            print("在点位列表中找到UUID: {}".format(pose.poseId))
                            return pose.poseId
                        elif hasattr(pose, 'id') and pose.id:
                            print("在点位列表中找到ID: {}".format(pose.id))
                            return pose.id
                
                print("在点位列表中未找到匹配的点位")
                return None
            else:
                print("获取点位列表失败")
                return None
                
        except Exception as e:
            print("从点位列表查找UUID时发生异常: {}".format(safe_str(e)))
            return None
    
    def _get_current_point_actions_from_tasks(self, tasks, current_task):
        """
        从任务信息中获取当前点位的动作列表
        
        Args:
            tasks: 任务信息
            current_task: 当前任务信息
            
        Returns:
            list: 当前点位的动作列表
        """
        try:
            # 首先尝试从param中获取，如果没有则从根级别获取
            current_point_name = current_task.get('param', {}).get('point_name', '')
            if not current_point_name:
                current_point_name = current_task.get('point_name', '')
            if not current_point_name:
                # 如果还是没有，尝试使用current_point作为点位名称
                current_point_name = current_task.get('current_point', '')
            if not current_point_name:
                return []
            
            # 确保字符串是安全的UTF-8编码
            try:
                current_point_name = str(current_point_name).encode('utf-8').decode('utf-8')
            except (UnicodeError, UnicodeDecodeError):
                current_point_name = str(current_point_name)
            
            # 从任务中查找当前点位对应的子任务
            sub_tasks = tasks.get('tasks', [])  # 修改为tasks而不是subTasks
            print("查找子任务，当前点位名称: {}".format(current_point_name))
            print("子任务列表: {}".format(sub_tasks))
            for sub_task in sub_tasks:
                # 安全地处理子任务的点位名称
                try:
                    sub_point_name = str(sub_task.get('point_name', '')).encode('utf-8').decode('utf-8')
                    sub_name = str(sub_task.get('name', '')).encode('utf-8').decode('utf-8')
                except (UnicodeError, UnicodeDecodeError):
                    sub_point_name = str(sub_task.get('point_name', ''))
                    sub_name = str(sub_task.get('name', ''))
                
                # 多种匹配方式：精确匹配、包含匹配、ID匹配
                exact_match = (sub_point_name == current_point_name or 
                              sub_name == current_point_name)
                
                # 检查是否包含当前点位ID
                contains_match = (current_point_name in sub_point_name or 
                                current_point_name in sub_name)
                
                # 检查子任务是否包含当前点位ID
                reverse_contains_match = (sub_point_name in current_point_name or 
                                        sub_name in current_point_name)
                
                if exact_match or contains_match or reverse_contains_match:
                    actions = sub_task.get('actions', [])
                    # 安全地处理字符串格式化，避免编码问题
                    try:
                        safe_current_point_name = str(current_point_name).encode('utf-8').decode('utf-8')
                        safe_actions = str(actions).encode('utf-8').decode('utf-8')
                        print("找到当前点位动作: {} -> {}".format(safe_current_point_name, safe_actions))
                        rospy.loginfo("找到当前点位动作: %s -> %s", safe_current_point_name, safe_actions)
                    except (UnicodeError, UnicodeDecodeError):
                        print("找到当前点位动作: {} -> {}".format(repr(current_point_name), repr(actions)))
                        rospy.loginfo("找到当前点位动作: %s -> %s", repr(current_point_name), repr(actions))
                    return actions
            
            # 安全地处理字符串格式化，避免编码问题
            try:
                safe_current_point_name = str(current_point_name).encode('utf-8').decode('utf-8')
                print("未找到当前点位 {} 对应的动作".format(safe_current_point_name))
                rospy.logwarn("未找到当前点位 %s 对应的动作", safe_current_point_name)
            except (UnicodeError, UnicodeDecodeError):
                print("未找到当前点位 {} 对应的动作".format(repr(current_point_name)))
                rospy.logwarn("未找到当前点位 %s 对应的动作", repr(current_point_name))
            return []
            
        except Exception as e:
            print("获取当前点位动作时发生异常: {}".format(safe_str(e)))
            rospy.logerr("获取当前点位动作时发生异常: %s", safe_str(e))
            return []

    def _get_current_map_id(self):
        """
        获取当前地图ID
        
        Returns:
            str: 当前地图ID
        """
        try:
            # 从地图信息中获取地图ID
            if self.check_reviced('map_info'):
                map_info = self.submsg_table['map_info']
                map_id = getattr(map_info, 'mapId', None) or getattr(map_info, 'mapname', 'unknown_map')
                rospy.loginfo("获取到当前地图ID: %s", map_id)
                return str(map_id)
            else:
                rospy.logwarn("无法获取地图信息，使用默认地图ID")
                return 'unknown_map'
                
        except Exception as e:
            rospy.logerr("获取地图ID时发生异常: %s", safe_str(e))
            return 'unknown_map'
    
    def mechanical_arm_action_cb(self, msg):
        """
        处理机械臂动作请求话题
        
        Args:
            msg: std_msgs.String 包含 "map_id|point_id" 格式的消息
        """
        try:
            print("=== 机械臂动作请求 ===")
            print("原始消息: {}".format(msg.data))
            
            # 解析消息格式: "map_id|point_id"
            if "|" in msg.data:
                map_id, point_id = msg.data.split("|", 1)
                print("地图ID: {}".format(map_id))
                print("点位ID: {}".format(point_id))
                
                # 调用机械臂控制器执行动作
                success = self.mechanical_arm_controller.execute_navigation_point_action(
                    map_id=map_id,
                    point_id=point_id,
                    wait_for_completion=True,  # 任务管理器需要等待完成
                    timeout=200  # 200秒超时
                )
                
                if success:
                    print("=== 机械臂动作执行成功 ===")
                else:
                    print("=== 机械臂动作执行失败 ===")
            else:
                print("=== 机械臂动作消息格式错误: {} ===".format(msg.data))
                
        except Exception as e:
            error_msg = "机械臂动作处理异常: {}".format(safe_str(e))
            print("[ERROR] {}".format(error_msg))
            rospy.logerr("%s", error_msg)
    
    def handle_mechanical_arm_action(self, req):
        """
        处理机械臂动作服务请求
        
        Args:
            req: MechanicalArmActionRequest
            
        Returns:
            MechanicalArmActionResponse
        """
        try:
            print("=== 机械臂动作服务请求 ===")
            print("地图ID: {}".format(req.map_id))
            print("点位ID: {}".format(req.point_id))
            print("拍摄类型: {}".format(req.camera_type if hasattr(req, 'camera_type') and req.camera_type else 'visible(默认)'))
            print("机械臂点标签: {}".format(req.label if hasattr(req, 'label') and req.label else '所有机械臂点'))
            print("补光灯开关: {}".format(req.fill_light_enabled if hasattr(req, 'fill_light_enabled') else False))
            print("等待完成: {}".format(req.wait_for_completion))
            print("超时时间: {}秒".format(req.timeout))
            
            # 调用机械臂控制器执行动作
            success = self.mechanical_arm_controller.execute_navigation_point_action(
                map_id=req.map_id,
                point_id=req.point_id,
                camera_type=req.camera_type if hasattr(req, 'camera_type') else '',  # 传递拍摄类型
                label=req.label if hasattr(req, 'label') else '',  # 传递机械臂点标签
                fill_light_enabled=req.fill_light_enabled if hasattr(req, 'fill_light_enabled') else False,  # 传递补光灯开关状态
                wait_for_completion=req.wait_for_completion,
                timeout=req.timeout
            )
            
            if success:
                print("=== 机械臂动作执行成功 ===")
                return MechanicalArmActionResponse(
                    success=True,
                    error_message="",
                    completion_message="机械臂动作执行完成"
                )
            else:
                print("=== 机械臂动作执行失败 ===")
                return MechanicalArmActionResponse(
                    success=False,
                    error_message="机械臂动作执行失败",
                    completion_message=""
                )
                
        except Exception as e:
            error_msg = "机械臂动作服务处理异常: {}".format(safe_str(e))
            print("[ERROR] {}".format(error_msg))
            rospy.logerr("%s", error_msg)
            return MechanicalArmActionResponse(
                success=False,
                error_message=error_msg,
                completion_message=""
            )
