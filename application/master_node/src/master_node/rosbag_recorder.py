#!/usr/bin/env python
#coding=utf-8
import commands
import sys
import os
import roslaunch
from time import sleep
import signal
from functools import partial
import time
import threading
import thread
from threading import Thread
from os.path import join, getsize
import rospy
from std_msgs.msg import String

class BagRecorder():
  # 环境变量
  env_dist = os.environ
  bag_directory = env_dist.get('DROBOT_BAG_DIR')
  timoo_pcap_directory = env_dist.get('DROBOT_LOG_DIR') + str('/rslidar_log')
  robot_type = rospy.get_param('/drobot_device_node/ChassisType', 9)

  if not os.path.exists(bag_directory):
    os.makedirs(bag_directory)

  chach_map_name=""
  topics = []
  filter_topics = ["/camera(.*)" "/rslidar_points" "/scan_grid" "/robot_pose_grid" "/diagnostics"\
                  "/localization/mapping/odometry_incre" "/localization/odometry/imu_incre"]

  slam_topics = ['/scan','/chassis_imu', '/chassis_odom', '/odom_wheel','/wheel_car_like','/tf',\
                 '/rslidar_packets', '/timmo_packets', '/timmo_status', '/timoo_packets', '/timoo_packets2', '/timoo_status',\
                 '/gps_pub', '/drobot_imu', '/recordPose', '/amcl_pose', '/localization_score', '/localization_bit',
                 '/lslidar_packet', '/lslidar_packet_difop']

  slam_topics_3d = ['/scan', '/chassis_imu', '/chassis_odom', '/odom_wheel', '/wheel_car_like',\
                    '/tf', '/rslidar_packets', '/timmo_packets', '/timmo_status', '/timoo_packets', '/timoo_packets2',\
                    '/timoo_status', '/gps_pub','/drobot_imu','/recordPose', '/amcl_pose',\
                    '/localization_score', '/localization_bit', '/lslidar_packet', '/lslidar_packet_difop']

  nav_topics_diff = ['/scan', '/tf', '/scan01', '/scan02', '/scan03', '/scan04', '/set_pose1',
    '/chassis_ultrasound0','/chassis_ultrasound1','/chassis_ultrasound2','/chassis_ultrasound3','/chassis_ultrasound4','/chassis_ultrasound5',
    '/chassis_ultrasound6','/chassis_ultrasound7','/chassis_fallprevention0','/chassis_fallprevention1','/chassis_fallprevention2','/chassis_fallprevention3',
    '/drobot_imu','/bt_navigator_node/global_costmap/local_costmap','/bt_navigator_node/global_costmap/footprint_master',
    '/bt_navigator_node/topo_node','/bt_navigator_node/DWALocalPlanner/fixpattern_path','/bt_navigator_node/DWALocalPlanner/fixpattern_lv1_path',
    '/bt_navigator_node/robot_local_planner/global_plan','/bt_navigator_node/robot_local_planner/local_plan',
    '/bt_navigator_node/fixpattern_global_planner/plan','/odom_wheel','/cmd_vel','/wheel_car_like',
    '/gps_pub','/localization_score', '/localization_bit','/bt_navigator_node/DWALocalPlanner/chargerPose','/bt_navigator_node/status', '/key_vel','/amcl_pose',
    '/sPath','/chassis_odom', '/drobot_millimeter_wave_radar_pcl_points','gps_pub']

  nav_topics_ackermann = [ '/scan','/tf', '/scan01', '/scan02', '/scan03', '/scan04', '/set_pose1',
    '/chassis_ultrasound0','/chassis_ultrasound1','/chassis_ultrasound2','/chassis_ultrasound3','/chassis_ultrasound4','/chassis_ultrasound5','/chassis_ultrasound6','/chassis_ultrasound7',
    '/chassis_fallprevention0','/chassis_fallprevention1','/chassis_fallprevention2','/chassis_fallprevention3', '/drobot_imu',
    '/bt_navigator_node/global_costmap/local_costmap','/bt_navigator_node/global_costmap/footprint_master',
    '/bt_navigator_node/topo_node','/bt_navigator_node/DWALocalPlanner/fixpattern_path', '/bt_navigator_node/DWALocalPlanner/fixpattern_lv1_path',
    '/bt_navigator_node/robot_local_planner/global_plan', '/bt_navigator_node/robot_local_planner/local_plan ',
    '/bt_navigator_node/fixpattern_global_planner/plan','/odom_wheel','/cmd_vel','/wheel_car_like','/gps_pub',
    '/localization_score', '/localization_bit','/bt_navigator_node/DWALocalPlanner/chargerPose','/bt_navigator_node/status',
    '/key_vel','/amcl_pose','/sPath', '/chassis_odom' , '/drobot_millimeter_wave_radar_pcl_points']

  localization = ['/scan','/chassis_imu', '/chassis_odom', '/odom_wheel','/wheel_car_like','/tf', '/rslidar_packets',\
                  '/timmo_packets', '/timmo_status', '/timoo_packets', '/timoo_packets2', '/timoo_status', '/gps_pub', '/drobot_imu',\
                  '/recordPose', '/amcl_pose', '/localization_score', '/lslidar_packet', '/lslidar_packet_difop']

  thread_check = ""
  recorder_status = False
  cache_bag_name = ""
  max_sum_bag_size = 15 * 1024
  max_per_bag_size = '500'
  process = roslaunch.scriptapi.ROSLaunch()

  def __del__(self):
    self.stopRosbagRecord()

  def __init__(self):
    self.thread_check = Thread(target = self.dfCheckThread)
    self.thread_check.start()
    total_size = self.getHomeDiskSize()
    if self.remaining_size_percent > 0.5:
      self.max_sum_bag_size = total_size * 0.5 
    else:
      self.max_sum_bag_size = total_size * 0.3 #取根目录的30%空间来存储record bag
  class RecordType():
    Default = 0
    Mapping = 1
    RunningDiff = 2
    Mapping3D = 3
    Localization = 4
    RunningAckermann = 5

  def CheckRecordingActive(self):
    return self.recorder_status,self.chach_map_name

  #开始录bag
  def startRosbagRecord(self, record_type, name=""):
    if record_type == self.RecordType.Default:
      record_topic = self.topics
    elif record_type == self.RecordType.Mapping:
      record_topic = self.slam_topics
    elif record_type == self.RecordType.Mapping3D:
      record_topic = self.slam_topics_3d
    elif record_type == self.RecordType.RunningDiff:
      record_topic = self.nav_topics_diff
    elif record_type == self.RecordType.RunningAckermann:
      record_topic = self.nav_topics_ackermann
    elif record_type == self.RecordType.Localization:
      record_topic = self.localization
    else:
      record_topic = self.topics
    topic_args = ''
    for topic in record_topic:
        topic_args += ' ' + topic
    #生成需要录制的topic
    aim_path = ""
    if self.recorder_status :
      return aim_path

    #生成需要录制的文件名
    current_time = self.getCurrentTime()
    if name == "":
      aim_path = self.bag_directory + "/" + current_time
      self.chach_map_name = current_time
    else:
      aim_path = self.bag_directory + "/" + name + "_" + current_time
      self.chach_map_name = name + "_" + current_time
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    if record_type == self.RecordType.Default:
      node = roslaunch.core.Node('rosbag', 'record', args = '-a --lz4 -x ' + self.filter_topics+' --split --size=' + self.max_per_bag_size +' -O ' + aim_path)
    else :
      node = roslaunch.core.Node('rosbag', 'record', args = '--lz4 -e ' + topic_args + ' --split --size=' + self.max_per_bag_size +' -O ' + aim_path)
    self.process = launch.launch(node)
    self.recorder_status = True
    return aim_path

  #结束录bag
  def stopRosbagRecord(self):
   if self.recorder_status == True:
    if self.process.is_alive():
      self.process.stop()
      self.recorder_status = False
      return True
    else:
      return False
   else:
      return False

  #获取系统当前时间: y-m-d-h-m-s
  def getCurrentTime(self):
    timenow = time.localtime(time.time())
    timeformat = """%d-%d-%d-%d-%d-%d""" \
      %(timenow.tm_year, timenow.tm_mon, timenow.tm_mday, timenow.tm_hour, timenow.tm_min, timenow.tm_sec)
    return timeformat

  #线程检测bag空间是否占满
  def dfCheckThread(self):
    t = threading.currentThread()
    t.do_run = True
    while t.do_run and not rospy.is_shutdown():
      parent_dir_size = self.getDirSize(self.bag_directory)
      timoo_pcap_size = self.getDirSize(self.timoo_pcap_directory)
      if parent_dir_size > self.max_sum_bag_size:
        self.freeDist()
      if timoo_pcap_size > 1024:
        self.freeTimooPcapDist()
      sleep(1)
    print ("shutdown thread")

  #获取目录文件的大小
  def getDirSize(self,dir):
    size = 0.0
    for root, dirs, files in os.walk(dir):
      for name in files:
        size += getsize(join(root, name))
    return size/1024/1024  #MB

  #获取record bag列表
  def getBagList(self):
    baglist=[]
    for parent,dirnames,filenames in os.walk(self.bag_directory):
      for filename in filenames:
        temp_path = os.path.join(parent,filename)
        #获取文件后缀名
        filename_arr = filename.split('.')
        if filename_arr[len(filename_arr) - 1] == 'bag' or filename_arr[len(filename_arr) - 1] == 'active':
          #获取bag时间
          lastmodDate = time.localtime(os.stat(temp_path).st_atime)
          file_status={}
          file_status["name"] = filename
          file_status["size"] = getsize(join(parent, filename))/1000.0/1000.0
          file_status["endTime"] = time.strftime("%Y-%m-%d %H:%M:%S", lastmodDate)
          baglist.append(file_status)
    return baglist

  #record bag空间占满后，清理较早的bag
  def freeDist(self):
    oldest_time = time.localtime(time.time())
    oldest_file_name = ''
    for parent,dirnames,filenames in os.walk(self.bag_directory):
      for filename in filenames:
        temp_path = os.path.join(parent,filename)
        #获取文件后缀名
        filename_arr = filename.split('.')
        if filename_arr[len(filename_arr) - 1] == 'bag' or filename_arr[len(filename_arr) - 1] == 'active':
          #获取该文件时间
          stats = os.stat(temp_path)
          lastmodDate = time.localtime(stats[8])
          if oldest_time > lastmodDate:
            oldest_time = lastmodDate
            oldest_file_name = temp_path
    if oldest_file_name != '':
      commands.getstatusoutput('rm ' + oldest_file_name)

  #天眸pcap包空间占满后，清理较早的pcap bag
  def freeTimooPcapDist(self):
    oldest_time = time.localtime(time.time())
    oldest_file_name = ''
    for parent,dirnames,filenames in os.walk(self.timoo_pcap_directory):
      for filename in filenames:
        temp_path = os.path.join(parent,filename)
        #获取文件后缀名
        filename_arr = filename.split('.')
        if filename_arr[len(filename_arr) - 1] == 'pcap':
          #获取该文件时间
          stats = os.stat(temp_path)
          lastmodDate = time.localtime(stats[8])
          if oldest_time > lastmodDate:
            oldest_time = lastmodDate
            oldest_file_name = temp_path
            rospy.loginfo("[master_node] delete pcap file = %s", oldest_file_name)
    if oldest_file_name != '':
      commands.getstatusoutput('rm ' + oldest_file_name)

#获取挂载在根目录磁盘的空间大小
  def getHomeDiskSize(self):
    info = os.statvfs('/') #挂载在根目录下的磁盘空间
    free_size = info.f_bsize * info.f_bavail / 1024 // 1024  #剩余空间，单位MB
    total_size = info.f_blocks * info.f_bsize / 1024 // 1024 #总空间，单位MB
    self.remaining_size_percent = round((free_size * 1.0 / total_size)*100, 2)
    return total_size
