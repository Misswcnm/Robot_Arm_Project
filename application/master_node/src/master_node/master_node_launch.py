#!/usr/bin/env python
#coding=utf-8

import os
import roslaunch
import rospy


class DrobotLaunch():
    def __init__(self,uuid,launch_file):
      self.__launch_file = launch_file
      self.__uuid = uuid
      self.__roslaunch = roslaunch.parent.ROSLaunchParent(self.__uuid, [self.__launch_file])
      self.__started = False
      self.__config = None
    def config_launch_file(self,args):
      roslaunch_file=[(self.__launch_file,args)]
      self.__config = self.load_config(roslaunch_file)
    def load_config(self,roslaunch_files, verbose=False, assign_machines=True):
        config = roslaunch.ROSLaunchConfig()
        loader = roslaunch.xmlloader.XmlLoader()
        roslaunch.config.load_roscore(loader, config,False)
        for f in roslaunch_files:
            if isinstance(f,tuple):
                f, args = f
            else:
                args = None
            try:
                loader.load(f, config, argv=args, verbose=verbose)
            except roslaunch.xmlloader.XmlParseException as e:
                raise RLException(e)
            except roslaunch.loader.LoadException as e:
                raise RLException(e)
        # choose machines for the nodes
        if assign_machines:
          config.assign_machines()
        return config
    def start(self):
      if self.__started:
        self.shutdown()
      rospy.loginfo("\033[1;32m ----------> [master_node_launch.py] Start launching file %s\033[0m",self.__launch_file)
      self.__roslaunch = roslaunch.parent.ROSLaunchParent(self.__uuid, [self.__launch_file])
      self.__roslaunch.config = self.__config
      if(os.path.exists(self.__launch_file)):
          self.__roslaunch.start()
          self.__started = True
      else:
          rospy.loginfo("[master_node_launch.py] no launch file exist %s", self.__launch_file)

    def shutdown(self):
      if not self.__started:
        return
      self.__roslaunch.shutdown()
      self.__started = False

class MasterNodeLaunch():
    # 环境变量
    env_dist = os.environ
    run_workspace_dir = env_dist.get('DROBOT_SYSTEM_HOME')
    runtime_public_dir = env_dist.get('DROBOT_RUNTIME_DIR')


    # launch文件路径
    dir_launch_applications = run_workspace_dir + '/launch/applications.launch'
    dir_launch_devices = run_workspace_dir + '/launch/devices.launch'
    dir_launch_drobot_navigation = run_workspace_dir + '/launch/drobot_navigation.launch'
    dir_launch_drobot_online_mapping = run_workspace_dir + '/launch/drobot_online_mapping.launch'
    dir_launch_drobot_restart_online_mapping = run_workspace_dir + '/launch/drobot_online_addmapping.launch'
    dir_launch_test = run_workspace_dir + '/launch/test.launch'
    dir_launch_lane_detection = run_workspace_dir+ '/launch/lane_detection.launch'
    dir_launch_localization = run_workspace_dir+ '/launch/localization.launch'
    dir_launch_navigation = run_workspace_dir+ '/launch/navigation.launch'
    dir_launch_parammanager = run_workspace_dir+ '/launch/param_manager.launch'
    dir_launch_lidar_device = run_workspace_dir+ '/launch/TM16.launch'
    dir_launch_calibrate_lslidar_c32 = run_workspace_dir+ '/launch/calibrate_lslidar_c32_tf.launch'
    dir_launch_calibrate_lslidar_c16 = run_workspace_dir+ '/launch/calibrate_lslidar_c16_tf.launch'
    dir_launch_follow_target_leg = run_workspace_dir+ '/launch/follow_target.launch'
    dir_launch_follow_target_line = run_workspace_dir+ '/launch/follow_line.launch'
    # m默认四个校正后的scan_tf
    dir_launch_calibrated_lidar_tf = runtime_public_dir+ '/public/calibrated_lidar_tf.launch'
    dir_launch_calibrated_lslidar_c32_tf = runtime_public_dir+ '/public/calibrated_lslidar_c32_tf.launch'
    # dir_launch_calibrated_lslidar_c16_tf = runtime_public_dir+ '/public/calibrated_lslidar_c16_tf.launch'
    dir_launch_calibrated_laser01_tf = runtime_public_dir+ '/public/calibrated_laser01_tf.launch'
    dir_launch_calibrated_laser02_tf = runtime_public_dir+ '/public/calibrated_laser02_tf.launch'
    dir_launch_calibrated_laser03_tf = runtime_public_dir+ '/public/calibrated_laser03_tf.launch'
    dir_launch_calibrated_laser04_tf = runtime_public_dir+ '/public/calibrated_laser04_tf.launch'
#    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
#    roslaunch.configure_logging(uuid)
    def __init__(self,uuid):
      self.uuid = uuid
      self.launch_applications = DrobotLaunch(self.uuid, self.dir_launch_applications)
      self.launch_devices = DrobotLaunch(self.uuid, self.dir_launch_devices)
      self.launch_test=DrobotLaunch(self.uuid, self.dir_launch_test)
      self.launch_drobot_navigation = DrobotLaunch(self.uuid, self.dir_launch_drobot_navigation)
      self.launch_drobot_online_mapping = DrobotLaunch(self.uuid, self.dir_launch_drobot_online_mapping)
      self.launch_lane_detection = DrobotLaunch(self.uuid, self.dir_launch_lane_detection)
      self.launch_drobot_restart_online_mapping = DrobotLaunch(self.uuid, self.dir_launch_drobot_restart_online_mapping)
      self.launch_localization = DrobotLaunch(self.uuid, self.dir_launch_localization)
      self.launch_navigation = DrobotLaunch(self.uuid, self.dir_launch_navigation)
      self.launch_param_manager = DrobotLaunch(self.uuid, self.dir_launch_parammanager)
      self.launch_follow_target_leg = DrobotLaunch(self.uuid, self.dir_launch_follow_target_leg)
      self.launch_follow_target_line = DrobotLaunch(self.uuid, self.dir_launch_follow_target_line)
      self.launch_calibrate_lslidar_c32 = DrobotLaunch(self.uuid, self.dir_launch_calibrate_lslidar_c32)
      self.launch_calibrate_lslidar_c16 = DrobotLaunch(self.uuid, self.dir_launch_calibrate_lslidar_c16)
      #self.launch_lidar_device = DrobotLaunch(self.uuid, self.dir_launch_lidar_device)
      self.launch_calibrated_lidar_tf = DrobotLaunch(self.uuid, self.dir_launch_calibrated_lidar_tf)
      self.launch_calibrated_lslidar_c32_tf = DrobotLaunch(self.uuid, self.dir_launch_calibrated_lslidar_c32_tf)
      # self.launch_calibrated_lslidar_c16_tf = DrobotLaunch(self.uuid, self.dir_launch_calibrated_lslidar_c16_tf)
      self.launch_calibrated_laser01_tf = DrobotLaunch(self.uuid, self.dir_launch_calibrated_laser01_tf)
      self.launch_calibrated_laser02_tf = DrobotLaunch(self.uuid, self.dir_launch_calibrated_laser02_tf)
      self.launch_calibrated_laser03_tf = DrobotLaunch(self.uuid, self.dir_launch_calibrated_laser03_tf)
      self.launch_calibrated_laser04_tf = DrobotLaunch(self.uuid, self.dir_launch_calibrated_laser04_tf)
