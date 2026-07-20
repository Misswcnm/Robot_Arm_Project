#!/usr/bin/env python
from enum import Enum
from enum import IntEnum, unique
from datetime import datetime, date
import rospy
import std_msgs.msg
import master_node.master_node_launch as MasterNodeLaunch
import master_node.initial_pose as MappingParamInit
from common_service.msg import RobotState
import thread
import threading
import time
import os

class MasterNodeState():
    @unique
    class state_type(Enum):
      Idle = "Idle"
      Mapping = "Mapping"
      RunningTask = "RunningTask"
      RestartMapping = "RestartMapping"
      Localization = "Localization"
      Sleep = "Sleep"
      WakingUp = "WakingUp"
      FollowTarget = "FollowTarget"
    state_pub_ = None
    state_initialized = False
    state_operation=state_type.Idle
    target_state = state_type.Idle
    lock = threading.Lock()
    log_path = os.environ.get('DROBOT_RUNTIME_DIR') +  '/log_dir/master_node-' + str(datetime.today().weekday()+1) + '.log'
    log_name = open(log_path, "a", buffering = 0)
    def __init__(self,uuid):
      self.master_node_launch = MasterNodeLaunch.MasterNodeLaunch(uuid)
      self.state_operation=self.state_type.Idle
      self.state_pub_ = rospy.Publisher("/robot_state", RobotState, queue_size=10,latch=True)
      state_initialized = False
      pass
    def __del__(self):
      self.log_name.close()
    def editState(self,state_type):
      if(state_type == self.state_type.Idle):
        self.state_operation=self.state_type.Idle
        self.state_pub_.publish(RobotState.IDLE)
      if(state_type == self.state_type.Mapping):
        self.state_operation=self.state_type.Mapping
        self.state_pub_.publish(RobotState.MAPPING)
      if(state_type == self.state_type.RunningTask):
        self.state_operation=self.state_type.RunningTask
        self.state_pub_.publish(RobotState.RUNNINGTASK)
      if(state_type == self.state_type.RestartMapping):
        self.state_operation=self.state_type.RestartMapping
        self.state_pub_.publish(RobotState.RESTARTMAPPING)
      if(state_type == self.state_type.Localization):
        self.state_operation=self.state_type.Localization
        self.state_pub_.publish(RobotState.LOCALIZATION)
      if(state_type == self.state_type.Sleep):
        self.state_operation=self.state_type.Sleep
        self.state_pub_.publish(RobotState.SLEEP)
      if(state_type == self.state_type.WakingUp):
        self.state_operation=self.state_type.WakingUp
        self.state_pub_.publish(RobotState.WAKINGUP)
      if(state_type == self.state_type.FollowTarget):
        self.state_operation=self.state_type.FollowTarget
        self.state_pub_.publish(RobotState.FOLLOWTARGET)
    def checkState(self,state_types):
      for states in state_types:
        if self.state_operation == states:
          return True
      return False
      pass
    def startProgram(self,data):
        rospy.loginfo("[master_node_state.py] param inited")
        self.initied = True
        self.init_sub.unregister()
        return
    def defaultLoad(self):
      rospy.loginfo("[master_node_state.py] Start loading [param] launch... ....")
      self.init_sub = rospy.Subscriber("/param_init",std_msgs.msg.Empty,self.startProgram,queue_size = 1 )
      self.master_node_launch.launch_param_manager.start()
      self.initied = False
      count = 0
      while not self.initied and not rospy.is_shutdown():
        rospy.loginfo("wait")
        rospy.sleep(1)
        count+=1
        if(count > 10): break
      rospy.loginfo("[master_node_state.py] Start loading [devices] launch... ....")
      self.master_node_launch.launch_devices.start()
      rospy.sleep(3)
      self.master_node_launch.launch_applications.start()
      rospy.sleep(3)
      self.master_node_launch.launch_calibrated_lidar_tf.start()
      self.master_node_launch.launch_calibrated_laser01_tf.start()
      self.master_node_launch.launch_calibrated_laser02_tf.start()
      self.master_node_launch.launch_calibrated_laser03_tf.start()
      self.master_node_launch.launch_calibrated_laser04_tf.start()
      self.master_node_launch.launch_calibrated_lslidar_c32_tf.start()
      # self.master_node_launch.launch_calibrated_lslidar_c16_tf.start()
      self.editState(self.state_type.Idle)

    def restartCalibratedLidarTF(self):
        self.master_node_launch.launch_calibrated_lidar_tf.start()

    def restartCalibratedLaser01TF(self):
        self.master_node_launch.launch_calibrated_laser01_tf.start()

    def restartCalibratedLaser02TF(self):
        self.master_node_launch.launch_calibrated_laser02_tf.start()

    def restartCalibratedLaser03TF(self):
        self.master_node_launch.launch_calibrated_laser03_tf.start()

    def restartCalibratedLaser04TF(self):
        self.master_node_launch.launch_calibrated_laser04_tf.start()

    def restartCalibratedLSLidar32TF(self):
        self.master_node_launch.launch_calibrated_lslidar_c32_tf.start()

    # def restartCalibratedLSLidar16TF(self):
    #     self.master_node_launch.launch_calibrated_lslidar_c16_tf.start()

    def restartLidarDevice(self):
        self.master_node_launch.launch_lidar_device.start()

    def changeState(self,state,args = [],arguement = ""):
      rospy.loginfo("[master_node_state.py] Change state [%s]",state)
      self.log_name.write(str(datetime.now().strftime('%Y-%m-%d %H:%M:%S')) + ' =========Change state========= ' + '\n')
      self.log_name.write(str(datetime.now().strftime('%Y-%m-%d %H:%M:%S')) + ' Current state = ' + str(self.state_operation) + '\n')
      self.log_name.write(str(datetime.now().strftime('%Y-%m-%d %H:%M:%S')) + ' Change state = ' + str(state) + '\n')
      self.lock.acquire()
      self.target_state = state
      if self.state_operation == state:
        self.lock.release()
        return True
      if state == self.state_type.Sleep:
        self.startSleep()
      elif state == self.state_type.Idle:
        self.IdleState()
      elif state == self.state_type.Mapping:
        self.startMapping()
      elif state == self.state_type.Localization:
        self.startLocalization()
      elif state == self.state_type.RunningTask:
        self.startNavigation()
      elif state == self.state_type.RestartMapping:
        ret = self.startReMapping(args)
        self.lock.release()
        return ret
      elif state == self.state_type.FollowTarget:
        if arguement is "car":
          self.startFollowCar()
        elif arguement is "people":
          self.startFollowPeople()
        elif arguement is "":
          self.startFollowTarget()
      else:
        self.lock.release()
        return False
      rospy.loginfo("[master_node_state.py] Current state %s",self.state_operation)
      self.lock.release()
      return True
    def startLocalization(self):
      if self.state_operation == self.state_type.Localization:
        return True
      if self.state_operation != self.state_type.Idle and self.state_operation != self.state_type.RunningTask:
        self.IdleState()
      if self.state_operation == self.state_type.Idle:
        self.master_node_launch.launch_localization.start()
      elif self.state_operation == self.state_type.RunningTask:
        self.stopNavigation()
      self.editState(self.state_type.Localization)
      return True
    def stopLocalization(self):
      if self.state_operation != self.state_type.Localization:
        return False
      self.master_node_launch.launch_localization.shutdown()
      self.editState(self.state_type.Idle)
      return True

    def startNavigation(self):
      if self.state_operation == self.state_type.RunningTask:
        return True
      if self.state_operation != self.state_type.Localization:
        self.startLocalization()
      self.master_node_launch.launch_navigation.start()
      self.editState(self.state_type.RunningTask)
      return True
    def stopNavigation(self):
      if self.state_operation != self.state_type.RunningTask:
        return False
      rospy.loginfo("stop navigation")
      self.master_node_launch.launch_navigation.shutdown()
      self.editState(self.state_type.Localization)
      return True

    def startMapping(self):
      if self.state_operation == self.state_type.Mapping:
        return True
      if self.state_operation != self.state_type.Idle:
        self.IdleState()
      MappingParamInit.reset_initial_pose()
      self.master_node_launch.launch_drobot_online_mapping.start()
      self.editState(self.state_type.Mapping)
      return True
    def startReMapping(self,args):
      if self.state_operation == self.state_type.RestartMapping:
        return True
      if self.state_operation != self.state_type.RunningTask and self.state_operation != self.state_type.Localization:
        return False
      self.IdleState()
      MappingParamInit.get_initial_pose()
      self.master_node_launch.launch_drobot_restart_online_mapping.config_launch_file(args)
      self.master_node_launch.launch_drobot_restart_online_mapping.start()
      self.editState(self.state_type.RestartMapping)
      return True
    def stopReMapping(self):
      if self.state_operation != self.state_type.RestartMapping:
        return False
      self.master_node_launch.launch_drobot_restart_online_mapping.shutdown()
      self.editState(self.state_type.Idle)
      return True
    def stopMapping(self):
      if self.state_operation != self.state_type.Mapping:
        return False
      self.master_node_launch.launch_drobot_online_mapping.shutdown()
      self.editState(self.state_type.Idle)
      return True
    def IdleState(self):
      if self.state_operation == self.state_type.Idle:
        return True
      if self.state_operation == self.state_type.Sleep:
        self.stopSleep()
      elif self.state_operation == self.state_type.WakingUp:
        time.sleep(1)
      elif self.state_operation == self.state_type.Localization:
        self.stopLocalization()
      elif self.state_operation == self.state_type.RunningTask:
        self.stopNavigation()
      elif self.state_operation == self.state_type.Mapping:
        self.stopMapping()
      elif self.state_operation == self.state_type.RestartMapping:
        self.stopReMapping()
      elif self.state_operation == self.state_type.FollowTarget:
        self.stopFollowTarget()
      else:
        rospy.logerr("[master_node_state.py] Unknow state,unable back to Idle")
        return False
      return self.IdleState()
    def startSleep(self):
      self.IdleState()
      if self.state_operation == self.state_type.Idle:
        self.editState(self.state_type.Sleep)
        return True
    def stopSleep(self):
      if self.state_operation == self.state_type.Sleep:
        rospy.loginfo("[master_node_state.py] Start wake up")
        def wakeUp():
          if self.state_operation != self.state_type.Sleep:
            return
          self.editState(self.state_type.WakingUp)
          time.sleep(8)
          self.editState(self.state_type.Idle)
          # self.changeState(self.target_state)
        thread.start_new_thread(wakeUp,())
        time.sleep(1)
    def startFollowTarget(self):
      if self.state_operation != self.state_type.Idle:
        self.IdleState()
      self.editState(self.state_type.FollowTarget)
      return True
    def startFollowPeople(self):
      if self.state_operation != self.state_type.Idle:
        self.IdleState()
      self.master_node_launch.launch_follow_target_leg.start()
      self.editState(self.state_type.FollowTarget)
      return True
    def startFollowCar(self):
      if self.state_operation != self.state_type.Idle:
        self.IdleState()
      self.master_node_launch.launch_follow_target_line.start()
      self.editState(self.state_type.FollowTarget)
      return True
    def stopFollowTarget(self):
      if self.state_operation != self.state_type.FollowTarget:
        return False
      self.master_node_launch.launch_follow_target_leg.shutdown()
      self.master_node_launch.launch_follow_target_line.shutdown()
      self.editState(self.state_type.Idle)
      return True
