#!/usr/bin/env python
#coding:utf-8

import sys
import os
sys.path.append(os.environ.get('DROBOT_DKDLL_DIR') + '/python2.7')
import paho.mqtt.client as mqtt
import threading
import time
import sys
import rospy
import json

class BaseMqtt:
  sub_table = {}
  submsg_table = {}
  submsg_recived_table = {}

  #INIT
  def __init__(self):
    self.vehicleId_ = rospy.get_param('~vehicleId', 'SMRPatrol001')
    self.client_ = mqtt.Client('VEHICLE_'+ str(self.vehicleId_) + '_PUB', protocol = 3)
    self.client_.reconnect_interval = 10
    self.init()

  def __del__(self):
    self.client_.disconnect()

  def init(self):
    # self.client_ = mqtt.Client(protocol=3)
    self.client_.username_pw_set("zzhkgcruiser", "sX5qY8mK4lL9wO9i")
    self.client_.on_connect = self.on_connect
    # self.client_.on_disconnect = self.on_disconnect
    self.client_.on_subscribe = self.on_subscribe
    self.client_.on_message = self.on_message
    self.client_.max_retry = 5
    for i in range(self.client_.max_retry):
      try:
        self.client_.connect(host="36.137.228.79", port = 1883, keepalive=60) #客户端连接到代理服务端，60心跳时间
        break  # 如果连接成功，跳出循环
      except Exception as e:
        time.sleep(2)  # 如果连接失败，等待2秒后再次尝试连接
    #创建线程 启动客户端
    thread1 = threading.Thread(target=self.client_.loop_forever)
    thread1.start()

  #MQTT CALLBACK
  def on_connect(self, client, userdata, flags, rc):
    rc_status = ["连接成功", "协议版本错误", "无效的客户端标识", "服务器无法使用", "用户名或密码错误", "无授权"]
    if rc == 0:
      rospy.loginfo("连接成功")
      pass
    else:
      rospy.loginfo("mqtt连接失败: %s"%rc_status[rc])

  # 断开连接回调函数
  # def on_disconnect(self, client, userdata, rc):
  #   rospy.loginfo("Disconnected with result code " + str(rc))
  #   # 发起重连
  #   self.client_.reconnect()

  def on_message(self, client, userdata, msg):
    rospy.loginfo(msg.topic + " " + str(msg.payload))

  def on_subscribe(self, client,userdata,mid,granted_qos):
    rospy.loginfo("消息发送成功")


  #ROS CALLBACK
  def gps_cb(self,data):
    print("开始发送gps数据")
    if self.submsg_recived_table['gps'] == False:
      self.submsg_recived_table['gps'] = True
    gps_msg={}
    gps_msg['latitude'] = data.Latitude
    gps_msg['longitude'] = data.Longitude
    gps_msg['altitude'] = data.Altitude
    gps_msg['quality'] = data.Quality

    #mqtt协议
    mqtt_cmd_data = {"gps_data": gps_msg}
    json_data = json.dumps(mqtt_cmd_data, ensure_ascii = False)
    self.client_.publish(topic="prod/v1/vpub/vehicle/realtime/TEST001", payload=json_data, qos=0)

    self.submsg_table['gps'] = gps_msg
    print("成功发送gps数据")
    pass


  #FUNCTION
  def create_subscriber(self,table_name,topic_name,topic_type,topic_callback,queuesize=30):
    self.sub_table[table_name] = rospy.Subscriber(topic_name,topic_type,topic_callback,queue_size = queuesize )
    self.submsg_table[table_name] = {}
    self.submsg_recived_table[table_name] = False
    pass

  def check_reviced(self,table_name):
    if not table_name in self.submsg_recived_table:
      return False
    if self.submsg_recived_table[table_name] == None or self.submsg_recived_table[table_name] == False:
      return False
    return True
