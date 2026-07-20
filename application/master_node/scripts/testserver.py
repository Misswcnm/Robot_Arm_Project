#!/usr/bin/env python2
import sys
import os

from tornado.ioloop import IOLoop
from tornado.web import RequestHandler
from tornado.web import Application
from tornado.websocket import WebSocketHandler
from tornado.httpserver import HTTPServer
from rosbridge_library.util import json, bson

import rospy

import actionlib
from tf.transformations import quaternion_from_euler
from actionlib_msgs.msg import *
from std_srvs.srv import Empty, EmptyRequest
from common_service.srv import *


class MasterNode:
    service_lookup_table = {}
    str_common_op = ''
    str_navigation_op = ''
    str_mapping_op = ''
    str_online_mapping_op = ''
    str_offline_mapping_op = ''
    str_drobot_task_op = ''
    str_map_edit_op = ''
    str_map_manage_op = ''
    str_record_bag_op = ''

    str_look = ''
    str_launch_= ''
    str_shutdown = ''
    a = 3434
    class IndexHandler(RequestHandler):


        def initialize(self):
            # add look up table
            print('initialize')

        def parse_operation(self,operation):
            operation_module = operation.partition('/')[0]
            print("received operation" + operation)
            if operation_module == 'navigate' or operation_module == 'map_manage' or operation_module == 'map_edit' or operation_module == 'initialize':
                #check if navigation is launched properly
                shell_cmd = 'bash ' + self.str_navigation_op + self.str_look
                print('executing shell: ' + shell_cmd)
                shell_res = os.popen(shell_cmd).read().splitlines()
                #for debug usage:
                for str in shell_res:
                    print('shell out: '+ str)
                #TODO: use better way to check
                if len(shell_res) != 9:
                    # some node hasn't launched properly
                    rospy.logwarn("navigation hasn't launched properly, start navigation now.")
                    shell_cmd = 'bash ' + self.str_navigation_op + self.str_launch
                    print('executing shell: ' + shell_cmd)
                    shell_res = os.popen(shell_cmd).read().splitlines()
                    #for debug usage:
                    for str in shell_res:
                        print('shell out: '+ str)
                    return False
            elif operation_module == 'mapping':
                #shutdown navigation first
                shell_cmd = 'bash ' + self.str_navigation_op + self.str_shutdown
                print('executing shell: ' + shell_cmd)
                shell_res = os.popen(shell_cmd).read().splitlines()
                #for debug usage:
                for str in shell_res:
                    print('shell out: '+ str)

            elif operation_module == 'task_manage':
                shell_cmd = 'bash ' + self.str_drobot_task_op + self.str_look
                print('executing shell: ' + shell_cmd)
                shell_res = os.popen(shell_cmd).read().splitlines()
                #for debug usage:
                for str in shell_res:
                    print('shell out: '+ str)
                #TODO: use better way to check
            elif operation_module == 'shutdown_navigate':
                shell_cmd = 'bash ' + self.str_navigation_op + self.str_shutdown
                print('executing shell: ' + shell_cmd)
                shell_res = os.popen(shell_cmd).read().splitlines()
                #for debug usage:
                for str in shell_res:
                    print('shell out: '+ str)
            return self.service_lookup_table.has_key(operation)

        def get(self,operation):
            print(operation)
            print(parent.a)

        def post(self,operation):
            print(operation)


def shutdown_hook():
    IOLoop.current().stop()

if __name__ == "__main__":

    rospy.init_node("rosbridge_http")
    rospy.on_shutdown(shutdown_hook)
    print("start http server listening to port 1819")
    app = Application([(r"/d-robot/wtf/",EchoWebSocket)])
    # server = HTTPServer(app)
    # server.listen(1819)
    app.listen(1819)
    IOLoop.current().start()
