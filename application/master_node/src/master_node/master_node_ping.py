#!/usr/bin/env python
#coding=utf-8

import commands

class MasterNodePing():
    def ping_base_link_to_imu(self):
        (status, output) = commands.getstatusoutput('rosnode ping -c 1 /base_link_to_imu')
        # 查找输出是否存在ERROR，没有说明ping通了
        if output.find('ERROR') == -1 and output.find('unknown node') == -1:
            return 'alive'
        else:
            return 'dead'
        pass

    def ping_base_link_to_laser(self):
        (status, output) = commands.getstatusoutput('rosnode ping -c 1 /base_link_to_laser')
        # 查找输出是否存在ERROR，没有说明ping通了
        if output.find('ERROR') == -1 and output.find('unknown node') == -1:
            return 'alive'
        else:
            return 'dead'
        pass

    def ping_scan_to_cloud_converter_node(self):
        (status, output) = commands.getstatusoutput('rosnode ping -c 1 /scan_to_cloud_converter_node')
        # 查找输出是否存在ERROR，没有说明ping通了
        if output.find('ERROR') == -1 and output.find('unknown node') == -1:
            return 'alive'
        else:
            return 'dead'
        pass

    def ping_drobot_device_node(self):
        (status, output) = commands.getstatusoutput('rosnode ping -c 1 /drobot_device_node')
        # 查找输出是否存在ERROR，没有说明ping通了
        if output.find('ERROR') == -1 and output.find('unknown node') == -1:
            return 'alive'
        else:
            return 'dead'
        pass

    def ping_sick_lms1xx(self):
        (status, output) = commands.getstatusoutput('rosnode ping -c 1 /lms1xx')
        # 查找输出是否存在ERROR，没有说明ping通了
        if output.find('ERROR') == -1 and output.find('unknown node') == -1:
            return 'alive'
        else:
            return 'dead'
        pass

    def ping_lidar3d(self):
        (status1, output1) = commands.getstatusoutput('rosnode ping -c 1 /rslidar_nodelet_manager')
        (status2, output2) = commands.getstatusoutput('rosnode ping -c 1 /rslidar_nodelet_manager_cloud')
        (status3, output3) = commands.getstatusoutput('rosnode ping -c 1 /rslidar_nodelet_manager_driver')
        # 查找输出是否存在ERROR，没有说明ping通了
        if (output1.find('ERROR') == -1 and output1.find('unknown node') == -1) and (output2.find('ERROR') == -1 and output2.find('unknown node') == -1) and (output3.find('ERROR') == -1 and output3.find('unknown node') == -1):
            return 'alive'
        else:
            return 'dead'
        pass

    def ping_sick_tim571(self):
        (status, output) = commands.getstatusoutput('rosnode ping -c 1 /sick_tim571_2050101')
        # 查找输出是否存在ERROR，没有说明ping通了
        if output.find('ERROR') == -1 and output.find('unknown node') == -1:
            return 'alive'
        else:
            return 'dead'
        pass

    def ping_xiaolv_base_node(self):
        (status, output) = commands.getstatusoutput('rosnode ping -c 1 /xiaolv_base_node')
        # 查找输出是否存在ERROR，没有说明ping通了
        if output.find('ERROR') == -1 and output.find('unknown node') == -1:
            return 'alive'
        else:
            return 'dead'
        pass

    def ping_robot_pose_publisher_node(self):
        (status, output) = commands.getstatusoutput('rosnode ping -c 1 /robot_pose_publisher_node')
        # 查找输出是否存在ERROR，没有说明ping通了
        if output.find('ERROR') == -1 and output.find('unknown node') == -1:
            return 'alive'
        else:
            return 'dead'
        pass

    def ping_path_record_node(self):
        (status, output) = commands.getstatusoutput('rosnode ping -c 1 /path_record_node')
        # 查找输出是否存在ERROR，没有说明ping通了
        if output.find('ERROR') == -1 and output.find('unknown node') == -1:
            return 'alive'
        else:
            return 'dead'
        pass

    def ping_drobot_task_manager(self):
        (status, output) = commands.getstatusoutput('rosnode ping -c 1 /drobot_task_manager')
        # 查找输出是否存在ERROR，没有说明ping通了
        if output.find('ERROR') == -1 and output.find('unknown node') == -1:
            return 'alive'
        else:
            return 'dead'
        pass

    def ping_drobot_pose_manager(self):
        (status, output) = commands.getstatusoutput('rosnode ping -c 1 /drobot_pose_manager')
        # 查找输出是否存在ERROR，没有说明ping通了
        if output.find('ERROR') == -1 and output.find('unknown node') == -1:
            return 'alive'
        else:
            return 'dead'
        pass

    def ping_map_manager_node(self):
        (status, output) = commands.getstatusoutput('rosnode ping -c 1 /map_manager_node')
        # 查找输出是否存在ERROR，没有说明ping通了
        if output.find('ERROR') == -1 and output.find('unknown node') == -1:
            return 'alive'
        else:
            return 'dead'
        pass

    def ping_localization(self):
        (status, output) = commands.getstatusoutput('rosnode ping -c 1 /localization')
        # 查找输出是否存在ERROR，没有说明ping通了
        if output.find('ERROR') == -1 and output.find('unknown node') == -1:
            return 'alive'
        else:
            return 'dead'
        pass

    def ping_bt_navigator_node(self):
        (status, output) = commands.getstatusoutput('rosnode ping -c 1 /bt_navigator_node')
        # 查找输出是否存在ERROR，没有说明ping通了
        if output.find('ERROR') == -1 and output.find('unknown node') == -1:
            return 'alive'
        else:
            return 'dead'
        pass

    def ping_cartographer_node(self):
        (status, output) = commands.getstatusoutput('rosnode ping -c 1 /cartographer_node')
        # 查找输出是否存在ERROR，没有说明ping通了
        if output.find('ERROR') == -1 and output.find('unknown node') == -1:
            return 'alive'
        else:
            return 'dead'
        pass

    def ping_cartographer_occupancy_grid_node(self):
        (status, output) = commands.getstatusoutput('rosnode ping -c 1 /cartographer_occupancy_grid_node')
        # 查找输出是否存在ERROR，没有说明ping通了
        if output.find('ERROR') == -1 and output.find('unknown node') == -1:
            return 'alive'
        else:
            return 'dead'
        pass

    def ping_sick_lane_detection(self):
        (status, output) = commands.getstatusoutput('rosnode ping -c 1 /sick_lane_detection')
        # 查找输出是否存在ERROR，没有说明ping通了
        if output.find('ERROR') == -1 and output.find('unknown node') == -1:
            return 'alive'
        else:
            return 'dead'
        pass
    def ping_drobot_manager_console(self):
        (status, output) = commands.getstatusoutput('rosnode ping -c 1 /drobot_manager_console_node')
        # 查找输出是否存在ERROR，没有说明ping通了
        if output.find('ERROR') == -1 and output.find('unknown node') == -1:
            return 'alive'
        else:
            return 'dead'
        pass
