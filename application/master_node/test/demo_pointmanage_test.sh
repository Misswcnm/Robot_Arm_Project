#!/bin/bash
target_ip="192.168.1.57:1819"
tartget_url="http://${target_ip}/d-robot"

# 该文件下接口需在选择地图后,操作当前地图下点信息
# 录制点/使用点初始化,需要在[Localization]或者[RunningTask]状态下进行

# 输入参数: 
#   $1:  状态  // 如 Idle(取消建图)
function ChangeState()
{
  curl -X GET -i "${tartget_url}/cmd/change_robot_state?state=$1"
}

# 获取地图列表
# 输入参数: 
#    $1 点名称,为空表示获取所有点信息
function PointGetPoseList(){
  curl -X GET -i  "${tartget_url}/pose_manage/get_pose?pose_name=$1"
}

# 自定义初始化 #1
# 导航前需要调用该功能确保定位能够初始化成功
# 输入参数: 
#    $1 点名称,如果为空,则需要使用Post方法传入坐标
#    $2 是否旋转初始化, 仅用于差分轮
function PointInitializeByPoint(){
  curl -X GET -i  "${tartget_url}/pose_manage/initialize_customized?point_name=$1&turning=$2"
}

# 自定义初始化 #2
function PointInitializeAtPoint(){
  curl -X POST -d '{"point": {"angle": 0,"position": {"x": 286,"y": 10}}}' -i  "${tartget_url}/pose_manage/initialize_customized?point_name=&turning=false"
}

# 记录当前点坐标
# 输入参数: 
#    $1 点名称,如果为空,则需要使用Post方法传入坐标
#    $2 点类型 #0:为初始点(已不使用) 1:为充电点 2:为导航点
# demo: `PointRecordCurrentPoint testpoint 2` 获取指定点名称的信息
function PointRecordCurrentPoint(){
  curl -X GET -i  "${tartget_url}/pose_manage/add_cur_pose?pose_name=$1&pose_type=$2"
}

# 记录指定点点坐标
# 输入参数: 
#    $1 点名称,如果为空,则需要使用Post方法传入坐标
function PointRecordPoint(){
  curl -X POST -d '{"position":{"point":{"x":10,"y":100},"angle":1.0},"name":"testpoint","type":2}' -i "${tartget_url}/pose_manage/add_pose" 
}

# 编辑已有的点
# 输入参数: 
#    $1 点名称,如果为空,则需要使用Post方法传入坐标
function PointRecordCurrentPoint(){
  curl -X POST -d '{"position":{"angle":-0.113782085999,"point":{"x":291,"y":33}}}' "${tartget_url}/pose_manage/edit_pose?pose_name=$1"
}

# 删除已有的点
# 输入参数: 
#    $1 点名称,如果为空,则需要使用Post方法传入坐标
function PointRecordCurrentPoint(){
  curl -X GET -i  "${tartget_url}/pose_manage/delete_pose?pose_name=$1"
}



$@