#!/bin/bash
target_ip="192.168.1.57:1819"
tartget_url="http://${target_ip}/d-robot"

# 该文件下接口可在除Mapping状态下接口调用


# 输入参数: 
#   $1:  状态  // 如 Idle(取消建图)
function ChangeState()
{
  curl -X GET -i "${tartget_url}/cmd/change_robot_state?state=$1"
}

# 获取地图列表
# 输入参数: 
#    $1 地图名称,为空表示获取所有地图信息
# demo: `MapGetMapsInfo` 获取所有地图信息
# demo: `MapGetMapsInfo ShellTest` 获取指定地图名称信息
function MapGetMapsInfo(){
  curl -X GET -i  "${tartget_url}/map_manage/get_map_info?map_name=$1"
}
# 获取地图图片
# 输入参数: 
#    $1 地图名称
# demo: `MapGetMapsPNG ShellTest` 获取指定地图图片
function MapGetMapsPNG(){
  curl -X GET -i  "${tartget_url}/map_manage/maps_pngs?map_name=$1"
}

# 删除地图
# 输入参数: 
#    $1 地图名称
function MapDeleteMap(){
  curl -X GET -i  "${tartget_url}/map_manage/delete_map?map_name=$1"
}

# 获取当前地图信息
function MapGetCurrentMap(){
  curl -X GET -i  "${tartget_url}/map_manage/current_map_info"
}

# 重命名地图
# 输入参数: 
#    $1 原来地图名称
#    $2 新的地图名称
function MapRenameMap(){
  curl -X GET -i  "${tartget_url}/map_manage/rename_map?origin_map_name=$1&new_map_name=$2"
}

# 切换当前地图
# 输入参数: 
#    $1 地图名称
# demo: `MapChangeMap ShellTest` 获取指定地图图片
function MapChangeMap(){
  curl -X GET -i  "${tartget_url}/map_manage/load_map?map_name=$1"
}



$@