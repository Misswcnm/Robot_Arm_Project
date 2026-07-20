#!/bin/bash
target_ip="192.168.1.57:1819"
tartget_url="http://${target_ip}/d-robot"
# 该文件下接口需在选择地图后,操作当前地图下任务信息
# 切换状态
# 输入参数: 
#   $1:  状态  // 如 Idle(取消建图)
function ChangeState()
{
  curl -X GET -i "${tartget_url}/cmd/change_robot_state?state=$1"
}

# 更新轨道图: 
#   $1:  状态  // 如 Idle(取消建图)
function TrackUpdateTrack()
{
  curl -X POST -i -d '{"Nodes":[{"name":"0","gridPosition":{"x":10,"y":10}},{"name":"1","gridPosition":{"x":20,"y":30}},{"name":"2","gridPosition":{"x":70,"y":90}},{"name":"3","gridPosition":{"x":66,"y":18}},{"name":"4","gridPosition":{"x":55,"y":44}},{"name":"5","gridPosition":{"x":93,"y":33}}],"Edges":[{"start":"0","end":"1","radius":5,"bidirectional":false}]}' "${tartget_url}/track_manage/add_track_graph"
}


# 获取轨道图: 
function TrackUpdateTrack()
{
  curl -X GET -i "${tartget_url}/track_manage/get_track_graph"
}

$@