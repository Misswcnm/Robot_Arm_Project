#!/bin/bash
target_ip="192.168.1.57:1819"
tartget_url="http://${target_ip}/d-robot"

# 该文件下API请求(除继续建图)需要切换至建图(Mapping)状态才可使用

## 开始建图
# 该接口可在任何状态下切换至建图状态,也可以使用该接口结束建图状态
# 输入参数: 
#   $1:  状态  // 如:Mapping(进入建图), Idle(取消建图)
function ChangeState()
{
  curl -X GET -i "${tartget_url}/cmd/change_robot_state?state=$1"
}
# 或使用以下接口 开始建图
function MappingStartScanMapping()
{
  curl -X GET -i "${tartget_url}/mapping/start_scan_map"
}
# 或使用以下接口 取消建图
function MappingCancleScanMapping()
{
  curl -X GET -i "${tartget_url}/mapping/cancel_scan_map"
}

# 获取实时建图数据
function MappingGetMappingData()
{
  curl -X GET -i "${tartget_url}/mapping/scan_map_png"
}

# 停止并保存建图
# 输入参数
#   $1: 地图名称
#   $2: 旋转角度
# demo `MappingStopSaveScanMap ShellTest 0.0`
function MappingStopSaveScanMap()
{
  #如:curl -X GET -i "${tartget_url}/cmd/stop_scan_map?map_name=test&?angle=0.0"
  curl -X GET -i "${tartget_url}/mapping/stop_scan_map?map_name=$1&?angle=$2"
}

# 继续建图需要切换至[RunningTask]或[Localization]状态下才能正确调用,请调用接口[change_robot_state]
# 在[Localization]或者[RunningTask]状态下确保定位准确后才能开始继续建图
# 调用[stop_scan_map]进行保存地图
function MappingRestartScanMap(){
  curl -X GET -i "${tartget_url}/mapping/restart_scan_map"
}

# 建图时记录当前点,保存地图后会在点列表中生成点信息
# 输入参数
#    $1: 点名称
# demo: `MappingRecordMappingPoint test1`
function MappingRecordMappingPoint(){
  #如: curl -X GET -i "${tartget_url}/mapping/record_mapping_pose?pose_name=testpoint"
  curl -X GET -i "${tartget_url}/mapping/record_mapping_pose?pose_name=$1"
}

$@