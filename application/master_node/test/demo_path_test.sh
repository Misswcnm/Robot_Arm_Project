#!/bin/bash
target_ip="192.168.1.57:1819"
tartget_url="http://${target_ip}/d-robot"

# 该文件下接口需在选择地图后,定位完成后,操作当前地图下路径


# 输入参数: 
#   $1:  状态  // 如 Idle(取消建图)
function ChangeState()
{
  curl -X GET -i "${tartget_url}/cmd/change_robot_state?state=$1"
}

# 开始录制路径, 请确保当前定位准确
function PathStartRecord(){
  curl -X GET -i  "${tartget_url}/path_manage/start_record_path"
}

# 停止并保存录制路径, 请确保当前定位准确
# 输入参数: 
#    $1 路径名称
function PathSaveRecord(){
  curl -X GET -i  "${tartget_url}/path_manage/save_record_path?path_name=$1"
}

#取消录制路径
function PathCancleRecord(){
  curl -X GET -i  "${tartget_url}/path_manage/cancel_record_path"
}

#获取录制路径状态
function PathGetRecordStatu(){
  curl -X GET -i  "${tartget_url}/path_manage/get_record_status"
}

#获取路径列表
function PathGetPathList(){
  curl -X GET -i  "${tartget_url}/path_manage/get_path_list"
}

#获取路径数据
# 输入参数: 
#    $1 路径名称
function PathGetPathData(){
  curl -X GET -i  "${tartget_url}/path_manage/get_path?path_name=$1"
}

#删除路径
# 输入参数: 
#    $1 路径名称
function PathDeletePath(){
  curl -X GET -i  "${tartget_url}/path_manage/delete_path?path_name=$1"
}

#生成手绘路径
function PathGeneratePath(){
  curl -X POST -d '{"name":"testpath","points":[{"name":"p0","gridPosition":{"x":346,"y":61,"angle":0}},{"name":"p1","gridPosition":{"x":389,"y":28,"angle":0},"actions":[{"type":"pause","param":{"millisecond":500}}]}],"lines":[{"name":"0_1","start":"p0","end":"p1","radius":-0.5}],"path":{"line":[{"name":"0_1"}]}}' -i "${tartget_url}/path_manage/generate_path"
}

#生成验证两点间是否能够生成路径
function PathVerifyPath(){
  curl -X POST -d '{"start":{"x":157,"y":117},"end":{"x":136,"y":78},"radius":24.113682}' -i "${tartget_url}/path_manage/verify_path_line"
}

#获取手绘路径列表
function PathGetManualPath(){
  curl -X GET -i  "${tartget_url}/path_manage/get_manual_path"
}

#更新手绘路径列表
function PathUpdateManualPath(){
  curl -X POST -d '{"name":"testpath","points":[{"name":"p0","gridPosition":{"x":346,"y":61,"angle":0}},{"name":"p1","gridPosition":{"x":389,"y":28,"angle":0},"actions":[{"type":"pause","param":{"millisecond":500}}]}],"lines":[{"name":"0_1","start":"p0","end":"p1","radius":5}],"path":{"line":[{"name":"0_1"}]}}' "${tartget_url}/path_manage/update_path"
}

$@