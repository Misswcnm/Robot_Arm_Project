#!/bin/bash
target_ip="192.168.1.57:1819"
tartget_url="http://${target_ip}/d-robot"

# 该文件下接口需在选择地图后,操作当前地图下任务信息

# 输入参数: 
#   $1:  状态  // 如 Idle(取消建图)
function ChangeState()
{
  curl -X GET -i "${tartget_url}/cmd/change_robot_state?state=$1"
}

# 导航至点#1
# 输入参数: 
#    $1 点名称
function TaskMoveToRecordPoint(){
  curl  -X GET -i "${tartget_url}/navigate/move_to?pose_name=$1"
}

# 导航至指定点#2
# 输入参数: 
#    指定点位置
function TaskMoveToPoint(){
  curl  -X POST -d '{"position":{"point":{"x":10,"y":23},"angle":1.57},"type":0}' "${tartget_url}/navigate/move_to"
}

# 开始跟随路径
# 输入参数: 
#    $1 指定路径名
function TaskFollowPath(){
  curl  -X GET -i "${tartget_url}/navigate/follow_path?path_name=$1"
}

# 创建任务队列
function TaskCreateTask(){
  curl -X POST -d '{"name":"aaa","tasks":[{"name":"PlayPathTask","param":{"path_name":"443"},"actions":[{"type":"WAIT","param":{"wait_time":10}},{"type":"PTZ_MOVE","param":{"yaw":10,"pitch":15}}]},{"name":"NavigationTask","param":{"point_name":"06"},"actions":[{"type":"WAIT","param":{"wait_time":5}},{"type":"PTZ_MOVE","param":{"yaw":10,"pitch":15}}]}]}' "${tartget_url}/task_manager/save_task_queue"
}


# 删除任务队列
function TaskDeleteTask(){
  curl -X GET -i "${tartget_url}/task_manager/delete_task_queue?task_name=$1"
}

#获取任务列表
# 输入参数: 
#    $1 任务名称,为空时表示获取所有任务
function TaskDeleteTask(){
  curl -X GET -i "${tartget_url}/task_manager/get_task_queue?task_name=$1"
}

#获取任务列表
# 输入参数: 
#    $1 任务名称,为空时表示获取所有任务
function TaskDeleteTask(){
  curl -X GET -i "${tartget_url}/task_manager/get_task_queue?task_name=$1"
}

# 开始执行任务
function TaskStartTask(){
  curl -X POST -i -d '{"name":"yu","loop":false,"loop_time":0}' "${tartget_url}/task_manager/start_task_queue"
}

# 暂停执行任务
function TaskPauseTask(){
  curl -X GET -i "${tartget_url}/task_manager/pause_task_queue"
}

# 停止执行任务
function TaskStopTask(){
  curl -X GET -i "${tartget_url}/task_manager/stop_task_queue"
}

# 继续执行任务
function TaskResumeTask(){
  curl -X GET -i "${tartget_url}/task_manager/resume_task_queue"
}

# 获取任务状态
function TaskGetTaskStatu(){
  curl -X GET -i "${tartget_url}/task_manager/get_task_status"
}


# 获取导航过程中状态
function TaskNavigateStatu(){
  curl -X GET -i "${tartget_url}/navigate/get_navigator_status"
}

# 获取导航规划路径
function TaskRealTimePath(){
  curl -X GET -i "${tartget_url}/navigate/get_realtime_path"
}

# 获取完成的任务报告
# 输入参数: 
#    $1 开始utc时间
#    $2 结束utc时间
#    $3 当前页码
#    $4 分页大小,默认为50
function TaskGetTaskReport(){
  curl -X GET -i "${tartget_url}/task_manager/get_task_report?start_time=$1&end_time=$2&current_page=$3&page_size=$4"
}

# 获取完成的任务报告图片
# 输入参数: 
#    $1 任务报告id
function TaskGetTaskReportImage(){
  curl -X GET -i "${tartget_url}/task_manager/get_report_image?id=$1"
}


$@