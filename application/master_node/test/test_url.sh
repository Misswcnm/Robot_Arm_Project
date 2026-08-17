#!/bin/bash

target_ip="192.168.1.57:1819"
# target_ip="shct-robot-002-1.natapp1.cc"
tartget_url="http://${target_ip}/d-robot"
function test()
{
  curl -d "{test}" "${tartget_url}/test"
}

function test_get_autocharge_status()
{
  curl "${tartget_url}/task_manager/get_autocharge_status"
}

function test_set_autocharge_status()
{
  curl -d '{"AutoChargeSwitch":true,"RestartTask":false,"PositionName":"211"}' "${tartget_url}/task_manager/set_autocharge_status"
}


function test_ultrasonic()
{
  curl "${tartget_url}/sensor_data/raw_ultrasonic"
}
function test_bumper()
{
  curl "${tartget_url}/sensor_data/bumper"
}

function test_battery()
{
  curl "${tartget_url}/sensor_data/battery"
}

function test_meteorology()
{
  curl "${tartget_url}/sensor_data/meteorology"
}

function test_start_lanedetection()
{
  curl "${tartget_url}/lane_detection/start_lane_detection"
}
function test_cancel_navigation()
{
  curl "${tartget_url}/navigate/cancel_navigation"
}

function test_cancel_lanedetection()
{
  curl "${tartget_url}/lane_detection/cancel_lane_detection"
}

function test_save_lanedetection()
{
  curl -d '{"saveMode":"renovate","filterSwitch":true}' "${tartget_url}/lane_detection/save_lane_detection"
}
function test_pelco_switch()
{
  curl -d '{"lighting":true,"thermal":true}' "${tartget_url}/cmd/ptz_switch"
}
function test_pelco_zero()
{
  curl -d '{"direction":{"direction":5,"speed":0}}' "${tartget_url}/cmd/ptz_direction_move"
}

function test_pelco_left()
{
  curl -d '{"direction":{"direction":4,"speed":0}}' "${tartget_url}/cmd/ptz_direction_move"
}

function test_pelco_right()
{
  curl -d '{"direction":{"direction":4,"speed":25}}' "${tartget_url}/cmd/ptz_direction_move"
}

function test_pelco_up()
{
  curl -d '{"direction":{"direction":1,"speed":25}}' "${tartget_url}/cmd/ptz_direction_move"
}

function test_pelco_down()
{
  curl -d '{"direction":{"direction":2,"speed":25}}' "${tartget_url}/cmd/ptz_direction_move"
}

function test_pelco_sw()
{
  curl -d '{"direction":{"direction":8,"speed":99}}' "${tartget_url}/cmd/ptz_direction_move"
}

function test_pelco_move()
{
  curl -d '{"location":{"yaw":0,"pitch":180,"wiper_state":false}}' "${tartget_url}/cmd/ptz_location_move"
}
function test_pause()
{
  while true;do
  curl "${tartget_url}/sensor_data/robot_pose_grid"
  done
}

function test_raw_scan()
{
  curl "${tartget_url}/sensor_data/raw_scan"
}

function test_raw_odom()
{
  curl "${tartget_url}/sensor_data/raw_odom"
}


function test_robot_footprint()
{
  curl "${tartget_url}/sensor_data/robot_footprint"
}

function test_realtime_path()
{
  curl "${tartget_url}/navigate/get_realtime_path"
}


function test_add_virtual_obstacles()
{
  curl -d '{"obstacles":{"circles":[{"center":{"x":109,"y":76},"radius":60.108235708594876}],"lines":[{"start":{"x":449,"y":806},"end":{"x":560,"y":802}}],"polygons":[[{"x":476,"y":672},{"x":489,"y":748},{"x":522,"y":719},{"x":554,"y":765},{"x":569,"y":676}]],"polylines":[[{"x":476,"y":672},{"x":489,"y":748},{"x":522,"y":719},{"x":554,"y":765},{"x":569,"y":676}]],"rectangles":[{"end":{"x":613,"y":362},"start":{"x":529,"y":281}}]}}' "${tartget_url}/navigate/add_virtual_obstacles"
}

function test_get_virtual_obstacles()
{
  curl  "${tartget_url}/navigate/get_virtual_obstacles"
}

function test_get_ultrasonic()
{
  curl  "${tartget_url}/sensor_data/raw_ultrasonic"
}

function test_initial_turning()
{
  curl  "${tartget_url}/map_manage/initialize_customized?point_name=test1&turning=true"
}

function test_light_control()
{
  curl -d '{"light_switch":{"Lock_light": true,"High_light": true}}' "${tartget_url}/cmd/light_control"
}

function test_led_text_control()
{
  curl -d '{"color":0,"text":"testtest"}' "${tartget_url}/cmd/show_led_text"
}

function test_switch_control()
{
  curl -d '{"switchs":[{"switch_bit":2,"value":true}]}' "${tartget_url}/cmd/switch_control"
}

function test_initial()
{
  curl -d '{"point": {"angle": 0,"position": {"x": 286,"y": 10}}}' "${tartget_url}/pose_manage/initialize_customized?turning=true"
}

function test_rotate()
{
  curl -d '{"angle":-90,"speed":0.3}'  "${tartget_url}/cmd/rotate_move"
}

function test_linear_move()
{
  curl -d '{"distance":0.0,"speed":0.2}'  "${tartget_url}/cmd/linear_move"
}

function test_brake()
{
  curl -d '{"brake": false}'  "${tartget_url}/cmd/move_brake"
}


function test_check_rotate()
{
  curl   "${tartget_url}/cmd/check_move_finished"
}
function test_stop_rotate()
{
  curl   "${tartget_url}/cmd/stop_move"
}


function test_save_task_queue()
{
  curl -d '{"name":"aaa","tasks":[{"name":"PlayPathTask","param":{"path_name":"443"},"actions":[{"type":"WAIT","param":{"wait_time":10}},{"type":"PTZ_MOVE","param":{"yaw":10,"pitch":15}}]},{"name":"NavigationTask","param":{"point_name":"06"},"actions":[{"type":"WAIT","param":{"wait_time":5}},{"type":"PTZ_MOVE","param":{"yaw":10,"pitch":15}}]}]}' "${tartget_url}/task_manager/save_task_queue"
}
function test_get_task_queue()
{
  curl "${tartget_url}/task_manager/get_task_queue"
#   {
#     "errorCode": "",
#     "msg": "successed",
#     "data": [
#         {
#             "tasks": [
#                 {
#                     "name": "NavigationTask",
#                     "param": {
#                         "point_name": "test"
#                     }
#                 },
#                 {
#                     "name": "PlayPathTask",
#                     "param": {
#                         "path_name": "test"
#                     }
#                 },
#                 {
#                     "name": "PlayPathTask",
#                     "param": {
#                         "path_name": "test2"
#                     }
#                 }
#             ],
#             "name": "test1"
#         },
#         {
#             "tasks": [
#                 {
#                     "name": "NavigationTask",
#                     "param": {
#                         "point_name": "test1"
#                     }
#                 },
#                 {
#                     "name": "PlayPathTask",
#                     "param": {
#                         "path_name": "MANUAL_test"
#                     }
#                 }
#             ],
#             "name": "testtest"
#         }
#     ],
#     "successed": true
# }
}

function test_delete_task_queue()
{
  curl  "${tartget_url}/task_manager/delete_task_queue?task_name=$1"
}
function test_start_task_queue()
{
  curl -d '{"name":"yu","loop":false,"loop_time":0}' "${tartget_url}/task_manager/start_task_queue"
}


function test_stop_task_queue()
{
  curl  "${tartget_url}/task_manager/stop_task_queue"
}


function test_pause_task_queue()
{
  curl  "${tartget_url}/task_manager/pause_task_queue"
}

function test_resume_task_queue()
{
  curl  "${tartget_url}/task_manager/resume_task_queue"
}

function test_is_task_queue_finished()
{
  curl  "${tartget_url}/task_manager/is_task_queue_finished"
}

function test_add_cur_pose(){
  curl  "${tartget_url}/pose_manage/add_cur_pose?pose_name=&pose_type=" #0:为初始点 暂不使用 1:为充电点 2:为导航点 3:为特殊直线
}

function test_add_pose(){
  curl -d '{"position":{"point":{"x":10,"y":100},"angle":1.0},"name":"testpoint","type":0}' "${tartget_url}/pose_manage/add_pose" #0:为初始点 暂不使用 1:为充电点 2:为导航点 3:为特殊直线
}

function test_edit_pose(){
  curl -d '{"position":{"angle":-0.113782085999,"point":{"x":291,"y":33}}}' "${tartget_url}/pose_manage/edit_pose?pose_name=110" #0:为初始点 暂不使用 1:为充电点 2:为导航点 3:为特殊直线
}

function test_add_path(){
  curl -d '{"name":"test","keyPoint":[{"pointName":"0","pointAction":[{"type":"pause","param":{"millisecond":500}}],"gridPose":{"x":0,"y":0}},{"pointName":"1","pointAction":[],"gridPose":{"x":10,"y":10}}]}' "${tartget_url}/path_manage/add_path"
}

function test_generate_path(){
  curl -d '{"name":"testpath","points":[{"name":"p0","gridPosition":{"x":346,"y":61,"angle":0}},{"name":"p1","gridPosition":{"x":389,"y":28,"angle":0},"actions":[{"type":"pause","param":{"millisecond":500}}]}],"lines":[{"name":"0_1","start":"p0","end":"p1","radius":-0.5}],"path":{"line":[{"name":"0_1"}]}}' "${tartget_url}/path_manage/generate_path"
}

function test_update_path(){
  curl -d '{"name":"Aaa","points":[{"name":"p0","gridPosition":{"x":346,"y":61,"angle":0}},{"name":"p1","gridPosition":{"x":389,"y":28,"angle":0},"actions":[{"type":"pause","param":{"millisecond":500}}]}],"lines":[{"name":"0_1","start":"p0","end":"p1","radius":5}],"path":{"line":[{"name":"0_1"}]}}' "${tartget_url}/path_manage/update_path"
}

function test_check_line(){
  curl -d '{"start":{"x":157,"y":117},"end":{"x":136,"y":78},"radius":24.113682}' "${tartget_url}/path_manage/verify_path_line"
}

function test_add_tarck(){
  curl -d '{"Nodes":[{"name":"0","gridPosition":{"x":10,"y":10}},{"name":"1","gridPosition":{"x":20,"y":30}},{"name":"2","gridPosition":{"x":70,"y":90}},{"name":"3","gridPosition":{"x":66,"y":18}},{"name":"4","gridPosition":{"x":55,"y":44}},{"name":"5","gridPosition":{"x":93,"y":33}}],"Edges":[{"start":"0","end":"1","radius":5,"bidirectional":false}]}' "${tartget_url}/track_manage/add_track_graph"
}


function test_update_param(){
  curl -d '{"params":[{"type":"bool","namespace":"/navigation/follow/avoid_obstacle","value":"true"}]}' "${tartget_url}/param/update_param"
}
function test_update_system_param(){
  curl -d '{"params":[{"type":"bool","namespace":"/test/follow/avoid_obstacle","value":"true"}]}' "${tartget_url}/param/update_system_param"
}

function test_get_navigator_status(){
  curl  "${tartget_url}/navigate/get_navigator_status"
}

function test_move_to(){
  curl -d '{"position":{"point":{"x":10,"y":23},"angle":1.57},"type":0}' "${tartget_url}/navigate/move_to"
}

function test_move_to_point(){
  curl  "${tartget_url}/navigate/move_to?pose_name=ty"
}

function test_edit_map(){
  curl -d '{"obstacles":{"polygons":[[{"x":50,"y":20},{"x":50,"y":100},{"x":100,"y":50}]]}}' "${tartget_url}/map_manage/edit_map?operation_type=restore"
}

function test_upload_laser_param(){
  curl  -F "file=@$1" "${tartget_url}/param/upload_laser_param"
}
$@
