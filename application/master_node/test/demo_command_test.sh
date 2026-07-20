
#!/bin/bash
target_ip="192.168.1.57:1819"
tartget_url="http://${target_ip}/d-robot"

# 定角度旋转(差分轮)
function CmdRotateMove()
{
  curl -X POST -i -d '{"angle":-90,"speed":0.3}'  "${tartget_url}/cmd/rotate_move"
}

# 定角度旋转(差分轮)
function CmdLinearMove()
{
  curl -X POST -i -d '{"distance":0.0,"speed":0.2}'  "${tartget_url}/cmd/linear_move"
}

# 定角度旋转(差分轮)
function CmdCheckMoveFinished()
{
  curl -X GET -i "${tartget_url}/cmd/check_move_finished"
}

# 控制机器人移动,该接口需频率发送
function CmdMove()
{
  for i in {1..10}
  do
    curl -X POST -d '{"speed":{"linearSpeed":0.2,"angularSpeed":0}}' "${tartget_url}/cmd/move"
    sleep 0.2
  done
    curl -X POST -d '{"speed":{"linearSpeed":0,"angularSpeed":0}}' "${tartget_url}/cmd/move"
}

# 发送速度0控制车辆停止可能由于减速度无法及时停止，可以通过发送brake命令强制抱死急停
function CmdBrake()
{
  curl -X POST -d '{"brake": true}' -i "${tartget_url}/cmd/move_brake"
}
# 获取软件急停状态
function CmdGetBrake()
{
  curl -X GET  -i "${tartget_url}/cmd/get_move_brake"
}

# 控制云台停止
function CmdPelcoStop()
{
  curl -X POST -d '{"direction":{"direction":0,"speed":20}}' "${tartget_url}/cmd/ptz_direction_move"
}

# 控制云台向上
function CmdPelcoUp()
{
  curl -X POST -d '{"direction":{"direction":1,"speed":20}}' "${tartget_url}/cmd/ptz_direction_move"
}

# 控制云台向下
function CmdPelcoDown()
{
  curl -X POST -d '{"direction":{"direction":2,"speed":20}}' "${tartget_url}/cmd/ptz_direction_move"
}

# 控制云台向左
function CmdPelcoLeft()
{
  curl -X POST -d '{"direction":{"direction":3,"speed":20}}' "${tartget_url}/cmd/ptz_direction_move"
}

# 控制云台向右
function CmdPelcoRight()
{
  curl -X POST -d '{"direction":{"direction":4,"speed":20}}' "${tartget_url}/cmd/ptz_direction_move"
}

# 控制云台雨刷开
function CmdPelcoRight()
{
  curl -X POST -d '{"direction":{"direction":5,"speed":20}}' "${tartget_url}/cmd/ptz_direction_move"
}

# 控制云台灯光开
function CmdPelcoRight()
{
  curl -X POST -d '{"direction":{"direction":6,"speed":20}}' "${tartget_url}/cmd/ptz_direction_move"
}
# 控制云台移动到指定角度
function CmdPelcoMove()
{
  curl -X POST -d '{"location":{"yaw":0,"pitch":180}}' "${tartget_url}/cmd/ptz_location_move"
}

# 获取云台状态
function CmdGetPelcoState()
{
  curl -X GET "${tartget_url}/sensor_data/ptz_state"
}

# 灯光控制
function CmdLightControl()
{
  curl -X POST -d '{"light_switch":{"High_light":false,"Low_light":false,"Warning_light":false,"UV_light":false,"Lock_light":false}}' "${tartget_url}/cmd/light_control"
}

# 灯光控制状态
function CmdGetLightState()
{
  curl -X GET "${tartget_url}/cmd/light_state"
}

# 开关控制
function CmdLightControl()
{
  curl -X POST '{"switchs":[{"switch_bit":2,"value":true}]}' "${tartget_url}/cmd/switch_control"
}

# 获取开关状态
function CmdLightControl()
{
  curl -X GET "${tartget_url}/sensor_data/switch_state"
}

$@
