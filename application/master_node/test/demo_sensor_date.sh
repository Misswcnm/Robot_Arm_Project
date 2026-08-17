
#!/bin/bash
target_ip="192.168.1.57:1819"
tartget_url="http://${target_ip}/d-robot"

# 灯光控制
function DataGetVersion()
{
  curl -X GET "${tartget_url}/cmd/get_product_version"
}

# 获取速度数据
function DataGetVelocity()
{
  curl -X GET "${tartget_url}/sensor_data/raw_velocity"
}
# 获取原始激光数据
function DataGetRawScan()
{
  curl -X GET "${tartget_url}/sensor_data/raw_scan"
}
# 获取机器人位置
function DataGetRobotPose()
{
  curl -X GET "${tartget_url}/sensor_data/robot_pose_grid"
}
# 灯光控制
function DataGetScanGrid()
{
  curl -X GET "${tartget_url}/sensor_data/scan_grid"
}
# 灯光控制
function DataGetOdom()
{
  curl -X GET "${tartget_url}/sensor_data/raw_odom"
}
# 灯光控制
function DataGetBattery()
{
  curl -X GET "${tartget_url}/sensor_data/battery"
}
# 灯光控制
function DataGetBumper()
{
  curl -X GET "${tartget_url}/sensor_data/bumper"
}
# 灯光控制
function DataGetUltrasonic()
{
  curl -X GET "${tartget_url}/sensor_data/raw_ultrasonic"
}
# 灯光控制
function DataGetIrsensor()
{
  curl -X GET "${tartget_url}/sensor_data/raw_irsensor"
}
# 灯光控制
function DataGetChassisState()
{
  curl -X GET "${tartget_url}/sensor_data/chassis_state"
}
# 灯光控制
function DataGetMeteorology()
{
  curl -X GET "${tartget_url}/sensor_data/meteorology"
}
# 灯光控制
function DataGetGps()
{
  curl -X GET "${tartget_url}/sensor_data/gps_data"
}
# 灯光控制
function DataGetGridGps()
{
  curl -X GET "${tartget_url}/sensor_data/grid_gps_data"
}
# 灯光控制
function DataGetRawImu()
{
  curl -X GET "${tartget_url}/sensor_data/raw_imu"
}
# 灯光控制
function DataGetSystemInfo()
{
  curl -X GET "${tartget_url}/sensor_data/get_system_info"
}

$@
