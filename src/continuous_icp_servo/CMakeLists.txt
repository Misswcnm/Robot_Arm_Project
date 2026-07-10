cmake_minimum_required(VERSION 3.10)
project(continuous_icp_servo)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dobot_msgs_v4 REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(continuous_icp_servo_node
  src/continuous_icp_servo_node.cpp
  src/geometry.cpp
  src/handeye_loader.cpp
  src/icp_solver.cpp
  src/point_cloud_utils.cpp
  src/spatial_index.cpp
  src/types.cpp
)

target_compile_features(continuous_icp_servo_node PUBLIC cxx_std_17)
target_include_directories(continuous_icp_servo_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(continuous_icp_servo_node
  rclcpp
  sensor_msgs
  std_msgs
  dobot_msgs_v4
)

install(TARGETS continuous_icp_servo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
