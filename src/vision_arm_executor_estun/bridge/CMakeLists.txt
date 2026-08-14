cmake_minimum_required(VERSION 3.8)
project(estun_codroid_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(jsoncpp CONFIG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetJoints.srv"
  "srv/GetPose.srv"
  "srv/GetRobotState.srv"
  "srv/Move.srv"
  "srv/RobotCommand.srv"
  "srv/SetDo.srv"
  "srv/SolvePose.srv"
)

add_executable(estun_codroid_bridge_node src/codroid_bridge_node.cpp)
target_compile_features(estun_codroid_bridge_node PUBLIC cxx_std_17)
# This workstation also contains an obsolete Boost 1.65 header-only install in
# /usr/local/include.  Beast comes from Ubuntu Boost 1.74; mixing the two makes
# Asio macros disappear.  Put one coherent system Boost tree first without
# changing global machine state.
set(SYSTEM_BOOST_WRAPPER "${CMAKE_CURRENT_BINARY_DIR}/system_boost")
file(MAKE_DIRECTORY "${SYSTEM_BOOST_WRAPPER}")
if(NOT EXISTS "${SYSTEM_BOOST_WRAPPER}/boost")
  file(CREATE_LINK "/usr/include/boost" "${SYSTEM_BOOST_WRAPPER}/boost" SYMBOLIC)
endif()
target_include_directories(estun_codroid_bridge_node BEFORE PRIVATE
  "${SYSTEM_BOOST_WRAPPER}")
ament_target_dependencies(estun_codroid_bridge_node rclcpp)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(estun_codroid_bridge_node
  Boost::system jsoncpp_lib "${cpp_typesupport_target}")

install(TARGETS estun_codroid_bridge_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
