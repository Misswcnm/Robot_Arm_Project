# 指定 CMake 的最低版本
cmake_minimum_required(VERSION 3.0)

# 设置项目的名称
project(DobotTcpDemo)

# 启用 C++11 标准
set(CMAKE_CXX_STANDARD 17)
# 设置构建类型为 Realease
set(CMAKE_BUILD_TYPE Realease)

# 添加可执行文件，并指定源文件
add_executable(DobotTcpDemo DobotTcpDemo.cpp api/BitConverter.cpp api/Dashboard.cpp api/DobotClient.cpp api/DobotMove.cpp api/ErrorInfoBean.cpp api/ErrorInfoHelper.cpp api/Feedback.cpp)

# Safe command-line example: connects to all three V3 ports and performs
# read-only status queries. It does not enable or move the robot.
add_executable(CR5BasicExample CR5BasicExample.cpp api/BitConverter.cpp api/Dashboard.cpp api/DobotClient.cpp api/DobotMove.cpp api/Feedback.cpp)

# 链接线程库
find_package(Threads REQUIRED)
target_link_libraries(DobotTcpDemo Threads::Threads)
target_link_libraries(CR5BasicExample Threads::Threads)
