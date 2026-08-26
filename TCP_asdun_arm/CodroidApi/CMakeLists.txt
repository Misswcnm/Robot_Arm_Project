cmake_minimum_required(VERSION 3.16)
project(RobotControl LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(realsense2 QUIET HINTS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../install/librealsense2/lib/cmake/realsense2)

set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_WRAPPER OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../apriltag
                 ${CMAKE_CURRENT_BINARY_DIR}/apriltag EXCLUDE_FROM_ALL)

add_executable(RobotControl
    src/RobotDemo.cpp
    src/WebSocketClient.cpp
    src/D455Camera.cpp
    src/icpwallfilter.cpp
    src/icptask.cpp
    src/icpexecutetask.cpp
    src/tagtask.cpp
    src/tagexecutetask.cpp
)

target_include_directories(RobotControl PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

# Some development machines have a stale partial Boost under /usr/local.
# Keep one coherent distro Boost include tree for Asio/Beast.
if(EXISTS "/usr/local/include/boost" AND EXISTS "/usr/include/boost/beast/core.hpp")
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/system-include)
    file(CREATE_LINK /usr/include/boost
         ${CMAKE_CURRENT_BINARY_DIR}/system-include/boost SYMBOLIC)
    target_include_directories(RobotControl BEFORE PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}/system-include)
endif()

target_link_libraries(RobotControl PRIVATE
    Eigen3::Eigen
    apriltag
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
    Threads::Threads
)

if(TARGET realsense2::realsense2)
    target_link_libraries(RobotControl PRIVATE realsense2::realsense2)
else()
    find_path(REALSENSE2_INCLUDE_DIR NAMES librealsense2/rs.hpp
        HINTS /usr/include /usr/local/include /opt/ros/foxy/include
              ${CMAKE_CURRENT_SOURCE_DIR}/../../install/librealsense2/include
              ${CMAKE_CURRENT_SOURCE_DIR}/../../src/librealsense/include)
    find_library(REALSENSE2_LIBRARY NAMES realsense2
        HINTS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu
              /usr/local/lib /opt/ros/foxy/lib)
              
    if(NOT REALSENSE2_INCLUDE_DIR OR NOT REALSENSE2_LIBRARY)
        message(FATAL_ERROR "librealsense2 headers/library not found")
    endif()
    target_include_directories(RobotControl PRIVATE ${REALSENSE2_INCLUDE_DIR})
    target_link_libraries(RobotControl PRIVATE ${REALSENSE2_LIBRARY})
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(RobotControl PRIVATE -Wall -Wextra -Wpedantic)
endif()
