cmake_minimum_required(VERSION 3.16)
project(MyRobotTest)

# 指定 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)
find_package(realsense2 QUIET HINTS
    ${CMAKE_CURRENT_SOURCE_DIR}/../install/librealsense2/lib/cmake/realsense2
)
find_package(Eigen3 REQUIRED NO_MODULE)

# 独立 C AprilTag 库：只编译 detector，不拉 ROS2、Python 和示例。
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_WRAPPER OFF CACHE BOOL "" FORCE)
add_subdirectory(apriltag EXCLUDE_FROM_ALL)

# hsm_test 当前位于 CodroidApi/src；D455 直接使用 librealsense2，不依赖 ROS。
add_executable(test
    CodroidApi/src/hsm_test.cpp
    CodroidApi/src/D455Camera.cpp
    CodroidApi/src/icptask.cpp
    CodroidApi/src/icpexecutetask.cpp
    CodroidApi/src/tagtask.cpp
    CodroidApi/src/tagexecutetask.cpp
)

# 将 CodroidApi/include 添加到头文件搜索路径中
target_include_directories(test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/CodroidApi/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(test PRIVATE Eigen3::Eigen apriltag)

if(TARGET realsense2::realsense2)
    target_link_libraries(test PRIVATE
        realsense2::realsense2
        ${OpenCV_LIBS}
    )
else()
    find_path(REALSENSE2_INCLUDE_DIR
        NAMES librealsense2/rs.hpp
        HINTS
            /usr/include
            /usr/local/include
            /opt/ros/foxy/include
            ${CMAKE_CURRENT_SOURCE_DIR}/../install/librealsense2/include
            ${CMAKE_CURRENT_SOURCE_DIR}/../src/librealsense/include
    )
    find_library(REALSENSE2_LIBRARY
        NAMES realsense2
        HINTS
            /usr/lib/aarch64-linux-gnu
            /usr/lib/x86_64-linux-gnu
            /usr/local/lib
            /opt/ros/foxy/lib
            ${CMAKE_CURRENT_SOURCE_DIR}/../install/librealsense2/lib
            ${CMAKE_CURRENT_SOURCE_DIR}/../data/build/librealsense2/Release
    )
    if(NOT REALSENSE2_INCLUDE_DIR OR NOT REALSENSE2_LIBRARY)
        message(FATAL_ERROR
            "未找到 librealsense2 头文件或库；NX 上应安装 librealsense2-dev")
    endif()
    target_include_directories(test PRIVATE ${REALSENSE2_INCLUDE_DIR})
    target_link_libraries(test PRIVATE
        ${REALSENSE2_LIBRARY}
        ${OpenCV_LIBS}
    )
endif()
