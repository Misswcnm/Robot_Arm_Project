#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <memory>
#include <opencv2/opencv.hpp>
#include "CodroidApi.h"
#include <librealsense2/rs.hpp>

class VisionGuidedRobot {
public:
    VisionGuidedRobot(const std::string& robot_host, const std::string& robot_port = "9000")
        : m_robotHost(robot_host), m_robotPort(robot_port) {
        m_api.reset(new c2::CodroidApi(robot_host, robot_port));
    }
    
    ~VisionGuidedRobot() {
        disconnect();
    }
    
    bool connect() {
        std::cout << "尝试连接机械臂..." << std::endl;
        auto response = m_api->getRobotState(3);
        if (response.code == c2::ResponseCode::OK) {
            m_connected = true;
            std::cout << "机械臂连接成功" << std::endl;
            return true;
        }
        std::cerr << "机械臂连接失败，错误码: " << static_cast<int>(response.code) << std::endl;
        return false;
    }
    
    void disconnect() {
        m_connected = false;
    }
    
    bool initializeRobot() {
        std::cout << "初始化机械臂..." << std::endl;
        if (!enableRobot()) {
            std::cerr << "机械臂上电失败" << std::endl;
            return false;
        }
        
        if (!setAutoMode()) {
            std::cerr << "设置自动模式失败" << std::endl;
            return false;
        }
        
        std::cout << "机械臂初始化完成" << std::endl;
        return true;
    }
    
    bool enableRobot() {
        auto response = m_api->sendUserCommand(c2::UserCommand::SwitchOn);
        if (response.code != c2::ResponseCode::OK) {
            std::cerr << "上电指令失败，错误码: " << static_cast<int>(response.code) << std::endl;
            return false;
        }
        return true;
    }
    
    bool setAutoMode() {
        auto response = m_api->sendUserCommand(c2::UserCommand::ToAuto);
        if (response.code != c2::ResponseCode::OK) {
            std::cerr << "设置自动模式失败，错误码: " << static_cast<int>(response.code) << std::endl;
            return false;
        }
        return true;
    }
    
    bool moveToJointPosition(const std::vector<double>& joint_angles, double speed = 30, double acc = 30) {
        std::cout << "移动到关节位置: [";
        for (size_t i = 0; i < joint_angles.size(); ++i) {
            std::cout << joint_angles[i];
            if (i < joint_angles.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
        
        c2::Point point;
        point.type = c2::PointType::Joint;
        
        for (size_t i = 0; i < joint_angles.size() && i < 6; ++i) {
            point.apos.jntPos[i] = joint_angles[i];
        }
        
        auto response = m_api->movJ(point, speed, acc);
        if (response.code != c2::ResponseCode::OK) {
            std::cerr << "移动指令发送失败，错误码: " << static_cast<int>(response.code) << std::endl;
            return false;
        }
        
        return waitForMovementComplete(joint_angles);
    }
    
    bool moveToCartesianPosition(const std::vector<double>& position, const std::vector<double>& orientation, 
                                double speed = 30, double acc = 30) {
        std::cout << "移动到笛卡尔位置 - 位置: [" << position[0] << ", " << position[1] << ", " << position[2]
                  << "], 姿态: [" << orientation[0] << ", " << orientation[1] << ", " << orientation[2] << "]" << std::endl;
        
        c2::Point point;
        point.type = c2::PointType::Cart;
        
        point.cpos.x = position[0];
        point.cpos.y = position[1];
        point.cpos.z = position[2];
        // point.cpos.a = orientation[0] * M_PI / 180.0;
        // point.cpos.b = orientation[1] * M_PI / 180.0;
        // point.cpos.c = orientation[2] * M_PI / 180.0;
        point.cpos.a = orientation[0] * M_PI;
        point.cpos.b = orientation[1] * M_PI;
        point.cpos.c = orientation[2] * M_PI;
        
        c2::Speed speed_obj(0, speed, speed * 0.32);
        c2::Acc acc_obj(0, acc, acc * 0.32);
        
        auto response = m_api->movL(point, speed_obj, acc_obj);
        if (response.code != c2::ResponseCode::OK) {
            std::cerr << "笛卡尔移动失败，错误码: " << static_cast<int>(response.code) << std::endl;
            return false;
        }
        return true;
    }
    
    std::vector<double> getCurrentJointPosition() {
        auto response = m_api->getJointPosition();
        if (response.code == c2::ResponseCode::OK && response.data.is_array()) {
            auto pos = response.data.get<std::vector<double>>();
            std::cout << "当前关节位置: [";
            for (size_t i = 0; i < pos.size(); ++i) {
                std::cout << pos[i];
                if (i < pos.size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
            return pos;
        }
        std::cerr << "获取关节位置失败" << std::endl;
        return {};
    }
    
    std::vector<double> getCurrentCartesianPosition() {
        auto response = m_api->getCartPosition();
        if (response.code == c2::ResponseCode::OK && response.data.is_array()) {
            auto pos = response.data.get<std::vector<double>>();
            std::cout << "当前笛卡尔位置: [";
            for (size_t i = 0; i < pos.size(); ++i) {
                std::cout << pos[i];
                if (i < pos.size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
            return pos;
        }
        std::cerr << "获取笛卡尔位置失败" << std::endl;
        return {};
    }
    
bool moveToCamera3DPoint(const cv::Point3f& point_in_camera, 
                        double approach_height = 0.1,
                        const std::vector<double>& orientation_override = {},
                        double speed = 30, double acc = 30) {
    std::cout << "=== 基于相机3D坐标移动开始 ===" << std::endl;
    std::cout << "相机坐标系坐标: (" << point_in_camera.x << ", " 
              << point_in_camera.y << ", " << point_in_camera.z << ")" << std::endl;
    
    // 转换到机器人基坐标系
    std::vector<double> target_position = camera3DToRobotCoordinates(point_in_camera);
    if (target_position.empty()) {
        std::cerr << "坐标转换失败" << std::endl;
        return false;
    }
    
    std::cout << "转换后的基坐标系坐标: [" << target_position[0] << ", " 
              << target_position[1] << ", " << target_position[2] << "]" << std::endl;
    
    // 设置接近点（物体上方）
    std::vector<double> approach_position = target_position;
    approach_position[2] += approach_height * 1000.0; // 转换为毫米
    
    // 确定姿态 - 优先使用传入的姿态，否则使用当前姿态
    std::vector<double> orientation;
    if (!orientation_override.empty()) {
        orientation = orientation_override;
        std::cout << "使用传入的姿态: [" << orientation[0] << "°, " 
                  << orientation[1] << "°, " << orientation[2] << "°]" << std::endl;
    } else {
        // 获取当前姿态
        auto current_pose = getCurrentCartesianPosition();
        if (current_pose.size() >= 6) {
            orientation = {current_pose[3], current_pose[4], current_pose[5]};
            std::cout << "使用当前姿态: [" << orientation[0] << "°, " 
                      << orientation[1] << "°, " << orientation[2] << "°]" << std::endl;
        }
    }
    
    // 直接使用笛卡尔运动方式
    return moveViaCartesianMotion(approach_position, target_position, orientation, speed, acc);
}

// 原来的笛卡尔运动方式（作为备选）
bool moveViaCartesianMotion(const std::vector<double>& approach_pos, 
                           const std::vector<double>& target_pos,
                           const std::vector<double>& orientation,
                           double speed, double acc) {
    std::cout << "使用笛卡尔运动方式..." << std::endl;
    
    // 安全移动：先到接近点，再到目标点
    std::cout << "第一步: 移动到接近点（高度: " << approach_pos[2] << ")" << std::endl;
    if (!moveToCartesianPosition(approach_pos, orientation, speed, acc)) {
        std::cerr << "接近点移动失败" << std::endl;
        return false;
    }
    
    std::cout << "第二步: 移动到目标点" << std::endl;
    if (!moveToCartesianPosition(target_pos, orientation, speed / 2, acc / 2)) {
        std::cerr << "目标点移动失败" << std::endl;
        return false;
    }
    
    std::cout << "=== 笛卡尔运动方式完成 ===" << std::endl;
    return true;
}



    
    // 新增：直接输入相机坐标系下的3D坐标进行移动
// 修改现有的 moveToCamera3DCoordinates 函数
bool moveToCamera3DCoordinates(double x, double y, double z, 
                              double approach_height = 0.1,
                              double speed = 30, double acc = 30) {
    cv::Point3f point_in_camera(x, y, z);
    
    // 获取当前姿态，而不是使用固定姿态
    auto current_pose = getCurrentCartesianPosition();
    if (current_pose.size() < 6) {
        std::cerr << "获取当前姿态失败，使用固定姿态" << std::endl;
        // 备用方案：使用固定姿态
        std::vector<double> fixed_orientation = {172.1, 1.216, -177.548};
        return moveToCamera3DPoint(point_in_camera, approach_height, fixed_orientation, speed, acc);
    }
    
    // 使用当前姿态（笛卡尔位置的后三个元素是姿态角）
    std::vector<double> current_orientation = {current_pose[3], current_pose[4], current_pose[5]};
    std::cout << "使用当前姿态: [" << current_orientation[0] << "°, " 
              << current_orientation[1] << "°, " << current_orientation[2] << "°]" << std::endl;
    
    return moveToCamera3DPoint(point_in_camera, approach_height, current_orientation, speed, acc);
}

bool isConnected() const {
    return m_connected;
}

private:
    bool waitForMovementComplete(const std::vector<double>& target_position, double tolerance = 0.01, int timeout_ms = 10000) {
        std::cout << "等待运动完成..." << std::endl;
        int wait_count = 0;
        const int max_wait_count = timeout_ms / 50;
        
        while (wait_count < max_wait_count) {
            auto current_position = getCurrentJointPosition();
            if (current_position.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                wait_count++;
                continue;
            }
            
            bool all_reached = true;
            for (size_t i = 0; i < current_position.size(); ++i) {
                double error = std::abs(current_position[i] - target_position[i]);
                if (error > tolerance) {
                    all_reached = false;
                    if (wait_count % 20 == 0) {
                        std::cout << "关节" << i << "误差: " << error << std::endl;
                    }
                    break;
                }
            }
            
            if (all_reached) {
                std::cout << "运动完成" << std::endl;
                return true;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            wait_count++;
        }
        
        std::cerr << "运动超时" << std::endl;
        return false;
    }
    
    // 相机3D坐标转换为机器人基坐标系坐标
    std::vector<double> camera3DToRobotCoordinates(const cv::Point3f& point_in_camera) {
    std::cout << "开始坐标转换..." << std::endl;
    std::cout << "输入相机坐标(米): (" << point_in_camera.x << ", " 
              << point_in_camera.y << ", " << point_in_camera.z << ")" << std::endl;
    
    try {
        // 1. 从相机坐标系转换到末端坐标系（单位：米）
        cv::Mat point_camera = (cv::Mat_<double>(4,1) << 
            point_in_camera.x, point_in_camera.y, point_in_camera.z, 1.0);
        
        // 手眼标定矩阵：相机到末端的变换矩阵 T_end_cam（单位：米）
        cv::Mat T_camera_to_end = (cv::Mat_<double>(4,4) << 
            -0.0023177327,   0.9998959726,  -0.0142362951, -0.0242884365,
            -0.9997701357,  -0.0026204112,  -0.0212793147,  0.0455824061,
            -0.021314406,    0.0141837029,   0.9996722056,  0.1411455543,
            0.0,             0.0,            0.0,           1.0);
        
        cv::Mat point_end = T_camera_to_end * point_camera;
        std::cout << "末端坐标系下的物体位置(米): (" << point_end.at<double>(0,0) << ", " 
                  << point_end.at<double>(1,0) << ", " << point_end.at<double>(2,0) << ")" << std::endl;
        


        // 2. 考虑工具坐标系偏移（16cm Z方向延长）
        // 工具坐标系相对于末端的变换：Z方向+0.16米
        cv::Mat T_tool_to_end = (cv::Mat_<double>(4,4) << 
            1.0, 0.0, 0.0, -0.024,
            0.0, 1.0, 0.0, 0.04, 
            0.0, 0.0, 1.0, 0.20,  // Z方向延长16cm
            0.0, 0.0, 0.0, 1.0);

        // cv::Mat T_tool_to_end = (cv::Mat_<double>(4,4) << 
        // -0.0023177327,   0.9998959726,  -0.0142362951, -0.0242884365,
        // -0.9997701357,  -0.0026204112,  -0.0212793147,  0.0455824061,
        // -0.021314406,    0.0141837029,   0.9996722056,  0.201455543,
        // 0.0, 0.0, 0.0, 1.0);
        
        // 物体在工具坐标系中的位置
        cv::Mat point_tool = point_end;
        
        // 3. 将工具坐标系下的点转换到末端坐标系
        cv::Mat point_end_with_tool = T_tool_to_end.inv() * point_tool;
        
        // 4. 获取当前末端在基坐标系下的位置（单位：毫米）
        auto current_pose = getCurrentCartesianPosition();
        if (current_pose.size() < 6) {
            std::cerr << "无法获取当前末端位姿" << std::endl;
            return {};
        }
        
        std::cout << "当前末端在基座坐标系下位置(毫米): [" << current_pose[0] << ", " 
                  << current_pose[1] << ", " << current_pose[2] << "]" << std::endl;
        
        // 5. 单位统一：将末端坐标系下的点从米转换为毫米
        double x_end_mm = point_end_with_tool.at<double>(0,0) * 1000.0;
        double y_end_mm = point_end_with_tool.at<double>(1,0) * 1000.0;
        double z_end_mm = point_end_with_tool.at<double>(2,0) * 1000.0;
        
        std::cout << "考虑工具偏移后的末端位置(毫米): (" << x_end_mm << ", " 
                  << y_end_mm << ", " << z_end_mm << ")" << std::endl;
        
        // 6. 计算物体在基坐标系下的位置（毫米）
        double x_base = current_pose[0];
        double y_base = current_pose[1]; 
        double z_base = current_pose[2];
        double a_base = current_pose[3] * M_PI / 180.0; // 弧度
        double b_base = current_pose[4] * M_PI / 180.0;
        double c_base = current_pose[5] * M_PI / 180.0;
        
        // 创建旋转矩阵
        cv::Mat R_base_end = eulerToRotationMatrix(c_base, b_base, a_base);
        
        // 物体在末端坐标系中的位置（毫米）
        cv::Mat point_end_mm = (cv::Mat_<double>(3,1) << x_end_mm, y_end_mm, z_end_mm);
        
        // 旋转到基坐标系方向
        cv::Mat point_base_relative = R_base_end * point_end_mm;
        
        // 加上末端在基坐标系中的位置
        std::vector<double> robot_position(3);
        robot_position[0] = x_base + point_base_relative.at<double>(0,0);
        robot_position[1] = y_base + point_base_relative.at<double>(1,0);
        robot_position[2] = z_base + point_base_relative.at<double>(2,0);
        
        std::cout << "基坐标系下的目标位置(毫米): [" << robot_position[0] << ", " 
                  << robot_position[1] << ", " << robot_position[2] << "]" << std::endl;
        
        // 7. 检查移动是否合理（安全边界检查）
        if (!isMovementSafe(robot_position)) {
            std::cerr << "移动目标位置超出安全范围" << std::endl;
            return {};
        }
        
        return robot_position;
        
    } catch (const cv::Exception& e) {
        std::cerr << "坐标转换异常: " << e.what() << std::endl;
        return {};
    }
}
    // 安全检查函数
    bool isMovementSafe(const std::vector<double>& position) {
        // 简单的位置边界检查（根据您的机器人工作空间调整）
        if (position[0] < -1000 || position[0] > 1000) return false;  // X轴限制
        if (position[1] < -1000 || position[1] > 1000) return false;  // Y轴限制  
        if (position[2] < 0 || position[2] > 1500) return false;      // Z轴限制
        
        return true;
    }
    
    // 欧拉角转旋转矩阵辅助函数（ZYX顺序）
    cv::Mat eulerToRotationMatrix(double rz, double ry, double rx) {
        cv::Mat R_x = (cv::Mat_<double>(3,3) << 
            1, 0, 0,
            0, cos(rx), -sin(rx),
            0, sin(rx), cos(rx));
        
        cv::Mat R_y = (cv::Mat_<double>(3,3) << 
            cos(ry), 0, sin(ry),
            0, 1, 0,
            -sin(ry), 0, cos(ry));
        
        cv::Mat R_z = (cv::Mat_<double>(3,3) << 
            cos(rz), -sin(rz), 0,
            sin(rz), cos(rz), 0,
            0, 0, 1);
        
        return R_z * R_y * R_x;
    }

    std::string m_robotHost;
    std::string m_robotPort;
    std::unique_ptr<c2::CodroidApi> m_api;
    std::atomic<bool> m_connected{false};
};

int main() {
    std::cout << "=== 视觉引导机械臂演示程序启动 ===" << std::endl;
    
    VisionGuidedRobot robot("192.168.2.5", "9000");
    
    if (!robot.connect()) {
        std::cerr << "机械臂连接失败" << std::endl;
        return -1;
    }
    
    if (!robot.initializeRobot()) {
        std::cerr << "机械臂初始化失败" << std::endl;
        return -1;
    }
    
    std::cout << "系统准备就绪，请输入相对于相机的三维坐标控制机械臂移动" << std::endl;
    std::cout << "坐标格式: x y z (单位: 米)" << std::endl;
    std::cout << "例如: 0.1 0.05 0.3" << std::endl;
    std::cout << "输入 'q' 退出程序" << std::endl;
    
    while (true) {
        std::cout << "\n=== 请输入三维坐标 ===" << std::endl;
        std::cout << "坐标 (x y z): ";
        
        std::string input;
        std::getline(std::cin, input);
        
        if (input == "q" || input == "quit") {
            break;
        }
        
        double x, y, z;
        if (sscanf(input.c_str(), "%lf %lf %lf", &x, &y, &z) != 3) {
            std::cerr << "输入格式错误，请重新输入" << std::endl;
            continue;
        }
        
        std::cout << "接收到坐标: x=" << x << ", y=" << y << ", z=" << z << std::endl;
        
       
        
        // 设置安全参数
        double approach_height = 0.1;
        double speed = 20;
        double acc = 20;
        
        std::cout << "开始移动到指定坐标..." << std::endl;
        
        if (robot.moveToCamera3DCoordinates(x, y, z, approach_height, speed, acc)) {
            std::cout << "✓ 移动成功完成" << std::endl;
        } else {
            std::cerr << "✗ 移动失败" << std::endl;
            

        }
        
        // 可选：等待一段时间后返回安全位置
        std::cout << "是否返回安全位置? (y/n): ";
        std::getline(std::cin, input);
        
        if (input == "y" || input == "Y") {
            std::cout << "返回安全位置..." << std::endl;
            std::vector<double> home_position = {86.0, 23.0, -112.0, -176.0, 2.0, 0.0};
            if (!robot.moveToJointPosition(home_position)) {
                std::cerr << "返回安全位置失败" << std::endl;
            }
        }
    }
    
    std::cout << "程序结束" << std::endl;
    return 0;
}