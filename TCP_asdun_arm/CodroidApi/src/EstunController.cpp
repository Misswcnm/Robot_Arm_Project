#include "EstunController.h"
#include <chrono>
#include <thread>
#include <cmath>

EstunController::EstunController(const std::string& host, const std::string& port)
    : m_host(host), m_port(port) {
    // 使用C++11兼容的方式创建unique_ptr
    m_api.reset(new c2::CodroidApi(host, port));
    
    // 设置默认全局复位点
    setGlobalResetPoint({0.0, 23.0, -112.0, -176.0, 2.0, 0.0});
}

EstunController::~EstunController() {
    disconnect();
}

bool EstunController::connect() {
    // 尝试发送一个简单的命令测试连接
    auto response = m_api->getRobotState(3);
    if (response.code == c2::ResponseCode::OK) {
        m_connected = true;
        startStatusMonitoring();
        std::cout << "Estun robot connected successfully" << std::endl;
        return true;
    }
    return false;
}

void EstunController::disconnect() {
    m_monitoring = false;
    if (m_monitorThread.joinable()) {
        m_monitorThread.join();
    }
    m_connected = false;
}

bool EstunController::enableRobot() {
    auto response = m_api->sendUserCommand(c2::UserCommand::SwitchOn);
    return response.code == c2::ResponseCode::OK;
}

bool EstunController::setAutoMode() {
    auto response = m_api->sendUserCommand(c2::UserCommand::ToAuto);
    return response.code == c2::ResponseCode::OK;
}

bool EstunController::moveJ(const c2::Point& point, double speed, double acc) {
    auto response = m_api->movJ(point, speed, acc);
    return response.code == c2::ResponseCode::OK;
}

bool EstunController::resetRobot(double speed, double acc) {
    c2::Point resetPoint;
    resetPoint.type = c2::PointType::Joint;
    for (int i = 0; i < 6; ++i) {
        resetPoint.apos.jntPos[i] = m_globalResetPoint[i];
    }
    
    // 发送移动指令
    auto response = m_api->movJ(resetPoint, speed, acc);
    if (response.code != c2::ResponseCode::OK) {
        std::cerr << "Failed to send move command to reset point!" << std::endl;
        return false;
    }
    
    // 等待运动完成
    if (!isMovementComplete(m_globalResetPoint)) {
        std::cerr << "Failed to reach reset position!" << std::endl;
        return false;
    }
    
    std::cout << "Robot reset to home position" << std::endl;
    return true;
}

void EstunController::setGlobalResetPoint(const std::vector<double>& position) {
    if (position.size() >= 6) {
        m_globalResetPoint = position;
    }
}

bool EstunController::goHome(double speed, double acc) {
    auto response = m_api->goHome(speed, acc);
    return response.code == c2::ResponseCode::OK;
}

void EstunController::setHomePosition(double* position) {
    m_api->setHomePosition(position);
}

std::vector<double> EstunController::getJointPosition() {
    auto response = m_api->getJointPosition();
    if (response.code == c2::ResponseCode::OK && response.data.is_array()) {
        return response.data.get<std::vector<double>>();
    }
    return {};
}

bool EstunController::isMovementComplete(const std::vector<double>& targetPosition, double tolerance) {
    int waitCount = 0;
    const int maxWaitCount = 200; // 200 * 50ms = 10秒超时
    
    while(true) {
        // 获取当前关节位置
        auto currentPosition = getJointPosition();
        if (currentPosition.size() != targetPosition.size()) {
            std::cerr << "Position size mismatch!" << std::endl;
            return false;
        }
        
        // 检查所有关节角度是否都达到目标值
        bool allReached = true;
        for (size_t i = 0; i < currentPosition.size(); ++i) {
            if (std::abs(currentPosition[i] - targetPosition[i]) > tolerance) {
                allReached = false;
                break; // 只要有一个关节未达到，就跳出循环
            }
        }
        
        if (allReached) {
            return true; // 所有关节都达到目标位置
        }
        
        // 添加超时检测
        if (waitCount++ > maxWaitCount) {
            std::cerr << "Movement timeout!" << std::endl;
            return false;
        }
        
        // 等待100ms后再次检查
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

bool EstunController::isConnected() const {
    return m_connected;
}

c2::RobotState EstunController::getRobotState() {
    auto response = m_api->getRobotState();
    if (response.code == c2::ResponseCode::OK) {
        return static_cast<c2::RobotState>(response.data.get<int>());
    }
    return c2::RobotState::Error;
}

void EstunController::setStatusCallback(std::function<void(c2::RobotState)> callback) {
    m_statusCallback = callback;
}

void EstunController::startStatusMonitoring() {
    m_monitoring = true;
    m_monitorThread = std::thread([this]() {
        while (m_monitoring) {
            auto state = getRobotState();
            if (m_statusCallback) {
                m_statusCallback(state);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    });
}
