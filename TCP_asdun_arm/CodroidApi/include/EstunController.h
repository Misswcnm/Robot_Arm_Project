#pragma once

#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <thread>
#include <atomic>
#include <iostream>
#include "CodroidApi.h"

class EstunController {
public:
    EstunController(const std::string& host, const std::string& port = "9000");
    ~EstunController();
    
    // 连接机械臂
    bool connect();
    
    // 断开连接
    void disconnect();
    
    // 上电机器人
    bool enableRobot();
    
    // 切换到自动模式
    bool setAutoMode();
    
    // 关节运动
    bool moveJ(const c2::Point& point, double speed = 30, double acc = 30);
    
    // 移动到全局复位点并等待完成
    bool resetRobot(double speed = 30, double acc = 30);
    
    // 设置全局复位点
    void setGlobalResetPoint(const std::vector<double>& position);
    
    // 回Home点
    bool goHome(double speed = 30, double acc = 30);
    
    // 设置Home点位置
    void setHomePosition(double* position);
    
    // 获取当前关节位置
    std::vector<double> getJointPosition();
    
    // 判断是否到达目标点位
    bool isMovementComplete(const std::vector<double>& targetPosition, double tolerance = 0.01);
    
    // 检查是否连接
    bool isConnected() const;
    
    // 获取机器人状态
    c2::RobotState getRobotState();
    
    // 设置状态回调
    void setStatusCallback(std::function<void(c2::RobotState)> callback);

private:
    // 启动状态监控线程
    void startStatusMonitoring();
    
    std::string m_host;
    std::string m_port;
    std::unique_ptr<c2::CodroidApi> m_api;
    std::atomic<bool> m_connected{false};
    std::atomic<bool> m_monitoring{false};
    std::thread m_monitorThread;
    std::function<void(c2::RobotState)> m_statusCallback;
    std::vector<double> m_globalResetPoint; // 全局复位点
};