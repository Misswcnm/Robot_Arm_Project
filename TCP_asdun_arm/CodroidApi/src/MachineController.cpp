#include "MachineController.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <nlohmann/json.hpp>  // 只包含一次
#include <cerrno>

using json = nlohmann::json;  // 添加这行来定义json别名


// 实现MachineController类的所有方法
MachineController::MachineController(unsigned int port) 
    : m_port(port), 
      m_listenSocket(-1), 
      m_clientSocket(-1),
      m_running(false) {}

MachineController::~MachineController() {
    stopServer();
}

bool MachineController::startServer() {
  // 创建监听套接字
        m_listenSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (m_listenSocket == -1) {
            std::cerr << "Failed to create socket: " << strerror(errno) << std::endl;
            return false;
        }
        
        // 设置SO_REUSEADDR
        int opt = 1;
        setsockopt(m_listenSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        // 绑定地址和端口
        sockaddr_in serverAddr;
        memset(&serverAddr, 0, sizeof(serverAddr));
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_addr.s_addr = INADDR_ANY;
        serverAddr.sin_port = htons(m_port);
        
        if (bind(m_listenSocket, (sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
            std::cerr << "Bind failed: " << strerror(errno) << std::endl;
            close(m_listenSocket);
            return false;
        }
        
        // 开始监听
        if (listen(m_listenSocket, 5) < 0) {
            std::cerr << "Listen failed: " << strerror(errno) << std::endl;
            close(m_listenSocket);
            return false;
        }
        
        std::cout << "Server listening on port: " << m_port << std::endl;
        m_running = true;
        return true;
}

void MachineController::stopServer() {
        m_running = false;
        if (m_clientSocket != -1) {
            shutdown(m_clientSocket, SHUT_RDWR);
            close(m_clientSocket);
            m_clientSocket = -1;
        }
        if (m_listenSocket != -1) {
            close(m_listenSocket);
            m_listenSocket = -1;
        }
}

bool MachineController::acceptConnection() {
        if (m_listenSocket == -1) return false;
        
        sockaddr_in clientAddr;
        socklen_t clientLen = sizeof(clientAddr);
        
        std::cout << "Waiting for client connection..." << std::endl;
        m_clientSocket = accept(m_listenSocket, (sockaddr*)&clientAddr, &clientLen);
        if (m_clientSocket < 0) {
            std::cerr << "Accept failed: " << strerror(errno) << std::endl;
            return false;
        }
        
        char clientIP[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &clientAddr.sin_addr, clientIP, INET_ADDRSTRLEN);
        std::cout << "Client connected from: " << clientIP << ":" << ntohs(clientAddr.sin_port) << std::endl;
        
        return true;
}

std::string MachineController::receivePosition() { 
            if (m_clientSocket == -1) return "";
        
        char buffer[1024] = {0};
        ssize_t bytesRead = recv(m_clientSocket, buffer, sizeof(buffer), 0);
        
        if (bytesRead < 0) {
            std::cerr << "Receive error: " << strerror(errno) << std::endl;
            return "";
        } else if (bytesRead == 0) {
            std::cout << "Client disconnected" << std::endl;
            close(m_clientSocket);
            m_clientSocket = -1;
            return "";
        }
        //将json字段解析为mapid_pose
        try {
        std::string rawData(buffer, bytesRead);
        auto jsonData = json::parse(rawData);

        // 提取type、 mapid 和 poseid
        std::string type = jsonData.value("type", "");
        std::string mapid  = jsonData.value("mapid", "");
        std::string poseid = jsonData.value("poseid", "");

        if ((type == "type1") &&(mapid.empty() || poseid.empty())) {
            std::cerr << "Invalid JSON: missing mapid or poseid" << std::endl;
            return "";
        }

        // 拼接成 mapid_poseid
        //std::string prefix = type+ "______" + mapid + "________" + poseid;

        std::string prefix = rawData;

        std::cout << "Received position接收位置数据______: " << prefix << std::endl;
        return prefix;

    } catch (const std::exception& e) {
        std::cerr << "JSON parse error: " << e.what() << std::endl;
        return "";
    }
        //return std::string(buffer, bytesRead);

}

void MachineController::sendStatus(const std::string& status) {
 if (m_clientSocket == -1) 
        {
            // 尝试重新连接
            if (!this->acceptConnection()) 
            {
                std::cerr << "无法发送状态: 无客户端连接" << std::endl;
                return;
            }
        }
        
        // 添加消息分隔符
        std::string msg = status + "\n";
        
        // 发送状态信息
        ssize_t sent = send(m_clientSocket, msg.c_str(), msg.size(), 0);
        
        if (sent < 0) 
        {
            std::cerr << "发送状态失败: " << strerror(errno) << std::endl;
            
            // 发送失败时关闭连接
            close(m_clientSocket);
            m_clientSocket = -1;
        } else 
        {
            std::cout << "已发送状态: " << status << std::endl;
        }
}
