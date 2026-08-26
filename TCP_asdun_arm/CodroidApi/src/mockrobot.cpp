#include <iostream>
#include <vector>
#include "CodroidApi.h"
#include "nlohmann/json.hpp"
#include <string>
#include <thread>
#include <atomic>
#include <functional>
#include <memory>
#include <cmath>
#include <filesystem>
#include <fstream>   // 新增，用于文件流
#include <map>       // 新增，用于std::map
#include <librealsense2/rs.hpp> // RealSense库
#include <opencv2/opencv.hpp>   // OpenCV库
#include <set>
#include <chrono>


namespace fs = std::filesystem;


// 机械臂状态枚举
enum class RobotState {
    IDLE,           // 复位/空闲状态
    INITIALIZING,   // 初始化中
    MOVING,         // 运动状态
    IMAGING,        // 拍照状态
    FAULT,          // 故障状态
    RESETTING       // 复位中
};

// 全局共享状态变量
std::atomic<RobotState> g_robotState(RobotState::IDLE);
std::mutex g_stateMutex;
std::condition_variable g_stateCV;
std::queue<std::string> g_positionQueue;  // 位置消息队列
std::mutex g_queueMutex;
std::mutex g_feedbackMutex;  // 反馈数据互斥锁
// 全局状态队列
std::queue<std::string> g_statusQueue;
std::mutex g_statusMutex;


// 拍照图片路径队列（原始也是图）（发送给客户端也是这个队列）
std::queue<std::string> g_imageQueue;
std::mutex g_imageMutex;

// 推理图片路径队列
std::queue<std::string> g_inferenceQueue;
std::mutex g_inferenceMutex;




//机械臂控制类
// 模拟机械臂控制器
class MockEstunController {
public:
    MockEstunController(const std::string& host, const std::string& port = "9000") {
        std::cout << "使用模拟机械臂控制器 (Mock)" << std::endl;
    }
    
    bool connect() {
        std::cout << "模拟机械臂连接成功" << std::endl;
        return true;
    }
    
    bool enableRobot() {
        std::cout << "模拟机械臂上电成功" << std::endl;
        return true;
    }
    
    bool setAutoMode() {
        std::cout << "模拟机械臂切换到自动模式" << std::endl;
        return true;
    }

    bool startTeachingDrag() {
        // 真机对应 UserCommand::ToReady：进入 Ready/手动示教模式。
        std::cout << "模拟机械臂进入 Ready/手动示教模式，允许人工拖拽" << std::endl;
        return true;
    }
    
    bool moveJ(const c2::Point& point, double speed = 30, double acc = 30) {
        std::cout << "模拟机械臂移动到目标点" << std::endl;
        return true;
    }
    
    bool resetRobot(double speed = 30, double acc = 30) {
        std::cout << "模拟机械臂复位" << std::endl;
        return true;
    }
    
    bool setDO(int port, int val, int timeout = 30) {
        std::cout << "模拟设置数字输出: 端口=" << port << ", 值=" << val << std::endl;
        return true;
    }
    
    bool isMovementComplete(const std::vector<double>& targetPosition, double tolerance = 0.01) {
        // 模拟运动完成
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        return true;
    }
    
    std::vector<double> getJointPosition() {
        // 返回模拟关节位置
        return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }
};


// 工控机通信类
class MachineController {
public:
    MachineController(unsigned int port) 
        : m_port(port), 
          m_listenSocket(-1), 
          m_clientSocket(-1),
          m_running(false) {}
    
    ~MachineController() {
        stopServer();
    }
    
    // 启动TCP服务器
    bool startServer() {
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
    
    // 停止服务器
    void stopServer() {
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
    
    // 接受客户端连接
    bool acceptConnection() 
    {
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
    
    // 接收位置数据  return prefix
    std::string receivePosition() 
    {
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
    
    // 发送状态信息
    void sendStatus(const std::string& status) 
    {
        std::lock_guard<std::mutex> lock(m_socketMutex);
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

//推理前后图片发送
void sendImage(const std::string& imagePath,
               const std::string& mapid,
               const std::string& poseid,
               const std::string& imageType)  // 添加imageType参数
{
    std::lock_guard<std::mutex> lock(m_socketMutex);
    if (m_clientSocket == -1) {
        if (!this->acceptConnection()) {
            std::cerr << "[Server] 无法发送图片: 无客户端连接" << std::endl;
            return;
        }
    }

    // 打开文件
    std::ifstream ifs(imagePath, std::ios::binary | std::ios::ate);
    if (!ifs.is_open()) {
        std::cerr << "[Server] 无法打开图片: " << imagePath << std::endl;
        return;
    }

    std::streamsize fileSize = ifs.tellg();
    ifs.seekg(0, std::ios::beg);

    std::string filename = imagePath.substr(imagePath.find_last_of("/\\") + 1);

    // === 构造 JSON 元数据 ===
    json meta;
    meta["type"] = imageType;  // 使用传入的imageType参数
    meta["mapid"] = mapid;
    meta["poseid"] = poseid;
    meta["filename"] = filename;
    meta["filesize"] = fileSize;

    std::string metaStr = meta.dump() + "\n"; // 以换行符结尾，方便客户端按行解析

    // 发送 JSON 元数据
    ssize_t sentMeta = send(m_clientSocket, metaStr.c_str(), metaStr.size(), 0);
    if (sentMeta < 0) {
        std::cerr << "[Server] 发送元数据失败: " << strerror(errno) << std::endl;
        return;
    }

    std::cout << "[Server] >>> 已发送元数据: " << metaStr << std::endl;

    // === 发送二进制数据 ===
    const size_t BUF_SIZE = 4096;
    char buffer[BUF_SIZE];
    std::streamsize totalSent = 0;

    while (ifs) {
        ifs.read(buffer, BUF_SIZE);
        std::streamsize bytesRead = ifs.gcount();
        if (bytesRead > 0) {
            ssize_t sent = send(m_clientSocket, buffer, bytesRead, 0);
            if (sent < 0) {
                std::cerr << "[Server] 发送图片数据失败: " << strerror(errno) << std::endl;
                close(m_clientSocket);
                m_clientSocket = -1;
                break;
            }
            totalSent += sent;
        }
    }

    std::cout << "[Server] <<< 图片发送完成: " << filename 
            << " (" << totalSent << " bytes)" << std::endl;
}
     //发送图片
    // void sendImage(const std::string& imagePath,
    //             const std::string& mapid,
    //             const std::string& poseid) 
    // {
    //     std::lock_guard<std::mutex> lock(m_socketMutex); // 添加线程锁
    //     if (m_clientSocket == -1) {
    //         if (!this->acceptConnection()) {
    //             std::cerr << "[Server] 无法发送图片: 无客户端连接" << std::endl;
    //             return;
    //         }
    //     }

    //     // 打开文件
    //     std::ifstream ifs(imagePath, std::ios::binary | std::ios::ate);
    //     if (!ifs.is_open()) {
    //         std::cerr << "[Server] 无法打开图片: " << imagePath << std::endl;
    //         return;
    //     }

    //     std::streamsize fileSize = ifs.tellg();
    //     ifs.seekg(0, std::ios::beg);

    //     std::string filename = imagePath.substr(imagePath.find_last_of("/\\") + 1);

    //     // === 构造 JSON 元数据 ===
    //     json meta;
    //     meta["type"] = "raw";
    //     meta["mapid"] = mapid;
    //     meta["poseid"] = poseid;
    //     meta["filename"] = filename;
    //     meta["filesize"] = fileSize;

    //     std::string metaStr = meta.dump() + "\n"; // 以换行符结尾，方便客户端按行解析

    //     // 发送 JSON 元数据
    //     ssize_t sentMeta = send(m_clientSocket, metaStr.c_str(), metaStr.size(), 0);
    //     if (sentMeta < 0) {
    //         std::cerr << "[Server] 发送元数据失败: " << strerror(errno) << std::endl;
    //         return;
    //     }

    //     std::cout << "[Server] >>> 已发送元数据: " << metaStr << std::endl;

    //     // === 发送二进制数据 ===
    //     const size_t BUF_SIZE = 4096;
    //     char buffer[BUF_SIZE];
    //     std::streamsize totalSent = 0;

    //     while (ifs) {
    //         ifs.read(buffer, BUF_SIZE);
    //         std::streamsize bytesRead = ifs.gcount();
    //         if (bytesRead > 0) {
    //             ssize_t sent = send(m_clientSocket, buffer, bytesRead, 0);
    //             if (sent < 0) {
    //                 std::cerr << "[Server] 发送图片数据失败: " << strerror(errno) << std::endl;
    //                 close(m_clientSocket);
    //                 m_clientSocket = -1;
    //                 break;
    //             }
    //             totalSent += sent;
    //         }
    //     }

    //     std::cout << "[Server] <<< 图片发送完成: " << filename 
    //             << " (" << totalSent << " bytes)" << std::endl;
    // }
    //遍历文件夹中的图片，调用发送图片


    int getClientSocket() const 
    {
        return m_clientSocket;
    }

   


private:
    unsigned int m_port;
    int m_listenSocket;
    int m_clientSocket;
    std::atomic<bool> m_running;
    std::mutex m_socketMutex; // 新增互斥锁
};

// 相机控制类
class CameraController {
public:
    CameraController() : m_captureSuccess(false) {}

     bool captureImage(const std::string& mapid, const std::string& poseid) 
    {
        m_captureSuccess = false;  // 重置捕获状态
        
        try {
            // RealSense管道配置
            rs2::pipeline pipe;
            rs2::config cfg;
            cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
            
            // 启动管道并捕获帧
            pipe.start(cfg);

            // ===== 新增预热阶段 ===== //
            std::cout << "相机预热中..." << std::endl;
            for(int i = 0; i < 50; i++) {
                // 等待帧并跳过不稳定帧
                rs2::frameset frames = pipe.wait_for_frames();
                // 添加短暂延迟稳定自动曝光
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            std::cout << "预热完成，开始拍摄照片" << std::endl;
            // ======================= //
            
            rs2::frameset frames = pipe.wait_for_frames();
            rs2::video_frame color_frame = frames.get_color_frame();
            

            // ==== 关键修改开始 ==== //
            // 转换到OpenCV格式并转换颜色空间
            cv::Mat image_rgb(cv::Size(640, 480), CV_8UC3, 
                            (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat image;
            cv::cvtColor(image_rgb, image, cv::COLOR_RGB2BGR); // RGB转BGR
            // ==== 关键修改结束 ==== //


            // 创建目录结构: mapid/poseid/raw/当前日期/
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now);
            
            // 获取当前日期作为文件夹名
            std::stringstream date_ss;
            date_ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d");
            std::string date_str = date_ss.str();
            std::filesystem::path currentPath = std::filesystem::path(__FILE__).parent_path();
            // 构建完整目录路径
            fs::path dir_path = currentPath/mapid / poseid / "raw" / date_str;
            
            // 创建目录（如果不存在）
            if (!fs::exists(dir_path)) {
                fs::create_directories(dir_path);
            }
            
            // 生成带时间戳的文件名
            std::stringstream filename_ss;
            filename_ss << std::put_time(std::localtime(&in_time_t), "capture_%H%M%S.png");
            std::string filename = filename_ss.str();
            
            // 完整文件路径
            fs::path full_path = dir_path / filename;
            
            // 保存图像
            //full_path = currentPath/mapid / poseid / "raw" / date_str/filename;
            if (cv::imwrite(full_path.string(), image)) {
                std::cout << "Image saved to: " << full_path.string() << std::endl;
                m_captureSuccess = true;
                m_lastImagePath = full_path.string(); // 保存路径
                // 将图片路径加入图片发送队列 原图
                {
                    std::lock_guard<std::mutex> lock(g_imageMutex);
                    g_imageQueue.push(m_lastImagePath);
                    std::cout << "保存图片已加入队列: " << m_lastImagePath << std::endl;
                }

                // 同时加入推理队列
                {
                    std::lock_guard<std::mutex> lock(g_inferenceMutex);
                    g_inferenceQueue.push(m_lastImagePath);
                    std::cout << "保存图片已加入推理队列: " << m_lastImagePath << std::endl;
                }

            } else {
                std::cerr << "Failed to save image!" << std::endl;
            }
            
            pipe.stop();
        } 
        catch (const rs2::error & e) {
            std::cerr << "RealSense error: " << e.what() << std::endl;
        }
        catch (const std::exception & e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
        
        return m_captureSuccess;
    }
    
    bool isImageCaptured() const 
    {
        return m_captureSuccess;
    }

    // 添加拍照后稳定时间
    void stabilizeAfterCapture() 
    {
        // 等待2秒确保机械臂稳定
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
        // 新增：重置拍照状态
    void resetCaptureState() 
    {
        m_captureSuccess = false;
    }

private:
    std::string m_lastImagePath;
    std::atomic<bool> m_captureSuccess;
};

    MockEstunController*estun=nullptr;
    CameraController* camera=nullptr;
    MachineController* positionServer=nullptr;

//json文件解析

std::vector<c2::Point> parsePositionData(const std::string& mapid, const std::string& poseid) 
{
    // 构建JSON文件路径: /home/gao/TCP-IP-CR-CPP-V3/CppDemo/{mapid}/{poseid}/{mapid}_{poseid}.json
    std::filesystem::path basePath = "/home/gao/TCP_asdun_arm/CodroidApi/src";
    std::filesystem::path jsonPath = basePath / mapid / poseid / (mapid + "_" + poseid + ".json");
    
    std::string filePath = jsonPath.string();
    std::cout << "Looking for JSON file at: " << filePath << std::endl;
    
    // 检查文件是否存在
    if (!std::filesystem::exists(jsonPath)) {
        std::cerr << "JSON file not found: " << filePath << std::endl;
        return {};
    }

    std::ifstream ifs(jsonPath);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open file: " << filePath << std::endl;
        return {};
    }

    json jsonData;
    try {
        ifs >> jsonData;
    } catch (const json::parse_error& e) {
        std::cerr << "JSON parse error [" << e.byte << "]: " << e.what() << std::endl;
        return {};
    }

    // 验证JSON结构是否为数组
    if (!jsonData.is_array()) {
        std::cerr << "Invalid JSON structure: Expected array of points" << std::endl;
        return {};
    }

    std::map<std::string, c2::Point> namedPoints;  // 使用有序map自动排序
    
    for (const auto& pointData : jsonData) {
        // 验证必需字段存在
        if (!pointData.contains("label") || !pointData.contains("joints")) {
            std::cerr << "Skipping invalid point (missing fields)" << std::endl;
            continue;
        }

        try {
            std::string label = pointData["label"].get<std::string>();
            const auto& joints = pointData["joints"];
            
            c2::Point point;
            point.type = c2::PointType::Joint;  // 设置为关节类型
            
            // 从joints对象中提取关节值到APos结构
            point.apos.jntPos[0] = joints["jntpos1"].get<double>();
            point.apos.jntPos[1] = joints["jntpos2"].get<double>();
            point.apos.jntPos[2] = joints["jntpos3"].get<double>();
            point.apos.jntPos[3] = joints["jntpos4"].get<double>();
            point.apos.jntPos[4] = joints["jntpos5"].get<double>();
            point.apos.jntPos[5] = joints["jntpos6"].get<double>();
            
            // 处理第7个关节位置（如果有）
            if (joints.contains("jntpos7")) {
                point.apos.jntPos[6] = joints["jntpos7"].get<double>();
            } else {
                point.apos.jntPos[6] = 0.0;
            }
            
            namedPoints.emplace(label, std::move(point));
        } catch (const json::exception& e) {
            std::cerr << "Point parsing error: " << e.what() << std::endl;
        }
    }
    
    // 构建结果vector
    std::vector<c2::Point> movementPoints;
    movementPoints.reserve(namedPoints.size());
    
    for (auto& [label, point] : namedPoints) {
        movementPoints.push_back(std::move(point));
    }
    
    std::cout << "Successfully parsed " << movementPoints.size() 
              << " points from " << filePath << std::endl;
    
    return movementPoints;
}



// 线程1: 主任务线程
void taskThread() 
{
    while (true) 
    {
        // 等待新的位置消息
        std::string positionPrefix;
        std::string positionarm;
        bool isType1Message = false;
        std::string type, mapid, poseid;


        {
            std::unique_lock<std::mutex> lock(g_queueMutex);
            if (g_positionQueue.empty()) 
            {
                lock.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            positionPrefix = g_positionQueue.front();
            g_positionQueue.pop();

            auto jsonData = json::parse(positionPrefix);
            type   = jsonData.value("type", "");
            mapid  = jsonData.value("mapid", "");
            poseid = jsonData.value("poseid", "");
 
            // 拼接成 mapid_poseid
            positionarm = mapid + "_" + poseid;

            std::string folderName = mapid + "_" + poseid;

            std::string prefix = "type1_";
            std::filesystem::path currentPath = std::filesystem::path(__FILE__).parent_path();
            std::filesystem::path fullPath = currentPath / mapid/ poseid;
            
            //type1 示教模式
            if (type == "type1")
            {
                std::cout << "[TYPE1]开始示教" <<  std::endl;
                std::cout << "mapid = " << mapid << ", poseid = " << poseid << std::endl;

                int command = jsonData.value("command", 0);
                int point_type = jsonData.value("point_type", -1);
                switch(command)
                {
                    //无command进入示教，开始拖拽
                    case 0:
                        break;
                    //command = 1,记录ref点
                    //ref点分为type_point = 0
                    case 1:
                    switch (point_type)
                    {
                        case -1:    //type1,mapid,poseid,无point_type
                            break;
                        case 0:
                            break;
                        case 1:
                            break;
                        default:
                            break;
                    }
                    //command > 1,记录work点
                    //work点分状态，
                    default:

                    
                }
                //有command => 记录command点 
                if (jsonData.contains("command"))
                    {
                        int command = jsonData["command"].get<int>();
                        if (command == 1)
                        //记录REF点
                        {
                            

                        }
                        if (command > 1) 
                        {

                        }


                    }
                else
                {
                // 没有 command => 开始站点示教
                // {}"type":"type1","mapid":"M","poseid":"P"} 可选point_type

                    if (!estun->startTeachingDrag())
                    {
                        positionServer->sendStatus(
                            "{\"type\":\"type1\",\"status\":\"failed\","
                            "\"error_code\":\"start_drag_failed\"}");
                        continue;
                    }

                    if(jsonData.contains("point_type"))
                    // 有point_type
                    {
                        if (point_type == 0)
                        // apriltag 示教开始
                        {

                        }
                        else if (point_type == 1)
                        // ICP 示教开始
                        {

                        }
                        
                    }
                    else
                    {

                    }
                }

                // 接收消息为type1，为示教状态，创建文件夹
                isType1Message = true;

                try 
                {
                    if (std::filesystem::create_directories(fullPath)) 
                    {
                        std::cout << "文件夹创建成功: " << fullPath << std::endl;
                    } else 
                    {
                        std::cout << "文件夹已存在或创建失败: " << fullPath << std::endl;
                    }
                } catch (const std::filesystem::filesystem_error& e) 
                {
                    std::cerr << "文件系统错误: " << e.what() << std::endl;
                }
            }
            if (isType1Message) 
            {
                std::cout << "正在处理type1消息: " << positionPrefix << std::endl;
                continue;
            }


           if(type == "type2")
            {
                std::cout << "[TYPE2]查询"<<std::endl;

                try {
                    // 取源码所在路径
                    std::filesystem::path basePath = std::filesystem::path(__FILE__).parent_path();

                    // 文件夹路径 (mapid/poseid)
                    std::filesystem::path folderPath = basePath / mapid / poseid;

                    // json 文件路径 (mapid/poseid/mapid_poseid.json)
                    std::filesystem::path jsonPath = folderPath / (folderName + ".json");

                    if (!std::filesystem::exists(jsonPath)) {
                        std::cerr << "JSON file not found: " << jsonPath << std::endl;
                        positionServer->sendStatus("{\"error\": \"json file not found\"}");
                    } else {
                        std::ifstream ifs(jsonPath);
                        json jsonData;
                        ifs >> jsonData;

                        std::string jsonStr = jsonData.dump();
                        positionServer->sendStatus(jsonStr);
                        std::cout << "已发送 type2 JSON 数据给客户端: " << jsonPath << std::endl;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "处理 type2 消息时出错: " << e.what() << std::endl;
                    positionServer->sendStatus("{\"error\": \"exception while reading json\"}");
                }

                continue; // 处理完 type2 跳过后续逻辑
            }

        }



        std::cout << "333333333333333333333333Received position prefix33333333333333333333333333333: " << positionarm << std::endl;
        // 更新状态为初始化
        g_robotState = RobotState::INITIALIZING;

        // 解析位置数据（仅用于获取mapid和poseid，不用于实际运动）
        auto movementPoints = parsePositionData(mapid, poseid);
        if (movementPoints.empty()) 
        {
            std::string map_poseid = mapid + "_" + poseid;
            std::cerr << "No movement points found for prefix: " << map_poseid << std::endl;
            g_robotState = RobotState::FAULT;
            
            // 故障后复位（模拟）
            std::cout << "模拟机械臂复位" << std::endl;
            g_robotState = RobotState::IDLE;
            continue;
        }

        // 模拟设置自动模式
        std::cout << "模拟机械臂设置为自动模式" << std::endl;

        // 跳过机械臂运动，直接进行拍照
        for (auto& point : movementPoints) {
            // 重置拍照状态
            camera->resetCaptureState();
            
            // 模拟移动到目标点
            g_robotState = RobotState::MOVING;
            std::cout << "模拟移动到目标点: (" 
                        << point.apos.jntPos[0] << ", " 
                        << point.apos.jntPos[1] << ", " 
                        << point.apos.jntPos[2] << ", " 
                        << point.apos.jntPos[3] << ", " 
                        << point.apos.jntPos[4] << ", " 
                        << point.apos.jntPos[5] << ")" << std::endl;
            
            // 模拟等待运动完成
            std::cout << "模拟等待运动完成..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            
            //补光灯开启
            if (!estun->setDO(1, 1)) {
                std::cerr << "Failed to turn on fill light!" << std::endl;
            }
            // 等待一段时间让补光灯稳定（例如0.5秒）
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // 拍照
            g_robotState = RobotState::IMAGING;
            std::cout << "Capturing image..." << std::endl;
            camera->captureImage(mapid, poseid);

            // 等待拍照完成
            while (!camera->isImageCaptured()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
                
            // 关闭补光灯
            if (!estun->setDO(1, 0)) {
                std::cerr << "Failed to turn off fill light!" << std::endl;
            }

            // 添加稳定时间
            camera->stabilizeAfterCapture();
            
            // 更新状态为准备移动
            g_robotState = RobotState::MOVING;
        }

        // 完成循环，模拟复位机械臂
        if (g_robotState != RobotState::FAULT) 
        {
            g_robotState = RobotState::RESETTING;
            std::cout << "模拟机械臂复位" << std::endl;
            
            //状态字段
            json statusJson;
            statusJson["mapid"] = mapid;
            statusJson["poseid"] = poseid;
            statusJson["status"] = "done";
            std::string statusMsg = statusJson.dump();

            // 发送完成状态
            {
                std::lock_guard<std::mutex> lock(g_statusMutex);
                g_statusQueue.push(statusMsg);
                std::cout << "发送状态: " << statusMsg << std::endl;
            }
        }
    }
};

//接收机位线程
void positionReceiverThread() {
    if (!positionServer->startServer()) {
        std::cerr << "Failed to start position server!" << std::endl;
        return;
    }
    
    while (true) {
        // 等待客户端连接
        if (positionServer->acceptConnection()) {
            // 持续接收来自该客户端的数据
            while (true) {
                std::string position = positionServer->receivePosition();

                if (position.empty()) {
                    break; // 客户端断开连接
                }
                
                {
                    std::lock_guard<std::mutex> lock(g_queueMutex);
                    g_positionQueue.push(position);
                }

                
                std::cout << "Received position__________客户端连接: " << position << std::endl;
            }
        }
        
        // 短暂休眠后尝试重新接受连接
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
};

// 状态发送线程
void statusSenderThread() {
    while (true) {
        std::string status;
        {
            std::lock_guard<std::mutex> lock(g_statusMutex);
            if (!g_statusQueue.empty()) {
                status = g_statusQueue.front();
                g_statusQueue.pop();
            }
        }
        
        if (!status.empty()) 
        {
            positionServer->sendStatus(status);
            std::cout<<"发送44444444454545454545454544454"<<status<<std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// 图片发送线程（推理前后都发）
void imageSenderThread() {
    while (true) {
        std::string imagePath;
        
        // 从队列中获取图片路径
        {
            std::unique_lock<std::mutex> lock(g_imageMutex);
            if (g_imageQueue.empty()) {
                lock.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            imagePath = g_imageQueue.front();
            g_imageQueue.pop();
        }
        
        std::cout << "[Image Sender] Processing image: " << imagePath << std::endl;
        
        // 从图片路径中解析 mapid 和 poseid
        fs::path pathObj(imagePath);
        std::vector<std::string> pathParts;
        for (const auto& part : pathObj) {
            if (!part.empty()) {
                pathParts.push_back(part.string());
            }
        }
        
        // 确保路径足够长以包含 mapid 和 poseid
        if (pathParts.size() < 4) {
            std::cerr << "[Image Sender] Invalid image path: " << imagePath << std::endl;
            continue;
        }
        
        // 提取 mapid 和 poseid
        std::string mapid, poseid, imageType;
        
        // 检查是原始图片还是推理结果图片
        if (std::find(pathParts.begin(), pathParts.end(), "raw") != pathParts.end()) {
            // 原始图片路径结构: .../mapid/poseid/raw/日期/文件名.png
            mapid = pathParts[pathParts.size() - 5];
            poseid = pathParts[pathParts.size() - 4];
            imageType = "raw";
        } else if (std::find(pathParts.begin(), pathParts.end(), "detect") != pathParts.end()) {
            // 推理结果图片路径结构: .../mapid/poseid/detect/日期/文件名.png
            mapid = pathParts[pathParts.size() - 5];
            poseid = pathParts[pathParts.size() - 4];
            imageType = "result";
        } else {
            std::cerr << "[Image Sender] Unknown image type: " << imagePath << std::endl;
            continue;
        }
        
        std::cout << "[Image Sender] Extracted mapid: " << mapid << ", poseid: " << poseid 
                  << ", type: " << imageType << std::endl;
        
        // 发送图片，传递imageType参数
        positionServer->sendImage(imagePath, mapid, poseid, imageType);
    }
}

// 图片发送线程
// void imageSenderThread() {
//     while (true) {
//         std::string imagePath;
        
//         // 从队列中获取图片路径
//         {
//             std::unique_lock<std::mutex> lock(g_imageMutex);
//             if (g_imageQueue.empty()) {
//                 lock.unlock();
//                 std::this_thread::sleep_for(std::chrono::milliseconds(100));
//                 continue;
//             }
//             imagePath = g_imageQueue.front();
//             g_imageQueue.pop();
//         }
        
//         std::cout << "[Image Sender] Processing image: " << imagePath << std::endl;
        
//         // 从图片路径中解析 mapid 和 poseid
//         // 路径格式: .../mapid/poseid/raw/日期/capture_时间.png
//         fs::path pathObj(imagePath);
        
//         // 获取路径的各个组成部分
//         std::vector<std::string> pathParts;
//         for (const auto& part : pathObj) {
//             if (!part.empty()) {
//                 pathParts.push_back(part.string());
//             }
//         }
        
//         // 确保路径足够长以包含 mapid 和 poseid
//         if (pathParts.size() < 4) {
//             std::cerr << "[Image Sender] Invalid image path: " << imagePath << std::endl;
//             continue;
//         }
        
//         // 通常 mapid 是倒数第4部分，poseid 是倒数第3部分
//         // 路径结构: .../mapid/poseid/raw/日期/文件名.png
//         std::string mapid = pathParts[pathParts.size() - 5];
//         std::string poseid = pathParts[pathParts.size() - 4];
        
//         std::cout << "[Image Sender] Extracted mapid: " << mapid << ", poseid: " << poseid << std::endl;
        
//         // 发送图片
//         positionServer->sendImage(imagePath, mapid, poseid);


//     }
// }

// 图片推理线程
void detectThread() {
    while (true) {
        std::string imagePath;
        
        // 从推理队列中获取图片路径
        {
            std::unique_lock<std::mutex> lock(g_inferenceMutex);
            if (g_inferenceQueue.empty()) {
                lock.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            imagePath = g_inferenceQueue.front();
            g_inferenceQueue.pop();
        }
        
        std::cout << "[YOLO Worker] Processing image from queue: " << imagePath << std::endl;
        
        // 从图片路径中解析 mapid 和 poseid
        fs::path pathObj(imagePath);
        std::vector<std::string> pathParts;
        for (const auto& part : pathObj) {
            if (!part.empty()) {
                pathParts.push_back(part.string());
            }
        }
        
        // 确保路径足够长以包含 mapid 和 poseid
        if (pathParts.size() < 4) {
            std::cerr << "[YOLO Worker] Invalid image path: " << imagePath << std::endl;
            continue;
        }
        
        // 提取 mapid 和 poseid
        std::string mapid = pathParts[pathParts.size() - 5];
        std::string poseid = pathParts[pathParts.size() - 4];
        
        // 构建推理结果保存路径
        fs::path rawPath(imagePath);
        fs::path detectPath = rawPath;
        
        // 将路径中的 "raw" 替换为 "detect"
        std::vector<std::string> newParts;
        for (const auto& part : pathObj) {
            if (part == "raw") {
                newParts.push_back("detect");
            } else {
                newParts.push_back(part.string());
            }
        }
        
        // 重建路径
        fs::path resultPath;
        for (const auto& part : newParts) {
            resultPath /= part;
        }
        
        // 创建检测结果目录
        fs::create_directories(resultPath.parent_path());
        
        // YOLOv5 命令，指定输出目录为检测结果路径
        std::string yoloCmd = "python3 /home/gao/yolov5/detect_arm.py "
                              "--weights /home/gao/yolov5/runs/train/armXJ_coco_init2_cont/weights/best.pt "
                              "--source " + imagePath + " "
                              "--img 1024 --conf 0.25 "
                              "--project " + resultPath.parent_path().parent_path().string() + " "
                              "--name " + resultPath.parent_path().filename().string() + " "
                              "--exist-ok " // 允许覆盖现有文件
                              "> /dev/null 2>&1";
        
        int ret = system(yoloCmd.c_str());
        
        if (ret == 0) {
            std::cout << "[YOLO Worker] Inference finished: " << imagePath << std::endl;
            
            // 假设YOLO输出文件与输入文件同名
            std::string resultImagePath = resultPath.string();
            
            // 将推理结果图片加入发送队列
            {
                std::lock_guard<std::mutex> lock(g_imageMutex);
                g_imageQueue.push(resultImagePath);
                std::cout << "推理结果图片已加入发送队列: " << resultImagePath << std::endl;
            }
        } else {
            std::cerr << "[YOLO Worker] Inference failed: " << imagePath << std::endl;
        }
        
        // 短暂休眠
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}


//线程5：图片推理线程
// void detectThread(const std::string& watchDir) {
//     while (true) {
//         try {
//             for (const auto& entry : fs::directory_iterator(watchDir)) {
//                 if (entry.is_regular_file()) 
//                 {
//                     std::string filePath = entry.path().string();

//                     // 只处理 .png 文件，跳过已经推理过的 .done 文件
//                     if ((filePath.find(".jpg") != std::string::npos || 
//                     filePath.find(".png") != std::string::npos) &&
//                     filePath.find(".done") == std::string::npos) 
//                     {
                        
//                         //std::cout << "[YOLO Worker] Found new image: " << filePath << std::endl;
//                         std::string resultDir = "/home/gao/TCP_asdun_arm/yolo_results/";
                        
//                         // YOLOv5 命令，指定输出目录
//                         std::string yoloCmd = "python3 /home/gao/yolov5/detect.py "
//                                               "--weights /home/gao/yolov5/runs/train/knob_exp_finetune/weights/best.pt "
//                                               "--source " + filePath + " "
//                                               "--img 1024 --conf 0.25 "
//                                               "--project " + resultDir + " "
//                                               "--name result "
//                                               "--exist-ok " // 允许覆盖现有文件
//                                               "> /dev/null 2>&1";


//                         int ret = system(yoloCmd.c_str());

//                         if (ret == 0) {
//                             //std::cout << "[YOLO Worker] Inference finished: " << filePath << std::endl;
//                             // 给文件加标记，避免重复推理
//                             fs::rename(filePath, filePath + ".done");
//                         } else {
//                             //std::cerr << "[YOLO Worker] Inference failed: " << filePath << std::endl;
//                         }
//                     }
//                 }
//             }
//         } catch (const std::exception& e) {
//             //std::cerr << "[YOLO Worker] Error: " << e.what() << std::endl;
//         }

//         // 每 2 秒扫描一次目录
//         std::this_thread::sleep_for(std::chrono::seconds(2));
//     }
// }






int main() {

    positionServer=new MachineController (8888);  // 作为服务器监听8888端口        
    camera=new CameraController();
    estun=new MockEstunController ("192.168.2.5", "9000");
    
    // 连接机械臂
    if (!estun->connect()) {
        std::cerr << "Failed to connect to robot" << std::endl;
        return 1;
    }
    
    
    //启用机器人
    if (!estun->enableRobot()) {
        std::cerr << "Failed to enable robot" << std::endl;
        return 1;
    }
    
    // 设置自动模式
    if (!estun->setAutoMode()) {
        std::cerr << "Failed to set auto mode" << std::endl;
        return 1;
    }
    sleep(2);
    if (!estun->resetRobot()) {
        std::cerr << "Failed to go to global reset point" << std::endl;
        return 1;
    }
    g_robotState = RobotState::IDLE;

    std::thread taskWorker(taskThread);
    std::thread positionReceiver(positionReceiverThread);
    std::thread senderWorker(statusSenderThread);
    std::thread detectWorker(detectThread);
    std::thread imageSender(imageSenderThread);


    positionReceiver.detach();
    taskWorker.detach();
    senderWorker.detach();
    detectWorker.detach();
    imageSender.detach();


    while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    }  
        
    return 0;

}
