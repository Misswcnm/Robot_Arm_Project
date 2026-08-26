#include <iostream>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <iomanip>
#include <sstream>

#include "RobotTask.h"
#include "EstunController.h"
#include "CameraController.h"
#include "MachineController.h"

namespace fs = std::filesystem;

//using json = nlohmann::json;


// 定义全局控制器指针
EstunController* estun = nullptr;
CameraController* camera = nullptr;
MachineController* positionServer = nullptr;

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
                std::cout << "11111111111111111111111111111111111: " <<  std::endl;
                std::cout <<positionarm<<std::endl;
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
                std::cout << "2222222222222222222222222222"<<std::endl;

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
        
        // 解析位置数据
        auto movementPoints = parsePositionData(mapid, poseid);
        if (movementPoints.empty()) 
        {
            std::string map_poseid = mapid + "_" + poseid;
            std::cerr << "No movement points found for prefix: " << map_poseid << std::endl;
            g_robotState = RobotState::FAULT;
            
            // 故障后复位
            estun->resetRobot();
            g_robotState = RobotState::IDLE;
            continue;
        }
        
        // 机械臂自动状态
        estun->setAutoMode();
        
        // 执行点到点运动
        for (auto& point : movementPoints) {

            // 重置拍照状态
            camera->resetCaptureState();
            
            
            // 移动到目标点
            g_robotState = RobotState::MOVING;
            std::cout << "Moving to point: (" 
                        << point.apos.jntPos[0] << ", " 
                        << point.apos.jntPos[1] << ", " 
                        << point.apos.jntPos[2] << ", " 
                        << point.apos.jntPos[3] << ", " 
                        << point.apos.jntPos[4] << ", " 
                        << point.apos.jntPos[5] << ")" << std::endl;

            // 将 c2::Point 转换为关节位置向量
            std::vector<double> targetPosition;
            for (int i = 0; i < 6; ++i) {
                targetPosition.push_back(point.apos.jntPos[i]);
            }
            
            // 执行移动
            if (!estun->moveJ(point, 30, 30)) {
                std::cerr << "Failed to move to point!" << std::endl;
                g_robotState = RobotState::FAULT;
                break;
            }  

            int waitCount = 0;         
            // 等待运动完成
            while (!estun->isMovementComplete(targetPosition)) 
            {
                    // 添加超时检测
                    if (waitCount++ > 200) 
                    {  // 200 * 50ms = 10秒超时
                        std::cerr << "Movement timeout!" << std::endl;
                        g_robotState = RobotState::FAULT;
                        break;
                    }
                    
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            
            // 拍照
            g_robotState = RobotState::IMAGING;
            std::cout << "Capturing image..." << std::endl;
            camera->captureImage();



            
            // 等待拍照完成
            while (!camera->isImageCaptured()) 
            {

                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            

            
            // 添加机械臂稳定时间
            camera->stabilizeAfterCapture();
            
            // 更新状态为准备移动
            g_robotState = RobotState::MOVING;
        }
        
        // 完成循环，复位机械臂
        if (g_robotState != RobotState::FAULT ) 
        {
            g_robotState = RobotState::RESETTING;
            estun->resetRobot();
            //状态字段
            json statusJson;
            statusJson["mapid"] = mapid;
            statusJson["poseid"] = poseid;
            statusJson["status"] = "done";   // 或者写 "1"
            std::string statusMsg = statusJson.dump();

            // 发送完成状态
            // std::string statusMsg = "作业点" + positionPrefix + "任务完成";
            {
                std::lock_guard<std::mutex> lock(g_statusMutex);
                g_statusQueue.push(statusMsg);
                std::cout << "发送状态: " << g_statusQueue.front()<< std::endl;
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
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

//传给客户端图片111
void imageMsgSenderThread() {
    while (true) {
        std::string imageMsg;
        {
            std::lock_guard<std::mutex> lock(g_imageMsgMutex);
            if (!g_imageMsgQueue.empty()) {
                imageMsg = g_imageMsgQueue.front();
                g_imageMsgQueue.pop();
            }
        }
        
        if (!imageMsg.empty()) 
        {
            // 使用专门的发送函数或直接使用 positionServer->sendStatus
            positionServer->sendStatus(imageMsg);
            std::cout << "已发送图片消息: " << imageMsg.substr(0, 100) << "..." << std::endl; // 只打印部分内容
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}




//线程5：图片推理线程111
void detectThread(const std::string& watchDir) {
    while (true) {
        try {
            for (const auto& entry : fs::directory_iterator(watchDir)) {
                if (entry.is_regular_file()) 
                {
                    std::string filePath = entry.path().string();
                    
                    // 检查是否为已推理但未发送的图片
                    if (filePath.find(".done") != std::string::npos) {
                        {
                            std::lock_guard<std::mutex> lock(g_sentImagesMutex);
                            // 如果图片尚未发送，则加入队列
                            if (g_sentImages.find(filePath) == g_sentImages.end()) {
                                std::lock_guard<std::mutex> imgLock(g_imageMutex);
                                g_imageQueue.push(filePath);
                                std::cout << "Added to send queue: " << filePath << std::endl;
                            }
                        }
                        continue;
                    }
                   
                    // 只处理 .png 文件，跳过已经推理过的 .done 文件
                    if ((filePath.find(".jpg") != std::string::npos || 
                    filePath.find(".png") != std::string::npos) &&
                    filePath.find(".done") == std::string::npos) 
                    {
                        // 获取原始文件名（不带路径）
                        std::string filename = fs::path(filePath).filename().string();
                        
                        // 设置 YOLO 输出目录
                        std::string resultDir = "/home/gao/TCP_asdun_arm/yolo_results/";
                        
                        // YOLOv5 命令，指定输出目录
                        std::string yoloCmd = "python3 /home/gao/yolov5/detect.py "
                                              "--weights /home/gao/yolov5/runs/train/knob_exp_finetune/weights/best.pt "
                                              "--source " + filePath + " "
                                              "--img 1024 --conf 0.25 "
                                              "--project " + resultDir + " "
                                              "--name result "
                                              "--exist-ok " // 允许覆盖现有文件
                                              "> /dev/null 2>&1";

                        int ret = system(yoloCmd.c_str());

                        if (ret == 0) {
                            // 构建推理结果图片的路径
                            // YOLO 默认会在项目目录下创建 expX 文件夹，我们使用 --name result 指定了文件夹名
                            std::string resultImagePath = resultDir + "result/" + filename;
                            
                            // 检查结果图片是否存在
                            if (fs::exists(resultImagePath)) {
                                // 将推理结果图片加入发送队列
                                {
                                    std::lock_guard<std::mutex> imgLock(g_imageMutex);
                                    g_imageQueue.push(resultImagePath);
                                    std::cout << "Added inference result to send queue: " << resultImagePath << std::endl;
                                }
                                
                                // 标记原始图片为已处理
                                fs::rename(filePath, filePath + ".done");
                            } else {
                                std::cerr << "Inference result not found: " << resultImagePath << std::endl;
                            }
                        } else {
                            std::cerr << "YOLO inference failed for: " << filePath << std::endl;
                        }
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "[Detect Thread] Error: " << e.what() << std::endl;
        }

        // 每 2 秒扫描一次目录
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}


//处理图片线程
void imageSenderThread() {
    while (true) {
        std::string imagePath;
        {
            std::lock_guard<std::mutex> lock(g_imageMutex);
            if (!g_imageQueue.empty()) {
                imagePath = g_imageQueue.front();
                g_imageQueue.pop();
            }
        }
        
        if (!imagePath.empty()) {
            // 检查是否已发送
            {
                std::lock_guard<std::mutex> lock(g_sentImagesMutex);
                if (g_sentImages.find(imagePath) != g_sentImages.end()) {
                    continue; // 已发送过，跳过
                }
            }
            
            try {
                // 读取图片文件
                std::ifstream imageFile(imagePath, std::ios::binary);
                if (!imageFile) {
                    std::cerr << "Failed to open image: " << imagePath << std::endl;
                    continue;
                }
                
                // 将图片内容读入缓冲区，使用 unsigned char
                std::vector<unsigned char> buffer(
                    (std::istreambuf_iterator<char>(imageFile)),
                    std::istreambuf_iterator<char>()
                );
                
                // 创建包含图片数据的JSON消息
                nlohmann::json imageMsg;
                imageMsg["type"] = "image";
                imageMsg["filename"] = std::filesystem::path(imagePath).filename().string();
                imageMsg["data"] = nlohmann::json::binary(buffer);
                
                // 将消息放入图片消息队列，而不是状态队列
                std::string message = imageMsg.dump();
                {
                    std::lock_guard<std::mutex> lock(g_imageMsgMutex);
                    g_imageMsgQueue.push(message);
                    std::cout << "图片消息已加入队列: " << imagePath << std::endl;
                }
                
                
                // 标记为已发送
                {
                    std::lock_guard<std::mutex> lock(g_sentImagesMutex);
                    g_sentImages.insert(imagePath);
                }
                
                std::cout << "Sent image: " << imagePath << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "Error sending image: " << e.what() << std::endl;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}