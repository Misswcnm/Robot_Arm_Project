#pragma once

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <queue>
#include <atomic>
#include <filesystem> 
#include <set>
#include "nlohmann/json.hpp"
#include "CodroidApi.h"
#include "EstunController.h"
#include "MachineController.h"
#include "CameraController.h"
#include "GlobalDefines.h"


// 声明全局控制器指针
extern EstunController* estun;
extern CameraController* camera;
extern MachineController* positionServer;

// 声明JSON解析函数
std::vector<c2::Point> parsePositionData(const std::string& mapid, const std::string& poseid);

// 声明线程函数
void taskThread();
void positionReceiverThread();
void statusSenderThread();
void detectThread(const std::string& watchDir);
void imageSenderThread();