#ifndef GLOBAL_DEFINES_H
#define GLOBAL_DEFINES_H

#include <atomic>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <set>
#include <string>

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
// 声明全局变量为extern（只声明，不定义）
extern std::atomic<RobotState> g_robotState;
extern std::mutex g_stateMutex;
extern std::condition_variable g_stateCV;
//位置消息队列
extern std::queue<std::string> g_positionQueue;
extern std::mutex g_queueMutex;
extern std::mutex g_feedbackMutex;
//机械臂状态队列
extern std::queue<std::string> g_statusQueue;
extern std::mutex g_statusMutex;


//图片发送队列
extern std::queue<std::string> g_imageQueue;
extern std::mutex g_imageMutex;

extern std::queue<std::string> g_imageMsgQueue;
extern std::mutex g_imageMsgMutex;

//已发送图片队列，供推理查询已推理未发送的图片
extern std::set<std::string> g_sentImages;
extern std::mutex g_sentImagesMutex;


#endif // GLOBAL_DEFINES_H