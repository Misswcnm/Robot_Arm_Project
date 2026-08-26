#include "GlobalDefines.h"

// 定义全局变量
std::atomic<RobotState> g_robotState(RobotState::IDLE);
std::mutex g_stateMutex;
std::condition_variable g_stateCV;
std::queue<std::string> g_positionQueue;
std::mutex g_queueMutex;
std::mutex g_feedbackMutex;
std::queue<std::string> g_statusQueue;
std::mutex g_statusMutex;
//已经发送图片集合
std::set<std::string> g_sentImages;
std::mutex g_sentImagesMutex;

//图片发送队列
std::queue<std::string> g_imageQueue;
std::mutex g_imageMutex;

std::queue<std::string> g_imageMsgQueue;
std::mutex g_imageMsgMutex;

