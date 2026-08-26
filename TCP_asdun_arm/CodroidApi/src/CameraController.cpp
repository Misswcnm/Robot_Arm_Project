#include "CameraController.h"
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <thread>

CameraController::CameraController() : m_captureSuccess(false) {}

bool CameraController::captureImage() {
   m_captureSuccess = false;  // 重置捕获状态
        
        try {
            // RealSense管道配置
            rs2::pipeline pipe;
            rs2::config cfg;
            cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
            
            // 启动管道并捕获帧
            pipe.start(cfg);
            rs2::frameset frames = pipe.wait_for_frames();
            rs2::video_frame color_frame = frames.get_color_frame();
            
            // 转换到OpenCV格式
            cv::Mat image(cv::Size(640, 480), CV_8UC3, 
                         (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            
            // 生成带时间戳的文件名
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&in_time_t), "capture_%Y%m%d_%H%M%S.png");
            std::string filename = "/home/gao/TCP_asdun_arm/picture/" + ss.str(); // 修改为实际存储路径
            
            // 保存图像
            if (cv::imwrite(filename, image)) {
                std::cout << "Image saved to: " << filename << std::endl;
                m_captureSuccess = true;
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

bool CameraController::isImageCaptured() const {
    return m_captureSuccess;
}

void CameraController::stabilizeAfterCapture() {
    // 等待2秒确保机械臂稳定
    std::this_thread::sleep_for(std::chrono::seconds(2));
}

void CameraController::resetCaptureState() {
    m_captureSuccess = false;
}