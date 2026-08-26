#include <iostream>
#include <string>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <thread>
#include <mutex>

namespace fs = std::filesystem;

// 全局变量（原函数中使用的）
std::mutex g_imageMutex;
std::queue<std::string> g_imageQueue;

class CameraCapture {
public:
    bool captureImage() {  // 移除mapid和poseid参数
        m_captureSuccess = false;  // 重置捕获状态
        
        try {
            // RealSense管道配置
            rs2::pipeline pipe;
            rs2::config cfg;
            cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
            
            // 启动管道并捕获帧
            pipe.start(cfg);

            // 相机预热
            std::cout << "相机预热中..." << std::endl;
            for(int i = 0; i < 50; i++) {
                rs2::frameset frames = pipe.wait_for_frames();
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            std::cout << "预热完成，开始拍摄照片" << std::endl;
            
            rs2::frameset frames = pipe.wait_for_frames();
            rs2::video_frame color_frame = frames.get_color_frame();
            
            // 转换到OpenCV格式并转换颜色空间
            cv::Mat image_rgb(cv::Size(640, 480), CV_8UC3, 
                            (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat image;
            cv::cvtColor(image_rgb, image, cv::COLOR_RGB2BGR); // RGB转BGR

            // 生成带时间戳的文件名
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now);
            
            std::stringstream filename_ss;
            filename_ss << std::put_time(std::localtime(&in_time_t), "capture_%Y%m%d_%H%M%S");
            filename_ss << ".png";
            std::string filename = filename_ss.str();
            
            // 完整文件路径（直接保存在当前目录）
            fs::path full_path = fs::current_path() / filename;
            
            // 保存图像
            if (cv::imwrite(full_path.string(), image)) {
                std::cout << "图像保存至: " << full_path.string() << std::endl;
                m_captureSuccess = true;
                m_lastImagePath = full_path.string();
                
                // 加入队列（原功能）
                {
                    std::lock_guard<std::mutex> lock(g_imageMutex);
                    g_imageQueue.push(m_lastImagePath);
                    std::cout << "图片已加入处理队列: " << m_lastImagePath << std::endl;
                }
            } else {
                std::cerr << "保存图像失败!" << std::endl;
            }
            
            pipe.stop();
        } 
        catch (const rs2::error & e) {
            std::cerr << "RealSense错误: " << e.what() << std::endl;
        }
        catch (const std::exception & e) {
            std::cerr << "错误: " << e.what() << std::endl;
        }
        
        return m_captureSuccess;
    }

    std::string getLastImagePath() const { return m_lastImagePath; }

private:
    bool m_captureSuccess = false;
    std::string m_lastImagePath;
};

int main() {
    CameraCapture camera;
    
    std::cout << "按回车键拍照（输入'q'退出）..." << std::endl;
    
    while (true) {
        std::string input;
        std::getline(std::cin, input);
        
        if (input == "q") {
            std::cout << "退出程序" << std::endl;
            break;
        }
        
        // 执行拍照
        if (camera.captureImage()) {
            std::cout << "\n拍照成功！" << std::endl;
            std::cout << "图像路径: " << camera.getLastImagePath() << std::endl;
            
            // 可选：使用OpenCV显示拍摄的图像
            cv::Mat img = cv::imread(camera.getLastImagePath());
            if (!img.empty()) {
                cv::namedWindow("拍摄结果", cv::WINDOW_AUTOSIZE);
                cv::imshow("拍摄结果", img);
                std::cout << "按任意键关闭窗口..." << std::endl;
                cv::waitKey(0);
                cv::destroyAllWindows();
            } else {
                std::cerr << "无法读取保存的图像文件" << std::endl;
            }
        } else {
            std::cerr << "拍照失败，请检查错误信息" << std::endl;
        }
        
        std::cout << "\n按回车键继续拍照（输入'q'退出）..." << std::endl;
    }

    return 0;
}