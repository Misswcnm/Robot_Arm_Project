#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

struct D455PointXYZ
{
    float x_m{};
    float y_m{};
    float z_m{};
};

struct D455Intrinsics
{
    int width{};
    int height{};
    float fx{};
    float fy{};
    float cx{};
    float cy{};
    int distortion_model{};
    std::array<float, 5> distortion{};
};

struct D455Frame
{
    // 独立拥有数据，离开 librealsense frameset 后仍然有效。
    cv::Mat color_bgr;
    std::vector<D455PointXYZ> point_cloud_color_m;
    D455Intrinsics color_intrinsics;
    double color_timestamp_ms{};
};

// 不依赖 ROS。按需启动 D455；ICP 的彩色图和点云来自同一组 frameset。
class D455Camera
{
public:
    D455Camera();
    ~D455Camera();

    D455Camera(const D455Camera &) = delete;
    D455Camera &operator=(const D455Camera &) = delete;

    // ICP：彩色 + 深度，并生成彩色光学坐标系 XYZ 点云。
    bool start(std::string &error);
    bool captureOne(D455Frame &output, std::string &error);

    // ICP 执行：相机保持启动；清空旧缓存后丢弃指定数量的新 frameset，
    // 只对最后一张执行对齐和点云计算，避免 NX 重复做无用计算。
    bool captureFreshPointCloud(D455Frame &output,
                                int discard_new_frames,
                                std::string &error);

    // AprilTag：只启用彩色流，不启动深度、对齐和点云计算。
    bool startColorOnly(std::string &error);
    bool captureColorOne(D455Frame &output, std::string &error);
    void stop() noexcept;
    bool running() const noexcept;

private:
    bool capturePointCloud(D455Frame &output,
                           int discard_new_frames,
                           bool copy_color_image,
                           std::string &error);

    struct Impl;
    std::unique_ptr<Impl> impl_;
};
