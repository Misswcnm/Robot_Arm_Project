#include "D455Camera.h"

#include <cmath>
#include <iostream>
#include <utility>

#include <librealsense2/rs.hpp>
#include <opencv2/imgproc.hpp>

struct D455Camera::Impl
{
    rs2::pipeline pipeline;
    std::unique_ptr<rs2::align> align_to_color;
    rs2::pointcloud pointcloud;
    bool started{false};
    bool depth_enabled{false};
};

namespace
{
bool copyColorMetadata(const rs2::video_frame &color, D455Frame &output,
                       std::string &error)
{
    if (!color) {
        error = "frameset 中缺少彩色帧";
        return false;
    }
    const rs2_intrinsics intrinsics =
        color.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    output.color_intrinsics.width = intrinsics.width;
    output.color_intrinsics.height = intrinsics.height;
    output.color_intrinsics.fx = intrinsics.fx;
    output.color_intrinsics.fy = intrinsics.fy;
    output.color_intrinsics.cx = intrinsics.ppx;
    output.color_intrinsics.cy = intrinsics.ppy;
    output.color_intrinsics.distortion_model = static_cast<int>(intrinsics.model);
    for (std::size_t index = 0;
         index < output.color_intrinsics.distortion.size(); ++index) {
        output.color_intrinsics.distortion[index] = intrinsics.coeffs[index];
    }
    output.color_timestamp_ms = color.get_timestamp();
    return true;
}

bool copyColorFrame(const rs2::video_frame &color, D455Frame &output,
                    std::string &error)
{
    if (!copyColorMetadata(color, output, error)) {
        return false;
    }
    const int width = color.get_width();
    const int height = color.get_height();
    const cv::Mat rgb(
        height, width, CV_8UC3, const_cast<void *>(color.get_data()),
        static_cast<std::size_t>(color.get_stride_in_bytes()));
    cv::cvtColor(rgb, output.color_bgr, cv::COLOR_RGB2BGR);
    if (output.color_bgr.empty()) {
        error = "彩色帧转换结果为空";
        return false;
    }
    return true;
}
}  // namespace

D455Camera::D455Camera() : impl_(std::make_unique<Impl>()) {}

D455Camera::~D455Camera()
{
    stop();
}

bool D455Camera::start(std::string &error)
{
    error.clear();
    if (impl_->started) {
        return true;
    }

    try {
        rs2::config config;
        // 与 NX 上已经验证过的低负载彩色配置一致；点云只按需抓一帧。
        config.enable_stream(
            RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 15);
        config.enable_stream(
            RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 15);

        impl_->pipeline.start(config);
        impl_->align_to_color =
            std::make_unique<rs2::align>(RS2_STREAM_COLOR);
        impl_->started = true;
        impl_->depth_enabled = true;

        // 给自动曝光和深度传感器预热约 1 秒；这些帧不用于记录。
        for (int index = 0; index < 15; ++index) {
            impl_->pipeline.wait_for_frames(5000);
        }
        return true;
    } catch (const rs2::error &exception) {
        error = std::string("RealSense 启动失败: ") + exception.what();
    } catch (const std::exception &exception) {
        error = std::string("相机启动失败: ") + exception.what();
    }

    stop();
    return false;
}

bool D455Camera::startColorOnly(std::string &error)
{
    error.clear();
    if (impl_->started) {
        if (!impl_->depth_enabled) {
            return true;
        }
        error = "D455 已按彩色+深度模式启动";
        return false;
    }

    try {
        rs2::config config;
        config.enable_stream(
            RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 15);
        impl_->pipeline.start(config);
        impl_->started = true;
        impl_->depth_enabled = false;
        for (int index = 0; index < 15; ++index) {
            impl_->pipeline.wait_for_frames(5000);
        }
        return true;
    } catch (const rs2::error &exception) {
        error = std::string("RealSense 彩色流启动失败: ") + exception.what();
    } catch (const std::exception &exception) {
        error = std::string("相机彩色流启动失败: ") + exception.what();
    }
    stop();
    return false;
}

bool D455Camera::captureColorOne(D455Frame &output, std::string &error)
{
    error.clear();
    output = D455Frame{};
    if (!impl_->started) {
        error = "D455 尚未启动";
        return false;
    }
    try {
        rs2::frameset stale;
        while (impl_->pipeline.poll_for_frames(&stale)) {
        }
        const rs2::frameset frames = impl_->pipeline.wait_for_frames(5000);
        return copyColorFrame(frames.get_color_frame(), output, error);
    } catch (const rs2::error &exception) {
        error = std::string("RealSense 彩色取帧失败: ") + exception.what();
    } catch (const cv::Exception &exception) {
        error = std::string("OpenCV 彩色转换失败: ") + exception.what();
    } catch (const std::exception &exception) {
        error = std::string("相机彩色取帧失败: ") + exception.what();
    }
    return false;
}

bool D455Camera::captureOne(D455Frame &output, std::string &error)
{
    return capturePointCloud(output, 0, true, error);
}

bool D455Camera::captureFreshPointCloud(D455Frame &output,
                                        int discard_new_frames,
                                        std::string &error)
{
    return capturePointCloud(
        output, discard_new_frames, false, error);
}

bool D455Camera::capturePointCloud(D455Frame &output,
                                   int discard_new_frames,
                                   bool copy_color_image,
                                   std::string &error)
{
    error.clear();
    output = D455Frame{};
    if (!impl_->started) {
        error = "D455 尚未启动";
        return false;
    }
    if (!impl_->depth_enabled || !impl_->align_to_color) {
        error = "D455 当前是仅彩色模式，不能生成点云";
        return false;
    }
    if (discard_new_frames < 0) {
        error = "丢弃新帧数量不能为负数";
        return false;
    }

    try {
        // 丢弃停稳期间留在同步器中的旧帧。
        rs2::frameset stale;
        while (impl_->pipeline.poll_for_frames(&stale)) {
        }

        // 只等待，不做 align/pointcloud.calculate；最后一张才生成点云。
        for (int index = 0; index < discard_new_frames; ++index) {
            impl_->pipeline.wait_for_frames(5000);
        }

        const rs2::frameset raw = impl_->pipeline.wait_for_frames(5000);
        const rs2::frameset aligned =
            impl_->align_to_color->process(raw).as<rs2::frameset>();
        const rs2::video_frame color = aligned.get_color_frame();
        const rs2::depth_frame depth = aligned.get_depth_frame();
        if (!color || !depth) {
            error = "同一 frameset 中缺少彩色帧或深度帧";
            return false;
        }

        const bool color_ok = copy_color_image
            ? copyColorFrame(color, output, error)
            : copyColorMetadata(color, output, error);
        if (!color_ok) {
            return false;
        }

        // 深度先对齐到彩色流，再生成 XYZ 点云。当前不使用纹理坐标，
        // 因此不调用 pointcloud.map_to(color)，减少 NX 上的无用计算。
        const rs2::points points = impl_->pointcloud.calculate(depth);
        const rs2::vertex *vertices = points.get_vertices();
        output.point_cloud_color_m.reserve(points.size());
        for (std::size_t index = 0; index < points.size(); ++index) {
            const rs2::vertex &point = vertices[index];
            if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                !std::isfinite(point.z) || point.z <= 0.0F) {
                continue;
            }
            output.point_cloud_color_m.push_back(
                {point.x, point.y, point.z});
        }

        if (output.point_cloud_color_m.empty()) {
            error = "点云没有有效 XYZ 点";
            return false;
        }
        return true;
    } catch (const rs2::error &exception) {
        error = std::string("RealSense 取帧失败: ") + exception.what();
    } catch (const cv::Exception &exception) {
        error = std::string("OpenCV 转换失败: ") + exception.what();
    } catch (const std::exception &exception) {
        error = std::string("相机取帧失败: ") + exception.what();
    }
    return false;
}

void D455Camera::stop() noexcept
{
    if (!impl_ || !impl_->started) {
        return;
    }
    try {
        impl_->pipeline.stop();
    } catch (const std::exception &exception) {
        std::cerr << "D455 停止失败: " << exception.what() << std::endl;
    }
    impl_->align_to_color.reset();
    impl_->started = false;
    impl_->depth_enabled = false;
}

bool D455Camera::running() const noexcept
{
    return impl_ && impl_->started;
}
