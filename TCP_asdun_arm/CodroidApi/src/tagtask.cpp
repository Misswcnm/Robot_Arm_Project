#include "TagTask.h"
#include "Define.h"

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

extern "C"
{
#include "apriltag.h"
#include "tag36h11.h"
}

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <utility>

namespace fs = std::filesystem;

struct TagTask::Impl
{
    apriltag_family_t *family{nullptr};
    apriltag_detector_t *detector{nullptr};

    Impl()
    {
        family = tag36h11_create();
        detector = apriltag_detector_create();
        if (!family || !detector) {
            return;
        }
        apriltag_detector_add_family(detector, family);
        detector->nthreads = 2;
        detector->quad_decimate = 1.0;
        detector->quad_sigma = 0.0;
        detector->refine_edges = 1;
        detector->decode_sharpening = 0.25;
        detector->debug = 0;
    }

    ~Impl()
    {
        if (detector) {
            apriltag_detector_destroy(detector);
        }
        if (family) {
            tag36h11_destroy(family);
        }
    }

    bool ready() const noexcept
    {
        return family && detector;
    }

    bool detect(const D455Frame &frame, int tag_id, double tag_size_mm,
                Eigen::Isometry3d &camera_tag, std::string &error);
};

namespace
{
Eigen::Isometry3d poseTransform(const json &pose)
{
    const double pi = std::acos(-1.0);
    const double a = pose.at("a").get<double>() * pi / 180.0;
    const double b = pose.at("b").get<double>() * pi / 180.0;
    const double c = pose.at("c").get<double>() * pi / 180.0;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() =
        (Eigen::AngleAxisd(c, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(b, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    transform.translation() = Eigen::Vector3d(
        pose.at("x").get<double>(), pose.at("y").get<double>(),
        pose.at("z").get<double>());
    return transform;
}

Eigen::Isometry3d poseTransform(const std::array<double, 6> &pose)
{
    const double pi = std::acos(-1.0);
    const double a = pose[3] * pi / 180.0;
    const double b = pose[4] * pi / 180.0;
    const double c = pose[5] * pi / 180.0;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() =
        (Eigen::AngleAxisd(c, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(b, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    transform.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    return transform;
}

TagTask::Transform flatTransform(const Eigen::Isometry3d &transform)
{
    TagTask::Transform output{};
    const Eigen::Matrix4d matrix = transform.matrix();
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            output[static_cast<std::size_t>(row * 4 + column)] =
                matrix(row, column);
        }
    }
    return output;
}

json matrixJson(const Eigen::Isometry3d &transform)
{
    json rows = json::array();
    const Eigen::Matrix4d matrix = transform.matrix();
    for (int row = 0; row < 4; ++row) {
        rows.push_back({matrix(row, 0), matrix(row, 1),
                        matrix(row, 2), matrix(row, 3)});
    }
    return rows;
}

bool matrixFromJson(const json &value, Eigen::Isometry3d &transform,
                    std::string &error)
{
    if (!value.is_array() || value.size() != 4) {
        error = "位姿变换必须是 4x4 矩阵";
        return false;
    }
    Eigen::Matrix4d matrix;
    for (int row = 0; row < 4; ++row) {
        if (!value[row].is_array() || value[row].size() != 4) {
            error = "位姿变换必须是 4x4 矩阵";
            return false;
        }
        for (int column = 0; column < 4; ++column) {
            matrix(row, column) = value[row][column].get<double>();
            if (!std::isfinite(matrix(row, column))) {
                error = "位姿变换包含非有限数值";
                return false;
            }
        }
    }
    if (!matrix.row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 1e-9)) {
        error = "位姿变换最后一行不是 [0,0,0,1]";
        return false;
    }
    transform = Eigen::Isometry3d(matrix);
    return true;
}

bool loadCommand(const fs::path &path, int command, json &points,
                 std::size_t &record_index, std::string &error)
{
    std::ifstream input(path);
    if (!input.is_open()) {
        error = "找不到点位文件: " + path.string();
        return false;
    }
    try {
        input >> points;
        if (!points.is_array()) {
            error = "点位文件不是 JSON 数组: " + path.string();
            return false;
        }
        for (std::size_t index = 0; index < points.size(); ++index) {
            const auto &point = points[index];
            if (point.value("command", -1) != command) {
                continue;
            }
            if (!point.contains("point_type") ||
                !point.at("point_type").is_number_integer() ||
                point.at("point_type").get<int>() != 0) {
                error = "command=" + std::to_string(command) +
                        " 不是 AprilTag 点(point_type=0)";
                return false;
            }
            if (!point.contains("angle") || !point.contains("pose")) {
                error = "AprilTag 点缺少 angle 或 pose";
                return false;
            }
            record_index = index;
            return true;
        }
    } catch (const std::exception &exception) {
        error = "点位 JSON 解析失败: " + std::string(exception.what());
        return false;
    }
    error = "点位文件中不存在 command=" + std::to_string(command);
    return false;
}

bool saveCommands(const fs::path &path, const json &points, std::string &error)
{
    std::ofstream output(path, std::ios::trunc);
    if (!output.is_open()) {
        error = "点位文件打开失败: " + path.string();
        return false;
    }
    output << points.dump(4);
    if (!output.good()) {
        error = "点位文件保存失败: " + path.string();
        return false;
    }
    return true;
}

bool loadCalibrationDocument(const fs::path &configured_path,
                             json &document,
                             fs::path &resolved_path,
                             std::string &error)
{
    fs::path current = configured_path;
    if (!current.is_absolute()) {
        current = fs::absolute(current);
    }
    try {
        for (int depth = 0; depth < 4; ++depth) {
            std::ifstream input(current);
            if (!input.is_open()) {
                error = "标定文件不存在: " + current.string();
                return false;
            }
            input >> document;
            if (document.is_string()) {
                fs::path target = document.get<std::string>();
                current = target.is_absolute() ? target : current.parent_path() / target;
                continue;
            }
            resolved_path = current;
            return true;
        }
    } catch (const std::exception &exception) {
        error = "标定 JSON 解析失败: " + std::string(exception.what());
        return false;
    }
    error = "标定 JSON 指针层级超过 4 层";
    return false;
}

bool loadHandeye(const fs::path &configured_path,
                 Eigen::Isometry3d &transform, std::string &error)
{
    json document;
    fs::path resolved_path;
    if (!loadCalibrationDocument(
            configured_path, document, resolved_path, error)) {
        return false;
    }
    if (!document.is_object() || !document.contains("T_flange_camera") ||
        !document["T_flange_camera"].contains("matrix")) {
        error = "手眼标定缺少 T_flange_camera.matrix: " +
                resolved_path.string();
        return false;
    }
    return matrixFromJson(
        document["T_flange_camera"]["matrix"], transform, error);
}

bool validateTcpCalibration(const fs::path &configured_path,
                            std::string &error)
{
    json document;
    fs::path resolved_path;
    if (!loadCalibrationDocument(
            configured_path, document, resolved_path, error)) {
        return false;
    }
    if (!document.is_object() || !document.contains("result") ||
        !document["result"].contains("tcp_offset_flange_mm")) {
        error = "TCP 标定缺少 result.tcp_offset_flange_mm: " +
                resolved_path.string();
        return false;
    }
    const json &offset = document["result"]["tcp_offset_flange_mm"];
    if (!offset.is_array() || offset.size() != 3) {
        error = "TCP 标定 tcp_offset_flange_mm 必须是 3 元数组";
        return false;
    }
    for (const auto &value : offset) {
        if (!value.is_number() || !std::isfinite(value.get<double>())) {
            error = "TCP 标定 tcp_offset_flange_mm 包含无效数值";
            return false;
        }
    }
    return true;
}

}  // namespace

bool loadTagTaskConfig(const fs::path &config_path,
                       TagTaskConfig &config,
                       std::string &error)
{
    error.clear();
    try {
        fs::path path = config_path;
        if (!path.is_absolute()) {
            path = fs::absolute(path);
        }
        std::ifstream input(path);
        if (!input.is_open()) {
            error = "AprilTag 配置文件不存在: " + path.string();
            return false;
        }
        json document;
        input >> document;
        if (!document.is_object()) {
            error = "AprilTag 配置必须是 JSON 对象";
            return false;
        }

        TagTaskConfig loaded;
        loaded.tag_size_mm = document.at("tag_size_mm").get<double>();
        if (!std::isfinite(loaded.tag_size_mm) || loaded.tag_size_mm <= 0.0) {
            error = "tag_size_mm 必须是大于 0 的毫米数值";
            return false;
        }
        const auto resolve_path = [&path](const std::string &value) {
            const fs::path configured(value);
            return configured.is_absolute()
                ? configured
                : (path.parent_path() / configured).lexically_normal();
        };
        if (document.contains("record_root")) {
            loaded.record_root = resolve_path(
                document.at("record_root").get<std::string>());
        }
        loaded.handeye_json = resolve_path(
            document.at("handeye_json").get<std::string>());
        loaded.tcp_calibration_json = resolve_path(
            document.at("tcp_calibration_json").get<std::string>());

        Eigen::Isometry3d unused_handeye;
        if (!loadHandeye(loaded.handeye_json, unused_handeye, error) ||
            !validateTcpCalibration(loaded.tcp_calibration_json, error)) {
            return false;
        }
        config = std::move(loaded);
        return true;
    } catch (const std::exception &exception) {
        error = "AprilTag 配置解析失败: " +
                std::string(exception.what());
        return false;
    }
}

bool TagTask::Impl::detect(const D455Frame &frame, int tag_id,
                           double tag_size_mm,
                           Eigen::Isometry3d &camera_tag,
                           std::string &error)
{
    if (!ready()) {
        error = "AprilTag detector 初始化失败";
        return false;
    }
    if (tag_id < 0 || tag_size_mm <= 0.0) {
        error = "tag_id 必须非负且 tag_size_mm 必须大于 0";
        return false;
    }
    if (frame.color_bgr.empty() ||
        frame.color_bgr.cols != frame.color_intrinsics.width ||
        frame.color_bgr.rows != frame.color_intrinsics.height) {
        error = "彩色图与相机内参尺寸不一致";
        return false;
    }

    cv::Mat gray;
    cv::cvtColor(frame.color_bgr, gray, cv::COLOR_BGR2GRAY);
    image_u8_t image{
        gray.cols, gray.rows, static_cast<int>(gray.step), gray.data};
    zarray_t *raw = apriltag_detector_detect(detector, &image);
    if (!raw) {
        error = "AprilTag 检测返回空结果";
        return false;
    }

    std::array<cv::Point2d, 4> corners{};
    double best_margin = -std::numeric_limits<double>::infinity();
    bool found = false;
    for (int index = 0; index < zarray_size(raw); ++index) {
        apriltag_detection_t *detection = nullptr;
        zarray_get(raw, index, &detection);
        if (!detection || detection->id != tag_id || detection->hamming != 0 ||
            detection->decision_margin <= best_margin) {
            continue;
        }
        // 逐项复制旧 ROS2 pose_estimation.cpp：不排序、不旋转角点。
        corners = {
            cv::Point2d(detection->p[0][0], detection->p[0][1]),
            cv::Point2d(detection->p[1][0], detection->p[1][1]),
            cv::Point2d(detection->p[2][0], detection->p[2][1]),
            cv::Point2d(detection->p[3][0], detection->p[3][1]),
        };
        best_margin = detection->decision_margin;
        found = true;
    }
    apriltag_detections_destroy(raw);
    if (!found) {
        error = "当前彩色帧未检测到 Tag ID " + std::to_string(tag_id);
        return false;
    }

    // 物点严格复制旧 ROS2 顺序。这里直接使用 mm，因此 solvePnP 的 tvec 也是 mm。
    const double half = tag_size_mm * 0.5;
    const std::array<cv::Point3d, 4> object_points{
        cv::Point3d(-half, -half, 0.0),
        cv::Point3d(+half, -half, 0.0),
        cv::Point3d(+half, +half, 0.0),
        cv::Point3d(-half, +half, 0.0),
    };
    const cv::Matx33d camera_matrix(
        frame.color_intrinsics.fx, 0.0, frame.color_intrinsics.cx,
        0.0, frame.color_intrinsics.fy, frame.color_intrinsics.cy,
        0.0, 0.0, 1.0);
    cv::Mat distortion(1, 5, CV_64F);
    for (int index = 0; index < 5; ++index) {
        distortion.at<double>(0, index) =
            frame.color_intrinsics.distortion[index];
    }

    cv::Mat rvec;
    cv::Mat tvec;
    if (!cv::solvePnP(object_points, corners, camera_matrix, distortion,
                      rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)) {
        error = "solvePnP 失败";
        return false;
    }
    cv::Mat rotation;
    cv::Rodrigues(rvec, rotation);
    camera_tag = Eigen::Isometry3d::Identity();
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            camera_tag.linear()(row, column) =
                rotation.at<double>(row, column);
        }
        camera_tag.translation()(row) = tvec.at<double>(row, 0);
    }
    std::cout << "[APRILTAG] ID=" << tag_id
              << " decision_margin=" << best_margin
              << " T_camera_tag.xyz_mm=["
              << camera_tag.translation().transpose() << "]" << std::endl;
    return true;
}

TagTask::TagTask(TagTaskConfig config)
    : impl_(std::make_unique<Impl>()),
      record_root_(std::move(config.record_root)),
      handeye_path_(std::move(config.handeye_json)),
      tcp_calibration_path_(std::move(config.tcp_calibration_json)),
      tag_size_mm_(config.tag_size_mm)
{
}

TagTask::~TagTask() = default;

bool TagTask::locateBaseTag(
    const D455Frame &frame, int tag_id,
    const std::array<double, 6> &flange_pose,
    Transform &base_tag, std::string &error) const
{
    error.clear();
    try {
        for (double value : flange_pose) {
            if (!std::isfinite(value)) {
                error = "当前法兰位姿包含非有限数值";
                return false;
            }
        }
        Eigen::Isometry3d camera_tag;
        if (!impl_->detect(frame, tag_id, tag_size_mm_, camera_tag, error)) {
            return false;
        }
        Eigen::Isometry3d flange_camera;
        if (!loadHandeye(handeye_path_, flange_camera, error)) {
            return false;
        }
        const Eigen::Isometry3d transform =
            poseTransform(flange_pose) * flange_camera * camera_tag;
        base_tag = flatTransform(transform);
        std::cout << "[APRILTAG] 定位完成: T_base_tag.xyz_mm=["
                  << transform.translation().transpose() << "]" << std::endl;
        return true;
    } catch (const cv::Exception &exception) {
        error = "AprilTag/OpenCV 定位失败: " + std::string(exception.what());
    } catch (const std::exception &exception) {
        error = "AprilTag 定位失败: " + std::string(exception.what());
    }
    return false;
}

bool TagTask::baseLateralObservationPose(
    const std::array<double, 6> &center_flange_pose,
    double lateral_mm,
    std::array<double, 6> &shifted_flange_pose,
    std::string &error) const
{
    error.clear();
    if (!std::isfinite(lateral_mm)) {
        error = "AprilTag 末端左右平移量不是有限数值";
        return false;
    }
    shifted_flange_pose = center_flange_pose;
    shifted_flange_pose[1] += lateral_mm;
    return true;
}

bool TagTask::recordReference(const std::string &mapid,
                              const std::string &poseid,
                              const D455Frame &frame,
                              int tag_id,
                              std::string &error) const
{
    error.clear();
    try {
        Eigen::Isometry3d camera_tag;
        if (!impl_->detect(
                frame, tag_id, tag_size_mm_, camera_tag, error)) {
            return false;
        }

        const fs::path path = record_root_ / mapid / poseid / "1.json";
        json points;
        std::size_t index = 0;
        if (!loadCommand(path, 1, points, index, error)) {
            return false;
        }
        Eigen::Isometry3d flange_camera;
        if (!loadHandeye(handeye_path_, flange_camera, error)) {
            return false;
        }
        const Eigen::Isometry3d base_flange =
            poseTransform(points[index].at("pose"));
        const Eigen::Isometry3d base_tag =
            base_flange * flange_camera * camera_tag;
        points[index]["tag_id"] = tag_id;
        points[index]["tag_pose"] = matrixJson(base_tag);
        return saveCommands(path, points, error);
    } catch (const cv::Exception &exception) {
        error = "AprilTag/OpenCV Ref 失败: " + std::string(exception.what());
    } catch (const std::exception &exception) {
        error = "AprilTag Ref 保存失败: " + std::string(exception.what());
    }
    return false;
}

bool TagTask::recordWork(const std::string &mapid,
                         const std::string &poseid,
                         int command,
                         std::string &error) const
{
    error.clear();
    if (command <= 1) {
        error = "AprilTag Work command 必须大于 1";
        return false;
    }
    try {
        const fs::path station = record_root_ / mapid / poseid;
        json reference_points;
        json work_points;
        std::size_t reference_index = 0;
        std::size_t work_index = 0;
        if (!loadCommand(station / "1.json", 1, reference_points,
                         reference_index, error) ||
            !loadCommand(station / (std::to_string(command) + ".json"),
                         command, work_points, work_index, error)) {
            return false;
        }
        if (!reference_points[reference_index].contains("tag_pose")) {
            error = "AprilTag Ref 缺少 tag_pose";
            return false;
        }
        Eigen::Isometry3d base_tag;
        if (!matrixFromJson(reference_points[reference_index]["tag_pose"],
                            base_tag, error)) {
            return false;
        }
        const Eigen::Isometry3d base_flange_work =
            poseTransform(work_points[work_index].at("pose"));
        const Eigen::Isometry3d tag_flange_work =
            base_tag.inverse() * base_flange_work;
        work_points[work_index]["trans"] = matrixJson(tag_flange_work);
        return saveCommands(
            station / (std::to_string(command) + ".json"),
            work_points, error);
    } catch (const std::exception &exception) {
        error = "AprilTag Work 保存失败: " + std::string(exception.what());
        return false;
    }
}
