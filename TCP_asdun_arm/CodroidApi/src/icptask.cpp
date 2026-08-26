#include "IcpTask.h"
#include "IcpWallFilter.h"
#include "Define.h"

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <optional>
#include <unordered_set>

namespace fs = std::filesystem;

namespace
{
struct Voxel
{
    std::int64_t x;
    std::int64_t y;
    std::int64_t z;

    bool operator==(const Voxel &other) const noexcept
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelHash
{
    std::size_t operator()(const Voxel &voxel) const noexcept
    {
        std::size_t seed = std::hash<std::int64_t>{}(voxel.x);
        seed ^= std::hash<std::int64_t>{}(voxel.y) +
                0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        seed ^= std::hash<std::int64_t>{}(voxel.z) +
                0x9e3779b9U + (seed << 6U) + (seed >> 2U);
        return seed;
    }
};

std::vector<D455PointXYZ> voxelDown(
    const std::vector<D455PointXYZ> &points, double voxel_size_m)
{
    std::vector<D455PointXYZ> output;
    output.reserve(points.size());
    std::unordered_set<Voxel, VoxelHash> occupied;
    occupied.reserve(points.size());

    for (const D455PointXYZ &point : points) {
        const Voxel voxel{
            static_cast<std::int64_t>(std::floor(point.x_m / voxel_size_m)),
            static_cast<std::int64_t>(std::floor(point.y_m / voxel_size_m)),
            static_cast<std::int64_t>(std::floor(point.z_m / voxel_size_m)),
        };
        if (occupied.insert(voxel).second) {
            output.push_back(point);
        }
    }
    return output;
}

bool savePly(const fs::path &path, const std::vector<D455PointXYZ> &points)
{
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output.is_open()) {
        return false;
    }
    output << "ply\n"
           << "format binary_little_endian 1.0\n"
           << "element vertex " << points.size() << "\n"
           << "property float x\nproperty float y\nproperty float z\n"
           << "end_header\n";
    for (const D455PointXYZ &point : points) {
        output.write(reinterpret_cast<const char *>(&point.x_m), sizeof(float));
        output.write(reinterpret_cast<const char *>(&point.y_m), sizeof(float));
        output.write(reinterpret_cast<const char *>(&point.z_m), sizeof(float));
    }
    return output.good();
}

Eigen::Isometry3d poseTransform(const std::array<double, 6> &pose)
{
    const double a = pose[3] * M_PI / 180.0;
    const double b = pose[4] * M_PI / 180.0;
    const double c = pose[5] * M_PI / 180.0;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() =
        (Eigen::AngleAxisd(c, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(b, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    // Codroid CPos 的 XYZ 单位为 mm，与现有 ROS2 T_base_flange 一致。
    transform.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    return transform;
}

bool loadRecordedPose(const fs::path &path, int command,
                      Eigen::Isometry3d &transform, std::string &error)
{
    std::ifstream input(path);
    if (!input.is_open()) {
        error = "找不到 normalRecord 位姿文件: " + path.string();
        return false;
    }

    json points;
    try {
        input >> points;
        for (const auto &point : points) {
            if (point.value("command", -1) != command) {
                continue;
            }
            if (!point.contains("point_type") ||
                !point.at("point_type").is_number_integer() ||
                point.at("point_type").get<int>() != 1) {
                error = "command=" + std::to_string(command) +
                        " 不是 ICP 点(point_type=1)";
                return false;
            }
            const auto &pose = point.at("pose");
            const std::array<double, 6> values{
                pose.at("x").get<double>(), pose.at("y").get<double>(),
                pose.at("z").get<double>(), pose.at("a").get<double>(),
                pose.at("b").get<double>(), pose.at("c").get<double>(),
            };
            transform = poseTransform(values);
            return true;
        }
    } catch (const std::exception &exception) {
        error = "normalRecord 位姿解析失败: " + std::string(exception.what());
        return false;
    }

    error = "normalRecord 中不存在 command=" + std::to_string(command);
    return false;
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
}  // namespace

IcpTask::IcpTask(fs::path record_root)
    : record_root_(std::move(record_root))
{
}

bool IcpTask::recordReference(
    const std::string &mapid,
    const std::string &poseid,
    const std::vector<std::vector<D455PointXYZ>> &frames,
    std::string &error) const
{
    error.clear();
    if (frames.size() < 2U) {
        error = "ICP Ref 至少需要 2 帧有效点云";
        return false;
    }

    try {
        std::size_t total_points = 0;
        for (const auto &frame : frames) {
            total_points += frame.size();
        }
        std::vector<D455PointXYZ> merged;
        merged.reserve(total_points);
        for (const auto &frame : frames) {
            merged.insert(merged.end(), frame.begin(), frame.end());
        }
        if (merged.empty()) {
            error = "ICP Ref 点云为空";
            return false;
        }

        IcpWallPlane wall_plane;
        IcpWallFilterStats wall_stats;
        std::vector<D455PointXYZ> object_cloud;
        if (!removeIcpWallOnce(
                merged, IcpWallFilterOptions{}, std::nullopt,
                object_cloud, wall_plane, wall_stats, error)) {
            error = "ICP Ref 墙面识别失败，拒绝保存模板: " + error;
            return false;
        }
        std::cout << "[ICP] Ref 墙面只估计一次: raw="
                  << wall_stats.input_points
                  << " estimate_5mm=" << wall_stats.estimate_points
                  << " removed_ratio=" << wall_stats.removed_ratio
                  << " remaining=" << wall_stats.remaining_points
                  << " normal_camera=["
                  << wall_plane.normal_camera[0] << ","
                  << wall_plane.normal_camera[1] << ","
                  << wall_plane.normal_camera[2] << "]" << std::endl;

        const auto level20 = voxelDown(object_cloud, 0.020);
        const auto level10 = voxelDown(object_cloud, 0.010);
        const auto level5 = voxelDown(object_cloud, 0.005);
        if (level20.size() < 10U || level10.size() < 10U ||
            level5.size() < 10U) {
            error = "ICP Ref 去墙后模板点数不足，拒绝保存";
            return false;
        }
        const fs::path record_dir = record_root_ / mapid / poseid;
        fs::create_directories(record_dir);

        const fs::path path20 = record_dir / "1_20mm.ply";
        const fs::path path10 = record_dir / "1_10mm.ply";
        const fs::path path5 = record_dir / "1_5mm.ply";
        const fs::path wall_path = record_dir / "1_wall.json";
        if (!savePly(path20, level20) || !savePly(path10, level10) ||
            !savePly(path5, level5) ||
            !saveIcpWallPlane(wall_path, wall_plane, wall_stats, error)) {
            std::error_code ignored;
            fs::remove(path20, ignored);
            fs::remove(path10, ignored);
            fs::remove(path5, ignored);
            fs::remove(wall_path, ignored);
            if (!error.empty()) {
                error = "ICP Ref 模板点云保存失败: " + error;
            } else {
                error = "ICP Ref 模板点云保存失败";
            }
            return false;
        }
        return true;
    } catch (const std::exception &exception) {
        error = "ICP Ref 记录失败: " + std::string(exception.what());
        return false;
    }
}

bool IcpTask::recordWork(const std::string &mapid,
                         const std::string &poseid,
                         int command,
                         std::string &error) const
{
    error.clear();
    if (command <= 1) {
        error = "ICP Work command 必须大于 1";
        return false;
    }

    try {
        const fs::path station_dir = record_root_ / mapid / poseid;
        Eigen::Isometry3d reference_pose;
        Eigen::Isometry3d work_pose;
        const fs::path reference_path = station_dir / "1.json";
        const fs::path work_path = station_dir /
                                   (std::to_string(command) + ".json");
        if (!loadRecordedPose(reference_path, 1, reference_pose, error) ||
            !loadRecordedPose(work_path, command, work_pose, error)) {
            return false;
        }

        const Eigen::Isometry3d T_ref_work = reference_pose.inverse() * work_pose;
        json points;
        {
            std::ifstream input(work_path);
            input >> points;
        }
        bool updated = false;
        for (auto &point : points) {
            if (point.value("command", -1) == command) {
                point["trans"] = matrixJson(T_ref_work);
                updated = true;
                break;
            }
        }
        if (!updated) {
            error = "ICP Work JSON 中不存在 command=" +
                    std::to_string(command);
            return false;
        }

        std::ofstream output(work_path, std::ios::trunc);
        if (!output.is_open()) {
            error = "ICP Work 文件打开失败: " + work_path.string();
            return false;
        }
        output << points.dump(4);
        if (!output.good()) {
            error = "ICP Work 保存失败: " + work_path.string();
            return false;
        }
        return true;
    } catch (const std::exception &exception) {
        error = "ICP Work 记录失败: " + std::string(exception.what());
        return false;
    }
}
