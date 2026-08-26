#include "IcpWallFilter.h"
#include "Define.h"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <numeric>
#include <random>
#include <unordered_set>

namespace fs = std::filesystem;

namespace
{
using Point = Eigen::Vector3d;

struct Voxel
{
    std::int64_t x{};
    std::int64_t y{};
    std::int64_t z{};

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

bool validPoint(const D455PointXYZ &point)
{
    return std::isfinite(point.x_m) && std::isfinite(point.y_m) &&
           std::isfinite(point.z_m);
}

std::vector<Point> estimateCloud(const std::vector<D455PointXYZ> &input,
                                 double voxel_m,
                                 std::size_t maximum_points)
{
    std::vector<Point> downsampled;
    downsampled.reserve(input.size());
    std::unordered_set<Voxel, VoxelHash> occupied;
    occupied.reserve(input.size());
    for (const D455PointXYZ &value : input) {
        if (!validPoint(value)) {
            continue;
        }
        const Point point(value.x_m, value.y_m, value.z_m);
        const Voxel voxel{
            static_cast<std::int64_t>(std::floor(point.x() / voxel_m)),
            static_cast<std::int64_t>(std::floor(point.y() / voxel_m)),
            static_cast<std::int64_t>(std::floor(point.z() / voxel_m)),
        };
        if (occupied.insert(voxel).second) {
            downsampled.push_back(point);
        }
    }
    if (downsampled.size() <= maximum_points) {
        return downsampled;
    }
    std::vector<Point> sampled;
    sampled.reserve(maximum_points);
    const double step = static_cast<double>(downsampled.size()) /
                        static_cast<double>(maximum_points);
    for (std::size_t index = 0; index < maximum_points; ++index) {
        sampled.push_back(
            downsampled[static_cast<std::size_t>(index * step)]);
    }
    return sampled;
}

Point arrayPoint(const std::array<double, 3> &value)
{
    return Point(value[0], value[1], value[2]);
}

void orientNormal(Point &normal,
                  const std::optional<Point> &expected)
{
    if (expected) {
        if (normal.dot(*expected) < 0.0) {
            normal = -normal;
        }
    } else if (normal.z() < 0.0) {
        normal = -normal;
    }
}

double normalDifferenceDeg(const Point &left, const Point &right)
{
    const double dot = std::clamp(left.dot(right), -1.0, 1.0);
    return std::acos(dot) * 180.0 / std::acos(-1.0);
}

bool normalAllowed(const Point &normal,
                   const std::optional<Point> &expected,
                   const IcpWallFilterOptions &options)
{
    if (std::abs(normal.z()) < options.minimum_abs_normal_z) {
        return false;
    }
    return !expected ||
           normalDifferenceDeg(normal, *expected) <=
               options.maximum_reference_normal_difference_deg;
}
}  // namespace

bool removeIcpWallOnce(
    const std::vector<D455PointXYZ> &input,
    const IcpWallFilterOptions &options,
    const std::optional<std::array<double, 3>> &expected_normal_camera,
    std::vector<D455PointXYZ> &object_cloud,
    IcpWallPlane &wall_plane,
    IcpWallFilterStats &stats,
    std::string &error)
{
    error.clear();
    object_cloud.clear();
    wall_plane = IcpWallPlane{};
    stats = IcpWallFilterStats{};
    stats.input_points = input.size();
    if (input.size() < 30U || !std::isfinite(options.estimate_voxel_m) ||
        options.estimate_voxel_m <= 0.0 ||
        !std::isfinite(options.plane_distance_m) ||
        options.plane_distance_m <= 0.0 ||
        options.ransac_iterations < 1 ||
        options.maximum_estimate_points < 30U) {
        error = "ICP 墙面估计输入或参数无效";
        return false;
    }

    std::optional<Point> expected;
    if (expected_normal_camera) {
        Point value = arrayPoint(*expected_normal_camera);
        if (!value.allFinite() || value.norm() < 1e-8) {
            error = "ICP Ref 墙法向无效";
            return false;
        }
        expected = value.normalized();
    }

    const std::vector<Point> estimate = estimateCloud(
        input, options.estimate_voxel_m, options.maximum_estimate_points);
    stats.estimate_points = estimate.size();
    if (estimate.size() < 30U) {
        error = "ICP 5mm 墙面估计点数不足";
        return false;
    }

    std::mt19937 random(0x455U);
    std::uniform_int_distribution<std::size_t> choose(0, estimate.size() - 1);
    Point best_normal = Point::Zero();
    double best_offset = 0.0;
    std::size_t best_inliers = 0;
    for (int iteration = 0; iteration < options.ransac_iterations;
         ++iteration) {
        const Point &first = estimate[choose(random)];
        const Point &second = estimate[choose(random)];
        const Point &third = estimate[choose(random)];
        Point normal = (second - first).cross(third - first);
        if (normal.norm() < 1e-8) {
            continue;
        }
        normal.normalize();
        orientNormal(normal, expected);
        if (!normalAllowed(normal, expected, options)) {
            continue;
        }
        const double offset = -normal.dot(first);
        std::size_t inliers = 0;
        for (const Point &point : estimate) {
            if (std::abs(normal.dot(point) + offset) <=
                options.plane_distance_m) {
                ++inliers;
            }
        }
        if (inliers > best_inliers) {
            best_inliers = inliers;
            best_normal = normal;
            best_offset = offset;
        }
    }

    const double estimate_ratio = static_cast<double>(best_inliers) /
                                  static_cast<double>(estimate.size());
    if (best_inliers < 10U ||
        estimate_ratio < options.minimum_inlier_ratio) {
        error = "ICP 墙面识别失败: 5mm best_ratio=" +
                std::to_string(estimate_ratio) + " < " +
                std::to_string(options.minimum_inlier_ratio);
        return false;
    }

    Point centroid = Point::Zero();
    std::size_t refine_count = 0;
    for (const Point &point : estimate) {
        if (std::abs(best_normal.dot(point) + best_offset) <=
            options.plane_distance_m) {
            centroid += point;
            ++refine_count;
        }
    }
    if (refine_count < 3U) {
        error = "ICP 墙面 PCA 精修点数不足";
        return false;
    }
    centroid /= static_cast<double>(refine_count);
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const Point &point : estimate) {
        if (std::abs(best_normal.dot(point) + best_offset) <=
            options.plane_distance_m) {
            const Point centered = point - centroid;
            covariance += centered * centered.transpose();
        }
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) {
        error = "ICP 墙面 PCA 精修失败";
        return false;
    }
    Point refined_normal = solver.eigenvectors().col(0).normalized();
    orientNormal(refined_normal, expected);
    if (!normalAllowed(refined_normal, expected, options)) {
        error = "ICP 墙面法向与 Ref 超过限制";
        return false;
    }
    const double refined_offset = -refined_normal.dot(centroid);

    object_cloud.reserve(input.size());
    for (const D455PointXYZ &value : input) {
        if (!validPoint(value)) {
            continue;
        }
        const Point point(value.x_m, value.y_m, value.z_m);
        if (std::abs(refined_normal.dot(point) + refined_offset) <=
            options.plane_distance_m) {
            ++stats.removed_points;
        } else {
            object_cloud.push_back(value);
        }
    }
    stats.remaining_points = object_cloud.size();
    stats.removed_ratio = input.empty()
        ? 0.0
        : static_cast<double>(stats.removed_points) /
              static_cast<double>(input.size());
    stats.normal_difference_deg = expected
        ? normalDifferenceDeg(refined_normal, *expected)
        : 0.0;
    if (object_cloud.size() < options.minimum_remaining_points) {
        object_cloud.clear();
        error = "ICP 墙面删除后剩余点数不足: " +
                std::to_string(stats.remaining_points);
        return false;
    }

    wall_plane.normal_camera = {
        refined_normal.x(), refined_normal.y(), refined_normal.z()};
    wall_plane.offset_m = refined_offset;
    return true;
}

bool saveIcpWallPlane(const fs::path &path,
                      const IcpWallPlane &plane,
                      const IcpWallFilterStats &stats,
                      std::string &error)
{
    try {
        json document{
            {"normal_camera", plane.normal_camera},
            {"offset_m", plane.offset_m},
            {"removed_ratio", stats.removed_ratio},
            {"remaining_points", stats.remaining_points},
        };
        std::ofstream output(path, std::ios::trunc);
        if (!output.is_open()) {
            error = "无法保存 ICP Ref 墙面参数: " + path.string();
            return false;
        }
        output << document.dump(4);
        if (!output.good()) {
            error = "ICP Ref 墙面参数写入失败: " + path.string();
            return false;
        }
        return true;
    } catch (const std::exception &exception) {
        error = "ICP Ref 墙面参数保存失败: " +
                std::string(exception.what());
        return false;
    }
}

bool loadIcpWallPlane(const fs::path &path,
                      IcpWallPlane &plane,
                      std::string &error)
{
    try {
        std::ifstream input(path);
        if (!input.is_open()) {
            error = "ICP Ref 墙面参数不存在，请重新录制 ICP Ref: " +
                    path.string();
            return false;
        }
        json document;
        input >> document;
        const auto normal =
            document.at("normal_camera").get<std::array<double, 3>>();
        const Point value = arrayPoint(normal);
        const double offset = document.at("offset_m").get<double>();
        if (!value.allFinite() || value.norm() < 1e-8 ||
            !std::isfinite(offset)) {
            error = "ICP Ref 墙面参数无效: " + path.string();
            return false;
        }
        const Point normalized = value.normalized();
        plane.normal_camera = {
            normalized.x(), normalized.y(), normalized.z()};
        plane.offset_m = offset;
        return true;
    } catch (const std::exception &exception) {
        error = "ICP Ref 墙面参数解析失败: " +
                std::string(exception.what());
        return false;
    }
}
