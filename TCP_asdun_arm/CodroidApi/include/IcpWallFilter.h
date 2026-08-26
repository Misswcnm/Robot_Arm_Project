#pragma once

#include "D455Camera.h"

#include <array>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

struct IcpWallPlane
{
    std::array<double, 3> normal_camera{};
    double offset_m{};
};

struct IcpWallFilterStats
{
    std::size_t input_points{};
    std::size_t estimate_points{};
    std::size_t removed_points{};
    std::size_t remaining_points{};
    double removed_ratio{};
    double normal_difference_deg{};
};

struct IcpWallFilterOptions
{
    // 5 mm 体素只用于墙面估计；最终删除操作回到输入原始点云。
    double estimate_voxel_m{0.005};
    double plane_distance_m{0.012};
    double minimum_inlier_ratio{0.25};
    double minimum_abs_normal_z{0.65};
    double maximum_reference_normal_difference_deg{10.0};
    int ransac_iterations{64};
    std::size_t maximum_estimate_points{8000};
    std::size_t minimum_remaining_points{200};
};

// 原始云 -> 5mm 墙估计 -> 回到原始云删除一次。失败时绝不返回完整场景。
bool removeIcpWallOnce(
    const std::vector<D455PointXYZ> &input,
    const IcpWallFilterOptions &options,
    const std::optional<std::array<double, 3>> &expected_normal_camera,
    std::vector<D455PointXYZ> &object_cloud,
    IcpWallPlane &wall_plane,
    IcpWallFilterStats &stats,
    std::string &error);

bool saveIcpWallPlane(const std::filesystem::path &path,
                      const IcpWallPlane &plane,
                      const IcpWallFilterStats &stats,
                      std::string &error);

bool loadIcpWallPlane(const std::filesystem::path &path,
                      IcpWallPlane &plane,
                      std::string &error);
