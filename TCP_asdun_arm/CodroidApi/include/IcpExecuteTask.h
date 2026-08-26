#pragma once

#include "D455Camera.h"
#include "Define.h"
#include "IcpWallFilter.h"

#include <array>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

using IcpTransform = std::array<double, 16>;

struct IcpReferenceRecord
{
    std::array<double, 6> angle{};
    std::array<double, 6> pose{};
    std::array<std::filesystem::path, 3> template_paths;
    IcpWallPlane wall_plane;
};

struct IcpWorkRecord
{
    int command{};
    IcpTransform trans{};  // T_ref_flange_work，按行展开。
};

struct IcpExecutionPlan
{
    IcpReferenceRecord reference;
    std::vector<IcpWorkRecord> works;
};

struct IcpExecuteOptions
{
    // 固定执行的视觉回正次数；每一步都重新取新点云、匹配并运动。
    int correction_steps{2};
    int discard_new_frames{3};
    double move_speed{15.0};
    double move_acc{15.0};
    double stable_seconds{0.5};
    double stable_timeout_seconds{12.0};
    double joint_arrival_tolerance_deg{0.5};
    // 以下门限只拒绝坏匹配/危险大跳变，不用于判断是否“收敛”。
    int minimum_inliers{500};
    double minimum_overlap{0.03};
    double maximum_rmse_mm{60.0};
    double maximum_correction_mm{500.0};
    double maximum_correction_deg{60.0};
    // 最后一轮 Ref 匹配仍超过此平移残差，Ref 定位失败，不得执行 Work。
    double final_maximum_residual_mm{100.0};
    // 第一轮回正前，沿示教 Ref 的基座 Y 轴等间距搜索四个初值。
    double initial_search_y_start_mm{80.0};
    double initial_search_y_end_mm{-20.0};
    int initial_search_samples{4};
    IcpWallFilterOptions wall_filter;
    int wall_estimation_attempts{3};
    std::size_t maximum_source_points{80000};
    int robot_timeout_seconds{30};
};

struct IcpRobotCallbacks
{
    std::function<c2::Response()> get_joint_position;
    std::function<c2::Response()> get_cart_position;
    std::function<c2::Response(const c2::Point &, double, double, int)> mov_j;
    std::function<c2::Response(const c2::CPos &, const c2::APos &, int)>
        cpos_to_apos;
};

struct IcpIterationResult
{
    int iteration{};
    double translation_mm{};
    double rotation_deg{};
    double rmse_mm{};
    int inliers{};
    double overlap{};
};

struct IcpExecuteResult
{
    bool corrections_completed{false};
    std::vector<IcpIterationResult> iterations;
    std::vector<int> completed_commands;
};

// 独立 C++ ICP 执行端：一次启动相机，完成固定两步 Ref 回正后执行相对 Work。
class IcpExecuteTask
{
public:
    IcpExecuteTask(std::filesystem::path record_root,
                   std::filesystem::path handeye_json);

    bool loadPlan(const std::string &mapid,
                  const std::string &poseid,
                  IcpExecutionPlan &plan,
                  std::string &error) const;

    bool execute(const std::string &mapid,
                 const std::string &poseid,
                 const IcpRobotCallbacks &robot,
                 IcpExecuteResult &result,
                 std::string &error,
                 const IcpExecuteOptions &options = {}) const;

private:
    std::filesystem::path record_root_;
    std::filesystem::path handeye_json_;
};
