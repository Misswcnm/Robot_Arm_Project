#pragma once

#include "D455Camera.h"
#include "Define.h"
#include "TagTask.h"

#include <array>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

struct TagReferenceRecord
{
    int tag_id{};
    std::array<double, 6> angle{};
    std::array<double, 6> pose{};
};

struct TagWorkRecord
{
    int command{};
    TagTask::Transform trans{};  // T_tag_flange_work，按行展开。
};

struct TagExecutionPlan
{
    TagReferenceRecord reference;
    std::vector<TagWorkRecord> works;
};

struct TagExecuteOptions
{
    double move_speed{15.0};
    double move_acc{15.0};
    double stable_seconds{0.5};
    double stable_timeout_seconds{12.0};
    double joint_arrival_tolerance_deg{0.5};
    // 保留旧字段以兼容既有调用；执行端固定使用四个 Y 搜索位置。
    double search_lateral_step_mm{20.0};
    int robot_timeout_seconds{30};
};

struct TagRobotCallbacks
{
    std::function<c2::Response()> get_joint_position;
    std::function<c2::Response()> get_cart_position;
    std::function<c2::Response(const c2::Point &, double, double, int)> mov_j;
    std::function<c2::Response(const c2::CPos &, const c2::APos &, int)>
        cpos_to_apos;
};

struct TagExecuteResult
{
    int tag_id{};
    TagTask::Transform base_tag{};
    double search_lateral_offset_mm{};
    std::vector<int> completed_commands;
};

// 独立 C++ AprilTag 执行端：Ref 观察位 -> 新帧定位 -> Ref-relative Work。
class TagExecuteTask
{
public:
    explicit TagExecuteTask(TagTaskConfig config);

    bool loadPlan(const std::string &mapid,
                  const std::string &poseid,
                  TagExecutionPlan &plan,
                  std::string &error) const;

    bool execute(const std::string &mapid,
                 const std::string &poseid,
                 const TagRobotCallbacks &robot,
                 TagExecuteResult &result,
                 std::string &error,
                 const TagExecuteOptions &options = {}) const;

private:
    std::filesystem::path record_root_;
    TagTask locator_;
};
