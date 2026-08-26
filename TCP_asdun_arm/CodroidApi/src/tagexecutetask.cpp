#include "TagExecuteTask.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <optional>
#include <thread>
#include <utility>

namespace fs = std::filesystem;

namespace
{
using Clock = std::chrono::steady_clock;

bool responseArray(const c2::Response &response, std::size_t size,
                   const std::string &label, std::vector<double> &values,
                   std::string &error)
{
    if (response.code != c2::ResponseCode::OK) {
        error = label + "失败: " + response.msg;
        return false;
    }
    if (!response.data.is_array() || response.data.size() < size) {
        error = label + "返回数据不足";
        return false;
    }
    values.clear();
    values.reserve(size);
    try {
        for (std::size_t index = 0; index < size; ++index) {
            const double value = response.data[index].get<double>();
            if (!std::isfinite(value)) {
                error = label + "返回非有限数值";
                return false;
            }
            values.push_back(value);
        }
    } catch (const std::exception &exception) {
        error = label + "返回解析失败: " + exception.what();
        return false;
    }
    return true;
}

bool readSix(const json &value, const std::array<const char *, 6> &keys,
             std::array<double, 6> &output, std::string &error)
{
    if (!value.is_object()) {
        error = "angle/pose 必须是 JSON 对象";
        return false;
    }
    try {
        for (std::size_t index = 0; index < keys.size(); ++index) {
            output[index] = value.at(keys[index]).get<double>();
            if (!std::isfinite(output[index])) {
                error = std::string(keys[index]) + " 不是有限数值";
                return false;
            }
        }
    } catch (const std::exception &exception) {
        error = "angle/pose 字段解析失败: " + std::string(exception.what());
        return false;
    }
    return true;
}

bool readTransform(const json &value, TagTask::Transform &output,
                   std::string &error)
{
    if (!value.is_array() || value.size() != 4) {
        error = "trans 必须是 4x4 矩阵";
        return false;
    }
    try {
        for (std::size_t row = 0; row < 4; ++row) {
            if (!value[row].is_array() || value[row].size() != 4) {
                error = "trans 必须是 4x4 矩阵";
                return false;
            }
            for (std::size_t column = 0; column < 4; ++column) {
                const double entry = value[row][column].get<double>();
                if (!std::isfinite(entry)) {
                    error = "trans 包含非有限数值";
                    return false;
                }
                output[row * 4 + column] = entry;
            }
        }
    } catch (const std::exception &exception) {
        error = "trans 解析失败: " + std::string(exception.what());
        return false;
    }
    if (std::abs(output[12]) > 1e-9 || std::abs(output[13]) > 1e-9 ||
        std::abs(output[14]) > 1e-9 || std::abs(output[15] - 1.0) > 1e-9) {
        error = "trans 最后一行不是 [0,0,0,1]";
        return false;
    }
    return true;
}

bool commandObject(const fs::path &path, int expected_command,
                   json &point, std::string &error)
{
    try {
        std::ifstream input(path);
        if (!input.is_open()) {
            error = "找不到点位文件: " + path.string();
            return false;
        }
        json points;
        input >> points;
        if (!points.is_array()) {
            error = "点位文件不是 JSON 数组: " + path.string();
            return false;
        }
        for (const json &candidate : points) {
            if (candidate.value("command", -1) == expected_command) {
                point = candidate;
                return true;
            }
        }
        error = "点位文件中没有 command=" +
                std::to_string(expected_command) + ": " + path.string();
    } catch (const std::exception &exception) {
        error = "点位 JSON 解析失败: " + std::string(exception.what());
    }
    return false;
}

Eigen::Isometry3d eigenTransform(const TagTask::Transform &flat)
{
    Eigen::Matrix4d matrix;
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            matrix(row, column) =
                flat[static_cast<std::size_t>(row * 4 + column)];
        }
    }
    return Eigen::Isometry3d(matrix);
}

c2::CPos targetCPos(const TagTask::Transform &base_tag,
                    const TagTask::Transform &tag_flange)
{
    const Eigen::Isometry3d target =
        eigenTransform(base_tag) * eigenTransform(tag_flange);
    const Eigen::Vector3d zyx = target.linear().eulerAngles(2, 1, 0);
    const double degrees = 180.0 / std::acos(-1.0);

    c2::CPos output;
    output.x = target.translation().x();
    output.y = target.translation().y();
    output.z = target.translation().z();
    // 保存端使用 Rz(c)*Ry(b)*Rx(a)，这里按相同约定反解为 a,b,c。
    output.a = zyx[2] * degrees;
    output.b = zyx[1] * degrees;
    output.c = zyx[0] * degrees;
    return output;
}

bool waitJointArrival(const TagRobotCallbacks &robot,
                      const std::array<double, 6> &target,
                      const TagExecuteOptions &options,
                      std::string &error)
{
    const auto deadline = Clock::now() +
        std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(options.stable_timeout_seconds));
    std::optional<Clock::time_point> arrived_since;
    std::string last_error;

    while (Clock::now() < deadline) {
        std::vector<double> raw;
        if (!responseArray(robot.get_joint_position(), 6, "读取当前关节",
                           raw, last_error)) {
            arrived_since.reset();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        double maximum_residual = 0.0;
        for (std::size_t index = 0; index < target.size(); ++index) {
            maximum_residual = std::max(
                maximum_residual,
                std::abs(std::remainder(raw[index] - target[index], 360.0)));
        }
        const auto now = Clock::now();
        if (maximum_residual <= options.joint_arrival_tolerance_deg) {
            if (!arrived_since) {
                arrived_since = now;
            }
            if (std::chrono::duration<double>(now - *arrived_since).count() >=
                options.stable_seconds) {
                std::cout << "[APRILTAG_EXEC] 到点并连续稳定 "
                          << options.stable_seconds
                          << "s, max_joint_residual=" << maximum_residual
                          << "deg" << std::endl;
                return true;
            }
        } else {
            arrived_since.reset();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    error = "机械臂未到达目标关节或未连续稳定 " +
            std::to_string(options.stable_seconds) + " 秒";
    if (!last_error.empty()) {
        error += "; 最后错误: " + last_error;
    }
    return false;
}

bool moveJoint(const TagRobotCallbacks &robot,
               const std::array<double, 6> &joints,
               const TagExecuteOptions &options,
               const std::string &label, std::string &error)
{
    c2::Point point;
    point.type = c2::PointType::Joint;
    for (std::size_t index = 0; index < joints.size(); ++index) {
        point.apos.jntPos[index] = joints[index];
    }
    const c2::Response response = robot.mov_j(
        point, options.move_speed, options.move_acc,
        options.robot_timeout_seconds);
    if (response.code != c2::ResponseCode::OK) {
        error = label + " movJ(APos) 失败: " + response.msg;
        return false;
    }
    std::cout << "[APRILTAG_EXEC] " << label << " 已发送" << std::endl;
    return waitJointArrival(robot, joints, options, error);
}

bool moveCartesianPose(const TagRobotCallbacks &robot,
                       const std::array<double, 6> &pose,
                       const TagExecuteOptions &options,
                       const std::string &label,
                       std::string &error)
{
    std::vector<double> current_joints;
    if (!responseArray(robot.get_joint_position(), 6, "读取搜索 IK 参考关节",
                       current_joints, error)) {
        return false;
    }
    c2::APos reference_joints;
    for (std::size_t index = 0; index < 6; ++index) {
        reference_joints.jntPos[index] = current_joints[index];
    }
    c2::CPos target;
    target.x = pose[0];
    target.y = pose[1];
    target.z = pose[2];
    target.a = pose[3];
    target.b = pose[4];
    target.c = pose[5];
    const c2::Response solved = robot.cpos_to_apos(
        target, reference_joints, options.robot_timeout_seconds);
    std::vector<double> solved_joints;
    if (!responseArray(solved, 6, label + " cposToAPos",
                       solved_joints, error)) {
        return false;
    }
    std::array<double, 6> joints{};
    std::copy_n(solved_joints.begin(), joints.size(), joints.begin());
    std::cout << "[APRILTAG_EXEC] " << label
              << " target_xyzabc_mm_deg=["
              << target.x << "," << target.y << "," << target.z << ","
              << target.a << "," << target.b << "," << target.c << "]"
              << std::endl;
    return moveJoint(robot, joints, options, label, error);
}

bool validCallbacks(const TagRobotCallbacks &robot, std::string &error)
{
    if (!robot.get_joint_position || !robot.get_cart_position ||
        !robot.mov_j || !robot.cpos_to_apos) {
        error = "AprilTag 执行端机械臂回调不完整";
        return false;
    }
    return true;
}
}  // namespace

TagExecuteTask::TagExecuteTask(TagTaskConfig config)
    : record_root_(config.record_root), locator_(std::move(config))
{
}

bool TagExecuteTask::loadPlan(const std::string &mapid,
                              const std::string &poseid,
                              TagExecutionPlan &plan,
                              std::string &error) const
{
    error.clear();
    plan = TagExecutionPlan{};
    if (mapid.empty() || poseid.empty()) {
        error = "mapid 和 poseid 不能为空";
        return false;
    }
    const fs::path station = record_root_ / mapid / poseid;
    try {
        json reference;
        if (!commandObject(station / "1.json", 1, reference, error)) {
            return false;
        }
        if (reference.value("point_type", -1) != 0) {
            error = "command=1 不是 AprilTag Ref(point_type=0)";
            return false;
        }
        plan.reference.tag_id = reference.at("tag_id").get<int>();
        if (plan.reference.tag_id < 0 ||
            !readSix(reference.at("angle"),
                     {"jntpos1", "jntpos2", "jntpos3",
                      "jntpos4", "jntpos5", "jntpos6"},
                     plan.reference.angle, error) ||
            !readSix(reference.at("pose"), {"x", "y", "z", "a", "b", "c"},
                     plan.reference.pose, error)) {
            if (error.empty()) {
                error = "AprilTag Ref 的 tag_id 无效";
            }
            return false;
        }

        std::vector<std::pair<int, fs::path>> command_files;
        if (!fs::is_directory(station)) {
            error = "点位目录不存在: " + station.string();
            return false;
        }
        for (const fs::directory_entry &entry : fs::directory_iterator(station)) {
            if (!entry.is_regular_file() || entry.path().extension() != ".json") {
                continue;
            }
            const std::string stem = entry.path().stem().string();
            std::size_t consumed = 0;
            int command = 0;
            try {
                command = std::stoi(stem, &consumed);
            } catch (...) {
                continue;
            }
            if (consumed == stem.size() && command > 1) {
                command_files.emplace_back(command, entry.path());
            }
        }
        std::sort(command_files.begin(), command_files.end(),
                  [](const auto &left, const auto &right) {
                      return left.first < right.first;
                  });
        for (const auto &[command, path] : command_files) {
            json point;
            if (!commandObject(path, command, point, error)) {
                return false;
            }
            if (point.value("point_type", -1) != 0) {
                error = "command=" + std::to_string(command) +
                        " 与 Ref 类型不一致";
                return false;
            }
            TagWorkRecord work;
            work.command = command;
            if (!point.contains("trans") ||
                !readTransform(point["trans"], work.trans, error)) {
                error = "command=" + std::to_string(command) + ": " + error;
                return false;
            }
            plan.works.push_back(work);
        }
        if (plan.works.empty()) {
            error = "AprilTag 点没有 command>1 的 Work";
            return false;
        }
        return true;
    } catch (const std::exception &exception) {
        error = "AprilTag 执行计划读取失败: " +
                std::string(exception.what());
        return false;
    }
}

bool TagExecuteTask::execute(const std::string &mapid,
                             const std::string &poseid,
                             const TagRobotCallbacks &robot,
                             TagExecuteResult &result,
                             std::string &error,
                             const TagExecuteOptions &options) const
{
    error.clear();
    result = TagExecuteResult{};
    if (!validCallbacks(robot, error)) {
        return false;
    }
    TagExecutionPlan plan;
    if (!loadPlan(mapid, poseid, plan, error)) {
        return false;
    }

    // 旧 executor 的顺序：先回到示教 Ref 观察关节位，再获取当前 Tag。
    if (!moveJoint(robot, plan.reference.angle, options,
                   "Ref 观察位", error)) {
        return false;
    }

    D455Camera camera;
    if (!camera.startColorOnly(error)) {
        return false;
    }
    result.tag_id = plan.reference.tag_id;
    struct SearchOffset {
        double lateral_mm;
        const char *label;
    };
    // 与 ICP 相同：从 Y+80 到 Y-20 等间距搜索四个位置。
    const std::array<SearchOffset, 4> search_offsets{{
        {80.0, "Y+80"},
        {46.6666666667, "Y+46.67"},
        {13.3333333333, "Y+13.33"},
        {-20.0, "Y-20"},
    }};

    bool located = false;
    std::string last_not_found;
    for (std::size_t search_index = 0;
        search_index < search_offsets.size(); ++search_index) {
        const SearchOffset &offset = search_offsets[search_index];
        std::array<double, 6> search_pose{};
        if (!locator_.baseLateralObservationPose(
                plan.reference.pose,
                offset.lateral_mm,
                search_pose, error) ||
            !moveCartesianPose(
                robot, search_pose, options,
                "Tag 末端 Y 搜索 " + std::string(offset.label) +
                    " base_y_offset=" +
                    std::to_string(offset.lateral_mm) + "mm",
                error)) {
            return false;
        }

        // 每个搜索位置均已完成到点稳定；清空缓存后只使用下一张新帧。
        D455Frame frame;
        if (!camera.captureColorOne(frame, error)) {
            return false;
        }
        std::cout << "[APRILTAG_EXEC] 搜索 " << offset.label
                  << " base_y_offset=" << offset.lateral_mm
                  << "mm 新彩色帧 timestamp_ms="
                  << frame.color_timestamp_ms << std::endl;

        std::vector<double> current_pose;
        if (!responseArray(robot.get_cart_position(), 6,
                           "读取定位时法兰位姿", current_pose, error)) {
            return false;
        }
        std::array<double, 6> flange_pose{};
        std::copy_n(current_pose.begin(), flange_pose.size(),
                    flange_pose.begin());
        if (locator_.locateBaseTag(
                frame, plan.reference.tag_id, flange_pose,
                result.base_tag, error)) {
            result.search_lateral_offset_mm = offset.lateral_mm;
            located = true;
            std::cout << "[APRILTAG_EXEC] 新彩色帧定位完成, position="
                      << offset.label << " base_y_offset="
                      << offset.lateral_mm << "mm timestamp_ms="
                      << frame.color_timestamp_ms << std::endl;
            break;
        }
        if (error.rfind("当前彩色帧未检测到 Tag ID ", 0) != 0) {
            return false;
        }
        last_not_found = error;
        std::cout << "[APRILTAG_EXEC] " << error
                  << ", 继续局部搜索" << std::endl;
    }
    camera.stop();
    if (!located) {
        error = last_not_found +
                "; 已沿末端基座 Y 搜索 +80/+46.67/+13.33/-20mm";
        return false;
    }

    for (const TagWorkRecord &work : plan.works) {
        const c2::CPos target = targetCPos(result.base_tag, work.trans);
        std::vector<double> current_joints;
        if (!responseArray(robot.get_joint_position(), 6, "读取 IK 参考关节",
                           current_joints, error)) {
            return false;
        }
        c2::APos reference_joints;
        for (std::size_t index = 0; index < 6; ++index) {
            reference_joints.jntPos[index] = current_joints[index];
        }
        const c2::Response solved = robot.cpos_to_apos(
            target, reference_joints, options.robot_timeout_seconds);
        std::vector<double> target_joints;
        if (!responseArray(solved, 6,
                           "command=" + std::to_string(work.command) +
                               " cposToAPos",
                           target_joints, error)) {
            return false;
        }
        std::array<double, 6> joints{};
        std::copy_n(target_joints.begin(), joints.size(), joints.begin());
        std::cout << "[APRILTAG_EXEC] command=" << work.command
                  << " target_xyzabc_mm_deg=["
                  << target.x << "," << target.y << "," << target.z << ","
                  << target.a << "," << target.b << "," << target.c << "]"
                  << std::endl;
        if (!moveJoint(robot, joints, options,
                       "Work command=" + std::to_string(work.command),
                       error)) {
            return false;
        }
        result.completed_commands.push_back(work.command);
    }
    return true;
}
