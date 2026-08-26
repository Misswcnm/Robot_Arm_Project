#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <filesystem>
#include <optional>
#include <thread>
#include <utility>
#include "D455Camera.h"
#include "IcpExecuteTask.h"
#include "IcpTask.h"
#include "TagExecuteTask.h"
#include "TagTask.h"
#include "Define.h"  // c2::Response / ResponseCode / UserCommand（轻量，不拉 WebSocket）

namespace fs = std::filesystem;

fs::path tagTaskConfigPath()
{
    const std::array<fs::path, 3> candidates{
        fs::path("config/tagtask.json"),
        fs::path("../config/tagtask.json"),
        fs::path("TCP_asdun_arm/config/tagtask.json"),
    };
    for (const fs::path &candidate : candidates) {
        if (fs::exists(candidate)) {
            return candidate;
        }
    }
    return candidates.front();
}

// =====================================================================
// 模拟机械臂 API —— 搬自 mockrobot.cpp 的 MockEstunController，
// 下沉到 CodroidApi 语义（sendUserCommand / getJointPosition / getCartPosition）。
// 返回类型与真机 c2::CodroidApi 一致（c2::Response），后续可直接换真机。
// =====================================================================
class MockCodroidApi
{
public:
    // 开始拖拽：真机对应 UserCommand::ToReady —— 进入 Ready/手动示教模式。
    c2::Response startDrag()
    {
        std::cout << "[MOCK] startDrag(): 进入 Ready/手动示教模式，允许人工拖拽"
                  << std::endl;
        return sendUserCommand(c2::UserCommand::ToReady);
    }

    c2::Response sendUserCommand(c2::UserCommand command)
    {
        std::cout << "[MOCK] sendUserCommand(" << static_cast<int>(command)
                  << ") 模拟成功" << std::endl;
        c2::Response response;
        response.code = c2::ResponseCode::OK;
        return response;
    }

    c2::Response getJointPosition()
    {
        std::cout << "[MOCK] getJointPosition() 被调用" << std::endl;
        c2::Response response;
        response.code = c2::ResponseCode::OK;
        response.data = {
            3.689346, -3.010254, 114.174728,
            32.767410, 80.538712, -13.923798,
        };
        return response;
    }

    c2::Response getCartPosition()
    {
        std::cout << "[MOCK] getCartPosition() 被调用" << std::endl;
        c2::Response response;
        response.code = c2::ResponseCode::OK;
        response.data = {
            463.596139, 250.136185, 359.053922,
            -166.388393, 6.193869, -70.683480,
        };
        return response;
    }
};

// 生产接入点：RobotApi 换成 c2::CodroidApi 即可；视觉执行类不依赖 ROS。
template <typename RobotApi>
TagRobotCallbacks tagRobotCallbacks(RobotApi &api)
{
    TagRobotCallbacks callbacks;
    callbacks.get_joint_position = [&api]() {
        return api.getJointPosition();
    };
    callbacks.get_cart_position = [&api]() {
        return api.getCartPosition();
    };
    callbacks.mov_j = [&api](const c2::Point &point, double speed,
                             double acc, int timeout) {
        return api.movJ(point, speed, acc, timeout);
    };
    callbacks.cpos_to_apos =
        [&api](const c2::CPos &target, const c2::APos &reference,
               int timeout) {
            return api.cposToAPos(target, reference, nullptr, nullptr, timeout);
        };
    return callbacks;
}

template <typename RobotApi>
bool tagExecute(RobotApi &api, const std::string &mapid,
                const std::string &poseid)
{
    TagTaskConfig config;
    std::string error;
    if (!loadTagTaskConfig(tagTaskConfigPath(), config, error)) {
        std::cerr << "[APRILTAG_EXEC] " << error << std::endl;
        return false;
    }
    TagExecuteTask task(std::move(config));
    TagExecuteResult result;
    if (!task.execute(mapid, poseid, tagRobotCallbacks(api), result, error)) {
        std::cerr << "[APRILTAG_EXEC] " << error << std::endl;
        return false;
    }
    std::cout << "[APRILTAG_EXEC] 完成 Tag ID " << result.tag_id
              << ", Work 数=" << result.completed_commands.size()
              << std::endl;
    return true;
}

template <typename RobotApi>
IcpRobotCallbacks icpRobotCallbacks(RobotApi &api)
{
    IcpRobotCallbacks callbacks;
    callbacks.get_joint_position = [&api]() {
        return api.getJointPosition();
    };
    callbacks.get_cart_position = [&api]() {
        return api.getCartPosition();
    };
    callbacks.mov_j = [&api](const c2::Point &point, double speed,
                             double acc, int timeout) {
        return api.movJ(point, speed, acc, timeout);
    };
    callbacks.cpos_to_apos =
        [&api](const c2::CPos &target, const c2::APos &reference,
               int timeout) {
            return api.cposToAPos(target, reference, nullptr, nullptr, timeout);
        };
    return callbacks;
}

template <typename RobotApi>
bool icpExecute(RobotApi &api, const std::string &mapid,
                const std::string &poseid)
{
    TagTaskConfig shared_config;
    std::string error;
    if (!loadTagTaskConfig(tagTaskConfigPath(), shared_config, error)) {
        std::cerr << "[ICP_EXEC] " << error << std::endl;
        return false;
    }
    IcpExecuteTask task(
        shared_config.record_root, shared_config.handeye_json);
    IcpExecuteResult result;
    IcpExecuteOptions options;
    options.correction_steps = 2;
    options.discard_new_frames = 3;
    if (!task.execute(mapid, poseid, icpRobotCallbacks(api), result,
                      error, options)) {
        std::cerr << "[ICP_EXEC] " << error << std::endl;
        return false;
    }
    std::cout << "[ICP_EXEC] 两步回正完成, steps=" << result.iterations.size()
              << ", Work 数=" << result.completed_commands.size()
              << std::endl;
    return true;
}

// =====================================================================
// 示教状态机（模板化，便于测试用 MockCodroidApi、生产用 c2::CodroidApi）
// =====================================================================


// 开始示教  1. 拖拽 2. 创建文件夹
template <typename RobotApi>
bool startTeaching(RobotApi &api, const std::string &mapid,
                   const std::string &poseid)
{
    // 1. 开始拖拽
    const c2::Response drag = api.startDrag();
    if (drag.code != c2::ResponseCode::OK) {
        std::cerr << "[TYPE1] 开始拖拽失败，未开始示教: " << drag.msg << std::endl;
        return false;
    }

    // 2. 创建 mapid/poseid 文件夹
    try {
        const fs::path dir_path = fs::path(".") / mapid / poseid;
        if (fs::create_directories(dir_path)) {
            std::cout << "成功创建文件夹: " << dir_path << std::endl;
        } else {
            std::cout << "文件夹已存在，无需重复创建: " << dir_path << std::endl;
        }
        return true;
    } catch (const fs::filesystem_error &e) {
        std::cerr << "创建文件夹失败: " << e.what() << std::endl;
        return false;
    }
}



// 普通点示教--保存 A(关节角) + C(笛卡尔位姿)，最前面用 command 字段标识
template <typename RobotApi>
bool normalRecord(RobotApi &api, const std::string &mapid,
                  const std::string &poseid, int command,
                  std::optional<int> point_type = std::nullopt)
{
    const c2::Response joints = api.getJointPosition();
    const c2::Response pose = api.getCartPosition();
    if (joints.code != c2::ResponseCode::OK || pose.code != c2::ResponseCode::OK) {
        std::cerr << "[NORMAL] 读取位姿失败" << std::endl;
        return false;
    }
    if (joints.data.size() < 6 || pose.data.size() < 6) {
        std::cerr << "[NORMAL] 位姿数据不完整" << std::endl;
        return false;
    }

    json point;
    point["command"] = command;
    // 普通点在报文中省略 point_type；持久化时显式写 null，读取端无需猜测。
    point["point_type"] = point_type ? json(*point_type) : json(nullptr);
    point["angle"] = {
        {"jntpos1", joints.data[0]},
        {"jntpos2", joints.data[1]},
        {"jntpos3", joints.data[2]},
        {"jntpos4", joints.data[3]},
        {"jntpos5", joints.data[4]},
        {"jntpos6", joints.data[5]},
    };
    point["pose"] = {
        {"x", pose.data[0]},
        {"y", pose.data[1]},
        {"z", pose.data[2]},
        {"a", pose.data[3]},
        {"b", pose.data[4]},
        {"c", pose.data[5]},
    };

    try {
        const fs::path dir_path = fs::path(".") / mapid / poseid;
        fs::create_directories(dir_path);
        const fs::path json_path = dir_path / (std::to_string(command) + ".json");

        std::ofstream ofs(json_path);
        if (!ofs.is_open()) {
            std::cerr << "[NORMAL] 无法打开 JSON 文件: " << json_path << std::endl;
            return false;
        }

        ofs << json::array({point}).dump(4);
        ofs.close();

        std::cout << "[NORMAL] 点位保存成功: " << json_path << std::endl;
        return true;
    } catch (const std::exception &e) {
        std::cerr << "[NORMAL] 异常: " << e.what() << std::endl;
        return false;
    }
}

template <typename RobotApi>
bool waitRobotStable(RobotApi &api, double stable_seconds = 2.0,
                     double timeout_seconds = 8.0)
{
    using Clock = std::chrono::steady_clock;
    const auto deadline = Clock::now() +
        std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(timeout_seconds));
    std::optional<std::array<double, 6>> previous;
    std::optional<Clock::time_point> stable_since;

    while (Clock::now() < deadline) {
        const c2::Response response = api.getCartPosition();
        const auto now = Clock::now();
        if (response.code != c2::ResponseCode::OK ||
            !response.data.is_array() || response.data.size() < 6) {
            previous.reset();
            stable_since.reset();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        std::array<double, 6> current{};
        for (std::size_t index = 0; index < current.size(); ++index) {
            current[index] = response.data[index].get<double>();
        }

        if (previous) {
            double position_squared = 0.0;
            double rotation_squared = 0.0;
            for (std::size_t index = 0; index < 3; ++index) {
                const double delta = current[index] - (*previous)[index];
                position_squared += delta * delta;
            }
            for (std::size_t index = 3; index < 6; ++index) {
                const double delta =
                    std::remainder(current[index] - (*previous)[index], 360.0);
                rotation_squared += delta * delta;
            }

            const double position_delta_mm = std::sqrt(position_squared);
            const double rotation_delta_deg = std::sqrt(rotation_squared);
            if (position_delta_mm < 0.3 && rotation_delta_deg < 0.1) {
                if (!stable_since) {
                    stable_since = now;
                }
                if (std::chrono::duration<double>(now - *stable_since).count() >=
                    stable_seconds) {
                    std::cout << "[CAMERA] 机械臂已连续停稳 "
                              << stable_seconds << " 秒" << std::endl;
                    return true;
                }
            } else {
                stable_since.reset();
            }
        }
        previous = current;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cerr << "[CAMERA] 机械臂在 " << timeout_seconds
              << " 秒内没有连续停稳 " << stable_seconds << " 秒" << std::endl;
    return false;
}

template <typename RobotApi>
bool tagRecord(RobotApi &api, const std::string &mapid,
               const std::string &poseid, int command, int tag_id = 0);

template <typename RobotApi>
bool icpRecord(RobotApi &api, const std::string &mapid,
               const std::string &poseid, int command);



struct TeachingSession
{
    std::string mapid;
    std::string poseid;
    std::optional<int> point_type;
    int tag_id{0};
};

std::optional<TeachingSession> activeTeaching;

template <typename RobotApi>
bool handleType1(RobotApi &api, const json &request)
{
    const std::string mapid = request.value("mapid", "");
    const std::string poseid = request.value("poseid", "");
    if (request.contains("command")) {
        std::cerr << "[TYPE1] 开始示教报文不能包含 command" << std::endl;
        return false;
    }
    std::optional<int> point_type;
    if (request.contains("point_type")) {
        const int value = request.value("point_type", -1);
        if (value != 0 && value != 1) {
            std::cerr << "[TYPE1] point_type 只能为 0/1，普通点请省略"
                      << std::endl;
            return false;
        }
        point_type = value;
    }
    if (!startTeaching(api, mapid, poseid)) {
        return false;
    }
    activeTeaching = TeachingSession{
        mapid, poseid, point_type, request.value("tag_id", 0)};
    return true;
}

template <typename RobotApi>
bool handleTeachingCommand(RobotApi &api, const json &request)
{
    const std::string mapid = request.value("mapid", "");
    const std::string poseid = request.value("poseid", "");
    if (!request.contains("command") || request.contains("point_type")) {
        std::cerr << "[COMMAND] 记录报文只允许 mapid/poseid/command，"
                     "point_type 只属于 type1" << std::endl;
        return false;
    }
    if (!activeTeaching || activeTeaching->mapid != mapid ||
        activeTeaching->poseid != poseid) {
        std::cerr << "[COMMAND] 请先发送匹配站点的 type1 开始示教"
                  << std::endl;
        return false;
    }
    const int command = request.value("command", 0);
    if (command < 1) {
        std::cerr << "[COMMAND] command 必须大于等于 1" << std::endl;
        return false;
    }
    if (!activeTeaching->point_type) {
        return normalRecord(api, mapid, poseid, command, std::nullopt);
    }
    return *activeTeaching->point_type == 0
        ? tagRecord(api, mapid, poseid, command, activeTeaching->tag_id)
        : icpRecord(api, mapid, poseid, command);
}


template <typename RobotApi>
bool tagRecord(RobotApi &api, const std::string &mapid,
               const std::string &poseid, int command, int tag_id)
{
    if (command < 1) {
        std::cerr << "[APRILTAG] command 必须大于等于 1" << std::endl;
        return false;
    }

    if (tag_id < 0) {
        std::cerr << "[APRILTAG] tag_id 必须大于等于 0" << std::endl;
        return false;
    }
    std::string error;
    TagTaskConfig config;
    if (!loadTagTaskConfig(tagTaskConfigPath(), config, error)) {
        std::cerr << "[APRILTAG] " << error << std::endl;
        return false;
    }
    // 每条报文使用独立 detector，不在请求之间保留隐藏状态。
    TagTask task(std::move(config));
    if (command == 1) {
        D455Camera camera;
        if (!camera.startColorOnly(error)) {
            std::cerr << "[APRILTAG] " << error << std::endl;
            return false;
        }
        // 相机先预热，机械臂连续停稳 2 秒后，丢弃缓存再取新彩色帧。
        if (!waitRobotStable(api, 2.0, 8.0)) {
            return false;
        }
        D455Frame frame;
        if (!camera.captureColorOne(frame, error)) {
            std::cerr << "[APRILTAG] " << error << std::endl;
            return false;
        }
        camera.stop();
        std::cout << "[APRILTAG] 新彩色帧获取完成"
                  << ", timestamp_ms=" << frame.color_timestamp_ms
                  << std::endl;
        if (!normalRecord(api, mapid, poseid, command, 0)) {
            return false;
        }
        if (!task.recordReference(mapid, poseid, frame, tag_id, error)) {
            std::cerr << "[APRILTAG] " << error << std::endl;
            return false;
        }
        std::cout << "[APRILTAG] Ref 保存完成: tag_id=" << tag_id
                  << ", T_base_tag 单位=mm" << std::endl;
        return true;
    }

    if (!waitRobotStable(api, 2.0, 8.0)) {
        return false;
    }
    if (!normalRecord(api, mapid, poseid, command, 0)) {
        return false;
    }
    if (!task.recordWork(mapid, poseid, command, error)) {
        std::cerr << "[APRILTAG] " << error << std::endl;
        return false;
    }
    std::cout << "[APRILTAG] Work command=" << command
              << " 的 T_tag_ref_flange_work 保存完成" << std::endl;
    return true;
}

template <typename RobotApi>
bool icpRecord(RobotApi &api, const std::string &mapid,
                  const std::string &poseid, int command)
{
    if (command < 1) {
        std::cerr << "[ICP] command 必须大于等于 1" << std::endl;
        return false;
    }
    if (!waitRobotStable(api, 2.0, 8.0)) {
        return false;
    }

    IcpTask task;
    std::string error;
    if (command == 1) {
        D455Camera camera;
        if (!camera.start(error)) {
            std::cerr << "[ICP] " << error << std::endl;
            return false;
        }

        std::vector<std::vector<D455PointXYZ>> frames;
        frames.reserve(5);
        for (int index = 0; index < 5; ++index) {
            D455Frame frame;
            if (camera.captureOne(frame, error)) {
                std::cout << "[ICP] Ref " << (index + 1) << "/5: "
                          << frame.point_cloud_color_m.size() << " 点"
                          << std::endl;
                frames.push_back(std::move(frame.point_cloud_color_m));
            } else {
                std::cerr << "[ICP] 第 " << (index + 1)
                          << " 帧失败: " << error << std::endl;
            }
        }
        camera.stop();

        // Ref 的 angle + pose 沿用普通点格式，IcpTask 只追加模板数据。
        if (!normalRecord(api, mapid, poseid, command, 1)) {
            return false;
        }
        if (!task.recordReference(mapid, poseid, frames, error)) {
            std::cerr << "[ICP] " << error << std::endl;
            return false;
        }
        std::cout << "[ICP] Ref 模板和位姿保存完成" << std::endl;
        return true;
    }

    // Work 同样持久化辅助 angle/pose，IcpTask 再把 trans 写进同一个 JSON。
    if (!normalRecord(api, mapid, poseid, command, 1)) {
        return false;
    }
    if (!task.recordWork(mapid, poseid, command, error)) {
        std::cerr << "[ICP] " << error << std::endl;
        return false;
    }
    std::cout << "[ICP] Work command=" << command
              << " 的 T_ref_work 保存完成" << std::endl;
    return true;
}


// =====================================================================
// 测试入口：模拟一次普通点示教
// =====================================================================
int main(int argc, char **argv)
{
    MockCodroidApi api;
    if (argc == 4 && std::string(argv[1]) == "--icp-plan") {
        TagTaskConfig config;
        std::string error;
        if (!loadTagTaskConfig(tagTaskConfigPath(), config, error)) {
            std::cerr << "[ICP_EXEC] " << error << std::endl;
            return 1;
        }
        IcpExecuteTask task(config.record_root, config.handeye_json);
        IcpExecutionPlan plan;
        if (!task.loadPlan(argv[2], argv[3], plan, error)) {
            std::cerr << "[ICP_EXEC] " << error << std::endl;
            return 1;
        }
        std::cout << "[ICP_EXEC] plan OK: works=" << plan.works.size()
                  << std::endl;
        return 0;
    }
    if (argc == 4 && std::string(argv[1]) == "--tag-plan") {
        TagTaskConfig config;
        std::string error;
        if (!loadTagTaskConfig(tagTaskConfigPath(), config, error)) {
            std::cerr << "[APRILTAG_EXEC] " << error << std::endl;
            return 1;
        }
        TagExecuteTask task(std::move(config));
        TagExecutionPlan plan;
        if (!task.loadPlan(argv[2], argv[3], plan, error)) {
            std::cerr << "[APRILTAG_EXEC] " << error << std::endl;
            return 1;
        }
        std::cout << "[APRILTAG_EXEC] plan OK: tag_id="
                  << plan.reference.tag_id << ", works="
                  << plan.works.size() << std::endl;
        return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--camera-test") {
        std::cout << "[TEST] D455 单帧彩色图 + 对齐点云采集" << std::endl;
        return tagRecord(api, "camera_test", "capture", 1, 0) ? 0 : 1;
    }

    // type1（无 command）→ 开始示教：拖拽 + 建档
    json start = {{"type", "type1"}, {"mapid", "M"}, {"poseid", "P"}};
    handleType1(api, start);

    // mechanical_arm_command（无 point_type）→ 使用 type1 已保存的类型。
    json rec1 = {{"type", "mechanical_arm_command"}, {"mapid", "M"},
                 {"poseid", "P"}, {"command", 1}};
    handleTeachingCommand(api, rec1);

    json rec2 = {{"type", "mechanical_arm_command"}, {"mapid", "M"},
                 {"poseid", "P"}, {"command", 2}};
    handleTeachingCommand(api, rec2);
    return 0;
}
