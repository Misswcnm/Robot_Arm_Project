#include "CodroidApi.h"
#include "D455Camera.h"
#include "IcpExecuteTask.h"
#include "IcpTask.h"
#include "TagExecuteTask.h"
#include "TagTask.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

namespace
{
constexpr double kArrivalToleranceDeg = 0.5;
constexpr double kArrivalStableSeconds = 0.5;
constexpr double kArrivalTimeoutSeconds = 12.0;
constexpr int kServerPort = 8888;
const std::array<double, 6> kResetJoints{
    86.0, 23.0, -112.0, -176.0, -85.0, 0.0};

std::atomic<bool> g_running{true};
std::atomic<int> g_listen_socket{-1};

void stopSignal(int)
{
    g_running = false;
    const int socket_fd = g_listen_socket.exchange(-1);
    if (socket_fd >= 0) {
        ::shutdown(socket_fd, SHUT_RDWR);
        ::close(socket_fd);
    }
}

fs::path tagTaskConfigPath()
{
    const std::array<fs::path, 5> candidates{
        fs::path("config/tagtask.json"),
        fs::path("../config/tagtask.json"),
        fs::path("../../config/tagtask.json"),
        fs::path("TCP_asdun_arm/config/tagtask.json"),
        fs::path("../TCP_asdun_arm/config/tagtask.json"),
    };
    for (const fs::path &candidate : candidates) {
        if (fs::exists(candidate)) {
            return candidate;
        }
    }
    return candidates.front();
}

bool safeIdentifier(const std::string &value)
{
    return !value.empty() && value != "." && value != ".." &&
           value.find('/') == std::string::npos &&
           value.find('\\') == std::string::npos;
}

bool responseArray(const c2::Response &response, std::size_t minimum,
                   std::vector<double> &values, std::string &error)
{
    if (response.code != c2::ResponseCode::OK) {
        error = response.msg.empty() ? "Codroid request failed" : response.msg;
        return false;
    }
    if (!response.data.is_array() || response.data.size() < minimum) {
        error = "Codroid returned incomplete pose data";
        return false;
    }
    try {
        values.clear();
        for (const auto &item : response.data) {
            values.push_back(item.get<double>());
        }
        return true;
    } catch (const std::exception &exception) {
        error = std::string("pose data parse failed: ") + exception.what();
        return false;
    }
}

std::optional<int> pointTypeFrom(const json &request)
{
    for (const char *key : {"point_type", "pointType"}) {
        if (request.contains(key) && request.at(key).is_number_integer()) {
            return request.at(key).get<int>();
        }
    }
    if (request.contains("params") && request.at("params").is_object()) {
        const json &params = request.at("params");
        for (const char *key : {"point_type", "pointType"}) {
            if (params.contains(key) && params.at(key).is_number_integer()) {
                return params.at(key).get<int>();
            }
        }
    }
    return std::nullopt;
}

bool hasPointTypeField(const json &request)
{
    if (request.contains("point_type") || request.contains("pointType")) {
        return true;
    }
    if (request.contains("params") && request.at("params").is_object()) {
        const json &params = request.at("params");
        return params.contains("point_type") || params.contains("pointType");
    }
    return false;
}

int tagIdFrom(const json &request)
{
    if (request.contains("tag_id") && request.at("tag_id").is_number_integer()) {
        return request.at("tag_id").get<int>();
    }
    if (request.contains("params") && request.at("params").is_object()) {
        return request.at("params").value("tag_id", 0);
    }
    return 0;
}

class RobotRuntime
{
public:
    RobotRuntime(std::string host, std::string port)
        : api_(std::move(host), std::move(port)) {}

    c2::Response getJointPosition() { return api_.getJointPosition(); }
    c2::Response getCartPosition() { return api_.getCartPosition(); }
    c2::Response movJ(const c2::Point &point, double speed, double acc,
                      int timeout) { return api_.movJ(point, speed, acc, timeout); }
    c2::Response cposToAPos(const c2::CPos &target,
                            const c2::APos &reference, int timeout)
    {
        return api_.cposToAPos(target, reference, nullptr, nullptr, timeout);
    }

    bool userCommand(c2::UserCommand command, std::string &error)
    {
        const c2::Response response = api_.sendUserCommand(command);
        if (response.code == c2::ResponseCode::OK) return true;
        error = response.msg.empty() ? "ESTUN controller command failed" : response.msg;
        return false;
    }

    bool waitJointArrival(const std::array<double, 6> &target,
                          std::string &error,
                          double tolerance_deg = kArrivalToleranceDeg,
                          double stable_seconds = kArrivalStableSeconds,
                          double timeout_seconds = kArrivalTimeoutSeconds)
    {
        const auto deadline = Clock::now() +
            std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(timeout_seconds));
        std::optional<Clock::time_point> arrived_since;
        std::string last_error;
        while (Clock::now() < deadline) {
            std::vector<double> current;
            if (!responseArray(getJointPosition(), 6, current, last_error)) {
                arrived_since.reset();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            double maximum_residual = 0.0;
            for (std::size_t index = 0; index < target.size(); ++index) {
                maximum_residual = std::max(
                    maximum_residual,
                    std::abs(std::remainder(current[index] - target[index], 360.0)));
            }
            const auto now = Clock::now();
            if (maximum_residual <= tolerance_deg) {
                if (!arrived_since) arrived_since = now;
                if (std::chrono::duration<double>(now - *arrived_since).count() >=
                    stable_seconds) {
                    std::cout << "[ROBOT] arrived and stable " << stable_seconds
                              << "s, max_joint_residual=" << maximum_residual
                              << "deg" << std::endl;
                    return true;
                }
            } else {
                arrived_since.reset();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        error = "arrival timeout: max joint residual must be <= " +
                std::to_string(tolerance_deg) + "deg for " +
                std::to_string(stable_seconds) + "s";
        if (!last_error.empty()) error += "; last error: " + last_error;
        return false;
    }

    bool moveJoint(const std::array<double, 6> &target, double speed,
                   double acc, std::string &error)
    {
        c2::Point point;
        point.type = c2::PointType::Joint;
        for (std::size_t index = 0; index < target.size(); ++index) {
            point.apos.jntPos[index] = target[index];
        }
        const c2::Response response = api_.movJ(point, speed, acc, 30);
        if (response.code != c2::ResponseCode::OK) {
            error = response.msg.empty() ? "movJ(APos) failed" : response.msg;
            return false;
        }
        return waitJointArrival(target, error);
    }

    bool reset(std::string &error)
    {
        if (!moveJoint(kResetJoints, 75.0, 40.0, error)) {
            error = "robot reset failed: " + error;
            return false;
        }
        std::cout << "[ROBOT] reset completed" << std::endl;
        return true;
    }

    bool preflight(std::string &error)
    {
        const c2::Response state = api_.getRobotState(5);
        if (state.code != c2::ResponseCode::OK) {
            error = "cannot connect to ESTUN Codroid: " + state.msg;
            return false;
        }
        return userCommand(c2::UserCommand::SwitchOn, error) &&
               userCommand(c2::UserCommand::ToAuto, error) && reset(error);
    }

    void stopBestEffort()
    {
        const c2::Response response = api_.stopMov(5);
        if (response.code != c2::ResponseCode::OK)
            std::cerr << "[ROBOT] stopMov failed: " << response.msg << std::endl;
    }

private:
    c2::CodroidApi api_;
};

TagRobotCallbacks tagCallbacks(RobotRuntime &robot)
{
    TagRobotCallbacks callbacks;
    callbacks.get_joint_position = [&robot] { return robot.getJointPosition(); };
    callbacks.get_cart_position = [&robot] { return robot.getCartPosition(); };
    callbacks.mov_j = [&robot](const c2::Point &p, double s, double a, int t) {
        return robot.movJ(p, s, a, t);
    };
    callbacks.cpos_to_apos = [&robot](const c2::CPos &p,
                                     const c2::APos &r, int t) {
        return robot.cposToAPos(p, r, t);
    };
    return callbacks;
}

IcpRobotCallbacks icpCallbacks(RobotRuntime &robot)
{
    IcpRobotCallbacks callbacks;
    callbacks.get_joint_position = [&robot] { return robot.getJointPosition(); };
    callbacks.get_cart_position = [&robot] { return robot.getCartPosition(); };
    callbacks.mov_j = [&robot](const c2::Point &p, double s, double a, int t) {
        return robot.movJ(p, s, a, t);
    };
    callbacks.cpos_to_apos = [&robot](const c2::CPos &p,
                                     const c2::APos &r, int t) {
        return robot.cposToAPos(p, r, t);
    };
    return callbacks;
}

struct TeachingSession
{
    std::string mapid;
    std::string poseid;
    std::optional<int> point_type;
    int tag_id{0};
};

class TaskEngine
{
public:
    TaskEngine(RobotRuntime &robot, TagTaskConfig config)
        : robot_(robot), config_(std::move(config)) {}

    json handle(const json &request)
    {
        std::unique_lock<std::mutex> lock(operation_mutex_, std::try_to_lock);
        if (!lock.owns_lock())
            return failure(request, "busy", "another robot/vision action is running; command not queued");
        try {
            return dispatch(request);
        } catch (const std::exception &exception) {
            return failure(request, "execution_error", exception.what());
        }
    }

private:
    json success(const json &request, const std::string &action,
                 const std::string &message, json data = json::object()) const
    {
        return {{"type", request.value("type", "")},
                {"mapid", request.value("mapid", "")},
                {"poseid", request.value("poseid", "")},
                {"action", action}, {"status", "done"},
                {"message", message}, {"data", std::move(data)}};
    }

    json failure(const json &request, const std::string &code,
                 const std::string &message) const
    {
        return {{"type", "error"},
                {"mapid", request.value("mapid", "")},
                {"poseid", request.value("poseid", "")},
                {"status", "failed"}, {"error_code", code},
                {"message", message}};
    }

    bool stationFields(const json &request, std::string &mapid,
                       std::string &poseid, std::string &error) const
    {
        mapid = request.value("mapid", "");
        poseid = request.value("poseid", "");
        if (!safeIdentifier(mapid) || !safeIdentifier(poseid)) {
            error = "invalid mapid/poseid";
            return false;
        }
        return true;
    }

    bool waitRobotStable(double stable_seconds, double timeout_seconds,
                         std::string &error)
    {
        const auto deadline = Clock::now() +
            std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(timeout_seconds));
        std::optional<std::array<double, 6>> previous;
        std::optional<Clock::time_point> stable_since;
        while (Clock::now() < deadline) {
            std::vector<double> raw;
            if (!responseArray(robot_.getCartPosition(), 6, raw, error)) {
                previous.reset(); stable_since.reset();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            std::array<double, 6> current{};
            std::copy_n(raw.begin(), current.size(), current.begin());
            const auto now = Clock::now();
            if (previous) {
                double xyz2 = 0.0, abc2 = 0.0;
                for (std::size_t i = 0; i < 3; ++i) {
                    const double d = current[i] - (*previous)[i]; xyz2 += d * d;
                }
                for (std::size_t i = 3; i < 6; ++i) {
                    const double d = std::remainder(current[i] - (*previous)[i], 360.0);
                    abc2 += d * d;
                }
                if (std::sqrt(xyz2) < 0.3 && std::sqrt(abc2) < 0.1) {
                    if (!stable_since) stable_since = now;
                    if (std::chrono::duration<double>(now - *stable_since).count() >= stable_seconds)
                        return true;
                } else {
                    stable_since.reset();
                }
            }
            previous = current;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        error = "robot did not remain stable for " + std::to_string(stable_seconds) + "s";
        return false;
    }

    bool normalRecord(const std::string &mapid, const std::string &poseid,
                      int command, std::optional<int> point_type,
                      std::string &error)
    {
        std::vector<double> joints, pose;
        if (!responseArray(robot_.getJointPosition(), 6, joints, error) ||
            !responseArray(robot_.getCartPosition(), 6, pose, error)) return false;
        json point;
        point["command"] = command;
        point["point_type"] = point_type ? json(*point_type) : json(nullptr);
        point["angle"] = {{"jntpos1", joints[0]}, {"jntpos2", joints[1]},
                          {"jntpos3", joints[2]}, {"jntpos4", joints[3]},
                          {"jntpos5", joints[4]}, {"jntpos6", joints[5]}};
        point["pose"] = {{"x", pose[0]}, {"y", pose[1]}, {"z", pose[2]},
                         {"a", pose[3]}, {"b", pose[4]}, {"c", pose[5]}};
        try {
            const fs::path directory = config_.record_root / mapid / poseid;
            fs::create_directories(directory);
            const fs::path target = directory / (std::to_string(command) + ".json");
            std::ofstream output(target, std::ios::trunc);
            if (!output.is_open()) { error = "cannot write " + target.string(); return false; }
            output << json::array({point}).dump(4);
            if (!output.good()) { error = "write failed: " + target.string(); return false; }
            std::cout << "[TEACH] saved " << target << std::endl;
            return true;
        } catch (const std::exception &exception) {
            error = exception.what(); return false;
        }
    }

    bool recordTag(const std::string &mapid, const std::string &poseid,
                   int command, int tag_id, std::string &error)
    {
        if (tag_id < 0) { error = "tag_id must be >= 0"; return false; }
        TagTask task(config_);
        const fs::path json_path = config_.record_root / mapid / poseid /
                                   (std::to_string(command) + ".json");
        if (command == 1) {
            D455Camera camera;
            if (!camera.startColorOnly(error) || !waitRobotStable(2.0, 8.0, error)) return false;
            D455Frame frame;
            if (!camera.captureColorOne(frame, error)) return false;
            camera.stop();
            std::cout << "[APRILTAG] new color frame timestamp_ms="
                      << frame.color_timestamp_ms << std::endl;
            if (!normalRecord(mapid, poseid, command, 0, error) ||
                !task.recordReference(mapid, poseid, frame, tag_id, error)) {
                std::error_code ignored; fs::remove(json_path, ignored); return false;
            }
            return true;
        }
        if (!waitRobotStable(2.0, 8.0, error) ||
            !normalRecord(mapid, poseid, command, 0, error) ||
            !task.recordWork(mapid, poseid, command, error)) {
            std::error_code ignored; fs::remove(json_path, ignored); return false;
        }
        return true;
    }

    bool recordIcp(const std::string &mapid, const std::string &poseid,
                   int command, std::string &error)
    {
        if (!waitRobotStable(2.0, 8.0, error)) return false;
        IcpTask task(config_.record_root);
        const fs::path directory = config_.record_root / mapid / poseid;
        const fs::path json_path = directory / (std::to_string(command) + ".json");
        if (command == 1) {
            D455Camera camera;
            if (!camera.start(error)) return false;
            std::vector<std::vector<D455PointXYZ>> frames;
            for (int index = 0; index < 5; ++index) {
                D455Frame frame;
                if (camera.captureOne(frame, error)) {
                    std::cout << "[ICP] Ref " << index + 1 << "/5: "
                              << frame.point_cloud_color_m.size() << " points" << std::endl;
                    frames.push_back(std::move(frame.point_cloud_color_m));
                }
            }
            camera.stop();
            if (!normalRecord(mapid, poseid, command, 1, error) ||
                !task.recordReference(mapid, poseid, frames, error)) {
                std::error_code ignored;
                fs::remove(json_path, ignored); fs::remove(directory / "1_20mm.ply", ignored);
                fs::remove(directory / "1_10mm.ply", ignored); fs::remove(directory / "1_5mm.ply", ignored);
                fs::remove(directory / "1_wall.json", ignored);
                return false;
            }
            return true;
        }
        if (!normalRecord(mapid, poseid, command, 1, error) ||
            !task.recordWork(mapid, poseid, command, error)) {
            std::error_code ignored; fs::remove(json_path, ignored); return false;
        }
        return true;
    }

    bool finishActiveTeaching(std::string &error)
    {
        if (!active_) return true;
        if (!robot_.userCommand(c2::UserCommand::ToAuto, error) || !robot_.reset(error)) return false;
        std::cout << "[TEACH] finished, switched to Auto and reset" << std::endl;
        active_.reset();
        return true;
    }

    json startTeaching(const json &request)
    {
        std::string mapid, poseid, error;
        if (!stationFields(request, mapid, poseid, error)) return failure(request, "invalid_request", error);
        const std::optional<int> point_type = pointTypeFrom(request);
        if (point_type && *point_type != 0 && *point_type != 1)
            return failure(request, "invalid_request", "type1 point_type must be 0 or 1; omit it for a normal point");
        if (active_ && (active_->mapid != mapid || active_->poseid != poseid) &&
            !finishActiveTeaching(error))
            return failure(request, "finish_previous_teaching_failed", error);
        if (!robot_.userCommand(c2::UserCommand::ToReady, error))
            return failure(request, "start_drag_failed", error);
        fs::create_directories(config_.record_root / mapid / poseid);
        active_ = TeachingSession{mapid, poseid, point_type, tagIdFrom(request)};
        return success(request, "vision_point_teach", "teaching started in Ready/manual mode");
    }

    json recordTeachingPoint(const json &request, int command)
    {
        std::string mapid, poseid, error;
        if (!stationFields(request, mapid, poseid, error)) return failure(request, "invalid_request", error);
        if (command == 0 || command == -1) {
            if (active_ &&
                (active_->mapid != mapid || active_->poseid != poseid)) {
                return failure(
                    request, "teaching_station_mismatch",
                    "finish command station does not match active teaching: " +
                        active_->mapid + "/" + active_->poseid);
            }
            const bool teaching_finished = active_.has_value();
            if (!finishActiveTeaching(error)) return failure(request, "teaching_finish_failed", error);
            return success(
                request, "vision_point_teach",
                command == 0
                    ? "teaching finished and robot reset"
                    : "teaching cancelled and robot reset",
                {{"teaching_finished", teaching_finished},
                 {"reset_completed", teaching_finished}});
        }
        if (command < 1) return failure(request, "invalid_request", "teaching command must be -1, 0 or >= 1");
        if (hasPointTypeField(request))
            return failure(request, "invalid_request", "point_type belongs only to the preceding type1 teaching-start request");
        if (!active_)
            return failure(request, "teaching_not_started", "send type1 with mapid/poseid and optional point_type before recording command");
        if (active_->mapid != mapid || active_->poseid != poseid)
            return failure(request, "teaching_station_mismatch", "mechanical_arm_command mapid/poseid does not match the active type1 station");
        const std::optional<int> point_type = active_->point_type;
        bool recorded = false;
        if (!point_type) recorded = normalRecord(mapid, poseid, command, std::nullopt, error);
        else if (*point_type == 0) recorded = recordTag(mapid, poseid, command, active_->tag_id, error);
        else if (*point_type == 1) recorded = recordIcp(mapid, poseid, command, error);
        else error = "unsupported point_type=" + std::to_string(*point_type);
        if (!recorded) return failure(request, "record_failed", error);
        return success(request, "vision_point_teach", "command " + std::to_string(command) + " saved");
    }

    std::vector<std::pair<int, fs::path>> commandFiles(const fs::path &station) const
    {
        std::vector<std::pair<int, fs::path>> files;
        if (!fs::is_directory(station)) return files;
        for (const auto &entry : fs::directory_iterator(station)) {
            if (!entry.is_regular_file() || entry.path().extension() != ".json") continue;
            try {
                std::size_t parsed = 0;
                const std::string stem = entry.path().stem().string();
                const int command = std::stoi(stem, &parsed);
                if (parsed == stem.size() && command > 0) files.emplace_back(command, entry.path());
            } catch (...) {}
        }
        std::sort(files.begin(), files.end(), [](const auto &a, const auto &b) { return a.first < b.first; });
        return files;
    }

    bool loadRecord(const fs::path &path, json &point, std::string &error) const
    {
        try {
            std::ifstream input(path); json document; input >> document;
            if (!document.is_array() || document.empty() || !document.front().is_object()) {
                error = "point JSON must be a non-empty array: " + path.string(); return false;
            }
            point = document.front(); return true;
        } catch (const std::exception &exception) {
            error = "point JSON parse failed: " + std::string(exception.what()); return false;
        }
    }

    bool executeNormal(const fs::path &station, std::string &error)
    {
        const auto files = commandFiles(station);
        if (files.empty()) { error = "station has no command JSON"; return false; }
        for (const auto &[command, path] : files) {
            json point;
            if (!loadRecord(path, point, error)) return false;
            try {
                const json &a = point.at("angle");
                const std::array<double, 6> joints{
                    a.at("jntpos1").get<double>(), a.at("jntpos2").get<double>(),
                    a.at("jntpos3").get<double>(), a.at("jntpos4").get<double>(),
                    a.at("jntpos5").get<double>(), a.at("jntpos6").get<double>()};
                std::cout << "[NORMAL_EXEC] command=" << command << std::endl;
                if (!robot_.moveJoint(joints, 15.0, 15.0, error)) return false;
            } catch (const std::exception &exception) {
                error = "command=" + std::to_string(command) + " angle parse failed: " + exception.what();
                return false;
            }
        }
        return true;
    }

    bool recoverAfterTask(std::string &message)
    {
        robot_.stopBestEffort();
        std::string reset_error;
        if (robot_.userCommand(c2::UserCommand::ToAuto, reset_error) &&
            robot_.reset(reset_error)) {
            message += "; recovered to reset point, next external task may continue";
            return true;
        }
        message += "; recovery reset also failed: " + reset_error;
        return false;
    }

    json stationFailureAndReset(const json &request,
                                const std::string &code,
                                std::string message)
    {
        // 必须先完成 stop/Auto/复位，再把终态响应发给小车；否则小车可能
        // 在机械臂仍位于失败姿态时驶向下一个站点。
        const bool reset_completed = recoverAfterTask(message);
        json response = failure(request, code, message);
        response["action"] = "vision_station_execute";
        response["data"] = {
            {"task_completed", true},
            {"reset_completed", reset_completed},
            {"next_station_allowed", reset_completed},
        };
        return response;
    }

    json executeStation(const json &request)
    {
        std::string mapid, poseid, error;
        if (!stationFields(request, mapid, poseid, error)) return failure(request, "invalid_request", error);
        if (!finishActiveTeaching(error))
            return stationFailureAndReset(request, "finish_teaching_failed", error);
        if (!robot_.userCommand(c2::UserCommand::ToAuto, error))
            return stationFailureAndReset(request, "auto_mode_failed", error);
        const fs::path station = config_.record_root / mapid / poseid;
        json reference;
        if (!loadRecord(station / "1.json", reference, error))
            return stationFailureAndReset(request, "load_station_failed", error);
        std::optional<int> point_type;
        if (reference.contains("point_type") && reference.at("point_type").is_number_integer())
            point_type = reference.at("point_type").get<int>();
        bool executed = false;
        if (!point_type) executed = executeNormal(station, error);
        else if (*point_type == 0) {
            TagExecuteTask task(config_); TagExecuteResult result; TagExecuteOptions options;
            options.stable_seconds = kArrivalStableSeconds;
            options.joint_arrival_tolerance_deg = kArrivalToleranceDeg;
            executed = task.execute(mapid, poseid, tagCallbacks(robot_), result, error, options);
        } else if (*point_type == 1) {
            IcpExecuteTask task(config_.record_root, config_.handeye_json);
            IcpExecuteResult result; IcpExecuteOptions options;
            options.correction_steps = 2; options.discard_new_frames = 3;
            options.stable_seconds = kArrivalStableSeconds;
            options.joint_arrival_tolerance_deg = kArrivalToleranceDeg;
            executed = task.execute(mapid, poseid, icpCallbacks(robot_), result, error, options);
        } else error = "invalid stored point_type=" + std::to_string(*point_type);
        if (!executed)
            return stationFailureAndReset(request, "execution_error", error);
        if (!robot_.reset(error))
            return stationFailureAndReset(request, "reset_failed", error);
        return success(
            request, "vision_station_execute",
            "station completed and robot reset",
            {{"task_completed", true},
             {"reset_completed", true},
             {"next_station_allowed", true}});
    }

    json queryStation(const json &request)
    {
        std::string mapid, poseid, error;
        if (!stationFields(request, mapid, poseid, error)) return failure(request, "invalid_request", error);
        const bool teaching_finished = active_.has_value();
        if (active_ && (active_->mapid != mapid || active_->poseid != poseid)) {
            return failure(
                request, "teaching_station_mismatch",
                "type2 station does not match active teaching: " +
                    active_->mapid + "/" + active_->poseid);
        }
        if (!finishActiveTeaching(error)) return failure(request, "teaching_finish_failed", error);
        json points = json::array();
        for (const auto &[command, path] : commandFiles(config_.record_root / mapid / poseid)) {
            (void)command; json point;
            if (!loadRecord(path, point, error)) return failure(request, "query_failed", error);
            points.push_back(std::move(point));
        }
        return success(
            request, "vision_station_points",
            teaching_finished
                ? "teaching finished, robot reset, and station points queried"
                : "station points queried",
            {{"points", points},
             {"teaching_finished", teaching_finished},
             {"reset_completed", teaching_finished}});
    }

    json deleteStation(const json &request)
    {
        std::string mapid, poseid, error;
        if (!stationFields(request, mapid, poseid, error)) return failure(request, "invalid_request", error);
        if (active_ && active_->mapid == mapid && active_->poseid == poseid && !finishActiveTeaching(error))
            return failure(request, "teaching_finish_failed", error);
        const fs::path station = config_.record_root / mapid / poseid;
        std::error_code filesystem_error;
        if (request.contains("command") && request.at("command").is_number_integer()) {
            const int command = request.at("command").get<int>();
            if (command < 1) return failure(request, "invalid_request", "delete command must be >= 1");
            fs::remove(station / (std::to_string(command) + ".json"), filesystem_error);
            if (command == 1) {
                fs::remove(station / "1_20mm.ply", filesystem_error);
                fs::remove(station / "1_10mm.ply", filesystem_error);
                fs::remove(station / "1_5mm.ply", filesystem_error);
            }
        } else fs::remove_all(station, filesystem_error);
        if (filesystem_error) return failure(request, "delete_failed", filesystem_error.message());
        return success(request, "vision_station_delete", "station data deleted");
    }

    json controllerCommand(const json &request, int command)
    {
        std::string error, action; bool ok = false;
        switch (command) {
        case 1: action = "robot_reset"; ok = robot_.userCommand(c2::UserCommand::ToAuto, error) && robot_.reset(error); active_.reset(); break;
        case 2: action = "robot_enable"; ok = robot_.userCommand(c2::UserCommand::SwitchOn, error); break;
        case 3: action = "robot_disable"; ok = robot_.userCommand(c2::UserCommand::SwitchOff, error); active_.reset(); break;
        case 4: action = "robot_start_drag"; ok = robot_.userCommand(c2::UserCommand::ToReady, error); break;
        case 5: action = "robot_stop_drag"; ok = robot_.userCommand(c2::UserCommand::ToAuto, error); active_.reset(); break;
        case 6: action = "robot_clear_error"; ok = robot_.userCommand(c2::UserCommand::ClearWarning, error); break;
        default: return failure(request, "unsupported_command", "mechanical_arm_command only supports command 1..6");
        }
        if (!ok) return failure(request, "controller_command_failed", error);
        json response = success(request, action, action + " completed");
        response["command"] = command;
        const char *legacy = command == 1 ? "fuwei" : command == 2 ? "shangdian" :
                             command == 3 ? "xiadian" : command == 4 ? "tuozhuai" :
                             command == 5 ? "zidong" : "qingchu";
        response[legacy] = "success";
        return response;
    }

    json dispatch(const json &request)
    {
        if (!request.is_object()) return failure(json::object(), "invalid_request", "request must be an object");
        const std::string type = request.value("type", "");
        const bool has_map = request.contains("mapid") && request.at("mapid").is_string() && !request.at("mapid").get<std::string>().empty();
        const bool has_pose = request.contains("poseid") && request.at("poseid").is_string() && !request.at("poseid").get<std::string>().empty();
        if (has_map != has_pose) return failure(request, "invalid_request", "mapid and poseid must be provided together");
        if (type == "type1") {
            if (request.contains("command")) return failure(request, "invalid_request", "type1 starts teaching and must not contain command");
            return startTeaching(request);
        }
        if (type == "type2") return queryStation(request);
        if (type == "type3") return deleteStation(request);
        if (type == "mechanical_arm_command" || (type.empty() && request.contains("command"))) {
            if (!request.contains("command") || !request.at("command").is_number_integer())
                return failure(request, "invalid_request", "command must be an integer");
            const int command = request.at("command").get<int>();
            return has_map ? recordTeachingPoint(request, command) : controllerCommand(request, command);
        }
        if (type.empty() && has_map) return executeStation(request);
        return failure(request, "unsupported_request", "unsupported NX request");
    }

    RobotRuntime &robot_;
    TagTaskConfig config_;
    std::optional<TeachingSession> active_;
    std::mutex operation_mutex_;
};

bool sendAll(int fd, const std::string &message)
{
    std::size_t total = 0;
    while (total < message.size()) {
        const ssize_t sent = ::send(fd, message.data() + total, message.size() - total, MSG_NOSIGNAL);
        if (sent <= 0) return false;
        total += static_cast<std::size_t>(sent);
    }
    return true;
}

std::optional<std::string> extractJsonObject(std::string &buffer)
{
    const std::size_t first = buffer.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) { buffer.clear(); return std::nullopt; }
    if (first > 0) buffer.erase(0, first);
    if (buffer.front() != '{') {
        const std::size_t next = buffer.find('{');
        if (next == std::string::npos) { buffer.clear(); return std::nullopt; }
        buffer.erase(0, next);
    }
    int depth = 0; bool quoted = false, escaped = false;
    for (std::size_t index = 0; index < buffer.size(); ++index) {
        const char current = buffer[index];
        if (quoted) {
            if (escaped) escaped = false;
            else if (current == '\\') escaped = true;
            else if (current == '"') quoted = false;
            continue;
        }
        if (current == '"') quoted = true;
        else if (current == '{') ++depth;
        else if (current == '}' && --depth == 0) {
            std::string output = buffer.substr(0, index + 1);
            buffer.erase(0, index + 1); return output;
        }
    }
    return std::nullopt;
}

void serveClient(int client, std::string peer, TaskEngine &engine)
{
    std::cout << "[NX] connected: " << peer << std::endl;
    std::string buffer; std::array<char, 4096> block{};
    while (g_running) {
        const ssize_t count = ::recv(client, block.data(), block.size(), 0);
        if (count <= 0) break;
        buffer.append(block.data(), static_cast<std::size_t>(count));
        while (const auto object = extractJsonObject(buffer)) {
            json response;
            try {
                const json request = json::parse(*object);
                std::cout << "[NX] request: " << request.dump() << std::endl;
                response = engine.handle(request);
            } catch (const std::exception &exception) {
                response = {{"type", "error"}, {"status", "failed"},
                            {"error_code", "invalid_json"}, {"message", exception.what()}};
            }
            std::cout << "[NX] result: " << response.dump() << std::endl;
            if (!sendAll(client, response.dump() + "\n")) break;
        }
    }
    ::shutdown(client, SHUT_RDWR); ::close(client);
    std::cout << "[NX] disconnected: " << peer << std::endl;
}

int runServer(TaskEngine &engine)
{
    const int listener = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listener < 0) { std::cerr << "socket failed: " << std::strerror(errno) << std::endl; return 1; }
    g_listen_socket = listener;
    int reuse = 1; ::setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{}; address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY); address.sin_port = htons(kServerPort);
    if (::bind(listener, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0 ||
        ::listen(listener, 8) < 0) {
        std::cerr << "TCP :" << kServerPort << " start failed: " << std::strerror(errno) << std::endl;
        ::close(listener); g_listen_socket = -1; return 1;
    }
    std::cout << "[NX] RobotControl listening on 0.0.0.0:" << kServerPort << std::endl;
    while (g_running) {
        sockaddr_in peer_address{}; socklen_t peer_size = sizeof(peer_address);
        const int client = ::accept(listener, reinterpret_cast<sockaddr *>(&peer_address), &peer_size);
        if (client < 0) { if (g_running) std::cerr << "accept failed: " << std::strerror(errno) << std::endl; continue; }
        char ip[INET_ADDRSTRLEN]{}; ::inet_ntop(AF_INET, &peer_address.sin_addr, ip, sizeof(ip));
        std::string peer = std::string(ip) + ":" + std::to_string(ntohs(peer_address.sin_port));
        std::thread(serveClient, client, std::move(peer), std::ref(engine)).detach();
    }
    return 0;
}
}  // namespace

int main(int argc, char **argv)
{
    const std::string robot_host = argc > 1 ? argv[1] : "192.168.2.5";
    const std::string robot_port = argc > 2 ? argv[2] : "9000";
    std::signal(SIGINT, stopSignal); std::signal(SIGTERM, stopSignal);
    TagTaskConfig config; std::string error;
    const fs::path config_path = tagTaskConfigPath();
    if (!loadTagTaskConfig(config_path, config, error)) {
        std::cerr << "[STARTUP] " << error << std::endl; return 1;
    }
    std::cout << "[STARTUP] config=" << fs::absolute(config_path)
              << ", record_root=" << config.record_root << std::endl;
    RobotRuntime robot(robot_host, robot_port);
    if (!robot.preflight(error)) { std::cerr << "[STARTUP] " << error << std::endl; return 1; }
    TaskEngine engine(robot, std::move(config));
    return runServer(engine);
}
