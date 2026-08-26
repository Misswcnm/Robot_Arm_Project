#include "IcpExecuteTask.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <optional>
#include <thread>
#include <unordered_set>
#include <utility>

namespace fs = std::filesystem;

namespace
{
using Clock = std::chrono::steady_clock;
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

class KdTree
{
public:
    explicit KdTree(std::vector<Point> points) : points_(std::move(points))
    {
        order_.resize(points_.size());
        std::iota(order_.begin(), order_.end(), 0);
        nodes_.reserve(points_.size());
        root_ = build(0, order_.size(), 0);
    }

    const std::vector<Point> &points() const noexcept { return points_; }

    bool nearest(const Point &query, double maximum_distance,
                 int &point_index, double &distance_squared) const
    {
        point_index = -1;
        distance_squared = maximum_distance * maximum_distance;
        search(root_, query, point_index, distance_squared);
        return point_index >= 0;
    }

private:
    struct Node
    {
        int point_index{-1};
        int left{-1};
        int right{-1};
        int axis{};
    };

    int build(std::size_t begin, std::size_t end, int depth)
    {
        if (begin >= end) {
            return -1;
        }
        const int axis = depth % 3;
        const std::size_t middle = begin + (end - begin) / 2;
        std::nth_element(
            order_.begin() + static_cast<std::ptrdiff_t>(begin),
            order_.begin() + static_cast<std::ptrdiff_t>(middle),
            order_.begin() + static_cast<std::ptrdiff_t>(end),
            [this, axis](int left, int right) {
                return points_[static_cast<std::size_t>(left)][axis] <
                       points_[static_cast<std::size_t>(right)][axis];
            });
        const int node_index = static_cast<int>(nodes_.size());
        nodes_.push_back(Node{});
        const int left = build(begin, middle, depth + 1);
        const int right = build(middle + 1, end, depth + 1);
        nodes_[static_cast<std::size_t>(node_index)] =
            Node{order_[middle], left, right, axis};
        return node_index;
    }

    void search(int node_index, const Point &query, int &best_index,
                double &best_distance_squared) const
    {
        if (node_index < 0) {
            return;
        }
        const Node &node = nodes_[static_cast<std::size_t>(node_index)];
        const Point &candidate =
            points_[static_cast<std::size_t>(node.point_index)];
        const double distance_squared = (candidate - query).squaredNorm();
        if (distance_squared < best_distance_squared) {
            best_distance_squared = distance_squared;
            best_index = node.point_index;
        }
        const double plane_delta = query[node.axis] - candidate[node.axis];
        const int near_child = plane_delta < 0.0 ? node.left : node.right;
        const int far_child = plane_delta < 0.0 ? node.right : node.left;
        search(near_child, query, best_index, best_distance_squared);
        if (plane_delta * plane_delta < best_distance_squared) {
            search(far_child, query, best_index, best_distance_squared);
        }
    }

    std::vector<Point> points_;
    std::vector<int> order_;
    std::vector<Node> nodes_;
    int root_{-1};
};

struct TemplateLevel
{
    double voxel_m{};
    double maximum_correspondence_m{};
    KdTree tree;

    TemplateLevel(double voxel, double maximum_correspondence,
                  std::vector<Point> points)
        : voxel_m(voxel),
          maximum_correspondence_m(maximum_correspondence),
          tree(std::move(points))
    {
    }
};

struct Alignment
{
    Eigen::Isometry3d current_to_reference{Eigen::Isometry3d::Identity()};
    double rmse_m{std::numeric_limits<double>::infinity()};
    int inliers{};
    double overlap{};
};

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
        error = "angle/pose 解析失败: " + std::string(exception.what());
        return false;
    }
    return true;
}

bool readTransform(const json &value, IcpTransform &output,
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
                const double item = value[row][column].get<double>();
                if (!std::isfinite(item)) {
                    error = "trans 包含非有限数值";
                    return false;
                }
                output[row * 4 + column] = item;
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

bool loadJsonDocument(const fs::path &configured_path, json &document,
                      fs::path &resolved, std::string &error)
{
    fs::path current = configured_path.is_absolute()
        ? configured_path
        : fs::absolute(configured_path);
    try {
        for (int depth = 0; depth < 4; ++depth) {
            std::ifstream input(current);
            if (!input.is_open()) {
                error = "标定文件不存在: " + current.string();
                return false;
            }
            input >> document;
            if (document.is_string()) {
                const fs::path target(document.get<std::string>());
                current = target.is_absolute()
                    ? target
                    : current.parent_path() / target;
                continue;
            }
            resolved = current;
            return true;
        }
    } catch (const std::exception &exception) {
        error = "标定 JSON 解析失败: " + std::string(exception.what());
        return false;
    }
    error = "标定 JSON 指针层级超过 4 层";
    return false;
}

bool jsonTransform(const json &value, Eigen::Isometry3d &output,
                   std::string &error)
{
    IcpTransform flat{};
    if (!readTransform(value, flat, error)) {
        return false;
    }
    Eigen::Matrix4d matrix;
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            matrix(row, column) =
                flat[static_cast<std::size_t>(row * 4 + column)];
        }
    }
    output = Eigen::Isometry3d(matrix);
    return true;
}

bool loadHandeye(const fs::path &path, Eigen::Isometry3d &output,
                 std::string &error)
{
    json document;
    fs::path resolved;
    if (!loadJsonDocument(path, document, resolved, error)) {
        return false;
    }
    if (!document.is_object() || !document.contains("T_flange_camera") ||
        !document["T_flange_camera"].contains("matrix")) {
        error = "手眼标定缺少 T_flange_camera.matrix: " + resolved.string();
        return false;
    }
    return jsonTransform(document["T_flange_camera"]["matrix"], output, error);
}

Eigen::Isometry3d poseTransform(const std::array<double, 6> &pose)
{
    const double radians = std::acos(-1.0) / 180.0;
    Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
    output.linear() =
        (Eigen::AngleAxisd(pose[5] * radians, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pose[4] * radians, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(pose[3] * radians, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    output.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    return output;
}

Eigen::Isometry3d flatTransform(const IcpTransform &flat)
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

c2::CPos transformCPos(const Eigen::Isometry3d &transform)
{
    const Eigen::Vector3d zyx = transform.linear().eulerAngles(2, 1, 0);
    const double degrees = 180.0 / std::acos(-1.0);
    c2::CPos output;
    output.x = transform.translation().x();
    output.y = transform.translation().y();
    output.z = transform.translation().z();
    output.a = zyx[2] * degrees;
    output.b = zyx[1] * degrees;
    output.c = zyx[0] * degrees;
    return output;
}

std::vector<Point> voxelDown(const std::vector<D455PointXYZ> &input,
                             double voxel_m, std::size_t maximum_points)
{
    std::vector<Point> output;
    output.reserve(input.size());
    std::unordered_set<Voxel, VoxelHash> occupied;
    occupied.reserve(input.size());
    for (const D455PointXYZ &value : input) {
        const Point point(value.x_m, value.y_m, value.z_m);
        const Voxel voxel{
            static_cast<std::int64_t>(std::floor(point.x() / voxel_m)),
            static_cast<std::int64_t>(std::floor(point.y() / voxel_m)),
            static_cast<std::int64_t>(std::floor(point.z() / voxel_m)),
        };
        if (occupied.insert(voxel).second) {
            output.push_back(point);
        }
    }
    if (output.size() <= maximum_points) {
        return output;
    }
    // RealSense 点按图像扫描顺序排列，不能简单截取前 N 点；均匀抽样保留全视野。
    std::vector<Point> sampled;
    sampled.reserve(maximum_points);
    const double step = static_cast<double>(output.size()) /
                        static_cast<double>(maximum_points);
    for (std::size_t index = 0; index < maximum_points; ++index) {
        sampled.push_back(output[static_cast<std::size_t>(index * step)]);
    }
    return sampled;
}

bool loadPly(const fs::path &path, std::vector<Point> &points,
             std::string &error)
{
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open()) {
        error = "ICP 模板不存在: " + path.string();
        return false;
    }
    std::string line;
    std::size_t vertex_count = 0;
    bool binary_little_endian = false;
    bool ascii = false;
    bool header_complete = false;
    while (std::getline(input, line)) {
        if (line.rfind("format binary_little_endian", 0) == 0) {
            binary_little_endian = true;
        } else if (line.rfind("format ascii", 0) == 0) {
            ascii = true;
        } else if (line.rfind("element vertex ", 0) == 0) {
            vertex_count = static_cast<std::size_t>(
                std::stoull(line.substr(std::string("element vertex ").size())));
        } else if (line == "end_header") {
            header_complete = true;
            break;
        }
    }
    if (!header_complete || vertex_count == 0 ||
        (!binary_little_endian && !ascii)) {
        error = "不支持或无效的 ICP PLY: " + path.string();
        return false;
    }
    points.clear();
    points.reserve(vertex_count);
    if (binary_little_endian) {
        for (std::size_t index = 0; index < vertex_count; ++index) {
            float xyz[3]{};
            input.read(reinterpret_cast<char *>(xyz), sizeof(xyz));
            if (!input.good()) {
                error = "ICP PLY 数据截断: " + path.string();
                return false;
            }
            if (std::isfinite(xyz[0]) && std::isfinite(xyz[1]) &&
                std::isfinite(xyz[2])) {
                points.emplace_back(xyz[0], xyz[1], xyz[2]);
            }
        }
    } else {
        for (std::size_t index = 0; index < vertex_count; ++index) {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            if (!(input >> x >> y >> z)) {
                error = "ICP ASCII PLY 数据截断: " + path.string();
                return false;
            }
            std::getline(input, line);
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                points.emplace_back(x, y, z);
            }
        }
    }
    if (points.size() < 10) {
        error = "ICP 模板有效点不足: " + path.string();
        return false;
    }
    return true;
}

bool alignOneStep(const std::vector<Point> &source,
                  const KdTree &target,
                  double maximum_correspondence_m,
                  Eigen::Isometry3d &accumulated,
                  Alignment &alignment)
{
    std::vector<Point> source_pairs;
    std::vector<Point> target_pairs;
    source_pairs.reserve(source.size());
    target_pairs.reserve(source.size());
    for (const Point &point : source) {
        const Point transformed = accumulated * point;
        int nearest_index = -1;
        double distance_squared = 0.0;
        if (!target.nearest(transformed, maximum_correspondence_m,
                            nearest_index, distance_squared)) {
            continue;
        }
        source_pairs.push_back(transformed);
        target_pairs.push_back(
            target.points()[static_cast<std::size_t>(nearest_index)]);
    }
    if (source_pairs.size() < 10) {
        alignment.inliers = 0;
        alignment.overlap = 0.0;
        alignment.rmse_m = std::numeric_limits<double>::infinity();
        return false;
    }

    Point source_centroid = Point::Zero();
    Point target_centroid = Point::Zero();
    for (std::size_t index = 0; index < source_pairs.size(); ++index) {
        source_centroid += source_pairs[index];
        target_centroid += target_pairs[index];
    }
    source_centroid /= static_cast<double>(source_pairs.size());
    target_centroid /= static_cast<double>(target_pairs.size());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (std::size_t index = 0; index < source_pairs.size(); ++index) {
        covariance += (source_pairs[index] - source_centroid) *
                      (target_pairs[index] - target_centroid).transpose();
    }
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d rotation = svd.matrixV() * svd.matrixU().transpose();
    if (rotation.determinant() < 0.0) {
        Eigen::Matrix3d corrected_v = svd.matrixV();
        corrected_v.col(2) *= -1.0;
        rotation = corrected_v * svd.matrixU().transpose();
    }
    const Point translation =
        target_centroid - rotation * source_centroid;
    Eigen::Isometry3d step = Eigen::Isometry3d::Identity();
    step.linear() = rotation;
    step.translation() = translation;
    accumulated = step * accumulated;

    double squared_sum = 0.0;
    for (std::size_t index = 0; index < source_pairs.size(); ++index) {
        squared_sum +=
            (rotation * source_pairs[index] + translation -
             target_pairs[index])
                .squaredNorm();
    }
    alignment.current_to_reference = accumulated;
    alignment.inliers = static_cast<int>(source_pairs.size());
    alignment.overlap = static_cast<double>(source_pairs.size()) /
                        static_cast<double>(source.size());
    alignment.rmse_m =
        std::sqrt(squared_sum / static_cast<double>(source_pairs.size()));
    return true;
}

bool alignPyramid(const std::vector<D455PointXYZ> &current,
                  const std::vector<TemplateLevel> &levels,
                  const Eigen::Isometry3d &initial,
                  const IcpExecuteOptions &options,
                  Alignment &alignment, std::string &error)
{
    Eigen::Isometry3d accumulated = initial;
    for (std::size_t index = 0; index < levels.size(); ++index) {
        const TemplateLevel &level = levels[index];
        const std::vector<Point> downsampled =
            voxelDown(current, level.voxel_m,
                      options.maximum_source_points);
        if (downsampled.size() < 10) {
            error = "ICP 当前点云下采样后点数不足";
            return false;
        }
        if (!alignOneStep(downsampled, level.tree,
                          level.maximum_correspondence_m,
                          accumulated, alignment)) {
            error = "ICP L" + std::to_string(index) + " 匹配点不足";
            return false;
        }
        std::cout << "[ICP_EXEC] L" << index
                  << " voxel=" << level.voxel_m * 1000.0 << "mm"
                  << " rmse=" << alignment.rmse_m * 1000.0 << "mm"
                  << " inliers=" << alignment.inliers
                  << " overlap=" << alignment.overlap << std::endl;
    }
    return true;
}

bool waitJointArrival(const IcpRobotCallbacks &robot,
                      const std::array<double, 6> &target,
                      const IcpExecuteOptions &options,
                      std::string &error)
{
    const auto deadline = Clock::now() +
        std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(options.stable_timeout_seconds));
    std::optional<Clock::time_point> arrived_since;
    std::string last_error;
    while (Clock::now() < deadline) {
        std::vector<double> joints;
        if (!responseArray(robot.get_joint_position(), 6, "读取当前关节",
                           joints, last_error)) {
            arrived_since.reset();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        double maximum_residual = 0.0;
        for (std::size_t index = 0; index < target.size(); ++index) {
            maximum_residual = std::max(
                maximum_residual,
                std::abs(std::remainder(joints[index] - target[index], 360.0)));
        }
        const auto now = Clock::now();
        if (maximum_residual <= options.joint_arrival_tolerance_deg) {
            if (!arrived_since) {
                arrived_since = now;
            }
            if (std::chrono::duration<double>(now - *arrived_since).count() >=
                options.stable_seconds) {
                std::cout << "[ICP_EXEC] 到点并连续稳定 "
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

bool moveJoint(const IcpRobotCallbacks &robot,
               const std::array<double, 6> &target,
               const IcpExecuteOptions &options,
               const std::string &label, std::string &error)
{
    c2::Point point;
    point.type = c2::PointType::Joint;
    for (std::size_t index = 0; index < target.size(); ++index) {
        point.apos.jntPos[index] = target[index];
    }
    const c2::Response moved = robot.mov_j(
        point, options.move_speed, options.move_acc,
        options.robot_timeout_seconds);
    if (moved.code != c2::ResponseCode::OK) {
        error = label + " movJ(APos) 失败: " + moved.msg;
        return false;
    }
    std::cout << "[ICP_EXEC] " << label << " 已发送" << std::endl;
    return waitJointArrival(robot, target, options, error);
}

bool solveAndMove(const IcpRobotCallbacks &robot,
                  const Eigen::Isometry3d &target,
                  const IcpExecuteOptions &options,
                  const std::string &label, std::string &error,
                  bool *ik_failed = nullptr)
{
    if (ik_failed) {
        *ik_failed = false;
    }
    std::vector<double> current_joints;
    if (!responseArray(robot.get_joint_position(), 6, "读取 IK 参考关节",
                       current_joints, error)) {
        return false;
    }
    c2::APos reference;
    for (std::size_t index = 0; index < 6; ++index) {
        reference.jntPos[index] = current_joints[index];
    }
    const c2::CPos cpos = transformCPos(target);
    const c2::Response solved = robot.cpos_to_apos(
        cpos, reference, options.robot_timeout_seconds);
    std::vector<double> solved_joints;
    if (!responseArray(solved, 6, label + " cposToAPos",
                       solved_joints, error)) {
        if (ik_failed) {
            *ik_failed = true;
        }
        return false;
    }
    std::array<double, 6> target_joints{};
    std::copy_n(solved_joints.begin(), target_joints.size(),
                target_joints.begin());
    std::cout << "[ICP_EXEC] " << label << " target_xyzabc_mm_deg=["
              << cpos.x << "," << cpos.y << "," << cpos.z << ","
              << cpos.a << "," << cpos.b << "," << cpos.c << "]"
              << std::endl;
    return moveJoint(robot, target_joints, options, label, error);
}

bool readCurrentPose(const IcpRobotCallbacks &robot,
                     Eigen::Isometry3d &pose, std::string &error)
{
    std::vector<double> raw;
    if (!responseArray(robot.get_cart_position(), 6, "读取当前法兰位姿",
                       raw, error)) {
        return false;
    }
    std::array<double, 6> values{};
    std::copy_n(raw.begin(), values.size(), values.begin());
    pose = poseTransform(values);
    return true;
}

bool validCallbacks(const IcpRobotCallbacks &robot, std::string &error)
{
    if (!robot.get_joint_position || !robot.get_cart_position ||
        !robot.mov_j || !robot.cpos_to_apos) {
        error = "ICP 执行端机械臂回调不完整";
        return false;
    }
    return true;
}
}  // namespace

IcpExecuteTask::IcpExecuteTask(fs::path record_root,
                               fs::path handeye_json)
    : record_root_(std::move(record_root)),
      handeye_json_(std::move(handeye_json))
{
}

bool IcpExecuteTask::loadPlan(const std::string &mapid,
                              const std::string &poseid,
                              IcpExecutionPlan &plan,
                              std::string &error) const
{
    error.clear();
    plan = IcpExecutionPlan{};
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
        if (reference.value("point_type", -1) != 1) {
            error = "command=1 不是 ICP Ref(point_type=1)";
            return false;
        }
        if (!readSix(reference.at("angle"),
                     {"jntpos1", "jntpos2", "jntpos3",
                      "jntpos4", "jntpos5", "jntpos6"},
                     plan.reference.angle, error) ||
            !readSix(reference.at("pose"), {"x", "y", "z", "a", "b", "c"},
                     plan.reference.pose, error)) {
            return false;
        }
        plan.reference.template_paths = {
            station / "1_20mm.ply",
            station / "1_10mm.ply",
            station / "1_5mm.ply",
        };
        for (const fs::path &path : plan.reference.template_paths) {
            if (!fs::is_regular_file(path)) {
                error = "ICP 模板不存在: " + path.string();
                return false;
            }
        }
        if (!loadIcpWallPlane(station / "1_wall.json",
                              plan.reference.wall_plane, error)) {
            return false;
        }

        std::vector<std::pair<int, fs::path>> command_files;
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
            if (point.value("point_type", -1) != 1) {
                error = "command=" + std::to_string(command) +
                        " 与 ICP Ref 类型不一致";
                return false;
            }
            IcpWorkRecord work;
            work.command = command;
            if (!point.contains("trans") ||
                !readTransform(point["trans"], work.trans, error)) {
                error = "command=" + std::to_string(command) + ": " + error;
                return false;
            }
            plan.works.push_back(work);
        }
        if (plan.works.empty()) {
            error = "ICP 点没有 command>1 的 Work";
            return false;
        }
        return true;
    } catch (const std::exception &exception) {
        error = "ICP 执行计划读取失败: " + std::string(exception.what());
        return false;
    }
}

bool IcpExecuteTask::execute(const std::string &mapid,
                             const std::string &poseid,
                             const IcpRobotCallbacks &robot,
                             IcpExecuteResult &result,
                             std::string &error,
                             const IcpExecuteOptions &options) const
{
    error.clear();
    result = IcpExecuteResult{};
    if (!validCallbacks(robot, error)) {
        return false;
    }
    if (options.correction_steps < 1 ||
        options.discard_new_frames < 0 ||
        !std::isfinite(options.final_maximum_residual_mm) ||
        options.final_maximum_residual_mm < 0.0 ||
        !std::isfinite(options.initial_search_y_start_mm) ||
        !std::isfinite(options.initial_search_y_end_mm) ||
        options.initial_search_samples < 2 ||
        options.wall_estimation_attempts < 1 ||
        !std::isfinite(options.wall_filter.estimate_voxel_m) ||
        options.wall_filter.estimate_voxel_m <= 0.0 ||
        !std::isfinite(options.wall_filter.plane_distance_m) ||
        options.wall_filter.plane_distance_m <= 0.0 ||
        !std::isfinite(options.wall_filter.minimum_inlier_ratio) ||
        options.wall_filter.minimum_inlier_ratio <= 0.0 ||
        options.wall_filter.minimum_inlier_ratio > 1.0 ||
        !std::isfinite(
            options.wall_filter.maximum_reference_normal_difference_deg) ||
        options.wall_filter.maximum_reference_normal_difference_deg <= 0.0 ||
        options.maximum_source_points < 10) {
        error = "ICP 执行参数无效";
        return false;
    }

    IcpExecutionPlan plan;
    if (!loadPlan(mapid, poseid, plan, error)) {
        return false;
    }
    Eigen::Isometry3d flange_camera;
    if (!loadHandeye(handeye_json_, flange_camera, error)) {
        return false;
    }

    const std::array<double, 3> voxels{0.020, 0.010, 0.005};
    const std::array<double, 3> distances{0.100, 0.050, 0.025};
    std::vector<TemplateLevel> levels;
    levels.reserve(3);
    for (std::size_t index = 0; index < 3; ++index) {
        std::vector<Point> points;
        if (!loadPly(plan.reference.template_paths[index], points, error)) {
            return false;
        }
        std::cout << "[ICP_EXEC] 模板 L" << index << " 加载 "
                  << points.size() << " 点" << std::endl;
        levels.emplace_back(voxels[index], distances[index], std::move(points));
    }

    // 整个 ICP 请求只启动、预热一次；析构也能保证失败路径关闭相机。
    D455Camera camera;
    if (!camera.start(error)) {
        return false;
    }
    if (!moveJoint(robot, plan.reference.angle, options,
                   "ICP Ref 观察位", error)) {
        return false;
    }

    const Eigen::Isometry3d base_flange_reference =
        poseTransform(plan.reference.pose);
    const Eigen::Isometry3d base_camera_reference =
        base_flange_reference * flange_camera;

    auto metricsAccepted = [&options](const Alignment &alignment,
                                      double translation_mm,
                                      double rotation_deg) {
        return alignment.inliers >= options.minimum_inliers &&
               alignment.overlap >= options.minimum_overlap &&
               alignment.rmse_m * 1000.0 <= options.maximum_rmse_mm &&
               translation_mm <= options.maximum_correction_mm &&
               rotation_deg <= options.maximum_correction_deg;
    };
    auto correctionTarget = [&flange_camera](
        const Alignment &alignment,
        const Eigen::Isometry3d &base_flange_current)
        -> Eigen::Isometry3d {
        Eigen::Isometry3d correction_camera =
            alignment.current_to_reference;
        // ICP 内部平移是 m，机械臂变换使用 mm。
        correction_camera.translation() *= 1000.0;
        const Eigen::Isometry3d flange_delta =
            flange_camera * correction_camera * flange_camera.inverse();
        const Eigen::Isometry3d target =
            base_flange_current * flange_delta.inverse();
        return target;
    };
    auto captureObjectCloud = [&](
        const std::string &label,
        D455Frame &frame,
        Eigen::Isometry3d &base_flange_current,
        Eigen::Isometry3d &initial,
        bool &fatal_failure,
        std::string &capture_error) {
        fatal_failure = false;
        std::string last_error;
        for (int attempt = 1;
             attempt <= options.wall_estimation_attempts; ++attempt) {
            D455Frame candidate_frame;
            if (!camera.captureFreshPointCloud(
                    candidate_frame, options.discard_new_frames,
                    last_error)) {
                std::cout << "[ICP_EXEC] " << label
                          << " 新点云获取失败 attempt=" << attempt << "/"
                          << options.wall_estimation_attempts << ": "
                          << last_error << std::endl;
                continue;
            }
            if (!readCurrentPose(robot, base_flange_current, last_error)) {
                fatal_failure = true;
                capture_error = last_error;
                return false;
            }
            const Eigen::Isometry3d base_camera_current =
                base_flange_current * flange_camera;
            initial = base_camera_reference.inverse() * base_camera_current;
            initial.translation() /= 1000.0;

            const Point reference_normal(
                plan.reference.wall_plane.normal_camera[0],
                plan.reference.wall_plane.normal_camera[1],
                plan.reference.wall_plane.normal_camera[2]);
            const Point expected_normal =
                initial.linear().transpose() * reference_normal;
            const std::array<double, 3> expected{
                expected_normal.x(), expected_normal.y(), expected_normal.z()};
            std::vector<D455PointXYZ> object_cloud;
            IcpWallPlane wall_plane;
            IcpWallFilterStats wall_stats;
            if (!removeIcpWallOnce(
                    candidate_frame.point_cloud_color_m,
                    options.wall_filter, expected,
                    object_cloud, wall_plane, wall_stats, last_error)) {
                std::cout << "[ICP_EXEC] " << label
                          << " 墙面识别失败 attempt=" << attempt << "/"
                          << options.wall_estimation_attempts << ": "
                          << last_error << "; 重新取帧" << std::endl;
                continue;
            }
            std::cout << "[ICP_EXEC] " << label
                      << " 墙面识别成功: raw=" << wall_stats.input_points
                      << " estimate_5mm=" << wall_stats.estimate_points
                      << " removed_ratio=" << wall_stats.removed_ratio
                      << " normal_difference="
                      << wall_stats.normal_difference_deg
                      << "deg remaining=" << wall_stats.remaining_points
                      << std::endl;
            candidate_frame.point_cloud_color_m = std::move(object_cloud);
            frame = std::move(candidate_frame);
            return true;
        }
        capture_error = label + " 连续 " +
                        std::to_string(options.wall_estimation_attempts) +
                        " 次墙面识别失败，Task failed: " + last_error;
        return false;
    };

    struct InitialCandidate {
        int sample{};
        double y_offset_mm{};
        Alignment alignment;
        Eigen::Isometry3d target{Eigen::Isometry3d::Identity()};
        double translation_mm{};
        double rotation_deg{};
        double score{std::numeric_limits<double>::infinity()};
    };

    for (int iteration = 1;
         iteration <= options.correction_steps; ++iteration) {
        if (iteration == 1) {
            std::optional<InitialCandidate> best;
            std::string last_candidate_error;
            for (int sample = 0;
                 sample < options.initial_search_samples; ++sample) {
                const double ratio = static_cast<double>(sample) /
                    static_cast<double>(options.initial_search_samples - 1);
                const double y_offset_mm =
                    options.initial_search_y_start_mm +
                    (options.initial_search_y_end_mm -
                     options.initial_search_y_start_mm) * ratio;
                Eigen::Isometry3d observation = base_flange_reference;
                observation.translation().y() += y_offset_mm;
                bool ik_failed = false;
                if (!solveAndMove(
                        robot, observation, options,
                        "ICP 初值搜索 " + std::to_string(sample + 1) + "/" +
                            std::to_string(options.initial_search_samples) +
                            " Y_offset=" + std::to_string(y_offset_mm) + "mm",
                        error, &ik_failed)) {
                    if (!ik_failed) {
                        return false;
                    }
                    last_candidate_error = error;
                    std::cout << "[ICP_EXEC] 初值候选 " << sample + 1
                              << " Y_offset=" << y_offset_mm
                              << "mm 无可用 IK，跳到下一候选: "
                              << error << std::endl;
                    continue;
                }

                D455Frame frame;
                Eigen::Isometry3d base_flange_current;
                Eigen::Isometry3d initial;
                bool fatal_capture_failure = false;
                if (!captureObjectCloud(
                        "初值候选 " + std::to_string(sample + 1),
                        frame, base_flange_current, initial,
                        fatal_capture_failure, error)) {
                    if (fatal_capture_failure) {
                        return false;
                    }
                    last_candidate_error = error;
                    std::cout << "[ICP_EXEC] 初值候选 " << sample + 1
                              << " Y_offset=" << y_offset_mm
                              << "mm 无有效去墙点云，跳到下一候选: "
                              << error << std::endl;
                    continue;
                }

                Alignment alignment;
                std::string candidate_error;
                if (!alignPyramid(frame.point_cloud_color_m, levels, initial,
                                  options, alignment, candidate_error)) {
                    std::cout << "[ICP_EXEC] 初值候选 " << sample + 1
                              << " Y_offset=" << y_offset_mm
                              << "mm 匹配失败: " << candidate_error
                              << std::endl;
                    last_candidate_error = candidate_error;
                    continue;
                }
                const double translation_mm =
                    alignment.current_to_reference.translation().norm() *
                    1000.0;
                const double rotation_deg = std::abs(Eigen::AngleAxisd(
                    alignment.current_to_reference.linear()).angle()) *
                    180.0 / std::acos(-1.0);
                const double rmse_mm = alignment.rmse_m * 1000.0;
                if (!metricsAccepted(
                        alignment, translation_mm, rotation_deg)) {
                    std::cout << "[ICP_EXEC] 初值候选 " << sample + 1
                              << " Y_offset=" << y_offset_mm
                              << "mm 质量门限未通过: residual="
                              << translation_mm << "mm/" << rotation_deg
                              << "deg rmse=" << rmse_mm
                              << "mm inliers=" << alignment.inliers
                              << " overlap=" << alignment.overlap
                              << std::endl;
                    last_candidate_error =
                        "质量门限未通过: inliers=" +
                        std::to_string(alignment.inliers) +
                        " overlap=" + std::to_string(alignment.overlap) +
                        " rmse_mm=" + std::to_string(rmse_mm);
                    continue;
                }

                // 同时奖励低误差和高覆盖率，避免只凭很小一块重叠选中候选。
                const double score = rmse_mm / alignment.overlap;
                std::cout << "[ICP_EXEC] 初值候选 " << sample + 1
                          << " Y_offset=" << y_offset_mm
                          << "mm score=" << score
                          << " rmse=" << rmse_mm
                          << "mm overlap=" << alignment.overlap
                          << " inliers=" << alignment.inliers << std::endl;
                if (!best || score < best->score) {
                    best = InitialCandidate{
                        sample + 1,
                        y_offset_mm,
                        alignment,
                        correctionTarget(alignment, base_flange_current),
                        translation_mm,
                        rotation_deg,
                        score};
                }
            }
            if (!best) {
                error = "ICP +80mm 到 -20mm 的四个初值均无有效匹配，"
                        "跳过全部 Work";
                if (!last_candidate_error.empty()) {
                    error += "; 最后候选: " + last_candidate_error;
                }
                return false;
            }

            const double rmse_mm = best->alignment.rmse_m * 1000.0;
            result.iterations.push_back({
                iteration, best->translation_mm, best->rotation_deg,
                rmse_mm, best->alignment.inliers, best->alignment.overlap});
            std::cout << "[ICP_EXEC] 选中初值候选 " << best->sample
                      << " Y_offset=" << best->y_offset_mm
                      << "mm score=" << best->score << std::endl;
            if (iteration == options.correction_steps &&
                best->translation_mm >
                    options.final_maximum_residual_mm) {
                error = "ICP Ref 最后一轮残差 " +
                        std::to_string(best->translation_mm) + "mm > " +
                        std::to_string(options.final_maximum_residual_mm) +
                        "mm，Ref 未收敛，跳过全部 Work";
                return false;
            }
            if (!solveAndMove(robot, best->target, options,
                              "ICP correction loop=1 best_initial",
                              error)) {
                return false;
            }
            continue;
        }

        D455Frame frame;
        Eigen::Isometry3d base_flange_current;
        Eigen::Isometry3d initial;
        bool fatal_capture_failure = false;
        if (!captureObjectCloud(
                "loop=" + std::to_string(iteration), frame,
                base_flange_current, initial,
                fatal_capture_failure, error)) {
            return false;
        }
        std::cout << "[ICP_EXEC] loop=" << iteration
                  << " 丢弃新帧=" << options.discard_new_frames
                  << " object_points=" << frame.point_cloud_color_m.size()
                  << " timestamp_ms=" << frame.color_timestamp_ms << std::endl;

        Alignment alignment;
        if (!alignPyramid(frame.point_cloud_color_m, levels, initial,
                          options, alignment, error)) {
            error = "ICP Ref 第 " + std::to_string(iteration) +
                    " 轮匹配失败，跳过全部 Work: " + error;
            return false;
        }
        Eigen::Isometry3d correction_camera = alignment.current_to_reference;
        const double translation_mm =
            correction_camera.translation().norm() * 1000.0;
        const double rotation_deg =
            std::abs(Eigen::AngleAxisd(correction_camera.linear()).angle()) *
            180.0 / std::acos(-1.0);
        const double rmse_mm = alignment.rmse_m * 1000.0;
        result.iterations.push_back({
            iteration, translation_mm, rotation_deg, rmse_mm,
            alignment.inliers, alignment.overlap});
        std::cout << "[ICP_EXEC] loop=" << iteration
                  << " residual=" << translation_mm << "mm/"
                  << rotation_deg << "deg rmse=" << rmse_mm
                  << "mm inliers=" << alignment.inliers
                  << " overlap=" << alignment.overlap << std::endl;

        if (!metricsAccepted(alignment, translation_mm, rotation_deg)) {
            error = "ICP Ref 第 " + std::to_string(iteration) +
                    " 轮质量检查失败，跳过全部 Work";
            return false;
        }
        if (iteration == options.correction_steps &&
            translation_mm > options.final_maximum_residual_mm) {
            error = "ICP Ref 最后一轮残差 " +
                    std::to_string(translation_mm) + "mm > " +
                    std::to_string(options.final_maximum_residual_mm) +
                    "mm，Ref 未收敛，跳过全部 Work";
            return false;
        }
        const Eigen::Isometry3d target =
            correctionTarget(alignment, base_flange_current);
        if (!solveAndMove(robot, target, options,
                          "ICP correction loop=" + std::to_string(iteration),
                          error)) {
            return false;
        }
        // solveAndMove 已完成到点判断并连续稳定 0.5 秒；下一步再丢弃 3 帧。
    }
    result.corrections_completed = true;
    std::cout << "[ICP_EXEC] 固定 " << options.correction_steps
              << " 步视觉回正完成，最后一轮 Ref 残差 <= "
              << options.final_maximum_residual_mm << "mm" << std::endl;

    Eigen::Isometry3d corrected_reference;
    if (!readCurrentPose(robot, corrected_reference, error)) {
        return false;
    }
    for (const IcpWorkRecord &work : plan.works) {
        const Eigen::Isometry3d target =
            corrected_reference * flatTransform(work.trans);
        if (!solveAndMove(robot, target, options,
                          "ICP Work command=" + std::to_string(work.command),
                          error)) {
            return false;
        }
        result.completed_commands.push_back(work.command);
    }
    camera.stop();
    return true;
}
