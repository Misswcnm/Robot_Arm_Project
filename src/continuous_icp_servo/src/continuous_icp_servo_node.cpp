#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <fstream>
#include <future>
#include <iostream>
#include <mutex>
#include <regex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <sys/select.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"

#include "dobot_msgs_v4/msg/tool_vector_actual.hpp"
#include "dobot_msgs_v4/srv/clear_error.hpp"
#include "dobot_msgs_v4/srv/disable_robot.hpp"
#include "dobot_msgs_v4/srv/enable_robot.hpp"
#include "dobot_msgs_v4/srv/mov_j.hpp"
#include "dobot_msgs_v4/srv/servo_p.hpp"
#include "dobot_msgs_v4/srv/set_collision_level.hpp"
#include "dobot_msgs_v4/srv/speed_factor.hpp"
#include "dobot_msgs_v4/srv/stop.hpp"



#include "continuous_icp_servo/geometry.hpp"
#include "continuous_icp_servo/handeye_loader.hpp"
#include "continuous_icp_servo/icp_solver.hpp"
#include "continuous_icp_servo/point_cloud_utils.hpp"
#include "continuous_icp_servo/spatial_index.hpp"

using namespace std::chrono_literals;
using namespace continuous_icp_servo;

class ContinuousIcpServoNode : public rclcpp::Node
{
public:
  // 节点启动：加载参数/手眼矩阵，订阅点云和 TCP，启动 ICP 线程与 ServoP 定时器。
  ContinuousIcpServoNode()
  : Node("continuous_icp_servo")
  {
    pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "/camera/camera/depth/color/points");
    tool_topic_ = declare_parameter<std::string>("tool_topic", "/dobot_msgs_v4/msg/ToolVectorActual");
    joint_topic_ = declare_parameter<std::string>("joint_topic", "/joint_states_robot");
    command_topic_ = declare_parameter<std::string>("command_topic", "/continuous_icp_servo/command");
    handeye_path_ = declare_parameter<std::string>(
      "handeye_path", "/home/ylx/Robot_Arm_Project/scripts/handeye_chessboard_result.json");
    icp_hz_ = declare_parameter<double>("icp_hz", 6.0);
    servo_hz_ = declare_parameter<double>("servo_hz", 20.0);
    max_step_mm_ = declare_parameter<double>("max_step_mm", 1.0);
    max_step_deg_ = declare_parameter<double>("max_step_deg", 0.15);
    max_icp_age_s_ = declare_parameter<double>("max_icp_age_s", 2.0);
    min_icp_overlap_ = declare_parameter<double>("min_icp_overlap", 0.12);
    max_icp_rmse_mm_ = declare_parameter<double>("max_icp_rmse_mm", 25.0);
    max_icp_trans_mm_ = declare_parameter<double>("max_icp_trans_mm", 120.0);
    max_icp_rot_deg_ = declare_parameter<double>("max_icp_rot_deg", 10.0);
    speed_percent_ = declare_parameter<int>("speed_percent", 10);

    if (!loadHandeye(handeye_path_, X_)) {
      throw std::runtime_error("failed to load handeye matrix: " + handeye_path_);
    }
    X_inv_ = X_.inverse();

    pc_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    tool_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    joint_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    command_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions pc_options;
    pc_options.callback_group = pc_group_;
    rclcpp::SubscriptionOptions tool_options;
    tool_options.callback_group = tool_group_;
    rclcpp::SubscriptionOptions joint_options;
    joint_options.callback_group = joint_group_;
    rclcpp::SubscriptionOptions command_options;
    command_options.callback_group = command_group_;

    pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ContinuousIcpServoNode::onPointCloud, this, std::placeholders::_1), pc_options);
    tool_sub_ = create_subscription<dobot_msgs_v4::msg::ToolVectorActual>(
      tool_topic_, 10, std::bind(&ContinuousIcpServoNode::onTool, this, std::placeholders::_1), tool_options);
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_topic_, 10, std::bind(&ContinuousIcpServoNode::onJointState, this, std::placeholders::_1), joint_options);
    command_sub_ = create_subscription<std_msgs::msg::String>(
      command_topic_, 10, [this](const std_msgs::msg::String::SharedPtr msg) {
        handleCommand(msg->data);
      }, command_options);

    service_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    mov_j_ = create_client<dobot_msgs_v4::srv::MovJ>(
      "/dobot_bringup_ros2/srv/MovJ", rmw_qos_profile_services_default, service_group_);
    servo_p_ = create_client<dobot_msgs_v4::srv::ServoP>(
      "/dobot_bringup_ros2/srv/ServoP", rmw_qos_profile_services_default, service_group_);
    stop_ = create_client<dobot_msgs_v4::srv::Stop>(
      "/dobot_bringup_ros2/srv/Stop", rmw_qos_profile_services_default, service_group_);
    clear_error_ = create_client<dobot_msgs_v4::srv::ClearError>(
      "/dobot_bringup_ros2/srv/ClearError", rmw_qos_profile_services_default, service_group_);
    disable_robot_ = create_client<dobot_msgs_v4::srv::DisableRobot>(
      "/dobot_bringup_ros2/srv/DisableRobot", rmw_qos_profile_services_default, service_group_);
    enable_robot_ = create_client<dobot_msgs_v4::srv::EnableRobot>(
      "/dobot_bringup_ros2/srv/EnableRobot", rmw_qos_profile_services_default, service_group_);
    speed_factor_ = create_client<dobot_msgs_v4::srv::SpeedFactor>(
      "/dobot_bringup_ros2/srv/SpeedFactor", rmw_qos_profile_services_default, service_group_);
    collision_ = create_client<dobot_msgs_v4::srv::SetCollisionLevel>(
      "/dobot_bringup_ros2/srv/SetCollisionLevel", rmw_qos_profile_services_default, service_group_);

    initRobotAsync();

    const auto servo_period = std::chrono::duration<double>(1.0 / std::max(1.0, servo_hz_));
    servo_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(servo_period),
      std::bind(&ContinuousIcpServoNode::servoTick, this));

    icp_thread_ = std::thread([this]() { icpLoop(); });
    keyboard_thread_ = std::thread([this]() { keyboardLoop(); });

    RCLCPP_INFO(get_logger(), "连续 ICP ServoP 节点已启动");
    printMenu();
  }

  // 退出时关掉连续伺服并发送 Stop，避免节点结束后机械臂继续执行旧命令。
  ~ContinuousIcpServoNode() override
  {
    running_.store(false);
    servo_enabled_.store(false);
    sendStop();
    if (icp_thread_.joinable()) {
      icp_thread_.join();
    }
    if (keyboard_thread_.joinable() && keyboard_thread_.get_id() != std::this_thread::get_id()) {
      keyboard_thread_.join();
    }
  }

private:
  // ToolVectorActual 回调：保存最新 TCP 位姿，供 ICP 初值和 ServoP 当前位姿使用。
  void onTool(const dobot_msgs_v4::msg::ToolVectorActual::SharedPtr msg)
  {
    const bool finite =
      std::isfinite(msg->x) && std::isfinite(msg->y) && std::isfinite(msg->z) &&
      std::isfinite(msg->rx) && std::isfinite(msg->ry) && std::isfinite(msg->rz);
    const double pos_norm = std::sqrt(msg->x * msg->x + msg->y * msg->y + msg->z * msg->z);
    if (!finite || pos_norm < 1.0) {
      const int dropped = ++tool_drop_count_;
      if (dropped == 1 || dropped % 50 == 0) {
        RCLCPP_WARN(
          get_logger(), "丢弃异常 ToolVectorActual xyz=[%.3f %.3f %.3f] rpy=[%.3f %.3f %.3f]",
          msg->x, msg->y, msg->z, msg->rx, msg->ry, msg->rz);
      }
      return;
    }

    std::lock_guard<std::mutex> lk(tool_mtx_);
    latest_tool_.x = msg->x;
    latest_tool_.y = msg->y;
    latest_tool_.z = msg->z;
    latest_tool_.rx = msg->rx;
    latest_tool_.ry = msg->ry;
    latest_tool_.rz = msg->rz;
    latest_tool_.stamp = now();
    latest_tool_.valid = true;
    const int count = ++tool_msg_count_;
    if (count == 1) {
      RCLCPP_INFO(
        get_logger(), "收到 ToolVectorActual xyz=[%.1f %.1f %.1f] rpy=[%.1f %.1f %.1f]",
        msg->x, msg->y, msg->z, msg->rx, msg->ry, msg->rz);
    }
  }

  // JointState 回调：持续维护最新关节角缓存；驱动发布弧度，MovJ 服务使用角度。
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.size() < 6) {
      return;
    }
    JointPose next;
    double abs_sum = 0.0;
    for (size_t i = 0; i < 6; ++i) {
      if (!std::isfinite(msg->position[i])) {
        return;
      }
      next.q_deg[i] = msg->position[i] * 180.0 / M_PI;
      abs_sum += std::abs(msg->position[i]);
    }
    if (abs_sum < 1e-6) {
      const int dropped = ++joint_drop_count_;
      if (dropped == 1 || dropped % 50 == 0) {
        RCLCPP_WARN(get_logger(), "丢弃疑似未连接的全零 JointState");
      }
      return;
    }
    next.stamp = now();
    next.valid = true;
    {
      std::lock_guard<std::mutex> lk(joint_mtx_);
      latest_joint_ = next;
    }
    const int count = ++joint_msg_count_;
    if (count == 1) {
      RCLCPP_INFO(
        get_logger(), "收到 JointState deg=[%.1f %.1f %.1f %.1f %.1f %.1f]",
        next.q_deg[0], next.q_deg[1], next.q_deg[2], next.q_deg[3], next.q_deg[4], next.q_deg[5]);
    }
  }

  // 点云回调只做轻处理：解析 xyz、去 NaN、5mm 下采样，然后保存最新帧。
  void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    Cloud pts;
    pts.reserve(msg->width * msg->height / 4);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const double x = *iter_x;
      const double y = *iter_y;
      const double z = *iter_z;
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
        pts.emplace_back(x, y, z);
      }
    }
    pts = voxelDown(pts, 0.005);
    std::lock_guard<std::mutex> lk(pc_mtx_);
    latest_pc_ = std::move(pts);
    latest_pc_stamp_ = now();
  }

  // 原子式取当前点云和 TCP 快照；返回失败时给出具体原因，便于现场排查 topic/pose。
  bool snapshot(Cloud & pc, ToolPose & tool, std::string * reason = nullptr)
  {
    rclcpp::Time pc_stamp;
    {
      std::lock_guard<std::mutex> lk(pc_mtx_);
      pc = latest_pc_;
      pc_stamp = latest_pc_stamp_;
    }
    {
      std::lock_guard<std::mutex> lk(tool_mtx_);
      tool = latest_tool_;
    }
    const bool has_pc_stamp = pc_stamp.nanoseconds() > 0;
    const bool has_tool_stamp = tool.stamp.nanoseconds() > 0;
    const double pc_age = has_pc_stamp ? (now() - pc_stamp).seconds() : 999.0;
    const double tool_age = has_tool_stamp ? (now() - tool.stamp).seconds() : 999.0;
    const bool pc_ok = pc.size() > 500 && has_pc_stamp && pc_age < 2.0;
    const bool tool_ok = tool.valid && has_tool_stamp && tool_age < 2.0;
    if (reason && (!pc_ok || !tool_ok)) {
      *reason =
        "pc_pts=" + std::to_string(pc.size()) +
        " pc_age=" + std::to_string(pc_age) + "s" +
        " tool_valid=" + std::string(tool.valid ? "true" : "false") +
        " tool_age=" + std::to_string(tool_age) + "s" +
        " tool_xyz=[" + std::to_string(tool.x) + "," + std::to_string(tool.y) + "," + std::to_string(tool.z) + "]";
    }
    return pc_ok && tool_ok;
  }

  // 录制参考帧：融合 5 帧点云，建立 20/10/5mm 三层模板金字塔和空间索引。
  void recordTemplate()
  {
    RCLCPP_INFO(get_logger(), "recording template: need 5 frames, wait up to 6s");
    std::vector<Cloud> frames;
    ToolPose ref_tool;
    const auto deadline = std::chrono::steady_clock::now() + 6s;
    int attempts = 0;
    while (frames.size() < 5 && rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      Cloud pc;
      ToolPose tool;
      std::string reason;
      if (snapshot(pc, tool, &reason)) {
        frames.push_back(pc);
        ref_tool = tool;
        RCLCPP_INFO(get_logger(), "  frame %zu: %zu pts", frames.size(), pc.size());
      } else {
        if ((attempts % 5) == 0) {
          RCLCPP_WARN(get_logger(), "  waiting frame: %s", reason.c_str());
        }
      }
      attempts++;
      std::this_thread::sleep_for(150ms);
    }
    if (frames.size() < 2) {
      RCLCPP_ERROR(get_logger(), "template failed: only %zu frames, need at least 2", frames.size());
      return;
    }

    Cloud merged;
    for (const auto & f : frames) {
      merged.insert(merged.end(), f.begin(), f.end());
    }
    const Cloud ref_5mm = voxelDown(merged, 0.005);

    std::vector<PyramidLevel> pyr;
    for (const auto & cfg : std::vector<std::pair<double, double>>{{0.020, 0.100}, {0.010, 0.050}, {0.005, 0.025}}) {
      PyramidLevel level;
      level.voxel = cfg.first;
      level.dmax = cfg.second;
      level.ref = cfg.first <= 0.005 ? ref_5mm : voxelDown(ref_5mm, cfg.first);
      level.index.build(level.ref, level.dmax);
      RCLCPP_INFO(
        get_logger(), "  L%zu: %zu pts voxel=%.0fmm dmax=%.0fmm",
        pyr.size(), level.ref.size(), level.voxel * 1000.0, level.dmax * 1000.0);
      pyr.push_back(std::move(level));
    }

    {
      std::lock_guard<std::mutex> lk(template_mtx_);
      ref_pyramid_ = std::move(pyr);
      T_base_tool_ref_ = poseToMatrixMm(ref_tool);
      T_base_camera_ref_ = T_base_tool_ref_ * X_;
      template_ready_ = true;
    }
    RCLCPP_INFO(
      get_logger(), "模板已就绪 ref xyz=[%.1f %.1f %.1f]mm",
      ref_tool.x, ref_tool.y, ref_tool.z);
    RCLCPP_INFO(get_logger(), "下一步：按 1-6 先偏移离开 ref，确认离开后按 s 开始连续伺服。");
    printMenu();
  }

  // 低频 ICP 工作线程；只更新最新视觉误差，不直接下发运动命令。
  void icpLoop()
  {
    const auto period = std::chrono::duration<double>(1.0 / std::max(0.5, icp_hz_));
    while (running_.load() && rclcpp::ok()) {
      const auto t0 = std::chrono::steady_clock::now();
      if (template_ready_.load()) {
        computeIcpOnce();
      }
      const auto elapsed = std::chrono::steady_clock::now() - t0;
      if (elapsed < period) {
        std::this_thread::sleep_for(period - elapsed);
      }
    }
  }

  // 执行一次金字塔 ICP：用机器人当前位姿给初值，再由粗到精修正 current -> ref。
  void computeIcpOnce()
  {
    Cloud pc;
    ToolPose tool;
    if (!snapshot(pc, tool)) {
      return;
    }

    std::vector<PyramidLevel> pyr;
    Mat4 T_base_camera_ref;
    {
      std::lock_guard<std::mutex> lk(template_mtx_);
      pyr = ref_pyramid_;
      T_base_camera_ref = T_base_camera_ref_;
    }

    Mat4 T_base_tool_cur = poseToMatrixMm(tool);
    Mat4 T_base_camera_cur = T_base_tool_cur * X_;
    Mat4 T_init_mm = T_base_camera_ref.inverse() * T_base_camera_cur;
    Mat4 T_acc = T_init_mm;
    T_acc.block<3, 1>(0, 3) /= 1000.0;

    double final_rmse = 999.0;
    int final_inliers = 0;
    double final_overlap = 0.0;

    for (const auto & level : pyr) {
      Cloud src_ds = voxelDown(pc, level.voxel);
      std::vector<Point> matched_src;
      std::vector<Point> matched_dst;
      matched_src.reserve(src_ds.size());
      matched_dst.reserve(src_ds.size());

      const Mat3 R_acc = T_acc.block<3, 3>(0, 0);
      const Point t_acc = T_acc.block<3, 1>(0, 3);
      for (const auto & p : src_ds) {
        const Point p_tf = R_acc * p + t_acc;
        Point q;
        double dist = 0.0;
        if (level.index.nearest(p_tf, level.dmax, q, dist)) {
          matched_src.push_back(p_tf);
          matched_dst.push_back(q);
        }
      }

      final_inliers = static_cast<int>(matched_src.size());
      final_overlap = src_ds.empty() ? 0.0 : static_cast<double>(final_inliers) / static_cast<double>(src_ds.size());
      if (final_inliers < 20) {
        continue;
      }

      Mat3 R_step;
      Point t_step;
      double rmse = 999.0;
      if (!kabschStep(matched_src, matched_dst, R_step, t_step, rmse)) {
        continue;
      }
      T_acc.block<3, 3>(0, 0) = R_step * T_acc.block<3, 3>(0, 0);
      T_acc.block<3, 1>(0, 3) = R_step * T_acc.block<3, 1>(0, 3) + t_step;
      final_rmse = rmse;
    }

    Mat4 T_icp_mm = T_acc;
    T_icp_mm.block<3, 1>(0, 3) *= 1000.0;
    const double trans_mm = T_icp_mm.block<3, 1>(0, 3).norm();
    const double rot_deg = Eigen::AngleAxisd(T_icp_mm.block<3, 3>(0, 0)).angle() * 180.0 / M_PI;

    IcpResult result;
    result.T_icp_mm = T_icp_mm;
    result.rmse_mm = final_rmse * 1000.0;
    result.overlap = final_overlap;
    result.inliers = final_inliers;
    result.seq = ++icp_seq_counter_;
    result.ok =
      final_inliers >= 500 &&
      result.overlap >= min_icp_overlap_ &&
      result.rmse_mm <= max_icp_rmse_mm_ &&
      trans_mm <= max_icp_trans_mm_ &&
      rot_deg <= max_icp_rot_deg_;
    result.stamp = now();

    {
      std::lock_guard<std::mutex> lk(icp_mtx_);
      latest_icp_ = result;
    }

    static int log_skip = 0;
    const bool should_log = servo_enabled_.load() && ((log_skip++ % 10) == 0 || !result.ok);
    if (should_log) {
      RCLCPP_INFO(
        get_logger(), "伺服中 ICP %s  RMSE=%.1fmm  匹配点=%d  overlap=%.2f  |t|=%.1fmm  |r|=%.2fdeg",
        result.ok ? "正常" : "异常", result.rmse_mm, result.inliers, result.overlap, trans_mm, rot_deg);
    }
  }

  // 高频 ServoP 定时器：读取最新 ICP 结果，限幅后发送一个很近的笛卡尔目标。
  // 若上一条 ServoP 还没返回，本周期跳过，避免服务请求和队列目标堆积。
  void servoTick()
  {
    if (!servo_enabled_.load() || !template_ready_.load()) {
      return;
    }
    if (!servo_p_->service_is_ready()) {
      static int no_service_log_skip = 0;
      if ((no_service_log_skip++ % 20) == 0) {
        RCLCPP_WARN(get_logger(), "连续伺服跳过：ServoP 服务未就绪");
      }
      return;
    }
    if (servoBusyOrRecover("连续伺服")) {
      missed_servo_cycles_.fetch_add(1);
      return;
    }

    IcpResult icp;
    {
      std::lock_guard<std::mutex> lk(icp_mtx_);
      icp = latest_icp_;
    }
    const double icp_age_s = (now() - icp.stamp).seconds();
    if (!icp.ok || icp_age_s > max_icp_age_s_) {
      const int bad_cycles = bad_icp_cycles_.fetch_add(1) + 1;
      static int bad_icp_log_skip = 0;
      if ((bad_icp_log_skip++ % 20) == 0) {
        RCLCPP_WARN(
          get_logger(),
          "连续伺服跳过：ICP %s age=%.2fs max_age=%.2fs rmse=%.1f/%.1fmm overlap=%.2f/%.2f inliers=%d",
          icp.ok ? "过期" : "质量不合格", icp_age_s, max_icp_age_s_,
          icp.rmse_mm, max_icp_rmse_mm_, icp.overlap, min_icp_overlap_, icp.inliers);
      }
      if (bad_cycles > static_cast<int>(std::max(1.0, servo_hz_) * 1.5)) {
        servo_enabled_.store(false);
        sendStop();
        RCLCPP_WARN(get_logger(), "连续伺服已停止：ICP 结果连续 1.5s 过期或质量不合格");
      }
      return;
    }
    bad_icp_cycles_.store(0);
    if (icp.seq == last_consumed_icp_seq_.load()) {
      return;
    }

    ToolPose tool;
    {
      std::lock_guard<std::mutex> lk(tool_mtx_);
      tool = latest_tool_;
    }
    if (!tool.valid || (now() - tool.stamp).seconds() > 0.25) {
      static int bad_tool_log_skip = 0;
      if ((bad_tool_log_skip++ % 20) == 0) {
        RCLCPP_WARN(
          get_logger(), "连续伺服等待：ToolVectorActual %s age=%.2fs",
          tool.valid ? "过期" : "无效", (now() - tool.stamp).seconds());
      }
      return;
    }

    const Mat4 T_cur = poseToMatrixMm(tool);
    // T_delta = T_ref_tool^-1 * T_cur_tool；右乘到当前位姿时要取逆才能得到 T_cur^-1 * T_ref。
    const Mat4 T_delta = X_ * icp.T_icp_mm * X_inv_;
    const Mat4 T_corr = T_delta.inverse();

    Point step_t = T_corr.block<3, 1>(0, 3);
    const double t_norm = step_t.norm();
    if (t_norm > max_step_mm_) {
      step_t *= max_step_mm_ / t_norm;
    }

    Eigen::AngleAxisd aa(T_corr.block<3, 3>(0, 0));
    double angle = aa.angle();
    const double max_angle = max_step_deg_ * M_PI / 180.0;
    if (std::abs(angle) > max_angle) {
      angle = std::copysign(max_angle, angle);
    }
    Mat4 T_step = Mat4::Identity();
    if (std::abs(angle) > 1e-6 && std::isfinite(aa.axis().x()) && std::isfinite(aa.axis().y()) && std::isfinite(aa.axis().z())) {
      T_step.block<3, 3>(0, 0) = Eigen::AngleAxisd(angle, aa.axis()).toRotationMatrix();
    }
    T_step.block<3, 1>(0, 3) = step_t;

    const Mat4 T_target = T_cur * T_step;
    const Point xyz = T_target.block<3, 1>(0, 3);
    const Point rpy = matrixToXyzDeg(T_target.block<3, 3>(0, 0));

    auto req = std::make_shared<dobot_msgs_v4::srv::ServoP::Request>();
    req->a = xyz.x();
    req->b = xyz.y();
    req->c = xyz.z();
    req->d = rpy.x();
    req->e = rpy.y();
    req->f = rpy.z();
    req->param_value.clear();

    uint64_t expected_seq = last_consumed_icp_seq_.load();
    if (expected_seq == icp.seq ||
        !last_consumed_icp_seq_.compare_exchange_strong(expected_seq, icp.seq)) {
      return;
    }
    servo_inflight_.store(true);
    servo_inflight_since_ms_.store(steadyMs());
    static int servo_send_log_skip = 0;
    if ((servo_send_log_skip++ % 20) == 0) {
      RCLCPP_INFO(
        get_logger(), "连续伺服已发送 ServoP xyz=[%.1f %.1f %.1f] rpy=[%.1f %.1f %.1f] step=[%.2f %.2f %.2f]mm",
        xyz.x(), xyz.y(), xyz.z(), rpy.x(), rpy.y(), rpy.z(), step_t.x(), step_t.y(), step_t.z());
    }
    servo_p_->async_send_request(
      req,
      [this](rclcpp::Client<dobot_msgs_v4::srv::ServoP>::SharedFuture future) {
        const auto res = future.get();
        if (res->res != 0) {
          RCLCPP_WARN(this->get_logger(), "连续伺服 ServoP 返回异常 res=%d", res->res);
        }
        servo_inflight_.store(false);
        servo_inflight_since_ms_.store(0);
      });
  }

  // 只负责下发、不关心返回；用于 Stop/ClearError 等低风险控制命令。
  template<class ClientT, class RequestT>
  void fireAndForget(const typename ClientT::SharedPtr & client, const std::shared_ptr<RequestT> & req)
  {
    if (client->service_is_ready()) {
      client->async_send_request(req);
    }
  }

  // 下发并打印返回码；用于启动时确认限速和碰撞等级是否真正被控制器接收。
  template<class ClientT, class RequestT>
  void sendAndLog(
    const typename ClientT::SharedPtr & client,
    const std::shared_ptr<RequestT> & req,
    const char * name)
  {
    if (!client->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "%s service not ready; command skipped", name);
      return;
    }
    client->async_send_request(
      req,
      [this, name](typename ClientT::SharedFuture future) {
        RCLCPP_INFO(this->get_logger(), "%s res=%d", name, future.get()->res);
      });
  }

  // 异步初始化机械臂：清错、重新使能、限速、设置碰撞等级；不阻塞 ROS 回调线程。
  void initRobotAsync()
  {
    std::thread([this]() {
      waitClient(clear_error_, "ClearError");
      waitClient(disable_robot_, "DisableRobot");
      waitClient(enable_robot_, "EnableRobot");
      waitClient(mov_j_, "MovJ");
      waitClient(speed_factor_, "SpeedFactor");
      waitClient(collision_, "SetCollisionLevel");
      RCLCPP_INFO(get_logger(), "初始化序列：ClearError -> DisableRobot -> EnableRobot -> SpeedFactor -> SetCollisionLevel");
      initClearError();
    }).detach();
  }

  // 初始化第1步：清错。收到 response 后再发下一条，避免 future.wait_for 收不到回包。
  void initClearError()
  {
    auto req = std::make_shared<dobot_msgs_v4::srv::ClearError::Request>();
    clear_error_->async_send_request(
      req,
      [this](rclcpp::Client<dobot_msgs_v4::srv::ClearError>::SharedFuture future) {
        RCLCPP_INFO(this->get_logger(), "ClearError res=%d", future.get()->res);
        initDisableRobot();
      });
  }

  // 初始化第2步：下使能。CR5 旧流程要求 Disable 后再 Enable，避免状态残留。
  void initDisableRobot()
  {
    auto req = std::make_shared<dobot_msgs_v4::srv::DisableRobot::Request>();
    disable_robot_->async_send_request(
      req,
      [this](rclcpp::Client<dobot_msgs_v4::srv::DisableRobot>::SharedFuture future) {
        RCLCPP_INFO(this->get_logger(), "DisableRobot res=%d", future.get()->res);
        initEnableRobot();
      });
  }

  // 初始化第3步：重新使能。Enable 返回后再设置速度和碰撞等级。
  void initEnableRobot()
  {
    auto req = std::make_shared<dobot_msgs_v4::srv::EnableRobot::Request>();
    enable_robot_->async_send_request(
      req,
      [this](rclcpp::Client<dobot_msgs_v4::srv::EnableRobot>::SharedFuture future) {
        RCLCPP_INFO(this->get_logger(), "EnableRobot res=%d", future.get()->res);
        initSpeedFactor();
      });
  }

  // 初始化第4步：全局限速。
  void initSpeedFactor()
  {
    auto req = std::make_shared<dobot_msgs_v4::srv::SpeedFactor::Request>();
    req->ratio = speed_percent_;
    speed_factor_->async_send_request(
      req,
      [this](rclcpp::Client<dobot_msgs_v4::srv::SpeedFactor>::SharedFuture future) {
        RCLCPP_INFO(this->get_logger(), "SpeedFactor res=%d", future.get()->res);
        initCollisionLevel();
      });
  }

  // 初始化第5步：碰撞等级 5。
  void initCollisionLevel()
  {
    auto req = std::make_shared<dobot_msgs_v4::srv::SetCollisionLevel::Request>();
    req->level = 5;
    collision_->async_send_request(
      req,
      [this](rclcpp::Client<dobot_msgs_v4::srv::SetCollisionLevel>::SharedFuture future) {
        RCLCPP_INFO(this->get_logger(), "SetCollisionLevel res=%d", future.get()->res);
        RCLCPP_INFO(this->get_logger(), "初始化完成：speed=%d collision=5", speed_percent_);
      });
  }

  // 等待服务短暂上线；超时只报警，避免节点启动被驱动状态永久卡住。
  template<class ClientPtr>
  void waitClient(const ClientPtr & client, const char * name)
  {
    const auto t0 = std::chrono::steady_clock::now();
    while (running_.load() && rclcpp::ok() && !client->service_is_ready()) {
      if (std::chrono::steady_clock::now() - t0 > 5s) {
        RCLCPP_WARN(get_logger(), "service not ready: %s", name);
        return;
      }
      std::this_thread::sleep_for(100ms);
    }
  }

  // 统一停止入口；键盘停止、异常停止、析构时都会调用。
  void sendStop()
  {
    auto req = std::make_shared<dobot_msgs_v4::srv::Stop::Request>();
    fireAndForget<rclcpp::Client<dobot_msgs_v4::srv::Stop>, dobot_msgs_v4::srv::Stop::Request>(stop_, req);
    servo_inflight_.store(false);
    servo_inflight_since_ms_.store(0);
    joint_move_inflight_.store(false);
    joint_move_since_ms_.store(0);
  }

  // CR5 关键关节限位预检查；控制器仍是最终保护，这里用于提前给出清晰现场日志。
  bool checkJointLimits(const std::array<double, 6> & joints, std::string & reason)
  {
    for (size_t i = 0; i < joints.size(); ++i) {
      if (!std::isfinite(joints[i])) {
        reason = "J" + std::to_string(i + 1) + " 非有限值";
        return false;
      }
    }
    const std::array<std::pair<double, double>, 6> limits{{
      {-360.0, 360.0}, {-160.0, 160.0}, {-160.0, 160.0},
      {-360.0, 360.0}, {-180.0, 180.0}, {-360.0, 360.0}}};
    for (size_t i = 0; i < joints.size(); ++i) {
      if (joints[i] < limits[i].first || joints[i] > limits[i].second) {
        reason =
          "J" + std::to_string(i + 1) + "=" + std::to_string(joints[i]) +
          "deg 超出 [" + std::to_string(limits[i].first) + "," +
          std::to_string(limits[i].second) + "]deg";
        return false;
      }
    }
    return true;
  }

  // 用可靠的 JointMovJ 做离开 ref 的偏移；ServoP 在本机已验证会触发 ERROR，不用于偏移键。
  void perturbJoints(const JointDelta & d, const std::string & label)
  {
    servo_enabled_.store(false);
    if (!template_ready_.load()) {
      RCLCPP_WARN(get_logger(), "%s：请先输入 r 录制 ref 模板，再用 1-6 偏移离开 ref。", label.c_str());
      return;
    }
    if (!mov_j_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "%s：MovJ 服务未就绪，不能偏移", label.c_str());
      return;
    }
    if (jointBusyOrRecover(label)) {
      RCLCPP_WARN(get_logger(), "%s：上一条关节偏移还在处理中，本次跳过", label.c_str());
      return;
    }

    JointPose joint;
    {
      std::lock_guard<std::mutex> lk(joint_mtx_);
      joint = latest_joint_;
    }
    const double joint_age_s = joint.stamp.nanoseconds() > 0 ? (now() - joint.stamp).seconds() : 999.0;
    if (!joint.valid || joint_age_s > 0.5) {
      RCLCPP_WARN(
        get_logger(), "%s：JointState %s age=%.2fs，不能偏移；请确认 /joint_states_robot 正在发布",
        label.c_str(), joint.valid ? "过期" : "无效", joint_age_s);
      return;
    }

    joint_move_inflight_.store(true);
    joint_move_since_ms_.store(steadyMs());

    std::array<double, 6> joints = joint.q_deg;
    joints[0] += d.j1;
    joints[1] += d.j2;
    joints[2] += d.j3;
    joints[3] += d.j4;
    joints[4] += d.j5;
    joints[5] += d.j6;
    std::string limit_reason;
    if (!checkJointLimits(joints, limit_reason)) {
      RCLCPP_WARN(get_logger(), "%s：目标关节超限，取消偏移：%s", label.c_str(), limit_reason.c_str());
      joint_move_inflight_.store(false);
      joint_move_since_ms_.store(0);
      return;
    }

    auto mov = std::make_shared<dobot_msgs_v4::srv::MovJ::Request>();
    mov->mode = true;
    mov->a = joints[0];
    mov->b = joints[1];
    mov->c = joints[2];
    mov->d = joints[3];
    mov->e = joints[4];
    mov->f = joints[5];
    mov->param_value = {"user=0", "tool=0"};

    RCLCPP_WARN(
      get_logger(), "%s：只执行 JointMovJ 偏移，不开启 ICP 伺服。ΔJ=[%.1f %.1f %.1f %.1f %.1f %.1f]deg",
      label.c_str(), d.j1, d.j2, d.j3, d.j4, d.j5, d.j6);
    RCLCPP_INFO(
      get_logger(), "%s：使用缓存 JointState age=%.2fs，已发送 JointMovJ 目标 J=[%.2f %.2f %.2f %.2f %.2f %.2f]deg",
      label.c_str(), joint_age_s, joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
    mov_j_->async_send_request(
      mov,
      [this, label](rclcpp::Client<dobot_msgs_v4::srv::MovJ>::SharedFuture mov_future) {
        const auto mov_res = mov_future.get();
        RCLCPP_INFO(this->get_logger(), "%s：JointMovJ 返回 res=%d", label.c_str(), mov_res->res);
        joint_move_inflight_.store(false);
        joint_move_since_ms_.store(0);
      });
  }

  // 关节偏移也做超时解锁；如果 ROS service 回调丢了，下一次按键不会永久卡死。
  bool jointBusyOrRecover(const std::string & label)
  {
    if (!joint_move_inflight_.load()) {
      return false;
    }
    const int64_t age_ms = steadyMs() - joint_move_since_ms_.load();
    if (age_ms > 3000) {
      RCLCPP_WARN(get_logger(), "%s：上一条关节偏移 %.1fs 未收到 ROS 回调，自动解锁", label.c_str(), age_ms / 1000.0);
      joint_move_inflight_.store(false);
      joint_move_since_ms_.store(0);
      return false;
    }
    return true;
  }

  // ServoP 没返回时避免继续堆命令；超过 2s 认为返回丢失，解锁后允许下一条命令。
  bool servoBusyOrRecover(const std::string & label)
  {
    if (!servo_inflight_.load()) {
      return false;
    }
    const int64_t age_ms = steadyMs() - servo_inflight_since_ms_.load();
    if (age_ms > 2000) {
      RCLCPP_WARN(get_logger(), "%s：上一条 ServoP %.1fs 未返回，自动解锁", label.c_str(), age_ms / 1000.0);
      servo_inflight_.store(false);
      servo_inflight_since_ms_.store(0);
      return false;
    }
    return true;
  }

  // 控制台中文菜单；每次启动、录模板、未知命令时打印，现场不靠记忆操作。
  void printMenu()
  {
    RCLCPP_INFO(get_logger(), "\n========== 连续 ICP ServoP 控制 ==========\n"
      "r : 录制 ref 模板\n"
      "1 : 小负向 JointMovJ 偏移   2 : 中负向 JointMovJ 偏移   3 : 大负向 JointMovJ 偏移\n"
      "4 : 小正向 JointMovJ 偏移   5 : 中正向 JointMovJ 偏移   6 : 大正向 JointMovJ 偏移\n"
      "s : 开启/关闭连续伺服回 ref\n"
      "x : 停止运动\n"
      "q : 退出\n"
      "提示：输入 1-6 只做可靠关节偏移，不会开启 ICP 伺服；ServoP 仅用于 s 连续伺服实验。\n"
      "========================================");
  }

  // 发送一个绝对 ServoP 目标；偏移测试和连续伺服都复用这个打包逻辑。
  void sendServoPTarget(const Mat4 & T_target, const std::string & label)
  {
    if (!servo_p_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "%s：ServoP 服务未就绪，未下发目标", label.c_str());
      return;
    }
    if (servoBusyOrRecover(label)) {
      RCLCPP_WARN(get_logger(), "%s：上一条 ServoP 还未返回，本次跳过", label.c_str());
      return;
    }

    const Point xyz = T_target.block<3, 1>(0, 3);
    const Point rpy = matrixToXyzDeg(T_target.block<3, 3>(0, 0));
    auto req = std::make_shared<dobot_msgs_v4::srv::ServoP::Request>();
    req->a = xyz.x();
    req->b = xyz.y();
    req->c = xyz.z();
    req->d = rpy.x();
    req->e = rpy.y();
    req->f = rpy.z();
    req->param_value.clear();

    servo_inflight_.store(true);
    servo_inflight_since_ms_.store(steadyMs());
    RCLCPP_INFO(
      get_logger(), "%s：已下发 6 参数 ServoP 目标 xyz=[%.1f %.1f %.1f] rpy=[%.1f %.1f %.1f]",
      label.c_str(), xyz.x(), xyz.y(), xyz.z(), rpy.x(), rpy.y(), rpy.z());
    servo_p_->async_send_request(
      req,
      [this, label](rclcpp::Client<dobot_msgs_v4::srv::ServoP>::SharedFuture future) {
        const auto res = future.get();
        RCLCPP_INFO(this->get_logger(), "%s：ServoP 返回 res=%d", label.c_str(), res->res);
        servo_inflight_.store(false);
        servo_inflight_since_ms_.store(0);
      });
  }

  // 统一处理键盘和 topic 命令；topic 可用: ros2 topic pub /continuous_icp_servo/command std_msgs/msg/String "{data: r}"。
  void handleCommand(std::string cmd)
  {
    cmd.erase(
      std::remove_if(cmd.begin(), cmd.end(), [](unsigned char c) { return std::isspace(c); }),
      cmd.end());
    if (!cmd.empty()) {
      RCLCPP_INFO(get_logger(), "收到命令：%s", cmd.c_str());
    }
    if (cmd == "r") {
      servo_enabled_.store(false);
      RCLCPP_INFO(get_logger(), "开始录制模板，请保持机械臂和场景稳定。");
      recordTemplate();
    } else if (cmd == "s") {
      if (!template_ready_.load()) {
        RCLCPP_WARN(get_logger(), "还没有模板。请先输入 r 录制 ref 模板。");
        return;
      }
      const bool next = !servo_enabled_.load();
      servo_enabled_.store(next);
      if (next) {
        bad_icp_cycles_.store(0);
        missed_servo_cycles_.store(0);
        servo_inflight_.store(false);
        servo_inflight_since_ms_.store(0);
        last_consumed_icp_seq_.store(0);
        RCLCPP_INFO(
          get_logger(), "连续伺服准备：ServoP服务=%s ICP阈值 age=%.2fs rmse<=%.1fmm overlap>=%.2f rot<=%.1fdeg",
          servo_p_->service_is_ready() ? "ready" : "not ready",
          max_icp_age_s_, max_icp_rmse_mm_, min_icp_overlap_, max_icp_rot_deg_);
      }
      RCLCPP_WARN(get_logger(), "连续 ICP 伺服：%s", next ? "开启" : "关闭");
      if (!next) {
        sendStop();
      }
    } else if (cmd == "x") {
      servo_enabled_.store(false);
      sendStop();
      RCLCPP_WARN(get_logger(), "已发送 Stop()，连续伺服关闭。");
    } else if (cmd == "q") {
      servo_enabled_.store(false);
      running_.store(false);
      sendStop();
      rclcpp::shutdown();
    } else if (cmd == "1") {
      perturbJoints({-5, 0, 0, -3, 0, 0}, "小负向偏移");
    } else if (cmd == "2") {
      perturbJoints({-10, -5, 0, -5, -3, 0}, "中负向偏移");
    } else if (cmd == "3") {
      perturbJoints({-15, -8, 0, -8, -5, -3}, "大负向偏移");
    } else if (cmd == "4") {
      perturbJoints({5, 0, 0, 3, 0, 0}, "小正向偏移");
    } else if (cmd == "5") {
      perturbJoints({10, 5, 0, 5, 3, 0}, "中正向偏移");
    } else if (cmd == "6") {
      perturbJoints({15, 8, 0, 8, 5, 3}, "大正向偏移");
    } else if (!cmd.empty()) {
      printMenu();
    }
  }

  // 简单键盘控制；run 脚本用 ros2 run 保证 stdin 直连，launch 时可改用 command topic。
  void keyboardLoop()
  {
    while (running_.load() && rclcpp::ok()) {
      fd_set readfds;
      FD_ZERO(&readfds);
      FD_SET(STDIN_FILENO, &readfds);
      timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 100000;
      const int ready = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);
      if (ready <= 0 || !FD_ISSET(STDIN_FILENO, &readfds)) {
        continue;
      }

      std::string cmd;
      if (!std::getline(std::cin, cmd)) {
        std::this_thread::sleep_for(100ms);
        continue;
      }
      handleCommand(cmd);
    }
  }

  std::string pointcloud_topic_;
  std::string tool_topic_;
  std::string joint_topic_;
  std::string command_topic_;
  std::string handeye_path_;
  double icp_hz_{6.0};
  double servo_hz_{20.0};
  double max_step_mm_{1.0};
  double max_step_deg_{0.15};
  double max_icp_age_s_{2.0};
  double min_icp_overlap_{0.12};
  double max_icp_rmse_mm_{25.0};
  double max_icp_trans_mm_{120.0};
  double max_icp_rot_deg_{10.0};
  int speed_percent_{10};

  Mat4 X_{Mat4::Identity()};
  Mat4 X_inv_{Mat4::Identity()};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Subscription<dobot_msgs_v4::msg::ToolVectorActual>::SharedPtr tool_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::CallbackGroup::SharedPtr pc_group_;
  rclcpp::CallbackGroup::SharedPtr tool_group_;
  rclcpp::CallbackGroup::SharedPtr joint_group_;
  rclcpp::CallbackGroup::SharedPtr command_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::Client<dobot_msgs_v4::srv::MovJ>::SharedPtr mov_j_;
  rclcpp::Client<dobot_msgs_v4::srv::ServoP>::SharedPtr servo_p_;
  rclcpp::Client<dobot_msgs_v4::srv::Stop>::SharedPtr stop_;
  rclcpp::Client<dobot_msgs_v4::srv::ClearError>::SharedPtr clear_error_;
  rclcpp::Client<dobot_msgs_v4::srv::DisableRobot>::SharedPtr disable_robot_;
  rclcpp::Client<dobot_msgs_v4::srv::EnableRobot>::SharedPtr enable_robot_;
  rclcpp::Client<dobot_msgs_v4::srv::SpeedFactor>::SharedPtr speed_factor_;
  rclcpp::Client<dobot_msgs_v4::srv::SetCollisionLevel>::SharedPtr collision_;
  rclcpp::TimerBase::SharedPtr servo_timer_;

  std::mutex pc_mtx_;
  Cloud latest_pc_;
  rclcpp::Time latest_pc_stamp_;

  std::mutex tool_mtx_;
  ToolPose latest_tool_;

  std::mutex joint_mtx_;
  JointPose latest_joint_;

  std::mutex template_mtx_;
  std::vector<PyramidLevel> ref_pyramid_;
  Mat4 T_base_tool_ref_{Mat4::Identity()};
  Mat4 T_base_camera_ref_{Mat4::Identity()};
  std::atomic_bool template_ready_{false};

  std::mutex icp_mtx_;
  IcpResult latest_icp_;
  uint64_t icp_seq_counter_{0};
  std::atomic<uint64_t> last_consumed_icp_seq_{0};

  std::atomic_bool running_{true};
  std::atomic_bool servo_enabled_{false};
  std::atomic_bool servo_inflight_{false};
  std::atomic_bool joint_move_inflight_{false};
  std::atomic<int64_t> servo_inflight_since_ms_{0};
  std::atomic<int64_t> joint_move_since_ms_{0};
  std::atomic<int> tool_msg_count_{0};
  std::atomic<int> tool_drop_count_{0};
  std::atomic<int> joint_msg_count_{0};
  std::atomic<int> joint_drop_count_{0};
  std::atomic<int> bad_icp_cycles_{0};
  std::atomic<int> missed_servo_cycles_{0};

  std::thread icp_thread_;
  std::thread keyboard_thread_;
};

// ROS2 入口：使用多线程 executor，让点云回调、服务回调和定时器互不长时间阻塞。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ContinuousIcpServoNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
