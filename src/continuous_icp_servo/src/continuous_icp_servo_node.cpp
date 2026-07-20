#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <deque>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <limits>
#include <regex>
#include <sstream>
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

#include "dobot_msgs_v4/srv/clear_error.hpp"
#include "dobot_msgs_v4/srv/disable_robot.hpp"
#include "dobot_msgs_v4/srv/enable_robot.hpp"
#include "dobot_msgs_v4/srv/get_pose.hpp"
#include "dobot_msgs_v4/srv/get_error_id.hpp"
#include "dobot_msgs_v4/srv/mov_j.hpp"
#include "dobot_msgs_v4/srv/robot_mode.hpp"
#include "dobot_msgs_v4/srv/servo_p.hpp"
#include "dobot_msgs_v4/srv/set_collision_level.hpp"
#include "dobot_msgs_v4/srv/speed_factor.hpp"
#include "dobot_msgs_v4/srv/stop.hpp"
#include "dobot_msgs_v4/srv/tool.hpp"
#include "dobot_msgs_v4/srv/user.hpp"



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
  // 节点启动：低频 ICP 只覆盖最新绝对目标，高频 ServoP 重复跟随该目标。
  ContinuousIcpServoNode()
  : Node("continuous_icp_servo")
  {
    pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "/camera/camera/depth/color/points");
    joint_topic_ = declare_parameter<std::string>("joint_topic", "/joint_states_robot");
    command_topic_ = declare_parameter<std::string>("command_topic", "/continuous_icp_servo/command");
    handeye_path_ = declare_parameter<std::string>(
      "handeye_path",
      "/home/ylx/Robot_Arm_Project/scripts/active_handeye_calibration.json");
    icp_hz_ = declare_parameter<double>("icp_hz", 6.0);
    servo_hz_ = declare_parameter<double>("servo_hz", 20.0);
    getpose_hz_ = declare_parameter<double>("getpose_hz", 30.0);
    pose_sync_tolerance_s_ = declare_parameter<double>("pose_sync_tolerance_s", 0.10);
    max_pose_age_s_ = declare_parameter<double>("max_pose_age_s", 0.25);
    max_icp_age_s_ = declare_parameter<double>("max_icp_age_s", 1.0);
    min_icp_inliers_ = declare_parameter<int>("min_icp_inliers", 500);
    min_icp_overlap_ = declare_parameter<double>("min_icp_overlap", 0.03);
    max_icp_rmse_mm_ = declare_parameter<double>("max_icp_rmse_mm", 60.0);
    max_icp_trans_mm_ = declare_parameter<double>("max_icp_trans_mm", 500.0);
    max_icp_rot_deg_ = declare_parameter<double>("max_icp_rot_deg", 60.0);
    point_to_plane_trigger_mm_ = declare_parameter<double>("point_to_plane_trigger_mm", 10.0);
    point_to_plane_dmax_m_ = declare_parameter<double>("point_to_plane_dmax_m", 0.010);
    point_to_plane_iterations_ = declare_parameter<int>("point_to_plane_iterations", 6);
    point_to_plane_max_points_ = static_cast<size_t>(std::max<int64_t>(
      1000, declare_parameter<int64_t>("point_to_plane_max_points", 80000)));
    speed_percent_ = declare_parameter<int>("speed_percent", 10);

    if (!loadHandeye(handeye_path_, X_)) {
      throw std::runtime_error("failed to load handeye matrix: " + handeye_path_);
    }
    X_inv_ = X_.inverse();

    pc_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    joint_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    command_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions pc_options;
    pc_options.callback_group = pc_group_;
    rclcpp::SubscriptionOptions joint_options;
    joint_options.callback_group = joint_group_;
    rclcpp::SubscriptionOptions command_options;
    command_options.callback_group = command_group_;

    pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ContinuousIcpServoNode::onPointCloud, this, std::placeholders::_1), pc_options);
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_topic_, 10, std::bind(&ContinuousIcpServoNode::onJointState, this, std::placeholders::_1), joint_options);
    command_sub_ = create_subscription<std_msgs::msg::String>(
      command_topic_, 10, [this](const std_msgs::msg::String::SharedPtr msg) {
        handleCommand(msg->data);
      }, command_options);

    service_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    mov_j_ = create_client<dobot_msgs_v4::srv::MovJ>(
      "/dobot_bringup_ros2/srv/MovJ", rmw_qos_profile_services_default, service_group_);
    get_pose_ = create_client<dobot_msgs_v4::srv::GetPose>(
      "/dobot_bringup_ros2/srv/GetPose", rmw_qos_profile_services_default, service_group_);
    robot_mode_ = create_client<dobot_msgs_v4::srv::RobotMode>(
      "/dobot_bringup_ros2/srv/RobotMode", rmw_qos_profile_services_default, service_group_);
    get_error_id_ = create_client<dobot_msgs_v4::srv::GetErrorID>(
      "/dobot_bringup_ros2/srv/GetErrorID", rmw_qos_profile_services_default, service_group_);
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
    user_ = create_client<dobot_msgs_v4::srv::User>(
      "/dobot_bringup_ros2/srv/User", rmw_qos_profile_services_default, service_group_);
    tool_ = create_client<dobot_msgs_v4::srv::Tool>(
      "/dobot_bringup_ros2/srv/Tool", rmw_qos_profile_services_default, service_group_);

    initRobotAsync();

    const auto servo_period = std::chrono::duration<double>(1.0 / std::max(1.0, servo_hz_));
    servo_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(servo_period),
      std::bind(&ContinuousIcpServoNode::servoTick, this));
    const auto pose_period = std::chrono::duration<double>(1.0 / std::max(1.0, getpose_hz_));
    pose_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(pose_period),
      std::bind(&ContinuousIcpServoNode::pollGetPose, this));

    icp_thread_ = std::thread([this]() { icpLoop(); });
    keyboard_thread_ = std::thread([this]() { keyboardLoop(); });

    RCLCPP_INFO(
      get_logger(), "连续 ICP ServoP 节点已启动：ICP=%.1fHz ServoP=%.1fHz GetPose=%.1fHz",
      icp_hz_, servo_hz_, getpose_hz_);
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
  struct TemplateModel {
    std::vector<PyramidLevel> pyramid;
    Cloud point_to_plane_ref;
    Cloud point_to_plane_normals;
    SpatialIndex point_to_plane_index;
    Mat4 T_base_tool_ref{Mat4::Identity()};
    Mat4 T_base_camera_ref{Mat4::Identity()};
  };

  static bool parseGetPose(const std::string & text, ToolPose & pose)
  {
    std::string cleaned = text;
    for (char & c : cleaned) {
      if (c == '{' || c == '}' || c == ',') {
        c = ' ';
      }
    }
    std::istringstream input(cleaned);
    if (!(input >> pose.x >> pose.y >> pose.z >> pose.rx >> pose.ry >> pose.rz)) {
      return false;
    }
    const bool finite =
      std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.z) &&
      std::isfinite(pose.rx) && std::isfinite(pose.ry) && std::isfinite(pose.rz);
    return finite && std::sqrt(pose.x * pose.x + pose.y * pose.y + pose.z * pose.z) > 1.0;
  }

  // dobot_bringup_v4的所有Dashboard服务共用同一个29999 socket，驱动内部没有命令互斥。
  // 节点侧保证任意时刻最多一个GetPose/ServoP/MovJ/Stop请求在途。
  bool tryAcquireDashboard()
  {
    bool expected = false;
    return dashboard_inflight_.compare_exchange_strong(expected, true);
  }

  void finishDashboardRequest()
  {
    dashboard_inflight_.store(false);
    if (stop_pending_.exchange(false)) {
      sendStop();
    } else if (diagnostics_pending_.exchange(false)) {
      requestRobotDiagnostics();
    }
  }

  void requestRobotDiagnostics()
  {
    if (!robot_mode_->service_is_ready() || !get_error_id_->service_is_ready() ||
      !tryAcquireDashboard())
    {
      return;
    }
    auto mode_req = std::make_shared<dobot_msgs_v4::srv::RobotMode::Request>();
    robot_mode_->async_send_request(
      mode_req, [this](rclcpp::Client<dobot_msgs_v4::srv::RobotMode>::SharedFuture mode_future) {
        try {
          const auto mode = mode_future.get();
          RCLCPP_ERROR(
            this->get_logger(), "ServoP故障诊断：RobotMode res=%d mode=%s",
            mode->res, mode->robot_return.c_str());
        } catch (const std::exception & e) {
          RCLCPP_ERROR(this->get_logger(), "RobotMode诊断失败：%s", e.what());
          finishDashboardRequest();
          return;
        }
        auto error_req = std::make_shared<dobot_msgs_v4::srv::GetErrorID::Request>();
        get_error_id_->async_send_request(
          error_req,
          [this](rclcpp::Client<dobot_msgs_v4::srv::GetErrorID>::SharedFuture error_future) {
            try {
              const auto error = error_future.get();
              RCLCPP_ERROR(
                this->get_logger(), "ServoP故障诊断：GetErrorID res=%d errors=%s",
                error->res, error->robot_return.c_str());
            } catch (const std::exception & e) {
              RCLCPP_ERROR(this->get_logger(), "GetErrorID诊断失败：%s", e.what());
            }
            finishDashboardRequest();
          });
      });
  }

  // 唯一的笛卡尔反馈来源：固定频率调用 GetPose()，并保留短历史用于点云时间对齐。
  void pollGetPose()
  {
    if (!robot_initialized_.load() || !get_pose_->service_is_ready() ||
      getpose_inflight_.exchange(true))
    {
      return;
    }
    if (!tryAcquireDashboard()) {
      getpose_inflight_.store(false);
      return;
    }
    auto req = std::make_shared<dobot_msgs_v4::srv::GetPose::Request>();
    req->user = 0;
    req->tool = 0;
    get_pose_->async_send_request(
      req, [this](rclcpp::Client<dobot_msgs_v4::srv::GetPose>::SharedFuture future) {
        getpose_inflight_.store(false);
        ToolPose next;
        try {
          const auto response = future.get();
          if (response->res != 0 || !parseGetPose(response->robot_return, next)) {
            const int dropped = ++getpose_drop_count_;
            if (dropped == 1 || dropped % 20 == 0) {
              RCLCPP_WARN(
                this->get_logger(), "GetPose无效 res=%d return=%s",
                response->res, response->robot_return.c_str());
            }
            finishDashboardRequest();
            return;
          }
        } catch (const std::exception & e) {
          RCLCPP_WARN(this->get_logger(), "GetPose调用异常：%s", e.what());
          finishDashboardRequest();
          return;
        }
        next.stamp = this->now();
        next.valid = true;
        {
          std::lock_guard<std::mutex> lk(tool_mtx_);
          if (latest_tool_.valid) {
            const Point previous(latest_tool_.x, latest_tool_.y, latest_tool_.z);
            const Point current(next.x, next.y, next.z);
            if ((current - previous).norm() > 200.0) {
              const int dropped = ++getpose_drop_count_;
              if (dropped == 1 || dropped % 20 == 0) {
                RCLCPP_WARN(this->get_logger(), "拒绝GetPose跳变：%.1fmm", (current - previous).norm());
              }
              finishDashboardRequest();
              return;
            }
          }
          latest_tool_ = next;
          pose_history_.push_back(next);
          while (pose_history_.size() > 100) {
            pose_history_.pop_front();
          }
        }
        const int count = ++getpose_count_;
        if (count == 1) {
          RCLCPP_INFO(
            this->get_logger(), "GetPose反馈 xyz=[%.1f %.1f %.1f] rpy=[%.1f %.1f %.1f]",
            next.x, next.y, next.z, next.rx, next.ry, next.rz);
        }
        finishDashboardRequest();
      });
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

  // 取最新点云，并从 GetPose 历史中选择时间最近的一帧，降低 eye-in-hand 运动同步误差。
  bool snapshot(
    Cloud & pc, ToolPose & tool, rclcpp::Time * source_stamp = nullptr,
    std::string * reason = nullptr)
  {
    rclcpp::Time pc_stamp;
    {
      std::lock_guard<std::mutex> lk(pc_mtx_);
      pc = latest_pc_;
      pc_stamp = latest_pc_stamp_;
    }
    {
      std::lock_guard<std::mutex> lk(tool_mtx_);
      double best_dt = std::numeric_limits<double>::infinity();
      for (const auto & candidate : pose_history_) {
        const double dt = std::abs((candidate.stamp - pc_stamp).seconds());
        if (dt < best_dt) {
          best_dt = dt;
          tool = candidate;
        }
      }
    }
    const bool has_pc_stamp = pc_stamp.nanoseconds() > 0;
    const bool has_tool_stamp = tool.stamp.nanoseconds() > 0;
    const double pc_age = has_pc_stamp ? (now() - pc_stamp).seconds() : 999.0;
    const double tool_age = has_tool_stamp ? (now() - tool.stamp).seconds() : 999.0;
    const double sync_dt = has_tool_stamp && has_pc_stamp ?
      std::abs((tool.stamp - pc_stamp).seconds()) : 999.0;
    const bool pc_ok = pc.size() > 500 && has_pc_stamp && pc_age < 2.0;
    const bool tool_ok =
      tool.valid && has_tool_stamp && tool_age < 2.0 && sync_dt <= pose_sync_tolerance_s_;
    if (source_stamp) {
      *source_stamp = pc_stamp;
    }
    if (reason && (!pc_ok || !tool_ok)) {
      *reason =
        "pc_pts=" + std::to_string(pc.size()) +
        " pc_age=" + std::to_string(pc_age) + "s" +
        " tool_valid=" + std::string(tool.valid ? "true" : "false") +
        " tool_age=" + std::to_string(tool_age) + "s" +
        " sync_dt=" + std::to_string(sync_dt) + "s" +
        " tool_xyz=[" + std::to_string(tool.x) + "," + std::to_string(tool.y) + "," + std::to_string(tool.z) + "]";
    }
    return pc_ok && tool_ok;
  }

  // 录制参考帧：建立不可变模板；ICP线程只复制shared_ptr，不再复制整套点云索引。
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
      if (snapshot(pc, tool, nullptr, &reason)) {
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

    auto model = std::make_shared<TemplateModel>();
    for (const auto & cfg : std::vector<std::pair<double, double>>{{0.020, 0.100}, {0.010, 0.050}, {0.005, 0.025}}) {
      PyramidLevel level;
      level.voxel = cfg.first;
      level.dmax = cfg.second;
      level.ref = cfg.first <= 0.005 ? ref_5mm : voxelDown(ref_5mm, cfg.first);
      level.index.build(level.ref, level.dmax);
      RCLCPP_INFO(
        get_logger(), "  L%zu: %zu pts voxel=%.0fmm dmax=%.0fmm",
        model->pyramid.size(), level.ref.size(), level.voxel * 1000.0, level.dmax * 1000.0);
      model->pyramid.push_back(std::move(level));
    }

    const auto normals_t0 = std::chrono::steady_clock::now();
    const size_t p2_stride = std::max<size_t>(
      1, (ref_5mm.size() + point_to_plane_max_points_ - 1) / point_to_plane_max_points_);
    model->point_to_plane_ref.reserve(
      std::min(ref_5mm.size(), point_to_plane_max_points_));
    for (size_t i = 0; i < ref_5mm.size(); i += p2_stride) {
      model->point_to_plane_ref.push_back(ref_5mm[i]);
    }
    model->point_to_plane_index.build(model->point_to_plane_ref, point_to_plane_dmax_m_);
    model->point_to_plane_normals = estimateNormals(
      model->point_to_plane_ref, model->point_to_plane_index, 0.015, 40);
    const double normals_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - normals_t0).count();
    RCLCPP_INFO(
      get_logger(), "  L3(point-to-plane): %zu pts normals=%.0fms",
      model->point_to_plane_ref.size(), normals_ms);

    model->T_base_tool_ref = poseToMatrixMm(ref_tool);
    model->T_base_camera_ref = model->T_base_tool_ref * X_;

    {
      std::lock_guard<std::mutex> lk(template_mtx_);
      ref_model_ = std::move(model);
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
    const auto compute_t0 = std::chrono::steady_clock::now();
    Cloud pc;
    ToolPose tool;
    rclcpp::Time source_stamp;
    if (!snapshot(pc, tool, &source_stamp)) {
      return;
    }

    std::shared_ptr<const TemplateModel> model;
    {
      std::lock_guard<std::mutex> lk(template_mtx_);
      model = ref_model_;
    }
    if (!model) {
      return;
    }

    Mat4 T_base_tool_cur = poseToMatrixMm(tool);
    Mat4 T_base_camera_cur = T_base_tool_cur * X_;
    Mat4 T_init_mm = model->T_base_camera_ref.inverse() * T_base_camera_cur;
    Mat4 T_acc = T_init_mm;
    T_acc.block<3, 1>(0, 3) /= 1000.0;

    double final_rmse = 999.0;
    int final_inliers = 0;
    double final_overlap = 0.0;
    Cloud fine_src;

    for (size_t level_id = 0; level_id < model->pyramid.size(); ++level_id) {
      const auto & level = model->pyramid[level_id];
      Cloud src_ds = voxelDown(pc, level.voxel);
      if (level_id + 1 == model->pyramid.size()) {
        fine_src = src_ds;
      }
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

    const double p2p_translation_mm = T_acc.block<3, 1>(0, 3).norm() * 1000.0;
    if (
      p2p_translation_mm <= point_to_plane_trigger_mm_ && !fine_src.empty() &&
      !model->point_to_plane_ref.empty())
    {
      Mat4 refined;
      double p2l_rmse = 999.0;
      int p2l_inliers = 0;
      double p2l_overlap = 0.0;
      if (pointToPlaneRefine(
          fine_src, model->point_to_plane_ref, model->point_to_plane_index,
          model->point_to_plane_normals, T_acc, refined,
          p2l_rmse, p2l_inliers, p2l_overlap,
          point_to_plane_dmax_m_, point_to_plane_iterations_,
          point_to_plane_max_points_))
      {
        T_acc = refined;
        final_rmse = p2l_rmse;
        final_inliers = p2l_inliers;
        final_overlap = p2l_overlap;
      }
    }

    Mat4 T_icp_mm = T_acc;
    T_icp_mm.block<3, 1>(0, 3) *= 1000.0;
    const double trans_mm = T_icp_mm.block<3, 1>(0, 3).norm();
    const double rot_deg = Eigen::AngleAxisd(T_icp_mm.block<3, 3>(0, 0)).angle() * 180.0 / M_PI;

    IcpResult result;
    result.T_icp_mm = T_icp_mm;
    const Mat4 T_delta = X_ * T_icp_mm * X_inv_;
    result.T_target_base = T_base_tool_cur * T_delta.inverse();
    result.rmse_mm = final_rmse * 1000.0;
    result.overlap = final_overlap;
    result.inliers = final_inliers;
    result.seq = ++icp_seq_counter_;
    result.compute_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - compute_t0).count();
    result.ok =
      final_inliers >= min_icp_inliers_ &&
      result.overlap >= min_icp_overlap_ &&
      result.rmse_mm <= max_icp_rmse_mm_ &&
      trans_mm <= max_icp_trans_mm_ &&
      rot_deg <= max_icp_rot_deg_;
    // 使用点云采集时刻而不是计算完成时刻，让Servo watchdog能识别慢ICP造成的陈旧目标。
    result.stamp = source_stamp;

    {
      std::lock_guard<std::mutex> lk(icp_mtx_);
      latest_icp_ = result;
    }

    static int log_skip = 0;
    const bool should_log = servo_enabled_.load() && ((log_skip++ % 10) == 0 || !result.ok);
    if (should_log) {
      RCLCPP_INFO(
        get_logger(), "伺服中 ICP %s RMSE=%.1fmm inliers=%d overlap=%.2f |t|=%.1fmm |r|=%.2fdeg time=%.0fms",
        result.ok ? "正常" : "异常", result.rmse_mm, result.inliers, result.overlap,
        trans_mm, rot_deg, result.compute_ms);
    }
  }

  // 高频ServoP重复发送低频ICP缓存的最新绝对目标；新ICP只覆盖缓存，不直接发运动命令。
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

    ToolPose tool;
    {
      std::lock_guard<std::mutex> lk(tool_mtx_);
      tool = latest_tool_;
    }
    const double pose_age_s = tool.stamp.nanoseconds() > 0 ?
      (now() - tool.stamp).seconds() : 999.0;
    if (!tool.valid || pose_age_s > max_pose_age_s_) {
      const int bad_cycles = bad_pose_cycles_.fetch_add(1) + 1;
      static int bad_tool_log_skip = 0;
      if ((bad_tool_log_skip++ % 20) == 0) {
        RCLCPP_WARN(
          get_logger(), "连续伺服等待：GetPose %s age=%.2fs",
          tool.valid ? "过期" : "无效", pose_age_s);
      }
      if (bad_cycles > static_cast<int>(std::max(1.0, servo_hz_) * 0.5)) {
        servo_enabled_.store(false);
        sendStop();
        RCLCPP_WARN(get_logger(), "连续伺服已停止：GetPose反馈连续0.5s无效");
      }
      return;
    }
    bad_pose_cycles_.store(0);

    const Mat4 T_cur = poseToMatrixMm(tool);
    const Mat4 & T_target = icp.T_target_base;
    const Point xyz = T_target.block<3, 1>(0, 3);
    const Point rpy = matrixToXyzDeg(T_target.block<3, 3>(0, 0));
    const double position_error =
      (T_target.block<3, 1>(0, 3) - T_cur.block<3, 1>(0, 3)).norm();
    const double rotation_error = Eigen::AngleAxisd(
      T_target.block<3, 3>(0, 0) * T_cur.block<3, 3>(0, 0).transpose()).angle() *
      180.0 / M_PI;

    auto req = std::make_shared<dobot_msgs_v4::srv::ServoP::Request>();
    req->a = xyz.x();
    req->b = xyz.y();
    req->c = xyz.z();
    req->d = rpy.x();
    req->e = rpy.y();
    req->f = rpy.z();
    req->param_value.clear();

    if (!tryAcquireDashboard()) {
      missed_servo_cycles_.fetch_add(1);
      return;
    }
    servo_inflight_.store(true);
    servo_inflight_since_ms_.store(steadyMs());
    static int servo_send_log_skip = 0;
    if ((servo_send_log_skip++ % 20) == 0) {
      RCLCPP_INFO(
        get_logger(), "ServoP latest-only seq=%lu target=[%.1f %.1f %.1f] err=%.2fmm/%.2fdeg age=%.2fs",
        static_cast<unsigned long>(icp.seq), xyz.x(), xyz.y(), xyz.z(),
        position_error, rotation_error, icp_age_s);
    }
    servo_p_->async_send_request(
      req,
      [this](rclcpp::Client<dobot_msgs_v4::srv::ServoP>::SharedFuture future) {
        try {
          const auto res = future.get();
          if (res->res != 0) {
            servo_enabled_.store(false);
            diagnostics_pending_.store(true);
            RCLCPP_ERROR(
              this->get_logger(), "ServoP失败 res=%d：立即关闭连续伺服并发送Stop；若仍为-1请查看驱动原始TCP反馈",
              res->res);
            sendStop();
          }
        } catch (const std::exception & e) {
          servo_enabled_.store(false);
          diagnostics_pending_.store(true);
          RCLCPP_ERROR(this->get_logger(), "ServoP调用异常：%s；立即Stop", e.what());
          sendStop();
        }
        servo_inflight_.store(false);
        servo_inflight_since_ms_.store(0);
        finishDashboardRequest();
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
      waitClient(get_pose_, "GetPose");
      waitClient(robot_mode_, "RobotMode");
      waitClient(get_error_id_, "GetErrorID");
      waitClient(user_, "User");
      waitClient(tool_, "Tool");
      waitClient(speed_factor_, "SpeedFactor");
      waitClient(collision_, "SetCollisionLevel");
      RCLCPP_INFO(get_logger(), "初始化序列：ClearError -> DisableRobot -> EnableRobot -> User0 -> Tool0 -> SpeedFactor -> Collision");
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

  // 初始化第3步：重新使能，随后强制统一User0/Tool0。
  void initEnableRobot()
  {
    auto req = std::make_shared<dobot_msgs_v4::srv::EnableRobot::Request>();
    enable_robot_->async_send_request(
      req,
      [this](rclcpp::Client<dobot_msgs_v4::srv::EnableRobot>::SharedFuture future) {
        RCLCPP_INFO(this->get_logger(), "EnableRobot res=%d", future.get()->res);
        initUser0();
      });
  }

  void initUser0()
  {
    auto req = std::make_shared<dobot_msgs_v4::srv::User::Request>();
    req->index = 0;
    user_->async_send_request(
      req, [this](rclcpp::Client<dobot_msgs_v4::srv::User>::SharedFuture future) {
        RCLCPP_INFO(this->get_logger(), "User(0) res=%d", future.get()->res);
        initTool0();
      });
  }

  void initTool0()
  {
    auto req = std::make_shared<dobot_msgs_v4::srv::Tool::Request>();
    req->index = 0;
    tool_->async_send_request(
      req, [this](rclcpp::Client<dobot_msgs_v4::srv::Tool>::SharedFuture future) {
        RCLCPP_INFO(this->get_logger(), "Tool(0) res=%d", future.get()->res);
        initSpeedFactor();
      });
  }

  // 全局限速。
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
        robot_initialized_.store(true);
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
    servo_inflight_.store(false);
    servo_inflight_since_ms_.store(0);
    joint_move_inflight_.store(false);
    joint_move_since_ms_.store(0);
    stop_pending_.store(true);
    if (!stop_->service_is_ready() || !tryAcquireDashboard()) {
      return;
    }
    stop_pending_.store(false);
    auto req = std::make_shared<dobot_msgs_v4::srv::Stop::Request>();
    stop_->async_send_request(
      req, [this](rclcpp::Client<dobot_msgs_v4::srv::Stop>::SharedFuture future) {
        try {
          const auto response = future.get();
          if (response->res != 0) {
            RCLCPP_WARN(this->get_logger(), "Stop返回异常 res=%d", response->res);
          }
        } catch (const std::exception & e) {
          RCLCPP_WARN(this->get_logger(), "Stop调用异常：%s", e.what());
        }
        finishDashboardRequest();
      });
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
    if (!robot_initialized_.load()) {
      RCLCPP_WARN(get_logger(), "%s：机械臂初始化尚未完成", label.c_str());
      return;
    }
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
    if (!tryAcquireDashboard()) {
      joint_move_inflight_.store(false);
      joint_move_since_ms_.store(0);
      RCLCPP_WARN(get_logger(), "%s：Dashboard正忙，本次偏移未发送，请重试", label.c_str());
      return;
    }
    mov_j_->async_send_request(
      mov,
      [this, label](rclcpp::Client<dobot_msgs_v4::srv::MovJ>::SharedFuture mov_future) {
        const auto mov_res = mov_future.get();
        RCLCPP_INFO(this->get_logger(), "%s：JointMovJ 返回 res=%d", label.c_str(), mov_res->res);
        joint_move_inflight_.store(false);
        joint_move_since_ms_.store(0);
        finishDashboardRequest();
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
        bad_pose_cycles_.store(0);
        missed_servo_cycles_.store(0);
        servo_inflight_.store(false);
        servo_inflight_since_ms_.store(0);
        RCLCPP_INFO(
          get_logger(), "连续伺服准备：最新绝对目标重复发送；ServoP=%s ICP age<=%.2fs inliers>=%d rmse<=%.1fmm overlap>=%.2f",
          servo_p_->service_is_ready() ? "ready" : "not ready",
          max_icp_age_s_, min_icp_inliers_, max_icp_rmse_mm_, min_icp_overlap_);
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
  std::string joint_topic_;
  std::string command_topic_;
  std::string handeye_path_;
  double icp_hz_{6.0};
  double servo_hz_{20.0};
  double getpose_hz_{30.0};
  double pose_sync_tolerance_s_{0.10};
  double max_pose_age_s_{0.25};
  double max_icp_age_s_{1.0};
  int min_icp_inliers_{500};
  double min_icp_overlap_{0.03};
  double max_icp_rmse_mm_{60.0};
  double max_icp_trans_mm_{500.0};
  double max_icp_rot_deg_{60.0};
  double point_to_plane_trigger_mm_{10.0};
  double point_to_plane_dmax_m_{0.010};
  int point_to_plane_iterations_{6};
  size_t point_to_plane_max_points_{80000};
  int speed_percent_{10};

  Mat4 X_{Mat4::Identity()};
  Mat4 X_inv_{Mat4::Identity()};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::CallbackGroup::SharedPtr pc_group_;
  rclcpp::CallbackGroup::SharedPtr joint_group_;
  rclcpp::CallbackGroup::SharedPtr command_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::Client<dobot_msgs_v4::srv::MovJ>::SharedPtr mov_j_;
  rclcpp::Client<dobot_msgs_v4::srv::GetPose>::SharedPtr get_pose_;
  rclcpp::Client<dobot_msgs_v4::srv::RobotMode>::SharedPtr robot_mode_;
  rclcpp::Client<dobot_msgs_v4::srv::GetErrorID>::SharedPtr get_error_id_;
  rclcpp::Client<dobot_msgs_v4::srv::ServoP>::SharedPtr servo_p_;
  rclcpp::Client<dobot_msgs_v4::srv::Stop>::SharedPtr stop_;
  rclcpp::Client<dobot_msgs_v4::srv::ClearError>::SharedPtr clear_error_;
  rclcpp::Client<dobot_msgs_v4::srv::DisableRobot>::SharedPtr disable_robot_;
  rclcpp::Client<dobot_msgs_v4::srv::EnableRobot>::SharedPtr enable_robot_;
  rclcpp::Client<dobot_msgs_v4::srv::SpeedFactor>::SharedPtr speed_factor_;
  rclcpp::Client<dobot_msgs_v4::srv::SetCollisionLevel>::SharedPtr collision_;
  rclcpp::Client<dobot_msgs_v4::srv::User>::SharedPtr user_;
  rclcpp::Client<dobot_msgs_v4::srv::Tool>::SharedPtr tool_;
  rclcpp::TimerBase::SharedPtr servo_timer_;
  rclcpp::TimerBase::SharedPtr pose_timer_;

  std::mutex pc_mtx_;
  Cloud latest_pc_;
  rclcpp::Time latest_pc_stamp_;

  std::mutex tool_mtx_;
  ToolPose latest_tool_;
  std::deque<ToolPose> pose_history_;

  std::mutex joint_mtx_;
  JointPose latest_joint_;

  std::mutex template_mtx_;
  std::shared_ptr<const TemplateModel> ref_model_;
  std::atomic_bool template_ready_{false};

  std::mutex icp_mtx_;
  IcpResult latest_icp_;
  uint64_t icp_seq_counter_{0};

  std::atomic_bool running_{true};
  std::atomic_bool robot_initialized_{false};
  std::atomic_bool servo_enabled_{false};
  std::atomic_bool servo_inflight_{false};
  std::atomic_bool getpose_inflight_{false};
  std::atomic_bool dashboard_inflight_{false};
  std::atomic_bool stop_pending_{false};
  std::atomic_bool diagnostics_pending_{false};
  std::atomic_bool joint_move_inflight_{false};
  std::atomic<int64_t> servo_inflight_since_ms_{0};
  std::atomic<int64_t> joint_move_since_ms_{0};
  std::atomic<int> getpose_count_{0};
  std::atomic<int> getpose_drop_count_{0};
  std::atomic<int> joint_msg_count_{0};
  std::atomic<int> joint_drop_count_{0};
  std::atomic<int> bad_icp_cycles_{0};
  std::atomic<int> bad_pose_cycles_{0};
  std::atomic<int> missed_servo_cycles_{0};

  std::thread icp_thread_;
  std::thread keyboard_thread_;
};

// ROS2 入口：使用多线程 executor，让点云回调、服务回调和定时器互不长时间阻塞。
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ContinuousIcpServoNode>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
