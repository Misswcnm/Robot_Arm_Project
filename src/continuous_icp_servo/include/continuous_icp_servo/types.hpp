#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"

namespace continuous_icp_servo
{

using Point = Eigen::Vector3d;
using Cloud = std::vector<Point>;
using Mat4 = Eigen::Matrix4d;
using Mat3 = Eigen::Matrix3d;

struct ToolPose {
  double x{0}, y{0}, z{0}, rx{0}, ry{0}, rz{0};
  rclcpp::Time stamp;
  bool valid{false};
};

struct JointPose {
  std::array<double, 6> q_deg{};
  rclcpp::Time stamp;
  bool valid{false};
};

struct IcpResult {
  Mat4 T_icp_mm{Mat4::Identity()};
  Mat4 T_target_base{Mat4::Identity()};
  double rmse_mm{999.0};
  double overlap{0.0};
  int inliers{0};
  double compute_ms{0.0};
  uint64_t seq{0};
  bool ok{false};
  rclcpp::Time stamp;
};

struct JointDelta {
  double j1{0}, j2{0}, j3{0}, j4{0}, j5{0}, j6{0};
};

struct Key {
  int64_t x{0}, y{0}, z{0};
  bool operator==(const Key & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct KeyHash {
  size_t operator()(const Key & k) const;
};

}  // namespace continuous_icp_servo
