#include "continuous_icp_servo/geometry.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <Eigen/Geometry>

namespace continuous_icp_servo
{

int64_t steadyMs()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

Eigen::Matrix3d rotXyzDeg(double rx, double ry, double rz)
{
  const double x = rx * M_PI / 180.0;
  const double y = ry * M_PI / 180.0;
  const double z = rz * M_PI / 180.0;
  return (Eigen::AngleAxisd(x, Point::UnitX()) *
          Eigen::AngleAxisd(y, Point::UnitY()) *
          Eigen::AngleAxisd(z, Point::UnitZ())).toRotationMatrix();
}

Eigen::Vector3d matrixToXyzDeg(const Mat3 & R)
{
  const double sy = std::clamp(R(0, 2), -1.0, 1.0);
  const double y = std::asin(sy);
  double x = 0.0;
  double z = 0.0;
  if (std::abs(std::cos(y)) > 1e-6) {
    x = std::atan2(-R(1, 2), R(2, 2));
    z = std::atan2(-R(0, 1), R(0, 0));
  } else {
    x = std::atan2(R(2, 1), R(1, 1));
    z = 0.0;
  }
  return {x * 180.0 / M_PI, y * 180.0 / M_PI, z * 180.0 / M_PI};
}

Mat4 poseToMatrixMm(const ToolPose & p)
{
  Mat4 T = Mat4::Identity();
  T.block<3, 3>(0, 0) = rotXyzDeg(p.rx, p.ry, p.rz);
  T.block<3, 1>(0, 3) = Point(p.x, p.y, p.z);
  return T;
}

}  // namespace continuous_icp_servo
