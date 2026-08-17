#pragma once

#include "continuous_icp_servo/types.hpp"

namespace continuous_icp_servo
{

int64_t steadyMs();
Eigen::Matrix3d rotXyzDeg(double rx, double ry, double rz);
Eigen::Vector3d matrixToXyzDeg(const Mat3 & R);
Mat4 poseToMatrixMm(const ToolPose & p);

}  // namespace continuous_icp_servo
