#pragma once

#include "continuous_icp_servo/types.hpp"

namespace continuous_icp_servo
{

bool kabschStep(
  const std::vector<Point> & src, const std::vector<Point> & dst,
  Mat3 & R, Point & t, double & rmse);

}  // namespace continuous_icp_servo
