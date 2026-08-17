#pragma once

#include "continuous_icp_servo/types.hpp"

namespace continuous_icp_servo
{

Key voxelKey(const Point & p, double voxel);
Cloud voxelDown(const Cloud & src, double voxel);

}  // namespace continuous_icp_servo
