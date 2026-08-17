#pragma once

#include "continuous_icp_servo/spatial_index.hpp"
#include "continuous_icp_servo/types.hpp"

namespace continuous_icp_servo
{

bool kabschStep(
  const std::vector<Point> & src, const std::vector<Point> & dst,
  Mat3 & R, Point & t, double & rmse);

Cloud estimateNormals(
  const Cloud & points, const SpatialIndex & index,
  double radius = 0.015, size_t max_neighbors = 40);

bool pointToPlaneRefine(
  const Cloud & src, const Cloud & target, const SpatialIndex & target_index,
  const Cloud & target_normals, const Mat4 & initial, Mat4 & refined,
  double & rmse, int & inliers, double & overlap,
  double dmax = 0.010, int max_iterations = 6, size_t max_points = 40000);

}  // namespace continuous_icp_servo
