#pragma once

#include <unordered_map>

#include "continuous_icp_servo/types.hpp"

namespace continuous_icp_servo
{

class SpatialIndex
{
public:
  SpatialIndex() = default;
  SpatialIndex(const Cloud & points, double cell_size);

  void build(const Cloud & points, double cell_size);
  bool nearest(const Point & q, double max_dist, Point & out, double & best_dist) const;

private:
  Cloud pts_;
  double cell_{0.05};
  std::unordered_map<Key, std::vector<int>, KeyHash> buckets_;
};

struct PyramidLevel {
  double voxel{0.01};
  double dmax{0.05};
  Cloud ref;
  SpatialIndex index;
};

}  // namespace continuous_icp_servo
