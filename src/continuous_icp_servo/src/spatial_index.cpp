#include "continuous_icp_servo/spatial_index.hpp"

#include <algorithm>
#include <cmath>

#include "continuous_icp_servo/point_cloud_utils.hpp"

namespace continuous_icp_servo
{

SpatialIndex::SpatialIndex(const Cloud & points, double cell_size)
{
  build(points, cell_size);
}

void SpatialIndex::build(const Cloud & points, double cell_size)
{
  pts_ = points;
  cell_ = cell_size;
  buckets_.clear();
  buckets_.reserve(pts_.size());
  for (int i = 0; i < static_cast<int>(pts_.size()); ++i) {
    buckets_[voxelKey(pts_[i], cell_)].push_back(i);
  }
}

bool SpatialIndex::nearest(const Point & q, double max_dist, Point & out, double & best_dist) const
{
  const auto center = voxelKey(q, cell_);
  const int radius = std::max(1, static_cast<int>(std::ceil(max_dist / cell_)));
  double best2 = max_dist * max_dist;
  bool found = false;
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dz = -radius; dz <= radius; ++dz) {
        const Key k{center.x + dx, center.y + dy, center.z + dz};
        const auto hit = buckets_.find(k);
        if (hit == buckets_.end()) {
          continue;
        }
        for (const int idx : hit->second) {
          const double d2 = (pts_[idx] - q).squaredNorm();
          if (d2 < best2) {
            best2 = d2;
            out = pts_[idx];
            found = true;
          }
        }
      }
    }
  }
  best_dist = std::sqrt(best2);
  return found;
}

}  // namespace continuous_icp_servo
