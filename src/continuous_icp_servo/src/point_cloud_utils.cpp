#include "continuous_icp_servo/point_cloud_utils.hpp"

#include <cmath>
#include <unordered_map>

namespace continuous_icp_servo
{

Key voxelKey(const Point & p, double voxel)
{
  return Key{
    static_cast<int64_t>(std::floor(p.x() / voxel)),
    static_cast<int64_t>(std::floor(p.y() / voxel)),
    static_cast<int64_t>(std::floor(p.z() / voxel))};
}

Cloud voxelDown(const Cloud & src, double voxel)
{
  if (src.empty() || voxel <= 0.0) {
    return src;
  }
  std::unordered_map<Key, size_t, KeyHash> seen;
  seen.reserve(src.size());
  Cloud out;
  out.reserve(src.size() / 4 + 1);
  for (const auto & p : src) {
    const auto k = voxelKey(p, voxel);
    if (seen.emplace(k, out.size()).second) {
      out.push_back(p);
    }
  }
  return out;
}

}  // namespace continuous_icp_servo
