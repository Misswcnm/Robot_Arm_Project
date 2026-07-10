#include "continuous_icp_servo/types.hpp"

#include <functional>

namespace continuous_icp_servo
{

size_t KeyHash::operator()(const Key & k) const
{
  const auto h1 = std::hash<int64_t>{}(k.x * 73856093);
  const auto h2 = std::hash<int64_t>{}(k.y * 19349663);
  const auto h3 = std::hash<int64_t>{}(k.z * 83492791);
  return h1 ^ (h2 << 1) ^ (h3 << 2);
}

}  // namespace continuous_icp_servo
