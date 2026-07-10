#pragma once

#include <string>

#include "continuous_icp_servo/types.hpp"

namespace continuous_icp_servo
{

bool loadHandeye(const std::string & path, Mat4 & X);

}  // namespace continuous_icp_servo
