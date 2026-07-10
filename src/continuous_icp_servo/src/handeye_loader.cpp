#include "continuous_icp_servo/handeye_loader.hpp"

#include <fstream>
#include <iterator>
#include <regex>
#include <vector>

namespace continuous_icp_servo
{

bool loadHandeye(const std::string & path, Mat4 & X)
{
  std::ifstream in(path);
  if (!in) {
    return false;
  }
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  const auto pos = text.find("\"matrix\"");
  if (pos == std::string::npos) {
    return false;
  }
  const std::string tail = text.substr(pos);
  const std::regex number_re(R"([-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?)");
  std::sregex_iterator it(tail.begin(), tail.end(), number_re);
  std::sregex_iterator end;
  std::vector<double> values;
  for (; it != end && values.size() < 16; ++it) {
    values.push_back(std::stod(it->str()));
  }
  if (values.size() != 16) {
    return false;
  }
  X = Mat4::Identity();
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      X(r, c) = values[r * 4 + c];
    }
  }
  return true;
}

}  // namespace continuous_icp_servo
