#include "continuous_icp_servo/handeye_loader.hpp"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <regex>
#include <vector>

namespace continuous_icp_servo
{

bool loadHandeye(const std::string & path, Mat4 & X)
{
  namespace fs = std::filesystem;
  fs::path current = fs::absolute(fs::path(path));
  const std::regex pointer_re("^\\s*\"([^\"]+)\"\\s*$");
  const std::regex number_re(R"([-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?)");

  for (int depth = 0; depth < 4; ++depth) {
    std::ifstream in(current);
    if (!in) {
      return false;
    }
    const std::string text(
      (std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const auto pos = text.find("\"matrix\"");
    if (pos == std::string::npos) {
      std::smatch match;
      if (!std::regex_match(text, match, pointer_re)) {
        return false;
      }
      fs::path target(match[1].str());
      current = target.is_absolute() ? target : current.parent_path() / target;
      continue;
    }

    const std::string tail = text.substr(pos);
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
  return false;
}

}  // namespace continuous_icp_servo
