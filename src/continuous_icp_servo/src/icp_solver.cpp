#include "continuous_icp_servo/icp_solver.hpp"

#include <cmath>

#include <Eigen/SVD>

namespace continuous_icp_servo
{

bool kabschStep(
  const std::vector<Point> & src, const std::vector<Point> & dst,
  Mat3 & R, Point & t, double & rmse)
{
  if (src.size() < 10 || src.size() != dst.size()) {
    return false;
  }
  Point cs = Point::Zero();
  Point cd = Point::Zero();
  for (size_t i = 0; i < src.size(); ++i) {
    cs += src[i];
    cd += dst[i];
  }
  cs /= static_cast<double>(src.size());
  cd /= static_cast<double>(dst.size());

  Mat3 H = Mat3::Zero();
  for (size_t i = 0; i < src.size(); ++i) {
    H += (src[i] - cs) * (dst[i] - cd).transpose();
  }

  Eigen::JacobiSVD<Mat3> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (svd.info() != Eigen::Success) {
    return false;
  }
  R = svd.matrixV() * svd.matrixU().transpose();
  if (R.determinant() < 0.0) {
    Mat3 V = svd.matrixV();
    V.col(2) *= -1.0;
    R = V * svd.matrixU().transpose();
  }
  t = cd - R * cs;

  double err2 = 0.0;
  for (size_t i = 0; i < src.size(); ++i) {
    err2 += (R * src[i] + t - dst[i]).squaredNorm();
  }
  rmse = std::sqrt(err2 / static_cast<double>(src.size()));
  return true;
}

}  // namespace continuous_icp_servo
