#include "continuous_icp_servo/icp_solver.hpp"

#include <cmath>
#include <algorithm>
#include <limits>
#include <numeric>

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

Cloud estimateNormals(
  const Cloud & points, const SpatialIndex & index,
  double radius, size_t max_neighbors)
{
  Cloud normals(points.size(), Point::Zero());
  for (size_t i = 0; i < points.size(); ++i) {
    auto ids = index.nearby(points[i], radius);
    if (ids.size() < 6) {
      continue;
    }
    if (ids.size() > max_neighbors) {
      std::nth_element(
        ids.begin(), ids.begin() + static_cast<std::ptrdiff_t>(max_neighbors), ids.end(),
        [&](int a, int b) {
          return (points[a] - points[i]).squaredNorm() <
                 (points[b] - points[i]).squaredNorm();
        });
      ids.resize(max_neighbors);
    }
    Point center = Point::Zero();
    for (const int id : ids) {
      center += points[id];
    }
    center /= static_cast<double>(ids.size());
    Mat3 covariance = Mat3::Zero();
    for (const int id : ids) {
      const Point d = points[id] - center;
      covariance.noalias() += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Mat3> eig(covariance);
    if (eig.info() == Eigen::Success) {
      const Point n = eig.eigenvectors().col(0).normalized();
      if (n.allFinite()) {
        normals[i] = n;
      }
    }
  }
  return normals;
}

bool pointToPlaneRefine(
  const Cloud & src, const Cloud & target, const SpatialIndex & target_index,
  const Cloud & target_normals, const Mat4 & initial, Mat4 & refined,
  double & rmse, int & inliers, double & overlap,
  double dmax, int max_iterations, size_t max_points)
{
  if (src.empty() || target.empty() || target.size() != target_normals.size()) {
    return false;
  }
  Mat4 T = initial;
  double last_rmse = std::numeric_limits<double>::infinity();
  const size_t stride = std::max<size_t>(1, (src.size() + max_points - 1) / max_points);

  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    std::vector<Eigen::Matrix<double, 6, 1>> rows;
    std::vector<double> residuals;
    rows.reserve(std::min(src.size(), max_points));
    residuals.reserve(rows.capacity());
    const Mat3 R = T.block<3, 3>(0, 0);
    const Point tr = T.block<3, 1>(0, 3);
    for (size_t i = 0; i < src.size(); i += stride) {
      const Point p = R * src[i] + tr;
      Point q;
      double distance = 0.0;
      int target_id = -1;
      if (!target_index.nearest(p, dmax, q, distance, &target_id) || target_id < 0) {
        continue;
      }
      const Point & n = target_normals[static_cast<size_t>(target_id)];
      if (n.squaredNorm() < 0.5) {
        continue;
      }
      const double residual = (p - q).dot(n);
      Eigen::Matrix<double, 6, 1> row;
      row << p.cross(n), n;
      rows.push_back(row);
      residuals.push_back(residual);
    }
    if (rows.size() < 30) {
      break;
    }

    std::vector<double> abs_residuals(residuals.size());
    std::transform(
      residuals.begin(), residuals.end(), abs_residuals.begin(),
      [](double value) {return std::abs(value);});
    const auto median_it = abs_residuals.begin() + abs_residuals.size() / 2;
    std::nth_element(abs_residuals.begin(), median_it, abs_residuals.end());
    const double robust_limit = std::max(dmax, 2.5 * (*median_it) + 1e-6);

    Eigen::Matrix<double, 6, 6> normal_matrix = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> rhs = Eigen::Matrix<double, 6, 1>::Zero();
    double squared_error = 0.0;
    inliers = 0;
    for (size_t i = 0; i < rows.size(); ++i) {
      if (std::abs(residuals[i]) > robust_limit) {
        continue;
      }
      normal_matrix.noalias() += rows[i] * rows[i].transpose();
      rhs.noalias() -= rows[i] * residuals[i];
      squared_error += residuals[i] * residuals[i];
      ++inliers;
    }
    if (inliers < 30) {
      break;
    }
    const Eigen::Matrix<double, 6, 1> delta =
      normal_matrix.completeOrthogonalDecomposition().solve(rhs);
    if (!delta.allFinite()) {
      break;
    }

    Point rotation_vector = delta.head<3>();
    Point translation = delta.tail<3>();
    const double rotation_norm = rotation_vector.norm();
    const double translation_norm = translation.norm();
    const double max_rotation = 0.5 * M_PI / 180.0;
    if (rotation_norm > max_rotation) {
      rotation_vector *= max_rotation / rotation_norm;
    }
    if (translation_norm > 0.003) {
      translation *= 0.003 / translation_norm;
    }

    Mat3 R_step = Mat3::Identity();
    if (rotation_vector.norm() > 1e-12) {
      R_step = Eigen::AngleAxisd(
        rotation_vector.norm(), rotation_vector.normalized()).toRotationMatrix();
    }
    T.block<3, 3>(0, 0) = R_step * T.block<3, 3>(0, 0);
    T.block<3, 1>(0, 3) = R_step * T.block<3, 1>(0, 3) + translation;
    rmse = std::sqrt(squared_error / static_cast<double>(inliers));
    if (std::abs(last_rmse - rmse) < 1e-5) {
      break;
    }
    last_rmse = rmse;
  }

  refined = T;
  inliers = 0;
  double squared_error = 0.0;
  const Mat3 R = T.block<3, 3>(0, 0);
  const Point tr = T.block<3, 1>(0, 3);
  size_t sampled = 0;
  for (size_t i = 0; i < src.size(); i += stride) {
    ++sampled;
    const Point p = R * src[i] + tr;
    Point q;
    double distance = 0.0;
    int target_id = -1;
    if (!target_index.nearest(p, dmax, q, distance, &target_id) || target_id < 0) {
      continue;
    }
    const Point & n = target_normals[static_cast<size_t>(target_id)];
    if (n.squaredNorm() < 0.5) {
      continue;
    }
    const double residual = (p - q).dot(n);
    squared_error += residual * residual;
    ++inliers;
  }
  overlap = sampled == 0 ? 0.0 : static_cast<double>(inliers) / static_cast<double>(sampled);
  rmse = inliers == 0 ? std::numeric_limits<double>::infinity() :
    std::sqrt(squared_error / static_cast<double>(inliers));
  return inliers >= 30 && std::isfinite(rmse);
}

}  // namespace continuous_icp_servo
