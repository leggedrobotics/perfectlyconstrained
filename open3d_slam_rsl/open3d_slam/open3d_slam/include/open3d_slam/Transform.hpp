/*
 * Transform.hpp
 *
 *  Created on: Nov 8, 2021
 *      Author: jelavice
 */

#pragma once
#include <Eigen/Dense>
#include <cstdint>
#include "open3d_slam/time.hpp"

namespace o3d_slam {

using Transform = Eigen::Isometry3d;

struct DeeperICPLogs {
  Time time_;
  Transform transform_;
  Eigen::Matrix<float, 3, 6> degenerateDirections_{Eigen::Matrix<float, 3, 6>::Zero(3, 6)};
  float totalICPtime{0.0f};
  int numberOfIterations{0};
  Eigen::Matrix<float, 6, 1> localizationCategory{Eigen::Matrix<float, 6, 1>::Constant(6, 1, 1)};
  //! Overlap between reading and reference point clouds. [0,1]
  float pointCloudOverlap_{0};
  //! Residual error of the registration. Unit: Meters.
  float residualError_{0};

  int nbMatches_{0};
};

struct TimestampedTransform {
  Time time_;
  Transform transform_;
};

TimestampedTransform interpolate(const TimestampedTransform& start, const TimestampedTransform& end, const Time& time);

Transform makeTransform(const Eigen::Vector3d& p, const Eigen::Quaterniond& q);

}  // namespace o3d_slam
