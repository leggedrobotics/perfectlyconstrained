#pragma once

#include <vector>

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include "PointMatcher.h"
#include "PointMatcherPrivate.h"
#include "error_term.hpp"
#include "utils.hpp"

class PointCloudRegistrationCeres{

 public:
  PointCloudRegistrationCeres(const Eigen::Matrix<float, -1, -1>& source_cloud,
                                               const Eigen::Matrix<float, -1, -1>& target_cloud,
                                               const Eigen::Matrix<float, -1, -1>& source_normal,
                                               const Eigen::Matrix<float, -1, -1>& target_normal,
                                               Eigen::Matrix<float, 6, 6> eigenVector,
                                               DegeneracySolverOptions& degenOptions);

  void solve(ceres::Solver::Options options, ceres::Solver::Summary* Summary, DegeneracySolverOptions& degenOptions, Eigen::Matrix<float, -1, -1> constraintProjectionMatrix);
  Eigen::Affine3d transformation();
  Eigen::Affine3d transformationSeparate();
  float violation();

 private:
  std::unique_ptr<ceres::Problem> problem_;
  Eigen::Matrix<double, 6, 1> state_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 3, 1> translation_ = Eigen::Matrix<double, 3, 1>::Zero();
  Eigen::Matrix<double, 3, 1> angleAxis_ = Eigen::Matrix<double, 3, 1>::Zero();
  Eigen::Matrix<double, 1, 1> residualWeight_ = Eigen::Matrix<double, 1, 1>::Constant(1, 1, 1);
  Eigen::Matrix<float, 6, 6> degenerateEigenVector_ = Eigen::Matrix<float, 6, 6>::Zero();


};