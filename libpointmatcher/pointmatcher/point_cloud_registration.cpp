#include "point_cloud_registration.hpp"

PointCloudRegistrationCeres::PointCloudRegistrationCeres(const Eigen::Matrix<float, -1, -1>& source_cloud,
                                               const Eigen::Matrix<float, -1, -1>& target_cloud,
                                               const Eigen::Matrix<float, -1, -1>& source_normal,
                                               const Eigen::Matrix<float, -1, -1>& target_normal,
                                               Eigen::Matrix<float, 6, 6> degenerateEigenVectors,
                                               DegeneracySolverOptions& degenOptions)
    : degenerateEigenVector_(degenerateEigenVectors) {

  // Pointer is deleted by ceres an a new one is generated.
  problem_.reset(new ceres::Problem());

  // For now these 2 options are used together.
  if ((degenOptions.useSixDofRegularization_) || (degenOptions.useThreeDofRegularization_))
  {
    std::cout << " NL-REG is enabled with degeneracy regularization " << std::endl;
    // Generate and assign degeneracy regularization rediduals.
    for (Eigen::Index j = 0; j < degenerateEigenVectors.cols(); j++){
      //std::cout << " degenerateEigenVectors.col(j): " << std::endl << degenerateEigenVectors.col(j) << std::endl;
      Eigen::Matrix<double, 6, 1> localEig = degenerateEigenVectors.col(j).template cast<double>();
      if ((localEig != Eigen::Matrix<double, 6, 1>::Zero(6, 1)))
      {

        std::cout << " Degeneracy bound added. " << std::endl<< localEig << std::endl;
        float realWeight = degenOptions.regularizationWeight_;

        if (realWeight > 0.0001)
        {
          DegeneracyCost* error_termDegen = new DegeneracyCost(localEig, degenOptions.regularizationWeight_);
          ceres::LossFunction* data_loss = new ceres::ScaledLoss(new ceres::TrivialLoss(), realWeight, ceres::TAKE_OWNERSHIP);
          problem_->AddResidualBlock(new ceres::AutoDiffCostFunction<DegeneracyCost, 1, 3, 3>(error_termDegen), data_loss,
                                      translation_.data(), angleAxis_.data());
        }                  
      }
    }
  }
  
  for (size_t i = 0u; i < source_cloud.cols(); i++) {
    Eigen::Vector3d source_ = source_cloud.col(i).topRows(3).template cast<double>();
    Eigen::Vector3d target_ = target_cloud.col(i).topRows(3).template cast<double>();
    Eigen::Vector3d source_normal_ = source_normal.col(i).template cast<double>();
    Eigen::Vector3d target_normal_ = target_normal.col(i).template cast<double>();
    ceres::LossFunction* no_loss = new ceres::TrivialLoss();


    if ((degenOptions.usePointToPlane_))
    {
    // Point to plane (angle axis, translation and rotation separete 1x3 )
    ErrorTermWithNormalsAngleAxis* error_term = new ErrorTermWithNormalsAngleAxis(source_, target_, target_normal_);
    problem_->AddResidualBlock(new ceres::AutoDiffCostFunction<ErrorTermWithNormalsAngleAxis, 1, 3, 3>(error_term), no_loss,
                               translation_.data(), angleAxis_.data());
    }


    if ((degenOptions.usePointToPoint_))
    {
      // Point to point (angle axis, translation and rotation separete 1x3 )
      ErrorTermPointToPointAngleAxis* error_term = new ErrorTermPointToPointAngleAxis(source_, target_, target_normal_);
      problem_->AddResidualBlock(new ceres::AutoDiffCostFunction<ErrorTermPointToPointAngleAxis, 3, 3, 3>(error_term), NULL,
                                translation_.data(), angleAxis_.data());
    }

  }

}

void PointCloudRegistrationCeres::solve(ceres::Solver::Options options, ceres::Solver::Summary* summary, DegeneracySolverOptions& degenOptions, Eigen::Matrix<float, -1, -1> constraintProjectionMatrix) {
  // This should be always true if you want to do operations in between.
  options.update_state_every_iteration = true;
  ceres::Solve(options, problem_.get(), summary);
}


Eigen::Affine3d PointCloudRegistrationCeres::transformation() {
  Eigen::Affine3d affine = Eigen::Affine3d::Identity();
  affine.matrix() = Isometry3d(state_).matrix();
  return affine;
}

Eigen::Affine3d PointCloudRegistrationCeres::transformationSeparate() {
  
  Eigen::Affine3d affine = Eigen::Affine3d::Identity();
  state_.block<3,1>(0,0) = angleAxis_;
  state_.block<3,1>(3,0) = translation_;
  affine.matrix() = Isometry3d(state_).matrix();
  return affine;
}

float PointCloudRegistrationCeres::violation() {

Eigen::Matrix<float, 6, 1> state = state_.template cast<float>();
Eigen::Matrix<float, 6, 1> res = degenerateEigenVector_.transpose()*state;

// std::cout << "angleAxis_ : " << angleAxis_ << std::endl;
// std::cout << "translation_ : " << translation_ << std::endl;
// std::cout << "degenerateEigenVector_ : " << std::endl << degenerateEigenVector_ << std::endl;
// std::cout << "state : " << std::endl << state_ << std::endl;
// std::cout << "Res  : " << res << std::endl;

return res.sum();
}

