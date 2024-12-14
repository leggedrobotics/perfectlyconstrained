//! Header
#include "utils.hpp"

// C++ standard library
#include <filesystem>


Eigen::Isometry3d Isometry3d(const Eigen::Matrix<double, 6, 1>& isometry_vector) {
  return Isometry3(isometry_vector.data());
}

Eigen::Affine3d Affine3d(const Eigen::Matrix<double, 7, 1>& affine_vector) {
  return Affine3(affine_vector.data());
}