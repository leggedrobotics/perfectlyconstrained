
/* Copyright (c) This file is based on the version available in astrobee project. (https://github.com/nasa/astrobee)
 * Modified by Turcan Tuna
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */
#pragma once

#include <string>
#include <vector>
#include <Eigen/Core>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/autodiff_local_parameterization.h>

#include <Eigen/Geometry>

struct DegeneracySolverOptions
{
bool isEnabled_{ true };
bool usePointToPoint_{ false };
bool usePointToPlane_{ true };
bool useBoundConstraints_{ false };
bool useSixDofRegularization_{ false };
bool useThreeDofRegularization_{ false };
float regularizationWeight_{ 0.0f };
};

// Assumes compact angle axis (3d vector where norm gives the angle) parameterization for rotations
// First 3 values of isometry_data are the compact angle axis, next 3 are the translation
template <typename T>
Eigen::Transform<T, 3, Eigen::Isometry> Isometry3(const T* isometry_data);

// Assumes compact angle axis (3d vector where norm gives the angle) parameterization for rotations
// First 3 values of isometry_data are the compact angle axis, next 3 are the translation, last is scale
template <typename T>
Eigen::Transform<T, 3, Eigen::Affine> Affine3(const T* affine_data);

Eigen::Isometry3d Isometry3d(const Eigen::Matrix<double, 6, 1>& isometry_vector);
Eigen::Affine3d Affine3d(const Eigen::Matrix<double, 7, 1>& affine_vector);


// For full matrix
template <typename T>
Eigen::Transform<T, 3, Eigen::Isometry> Isometry3(const T* isometry_data) {
  Eigen::Matrix<T, 3, 3> rotation;
  ceres::AngleAxisToRotationMatrix(isometry_data, rotation.data());
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> translation(&isometry_data[3]);
  Eigen::Transform<T, 3, Eigen::Isometry> isometry_3(Eigen::Transform<T, 3, Eigen::Isometry>::Identity());
  isometry_3.linear() = rotation;
  isometry_3.translation() = translation;
  return isometry_3;
}

// For separate rotation and translation
template <typename T>
Eigen::Transform<T, 3, Eigen::Isometry> Isometry3(const T* rotation, const T* translation) {
  Eigen::Matrix<T, 3, 3> rotations;
  ceres::AngleAxisToRotationMatrix(rotation, rotations.data());
  Eigen::Map<const Eigen::Matrix<T, 3, 1>> translations(&translation[3]);
  Eigen::Transform<T, 3, Eigen::Isometry> isometry_3(Eigen::Transform<T, 3, Eigen::Isometry>::Identity());
  isometry_3.linear() = rotations;
  isometry_3.translation() = translations;
  return isometry_3;
}

template <typename T>
Eigen::Transform<T, 3, Eigen::Affine> Affine3(const T* affine_data) {
  const Eigen::Transform<T, 3, Eigen::Isometry> isometry_3 = Isometry3(affine_data);
  const T scale = affine_data[6];
  Eigen::Transform<T, 3, Eigen::Affine> affine_3;
  affine_3.linear() = scale * isometry_3.linear();
  affine_3.translation() = isometry_3.translation();
  return affine_3;
}