/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

#pragma once
// C++
#include <map>
#include <memory>
#include <mutex>

// Eigen
#include <Eigen/Dense>

// Workspace
#include "open3d_slam_ros/ImuSignalLowPassFilter.hpp"

namespace o3d_slam {
struct ImuMeasurement {
  double timestamp;
  Eigen::Vector3d acceleration;
  Eigen::Vector3d angularVelocity;
};

// Datatypes
// Map from time to 6D IMU measurements
typedef std::map<double, ImuMeasurement, std::less<double>, Eigen::aligned_allocator<std::pair<const double, ImuMeasurement>>> TimeToImuMap;

// Actual Class
class ImuBuffer {
 public:
  // Constructor
  ImuBuffer();

  // Destructor
  virtual ~ImuBuffer() = default;

  // Add to buffers
  Eigen::Matrix<double, 6, 1> addToImuBuffer(double ts, const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel);

  // Getters
  double getImuRate() const { return imuRate_; }
  double getLatestTimestampInBuffer();
  void getLastTwoMeasurements(TimeToImuMap& imuMap);

  // Public member functions
  /// Determine initial IMU pose w.r.t to gravity vector pointing up
  bool estimateAttitudeFromImu(Eigen::Quaterniond& initAttitude, double& gravityMagnitude, Eigen::Vector3d& gyrBias);

  // Integrate NavState from Timestamp
  bool getIMUBufferIteratorsInInterval(const double& tsStart, const double& tsEnd, TimeToImuMap::iterator& startIterator,
                                       TimeToImuMap::iterator& endIterator);

 private:
  // Member variables
  TimeToImuMap timeToImuBuffer_;  // IMU buffer
  double imuRate_ = 400;  // Rate of IMU input (Hz) - Used to calculate minimum measurements needed to calculate gravity and init attitude
  int imuBufferLength_ = 800;
  double imuPoseInitWaitSecs_ = 1.5;  // Multiplied with _imuRate
  int verboseLevel_ = 0;
  double tLatestInBuffer_ = 0.0;
  std::mutex writeInBufferMutex_;
  bool useImuSignalLowPassFilter_ = true;

  // Low pass filter
  std::unique_ptr<ImuSignalLowPassFilter> imuSignalLowPassFilterPtr_;
};
}  // namespace o3d_slam
