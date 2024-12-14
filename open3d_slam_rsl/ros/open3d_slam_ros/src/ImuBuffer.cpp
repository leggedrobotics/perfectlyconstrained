/*
Copyright 2023 by Julian Nubert, Robotic Systems Lab, ETH Zurich.
All rights reserved.
This file is released under the "BSD-3-Clause License".
Please see the LICENSE file that has been included as part of this package.
 */

// Implementation
#include "open3d_slam_ros/ImuBuffer.hpp"

// CPP
#include <iomanip>
#include <iostream>

// Workspace
#define YELLOW_START "\033[33m"
#define COLOR_END "\033[0m"
#define RED_START "\033[31m"
#define GREEN_START "\033[92m"

namespace o3d_slam {
// Constructor ---------------------------------------------------
// Constructor
ImuBuffer::ImuBuffer() {
  // Reset IMU Buffer
  timeToImuBuffer_.clear();

  // If low pass filter is used, initialize it

  imuSignalLowPassFilterPtr_ = std::make_unique<ImuSignalLowPassFilter>(60.0, 1.0 / 400.0);
}

// Public --------------------------------------------------------
// Returns actually added IMU measurements
Eigen::Matrix<double, 6, 1> ImuBuffer::addToImuBuffer(double ts, const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel) {
  // Check that imuBufferLength was set
  if (imuBufferLength_ < 0) {
    throw std::runtime_error("o3d_SLAM ImuBuffer: imuBufferLength has to be set by the user.");
  }

  // On anymal created artifacts.
  // Copy of IMU measurements
  Eigen::Matrix<double, 6, 1> filteredImuMeas;
  // Potentially low pass filter IMU measurements
  // if (useImuSignalLowPassFilter_) {
  // filteredImuMeas = imuSignalLowPassFilterPtr_->filter(linearAcc, angularVel);
  //} else {
  filteredImuMeas << linearAcc, angularVel;
  //}

  // Convert to gtsam type
  ImuMeasurement imuMeas;
  imuMeas.timestamp = ts;
  // imuMeas.acceleration = filteredImuMeas.head<3>();
  // imuMeas.angularVelocity = filteredImuMeas.tail<3>();

  imuMeas.acceleration = linearAcc;
  imuMeas.angularVelocity = angularVel;

  // Add to buffer
  {
    // Writing to IMU buffer --> acquire mutex
    const std::lock_guard<std::mutex> writeInBufferLock(writeInBufferMutex_);
    timeToImuBuffer_[ts] = imuMeas;
    // Update latest timestamp in buffer
    if (ts > tLatestInBuffer_) {
      tLatestInBuffer_ = ts;
    }
  }

  // If IMU buffer is too large, remove first element
  if (timeToImuBuffer_.size() > imuBufferLength_) {
    timeToImuBuffer_.erase(timeToImuBuffer_.begin());
  }

  if (timeToImuBuffer_.size() > imuBufferLength_) {
    std::ostringstream errorStream;
    errorStream << "o3d_SLAM-ImuBuffer"
                << " IMU Buffer has grown too large. It contains " << timeToImuBuffer_.size() << " measurements instead of "
                << imuBufferLength_ << ".";
    throw std::runtime_error(errorStream.str());
  }

  return filteredImuMeas;
}

double ImuBuffer::getLatestTimestampInBuffer() {
  // Reading from IMU buffer --> acquire mutex
  const std::lock_guard<std::mutex> writeInBufferLock(writeInBufferMutex_);
  return tLatestInBuffer_;
}

void ImuBuffer::getLastTwoMeasurements(TimeToImuMap& imuMap) {
  TimeToImuMap::iterator endItr = --(timeToImuBuffer_.end());
  TimeToImuMap::iterator previousItr = --(--(timeToImuBuffer_.end()));

  // Write into IMU Map
  imuMap[previousItr->first] = previousItr->second;
  imuMap[endItr->first] = endItr->second;
}

bool ImuBuffer::estimateAttitudeFromImu(Eigen::Quaterniond& initAttitude, double& gravityMagnitude, Eigen::Vector3d& gyrBias) {
  // Make sure that imuBuffer is long enough
  if (imuBufferLength_ < (imuRate_ * imuPoseInitWaitSecs_)) {
    throw std::runtime_error(
        "ImuBufferLength is not large enough for "
        "initialization. Must be at least 1 second.");
  }

  // Get timestamp of first message for lookup
  if (timeToImuBuffer_.size() < (imuRate_ * imuPoseInitWaitSecs_)) {
    return false;
  } else {
    // Accumulate Acceleration part of IMU Messages
    Eigen::Vector3d initAccMean(0.0, 0.0, 0.0), initGyrMean(0.0, 0.0, 0.0);
    for (auto& itr : timeToImuBuffer_) {
      initAccMean += itr.second.acceleration;
      initGyrMean += itr.second.angularVelocity;
    }

    // Average IMU measurements and set assumed gravity direction
    initAccMean /= timeToImuBuffer_.size();
    gravityMagnitude = initAccMean.norm();
    Eigen::Vector3d gUnitVecInWorld = Eigen::Vector3d(0.0, 0.0, 1.0);  // ROS convention

    // Normalize gravity vectors to remove the affect of gravity magnitude from
    // place-to-place
    initAccMean.normalize();
    initAttitude = Eigen::Quaterniond().setFromTwoVectors(initAccMean, gUnitVecInWorld);

    // Gyro
    initGyrMean /= timeToImuBuffer_.size();
    gyrBias = initGyrMean;

    Eigen::Matrix3d transform(initAttitude);
    double yaw = 0.0;
    double pitch = -asin(transform(2, 0));
    double roll = atan2(transform(2, 1), transform(2, 2));
    yaw = atan2(transform(1, 0) / cos(pitch), transform(0, 0) / cos(pitch));
    // std::cout << YELLOW_START << "Iinitial Attitude Estimation o3d_SLAM " << GREEN_START << " Initial Roll/Pitch/Yaw(deg):" << COLOR_END
    // << roll * 180 / M_PI
    //          << "," << pitch * 180 / M_PI << "," << yaw * 180 / M_PI << std::endl;

    // Calculate robot initial orientation using gravity vector.
    std::cout << YELLOW_START << "o3d_SLAM-ImuBuffer" << COLOR_END << " Gravity Magnitude: " << gravityMagnitude << std::endl;
    std::cout << YELLOW_START << "o3d_SLAM-ImuBuffer" << COLOR_END << " Mean IMU Acceleration Vector(x,y,z): " << initAccMean.transpose()
              << " - Gravity Unit Vector(x,y,z): " << gUnitVecInWorld.transpose() << std::endl;
    std::cout << YELLOW_START << "o3d_SLAM-ImuBuffer" << GREEN_START << " Yaw/Pitch/Roll(deg): " << yaw * (180.0 / M_PI) << ", "
              << pitch * (180.0 / M_PI) << ", " << roll * (180.0 / M_PI) << COLOR_END << std::endl;
    std::cout << YELLOW_START << "o3d_SLAM-ImuBuffer" << COLOR_END << "  Gyro bias(x,y,z): " << initGyrMean.transpose() << std::endl;
  }
  return true;
}

bool ImuBuffer::getIMUBufferIteratorsInInterval(const double& tsStart, const double& tsEnd, TimeToImuMap::iterator& startIterator,
                                                TimeToImuMap::iterator& endIterator) {
  // Check if timestamps are in correct order
  if (tsStart >= tsEnd) {
    std::cerr << YELLOW_START << "GMsf-ImuBuffer" << RED_START << " IMU Lookup Timestamps are not correct ts_start(" << std::fixed
              << tsStart << ") >= ts_end(" << tsEnd << ")\n";
    return false;
  }

  // Get Iterator Belonging to ts_start
  startIterator = timeToImuBuffer_.lower_bound(tsStart);
  // Get Iterator Belonging to ts_end
  endIterator = timeToImuBuffer_.lower_bound(tsEnd);

  // Check if it is first value in the buffer which means there is no value
  // before to interpolate with
  if (startIterator == timeToImuBuffer_.begin()) {
    std::cerr << YELLOW_START << "GMsf-ImuBuffer" << RED_START
              << " Lookup requires first message of IMU buffer, cannot "
                 "Interpolate back, "
                 "Lookup Start/End: "
              << std::fixed << tsStart << "/" << tsEnd << ", Buffer Start/End: " << timeToImuBuffer_.begin()->first << "/"
              << timeToImuBuffer_.rbegin()->first << std::endl;
    return false;
  }

  // Check if lookup start time is ahead of buffer start time
  if (startIterator == timeToImuBuffer_.end()) {
    std::cerr << YELLOW_START << "GMsf-ImuBuffer" << RED_START
              << " IMU Lookup start time ahead latest IMU message in the "
                 "buffer, lookup: "
              << tsStart << ", latest IMU: " << timeToImuBuffer_.rbegin()->first << std::endl;
    return false;
  }

  // Check if last value is valid
  if (endIterator == timeToImuBuffer_.end()) {
    std::cerr << YELLOW_START << "GMsf-ImuBuffer" << RED_START << " Lookup is past IMU buffer, with lookup Start/End: " << std::fixed
              << tsStart << "/" << tsEnd << " and latest IMU: " << timeToImuBuffer_.rbegin()->first << std::endl;
    --endIterator;
  }

  // Check if two IMU messages are different
  if (startIterator == endIterator) {
    std::cerr << YELLOW_START << "GMsf-ImuBuffer" << RED_START
              << " Not Enough IMU values between timestamps , with Start/End: " << std::fixed << tsStart << "/" << tsEnd
              << ", with diff: " << tsEnd - tsStart << std::endl;
    return false;
  }

  // If everything is good
  return true;
}
}  // namespace o3d_slam
