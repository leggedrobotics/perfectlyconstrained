/*
 * DataProcessorRos.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#include "open3d_slam_ros/DataProcessorRos.hpp"
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "open3d_slam/magic.hpp"
#include "open3d_slam/typedefs.hpp"

namespace o3d_slam {

DataProcessorRos::DataProcessorRos(ros::NodeHandlePtr nh) : nh_(nh) {}

void DataProcessorRos::initCommonRosStuff() {
  cloudTopic_ = nh_->param<std::string>("cloud_topic", "");
  odometryTopic_ = nh_->param<std::string>("odometry_topic", "");
  poseStampedTopic_ = nh_->param<std::string>("pose_stamped_topic", "");
  imuTopic_ = nh_->param<std::string>("imu_topic", "/sensors/imu");
  poseStampedWithCovarianceTopic_ = nh_->param<std::string>("pose_stamped_with_covariance_topic", "");

  std::cout << "Cloud topic is given as " << cloudTopic_ << std::endl;
  std::cout << "Odometry topic is given as " << odometryTopic_ << std::endl;
  std::cout << "Pose Stamped topic is given as " << poseStampedTopic_ << std::endl;
  std::cout << "Pose Stamped With Covariance topic is given as " << poseStampedWithCovarianceTopic_ << std::endl;

  const bool localReadGps = nh_->param<bool>("use_gps_for_ground_truth", false);
  if (localReadGps) {
    pubMeasWorldGnssPath_ = nh_->advertise<nav_msgs::Path>("measGnss_path_world_gnss", 1, true);
  }
  rawCloudPub_ = nh_->advertise<sensor_msgs::PointCloud2>("raw_cloud", 1, true);
  registeredCloudPub_ = nh_->advertise<sensor_msgs::PointCloud2>("registered_cloud", 1, true);

  const bool localOfflineReplay = nh_->param<bool>("is_read_from_rosbag", false);
  if (localOfflineReplay) {
    offlineBestGuessPathPub_ = nh_->advertise<nav_msgs::Path>("best_guess_path", 1, true);
    offlinePathPub_ = nh_->advertise<nav_msgs::Path>("tracked_path", 1, true);
    offlineDifferenceLinePub_ = nh_->advertise<visualization_msgs::Marker>("differenceLines", true);
    condNumberArrowPublisher_ = nh_->advertise<visualization_msgs::MarkerArray>("localizabilityArrows", false);
  }

  surfaceNormalPub_ = nh_->advertise<visualization_msgs::Marker>("surfaceNormals", true);
  addedImuMeasPub_ = addedImuMeasPub_ = nh_->advertise<sensor_msgs::Imu>("added_imu_measurements", 40);
  numAccumulatedRangeDataDesired_ = nh_->param<int>("num_accumulated_range_data", 1);
}

void DataProcessorRos::processMeasurement(const PointCloud& cloud, const Time& timestamp) {
  std::cout << "Warning you have not implemented processMeasurement!!! \n";
}

void DataProcessorRos::processOdometry(const Transform& transform, const Time& timestamp) {
  std::cout << "Warning you have not implemented processOdometry!!! \n";
}

std::shared_ptr<SlamWrapper> DataProcessorRos::getSlamPtr() {
  return slam_;
}

void DataProcessorRos::accumulateAndProcessRangeData(const PointCloud& cloud, const Time& timestamp) {
  const size_t minNumCloudsReceived = magic::skipFirstNPointClouds;
  if (numPointCloudsReceived_ < minNumCloudsReceived) {
    ++numPointCloudsReceived_;
    return;
    // somehow the first cloud can be missing a lot of points when running with ouster os-128 on the robot
    // if we skip that first measurement, it all works okay
    // we skip first five, just to be extra safe
  }

  accumulatedCloud_ += cloud;
  ++numAccumulatedRangeDataCount_;
  if (numAccumulatedRangeDataCount_ < numAccumulatedRangeDataDesired_) {
    return;
  }

  if (accumulatedCloud_.IsEmpty()) {
    std::cout << "Trying to insert and empyt cloud!!! Skipping the measurement \n";
    return;
  }

  processMeasurement(accumulatedCloud_, timestamp);

  numAccumulatedRangeDataCount_ = 0;
  accumulatedCloud_.Clear();
}

void DataProcessorRos::processOdometryData(const Transform& transform, const Time& timestamp) {
  processOdometry(transform, timestamp);
}

}  // namespace o3d_slam
