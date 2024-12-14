/*
 * OnlineDataProcessorRos.hpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include "open3d_slam/SlamWrapper.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam_ros/DataProcessorRos.hpp"
#include "open3d_slam_ros/GnssHandler.hpp"
#include "open3d_slam_ros/ImuBuffer.hpp"

namespace o3d_slam {

class OnlineRangeDataProcessorRos : public DataProcessorRos {
  using BASE = DataProcessorRos;

 public:
  OnlineRangeDataProcessorRos(ros::NodeHandlePtr nh);
  ~OnlineRangeDataProcessorRos() override = default;

  void initialize() override;
  void startProcessing() override;
  void processMeasurement(const PointCloud& cloud, const Time& timestamp) override;
  void processOdometry(const Transform& cloud, const Time& timestamp) override;

  bool initializeTheTransformBuffers_ = true;

  void staticTfCallback(const ros::TimerEvent&);
  void gnssCallback_(const sensor_msgs::NavSatFix::ConstPtr& gnssMsgPtr);

 private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  bool readCalibrationIfNeeded();
  void poseStampedCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void poseStampedWithCovarianceCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_ptr);
  void publishAddedImuMeas_(const Eigen::Matrix<double, 6, 1>& addedImuMeas, const ros::Time& stamp);

  std::optional<visualization_msgs::Marker> generateMarkersForSurfaceNormalVectors(const open3d::geometry::PointCloud& pointCloud,
                                                                                   const ros::Time& timestamp,
                                                                                   const o3d_slam::RgbaColorMap::Values& color);

  void addToPathMsg(nav_msgs::PathPtr pathPtr, const std::string& frameName, const ros::Time& stamp, const Eigen::Vector3d& t,
                    const int maxBufferLength);

  // std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  ros::Subscriber cloudSubscriber_;
  ros::Subscriber gnssSubscriber_;
  ros::Subscriber odometrySubscriber_;
  ros::Subscriber poseStampedCovarianceSubscriber_;
  ros::Subscriber poseStampedSubscriber_;
  ros::Subscriber imuSubscriber_;

  ros::Timer staticTfCallback_;

  bool poseStampedCallBackEnabled_ = false;
  bool odometryCallBackEnabled_ = false;
  bool poseStampedWithCovarianceCallBackEnabled_ = false;
  bool isAttitudeInitialized_ = false;
  bool isStaticTransformFound_ = false;

  std::shared_ptr<ImuBuffer> imuBufferPtr_;
  Transform lidarToImu_ = Transform::Identity();

  o3d_slam::RgbaColorMap colorMap_;

  // GNSS Handler
  std::shared_ptr<o3d_slam::GnssHandler> gnssHandlerPtr_;
  nav_msgs::PathPtr measGnss_worldGnssPathPtr_;

  // Initialization
  bool gpsInitialized_ = false;

  // tf2_ros::Buffer tfBuffer_;
  // tf2_ros::TransformListener tfListener_;
  // std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
};

}  // namespace o3d_slam
