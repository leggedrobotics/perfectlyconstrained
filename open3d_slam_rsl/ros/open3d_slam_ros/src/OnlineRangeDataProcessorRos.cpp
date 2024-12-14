/*
 * OnlineRangeDataProcessorRos.cpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#include "open3d_slam_ros/OnlineRangeDataProcessorRos.hpp"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include "open3d_conversions/open3d_conversions.h"
#include "open3d_slam/math.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam_ros/SlamWrapperRos.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"
namespace o3d_slam {

OnlineRangeDataProcessorRos::OnlineRangeDataProcessorRos(ros::NodeHandlePtr nh) : BASE(nh), tfListener_(tfBuffer_) {}

void OnlineRangeDataProcessorRos::initialize() {
  initCommonRosStuff();
  slam_ = std::make_shared<SlamWrapperRos>(nh_);
  imuBufferPtr_ = std::make_shared<ImuBuffer>();
  // If this is calling the ros wrapper version. Which overrides the base. The base function is later called within the overridden version.
  slam_->loadParametersAndInitialize();

  if (slam_->useGPSforGroundTruth_) {
    // GNSS Handler
    gnssHandlerPtr_ = std::make_shared<o3d_slam::GnssHandler>();
    measGnss_worldGnssPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);
  }
}

bool OnlineRangeDataProcessorRos::readCalibrationIfNeeded() {
  if (slam_->frames_.rangeSensorFrame == "default") {
    ROS_WARN_STREAM_THROTTLE(
        2.0, "Range sensor frame is not set yet (cloud didnt arrive yet). Delaying the transformation look-up. (Throttled 2s)");
    return false;
  }

  // The frames are identical. This is often not the case since the odometry follows a certain frame like base but the point clouds arrive
  // in the lidar frame.
  if ((slam_->frames_.rangeSensorFrame == slam_->frames_.assumed_external_odometry_tracked_frame) || !slam_->isUsingOdometryTopic()) {
    slam_->setExternalOdometryFrameToCloudFrameCalibration(Eigen::Isometry3d::Identity());
    return true;
  }

  if (!isStaticTransformFound_) {
    try {
      // Waits for the transform to be available. After this we dont need to have timeout for the lookup itself
      if (!tfBuffer_.canTransform(slam_->frames_.rangeSensorFrame, slam_->frames_.assumed_external_odometry_tracked_frame, ros::Time(0.0),
                                  ros::Duration(0.2))) {
        ROS_WARN_STREAM_THROTTLE(0.5, "Transform not available yet: [" << slam_->frames_.rangeSensorFrame << "] to ["
                                                                       << slam_->frames_.assumed_external_odometry_tracked_frame
                                                                       << "]. Thottled 0.5s");
        return false;
      }

      auto tfTransformation = tfBuffer_.lookupTransform(
          slam_->frames_.rangeSensorFrame, slam_->frames_.assumed_external_odometry_tracked_frame, ros::Time(0.0), ros::Duration(0.0));

      ROS_INFO_STREAM("\033[92m"
                      << "Found the transform between " << slam_->frames_.rangeSensorFrame << " and "
                      << slam_->frames_.assumed_external_odometry_tracked_frame << "\033[0m");
      ROS_INFO_STREAM("\033[92m"
                      << "You dont believe me? Here it is:\n " << tfTransformation << "\033[0m");

      // Set the frame transformation between the external odometry frame and the range sensor frame.
      slam_->setExternalOdometryFrameToCloudFrameCalibration(tf2::transformToEigen(tfTransformation));

      if (slam_->isIMUattitudeInitializationEnabled()) {
        // Waits for the transform to be available. After this we dont need to have timeout for the lookup itself
        if (!tfBuffer_.canTransform(slam_->frames_.rangeSensorFrame, slam_->frames_.imuFrame, ros::Time(0.0), ros::Duration(0.2))) {
          ROS_WARN_STREAM("Transform not available yet: [" << slam_->frames_.rangeSensorFrame << "] to [" << slam_->frames_.imuFrame
                                                           << "].");
          return false;
        }

        // <X frame to Y frame> for the tf look up.
        auto RangeSensorFrameToimuFrame =
            tfBuffer_.lookupTransform(slam_->frames_.rangeSensorFrame, slam_->frames_.imuFrame, ros::Time(0.0), ros::Duration(0.0));

        ROS_INFO_STREAM("\033[92m"
                        << "Found the transform between " << slam_->frames_.rangeSensorFrame << " and " << slam_->frames_.imuFrame
                        << "\033[0m");
        ROS_INFO_STREAM("\033[92m"
                        << "You dont believe me? Here it is:\n " << RangeSensorFrameToimuFrame << "\033[0m");

        // Set the frame transformation between the external odometry frame and the range sensor frame.
        lidarToImu_.matrix() = tf2::transformToEigen(RangeSensorFrameToimuFrame).matrix();  //.inverse();
      }
      return true;

    } catch (const tf2::TransformException& exception) {
      ROS_WARN_STREAM("Caught exception while looking for the transform frame: " << slam_->frames_.rangeSensorFrame << " to "
                                                                                 << slam_->frames_.assumed_external_odometry_tracked_frame
                                                                                 << "." << exception.what());
      return false;
    }
  } else {
    ROS_WARN_STREAM("This is unexpected, something is off.");
    return false;
  }
}

void OnlineRangeDataProcessorRos::startProcessing() {
  slam_->startWorkers();

  // The point cloud subscriber
  cloudSubscriber_ = nh_->subscribe(cloudTopic_, 2, &OnlineRangeDataProcessorRos::cloudCallback, this, ros::TransportHints().tcpNoDelay());

  if (slam_->useGPSforGroundTruth_) {
    gnssSubscriber_ =
        nh_->subscribe(slam_->gpsTopic_, 40, &OnlineRangeDataProcessorRos::gnssCallback_, this, ros::TransportHints().tcpNoDelay());
  }

  // Redundant listening of pose topics. It is really tiresome to the developer to keep the support for all types.
  poseStampedSubscriber_ =
      nh_->subscribe(poseStampedTopic_, 40, &OnlineRangeDataProcessorRos::poseStampedCallback, this, ros::TransportHints().tcpNoDelay());
  odometrySubscriber_ =
      nh_->subscribe(odometryTopic_, 40, &OnlineRangeDataProcessorRos::odometryCallback, this, ros::TransportHints().tcpNoDelay());
  poseStampedCovarianceSubscriber_ =
      nh_->subscribe(poseStampedWithCovarianceTopic_, 40, &OnlineRangeDataProcessorRos::poseStampedWithCovarianceCallback, this,
                     ros::TransportHints().tcpNoDelay());

  if (slam_->isIMUattitudeInitializationEnabled()) {
    imuSubscriber_ = nh_->subscribe<sensor_msgs::Imu>(imuTopic_, 40, &OnlineRangeDataProcessorRos::imuCallback, this,
                                                      ros::TransportHints().tcpNoDelay());
  } else {
    isAttitudeInitialized_ = true;
  }

  // A timer to read the static calibration we between the provided tracked frame by odometry and the point cloud frame.
  staticTfCallback_ = nh_->createTimer(ros::Duration(0.1), &OnlineRangeDataProcessorRos::staticTfCallback, this);

  std::cout << " Open3d_slam Subscribers are set." << std::endl;

  // Number of spinners should be equal to the number of bscribers
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
  slam_->stopWorkers();
}

void OnlineRangeDataProcessorRos::staticTfCallback(const ros::TimerEvent&) {
  // Transform tfQueriedLatestOdometry;
  // bool succ= o3d_slam::lookupTransform("lidar", "imu_link", ros::Time(0.0), tfBuffer_, tfQueriedLatestOdometry);

  if (!slam_->isUsingOdometryTopic()) {
    slam_->setExternalOdometryFrameToCloudFrameCalibration(Eigen::Isometry3d::Identity());
    staticTfCallback_.stop();
  }

  if (readCalibrationIfNeeded()) {
    // If IMU initialization is enabled we need to wait for the IMU callback to initialize the attitude.
    if (!slam_->isIMUattitudeInitializationEnabled()) {
      // This casts isometry3d to affine3d.
      Eigen::Affine3d eigenTransform = slam_->getExternalOdometryFrameToCloudFrameCalibration();
      geometry_msgs::TransformStamped calibrationAsTransform = tf2::eigenToTransform(eigenTransform);

      geometry_msgs::PoseStamped odomPose;

      const auto latestOdomMeasurement = slam_->getLatestOdometryPoseMeasurement();
      odomPose.pose = o3d_slam::getPose(latestOdomMeasurement.transform_.matrix());

      // Actual transformation applied to the odometry measurement. Reads as pose of Lidar frame in the external odometry frame.
      geometry_msgs::PoseStamped odomPose_transformed;
      tf2::doTransform(odomPose, odomPose_transformed, calibrationAsTransform);

      // odomPose_transformed.position=odomPose.position;
      odomPose_transformed.pose.orientation.w = 1.0;
      odomPose_transformed.pose.orientation.z = 0.0;
      odomPose_transformed.pose.orientation.y = 0.0;
      odomPose_transformed.pose.orientation.x = 0.0;
      ROS_INFO("Initial Transform is set. Nice. The rotation is enforced to be identity.");

      if (!slam_->isUseExistingMapEnabled()) {
        slam_->setInitialTransform(o3d_slam::getTransform(odomPose_transformed.pose).matrix());
      }
    }

    ROS_INFO("Static TF reader callback is terminated after successfully reading the transform.");
    staticTfCallback_.stop();
  }
}

void OnlineRangeDataProcessorRos::processMeasurement(const PointCloud& cloud, const Time& timestamp) {
  if (!slam_->isUseExistingMapEnabled() && slam_->isUsingOdometryTopic()) {
    if (!slam_->isInitialTransformSet()) {
      ROS_WARN_THROTTLE(1, "Initial Transform not set yet, skipping the measurement. Throttled 1s");
      return;
    }
  }

  // Add the range scan to the pointcloud processing buffer. This is actually a buffer with size 1, so no queue.
  // The add range scan comes first since scan2scan odometry would create its own odometry measurements.
  if (!slam_->addRangeScan(cloud, timestamp)) {
    ROS_WARN("Failed to add range scan. This is unexpected. Skipping the measurement.");
    return;
  }

  if (slam_->isOdometryPoseBufferEmpty()) {
    ROS_WARN("Odometry Buffer is empty! But a point cloud has arrived and waiting to be processed. Skipping this cloud.");
    return;
  }

  if (slam_->isUsingOdometryTopic()) {
    if (!slam_->doesOdometrybufferHasMeasurement(timestamp)) {
      ROS_WARN(
          "Pointcloud is here, pose buffer is not empty but odometry with the right stamp not available yet. Skipping the measurement.");

      return;
    }
  }

  // Re-publish the raw point cloud for visualization purposes.
  o3d_slam::publishCloud(cloud, slam_->frames_.rangeSensorFrame, toRos(timestamp), rawCloudPub_);

  // TODO(TT) Is this the best place to do this? (ofc its not)
  // Get the latest registered point cloud and publish it.
  std::tuple<PointCloud, Time, Transform> cloudTimePair = slam_->getLatestRegisteredCloudTimestampPair();
  std::tuple<Time, Transform> bestGuessTimePair = slam_->getLatestRegistrationBestGuess();

  if (std::get<0>(cloudTimePair).IsEmpty()) {
    ROS_WARN("Registered Cloud will not be published. Registration didn't take place yet.");
    return;
  }

  if (isTimeValid(std::get<1>(cloudTimePair))) {
    o3d_slam::publishCloud(std::get<0>(cloudTimePair), slam_->frames_.rangeSensorFrame, toRos(std::get<1>(cloudTimePair)),
                           registeredCloudPub_);

    if (surfaceNormalPub_.getNumSubscribers() > 0u || surfaceNormalPub_.isLatched()) {
      auto surfaceNormalLineMarker{
          generateMarkersForSurfaceNormalVectors(std::get<0>(cloudTimePair), toRos(std::get<1>(cloudTimePair)), colorMap_[ColorKey::kRed])};

      if (surfaceNormalLineMarker != std::nullopt) {
        surfaceNormalPub_.publish(surfaceNormalLineMarker.value());
      }
    }

  } else {
    ROS_WARN("Registered Cloud will be published with original stamp. Should only happen at start-up.");
    o3d_slam::publishCloud(std::get<0>(cloudTimePair), slam_->frames_.rangeSensorFrame, toRos(timestamp), registeredCloudPub_);

    if (surfaceNormalPub_.getNumSubscribers() > 0u || surfaceNormalPub_.isLatched()) {
      auto surfaceNormalLineMarker{
          generateMarkersForSurfaceNormalVectors(std::get<0>(cloudTimePair), toRos(timestamp), colorMap_[ColorKey::kRed])};

      if (surfaceNormalLineMarker != std::nullopt) {
        // ROS_DEBUG("Publishing point cloud surface normals for publisher '%s'.", parameters_.pointCloudPublisherTopic_.c_str());
        surfaceNormalPub_.publish(surfaceNormalLineMarker.value());
      }
    }

    return;
  }

  if ((!isTimeValid(std::get<0>(bestGuessTimePair)))) {
    ROS_WARN("bestGuessTimePair Transform Time is not valid at processMeasurement level.");
    return;
  }

  if ((!isTimeValid(std::get<1>(cloudTimePair)))) {
    ROS_WARN("Transform Time is not valid at processMeasurement level.");
    return;
  }

  // TODO [TT]
  // Functionize this stuff.
  Transform calculatedTransform = std::get<2>(cloudTimePair);

  geometry_msgs::PoseStamped poseStamped;
  Eigen::Quaterniond rotation(calculatedTransform.rotation());

  poseStamped.header.stamp = toRos(std::get<1>(cloudTimePair));
  poseStamped.pose.position.x = calculatedTransform.translation().x();
  poseStamped.pose.position.y = calculatedTransform.translation().y();
  poseStamped.pose.position.z = calculatedTransform.translation().z();
  poseStamped.pose.orientation.w = rotation.w();
  poseStamped.pose.orientation.x = rotation.x();
  poseStamped.pose.orientation.y = rotation.y();
  poseStamped.pose.orientation.z = rotation.z();

  slam_->appendPoseToTrackedPath(poseStamped);

  // Best guess path
  Transform bestGuessTransform = std::get<1>(bestGuessTimePair);

  geometry_msgs::PoseStamped bestGuessPoseStamped;
  Eigen::Quaterniond bestGuessRotation(bestGuessTransform.rotation());

  // Until we identify the time issue with best guess use cloud time. These are supposed to be same since they are paired.
  bestGuessPoseStamped.header.stamp = toRos(std::get<0>(bestGuessTimePair));
  bestGuessPoseStamped.pose.position.x = bestGuessTransform.translation().x();
  bestGuessPoseStamped.pose.position.y = bestGuessTransform.translation().y();
  bestGuessPoseStamped.pose.position.z = bestGuessTransform.translation().z();
  bestGuessPoseStamped.pose.orientation.w = bestGuessRotation.w();
  bestGuessPoseStamped.pose.orientation.x = bestGuessRotation.x();
  bestGuessPoseStamped.pose.orientation.y = bestGuessRotation.y();
  bestGuessPoseStamped.pose.orientation.z = bestGuessRotation.z();

  slam_->appendPoseToBestGuessPath(bestGuessPoseStamped);
  return;
}

std::optional<visualization_msgs::Marker> OnlineRangeDataProcessorRos::generateMarkersForSurfaceNormalVectors(
    const open3d::geometry::PointCloud& pointCloud, const ros::Time& timestamp, const o3d_slam::RgbaColorMap::Values& color) {
  if (pointCloud.IsEmpty()) {
    ROS_WARN("Point cloud is empty.");
    return {};
  }
  if (!pointCloud.HasNormals()) {
    ROS_WARN("Point cloud has no normals");
    return {};
  }

  std_msgs::ColorRGBA colorMsg;
  colorMsg.r = color[0];
  colorMsg.g = color[1];
  colorMsg.b = color[2];
  colorMsg.a = color[3];

  visualization_msgs::Marker vectorsMarker;
  vectorsMarker.header.stamp = timestamp;
  vectorsMarker.header.frame_id = slam_->frames_.rangeSensorFrame;
  vectorsMarker.ns = "surface_normals";
  vectorsMarker.action = visualization_msgs::Marker::ADD;
  vectorsMarker.type = visualization_msgs::Marker::LINE_LIST;
  vectorsMarker.pose.orientation.w = 1.0;
  vectorsMarker.id = 0;
  vectorsMarker.scale.x = 0.02;
  vectorsMarker.color = colorMsg;
  vectorsMarker.points.resize(pointCloud.points_.size() * 2);

  const auto& surfaceNormalsView = pointCloud.normals_;
  for (size_t i = 0; i < pointCloud.points_.size(); i += 2) {
    // The actual position of the point that the surface normal belongs to.
    vectorsMarker.points[i].x = pointCloud.points_[i][0];
    vectorsMarker.points[i].y = pointCloud.points_[i][1];
    vectorsMarker.points[i].z = pointCloud.points_[i][2];

    // End if arrow.
    vectorsMarker.points[i + 1].x = pointCloud.points_[i][0] + surfaceNormalsView[i][0] * 0.09;
    vectorsMarker.points[i + 1].y = pointCloud.points_[i][1] + surfaceNormalsView[i][1] * 0.09;
    vectorsMarker.points[i + 1].z = pointCloud.points_[i][2] + surfaceNormalsView[i][2] * 0.09;
  }

  return vectorsMarker;
}

void OnlineRangeDataProcessorRos::processOdometry(const Transform& transform, const Time& timestamp) {
  // If we depend on external odometry.
  if (slam_->isUsingOdometryTopic()) {
    // Add pose to buffer
    if (!slam_->addOdometryPoseToBuffer(transform, timestamp)) {
      ROS_ERROR_STREAM("Failed to add odometry pose to buffer. Exiting.");
      return;
    }

    // When there is no IMU msg available we need to bypass this condition.
    if (!slam_->isIMUattitudeInitializationEnabled()) {
      isAttitudeInitialized_ = true;
    }

    if (!isAttitudeInitialized_) {
      ROS_WARN_STREAM_THROTTLE(1, "Attitude not initialized yet, waiting IMU measurements. Throttled 1s");
      return;
    }

    ROS_DEBUG_STREAM("Odometry is processed at time: " << toString(timestamp));
  }
}

void OnlineRangeDataProcessorRos::addToPathMsg(nav_msgs::PathPtr pathPtr, const std::string& fixedFrameName, const ros::Time& stamp,
                                               const Eigen::Vector3d& t, const int maxBufferLength) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = fixedFrameName;
  pose.header.stamp = stamp;
  pose.pose.position.x = t(0);
  pose.pose.position.y = t(1);
  pose.pose.position.z = t(2);
  pathPtr->header.frame_id = fixedFrameName;
  pathPtr->header.stamp = stamp;
  pathPtr->poses.push_back(pose);
  if (pathPtr->poses.size() > maxBufferLength) {
    pathPtr->poses.erase(pathPtr->poses.begin());
  }
}

void OnlineRangeDataProcessorRos::gnssCallback_(const sensor_msgs::NavSatFix::ConstPtr& gnssMsgPtr) {
  // Static method variables
  static Eigen::Vector3d accumulatedCoordinates__(0.0, 0.0, 0.0);
  static Eigen::Vector3d W_t_W_Gnss_km1__;
  static int gnssCallbackCounter__ = 0;
  Eigen::Vector3d W_t_W_Gnss;

  // Start
  ++gnssCallbackCounter__;
  Eigen::Vector3d gnssCoord = Eigen::Vector3d(gnssMsgPtr->latitude, gnssMsgPtr->longitude, gnssMsgPtr->altitude);
  Eigen::Vector3d estCovarianceXYZ(gnssMsgPtr->position_covariance[0], gnssMsgPtr->position_covariance[4],
                                   gnssMsgPtr->position_covariance[8]);

  if (not gpsInitialized_) {
    if (gnssCallbackCounter__ < 20 + 1) {
      // Wait until measurements got accumulated
      accumulatedCoordinates__ += gnssCoord;
      if (!(gnssCallbackCounter__ % 10)) {
        std::cout << YELLOW_START << "AnymalEstimator" << COLOR_END << " NOT ENOUGH Gnss MESSAGES ARRIVED!" << std::endl;
      }
      return;
    } else if (gnssCallbackCounter__ == 20 + 1) {
      gnssHandlerPtr_->initHandler(accumulatedCoordinates__ / 20);
      ++gnssCallbackCounter__;
    }
    // Convert to cartesian coordinates
    gnssHandlerPtr_->convertNavSatToPosition(gnssCoord, W_t_W_Gnss);

    // TODO: Clean up
    double initYawEnuLidar = 0.0;
    gnssHandlerPtr_->setInitYaw(initYawEnuLidar);
    gpsInitialized_ = true;

    gnssHandlerPtr_->initHandler(gnssHandlerPtr_->getInitYaw());
  }

  // Convert to cartesian coordinates
  gnssHandlerPtr_->convertNavSatToPosition(gnssCoord, W_t_W_Gnss);

  /// Add GNSS to Path
  addToPathMsg(measGnss_worldGnssPathPtr_, "enu", gnssMsgPtr->header.stamp, W_t_W_Gnss, 800 * 4);
  /// Publish path
  pubMeasWorldGnssPath_.publish(measGnss_worldGnssPathPtr_);
}

void OnlineRangeDataProcessorRos::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  ROS_DEBUG_STREAM("A point cloud has arrived.");
  slam_->frames_.rangeSensorFrame = msg->header.frame_id;
  /*if (msg->header.frame_id != slam_->frames_.rangeSensorFrame){
    ROS_ERROR_STREAM("You failed to provide the right frame id in the parameters and the cloud. Exiting.");
    ROS_ERROR_STREAM("Frame from the msg: " << msg->header.frame_id << " Frame from the parameters: " << slam_->frames_.rangeSensorFrame);
    return;
  }*/

  open3d::geometry::PointCloud cloud;

  if (!open3d_conversions::rosToOpen3d(msg, cloud, false, true)) {
    ROS_ERROR_STREAM("Conversion Failed.");
  }

  const Time timestamp = fromRos(msg->header.stamp);
  accumulateAndProcessRangeData(cloud, timestamp);
}

void OnlineRangeDataProcessorRos::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_ptr) {
  // Add to buffer
  Eigen::Vector3d linearAcc(imu_ptr->linear_acceleration.x, imu_ptr->linear_acceleration.y, imu_ptr->linear_acceleration.z);
  Eigen::Vector3d angularVel(imu_ptr->angular_velocity.x, imu_ptr->angular_velocity.y, imu_ptr->angular_velocity.z);

  Eigen::Matrix<double, 6, 1> addedImuMeasurements;
  addedImuMeasurements = imuBufferPtr_->addToImuBuffer(imu_ptr->header.stamp.toSec(), linearAcc, angularVel);
  publishAddedImuMeas_(addedImuMeasurements, imu_ptr->header.stamp);
  slam_->frames_.imuFrame = imu_ptr->header.frame_id;

  if (isAttitudeInitialized_) {
    return;
  }

  Eigen::Quaterniond initAttitude = Eigen::Quaterniond::Identity();
  Eigen::Vector3d gyrBias = Eigen::Vector3d::Zero();
  double estimatedGravityMagnitude{0.0};
  if (!(imuBufferPtr_->estimateAttitudeFromImu(initAttitude, estimatedGravityMagnitude, gyrBias))) {
    return;
  }

  isAttitudeInitialized_ = true;
  Eigen::Vector3d gravityVector = Eigen::Vector3d(0, 0, 9.80665);
  Eigen::Vector3d estimatedGravityVector = Eigen::Vector3d(0, 0, estimatedGravityMagnitude);
  Eigen::Vector3d gravityVectorError = estimatedGravityVector - gravityVector;
  Eigen::Vector3d gravityVectorErrorInImuFrame = initAttitude.inverse().matrix() * gravityVectorError;
  std::cout << " Gravity error in IMU frame is: " << gravityVectorErrorInImuFrame.transpose() << std::endl;

  if (!slam_->isExternalOdometryFrameToCloudFrameCalibrationSet()) {
    std::cout << " Calibration is not available yet. Returning from IMU attitude initialization. "
              << " \n";
    return;
  }

  // We conciously read from the object to ensure the transformations are nicely set.
  Eigen::Affine3d eigenTransform = slam_->getExternalOdometryFrameToCloudFrameCalibration();
  geometry_msgs::TransformStamped calibrationAsTransform = tf2::eigenToTransform(eigenTransform);

  // Bookkeeping
  geometry_msgs::PoseStamped odomPose_transformed;
  geometry_msgs::PoseStamped odomPose;

  const auto latestOdomMeasurement = slam_->getLatestOdometryPoseMeasurement();
  odomPose.pose = o3d_slam::getPose(latestOdomMeasurement.transform_.matrix());

  // Actual transformation applied to the odometry measurement. Reads as pose of Lidar frame in the external odometry frame.
  tf2::doTransform(odomPose, odomPose_transformed, calibrationAsTransform);

  // Here we dont want to use the orientation of the odometry measurement. We will acquire it from IMU anyway.
  odomPose_transformed.pose.orientation.w = 1.0;
  odomPose_transformed.pose.orientation.z = 0.0;
  odomPose_transformed.pose.orientation.y = 0.0;
  odomPose_transformed.pose.orientation.x = 0.0;

  // Convert the attitude of the IMU to the attitude of the LiDAR.
  Transform initAttitudeOfLiDAR = initAttitude * lidarToImu_.inverse();

  std::cout << " The initial pose of LiDAR is: "
            << "\033[92m" << o3d_slam::asString(initAttitudeOfLiDAR) << " \n";

  // This casts isometry3d to affine3d.
  Transform newTransform = o3d_slam::getTransform(odomPose_transformed.pose) * initAttitudeOfLiDAR;

  // initialTransform.affine().matrix().block<3, 3>(0, 0) = initAttitudeOfLiDAR.affine().matrix().block<3, 3>(0, 0);
  slam_->setInitialTransform(newTransform.matrix());
}

void OnlineRangeDataProcessorRos::publishAddedImuMeas_(const Eigen::Matrix<double, 6, 1>& addedImuMeas, const ros::Time& stamp) {
  // Publish added imu measurement
  if (addedImuMeasPub_.getNumSubscribers() == 0 && !addedImuMeasPub_.isLatched()) {
    // Early Return since IMU is at 400hz. We dont want to publish this if no one is listening.
    return;
  }

  sensor_msgs::Imu addedImuMeasMsg;
  addedImuMeasMsg.header.stamp = stamp;
  addedImuMeasMsg.header.frame_id = slam_->frames_.imuFrame;
  addedImuMeasMsg.linear_acceleration.x = addedImuMeas(0);
  addedImuMeasMsg.linear_acceleration.y = addedImuMeas(1);
  addedImuMeasMsg.linear_acceleration.z = addedImuMeas(2);
  addedImuMeasMsg.angular_velocity.x = addedImuMeas(3);
  addedImuMeasMsg.angular_velocity.y = addedImuMeas(4);
  addedImuMeasMsg.angular_velocity.z = addedImuMeas(5);
  addedImuMeasPub_.publish(addedImuMeasMsg);
}

void OnlineRangeDataProcessorRos::poseStampedCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  if ((odometryCallBackEnabled_ || poseStampedWithCovarianceCallBackEnabled_)) {
    return;
  }

  poseStampedCallBackEnabled_ = true;

  geometry_msgs::Pose odomPose = msg->pose;
  processOdometryData(o3d_slam::getTransform(odomPose), fromRos(msg->header.stamp));
}

void OnlineRangeDataProcessorRos::poseStampedWithCovarianceCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
  // This is expected to be the default. So we dont return here.

  // if ((poseStampedCallBackEnabled_ || odometryCallBackEnabled_))
  //{
  // std::cout << "Already an odometry measurement for this timestamp. Skipping poseStampedWithCovarianceCallback" << std::endl;
  //  return;
  //}

  poseStampedWithCovarianceCallBackEnabled_ = true;

  geometry_msgs::Pose odomPose = msg->pose.pose;
  processOdometryData(o3d_slam::getTransform(odomPose), fromRos(msg->header.stamp));
  ROS_DEBUG_STREAM("Pose with covariance callback is called.");
}

void OnlineRangeDataProcessorRos::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  if ((poseStampedCallBackEnabled_ || poseStampedWithCovarianceCallBackEnabled_)) {
    // std::cout << "Already an odometry measurement for this timestamp. Skipping odometryCallback" << std::endl;
    return;
  }
  odometryCallBackEnabled_ = true;

  geometry_msgs::Pose odomPose;
  odomPose.orientation = msg->pose.pose.orientation;
  odomPose.position = msg->pose.pose.position;

  processOdometryData(o3d_slam::getTransform(odomPose), fromRos(msg->header.stamp));
}

}  // namespace o3d_slam
