#include "open3d_slam_ros/RosbagRangeDataProcessorRos.hpp"
#include <open3d/io/PointCloudIO.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <filesystem>
#include <random>
#include "open3d_conversions/open3d_conversions.h"
#include "open3d_slam_ros/SlamWrapperRos.hpp"

#include <rosbag/view.h>
#include "open3d_slam/time.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"

namespace o3d_slam {

RosbagRangeDataProcessorRos::RosbagRangeDataProcessorRos(ros::NodeHandlePtr nh) : BASE(nh) {}

void RosbagRangeDataProcessorRos::initialize() {
  initCommonRosStuff();
  slam_ = std::make_shared<SlamWrapperRos>(nh_);
  slam_->loadParametersAndInitialize();
  rosbagFilename_ = nh_->param<std::string>("rosbag_filepath", "");
  ROS_INFO_STREAM("Reading from rosbag: " << rosbagFilename_);

  package_path_ = ros::package::getPath("open3d_slam_ros");
  if (package_path_.empty()) {
    ROS_ERROR("Could not find package open3d_slam_ros");
    return;
  }
  ROS_INFO_STREAM("Package path: " << package_path_);

  if (slam_->isRMSenabled_) {
    mrs_lib::ParamLoader param_loader(*nh_, "RMS");
    rms = std::make_shared<rms::RMS>(param_loader);
    rms->setLambda(slam_->rmsLambda_);
    rms->setRMSVoxelSize(slam_->rmsVoxelSize_);
  }

  // GNSS Handler
  mapEnuTransform_ = Eigen::Isometry3d::Identity();
  gnssHandlerPtr_ = std::make_shared<o3d_slam::GnssHandler>();
  measGnss_worldGnssPathPtr_ = nav_msgs::PathPtr(new nav_msgs::Path);

  // Initialize the calibration Transform.
  baseToLidarTransform_.transform.rotation.w = 1.0;
  baseToLidarTransform_.transform.rotation.z = 0.0;
  baseToLidarTransform_.transform.rotation.y = 0.0;
  baseToLidarTransform_.transform.rotation.x = 0.0;

  baseToLidarTransform_.transform.translation.z = 0.0;
  baseToLidarTransform_.transform.translation.y = 0.0;
  baseToLidarTransform_.transform.translation.x = 0.0;

  // For ANYmal Forest GPS
  gpsToLidarTransform_.transform.translation.x = 0.006;  //;0.081;
  gpsToLidarTransform_.transform.translation.y = 0.455;
  gpsToLidarTransform_.transform.translation.z = 0.484;  // 0.64
  gpsToLidarTransform_.transform.rotation.x = 0.0;
  gpsToLidarTransform_.transform.rotation.y = 0.0;
  gpsToLidarTransform_.transform.rotation.z = 0.707;
  gpsToLidarTransform_.transform.rotation.w = 0.707;

  tfBuffer_ = std::make_unique<tf2_ros::Buffer>();
  tfListener_ = std::make_unique<tf2_ros::TransformListener>(*tfBuffer_);
  timeDiff_ = ros::Duration(0.0);

  // Provide questionare.
  ROS_INFO_STREAM("\033[33m"
                  << " Did you check the odometry topic frame? "
                  << "\033[39m");
  ROS_INFO_STREAM("\033[33m"
                  << " Did you check the check if loopclosure enabled? "
                  << "\033[39m");
  ROS_INFO_STREAM("\033[33m"
                  << " Is clock available in your bag? "
                  << "\033[39m");
  ROS_INFO_STREAM("\033[33m"
                  << " Did you pray this works? "
                  << "\033[39m");
  ROS_INFO_STREAM("\033[33m"
                  << " Did you check if tf_static exists in your bag? "
                  << "\033[39m");

  const ros::WallTime first{ros::WallTime::now() + ros::WallDuration(2.0)};
  ros::WallTime::sleepUntil(first);
}

void RosbagRangeDataProcessorRos::addToPathMsg(nav_msgs::PathPtr pathPtr, const std::string& fixedFrameName, const ros::Time& stamp,
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

bool RosbagRangeDataProcessorRos::createOutputDirectory() {
  // TODO(TT) This folder is currently not used.

  // Check if the output folder exists.
  if (std::filesystem::is_directory("log/slam_loggers")) {
    return true;
  }

  // If the folder doesn't exist, create it.
  try {
    return std::filesystem::create_directories("log/slam_loggers");
  } catch (const std::exception& exception) {
    ROS_ERROR_STREAM("Caught an exception trying to create output folder: " << exception.what());
  }

  return false;
}

std::string RosbagRangeDataProcessorRos::buildUpLogFilename(const std::string& typeSuffix, const std::string& extension) {
  ros::WallTime stamp = ros::WallTime::now();
  std::stringstream ss;
  ss << stamp.sec << "_" << stamp.nsec;

  // Add prefixes.
  // Not sure if adding time is the best thing to do since we dont keep a ring buffer.
  // std::string filename = slam_->mapSavingFolderPath_ + typeSuffix + ss.str() + extension;
  std::string filename = slam_->mapSavingFolderPath_ + typeSuffix + extension;

  return filename;
}

visualization_msgs::Marker RosbagRangeDataProcessorRos::createLineStripMarker() {
  visualization_msgs::Marker marker;
  marker.header.frame_id = slam_->frames_.mapFrame;
  marker.header.stamp = tracker;
  marker.ns = "path";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.color.r = 1.0;
  marker.color.a = 1.0;
  marker.points.reserve(trackedPath_.poses.size());

  for (const auto& pose : trackedPath_.poses) {
    geometry_msgs::Point point;
    point.x = pose.pose.position.x;
    point.y = pose.pose.position.y;
    point.z = pose.pose.position.z;
    marker.points.push_back(point);
  }

  return marker;
}

o3d_slam::PointCloud RosbagRangeDataProcessorRos::lineStripToPointCloud(const visualization_msgs::MarkerArray& marker_array,
                                                                        const int num_samples) {
  // Create point stream
  std::vector<Eigen::Vector3d> points;

  // Iterate over markers in marker array to find line strip
  for (auto marker : marker_array.markers) {
    for (size_t j = 0; j < marker.points.size(); j++) {
      // Sample points between start and end of line strip
      float dx = (marker.points[j + 1].x - marker.points[j].x) / num_samples;
      float dy = (marker.points[j + 1].y - marker.points[j].y) / num_samples;
      float dz = (marker.points[j + 1].z - marker.points[j].z) / num_samples;

      for (int i = 0; i < num_samples; i++) {
        Eigen::Vector3d point = Eigen::Vector3d::Zero();
        point.x() = marker.points[j].x + i * dx;
        point.y() = marker.points[j].y + i * dy;
        point.z() = marker.points[j].z + i * dz;

        points.push_back(point);
      }
    }
  }
  // Create point cloud
  o3d_slam::PointCloud cloud(points);
  return cloud;
}

visualization_msgs::MarkerArray RosbagRangeDataProcessorRos::convertPathToMarkerArray(const nav_msgs::Path& path) {
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(1);
  marker_array.markers[0] = createLineStripMarker();
  return marker_array;
}

void RosbagRangeDataProcessorRos::calculateSurfaceNormals(o3d_slam::PointCloud& cloud) {
  if (cloud.points_.size() == 0u) {
    ROS_ERROR_STREAM("Cloud is empty. Skipping the normal calculation.");
    return;
  }

  // Create a KD-Tree and estimate surface normals.
  open3d::geometry::KDTreeSearchParamHybrid param(5, 20);
  cloud.EstimateNormals(param);
  cloud.NormalizeNormals();
  cloud.OrientNormalsTowardsCameraLocation();
}

void RosbagRangeDataProcessorRos::startProcessing() {
  if (!createOutputDirectory()) {
    std::cout << "Couldn't create the directory "
              << "\n";
    return;
  }

  std::string trackedPosesFilename_ = buildUpLogFilename("slam_poses");
  std::remove(trackedPosesFilename_.c_str());

  const std::string poseLogFileHeader_ = "# timestamp x y z q_x q_y q_z q_w";

  // Open file and set numerical precision to the max.
  poseFile_.open(trackedPosesFilename_, std::ios_base::app);
  poseFile_.precision(std::numeric_limits<double>::max_digits10);
  poseFile_ << poseLogFileHeader_ << std::endl;

  std::string trackedgnssFilename_ = buildUpLogFilename("gnss_poses");
  std::remove(trackedgnssFilename_.c_str());

  if (slam_->useGPSforGroundTruth_) {
    gnssFile_.open(trackedgnssFilename_, std::ios_base::app);
    gnssFile_.precision(std::numeric_limits<double>::max_digits10);
    gnssFile_ << poseLogFileHeader_ << std::endl;
  }

  if (slam_->saveProcessedBag_) {
    std::string outBagPath_ = buildUpLogFilename("processed_slam", ".bag");
    std::remove(outBagPath_.c_str());
    outBag.open(outBagPath_, rosbag::bagmode::Write);
    outBag.setCompression(rosbag::compression::LZ4);
  }

  const std::string filename_localizability = package_path_ + "/data/maps/replayed_localizability_categories.txt";

  // Remove the existing file.
  std::remove(filename_localizability.c_str());
  const std::string poseLogFileHeader_localizability = "timestamp transX transY transZ rotX rotY rotZ";

  // Open file and set numerical precision to the max.
  file_localizability.open(filename_localizability, std::ios_base::app);
  file_localizability.precision(std::numeric_limits<double>::max_digits10);
  file_localizability << poseLogFileHeader_localizability << std::endl;

  const std::string filename_stats = package_path_ + "/data/maps/replayed_statistics.txt";

  // Remove the existing file.
  std::remove(filename_stats.c_str());
  const std::string poseLogFileHeader_stats = "timestamp totalIteration totalTime residual transformationUpdateNorm nbMatches";

  // Open file and set numerical precision to the max.
  file_stats.open(filename_stats, std::ios_base::app);
  file_stats.precision(std::numeric_limits<double>::max_digits10);
  file_stats << poseLogFileHeader_stats << std::endl;

  // Iterate and process the bag.
  if (processRosbag()) {
    // Create the magical tube.
    visualization_msgs::MarkerArray lineStrip = convertPathToMarkerArray(trackedPath_);

    o3d_slam::PointCloud samplecloud;
    // Magic number, points per circle strip. If you set it too high, cloud becomes mega heavy.
    int numPoints = 50;
    // Define the radius of the tube
    float tube_radius = 0.02;

    samplecloud = lineStripToPointCloud(lineStrip, numPoints);

    calculateSurfaceNormals(samplecloud);

    // Create a new point cloud for the tube
    o3d_slam::PointCloud tube_cloud;
    std::vector<Eigen::Vector3d> tube_cloud_normal_vector;
    std::vector<Eigen::Vector3d> tube_cloud_point_vector;

    tube_cloud_normal_vector.reserve(samplecloud.points_.size() * 36);
    tube_cloud_point_vector.reserve(samplecloud.points_.size() * 36);

    // Iterate over the points in the original point cloud
    for (int i = 0; i < samplecloud.points_.size() - numPoints; i++) {
      // Retrieve the surface normal
      Eigen::Vector3d normal = samplecloud.normals_[i].normalized();

      // Calculate the direction vector
      const Eigen::Vector3d direction = normal.unitOrthogonal().cross(normal).normalized();

      // Generate points around the current point
      const Eigen::Vector3d current_point = samplecloud.points_[i];
      const Eigen::Vector3d current_normal = samplecloud.normals_[i];
      for (float angle = 0; angle <= 360; angle += 10) {
        float x = current_point.x() + tube_radius * (direction.x() * cos(angle) + normal.x() * sin(angle));
        float y = current_point.y() + tube_radius * (direction.y() * cos(angle) + normal.y() * sin(angle));
        float z = current_point.z() + tube_radius * (direction.z() * cos(angle) + normal.z() * sin(angle));

        Eigen::Vector3d tube_point(x, y, z);

        tube_cloud_normal_vector.push_back(current_normal);
        tube_cloud_point_vector.push_back(tube_point);
      }
    }
    tube_cloud.normals_ = tube_cloud_normal_vector;
    tube_cloud.points_ = tube_cloud_point_vector;

    // Copy the surface normals to the new point cloud
    for (int i = 0; i < samplecloud.points_.size() - numPoints; i++) {
      for (int j = 0; j < 36; j++) {
        tube_cloud.normals_[i * 36 + j] = samplecloud.normals_[i];
      }
    }

    std::string nameWithCorrectSuffix = slam_->mapSavingFolderPath_ + "robotPathAsMesh.pcd";
    open3d::io::WritePointCloudToPCD(nameWithCorrectSuffix, tube_cloud, open3d::io::WritePointCloudOption());
    ROS_INFO_STREAM("Successfully saved the poses as point cloud. Waiting for user to terminate.");
  }

  // Close file handle.
  poseFile_.close();
  if (slam_->saveProcessedBag_) {
    outBag.close();
  }
  if (slam_->useGPSforGroundTruth_) {
    gnssFile_.close();
  }

  // Close file handle.
  file_localizability.close();
  file_stats.close();

  ros::spin();
  slam_->stopWorkers();
  return;
}

bool RosbagRangeDataProcessorRos::validateAnalysisTopic(const rosbag::Bag& bag, const std::vector<std::string>& mandatoryTopics) {
  // Get a view on the data and check if all mandatory topics are present.
  bool mandatoryTopicsInRosbag{true};

  // Iterate over the mandatory topics and check if they are in the bag.
  for (const auto& topic : mandatoryTopics) {
    rosbag::View topicView(bag, rosbag::TopicQuery(topic));
    if (topicView.size() == 0u) {
      if (topic == clockTopic_) {
        // This means this is our second time coming here. So actually the the alternative topic is not working either.
        if (topic == slam_->asyncOdometryTopic_) {
          ROS_WARN_STREAM(clockTopic_ << " topic does not exist in the rosbag. This is breaking.");
          mandatoryTopicsInRosbag = false;
        } else {
          ROS_WARN_STREAM(clockTopic_ << " topic does not exist in the rosbag. Using alternative topic: " << slam_->asyncOdometryTopic_
                                      << " as clock.");
          clockTopic_ = slam_->asyncOdometryTopic_;
        }

      } else if (topic == tfTopic_) {
        ROS_WARN_STREAM("No data under the topic: " << topic << " was found. This was optional so okay.");
        continue;
      } else if (topic == tfStaticTopic_) {
        ROS_ERROR_STREAM("No data under the topic: "
                         << topic << " was found. This is NOT optional. But if you make tf_static available external its okay.");
        continue;
      } else {
        ROS_ERROR_STREAM("No data under the topic: " << topic << " was found.");
        mandatoryTopicsInRosbag = false;
      }
    } else {
      if (topic == slam_->asyncOdometryTopic_) {
        // Get a view for the specific topic only
        rosbag::View view(bag, rosbag::TopicQuery(topic));
        for (const auto& messageInstance : view) {
          if (messageInstance.getDataType() == "geometry_msgs/PoseWithCovarianceStamped") {
            ROS_WARN_STREAM(" ' " << topic
                                  << "' topic does not support automatic frame detection. Msg Type: " << messageInstance.getDataType()
                                  << ". Assumed tracked odometry frame: " << slam_->frames_.assumed_external_odometry_tracked_frame);
            break;
          } else if (messageInstance.getDataType() == "nav_msgs/Odometry") {
            nav_msgs::Odometry::ConstPtr message = messageInstance.instantiate<nav_msgs::Odometry>();
            if (message != nullptr) {
              slam_->frames_.assumed_external_odometry_tracked_frame = message->child_frame_id;
              ROS_WARN_STREAM(topic << " frame_id is set to: " << slam_->frames_.assumed_external_odometry_tracked_frame);
              break;
            }  // if
          } else if (messageInstance.getDataType() == "geometry_msgs/PoseStamped") {
            ROS_WARN_STREAM(" ' " << topic
                                  << "' topic does not support automatic frame detection. Msg Type: " << messageInstance.getDataType()
                                  << ". Assumed tracked odometry frame: " << slam_->frames_.assumed_external_odometry_tracked_frame);
            break;
          } else {
            ROS_ERROR_STREAM(topic << " msg type is NOT SUPPORTED");
            return false;
          }

        }  // for
      }    // if
    }      // if
  }        // for

  if (slam_->useSyncedPoses_) {
    ROS_WARN_STREAM("Sync poses are enabled. This does not support automatic frame detection. Assumed tracked odometry frame: "
                    << slam_->frames_.assumed_external_odometry_tracked_frame);
  }

  if (!mandatoryTopicsInRosbag) {
    ROS_ERROR("All required topics are not within the rosbag. Replay operations are terminated. Waiting user to terminate.");
    return false;
  }

  return true;
}

void RosbagRangeDataProcessorRos::processMeasurement(const PointCloud& cloud, const Time& timestamp) {
  // placeholder
}

bool RosbagRangeDataProcessorRos::processBuffers(SlamInputsBuffer& buffer) {
  // This is as fast as the thread can go
  if (buffer.empty()) {
    ROS_DEBUG("Empty buffer");
    return false;
  }

  if (slam_->useSyncedPoses_) {
    // Provide the pose prior
    auto& odometryPose = buffer.front()->odometryPose_;
    geometry_msgs::Pose odomPose = odometryPose->pose;

    if (isFirstMessage_) {
      Eigen::Isometry3d eigenTransform = Eigen::Isometry3d::Identity();
      slam_->setExternalOdometryFrameToCloudFrameCalibration(eigenTransform);
    }

    if (isFirstMessage_ && isStaticTransformFound_) {
      geometry_msgs::Pose initialPose;
      initialPose.position = odomPose.position;
      initialPose.orientation.w = 1.0;
      initialPose.orientation.z = 0.0;
      initialPose.orientation.y = 0.0;
      initialPose.orientation.x = 0.0;
      slam_->setInitialTransform(o3d_slam::getTransform(initialPose).matrix());
      isFirstMessage_ = false;
    }

    // Add to the odometry buffer.
    if (!(slam_->addOdometryPoseToBuffer(o3d_slam::getTransform(odomPose), fromRos(odometryPose->header.stamp)))) {
      std::cout << "Couldn't Add pose to buffer" << std::endl;
      return false;
    }

  } else {
    // Buffer is empty, means the async odometry poses did not arrive before the point cloud.
    if (slam_->isOdometryPoseBufferEmpty()) {
      std::cout << "Odometry Buffer is empty!" << std::endl;
      return false;
    }

    if (!slam_->isInitialTransformSet()) {
      std::cout << "Initial transform not set yet! Popping the measurement." << std::endl;
      return false;
    }
  }

  auto& pointCloud = buffer.front()->pointCloud_;

  // Convert to o3d cloud
  open3d::geometry::PointCloud cloud;

  if (slam_->downsamplePointCloudForReplay_) {
    if (!open3d_conversions::rosToOpen3d(pointCloud, cloud, false, false)) {
      std::cout << "Couldn't convert the point cloud" << std::endl;
      return false;
    }

    cloud = *(cloud.UniformDownSample(slam_->downSamplingSkippingRate_));
  } else {
    if (!open3d_conversions::rosToOpen3d(pointCloud, cloud, false, false)) {
      std::cout << "Couldn't convert the point cloud" << std::endl;
      return false;
    }
  }

  const Time timestamp = fromRos(pointCloud->header.stamp);

  if (!slam_->doesOdometrybufferHasMeasurement(timestamp)) {
    std::cout << "RosbagReplayer:: Pointcloud is here, pose buffer is not empty but odometry with the right stamp not available yet. "
                 "Popping the measurement."
              << std::endl;
    buffer.pop_front();
    return false;
  }

  // Add the cloud to queue, internally checks if there is a matching odometry pose in the odometryBuffer
  if (slam_->addRangeScan(cloud, timestamp)) {
    auto timeTuple = usePairForRegistration();
  } else {
    std::cout << "RosbagReplayer:: Couldn't add range scan. Popping this measurement from the buffer." << std::endl;
    buffer.pop_front();
    return false;
  }

  deeperICPLogs_ = slam_->offlineGetICPLogs();

  logToFiles();

  // Publish the tfs
  slam_->offlineTfWorker();

  // Publish Submap markers
  slam_->offlineVisualizationWorker();

  std::tuple<Time, Transform> bestGuessTimePair = slam_->getLatestRegistrationBestGuess();
  Transform bestGuessTransform = std::get<1>(bestGuessTimePair);

  // If GNSS is used then the recorded poses are stored in GPS frame.
  if (slam_->useGPSforGroundTruth_) {
    Eigen::Isometry3d gpsLidareigenTransform = tf2::transformToEigen(gpsToLidarTransform_);
    bestGuessTransform = bestGuessTransform * gpsLidareigenTransform;
  }

  geometry_msgs::PoseStamped bestGuessPoseStamped;
  Eigen::Quaterniond bestGuessRotation(bestGuessTransform.rotation());

  bestGuessPoseStamped.header.stamp = toRos(std::get<0>(bestGuessTimePair));
  bestGuessPoseStamped.pose.position.x = bestGuessTransform.translation().x();
  bestGuessPoseStamped.pose.position.y = bestGuessTransform.translation().y();
  bestGuessPoseStamped.pose.position.z = bestGuessTransform.translation().z();
  bestGuessPoseStamped.pose.orientation.w = bestGuessRotation.w();
  bestGuessPoseStamped.pose.orientation.x = bestGuessRotation.x();
  bestGuessPoseStamped.pose.orientation.y = bestGuessRotation.y();
  bestGuessPoseStamped.pose.orientation.z = bestGuessRotation.z();
  bestGuessPath_.poses.push_back(bestGuessPoseStamped);
  bestGuessPath_.header.stamp = toRos(std::get<0>(bestGuessTimePair));  // This guess supposed to be associated with the pose we give to it.
  bestGuessPath_.header.frame_id = slam_->frames_.mapFrame;

  if (offlineBestGuessPathPub_.getNumSubscribers() > 0u || offlineBestGuessPathPub_.isLatched()) {
    offlineBestGuessPathPub_.publish(bestGuessPath_);
  }

  // Check the latest registered cloud buffer
  std::tuple<PointCloud, Time, Transform> cloudTimePair = slam_->getLatestRegisteredCloudTimestampPair();
  Transform calculatedTransform = std::get<2>(cloudTimePair);

  // If GNSS is used then the recorded poses are stored in GPS frame.
  if (slam_->useGPSforGroundTruth_) {
    Eigen::Isometry3d gpsLidareigenTransform = tf2::transformToEigen(gpsToLidarTransform_);
    calculatedTransform = calculatedTransform * gpsLidareigenTransform;
  }

  geometry_msgs::PoseStamped poseStamped;
  Eigen::Quaterniond rotation(calculatedTransform.rotation());

  poseStamped.header.stamp = toRos(std::get<1>(cloudTimePair));
  poseStamped.header.frame_id = slam_->frames_.mapFrame;
  poseStamped.pose.position.x = calculatedTransform.translation().x();
  poseStamped.pose.position.y = calculatedTransform.translation().y();
  poseStamped.pose.position.z = calculatedTransform.translation().z();
  poseStamped.pose.orientation.w = rotation.w();
  poseStamped.pose.orientation.x = rotation.x();
  poseStamped.pose.orientation.y = rotation.y();
  poseStamped.pose.orientation.z = rotation.z();
  trackedPath_.poses.push_back(poseStamped);

  if (slam_->saveProcessedBag_) {
    outBag.write("/slam_optimized_poses", toRos(std::get<1>(cloudTimePair)), poseStamped);
  }

  nav_msgs::Odometry odometryPose;

  odometryPose.header.stamp = toRos(std::get<1>(cloudTimePair));
  odometryPose.header.frame_id = slam_->frames_.mapFrame;
  odometryPose.child_frame_id = slam_->frames_.rangeSensorFrame;
  odometryPose.pose.pose.position.x = calculatedTransform.translation().x();
  odometryPose.pose.pose.position.y = calculatedTransform.translation().y();
  odometryPose.pose.pose.position.z = calculatedTransform.translation().z();
  odometryPose.pose.pose.orientation.w = rotation.w();
  odometryPose.pose.pose.orientation.x = rotation.x();
  odometryPose.pose.pose.orientation.y = rotation.y();
  odometryPose.pose.pose.orientation.z = rotation.z();

  if (slam_->saveProcessedBag_) {
    outBag.write("/slam_optimized_poses_odometry", toRos(std::get<1>(cloudTimePair)), odometryPose);
  }

  const double stamp = pointCloud->header.stamp.toSec();
  poseFile_ << stamp << " ";
  poseFile_ << calculatedTransform.translation().x() << " " << calculatedTransform.translation().y() << " "
            << calculatedTransform.translation().z() << " ";
  poseFile_ << rotation.x() << " " << rotation.y() << " " << rotation.z() << " " << rotation.w() << std::endl;

  trackedPath_.header.stamp = toRos(std::get<1>(cloudTimePair));
  trackedPath_.header.frame_id = slam_->frames_.mapFrame;

  if (offlinePathPub_.getNumSubscribers() > 0u || offlinePathPub_.isLatched()) {
    offlinePathPub_.publish(trackedPath_);
  }

  if (slam_->useGPSforGroundTruth_) {
    if (pubMeasWorldGnssPath_.getNumSubscribers() > 0u || pubMeasWorldGnssPath_.isLatched()) {
      if (measGnss_worldGnssPathPtr_->poses.size() != 0u) {
        /// Publish path
        pubMeasWorldGnssPath_.publish(*measGnss_worldGnssPathPtr_);
      }
    }
  }

  drawDegeneracyArrows(std::get<2>(cloudTimePair), toRos(std::get<1>(cloudTimePair)));
  drawLinesBetweenPoses(trackedPath_, bestGuessPath_, toRos(std::get<1>(cloudTimePair)));

  if (isTimeValid(std::get<1>(cloudTimePair)) && !(std::get<0>(cloudTimePair).IsEmpty())) {
    o3d_slam::publishCloud(std::get<0>(cloudTimePair), slam_->frames_.rangeSensorFrame, toRos(std::get<1>(cloudTimePair)),
                           registeredCloudPub_);
    ros::spinOnce();
  } else {
    ROS_ERROR_STREAM("Cloud is empty or time is invalid. Skipping the cloud.");
    return false;
  }

  if (surfaceNormalPub_.getNumSubscribers() > 0u || surfaceNormalPub_.isLatched()) {
    auto surfaceNormalLineMarker{
        generateMarkersForSurfaceNormalVectors(std::get<0>(cloudTimePair), toRos(std::get<1>(cloudTimePair)), colorMap_[ColorKey::kRed])};

    if (surfaceNormalLineMarker != std::nullopt) {
      surfaceNormalPub_.publish(surfaceNormalLineMarker.value());
    }
  }

  // Pop oldest input point cloud from the queue.
  buffer.pop_front();

  if (slam_->saveProcessedBag_) {
    sensor_msgs::PointCloud2 outCloud;
    open3d_conversions::open3dToRos(std::get<0>(cloudTimePair), outCloud, slam_->frames_.rangeSensorFrame);
    outCloud.header.stamp = toRos(std::get<1>(cloudTimePair));
    outBag.write("/registered_cloud", toRos(std::get<1>(cloudTimePair)), outCloud);

    PointCloud transformedCloud = std::get<0>(cloudTimePair);

    transformedCloud.Transform(std::get<2>(cloudTimePair).matrix());
    sensor_msgs::PointCloud2 transformedRosCloud;
    open3d_conversions::open3dToRos(transformedCloud, transformedRosCloud, slam_->frames_.rangeSensorFrame);
    transformedRosCloud.header.stamp = toRos(std::get<1>(cloudTimePair));
    outBag.write("/transformed_registered_cloud", toRos(std::get<1>(cloudTimePair)), transformedRosCloud);

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = poseStamped.header.stamp;
    transformStamped.header.frame_id = slam_->frames_.mapFrame;
    transformStamped.child_frame_id = slam_->frames_.rangeSensorFrame;
    transformStamped.transform.translation.x = poseStamped.pose.position.x;
    transformStamped.transform.translation.y = poseStamped.pose.position.y;
    transformStamped.transform.translation.z = poseStamped.pose.position.z;
    transformStamped.transform.rotation = poseStamped.pose.orientation;

    tf2_msgs::TFMessage tfMessage;
    tfMessage.transforms.push_back(transformStamped);

    outBag.write("/tf", toRos(std::get<1>(cloudTimePair)), tfMessage);
  }

  const ros::WallTime arbitrarySleep{ros::WallTime::now() + ros::WallDuration(slam_->relativeSleepDuration_)};
  ros::WallTime::sleepUntil(arbitrarySleep);

  return true;
}

visualization_msgs::Marker RosbagRangeDataProcessorRos::generateEigenVectorArrowMarkers(std::vector<float> mean, Eigen::VectorXf eigVector,
                                                                                        int id, int colorId, std::string ns, float scale,
                                                                                        const ros::Time& stamp) {
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.stamp = stamp;
  marker.header.frame_id = slam_->frames_.mapFrame;

  if (ns == "translation") {
    colorId = 0;  // yellow
  }
  if (ns == "rotation") {
    colorId = 3;  // cyan
  }

  const auto& itColor{localizabilityArrorColors_.find(colorId)};
  marker.color.r = itColor->second[0];
  marker.color.g = itColor->second[1];
  marker.color.b = itColor->second[2];
  marker.color.a = itColor->second[3];
  marker.ns = ns;
  marker.id = id;
  marker.lifetime = ros::Duration();
  marker.scale.x = 0.8;
  marker.scale.y = 2.4;
  marker.scale.z = 0.8;
  marker.points.resize(2u);

  if (eigVector.isZero()) {
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    scale = 0.1;
  } else {
    scale = 4.0;
  }

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.points[0].x = mean[0];
  marker.points[0].y = mean[1];
  marker.points[0].z = mean[2];

  marker.points[1].x = mean[0] + eigVector(0) * scale;
  marker.points[1].y = mean[1] + eigVector(1) * scale;
  marker.points[1].z = mean[2] + eigVector(2) * scale;

  return marker;
}

void RosbagRangeDataProcessorRos::drawDegeneracyArrows(const Transform currentPose, const ros::Time& stamp) {
  if (o3d_slam::toSecondsSinceFirstMeasurement(deeperICPLogs_.time_) <= 0.0) {
    return;
  }

  std::vector<float> vec;
  vec.push_back(currentPose.translation().x());
  vec.push_back(currentPose.translation().y());
  vec.push_back(currentPose.translation().z());

  Eigen::Matrix<float, 3, 6> PharosArrows_ = deeperICPLogs_.degenerateDirections_;

  if (condNumberArrowPublisher_.getNumSubscribers() > 0u || condNumberArrowPublisher_.isLatched()) {
    visualization_msgs::MarkerArray arrowArray;
    for (int i = 0; i < 6; i++) {
      std::string ns = "rotation";
      if (i > 2) {
        ns = "translation";
      }

      visualization_msgs::Marker arrow;
      visualization_msgs::Marker reverseArrow;
      arrow = generateEigenVectorArrowMarkers(vec, PharosArrows_.col(i), i, i, ns, 1.0, stamp);
      reverseArrow = generateEigenVectorArrowMarkers(vec, -PharosArrows_.col(i), i + 3, i, ns, 1.0, stamp);
      arrowArray.markers.emplace_back(std::move(arrow));
      arrowArray.markers.emplace_back(std::move(reverseArrow));
    }

    condNumberArrowPublisher_.publish(arrowArray);
  }
}

void RosbagRangeDataProcessorRos::logToFiles() {
  if (o3d_slam::toSecondsSinceFirstMeasurement(deeperICPLogs_.time_) <= 0.0) {
    return;
  }

  generateLocalizabilityCategory();
  generateSystemStatsFile();
}

void RosbagRangeDataProcessorRos::generateSystemStatsFile() {
  // Logging Values
  auto& size = deeperICPLogs_.numberOfIterations;
  auto& timestamp = deeperICPLogs_.time_;
  auto& totalICPtime_ = deeperICPLogs_.totalICPtime;
  auto& residualError_ = deeperICPLogs_.residualError_;
  auto& nbMatches_ = deeperICPLogs_.nbMatches_;
  auto& numberOfIterations = deeperICPLogs_.numberOfIterations;
  auto& motion = deeperICPLogs_.transform_;

  if (slam_->isRMSenabled_) {
    totalICPtime_ = totalICPtime_ + rmsTiming_;
  }

  file_stats << o3d_slam::toSecondsSinceFirstMeasurement(timestamp) << " " << numberOfIterations << " " << totalICPtime_ << " "
             << residualError_ << " " << motion.matrix().norm() << " " << nbMatches_ << std::endl;
}

void RosbagRangeDataProcessorRos::generateLocalizabilityCategory() {
  // Values
  auto& categories = deeperICPLogs_.localizationCategory;
  auto& timestamp = deeperICPLogs_.time_;

  // auto size = regNorms.size();
  file_localizability << o3d_slam::toSecondsSinceFirstMeasurement(timestamp) << " " << categories.col(0).row(3).value() << " "
                      << categories.col(0).row(4).value() << " " << categories.col(0).row(5).value() << " "
                      << categories.col(0).row(0).value() << " " << categories.col(0).row(1).value() << " "
                      << categories.col(0).row(2).value() << std::endl;
}

std::optional<visualization_msgs::Marker> RosbagRangeDataProcessorRos::generateMarkersForSurfaceNormalVectors(
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

void RosbagRangeDataProcessorRos::drawLinesBetweenPoses(const nav_msgs::Path& path1, const nav_msgs::Path& path2, const ros::Time& stamp) {
  if (!offlineDifferenceLinePub_.getNumSubscribers() > 0u && !offlineDifferenceLinePub_.isLatched()) {
    return;
  }

  if (path1.poses.size() != path2.poses.size()) {
    ROS_ERROR_STREAM("Path sizes are not equal. Skipping the line drawing.");
    return;
  }

  visualization_msgs::Marker line_list;
  // Change the frame_id as per your requirement
  line_list.header.frame_id = slam_->frames_.mapFrame;
  line_list.header.stamp = stamp;
  line_list.ns = "paths";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.008;  // Line width

  // Setting color for the lines (you can change color as needed)
  line_list.color.r = 0.0;
  line_list.color.g = 1.0;
  line_list.color.b = 1.0;
  line_list.color.a = 1.0;  // Alpha

  for (size_t i = 0; i < path1.poses.size(); i++) {
    geometry_msgs::Point p_start;
    p_start.x = path1.poses[i].pose.position.x;
    p_start.y = path1.poses[i].pose.position.y;
    p_start.z = path1.poses[i].pose.position.z;
    line_list.points.push_back(p_start);

    geometry_msgs::Point p_end;
    p_end.x = path2.poses[i].pose.position.x;
    p_end.y = path2.poses[i].pose.position.y;
    p_end.z = path2.poses[i].pose.position.z;
    line_list.points.push_back(p_end);
  }

  offlineDifferenceLinePub_.publish(line_list);
}

std::tuple<ros::WallDuration, ros::WallDuration, ros::WallDuration> RosbagRangeDataProcessorRos::usePairForRegistration() {
  const ros::WallTime odometryProcessingStartTime{ros::WallTime::now()};

  slam_->callofflineOdometryWorker();

  const auto odometryProcessingElapsed{ros::WallTime::now() - odometryProcessingStartTime};

  // Odometry is processed, now mapping
  const ros::WallTime mappingProcessingStartTime{ros::WallTime::now()};

  slam_->callofflineMappingWorker();

  const auto mappingProcessingElapsed{ros::WallTime::now() - mappingProcessingStartTime};
  ROS_INFO_STREAM("Mapping Operations took "
                  << "\033[92m" << mappingProcessingElapsed.toNSec() / 1000000u << "ms"
                  << "\033[0m");

  // Mapping is processed, now checking loop closures,
  const ros::WallTime loopclosureProcessingStartTime{ros::WallTime::now()};

  slam_->callofflineLoopClosureWorker();

  const auto loopclosureProcessingElapsed{ros::WallTime::now() - loopclosureProcessingStartTime};

  slam_->callofflineDenseMapWorker();

  return {std::make_tuple(odometryProcessingElapsed, mappingProcessingElapsed, loopclosureProcessingElapsed)};
}

bool RosbagRangeDataProcessorRos::readCalibrationIfNeeded() {
  if ((slam_->useSyncedPoses_) && (!slam_->useGPSforGroundTruth_)) {
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    baseToLidarTransform_ = tf2::eigenToTransform(transform);
    isStaticTransformFound_ = true;
    return true;
  }

  if (isStaticTransformFound_) {
    return true;
  }

  if (slam_->useGPSforGroundTruth_) {
    try {
      auto transformation_gps =
          tfBuffer_->lookupTransform(slam_->frames_.rangeSensorFrame, slam_->frames_.gpsFrame, tracker, ros::Duration(0.0));
      ros::spinOnce();

      ROS_INFO_STREAM("\033[92m"
                      << "Found the transform between " << slam_->frames_.rangeSensorFrame << " and " << slam_->frames_.gpsFrame
                      << "\033[0m");

      ROS_INFO_STREAM("\033[92m"
                      << "You dont believe me? Here it is:\n " << transformation_gps << "\033[0m");
      gpsToLidarTransform_ = transformation_gps;

    } catch (const tf2::TransformException& exception) {
      ROS_WARN_STREAM_THROTTLE(
          0.2, "Caught exception while looking for the transform fingers crossed it will appear soon: " << exception.what());
      return true;
    }
  }

  if (slam_->frames_.rangeSensorFrame != slam_->frames_.assumed_external_odometry_tracked_frame) {
    try {
      auto transformation = tfBuffer_->lookupTransform(slam_->frames_.rangeSensorFrame,
                                                       slam_->frames_.assumed_external_odometry_tracked_frame, tracker, ros::Duration(0.0));
      ros::spinOnce();
      ROS_INFO_STREAM("\033[92m"
                      << "Found the transform between " << slam_->frames_.rangeSensorFrame << " and "
                      << slam_->frames_.assumed_external_odometry_tracked_frame << "\033[0m");
      ROS_INFO_STREAM("\033[92m"
                      << "You dont believe me? Here it is:\n " << transformation << "\033[0m");

      baseToLidarTransform_ = transformation;
      isStaticTransformFound_ = true;

    } catch (const tf2::TransformException& exception) {
      ROS_WARN_STREAM_THROTTLE(
          0.2, "Caught exception while looking for the transform fingers crossed it will appear soon: " << exception.what());
      return true;
    }
    // This doesn't make anysense. Fix. We always return true.
    return true;

  } else {
    ros::spinOnce();
    isStaticTransformFound_ = true;
    return true;
  }
}

bool RosbagRangeDataProcessorRos::processRosbag() {
  std::vector<std::string> topics;

  if (slam_->useGPSforGroundTruth_) {
    ROS_INFO_STREAM("\033[33m"
                    << " GPS based ground truth registration is enabled. Expecting GNSS topic."
                    << "\033[39m");
    topics.push_back(slam_->gpsTopic_);
  }

  // We if we have clock topic fine, otherwise using the odometry topic as clock master.
  topics.push_back(clockTopic_);
  topics.push_back(cloudTopic_);
  topics.push_back(tfStaticTopic_);

  if (slam_->isUsingOdometryTopic()) {
    if (slam_->useSyncedPoses_) {
      topics.push_back(poseStampedTopic_);
    } else {
      // This is alternating between different common ROS msg types. I.e. poseStampedWithCovariance and Odometry
      topics.push_back(slam_->asyncOdometryTopic_);
    }
  }

  if (slam_->rePublishTf_) {
    ROS_INFO_STREAM(
        "\033[33m"
        << " So you like living risky? Tf will be republished. It is YOUR responsibility to ensure there is no frame duplication."
        << "\033[39m");
    topics.push_back(tfTopic_);
  }

  Timer rosbagTimer;

  // Open ROS bag.
  rosbag::Bag bag;
  try {
    bag.open(rosbagFilename_, rosbag::bagmode::Read);
  } catch (const rosbag::BagIOException& e) {
    ROS_ERROR_STREAM("Error opening ROS bag: '" << rosbagFilename_ << "'");
    return false;
  }
  ROS_INFO_STREAM("ROS bag '" << rosbagFilename_ << "' open.");

  if (!validateAnalysisTopic(bag, topics)) {
    bag.close();
    return false;
  }

  ROS_INFO_STREAM("\033[92m"
                  << " Here wo go... "
                  << "\033[0m");
  const ros::WallTime first{ros::WallTime::now() + ros::WallDuration(2.0)};
  ros::WallTime::sleepUntil(first);

  // The bag view we iterate over.
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  ros::Time stampLastIteration{view.getBeginTime()};
  ros::WallTime wallStampLastIteration{ros::WallTime::now()};
  const ros::WallTime wallStampStartSequentialRun{ros::WallTime::now()};

  SlamInputsBuffer slamInputsBuffer;
  std::unique_ptr<SlamInputs> slamInputs;

  for (const auto& messageInstance : view) {
    // If the node is shutdown, stop processing and do early return.
    if (!ros::ok()) {
      return false;
    }

    if (!slamInputs) {
      slamInputs = std::make_unique<SlamInputs>();
    }

    bool isInvalidMessageInBag = false;

    // Update time registers.
    if ((ros::Time::now() - stampLastIteration) >= ros::Duration(1.0)) {
      stampLastIteration = ros::Time::now();
      wallStampLastIteration = ros::WallTime::now();
    }

    // Clock.
    if (messageInstance.getTopic() == clockTopic_) {
      if (messageInstance.getDataType() == "rosgraph_msgs/Clock") {
        rosgraph_msgs::Clock::ConstPtr clockMessage = messageInstance.instantiate<rosgraph_msgs::Clock>();
        if (clockMessage != nullptr) {
          clockPublisher_.publish(clockMessage);
          // Assign the current "time" to a global variable for async operations.
          tracker = clockMessage->clock;
        } else {
          ROS_ERROR("clockMsgs not initialized something is wrong.");
          isInvalidMessageInBag = true;
          return false;
        }

      } else if ((messageInstance.getDataType() == "nav_msgs/Odometry") || (messageInstance.getDataType() == "geometry_msgs/PoseStamped") ||
                 (messageInstance.getDataType() == "geometry_msgs/PoseWithCovarianceStamped")) {
        ROS_WARN_STREAM_ONCE("SHOWN ONCE: Using async odometry topic as clock master. This is sub-optimal. Topic: " << clockTopic_);

        rosgraph_msgs::Clock clockMessage;
        clockMessage.clock = messageInstance.getTime();
        clockPublisher_.publish(clockMessage);
        // Assign the current "time" to a global variable for async operations.
        tracker = clockMessage.clock;

      } else {
        ROS_ERROR("This type of clock alternative is not supported");
        return false;
      }

      // Spins once here.
      if (!readCalibrationIfNeeded()) {
        ROS_ERROR("Calibration failed to read. Exiting.");
        return false;
      }

      // Align the clock and the bag time.
      if (timeDiff_ == ros::Duration(0.0)) {
        timeDiff_ = tracker - view.getBeginTime();

        // If the bag time is ahead of the clock, we need to calculate the otherway around.
        if (timeDiff_ < ros::Duration(0.0)) {
          timeDiff_ = view.getBeginTime() - tracker;
        }

        ROS_INFO_STREAM("The calculated time difference between the clock and bag is: " << timeDiff_.toNSec() / 1000000u << "ms");
      }

      // Skip processes based on the required bag start time.
      if ((std::abs(((tracker + timeDiff_) - view.getBeginTime()).toSec() <= slam_->bagReplayStartTime_)) &&
          (slam_->bagReplayStartTime_ != 0.0)) {
        ROS_INFO_STREAM("Rosbag starting: " << std::abs(((tracker + timeDiff_) - view.getBeginTime()).toSec()) << " / "
                                            << slam_->bagReplayEndTime_ << " s");
        isBagReadyToPlay_ = false;
        continue;
      } else {
        isBagReadyToPlay_ = true;
      }

      // Process data in buffers.
      if (processBuffers(slamInputsBuffer)) {
        // const auto processingElapsed{ros::WallTime::now() - processingStartTime};
        // ROS_INFO_STREAM("Processing buffer took " << processingElapsed.toNSec() / 1000000u << "ms");
        // Display progress to the user.
        const auto sequentialRunElapsed{ros::WallTime::now() - wallStampStartSequentialRun};

        ROS_INFO_STREAM("Replay run time: " << sequentialRunElapsed.toSec()
                                            << "s. ROS bag time: " << ((tracker + timeDiff_) - view.getBeginTime()).toSec() << " / "
                                            << (view.getEndTime() - view.getBeginTime()) << " s");

        // Play the bag until a certain time that the user decides.
        if (((tracker + timeDiff_) - view.getBeginTime()).toSec() >= slam_->bagReplayEndTime_) {
          break;
        }
      }
    }

    // Gnss message.
    if (messageInstance.getTopic() == slam_->gpsTopic_) {
      sensor_msgs::NavSatFix::ConstPtr message = messageInstance.instantiate<sensor_msgs::NavSatFix>();
      if (message != nullptr) {
        // Static method variables
        static Eigen::Vector3d accumulatedCoordinates__(0.0, 0.0, 0.0);
        static Eigen::Vector3d W_t_W_Gnss_km1__;
        static int gnssCallbackCounter__ = 0;
        Eigen::Vector3d W_t_W_Gnss;

        // Start
        ++gnssCallbackCounter__;
        Eigen::Vector3d gnssCoord = Eigen::Vector3d(message->latitude, message->longitude, message->altitude);
        Eigen::Vector3d estCovarianceXYZ(message->position_covariance[0], message->position_covariance[4], message->position_covariance[8]);

        if (not gpsInitialized_) {
          if (gnssCallbackCounter__ < 20 + 1) {
            // Wait until measurements got accumulated
            accumulatedCoordinates__ += gnssCoord;
            if (!(gnssCallbackCounter__ % 10)) {
              std::cout << YELLOW_START << "Offline Replayer:" << COLOR_END << " NOT ENOUGH Gnss MESSAGES ARRIVED!" << std::endl;
            }
            // return was here
          } else if (gnssCallbackCounter__ == 20 + 1) {
            gnssHandlerPtr_->initHandler(accumulatedCoordinates__ / 20);
            ++gnssCallbackCounter__;

            // Convert to cartesian coordinates
            gnssHandlerPtr_->convertNavSatToPosition(gnssCoord, W_t_W_Gnss);

            geometry_msgs::PoseStamped odomPose_transformed;
            geometry_msgs::PoseStamped odomPose;
            odomPose.pose.position.x = W_t_W_Gnss.x();
            odomPose.pose.position.y = W_t_W_Gnss.y();
            odomPose.pose.position.z = W_t_W_Gnss.z();
            odomPose.pose.orientation.w = 1.0;
            odomPose.pose.orientation.x = 0.0;
            odomPose.pose.orientation.y = 0.0;
            odomPose.pose.orientation.z = 0.0;

            Eigen::Affine3d transform = Eigen::Affine3d::Identity();
            tf2::doTransform(odomPose, odomPose_transformed, tf2::eigenToTransform(transform));

            // TODO: Clean up
            double initYawEnuLidar = 0.0;
            gnssHandlerPtr_->setInitYaw(initYawEnuLidar);
            gpsInitialized_ = true;

            gnssHandlerPtr_->initHandler(gnssHandlerPtr_->getInitYaw());

            const double stamp = message->header.stamp.toSec();
            gnssFile_ << stamp << " ";
            gnssFile_ << odomPose_transformed.pose.position.x << " " << odomPose_transformed.pose.position.y << " "
                      << odomPose_transformed.pose.position.z << " ";
            gnssFile_ << odomPose_transformed.pose.orientation.x << " " << odomPose_transformed.pose.orientation.y << " "
                      << odomPose_transformed.pose.orientation.z << " " << odomPose.pose.orientation.w << std::endl;
          }

        } else {
          if (std::to_string(message->status.status) != "2") {
            ROS_WARN_STREAM("Navsat Status: " << std::to_string(message->status.status));
          } else {
            // Convert to cartesian coordinates
            gnssHandlerPtr_->convertNavSatToPosition(gnssCoord, W_t_W_Gnss);

            geometry_msgs::PoseStamped odomPose_transformed;
            geometry_msgs::PoseStamped odomPose;
            odomPose.pose.position.x = W_t_W_Gnss.x();
            odomPose.pose.position.y = W_t_W_Gnss.y();
            odomPose.pose.position.z = W_t_W_Gnss.z();
            odomPose.pose.orientation.w = 1.0;
            odomPose.pose.orientation.x = 0.0;
            odomPose.pose.orientation.y = 0.0;
            odomPose.pose.orientation.z = 0.0;
            // geometry_msgs::TransformStamped inverseTransform = tf2::eigenToTransform(eigenTransform.inverse());

            Eigen::Affine3d transform = Eigen::Affine3d::Identity();  // gpsToLidarTransform_
            tf2::doTransform(odomPose, odomPose_transformed, tf2::eigenToTransform(transform));
            // W_t_W_Gnss << odomPose_transformed.pose.position.x, odomPose_transformed.pose.position.y,
            // odomPose_transformed.pose.position.z;
            /// Add GNSS to Path
            addToPathMsg(measGnss_worldGnssPathPtr_, "enu", message->header.stamp, W_t_W_Gnss, 800 * 4);  // this is not map_o3d but enu
            const double stamp = message->header.stamp.toSec();
            gnssFile_ << stamp << " ";
            gnssFile_ << odomPose_transformed.pose.position.x << " " << odomPose_transformed.pose.position.y << " "
                      << odomPose_transformed.pose.position.z << " ";
            gnssFile_ << odomPose_transformed.pose.orientation.x << " " << odomPose_transformed.pose.orientation.y << " "
                      << odomPose_transformed.pose.orientation.z << " " << odomPose_transformed.pose.orientation.w << std::endl;
          }
        }

      } else {
        isInvalidMessageInBag = true;
        ROS_WARN("Invalid message found in ROS bag.");
      }
    }

    // Tf static.
    if (messageInstance.getTopic() == tfStaticTopic_) {
      tf2_msgs::TFMessage::ConstPtr message = messageInstance.instantiate<tf2_msgs::TFMessage>();
      if (message != nullptr) {
        staticTransformBroadcaster_.sendTransform(message->transforms);

      } else {
        isInvalidMessageInBag = true;
        ROS_WARN("Invalid message found in ROS bag.");
      }
    }

    // Tf.
    if (messageInstance.getTopic() == tfTopic_) {
      tf2_msgs::TFMessage::ConstPtr message = messageInstance.instantiate<tf2_msgs::TFMessage>();
      if (message != nullptr) {
        transformBroadcaster_.sendTransform(message->transforms);

        if (slam_->useGPSforGroundTruth_) {
          geometry_msgs::TransformStamped transformStamped = tf2::eigenToTransform(mapEnuTransform_);
          transformStamped.header.stamp = tracker;
          transformStamped.header.frame_id = slam_->frames_.mapFrame;
          transformStamped.child_frame_id = "enu";
          transformBroadcaster_.sendTransform(transformStamped);
        }

      } else {
        isInvalidMessageInBag = true;
        ROS_WARN("Invalid message found in ROS bag.");
      }
    }

    if (!isBagReadyToPlay_) {
      continue;
    }

    // Pointcloud
    if (messageInstance.getTopic() == cloudTopic_) {
      sensor_msgs::PointCloud2::ConstPtr message = messageInstance.instantiate<sensor_msgs::PointCloud2>();
      if (message != nullptr) {
        if (slam_->isRMSenabled_) {
          sensor_msgs::PointCloud2::Ptr rmsPointCloud = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
          *rmsPointCloud = *message;
          const auto startTime__ = std::chrono::steady_clock::now();
          rms->sample(rmsPointCloud);
          const auto endTime__ = std::chrono::steady_clock::now();
          rmsTiming_ = std::chrono::duration_cast<std::chrono::microseconds>(endTime__ - startTime__).count() / 1e3;
          slamInputs->pointCloud_ = rmsPointCloud;
        } else {
          slamInputs->pointCloud_ = message;
        }

        if (slamInputs->pointCloud_) {
          // Read the point cloud frame from the topic. Think whether having the option to change it makes sense.
          slam_->frames_.rangeSensorFrame = slamInputs->pointCloud_->header.frame_id;
          if (slam_->frames_.rangeSensorFrame != slamInputs->pointCloud_->header.frame_id) {
            ROS_ERROR("Tracked frame and PC frame are not same. This is not supported yet.");
            ROS_ERROR_STREAM("Tracked frame: " << slam_->frames_.rangeSensorFrame
                                               << " and PC frame: " << slamInputs->pointCloud_->header.frame_id);
            ROS_ERROR("Terminating the replayer node.");
            return false;
          }
        } else {
          isInvalidMessageInBag = true;
          ROS_WARN("Invalid message found in ROS bag.");
        }

        if (!slam_->useSyncedPoses_) {
          if (slamInputs && slamInputs->pointCloud_) {
            // std::cout << " slamInputs IS HEALTHY adding to buffer  " << std::endl;
            slamInputsBuffer.emplace_back(std::move(slamInputs));
          } else {
            std::cout << " slamInputs is not initialized.  " << std::endl;
          }
        }

      } else {
        isInvalidMessageInBag = true;
        ROS_WARN("Invalid message found in ROS bag.");
      }
    }

    if (slam_->useSyncedPoses_) {
      // Synced odometry poses. This expects the odometry poses exactly in the same timestamp as the pc msgs.
      if (messageInstance.getTopic() == poseStampedTopic_) {
        geometry_msgs::PoseStamped::ConstPtr message = messageInstance.instantiate<geometry_msgs::PoseStamped>();
        if (message != nullptr) {
          // Add pose to buffer.
          slamInputs->odometryPose_ = message;

        } else {
          isInvalidMessageInBag = true;
          ROS_WARN("Invalid message found in ROS bag.");
          return false;
        }
      }
    } else {
      // Async Msgs. Currently only supports geometry_msgs::PoseWithCovarianceStamped
      if (messageInstance.getTopic() == slam_->asyncOdometryTopic_) {
        if (messageInstance.getDataType() == "geometry_msgs/PoseWithCovarianceStamped") {
          geometry_msgs::PoseWithCovarianceStamped::ConstPtr message =
              messageInstance.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
          if (message != nullptr) {
            geometry_msgs::PoseStamped odomPose;
            odomPose.pose = message->pose.pose;

            geometry_msgs::PoseStamped odomPose_transformed;
            Eigen::Isometry3d eigenTransform = tf2::transformToEigen(baseToLidarTransform_);
            slam_->setExternalOdometryFrameToCloudFrameCalibration(eigenTransform);
            tf2::doTransform(odomPose, odomPose_transformed, baseToLidarTransform_);

            if (isFirstMessage_ && isStaticTransformFound_) {
              geometry_msgs::PoseStamped initialPose = odomPose_transformed;

              // initialPose.position=odomPose.position;
              initialPose.pose.orientation.w = 1.0;
              initialPose.pose.orientation.z = 0.0;
              initialPose.pose.orientation.y = 0.0;
              initialPose.pose.orientation.x = 0.0;
              ROS_INFO("Initial Transform is set. Nice.");
              std::cout << " Initial Transform value (Rotation enforced to be Identity): "
                        << "\033[92m" << o3d_slam::asString(o3d_slam::getTransform(initialPose.pose)) << " \n"
                        << "\033[0m";
              slam_->setInitialTransform(o3d_slam::getTransform(initialPose.pose).matrix());
              isFirstMessage_ = false;
            }

            if (!(slam_->addOdometryPoseToBuffer(o3d_slam::getTransform(odomPose.pose), fromRos(message->header.stamp)))) {
              ROS_ERROR("Couldn't add pose to buffer. This is not unexpected, you should be concerned.");
              return false;
            }

          } else {
            isInvalidMessageInBag = true;
            ROS_WARN("Invalid message found in Rosbag, you are allowed to panic.");
            return false;
          }

        } else if (messageInstance.getDataType() == "nav_msgs/Odometry") {
          nav_msgs::Odometry::ConstPtr message = messageInstance.instantiate<nav_msgs::Odometry>();
          if (message != nullptr) {
            nav_msgs::Odometry odomPose;
            odomPose.pose.pose = message->pose.pose;

            nav_msgs::Odometry odomPose_transformed;
            Eigen::Isometry3d eigenTransform = tf2::transformToEigen(baseToLidarTransform_);
            slam_->setExternalOdometryFrameToCloudFrameCalibration(eigenTransform);
            tf2::doTransform(odomPose.pose.pose, odomPose_transformed.pose.pose, baseToLidarTransform_);

            if (isFirstMessage_ && isStaticTransformFound_) {
              nav_msgs::Odometry initialPose = odomPose_transformed;

              initialPose.pose.pose.orientation.w = 1.0;
              initialPose.pose.pose.orientation.z = 0.0;
              initialPose.pose.pose.orientation.y = 0.0;
              initialPose.pose.pose.orientation.x = 0.0;
              ROS_INFO("Initial Transform is set. Nice.");
              slam_->setInitialTransform(o3d_slam::getTransform(initialPose.pose.pose).matrix());
              isFirstMessage_ = false;
            }

            if (!(slam_->addOdometryPoseToBuffer(o3d_slam::getTransform(odomPose.pose.pose), fromRos(message->header.stamp)))) {
              ROS_ERROR("Couldn't Add pose to buffer. This is not unexpected, you should be concerned.");
              return false;
            }

          } else {
            isInvalidMessageInBag = true;
            ROS_WARN("Invalid message found in Rosbag, you are allowed to panic.");
          }
        } else if (messageInstance.getDataType() == "geometry_msgs/PoseStamped") {
          geometry_msgs::PoseStamped::ConstPtr message = messageInstance.instantiate<geometry_msgs::PoseStamped>();
          if (message != nullptr) {
            geometry_msgs::PoseStamped odomPose;
            odomPose.pose = message->pose;

            nav_msgs::Odometry odomPose_transformed;
            Eigen::Isometry3d eigenTransform = tf2::transformToEigen(baseToLidarTransform_);
            // geometry_msgs::TransformStamped inverseTransform = tf2::eigenToTransform(eigenTransform.inverse());
            slam_->setExternalOdometryFrameToCloudFrameCalibration(eigenTransform);
            tf2::doTransform(odomPose.pose, odomPose_transformed.pose.pose, baseToLidarTransform_);

            if (isFirstMessage_ && isStaticTransformFound_) {
              nav_msgs::Odometry initialPose = odomPose_transformed;

              initialPose.pose.pose.orientation.w = 1.0;
              initialPose.pose.pose.orientation.z = 0.0;
              initialPose.pose.pose.orientation.y = 0.0;
              initialPose.pose.pose.orientation.x = 0.0;
              ROS_INFO("Initial Transform is set. Nice.");
              slam_->setInitialTransform(o3d_slam::getTransform(initialPose.pose.pose).matrix());
              isFirstMessage_ = false;
            }

            if ((slam_->asyncOdometryTopic_ == "/noised_prior") || (slam_->asyncOdometryTopic_ == "/very_noised_prior")) {
              if ((slam_->asyncOdometryTopic_ == "/noised_prior")) {
                ROS_INFO_STREAM_THROTTLE(1, "\033[92m"
                                                << "You are working with a noised prior care."
                                                << "\033[0m");
              }

              if ((slam_->asyncOdometryTopic_ == "/very_noised_prior")) {
                ROS_INFO_STREAM_THROTTLE(1, "\033[92m"
                                                << "You are working with VERY HIGH noised prior care."
                                                << "\033[0m");
              }

            } else {
              // // This was used for initial experiments
              // geometry_msgs::Pose noisy_pose = addUniformNoiseToPose(odomPose.pose, 0.01, 0.005);  // Adjust noise magnitudes as needed

              // This was used for initial experiments
              // geometry_msgs::Pose noisy_pose = addNoiseToPose_with_mean(odomPose.pose, 0.01, 0.005, 0.05, 0.0, false);

              // ROS_INFO_STREAM("\033[92m"
              //                 << "SAVING NOISED PRIOR"
              //                 << "\033[0m");

              // if (!(slam_->addOdometryPoseToBuffer(o3d_slam::getTransform(noisy_pose), fromRos(message->header.stamp)))) {
              //   ROS_ERROR("Couldn't Add pose to buffer. This is not unexpected, you should be concerned.");
              //   return false;
              // }

              // geometry_msgs::PoseStamped saveNoisedPose;
              // saveNoisedPose.pose = noisy_pose;
              // saveNoisedPose.header = message->header;

              // noisedoutBag.write("/very_noised_prior", message->header.stamp, saveNoisedPose);
            }

            if (!(slam_->addOdometryPoseToBuffer(o3d_slam::getTransform(odomPose.pose), fromRos(message->header.stamp)))) {
              ROS_ERROR("Couldn't Add pose to buffer. This is not unexpected, you should be concerned.");
              return false;
            }

          } else {
            isInvalidMessageInBag = true;
            ROS_WARN("Invalid message found in Rosbag, you are allowed to panic.");
          }
        } else {
          ROS_WARN("This msg type is not supported yet. Exiting.");
          return false;
        }
      }  // else
    }

    if (slam_->isUsingOdometryTopic()) {
      if (slam_->useSyncedPoses_) {
        // Expecting sync odometry and PC.
        if (slamInputs && slamInputs->pointCloud_ && slamInputs->odometryPose_ &&
            slamInputs->pointCloud_->header.stamp == slamInputs->odometryPose_->header.stamp) {
          slamInputsBuffer.emplace_back(std::move(slamInputs));
        }
      }
    }

    const ros::WallDuration processingWallDurationActual{ros::WallTime::now() - wallStampLastIteration};
    const ros::Duration processingDurationActual{tracker - stampLastIteration};
    const auto rosWallTimeRatio{processingDurationActual.toSec() / processingWallDurationActual.toSec()};
    // ROS_INFO_STREAM("Processing duration. Wall = " << processingWallDurationActual.toSec()
    //                                               << "s ; ROS = " << processingDurationActual.toSec() << "s.");
    // ROS_INFO_STREAM("Desired processing rate = " << slamInputsmaxProcessingRate_);
    // ROS_INFO_STREAM("Actual Ratio ros/wall time = " << rosWallTimeRatio);

    if (maxProcessingRate_ <= 0) {
      continue;
    }

    // ROS time -> sleep in scaled wall time.
    if (rosWallTimeRatio > maxProcessingRate_) {
      // Check if we are reaching the desired processing rate. If not, sleep in scaled wall time.
      const auto desiredProcessingPeriod{ros::WallDuration(1.0 / maxProcessingRate_)};
      const auto scaledWallSleep = rosWallTimeRatio != 0 ? desiredProcessingPeriod.toSec() / rosWallTimeRatio : 0;
      // ROS_INFO_STREAM("Scaled sleep duration = " << scaledWallSleep);
      const ros::WallTime processingWallTimeExpected{ros::WallTime::now() + ros::WallDuration(scaledWallSleep)};
      ros::WallTime::sleepUntil(processingWallTimeExpected);

      // Update time register.
      wallStampLastIteration = ros::WallTime::now();
    }
  }

  processBuffers(slamInputsBuffer);

  ROS_INFO("Finished running through the bag.");
  const ros::Time bag_begin_time = view.getBeginTime();
  const ros::Time bag_end_time = view.getEndTime();

  const double bag_duration = (bag_end_time - bag_begin_time).toSec();
  const double paramBagDuration = slam_->bagReplayEndTime_ - slam_->bagReplayStartTime_;
  double printedBagDuration = bag_duration;

  if (paramBagDuration < bag_duration) {
    printedBagDuration = paramBagDuration;
  }

  std::cout << "Rosbag processing finished. Rosbag duration: " << printedBagDuration << " sec."
            << " Time elapsed for processing: " << rosbagTimer.elapsedSec() << " sec. \n \n";

  bag.close();

  slam_->offlineFinishProcessing();

  return true;
}
// (odomPose.pose, 0.05, 0.02, 0.05, 0.0, false);
geometry_msgs::Pose RosbagRangeDataProcessorRos::addNoiseToPose_with_mean(const geometry_msgs::Pose& original_pose,
                                                                          double position_noise_magnitude,
                                                                          double orientation_noise_magnitude, double position_mean,
                                                                          double orientation_mean, bool use_zero_mean) {
  // Copy the original pose to modify
  geometry_msgs::Pose noisy_pose = original_pose;

  // Random device
  std::random_device rd;
  std::mt19937 gen(rd());

  if (use_zero_mean) {
    // Uniform distributions for zero-mean noise
    std::uniform_real_distribution<> pos_dist(-position_noise_magnitude, position_noise_magnitude);
    std::uniform_real_distribution<> orient_dist(-orientation_noise_magnitude, orientation_noise_magnitude);

    // Add uniform noise to position
    noisy_pose.position.x += pos_dist(gen);
    noisy_pose.position.y += pos_dist(gen);
    noisy_pose.position.z += pos_dist(gen);

    // Add uniform noise to orientation quaternion
    noisy_pose.orientation.x += orient_dist(gen);
    noisy_pose.orientation.y += orient_dist(gen);
    noisy_pose.orientation.z += orient_dist(gen);
    noisy_pose.orientation.w += orient_dist(gen);
  } else {
    // Gaussian distributions for non-zero mean noise
    std::normal_distribution<> pos_dist(position_mean, position_noise_magnitude);
    std::normal_distribution<> orient_dist(orientation_mean, orientation_noise_magnitude);

    // Add Gaussian noise to position
    noisy_pose.position.x += pos_dist(gen);
    noisy_pose.position.y += pos_dist(gen);
    noisy_pose.position.z += pos_dist(gen);

    // Add Gaussian noise to orientation quaternion
    noisy_pose.orientation.x += orient_dist(gen);
    noisy_pose.orientation.y += orient_dist(gen);
    noisy_pose.orientation.z += orient_dist(gen);
    noisy_pose.orientation.w += orient_dist(gen);
  }

  // Normalize quaternion to ensure it represents a valid rotation
  double norm = std::sqrt(noisy_pose.orientation.x * noisy_pose.orientation.x + noisy_pose.orientation.y * noisy_pose.orientation.y +
                          noisy_pose.orientation.z * noisy_pose.orientation.z + noisy_pose.orientation.w * noisy_pose.orientation.w);
  noisy_pose.orientation.x /= norm;
  noisy_pose.orientation.y /= norm;
  noisy_pose.orientation.z /= norm;
  noisy_pose.orientation.w /= norm;

  return noisy_pose;
}

geometry_msgs::Pose RosbagRangeDataProcessorRos::addUniformNoiseToPose(const geometry_msgs::Pose& original_pose,
                                                                       double position_noise_magnitude,
                                                                       double orientation_noise_magnitude) {
  // Copy the original pose to modify
  geometry_msgs::Pose noisy_pose = original_pose;

  // Random device
  std::random_device rd;
  // const int kRandomSeed = 0x18273645;
  // std::seed_seq s2{kRandomSeed};
  std::mt19937 gen(rd());

  // Uniform distributions for position and orientation noise
  std::uniform_real_distribution<> pos_dist(-position_noise_magnitude, position_noise_magnitude);
  std::uniform_real_distribution<> orient_dist(-orientation_noise_magnitude, orientation_noise_magnitude);

  // Add uniform noise to position
  noisy_pose.position.x += pos_dist(gen);
  noisy_pose.position.y += pos_dist(gen);
  noisy_pose.position.z += pos_dist(gen);

  // Add uniform noise to orientation quaternion (simple method, not preserving exact orientation)
  noisy_pose.orientation.x += orient_dist(gen);
  noisy_pose.orientation.y += orient_dist(gen);
  noisy_pose.orientation.z += orient_dist(gen);
  noisy_pose.orientation.w += orient_dist(gen);

  // Normalize quaternion to ensure it represents a valid rotation
  double norm = std::sqrt(noisy_pose.orientation.x * noisy_pose.orientation.x + noisy_pose.orientation.y * noisy_pose.orientation.y +
                          noisy_pose.orientation.z * noisy_pose.orientation.z + noisy_pose.orientation.w * noisy_pose.orientation.w);
  noisy_pose.orientation.x /= norm;
  noisy_pose.orientation.y /= norm;
  noisy_pose.orientation.z /= norm;
  noisy_pose.orientation.w /= norm;

  return noisy_pose;
}

Eigen::Matrix4d RosbagRangeDataProcessorRos::pose_stamped_to_matrix(const geometry_msgs::Pose& pose_stamped) {
  Eigen::Vector3d position(pose_stamped.position.x, pose_stamped.position.y, pose_stamped.position.z);
  Eigen::Quaterniond orientation(pose_stamped.orientation.w, pose_stamped.orientation.x, pose_stamped.orientation.y,
                                 pose_stamped.orientation.z);

  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix.block<3, 3>(0, 0) = orientation.toRotationMatrix();
  matrix.block<3, 1>(0, 3) = position;

  return matrix;
}

geometry_msgs::Pose RosbagRangeDataProcessorRos::matrix_to_pose_stamped(const Eigen::Matrix4d& matrix) {
  geometry_msgs::Pose pose;

  Eigen::Quaterniond orientation(matrix.block<3, 3>(0, 0));
  pose.orientation.w = orientation.w();
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();

  Eigen::Vector3d position(matrix.block<3, 1>(0, 3));
  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();

  return pose;
}

geometry_msgs::Pose RosbagRangeDataProcessorRos::motionBasedNoise(const geometry_msgs::Pose poseMsg, double transNoise,
                                                                  double directionalTransNoise, double rotationNoise) {
  // Convert to PoseStamped and add to the logs.
  // poseStamped.header = poseStampedMsg->header;
  // poseStamped.pose = poseStampedMsg->pose.pose;

  geometry_msgs::Pose noisedPose = poseMsg;

  // Convert to 4x4 matrix to get the diff.
  Eigen::Matrix4d prePose = pose_stamped_to_matrix(prePoseStamped_);
  Eigen::Matrix4d Pose = pose_stamped_to_matrix(poseMsg);

  // Motion Difference
  Eigen::Matrix4d motionMatrix = prePose.inverse() * Pose;

  // Convert back to PoseStamped
  geometry_msgs::Pose motion = matrix_to_pose_stamped(motionMatrix);

  // Memory
  prePoseStamped_ = poseMsg;

  float positionNorm =
      std::sqrt(motion.position.x * motion.position.x + motion.position.y * motion.position.y + motion.position.z * motion.position.z);

  // std::cout << "positionNorm: " << positionNorm << std::endl;

  double positionNoiseVariance_{transNoise};  // 0.1   0.05  worked  0.015 nice

  // ADD NOISE
  std::random_device randomDevice;
  std::seed_seq randomSeed{randomDevice(), randomDevice(), randomDevice(), randomDevice(),
                           randomDevice(), randomDevice(), randomDevice(), randomDevice()};
  std::mt19937 randomNumberGenerator(randomSeed);  // randomSeed 5 works

  int signTranslation = 1;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 1.0);

  if (dis(gen) < 0.5) {
    signTranslation = -1;
  } else {
    signTranslation = 1;
  }

  std::normal_distribution<double> distribution(signTranslation * positionNorm * positionNoiseVariance_ * positionNoiseVariance_,
                                                positionNorm * positionNoiseVariance_);

  std::random_device randomDeviceRot;
  std::seed_seq randomSeedRot{randomDeviceRot(), randomDeviceRot(), randomDeviceRot(), randomDeviceRot(),
                              randomDeviceRot(), randomDeviceRot(), randomDeviceRot(), randomDeviceRot()};
  std::mt19937 randomNumberGeneratorRot(randomSeedRot);  // randomSeed 5 works
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////

  geometry_msgs::Quaternion quat = motion.orientation;
  Eigen::Quaterniond q(quat.w, quat.x, quat.y, quat.z);
  double angle = 2 * std::acos(q.w());
  Eigen::Vector3d axis(q.x(), q.y(), q.z());
  axis /= std::sqrt(1 - q.w() * q.w());
  Eigen::AngleAxisd aa(angle, axis);

  // std::cout << "Angle of rotation: " << aa.angle() << std::endl;
  // std::cout << "Axis of rotation: (" << aa.axis()(0) << ", " << aa.axis()(1) << ", " << aa.axis()(2) << ")" << std::endl;

  int signRotation = 1;
  std::random_device rdRot;
  std::mt19937 genRot(rdRot());
  std::uniform_real_distribution<> disRot(0.0, 1.0);

  if (disRot(genRot) < 0.5) {
    signRotation = -1;
  } else {
    signRotation = 1;
  }

  double rotationNoiseVariance_{rotationNoise};
  std::normal_distribution<double> distributionRot(signRotation * aa.angle() * rotationNoiseVariance_ * rotationNoiseVariance_,
                                                   aa.angle() * rotationNoiseVariance_);

  // Add gaussian noise to the localization position.
  noisedPose.position.x = noisedPose.position.x + distribution(randomNumberGenerator);
  noisedPose.position.y = noisedPose.position.y + distribution(randomNumberGenerator);
  noisedPose.position.z = noisedPose.position.z + distribution(randomNumberGenerator);

  geometry_msgs::Point normalized_direction = normalizeVector(motion.position);
  std::uniform_real_distribution<double> directional_dist(0.0, directionalTransNoise);
  std::normal_distribution<double> normal_distr(-directionalTransNoise, -directionalTransNoise / 3);

  // Add additional noise in the direction of motion
  // noisedPose.position.x += normalized_direction.x * directional_dist(gen);
  // noisedPose.position.y += normalized_direction.y * directional_dist(gen);
  // noisedPose.position.z += normalized_direction.z * directional_dist(gen);

  // std::cout << "Directional Noise x: " << (normalized_direction.x * normal_distr(gen) * positionNorm) << std::endl;
  // std::cout << "Directional Noise y: " << (normalized_direction.y * normal_distr(gen) * positionNorm) << std::endl;
  // std::cout << "Directional Noise z: " << (normalized_direction.z * normal_distr(gen) * positionNorm) << std::endl;

  noisedPose.position.x += noisedPose.position.x + (normalized_direction.x * normal_distr(gen) * positionNorm);
  noisedPose.position.y += noisedPose.position.y + (normalized_direction.y * normal_distr(gen) * positionNorm);
  noisedPose.position.z += noisedPose.position.z + (normalized_direction.z * normal_distr(gen) * positionNorm);

  // tf::Quaternion quaternionNoise;
  tf::Quaternion quaternionNoise(distributionRot(randomNumberGeneratorRot), distributionRot(randomNumberGeneratorRot),
                                 distributionRot(randomNumberGeneratorRot), distributionRot(randomNumberGeneratorRot));

  /*
quaternionNoise.setRPY(distributionRot(randomNumberGeneratorRot), distributionRot(randomNumberGeneratorRot),
distributionRot(randomNumberGeneratorRot));
*/
  tf::Quaternion quaternionTf(noisedPose.orientation.x, noisedPose.orientation.y, noisedPose.orientation.z, noisedPose.orientation.w);
  quaternionTf = quaternionTf + quaternionNoise;
  quaternionTf.normalize();

  tf::quaternionTFToMsg(quaternionTf, noisedPose.orientation);

  return noisedPose;
}

// Function to normalize a vector
geometry_msgs::Point RosbagRangeDataProcessorRos::normalizeVector(const geometry_msgs::Point& vector) {
  geometry_msgs::Point normalized_vector;
  double norm = vectorNorm(vector);
  if (norm > 0) {
    normalized_vector.x = vector.x / norm;
    normalized_vector.y = vector.y / norm;
    normalized_vector.z = vector.z / norm;
  }
  return normalized_vector;
}

// Helper function to calculate the norm of a vector
double RosbagRangeDataProcessorRos::vectorNorm(const geometry_msgs::Point& vector) {
  return std::sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

}  // namespace o3d_slam
