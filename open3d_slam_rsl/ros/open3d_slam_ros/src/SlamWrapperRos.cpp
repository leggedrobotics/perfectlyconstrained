/*
 * SlamWrapperRosRos.cpp
 *
 *  Created on: Apr 19, 2022
 *      Author: jelavice
 */

#include "open3d_slam_ros/SlamWrapperRos.hpp"

#include <open3d/Open3D.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <fstream>

#include "open3d_conversions/open3d_conversions.h"
#include "open3d_slam/Mapper.hpp"
#include "open3d_slam/Odometry.hpp"
#include "open3d_slam/OptimizationProblem.hpp"
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/assert.hpp"
#include "open3d_slam/constraint_builders.hpp"
#include "open3d_slam/croppers.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/math.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam_lua_io/parameter_loaders.hpp"
#include "open3d_slam_ros/helpers_ros.hpp"

#ifdef open3d_slam_ros_OPENMP_FOUND
#include <omp.h>
#endif

namespace o3d_slam {

namespace {
// Frames were used to be included here.
}

SlamWrapperRos::SlamWrapperRos(ros::NodeHandlePtr nh) : BASE(), nh_(nh) {
  tfBroadcaster_.reset(new tf2_ros::TransformBroadcaster());

  prevPublishedTimeScanToScan_ = fromUniversal(0);
  prevPublishedTimeScanToMap_ = fromUniversal(0);
}

SlamWrapperRos::~SlamWrapperRos() {
  if (tfWorker_.joinable()) {
    tfWorker_.join();
    std::cout << "Joined tf worker \n";
  }
  if (visualizationWorker_.joinable()) {
    visualizationWorker_.join();
    std::cout << "Joined visualization worker \n";
  }
  if (params_.odometry_.isPublishOdometryMsgs_ && odomPublisherWorker_.joinable()) {
    odomPublisherWorker_.join();
    std::cout << "Joined odom publisher worker \n";
  }
}

void SlamWrapperRos::startWorkers() {
  tfWorker_ = std::thread([this]() { tfWorker(); });
  visualizationWorker_ = std::thread([this]() { visualizationWorker(); });
  if (params_.odometry_.isPublishOdometryMsgs_) {
    odomPublisherWorker_ = std::thread([this]() { odomPublisherWorker(); });
  }

  BASE::startWorkers();
}

void SlamWrapperRos::odomPublisherWorker() {
  ros::Rate r(100.0);
  while (ros::ok()) {
    auto getTransformMsg = [this](const Transform& T, const Time& t) {
      ros::Time timestamp = toRos(t);
      geometry_msgs::TransformStamped transformMsg = o3d_slam::toRos(T.matrix(), timestamp, frames_.mapFrame, frames_.rangeSensorFrame);
      return transformMsg;
    };

    auto getOdomMsg = [](const geometry_msgs::TransformStamped& transformMsg) {
      nav_msgs::Odometry odomMsg;
      odomMsg.header = transformMsg.header;
      odomMsg.child_frame_id = transformMsg.child_frame_id;
      odomMsg.pose.pose.orientation = transformMsg.transform.rotation;
      odomMsg.pose.pose.position.x = transformMsg.transform.translation.x;
      odomMsg.pose.pose.position.y = transformMsg.transform.translation.y;
      odomMsg.pose.pose.position.z = transformMsg.transform.translation.z;
      return odomMsg;
    };

    const Time latestScanToScan = latestScanToScanRegistrationTimestamp_;
    const bool isAlreadyPublished = latestScanToScan == prevPublishedTimeScanToScanOdom_;
    if (!isAlreadyPublished && odometry_->hasProcessedMeasurements()) {
      const Transform T = odometry_->getOdomToRangeSensor(latestScanToScan);
      geometry_msgs::TransformStamped transformMsg = getTransformMsg(T, latestScanToScan);
      nav_msgs::Odometry odomMsg = getOdomMsg(transformMsg);
      publishIfSubscriberExists(transformMsg, scan2scanTransformPublisher_);
      publishIfSubscriberExists(odomMsg, scan2scanOdomPublisher_);
      prevPublishedTimeScanToScanOdom_ = latestScanToScan;
    }

    const Time latestScanToMap = latestScanToMapRefinementTimestamp_;
    const bool isScanToMapAlreadyPublished = latestScanToMap == prevPublishedTimeScanToMapOdom_;
    if (!isScanToMapAlreadyPublished && mapper_->hasProcessedMeasurements()) {
      const Transform T = mapper_->getMapToRangeSensor(latestScanToMap);
      geometry_msgs::TransformStamped transformMsg = getTransformMsg(T, latestScanToMap);
      nav_msgs::Odometry odomMsg = getOdomMsg(transformMsg);
      publishIfSubscriberExists(transformMsg, scan2mapTransformPublisher_);
      publishIfSubscriberExists(odomMsg, scan2mapOdomPublisher_);
      prevPublishedTimeScanToMapOdom_ = latestScanToMap;
    }

    ros::spinOnce();
    r.sleep();
  }
}

void SlamWrapperRos::offlineTfWorker() {
  const Time latestScanToScan = latestScanToScanRegistrationTimestamp_;
  const bool isAlreadyPublished = latestScanToScan == prevPublishedTimeScanToScan_;
  if ((!isAlreadyPublished) && (odometry_->hasProcessedMeasurements())) {
    const Transform T = odometry_->getOdomToRangeSensor(latestScanToScan);
    ros::Time timestamp = toRos(latestScanToScan);
    // o3d_slam::publishTfTransform(T.matrix(), timestamp, o3d_slam::odomFrame, frames_.rangeSensorFrame, tfBroadcaster_.get());
    o3d_slam::publishTfTransform(T.matrix(), timestamp, frames_.mapFrame, "raw_odom_o3d", tfBroadcaster_.get());
    prevPublishedTimeScanToScan_ = latestScanToScan;
  }

  const Time latestScanToMap = latestScanToMapRefinementTimestamp_;
  const bool isScanToMapAlreadyPublished = latestScanToMap == prevPublishedTimeScanToMap_;
  if (!isScanToMapAlreadyPublished && mapper_->hasProcessedMeasurements()) {
    publishMapToOdomTf(latestScanToMap);
    prevPublishedTimeScanToMap_ = latestScanToMap;
  }
}

void SlamWrapperRos::tfWorker() {
  ros::WallRate r(500.0);
  while (ros::ok()) {
    const Time latestScanToScan = latestScanToScanRegistrationTimestamp_;
    const bool isAlreadyPublished = latestScanToScan == prevPublishedTimeScanToScan_;
    if ((!isAlreadyPublished) && (odometry_->hasProcessedMeasurements())) {
      const Transform T = odometry_->getOdomToRangeSensor(latestScanToScan);
      ros::Time timestamp = toRos(latestScanToScan);
      // This distinguish the lidar frame in regular anymal tf and the re-publish by o3d.
      // std::string appendedSensor = frames_.rangeSensorFrame + "_o3d";
      o3d_slam::publishTfTransform(T.matrix().inverse(), timestamp, frames_.rangeSensorFrame, frames_.odomFrame, tfBroadcaster_.get());
      // o3d_slam::publishTfTransform(T.matrix(), timestamp, frames_.mapFrame, "raw_odom_o3d", tfBroadcaster_.get());
      prevPublishedTimeScanToScan_ = latestScanToScan;
    }

    const Time latestScanToMap = latestScanToMapRefinementTimestamp_;

    if (!isTimeValid(latestScanToMap)) {
      continue;
    }

    const bool isScanToMapAlreadyPublished = latestScanToMap == prevPublishedTimeScanToMap_;
    if (!isScanToMapAlreadyPublished && mapper_->hasProcessedMeasurements()) {
      publishMapToOdomTf(latestScanToMap);
      prevPublishedTimeScanToMap_ = latestScanToMap;

      if (trackedPathPub_.getNumSubscribers() > 0u || trackedPathPub_.isLatched()) {
        mapper_->trackedPath_.header.stamp = o3d_slam::toRos(latestScanToMap);
        mapper_->trackedPath_.header.frame_id = frames_.mapFrame;
        trackedPathPub_.publish(mapper_->trackedPath_);
      }

      if (bestGuessPathPub_.getNumSubscribers() > 0u || bestGuessPathPub_.isLatched()) {
        mapper_->bestGuessPath_.header.stamp = o3d_slam::toRos(latestScanToMap);
        mapper_->bestGuessPath_.header.frame_id = frames_.mapFrame;
        bestGuessPathPub_.publish(mapper_->bestGuessPath_);
      }

      // This function publishesh the lines that illustrate the corrections by the fine registration.
      drawLinesBetweenPoses(mapper_->trackedPath_, mapper_->bestGuessPath_, toRos(latestScanToMap));
    }

    ros::spinOnce();
    r.sleep();
  }
}

void SlamWrapperRos::drawLinesBetweenPoses(const nav_msgs::Path& path1, const nav_msgs::Path& path2, const ros::Time& stamp) {
  if (!differenceLinePub_.getNumSubscribers() > 0u && !differenceLinePub_.isLatched()) {
    return;
  }

  if (path1.poses.size() != path2.poses.size()) {
    ROS_ERROR_STREAM("Path sizes are not equal. Skipping the line drawing.");
    return;
  }

  visualization_msgs::Marker line_list;
  line_list.header.frame_id = frames_.mapFrame;  // Change the frame_id as per your requirement
  line_list.header.stamp = stamp;
  line_list.ns = "paths";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.006;  // Line width

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

  differenceLinePub_.publish(line_list);
}

void SlamWrapperRos::offlineVisualizationWorker() {
  const Time scanToScanTimestamp = latestScanToScanRegistrationTimestamp_;
  ros::Time timestamp = toRos(scanToScanTimestamp);
  o3d_slam::publishSubmapCoordinateAxes(mapper_->getSubmaps(), frames_.mapFrame, timestamp, submapOriginsPub_);
}

void SlamWrapperRos::visualizationWorker() {
  ros::WallRate r(20.0);
  while (ros::ok()) {
    const Time scanToScanTimestamp = latestScanToScanRegistrationTimestamp_;
    if (odometryInputPub_.getNumSubscribers() > 0 && isTimeValid(scanToScanTimestamp)) {
      const PointCloud odomInput = odometry_->getPreProcessedCloud();
      o3d_slam::publishCloud(odomInput, frames_.rangeSensorFrame, toRos(scanToScanTimestamp), odometryInputPub_);
    }

    const Time scanToMapTimestamp = latestScanToMapRefinementTimestamp_;
    if (isTimeValid(scanToMapTimestamp)) {
      publishDenseMap(scanToMapTimestamp);
      publishMaps(scanToMapTimestamp);
      if (!params_.mapper_.republishMap_ && params_.mapper_.isUseInitialMap_) {
        // publish initial map only once
        break;
      }
    }

    ros::spinOnce();
    r.sleep();
  }
}

bool SlamWrapperRos::readLibpointmatcherConfig(const std::string& path) {
  std::ifstream fileStream(path.c_str());
  if (!fileStream.good()) {
    ROS_ERROR_STREAM("Cannot load ICP configuration from " << path.c_str() << " .");
    return false;
  }
  mapper_->icp_.loadFromYaml(fileStream);
  isRMSenabled_ = mapper_->icp_.isRMSenabled();
  rmsLambda_ = mapper_->icp_.getRMSlambda();
  rmsVoxelSize_ = params_.mapper_.mapBuilder_.mapVoxelSize_;

  if (isRMSenabled_) {
    ROS_WARN_STREAM("##################################################");
    ROS_WARN_STREAM("Open3D-SLAM: RMS by Petracek et al. is enabled.");
    ROS_WARN_STREAM("Tunable parameter Lambda is set : " << rmsLambda_);
    ROS_WARN_STREAM("Voxel Size : " << rmsVoxelSize_);
    ROS_WARN_STREAM("##################################################");
  }

  return true;
}

void SlamWrapperRos::loadParametersAndInitialize() {
  odometryInputPub_ = nh_->advertise<sensor_msgs::PointCloud2>("odom_input", 1, true);
  mappingInputPub_ = nh_->advertise<sensor_msgs::PointCloud2>("mapping_input", 1, true);
  assembledMapPub_ = nh_->advertise<sensor_msgs::PointCloud2>("assembled_map", 1, true);
  denseMapPub_ = nh_->advertise<sensor_msgs::PointCloud2>("dense_map", 1, true);
  submapsPub_ = nh_->advertise<sensor_msgs::PointCloud2>("submaps", 1, true);
  submapOriginsPub_ = nh_->advertise<visualization_msgs::MarkerArray>("submap_origins", 1, true);

  scan2scanTransformPublisher_ = nh_->advertise<geometry_msgs::TransformStamped>("scan2scan_transform", 1, true);
  scan2scanOdomPublisher_ = nh_->advertise<nav_msgs::Odometry>("scan2scan_odometry", 1, true);
  scan2mapTransformPublisher_ = nh_->advertise<geometry_msgs::TransformStamped>("scan2map_transform", 1, true);
  scan2mapOdomPublisher_ = nh_->advertise<nav_msgs::Odometry>("scan2map_odometry", 1, true);

  trackedPathPub_ = nh_->advertise<nav_msgs::Path>("tracked_path_live", 1, false);
  bestGuessPathPub_ = nh_->advertise<nav_msgs::Path>("best_guess_path_live", 1, false);
  differenceLinePub_ = nh_->advertise<visualization_msgs::Marker>("differenceLines", false);

  saveMapSrv_ = nh_->advertiseService("save_map", &SlamWrapperRos::saveMapCallback, this);
  saveSubmapsSrv_ = nh_->advertiseService("save_submaps", &SlamWrapperRos::saveSubmapsCallback, this);

  // auto &logger = open3d::utility::Logger::GetInstance();
  // logger.SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);
  // Read Parameters
  const bool isOfflineReplay = o3d_slam::tryGetParam<bool>("is_read_from_rosbag", *nh_);

  folderPath_ = ros::package::getPath("open3d_slam_ros") + "/data/";
  mapSavingFolderPath_ = nh_->param<std::string>("map_saving_folder", folderPath_);

  // Offline advanced parameters
  useSyncedPoses_ = nh_->param<bool>("use_syncronized_poses_to_replay", false);
  rePublishTf_ = nh_->param<bool>("republish_tf_topic", false);

  relativeSleepDuration_ = nh_->param<double>("relative_sleep_duration", 0.0);
  bagReplayStartTime_ = nh_->param<double>("replay_start_time_as_second", 0.0);
  bagReplayEndTime_ = nh_->param<double>("replay_end_time_as_second", 8000.0);
  asyncOdometryTopic_ = nh_->param<std::string>("async_pose_topic", "/state_estimator/pose_in_odom");
  gpsTopic_ = nh_->param<std::string>("gps_topic", "");

  // Read whether subscribe to GPS. (Only for anymal forest experiment)
  useGPSforGroundTruth_ = nh_->param<bool>("use_gps_for_ground_truth", false);
  downsamplePointCloudForReplay_ = nh_->param<bool>("downsample_cloud_for_replay", false);
  saveProcessedBag_ = nh_->param<bool>("save_processed_bag", false);
  downSamplingSkippingRate_ = nh_->param<int>("downSamplingSkippingRate_", 5);

  // We read this from the pointcloud header. We dont support frame moving yet.
  frames_.rangeSensorFrame = "default";
  frames_.assumed_external_odometry_tracked_frame = nh_->param<std::string>("assumed_external_odometry_tracked_frame", "default");
  frames_.gpsFrame = nh_->param<std::string>("gps_frame", "default");

  if (isOfflineReplay) {
    ROS_INFO_STREAM("\033[92m"
                    << "The assumed external odometry tracked frame is: " << frames_.assumed_external_odometry_tracked_frame << "\033[0m");
    // ROS_INFO_STREAM("\033[92m" << "The tracked sensor frame and the expected cloud header frame is: " << frames_.rangeSensorFrame <<
    // "\033[0m");
    ROS_INFO_STREAM("Replay Time Config: Start Time(s): " << bagReplayStartTime_ << " End Time(s): " << bagReplayEndTime_);
  }

  const std::string paramFolderPath = nh_->param<std::string>("parameter_folder_path", "");
  const std::string icpYamlPath = nh_->param<std::string>("icp_yaml_full_path", "");
  const std::string paramFilename = nh_->param<std::string>("parameter_filename", "");
  SlamParameters params;
  io_lua::loadParameters(paramFolderPath, paramFilename, &params_);
  BASE::loadParametersAndInitialize();

  // Set and load the libpointmatcher config here.
  // std::string libpointmatcherConfigPath = ros::package::getPath("open3d_slam_ros") + "/param/icp.yaml";
  ROS_INFO_STREAM("libpointmatcherConfigPath: " << icpYamlPath);

  if (!readLibpointmatcherConfig(icpYamlPath)) {
    ROS_ERROR_STREAM("Returning early couldnt load ICP params for libpointmatcher ");
    return;
  }
}

bool SlamWrapperRos::saveMapCallback(open3d_slam_msgs::SaveMap::Request& req, open3d_slam_msgs::SaveMap::Response& res) {
  const bool savingResult = saveMap(mapSavingFolderPath_);
  res.statusMessage = savingResult ? "Map saved to: " + mapSavingFolderPath_ : "Error while saving map";
  return true;
}

bool SlamWrapperRos::saveSubmapsCallback(open3d_slam_msgs::SaveSubmaps::Request& req, open3d_slam_msgs::SaveSubmaps::Response& res) {
  const bool savingResult = saveSubmaps(mapSavingFolderPath_);
  res.statusMessage = savingResult ? "Submaps saved to: " + mapSavingFolderPath_ : "Error while saving submaps";
  return true;
}

void SlamWrapperRos::publishMapToOdomTf(const Time& time) {
  const ros::Time timestamp = toRos(time);
  // This is commented out since the map and odonm frames published by the o3d are connected through the range sensor frame.
  // o3d_slam::publishTfTransform(mapper_->getMapToOdom(time).matrix(), timestamp, frames_.mapFrame, frames_.odomFrame,
  // tfBroadcaster_.get());
  o3d_slam::publishTfTransform(mapper_->getMapToRangeSensor(time).matrix(), timestamp, frames_.mapFrame, "raw_rs_o3d",
                               tfBroadcaster_.get());

  if (!(mapper_->getMapToRangeSensorBuffer().empty())) {
    auto latestMapToRangeMeasurement_ = mapper_->getMapToRangeSensorBuffer().latest_measurement();

    if ((isTimeValid(latestMapToRangeMeasurement_.time_))) {
      // Publish lidar to map transform.
      // Since base is the parent of lidar, we cant publish o3d_map as the parent of lidar. Hence we publish it as a child of lidar.
      std::string adaptedMapFrame = frames_.mapFrame;
      o3d_slam::publishTfTransform(latestMapToRangeMeasurement_.transform_.matrix().inverse(),
                                   o3d_slam::toRos(latestMapToRangeMeasurement_.time_), frames_.rangeSensorFrame, adaptedMapFrame,
                                   tfBroadcaster_.get());
    }
  }
}

void SlamWrapperRos::publishDenseMap(const Time& time) {
  if (denseMapVisualizationUpdateTimer_.elapsedMsec() < params_.visualization_.visualizeEveryNmsec_) {
    return;
  }
  const auto denseMap = mapper_->getActiveSubmap().getDenseMapCopy();  // copy
  const ros::Time timestamp = toRos(time);
  o3d_slam::publishCloud(denseMap.toPointCloud(), frames_.mapFrame, timestamp, denseMapPub_);
}

void SlamWrapperRos::publishMaps(const Time& time) {
  if (visualizationUpdateTimer_.elapsedMsec() < params_.visualization_.visualizeEveryNmsec_ && !isVisualizationFirstTime_) {
    return;
  }

  const ros::Time timestamp = toRos(time);
  {
    PointCloud map = mapper_->getAssembledMapPointCloud();
    voxelize(params_.visualization_.assembledMapVoxelSize_, &map);
    o3d_slam::publishCloud(map, frames_.mapFrame, timestamp, assembledMapPub_);
  }
  o3d_slam::publishCloud(mapper_->getPreprocessedScan(), frames_.rangeSensorFrame, timestamp, mappingInputPub_);
  o3d_slam::publishSubmapCoordinateAxes(mapper_->getSubmaps(), frames_.mapFrame, timestamp, submapOriginsPub_);
  if (submapsPub_.getNumSubscribers() > 0) {
    open3d::geometry::PointCloud cloud;
    o3d_slam::assembleColoredPointCloud(mapper_->getSubmaps(), &cloud);
    voxelize(params_.visualization_.submapVoxelSize_, &cloud);
    o3d_slam::publishCloud(cloud, frames_.mapFrame, timestamp, submapsPub_);
  }

  visualizationUpdateTimer_.reset();
  isVisualizationFirstTime_ = false;
}

}  // namespace o3d_slam
