/*
 * SlamWrapperRos.hpp
 *
 *  Created on: Apr 19, 2022
 *      Author: jelavice
 */

#pragma once

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "open3d_slam/SlamWrapper.hpp"
#include "open3d_slam_msgs/SaveMap.h"
#include "open3d_slam_msgs/SaveSubmaps.h"

namespace o3d_slam {

class SlamWrapperRos : public SlamWrapper {
  using BASE = SlamWrapper;

 public:
  SlamWrapperRos(ros::NodeHandlePtr nh);
  ~SlamWrapperRos() override;

  bool saveMapCallback(open3d_slam_msgs::SaveMap::Request& req, open3d_slam_msgs::SaveMap::Response& res);
  bool saveSubmapsCallback(open3d_slam_msgs::SaveSubmaps::Request& req, open3d_slam_msgs::SaveSubmaps::Response& res);
  void loadParametersAndInitialize() override;
  void startWorkers() override;

  bool readLibpointmatcherConfig(const std::string& path);

  void offlineTfWorker() override;
  void offlineVisualizationWorker() override;

  void drawLinesBetweenPoses(const nav_msgs::Path& path1, const nav_msgs::Path& path2, const ros::Time& stamp);

  geometry_msgs::TransformStamped baseToLidarTransform_;
  bool isStaticTransformAttempted_ = true;

 private:
  void tfWorker();
  void odomPublisherWorker();
  void visualizationWorker();
  void publishMaps(const Time& time);
  void publishDenseMap(const Time& time);
  void publishMapToOdomTf(const Time& time);

  ros::NodeHandlePtr nh_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  ros::Publisher odometryInputPub_, mappingInputPub_, submapOriginsPub_, assembledMapPub_, denseMapPub_, submapsPub_, bestGuessPathPub_,
      trackedPathPub_;
  ros::Publisher differenceLinePub_;
  ros::Publisher scan2scanTransformPublisher_, scan2scanOdomPublisher_, scan2mapTransformPublisher_, scan2mapOdomPublisher_;
  ros::ServiceServer saveMapSrv_, saveSubmapsSrv_;
  bool isVisualizationFirstTime_ = true;
  std::thread tfWorker_, visualizationWorker_, odomPublisherWorker_;
  Time prevPublishedTimeScanToScan_, prevPublishedTimeScanToMap_;
  Time prevPublishedTimeScanToScanOdom_, prevPublishedTimeScanToMapOdom_;
};

}  // namespace o3d_slam
