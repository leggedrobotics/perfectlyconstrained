/*
 * DataProcessorRos.hpp
 *
 *  Created on: Apr 21, 2022
 *      Author: jelavice
 */

#pragma once
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <std_msgs/ColorRGBA.h>
#include "open3d_slam/SlamWrapper.hpp"
#include "open3d_slam/time.hpp"
#include "open3d_slam/typedefs.hpp"

namespace o3d_slam {

enum class ColorKey { kWhite = 0, kRed, kGreen, kBlue, kCyan, kYellow, kGold, kGrey, kLavender, kOrange, kBlack };

struct RgbaColorMap {
  using Values = std::vector<float>;

  RgbaColorMap() = default;

  Values operator[](const ColorKey id) const { return rgb_.at(id); }

  const std::unordered_map<ColorKey, Values> rgb_ = {{ColorKey::kWhite, {1, 1, 1, 1}},
                                                     {ColorKey::kBlue, {0, 0, 1, 1}},
                                                     {ColorKey::kCyan, {0, 1, 1, 1}},
                                                     {ColorKey::kRed, {1, 0, 0, 1}},
                                                     {ColorKey::kGreen, {0, 1, 0, 1}},
                                                     {ColorKey::kGrey, {0.705, 0.674, 0.678, 1}},
                                                     {ColorKey::kLavender, {0.560, 0.501, 0.674, 1}},
                                                     {ColorKey::kYellow, {1, 1, 0.2, 1}},
                                                     {ColorKey::kGold, {0.898, 0.784, 0.462, 1}},
                                                     {ColorKey::kOrange, {1, 0.501, 0, 1}},
                                                     {ColorKey::kBlack, {0, 0, 0, 1}}};
};

class DataProcessorRos {
 public:
  DataProcessorRos(ros::NodeHandlePtr nh);
  virtual ~DataProcessorRos() = default;

  virtual void initialize() = 0;
  virtual void startProcessing() = 0;
  virtual void processMeasurement(const PointCloud& cloud, const Time& timestamp);
  virtual void processOdometry(const Transform& cloud, const Time& timestamp);
  void accumulateAndProcessRangeData(const PointCloud& cloud, const Time& timestamp);
  void processOdometryData(const Transform& transform, const Time& timestamp);
  void initCommonRosStuff();
  std::shared_ptr<SlamWrapper> getSlamPtr();

 protected:
  size_t numAccumulatedRangeDataCount_ = 0;
  size_t numPointCloudsReceived_ = 0;
  size_t numAccumulatedRangeDataDesired_ = 1;
  PointCloud accumulatedCloud_;

  ros::Publisher rawCloudPub_;
  ros::Publisher pubMeasWorldGnssPath_;
  ros::Publisher registeredCloudPub_;
  ros::Publisher offlinePathPub_;
  ros::Publisher surfaceNormalPub_;
  ros::Publisher offlineDifferenceLinePub_;
  ros::Publisher offlineBestGuessPathPub_;
  ros::Publisher condNumberArrowPublisher_;

  ros::Publisher addedImuMeasPub_;
  std::string cloudTopic_{""};
  std::string odometryTopic_{""};
  std::string poseStampedWithCovarianceTopic_{""};
  std::string poseStampedTopic_{""};
  std::string imuTopic_{""};

  std::shared_ptr<SlamWrapper> slam_;
  ros::NodeHandlePtr nh_;
};

}  // namespace o3d_slam
