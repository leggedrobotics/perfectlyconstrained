#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <deque>
#include <memory>
#include <string>
#include <vector>
#include "open3d_slam_ros/GnssHandler.hpp"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "open3d_slam/SlamWrapper.hpp"
#include "open3d_slam/output.hpp"

#include <std_srvs/Empty.h>

#include <rms/rms.h>
#include "open3d_slam_ros/DataProcessorRos.hpp"

namespace o3d_slam {

struct SlamInputs {
  sensor_msgs::PointCloud2::ConstPtr pointCloud_;
  geometry_msgs::PoseStamped::ConstPtr odometryPose_;

  SlamInputs() = default;

  SlamInputs(sensor_msgs::PointCloud2::ConstPtr pointCloud, geometry_msgs::PoseStamped::ConstPtr odometryPose)
      : pointCloud_(pointCloud), odometryPose_(odometryPose) {}
};

class RosbagRangeDataProcessorRos : public DataProcessorRos {
  using BASE = DataProcessorRos;
  using SlamInputsBuffer = std::deque<std::unique_ptr<SlamInputs>>;

 public:
  RosbagRangeDataProcessorRos(ros::NodeHandlePtr nh);
  ~RosbagRangeDataProcessorRos() override = default;

  void initialize() override;
  void startProcessing() override;
  void processMeasurement(const PointCloud& cloud, const Time& timestamp) override;
  void poseStampedCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  bool processBuffers(SlamInputsBuffer& buffer);
  void drawLinesBetweenPoses(const nav_msgs::Path& path1, const nav_msgs::Path& path2, const ros::Time& stamp);
  std::optional<visualization_msgs::Marker> generateMarkersForSurfaceNormalVectors(const open3d::geometry::PointCloud& o3d_pc,
                                                                                   const ros::Time& timestamp,
                                                                                   const o3d_slam::RgbaColorMap::Values& color);

  bool validateAnalysisTopic(const rosbag::Bag& bag, const std::vector<std::string>& mandatoryTopics);
  bool readCalibrationIfNeeded();

  std::tuple<ros::WallDuration, ros::WallDuration, ros::WallDuration> usePairForRegistration();

 private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  bool processRosbag();

  std::string rosbagFilename_;

  //! Parameters.
  Parameters parameters_;
  nav_msgs::Path trackedPath_;
  nav_msgs::Path bestGuessPath_;
  std::ofstream poseFile_;
  std::ofstream gnssFile_;
  std::ofstream imuFile_;
  std::string asyncOdometryFrame_;

  std::string buildUpLogFilename(const std::string& typeSuffix, const std::string& extension = ".txt");
  bool createOutputDirectory();
  visualization_msgs::MarkerArray convertPathToMarkerArray(const nav_msgs::Path& path);
  visualization_msgs::Marker createLineStripMarker();

  o3d_slam::PointCloud lineStripToPointCloud(const visualization_msgs::MarkerArray& marker_array, const int num_samples);

  void calculateSurfaceNormals(o3d_slam::PointCloud& cloud);

  void addToPathMsg(nav_msgs::PathPtr pathPtr, const std::string& frameName, const ros::Time& stamp, const Eigen::Vector3d& t,
                    const int maxBufferLength);

  geometry_msgs::Pose addUniformNoiseToPose(const geometry_msgs::Pose& original_pose, double position_noise_magnitude,
                                            double orientation_noise_magnitude);

  geometry_msgs::Pose addNoiseToPose_with_mean(const geometry_msgs::Pose& original_pose, double position_noise_magnitude,
                                               double orientation_noise_magnitude, double position_mean, double orientation_mean,
                                               bool use_zero_mean);

  geometry_msgs::Pose motionBasedNoise(const geometry_msgs::Pose poseMsg, double transNoise, double directionalTransNoise,
                                       double rotationNoise);
  geometry_msgs::Pose matrix_to_pose_stamped(const Eigen::Matrix4d& matrix);
  Eigen::Matrix4d pose_stamped_to_matrix(const geometry_msgs::Pose& pose_stamped);

  geometry_msgs::Point normalizeVector(const geometry_msgs::Point& vector);
  double vectorNorm(const geometry_msgs::Point& vector);
  void logToFiles();
  void generateLocalizabilityCategory();
  void generateSystemStatsFile();
  void drawDegeneracyArrows(const Transform currentPose, const ros::Time& stamp);
  visualization_msgs::Marker generateEigenVectorArrowMarkers(std::vector<float> mean, Eigen::VectorXf eigVector, int id, int colorId,
                                                             std::string ns, float scale, const ros::Time& stamp);

  //! Publishers.
  ros::Publisher clockPublisher_;
  ros::Publisher inputPointCloudPublisher_;
  ros::ServiceServer sleepServer_;
  sensor_msgs::PointCloud2 registeredCloud_;

  // GNSS Handler
  std::shared_ptr<o3d_slam::GnssHandler> gnssHandlerPtr_;
  nav_msgs::PathPtr measGnss_worldGnssPathPtr_;

  // Initialization
  bool gpsInitialized_{false};

  //! Tf2.
  tf2_ros::TransformBroadcaster transformBroadcaster_;
  tf2_ros::StaticTransformBroadcaster staticTransformBroadcaster_;

  std::deque<geometry_msgs::PoseStamped> registeredPoses_;

  ros::Time tracker;

  //! Maximum processing rate.
  double maxProcessingRate_{-0.1};

  //! Tf topic name.
  std::string tfTopic_{"/tf"};

  //! Static Tf topic name.
  std::string tfStaticTopic_{"/tf_static"};

  //! ROS clock topic name.
  std::string clockTopic_{"/clock"};

  bool isFirstMessage_ = true;
  bool isStaticTransformFound_ = false;

  ros::Duration timeDiff_;
  rosbag::Bag outBag;
  rosbag::Bag noisedoutBag;

  geometry_msgs::TransformStamped baseToLidarTransform_;
  geometry_msgs::TransformStamped gpsToLidarTransform_;
  std::string odometryHeader_{"/bestHeaderThereis"};
  std::vector<geometry_msgs::TransformStamped> staticTransforms_;
  o3d_slam::Transform mapEnuTransform_;

  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::unique_ptr<tf2_ros::TransformListener> tfListener_;

  bool isBagReadyToPlay_ = false;

  o3d_slam::RgbaColorMap colorMap_;
  std::string package_path_ = "";

  geometry_msgs::Pose prePoseStamped_;
  DeeperICPLogs deeperICPLogs_;

  std::ofstream file_localizability;
  std::ofstream file_stats;

  // RMS from Petracek et al.
  std::shared_ptr<rms::RMS> rms;
  std::vector<float> rmsTimeVector_;
  float rmsTiming_{0.0f};

  const std::unordered_map<int, std::vector<float>> localizabilityArrorColors_ = {
      {0, {1, 1, 0.2, 1}},             // Yellow.
      {1, {1, 0.501, 0, 1}},           // Orange.
      {2, {1, 1, 1, 1}},               // White.
      {3, {0, 1, 1, 1}},               // Cyan.
      {4, {0.4, 1, 0.4, 1}},           // Light green.
      {5, {0.635, 0.623, 0.164, 1}},   // Lavander.
      {6, {0.8, 1, 0.8, 1}},           // Green.
      {7, {0.976, 0.584, 0.937, 1}},   // pink
      {8, {0, 0, 0.3, 1}},             // Blue.
      {9, {0.941, 0.807, 0.639, 1}},   // Skin.
      {10, {0.188, 0.533, 0.003, 1}},  // Dark green.
      {11, {0.976, 0.862, 0.874, 1}},  // Light pink.
      {12, {0.705, 0.674, 0.678, 1}},  // Grey
      {13, {1, 0, 0, 1}},              // red
      {14, {0, 0, 0, 1}}               // Dark yellow.
  };
};

}  // namespace o3d_slam
