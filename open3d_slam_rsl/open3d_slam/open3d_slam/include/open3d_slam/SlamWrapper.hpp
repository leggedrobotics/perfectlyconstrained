/*
 * SlamWrapper.hpp
 *
 *  Created on: Nov 23, 2021
 *      Author: jelavice
 */

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <future>
#include <thread>
#include "open3d_slam/CircularBuffer.hpp"
#include "open3d_slam/Constraint.hpp"
#include "open3d_slam/Parameters.hpp"
#include "open3d_slam/Submap.hpp"
#include "open3d_slam/ThreadSafeBuffer.hpp"
#include "open3d_slam/TransformInterpolationBuffer.hpp"
#include "open3d_slam/croppers.hpp"
#include "open3d_slam/typedefs.hpp"

namespace o3d_slam {

class LidarOdometry;
class Mapper;
class SubmapCollection;
class OptimizationProblem;
class MotionCompensation;

class SlamWrapper {
  struct TimestampedPointCloud {
    Time time_;
    PointCloud cloud_;
  };

  struct RegisteredPointCloud {
    TimestampedPointCloud raw_;
    Transform transform_;
    std::string sourceFrame_, targetFrame_;
    size_t submapId_;
  };

  struct ScanToMapRegistrationBestGuess {
    Transform transform_;
    std::string sourceFrame_, targetFrame_;
    Time time_;
  };

  // Moved from frame.hpp to here, while this is not optimal allows the user to set frames through parameters.
  struct Frames {
    std::string odomFrame = "odom_o3d";
    std::string rangeSensorFrame = "default";  // This is the frame of the pointclud. Read from the pointcloud callback.
    std::string mapFrame = "map_o3d";          // Already Parametrized through lua loader.
    std::string imageFrame = "image_frame_o3d";
    std::string assumed_external_odometry_tracked_frame = "external_odometry_tracked_frame";
    std::string imuFrame = "default";  // Frame of the IMU. Read from the imu callback.
    std::string gpsFrame = "default";  // Frame of the GPS. Read from the gps navsatfix callback.
  };

 public:
  SlamWrapper();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual ~SlamWrapper();

  virtual bool addRangeScan(const open3d::geometry::PointCloud cloud, const Time timestamp);
  virtual void loadParametersAndInitialize();
  virtual void startWorkers();
  virtual void stopWorkers();
  virtual void finishProcessing();
  virtual void offlineFinishProcessing();

  // Virtual functions for the offlane single-thread workers.
  virtual void offlineTfWorker();
  virtual void offlineVisualizationWorker();

  const MapperParameters& getMapperParameters() const;
  MapperParameters* getMapperParametersPtr();
  size_t getOdometryBufferSize() const;
  size_t getMappingBufferSize() const;
  size_t getOdometryBufferSizeLimit() const;
  size_t getMappingBufferSizeLimit() const;
  std::string getParameterFilePath() const;
  std::tuple<PointCloud, Time, Transform> getLatestRegisteredCloudTimestampPair() const;
  std::tuple<Time, Transform> getLatestRegistrationBestGuess() const;
  void setExternalOdometryFrameToCloudFrameCalibration(const Eigen::Isometry3d& transform);

  // Add an odometry msg to the pose buffer.
  bool addOdometryPoseToBuffer(const Transform& transform, const Time& timestamp) const;

  // A getter for the IMU based initialization flag.
  bool isIMUattitudeInitializationEnabled();

  void setDirectoryPath(const std::string& path);
  void setMapSavingDirectoryPath(const std::string& path);
  void setParameterFilePath(const std::string& path);
  void setInitialMap(const PointCloud& initialMap);
  bool isInitialTransformSet();
  void setInitialTransform(const Eigen::Matrix4d initialTransform);
  bool isUseExistingMapEnabled() const;

  bool saveMap(const std::string& directory);
  void offlinePublishMaps(const Time& time);
  bool saveDenseSubmaps(const std::string& directory);
  bool saveSubmaps(const std::string& directory, const bool& isDenseMap = false);

  void appendPoseToTrackedPath(geometry_msgs::PoseStamped transform);
  void appendPoseToBestGuessPath(geometry_msgs::PoseStamped transform);

  // A simple worker call function for offlane replay package.
  void usePairForRegistration();

  // A simple worker call function for offlane replay package.
  void callofflineOdometryWorker();

  // A simple worker call function for offlane replay package.
  void callofflineMappingWorker();

  // A simple worker call function for offlane replay package.
  void callofflineLoopClosureWorker();

  void offlineDenseMapWorker();
  void callofflineDenseMapWorker();

  // A simple getter function that returns if the odometry buffer is empty.
  bool isOdometryPoseBufferEmpty();

  // A simple getter function that check if odometry pose is available for a certain time. Used for replaying.
  bool doesOdometrybufferHasMeasurement(const Time& t);

  // Returns a boolean value whether the static transformation between the odometry and range sensor is set.
  bool isExternalOdometryFrameToCloudFrameCalibrationSet();

  // Returns whether external odometry is used for prior.
  bool isUsingOdometryTopic() const;

  // Return the acquired the static transformation between the odometry and range sensor. If not available returns empty transform.
  Transform getExternalOdometryFrameToCloudFrameCalibration();
  TimestampedTransform getLatestMapToRangeMeasurement() const;
  TimestampedTransform getLatestOdometryPoseMeasurement() const;

  DeeperICPLogs offlineGetICPLogs();

  std::string mapSavingFolderPath_{""};
  TimestampedTransform latestMapToRangeMeasurement_;
  bool useGPSforGroundTruth_{false};
  bool downsamplePointCloudForReplay_{false};
  bool isRMSenabled_{false};
  float rmsLambda_ = -1.0f;
  float rmsVoxelSize_ = -1.0f;
  bool saveProcessedBag_{false};
  int downSamplingSkippingRate_{5};

  // If set to true, expects odometry msgs in the replayed rosbag to be exactly synced with the pointclouds.
  bool useSyncedPoses_ = false;
  bool rePublishTf_ = false;

  // The start and end time for the rosbag replay. In seconds.
  double bagReplayStartTime_ = 0.0;
  double bagReplayEndTime_ = 0.0;
  double relativeSleepDuration_ = 0.0;

  // The variable for asyncronized odometry pose msgs. Used for rosbag replay.
  std::string asyncOdometryTopic_{"/state_estimator/pose_in_odom"};

  // GPS topic name. Used for rosbag replay and live viz.
  std::string gpsTopic_{""};

  // Initialize the frames struct, accessible through the slamWrapper object.
  Frames frames_;

 private:
  void checkIfOptimizedGraphAvailable();
  void odometryWorker();
  void unifiedWorker();
  void unifiedWorkerOdom();
  void unifiedWorkerMap();

  void mappingWorker();
  void offlineOdometryWorker();
  void offlineMappingWorker();
  void offlineLoopClosureWorker();
  void loopClosureWorker();
  void computeFeaturesIfReady();
  void attemptLoopClosuresIfReady();
  void updateSubmapsAndTrajectory();
  void denseMapWorker();

 protected:
  // buffers
  CircularBuffer<RegisteredPointCloud> registeredCloudBuffer_;
  CircularBuffer<ScanToMapRegistrationBestGuess> registrationBestGuessBuffer_;

  CircularBuffer<TimestampedPointCloud> odometryBuffer_, mappingBuffer_;
  ThreadSafeBuffer<TimestampedSubmapId> loopClosureCandidates_;

  // parameters
  SlamParameters params_;
  //	MapperParameters mapperParams_;
  //	OdometryParameters odometryParams_;
  //	VisualizationParameters visualizationParameters_;
  //	SavingParameters savingParameters_;
  //	ConstantVelocityMotionCompensationParameters motionCompensationParameters_;
  std::string folderPath_;

  // modules
  std::shared_ptr<MotionCompensation> motionCompensationOdom_, motionCompensationMap_;
  std::shared_ptr<LidarOdometry> odometry_;
  std::shared_ptr<Mapper> mapper_;
  std::shared_ptr<SubmapCollection> submaps_;
  std::shared_ptr<OptimizationProblem> optimizationProblem_;

  // multithreading
  std::thread odometryWorker_, mappingWorker_, loopClosureWorker_, denseMapWorker_, unifiedWorker_, unifiedWorkerOdom_, unifiedWorkerMap_;
  std::future<void> computeFeaturesResult_;

  // timing
  Timer mappingStatisticsTimer_, odometryStatisticsTimer_, visualizationUpdateTimer_, denseMapVisualizationUpdateTimer_,
      denseMapStatiscticsTimer_;
  Timer mapperOnlyTimer_;
  Time latestScanToMapRefinementTimestamp_;
  Time latestScanToScanRegistrationTimestamp_;

  // bookkeeping
  bool isOptimizedGraphAvailable_ = false;
  bool isRunWorkers_ = true;
  int numLatesLoopClosureConstraints_ = -1;
  Constraints lastLoopClosureConstraints_;

  PointCloud woololo_prev;
  PointCloud woololo_curr;
};

}  // namespace o3d_slam
