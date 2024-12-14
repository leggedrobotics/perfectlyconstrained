/*
 * Mapper.cpp
 *
 *  Created on: Sep 26, 2021
 *      Author: jelavice
 */

#include "open3d_slam/Mapper.hpp"
#include "open3d_slam/ScanToMapRegistration.hpp"
#include "open3d_slam/Voxel.hpp"
#include "open3d_slam/assert.hpp"
#include "open3d_slam/helpers.hpp"
#include "open3d_slam/math.hpp"
#include "open3d_slam/output.hpp"
#include "open3d_slam/time.hpp"

#include "open3d_conversions/open3d_conversions.h"

#include <open3d/Open3D.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/utility/Eigen.h>
#include <open3d/utility/Helper.h>

#include <pointmatcher/PointMatcher.h>

namespace o3d_slam {

namespace {
namespace registration = open3d::pipelines::registration;
const bool isCheckTransformChainingAndPrintResult = false;
}  // namespace

Mapper::Mapper(const TransformInterpolationBuffer& odomToRangeSensorBuffer, std::shared_ptr<SubmapCollection> submaps)
    : odomToRangeSensorBuffer_(odomToRangeSensorBuffer), submaps_(submaps) {
  update(params_);
}

void Mapper::setParameters(const MapperParameters& p) {
  params_ = p;
  update(p);
}

MapperParameters* Mapper::getParametersPtr() {
  return &params_;
}

void Mapper::setExternalOdometryFrameToCloudFrameCalibration(const Eigen::Isometry3d& transform) {
  // Not thread safe?
  calibration_ = transform.matrix();
  isCalibrationSet_ = true;
  return;
}

bool Mapper::isExternalOdometryFrameToCloudFrameCalibrationSet() {
  return isCalibrationSet_;
}

void Mapper::loopClosureUpdate(const Transform& loopClosureCorrection) {
  mapToRangeSensor_ = loopClosureCorrection * mapToRangeSensor_;
  mapToRangeSensorPrev_ = loopClosureCorrection * mapToRangeSensorPrev_;
}

bool Mapper::hasProcessedMeasurements() const {
  return !mapToRangeSensorBuffer_.empty();
}

void Mapper::update(const MapperParameters& p) {
  scan2MapReg_ = scanToMapRegistrationFactory(p);
  submaps_->setParameters(p);
}

Transform Mapper::getMapToOdom(const Time& timestamp) const {
  const Transform odomToRangeSensor = getTransform(timestamp, odomToRangeSensorBuffer_);
  const Transform mapToRangeSensor = getTransform(timestamp, mapToRangeSensorBuffer_);
  return mapToRangeSensor * odomToRangeSensor.inverse();
  //	return getTransform(timestamp, mapToOdomBuffer_);
}

Transform Mapper::getMapToRangeSensor(const Time& timestamp) const {
  return getTransform(timestamp, mapToRangeSensorBuffer_);
}

bool Mapper::isRegistrationBestGuessBufferEmpty() const {
  return bestGuessBuffer_.empty();
}

Transform Mapper::getRegistrationBestGuess(const Time& timestamp) const {
  return getTransform(timestamp, bestGuessBuffer_);
}

const SubmapCollection& Mapper::getSubmaps() const {
  return *submaps_;
}

const Submap& Mapper::getActiveSubmap() const {
  return submaps_->getActiveSubmap();
}

const TransformInterpolationBuffer& Mapper::getMapToRangeSensorBuffer() const {
  return mapToRangeSensorBuffer_;
}

SubmapCollection* Mapper::getSubmapsPtr() {
  return submaps_.get();
}

void Mapper::setMapToRangeSensor(const Transform& t) {
  mapToRangeSensor_ = t;
}
void Mapper::setMapToRangeSensorInitial(const Transform& t) {
  if (isNewValueSetMapper_) {
    return;
  }

  std::cout << " Setting initial value transform at mapper: "
            << "\033[92m" << asString(t) << " \n"
            << "\033[0m";

  mapToRangeSensorPrev_ = t;
  mapToRangeSensor_ = t;
  isNewValueSetMapper_ = true;
  isInitialTransformSet_ = true;
}

const PointCloud& Mapper::getPreprocessedScan() const {
  return preProcessedScan_;
}

const ScanToMapRegistration& Mapper::getScanToMapRegistration() const {
  return *scan2MapReg_;
}

// Entry point to scan2map registration
bool Mapper::addRangeMeasurement(const Mapper::PointCloud& rawScan, const Time& timestamp) {
  if (!params_.isUseInitialMap_) {
    if (!isCalibrationSet_) {
      std::cerr << "Calibration is not set. Returning from mapping." << std::endl;
      return false;
    }
  }

  submaps_->setMapToRangeSensor(mapToRangeSensor_);

  // insert first scan
  if (submaps_->getActiveSubmap().isEmpty()) {
    if (params_.isUseInitialMap_) {
      assert_true(scan2MapReg_->isMergeScanValid(rawScan), "Init map invalid!!!!");
      submaps_->insertScan(rawScan, rawScan, mapToRangeSensor_, timestamp);

    } else {
      mapToRangeSensorPrev_ = mapToRangeSensor_;
      const ProcessedScans processed = scan2MapReg_->processForScanMatchingAndMerging(rawScan, mapToRangeSensor_);

      // Depending more on the initial transform.
      submaps_->insertScan(rawScan, *processed.merge_, mapToRangeSensor_, timestamp);
      mapToRangeSensorBuffer_.push(timestamp, mapToRangeSensor_);
      bestGuessBuffer_.push(timestamp, mapToRangeSensor_);
    }
    return true;
  }

  if (timestamp <= lastMeasurementTimestamp_) {
    std::cerr << "\n\n !!!!! MAPPER WARNING: Measurements came out of order!!!! \n\n";
    std::cerr << "Using the previously calculated odometry motion to propagate measurement. \n";

    int64_t uts_timestamp = toUniversal(timestamp);
    int64_t ns_since_unix_epoch = (uts_timestamp - kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll) * 100ll;
    std::cout << " timestamp: "
              << "\033[92m" << ns_since_unix_epoch << " \n"
              << "\033[0m";

    int64_t uts_timestamp_prev = toUniversal(lastMeasurementTimestamp_);
    int64_t ns_since_unix_epoch_prev = (uts_timestamp_prev - kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll) * 100ll;
    std::cout << " lastMeasurementTimestamp_: "
              << "\033[92m" << ns_since_unix_epoch_prev << " \n"
              << "\033[0m";

    int64_t diff_millisecond_since_unix_epoch = (ns_since_unix_epoch - ns_since_unix_epoch_prev) / 1000000ll;

    std::cout << " diff_millisecond_since_unix_epoch: "
              << "\033[92m" << diff_millisecond_since_unix_epoch << " \n"
              << "\033[0m";

    // Latest arbitrary time:
    Time arbitraryLatestTime = odomToRangeSensorBuffer_.latest_time();

    const Transform odomToRangeSensor = getTransform(arbitraryLatestTime, odomToRangeSensorBuffer_) * calibration_.inverse();
    const Transform odomToRangeSensorPrev = getTransform(lastMeasurementTimestamp_, odomToRangeSensorBuffer_) * calibration_.inverse();
    Transform odometryMotion = odomToRangeSensorPrev.inverse() * odomToRangeSensor;
    Transform backupTransform = mapToRangeSensorPrev_ * odometryMotion;

    // Pass to placeholders variables
    mapToRangeSensor_.matrix() = backupTransform.matrix();
    mapToRangeSensorBuffer_.push(arbitraryLatestTime, mapToRangeSensor_);
    bestGuessBuffer_.push(arbitraryLatestTime, mapToRangeSensorPrev_);
    submaps_->setMapToRangeSensor(mapToRangeSensor_);
    lastMeasurementTimestamp_ = arbitraryLatestTime;
    mapToRangeSensorPrev_ = mapToRangeSensor_;

    return true;
  }

  bool isOdomOkay = odomToRangeSensorBuffer_.has(timestamp);
  if (!isOdomOkay) {
    std::cerr << "WARNING: odomToRangeSensorBuffer_ DOES NOT HAVE THE DESIRED TRANSFORM! \n";
    std::cerr << "Going to attempt the scan to map refinement anyway. \n";
  }

  checkTransformChainingAndPrintResult(isCheckTransformChainingAndPrintResult);

  Transform mapToRangeSensorEstimate = mapToRangeSensorPrev_;

  // This is where we prepare the scan2map initial guess.
  if (isOdomOkay && !isNewValueSetMapper_ && !isIgnoreOdometryPrediction_) {
    // Get the pose at the current timestamp of the PC we are operating with.
    const Transform odomToRangeSensor = getTransform(timestamp, odomToRangeSensorBuffer_) * calibration_.inverse();

    // Get the pose at the previous PC timestamp we already calculated the scan2map refinement for.
    const Transform odomToRangeSensorPrev = getTransform(lastMeasurementTimestamp_, odomToRangeSensorBuffer_) * calibration_.inverse();

    // Calculate the motion between the two poses. This immediately allows us get rid of `drift` in the odometry.
    Transform odometryMotion = odomToRangeSensorPrev.inverse() * odomToRangeSensor;
    odometryMotionMemory_ = odometryMotion;

    // Apply the calculated odometry motion to the previous scan2map refined pose.
    mapToRangeSensorEstimate = mapToRangeSensorPrev_ * odometryMotion;

    if (odometryMotion.translation().norm() == 0.0) {
      std::cout << " Odometry translation shouldnt be 0. "
                << "\033[92m" << asString(odometryMotion) << " \n"
                << "\033[0m";
    }

    /*int64_t uts_timestamp = toUniversal(timestamp);
    int64_t ns_since_unix_epoch = (uts_timestamp - kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll) * 100ll;
    std::cout << " timestamp: "
              << "\033[92m" << ns_since_unix_epoch << " \n"
              << "\033[0m";

    int64_t uts_timestamp_prev = toUniversal(lastMeasurementTimestamp_);
    int64_t ns_since_unix_epoch_prev = (uts_timestamp_prev - kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll) * 100ll;
    std::cout << " lastMeasurementTimestamp_: "
              << "\033[92m" << ns_since_unix_epoch_prev << " \n"
              << "\033[0m";

    std::cout << " odomToRangeSensor: "
              << "\033[92m" << asString(odomToRangeSensor) << " \n"
              << "\033[0m";
    std::cout << " odomToRangeSensorPrev: "
              << "\033[92m" << asString(odomToRangeSensorPrev) << " \n"
              << "\033[0m";
    std::cout << " odometryMotion: "
              << "\033[92m" << asString(odometryMotion) << " \n"
              << "\033[0m";
    std::cout << " mapToRangeSensorEstimate: "
              << "\033[92m" << asString(mapToRangeSensorEstimate) << " \n"
              << "\033[0m";*/
  }

  isIgnoreOdometryPrediction_ = false;

  // Start auxilary time measurement.
  auxilaryTimer_.startStopwatch();

  const ProcessedScans processed = scan2MapReg_->processForScanMatchingAndMerging(rawScan, mapToRangeSensor_);
  auto croppedCloud = open3d_conversions::createSimilarPointmatcherCloud(processed.match_->points_.size());
  open3d_conversions::open3dToPointmatcher(*processed.match_, *croppedCloud);

  const double auxilarytimeElapsed = auxilaryTimer_.elapsedMsecSinceStopwatchStart();
  auxilaryTimer_.addMeasurementMsec(auxilarytimeElapsed);

  if (params_.isPrintTimingStatistics_) {
    std::cout << " Auxilary time: "
              << "\033[92m" << auxilarytimeElapsed << " msec \n"
              << "\033[0m";
  }

  // Convert the initial guess to pointmatcher
  pointmatcher_ros::PmTfParameters transformReadingToReferenceInitialGuess;
  transformReadingToReferenceInitialGuess.matrix() = mapToRangeSensorEstimate.matrix().cast<float>();

  // std::cout << "Number of points in submap: " << submaps_->getActiveSubmap().getNbPoints() << std::endl;

  // Crop the submap around the robot.
  const PointCloudPtr mapPatch = scan2MapReg_->cropSubmap(submaps_->getActiveSubmap(), mapToRangeSensor_);

  if (mapPatch->IsEmpty()) {
    std::cout << "\033[92m"
              << " Map patch is empty impossible. Returning"
              << " \n"
              << "\033[0m";
    return false;
  }

  // Initialize the registered pose.
  pointmatcher_ros::PmTfParameters correctedTransform = transformReadingToReferenceInitialGuess;

  try {
    double passedTime =
        std::chrono::duration_cast<std::chrono::milliseconds>(timestamp - lastReferenceInitializationTimestamp_).count() / 1e3;
    // std::cerr << "Passed Time: " << passedTime << std::endl;

    if (isNewValueSetMapper_ || (passedTime >= params_.scanMatcher_.icp_.referenceCloudSettingPeriod_)) {
      {
        std::lock_guard<std::mutex> lck(mapManipulationMutex_);
        // Create pointmatcher cloud
        open3d_conversions::PmStampedPointCloud activeSubmapPm_ =
            *open3d_conversions::createSimilarPointmatcherCloud(mapPatch->points_.size());

        // Convert to pointmatcher. This is time consuming.
        open3d_conversions::open3dToPointmatcher(*mapPatch, activeSubmapPm_);

        referenceInitTimer_.startStopwatch();

        lastReferenceInitializationTimestamp_ = timestamp;
        if (!icp_.initReference(activeSubmapPm_.dataPoints_)) {
          std::cout << "Failed to initialize reference cloud. Exitting. " << std::endl;
          return false;
        }
      }

      const double referenceInittimeElapsed = referenceInitTimer_.elapsedMsecSinceStopwatchStart();
      referenceInitTimer_.addMeasurementMsec(referenceInittimeElapsed);

      if (params_.isPrintTimingStatistics_) {
        std::cout << " Reference Cloud Re-init time: "
                  << "\033[92m" << referenceInittimeElapsed << " msec \n"
                  << "\033[0m";
      }
    } else {
      referenceInitTimer_.reset();
    }

    {
      std::lock_guard<std::mutex> lck(mapManipulationMutex_);

      // We are explicitly setting the reference cloud above. Hence the empty placeholder.
      open3d_conversions::PmStampedPointCloud emptyPlaceholder;

      correctedTransform =
          icp_.compute(croppedCloud->dataPoints_, emptyPlaceholder.dataPoints_, transformReadingToReferenceInitialGuess, false);
      deeperICPLogs_.degenerateDirections_ = icp_.getDegenerateDirections();
      deeperICPLogs_.totalICPtime = icp_.getTotalICPtime();
      deeperICPLogs_.localizationCategory = icp_.getLocalizationCategory();
      deeperICPLogs_.numberOfIterations = icp_.getTotalNumberOfIterations();

      if (icp_.getIterationMatches().size() > 0) {
        deeperICPLogs_.nbMatches_ = int(icp_.getIterationMatches().back());

      } else {
        deeperICPLogs_.nbMatches_ = 0;
      }

      if (icp_.errorMinimizer->getIterationWiseResidualErrors().size() > 0) {
        deeperICPLogs_.residualError_ = icp_.errorMinimizer->getIterationWiseResidualErrors().back();
      } else {
        deeperICPLogs_.residualError_ = -1;
      }

      deeperICPLogs_.time_ = timestamp;
    }

    if (firstRefinement_) {
      std::cout << "\033[92m"
                << " Open3d SLAM is running properly."
                << " \n "
                << "\033[0m";
      firstRefinement_ = false;
    }

    const double timeElapsed = testmapperOnlyTimer_.elapsedMsecSinceStopwatchStart();
    testmapperOnlyTimer_.addMeasurementMsec(timeElapsed);

    if (params_.isPrintTimingStatistics_) {
      std::cout << " Scan2Map Registration: "
                << "\033[92m" << timeElapsed << " msec \n "
                << "\033[0m";
    }

  } catch (const std::runtime_error& error) {
    std::cout << "Experienced a runtime error while running libpointmatcher ICP: " << error.what() << std::endl;
  }

  // Pass the calculated transform to o3d transform.
  Transform correctedTransform_o3d;
  correctedTransform_o3d.matrix() = correctedTransform.matrix().cast<double>();
  deeperICPLogs_.transform_ = mapToRangeSensorEstimate.matrix().inverse() * correctedTransform_o3d.matrix();

  if (isNewValueSetMapper_) {
    if (isTimeValid(timestamp)) {
      initTime_ = timestamp;
    }

    std::cout << "\033[92m"
              << "Setting initial value for the map to range sensor transform. ONLY expected at the start-up."
              << "\033[0m"
              << "\n";
    mapToRangeSensorPrev_ = mapToRangeSensor_;
    mapToRangeSensorBuffer_.push(timestamp, mapToRangeSensor_);
    bestGuessBuffer_.push(timestamp, mapToRangeSensorEstimate);
    isNewValueSetMapper_ = false;
    isIgnoreOdometryPrediction_ = true;
    return true;
  }

  // Pass to placeholders variables
  preProcessedScan_ = *processed.match_;
  mapToRangeSensor_.matrix() = correctedTransform_o3d.matrix();
  mapToRangeSensorBuffer_.push(timestamp, mapToRangeSensor_);
  bestGuessBuffer_.push(timestamp, mapToRangeSensorEstimate);
  submaps_->setMapToRangeSensor(mapToRangeSensor_);

  // Given a map, we don't want to add the mis-aligned first scans to the map. Hence giving the registration time to converge.
  double timeSinceInit = toSecondsSinceFirstMeasurement(timestamp) - toSecondsSinceFirstMeasurement(initTime_);
  if ((params_.isUseInitialMap_ && !params_.isMergeScansIntoMap_) ||
      (timeSinceInit < params_.mapMergeDelayInSeconds_ && params_.isUseInitialMap_ && params_.isMergeScansIntoMap_)) {
    // Early return before inserting the scans.
    lastMeasurementTimestamp_ = timestamp;
    mapToRangeSensorPrev_ = mapToRangeSensor_;

    if (params_.isPrintTimingStatistics_) {
      std::cout << "o3d_slam Mapper: "
                << "\033[92m Skipping Merging to Map. \n "
                << "\033[0m";
    }

    return true;
  }

  scanInsertionTimer_.startStopwatch();
  // Concatenate registered cloud into map
  const Transform sensorMotion = mapToRangeSensorLastScanInsertion_.inverse() * mapToRangeSensor_;
  const bool isMovedTooLittle = sensorMotion.translation().norm() < params_.minMovementBetweenMappingSteps_;
  if (!isMovedTooLittle) {
    // Timer t("scan_insertion_and_bookeeping");
    submaps_->insertScan(rawScan, *processed.merge_, mapToRangeSensor_, timestamp);
    mapToRangeSensorLastScanInsertion_ = mapToRangeSensor_;
  }

  lastMeasurementTimestamp_ = timestamp;
  mapToRangeSensorPrev_ = mapToRangeSensor_;

  const double insertiontimeElapsed = scanInsertionTimer_.elapsedMsecSinceStopwatchStart();
  scanInsertionTimer_.addMeasurementMsec(insertiontimeElapsed);

  if (params_.isPrintTimingStatistics_) {
    std::cout << "Scan Insertion: "
              << "\033[92m" << insertiontimeElapsed << " msec \n "
              << "\033[0m";
  }

  return true;
}

Mapper::PointCloud Mapper::getAssembledMapPointCloud() const {
  const int nPoints = submaps_->getTotalNumPoints();

  if (nPoints == 0) {
    std::cerr << "No points in the assembled map. Returning empty cloud." << std::endl;
    return PointCloud();
  }

  PointCloud cloud;
  const Submap& activeSubmap = getActiveSubmap();
  cloud.points_.reserve(nPoints);
  if (activeSubmap.getMapPointCloud().HasColors()) {
    cloud.colors_.reserve(nPoints);
  }
  if (activeSubmap.getMapPointCloud().HasNormals()) {
    cloud.normals_.reserve(nPoints);
  }

  for (size_t j = 0; j < submaps_->getNumSubmaps(); ++j) {
    const PointCloud submap = submaps_->getSubmap(j).getMapPointCloudCopy();
    for (size_t i = 0; i < submap.points_.size(); ++i) {
      cloud.points_.push_back(submap.points_.at(i));
      if (submap.HasColors()) {
        cloud.colors_.push_back(submap.colors_.at(i));
      }
      if (submap.HasNormals()) {
        cloud.normals_.push_back(submap.normals_.at(i));
      }
    }
  }

  return cloud;
}

void Mapper::checkTransformChainingAndPrintResult(bool isCheckTransformChainingAndPrintResult) const {
  if (isCheckTransformChainingAndPrintResult && odomToRangeSensorBuffer_.size() > 70 && mapToRangeSensorBuffer_.size() > 70) {
    /*const auto odom1 = odomToRangeSensorBuffer_.latest_measurement(60).transform_;
    const auto odom2 = odomToRangeSensorBuffer_.latest_measurement(20).transform_;
    const auto start = mapToRangeSensorBuffer_.latest_measurement(60).transform_;
    const auto gt = mapToRangeSensorBuffer_.latest_measurement(20).transform_;
    const Transform mapMotion = start.inverse() * gt;
    const Transform odomMotion = odom1.inverse() * odom2;
    std::cout << "start      :  " << asString(start) << "\n";
    std::cout << "gt         :  " << asString(gt) << "\n";
    std::cout << "gt computed:  " << asString(start * mapMotion) << "\n";
    std::cout << "est        : " << asString(start * odomMotion) << "\n\n";*/
  }
}

} /* namespace o3d_slam */
