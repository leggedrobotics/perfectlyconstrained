#pragma once

#include <pointmatcher_ros/PmTf.h>
#include <pointmatcher_ros/StampedPointCloud.h>

#include <string_view>

namespace open3d_conversions {

//! Point cloud descriptor name for normals.
constexpr std::string_view pointCloudDescriptorNameNormals{"normals"};

using PM = PointMatcher<float>;
using PmDataPoints = PM::DataPoints;

/* Geometry */
using Matrix6d = Eigen::Matrix<double, 6, 6>;
/* Point Clouds */
using PmIcp = pointmatcher_ros::PmIcp;
using PmMatches = pointmatcher_ros::PmMatches;
using PmMatrix = pointmatcher_ros::PmMatrix;
using PmStampedPointCloud = pointmatcher_ros::StampedPointCloud;
using PmTf = pointmatcher_ros::PmTf;
using PmTfParameters = pointmatcher_ros::PmTfParameters;
using PmPointCloudFilters = pointmatcher_ros::PmPointCloudFilters;
// using backendParameters = PointMatcher<float>::backendParameters;
typedef typename std::vector<Eigen::Matrix<float, -1, -1>, Eigen::aligned_allocator<Eigen::Matrix<float, -1, -1>>> vectorMat;

}  // namespace open3d_conversions