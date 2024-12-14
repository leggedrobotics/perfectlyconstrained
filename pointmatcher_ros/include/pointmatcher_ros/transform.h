#ifndef __POINTMATCHER_ROS_TRANSFORM_H
#define __POINTMATCHER_ROS_TRANSFORM_H

// eigen
#include <Eigen/Eigen>

// nav_msgs
#include <nav_msgs/Odometry.h>

// tf2_ros
#include <tf2_ros/transform_listener.h>

// pointmatcher
#include <pointmatcher/PointMatcher.h>

// pointmatcher_ros
#include "pointmatcher_ros/StampedPointCloud.h"

namespace ros
{
struct Time;
}

namespace tf
{
struct Transform;
struct TransformListener;
struct StampedTransform;
} // namespace tf

namespace pointmatcher_ros
{
// tf to Eigen
template<typename T>
typename PointMatcher<T>::TransformationParameters transformListenerToEigenMatrix(const tf::TransformListener& listener,
                                                                                  const std::string& target, const std::string& source,
                                                                                  const ros::Time& stamp);

// Odom to Eigen
template<typename T>
typename PointMatcher<T>::TransformationParameters odomMsgToEigenMatrix(const nav_msgs::Odometry& odom);

// Pose to Eigen
template<typename T>
typename PointMatcher<T>::TransformationParameters poseMsgToEigenMatrix(const geometry_msgs::Pose& pose);

// Eigen to Odom
template<typename T>
nav_msgs::Odometry eigenMatrixToOdomMsg(const typename PointMatcher<T>::TransformationParameters& inTr, const std::string& frame_id,
                                        const ros::Time& stamp);

// Eigen to Pose
template<typename T>
geometry_msgs::Pose eigenMatrixToPoseMsg(const typename PointMatcher<T>::TransformationParameters& inTr);


// Eigen to Transform
template<typename T>
tf::Transform eigenMatrixToTransform(const typename PointMatcher<T>::TransformationParameters& inTr);

// Eigen to Stamped Transform
template<typename T>
tf::StampedTransform eigenMatrixToStampedTransform(const typename PointMatcher<T>::TransformationParameters& inTr,
                                                   const std::string& target, const std::string& source, const ros::Time& stamp);

// 2D / 3D transform
template<typename T>
typename PointMatcher<T>::TransformationParameters eigenMatrixToDim(const typename PointMatcher<T>::TransformationParameters& matrix,
                                                                    int dimp1);

/**
 * @brief Transforms a point cloud to a target frame, using a fixed frame as reference.
 *
 * @param fixedFrame[in]  Fixed frame, to be used as reference for the motion between the point cloud and the target timestamp.
 * @param targetFrame[in] Target frame.
 * @param targetStamp[in] Target timestamp.
 * @param pointCloud[in/out]  Point cloud to transform.
 * @return true       If successul, false otherwise.
 */
bool transformCloudToFrame(const std::string& fixedFrame, const std::string& targetFrame, const ros::Time& targetStamp,
                           const tf2_ros::Buffer& tfBuffer, StampedPointCloud& pointCloud, double waitTimeTfLookup = 0.1);


} // namespace pointmatcher_ros

#endif //__POINTMATCHER_ROS_TRANSFORM_H