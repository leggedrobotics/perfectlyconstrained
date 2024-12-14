#ifndef __POINTMATCHER_ROS_DESERIALIZATION_POINT_CLOUD_H
#define __POINTMATCHER_ROS_DESERIALIZATION_POINT_CLOUD_H

// pointmatcher
#include <pointmatcher/PointMatcher.h>

// ros
#include <ros/time.h>

// sensor_msgs
#include <sensor_msgs/PointCloud2.h>

namespace pointmatcher_ros
{
template<typename T>
typename PointMatcher<T>::DataPoints rosMsgToPointMatcherCloud(const sensor_msgs::PointCloud2& rosMsg, const bool isDense = false);
} // namespace pointmatcher_ros

#endif //__POINTMATCHER_ROS_DESERIALIZATION_POINT_CLOUD_H