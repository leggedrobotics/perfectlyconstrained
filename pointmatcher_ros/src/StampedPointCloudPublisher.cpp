
#include "pointmatcher_ros/StampedPointCloudPublisher.h"

#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

#include <sensor_msgs/PointCloud2.h>

namespace pointmatcher_ros
{

StampedPointCloudPublisher::StampedPointCloudPublisher() = default;

void StampedPointCloudPublisher::advertiseFromRosParameters(ros::NodeHandle nodeHandle, const std::string& parametersKey)
{
    if (!nodeHandle.getParam("publishers/" + parametersKey + "/topic", parameters_.pointCloudPublisherTopic_))
    {
        ROS_ERROR("Topic name for point cloud publisher could not be fetched from the ROS Parameter Server.");
        return;
    }
    parameters_.queueSizeOfPublishers_ = nodeHandle.param("publishers/" + parametersKey + "/queue_size", false);
    parameters_.latchPublishers_ = nodeHandle.param("publishers/" + parametersKey + "/latch", false);
    parameters_.publishSurfaceNormals_ = nodeHandle.param("publishers/" + parametersKey + "/surface_normals", false);
    parameters_.markersColor_ = static_cast<ColorKey>(
        nodeHandle.param("publishers/" + parametersKey + "/surface_normals_color", static_cast<int>(ColorKey::kWhite)));

    setUpPublishers(nodeHandle);
}

void StampedPointCloudPublisher::advertiseFromTopicName(ros::NodeHandle nodeHandle, const std::string& topicName, const int queueSize,
                                                        const bool latch, const bool publishSurfaceNormals, const ColorKey color)
{
    parameters_.pointCloudPublisherTopic_ = topicName;
    parameters_.queueSizeOfPublishers_ = queueSize;
    parameters_.latchPublishers_ = latch;
    parameters_.publishSurfaceNormals_ = publishSurfaceNormals;
    parameters_.markersColor_ = color;

    setUpPublishers(nodeHandle);
}

bool StampedPointCloudPublisher::isLatched() const
{
    if (!parameters_.publishSurfaceNormals_)
    {
        return pointCloudPublisher_->isLatched();
    }

    return pointCloudPublisher_->isLatched() || normalsMarkersPublisher_->isLatched();
}

size_t StampedPointCloudPublisher::getNumSubscribers() const
{
    if (!parameters_.publishSurfaceNormals_)
    {
        return pointCloudPublisher_->getNumSubscribers();
    }

    return std::max(pointCloudPublisher_->getNumSubscribers(), normalsMarkersPublisher_->getNumSubscribers());
}

void StampedPointCloudPublisher::publish(const StampedPointCloud& pointCloud, const ros::Time targetStamp) const
{
    if (pointCloudPublisher_ == std::nullopt)
    {
        ROS_ERROR("Point cloud data cannot be published: Publisher '%s' has not been initialized properly.",
                  parameters_.pointCloudPublisherTopic_.c_str());
        return;
    }

    // Overwrite timestamp if a specific one is provided.
    ros::Time timestamp{ pointCloud.header_.stamp };
    if (targetStamp != ros::Time(0))
    {
        timestamp = targetStamp;
    }

    publishPointCloud(pointCloud, timestamp);

    if (parameters_.publishSurfaceNormals_)
    {
        publishSurfaceNormals(pointCloud, timestamp);
    }
}

void StampedPointCloudPublisher::setUpPublishers(ros::NodeHandle nodeHandle)
{
    pointCloudPublisher_ = nodeHandle.advertise<sensor_msgs::PointCloud2>(
        parameters_.pointCloudPublisherTopic_, parameters_.queueSizeOfPublishers_, parameters_.latchPublishers_);

    if (parameters_.publishSurfaceNormals_)
    {
        normalsMarkersPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>(parameters_.pointCloudPublisherTopic_ + "_"
                                                                                        + parameters_.normals_.pointCloudFieldId_,
                                                                                    parameters_.queueSizeOfPublishers_,
                                                                                    parameters_.latchPublishers_);
    }
}

void StampedPointCloudPublisher::publishPointCloud(const StampedPointCloud& pointCloud, const ros::Time& timestamp) const
{
    if (pointCloudPublisher_->getNumSubscribers() == 0u && !pointCloudPublisher_->isLatched())
    {
        return;
    }
    ROS_DEBUG("Publishing point cloud for publisher '%s'.", parameters_.pointCloudPublisherTopic_.c_str());

    pointCloudPublisher_->publish(pointCloud.toRosMsg(timestamp));
}

void StampedPointCloudPublisher::publishSurfaceNormals(const StampedPointCloud& pointCloud, const ros::Time& timestamp) const
{
    if (normalsMarkersPublisher_->getNumSubscribers() > 0u || normalsMarkersPublisher_->isLatched())
    {
        auto normalMarkers{ generateMarkersForSurfaceNormalVectors(
            pointCloud, timestamp, parameters_.normals_, colorMap_[parameters_.markersColor_]) };
        if (normalMarkers != std::nullopt)
        {
            ROS_DEBUG("Publishing point cloud surface normals for publisher '%s'.", parameters_.pointCloudPublisherTopic_.c_str());
            normalsMarkersPublisher_->publish(normalMarkers.value());
        }
    }
}


} // namespace pointmatcher_ros