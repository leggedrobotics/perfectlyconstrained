
#include "pointmatcher_ros/IcpMatchesPublisher.h"

#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

#include <sensor_msgs/PointCloud2.h>

namespace pointmatcher_ros
{

IcpMatchesPublisher::IcpMatchesPublisher() = default;

void IcpMatchesPublisher::advertiseFromRosParameters(ros::NodeHandle nodeHandle, const std::string& parametersKey)
{
    if (!nodeHandle.getParam("publishers/" + parametersKey + "/reading_topic", parameters_.readingPointCloudPublisherTopic_))
    {
        ROS_ERROR("Topic name for point cloud publisher could not be fetched from the ROS Parameter Server.");
        return;
    }
    if (!nodeHandle.getParam("publishers/" + parametersKey + "/reference_topic", parameters_.referencePointCloudPublisherTopic_))
    {
        ROS_ERROR("Topic name for point cloud publisher could not be fetched from the ROS Parameter Server.");
        return;
    }
    parameters_.queueSizeOfPublishers_ = nodeHandle.param("publishers/" + parametersKey + "/queue_size", false);
    parameters_.latchPublishers_ = nodeHandle.param("publishers/" + parametersKey + "/latch", false);
    parameters_.publishSurfaceNormals_ = nodeHandle.param("publishers/" + parametersKey + "/surface_normals", false);
    parameters_.readingMarkersColor_ = static_cast<ColorKey>(
        nodeHandle.param("publishers/" + parametersKey + "/reading_surface_normals_color", static_cast<int>(ColorKey::kWhite)));
    parameters_.referenceMarkersColor_ = static_cast<ColorKey>(
        nodeHandle.param("publishers/" + parametersKey + "/reference_surface_normals_color", static_cast<int>(ColorKey::kWhite)));

    setUpPublishers(nodeHandle);
}

bool IcpMatchesPublisher::isLatched() const
{
    if (!parameters_.publishSurfaceNormals_)
    {
        return readingPointsMatchedPointCloudPublisher_->isLatched() || referencePointsMatchedPointCloudPublisher_->isLatched();
    }

    return readingPointsMatchedPointCloudPublisher_->isLatched() || readingPointsMatchedNormalMarkersPublisher_->isLatched()
        || referencePointsMatchedPointCloudPublisher_->isLatched() || referencePointsMatchedNormalMarkersPublisher_->isLatched();
}

size_t IcpMatchesPublisher::getNumSubscribers() const
{
    if (!parameters_.publishSurfaceNormals_)
    {
        return std::max(readingPointsMatchedPointCloudPublisher_->getNumSubscribers(),
                        referencePointsMatchedPointCloudPublisher_->getNumSubscribers());
    }

    return std::max(std::max(readingPointsMatchedPointCloudPublisher_->getNumSubscribers(),
                             readingPointsMatchedNormalMarkersPublisher_->getNumSubscribers()),
                    std::max(referencePointsMatchedPointCloudPublisher_->getNumSubscribers(),
                             referencePointsMatchedNormalMarkersPublisher_->getNumSubscribers()));
}

void IcpMatchesPublisher::processMatches(const StampedPointCloud& readingPointCloud,
                                         const StampedPointCloud& referencePointCloud,
                                         const PmMatches& matches,
                                         const PmOutlierWeights& outlierWeights)
{
    if (getNumSubscribers() == 0u && !isLatched())
    {
        ROS_DEBUG("Skipping extraction of matched points for visualization. No subscribers detected for topic '%s'",
                  parameters_.readingPointCloudPublisherTopic_.c_str());
        return;
    }

    // Build variables for direct access to data points member.
    const auto& readingPoints = readingPointCloud.dataPoints_;
    const auto& referencePoints = referencePointCloud.dataPoints_;
    const PmIndex numberOfReadingPoints{ readingPoints.features.cols() };
    const PmIndex numberOfMatchedPoints{ (outlierWeights.array() != 0.0).count() };

    // Initialize containers for matched points.
    auto& readingPointsMatched = readingPointsMatchedPointCloud_.dataPoints_;
    auto& referencePointsMatched = referencePointsMatchedPointCloud_.dataPoints_;
    readingPointsMatched = readingPoints.createSimilarEmpty(numberOfMatchedPoints);
    referencePointsMatched = referencePoints.createSimilarEmpty(numberOfMatchedPoints);

    // Count the number of nearest neighbors computed per point.
    const PmIndex knn{ outlierWeights.rows() };

    PmIndex j = 0;
    for (PmIndex i = 0; i < numberOfReadingPoints; ++i) // nb pts
    {
        for (PmIndex k = 0; k < knn; k++) // knn
        {
            if (matches.dists(k, i) == PmMatches::InvalidDist)
            {
                continue;
            }

            if (outlierWeights(k, i) == 0.0)
            {
                continue;
            }

            readingPointsMatched.features.col(j) = readingPoints.features.col(i);
            readingPointsMatched.descriptors.col(j) = readingPoints.descriptors.col(i);

            const PmIndex refIndex(matches.ids(k, i));
            referencePointsMatched.features.col(j) = referencePoints.features.col(refIndex);
            referencePointsMatched.descriptors.col(j) = referencePoints.descriptors.col(refIndex);
            ++j;
        }
    }

    readingPointsMatchedPointCloud_.header_ = readingPointCloud.header_;
    referencePointsMatchedPointCloud_.header_ = readingPointCloud.header_;
}

void IcpMatchesPublisher::publish() const
{
    if (readingPointsMatchedPointCloudPublisher_ == std::nullopt || referencePointsMatchedPointCloudPublisher_ == std::nullopt)
    {
        ROS_ERROR("Point cloud data cannot be published: Publisher '%s' has not been initialized properly.",
                  parameters_.readingPointCloudPublisherTopic_.c_str());
        return;
    }

    publishPointClouds();

    if (parameters_.publishSurfaceNormals_)
    {
        publishSurfaceNormals();
    }
}

void IcpMatchesPublisher::setUpPublishers(ros::NodeHandle nodeHandle)
{
    readingPointsMatchedPointCloudPublisher_ = nodeHandle.advertise<sensor_msgs::PointCloud2>(
        parameters_.readingPointCloudPublisherTopic_, parameters_.queueSizeOfPublishers_, parameters_.latchPublishers_);

    referencePointsMatchedPointCloudPublisher_ = nodeHandle.advertise<sensor_msgs::PointCloud2>(
        parameters_.referencePointCloudPublisherTopic_, parameters_.queueSizeOfPublishers_, parameters_.latchPublishers_);

    if (parameters_.publishSurfaceNormals_)
    {
        readingPointsMatchedNormalMarkersPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>(
            parameters_.readingPointCloudPublisherTopic_ + "_" + parameters_.normals_.pointCloudFieldId_,
            parameters_.queueSizeOfPublishers_,
            parameters_.latchPublishers_);
        referencePointsMatchedNormalMarkersPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>(
            parameters_.referencePointCloudPublisherTopic_ + "_" + parameters_.normals_.pointCloudFieldId_,
            parameters_.queueSizeOfPublishers_,
            parameters_.latchPublishers_);
    }
}

void IcpMatchesPublisher::publishPointClouds() const
{
    if (readingPointsMatchedPointCloudPublisher_->getNumSubscribers() > 0u || readingPointsMatchedPointCloudPublisher_->isLatched())
    {
        ROS_DEBUG("Publishing point cloud for publisher '%s'.", parameters_.readingPointCloudPublisherTopic_.c_str());
        readingPointsMatchedPointCloudPublisher_->publish(readingPointsMatchedPointCloud_.toRosMsg());
    }
    if (referencePointsMatchedPointCloudPublisher_->getNumSubscribers() > 0u || referencePointsMatchedPointCloudPublisher_->isLatched())
    {
        ROS_DEBUG("Publishing point cloud for publisher '%s'.", parameters_.referencePointCloudPublisherTopic_.c_str());
        referencePointsMatchedPointCloudPublisher_->publish(referencePointsMatchedPointCloud_.toRosMsg());
    }
}

void IcpMatchesPublisher::publishSurfaceNormals() const
{
    if (readingPointsMatchedNormalMarkersPublisher_->getNumSubscribers() > 0u || readingPointsMatchedNormalMarkersPublisher_->isLatched())
    {
        auto readingPointsMatchedNormalMarkers{ generateMarkersForSurfaceNormalVectors(readingPointsMatchedPointCloud_,
                                                                                       readingPointsMatchedPointCloud_.header_.stamp,
                                                                                       parameters_.normals_,
                                                                                       colorMap_[parameters_.readingMarkersColor_]) };
        if (readingPointsMatchedNormalMarkers != std::nullopt)
        {
            ROS_DEBUG("Publishing point cloud surface normals for publisher '%s'.", parameters_.readingPointCloudPublisherTopic_.c_str());
            readingPointsMatchedNormalMarkersPublisher_->publish(readingPointsMatchedNormalMarkers.value());
        }
    }

    if (referencePointsMatchedNormalMarkersPublisher_->getNumSubscribers() > 0u
        || referencePointsMatchedNormalMarkersPublisher_->isLatched())
    {
        auto referencePointsMatchedNormalMarkers{ generateMarkersForSurfaceNormalVectors(referencePointsMatchedPointCloud_,
                                                                                         referencePointsMatchedPointCloud_.header_.stamp,
                                                                                         parameters_.normals_,
                                                                                         colorMap_[parameters_.referenceMarkersColor_]) };
        if (referencePointsMatchedNormalMarkers != std::nullopt)
        {
            ROS_DEBUG("Publishing point cloud surface normals for publisher '%s'.", parameters_.referencePointCloudPublisherTopic_.c_str());
            referencePointsMatchedNormalMarkersPublisher_->publish(referencePointsMatchedNormalMarkers.value());
        }
    }
}


} // namespace pointmatcher_ros