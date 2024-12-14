#pragma once

#include <optional>

#include <ros/ros.h>

#include <pointmatcher/PointMatcher.h>

#include "pointmatcher_ros/Colors.h"
#include "pointmatcher_ros/StampedPointCloud.h"
#include "pointmatcher_ros/visualization_utils.h"

namespace pointmatcher_ros
{

/**
 * @brief Parameters for the Point Cloud Publisher objects.
 * 
 */
struct IcpMatchesPublisherParameters
{
    //! Reading point cloud publisher topic name.
    std::string readingPointCloudPublisherTopic_{ "/reading_point_cloud" };

    //! Rereference point cloud publisher topic name.
    std::string referencePointCloudPublisherTopic_{ "/reference_point_cloud" };

    //! Queue size for this publisher.
    int queueSizeOfPublishers_{ 0 };

    //! Whether the publishers of this point cloud should be latched.
    bool latchPublishers_{ false };

    //! Whether surface normals should be published.
    bool publishSurfaceNormals_{ false };

    //! Color for displaying reading point cloud markers.
    ColorKey readingMarkersColor_{ ColorKey::kYellow };

    //! Color for displaying reference point cloud markers.
    ColorKey referenceMarkersColor_{ ColorKey::kBlue };

    NormalVectorsMarkerGenerationParameters normals_;
};

/**
 * @brief Stamped Point Cloud Publisher.
 * A point cloud publisher that publishes a point cloud and additional information from its descriptors (e.g. normal vectors, bounding box, etc)
 */
class IcpMatchesPublisher
{
public:
    using Parameters = IcpMatchesPublisherParameters;

    /**
     * @brief Construct a new Stamped Point Cloud Publisher object.
     * 
     */
    IcpMatchesPublisher();

    /**
     * @brief Advertises publishers with their configuration fetched from the ROS Parameter Server.
     * The configuration of the publishers is expected to follow the following format within the NodeHandle namespace:
     *>     input_point_cloud_matched:
     *>        latch: false
     *>        queue_size: 1
     *>        surface_normals: false
     *>        topic: input_point_cloud_matched
     * 
     * @param nodeHandle    ROS node handle.
     * @param parametersKey Key for the publisher parameters.
     */
    void advertiseFromRosParameters(ros::NodeHandle nodeHandle, const std::string& parametersKey);

    /**
     * @brief Whether the publisher is latched.
     * 
     * @return true   If latched, false otherwise.
     */
    bool isLatched() const;

    /**
     * @brief Get the Number of subscribers for this publisher.
     * 
     * @return size_t   Number of subscribers.
     */
    size_t getNumSubscribers() const;

    /**
     * @brief Process matches from the libpointmatcher's ICP algorithm, which can be published for visualization later on.
     * 
     * @param readingPointCloud     Reading point cloud used for registration.
     * @param referencePointCloud   Reference point cloud used for registration.
     * @param matches               Point matches computed by registration.
     * @param outlierWeights        Outlier weights computed by registration.
     */
    void processMatches(const StampedPointCloud& readingPointCloud, const StampedPointCloud& referencePointCloud,
                        const PmMatches& icpMatches, const PmOutlierWeights& icpOutlierWeights);

    /**
     * @brief Publish all the topics for a given point cloud.
     * 
     */
    void publish() const;

private:
    /**
     * @brief Sets up the publisher.
     * 
     * @param nodeHandle  ROS node handle.
     */
    void setUpPublishers(ros::NodeHandle nodeHandle);

    /**
     * @brief Publishes the matched points point clouds.
     * 
     */
    void publishPointClouds() const;

    /**
     * @brief Publishes the surface normals of the matched points.
     * 
     */
    void publishSurfaceNormals() const;

    //! Publisher parameters.
    Parameters parameters_;

    //! RGBA color map, used for visualization markers.
    RgbaColorMap colorMap_;

    //! ROS publishers.
    std::optional<ros::Publisher> readingPointsMatchedPointCloudPublisher_;
    std::optional<ros::Publisher> readingPointsMatchedNormalMarkersPublisher_;
    std::optional<ros::Publisher> referencePointsMatchedPointCloudPublisher_;
    std::optional<ros::Publisher> referencePointsMatchedNormalMarkersPublisher_;

    //! Reading points that were matched against the map.
    StampedPointCloud readingPointsMatchedPointCloud_;
    //! Reference/map points that were matched
    StampedPointCloud referencePointsMatchedPointCloud_;
};

} // namespace pointmatcher_ros