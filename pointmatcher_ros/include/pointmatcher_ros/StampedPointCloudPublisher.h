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
struct StampedPointCloudPublisherParameters
{
    //! Point cloud publisher topic name.
    std::string pointCloudPublisherTopic_{ "/point_cloud" };

    //! Queue size for this publisher.
    int queueSizeOfPublishers_{ 0 };

    //! Whether the publishers of this point cloud should be latched.
    bool latchPublishers_{ false };

    //! Whether surface normals should be published.
    bool publishSurfaceNormals_{ false };

    //! Color for displaying point cloud markers.
    ColorKey markersColor_{ ColorKey::kYellow };

    NormalVectorsMarkerGenerationParameters normals_;
};

/**
 * @brief Stamped Point Cloud Publisher.
 * A point cloud publisher that publishes a point cloud and additional information from its descriptors (e.g. normal vectors, bounding box, etc)
 */
class StampedPointCloudPublisher
{
public:
    using Parameters = StampedPointCloudPublisherParameters;

    /**
     * @brief Construct a new Stamped Point Cloud Publisher object.
     * 
     */
    StampedPointCloudPublisher();

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
     * @brief Advertises publishers with their configuration provided directly as function argument.
     * 
     * @param nodeHandle      ROS node handle.
     * @param topicName       Topic name.
     * @param queueSize       Queue size.
     * @param latch           Whether the publisher should be latched.
     * @param publishNormals  Whether surface normal topics should be published (if available)
     */
    void advertiseFromTopicName(ros::NodeHandle nodeHandle, const std::string& topicName, int queueSize = 1, bool latch = false,
                                bool publishNormals = false, ColorKey color = ColorKey::kWhite);

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
     * @brief Publish all the topics for a given point cloud.
     * 
     * @param pointCloud    The point cloud to publish.
     * @param targetStamp   Timestamp for publishing the data. If targetStamp == ros::Time(0), the original timestamp is used.
     */
    void publish(const StampedPointCloud& pointCloud, const ros::Time targetStamp = ros::Time(0)) const;

private:
    /**
     * @brief Sets up the publisher.
     * 
     * @param nodeHandle  ROS node handle.
     */
    void setUpPublishers(ros::NodeHandle nodeHandle);

    /**
     * @brief Publishes a point cloud.
     * 
     * @param pointCloud  The point cloud to publish.
     * @param timestamp   Timestamp to publish the point cloud.
     */
    void publishPointCloud(const StampedPointCloud& pointCloud, const ros::Time& timestamp) const;

    /**
     * @brief Publishes the surface normals of a point cloud.
     * 
     * @param pointCloud  The point cloud that provides the surface normals.
     * @param timestamp   Timestamp to publish the point cloud.
     */
    void publishSurfaceNormals(const StampedPointCloud& pointCloud, const ros::Time& timestamp) const;

    //! Publisher parameters.
    Parameters parameters_;

    //! RGBA color map, used for visualization markers.
    RgbaColorMap colorMap_;

    //! ROS publishers.
    std::optional<ros::Publisher> pointCloudPublisher_;
    std::optional<ros::Publisher> normalsMarkersPublisher_;
};

} // namespace pointmatcher_ros