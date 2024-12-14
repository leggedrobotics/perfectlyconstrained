
#include "pointmatcher_ros/visualization_utils.h"

#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

namespace pointmatcher_ros
{

std::optional<visualization_msgs::Marker> generateMarkersForSurfaceNormalVectors(const StampedPointCloud& pointCloud,
                                                                                 const ros::Time& timestamp,
                                                                                 const NormalVectorsMarkerGenerationParameters& parameters,
                                                                                 const RgbaColorMap::Values& color)
{
    if (pointCloud.isEmpty())
    {
        return {};
    }
    if (!pointCloud.dataPoints_.descriptorExists(parameters.pointCloudFieldId_))
    {
        ROS_WARN("Point cloud has no normals descriptor. Publisher configuration or normal computation might be inconsistent.");
        return {};
    }

    /* Common values used for all visualizations */
    const std::size_t numberOfPoints{ pointCloud.getSize() };
    std_msgs::ColorRGBA colorMsg;
    colorMsg.r = color[0];
    colorMsg.g = color[1];
    colorMsg.b = color[2];
    colorMsg.a = color[3];

    /* Vectors */
    visualization_msgs::Marker vectorsMarker;
    vectorsMarker.header.stamp = timestamp;
    vectorsMarker.header.frame_id = pointCloud.header_.frame_id;
    vectorsMarker.ns = parameters.vectorsMarkerNamespace_;
    vectorsMarker.action = visualization_msgs::Marker::ADD;
    vectorsMarker.type = visualization_msgs::Marker::LINE_LIST;
    vectorsMarker.pose.orientation.w = 1.0;
    vectorsMarker.id = parameters.vectorsMarkerId_;
    vectorsMarker.scale.x = parameters.vectorsWidth_;
    vectorsMarker.color = colorMsg;
    vectorsMarker.points.resize(numberOfPoints * 2);

    const auto& surfaceNormalsView{ pointCloud.dataPoints_.getDescriptorViewByName(parameters.pointCloudFieldId_) };
    for (size_t i = 0; i < numberOfPoints; i += 2)
    {
        // The actual position of the point that the surface normal belongs to.
        vectorsMarker.points[i].x = pointCloud.dataPoints_.features(0, i);
        vectorsMarker.points[i].y = pointCloud.dataPoints_.features(1, i);
        vectorsMarker.points[i].z = pointCloud.dataPoints_.features(2, i);

        // End if arrow.
        vectorsMarker.points[i + 1].x = pointCloud.dataPoints_.features(0, i) + surfaceNormalsView(0, i) * parameters.vectorsScalingFactor_;
        vectorsMarker.points[i + 1].y = pointCloud.dataPoints_.features(1, i) + surfaceNormalsView(1, i) * parameters.vectorsScalingFactor_;
        vectorsMarker.points[i + 1].z = pointCloud.dataPoints_.features(2, i) + surfaceNormalsView(2, i) * parameters.vectorsScalingFactor_;
    }

    return vectorsMarker;
}


} // namespace pointmatcher_ros