#pragma once

// std
#include <vector>

// pointmatcher
#include <pointmatcher/IO.h>
#include <pointmatcher/PointMatcher.h>

// sensor_msgs
#include <sensor_msgs/PointCloud2.h>

namespace pointmatcher_ros
{

template<typename ScalarType>
class RosPointCloud2Deserializer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using PM = PointMatcher<ScalarType>;
    using DataPoints = typename PM::DataPoints;

    /**
     * @brief Deserializes a sensor_msgs/PointCloud2 objects into a Pointmatcher point cloud (DataPoints)
     * 
     * @param rosMsg        Input message of type sensor_msgs/PointCloud2.
     * @return DataPoints   Output point cloud.
     */
    static DataPoints deserialize(const sensor_msgs::PointCloud2& rosMsg);

private:
    using PMIO = PointMatcherIO<ScalarType>;
    using PM_types = typename PMIO::PMPropTypes;
    using Index = typename DataPoints::Index;
    using Label = typename DataPoints::Label;
    using Labels = typename DataPoints::Labels;
    using IndexMatrix = typename PM::IndexMatrix;
    using IndexGridView = typename DataPoints::IndexGridView;
    using View = typename DataPoints::View;
    using FieldNamesList = std::vector<std::string>;

    /**
     * @brief Extract the feature and descriptor labels of a given sensor_msgs/PointCloud2 message.
     * 
     * @param rosMsg        Input message of type sensor_msgs/PointCloud2.
     * @param featLabels    Feature labels. Assumed to be empty, will be filled by this method.
     * @param descLabels    Descriptor labels. Assumed to be empty, will be filled by this method.
     */
    static void extractFieldLabels(const sensor_msgs::PointCloud2& rosMsg, Labels& featLabels, Labels& descLabels);

    /**
     * @brief Fills data from a scalar descriptor into a Pointmatcher point cloud.
     * 
     * @param rosMsg        Input message of type sensor_msgs/PointCloud2.
     * @param fieldName     Name of the point cloud field that corresponds to the descriptor. 
     * @param pointCount    Number of points in the input point cloud.
     * @param view          View on the output point cloud, will be modified.
     */
    static void fillScalarDataIntoView(const sensor_msgs::PointCloud2& rosMsg, const std::string& fieldName, const size_t pointCount,
                                       View& view);

    /**
     * @brief Fills data from a vector feature or descriptor into a Pointmatcher point cloud.
     * 
     * @param rosMsg            Input message of type sensor_msgs/PointCloud2.
     * @param fieldName         Name of the point cloud field that corresponds to the descriptor. 
     * @param is3dPointCloud    Whether the point cloud is 3D or 2D.
     * @param pointCount        Number of points in the input point cloud.
     * @param view              View on the output point cloud, will be modified.
     */
    static void fillVectorDataIntoView(const sensor_msgs::PointCloud2& rosMsg, const FieldNamesList& fieldNames, const bool is3dPointCloud,
                                       const size_t pointCount, View& view);

    /**
     * @brief Fills data from a color descriptor into a Pointmatcher point cloud.
     * 
     * @param rosMsg            Input message of type sensor_msgs/PointCloud2.
     * @param fieldName         Name of the point cloud field that corresponds to the descriptor. 
     * @param pointCount        Number of points in the input point cloud.
     * @param view              View on the output point cloud, will be modified.
     */
    static void fillColorDataIntoView(const sensor_msgs::PointCloud2& rosMsg, const FieldNamesList& fieldNames, const size_t pointCount,
                                      View& view);

    /**
     * @brief Fills the 'index grid' from a ROS message into a Pointmatcher point cloud.
     * @remark This method expectes the ROS message to contain a non-dense (organized) point cloud.
     * 
     * @param rosMsg            Input message of type sensor_msgs/PointCloud2.
     * @param pointCount        Number of points in the input point cloud.
     * @param cloud             Point cloud. Its index grid will be modified
     */
    static void fillIndexGrid(const sensor_msgs::PointCloud2& rosMsg, const size_t pointCount, DataPoints& cloud);

    /**
     * @brief Fills a Pointmatcher point cloud with data from a sensor_msgs/PointCloud2 message.
     * 
     * @param rosMsg            Input message of type sensor_msgs/PointCloud2.
     * @param is3dPointCloud    Whether the point cloud is 3D or 2D.
     * @param pointCloud        View on the output point cloud, will be modified.
     */
    static void fillPointCloudValues(const sensor_msgs::PointCloud2& rosMsg, const bool is3dPointCloud, DataPoints& pointCloud);
};

} // namespace pointmatcher_ros