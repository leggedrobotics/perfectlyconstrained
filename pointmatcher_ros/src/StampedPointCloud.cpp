
#include "pointmatcher_ros/StampedPointCloud.h"

// c++ standard library
#include <cmath>

// pcl
#include <pcl/io/ply_io.h>

// pcl conversions
#include <pcl_conversions/pcl_conversions.h>

// pointmatcher_ros
#include "pointmatcher_ros/serialization.h"
#include "pointmatcher_ros/RosPointCloud2Deserializer.h"

namespace pointmatcher_ros
{

StampedPointCloud::StampedPointCloud() :
    transformator_(std::shared_ptr<PmTransformator>(Pm::get().REG(Transformation).create("RigidTransformation")))
{
    clear();
}

StampedPointCloud StampedPointCloud::FromFile(const std::string& filePath, const ros::Time& stamp, const std::string& frameId)
{
    StampedPointCloud pointCloud;
    pointCloud.fromFile(filePath, stamp, frameId);
    return pointCloud;
}

bool StampedPointCloud::fromFile(const std::string& filePath, const ros::Time& stamp, const std::string& frameId)
{
    pcl::PLYReader reader;
    pcl::PointCloud<pcl::PointXYZINormal> pointCloudPcl;
    if (reader.read(filePath, pointCloudPcl) < 0)
    {
        return false;
    }
    pcl_conversions::toPCL(stamp, pointCloudPcl.header.stamp);
    pointCloudPcl.header.frame_id = frameId;

    // Erase points with NaN as curvature.
    const unsigned int size = pointCloudPcl.size();
    for (auto point = pointCloudPcl.end(); point >= pointCloudPcl.begin(); point--)
    {
        if (std::isnan(point->curvature))
        {
            ROS_DEBUG_STREAM("Removing point (" << std::distance(pointCloudPcl.begin(), point) << ") with curvature == NaN.");
            pointCloudPcl.erase(point);
        }
    }
    ROS_INFO_STREAM("Erased " << size - pointCloudPcl.size() << " invalid points from the point cloud.");

    sensor_msgs::PointCloud2 pointCloudRos;
    pcl::toROSMsg(pointCloudPcl, pointCloudRos);
    fromRosMsg(pointCloudRos);
    return true;
}

void StampedPointCloud::toFile(const std::string& filePath) const
{
    const sensor_msgs::PointCloud2 pointCloudRos = toRosMsg();
    pcl::PointCloud<pcl::PointXYZINormal> pointCloudPcl;
    pcl::fromROSMsg(pointCloudRos, pointCloudPcl);

    pcl::PLYWriter writer;
    writer.write(filePath, pointCloudPcl, false, false);
}

StampedPointCloud StampedPointCloud::FromRosMsg(const sensor_msgs::PointCloud2& msg)
{
    StampedPointCloud pointCloud;
    pointCloud.fromRosMsg(msg);
    return pointCloud;
}

void StampedPointCloud::fromRosMsg(const sensor_msgs::PointCloud2& msg)
{
    header_ = msg.header;
    dataPoints_ = RosPointCloud2Deserializer<float>::deserialize(msg);
}

sensor_msgs::PointCloud2 StampedPointCloud::toRosMsg(const ros::Time stamp) const
{
    sensor_msgs::PointCloud2 msg;
    toRosMsg(msg);

    // Check that the timestamp is well-defined.
    if (stamp != ros::Time(0))
    {
        msg.header.stamp = stamp;
    }

    return msg;
}

void StampedPointCloud::toRosMsg(sensor_msgs::PointCloud2& msg) const
{
    msg = pointmatcher_ros::pointMatcherCloudToRosMsg<float>(dataPoints_, header_.frame_id, header_.stamp);
}

StampedPointCloud StampedPointCloud::createSimilarEmpty() const
{
    StampedPointCloud similarEmtpy;
    similarEmtpy.header_ = header_;
    similarEmtpy.dataPoints_ = dataPoints_.createSimilarEmpty();
    return similarEmtpy;
}

bool StampedPointCloud::isEmpty() const
{
    return getSize() == 0;
}

unsigned int StampedPointCloud::getSize() const
{
    return dataPoints_.getNbPoints();
}

void StampedPointCloud::clear()
{
    header_ = std_msgs::Header();
    PmDataPoints::Labels featLabels;
    featLabels.push_back(PmDataPoints::Label("x", 1));
    featLabels.push_back(PmDataPoints::Label("y", 1));
    featLabels.push_back(PmDataPoints::Label("z", 1));
    featLabels.push_back(PmDataPoints::Label("pad", 1));
    PmDataPoints::Labels descLabels;
    const unsigned int pointCount = 0;
    dataPoints_ = PmDataPoints(featLabels, descLabels, pointCount);
}

bool StampedPointCloud::descriptorExists(const std::string& name) const
{
    return dataPoints_.descriptorExists(name);
}

void StampedPointCloud::addOneDimensionalDescriptor(const std::string& name, const float value)
{
    dataPoints_.addDescriptor(name, PmMatrix::Constant(1, getSize(), value));
}

PmDataPointsView StampedPointCloud::getDescriptorView(const std::string& name)
{
    // The following line can throw an exception if the descriptor does not exist.
    return dataPoints_.getDescriptorViewByName(name);
}

PmDataPointsConstView StampedPointCloud::getDescriptorConstView(const std::string& name) const
{
    // The following line can throw an exception if the descriptor does not exist.
    return dataPoints_.getDescriptorViewByName(name);
}

void StampedPointCloud::setDescriptorFromDescriptor(const std::string& sourceDescriptorName, const std::string& targetDescriptorName)
{
    dataPoints_.addDescriptor(sourceDescriptorName, dataPoints_.getDescriptorViewByName(targetDescriptorName));
}

bool StampedPointCloud::transform(const PmTf& tf)
{
    // Validate the transformation frames.
    if (tf.sourceFrameId_ != header_.frame_id)
    {
        ROS_ERROR_STREAM("Point cloud transformation failed due to inconsistent frames. "
                         << "Point cloud frame: '" << header_.frame_id << "', transformation source frame: '" << tf.sourceFrameId_ << "'.");
        return false;
    }
    header_.frame_id = tf.targetFrameId_;

    return transform(tf.parameters_);
}

bool StampedPointCloud::transform(const PmTfParameters& transform)
{
    try
    {
        transformator_->inPlaceCompute(transform, dataPoints_);
        return true;
    }
    catch (const std::exception& exception)
    {
        ROS_ERROR_STREAM("Caught exception while transforming point cloud: " << exception.what());
        return false;
    }
}

bool StampedPointCloud::filter(PmPointCloudFilter& filter)
{
    try
    {
        filter.inPlaceFilter(dataPoints_);
        return true;
    }
    catch (const std::exception& exception)
    {
        ROS_ERROR_STREAM("Caught exception while filtering point cloud: " << exception.what());
        return false;
    }
}

bool StampedPointCloud::filter(PmPointCloudFilters& filters)
{
    try
    {
        filters.apply(dataPoints_);
        return true;
    }
    catch (const std::exception& exception)
    {
        ROS_ERROR_STREAM("Caught exception while filtering point cloud: " << exception.what());
        return false;
    }
}

bool StampedPointCloud::filterByDistance(const float distanceThreshold, const bool keepInside)
{
    PmMatrix dummy;
    return filterByDistance(distanceThreshold, keepInside, dummy);
}

bool StampedPointCloud::filterByDistance(const float distanceThreshold, const bool keepInside, PmMatrix& newIdToOldId)
{
    const float distanceThresholdSquared = std::pow(distanceThreshold, 2);
    newIdToOldId = PmMatrix(1, getSize());
    unsigned int newId = 0;
    for (unsigned int oldId = 0; oldId < getSize(); oldId++)
    {
        if ((dataPoints_.features.col(oldId).head(3).squaredNorm() <= distanceThresholdSquared) == keepInside)
        {
            dataPoints_.setColFrom(newId, dataPoints_, oldId);
            newIdToOldId(0, newId) = oldId;
            newId++;
        }
    }
    dataPoints_.conservativeResize(newId);
    newIdToOldId.conservativeResize(Eigen::NoChange, newId);
    return true;
}

bool StampedPointCloud::filterByBoundingBox(const float xMin, const float xMax, const float yMin, const float yMax, const float zMin,
                                            const float zMax, const bool keepInside)
{
    auto boundingBoxFilter = Pm::get().DataPointsFilterRegistrar.create("BoundingBoxDataPointsFilter",
                                                                        { { "xMin", PointMatcherSupport::toParam(xMin) },
                                                                          { "xMax", PointMatcherSupport::toParam(xMax) },
                                                                          { "yMin", PointMatcherSupport::toParam(yMin) },
                                                                          { "yMax", PointMatcherSupport::toParam(yMax) },
                                                                          { "zMin", PointMatcherSupport::toParam(zMin) },
                                                                          { "zMax", PointMatcherSupport::toParam(zMax) },
                                                                          { "removeInside", PointMatcherSupport::toParam(!keepInside) } });
    const bool success = filter(*boundingBoxFilter);
    return success;
}

bool StampedPointCloud::filterByThresholding(const std::string& descriptorName, const unsigned int& descriptorDimension,
                                             const float threshold, const bool keepOverThreshold)
{
    StampedPointCloud pointsUnderThreshold;
    StampedPointCloud pointsOverThreshold;
    splitByThresholding(descriptorName, descriptorDimension, threshold, pointsUnderThreshold, pointsOverThreshold);
    if (keepOverThreshold)
    {
        *this = std::move(pointsOverThreshold);
    }
    else
    {
        *this = std::move(pointsUnderThreshold);
    }
    return true;
}

bool StampedPointCloud::add(const StampedPointCloud& other)
{
    if (other.header_.frame_id != header_.frame_id)
    {
        ROS_ERROR_STREAM("Point cloud concatenation failed due to inconsistent frames. "
                         << "This frame: '" << header_.frame_id << "', other frame: '" << other.header_.frame_id << "'.");
        return false;
    }

    if (other.isEmpty())
    {
        ROS_WARN_STREAM("Point cloud to add is empty, concatenation is not executed.");
    }
    else if (dataPoints_.features.rows() != other.dataPoints_.features.rows())
    {
        ROS_INFO_STREAM("Point clouds to concatenate have different features, overwriting them.");
        dataPoints_ = other.dataPoints_;
    }
    else
    {
        ROS_DEBUG_STREAM("Concatenating point clouds of sizes " << getSize() << " and " << other.getSize() << " ...");
        try
        {
            dataPoints_.concatenate(other.dataPoints_);
        }
        catch (const std::exception& exception)
        {
            ROS_ERROR_STREAM("Caught exception while concatenating point clouds: " << exception.what());
            return false;
        }
    }

    if (dataPoints_.isOrganized())
    {
        // deallocate index grid, as we assume that concatenation breaks point cloud ordering
        dataPoints_.deallocateIndexGrid();
    }

    //TODO(ynava) Introduce flag to decide whether the stamp should be overwritten. The behavior of this function is very non-explicit at the moment.
    // Update stamp for all of the above cases.
    header_.stamp = std::max(header_.stamp, other.header_.stamp);
    return true;
}

bool StampedPointCloud::addNonOverlappingPoints(const StampedPointCloud& other, const float maxDistOverlappingPoints, PmMatches& matches)
{
    // Split up the other point cloud in overlapping and non-overlapping points.
    StampedPointCloud otherOverlappingPoints;
    StampedPointCloud otherNonOverlappingPoints;
    if (!splitByOverlap(other, maxDistOverlappingPoints, matches, otherOverlappingPoints, otherNonOverlappingPoints))
    {
        ROS_ERROR_STREAM("Overlap could not be found.");
        return false;
    }

    // Only add the non-overlapping points.
    if (!add(otherNonOverlappingPoints))
    {
        ROS_ERROR_STREAM("Non-overlapping points could not be added.");
        return false;
    }

    return true;
}

bool StampedPointCloud::splitByOverlap(const StampedPointCloud& other, const float distanceThreshold, PmMatches matches,
                                       StampedPointCloud& otherOverlappingPoints, StampedPointCloud& otherNonOverlappingPoints) const
{
    if (other.header_.frame_id != header_.frame_id)
    {
        ROS_ERROR_STREAM("Point cloud finding overlap failed due to inconsistent frames. "
                         << "This frame: '" << header_.frame_id << "', other frame: '" << other.header_.frame_id << "'.");
        return false;
    }

    if (isEmpty())
    {
        // The other point cloud is completely non-overlapping.
        otherOverlappingPoints = other.createSimilarEmpty();
        otherNonOverlappingPoints = other;
    }
    else
    {
        otherOverlappingPoints = other.createSimilarEmpty();
        otherNonOverlappingPoints = other.createSimilarEmpty();

        if (matches.dists.cols() == 0u)
        {
            ROS_WARN("Matches container empty. NNS will be run");
            // Build and populate NNS.
            matches = PmMatches(PmMatches::Dists(1, other.getSize()), PmMatches::Ids(1, other.getSize()));
            std::shared_ptr<NNS> featureNNS(
                NNS::create(dataPoints_.features, dataPoints_.features.rows() - 1, NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));
            featureNNS->knn(other.dataPoints_.features, matches.ids, matches.dists, 1, 0);
        }

        const float distanceThresholdSquared = std::pow(distanceThreshold, 2);
        unsigned int otherOverlappingPointsSize = 0;
        unsigned int otherNonOverlappingPointsSize = 0;
        for (unsigned int i = 0; i < other.getSize(); ++i)
        {
            if (matches.dists(i) <= distanceThresholdSquared)
            {
                // Other point is near, considered as overlapping.
                otherOverlappingPoints.dataPoints_.setColFrom(otherOverlappingPointsSize, other.dataPoints_, i);
                otherOverlappingPointsSize++;
            }
            else
            {
                // Other point is far, considered as non-overlapping.
                otherNonOverlappingPoints.dataPoints_.setColFrom(otherNonOverlappingPointsSize, other.dataPoints_, i);
                otherNonOverlappingPointsSize++;
            }
        }

        otherOverlappingPoints.dataPoints_.conservativeResize(otherOverlappingPointsSize);
        otherNonOverlappingPoints.dataPoints_.conservativeResize(otherNonOverlappingPointsSize);
    }

    return true;
}

void StampedPointCloud::splitByThresholding(const std::string& descriptorName, const unsigned int& descriptorDimension,
                                            const float threshold, StampedPointCloud& pointsUnderThreshold,
                                            StampedPointCloud& pointsOverThreshold) const
{
    if (!descriptorExists(descriptorName))
    {
        // This can happen e.g. for empty maps.
        ROS_DEBUG_STREAM("The point cloud does not contain the descriptor '" << descriptorName << "'.");
        // Similar behavior as countStaticPoints(..):
        pointsOverThreshold = *this;
        pointsUnderThreshold = createSimilarEmpty();
        return;
    }

    const PmDataPointsConstView descriptorValues = getDescriptorConstView(descriptorName);
    pointsOverThreshold = createSimilarEmpty();
    pointsUnderThreshold = createSimilarEmpty();
    unsigned int pointsOverThresholdCount = 0;
    unsigned int pointsUnderThresholdCount = 0;
    for (unsigned int i = 0; i < getSize(); i++)
    {
        if (descriptorValues(descriptorDimension, i) >= threshold)
        {
            pointsOverThreshold.dataPoints_.setColFrom(pointsOverThresholdCount, dataPoints_, i);
            pointsOverThresholdCount++;
        }
        else
        {
            pointsUnderThreshold.dataPoints_.setColFrom(pointsUnderThresholdCount, dataPoints_, i);
            pointsUnderThresholdCount++;
        }
    }
    pointsOverThreshold.dataPoints_.conservativeResize(pointsOverThresholdCount);
    pointsUnderThreshold.dataPoints_.conservativeResize(pointsUnderThresholdCount);
}

unsigned int StampedPointCloud::countPointsOverThreshold(const std::string& descriptorName, const float threshold) const
{
    if (!descriptorExists(descriptorName))
    {
        // This can happen e.g. for empty maps.
        ROS_DEBUG_STREAM("The point cloud does not contain the descriptor '" << descriptorName << "'.");
        // Similar behavior as splitByThresholding(..):
        return getSize();
    }

    const PmDataPointsConstView descriptorValues = getDescriptorConstView(descriptorName);
    return (descriptorValues.array() >= threshold).count();
}

PmMatrix StampedPointCloud::toSphericalCoordinates() const
{
    // 0 = radius, 1 = azimuth, 2 = inclination.
    PmMatrix sphericalCoordinates = PmMatrix(3, getSize());

    // r = norm(z)
    sphericalCoordinates.row(0) = dataPoints_.features.topRows(3).colwise().norm();

    // azimuth = atan2(y,x)
    sphericalCoordinates.row(1) =
        dataPoints_.features.row(1).binaryExpr(dataPoints_.features.row(0), [](float y, float x) { return std::atan2(y, x); });

    // inclination = acos(z/r)
    sphericalCoordinates.row(2) =
        dataPoints_.features.row(2).binaryExpr(sphericalCoordinates.row(0), [](float z, float r) { return std::acos(z / r); });

    return sphericalCoordinates;
}

std::ostream& operator<<(std::ostream& ostream, const StampedPointCloud& pointCloud)
{
    ostream << "Frame: " << pointCloud.header_.frame_id << "\n";
    ostream << "Stamp: " << pointCloud.header_.stamp.toSec() << "\n";
    ostream << "Size: " << pointCloud.getSize() << "\n";
    ostream << "Is organized?: " << pointCloud.dataPoints_.isOrganized() << "\n";
    return ostream;
}

} // namespace pointmatcher_ros
