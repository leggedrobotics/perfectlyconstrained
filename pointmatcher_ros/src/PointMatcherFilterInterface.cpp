
#include "pointmatcher_ros/PointMatcherFilterInterface.h"

// C++ standard library
#include <exception>
#include <fstream>

namespace pointmatcher_ros
{

bool PointMatcherFilterInterface::readPipelineFile(const std::string& fileName)
{
    std::ifstream inFile(fileName.c_str());
    if (!inFile.good())
    {
        ROS_ERROR_STREAM("Couldn't open pipeline description file from \"" << fileName << "\"");
        return false;
    }

    filters_ = PointMatcher<float>::DataPointsFilters(inFile);
    return true;
}

PointMatcher<float>::DataPoints PointMatcherFilterInterface::process(const PointMatcher<float>::DataPoints& input)
{
    // Copy input point cloud.
    auto localInput = input;

    // Apply filters to point cloud.
    processInPlace(localInput);

    return localInput;
}

void PointMatcherFilterInterface::processInPlace(PointMatcher<float>::DataPoints& input)
{
    if (input.getNbPoints() == 0u)
    {
        // Point cloud is empty. Calling the filtering methods is unnecessary.
        return;
    }

    try
    {
        filters_.apply(input);
    }
    catch (const std::runtime_error& e)
    {
        ROS_WARN_STREAM_THROTTLE(10.0, "Caught exception: " << e.what() << ".  Point cloud unchanged. (Throttled: 10s)");
    }
}

} // namespace pointmatcher_ros