#pragma once

// ros
#include <ros/console.h>

// PointMatcher
#include <pointmatcher/PointMatcher.h>

namespace pointmatcher_ros
{

class PointMatcherFilterInterface
{
public:
    std::string getDataType() const { return dataType_; }
    void setDataType(std::string newValue) { dataType_ = newValue; }

    bool readPipelineFile(const std::string& fileName);

    PointMatcher<float>::DataPoints process(const PointMatcher<float>::DataPoints& input);

    void processInPlace(PointMatcher<float>::DataPoints& input);

private:
    std::string dataType_;
    PointMatcher<float>::DataPointsFilters filters_;
};

} // namespace pointmatcher_ros