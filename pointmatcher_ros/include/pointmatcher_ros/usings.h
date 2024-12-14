#pragma once

// nabo
#include <nabo/nabo.h>

// pointmatcher
#include <pointmatcher/PointMatcher.h>

namespace pointmatcher_ros
{

// central definition of numeric type.
using NumericType = float;

// nabo
using NNS = Nabo::NearestNeighbourSearch<NumericType>;
using NNSearchType = NNS::SearchType;

// pointmatcher
using Pm = PointMatcher<NumericType>;
using PmDataPoints = Pm::DataPoints;
using PmDataPointsView = PmDataPoints::View;
using PmDataPointsConstView = PmDataPoints::ConstView;
using PmIcp = Pm::ICP;
using PmIndex = Pm::DataPoints::Index;
using PmPointCloudFilter = Pm::DataPointsFilter;
using PmPointCloudFilters = Pm::DataPointsFilters;
using PmTransformator = Pm::Transformation;
using PmTfParameters = Pm::TransformationParameters;
using PmMatrix = Pm::Matrix;
using PmMatches = Pm::Matches;
using PmOutlierWeights = Pm::OutlierWeights;

} // namespace pointmatcher_ros