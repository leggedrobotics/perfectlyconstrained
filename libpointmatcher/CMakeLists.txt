cmake_minimum_required(VERSION 3.5.1)
project(libpointmatcher)

# Pointmatcher lib and install
set(POINTMATCHER_SRC
	pointmatcher/Logger.cpp
	pointmatcher/Exceptions.cpp
	pointmatcher/DataPoints.cpp
	pointmatcher/Matches.cpp
	pointmatcher/ICP.cpp
	pointmatcher/Registry.cpp
	pointmatcher/Registrar.cpp
	pointmatcher/DataPointsFilter.cpp
	pointmatcher/Matcher.cpp
	pointmatcher/OutlierFilter.cpp
	pointmatcher/ErrorMinimizer.cpp
	pointmatcher/Transformation.cpp
	pointmatcher/TransformationChecker.cpp
	pointmatcher/Inspector.cpp
	pointmatcher/IO.cpp
	pointmatcher/IOFunctions.cpp
	pointmatcher/Bibliography.cpp
	pointmatcher/Timer.cpp
	pointmatcher/Histogram.cpp
	pointmatcher/Parametrizable.cpp
	pointmatcher/LoggerImpl.cpp
	pointmatcher/MatchersImpl.cpp
	pointmatcher/OutlierFiltersImpl.cpp
	pointmatcher/TransformationsImpl.cpp
	pointmatcher/TransformationCheckersImpl.cpp
	pointmatcher/InspectorsImpl.cpp
	pointmatcher/point_cloud_registration.cpp
	pointmatcher/utils.cpp
#ErrorMinimizers
	pointmatcher/ErrorMinimizers/PointToPlane.cpp
	pointmatcher/ErrorMinimizers/PointToPlaneWithCov.cpp
	pointmatcher/ErrorMinimizers/PointToPoint.cpp
	pointmatcher/ErrorMinimizers/PointToPointWithCov.cpp
	pointmatcher/ErrorMinimizers/PointToPointSimilarity.cpp
	pointmatcher/ErrorMinimizers/Identity.cpp
#DataPointsFilters	
	pointmatcher/DataPointsFilters/Identity.cpp
	pointmatcher/DataPointsFilters/HiddenPointRemoval.cpp
	pointmatcher/DataPointsFilters/RemoveNaN.cpp
	pointmatcher/DataPointsFilters/MaxDist.cpp
	pointmatcher/DataPointsFilters/MinDist.cpp
	pointmatcher/DataPointsFilters/BoundingBox.cpp
	pointmatcher/DataPointsFilters/MaxQuantileOnAxis.cpp
	pointmatcher/DataPointsFilters/MaxDensity.cpp
	pointmatcher/DataPointsFilters/SurfaceNormal.cpp
	pointmatcher/DataPointsFilters/SamplingSurfaceNormal.cpp
	pointmatcher/DataPointsFilters/OrientNormals.cpp
	pointmatcher/DataPointsFilters/IncidenceAngle.cpp
	pointmatcher/DataPointsFilters/RandomSampling.cpp
	pointmatcher/DataPointsFilters/MaxPointCount.cpp
	pointmatcher/DataPointsFilters/FixStepSampling.cpp
	pointmatcher/DataPointsFilters/Shadow.cpp
	pointmatcher/DataPointsFilters/SimpleSensorNoise.cpp
	pointmatcher/DataPointsFilters/ObservationDirection.cpp
	pointmatcher/DataPointsFilters/VoxelGrid.cpp
	pointmatcher/DataPointsFilters/CutAtDescriptorThreshold.cpp
	pointmatcher/DataPointsFilters/Elipsoids.cpp
	pointmatcher/DataPointsFilters/Gestalt.cpp
	pointmatcher/DataPointsFilters/OctreeGrid.cpp
	pointmatcher/DataPointsFilters/NormalSpace.cpp
	pointmatcher/DataPointsFilters/CovarianceSampling.cpp
	pointmatcher/DataPointsFilters/DistanceLimit.cpp
	pointmatcher/DataPointsFilters/RemoveSensorBias.cpp
	pointmatcher/DataPointsFilters/OrganizedCloudSurfaceNormal.cpp
)

file(GLOB_RECURSE POINTMATCHER_HEADERS "pointmatcher/*.h")

# Build type options:
#	* catkin.
#	* cmake.
#	* ament (future).
# Regardless of the type chosen, the library 'libnabo' must be configured in the same way for the linker to suceed.
set(BUILD_TYPE "catkin")
if (BUILD_TYPE STREQUAL "catkin")
	include(CatkinBuild.cmake)
else()
	include(PureCMakeBuild.cmake)
endif()
