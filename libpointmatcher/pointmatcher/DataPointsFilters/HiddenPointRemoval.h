#pragma once

#include "PointMatcher.h"

template<typename T>
struct HiddenPointRemovalDataPointsFilter : public PointMatcher<T>::DataPointsFilter
{
  	// Type definitions
	typedef PointMatcher<T> PM;
	typedef typename PM::DataPoints DataPoints;
	typedef typename PM::DataPointsFilter DataPointsFilter;

	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef PointMatcherSupport::Parametrizable P;
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParameterDoc ParameterDoc;
	typedef Parametrizable::ParametersDoc ParametersDoc;
	typedef Parametrizable::InvalidParameter InvalidParameter;

	typedef typename PointMatcher<T>::Matrix Matrix;
	typedef typename PointMatcher<T>::Vector Vector;
	typedef typename Eigen::Matrix<T,2,1> Vector2;
	typedef typename Eigen::Matrix<T,3,1> Vector3;
	typedef typename PointMatcher<T>::DataPoints::InvalidField InvalidField;


	inline static const std::string description()
	{
		return "Remove hidden points from the point cloud based on visibility.";
	}

	inline static const ParametersDoc availableParameters()
	{
		return {
			{"vPositionX", "The X component of the position which the point cloud is visualized from. ", "0", "-inf", "inf", &P::Comp<T>},
			{"vPositionY", "The Y component of the position which the point cloud is visualized from. ", "0", "-inf", "inf", &P::Comp<T>},
			{"vPositionZ", "The Z component of the position which the point cloud is visualized from. ", "0", "-inf", "inf", &P::Comp<T>},
			{"radius", "The radius of the spherical projection. ", "100", "0", "inf", P::Comp<T>}
		};
	}

	const T vPositionX;
	const T vPositionY;
	const T vPositionZ;
	const T radius;

	//Constructor, uses parameter interface
    HiddenPointRemovalDataPointsFilter(const Parameters& params = Parameters());

    HiddenPointRemovalDataPointsFilter();
  // Destructor
	virtual ~HiddenPointRemovalDataPointsFilter() {};

	virtual DataPoints filter(const DataPoints& input);
	virtual void inPlaceFilter(DataPoints& cloud);
};
