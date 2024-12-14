#include "HiddenPointRemoval.h"


// HiddenPointRemovalDataPointsFilter
template <typename T>
HiddenPointRemovalDataPointsFilter<T>::HiddenPointRemovalDataPointsFilter() :
	vPositionX(0),
	vPositionY(0),
	vPositionZ(0),
	radius(100)
{
}

template <typename T>
HiddenPointRemovalDataPointsFilter<T>::HiddenPointRemovalDataPointsFilter(const Parameters& params) :
	PointMatcher<T>::DataPointsFilter("HiddenPointRemovalDataPointsFilter",
                                      HiddenPointRemovalDataPointsFilter::availableParameters(), params),
	vPositionX(Parametrizable::get<T>("vPositionX")),
	vPositionY(Parametrizable::get<T>("vPositionY")),
	vPositionZ(Parametrizable::get<T>("vPositionZ")),
    radius(Parametrizable::get<T>("radius"))

{
}

template <typename T>
typename PointMatcher<T>::DataPoints
HiddenPointRemovalDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

template <typename T>
void HiddenPointRemovalDataPointsFilter<T>::inPlaceFilter(DataPoints& /*cloud*/)
{
}

template struct HiddenPointRemovalDataPointsFilter<float>;
template struct HiddenPointRemovalDataPointsFilter<double>;


