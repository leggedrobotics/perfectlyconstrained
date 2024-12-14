// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2018,
François Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
You can contact the authors at <f dot pomerleau at gmail dot com> and
<stephane at magnenat dot net>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#include "RemoveNaN.h"

// RemoveNaNDataPointsFilter
template<typename T>
typename PointMatcher<T>::DataPoints RemoveNaNDataPointsFilter<T>::filter(
	const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

// In-place filter
template<typename T>
void RemoveNaNDataPointsFilter<T>::inPlaceFilter(
	DataPoints& cloud)
{
	using ArrayBooleans = Eigen::Array<bool, 1, Eigen::Dynamic>;

	const int nbPointsIn = cloud.features.cols();

	// Given that this filter removes the structure from the linear array of points, there is no purpose for keeping the order of the 
	// point cloud in the index grid anymore.
	if(cloud.isOrganized()) {
		cloud.deallocateIndexGrid();
	}

	// Instantiate array of booleans to flag points as valid.
    // The following line implements a fast version of isNaN(), suggested by the author of Eigen: https://forum.kde.org/viewtopic.php?f=74&t=91514
    const ArrayBooleans isValidPoint{ (cloud.features.array() == cloud.features.array()).colwise().all() };

	int j = 0;
	for (int i = 0; i < nbPointsIn; ++i)
	{
		if (isValidPoint(i))
		{
			cloud.setColFrom(j, cloud, i);
			++j;
		}
	}

	cloud.conservativeResize(j);
}

template struct RemoveNaNDataPointsFilter<float>;
template struct RemoveNaNDataPointsFilter<double>;

