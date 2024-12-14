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
#include "OctreeGrid.h"

#include "utils/octree/Octree.h"
#include "utils/octree/OctreeSamplers.h"

// OctreeGridDataPointsFilter
template <typename T>
OctreeGridDataPointsFilter<T>::OctreeGridDataPointsFilter(const Parameters& params) :
	PointMatcher<T>::DataPointsFilter("OctreeGridDataPointsFilter", 
		OctreeGridDataPointsFilter::availableParameters(), params),
	buildParallel{Parametrizable::get<bool>("buildParallel")},
	maxPointByNode{Parametrizable::get<std::size_t>("maxPointByNode")},
	maxSizeByNode{Parametrizable::get<T>("maxSizeByNode")},
	centerAtOrigin{Parametrizable::get<bool>("centerAtOrigin")}
{
	try 
	{
		const int sm = this->template get<int>("samplingMethod");
		samplingMethod = SamplingMethod(sm);
	}
	catch (const InvalidParameter& e) 
	{
		samplingMethod = SamplingMethod::FIRST_PTS;
	}
}

template <typename T>
typename PointMatcher<T>::DataPoints
OctreeGridDataPointsFilter<T>::filter(const DataPoints& input)
{
	DataPoints output(input);
	inPlaceFilter(output);
	return output;
}

template <typename T>
void OctreeGridDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
	// Compute the dimension of the point cloud features.
	const std::size_t featDim{static_cast<std::size_t>(cloud.features.rows())};
	
	assert(featDim == 4 or featDim == 3);

	// Dispatch to dimension-specific filters.
	if(featDim == featDimension2d) {
		// Quadtree, 3D case.
		this->sample<2>(cloud);
	}
	else if(featDim == featDimension3d) {
		// Octree, 3D case.
		this->sample<3>(cloud);
	}
}

template<typename T>
template<std::size_t dim>
void OctreeGridDataPointsFilter<T>::sample(DataPoints& cloud)
{
	Octree_<T,dim> octree;
	
	octree.build(cloud, maxPointByNode, maxSizeByNode, buildParallel, centerAtOrigin);
	
	// Dispatch 
	switch(samplingMethod)
	{
		case SamplingMethod::FIRST_PTS:
		{
			FirstPtsSampler<T> sampler(cloud);
			octree.visit(sampler);
			sampler.finalize();
			break;
		}
		case SamplingMethod::RAND_PTS:
		{
			RandomPtsSampler<T> sampler(cloud); //FIXME: add seed parameter
			octree.visit(sampler);
			sampler.finalize();
			break;
		}
		case SamplingMethod::CENTROID:
		{
			CentroidSampler<T> sampler(cloud);
			octree.visit(sampler);
			sampler.finalize();
			break;
		}
		case SamplingMethod::MEDOID:
		{
			MedoidSampler<T> sampler(cloud);
			octree.visit(sampler);
			sampler.finalize();
			break;
		}
	}
}


template struct OctreeGridDataPointsFilter<float>;
template struct OctreeGridDataPointsFilter<double>;
