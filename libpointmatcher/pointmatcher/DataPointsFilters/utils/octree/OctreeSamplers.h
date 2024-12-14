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
#pragma once

#include "PointMatcher.h"

#include "Octree.h"

#include <vector>

template<typename T>
struct OctreeSampler
{
	public:
    typedef PointMatcher<T> PM;
    using DataPoints = typename PM::DataPoints;
};

template<typename T>
struct FirstPtsSampler : public OctreeSampler<T>
{
	using typename OctreeSampler<T>::DataPoints;

    std::size_t idx;
    DataPoints& pts;

    //Build a vector to map the old indexes to new indexes,
    // in case we sample pts at the begining of the pointcloud
    std::vector<std::size_t> indexVector;

    FirstPtsSampler(DataPoints& dp);
    virtual ~FirstPtsSampler() {}

    template<std::size_t dim>
    bool operator()(Octree_<T, dim>& oc);

    virtual bool finalize();
};

template<typename T>
struct RandomPtsSampler : public FirstPtsSampler<T>
{
	using typename FirstPtsSampler<T>::DataPoints;
    using FirstPtsSampler<T>::idx;
    using FirstPtsSampler<T>::indexVector;
    using FirstPtsSampler<T>::pts;

    const std::size_t seed;

    RandomPtsSampler(DataPoints& dp);
    RandomPtsSampler(DataPoints& dp, const std::size_t seed_);
    virtual ~RandomPtsSampler() {}

    template<std::size_t dim>
    bool operator()(Octree_<T, dim>& oc);

    virtual bool finalize();
};

template<typename T>
struct CentroidSampler : public FirstPtsSampler<T>
{
	using typename FirstPtsSampler<T>::DataPoints;
    using FirstPtsSampler<T>::idx;
    using FirstPtsSampler<T>::indexVector;
    using FirstPtsSampler<T>::pts;

    CentroidSampler(DataPoints& dp);

    virtual ~CentroidSampler() {}

    template<std::size_t dim>
    bool operator()(Octree_<T, dim>& oc);
};

//Nearest point from the centroid (contained in the cloud)
template<typename T>
struct MedoidSampler : public FirstPtsSampler<T>
{
	using typename FirstPtsSampler<T>::DataPoints;
    using FirstPtsSampler<T>::idx;
    using FirstPtsSampler<T>::indexVector;
    using FirstPtsSampler<T>::pts;

    MedoidSampler(DataPoints& dp);

    virtual ~MedoidSampler() {}

    template<std::size_t dim>
    bool operator()(Octree_<T, dim>& oc);
};

#include "OctreeSamplers.tpp"
