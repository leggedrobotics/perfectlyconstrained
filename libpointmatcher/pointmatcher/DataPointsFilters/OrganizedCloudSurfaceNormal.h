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

#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Eigenvalues>
#include <Eigen/QR>

//! Surface normals estimation. Find the normal for every point using eigen-decomposition of neighbour points
template<typename T>
struct OrganizedCloudSurfaceNormalDataPointsFilter : public PointMatcher<T>::DataPointsFilter
{
    typedef PointMatcherSupport::Parametrizable Parametrizable;
    typedef PointMatcherSupport::Parametrizable P;
    typedef Parametrizable::Parameters Parameters;
    typedef Parametrizable::ParameterDoc ParameterDoc;
    typedef Parametrizable::ParametersDoc ParametersDoc;
    typedef Parametrizable::InvalidParameter InvalidParameter;

    typedef typename PointMatcher<T>::Vector Vector;
    typedef typename PointMatcher<T>::Matrix Matrix;
    typedef typename PointMatcher<T>::DataPoints DataPoints;
    typedef typename PointMatcher<T>::DataPoints::View View;
    typedef typename PointMatcher<T>::DataPoints::InvalidField InvalidField;
    typedef typename PointMatcher<T>::DataPoints::Index Index;
    typedef typename PointMatcher<T>::DataPoints::GridIndex GridIndex;
    typedef typename PointMatcher<T>::DataPoints::Label Label;
    typedef typename PointMatcher<T>::DataPoints::Labels Labels;
    typedef typename Eigen::Matrix<T, 3, 3> FixedSizeMatrix3;

    inline static const std::string description()
    {
        return "This filter extracts the surface normal vector and density around each point by taking the eigenvector corresponding "
               "to the smallest eigenvalue of its nearest neighbors. Nearest neighbors are found using the DataPoints::IndexGrid "
               "structure. "
               "This filter only works with 3D points.\n\n"
               "Required descriptors: none.\n"
               "Produced descritors:  normals, densities.\n"
               "Altered descriptors:  none.\n"
               "Altered features:     none.\n";
    }
    inline static const ParametersDoc availableParameters()
    {
        return {
            { "knn", "number of nearest neighbors to consider, including the point itself", "5", "3", "2147483647", &P::Comp<unsigned> },
            { "maxDist", "maximum distance to consider for neighbors", "inf", "0", "inf", &P::Comp<T> }
        };
    }

    const unsigned knn;
    const T maxDist;
    T maxDistSquared;
    const T machineEpsilon{ std::numeric_limits<T>::epsilon() };

    OrganizedCloudSurfaceNormalDataPointsFilter(const Parameters& params = Parameters());
    virtual ~OrganizedCloudSurfaceNormalDataPointsFilter(){};
    virtual DataPoints filter(const DataPoints& input);
    virtual void inPlaceFilter(DataPoints& cloud);

    template<typename Derived>
    
    bool processPatchAroundPoint(const DataPoints& cloud, const Index& nbRows, const Index& nbCols, const Index& featDim,
                                 const GridIndex& centerGridIndex, const Index& maxIndexOffset,
                                 Eigen::SelfAdjointEigenSolver<FixedSizeMatrix3>& eigenSolver, Eigen::MatrixBase<Derived>& selectedFeatures,
                                 View& normals, View& densities) const;
};
