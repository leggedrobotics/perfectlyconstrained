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
#include "OrganizedCloudSurfaceNormal.h"

#include "PointMatcherPrivate.h"
#include "IO.h"
#include "utils/utils.h"

// OrganizedCloudSurfaceNormalDataPointsFilter
// Constructor
template<typename T>
OrganizedCloudSurfaceNormalDataPointsFilter<T>::OrganizedCloudSurfaceNormalDataPointsFilter(const Parameters& params) :
    PointMatcher<T>::DataPointsFilter("OrganizedCloudSurfaceNormalDataPointsFilter",
                                      OrganizedCloudSurfaceNormalDataPointsFilter::availableParameters(), params),
    knn(Parametrizable::get<int>("knn")),
    maxDist(Parametrizable::get<T>("maxDist"))
{
    maxDistSquared = maxDist * maxDist;
}

// Compute
template<typename T>
typename PointMatcher<T>::DataPoints OrganizedCloudSurfaceNormalDataPointsFilter<T>::filter(const DataPoints& input)
{
    DataPoints output(input);
    inPlaceFilter(output);
    return output;
}

// In-place filter
template<typename T>
void OrganizedCloudSurfaceNormalDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
    using namespace PointMatcherSupport;

    if (!cloud.isOrganized())
    {
        throw InvalidField("OrganizedCloudSurfaceNormalDataPointsFilter: Error, input point cloud is not organized.");
    }

    // Reserve memory for new descriptors
    const Index nbRows{ cloud.getHeight() };
    const Index nbCols{ cloud.getWidth() };
    const int featDim(cloud.features.rows() - 1);
    const int dimNormals{ featDim };
    constexpr int dimDensities(1);

    Labels cloudLabels;
    cloudLabels.emplace_back(Label("normals", dimNormals));
    cloudLabels.emplace_back(Label("densities", dimDensities));

    // Allocate descriptors.
    cloud.allocateDescriptors(cloudLabels);

    // Get views of descriptors.
    boost::optional<View> normals = cloud.getDescriptorViewByName("normals");
    boost::optional<View> densities = cloud.getDescriptorViewByName("densities");

    // Set descriptors to zero.
    normals->setZero();
    densities->setZero();

    // Compute neighbors of each point from index grid.
    const Index maxIndexOffset{ knn / 2 };
    Matrix selectedFeatures(featDim, knn * knn);
    Eigen::SelfAdjointEigenSolver<FixedSizeMatrix3> eigenSolver;
    for (Index col{ 0 }; col < nbCols; ++col)
    {
        for (Index row{ 0 }; row < nbRows; ++row)
        {
            if (cloud.indexGrid(row, col) == DataPoints::EmptyGridValue)
            {
                continue;
            }

            const GridIndex centerGridIndex{ row, col };
            processPatchAroundPoint(
                cloud, nbRows, nbCols, featDim, centerGridIndex, maxIndexOffset, eigenSolver, selectedFeatures, *normals, *densities);
        }
    }
}

template<typename DerivedA, typename DerivedB>
typename DerivedA::Scalar squaredEuclideanDistance(const Eigen::MatrixBase<DerivedA>& v0, const Eigen::MatrixBase<DerivedB>& v1)
{
    return (v0 - v1).squaredNorm();
}

template<typename T>
template<typename Derived>
bool OrganizedCloudSurfaceNormalDataPointsFilter<T>::processPatchAroundPoint(const DataPoints& cloud, const Index& nbRows,
                                                                             const Index& nbCols, const Index& featDim,
                                                                             const GridIndex& centerGridIndex, const Index& maxIndexOffset,
                                                                             Eigen::SelfAdjointEigenSolver<FixedSizeMatrix3>& eigenSolver,
                                                                             Eigen::MatrixBase<Derived>& selectedFeatures, View& normals,
                                                                             View& densities) const
{
    using namespace PointMatcherSupport;
    const Index centerLinearIndex{ cloud.computeLinearIndexFromGridIndex(centerGridIndex.row, centerGridIndex.col) };

    // Build up matrix of stacked points.
    Index validNeighborsCounter{ 0 };
    for (Index colOffset{ -maxIndexOffset }; colOffset < maxIndexOffset; ++colOffset)
    {
        for (Index rowOffset{ -maxIndexOffset }; rowOffset < maxIndexOffset; ++rowOffset)
        {
            // Compute coordinates of neighbor.
            const Index neighborGridIndexRow{ centerGridIndex.row + rowOffset };
            const Index neighborGridIndexCol{ centerGridIndex.col + colOffset };
            if (neighborGridIndexRow < 0 || neighborGridIndexCol < 0 || neighborGridIndexRow >= nbRows || neighborGridIndexCol >= nbCols)
            {
                // Skip point if it goes over the boundary of the index grid.
                continue;
            }

            if (cloud.indexGrid(neighborGridIndexRow, neighborGridIndexCol) == DataPoints::EmptyGridValue)
            {
                // Skip point if its empty.
                continue;
            }

            // Check that the distance between the center and the neighbor is less than a given threshold.
            const Index neighborLinearIndex{ cloud.computeLinearIndexFromGridIndex(neighborGridIndexRow, neighborGridIndexCol) };
            if (squaredEuclideanDistance(cloud.features.col(centerLinearIndex), cloud.features.col(neighborLinearIndex)) > maxDistSquared)
            {
                // Skip neighbor if the distance to the center is too big.
                continue;
            }

            // Add neighbor to list.
            selectedFeatures.col(validNeighborsCounter) = cloud.features.block(0, neighborLinearIndex, featDim, 1);
            ++validNeighborsCounter;
        }
    }

    // Check if valid neighbors could be found for the center point.
    if (validNeighborsCounter <= 1)
    {
        // Skip point if there were no valid neighbors found.
        return false;
    }

    // Compute mean and re-center collection of stacked points against the mean.
    const Vector mean{ selectedFeatures.leftCols(validNeighborsCounter).rowwise().sum() / T(validNeighborsCounter) };
    const Matrix NN{ selectedFeatures.leftCols(validNeighborsCounter).colwise() - mean };
    const FixedSizeMatrix3 covariance{ NN * NN.transpose() / static_cast<T>(nbCols) };

    // Compute eigenvalues and eigenvectors of the covariance matrix.
    eigenSolver.computeDirect(covariance);
    const Vector eigenVa{ eigenSolver.eigenvalues().real() };
    const Matrix eigenVe{ eigenSolver.eigenvectors().real() };

    // Compute a robust threshold for eigenvalue-based rank computation, inspired on the documentation of
    // the libeigen Eigenvalue solver we use, and on the implementation of the same concept for libeigen SVD solvers.
    const T premultipliedThreshold{ (static_cast<T>(featDim) * machineEpsilon) * eigenVa(2) };
    const Index rankOfC{ (eigenVa.array().abs() > premultipliedThreshold).count() };

    // Validate the rank of the covariance matrix.
    if (rankOfC + 1 < featDim)
    {
        // Skip point if the the covariance of its neighbors is rank defficient, as it can lead to degenerate normals.
        return false;
    }

    // Compute normal and clamp it to [-1,1] to handle approximation errors
    normals.col(centerLinearIndex) = computeNormalFromOrderedEigenvectors<T>(eigenVa, eigenVe).cwiseMax(-1.0).cwiseMin(1.0);
    densities.col(centerLinearIndex).setConstant(computeDensity<T>(NN));

    return true;
}

template struct OrganizedCloudSurfaceNormalDataPointsFilter<float>;
template struct OrganizedCloudSurfaceNormalDataPointsFilter<double>;
