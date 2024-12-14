// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2012,
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

#ifndef POINT_TO_PLANE_ERROR_MINIMIZER_H
#define POINT_TO_PLANE_ERROR_MINIMIZER_H

#include "PointMatcher.h"

template<typename T>
struct PointToPlaneErrorMinimizer: public PointMatcher<T>::ErrorMinimizer
{
    typedef PointMatcherSupport::Parametrizable Parametrizable;
    typedef PointMatcherSupport::Parametrizable P;
    typedef Parametrizable::Parameters Parameters;
    typedef Parametrizable::ParameterDoc ParameterDoc;
    typedef Parametrizable::ParametersDoc ParametersDoc;

    typedef typename PointMatcher<T>::DataPoints DataPoints;
    typedef typename PointMatcher<T>::Matches Matches;
    typedef typename PointMatcher<T>::OutlierWeights OutlierWeights;
    typedef typename PointMatcher<T>::ErrorMinimizer ErrorMinimizer;
    typedef typename PointMatcher<T>::ErrorMinimizer::ErrorElements ErrorElements;
    typedef typename PointMatcher<T>::TransformationParameters TransformationParameters;
    typedef typename PointMatcher<T>::Vector Vector;
    typedef typename PointMatcher<T>::Matrix Matrix;
    typedef typename PointMatcher<T>::LocalizabilityParametersForErrorMinimization LocalizabilityParametersForErrorMinimization;

	virtual inline const std::string name()
	{
		return "PointToPlaneErrorMinimizer";
	}

    inline static const std::string description()
    {
        return "Point-to-plane error (or point-to-line in 2D). Per \\cite{Chen1991Point2Plane}.";
    }

    inline static const ParametersDoc availableParameters()
    {
        return {
                {"force2D", "If set to true(1), the minimization will be forced to give a solution in 2D (i.e., on the XY-plane) even with 3D inputs.", "0", "0", "1", &P::Comp<bool>},
                {"force4DOF", "If set to true(1), the minimization will optimize only yaw and translation, pitch and roll will follow the prior.", "0", "0", "1", &P::Comp<bool>}

        };
    }

    const bool force2D;
    const bool force4DOF;

    PointToPlaneErrorMinimizer(const Parameters& params = Parameters());
    PointToPlaneErrorMinimizer(const ParametersDoc paramsDoc, const Parameters& params);
    //virtual TransformationParameters compute(const DataPoints& filteredReading, const DataPoints& filteredReference, const OutlierWeights& outlierWeights, const Matches& matches);
  virtual TransformationParameters compute(const ErrorElements& mPts, LocalizabilityParametersForErrorMinimization& localizabilityParametersForErrorMinimization);
  TransformationParameters compute_in_place(ErrorElements& mPts, LocalizabilityParametersForErrorMinimization& localizabilityParametersForErrorMinimization);
    virtual T getResidualError(const DataPoints& filteredReading, const DataPoints& filteredReference, const OutlierWeights& outlierWeights, const Matches& matches) const;
    virtual T getOverlap() const;

    static T computeResidualError(ErrorElements mPts, const bool& force2D);
};

// Main wrapper for inequality constrained optimization problem.
template<typename T, typename Matrix, typename Vector, typename LocalizabilityParametersForErrorMinimization>
void solvePossiblyUnderdeterminedLinearSystemWithInequalityConstraints(
    Vector& x, int& activeInequalityConstraints, int& numberOfEqualityConstraints,
    int& totalNumberOfConstraints, const Matrix& A, const Vector& b,
    LocalizabilityParametersForErrorMinimization& localizabilityParametersForErrorMinimization);

// Main wrapper for equality constrained optimization problem.
template<typename T, typename Matrix, typename Vector, typename LocalizabilityParametersForErrorMinimization>
void solvePossiblyUnderdeterminedLinearSystemWithEqualityConstraints(
    Vector& x, const Matrix& A, const Vector& b,
    LocalizabilityParametersForErrorMinimization& localizabilityParametersForErrorMinimization);

// Augmented the optimization problem to incoorporate degeneracy information.
template<typename T, typename Matrix, typename Vector, typename LocalizabilityParametersForErrorMinimization>
void generateConstrainedOptimizationProblem(Matrix& augmentedA, Vector& augmentedb, const std::vector<int>& degenerateDirectionIndices,
                                            LocalizabilityParametersForErrorMinimization& localizabilityParametersForErrorMinimization);

// Augmented the optimization problem to incoorporate degeneracy information. Overload for inequality constraints.
template<typename T, typename LocalizabilityParametersForErrorMinimization>
void generateConstrainedOptimizationProblem(Eigen::MatrixXd& constraintMatrix, Eigen::VectorXd& Alb, Eigen::VectorXd& Aub, int& numberOfEqualityConstraints,
                                            const std::vector<int>& degenerateDirectionIndices,
                                            LocalizabilityParametersForErrorMinimization& localizabilityParametersForErrorMinimization);

// Generate Regularized Least Squares problem.
template<typename T, typename Matrix, typename Vector, typename LocalizabilityParametersForErrorMinimization>
void generateRegularizedOptimizationProblem(Matrix& augmentedA,
                                            Vector& augmentedb,
                                            const std::vector<int>& degenerateDirectionIndices,
                                            LocalizabilityParametersForErrorMinimization& localizabilityParametersForErrorMinimization, const int& nbConstraints);


// Reads and assigns the degenerate indices for the matrix augmentation.
template<typename T, typename LocalizabilityParametersForErrorMinimization>
void readLocalizabilityFlags(std::vector<int>& degenerateIndices,
                             LocalizabilityParametersForErrorMinimization& localizabilityParametersForErrorMinimization);

// Based on the problem status solve a linear equation system.
template<typename T, typename MatrixA, typename Vector>
void solvePossiblyUnderdeterminedLinearSystem(const MatrixA& A, const Vector& b, Vector& x, const bool isConstrained);

// Eigen suggestion for pseudo inverse using SVD and not orthogonal decomposition.
template<typename T, typename Matrix>
Matrix pseudoInverse(const Matrix &a, double epsilon = std::numeric_limits<double>::epsilon())
{
  //using WorkingMatType = Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, MatType::MaxRowsAtCompileTime, MatType::MaxColsAtCompileTime>;
  Eigen::BDCSVD<Matrix> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
  svd.setThreshold(epsilon*std::max(a.cols(), a.rows()));
  Eigen::Index rank = svd.rank();
  //Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, MatType::RowsAtCompileTime,
  //              0, Eigen::BDCSVD<Matrix>::MaxDiagSizeAtCompileTime, MatType::MaxRowsAtCompileTime>
  Matrix tmp = svd.matrixU().leftCols(rank).adjoint();
  tmp = svd.singularValues().head(rank).asDiagonal().inverse() * tmp;
  return svd.matrixV().leftCols(rank) * tmp;
}

/*inline Mat3 ClosestSVDRotationMatrix(const Mat3 & rotMat)
{
  // Closest orthogonal matrix
  Eigen::JacobiSVD<Mat3> svd(rotMat,Eigen::ComputeFullV|Eigen::ComputeFullU);
  Mat3 U = svd.matrixU();
  Mat3 V = svd.matrixV();
  return U*V.transpose();
}
*/

#endif
