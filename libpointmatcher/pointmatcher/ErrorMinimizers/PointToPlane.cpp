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

#include <iostream>

#include <Eigen/SVD>
#include <Eigen/Eigenvalues>

#include "ErrorMinimizersImpl.h"
#include "PointMatcherPrivate.h"
#include "Functions.h"
#include "Timer.h"

// qpmad InEqualityConstrained Opt. Module
#include <qpmad/solver.h>

// TSVD
#include "TSVD.hpp"

// message logger
#include <message_logger/message_logger.hpp>

using namespace Eigen;
using namespace std;

typedef PointMatcherSupport::Parametrizable Parametrizable;
typedef PointMatcherSupport::Parametrizable P;
typedef Parametrizable::Parameters Parameters;
typedef Parametrizable::ParameterDoc ParameterDoc;
typedef Parametrizable::ParametersDoc ParametersDoc;

template<typename T>
PointToPlaneErrorMinimizer<T>::PointToPlaneErrorMinimizer(const Parameters& params):
	ErrorMinimizer(name(), availableParameters(), params),
	force2D(Parametrizable::get<T>("force2D")),
    force4DOF(Parametrizable::get<T>("force4DOF"))
{
	if(force2D)
		{
			if (force4DOF)
			{
				throw PointMatcherSupport::ConfigurationError("Force 2D cannot be used together with force4DOF.");
			}
			else
			{
				LOG_INFO_STREAM("PointMatcher::PointToPlaneErrorMinimizer - minimization will be in 2D.");
			}
		}
	else if(force4DOF)
	{
	 	LOG_INFO_STREAM("PointMatcher::PointToPlaneErrorMinimizer - minimization will be in 4-DOF (yaw,x,y,z).");
	}
	else
	{
		LOG_INFO_STREAM("PointMatcher::PointToPlaneErrorMinimizer - minimization will be in 3D.");
	}
}

template<typename T>
PointToPlaneErrorMinimizer<T>::PointToPlaneErrorMinimizer(const ParametersDoc paramsDoc, const Parameters& params):
	ErrorMinimizer(name(), paramsDoc, params),
	force2D(Parametrizable::get<T>("force2D")),
	force4DOF(Parametrizable::get<T>("force4DOF"))
{
	if(force2D)
	{
		if (force4DOF)
		{
			throw PointMatcherSupport::ConfigurationError("Force 2D cannot be used together with force4DOF.");
		}
		else
		{
			LOG_INFO_STREAM("PointMatcher::PointToPlaneErrorMinimizer - minimization will be in 2D.");
		}
	}
	else if(force4DOF)
	{
		LOG_INFO_STREAM("PointMatcher::PointToPlaneErrorMinimizer - minimization will be in 4-DOF (yaw,x,y,z).");
	}
	else
	{
		LOG_INFO_STREAM("PointMatcher::PointToPlaneErrorMinimizer - minimization will be in 3D.");
	}
}


template<typename T, typename MatrixA, typename Vector>
void solvePossiblyUnderdeterminedLinearSystem(const MatrixA& A, const Vector& b, Vector& x, const bool isConstrained)
{
    assert(A.cols() == A.rows());
    assert(b.cols() == 1);
    assert(b.rows() == A.rows());
    assert(x.cols() == 1);
    assert(x.rows() == A.cols());

    typedef typename PointMatcher<T>::Matrix Matrix;
    
    if (isConstrained)
    {
        BOOST_AUTO(solverQR, A.householderQr());
        x = solverQR.solve(b);
        //x = A.template cast<double>().jacobiSvd(ComputeThinU | ComputeThinV).solve(b.template cast<double>()).template cast<T>();
    } else{
        BOOST_AUTO(solverQR, A.householderQr());
        x = solverQR.solve(b);
    }
}

template<typename T>
typename PointMatcher<T>::TransformationParameters PointToPlaneErrorMinimizer<T>::compute(const ErrorElements& mPts_const, LocalizabilityParametersForErrorMinimization& localizabilityParametersForErrorMinimization)
{
		ErrorElements mPts = mPts_const;
		return compute_in_place(mPts, localizabilityParametersForErrorMinimization);
}

template<typename T>
typename PointMatcher<T>::TransformationParameters PointToPlaneErrorMinimizer<T>::compute_in_place(ErrorElements& mPts, LocalizabilityParametersForErrorMinimization& localizabilityParametersForErrorMinimization)
{
    const Eigen::Index dim{ mPts.reading.features.rows() };
    assert(dim == 4);
    assert(mPts.reading.features.cols() > 0);

    const Vector meanReading{mPts.reading.features.topRows(dim-1).rowwise().mean()};
    mPts.reading.features.topRows(dim-1).colwise() -= meanReading;
    const Vector meanReference{mPts.reference.features.topRows(dim-1).rowwise().mean()};
    mPts.reference.features.topRows(dim-1).colwise() -= meanReference;

    // Get the reference to the optimization matrices.
    const Matrix& A = localizabilityParametersForErrorMinimization.A_;
    const Vector& b = localizabilityParametersForErrorMinimization.b_;

    // Initialize the optimization output. This variable should live through all operations and passed to the minimizer. Thats why we define it here.
    Vector x(A.rows());

    // Select between different solvers.
    switch (localizabilityParametersForErrorMinimization.degeneracyAwarenessMethod)
    {
        case PointMatcher<T>::DegeneracyAwarenessMethod::kNone: {
            // Regular vanilla ICP.
            solvePossiblyUnderdeterminedLinearSystem<T>(A, b, x, false);
        }
        break;
        case PointMatcher<T>::DegeneracyAwarenessMethod::kSolutionRemapping: {

            const int numberOfConstraints = A.rows()
                    - (localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityXyz_.sum()
                    + localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityRpy_.sum());
            if (localizabilityParametersForErrorMinimization.useTruncatedSVD)
            {
                // TSVD
                if (numberOfConstraints == 0)
                {
                    // Regular vanilla ICP.
                    solvePossiblyUnderdeterminedLinearSystem<T>(A, b, x, false);
                }else
                {
                    // TSVD
                    TSVD<Matrix> tsvd;
                    tsvd.computeFromExistingSinv(A, localizabilityParametersForErrorMinimization.sinv_external_);
                    tsvd.solve(b, x);
                    localizabilityParametersForErrorMinimization.constraintResidual_ = (A*x - b).tail(numberOfConstraints).norm();
                }

            }else{
                
                // Zhang et al. (Solution Remapping)
                solvePossiblyUnderdeterminedLinearSystem<T>(A, b, x, false);

                // Project the solution based on the detection results.
                // TODO Do we really need the copy here?
                const Eigen::Matrix<T, 6, 1> optVariablesCopied{ x };
                x = localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.solutionRemappingProjectionMatrix_ * optVariablesCopied;
                localizabilityParametersForErrorMinimization.constraintResidual_ = (A*x - b).tail(numberOfConstraints).norm();
            }

        }
        break;
        case PointMatcher<T>::DegeneracyAwarenessMethod::kOptimizedEqualityConstraints:
        case PointMatcher<T>::DegeneracyAwarenessMethod::kEqualityConstraints: {

            // Do equality constrained optimization.
            solvePossiblyUnderdeterminedLinearSystemWithEqualityConstraints<T>(x, A, b, localizabilityParametersForErrorMinimization);
        }
        break;
        case PointMatcher<T>::DegeneracyAwarenessMethod::kInequalityConstraints: {

            // Do inequality constrained optimization with quadratic programming.
            int activeInequalityConstraints{-1};
            int numberOfEqualityConstraints{-1};
            int totalNumberOfConstraints{-1};

            // Do inequality constrained optimization with quadratic programming.
            solvePossiblyUnderdeterminedLinearSystemWithInequalityConstraints<T>(x, activeInequalityConstraints, numberOfEqualityConstraints, totalNumberOfConstraints, A, b, localizabilityParametersForErrorMinimization);

            PointToPlaneErrorMinimizer::setActiveInequalityConstraintSize(activeInequalityConstraints);
            PointToPlaneErrorMinimizer::setTotalNumberOfConstraintSize(totalNumberOfConstraints);
            PointToPlaneErrorMinimizer::setNumberOfEqualityConstraintSize(numberOfEqualityConstraints);

        }
        break;
        default: {
            // Default fallback method is to do regular vanilla ICP.
            solvePossiblyUnderdeterminedLinearSystem<T>(A, b, x, false);
        }
    }

    // Transform parameters to matrix
    Matrix mOut;
    if (dim == 4 && !force2D)
    {
        Eigen::Transform<T, 3, Eigen::Affine> transform{Eigen::Transform<T, 3, Eigen::Affine>::Identity()};
        // PLEASE DONT USE EULAR ANGLES!!!!
        // Rotation in Eular angles follow roll-pitch-yaw (1-2-3) rule
        /*transform = Eigen::AngleAxis<T>(x(0), Eigen::Matrix<T,1,3>::UnitX())
				* Eigen::AngleAxis<T>(x(1), Eigen::Matrix<T,1,3>::UnitY())
				* Eigen::AngleAxis<T>(x(2), Eigen::Matrix<T,1,3>::UnitZ());*/

        // Normal 6DOF takes the whole rotation vector from the solution to construct the output quaternion
        if (!force4DOF)
        {
            
            const T rotationAngle{std::atan(x.head(3).norm())};
            transform = Eigen::AngleAxis<T>(rotationAngle, x.head(3).stableNormalized());
            //transform = Eigen::AngleAxis<T>(x.head(3).norm(), x.head(3).normalized()); //x=[alpha,beta,gamma,x,y,z]
        }
        else // 4DOF needs only one number, the rotation around the Z axis
        {
            Vector unitZ(3, 1);
            unitZ << 0, 0, 1;
            transform = Eigen::AngleAxis<T>(x(0), unitZ); //x=[gamma,x,y,z]
        }

        // Reverse roll-pitch-yaw conversion, very useful piece of knowledge, keep it with you all time!
        /*const T pitch = -asin(transform(2,0));
				const T roll = atan2(transform(2,1), transform(2,2));
				const T yaw = atan2(transform(1,0) / cos(pitch), transform(0,0) / cos(pitch));
				std::cerr << "d angles" << x(0) - roll << ", " << x(1) - pitch << "," << x(2) - yaw << std::endl;*/
        if (!force4DOF)
        {
            transform.translation() = x.segment(3, 3); //x=[alpha,beta,gamma,x,y,z]
        }
        else
        {
            transform.translation() = x.segment(1, 3); //x=[gamma,x,y,z]
        }
        mOut = transform.matrix();

        if (mOut != mOut)
        {
            // Degenerate situation. This can happen when the source and reading clouds
            // are identical, and then b and x above are 0, and the rotation matrix cannot
            // be determined, it comes out full of NaNs. The correct rotation is the identity.
            mOut.block(0, 0, dim - 1, dim - 1) = Matrix::Identity(dim - 1, dim - 1);
        }
    }
    else
    {
        Eigen::Transform<T, 2, Eigen::Affine> transform;
        transform = Eigen::Rotation2D<T>(x(0));
        transform.translation() = x.segment(1, 2);

        if (force2D)
        {
            mOut = Matrix::Identity(dim, dim);
            mOut.topLeftCorner(2, 2) = transform.matrix().topLeftCorner(2, 2);
            mOut.topRightCorner(2, 1) = transform.matrix().topRightCorner(2, 1);
        }
        else
        {
            mOut = transform.matrix();
        }
    }
    return mOut;
}

template<typename T, typename Matrix, typename Vector, typename LocalizabilityParametersForErrorMinimization>
void solvePossiblyUnderdeterminedLinearSystemWithInequalityConstraints(
    Vector& x,
    int& activeInequalityConstraints,
    int& numberOfEqualityConstraints,
    int& totalNumberOfConstraints,
    const Matrix& A,
    const Vector& b,
    LocalizabilityParametersForErrorMinimization& localizabilityParametersForErrorMinimization)
{
    const int numberOfConstraints = A.rows()
        - (localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityXyz_.sum()
           + localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityRpy_.sum());

    totalNumberOfConstraints = numberOfConstraints;
    const bool doConstraintOptimization = (numberOfConstraints != 0) ? true : false;

    if (!doConstraintOptimization)
    {
        // Regular vanilla ICP.
        solvePossiblyUnderdeterminedLinearSystem<T>(A, b, x, false);
    }
    else
    {
        // Cast to double
        const Eigen::MatrixXd Adouble = A.template cast<double>();
        const Eigen::MatrixXd bdouble = b.template cast<double>();
        Eigen::MatrixXd HQP{ Adouble.transpose() * Adouble };
        Eigen::VectorXd hQP{ -Adouble.transpose() * bdouble };

        Eigen::VectorXd xQP;
        Eigen::VectorXd lb;
        Eigen::VectorXd ub;
        Eigen::MatrixXd constraintMatrix = Eigen::MatrixXd::Zero(numberOfConstraints, 6);
        Eigen::VectorXd Alb = Eigen::VectorXd::Zero(numberOfConstraints, 1);
        Eigen::VectorXd Aub = Eigen::VectorXd::Zero(numberOfConstraints, 1);
        constraintMatrix.resize(numberOfConstraints, 6);

        // Read localizability flags to indices.
        std::vector<int> degenerateDirectionIndices;
        readLocalizabilityFlags<T>(degenerateDirectionIndices, localizabilityParametersForErrorMinimization);
        int numberOfEqConstraints{0};
        
        // Populate constrained optimization variables.
        generateConstrainedOptimizationProblem<T>(
            constraintMatrix, Alb, Aub, numberOfEqConstraints, degenerateDirectionIndices, localizabilityParametersForErrorMinimization);

        numberOfEqualityConstraints = numberOfEqConstraints;

        qpmad::Solver solverStandard;
        qpmad::SolverParameters param;
        param.hessian_type_ = qpmad::SolverParameters::HESSIAN_LOWER_TRIANGULAR;
        //param.hessian_type_ = qpmad::SolverParameters::HessianType::HESSIAN_CHOLESKY_FACTOR;
        //param.hessian_type_ = qpmad::SolverParameters::HessianType::HESSIAN_INVERTED_CHOLESKY_FACTOR;
        //param.return_inverted_cholesky_factor_ = true;
        qpmad::Solver::ReturnStatus status =
            solverStandard.solve(xQP, HQP, hQP, Eigen::VectorXd(), Eigen::VectorXd(), constraintMatrix, Alb, Aub, param);

        if (status != qpmad::Solver::OK)
        {
            LOG_WARNING_STREAM("Error in solving inequality constrained optimization problem with QPmad library.");
        }

        Eigen::VectorXd dual;
        Eigen::Matrix<qpmad::MatrixIndex, Eigen::Dynamic, 1> indices;
        Eigen::Matrix<bool, Eigen::Dynamic, 1> is_lower;
        int activeInequalityConstraintSize = 0;
        solverStandard.getInequalityDual(dual, indices, is_lower);
        std::cout << "Number of active Inquality Constraints: " << dual.size() << std::endl;
        activeInequalityConstraints = dual.size();

        std::cout << "Do Constraints satisfied? 0 == satisfied: " << std::endl;
        std::cout << constraintMatrix*xQP << std::endl;

        // Convert from double to float.
        const Eigen::VectorXf xQPfloat = xQP.template cast<float>();
        x.row(0) << xQPfloat(0);
        x.row(1) << xQPfloat(1);
        x.row(2) << xQPfloat(2);
        x.row(3) << xQPfloat(3);
        x.row(4) << xQPfloat(4);
        x.row(5) << xQPfloat(5);

        localizabilityParametersForErrorMinimization.constraintResidual_ = (constraintMatrix.template cast<float>()*xQPfloat).tail(numberOfEqConstraints).norm();
    }
}

template<typename T, typename Matrix, typename Vector, typename LocalizabilityParametersForErrorMinimization>
void solvePossiblyUnderdeterminedLinearSystemWithEqualityConstraints(
    Vector& x,
    const Matrix& A,
    const Vector& b,
    LocalizabilityParametersForErrorMinimization& localizabilityParametersForErrorMinimization)
{
    PointMatcherSupport::timer t;

    const int numberOfConstraints = A.rows()
        - (localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityXyz_.sum()
           + localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityRpy_.sum());
    const bool doConstraintOptimization = (numberOfConstraints != 0) ? true : false;

    if (!doConstraintOptimization)
    {
        // Regular vanilla ICP.
        solvePossiblyUnderdeterminedLinearSystem<T>(A, b, x, false);
    }
    else
    {
        // We dont know the size beforehand. We can initialize bigger and then resize.
        Matrix postAugmentedA;
        Vector postAugmentedb;

        // Constrained optimization hessian
        Matrix augmentedA = Matrix::Zero(A.rows() + numberOfConstraints, A.rows() + numberOfConstraints);
        augmentedA.topLeftCorner(A.rows(), A.cols()) = A;

        // Constrained optimization constraint vector. (6+c) x (1)
        Vector Augmentedb(Vector::Zero(augmentedA.rows(), 1));
        Augmentedb.head(b.rows()) = b;

        // Resize the output accordingly to the constrained problem.
        x.resize(augmentedA.rows());

        // Read localizability flags to indices.
        std::vector<int> degenerateDirectionIndices;
        readLocalizabilityFlags<T>(degenerateDirectionIndices, localizabilityParametersForErrorMinimization);

        if (localizabilityParametersForErrorMinimization.enableStandardWeightRegularization_){

            // L-REG, generate the regularized optimization problem.
            // Populate constrained optimization variables.
            generateRegularizedOptimizationProblem<T>(
                augmentedA, Augmentedb, degenerateDirectionIndices, localizabilityParametersForErrorMinimization, numberOfConstraints);

            // Find non-zero columns:
            Eigen::Matrix<bool, 1, Eigen::Dynamic> non_zeros = augmentedA.template cast<bool>().colwise().any();
            postAugmentedA.conservativeResize(augmentedA.rows(), non_zeros.count());

            // fill result matrix:
            Eigen::Index j=0;
            for(Eigen::Index i=0; i<augmentedA.cols(); ++i)
            {
                if(non_zeros(i)){
                    postAugmentedA.col(j++) = augmentedA.col(i);
                }
            }

            T realWeight = localizabilityParametersForErrorMinimization.regularizationWeight_;
            std::cout << "L-REG Weight: " << realWeight << std::endl;
            std::cout << "Before postAugmentedA.bottomLeftCorner(numberOfConstraints, A.cols()): " << std::endl << postAugmentedA.bottomLeftCorner(numberOfConstraints, A.cols()) << std::endl;
            postAugmentedA.bottomLeftCorner(numberOfConstraints, A.cols()) = postAugmentedA.bottomLeftCorner(numberOfConstraints, A.cols()) * realWeight; //; lcurveOptimizer->getOptimalRegularizationWeight();
            std::cout << "postAugmentedA.bottomLeftCorner(numberOfConstraints, A.cols()): " << std::endl << postAugmentedA.bottomLeftCorner(numberOfConstraints, A.cols()) << std::endl;

            // find non-zero columns:
            Eigen::Matrix<bool, 1, Eigen::Dynamic> non_zeros_row = postAugmentedA.template cast<bool>().rowwise().any();
            postAugmentedA.conservativeResize(non_zeros_row.count(), postAugmentedA.cols());

            // fill result matrix:
            Eigen::Index j2=0;
            for(Eigen::Index i=0; i<postAugmentedA.rows(); ++i)
            {
                if(non_zeros_row(i)){
                    postAugmentedA.row(j2++) = postAugmentedA.row(i);
                }
            }
            Augmentedb.conservativeResize(non_zeros_row.count(), Augmentedb.cols());

        }else
        {
            // Populate constrained optimization variables.
            generateConstrainedOptimizationProblem<T>(
                augmentedA, Augmentedb, degenerateDirectionIndices, localizabilityParametersForErrorMinimization);
        }

        if (localizabilityParametersForErrorMinimization.enableStandardWeightRegularization_){

            solvePossiblyUnderdeterminedLinearSystem<T>(postAugmentedA, Augmentedb, x, true);

            std::cout << "postAugmentedA:" << std::endl;
            std::cout << postAugmentedA << std::endl;
            
            std::cout << "x" << std::endl;
            std::cout << x << std::endl;

            std::cout << "Does Constraints satisfied?" << std::endl;
            std::cout << postAugmentedA*x << std::endl;


            localizabilityParametersForErrorMinimization.constraintResidual_ = (postAugmentedA*x - Augmentedb).tail(numberOfConstraints).norm();


        }else{
            // L-Reg enabled but there are no constraints.
            solvePossiblyUnderdeterminedLinearSystem<T>(augmentedA, Augmentedb, x, false);
            
            std::cout << "augmentedA:" << std::endl;
            std::cout << augmentedA << std::endl;
            
            std::cout << "x" << std::endl;
            std::cout << x << std::endl;

            std::cout << "Does Constraints satisfied?" << std::endl;
            std::cout << augmentedA*x << std::endl;

            localizabilityParametersForErrorMinimization.constraintResidual_ = (augmentedA*x - Augmentedb).tail(numberOfConstraints).norm();

        }

        for (Eigen::Index b = 0; b < A.rows(); b++)
        {
            const T val = x.row(b).col(0).value();
            if( (std::isnan(val)) || ( std::isinf(val))){
                //throw std::runtime_error("Optimization failed. NaN or Inf value detected in the solution.");
                MELO_ERROR_STREAM("x.col(b).row(0).value(): "<< val <<" IsNAN: " << std::isnan(val) << " IsINF: " << std::isinf(val));
                MELO_ERROR_STREAM("Optimization failed. NaN or Inf value detected in the solution.");
                // Write x std::cout 
                std::cout << "x: " << x.head(6).transpose() << std::endl;
            }
        }
    }
}

template<typename T, typename Matrix, typename Vector, typename LocalizabilityParametersForErrorMinimization>
void generateRegularizedOptimizationProblem(Matrix& augmentedA,
                                            Vector& augmentedb,
                                            const std::vector<int>& degenerateDirectionIndices,
                                            LocalizabilityParametersForErrorMinimization& localizabilityParametersForErrorMinimization, const int& nbConstraints)
{
    int constraintCounter{ 0 };
    int rotationConstraintCounter{ 0 };
    const int translationIndexOffset{ 3 };
    const int sizeOfTheProblem{ 6 };

    for (const auto& index : degenerateDirectionIndices)
    {
        // The first the rotation subspace is placed as the 3x3 topLeftCorner of the optimization hessian. Thus we are comparing against 3.
        const bool inRotationSubpace = (index < translationIndexOffset) ? true : false;

        if (inRotationSubpace)
        {
            // 0 to 5  indeces are full. 6 already starts on the new line.
            Vector temp = Vector::Zero(3);
            temp = localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.col(index);
            for (Eigen::Index b = 0; b < 3; b++)
            {
                augmentedA.row(sizeOfTheProblem + constraintCounter).col(b) << temp[b];
            }

            // Append the constraint value.
            augmentedb.row(sizeOfTheProblem + constraintCounter).col(0) << 0.0f;
            
        }
        else{
            // Place the eigenvector to the optimization hessian. Since hessian is symmetric positive definite, we place it symmetrically as well.
            Vector temp = Vector::Zero(3);
            temp = localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.col(index - translationIndexOffset);
            for (Eigen::Index b = 0; b < 3; b++)
            {

                augmentedA.row(sizeOfTheProblem + constraintCounter).col(b+translationIndexOffset) << temp[b];
            }

            // Append the constraint value.
            augmentedb.row(sizeOfTheProblem + constraintCounter).col(0) << 0.0f;
        }
        ++constraintCounter;
    }

    // Set the rhs of the constraints to 0.
    augmentedb.block(sizeOfTheProblem, 0, sizeOfTheProblem + constraintCounter, 1) << 0.0f;

}

template<typename T, typename Matrix, typename Vector, typename LocalizabilityParametersForErrorMinimization>
void generateConstrainedOptimizationProblem(Matrix& augmentedA,
                                            Vector& augmentedb,
                                            const std::vector<int>& degenerateDirectionIndices,
                                            LocalizabilityParametersForErrorMinimization& localizabilityParametersForErrorMinimization)
{
    int constraintCounter{ 0 };
    const int translationIndexOffset{ 3 };
    const int sizeOfTheProblem{ 6 };

    for (const auto& index : degenerateDirectionIndices)
    {
        // The first the rotation subspace is placed as the 3x3 topLeftCorner of the optimization hessian. Thus, we are comparing against 3.
        const bool inRotationSubpace = (index < translationIndexOffset) ? true : false;

        if (inRotationSubpace)
        {

            for (Eigen::Index i = 0;
                 i < localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityRpy_.rows();
                 i++)
            {
                // i will always go from 0 to 2 since we now add x y z components to all constraints.
                augmentedA.row(i).col(sizeOfTheProblem + constraintCounter) =
                    localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.row(i).col(index);
                augmentedA.row(sizeOfTheProblem + constraintCounter).col(i) =
                    localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.row(i).col(index);
            }

            // Append the constraint value.
            augmentedb.row(sizeOfTheProblem + constraintCounter).col(0) =
                localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityConstraints_
                    .rotationConstraintValues_.row(index)
                    .col(0);
        }
        else
        {
            // Place the eigenvector to the optimization hessian. Since hessian is symmetric positive definite, we place it symmetrically as well.
            for (Eigen::Index i = 0;
                 i < localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityXyz_.rows();
                 i++)
            {
                augmentedA.row(i + translationIndexOffset).col(sizeOfTheProblem + constraintCounter) =
                    localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.row(i).col(
                        index - translationIndexOffset);
                augmentedA.row(sizeOfTheProblem + constraintCounter).col(i + translationIndexOffset) =
                    localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.row(i).col(
                        index - translationIndexOffset);
            }
            // Append the constraint value.
            augmentedb.row(sizeOfTheProblem + constraintCounter).col(0) =
                localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityConstraints_
                    .translationConstraintValues_.row(index - translationIndexOffset)
                    .col(0);
        }
        ++constraintCounter;
    }
}

template<typename T, typename LocalizabilityParametersForErrorMinimization>
void generateConstrainedOptimizationProblem(Eigen::MatrixXd& constraintMatrix,
                                            Eigen::VectorXd& Alb,
                                            Eigen::VectorXd& Aub,
                                            int& numberOfEqualityConstraints,
                                            const std::vector<int>& degenerateDirectionIndices,
                                            LocalizabilityParametersForErrorMinimization& localizabilityParametersForErrorMinimization)
{
    const int translationIndexOffset{ 3 };
    int constraintCount{ 0 };

    for (const auto& index : degenerateDirectionIndices)
    {
        const bool inRotationSubpace = (index < translationIndexOffset) ? true : false;

        if (inRotationSubpace)
        {
            T velocityAlignment = localizabilityParametersForErrorMinimization.angularVelocityVector_.normalized().dot(localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.col(index));
            MELO_INFO_STREAM(message_logger::color::yellow << "The rotation bound: " << localizabilityParametersForErrorMinimization.inequalityBoundMultiplier_ * 0.5);

            for (Eigen::Index i = 0; i < 3; i++)
            {
                // Always go from 0 to 2 since we now add x y z components to all constraints.
                constraintMatrix.row(constraintCount).col(i) =
                    localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.rotationEigenvectors_.row(i)
                        .col(index)
                        .template cast<double>();
            }

            if (localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityConstraints_
                    .rotationConstraintValues_.row(index)
                    .col(0)
                    .value()
                >= 0.0f)
            {
                ++numberOfEqualityConstraints;
                // These constraints simply prevents optimization to update in the given direction.
                Alb.row(constraintCount).col(0) << -localizabilityParametersForErrorMinimization.inequalityBoundMultiplier_ * 0.5;
                Aub.row(constraintCount).col(0) << localizabilityParametersForErrorMinimization.inequalityBoundMultiplier_ * 0.5;
                MELO_INFO_STREAM(message_logger::color::white << "----Adding Inequality Constraint (rot)------");
                MELO_INFO_STREAM(message_logger::color::white << "----Constraint Value : " << Alb.row(constraintCount).col(0) << " " << Aub.row(constraintCount).col(0));

            }
            else
            {
                // DISABLING PARTIAL LOCALIZABILITY FOR ANALYSIS PAPER
                /*
                // Partial constraints were in place.
                if (localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityConstraints_
                        .rotationConstraintValues_.row(index)
                        .col(0)
                        .value()
                    < 0.0f)
                {
                    // If the original shift is negative it should be a lower bound.
                    const double constraintValue =
                        static_cast<double>(localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_
                                                .localizabilityConstraints_.rotationConstraintValues_.row(index)
                                                .col(0)
                                                .value());
                    Alb.row(constraintCount).col(0) << constraintValue + (constraintValue * localizabilityParametersForErrorMinimization.inequalityBoundMultiplier_ *2.0f);
                    Aub.row(constraintCount).col(0) << constraintValue - (constraintValue * localizabilityParametersForErrorMinimization.inequalityBoundMultiplier_ *2.0f);

                    //Alb.row(constraintCount).col(0) << constraintValue; // + (val * 0.1);
                    //Aub.row(constraintCount).col(0) << 0.0; //val - (val * 0.1);

                    MELO_INFO_STREAM(message_logger::color::white << "----Adding Inequality Constraint (rotation)------");
                    MELO_INFO_STREAM(message_logger::color::white << "----Constraint Value : " << constraintValue);
                    MELO_INFO_STREAM(message_logger::color::white << "Calculated Lower Bound (Rotation): " << Alb.row(constraintCount).col(0));
                    MELO_INFO_STREAM(message_logger::color::white << "Calculated Upper Bound (Rotation): " << Aub.row(constraintCount).col(0));
                }
                else
                {
                    // If the original shift is positive, it should be a upper bound.
                    const double constraintValue =
                        static_cast<double>(localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_
                                                .localizabilityConstraints_.rotationConstraintValues_.row(index)
                                                .col(0)
                                                .value());
                    
                    Alb.row(constraintCount).col(0) << constraintValue - (constraintValue * localizabilityParametersForErrorMinimization.inequalityBoundMultiplier_* 2.0f);
                    Aub.row(constraintCount).col(0) << constraintValue + (constraintValue * localizabilityParametersForErrorMinimization.inequalityBoundMultiplier_*2.0f);

                    //Alb.row(constraintCount).col(0) << 0.0; //val - (val * 0.1);
                    //Aub.row(constraintCount).col(0) << constraintValue; // + (val * 0.1);

                    MELO_INFO_STREAM(message_logger::color::white << "----Adding Inequality Constraint (rotation)------");
                    MELO_INFO_STREAM(message_logger::color::white << "----Constraint Value : " << constraintValue);
                    MELO_INFO_STREAM(message_logger::color::white << "Calculated Lower Bound (Rotation): " << Alb.row(constraintCount).col(0));
                    MELO_INFO_STREAM(message_logger::color::white << "Calculated Upper Bound (Rotation): " << Aub.row(constraintCount).col(0));
                }*/
            }
        }
        else
        {
            // Add all 3 components of the eigenvector to the constraint matrix. Cast to double for qpmad compatibility.
            for (Eigen::Index i = 0; i < 3; i++)
            {
                constraintMatrix.row(constraintCount).col(i + translationIndexOffset) =
                    localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.row(i)
                        .col(index - translationIndexOffset)
                        .template cast<double>();
            }


            if ((localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityConstraints_
                     .translationConstraintValues_.row(index - translationIndexOffset)
                     .col(0)
                     .value()
                 >= 0.0f))
            {

                MELO_INFO_STREAM(message_logger::color::yellow << "The translation bound: " << localizabilityParametersForErrorMinimization.inequalityBoundMultiplier_);

                ++numberOfEqualityConstraints;
                Alb.row(constraintCount).col(0) << -localizabilityParametersForErrorMinimization.inequalityBoundMultiplier_;
                Aub.row(constraintCount).col(0) << localizabilityParametersForErrorMinimization.inequalityBoundMultiplier_;

                MELO_INFO_STREAM(message_logger::color::white << "----Adding Inequality Constraint (trans)------");
                MELO_INFO_STREAM(message_logger::color::white << "----Constraint Value : " << Alb.row(constraintCount).col(0) << " " << Aub.row(constraintCount).col(0));

            }
            else
            {
                /*
                // Partial constraints were in place.

                T velocityAlignment = localizabilityParametersForErrorMinimization.linearVelocityVector_.normalized().dot(localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.translationEigenvectors_.col(index - translationIndexOffset));
                MELO_INFO_STREAM(message_logger::color::yellow << "TRANS Velocity alignment of index:" <<index << " is " << velocityAlignment);

                if (localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityConstraints_
                        .translationConstraintValues_.row(index - translationIndexOffset)
                        .col(0)
                        .value()
                    < 0.0f)
                {
                    const double constraintValue =
                        static_cast<double>(localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_
                                                .localizabilityConstraints_.translationConstraintValues_.row(index - translationIndexOffset)
                                                .col(0)
                                                .value());


                    Alb.row(constraintCount).col(0) << constraintValue + (constraintValue * localizabilityParametersForErrorMinimization.inequalityBoundMultiplier_);
                    Aub.row(constraintCount).col(0) << constraintValue - (constraintValue * localizabilityParametersForErrorMinimization.inequalityBoundMultiplier_);


                    // If the original shift is negative it should be a lower bound.
                    //Alb.row(constraintCount).col(0) << constraintValue; // + (val * 0.1);
                    //Aub.row(constraintCount).col(0) << 0.0; //val - (val * 0.1);

                    MELO_INFO_STREAM(message_logger::color::white << "----Adding Inequality Constraint (Translation)------");
                    MELO_INFO_STREAM(message_logger::color::white << "----Constraint Value : " << constraintValue);
                    MELO_INFO_STREAM(message_logger::color::white << "Calculated Lower Bound (Translation): " << Alb.row(constraintCount).col(0));
                    MELO_INFO_STREAM(message_logger::color::white << "Calculated Upper Bound (Translation): " << Aub.row(constraintCount).col(0));
                }
                else
                {
                    // If the original shift is positive, it should be a upper bound.
                    const double constraintValue =
                        static_cast<double>(localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_
                                                .localizabilityConstraints_.translationConstraintValues_.row(index - translationIndexOffset)
                                                .col(0)
                                                .value());
                    
                    Alb.row(constraintCount).col(0) << constraintValue - (constraintValue * localizabilityParametersForErrorMinimization.inequalityBoundMultiplier_);
                    Aub.row(constraintCount).col(0) << constraintValue + (constraintValue * localizabilityParametersForErrorMinimization.inequalityBoundMultiplier_);

                    //Alb.row(constraintCount).col(0) << 0.0; //val - (val * 0.1);
                    //Aub.row(constraintCount).col(0) << constraintValue; // + (val * 0.1);

                    MELO_INFO_STREAM(message_logger::color::white << "----Adding Inequality Constraint (Translation)------");
                    MELO_INFO_STREAM(message_logger::color::white << "----Constraint Value : " << constraintValue);
                    MELO_INFO_STREAM(message_logger::color::white << "Calculated Lower Bound (Translation): " << Alb.row(constraintCount).col(0));
                    MELO_INFO_STREAM(message_logger::color::white << "Calculated Upper Bound (Translation): " << Aub.row(constraintCount).col(0));
                }*/
            }
        }
        ++constraintCount;
    }
}

template<typename T, typename LocalizabilityParametersForErrorMinimization>
void readLocalizabilityFlags(std::vector<int>& degenerateIndices,
                             LocalizabilityParametersForErrorMinimization& localizabilityParametersForErrorMinimization)
{
    // The order of the values do matter. We first iterate through rotational sub-space since in the optimization rotation sub-space comes first.
    for (Eigen::Index i = 0; i < localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityRpy_.rows(); i++)
    {
        if (localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityRpy_.row(i).value()
            == static_cast<T>(PointMatcher<T>::LocalizabilityCategory::kNonLocalizable))
        {
            degenerateIndices.emplace_back(i);
        }
    }

    for (Eigen::Index i = 0; i < localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityXyz_.rows(); i++)
    {
        if (localizabilityParametersForErrorMinimization.localizabilityAnalysisResults_.localizabilityXyz_.row(i).value()
            == static_cast<T>(PointMatcher<T>::LocalizabilityCategory::kNonLocalizable))
        {
            degenerateIndices.emplace_back(i + 3);
        }
    }
}

template<typename T>
T PointToPlaneErrorMinimizer<T>::computeResidualError(ErrorElements mPts, const bool& force2D)
{
	const int dim = mPts.reading.features.rows();
	const int nbPts = mPts.reading.features.cols();

	// Adjust if the user forces 2D minimization on XY-plane
	int forcedDim = dim - 1;
	if(force2D && dim == 4)
	{
		mPts.reading.features.conservativeResize(3, Eigen::NoChange);
		mPts.reading.features.row(2) = Matrix::Ones(1, nbPts);
		mPts.reference.features.conservativeResize(3, Eigen::NoChange);
		mPts.reference.features.row(2) = Matrix::Ones(1, nbPts);
		forcedDim = dim - 2;
	}

	// Fetch normal vectors of the reference point cloud (with adjustment if needed)
	const BOOST_AUTO(normalRef, mPts.reference.getDescriptorViewByName("normals").topRows(forcedDim));

	// Note: Normal vector must be precalculated to use this error. Use appropriate input filter.
	assert(normalRef.rows() > 0);

	const Matrix deltas = mPts.reading.features - mPts.reference.features;

	// dotProd = dot(deltas, normals) = d.n
	Matrix dotProd = Matrix::Zero(1, normalRef.cols());
	for(int i = 0; i < normalRef.rows(); i++)
	{
		dotProd += (deltas.row(i).array() * normalRef.row(i).array()).matrix();
	}
	// residual = w*(d.n)²
	dotProd = (mPts.weights.row(0).array() * dotProd.array().square()).matrix();

	// return sum of the norm of each dot product
	return dotProd.sum();
}


template<typename T>
T PointToPlaneErrorMinimizer<T>::getResidualError(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const OutlierWeights& outlierWeights,
	const Matches& matches) const
{
	assert(matches.ids.rows() > 0);

	// Fetch paired points
	typename ErrorMinimizer::ErrorElements mPts(filteredReading, filteredReference, outlierWeights, matches);

	return PointToPlaneErrorMinimizer::computeResidualError(mPts, force2D);
}

template<typename T>
T PointToPlaneErrorMinimizer<T>::getOverlap() const
{

	// Gather some information on what kind of point cloud we have
	const bool hasReadingNoise = this->lastErrorElements.reading.descriptorExists("simpleSensorNoise");
	const bool hasReferenceNoise = this->lastErrorElements.reference.descriptorExists("simpleSensorNoise");
	const bool hasReferenceDensity = this->lastErrorElements.reference.descriptorExists("densities");

	const int nbPoints = this->lastErrorElements.reading.features.cols();
	const int dim = this->lastErrorElements.reading.features.rows();

	// basix safety check
	if(nbPoints == 0)
	{
		throw std::runtime_error("Error, last error element empty. Error minimizer needs to be called at least once before using this method.");
	}

	Eigen::Array<T, 1, Eigen::Dynamic>  uncertainties(nbPoints);

	// optimal case
	if (hasReadingNoise && hasReferenceNoise && hasReferenceDensity)
	{
		// find median density

		Matrix densities = this->lastErrorElements.reference.getDescriptorViewByName("densities");
		vector<T> values(densities.data(), densities.data() + densities.size());

		// sort up to half the values
		nth_element(values.begin(), values.begin() + (values.size() * 0.5), values.end());

		// extract median value
		const T medianDensity = values[values.size() * 0.5];
		const T medianRadius = 1.0/pow(medianDensity, 1/3.0);

		uncertainties = (medianRadius +
						this->lastErrorElements.reading.getDescriptorViewByName("simpleSensorNoise").array() +
						this->lastErrorElements.reference.getDescriptorViewByName("simpleSensorNoise").array());
	}
	else if(hasReadingNoise && hasReferenceNoise)
	{
		uncertainties = this->lastErrorElements.reading.getDescriptorViewByName("simpleSensorNoise") +
						this->lastErrorElements.reference.getDescriptorViewByName("simpleSensorNoise");
	}
	else if(hasReadingNoise)
	{
		uncertainties = this->lastErrorElements.reading.getDescriptorViewByName("simpleSensorNoise");
	}
	else if(hasReferenceNoise)
	{
		uncertainties = this->lastErrorElements.reference.getDescriptorViewByName("simpleSensorNoise");
	}
	else
	{
		LOG_INFO_STREAM("PointToPlaneErrorMinimizer - warning, no sensor noise and density. Using best estimate given outlier rejection instead.");
		return this->getWeightedPointUsedRatio();
	}


	const Vector dists = (this->lastErrorElements.reading.features.topRows(dim-1) - this->lastErrorElements.reference.features.topRows(dim-1)).colwise().norm();


	// here we can only loop through a list of links, but we are interested in whether or not
	// a point has at least one valid match.
	int count = 0;
	int nbUniquePoint = 1;
	Vector lastValidPoint = this->lastErrorElements.reading.features.col(0) * 2.;
	for(int i=0; i < nbPoints; i++)
	{
		const Vector point = this->lastErrorElements.reading.features.col(i);

		if(lastValidPoint != point)
		{
			// NOTE: we tried with the projected distance over the normal vector before:
			// projectionDist = delta dotProduct n.normalized()
			// but this doesn't make sense 


			if(PointMatcherSupport::anyabs(dists(i, 0)) < (uncertainties(0,i)))
			{
				lastValidPoint = point;
				count++;
			}
		}

		// Count unique points
		if(i > 0)
		{
			if(point != this->lastErrorElements.reading.features.col(i-1))
				nbUniquePoint++;
		}

	}
	//cout << "count: " << count << ", nbUniquePoint: " << nbUniquePoint << ", this->lastErrorElements.nbRejectedPoints: " << this->lastErrorElements.nbRejectedPoints << endl;

	return (T)count/(T)(nbUniquePoint + this->lastErrorElements.nbRejectedPoints);
}

template struct PointToPlaneErrorMinimizer<float>;
template struct PointToPlaneErrorMinimizer<double>;
