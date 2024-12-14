#include <Eigen/Core>
#include <Eigen/SVD>

template <class MatrixType>
class TSVD
{
    typedef typename MatrixType::Scalar Scalar;
    Eigen::JacobiSVD<MatrixType> svd;
    int numTruncated{0};
    Eigen::DiagonalMatrix<Scalar, Eigen::Dynamic> sinv;

public:

    // Constructor
    TSVD() {;}

    // Converter
    TSVD& computeFromExistingSinv(const MatrixType& matrix, const Eigen::DiagonalMatrix<Scalar, Eigen::Dynamic>& sinv_external){

        // Get the SVD of the hessian.
        svd.compute(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);

        // In 1 sentence: set the eigenvalue of the degenerate eigenvector to 0. Rest to 1/eigenvalue.
        if ((sinv_external.diagonal().array() == 0.000).any())
        {
           numTruncated = (sinv_external.diagonal().array() == 0.000).count();
            sinv.diagonal() = sinv_external.diagonal();
            return *this;

        }else{
            numTruncated = 0;

            // We dont need to deal with sinv.
            return *this;
        }
    }
        
    template <class VectorType1, class VectorType2>
    void solve(const Eigen::MatrixBase<VectorType1>& b, Eigen::MatrixBase<VectorType2>& optimizedVariables) const {
        if(numTruncated > 0){
            // Use the re-mapped eigen values
            std::cout << "TSVD is active"<< std::endl;
            std::cout << "Number of Truncated eigenvalues: " << numTruncated << std::endl;
            std::cout << "Solution: " << std::endl << sinv.diagonal() << std::endl;
            
            MatrixType tbsolved = MatrixType::Identity(6,6);
            tbsolved = svd.matrixV() * sinv * svd.matrixU().transpose();

            optimizedVariables = tbsolved * b;
        } else {
            optimizedVariables = svd.solve(b);
        }
    }
};