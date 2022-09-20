#pragma once

#include <Eigen/SVD>

#include <algorithm>
#include <limits>

namespace math {

/*!
 * Return the Moore–Penrose pseudoinverse of a matrix.
 *
 * The pseudoinverse of matrix \p A is computed by using its singular value
 * decomposition.
 *
 * \param A The matrix of which to compute the pseudoinverse.
 * \param eps The factor for determining the tolerance (defaults to the
 *        machine epsilon of the element data type of matrix \p A).
 * \return The pseudoinverse.
 *
 * \note Will only work with dynamic size Eigen matrices!
 */
template<typename MatType>
MatType pseudoinverse(const MatType& A,
                      typename MatType::Scalar eps = std::numeric_limits<typename MatType::Scalar>::epsilon())
{
    Eigen::JacobiSVD<MatType> svd(A, (Eigen::ComputeThinU | Eigen::ComputeThinV));
    typename MatType::Scalar tol = eps * std::max(A.cols(), A.rows()) * svd.singularValues().array().abs()[0];
    return svd.matrixV() * ((svd.singularValues().array().abs() > tol).select(svd.singularValues().array().inverse(), 0)).matrix().asDiagonal() * svd.matrixU().adjoint();
}

} // namespace math
