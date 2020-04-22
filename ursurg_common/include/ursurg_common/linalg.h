/*
 * Raven2 - Software for the Raven II surgical robot
 * Copyright (C) 2016-2017 Kim Lindberg Schwaner <kils@mmmi.sdu.dk>
 *
 * This file is part of Raven2.
 *
 * Raven2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Raven2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Raven2.  If not, see <http://www.gnu.org/licenses/>.
 */

/*!
 * \file
*/

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
