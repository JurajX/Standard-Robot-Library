#ifndef SRL_MATH_IMPL_HPP
#define SRL_MATH_IMPL_HPP
#pragma once

#include <srl/math/math.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <fmt/format.h>

#include <stdexcept>
#include <tuple>

namespace srl::math {

//
// ----- Floating point functions
template<srl::concepts::floating_point FP>
auto eq(const FP &A, const FP &B, const FP &r_tol, const FP &a_tol) -> bool
{
    return (abs(A - B) < a_tol + r_tol * abs(B));
}

//
// ----- Matrix functions
template<srl::concepts::matrix Mat>
auto matEq(const Mat &A, const Mat &B, const typename Mat::Scalar &r_tol, const typename Mat::Scalar &a_tol) -> bool
{
    return (A - B).norm() < a_tol + r_tol * B.norm();
}

template<srl::concepts::dynamic_or_square_matrix Mat>
auto isPositiveDefinite(const Mat &matrix) -> bool
{
    if constexpr (srl::concepts::dynamic_matrix<Mat>) {
        if (matrix.rows() != matrix.cols())
            throw std::invalid_argument(fmt::format("Expected a square matrix, got ({}x{})", matrix.rows(), matrix.cols()));
    }
    // Cholesky decomposition - exists only for symmetric, positive definite matrices
    Eigen::LLT<Mat> llt(matrix);
    if (llt.info() == Eigen::NumericalIssue)
        return false;
    return true;
}

template<srl::concepts::matrix Mat>
auto svd(const Mat &mat) -> auto
{
    unsigned int options { 0 };
    if constexpr (srl::concepts::dynamic_matrix<Mat>) {
        options = Eigen::ComputeThinU | Eigen::ComputeThinV;
    } else {
        options = Eigen::ComputeFullU | Eigen::ComputeFullV;
    }
    Eigen::JacobiSVD<Mat> svd_holder(mat, options);
    auto U { svd_holder.matrixU() };
    auto diag { svd_holder.singularValues() };
    auto V { svd_holder.matrixV() };
    return std::make_tuple(U, diag, V);
}

template<srl::concepts::floating_point FP>
auto isInertia(const srl::Matrix3<FP> &inertia) -> bool
{
    auto [U, diag, V] { svd(inertia) };
    bool isPosDef { (U * V.transpose()).isApprox(kMatrixId3<FP>) };
    bool fulfilTriangIneq { diag(0) < (diag(1) + diag(2)) && diag(1) < (diag(2) + diag(0)) && diag(2) < (diag(0) + diag(1)) };
    if (isPosDef && fulfilTriangIneq)
        return true;
    return false;
}

//
// ----- Lie group functions
template<srl::concepts::floating_point FP>
auto rotation(const FP &angle, const srl::Vector3<FP> &normed_axis) -> srl::Matrix3<FP>
{
    return Eigen::AngleAxis<FP>(angle, normed_axis).matrix();
}

template<srl::concepts::floating_point FP>
auto rotation(const srl::Vector3<FP> &rot_vec) -> srl::Matrix3<FP>
{
    FP norm { rot_vec.norm() };
    if (eq<FP>(norm, 0, 0, srl::kEps<FP>)) {
        return srl::kMatrixId3<FP>;
    }
    return rotation(norm, static_cast<srl::Vector3<FP>>((1 / norm) * rot_vec));
}

template<srl::concepts::floating_point FP>
auto adjoint(const srl::Matrix3<FP> &element, const srl::Matrix3<FP> &rotation) -> srl::Matrix3<FP>
{
    return rotation * element * rotation.transpose();
}

template<srl::concepts::floating_point FP>
auto adjoint(const srl::Vector3<FP> &diagonal, const srl::Matrix3<FP> &rotation) -> srl::Matrix3<FP>
{
    return adjoint(static_cast<srl::Matrix3<FP>>(diagonal.asDiagonal()), rotation);
}

template<srl::concepts::floating_point FP>
auto so3element(const srl::Vector3<FP> &vec) -> srl::Matrix3<FP>
{
    return { vec[0] * srl::kSo3<FP>[0] + vec[1] * srl::kSo3<FP>[1] + vec[2] * srl::kSo3<FP>[2] };
}

}    // namespace srl::math

#endif    // SRL_MATH_IMPL_HPP
