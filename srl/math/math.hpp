#ifndef SRL_MATH_HPP
#define SRL_MATH_HPP
#pragma once

#include <srl/concepts.hpp>
#include <srl/constants.hpp>
#include <srl/types.hpp>

#include <cmath>

namespace srl::math {

//
// ----- Floating point functions

using std::abs;

/**
 * @brief Compare if two floating point numbers are equal withing the given precision.
 *
 * Performs \f$ \lvert A - B \rvert < a_tol + r_tol * B \f$.
 *
 * **Example**
 * \snippet example_math.cpp example_eq
 * **Example Output**
 * \snippet example_math.cpp example_eq_output
 *
 * @tparam FP Floating point type.
 * @param A Left-hand side parameter.
 * @param B Right-hand side parameter.
 * @param r_tol The absolute precision of the comparison.
 * @param a_tol The relative precision of the comparison.
 * @return bool true if the two floats 'are equal', false otherwise.
 */
template<srl::concepts::floating_point FP>
auto eq(const FP &A, const FP &B, const FP &r_tol = static_cast<FP>(1e-5), const FP &a_tol = static_cast<FP>(1e-8)) -> bool;

//
// ----- Matrix functions

/**
 * @brief Compare if two matrices are equal withing the given precision.
 *
 * Performs \f$ \lvert A - B \rvert < a_tol + r_tol * B \f$, where the norm the Euclidean (Frobenius) norm.
 *
 * **Example**
 * \snippet example_math.cpp example_matEq
 * **Example Output**
 * \snippet example_math.cpp example_matEq_output
 *
 * @tparam Mat Matrix type.
 * @param A Left-hand side parameter.
 * @param B Right-hand side parameter.
 * @param r_tol The absolute precision of the comparison.
 * @param a_tol The relative precision of the comparison.
 * @return bool true if the two matrices 'are equal', false otherwise.
 */
template<srl::concepts::matrix Mat>
auto matEq(
  const Mat &A,
  const Mat &B,
  const typename Mat::Scalar &r_tol = static_cast<typename Mat::Scalar>(1e-5),
  const typename Mat::Scalar &a_tol = static_cast<typename Mat::Scalar>(1e-8)) -> bool;

/**
 * @brief Check if the given matrix is positive definite.
 *
 * **Example**
 * \snippet example_math.cpp example_isPositiveDefinite
 * **Example Output**
 * \snippet example_math.cpp example_isPositiveDefinite_output
 *
 * @tparam Mat Eigen matrix type.
 * @param matrix The matrix to check.
 * @return bool true if the given matrix is positive definite, false otherwise.
 */
template<srl::concepts::dynamic_or_square_matrix Mat>
auto isPositiveDefinite(const Mat &matrix) -> bool;

/**
 * @brief Perform singular value decomposition of the given matrix.
 *
 * **Example**
 * \snippet example_math.cpp example_svd
 * **Example Output**
 * \snippet example_math.cpp example_svd_output
 *
 * @tparam Mat Eigen matrix type.
 * @param mat The matrix to decompose.
 * @return auto std::tuple containing the decomposition U, D, V; where \f$ \mathrm{mat} = U \cdot \mathrm{diag}(D) \cdot V^{*} \f$.
 */
template<srl::concepts::matrix Mat>
auto svd(const Mat &mat) -> auto;

/**
 * @brief Check if the given matrix is a valid inertia tensor.
 *
 * **Example**
 * \snippet example_math.cpp example_isInertia
 * **Example Output**
 * \snippet example_math.cpp example_isInertia_output
 *
 * @tparam FP Floating point type.
 * @param inertia The matrix to check.
 * @return bool true if the given matrix is a valid inertia, false otherwise.
 */
template<srl::concepts::floating_point FP>
auto isInertia(const srl::Matrix3<FP> &inertia) -> bool;

//
// ----- Lie group functions
/**
 * @brief Create a rotation matrix from an angle and a normed axis.
 *
 * **Example**
 * \snippet example_math.cpp example_rotation1
 * **Example Output**
 * \snippet example_math.cpp example_rotation1_output
 *
 * @tparam FP Floating point type.
 * @param angle The angle indicating how much to rotate.
 * @param normed_axis The axis around which to rotate.
 * @return srl::Matrix3<FP> The rotation matrix.
 */
template<srl::concepts::floating_point FP>
auto rotation(const FP &angle, const srl::Vector3<FP> &normed_axis) -> srl::Matrix3<FP>;

/**
 * @brief Create a rotation matrix from a rotation vector.
 *
 * **Example**
 * \snippet example_math.cpp example_rotation2
 * **Example Output**
 * \snippet example_math.cpp example_rotation2_output
 *
 * @tparam FP Floating point type.
 * @param rot_vec The rotation vector representing the rotation.
 * @return srl::Matrix3<FP> The rotation matrix.
 */
template<srl::concepts::floating_point FP>
auto rotation(const srl::Vector3<FP> &rot_vec) -> srl::Matrix3<FP>;

/**
 * @brief Apply an adjoint transformation to a matrix, i.e. \f$ \mathrm{rotation} \cdot \mathrm{element} \cdot \mathrm{rotation}^{T} \f$.
 *
 * **Example**
 * \snippet example_math.cpp example_adjoint1
 * **Example Output**
 * \snippet example_math.cpp example_adjoint1_output
 *
 * @tparam FP Floating point type.
 * @param element The matrix to which the adjoint is applied.
 * @param rotation The rotation matrix used for adjoint operation.
 * @return srl::Matrix3<FP> The adjoint of the element by rotation.
 */
template<srl::concepts::floating_point FP>
auto adjoint(const srl::Matrix3<FP> &element, const srl::Matrix3<FP> &rotation) -> srl::Matrix3<FP>;

/**
 * @brief Apply an adjoint transformation to a matrix, i.e. \f$ \mathrm{rotation} \cdot \mathrm{diag(element)} \cdot \mathrm{rotation}^{T} \f$.
 *
 * **Example**
 * \snippet example_math.cpp example_adjoint2
 * **Example Output**
 * \snippet example_math.cpp example_adjoint2_output
 *
 * @tparam FP Floating point type.
 * @param diagonal The diagonal of a matrix to which the adjoint is applied.
 * @param rotation The rotation matrix used for adjoint operation.
 * @return srl::Matrix3<FP> The adjoint of the diagonal by rotation.
 */
template<srl::concepts::floating_point FP>
auto adjoint(const srl::Vector3<FP> &diagonal, const srl::Matrix3<FP> &rotation) -> srl::Matrix3<FP>;

/**
 * @brief Performs \f$ \sum_i L_{ijk} \mathrm{vec}_{i} \f$, where \f$ L_{i} \f$ are the generators of so3 Lie algebra.
 *
 * **Example**
 * \snippet example_math.cpp example_so3element
 * **Example Output**
 * \snippet example_math.cpp example_so3element_output
 *
 * @tparam FP Floating point type.
 * @param vec The vector to contract with so3 generators.
 * @return srl::Matrix3<FP> The Lie algebra element.
 */
template<srl::concepts::floating_point FP>
auto so3element(const srl::Vector3<FP> &vec) -> srl::Matrix3<FP>;

}    // namespace srl::math

#include <srl/math/math-impl.hpp>

// ===== ===== ===== ===== Instantiations
/// tell doxygen to skip from here \cond
// ----- Matrix functions
extern template auto srl::math::isPositiveDefinite(const srl::Matrix3f &matrix) -> bool;
extern template auto srl::math::isPositiveDefinite(const srl::Matrix3d &matrix) -> bool;

extern template auto srl::math::isInertia(const srl::Matrix3f &inertia) -> bool;
extern template auto srl::math::isInertia(const srl::Matrix3d &inertia) -> bool;

// - Lie group functions
extern template auto srl::math::rotation(const float &angle, const srl::Vector3f &normed_axis) -> srl::Matrix3f;
extern template auto srl::math::rotation(const double &angle, const srl::Vector3d &normed_axis) -> srl::Matrix3d;

extern template auto srl::math::rotation(const srl::Vector3f &rot_vec) -> srl::Matrix3f;
extern template auto srl::math::rotation(const srl::Vector3d &rot_vec) -> srl::Matrix3d;

extern template auto srl::math::adjoint(const srl::Matrix3f &element, const srl::Matrix3f &rotation) -> srl::Matrix3f;
extern template auto srl::math::adjoint(const srl::Matrix3d &element, const srl::Matrix3d &rotation) -> srl::Matrix3d;

extern template auto srl::math::adjoint(const srl::Vector3f &diagonal, const srl::Matrix3f &rotation) -> srl::Matrix3f;
extern template auto srl::math::adjoint(const srl::Vector3d &diagonal, const srl::Matrix3d &rotation) -> srl::Matrix3d;

extern template auto srl::math::so3element(const srl::Vector3f &vec) -> srl::Matrix3f;
extern template auto srl::math::so3element(const srl::Vector3d &vec) -> srl::Matrix3d;
/// tell doxygen to skip untill here \endcond

#endif    // SRL_MATH_HPP
