#ifndef SRL_TYPES_HPP
#define SRL_TYPES_HPP
#pragma once

#include <srl/common/concepts.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <utility>
#include <vector>

using std::size_t;

namespace srl {
//
// ----- std types
/// @brief Typedef for unsigned int.
using uint = unsigned int;

//
// ----- std types
/// @brief Typedef for the standard library vector.
using std::vector;

//
// ----- Vectors
/// @brief Typedef for the Eigen 3-dim vector.
/// @tparam FP Floating point type.
template<srl::concepts::floating_point FP>
using Vector3 = Eigen::Matrix<FP, 3, 1, Eigen::AutoAlign>;
/// @brief Typedef for the Eigen 3-dim double vector.
using Vector3d = Vector3<double>;
/// @brief Typedef for the Eigen 3-dim float vector.
using Vector3f = Vector3<float>;

/// @brief Typedef for the N-dim Eigen vector.
/// @tparam FP Floating point type.
/// @tparam N Size of the vector.
template<srl::concepts::floating_point FP, size_t N>
using VectorN = Eigen::Matrix<FP, N, 1, Eigen::AutoAlign>;
/// @brief Typedef for the N-dim Eigen double vector.
/// @tparam N Size of the vector.
template<size_t N>
using VectorNd = VectorN<double, N>;
/// @brief Typedef for the N-dim Eigen float vector.
/// @tparam N Size of the vector.
template<size_t N>
using VectorNf = VectorN<float, N>;

/// @brief Typedef for the Eigen dynamic size vector.
/// @tparam FP Floating point type.
template<srl::concepts::floating_point FP>
using VectorX = Eigen::Matrix<FP, Eigen::Dynamic, 1, Eigen::AutoAlign>;
/// @brief Typedef for the Eigen dynamic size double vector.
using VectorXd = VectorX<double>;
/// @brief Typedef for the Eigen dynamic size float vector.
using VectorXf = VectorX<float>;

//
// ----- Maps & Strides
/// @brief Typedef for a dynamic Eigen Stride.
using Stride = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;
/// @brief Typedef for a Eigen Map.
template<class PlainObjectType, class StrideType = Eigen::Stride<0, 0>, Eigen::AlignmentType MapOptions = Eigen::Unaligned>
using Map = Eigen::Map<PlainObjectType, MapOptions, StrideType>;

//
// ----- Matrices
/// @brief Typedef for the 3-dim Eigen square matrix.
/// @tparam FP Floating point type.
template<srl::concepts::floating_point FP>
using Matrix3 = Eigen::Matrix<FP, 3, 3, Eigen::AutoAlign>;
/// @brief Typedef for the 3-dim Eigen square double matrix.
using Matrix3d = Matrix3<double>;
/// @brief Typedef for the 3-dim Eigen square float matrix.
using Matrix3f = Matrix3<float>;

/// @brief Typedef for the N-dim Eigen square matrix.
/// @tparam FP Floating point type.
/// @tparam N Size of the vector.
template<srl::concepts::floating_point FP, size_t N>
using MatrixN = Eigen::Matrix<FP, N, N, Eigen::AutoAlign>;
/// @brief Typedef for the N-dim Eigen square double matrix.
/// @tparam N Size of the vector.
template<size_t N>
using MatrixNd = MatrixN<double, N>;
/// @brief Typedef for the N-dim Eigen square float matrix.
/// @tparam N Size of the vector.
template<size_t N>
using MatrixNf = MatrixN<float, N>;

/// @brief Typedef for the dynamic size Eigen matrix.
/// @tparam FP Floating point type.
template<srl::concepts::floating_point FP>
using MatrixX = Eigen::Matrix<FP, Eigen::Dynamic, Eigen::Dynamic, Eigen::AutoAlign>;
/// @brief Typedef for the dynamic size Eigen double matrix.
using MatrixXd = MatrixX<double>;
/// @brief Typedef for the dynamic size Eigen float matrix.
using MatrixXf = MatrixX<float>;

/// @brief Typedef for the 3 times dynamic size Eigen matrix.
/// @tparam FP Floating point type.
template<srl::concepts::floating_point FP>
using Matrix3X = Eigen::Matrix<FP, 3, Eigen::Dynamic, Eigen::AutoAlign>;
/// @brief Typedef for the 3 times dynamic size Eigen double matrix.
using Matrix3Xd = Matrix3X<double>;
/// @brief Typedef for the 3 times dynamic size Eigen float matrix.
using Matrix3Xf = Matrix3X<float>;

//
// ----- Transformations
/// @brief Typedef for the 3-dim Eigen isometry transformation.
/// @tparam FP Floating point type.
template<srl::concepts::floating_point FP>
using Transform3 = Eigen::Transform<FP, 3, Eigen::Isometry>;
/// @brief Typedef for the 3-dim Eigen isometry double transformation.
using Transform3d = Transform3<double>;
/// @brief Typedef for the 3-dim Eigen isometry float transformation.
using Transform3f = Transform3<float>;

//
// ----- Angle Axis
/// @brief Typedef for the 3-dim Eigen angle-axis rotation representation.
/// @tparam FP Floating point type.
template<srl::concepts::floating_point FP>
using AAxis = Eigen::AngleAxis<FP>;
/// @brief Typedef for the 3-dim Eigen angle-axis rotation representation for double.
using AAxisd = AAxis<double>;
/// @brief Typedef for the 3-dim Eigen angle-axis rotation representation for float.
using AAxisf = AAxis<float>;

//
// ----- Quaternions
/// @brief Typedef for the Eigen quaternions.
/// @tparam FP Floating point type.
template<srl::concepts::floating_point FP>
using Quat = Eigen::Quaternion<FP, Eigen::AutoAlign>;
/// @brief Typedef for the Eigen double quaternions.
using Quatd = Quat<double>;
/// @brief Typedef for the Eigen float quaternions.
using Quatf = Quat<float>;

//
// ----- Pair of Quaternion and Vector3
/// @brief Typedef for an standard library pair of Eigen quaternions and 3-dim vectors.
/// @tparam FP Floating point type.
template<srl::concepts::floating_point FP>
using QuatVec3 = std::pair<srl::Quat<FP>, srl::Vector3<FP>>;
/// @brief Typedef for an standard library pair of Eigen double quaternions and 3-dim vectors.
using QuatVec3d = QuatVec3<double>;
/// @brief Typedef for an standard library pair of Eigen float quaternions and 3-dim vectors.
using QuatVec3f = QuatVec3<float>;

}    // namespace srl

#endif    // SRL_TYPES_HPP
