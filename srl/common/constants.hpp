#ifndef SRL_CONSTANTS_HPP
#define SRL_CONSTANTS_HPP
#pragma once

#include <srl/common/common.hpp>

#include <numbers>

namespace srl {

/// @brief Definition of epsilon, a constant for floating point precision.
/// @tparam FP Floating point type.
template<srl::concepts::floating_point FP>
inline constexpr FP kEps { static_cast<FP>(1e-5) };
/// @brief Definition of epsilon, a constant for double precision.
inline constexpr double kEpsd { kEps<double> };
/// @brief Definition of epsilon, a constant for float precision.
inline constexpr float kEpsf { kEps<float> };

/// @brief Definition of a dimension size constant.
inline constexpr size_t kDim { 3 };

/// @brief Definition of the normal gravitational acceleration.
/// @tparam FP Floating point type.
template<srl::concepts::floating_point FP>
inline constexpr FP kG { static_cast<FP>(9.80665) };
/// @brief Definition of the normal gravitational acceleration for double.
inline constexpr double kGd { kG<double> };
/// @brief Definition of the normal gravitational acceleration for float.
inline constexpr float kGf { kG<float> };

/// @brief Definition of the number \f$ \pi \f$.
/// @tparam FP Floating point type.
template<srl::concepts::floating_point FP>
inline constexpr FP kPI { std::numbers::pi_v<FP> };
/// @brief Definition of the number \f$ \pi \f$ for double.
inline constexpr double kPId { std::numbers::pi_v<double> };
/// @brief Definition of the number \f$ \pi \f$ for float.
inline constexpr float kPIf { std::numbers::pi_v<float> };

/// @brief Definition of the 3-dim zero matrix.
/// @tparam FP Floating point type.
template<srl::concepts::floating_point FP>
inline const Matrix3<FP> kMatrixZero3 { Matrix3<FP>::Zero() };
/// @brief Definition of the 3-dim zero matrix for double.
inline const Matrix3d kMatrixZero3d { Matrix3d::Zero() };
/// @brief Definition of the 3-dim zero matrix for float.
inline const Matrix3f kMatrixZero3f { Matrix3f::Zero() };

/// @brief Definition of the 3-dim identity matrix.
/// @tparam FP Floating point type.
template<srl::concepts::floating_point FP>
inline const Matrix3<FP> kMatrixId3 { Matrix3<FP>::Identity() };
/// @brief Definition of the 3-dim identity matrix for double.
inline const Matrix3d kMatrixId3d { Matrix3d::Identity() };
/// @brief Definition of the 3-dim identity matrix for float.
inline const Matrix3f kMatrixId3f { Matrix3f::Identity() };

/// @brief Definition of the 3-dim identity transformation.
/// @tparam FP Floating point type.
template<srl::concepts::floating_point FP>
inline const Transform3<FP> kTransformId3 { Transform3<FP>::Identity() };
/// @brief Definition of the 3-dim identity transformation for double.
inline const Transform3d kTransformId3d { Transform3d::Identity() };
/// @brief Definition of the 3-dim identity transformation for float.
inline const Transform3f kTransformId3f { Transform3f::Identity() };

/// @brief Definition of the so3 generators.
/// @tparam FP Floating point type.
template<srl::concepts::floating_point FP>
inline const srl::vector<Matrix3<FP>> kSo3 {
    Matrix3<FP> { { 0, 0, 0 }, { 0, 0, -1 },  { 0, 1, 0 }},
    Matrix3<FP> { { 0, 0, 1 },  { 0, 0, 0 }, { -1, 0, 0 }},
    Matrix3<FP> {{ 0, -1, 0 },  { 1, 0, 0 },  { 0, 0, 0 }}
};
/// @brief Definition of the so3 generators for double.
inline const srl::vector<Matrix3d> kSo3d { kSo3<double> };
/// @brief Definition of the so3 generators for float.
inline const srl::vector<Matrix3f> kSo3f { kSo3<float> };

}    // namespace srl

#endif    // SRL_CONSTANTS_HPP
