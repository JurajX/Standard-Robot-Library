#ifndef SRL_ROBOT_PARAMS_HPP
#define SRL_ROBOT_PARAMS_HPP
#pragma once

#include <srl/common/concepts.hpp>
#include <srl/common/constants.hpp>
#include <srl/common/types.hpp>

namespace srl::resources {
// panda parameters for testing
namespace panda {

inline constexpr size_t numLinks { 7 };

template<srl::concepts::floating_point FP>
inline const srl::Matrix3X<FP> jointRotAxes {
    {0., 0., 0.,  0., 0.,  0.,  0.},
    {0., 1., 0., -1., 0., -1.,  0.},
    {1., 0., 1.,  0., 1.,  0., -1.}
};

template<srl::concepts::floating_point FP>
inline const srl::Matrix3X<FP> frameCoos {
    {   0.,    0.,    0., 0.0825, -0.0825,    0., 0.0880,      0.},
    {   0.,    0.,    0.,     0.,      0.,    0.,     0.,      0.},
    {0.138, 0.192, 0.194,  0.122,   0.124, 0.260, -0.052, -0.0550}
};

template<srl::concepts::floating_point FP>
inline const srl::vector<FP> masses { 3.5, 3.5, 2.5, 2.5, 3.0, 2.0, 0.5 };

template<srl::concepts::floating_point FP>
inline const srl::Matrix3X<FP> linkCoM {
    {   0.,    0., 0.044, -0.0385,   0., 0.065,     0.},
    {-0.04,  0.04,  0.02,   -0.02, 0.04, -0.01,     0.},
    { 0.14, 0.052, 0.078,   0.044, 0.15,  0.01, -0.028}
};

template<srl::concepts::floating_point FP>
inline const srl::Matrix3X<FP> principalInertias {
    {0.0180, 0.0180, 0.0088, 0.0088, 0.0100, 0.0040, 0.0002},
    {0.0180, 0.0180, 0.0088, 0.0088, 0.0100, 0.0060, 0.0002},
    {0.0044, 0.0044, 0.0025, 0.0025, 0.0030, 0.0040, 0.0003}
};

template<srl::concepts::floating_point FP>
inline const srl::Matrix3X<FP> rotationOfPrincipalAxes {
    {0.349, 0.349,    0.,     0., -0.175, 0., 0.},
    {   0.,    0., 0.785, -0.785,     0., 0., 0.},
    {   0.,    0.,    0.,     0.,     0., 0., 0.}
};

template<srl::concepts::floating_point FP>
inline const srl::vector<FP> damping { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 };

template<srl::concepts::floating_point FP>
inline const srl::vector<FP> q_min { -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973 };
template<srl::concepts::floating_point FP>
inline const srl::vector<FP> q_max { 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973 };
template<srl::concepts::floating_point FP>
inline const srl::vector<FP> dq_min { -2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100 };
template<srl::concepts::floating_point FP>
inline const srl::vector<FP> dq_max { 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100 };
template<srl::concepts::floating_point FP>
inline const srl::vector<FP> ddq_min { -15, -7.5, -10, -12.5, -15, -20, -20 };
template<srl::concepts::floating_point FP>
inline const srl::vector<FP> ddq_max { 15, 7.5, 10, 12.5, 15, 20, 20 };
template<srl::concepts::floating_point FP>
inline const srl::vector<FP> tau_min { -87., -87., -87., -87., -12., -12., -12. };
template<srl::concepts::floating_point FP>
inline const srl::vector<FP> tau_max { 87., 87., 87., 87., 12., 12., 12. };

}    // namespace panda

}    // namespace srl::resources

#endif    // SRL_ROBOT_PARAMS_HPP
