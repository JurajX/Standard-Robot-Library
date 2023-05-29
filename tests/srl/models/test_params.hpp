#ifndef SRL_MODELS_TEST_PARAMS_HPP
#define SRL_MODELS_TEST_PARAMS_HPP
#pragma once

#include <srl/common/common.hpp>

namespace pendulum {
// --------------- Single Pendulum
namespace single {

inline constexpr size_t numLinks { 1 };

template<srl::concepts::floating_point FP>
inline const srl::types::Matrix3X<FP> jointRotAxes { { 0. }, { 1. }, { 0. } };

template<srl::concepts::floating_point FP>
inline const srl::types::Matrix3X<FP> frameCoos {
    {0.,   0.},
    {0.,   0.},
    {1., -20.}
};

template<srl::concepts::floating_point FP>
inline const srl::types::Matrix3X<FP> linkCoM { { 0. }, { 0. }, { -10. } };

template<srl::concepts::floating_point FP>
inline const std::vector<FP> masses { 10. };

template<srl::concepts::floating_point FP>
inline const std::vector<srl::types::Matrix3<FP>> inertias {
    srl::types::Vector3<FP> {3., 4., 5.}
     .asDiagonal()
};

template<srl::concepts::floating_point FP>
inline const std::vector<FP> damping { 0. };

}    // namespace single

// --------------- Double Pendulum
namespace dual {

inline constexpr size_t numLinks { 2 };

template<srl::concepts::floating_point FP>
inline const srl::types::Matrix3X<FP> jointRotAxesPlanar {
    {0., 0.},
    {1., 1.},
    {0., 0.}
};

template<srl::concepts::floating_point FP>
inline const srl::types::Matrix3X<FP> jointRotAxesNonPlanar {
    {0., 1.},
    {1., 0.},
    {0., 0.}
};

template<srl::concepts::floating_point FP>
inline const srl::types::Matrix3X<FP> frameCoos {
    {0.,  0.,  0.},
    {0.,  0.,  0.},
    {0., -2., -4.}
};

template<srl::concepts::floating_point FP>
inline const std::vector<FP> masses { 10., 5. };

template<srl::concepts::floating_point FP>
inline const srl::types::Matrix3X<FP> linkCoM {
    { 0.,  0.},
    { 0.,  0.},
    {-1., -2.}
};

template<srl::concepts::floating_point FP>
inline const std::vector<srl::types::Matrix3<FP>> inertias {
    srl::types::Vector3<FP> {3., 4., 5.}
     .asDiagonal(),
    srl::types::Vector3<FP> {2., 3., 2.}
     .asDiagonal()
};

template<srl::concepts::floating_point FP>
inline const std::vector<FP> damping { 0., 0. };

}    // namespace dual

// --------------- Quadruple Pendulum
namespace quadruple {

inline constexpr size_t numLinks { 4 };

template<srl::concepts::floating_point FP>
inline const srl::types::Matrix3X<FP> jointRotAxesX {
    {1., 1.},
    {0., 0.},
    {0., 0.}
};

template<srl::concepts::floating_point FP>
inline const srl::types::Matrix3X<FP> jointRotAxesY {
    {0., 0.},
    {1., 1.},
    {0., 0.}
};

template<srl::concepts::floating_point FP>
inline const srl::types::Matrix3X<FP> jointRotAxesZ {
    {0., 0.},
    {0., 0.},
    {1., 1.}
};

template<srl::concepts::floating_point FP>
inline const srl::types::Matrix3X<FP> frameCoos {
    {0.,  0., 1., 0.,  0.},
    {0.,  0., 0., 1.,  0.},
    {0., -1., 0., 0., -1.}
};

template<srl::concepts::floating_point FP>
inline const std::vector<FP> masses { 1., 2., 3., 4. };

template<srl::concepts::floating_point FP>
inline const srl::types::Matrix3X<FP> linkCoM {
    {0.0,  0.0, 0.0,  0.0},
    {0.5,  0.0, 0.5,  0.0},
    {0.0, -0.5, 0.0, -0.5}
};

template<srl::concepts::floating_point FP>
inline const std::vector<srl::types::Matrix3<FP>> inertias {
    srl::types::Vector3<FP> {1., 1., 1.}
     .asDiagonal(),
    srl::types::Vector3<FP> {2., 2., 2.}
     .asDiagonal(),
    srl::types::Vector3<FP> {3., 3., 3.}
     .asDiagonal(),
    srl::types::Vector3<FP> {4., 4., 4.}
     .asDiagonal()
};

template<srl::concepts::floating_point FP>
inline const std::vector<FP> damping { 0., 0., 0., 0. };

}    // namespace quadruple

}    // namespace pendulum

namespace chain {
inline constexpr size_t numLinks { 8 };

template<srl::concepts::floating_point FP>
inline const srl::types::Matrix3X<FP> jointRotAxes {
    {1., 1., 1., 1., 1., 1., 1., 1.},
    {2., 2., 2., 2., 2., 2., 2., 2.},
    {3., 3., 3., 3., 3., 3., 3., 3.}
};

template<srl::concepts::floating_point FP>
inline const srl::types::Matrix3X<FP> frameCoos {
    {0., 1., 1., 1., 1., 1., 1., 1., 1.},
    {0., 1., 1., 1., 1., 1., 1., 1., 1.},
    {0., 1., 1., 1., 1., 1., 1., 1., 1.}
};

template<srl::concepts::floating_point FP>
inline const std::vector<FP> masses { 1., 2., 3., 4., 5., 6., 7., 8. };

template<srl::concepts::floating_point FP>
inline const srl::types::Matrix3X<FP> linkCoM {
    {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5},
    {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5},
    {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5}
};

template<srl::concepts::floating_point FP>
inline const std::vector<srl::types::Matrix3<FP>> inertias {
    srl::types::Vector3<FP> {1., 1., 1.}
     .asDiagonal(), srl::types::Vector3<FP> {2., 2., 2.}
     .asDiagonal(),
    srl::types::Vector3<FP> {3., 3., 3.}
     .asDiagonal(), srl::types::Vector3<FP> {4., 4., 4.}
     .asDiagonal(),
    srl::types::Vector3<FP> {5., 5., 5.}
     .asDiagonal(), srl::types::Vector3<FP> {6., 6., 6.}
     .asDiagonal(),
    srl::types::Vector3<FP> {7., 7., 7.}
     .asDiagonal(), srl::types::Vector3<FP> {8., 8., 8.}
     .asDiagonal()
};

template<srl::concepts::floating_point FP>
inline const std::vector<FP> damping { 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08 };

}    // namespace chain

#endif    // SRL_MODELS_TEST_PARAMS_HPP
