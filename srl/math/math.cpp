#include <srl/math/math.hpp>

namespace srl::math {

/// tell doxygen to skip from here \cond

//
// ----- Matrix functions
template auto isPositiveDefinite(const srl::Matrix3f &matrix) -> bool;
template auto isPositiveDefinite(const srl::Matrix3d &matrix) -> bool;

template auto isInertia(const srl::Matrix3f &inertia) -> bool;
template auto isInertia(const srl::Matrix3d &inertia) -> bool;

//
// - Lie group functions
template auto rotation(const float &angle, const srl::Vector3f &normed_axis) -> srl::Matrix3f;
template auto rotation(const double &angle, const srl::Vector3d &normed_axis) -> srl::Matrix3d;

template auto rotation(const srl::Vector3f &rot_vec) -> srl::Matrix3f;
template auto rotation(const srl::Vector3d &rot_vec) -> srl::Matrix3d;

template auto adjoint(const srl::Matrix3f &element, const srl::Matrix3f &rotation) -> srl::Matrix3f;
template auto adjoint(const srl::Matrix3d &element, const srl::Matrix3d &rotation) -> srl::Matrix3d;

template auto adjoint(const srl::Vector3f &diagonal, const srl::Matrix3f &rotation) -> srl::Matrix3f;
template auto adjoint(const srl::Vector3d &diagonal, const srl::Matrix3d &rotation) -> srl::Matrix3d;

template auto so3element(const srl::Vector3f &vec) -> srl::Matrix3f;
template auto so3element(const srl::Vector3d &vec) -> srl::Matrix3d;

/// tell doxygen to skip untill here \endcond

}    // namespace srl::math
