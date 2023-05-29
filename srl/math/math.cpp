#include <srl/math/math.hpp>

namespace srl::math {

/// tell doxygen to skip from here \cond

//
// ----- Matrix functions
template auto isPositiveDefinite(const types::Matrix3f &matrix) -> bool;
template auto isPositiveDefinite(const types::Matrix3d &matrix) -> bool;

template auto isInertia(const types::Matrix3f &inertia) -> bool;
template auto isInertia(const types::Matrix3d &inertia) -> bool;

//
// - Lie group functions
template auto rotation(const float &angle, const types::Vector3f &normed_axis) -> types::Matrix3f;
template auto rotation(const double &angle, const types::Vector3d &normed_axis) -> types::Matrix3d;

template auto rotation(const types::Vector3f &rot_vec) -> types::Matrix3f;
template auto rotation(const types::Vector3d &rot_vec) -> types::Matrix3d;

template auto adjoint(const types::Matrix3f &element, const types::Matrix3f &rotation) -> types::Matrix3f;
template auto adjoint(const types::Matrix3d &element, const types::Matrix3d &rotation) -> types::Matrix3d;

template auto adjoint(const types::Vector3f &diagonal, const types::Matrix3f &rotation) -> types::Matrix3f;
template auto adjoint(const types::Vector3d &diagonal, const types::Matrix3d &rotation) -> types::Matrix3d;

template auto so3element(const types::Vector3f &vec) -> types::Matrix3f;
template auto so3element(const types::Vector3d &vec) -> types::Matrix3d;

/// tell doxygen to skip untill here \endcond

}    // namespace srl::math
