#include <srl/math/math.hpp>

namespace srl::math {

/// tell doxygen to skip from here \cond

//
// ----- Matrix functions
template auto isPositiveDefinite(const Matrix3f &matrix) -> bool;
template auto isPositiveDefinite(const Matrix3d &matrix) -> bool;

template auto isInertia(const Matrix3f &inertia) -> bool;
template auto isInertia(const Matrix3d &inertia) -> bool;

//
// - Lie group functions
template auto rotation(const float &angle, const Vector3f &normed_axis) -> Matrix3f;
template auto rotation(const double &angle, const Vector3d &normed_axis) -> Matrix3d;

template auto rotation(const Vector3f &rot_vec) -> Matrix3f;
template auto rotation(const Vector3d &rot_vec) -> Matrix3d;

template auto adjoint(const Matrix3f &element, const Matrix3f &rotation) -> Matrix3f;
template auto adjoint(const Matrix3d &element, const Matrix3d &rotation) -> Matrix3d;

template auto adjoint(const Vector3f &diagonal, const Matrix3f &rotation) -> Matrix3f;
template auto adjoint(const Vector3d &diagonal, const Matrix3d &rotation) -> Matrix3d;

template auto so3element(const Vector3f &vec) -> Matrix3f;
template auto so3element(const Vector3d &vec) -> Matrix3d;

/// tell doxygen to skip untill here \endcond

}    // namespace srl::math
