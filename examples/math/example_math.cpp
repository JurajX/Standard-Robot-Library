#include <srl/math/math.hpp>

#include <srl/common/constants.hpp>
#include <srl/common/types.hpp>

#include <fmt/format.h>

#include <iostream>

auto main(int /*argc*/, char ** /*argv*/) -> int
{
    fmt::print("----- Start of the example.\n");

    fmt::print("\n----- ----- Example for eq.\n");
    {    // NOLINTBEGIN
        /// [example_eq]
        bool res1 { srl::math::eq(0.0, 0.01, 0., 0.1) };
        bool res2 { srl::math::eq(0.0, 0.01, 0., 0.001) };
        fmt::print("The numbers 0.0 and 0.01 are equal depending on the precision:\n - res1 is {},\n - res2 is {}.\n", res1, res2);
        /// [example_eq]
        /// [example_eq_output]
        /*
        The numbers 0.0 and 0.01 are equal depending on the precision:
         - res1 is true,
         - res2 is false.
        */
        /// [example_eq_output]
    }    // NOLINTEND

    fmt::print("\n----- ----- Example for matEq.\n");
    {    // NOLINTBEGIN
        /// [example_matEq]
        const srl::uint dim { 10 };
        srl::MatrixXd a { srl::MatrixXd::Zero(dim, dim) };
        srl::MatrixXd b { srl::MatrixXd::Zero(dim, dim) };
        b.array() = 0.001;    // norm is 0.01
        bool res1 { srl::math::matEq(a, b, 0., 0.1) };
        bool res2 { srl::math::matEq(a, b, 0., 0.001) };
        fmt::print("The matrices a and b are equal depending on the precision:\n - res1 is {},\n - res2 is {}.\n", res1, res2);
        /// [example_matEq]
        /// [example_matEq_output]
        /*
        The matrices a and b are equal depending on the precision:
         - res1 is true,
         - res2 is false.
        */
        /// [example_matEq_output]
    }    // NOLINTEND

    fmt::print("\n----- ----- Example for isPositiveDefinite.\n");
    {
        /// [example_isPositiveDefinite]
        bool res1 { srl::math::isPositiveDefinite<srl::Matrix3d>(srl::kMatrixId3d) };
        bool res2 { srl::math::isPositiveDefinite<srl::Matrix3d>(-srl::kMatrixId3d) };
        fmt::print("The check for positive definitness gives:\n - res1 is {},\n - res2 is {}.\n", res1, res2);
        /// [example_isPositiveDefinite]
        /// [example_isPositiveDefinite_output]
        /*
        The check for positive definitness gives:
         - res1 is true,
         - res2 is false.
        */
        /// [example_isPositiveDefinite_output]
    }

    fmt::print("\n----- ----- Example for svd.\n");
    {
        /// [example_svd]
        const srl::Matrix3d mat {
            {1, 2, 3},
            {4, 5, 6},
            {7, 8, 9}
        };
        auto [U, D, V] { srl::math::svd(mat) };
        std::cout << "Performing U*D*V.transpose() should give back the original matrix:\n" << U * D.asDiagonal() * V.transpose() << '\n';
        /// [example_svd]
        /// [example_svd_output]
        /*
        Performing U*D*V.transpose() should give back the original matrix:
        1 2 3
        4 5 6
        7 8 9
        */
        /// [example_svd_output]
    }

    fmt::print("\n----- ----- Example for isInertia.\n");
    {
        /// [example_isInertia]
        const srl::Matrix3d mat {
            {1, 0, 0},
            {0, 2, 0},
            {0, 0, 4}
        };
        bool res1 { srl::math::isInertia(srl::kMatrixId3d) };
        bool res2 { srl::math::isInertia(mat) };
        fmt::print("The check for inertia gives:\n - res1 is {},\n - res2 is {}.\n", res1, res2);
        /// [example_isInertia]
        /// [example_isInertia_output]
        /*
        The check for positive definitness gives:
         - res1 is true,
         - res2 is false.
        */
        /// [example_isInertia_output]
    }

    fmt::print("\n----- ----- Example for rotation1.\n");
    {
        /// [example_rotation1]
        auto res { srl::math::rotation<double>(srl::kPId / 2, srl::Vector3d::UnitZ()) };
        std::cout << "The rotation matrix for rotation by pi/2 around z-axis is:\n" << res << '\n';
        /// [example_rotation1]
        /// [example_rotation1_output]
        /*
        The rotation matrix for rotation by pi/2 around z-axis is:
                 -1 -1.22465e-16            0
        1.22465e-16           -1            0
                  0            0            1
        */
        /// [example_rotation1_output]
    }

    fmt::print("\n----- ----- Example for rotation2.\n");
    {
        /// [example_rotation2]
        srl::Vector3d vec { (srl::kPId / 2) * srl::Vector3d::UnitZ() };
        auto res { srl::math::rotation<double>(vec) };
        std::cout << "The rotation matrix for rotation by pi/2 around z-axis is:\n" << res << '\n';
        /// [example_rotation2]
        /// [example_rotation2_output]
        /*
        The rotation matrix for rotation by pi/2 around z-axis is:
                 -1 -1.22465e-16            0
        1.22465e-16           -1            0
                  0            0            1
        */
        /// [example_rotation2_output]
    }

    fmt::print("\n----- ----- Example for adjoint1.\n");
    {
        /// [example_adjoint1]
        const srl::Matrix3d mat {
            {1, 0, 0},
            {0, 2, 0},
            {0, 0, 4}
        };
        const srl::Matrix3d rot {
            {0, -1, 0},
            {1,  0, 0},
            {0,  0, 1}
        };

        auto res { srl::math::adjoint<double>(mat, rot) };
        std::cout << "The matrix resulting from applying adjoint transformation is:\n" << res << '\n';
        /// [example_adjoint1]
        /// [example_adjoint1_output]
        /*
        The matrix resulting from applying adjoint transformation is:
        2 0 0
        0 1 0
        0 0 4
        */
        /// [example_adjoint1_output]
    }

    fmt::print("\n----- ----- Example for adjoint2.\n");
    {
        /// [example_adjoint2]
        const srl::Vector3d vec { 1, 2, 4 };
        const srl::Matrix3d rot {
            {0, -1, 0},
            {1,  0, 0},
            {0,  0, 1}
        };

        auto res { srl::math::adjoint<double>(vec, rot) };
        std::cout << "The matrix resulting from applying adjoint transformation is:\n" << res << '\n';
        /// [example_adjoint2]
        /// [example_adjoint2_output]
        /*
        The matrix resulting from applying adjoint transformation is:
        2 0 0
        0 1 0
        0 0 4
        */
        /// [example_adjoint2_output]
    }

    fmt::print("\n----- ----- Example for so3element.\n");
    {
        /// [example_so3element]
        const srl::Vector3d vec { 1, 2, 3 };
        auto res { srl::math::so3element<double>(vec) };
        std::cout << "The so3 element is:\n" << res << '\n';
        /// [example_so3element]
        /// [example_so3element_output]
        /*
        The so3 element is:
         0 -3  2
         3  0 -1
        -2  1  0
        */
        /// [example_so3element_output]
    }

    fmt::print("----- End of the example.\n");

    return 0;
}
