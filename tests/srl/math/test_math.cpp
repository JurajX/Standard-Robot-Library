#include <gtest/gtest.h>

#include <srl/math/math.hpp>

#include <srl/common/common.hpp>

#include <Eigen/Geometry>

#include <limits>

using FPtypes = ::testing::Types<float, double>;

// ----- ----- ----- ----- ----- ----- ----- floating point functions
template<srl::concepts::floating_point FP>
class TestEq : public ::testing::Test
{
protected:
    const static inline FP zero { 0 };
    const static inline FP precision1 { static_cast<FP>(0.1) };
    const static inline FP precision2 { static_cast<FP>(0.001) };
};
TYPED_TEST_SUITE(TestEq, FPtypes, );

TYPED_TEST(TestEq, Eq)    // NOLINT
{
    // Test the floating point equal (eq) fn.
    // set-up
    using FP = TypeParam;
    const FP val1 { this->zero };
    const FP val2 { static_cast<FP>(0.01) };
    // run
    // check
    EXPECT_TRUE(srl::math::eq<FP>(val1, val2, this->zero, this->precision1));
    EXPECT_FALSE(srl::math::eq<FP>(val1, val2, this->zero, this->precision2));
}

// ----- ----- ----- ----- ----- ----- ----- matrix functions
template<srl::concepts::floating_point FP>
class TestMatEq : public ::testing::Test
{
protected:
    const static inline FP zero { 0 };
    const static inline FP precision1 { static_cast<FP>(0.1) };
    const static inline FP precision2 { static_cast<FP>(0.001) };
};
TYPED_TEST_SUITE(TestMatEq, FPtypes, );

TYPED_TEST(TestMatEq, MatEq)    // NOLINT
{
    // Test the matrix equal (matEq) fn.
    // set-up
    using FP = TypeParam;
    const uint32_t dim { 10 };
    const srl::types::MatrixX<FP> mat1 { srl::types::MatrixX<FP>::Zero(dim, dim) };
    srl::types::MatrixX<FP> mat2 { srl::types::MatrixX<FP>::Zero(dim, dim) };
    mat2.array() = this->precision2;    // norm is sqrt(dim^2 * precision2^2)
    // run
    // check
    EXPECT_TRUE(srl::math::matEq(mat1, mat2, this->zero, this->precision1));
    EXPECT_FALSE(srl::math::matEq(mat1, mat2, this->zero, this->precision2));
}

// ----- ----- ----- ----- ----- ----- ----- Lie group functions
template<srl::concepts::floating_point FP>
class TestRotation : public ::testing::Test
{
protected:
    const static inline FP piHalf { srl::constants::kPI<FP> / 2 };
    const static inline srl::types::Matrix3<FP> rotByPiHalfAroundX {
        {1, 0,  0},
        {0, 0, -1},
        {0, 1,  0}
    };
    const static inline srl::types::Matrix3<FP> rotByPiHalfAroundY {
        { 0, 0, 1},
        { 0, 1, 0},
        {-1, 0, 0}
    };
    const static inline srl::types::Matrix3<FP> rotByPiHalfAroundZ {
        {0, -1, 0},
        {1,  0, 0},
        {0,  0, 1}
    };
};
TYPED_TEST_SUITE(TestRotation, FPtypes, );

TYPED_TEST(TestRotation, AngleAxisInputs)    // NOLINT
{
    // Test the rotation fn with angle and axis arguments.
    // set-up
    // run
    const auto actualX { srl::math::rotation<TypeParam>(this->piHalf, srl::types::Vector3<TypeParam>::UnitX()) };
    const auto actualY { srl::math::rotation<TypeParam>(this->piHalf, srl::types::Vector3<TypeParam>::UnitY()) };
    const auto actualZ { srl::math::rotation<TypeParam>(this->piHalf, srl::types::Vector3<TypeParam>::UnitZ()) };
    // check
    EXPECT_TRUE(actualX.isApprox(this->rotByPiHalfAroundX, srl::constants::kEps<TypeParam>));
    EXPECT_TRUE(actualY.isApprox(this->rotByPiHalfAroundY, srl::constants::kEps<TypeParam>));
    EXPECT_TRUE(actualZ.isApprox(this->rotByPiHalfAroundZ, srl::constants::kEps<TypeParam>));
}

TYPED_TEST(TestRotation, AngleAxisInputsZero)    // NOLINT
{
    // Test the rotation fn with angle and axis arguments.
    // set-up
    const auto eps { std::numeric_limits<TypeParam>::epsilon() };
    // run
    const auto actual1 { srl::math::rotation<TypeParam>(0, srl::types::Vector3<TypeParam>::UnitX()) };
    const auto actual2 { srl::math::rotation<TypeParam>(eps, srl::types::Vector3<TypeParam>::UnitY()) };
    // check
    EXPECT_TRUE(actual1.isApprox(srl::constants::kMatrixId3<TypeParam>, srl::constants::kEps<TypeParam>));
    EXPECT_TRUE(actual2.isApprox(srl::constants::kMatrixId3<TypeParam>, srl::constants::kEps<TypeParam>));
}

TYPED_TEST(TestRotation, RotationVectorInputs)    // NOLINT
{
    // Test the rotation fn with rotation vector argument.
    // set-up
    // run
    const auto actualX { srl::math::rotation<TypeParam>(this->piHalf * srl::types::Vector3<TypeParam>::UnitX()) };
    const auto actualY { srl::math::rotation<TypeParam>(this->piHalf * srl::types::Vector3<TypeParam>::UnitY()) };
    const auto actualZ { srl::math::rotation<TypeParam>(this->piHalf * srl::types::Vector3<TypeParam>::UnitZ()) };
    // check
    EXPECT_TRUE(actualX.isApprox(this->rotByPiHalfAroundX, srl::constants::kEps<TypeParam>));
    EXPECT_TRUE(actualY.isApprox(this->rotByPiHalfAroundY, srl::constants::kEps<TypeParam>));
    EXPECT_TRUE(actualZ.isApprox(this->rotByPiHalfAroundZ, srl::constants::kEps<TypeParam>));
}

TYPED_TEST(TestRotation, RotationVectorInputsZero)    // NOLINT
{
    // Test the rotation fn with angle and axis arguments.
    // set-up
    const auto eps { std::numeric_limits<TypeParam>::epsilon() };
    // run
    const auto actual1 { srl::math::rotation<TypeParam>(0 * srl::types::Vector3<TypeParam>::UnitX()) };
    const auto actual2 { srl::math::rotation<TypeParam>(eps * srl::types::Vector3<TypeParam>::UnitY()) };
    // check
    EXPECT_TRUE(actual1.isApprox(srl::constants::kMatrixId3<TypeParam>, srl::constants::kEps<TypeParam>));
    EXPECT_TRUE(actual2.isApprox(srl::constants::kMatrixId3<TypeParam>, srl::constants::kEps<TypeParam>));
}

template<srl::concepts::floating_point FP>
class TestAdjoint : public TestRotation<FP>
{ };
TYPED_TEST_SUITE(TestAdjoint, FPtypes, );

TYPED_TEST(TestAdjoint, MatrixMatrixInputs)    // NOLINT
{
    // Test the adjoint fn with two matrix agruments.
    // set-up
    // run
    const auto actualX { srl::math::adjoint<TypeParam>(srl::constants::kMatrixId3<TypeParam>, this->rotByPiHalfAroundX) };
    const auto actualY { srl::math::adjoint<TypeParam>(srl::constants::kMatrixId3<TypeParam>, this->rotByPiHalfAroundY) };
    const auto actualZ { srl::math::adjoint<TypeParam>(srl::constants::kMatrixId3<TypeParam>, this->rotByPiHalfAroundZ) };
    // check
    EXPECT_TRUE(actualX.isApprox(srl::constants::kMatrixId3<TypeParam>, srl::constants::kEps<TypeParam>));
    EXPECT_TRUE(actualY.isApprox(srl::constants::kMatrixId3<TypeParam>, srl::constants::kEps<TypeParam>));
    EXPECT_TRUE(actualZ.isApprox(srl::constants::kMatrixId3<TypeParam>, srl::constants::kEps<TypeParam>));
}

TYPED_TEST(TestAdjoint, VectorMatrixInputs)    // NOLINT
{
    // Test the adjoint fn with two vector agruments.
    // set-up
    const srl::types::Vector3<TypeParam> input { 1, 2, 3 };
    const srl::types::Matrix3<TypeParam> expectedX { (this->rotByPiHalfAroundX.cwiseAbs() * input).asDiagonal() };
    const srl::types::Matrix3<TypeParam> expectedY { (this->rotByPiHalfAroundY.cwiseAbs() * input).asDiagonal() };
    const srl::types::Matrix3<TypeParam> expectedZ { (this->rotByPiHalfAroundZ.cwiseAbs() * input).asDiagonal() };
    // run
    const auto actualX { srl::math::adjoint<TypeParam>(input, this->rotByPiHalfAroundX) };
    const auto actualY { srl::math::adjoint<TypeParam>(input, this->rotByPiHalfAroundY) };
    const auto actualZ { srl::math::adjoint<TypeParam>(input, this->rotByPiHalfAroundZ) };
    // check
    EXPECT_TRUE(actualX.isApprox(expectedX, srl::constants::kEps<TypeParam>));
    EXPECT_TRUE(actualY.isApprox(expectedY, srl::constants::kEps<TypeParam>));
    EXPECT_TRUE(actualZ.isApprox(expectedZ, srl::constants::kEps<TypeParam>));
}

template<srl::concepts::floating_point FP>
class TestSo3element : public ::testing::Test
{ };
TYPED_TEST_SUITE(TestSo3element, FPtypes, );

TYPED_TEST(TestSo3element, TestCreation)    // NOLINT
{
    // Test the so3element fn.
    const auto actualX { srl::math::so3element(srl::types::Vector3<TypeParam> { 1, 0, 0 }) };
    const auto actualY { srl::math::so3element(srl::types::Vector3<TypeParam> { 0, 1, 0 }) };
    const auto actualZ { srl::math::so3element(srl::types::Vector3<TypeParam> { 0, 0, 1 }) };
    const auto actualXYZ { srl::math::so3element(srl::types::Vector3<TypeParam> { 1, 1, 1 }) };

    const auto expectedXYZ { srl::constants::kSo3<TypeParam>[0] + srl::constants::kSo3<TypeParam>[1] + srl::constants::kSo3<TypeParam>[2] };
    EXPECT_TRUE(actualX.isApprox(srl::constants::kSo3<TypeParam>[0], srl::constants::kEps<TypeParam>));
    EXPECT_TRUE(actualY.isApprox(srl::constants::kSo3<TypeParam>[1], srl::constants::kEps<TypeParam>));
    EXPECT_TRUE(actualZ.isApprox(srl::constants::kSo3<TypeParam>[2], srl::constants::kEps<TypeParam>));
    EXPECT_TRUE(actualXYZ.isApprox(expectedXYZ, srl::constants::kEps<TypeParam>));
}

// ----- ----- ----- ----- ----- ----- ----- Matrix functions
template<srl::concepts::floating_point FP>
class TestInertia : public TestRotation<FP>
{
protected:
    const static inline srl::types::Matrix3<FP> rot { srl::types::AAxis<FP>(static_cast<FP>(1.2), srl::types::Vector3<FP> { 1, 2, 3 }.normalized()) };
    const static inline Eigen::DiagonalMatrix<FP, 3> goodPrincipal { 3, 4, 5 };
    const static inline Eigen::DiagonalMatrix<FP, 3> badPrincipal { 1, 4, 5 };
    const static inline Eigen::DiagonalMatrix<FP, 3> notPrincipal { 10, -4, 5 };
    const static inline srl::types::Matrix3<FP> inertia1 { rot * goodPrincipal * rot.transpose() };
    const static inline srl::types::Matrix3<FP> inertia2 { TestRotation<FP>::rotByPiHalfAroundX * goodPrincipal
                                                           * TestRotation<FP>::rotByPiHalfAroundX.transpose() };

    const static inline srl::types::Matrix3<FP> onlyPosDef1 { rot * badPrincipal * rot.transpose() };
    const static inline srl::types::Matrix3<FP> onlyPosDef2 { TestRotation<FP>::rotByPiHalfAroundZ * badPrincipal
                                                              * TestRotation<FP>::rotByPiHalfAroundZ.transpose() };

    const static inline srl::types::Matrix3<FP> notPosDef1 { TestRotation<FP>::rotByPiHalfAroundX * goodPrincipal
                                                             * TestRotation<FP>::rotByPiHalfAroundZ.transpose() };
    const static inline srl::types::Matrix3<FP> notPosDef2 { rot * badPrincipal * TestRotation<FP>::rotByPiHalfAroundZ.transpose() };
    const static inline srl::types::Matrix3<FP> notPosDef3 { rot * notPrincipal * rot.transpose() };
};
TYPED_TEST_SUITE(TestInertia, FPtypes, );

TYPED_TEST(TestInertia, TestIsPositiveDefinite)    // NOLINT
{
    // Test the isPositiveDefinite fn.
    EXPECT_TRUE(srl::math::isPositiveDefinite(this->inertia1));
    EXPECT_TRUE(srl::math::isPositiveDefinite(this->inertia2));
    EXPECT_TRUE(srl::math::isPositiveDefinite(this->onlyPosDef1));
    EXPECT_TRUE(srl::math::isPositiveDefinite(this->onlyPosDef2));
    EXPECT_FALSE(srl::math::isPositiveDefinite(this->notPosDef1));
    EXPECT_FALSE(srl::math::isPositiveDefinite(this->notPosDef2));
    EXPECT_FALSE(srl::math::isPositiveDefinite(this->notPosDef3));
}

TYPED_TEST(TestInertia, TestIsInertia)    // NOLINT
{
    // Test the isInertia fn.
    EXPECT_TRUE(srl::math::isInertia(this->inertia1));
    EXPECT_TRUE(srl::math::isInertia(this->inertia2));
    EXPECT_FALSE(srl::math::isInertia(this->onlyPosDef1));
    EXPECT_FALSE(srl::math::isInertia(this->onlyPosDef2));
    EXPECT_FALSE(srl::math::isInertia(this->notPosDef1));
    EXPECT_FALSE(srl::math::isInertia(this->notPosDef2));
    EXPECT_FALSE(srl::math::isInertia(this->notPosDef3));
}

template<srl::concepts::floating_point FP>
class TestSVD : public ::testing::Test
{
protected:
    const static inline srl::types::Matrix3<FP> mat {
        {1, 2, 3},
        {4, 5, 6},
        {7, 8, 9}
    };
};
TYPED_TEST_SUITE(TestSVD, FPtypes, );

TYPED_TEST(TestSVD, TestSVD)    // NOLINT
{
    // Test the singular value decomposition (svd) fn.
    // set-up
    // run
    const auto [Umat, Dmat, Vmat] { srl::math::svd(this->mat) };
    const srl::types::Matrix3<TypeParam> actual { Umat * Dmat.asDiagonal() * Vmat.transpose() };
    // check
    EXPECT_TRUE(actual.isApprox(this->mat, srl::constants::kEps<TypeParam>));
}

auto main(int argc, char **argv) -> int
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
