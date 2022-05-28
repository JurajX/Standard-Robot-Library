#include <gtest/gtest.h>

#include <srl/math/math.hpp>

#include <srl/concepts.hpp>
#include <srl/constants.hpp>
#include <srl/types.hpp>

#include <Eigen/Geometry>

#include <limits>

using FPtypes = ::testing::Types<float, double>;

// ----- ----- ----- ----- ----- ----- ----- floating point functions
template<srl::concepts::floating_point FP>
class TestEq : public ::testing::Test
{ };
TYPED_TEST_SUITE(TestEq, FPtypes);

TYPED_TEST(TestEq, Eq)
{
    // Test the floating point equal (eq) fn.
    // set-up
    using FP = TypeParam;
    auto a { static_cast<FP>(0.0) };
    auto b { static_cast<FP>(0.01) };
    auto precision1 { static_cast<FP>(0.1) };
    auto precision2 { static_cast<FP>(0.001) };
    // run
    // check
    EXPECT_TRUE(srl::math::eq<FP>(a, b, 0, precision1));
    EXPECT_FALSE(srl::math::eq<FP>(a, b, 0, precision2));
}

// ----- ----- ----- ----- ----- ----- ----- matrix functions
template<srl::concepts::floating_point FP>
class TestMatEq : public ::testing::Test
{ };
TYPED_TEST_SUITE(TestMatEq, FPtypes);

TYPED_TEST(TestMatEq, MatEq)
{
    // Test the matrix equal (matEq) fn.
    // set-up
    using FP = TypeParam;
    srl::MatrixX<FP> a { srl::MatrixX<FP>::Zero(10, 10) };
    srl::MatrixX<FP> b { srl::MatrixX<FP>::Zero(10, 10) };
    b.array() = static_cast<FP>(0.001);    // norm is 0.01
    auto precision1 { static_cast<FP>(0.1) };
    auto precision2 { static_cast<FP>(0.001) };
    // run
    // check
    EXPECT_TRUE(srl::math::matEq(a, b, 0, precision1));
    EXPECT_FALSE(srl::math::matEq(a, b, 0, precision2));
}

// ----- ----- ----- ----- ----- ----- ----- Lie group functions
template<srl::concepts::floating_point FP>
class TestRotation : public ::testing::Test
{
protected:
    FP piHalf { srl::kPI<FP> / 2 };
    srl::Matrix3<FP> rotByPiHalfAroundX {
        {1, 0,  0},
        {0, 0, -1},
        {0, 1,  0}
    };
    srl::Matrix3<FP> rotByPiHalfAroundY {
        { 0, 0, 1},
        { 0, 1, 0},
        {-1, 0, 0}
    };
    srl::Matrix3<FP> rotByPiHalfAroundZ {
        {0, -1, 0},
        {1,  0, 0},
        {0,  0, 1}
    };
};
TYPED_TEST_SUITE(TestRotation, FPtypes);

TYPED_TEST(TestRotation, AngleAxisInputs)
{
    // Test the rotation fn with angle and axis arguments.
    // set-up
    // run
    auto actualX { srl::math::rotation<TypeParam>(this->piHalf, srl::Vector3<TypeParam>::UnitX()) };
    auto actualY { srl::math::rotation<TypeParam>(this->piHalf, srl::Vector3<TypeParam>::UnitY()) };
    auto actualZ { srl::math::rotation<TypeParam>(this->piHalf, srl::Vector3<TypeParam>::UnitZ()) };
    // check
    EXPECT_TRUE(actualX.isApprox(this->rotByPiHalfAroundX, srl::kEps<TypeParam>));
    EXPECT_TRUE(actualY.isApprox(this->rotByPiHalfAroundY, srl::kEps<TypeParam>));
    EXPECT_TRUE(actualZ.isApprox(this->rotByPiHalfAroundZ, srl::kEps<TypeParam>));
}

TYPED_TEST(TestRotation, AngleAxisInputsZero)
{
    // Test the rotation fn with angle and axis arguments.
    // set-up
    auto eps { std::numeric_limits<TypeParam>::epsilon() };
    // run
    auto actual1 { srl::math::rotation<TypeParam>(0, srl::Vector3<TypeParam>::UnitX()) };
    auto actual2 { srl::math::rotation<TypeParam>(eps, srl::Vector3<TypeParam>::UnitY()) };
    // check
    EXPECT_TRUE(actual1.isApprox(srl::kMatrixId3<TypeParam>, srl::kEps<TypeParam>));
    EXPECT_TRUE(actual2.isApprox(srl::kMatrixId3<TypeParam>, srl::kEps<TypeParam>));
}

TYPED_TEST(TestRotation, RotationVectorInputs)
{
    // Test the rotation fn with rotation vector argument.
    // set-up
    // run
    auto actualX { srl::math::rotation<TypeParam>(this->piHalf * srl::Vector3<TypeParam>::UnitX()) };
    auto actualY { srl::math::rotation<TypeParam>(this->piHalf * srl::Vector3<TypeParam>::UnitY()) };
    auto actualZ { srl::math::rotation<TypeParam>(this->piHalf * srl::Vector3<TypeParam>::UnitZ()) };
    // check
    EXPECT_TRUE(actualX.isApprox(this->rotByPiHalfAroundX, srl::kEps<TypeParam>));
    EXPECT_TRUE(actualY.isApprox(this->rotByPiHalfAroundY, srl::kEps<TypeParam>));
    EXPECT_TRUE(actualZ.isApprox(this->rotByPiHalfAroundZ, srl::kEps<TypeParam>));
}

TYPED_TEST(TestRotation, RotationVectorInputsZero)
{
    // Test the rotation fn with angle and axis arguments.
    // set-up
    auto eps { std::numeric_limits<TypeParam>::epsilon() };
    // run
    auto actual1 { srl::math::rotation<TypeParam>(0 * srl::Vector3<TypeParam>::UnitX()) };
    auto actual2 { srl::math::rotation<TypeParam>(eps * srl::Vector3<TypeParam>::UnitY()) };
    // check
    EXPECT_TRUE(actual1.isApprox(srl::kMatrixId3<TypeParam>, srl::kEps<TypeParam>));
    EXPECT_TRUE(actual2.isApprox(srl::kMatrixId3<TypeParam>, srl::kEps<TypeParam>));
}

template<srl::concepts::floating_point FP>
class TestAdjoint : public TestRotation<FP>
{ };
TYPED_TEST_SUITE(TestAdjoint, FPtypes);

TYPED_TEST(TestAdjoint, MatrixMatrixInputs)
{
    // Test the adjoint fn with two matrix agruments.
    // set-up
    // run
    auto actualX { srl::math::adjoint<TypeParam>(srl::kMatrixId3<TypeParam>, this->rotByPiHalfAroundX) };
    auto actualY { srl::math::adjoint<TypeParam>(srl::kMatrixId3<TypeParam>, this->rotByPiHalfAroundY) };
    auto actualZ { srl::math::adjoint<TypeParam>(srl::kMatrixId3<TypeParam>, this->rotByPiHalfAroundZ) };
    // check
    EXPECT_TRUE(actualX.isApprox(srl::kMatrixId3<TypeParam>, srl::kEps<TypeParam>));
    EXPECT_TRUE(actualY.isApprox(srl::kMatrixId3<TypeParam>, srl::kEps<TypeParam>));
    EXPECT_TRUE(actualZ.isApprox(srl::kMatrixId3<TypeParam>, srl::kEps<TypeParam>));
}

TYPED_TEST(TestAdjoint, VectorMatrixInputs)
{
    // Test the adjoint fn with two vector agruments.
    // set-up
    srl::Vector3<TypeParam> input { 1, 2, 3 };
    srl::Matrix3<TypeParam> expectedX { (this->rotByPiHalfAroundX.cwiseAbs() * input).asDiagonal() };
    srl::Matrix3<TypeParam> expectedY { (this->rotByPiHalfAroundY.cwiseAbs() * input).asDiagonal() };
    srl::Matrix3<TypeParam> expectedZ { (this->rotByPiHalfAroundZ.cwiseAbs() * input).asDiagonal() };
    // run
    auto actualX { srl::math::adjoint<TypeParam>(input, this->rotByPiHalfAroundX) };
    auto actualY { srl::math::adjoint<TypeParam>(input, this->rotByPiHalfAroundY) };
    auto actualZ { srl::math::adjoint<TypeParam>(input, this->rotByPiHalfAroundZ) };
    // check
    EXPECT_TRUE(actualX.isApprox(expectedX, srl::kEps<TypeParam>));
    EXPECT_TRUE(actualY.isApprox(expectedY, srl::kEps<TypeParam>));
    EXPECT_TRUE(actualZ.isApprox(expectedZ, srl::kEps<TypeParam>));
}

template<srl::concepts::floating_point FP>
class TestSo3element : public ::testing::Test
{ };
TYPED_TEST_SUITE(TestSo3element, FPtypes);

TYPED_TEST(TestSo3element, TestCreation)
{
    // Test the so3element fn.
    auto actualX { srl::math::so3element(srl::Vector3<TypeParam> { 1, 0, 0 }) };
    auto actualY { srl::math::so3element(srl::Vector3<TypeParam> { 0, 1, 0 }) };
    auto actualZ { srl::math::so3element(srl::Vector3<TypeParam> { 0, 0, 1 }) };
    auto actualXYZ { srl::math::so3element(srl::Vector3<TypeParam> { 1, 1, 1 }) };

    auto expectedXYZ { srl::kSo3<TypeParam>[0] + srl::kSo3<TypeParam>[1] + srl::kSo3<TypeParam>[2] };
    EXPECT_TRUE(actualX.isApprox(srl::kSo3<TypeParam>[0], srl::kEps<TypeParam>));
    EXPECT_TRUE(actualY.isApprox(srl::kSo3<TypeParam>[1], srl::kEps<TypeParam>));
    EXPECT_TRUE(actualZ.isApprox(srl::kSo3<TypeParam>[2], srl::kEps<TypeParam>));
    EXPECT_TRUE(actualXYZ.isApprox(expectedXYZ, srl::kEps<TypeParam>));
}

// ----- ----- ----- ----- ----- ----- ----- Matrix functions
template<srl::concepts::floating_point FP>
class TestInertia : public TestRotation<FP>
{
protected:
    srl::Matrix3<FP> rot { srl::AAxis<FP>(static_cast<FP>(1.2), srl::Vector3<FP> { 1, 2, 3 }.normalized()) };
    Eigen::DiagonalMatrix<FP, 3> goodPrincipal { 3, 4, 5 };
    Eigen::DiagonalMatrix<FP, 3> badPrincipal { 1, 4, 5 };
    Eigen::DiagonalMatrix<FP, 3> notPrincipal { 10, -4, 5 };
    srl::Matrix3<FP> inertia1 { rot * goodPrincipal * rot.transpose() };
    srl::Matrix3<FP> inertia2 { this->rotByPiHalfAroundX * goodPrincipal * this->rotByPiHalfAroundX.transpose() };

    srl::Matrix3<FP> onlyPosDef1 { rot * badPrincipal * rot.transpose() };
    srl::Matrix3<FP> onlyPosDef2 { this->rotByPiHalfAroundZ * badPrincipal * this->rotByPiHalfAroundZ.transpose() };

    srl::Matrix3<FP> notPosDef1 { this->rotByPiHalfAroundX * goodPrincipal * this->rotByPiHalfAroundZ.transpose() };
    srl::Matrix3<FP> notPosDef2 { rot * badPrincipal * this->rotByPiHalfAroundZ.transpose() };
    srl::Matrix3<FP> notPosDef3 { rot * notPrincipal * rot.transpose() };
};
TYPED_TEST_SUITE(TestInertia, FPtypes);

TYPED_TEST(TestInertia, TestIsPositiveDefinite)
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

TYPED_TEST(TestInertia, TestIsInertia)
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
    srl::Matrix3<FP> mat {
        {1, 2, 3},
        {4, 5, 6},
        {7, 8, 9}
    };
};
TYPED_TEST_SUITE(TestSVD, FPtypes);

TYPED_TEST(TestSVD, TestSVD)
{
    // Test the singular value decomposition (svd) fn.
    // set-up
    // run
    auto [U, D, V] { srl::math::svd(this->mat) };
    srl::Matrix3<TypeParam> actual { U * D.asDiagonal() * V.transpose() };
    // check
    EXPECT_TRUE(actual.isApprox(this->mat, srl::kEps<TypeParam>));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
