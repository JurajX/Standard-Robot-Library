#include <gtest/gtest.h>

#include <srl/models/model.hpp>

#include <srl/common/common.hpp>

#include <stdexcept>

//
// =============== =============== Test Setters and Getters
using FPtypes = ::testing::Types<float, double>;

template<srl::concepts::floating_point FP>
class TestSettersAndGetters : public ::testing::Test
{
protected:
    const static inline srl::uint nJoints { 3 };
    const static inline srl::uint nJointsBad { 4 };
    const static inline srl::Vector3<FP> gravity1 { 0, 0, -1 * srl::kG<FP> };
    const static inline srl::Vector3<FP> gravity2 { 0, -1, 0 };
    const static inline srl::Matrix3X<FP> jRotAxes {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };
    const static inline srl::Matrix3X<FP> jRotAxesBad {
        {2, 0, 0},
        {0, 2, 0},
        {0, 0, 2}
    };
    const static inline srl::Matrix3X<FP> lLengths {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 1}
    };
    const static inline srl::VectorX<FP> lMasses {
        {1, 1, 1}
    };
    const static inline srl::VectorX<FP> lMassesBad1 {
        {-1, 1, 1}
    };
    const static inline srl::Matrix3X<FP> lCoM {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };
    const static inline srl::Matrix3<FP> inertia {
        {3, 0, 0},
        {0, 4, 0},
        {0, 0, 5}
    };
    const static inline srl::Matrix3<FP> inertiaBad {
        {-3, 0, 0},
        { 0, 4, 0},
        { 0, 0, 5}
    };

    const static inline srl::Matrix3X<FP> principals {
        {3, 3, 3},
        {4, 4, 4},
        {5, 5, 5}
    };
    const static inline srl::Matrix3X<FP> principalsBad {
        {3, 3},
        {4, 4},
        {5, 5}
    };
    const static inline srl::Matrix3X<FP> rots {
        {1, 0, 0},
        {0, 2, 0},
        {0, 0, 3}
    };

    const static inline srl::VectorX<FP> &damping { lMasses };
    const static inline srl::VectorX<FP> &dampingBad1 { lMassesBad1 };

    const static inline srl::AAxis<FP> baseRot { srl::kPI<FP> / 3, srl::Vector3<FP>::UnitZ() };
    const static inline srl::Vector3<FP> baseTran { 1, 2, 3 };
    const static inline srl::Transform3<FP> baseTf { srl::Transform3<FP>(srl::kTransformId3<FP>).rotate(baseRot).translate(baseTran) };
    const static inline srl::QuatVec3<FP> baseQV3 { baseRot, baseTran };

    srl::model::RobotModel<FP> model { nJoints };
    srl::model::RobotModel<FP> modelBad { nJointsBad };
};
TYPED_TEST_SUITE(TestSettersAndGetters, FPtypes, );

TYPED_TEST(TestSettersAndGetters, NumberOfJointsAndLinks)    // NOLINT
{
    // Testing setNumberOfJoints, getNumberOfLinks
    EXPECT_EQ(this->model.getNumberOfJoints(), this->nJoints);
    EXPECT_EQ(this->model.getNumberOfLinks(), this->nJoints + 1);
    EXPECT_EQ(this->modelBad.getNumberOfJoints(), this->nJointsBad);
    EXPECT_EQ(this->modelBad.getNumberOfLinks(), this->nJointsBad + 1);
}

TYPED_TEST(TestSettersAndGetters, JointRotationAxes)    // NOLINT
{
    // Testing setJointRotationAxes and getJointRotationAxes
    // normal setting
    this->model.setJointRotationAxes(this->jRotAxes);
    EXPECT_THROW(this->model.setJointRotationAxes(this->jRotAxesBad), std::invalid_argument);    // NOLINT
    EXPECT_EQ(this->model.getJointRotationAxes(), this->jRotAxes);
    // normalise = true
    this->model.setJointRotationAxes(this->jRotAxesBad, true);
    EXPECT_EQ(this->model.getJointRotationAxes(), this->jRotAxes);
    // incorrect size
    EXPECT_THROW(this->modelBad.setJointRotationAxes(this->jRotAxes), std::invalid_argument);    // NOLINT
}

TYPED_TEST(TestSettersAndGetters, LinkLengths)    // NOLINT
{
    // Testing setLinkLengths and getLinkLengths
    // normal setting
    this->model.setLinkLengths(this->lLengths);
    EXPECT_EQ(this->model.getLinkLengths(), this->lLengths);
    // incorrect size
    EXPECT_THROW(this->modelBad.setLinkLengths(this->lLengths), std::invalid_argument);    // NOLINT
}

TYPED_TEST(TestSettersAndGetters, LinkMasses)    // NOLINT
{
    // Testing setLinkMasses and getLinkMasses
    // normal setting
    this->model.setLinkMasses(this->lMasses);
    EXPECT_EQ(this->model.getLinkMasses(), this->lMasses);
    // incorrect size
    EXPECT_THROW(this->model.setLinkMasses(this->lMassesBad1), std::invalid_argument);    // NOLINT
    EXPECT_THROW(this->modelBad.setLinkMasses(this->lMasses), std::invalid_argument);     // NOLINT
}

TYPED_TEST(TestSettersAndGetters, LinkCentresOfMasses)    // NOLINT
{
    // Testing setLinkCentresOfMasses and getLinkCentresOfMasses
    // normal setting
    this->model.setLinkCentresOfMasses(this->lCoM);
    EXPECT_EQ(this->model.getLinkCentresOfMasses(), this->lCoM);
    // incorrect size
    EXPECT_THROW(this->modelBad.setLinkCentresOfMasses(this->lCoM), std::invalid_argument);    // NOLINT
}

TYPED_TEST(TestSettersAndGetters, LinkInertiaTensors)    // NOLINT
{
    // Testing setLinkInertiaTensors and getLinkInertiaTensors
    const srl::vector<srl::Matrix3<TypeParam>> inertias { this->inertia, this->inertia, this->inertia };
    const srl::vector<srl::Matrix3<TypeParam>> inertiasBad { this->inertia, this->inertia, this->inertiaBad };
    srl::vector<srl::Matrix3<TypeParam>> expected;
    for (srl::uint idx = 0; idx < this->nJoints; idx += 1) {
        const srl::Matrix3<TypeParam> rot { srl::math::rotation(srl::Vector3<TypeParam> { this->rots.col(idx) }) };
        expected.emplace_back(srl::math::adjoint(srl::Vector3<TypeParam> { this->principals.col(idx) }, rot));
    }
    // normal setting
    this->model.setLinkInertiaTensors(inertias);
    EXPECT_EQ(this->model.getLinkInertiaTensors(), inertias);
    this->model.setLinkInertiaTensors(this->principals, this->rots);
    EXPECT_EQ(this->model.getLinkInertiaTensors(), expected);
    // incorrect size
    EXPECT_THROW(this->model.setLinkInertiaTensors(inertiasBad), std::invalid_argument);                        // NOLINT
    EXPECT_THROW(this->model.setLinkInertiaTensors(this->principalsBad, this->rots), std::invalid_argument);    // NOLINT
    EXPECT_THROW(this->modelBad.setLinkInertiaTensors(inertias), std::invalid_argument);                        // NOLINT
    EXPECT_THROW(this->modelBad.setLinkInertiaTensors(this->principals, this->rots), std::invalid_argument);    // NOLINT
}

TYPED_TEST(TestSettersAndGetters, JointDampings)    // NOLINT
{
    // Testing setJointDampings and getJointDampings
    // normal setting
    this->model.setJointDampings(this->damping);
    EXPECT_EQ(this->model.getJointDampings(), this->damping);
    // incorrect size
    EXPECT_THROW(this->model.setJointDampings(this->dampingBad1), std::invalid_argument);    // NOLINT
    EXPECT_THROW(this->modelBad.setJointDampings(this->damping), std::invalid_argument);     // NOLINT
}

TYPED_TEST(TestSettersAndGetters, GravityAcc)    // NOLINT
{
    // Testing setGravityAcc and getGravityAcc
    // normal setting
    this->model.setGravityAcc(this->gravity1);
    EXPECT_EQ(this->model.getGravityAcc(), this->gravity1);
    // incorrect size
    this->model.setGravityAcc(this->gravity2);
    EXPECT_EQ(this->model.getGravityAcc(), this->gravity2 * srl::kG<TypeParam>);
}

TYPED_TEST(TestSettersAndGetters, BaseTransform)    // NOLINT
{
    // Testing setBaseTransform and getBaseTransform
    // normal setting
    this->model.setBaseTransform(this->baseTf);
    EXPECT_TRUE(this->model.getBaseTransform().isApprox(this->baseTf, srl::kEps<TypeParam>));
}

TYPED_TEST(TestSettersAndGetters, BasePose)    // NOLINT
{
    // Testing setBasePose and getBasePose
    // normal setting
    this->model.setBasePose(this->baseQV3);
    EXPECT_TRUE(this->model.getBasePose().first.isApprox(this->baseQV3.first, srl::kEps<TypeParam>));
    EXPECT_TRUE(this->model.getBasePose().second.isApprox(this->baseQV3.second, srl::kEps<TypeParam>));
}

auto main(int argc, char **argv) -> int
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}