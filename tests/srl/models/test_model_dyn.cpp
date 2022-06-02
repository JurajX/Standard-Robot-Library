#include <gtest/gtest.h>

#include <srl/models/model.hpp>

#include <srl/utils/data_gen.hpp>

#include <srl/concepts.hpp>
#include <srl/constants.hpp>
#include <srl/types.hpp>

#include <cmath>
#include <tuple>

#include "test_params.hpp"

//
// =============== =============== Type Tests
using Indexables = ::testing::Types<srl::VectorXd, srl::vector<double>>;

template<srl::concepts::indexable Cont>
class TestDynamicsType : public ::testing::Test
{
protected:
    using FP = double;
    srl::QuatVec3<FP> basePose { srl::Quat<FP>::Identity(), srl::Vector3<FP>::Zero() };
    srl::Vector3<FP> gravityDirection { 0, 0, -1 };
    srl::model::RobotModel<FP> model { chain::jointRotAxes<FP>,
                                       chain::frameCoos<FP>,
                                       chain::masses<FP>,
                                       chain::linkCoM<FP>,
                                       chain::principalInertias<FP>,
                                       chain::rotationOfPrincipalAxes<FP>,
                                       chain::damping<FP>,
                                       basePose,
                                       gravityDirection,
                                       true };

    TestDynamicsType();
    Cont cont;
    Cont contBad;
};
TYPED_TEST_SUITE(TestDynamicsType, Indexables, );

template<srl::concepts::indexable Cont>
TestDynamicsType<Cont>::TestDynamicsType()
{
    if constexpr (srl::concepts::matrix<Cont>) {    // check if Cont fulfils the matrix concept
        cont = Cont {
            {0., 0., 0., 0., 0., 0., 0., 0.}
        };
        contBad = Cont {
            {0., 0., 0., 0., 0., 0., 0., 0., 0.}
        };
    } else {
        cont    = Cont { 0., 0., 0., 0., 0., 0., 0., 0. };
        contBad = Cont { 0., 0., 0., 0., 0., 0., 0., 0., 0. };
    }
}

TYPED_TEST(TestDynamicsType, InputType_fk)
{
    // Testing argument acceptation of dynamics functions.
    EXPECT_NO_THROW(this->model.getMassMatrixAndPotEnergy(this->cont));
    EXPECT_NO_THROW(this->model.getLagrangian(this->cont, this->cont));
    EXPECT_NO_THROW(this->model.getEomParams(this->cont));
    EXPECT_NO_THROW(this->model.getMotorTorque(this->cont, this->cont, this->cont));
    EXPECT_NO_THROW(this->model.getAngularAcceleration(this->cont, this->cont, this->cont));
#ifndef NDEBUG
    EXPECT_THROW(this->model.getMassMatrixAndPotEnergy(this->contBad), std::invalid_argument);
    EXPECT_THROW(this->model.getLagrangian(this->contBad, this->cont), std::invalid_argument);
    EXPECT_THROW(this->model.getLagrangian(this->cont, this->contBad), std::invalid_argument);
    EXPECT_THROW(this->model.getEomParams(this->contBad), std::invalid_argument);
    EXPECT_THROW(this->model.getMotorTorque(this->contBad, this->cont, this->cont), std::invalid_argument);
    EXPECT_THROW(this->model.getMotorTorque(this->cont, this->contBad, this->cont), std::invalid_argument);
    EXPECT_THROW(this->model.getMotorTorque(this->cont, this->cont, this->contBad), std::invalid_argument);
    EXPECT_THROW(this->model.getAngularAcceleration(this->contBad, this->cont, this->cont), std::invalid_argument);
    EXPECT_THROW(this->model.getAngularAcceleration(this->cont, this->contBad, this->cont), std::invalid_argument);
    EXPECT_THROW(this->model.getAngularAcceleration(this->cont, this->cont, this->contBad), std::invalid_argument);
#endif
}

//
// =============== =============== Test Lagrangian Calculation
using FPtypes = ::testing::Types<float, double>;

//
// --------------- Single Pendulum
template<srl::concepts::floating_point FP>
class TestSinglePendulum : public ::testing::Test
{
protected:
    FP rTol { sizeof(FP) == sizeof(double) ? static_cast<FP>(1e-5) : static_cast<FP>(1e-3) };
    FP aTol { sizeof(FP) == sizeof(double) ? static_cast<FP>(1e-8) : static_cast<FP>(1e-3) };

    srl::model::RobotModel<FP> model {
        pendulum::single::jointRotAxes<FP>, pendulum::single::frameCoos<FP>,         pendulum::single::masses<FP>,
        pendulum::single::linkCoM<FP>,      pendulum::single::principalInertias<FP>, pendulum::single::rotationOfPrincipalAxes<FP>,
        pendulum::single::damping<FP>
    };

    srl::uint nJoints { this->model.getNumberOfJoints() };
#ifndef NDEBUG
    size_t size { 1'000 };
#else
    size_t size { 10'000 };
#endif
    srl::vector<srl::VectorX<FP>> qSet { srl::utils::getRandVectors<FP>(size, nJoints, -srl::kPI<FP>, srl::kPI<FP>) };
    srl::vector<srl::VectorX<FP>> dqSet { srl::utils::getRandVectors<FP>(size, nJoints, -srl::kPI<FP>, srl::kPI<FP>) };

    srl::Vector3<FP> com { model.getLinkCentresOfMasses().col(0) };
    srl::Vector3<FP> rotAxis { model.getJointRotationAxes().col(0) };

    FP half { 0.5 };
    FP mass { model.getLinkMasses()[0] };
    FP projectedInertia { rotAxis.transpose() * model.getLinkInertiaTensors()[0] * rotAxis };
};
TYPED_TEST_SUITE(TestSinglePendulum, FPtypes, );

TYPED_TEST(TestSinglePendulum, PhysicalAroundCom)
{
    // Test Lagrangian calculation on single pendulum rotating around CoM.
    using FP = TypeParam;
    this->model.setLinkCentresOfMasses(srl::Matrix3X<FP>::Zero(srl::kDim, this->nJoints));    // rotate around CoM -> set CoM to zeros;
    for (size_t idx = 0; idx < this->size; idx += 1) {
        auto &q { this->qSet[idx] };
        auto &dq { this->dqSet[idx] };
        FP T { this->half * this->projectedInertia * dq[0] * dq[0] };
        FP returned { this->model.getLagrangian(q, dq) };
        EXPECT_TRUE(srl::math::eq(T, returned, this->rTol, this->aTol));    // rotation around CoM => no potential energy
    }
}

TYPED_TEST(TestSinglePendulum, Physical)
{
    // Test Lagrangian calculation on single pendulum not rotating around CoM.
    using FP = TypeParam;
    for (size_t idx = 0; idx < this->size; idx += 1) {
        auto &q { this->qSet[idx] };
        auto &dq { this->dqSet[idx] };

        FP Trot { this->half * this->projectedInertia * dq[0] * dq[0] };
        FP vel { this->com.norm() * dq[0] };
        FP Ttran { this->half * this->mass * vel * vel };

        FP rotatedCom { this->com.norm() * (-1) * std::cos(q[0]) };
        FP V { this->mass * srl::kG<FP> * rotatedCom };

        FP expected { (Trot + Ttran) - V };
        FP returned { this->model.getLagrangian(q, dq) };
        EXPECT_TRUE(srl::math::eq(expected, returned, this->rTol, this->aTol));
    }
}

//
// --------------- Double Pendulum
template<srl::concepts::floating_point FP>
class TestDoublePendulum : public ::testing::Test
{
protected:
    FP rTol { sizeof(FP) == sizeof(double) ? static_cast<FP>(1e-5) : static_cast<FP>(1e-3) };
    FP aTol { sizeof(FP) == sizeof(double) ? static_cast<FP>(1e-8) : static_cast<FP>(1e-3) };

    srl::model::RobotModel<FP> model { pendulum::dual::jointRotAxesPlanar<FP>,
                                       pendulum::dual::frameCoos<FP>,
                                       pendulum::dual::masses<FP>,
                                       pendulum::dual::linkCoM<FP>,
                                       pendulum::dual::principalInertias<FP>,
                                       pendulum::dual::rotationOfPrincipalAxes<FP>,
                                       pendulum::dual::damping<FP> };

    srl::uint nJoints { this->model.getNumberOfJoints() };
#ifndef NDEBUG
    size_t size { 1'000 };
#else
    size_t size { 10'000 };
#endif
    srl::vector<srl::VectorX<FP>> qSet { srl::utils::getRandVectors<FP>(size, nJoints, -srl::kPI<FP>, srl::kPI<FP>) };
    srl::vector<srl::VectorX<FP>> dqSet { srl::utils::getRandVectors<FP>(size, nJoints, -srl::kPI<FP>, srl::kPI<FP>) };

    srl::vector<srl::Vector3<FP>> com { model.getLinkCentresOfMasses().col(0), model.getLinkCentresOfMasses().col(1) };
    srl::vector<srl::Vector3<FP>> rotAxes { model.getJointRotationAxes().col(0), model.getJointRotationAxes().col(1) };
    srl::vector<srl::Vector3<FP>> frameCoos { model.getLinkLengths().col(1), model.getLinkLengths().col(2) };

    FP half { 0.5 };
    srl::vector<FP> mass { model.getLinkMasses()[0], model.getLinkMasses()[1] };
    srl::vector<FP> projectedInertias { rotAxes[0].transpose() * model.getLinkInertiaTensors()[0] * rotAxes[0],
                                        rotAxes[1].transpose() * model.getLinkInertiaTensors()[1] * rotAxes[1] };
    auto setNonPlanarRotAxes() -> void;
};
TYPED_TEST_SUITE(TestDoublePendulum, FPtypes, );

template<srl::concepts::floating_point FP>
auto TestDoublePendulum<FP>::setNonPlanarRotAxes() -> void
{
    model.setJointRotationAxes(pendulum::dual::jointRotAxesNonPlanar<FP>);
    rotAxes           = { model.getJointRotationAxes().col(0), model.getJointRotationAxes().col(1) };
    projectedInertias = { rotAxes[0].transpose() * model.getLinkInertiaTensors()[0] * rotAxes[0],
                          rotAxes[1].transpose() * model.getLinkInertiaTensors()[1] * rotAxes[1] };
}

TYPED_TEST(TestDoublePendulum, PhysicalPlanar)
{
    // Test Lagrangian calculation on planar double pendulum.
    using FP = TypeParam;
    FP fCooNorm { this->frameCoos[0].norm() };
    FP com0norm { this->com[0].norm() };
    FP com1norm { this->com[1].norm() };

    for (size_t idx = 0; idx < this->size; idx += 1) {
        auto &q { this->qSet[idx] };
        auto &dq { this->dqSet[idx] };

        FP V1 { this->mass[0] * srl::kG<FP> * com0norm * (-std::cos(q[0])) };
        FP V2_1 { this->mass[1] * srl::kG<FP> * fCooNorm * (-std::cos(q[0])) };
        FP V2_2 { this->mass[1] * srl::kG<FP> * com1norm * (-std::cos(q[0] + q[1])) };

        FP T1 { this->half * (this->projectedInertias[0] * dq[0] * dq[0] + this->projectedInertias[1] * (dq[0] + dq[1]) * (dq[0] + dq[1])) };
        FP tmp1 { this->mass[1] * (fCooNorm * fCooNorm + com1norm * com1norm + 2 * fCooNorm * com1norm * std::cos(q[1]))
                  + this->mass[0] * com0norm * com0norm };
        FP tmp2 { this->mass[1] * com1norm * com1norm };
        FP tmp3 { tmp2 + this->mass[1] * fCooNorm * com1norm * std::cos(q[1]) };
        FP T2 { this->half * (tmp1 * dq[0] * dq[0] + tmp2 * dq[1] * dq[1] + 2 * tmp3 * dq[0] * dq[1]) };

        FP expected { (T1 + T2) - (V1 + V2_1 + V2_2) };
        FP returned { this->model.getLagrangian(q, dq) };
        EXPECT_TRUE(srl::math::eq(expected, returned, this->rTol, this->aTol));
    }
}

TYPED_TEST(TestDoublePendulum, PhysicalNonPlanar)
{
    // Test Lagrangian calculation on non-planar double pendulum.
    this->setNonPlanarRotAxes();
    using FP = TypeParam;
    FP fCooNorm { this->frameCoos[0].norm() };
    FP com0norm { this->com[0].norm() };
    FP com1norm { this->com[1].norm() };

    srl::vector<srl::Matrix3<FP>> inertias { this->model.getLinkInertiaTensors()[0], this->model.getLinkInertiaTensors()[1] };
    srl::MatrixX<FP> rotProjInertias { srl::MatrixX<FP>::Zero(2, 2) };
    srl::Matrix3<FP> tmp;
    FP tmp1, tmp2;

    for (size_t idx = 0; idx < this->size; idx += 1) {
        auto &q { this->qSet[idx] };
        auto &dq { this->dqSet[idx] };

        FP V1 { this->mass[0] * srl::kG<FP> * com0norm * (-std::cos(q[0])) };
        FP V2 { this->mass[1] * srl::kG<FP> * fCooNorm * (-std::cos(q[0])) };
        FP V3 { this->mass[1] * srl::kG<FP> * com1norm * (-std::cos(q[0]) * std::cos(q[1])) };

        srl::Matrix3<FP> R1 { srl::math::rotation<FP>(q[0], this->rotAxes[0]) };
        srl::Matrix3<FP> R2 { srl::math::rotation<FP>(q[1], this->rotAxes[1]) };
        srl::Matrix3<FP> R12 { R1 * R2 };
        tmp                   = inertias[0] + R12 * inertias[1] * R12.transpose();
        rotProjInertias(0, 0) = this->rotAxes[0].transpose() * tmp * this->rotAxes[0];
        tmp                   = R2 * inertias[1] * R12.transpose();
        rotProjInertias(1, 0) = this->rotAxes[1].transpose() * tmp * this->rotAxes[0];
        rotProjInertias(0, 1) = rotProjInertias(1, 0);
        rotProjInertias(1, 1) = this->rotAxes[1].transpose() * inertias[1] * this->rotAxes[1];

        FP T1 { this->half * dq.transpose() * rotProjInertias * dq };
        tmp1 = this->mass[0] * com0norm * com0norm;
        tmp1 += this->mass[1] * (fCooNorm + com1norm * std::cos(q[1])) * (fCooNorm + com1norm * std::cos(q[1]));
        tmp2 = this->mass[1] * com1norm * com1norm;
        FP T2 { this->half * (tmp1 * dq[0] * dq[0] + tmp2 * dq[1] * dq[1]) };

        FP expected { (T1 + T2) - (V1 + V2 + V3) };
        FP returned { this->model.getLagrangian(q, dq) };

        EXPECT_TRUE(srl::math::eq(expected, returned, this->rTol, this->aTol));
    }
}

//
// --------------- Quadruple Pendulum
// TODO

//
// =============== =============== Test Derivative
class TestDerivatives : public ::testing::Test
{
protected:
    using FP = double;
    srl::QuatVec3<FP> basePose { srl::Quat<FP>::Identity(), srl::Vector3<FP>::Zero() };
    srl::Vector3<FP> gravityDirection { 0, 0, -1 };
    srl::model::RobotModel<FP> model { chain::jointRotAxes<FP>,
                                       chain::frameCoos<FP>,
                                       chain::masses<FP>,
                                       chain::linkCoM<FP>,
                                       chain::principalInertias<FP>,
                                       chain::rotationOfPrincipalAxes<FP>,
                                       chain::damping<FP>,
                                       basePose,
                                       gravityDirection,
                                       true };

    srl::Map<const srl::VectorX<FP>> damping { chain::damping<FP>.data(), static_cast<srl::uint>(chain::damping<FP>.size()) };
#ifndef NDEBUG
    size_t size { 10 };
#else
    size_t size { 1'000 };
#endif
    srl::uint nJoints { this->model.getNumberOfJoints() };
    srl::vector<srl::VectorX<FP>> qSet { srl::utils::getRandVectors<FP>(size, nJoints, -srl::kPI<FP>, srl::kPI<FP>) };
    FP delta { 1e-5 };
    FP aTol { 1e-5 };
    FP rTol { 1e-3 };
};

TEST_F(TestDerivatives, TestMassMatrix)
{
    // Test numerically the calculation of mass matrix derivatives (getEomParams).
    for (size_t idx = 0; idx < size; idx += 1) {
        auto &q { qSet[idx] };
        auto mm { std::get<0>(model.getEomParams(q)) };
        srl::vector<srl::MatrixX<FP>> actual;
        std::transform(mm.begin() + 1, mm.end(), std::back_inserter(actual), [](auto &m) -> srl::MatrixX<FP> { return m; });
        for (srl::uint iJoint = 0; iJoint < nJoints; iJoint += 1) {
            srl::VectorX<FP> h { q };
            h[iJoint] += delta;
            srl::MatrixX<FP> expected { (std::get<0>(model.getMassMatrixAndPotEnergy(h)) - std::get<0>(model.getMassMatrixAndPotEnergy(q))) / delta };
            EXPECT_TRUE(srl::math::matEq(actual[iJoint], expected, rTol, aTol));
        }
    }
}

TEST_F(TestDerivatives, TestPotEnergy)
{
    // Test numerically the calculation of potential energy derivatives (getEomParams).
    delta = 1e-7;
    rTol  = 5e-2;
    for (size_t idx = 0; idx < size; idx += 1) {
        auto &q { qSet[idx] };
        srl::VectorX<FP> actual { std::get<2>(model.getEomParams(q)).tail(nJoints) };
        for (srl::uint iJoint = 0; iJoint < nJoints; iJoint += 1) {
            srl::VectorX<FP> h { q };
            h[iJoint] += delta;
            FP expected { (std::get<1>(model.getMassMatrixAndPotEnergy(h)) - std::get<1>(model.getMassMatrixAndPotEnergy(q))) / delta };
            EXPECT_TRUE(srl::math::eq(actual[iJoint], expected, rTol, aTol));
        }
    }
}

TEST_F(TestDerivatives, TestChristoffelSymbols)
{
    // Test the calculation of Christoffel symbols (getEomParams).
    for (size_t idx = 0; idx < size; idx += 1) {
        auto &q { qSet[idx] };
        auto eomParams { model.getEomParams(q) };
        auto mm { std::get<0>(eomParams) };
        auto chs { std::get<1>(eomParams) };
        for (srl::uint i = 0; i < nJoints; i += 1) {
            for (srl::uint j = 0; j < nJoints; j += 1) {
                for (srl::uint k = 0; k < nJoints; k += 1) {
                    FP expected_ijk { 0.5 * (mm[1 + k](i, j) + mm[1 + j](i, k) - mm[1 + i](j, k)) };
                    EXPECT_TRUE(srl::math::eq(chs[i](j, k), expected_ijk, rTol, aTol));
                }
            }
        }
    }
}

TEST_F(TestDerivatives, TestMotorTorques)
{
    // Test the calculation of motor torques (getAngularAcceleration).
    for (size_t idx = 0; idx < size; idx += 1) {
        auto &q { qSet[idx] };
        srl::VectorX<FP> actual { model.getMotorTorque(q, q, q) };
        auto [mm, chs, pe] { model.getEomParams(q) };
        srl::VectorX<FP> expected { mm[0] * q + pe.tail(nJoints) + q.cwiseProduct(damping) };
        for (srl::uint iJoint = 0; iJoint < nJoints; iJoint += 1) {
            expected[iJoint] += q.transpose() * chs[iJoint] * q;
        }
        EXPECT_TRUE(srl::math::matEq(actual, expected, rTol, aTol));
    }
}

TEST_F(TestDerivatives, TestAngularAccelerations)
{
    // Test the calculation of angular accelerations (getAngularAcceleration).
    for (size_t idx = 0; idx < size; idx += 1) {
        auto &q { qSet[idx] };
        srl::VectorX<FP> actual { model.getAngularAcceleration(q, q, q) };
        auto [mm, chs, pe] { model.getEomParams(q) };
        srl::VectorX<FP> expected { q - pe.tail(nJoints) - q.cwiseProduct(damping) };
        for (srl::uint iJoint = 0; iJoint < nJoints; iJoint += 1) {
            expected[iJoint] -= q.transpose() * chs[iJoint] * q;
        }
        expected = mm[0].inverse() * expected.eval();
        EXPECT_TRUE(srl::math::matEq(actual, expected, rTol, aTol));
    }
}

//
// =============== =============== Main
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}