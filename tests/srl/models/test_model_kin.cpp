#include <gtest/gtest.h>

#include <srl/models/model.hpp>

#include <srl/concepts.hpp>
#include <srl/constants.hpp>
#include <srl/types.hpp>

#include <stdexcept>
#include <utility>

class TestKinematics : public ::testing::Test
{
protected:
    TestKinematics();
    const static inline srl::uint kJoints { 3 };
    const static inline srl::uint kLinks { kJoints + 1 };
    const static inline srl::Matrix3Xd jRotAxes {
        {0, 0, 1},
        {1, 1, 0},
        {0, 0, 0}
    };
    const static inline srl::Matrix3Xd lLengths {
        {0, 0, 1, 0},
        {0, 0, 1, 0},
        {0, 0, 1, 1}
    };

    const static inline srl::AAxisd baseRot { srl::kPId / 3, srl::Vector3d::UnitZ() };
    const static inline srl::Vector3d baseTran { 1, 2, 3 };
    const static inline srl::QuatVec3d baseQV3 { baseRot, baseTran };

    static inline srl::model::RobotModeld model { kJoints };
};

TestKinematics::TestKinematics()
{
    model.setJointRotationAxes(jRotAxes);
    model.setLinkLengths(lLengths);
    model.setBasePose(baseQV3);
}

//
// =============== =============== Type Tests
using Indexables = ::testing::Types<srl::VectorXd, srl::vector<double>>;

template<srl::concepts::indexable Cont>
class TestKinematicsType : public TestKinematics
{
protected:
    TestKinematicsType();
    Cont jAngles;
    Cont jAnglesBad;
};
TYPED_TEST_SUITE(TestKinematicsType, Indexables);

template<srl::concepts::indexable Cont>
TestKinematicsType<Cont>::TestKinematicsType()
{
    if constexpr (srl::concepts::matrix<Cont>) {    // check if Cont fulfils the matrix concept
        jAngles = Cont {
            {0., 0., 0.}
        };
        jAnglesBad = Cont {
            {0., 0., 0., 0.}
        };
    } else {
        jAngles    = Cont { 0., 0., 0. };
        jAnglesBad = Cont { 0., 0., 0., 0. };
    }
}

TYPED_TEST(TestKinematicsType, InputType_fk)
{
    // Testing argument acceptation of fk and fkQuat
    EXPECT_NO_THROW(this->model.fkQuat(this->jAngles));
    EXPECT_NO_THROW(this->model.fk(this->jAngles));
    EXPECT_NO_THROW(this->model.fkAllQuat(this->jAngles));
    EXPECT_NO_THROW(this->model.fkAll(this->jAngles));
#ifndef NDEBUG
    EXPECT_THROW(this->model.fkQuat(this->jAnglesBad), std::invalid_argument);
    EXPECT_THROW(this->model.fk(this->jAnglesBad), std::invalid_argument);
    EXPECT_THROW(this->model.fkAllQuat(this->jAnglesBad), std::invalid_argument);
    EXPECT_THROW(this->model.fkAll(this->jAnglesBad), std::invalid_argument);
#endif
}

//
// =============== =============== Calculation Tests
auto jointAngles { ::testing::Values(
  srl::vector { 0., 0., 0. },
  srl::vector { srl::kPId / 6, srl::kPId / 6, srl::kPId / 6 },
  srl::vector { srl::kPId / 4, srl::kPId / 4, srl::kPId / 4 },
  srl::vector { srl::kPId / 2, srl::kPId / 2, srl::kPId / 2 },
  srl::vector { srl::kPId, srl::kPId, srl::kPId }) };

class TestKinematicsCalc
    : public TestKinematics
    , public ::testing::WithParamInterface<srl::vector<double>>
{
protected:
    auto calculateFk(srl::vector<double> &angles, srl::Matrix3d rot = srl::kMatrixId3d, srl::Vector3d tran = srl::Vector3d::Zero())
      -> std::pair<srl::Matrix3d, srl::Vector3d>;
    auto calculateFkAll(srl::vector<double> &angles, srl::Matrix3d rot = srl::kMatrixId3d, srl::Vector3d tran = srl::Vector3d::Zero())
      -> srl::vector<std::pair<srl::Matrix3d, srl::Vector3d>>;
};

auto TestKinematicsCalc::calculateFk(srl::vector<double> &angles, srl::Matrix3d rot, srl::Vector3d tran) -> std::pair<srl::Matrix3d, srl::Vector3d>
{
    for (srl::uint iJoint = 0; iJoint < kJoints; iJoint += 1) {
        tran = tran + (rot * lLengths.col(iJoint));
        rot  = rot * srl::AAxisd(angles[iJoint], jRotAxes.col(iJoint));
    }
    tran = tran + (rot * lLengths.col(kJoints));
    return { rot, tran };
}

auto TestKinematicsCalc::calculateFkAll(srl::vector<double> &angles, srl::Matrix3d rot, srl::Vector3d tran)
  -> srl::vector<std::pair<srl::Matrix3d, srl::Vector3d>>
{
    srl::vector<std::pair<srl::Matrix3d, srl::Vector3d>> fkAll;
    fkAll.emplace_back(rot, tran);
    for (srl::uint iJoint = 0; iJoint < kJoints; iJoint += 1) {
        tran = tran + (rot * lLengths.col(iJoint));
        rot  = rot * srl::AAxisd(angles[iJoint], jRotAxes.col(iJoint));
        fkAll.emplace_back(rot, tran);
    }
    tran = tran + (rot * lLengths.col(kJoints));
    fkAll.emplace_back(rot, tran);
    return fkAll;
}

// --------------- simple FK
TEST_P(TestKinematicsCalc, fkRobotBase)
{
    // Testing fk and fkQuat w.r.t the robot base
    auto angles { GetParam() };
    auto [expectedRot, expectedTran] { calculateFk(angles) };

    auto actualFk { model.fkQuat(angles, true) };
    auto actualTf { model.fk(angles, true) };

    EXPECT_TRUE(actualFk.first.toRotationMatrix().isApprox(expectedRot, srl::kEpsd));
    EXPECT_TRUE(actualFk.second.isApprox(expectedTran, srl::kEpsd));

    EXPECT_TRUE(actualTf.linear().isApprox(expectedRot, srl::kEpsd));
    EXPECT_TRUE(actualTf.translation().isApprox(expectedTran, srl::kEpsd));
}
INSTANTIATE_TEST_SUITE_P(Test_fkRobotBase, TestKinematicsCalc, jointAngles);

TEST_P(TestKinematicsCalc, fkBase)
{
    // Testing fk and fkQuat w.r.t the the origin
    auto angles { GetParam() };
    auto [expectedRot, expectedTran] { calculateFk(angles, baseRot.toRotationMatrix(), baseTran) };

    auto actualFk { model.fkQuat(angles, false) };
    auto actualTf { model.fk(angles, false) };

    EXPECT_TRUE(actualFk.first.toRotationMatrix().isApprox(expectedRot, srl::kEpsd));
    EXPECT_TRUE(actualFk.second.isApprox(expectedTran, srl::kEpsd));

    EXPECT_TRUE(actualTf.linear().isApprox(expectedRot, srl::kEpsd));
    EXPECT_TRUE(actualTf.translation().isApprox(expectedTran, srl::kEpsd));
}
INSTANTIATE_TEST_SUITE_P(Test_fkBase, TestKinematicsCalc, jointAngles);

// --------------- all FK
TEST_P(TestKinematicsCalc, fkAllBase)
{
    // Testing fkAll
    auto angles { GetParam() };

    auto expected { calculateFkAll(angles) };

    auto actualQuats { model.fkAllQuat(angles, true) };
    auto actualTfs { model.fkAll(angles, true) };
    EXPECT_EQ(actualQuats.size(), expected.size());
    EXPECT_EQ(actualTfs.size(), expected.size());

    for (srl::uint idx = 0; idx < expected.size(); idx += 1) {
        EXPECT_TRUE(actualQuats[idx].first.toRotationMatrix().isApprox(expected[idx].first, srl::kEpsd));
        EXPECT_TRUE(actualQuats[idx].second.isApprox(expected[idx].second, srl::kEpsd));
        EXPECT_TRUE(actualTfs[idx].linear().isApprox(expected[idx].first, srl::kEpsd));
        EXPECT_TRUE(actualTfs[idx].translation().isApprox(expected[idx].second, srl::kEpsd));
    }
}
INSTANTIATE_TEST_SUITE_P(Test_fkAllBase, TestKinematicsCalc, jointAngles);

TEST_P(TestKinematicsCalc, fkAll)
{
    // Testing fkAll
    auto angles { GetParam() };

    auto expected { calculateFkAll(angles, baseRot.toRotationMatrix(), baseTran) };

    auto actualQuats { model.fkAllQuat(angles) };
    auto actualTfs { model.fkAll(angles) };
    EXPECT_EQ(actualQuats.size(), expected.size());
    EXPECT_EQ(actualTfs.size(), expected.size());

    for (srl::uint idx = 0; idx < expected.size(); idx += 1) {
        EXPECT_TRUE(actualQuats[idx].first.toRotationMatrix().isApprox(expected[idx].first, srl::kEpsd));
        EXPECT_TRUE(actualQuats[idx].second.isApprox(expected[idx].second, srl::kEpsd));
        EXPECT_TRUE(actualTfs[idx].linear().isApprox(expected[idx].first, srl::kEpsd));
        EXPECT_TRUE(actualTfs[idx].translation().isApprox(expected[idx].second, srl::kEpsd));
    }
}
INSTANTIATE_TEST_SUITE_P(Test_fkAll, TestKinematicsCalc, jointAngles);

//
// =============== =============== Main
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}