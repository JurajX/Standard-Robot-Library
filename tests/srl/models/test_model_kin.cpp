#include <gtest/gtest.h>

#include <srl/models/model.hpp>

#include <srl/common/common.hpp>

#include <stdexcept>
#include <utility>

class TestKinematics : public ::testing::Test
{
protected:
    TestKinematics();
    const static inline uint32_t kJoints { 3 };
    const static inline uint32_t kLinks { kJoints + 1 };
    const static inline srl::types::Matrix3Xd jRotAxes {
        {0, 0, 1},
        {1, 1, 0},
        {0, 0, 0}
    };
    const static inline srl::types::Matrix3Xd lLengths {
        {0, 0, 1, 0},
        {0, 0, 1, 0},
        {0, 0, 1, 1}
    };

    const static inline srl::types::AAxisd baseRot { srl::constants::kPId / 3, srl::types::Vector3d::UnitZ() };
    const static inline srl::types::Vector3d baseTran { 1, 2, 3 };
    const static inline srl::types::QuatVec3d baseQV3 { baseRot, baseTran };

    srl::model::RobotModel<double> model { kJoints };    // NOLINT
};

TestKinematics::TestKinematics()
{
    model.setJointRotationAxes(jRotAxes);
    model.setLinkLengths(lLengths);
    model.setBasePose(baseQV3);
}

//
// =============== =============== Type Tests
using Indexables = ::testing::Types<srl::types::VectorXd, std::vector<double>>;

template<srl::concepts::indexable Cont>
class TestKinematicsType : public TestKinematics
{
protected:
    TestKinematicsType();
    Cont jAngles;       // NOLINT
    Cont jAnglesBad;    // NOLINT
};
TYPED_TEST_SUITE(TestKinematicsType, Indexables, );

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

TYPED_TEST(TestKinematicsType, InputType_fk)    // NOLINT
{
    // Testing argument acceptation of fk and fkQuat
    EXPECT_NO_THROW(this->model.fkQuat(this->jAngles));       // NOLINT
    EXPECT_NO_THROW(this->model.fk(this->jAngles));           // NOLINT
    EXPECT_NO_THROW(this->model.fkAllQuat(this->jAngles));    // NOLINT
    EXPECT_NO_THROW(this->model.fkAll(this->jAngles));        // NOLINT
#ifndef NDEBUG
    EXPECT_THROW(this->model.fkQuat(this->jAnglesBad), std::invalid_argument);       // NOLINT
    EXPECT_THROW(this->model.fk(this->jAnglesBad), std::invalid_argument);           // NOLINT
    EXPECT_THROW(this->model.fkAllQuat(this->jAnglesBad), std::invalid_argument);    // NOLINT
    EXPECT_THROW(this->model.fkAll(this->jAnglesBad), std::invalid_argument);        // NOLINT
#endif
}

//
// =============== =============== Calculation Tests
const auto jointAngles { ::testing::Values(
  std::vector { 0., 0., 0. },
  std::vector { srl::constants::kPId / 6, srl::constants::kPId / 6, srl::constants::kPId / 6 },
  std::vector { srl::constants::kPId / 4, srl::constants::kPId / 4, srl::constants::kPId / 4 },
  std::vector { srl::constants::kPId / 2, srl::constants::kPId / 2, srl::constants::kPId / 2 },
  std::vector { srl::constants::kPId, srl::constants::kPId, srl::constants::kPId }) };

class TestKinematicsCalc
    : public TestKinematics
    , public ::testing::WithParamInterface<std::vector<double>>
{
protected:
    static auto calculateFk(
      std::vector<double> &angles,
      srl::types::Matrix3d rot  = srl::constants::kMatrixId3d,
      srl::types::Vector3d tran = srl::types::Vector3d::Zero()) -> std::pair<srl::types::Matrix3d, srl::types::Vector3d>;
    static auto calculateFkAll(
      std::vector<double> &angles,
      srl::types::Matrix3d rot  = srl::constants::kMatrixId3d,
      srl::types::Vector3d tran = srl::types::Vector3d::Zero()) -> std::vector<std::pair<srl::types::Matrix3d, srl::types::Vector3d>>;
};

auto TestKinematicsCalc::calculateFk(std::vector<double> &angles, srl::types::Matrix3d rot, srl::types::Vector3d tran)
  -> std::pair<srl::types::Matrix3d, srl::types::Vector3d>
{
    for (uint32_t iJoint = 0; iJoint < kJoints; iJoint += 1) {
        tran = tran + (rot * lLengths.col(iJoint));
        rot  = rot * srl::types::AAxisd(angles[iJoint], jRotAxes.col(iJoint));
    }
    tran = tran + (rot * lLengths.col(kJoints));
    return { rot, tran };
}

auto TestKinematicsCalc::calculateFkAll(std::vector<double> &angles, srl::types::Matrix3d rot, srl::types::Vector3d tran)
  -> std::vector<std::pair<srl::types::Matrix3d, srl::types::Vector3d>>
{
    std::vector<std::pair<srl::types::Matrix3d, srl::types::Vector3d>> fkAll;
    fkAll.emplace_back(rot, tran);
    for (uint32_t iJoint = 0; iJoint < kJoints; iJoint += 1) {
        tran = tran + (rot * lLengths.col(iJoint));
        rot  = rot * srl::types::AAxisd(angles[iJoint], jRotAxes.col(iJoint));
        fkAll.emplace_back(rot, tran);
    }
    tran = tran + (rot * lLengths.col(kJoints));
    fkAll.emplace_back(rot, tran);
    return fkAll;
}

// --------------- simple FK
TEST_P(TestKinematicsCalc, fkRobotBase)    // NOLINT
{
    // Testing fk and fkQuat w.r.t the robot base
    auto angles { GetParam() };
    auto [expectedRot, expectedTran] { calculateFk(angles) };

    auto actualFk { model.fkQuat(angles, true) };
    auto actualTf { model.fk(angles, true) };

    EXPECT_TRUE(actualFk.first.toRotationMatrix().isApprox(expectedRot, srl::constants::kEpsd));
    EXPECT_TRUE(actualFk.second.isApprox(expectedTran, srl::constants::kEpsd));

    EXPECT_TRUE(actualTf.linear().isApprox(expectedRot, srl::constants::kEpsd));
    EXPECT_TRUE(actualTf.translation().isApprox(expectedTran, srl::constants::kEpsd));
}
INSTANTIATE_TEST_SUITE_P(Test_fkRobotBase, TestKinematicsCalc, jointAngles);    // NOLINT

TEST_P(TestKinematicsCalc, fkBase)    // NOLINT
{
    // Testing fk and fkQuat w.r.t the the origin
    auto angles { GetParam() };
    auto [expectedRot, expectedTran] { calculateFk(angles, baseRot.toRotationMatrix(), baseTran) };

    auto actualFk { model.fkQuat(angles, false) };
    auto actualTf { model.fk(angles, false) };

    EXPECT_TRUE(actualFk.first.toRotationMatrix().isApprox(expectedRot, srl::constants::kEpsd));
    EXPECT_TRUE(actualFk.second.isApprox(expectedTran, srl::constants::kEpsd));

    EXPECT_TRUE(actualTf.linear().isApprox(expectedRot, srl::constants::kEpsd));
    EXPECT_TRUE(actualTf.translation().isApprox(expectedTran, srl::constants::kEpsd));
}
INSTANTIATE_TEST_SUITE_P(Test_fkBase, TestKinematicsCalc, jointAngles);    // NOLINT

// --------------- all FK
TEST_P(TestKinematicsCalc, fkAllBase)    // NOLINT
{
    // Testing fkAll
    auto angles { GetParam() };

    auto expected { calculateFkAll(angles) };

    auto actualQuats { model.fkAllQuat(angles, true) };
    auto actualTfs { model.fkAll(angles, true) };
    EXPECT_EQ(actualQuats.size(), expected.size());
    EXPECT_EQ(actualTfs.size(), expected.size());

    for (uint32_t idx = 0; idx < expected.size(); idx += 1) {
        EXPECT_TRUE(actualQuats[idx].first.toRotationMatrix().isApprox(expected[idx].first, srl::constants::kEpsd));
        EXPECT_TRUE(actualQuats[idx].second.isApprox(expected[idx].second, srl::constants::kEpsd));
        EXPECT_TRUE(actualTfs[idx].linear().isApprox(expected[idx].first, srl::constants::kEpsd));
        EXPECT_TRUE(actualTfs[idx].translation().isApprox(expected[idx].second, srl::constants::kEpsd));
    }
}
INSTANTIATE_TEST_SUITE_P(Test_fkAllBase, TestKinematicsCalc, jointAngles);    // NOLINT

TEST_P(TestKinematicsCalc, fkAll)    // NOLINT
{
    // Testing fkAll
    auto angles { GetParam() };

    auto expected { calculateFkAll(angles, baseRot.toRotationMatrix(), baseTran) };

    auto actualQuats { model.fkAllQuat(angles) };
    auto actualTfs { model.fkAll(angles) };
    EXPECT_EQ(actualQuats.size(), expected.size());
    EXPECT_EQ(actualTfs.size(), expected.size());

    for (uint32_t idx = 0; idx < expected.size(); idx += 1) {
        EXPECT_TRUE(actualQuats[idx].first.toRotationMatrix().isApprox(expected[idx].first, srl::constants::kEpsd));
        EXPECT_TRUE(actualQuats[idx].second.isApprox(expected[idx].second, srl::constants::kEpsd));
        EXPECT_TRUE(actualTfs[idx].linear().isApprox(expected[idx].first, srl::constants::kEpsd));
        EXPECT_TRUE(actualTfs[idx].translation().isApprox(expected[idx].second, srl::constants::kEpsd));
    }
}
INSTANTIATE_TEST_SUITE_P(Test_fkAll, TestKinematicsCalc, jointAngles);    // NOLINT

//
// =============== =============== Main
auto main(int argc, char **argv) -> int
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}