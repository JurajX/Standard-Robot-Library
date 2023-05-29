#ifndef SRL_MODELS_IMPL_HPP
#define SRL_MODELS_IMPL_HPP
#pragma once

#include <srl/models/model.hpp>

#include <srl/math/math.hpp>

#include <fmt/format.h>

#include <stdexcept>
#include <string>

namespace srl::model {

//
// =============== =============== Constructors

// --------------- helpers
template<srl::concepts::floating_point FP>
auto RobotModel<FP>::init() -> void
{
    cooTf_.reserve(kNumLinks_);
    comTfed_.reserve(kNumLinks_);
    rotAxesTfed_.reserve(kNumLinks_);
    qhLcT_.reserve(kNumLinks_);
    massMatrix_.reserve(kNumLinks_);
    christoffelSymbols_.reserve(kNumJoints_);

    for (uint32_t idx = 0; idx < kNumLinks_; idx += 1) {
        cooTf_.emplace_back(dataCooTf_.data() + (idx * kNumJointsDims_ * kNumJointsDims_), kNumJointsDims_, kNumJointsDims_);
        comTfed_.emplace_back(dataComTfed_.data() + (idx * kNumJointsDims_ * kNumJoints_), kNumJointsDims_, kNumJoints_);
        frameCoosTfed_.emplace_back(dataFrameCoosTfed_.data() + (idx * kNumJointsDims_ * kNumJoints_), kNumJointsDims_, kNumJoints_);
        rotAxesTfed_.emplace_back(dataRotAxesTfed_.data() + (idx * kNumJoints_ * kNumJointsDims_), kNumJoints_, kNumJointsDims_);
        qhLcT_.emplace_back(dataQhLcT_.data() + (idx * kNumJoints_ * kNumJointsDims_), kNumJoints_, kNumJointsDims_);
        massMatrix_.emplace_back(dataMassMatrix_.data() + (idx * kNumJoints_ * kNumJoints_), kNumJoints_, kNumJoints_);
    }
    for (uint32_t iDer = 0; iDer < kNumJoints_; iDer += 1) {
        christoffelSymbols_.emplace_back(dataChristoffelSymbols_.data() + (iDer * kNumJoints_ * kNumJoints_), kNumJoints_, kNumJoints_);
    }
}

// --------------- constructors
template<srl::concepts::floating_point FP>
RobotModel<FP>::RobotModel(
  const types::Matrix3X<FP> &joint_rotation_axes,
  const types::Matrix3X<FP> &link_lengths,
  const std::vector<FP> &link_masses,
  const types::Matrix3X<FP> &link_centres_of_mass,
  const std::vector<types::Matrix3<FP>> &link_inertia_tensors,
  const std::vector<FP> &joint_coulomb_dampings,
  const types::QuatVec3<FP> &base_pose,
  const types::Vector3<FP> &gravity_direction,
  const bool normalise_joint_rotation_axes)
    : kNumJoints_ { static_cast<uint32_t>(joint_rotation_axes.cols()) }
    , basePose_ { base_pose }
{
    setJointRotationAxes(joint_rotation_axes, normalise_joint_rotation_axes);
    setLinkLengths(link_lengths);
    setLinkMasses(link_masses);
    setLinkCentresOfMasses(link_centres_of_mass);
    setLinkInertiaTensors(link_inertia_tensors);
    setJointDampings(joint_coulomb_dampings);
    setGravityAcc(gravity_direction);

    init();
}

template<srl::concepts::floating_point FP>
RobotModel<FP>::RobotModel(
  const types::Matrix3X<FP> &joint_rotation_axes,
  const types::Matrix3X<FP> &link_lengths,
  const std::vector<FP> &link_masses,
  const types::Matrix3X<FP> &link_centres_of_mass,
  const types::Matrix3X<FP> &principal_link_inertias,
  const types::Matrix3X<FP> &principal_rotations,
  const std::vector<FP> &joint_coulomb_dampings,
  const types::QuatVec3<FP> &base_pose,
  const types::Vector3<FP> &gravity_direction,
  const bool normalise_joint_rotation_axes)
    : RobotModel { joint_rotation_axes,
                   link_lengths,
                   link_masses,
                   link_centres_of_mass,
                   makeLinkInertiaTensors(principal_link_inertias, principal_rotations),
                   joint_coulomb_dampings,
                   base_pose,
                   gravity_direction,
                   normalise_joint_rotation_axes }
{ }

template<srl::concepts::floating_point FP>
RobotModel<FP>::RobotModel(const uint32_t &number_of_joints)
    : kNumJoints_ { number_of_joints }
{
    init();
}

//
// =============== =============== Kinematics Calculations

// --------------- helpers
template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::populateRots(const Container &joint_angles, const bool wrt_rbt_base) -> void
{
#ifndef NDEBUG
    check(static_cast<uint32_t>(joint_angles.size()), "joint_angles size", kNumJoints_);
#endif
    qRots_.resize(kNumLinks_ + 1);

    qRots_[0] = wrt_rbt_base ? types::Quat<FP>::Identity() : basePose_.first;
    for (uint32_t iJoint = 0; iJoint < kNumJoints_; iJoint += 1)
        qRots_[iJoint + 1] = types::Quat<FP>({ joint_angles[iJoint], jointRotationAxes_.col(iJoint) });
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::scanRots(const bool wrt_rbt_base) -> void
{
    for (uint32_t iJoint = (static_cast<uint32_t>(wrt_rbt_base) + 1); iJoint < kNumLinks_; iJoint += 1)
        qRots_[iJoint] = qRots_[iJoint - 1] * qRots_[iJoint];

    qRots_[kNumLinks_] = qRots_[kNumLinks_ - 1];
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::populatePos(const bool wrt_rbt_base) -> void
{
    vPos_.resize(kDim_, kNumLinks_ + 1);

    vPos_.col(0) = wrt_rbt_base ? types::Vector3<FP>::Zero() : basePose_.second;
    for (uint32_t iLink = 0; iLink < kNumLinks_; iLink += 1) {
        vPos_.col(iLink + 1) = vPos_.col(iLink) + qRots_[iLink] * frameCoos_.col(iLink);
    }
}

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::makeFk(const Container &joint_angles, const bool wrt_rbt_base) -> void
{
    populateRots(joint_angles, wrt_rbt_base);
    scanRots(wrt_rbt_base);
    populatePos(wrt_rbt_base);
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::populateTfs() -> void
{
    frameTfs_.resize(kNumLinks_ + 1);

    for (uint32_t idx = 0; idx < kNumLinks_ + 1; idx += 1) {
        frameTfs_[idx].linear()      = qRots_[idx].toRotationMatrix();
        frameTfs_[idx].translation() = vPos_.col(idx);
    }
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::populateQVs() -> void
{
    frameQuatVec3s_.resize(kNumLinks_ + 1);

    for (uint32_t idx = 0; idx < kNumLinks_ + 1; idx += 1) {
        frameQuatVec3s_[idx] = { qRots_[idx], vPos_.col(idx) };
    }
}

// --------------- simple FK

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::fkQuat(const Container &joint_angles, const bool wrt_rbt_base) -> types::QuatVec3<FP>
{
    makeFk(joint_angles, wrt_rbt_base);
    return { qRots_[kNumLinks_], vPos_.col(kNumLinks_) };
}

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::fk(const Container &joint_angles, const bool wrt_rbt_base) -> types::Transform3<FP>
{
    makeFk(joint_angles, wrt_rbt_base);
    types::Transform3<FP> tf;
    tf.linear()      = qRots_[kNumLinks_].toRotationMatrix();
    tf.translation() = vPos_.col(kNumLinks_);
    return tf;
}

// --------------- all FK

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::fkAllQuat(const Container &joint_angles, const bool wrt_rbt_base) -> const std::vector<types::QuatVec3<FP>> &
{
    makeFk(joint_angles, wrt_rbt_base);
    populateQVs();
    return frameQuatVec3s_;
}

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::fkAll(const Container &joint_angles, const bool wrt_rbt_base) -> const std::vector<types::Transform3<FP>> &
{
    makeFk(joint_angles, wrt_rbt_base);
    populateTfs();
    return frameTfs_;
}

//
// =============== =============== Dynamics Calculations

// --------------- helpers

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::makeBigRot(const Container &joint_angles, bool derivatives) -> void
{
    populateRots(joint_angles, true);
    // rotation transforms
    for (uint32_t iCol = 0; iCol < kNumJoints_; iCol += 1) {
        types::Quat<FP> product { qRots_[iCol + 1] };
        cooTf_[0].block(iCol * kDim_, iCol * kDim_, kDim_, kDim_) = product.toRotationMatrix();
        for (uint32_t iRow = iCol - 1; iRow < iCol; iRow -= 1) {
            product                                               = qRots_[iRow + 1] * product;
            cooTf_[0].block(iRow * kDim_, iCol * kDim_, kDim_, kDim_) = product.toRotationMatrix();
        }
    }
    if (not derivatives) {
        return;
    }
    // partial derivatives of rotation transforms w.r.t joint angles
    for (uint32_t iDer = 0; iDer < kNumJoints_; iDer += 1) {
        for (uint32_t iCol = iDer; iCol < kNumJoints_; iCol += 1) {
            types::Matrix3<FP> product { So3Axes_[iDer] * cooTf_[0].block(iDer * kDim_, iCol * kDim_, kDim_, kDim_) };
            cooTf_[iDer + 1].block(iDer * kDim_, iCol * kDim_, kDim_, kDim_) = product;
            for (uint32_t iRow = iDer - 1; iRow < iDer; iRow -= 1) {
                product                                                       = qRots_[iRow + 1] * product;
                cooTf_[iDer + 1].block(iRow * kDim_, iCol * kDim_, kDim_, kDim_) = product;
            }
        }
    }
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::makeCentreOfMassCoos(bool derivatives) -> void
{
    uint32_t count { derivatives ? kNumLinks_ : 1 };
    for (uint32_t idx = 0; idx < count; idx += 1) {
        comTfed_[idx] = cooTf_[idx] * frameCoosAndLinkCoM_;
    }
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::makeMassMatrix(bool derivatives) -> void
{
    uint32_t count { derivatives ? kNumLinks_ : 1 };
    // calculate transformed rotation axes, needed for the inertia part of the mass matrix
    for (uint32_t idx = 0; idx < count; idx += 1) {
        uint32_t rowLimit { (idx == 0) ? kNumJoints_ : idx };
        for (uint32_t iRow = 0; iRow < rowLimit; iRow += 1) {
            rotAxesTfed_[idx].block(iRow, 0, 1, kNumJointsDims_) =
              jointRotationAxes_.col(iRow).transpose() * cooTf_[idx].block(iRow * kDim_, 0, kDim_, kNumJointsDims_);
        }
    }
    for (uint32_t iCol = 0; iCol < kNumJoints_; iCol += 1) {
        rotAxesTfedInertia_.block(0, iCol * kDim_, kNumJoints_, kDim_) = rotAxesTfed_[0].block(0, iCol * kDim_, kNumJoints_, kDim_) * linkInertias_[iCol];
    }

    // calculate rotation axes contracted with so3 contractions of centre of masses of links, needed for the translation part of the mass matrix
    for (uint32_t iRow = 0; iRow < kNumJoints_; iRow += 1) {
        auto q { jointRotationAxes_.col(iRow) };
        auto R0 { (iRow == 0) ? srl::constants::kMatrixId3<FP> : cooTf_[0].block(0, (iRow - 1) * kDim_, kDim_, kDim_) };
        for (uint32_t iCol = iRow; iCol < kNumJoints_; iCol += 1) {
            auto L0 { srl::math::so3element<FP>(comTfed_[0].block(iRow * kDim_, iCol, kDim_, 1)) };
            qhLcT_[0].block(iRow, iCol * kDim_, 1, kDim_).transpose() = R0 * (L0 * q);
            for (uint32_t idx = 1; idx < count; idx += 1) {
                auto Ri { (iRow == 0) ? srl::constants::kMatrixZero3<FP> : cooTf_[idx].block(0, (iRow - 1) * kDim_, kDim_, kDim_) };
                auto Li { srl::math::so3element<FP>(comTfed_[idx].block(iRow * kDim_, iCol, kDim_, 1)) };
                qhLcT_[idx].block(iRow, iCol * kDim_, 1, kDim_).transpose() = (iRow < idx) ? (R0 * (Li * q)) : (Ri * (L0 * q));
            }
        }
    }
    qhLcTmass_ = qhLcT_[0].array().rowwise() * linkMassVec_.transpose().array();

    // build the mass matrix
    massMatrix_[0] = rotAxesTfedInertia_ * rotAxesTfed_[0].transpose() + qhLcTmass_ * qhLcT_[0].transpose();
    for (uint32_t iDer = 1; iDer < count; iDer += 1) {
        massMatrix_[iDer] = rotAxesTfedInertia_ * rotAxesTfed_[iDer].transpose() + qhLcTmass_ * qhLcT_[iDer].transpose();
        massMatrix_[iDer] += massMatrix_[iDer].transpose().eval();
    }
}

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::makeEomParams(const Container &joint_angles, bool derivatives) -> void
{
    makeBigRot(joint_angles, derivatives);
    makeCentreOfMassCoos(derivatives);
    makeMassMatrix(derivatives);

    uint32_t count { derivatives ? kNumLinks_ : 1 };
    for (uint32_t idx = 0; idx < count; idx += 1) {
        potEnergy_[idx] = -gravityAcc_.transpose() * (comTfed_[idx].block(0, 0, kDim_, kNumJoints_) * linkMasses_);
    }

    if (not derivatives) {
        return;
    }

    auto data { dataMassMatrix_.data() + kNumJoints_ * kNumJoints_ };
    for (uint32_t iDer = 0; iDer < kNumJoints_; iDer += 1) {
        types::Map<types::MatrixX<FP>> &ijk = massMatrix_[iDer + 1];
        types::Map<types::MatrixX<FP>, types::Stride> jik { data + iDer,
                                                            kNumJoints_,
                                                            kNumJoints_,
                                                            types::Stride(kNumJoints_, kNumJoints_ * kNumJoints_) };
        types::Map<types::MatrixX<FP>, types::Stride> jki { data + iDer,
                                                            kNumJoints_,
                                                            kNumJoints_,
                                                            types::Stride(kNumJoints_ * kNumJoints_, kNumJoints_) };
        christoffelSymbols_[iDer] = 0.5 * (jki + jik - ijk);
    }
}

// --------------- dynamics

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::getMassMatrixAndPotEnergy(const Container &joint_angles) -> std::tuple<types::MatrixX<FP>, FP>
{
#ifndef NDEBUG
    check(static_cast<uint32_t>(joint_angles.size()), "joint_angles size", kNumJoints_);
#endif
    makeEomParams(joint_angles, false);
    types::MatrixX<FP> massMatrix { massMatrix_[0] };
    FP potEnergy { potEnergy_[0] };
    return std::make_tuple(massMatrix, potEnergy);
}

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::getLagrangian(const Container &joint_angles, const Container &joint_velocities) -> FP
{
#ifndef NDEBUG
    check(static_cast<uint32_t>(joint_angles.size()), "joint_angles size", kNumJoints_);
    check(static_cast<uint32_t>(joint_velocities.size()), "joint_velocities size", kNumJoints_);
#endif
    types::Map<const types::VectorX<FP>> jointVels { joint_velocities.data(), static_cast<uint32_t>(joint_velocities.size()) };
    makeEomParams(joint_angles, false);
    FP kinEnergy { 0.5 * jointVels.transpose() * massMatrix_[0] * jointVels };
    return kinEnergy - potEnergy_[0];
}

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::getEomParams(const Container &joint_angles)
  -> std::tuple<const std::vector<types::Map<types::MatrixX<FP>>>, const std::vector<types::Map<types::MatrixX<FP>>>, const types::VectorX<FP>>
{
#ifndef NDEBUG
    check(static_cast<uint32_t>(joint_angles.size()), "joint_angles size", kNumJoints_);
#endif
    makeEomParams(joint_angles);
    return std::make_tuple(massMatrix_, christoffelSymbols_, potEnergy_);
}

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::getMotorTorque(const Container &joint_angles, const Container &joint_velocities, const Container &joint_accelerations)
  -> types::VectorX<FP>
{
#ifndef NDEBUG
    check(static_cast<uint32_t>(joint_angles.size()), "joint_angles size", kNumJoints_);
    check(static_cast<uint32_t>(joint_velocities.size()), "joint_velocities size", kNumJoints_);
    check(static_cast<uint32_t>(joint_accelerations.size()), "joint_accelerations size", kNumJoints_);
#endif
    types::Map<const types::VectorX<FP>> jointVels { joint_velocities.data(), static_cast<uint32_t>(joint_velocities.size()) };
    types::Map<const types::VectorX<FP>> jointAcce { joint_accelerations.data(), static_cast<uint32_t>(joint_accelerations.size()) };
    makeEomParams(joint_angles);
    // torques = inertia torque + gravity torque + friction torque
    torques_ = massMatrix_[0] * jointAcce + potEnergy_.tail(kNumJoints_) + jointVels.cwiseProduct(jointDamping_);
    // torques += Christoffel torque
    for (uint32_t iJoint = 0; iJoint < kNumJoints_; iJoint += 1) {
        torques_[iJoint] += jointVels.transpose() * christoffelSymbols_[iJoint] * jointVels;
    }
    return torques_;
}

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::getAngularAcceleration(const Container &joint_angles, const Container &joint_velocities, const Container &motor_torques)
  -> types::VectorX<FP>
{
#ifndef NDEBUG
    check(static_cast<uint32_t>(joint_angles.size()), "joint_angles size", kNumJoints_);
    check(static_cast<uint32_t>(joint_velocities.size()), "joint_velocities size", kNumJoints_);
    check(static_cast<uint32_t>(motor_torques.size()), "motor_torques size", kNumJoints_);
#endif
    types::Map<const types::VectorX<FP>> jointVels { joint_velocities.data(), static_cast<uint32_t>(joint_velocities.size()) };
    types::Map<const types::VectorX<FP>> motorTorq { motor_torques.data(), static_cast<uint32_t>(motor_torques.size()) };
    makeEomParams(joint_angles);
    // torques = applied motor torques - gravity torques - friction torque
    torques_ = motorTorq - potEnergy_.tail(kNumJoints_) - jointVels.cwiseProduct(jointDamping_);
    // torques -= Christoffel torque
    for (uint32_t iJoint = 0; iJoint < kNumJoints_; iJoint += 1) {
        torques_[iJoint] -= jointVels.transpose() * christoffelSymbols_[iJoint] * jointVels;
    }
    return massMatrix_[0].inverse() * torques_;
}

//
// =============== =============== Jacobian Calculations

// --------------- all Jacobian
template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::getJacobian(const Container &joint_angles)
  -> std::tuple<const std::vector<types::Map<types::MatrixX<FP>>>, const std::vector<types::Map<types::MatrixX<FP>>>>
{
    makeBigRot(joint_angles);
    for (uint32_t idx = 0; idx < kNumLinks_; idx += 1) {
        frameCoosTfed_[idx] = cooTf_[idx] * frameCoosTriu_;
    }
    return {
        {        cooTf_.begin() + 1,         cooTf_.end()},
        {frameCoosTfed_.begin() + 1, frameCoosTfed_.end()}
    };
}

//
// =============== =============== Setters and Getters

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::getNumberOfJoints() -> uint32_t
{
    return kNumJoints_;
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::getNumberOfLinks() -> uint32_t
{
    return kNumLinks_;
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::setJointRotationAxes(const types::Matrix3X<FP> &joint_rotation_axes, const bool normalise) -> void
{
    check(static_cast<uint32_t>(joint_rotation_axes.cols()), "joint_rotation_axes size", kNumJoints_);
    if (normalise) {
        jointRotationAxes_ = joint_rotation_axes;
        for (uint32_t iJoint = 0; iJoint < kNumJoints_; iJoint += 1)
            jointRotationAxes_.col(iJoint).normalize();
    } else {
        for (uint32_t iJoint = 0; iJoint < kNumJoints_; iJoint += 1) {
            if (not srl::math::eq<FP>(joint_rotation_axes.col(iJoint).norm(), 1)) {
                std::string msg { fmt::format("The rotation axis of joint {} (1-based) is not normed.", iJoint + 1) };
                // TODO log
                throw std::invalid_argument(msg);
            }
        }
        jointRotationAxes_ = joint_rotation_axes;
    }
    types::Vector3<FP> axis;
    So3Axes_.resize(kNumJoints_);
    for (uint32_t iJoint = 0; iJoint < kNumJoints_; iJoint += 1)
        So3Axes_[iJoint] = srl::math::so3element<FP>(types::Vector3<FP> { jointRotationAxes_.col(iJoint) });
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::getJointRotationAxes() -> types::Matrix3X<FP>
{
    return jointRotationAxes_;
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::setLinkLengths(const types::Matrix3X<FP> &link_lengths) -> void
{
    check(static_cast<uint32_t>(link_lengths.cols()), "link_lengths size", kNumLinks_);
    frameCoos_ = link_lengths;
    for (uint32_t iLink = 1; iLink < kNumLinks_; iLink += 1) {
        frameCoosAndLinkCoM_.block((iLink - 1) * kDim_, iLink, kDim_, kNumJoints_ - iLink).colwise() = frameCoos_.col(iLink);
        frameCoosTriu_.block((iLink - 1) * kDim_, iLink - 1, kDim_, kNumLinks_ - iLink).colwise()    = frameCoos_.col(iLink);
    }
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::getLinkLengths() -> types::Matrix3X<FP>
{
    return frameCoos_;
}

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::setLinkMasses(const Container &link_masses) -> void
{
    check(static_cast<uint32_t>(link_masses.size()), "link_masses size", kNumJoints_);
    for (uint32_t iLink = 0; iLink < kNumJoints_; iLink += 1) {
        if (link_masses[iLink] < 0) {
            std::string msg { fmt::format("The mass of link {} (0-based) is not normed.", iLink + 1) };
            // TODO log
            throw std::invalid_argument(msg);
        }
    }
    for (uint32_t iLink = 0; iLink < kNumJoints_; iLink += 1) {
        linkMasses_[iLink]             = link_masses[iLink];
        linkMassVec_[iLink * kDim_ + 0] = link_masses[iLink];
        linkMassVec_[iLink * kDim_ + 1] = link_masses[iLink];
        linkMassVec_[iLink * kDim_ + 2] = link_masses[iLink];
    }
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::getLinkMasses() -> types::VectorX<FP>
{
    return linkMasses_;
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::setLinkCentresOfMasses(const types::Matrix3X<FP> &link_centres_of_mass) -> void
{
    check(static_cast<uint32_t>(link_centres_of_mass.cols()), "link_centres_of_mass size", kNumJoints_);
    linkCoMs_ = link_centres_of_mass;
    for (uint32_t iLink = 0; iLink < kNumJoints_; iLink += 1) {
        frameCoosAndLinkCoM_.block(iLink * kDim_, iLink, kDim_, 1) = linkCoMs_.col(iLink);
    }
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::getLinkCentresOfMasses() -> types::Matrix3X<FP>
{
    return linkCoMs_;
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::setLinkInertiaTensors(const std::vector<types::Matrix3<FP>> &link_inertia_tensors) -> void
{
    check(static_cast<uint32_t>(link_inertia_tensors.size()), "link_inertia_tensors size", kNumJoints_);
    for (uint32_t iLink = 0; iLink < kNumJoints_; iLink += 1) {
        if (not srl::math::isInertia<FP>(link_inertia_tensors[iLink])) {
            std::string msg { fmt::format("The inertia of link {} (0-based) does not represent a valid inertia tensor.", iLink + 1) };
            // TODO log
            throw std::invalid_argument(msg);
        }
    }
    linkInertias_ = link_inertia_tensors;
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::setLinkInertiaTensors(const types::Matrix3X<FP> &principal_link_inertias, const types::Matrix3X<FP> &principal_rotations) -> void
{
    setLinkInertiaTensors(makeLinkInertiaTensors(principal_link_inertias, principal_rotations));
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::getLinkInertiaTensors() -> std::vector<types::Matrix3<FP>>
{
    return linkInertias_;
}

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::setJointDampings(const Container &joint_coulomb_dampings) -> void
{
    check(static_cast<uint32_t>(joint_coulomb_dampings.size()), "joint_coulomb_dampings size", kNumJoints_);
    for (uint32_t iJoint = 0; iJoint < kNumJoints_; iJoint += 1) {
        if (joint_coulomb_dampings[iJoint] < 0) {
            std::string msg { fmt::format("The damping of joint {} (1-based) is negative.", iJoint + 1) };
            // TODO log
            throw std::invalid_argument(msg);
        }
    }
    for (uint32_t iLink = 0; iLink < kNumJoints_; iLink += 1) {
        jointDamping_[iLink] = joint_coulomb_dampings[iLink];
    }
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::getJointDampings() -> types::VectorX<FP>
{
    return jointDamping_;
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::setGravityAcc(const types::Vector3<FP> &gravity_direction) -> void
{
    gravityAcc_ = (srl::constants::kG<FP> / gravity_direction.norm()) * gravity_direction;
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::getGravityAcc() -> types::Vector3<FP>
{
    return gravityAcc_;
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::setBaseTransform(const types::Transform3<FP> &base_tf) -> void
{
    basePose_ = { types::Quat<FP>(base_tf.linear()), base_tf.translation() };
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::getBaseTransform() -> types::Transform3<FP>
{
    types::Transform3<FP> tf;
    tf.linear()      = basePose_.first.toRotationMatrix();
    tf.translation() = basePose_.second;
    return tf;
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::setBasePose(const types::QuatVec3<FP> &base_pose) -> void
{
    basePose_ = base_pose;
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::getBasePose() -> types::QuatVec3<FP>
{
    return basePose_;
}

//
// =============== =============== Helpers

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::makeLinkInertiaTensors(const types::Matrix3X<FP> &principal_link_inertias, const types::Matrix3X<FP> &principal_rotations)
  -> std::vector<types::Matrix3<FP>>
{
    check(principal_link_inertias.cols(), "principal_link_inertias size", principal_rotations.cols(), "principal_rotations size");
    std::vector<types::Matrix3<FP>> inertias;
    inertias.reserve(static_cast<size_t>(principal_rotations.cols()));
    types::Vector3<FP> principals;
    types::Matrix3<FP> rot;
    for (uint32_t iLink = 0; iLink < principal_link_inertias.cols(); iLink += 1) {
        principals = principal_link_inertias.col(iLink);
        rot        = srl::math::rotation(types::Vector3<FP> { principal_rotations.col(iLink) });
        inertias.push_back(srl::math::adjoint(principals, rot));
    }
    return inertias;
}

template<srl::concepts::floating_point FP>
template<srl::concepts::integral T>
auto RobotModel<FP>::check(const T &actual, std::string_view arg_name, const T &desired, std::string_view desired_name, const bool runtime) -> void
{
    if (actual != desired) {
        std::string msg { fmt::format("The value of the '{}', which is {}, must be equal to ", arg_name, actual) };
        if (desired_name == "")
            msg += fmt::format("{}.", desired);
        else
            msg += fmt::format("the value of '{}', which is {}.", desired_name, desired);
        // TODO log msg
        if (runtime)
            throw std::runtime_error(msg);
        throw std::invalid_argument(msg);
    }
}

}    // namespace srl::model

#endif    // SRL_MODELS_IMPL_HPP
