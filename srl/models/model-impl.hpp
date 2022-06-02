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

    for (srl::uint idx = 0; idx < kNumLinks_; idx += 1) {
        cooTf_.emplace_back(dataCooTf_.data() + (idx * kNumJointsDims_ * kNumJointsDims_), kNumJointsDims_, kNumJointsDims_);
        comTfed_.emplace_back(dataComTfed_.data() + (idx * kNumJointsDims_ * kNumJoints_), kNumJointsDims_, kNumJoints_);
        frameCoosTfed_.emplace_back(dataFrameCoosTfed_.data() + (idx * kNumJointsDims_ * kNumJoints_), kNumJointsDims_, kNumJoints_);
        rotAxesTfed_.emplace_back(dataRotAxesTfed_.data() + (idx * kNumJoints_ * kNumJointsDims_), kNumJoints_, kNumJointsDims_);
        qhLcT_.emplace_back(dataQhLcT_.data() + (idx * kNumJoints_ * kNumJointsDims_), kNumJoints_, kNumJointsDims_);
        massMatrix_.emplace_back(dataMassMatrix_.data() + (idx * kNumJoints_ * kNumJoints_), kNumJoints_, kNumJoints_);
    }
    for (srl::uint iDiff = 0; iDiff < kNumJoints_; iDiff += 1) {
        christoffelSymbols_.emplace_back(dataChristoffelSymbols_.data() + (iDiff * kNumJoints_ * kNumJoints_), kNumJoints_, kNumJoints_);
    }
}

// --------------- constructors
template<srl::concepts::floating_point FP>
RobotModel<FP>::RobotModel(
  const srl::Matrix3X<FP> &joint_rotation_axes,
  const srl::Matrix3X<FP> &link_lengths,
  const srl::vector<FP> &link_masses,
  const srl::Matrix3X<FP> &link_centres_of_mass,
  const srl::vector<srl::Matrix3<FP>> &link_inertia_tensors,
  const srl::vector<FP> &joint_coulomb_dampings,
  const srl::QuatVec3<FP> &base_pose,
  const srl::Vector3<FP> &gravity_direction,
  const bool normalise_joint_rotation_axes)
    : kNumJoints_ { static_cast<srl::uint>(joint_rotation_axes.cols()) }
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
  const srl::Matrix3X<FP> &joint_rotation_axes,
  const srl::Matrix3X<FP> &link_lengths,
  const srl::vector<FP> &link_masses,
  const srl::Matrix3X<FP> &link_centres_of_mass,
  const srl::Matrix3X<FP> &principal_link_inertias,
  const srl::Matrix3X<FP> &principal_rotations,
  const srl::vector<FP> &joint_coulomb_dampings,
  const srl::QuatVec3<FP> &base_pose,
  const srl::Vector3<FP> &gravity_direction,
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
RobotModel<FP>::RobotModel(const srl::uint &number_of_joints)
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
    check(static_cast<srl::uint>(joint_angles.size()), "joint_angles size", kNumJoints_);
#endif
    qRots_.resize(kNumLinks_ + 1);

    qRots_[0] = wrt_rbt_base ? srl::Quat<FP>::Identity() : basePose_.first;
    for (srl::uint iJoint = 0; iJoint < kNumJoints_; iJoint += 1)
        qRots_[iJoint + 1] = srl::Quat<FP>({ joint_angles[iJoint], jointRotationAxes_.col(iJoint) });
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::scanRots(const bool wrt_rbt_base) -> void
{
    for (srl::uint iJoint = (1 + wrt_rbt_base); iJoint < kNumLinks_; iJoint += 1)
        qRots_[iJoint] = qRots_[iJoint - 1] * qRots_[iJoint];

    qRots_[kNumLinks_] = qRots_[kNumLinks_ - 1];
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::populatePos(const bool wrt_rbt_base) -> void
{
    vPos_.resize(kDim, kNumLinks_ + 1);

    vPos_.col(0) = wrt_rbt_base ? srl::Vector3<FP>::Zero() : basePose_.second;
    for (srl::uint iLink = 0; iLink < kNumLinks_; iLink += 1) {
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

    for (srl::uint idx = 0; idx < kNumLinks_ + 1; idx += 1) {
        frameTfs_[idx].linear()      = qRots_[idx].toRotationMatrix();
        frameTfs_[idx].translation() = vPos_.col(idx);
    }
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::populateQVs() -> void
{
    frameQuatVec3s_.resize(kNumLinks_ + 1);

    for (srl::uint idx = 0; idx < kNumLinks_ + 1; idx += 1) {
        frameQuatVec3s_[idx] = { qRots_[idx], vPos_.col(idx) };
    }
}

// --------------- simple FK

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::fkQuat(const Container &joint_angles, const bool wrt_rbt_base) -> srl::QuatVec3<FP>
{
    makeFk(joint_angles, wrt_rbt_base);
    return { qRots_[kNumLinks_], vPos_.col(kNumLinks_) };
}

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::fk(const Container &joint_angles, const bool wrt_rbt_base) -> srl::Transform3<FP>
{
    makeFk(joint_angles, wrt_rbt_base);
    srl::Transform3<FP> tf;
    tf.linear()      = qRots_[kNumLinks_].toRotationMatrix();
    tf.translation() = vPos_.col(kNumLinks_);
    return tf;
}

// --------------- all FK

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::fkAllQuat(const Container &joint_angles, const bool wrt_rbt_base) -> const srl::vector<srl::QuatVec3<FP>> &
{
    makeFk(joint_angles, wrt_rbt_base);
    populateQVs();
    return frameQuatVec3s_;
}

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::fkAll(const Container &joint_angles, const bool wrt_rbt_base) -> const srl::vector<srl::Transform3<FP>> &
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
    for (srl::uint iCol = 0; iCol < kNumJoints_; iCol += 1) {
        srl::Quat<FP> product { qRots_[iCol + 1] };
        cooTf_[0].block(iCol * kDim, iCol * kDim, kDim, kDim) = product.toRotationMatrix();
        for (srl::uint iRow = iCol - 1; iRow < iCol; iRow -= 1) {
            product                                               = qRots_[iRow + 1] * product;
            cooTf_[0].block(iRow * kDim, iCol * kDim, kDim, kDim) = product.toRotationMatrix();
        }
    }
    if (not derivatives) {
        return;
    }
    // partial derivatives of rotation transforms w.r.t joint angles
    for (srl::uint iDiff = 0; iDiff < kNumJoints_; iDiff += 1) {
        for (srl::uint iCol = iDiff; iCol < kNumJoints_; iCol += 1) {
            srl::Matrix3<FP> product { So3Axes_[iDiff] * cooTf_[0].block(iDiff * kDim, iCol * kDim, kDim, kDim) };
            cooTf_[iDiff + 1].block(iDiff * kDim, iCol * kDim, kDim, kDim) = product;
            for (srl::uint iRow = iDiff - 1; iRow < iDiff; iRow -= 1) {
                product                                                       = qRots_[iRow + 1] * product;
                cooTf_[iDiff + 1].block(iRow * kDim, iCol * kDim, kDim, kDim) = product;
            }
        }
    }
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::makeCentreOfMassCoos(bool derivatives) -> void
{
    srl::uint count { derivatives ? kNumLinks_ : 1 };
    for (srl::uint idx = 0; idx < count; idx += 1) {
        comTfed_[idx] = cooTf_[idx] * frameCoosAndLinkCoM_;
    }
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::makeMassMatrix(bool derivatives) -> void
{
    srl::uint count { derivatives ? kNumLinks_ : 1 };
    // calculate transformed rotation axes, needed for the inertia part of the mass matrix
    for (srl::uint idx = 0; idx < count; idx += 1) {
        srl::uint rowLimit { (idx == 0) ? kNumJoints_ : idx };
        for (srl::uint iRow = 0; iRow < rowLimit; iRow += 1) {
            rotAxesTfed_[idx].block(iRow, 0, 1, kNumJointsDims_) =
              jointRotationAxes_.col(iRow).transpose() * cooTf_[idx].block(iRow * kDim, 0, kDim, kNumJointsDims_);
        }
    }
    for (srl::uint iCol = 0; iCol < kNumJoints_; iCol += 1) {
        rotAxesTfedInertia_.block(0, iCol * kDim, kNumJoints_, kDim) = rotAxesTfed_[0].block(0, iCol * kDim, kNumJoints_, kDim) * linkInertias_[iCol];
    }

    // calculate rotation axes contracted with so3 contractions of centre of masses of links, needed for the translation part of the mass matrix
    for (srl::uint iRow = 0; iRow < kNumJoints_; iRow += 1) {
        auto q { jointRotationAxes_.col(iRow) };
        auto R0 { (iRow == 0) ? srl::kMatrixId3<FP> : cooTf_[0].block(0, (iRow - 1) * kDim, kDim, kDim) };
        for (srl::uint iCol = iRow; iCol < kNumJoints_; iCol += 1) {
            auto L0 { srl::math::so3element<FP>(comTfed_[0].block(iRow * kDim, iCol, kDim, 1)) };
            qhLcT_[0].block(iRow, iCol * kDim, 1, kDim).transpose() = R0 * (L0 * q);
            for (srl::uint idx = 1; idx < count; idx += 1) {
                auto Ri { (iRow == 0) ? srl::kMatrixZero3<FP> : cooTf_[idx].block(0, (iRow - 1) * kDim, kDim, kDim) };
                auto Li { srl::math::so3element<FP>(comTfed_[idx].block(iRow * kDim, iCol, kDim, 1)) };
                qhLcT_[idx].block(iRow, iCol * kDim, 1, kDim).transpose() = (iRow < idx) ? (R0 * (Li * q)) : (Ri * (L0 * q));
            }
        }
    }
    qhLcTmass_ = qhLcT_[0].array().rowwise() * linkMassVec_.transpose().array();

    // build the mass matrix
    massMatrix_[0] = rotAxesTfedInertia_ * rotAxesTfed_[0].transpose() + qhLcTmass_ * qhLcT_[0].transpose();
    for (srl::uint iDiff = 1; iDiff < count; iDiff += 1) {
        massMatrix_[iDiff] = rotAxesTfedInertia_ * rotAxesTfed_[iDiff].transpose() + qhLcTmass_ * qhLcT_[iDiff].transpose();
        massMatrix_[iDiff] += massMatrix_[iDiff].transpose().eval();
    }
}

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::makeEomParams(const Container &joint_angles, bool derivatives) -> void
{
    makeBigRot(joint_angles, derivatives);
    makeCentreOfMassCoos(derivatives);
    makeMassMatrix(derivatives);

    srl::uint count { derivatives ? kNumLinks_ : 1 };
    for (srl::uint idx = 0; idx < count; idx += 1) {
        potEnergy_[idx] = -gravityAcc_.transpose() * (comTfed_[idx].block(0, 0, kDim, kNumJoints_) * linkMasses_);
    }

    if (not derivatives) {
        return;
    }

    auto data { dataMassMatrix_.data() + kNumJoints_ * kNumJoints_ };
    for (srl::uint iDiff = 0; iDiff < kNumJoints_; iDiff += 1) {
        srl::Map<srl::MatrixX<FP>> &ijk = massMatrix_[iDiff + 1];
        srl::Map<srl::MatrixX<FP>, srl::Stride> jik { data + iDiff, kNumJoints_, kNumJoints_, srl::Stride(kNumJoints_, kNumJoints_ * kNumJoints_) };
        srl::Map<srl::MatrixX<FP>, srl::Stride> jki { data + iDiff, kNumJoints_, kNumJoints_, srl::Stride(kNumJoints_ * kNumJoints_, kNumJoints_) };
        christoffelSymbols_[iDiff] = 0.5 * (jki + jik - ijk);
    }
}

// --------------- dynamics

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::getMassMatrixAndPotEnergy(const Container &joint_angles) -> std::tuple<srl::MatrixX<FP>, FP>
{
#ifndef NDEBUG
    check(static_cast<srl::uint>(joint_angles.size()), "joint_angles size", kNumJoints_);
#endif
    makeEomParams(joint_angles, false);
    srl::MatrixX<FP> massMatrix { massMatrix_[0] };
    FP potEnergy { potEnergy_[0] };
    return std::make_tuple(massMatrix, potEnergy);
}

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::getLagrangian(const Container &joint_angles, const Container &joint_velocities) -> FP
{
#ifndef NDEBUG
    check(static_cast<srl::uint>(joint_angles.size()), "joint_angles size", kNumJoints_);
    check(static_cast<srl::uint>(joint_velocities.size()), "joint_velocities size", kNumJoints_);
#endif
    srl::Map<const srl::VectorX<FP>> jointVels { joint_velocities.data(), static_cast<srl::uint>(joint_velocities.size()) };
    makeEomParams(joint_angles, false);
    FP kinEnergy { 0.5 * jointVels.transpose() * massMatrix_[0] * jointVels };
    return kinEnergy - potEnergy_[0];
}

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::getEomParams(const Container &joint_angles)
  -> std::tuple<const srl::vector<srl::Map<srl::MatrixX<FP>>>, const srl::vector<srl::Map<srl::MatrixX<FP>>>, const srl::VectorX<FP>>
{
#ifndef NDEBUG
    check(static_cast<srl::uint>(joint_angles.size()), "joint_angles size", kNumJoints_);
#endif
    makeEomParams(joint_angles);
    return std::make_tuple(massMatrix_, christoffelSymbols_, potEnergy_);
}

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::getMotorTorque(const Container &joint_angles, const Container &joint_velocities, const Container &joint_accelerations)
  -> srl::VectorX<FP>
{
#ifndef NDEBUG
    check(static_cast<srl::uint>(joint_angles.size()), "joint_angles size", kNumJoints_);
    check(static_cast<srl::uint>(joint_velocities.size()), "joint_velocities size", kNumJoints_);
    check(static_cast<srl::uint>(joint_accelerations.size()), "joint_accelerations size", kNumJoints_);
#endif
    srl::Map<const srl::VectorX<FP>> jointVels { joint_velocities.data(), static_cast<srl::uint>(joint_velocities.size()) };
    srl::Map<const srl::VectorX<FP>> jointAcce { joint_accelerations.data(), static_cast<srl::uint>(joint_accelerations.size()) };
    makeEomParams(joint_angles);
    // torques = inertia torque + gravity torque + friction torque
    torques_ = massMatrix_[0] * jointAcce + potEnergy_.tail(kNumJoints_) + jointVels.cwiseProduct(jointDamping_);
    // torques += Christoffel torque
    for (srl::uint iJoint = 0; iJoint < kNumJoints_; iJoint += 1) {
        torques_[iJoint] += jointVels.transpose() * christoffelSymbols_[iJoint] * jointVels;
    }
    return torques_;
}

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::getAngularAcceleration(const Container &joint_angles, const Container &joint_velocities, const Container &motor_torques)
  -> srl::VectorX<FP>
{
#ifndef NDEBUG
    check(static_cast<srl::uint>(joint_angles.size()), "joint_angles size", kNumJoints_);
    check(static_cast<srl::uint>(joint_velocities.size()), "joint_velocities size", kNumJoints_);
    check(static_cast<srl::uint>(motor_torques.size()), "motor_torques size", kNumJoints_);
#endif
    srl::Map<const srl::VectorX<FP>> jointVels { joint_velocities.data(), static_cast<srl::uint>(joint_velocities.size()) };
    srl::Map<const srl::VectorX<FP>> motorTorq { motor_torques.data(), static_cast<srl::uint>(motor_torques.size()) };
    makeEomParams(joint_angles);
    // torques = applied motor torques - gravity torques - friction torque
    torques_ = motorTorq - potEnergy_.tail(kNumJoints_) - jointVels.cwiseProduct(jointDamping_);
    // torques -= Christoffel torque
    for (srl::uint iJoint = 0; iJoint < kNumJoints_; iJoint += 1) {
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
  -> std::tuple<const srl::vector<srl::Map<srl::MatrixX<FP>>>, const srl::vector<srl::Map<srl::MatrixX<FP>>>>
{
    makeBigRot(joint_angles);
    for (srl::uint idx = 0; idx < kNumLinks_; idx += 1) {
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
auto RobotModel<FP>::getNumberOfJoints() -> srl::uint
{
    return kNumJoints_;
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::getNumberOfLinks() -> srl::uint
{
    return kNumLinks_;
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::setJointRotationAxes(const srl::Matrix3X<FP> &joint_rotation_axes, const bool normalise) -> void
{
    check(static_cast<srl::uint>(joint_rotation_axes.cols()), "joint_rotation_axes size", kNumJoints_);
    if (normalise) {
        jointRotationAxes_ = joint_rotation_axes;
        for (srl::uint iJoint = 0; iJoint < kNumJoints_; iJoint += 1)
            jointRotationAxes_.col(iJoint).normalize();
    } else {
        for (srl::uint iJoint = 0; iJoint < kNumJoints_; iJoint += 1) {
            if (not srl::math::eq<FP>(joint_rotation_axes.col(iJoint).norm(), 1)) {
                std::string msg { fmt::format("The rotation axis of joint {} (1-based) is not normed.", iJoint + 1) };
                // TODO log
                throw std::invalid_argument(msg);
            }
        }
        jointRotationAxes_ = joint_rotation_axes;
    }
    srl::Vector3<FP> axis;
    So3Axes_.resize(kNumJoints_);
    for (srl::uint iJoint = 0; iJoint < kNumJoints_; iJoint += 1)
        So3Axes_[iJoint] = srl::math::so3element<FP>(srl::Vector3<FP> { jointRotationAxes_.col(iJoint) });
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::getJointRotationAxes() -> srl::Matrix3X<FP>
{
    return jointRotationAxes_;
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::setLinkLengths(const srl::Matrix3X<FP> &link_lengths) -> void
{
    check(static_cast<srl::uint>(link_lengths.cols()), "link_lengths size", kNumLinks_);
    frameCoos_ = link_lengths;
    for (srl::uint iLink = 1; iLink < kNumLinks_; iLink += 1) {
        frameCoosAndLinkCoM_.block((iLink - 1) * kDim, iLink, kDim, kNumJoints_ - iLink).colwise() = frameCoos_.col(iLink);
        frameCoosTriu_.block((iLink - 1) * kDim, iLink - 1, kDim, kNumLinks_ - iLink).colwise()    = frameCoos_.col(iLink);
    }
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::getLinkLengths() -> srl::Matrix3X<FP>
{
    return frameCoos_;
}

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::setLinkMasses(const Container &link_masses) -> void
{
    check(static_cast<srl::uint>(link_masses.size()), "link_masses size", kNumJoints_);
    for (srl::uint iLink = 0; iLink < kNumJoints_; iLink += 1) {
        if (link_masses[iLink] < 0) {
            std::string msg { fmt::format("The mass of link {} (0-based) is not normed.", iLink + 1) };
            // TODO log
            throw std::invalid_argument(msg);
        }
    }
    for (srl::uint iLink = 0; iLink < kNumJoints_; iLink += 1) {
        linkMasses_[iLink]             = link_masses[iLink];
        linkMassVec_[iLink * kDim + 0] = link_masses[iLink];
        linkMassVec_[iLink * kDim + 1] = link_masses[iLink];
        linkMassVec_[iLink * kDim + 2] = link_masses[iLink];
    }
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::getLinkMasses() -> srl::VectorX<FP>
{
    return linkMasses_;
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::setLinkCentresOfMasses(const srl::Matrix3X<FP> &link_centres_of_mass) -> void
{
    check(static_cast<srl::uint>(link_centres_of_mass.cols()), "link_centres_of_mass size", kNumJoints_);
    linkCoMs_ = link_centres_of_mass;
    for (srl::uint iLink = 0; iLink < kNumJoints_; iLink += 1) {
        frameCoosAndLinkCoM_.block(iLink * kDim, iLink, kDim, 1) = linkCoMs_.col(iLink);
    }
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::getLinkCentresOfMasses() -> srl::Matrix3X<FP>
{
    return linkCoMs_;
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::setLinkInertiaTensors(const srl::vector<srl::Matrix3<FP>> &link_inertia_tensors) -> void
{
    check(static_cast<srl::uint>(link_inertia_tensors.size()), "link_inertia_tensors size", kNumJoints_);
    for (srl::uint iLink = 0; iLink < kNumJoints_; iLink += 1) {
        if (not srl::math::isInertia<FP>(link_inertia_tensors[iLink])) {
            std::string msg { fmt::format("The inertia of link {} (0-based) does not represent a valid inertia tensor.", iLink + 1) };
            // TODO log
            throw std::invalid_argument(msg);
        }
    }
    linkInertias_ = link_inertia_tensors;
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::setLinkInertiaTensors(const srl::Matrix3X<FP> &principal_link_inertias, const srl::Matrix3X<FP> &principal_rotations) -> void
{
    setLinkInertiaTensors(makeLinkInertiaTensors(principal_link_inertias, principal_rotations));
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::getLinkInertiaTensors() -> srl::vector<srl::Matrix3<FP>>
{
    return linkInertias_;
}

template<srl::concepts::floating_point FP>
template<srl::concepts::indexable Container>
auto RobotModel<FP>::setJointDampings(const Container &joint_coulomb_dampings) -> void
{
    check(static_cast<srl::uint>(joint_coulomb_dampings.size()), "joint_coulomb_dampings size", kNumJoints_);
    for (srl::uint iJoint = 0; iJoint < kNumJoints_; iJoint += 1) {
        if (joint_coulomb_dampings[iJoint] < 0) {
            std::string msg { fmt::format("The damping of joint {} (1-based) is negative.", iJoint + 1) };
            // TODO log
            throw std::invalid_argument(msg);
        }
    }
    for (srl::uint iLink = 0; iLink < kNumJoints_; iLink += 1) {
        jointDamping_[iLink] = joint_coulomb_dampings[iLink];
    }
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::getJointDampings() -> srl::VectorX<FP>
{
    return jointDamping_;
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::setGravityAcc(const srl::Vector3<FP> &gravity_direction) -> void
{
    gravityAcc_ = (srl::kG<FP> / gravity_direction.norm()) * gravity_direction;
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::getGravityAcc() -> srl::Vector3<FP>
{
    return gravityAcc_;
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::setBaseTransform(const srl::Transform3<FP> &base_tf) -> void
{
    basePose_ = { srl::Quat<FP>(base_tf.linear()), base_tf.translation() };
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::getBaseTransform() -> srl::Transform3<FP>
{
    srl::Transform3<FP> tf;
    tf.linear()      = basePose_.first.toRotationMatrix();
    tf.translation() = basePose_.second;
    return tf;
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::setBasePose(const QuatVec3<FP> &base_pose) -> void
{
    basePose_ = base_pose;
}

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::getBasePose() -> QuatVec3<FP>
{
    return basePose_;
}

//
// =============== =============== Helpers

template<srl::concepts::floating_point FP>
auto RobotModel<FP>::makeLinkInertiaTensors(const srl::Matrix3X<FP> &principal_link_inertias, const srl::Matrix3X<FP> &principal_rotations)
  -> srl::vector<srl::Matrix3<FP>>
{
    check(principal_link_inertias.cols(), "principal_link_inertias size", principal_rotations.cols(), "principal_rotations size");
    srl::vector<srl::Matrix3<FP>> inertias;
    inertias.reserve(static_cast<size_t>(principal_rotations.cols()));
    srl::Vector3<FP> principals;
    srl::Matrix3<FP> rot;
    for (srl::uint iLink = 0; iLink < principal_link_inertias.cols(); iLink += 1) {
        principals = principal_link_inertias.col(iLink);
        rot        = srl::math::rotation(srl::Vector3<FP> { principal_rotations.col(iLink) });
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
