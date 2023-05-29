#ifndef SRL_MODELS_HPP
#define SRL_MODELS_HPP
#pragma once

#include <srl/common/common.hpp>

#include <string_view>

namespace srl::model {

/// @brief Robot model class.
/// @tparam FP Floating point type.
template<srl::concepts::floating_point FP>
class RobotModel
{
public:
    /// @brief Typedef for internal reference to the template
    using value_type = FP;
    /// @brief Typedef for internal reference to the template
    using Scalar = FP;

    /// @brief Destructor.
    virtual ~RobotModel() = default;

    //
    // =============== =============== Constructors
private:
    // --------------- helpers
    // Initialises vectors<types::Map> with allocated data, so that they behave as tensors.
    auto init() -> void;

public:
    // --------------- constructors
    /**
     * @brief Construct a new Robot Model object.
     *
     * One chooses a base frame orientation of the robot. This frame is translated to each joint, in the default (zero) configuration. The
     * translations correspond to the *link_lengths*. The *joint_rotation_axes*, *link_centres_of_mass*, and *link_inertia_tensors* are expressed in
     * these frames. The *gravity_direction* is expressed w.r.t. the base frame.
     *
     * @param joint_rotation_axes Vector of rotation axes of the revolute joints of the robot.
     * @param link_lengths Length vectors of the robot links in the default (zero) position of the joints.
     * @param link_masses The masses of the links.
     * @param link_centres_of_mass The centres of mass expressed in the link's own coordinate system.
     * @param link_inertia_tensors The inertias of each moving link.
     * @param joint_coulomb_dampings The coulomb damping constant of the joints.
     * @param base_pose The position and orientation of the robot's base in the space.
     * @param gravity_direction The direction of the gravitational acceleration.
     * @param normalise_joint_rotation_axes Determines if the 'joint_rotation_axes' parameter should be normalised before using.
     */
    RobotModel(
      const srl::types::Matrix3X<FP> &joint_rotation_axes,
      const srl::types::Matrix3X<FP> &link_lengths,
      const std::vector<FP> &link_masses,
      const srl::types::Matrix3X<FP> &link_centres_of_mass,
      const std::vector<srl::types::Matrix3<FP>> &link_inertia_tensors,
      const std::vector<FP> &joint_coulomb_dampings,
      const srl::types::QuatVec3<FP> &base_pose        = { srl::types::Quat<FP>::Identity(), srl::types::Vector3<FP>::Zero() },
      const srl::types::Vector3<FP> &gravity_direction = { 0, 0, -1 },
      const bool normalise_joint_rotation_axes         = false);

    /**
     * @brief Construct a new Robot Model object.
     *
     * One chooses a base frame orientation of the robot. This frame is translated to each joint, in the default (zero) configuration. The subsequent
     * translations correspond to the *link_lengths*. The *joint_rotation_axes*, *link_centres_of_mass*, and *principal_rotations* are expressed in
     * these frames. The *gravity_direction* is expressed w.r.t. the base frame.
     *
     * @param joint_rotation_axes Vector of rotation axes of the revolute joints of the robot.
     * @param link_lengths Length vectors of the robot links in the default (zero) position of the joints.
     * @param link_masses The masses of the links.
     * @param link_centres_of_mass The centres of mass expressed in the link's own coordinate system.
     * @param principal_link_inertias The principal components of the inertias of each moving link.
     * @param principal_rotations The rotations of the principal components to the frame associated with the link.
     * @param joint_coulomb_dampings The coulomb damping constant of the joints.
     * @param base_pose The position and orientation of the robot's base in the space.
     * @param gravity_direction The direction of the gravitational acceleration.
     * @param normalise_joint_rotation_axes Determines if the 'joint_rotation_axes' parameter should be normalised before using.
     */
    RobotModel(
      const srl::types::Matrix3X<FP> &joint_rotation_axes,
      const srl::types::Matrix3X<FP> &link_lengths,
      const std::vector<FP> &link_masses,
      const srl::types::Matrix3X<FP> &link_centres_of_mass,
      const srl::types::Matrix3X<FP> &principal_link_inertias,
      const srl::types::Matrix3X<FP> &principal_rotations,
      const std::vector<FP> &joint_coulomb_dampings,
      const srl::types::QuatVec3<FP> &base_pose        = { srl::types::Quat<FP>::Identity(), srl::types::Vector3<FP>::Zero() },
      const srl::types::Vector3<FP> &gravity_direction = { 0, 0, -srl::constants::kG<FP> },
      const bool normalise_joint_rotation_axes         = false);

    /**
     * @brief Construct a new Robot Model object.
     *
     * @param number_of_joints The number of joints.
     */
    RobotModel(const uint32_t &number_of_joints = 0);

    //
    // =============== =============== Kinematics Calculations
private:
    // --------------- helpers

    // Populate qRots_ with rotation quaternions representing the rotation of joints. The 0-th element represents the base rotation.
    template<srl::concepts::indexable Container>
    auto populateRots(const Container &joint_angles, const bool wrt_rbt_base = false) -> void;

    // Perform inclusive (multiplication) scan on qRots_ and save the results back to qRots_. This represents rotations of joint frames.
    auto scanRots(const bool wrt_rbt_base = false) -> void;

    // Performs vPos_[i+1] = vPos_[i] + qRots_[i] * frameCoos_[i]. This represents translations of joint frames.
    // The 0-th element represents the base translation.
    auto populatePos(const bool wrt_rbt_base) -> void;

    // Calls populateRots, scanRots, and populatePos consecutively.
    template<srl::concepts::indexable Container>
    auto makeFk(const Container &joint_angles, const bool wrt_rbt_base = false) -> void;

    // Saves rotations and translations of joint frames to frameTfs_.
    auto populateQVs() -> void;

    // Saves rotations and translations of joint frames to frameQuatVec3s_.
    auto populateTfs() -> void;

public:
    // --------------- simple FK

    /**
     * @brief Compute the forward kinematics of the flange.
     *
     * **Example**
     * \snippet example_model.cpp example_fkQuat
     * **Example Output**
     * \snippet example_model.cpp example_fkQuat_output
     *
     * @tparam Container Indexable container type holding the joint angles.
     * @param joint_angles Joint angles for which to compute the forward kinematics.
     * @param wrt_rbt_base If true, the computation is performed with respect to the robot base, hence, the robot base position and orientation
     * is not considered in the calculation. Otherwise, the result is 'pre-transformed' by the position and orientation of the base.
     * @return srl::types::QuatVec3<FP> The position and orientation of the flange.
     */
    template<srl::concepts::indexable Container>
    auto fkQuat(const Container &joint_angles, const bool wrt_rbt_base = false) -> srl::types::QuatVec3<FP>;

    /**
     * @brief Compute the forward kinematics of the flange.
     *
     * **Example**
     * \snippet example_model.cpp example_fk
     * **Example Output**
     * \snippet example_model.cpp example_fk_output
     *
     * @tparam Container Indexable container type holding the joint angles.
     * @param joint_angles Joint angles for which to compute the forward kinematics.
     * @param wrt_rbt_base Indicates with respect to which frame is the computation performed. (For detail see RobotModel::fkQuat.)
     * @return srl::types::Transform3<FP> The position and orientation of the flange.
     */
    template<srl::concepts::indexable Container>
    auto fk(const Container &joint_angles, const bool wrt_rbt_base = false) -> srl::types::Transform3<FP>;

    // --------------- all FK

    /**
     * @brief Compute the forward kinematics of each link.
     *
     * For performance reasons, this function returns a reference to the class' internal object.
     *
     * **Example**
     * \snippet example_model.cpp example_fkAllQuat
     * **Example Output**
     * \snippet example_model.cpp example_fkAllQuat_output
     *
     * @tparam Container Indexable container type holding the joint angles.
     * @param joint_angles Joint angles for which to compute the forward kinematics.
     * @param wrt_rbt_base Indicates with respect to which frame is the computation performed. (For detail see RobotModel::fkQuat.)
     * @return std::vector<srl::types::QuatVec3<FP>>& A vector reference containing position and orientation of each link of the robot (incl. the
     * base).
     */
    template<srl::concepts::indexable Container>
    auto fkAllQuat(const Container &joint_angles, const bool wrt_rbt_base = false) -> const std::vector<srl::types::QuatVec3<FP>> &;

    /**
     * @brief Compute the forward kinematics of each link.
     *
     * For performance reasons, this function returns a reference to the class' internal object.
     *
     * **Example**
     * \snippet example_model.cpp example_fkAll
     * **Example Output**
     * \snippet example_model.cpp example_fkAll_output
     *
     * @tparam Container Indexable container type holding the joint angles.
     * @param joint_angles Joint angles for which to compute the forward kinematics.
     * @param wrt_rbt_base Indicates with respect to which frame is the computation performed. (For detail see RobotModel::fkQuat.)
     * @return std::vector<srl::types::Transform3<FP>>& A vector reference containing position and orientation of each link of the robot (incl. the
     * base).
     */
    template<srl::concepts::indexable Container>
    auto fkAll(const Container &joint_angles, const bool wrt_rbt_base = false) -> const std::vector<srl::types::Transform3<FP>> &;

    //
    // =============== =============== Dynamics Calculations
private:
    // --------------- helpers

    // Calculate a blcok upper triangular matrix, cooTf_,  that contains:
    //  - transformation blocks from every joint frame to any other joint frame, and
    //  - derivatives of these transformation blocks wrt. each joint angle.
    template<srl::concepts::indexable Container>
    auto makeBigRot(const Container &joint_angles, bool derivatives = true) -> void;

    // Calculate a block upper triangular matrix, comTfed_, that contains:
    //  - the centre of mass of every link expressed in every joint frame, and
    //  - derivatives of these centre of mass wrt. each joint angle.
    auto makeCentreOfMassCoos(bool derivatives = true) -> void;

    // Calculate:
    //  - the mass matrix, massMatrix_, of the robot, and
    //  - derivatives of the mass matrix wrt. each joint angle.
    auto makeMassMatrix(bool derivatives = true) -> void;

    // Calculate:
    //  - the mass matrix, massMatrix_, of the robot by calling makeMassMatrix,
    //  - the potential energy, potEnergy_, and its derivatives wrt. each joint angle, and
    //  - the Christoffel symbols, christoffelSymbols_, for the robot.
    template<srl::concepts::indexable Container>
    auto makeEomParams(const Container &joint_angles, bool derivatives = true) -> void;

public:
    // --------------- dynamics
    /**
     * @brief Calculates the mass matrix and the potential energy of the robot for the given joint configuration.
     *
     * This function returns deep copies of the values, i.e. the returned objects are owners of the memory.
     *
     * **Example**
     * \snippet example_model.cpp example_getMassMatrixAndPotEnergy
     * **Example Output**
     * \snippet example_model.cpp example_getMassMatrixAndPotEnergy_output
     *
     * @tparam Container Indexable container type holding the joint angles.
     * @param joint_angles Joint angles for which to compute the mass matrix and potential energy.
     * @return std::tuple<srl::types::MatrixX<FP>, FP> The mass matrix and potential energy.
     */
    template<srl::concepts::indexable Container>
    auto getMassMatrixAndPotEnergy(const Container &joint_angles) -> std::tuple<srl::types::MatrixX<FP>, FP>;

    /**
     * @brief Calculate the Lagrangian of the robot for the given joint configuration.
     *
     * **Example**
     * \snippet example_model.cpp example_getLagrangian
     * **Example Output**
     * \snippet example_model.cpp example_getLagrangian_output
     *
     * @tparam Container Indexable container type holding the joint angles and velocities.
     * @param joint_angles Joint angles for which to compute the Lagrangian.
     * @param joint_velocities Joint velocities for which to compute the Lagrangian.
     * @return FP The Lagrangian of the robot.
     */
    template<srl::concepts::indexable Container>
    auto getLagrangian(const Container &joint_angles, const Container &joint_velocities) -> FP;

    /**
     * @brief Calculate the mass matrix with derivatives, Christoffel symbols, and potential energy with derivatives for the given joint
     * configuration; the derivatives are wrt. each joint angle.
     *
     * The returned values are as follows:
     *  - The first element in the returned tuple contains mass matrix, \f$ M \f$, and its derivatives:
     *    - \f$ (M, \frac{\partial M}{\partial \theta_1}, \cdots, \frac{\partial M}{\partial \theta_n}) \f$, where \f$ n \f$ is the number of joints.
     *  - The second element in the returned tuple contains the Christoffel symbols, \f$ \Gamma \f$:
     *    - \f$ (\Gamma_{1jk}, \cdots, \Gamma_{njk}) \f$, where \f$ n \f$ is the number of joints.
     *  - The third element in the returned tuple contains the potential energy, \f$ V \f$, and its derivatives:
     *    - \f$ (V, \frac{\partial V}{\partial \theta_1}, \cdots, \frac{\partial V}{\partial \theta_n}) \f$, where \f$ n \f$ is the number of joints.
     *
     * For performance reasons, this function returns objects that point to the memory allocated by the class' internal objects.
     *
     * **Example**
     * \snippet example_model.cpp example_getEomParams
     * **Example Output**
     * \snippet example_model.cpp example_getEomParams_output
     *
     * @tparam Container Indexable container type holding the joint angles and velocities.
     * @param joint_angles Joint angles for which to compute the Lagrangian.
     * @return std::tuple<const std::vector<srl::types::Map<srl::types::MatrixX<FP>>>, const std::vector<srl::types::Map<srl::types::MatrixX<FP>>>,
     * const srl::types::VectorX<FP>> The mass matrix with derivatives, the Christoffel symbols, and the potential energy with derivatives,
     * respectively.
     */
    template<srl::concepts::indexable Container>
    auto getEomParams(const Container &joint_angles) -> std::tuple<
      const std::vector<srl::types::Map<srl::types::MatrixX<FP>>>,
      const std::vector<srl::types::Map<srl::types::MatrixX<FP>>>,
      const srl::types::VectorX<FP>>;

    /**
     * @brief Calculate the inverse dynamics of the robot.
     *
     * **Example**
     * \snippet example_model.cpp example_getMotorTorque
     * **Example Output**
     * \snippet example_model.cpp example_getMotorTorque_output
     *
     * @tparam Container Indexable container type holding the joint angles, velocities, and accelerations.
     * @param joint_angles Joint angles for which to compute the inverse dynamics.
     * @param joint_velocities Joint velocities for which to compute the inverse dynamics.
     * @param joint_accelerations Joint accelerations for which to compute the inverse dynamics.
     * @return srl::types::VectorX<FP> The motor torques 'necessary to follow' the provided angles, velocities, and accelerations.
     */
    template<srl::concepts::indexable Container>
    auto getMotorTorque(const Container &joint_angles, const Container &joint_velocities, const Container &joint_accelerations)
      -> srl::types::VectorX<FP>;

    /**
     * @brief Calculate the forward dynamics of the robot.
     *
     * **Example**
     * \snippet example_model.cpp example_getAngularAcceleration
     * **Example Output**
     * \snippet example_model.cpp example_getAngularAcceleration_output
     *
     * @tparam Container Indexable container type holding the joint angles, velocities, and motor torques.
     * @param joint_angles Joint angles for which to compute the forward dynamics.
     * @param joint_velocities Joint velocities for which to compute the forward dynamics.
     * @param motor_torques Motor torques for which to compute the forward dybamics.
     * @return srl::types::VectorX<FP> The joint accelerations resulting from the provided angles, velocities, and motor torques.
     */
    template<srl::concepts::indexable Container>
    auto getAngularAcceleration(const Container &joint_angles, const Container &joint_velocities, const Container &motor_torques)
      -> srl::types::VectorX<FP>;

    //
    // =============== =============== Jacobian Calculations
public:
    // --------------- all Jacobian

    /**
     * @brief Calculate the complete Jacobian for rotation and translation of link frames.
     *
     * **Example**
     * \snippet example_model.cpp example_getJacobian
     * **Example Output**
     * \snippet example_model.cpp example_getJacobian_output
     *
     * @tparam Container Indexable container type holding the joint angles, velocities, and motor torques.
     * @param joint_angles Joint angles for which to compute the forward dynamics.
     * @return std::tuple<const std::vector<srl::types::Map<srl::types::MatrixX<FP>>>, const std::vector<srl::types::Map<srl::types::MatrixX<FP>>>>
     * The derivatives of the rotation matrix, the derivatives of the frame coordinates, respectively.
     */
    template<srl::concepts::indexable Container>
    auto getJacobian(const Container &joint_angles)
      -> std::tuple<const std::vector<srl::types::Map<srl::types::MatrixX<FP>>>, const std::vector<srl::types::Map<srl::types::MatrixX<FP>>>>;

    //
    // =============== =============== Setters and Getters
public:
    /// @brief Get the number of joints of the robot.
    /// @return uint32_t The number of joints.
    auto getNumberOfJoints() -> uint32_t;
    /// @brief Get the number of links of the robot.
    /// @return uint32_t The number of links.
    auto getNumberOfLinks() -> uint32_t;

    /// @brief Set the rotation axes of joints.
    /// @param joint_rotation_axes The rotation axes the be set.
    /// @param normalise If the given axes should be normalised (the set axes must be normalised).
    auto setJointRotationAxes(const srl::types::Matrix3X<FP> &joint_rotation_axes, const bool normalise = false) -> void;
    /// @brief Get the rotation axes of joints.
    /// @return srl::types::Matrix3X<FP> The rotation axes.
    auto getJointRotationAxes() -> srl::types::Matrix3X<FP>;

    /// @brief Set the length vectors of the links.
    /// @param link_lengths The length vectors to be set. (See also RobotModel::RobotModel)
    auto setLinkLengths(const srl::types::Matrix3X<FP> &link_lengths) -> void;
    /// @brief Get the length vectors of the links.
    /// @return srl::types::Matrix3X<FP> The length vectors. (See also RobotModel::RobotModel)
    auto getLinkLengths() -> srl::types::Matrix3X<FP>;

    /// @brief Set the link masses.
    /// @param link_masses The link masses to be set.
    template<srl::concepts::indexable Container>
    auto setLinkMasses(const Container &link_masses) -> void;
    /// @brief Get the link masses.
    /// @return std::vector<FP> The link masses.
    auto getLinkMasses() -> srl::types::VectorX<FP>;

    /// @brief Set the centres of mass of the links.
    /// @param link_centres_of_mass The centres of mass to be set.
    auto setLinkCentresOfMasses(const srl::types::Matrix3X<FP> &link_centres_of_mass) -> void;
    /// @brief Get the centres of mass of the links.
    /// @return srl::types::Matrix3X<FP> The centres of mass.
    auto getLinkCentresOfMasses() -> srl::types::Matrix3X<FP>;

    /// @brief Set the inertia tensors of the links.
    /// The inertia tensor must be symmetric, positive-definite, matrix with its principal axes fulfilling the friangle inequalities.
    /// @param link_inertia_tensors The inertia tensors to be set.
    auto setLinkInertiaTensors(const std::vector<srl::types::Matrix3<FP>> &link_inertia_tensors) -> void;
    /// @brief Set the inertia tensors of the links.
    /// The principal axes must fulfill the friangle inequalities.
    /// @param principal_link_inertias The principal components of the inertias of each moving link.
    /// @param principal_rotations The rotations of the principal components to the frame associated with the link.
    auto setLinkInertiaTensors(const srl::types::Matrix3X<FP> &principal_link_inertias, const srl::types::Matrix3X<FP> &principal_rotations) -> void;
    /// @brief Get the inertia tensors of the links.
    /// @return std::vector<srl::types::Matrix3<FP>> The inertia tensors.
    auto getLinkInertiaTensors() -> std::vector<srl::types::Matrix3<FP>>;

    /// @brief Set the Coulomb dampings for the joints.
    /// @param joint_coulomb_dampings The Coulomb dampings to be set.
    template<srl::concepts::indexable Container>
    auto setJointDampings(const Container &joint_coulomb_dampings) -> void;
    /// @brief Get the Coulomb dampings for the joints.
    /// @return std::vector<FP> The Coulomb dampings.
    auto getJointDampings() -> srl::types::VectorX<FP>;

    /// @brief Set the direction of the gravity acceleration.
    /// @param gravity_direction The direction to be set.
    auto setGravityAcc(const srl::types::Vector3<FP> &gravity_direction) -> void;
    /// @brief Get the direction of the gravity acceleration.
    /// @return srl::types::Vector3<FP> The direction of the gravity acceleration.
    auto getGravityAcc() -> srl::types::Vector3<FP>;

    /// @brief Set the base position and rotation.
    /// @param base_tf The base position and rotation to be set.
    auto setBaseTransform(const srl::types::Transform3<FP> &base_tf) -> void;
    /// @brief Get the base position and rotation.
    /// @return srl::types::Transform3<FP> The base position and rotation.
    auto getBaseTransform() -> srl::types::Transform3<FP>;

    /// @brief Set the base position and rotation.
    /// @param base_pose The base position and rotation to be set.
    auto setBasePose(const srl::types::QuatVec3<FP> &base_pose) -> void;
    /// @brief Get the base position and rotation.
    /// @return srl::types::QuatVec3<FP> The base position and rotation.
    auto getBasePose() -> srl::types::QuatVec3<FP>;

    //
    // =============== =============== Helpers
private:
    // Create vector of inertia tensors from provided principal inertias and their rotations.
    auto static makeLinkInertiaTensors(const srl::types::Matrix3X<FP> &principal_link_inertias, const srl::types::Matrix3X<FP> &principal_rotations)
      -> std::vector<srl::types::Matrix3<FP>>;

    // Check the actual value against desired values and throw if not equal.
    template<srl::concepts::integral T>
    auto static check(const T &actual, std::string_view arg_name, const T &desired, std::string_view desired_name = "", const bool runtime = false)
      -> void;

    //
    // =============== =============== Members
private:
    static inline constexpr uint32_t kDim_ { srl::constants::kDim };
    static inline const std::vector<srl::types::Matrix3<FP>> kSo3 { srl::constants::kSo3<FP> };

    // --------------- size variables
    uint32_t const kNumJoints_;
    uint32_t const kNumLinks_ { kNumJoints_ + 1 };
    uint32_t const kNumJointsDims_ { kNumJoints_ * kDim_ };

    // --------------- set/get -able variables
    srl::types::Vector3<FP> gravityAcc_;
    srl::types::Matrix3X<FP> jointRotationAxes_;
    std::vector<srl::types::Matrix3<FP>> So3Axes_ { kNumJoints_ };
    srl::types::Matrix3X<FP> frameCoos_;
    srl::types::QuatVec3<FP> basePose_ { srl::types::Quat<FP>::Identity(), srl::types::Vector3<FP>::Zero() };
    srl::types::VectorX<FP> linkMasses_ { srl::types::VectorX<FP>::Zero(kNumJoints_) };
    srl::types::Matrix3X<FP> linkCoMs_;
    std::vector<srl::types::Matrix3<FP>> linkInertias_;
    srl::types::VectorX<FP> jointDamping_ { srl::types::VectorX<FP>::Zero(kNumJoints_) };

    // --------------- variables derived from set/get -able (used for calculation speed-up)
    srl::types::MatrixX<FP> frameCoosAndLinkCoM_ { srl::types::MatrixX<FP>::Zero(kNumJointsDims_, kNumJoints_) };
    srl::types::MatrixX<FP> frameCoosTriu_ { srl::types::MatrixX<FP>::Zero(kNumJointsDims_, kNumJoints_) };
    srl::types::VectorX<FP> linkMassVec_ { srl::types::VectorX<FP>::Zero(kNumJointsDims_) };

    // --------------- internal state variables for computations
    std::vector<srl::types::Quat<FP>> qRots_ { kNumLinks_ + 1 };
    srl::types::Matrix3X<FP> vPos_ { kDim_, kNumLinks_ + 1 };
    std::vector<srl::types::QuatVec3<FP>> frameQuatVec3s_ { kNumLinks_ + 1 };
    std::vector<srl::types::Transform3<FP>> frameTfs_ { kNumLinks_ + 1 };

    srl::types::MatrixX<FP> dataCooTf_ { srl::types::MatrixX<FP>::Zero(kNumJointsDims_ * kNumJointsDims_, kNumLinks_) };
    srl::types::MatrixX<FP> dataComTfed_ { srl::types::MatrixX<FP>::Zero(kNumJointsDims_ * kNumJoints_, kNumLinks_) };
    srl::types::MatrixX<FP> dataFrameCoosTfed_ { srl::types::MatrixX<FP>::Zero(kNumJointsDims_ * kNumJoints_, kNumLinks_) };
    srl::types::MatrixX<FP> dataRotAxesTfed_ { srl::types::MatrixX<FP>::Zero(kNumJoints_ * kNumJointsDims_, kNumLinks_) };
    srl::types::MatrixX<FP> rotAxesTfedInertia_ { srl::types::MatrixX<FP>::Zero(kNumJoints_, kNumJointsDims_) };
    srl::types::MatrixX<FP> dataQhLcT_ { srl::types::MatrixX<FP>::Zero(kNumJoints_ * kNumJointsDims_, kNumLinks_) };
    srl::types::MatrixX<FP> qhLcTmass_ { srl::types::MatrixX<FP>::Zero(kNumJoints_, kNumJointsDims_) };
    srl::types::MatrixX<FP> dataMassMatrix_ { srl::types::MatrixX<FP>::Zero(kNumJoints_ * kNumJoints_, kNumLinks_) };
    srl::types::MatrixX<FP> dataChristoffelSymbols_ { srl::types::MatrixX<FP>::Zero(kNumJoints_ * kNumJoints_, kNumJoints_) };
    srl::types::VectorX<FP> potEnergy_ { srl::types::VectorX<FP>::Zero(kNumLinks_) };
    srl::types::VectorX<FP> torques_ { srl::types::VectorX<FP>::Zero(kNumJoints_) };

    std::vector<srl::types::Map<srl::types::MatrixX<FP>>> cooTf_;                 // linked to dataCooTf_
    std::vector<srl::types::Map<srl::types::MatrixX<FP>>> comTfed_;               // linked to dataComTfed_
    std::vector<srl::types::Map<srl::types::MatrixX<FP>>> frameCoosTfed_;         // linked to dataFrameCoosTfed_
    std::vector<srl::types::Map<srl::types::MatrixX<FP>>> rotAxesTfed_;           // linked to dataRotAxesTfed_
    std::vector<srl::types::Map<srl::types::MatrixX<FP>>> qhLcT_;                 // linked to dataQhLcT_
    std::vector<srl::types::Map<srl::types::MatrixX<FP>>> massMatrix_;            // linked to dataMassMatrix_
    std::vector<srl::types::Map<srl::types::MatrixX<FP>>> christoffelSymbols_;    // linked to dataChristoffelSymbols_
};

}    // namespace srl::model

#include <srl/models/model-impl.hpp>

// ===== ===== ===== ===== Instantiations
/// tell doxygen to skip from here \cond
extern template class srl::model::RobotModel<float>;
extern template class srl::model::RobotModel<double>;
/// tell doxygen to skip untill here \endcond

#endif    // SRL_MODELS_HPP
