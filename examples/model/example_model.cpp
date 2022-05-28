#include <srl/models/model.hpp>

#include <srl/utils/data_gen.hpp>
#include <srl/utils/timers.hpp>

#include <resources/robot_params.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <iostream>
#include <tuple>

int main(int, char *[])
{
    fmt::print("----- Start of the example.\n");

    using FP = double;
    srl::model::RobotModel<FP> model {
        srl::resources::panda::jointRotAxes<FP>, srl::resources::panda::frameCoos<FP>,         srl::resources::panda::masses<FP>,
        srl::resources::panda::linkCoM<FP>,      srl::resources::panda::principalInertias<FP>, srl::resources::panda::rotationOfPrincipalAxes<FP>,
        srl::resources::panda::damping<FP>
    };
    size_t nDoF { model.getNumberOfJoints() };

    //
    // =============== =============== Kinematics Examples

    {
        /// [example_fkQuat]
        // srl::model::RobotModel<FP> model { /* ... model parameters for nDoF robot ... */ };
        srl::vector<FP> q(nDoF, 1.1);
        srl::QuatVec3<FP> flange { model.fkQuat(q) };
        std::cout << "The rotation of the flange is " << flange.first << ".\n";
        std::cout << "The position of the flange is " << flange.second.transpose() << ".\n";
        /// [example_fkQuat]
        /// [example_fkQuat_output]
        /*
        The rotation of the flange is 0.772961i + 0.366364j + 0.344632k + 0.386702.
        The position of the flange is 0.332618 0.192598 0.765341.
        */
        /// [example_fkQuat_output]
    }

    {
        /// [example_fk]
        // srl::model::RobotModel<FP> model { /* ... model parameters for nDoF robot ... */ };
        srl::vector<FP> q(nDoF, 1.1);
        srl::Transform3<FP> flange { model.fk(q) };
        std::cout << "The rotation of the flange is\n" << flange.linear() << "\n";
        std::cout << "The position of the flange is " << flange.translation().transpose() << ".\n";
        /// [example_fk]
        /// [example_fk_output]
        /*
        The rotation of the flange is
         0.494014   0.29983   0.81612
         0.832909 -0.432479  -0.34529
         0.249426  0.850332 -0.463382
        The position of the flange is 0.332618 0.192598 0.765341.
        */
        /// [example_fk_output]
    }

    {
        /// [example_fkAllQuat]
        // srl::model::RobotModel<FP> model { /* ... model parameters for nDoF robot ... */ };
        srl::vector<FP> q(nDoF, 1.1);
        auto link_frames { model.fkAllQuat(q) };
        std::cout << "The size of the link_frames vector is " << link_frames.size() << ".\n";
        std::cout << "The rotation of the flange is " << link_frames.back().first << ".\n";
        std::cout << "The position of the flange is " << link_frames.back().second.transpose() << ".\n";
        /// [example_fkAllQuat]
        /// [example_fkAllQuat_output]
        /*
        The size of the link_frames vector is 9.
        The rotation of the flange is 0.772961i + 0.366364j + 0.344632k + 0.386702.
        The position of the flange is 0.332618 0.192598 0.765341.
        */
        /// [example_fkAllQuat_output]
    }

    {
        /// [example_fkAll]
        // srl::model::RobotModel<FP> model { /* ... model parameters for nDoF robot ... */ };
        srl::vector<FP> q(nDoF, 1.1);
        auto link_frames { model.fkAll(q) };
        std::cout << "The size of the link_frames vector is " << link_frames.size() << ".\n";
        std::cout << "The rotation of the flange is\n" << link_frames.back().linear() << "\n";
        std::cout << "The position of the flange is " << link_frames.back().translation().transpose() << ".\n";
        /// [example_fkAll]
        /// [example_fkAll_output]
        /*
        The size of the link_frames vector is 9.
        The rotation of the flange is
         0.494014   0.29983   0.81612
         0.832909 -0.432479  -0.34529
         0.249426  0.850332 -0.463382
        The position of the flange is 0.332618 0.192598 0.765341.
        */
        /// [example_fkAll_output]
    }

    //
    // =============== =============== Dynamics Examples

    {
        /// [example_getMassMatrixAndPotEnergy]
        // srl::model::RobotModel<FP> model { /* ... model parameters for nDoF robot ... */ };
        srl::vector<FP> q(nDoF, 1.1);
        auto [mass_matrix, potential_energy] { model.getMassMatrixAndPotEnergy(q) };
        fmt::print("The size of mass_matrix is {}x{}.\n", mass_matrix.rows(), mass_matrix.cols());
        fmt::print("The value of potential_energy of the system for the given parameters is: {}.\n", potential_energy);
        /// [example_getMassMatrixAndPotEnergy]
        /// [example_getMassMatrixAndPotEnergy_output]
        /*
        The size of mass_matrix is 7x7.
        The value of potential_energy of the system for the given parameters is: 54.24055653981205.
        */
        /// [example_getMassMatrixAndPotEnergy_output]
    }

    {
        /// [example_getLagrangian]
        // srl::model::RobotModel<FP> model { /* ... model parameters for nDoF robot ... */ };
        srl::vector<FP> q(nDoF, 1.1);
        srl::vector<FP> dq(nDoF, 1.1);
        double lagrangian { model.getLagrangian(q, dq) };
        fmt::print("The value of Lagrangian of the system for the given parameters is: {}.\n", lagrangian);
        /// [example_getLagrangian]
        /// [example_getLagrangian_output]
        /*
        The value of Lagrangian of the system for the given parameters is: -51.944515552169676.
        */
        /// [example_getLagrangian_output]
    }

    {
        /// [example_getEomParams]
        // srl::model::RobotModel<FP> model { /* ... model parameters for nDoF robot ... */ };
        srl::vector<FP> q(nDoF, 1.1);
        auto [mass_matrix, christoffel_symbols, potential_energy] { model.getEomParams(q) };
        fmt::print(
          "The size of mass_matrix is {}, size of the inner matrices is {}x{}.\n",
          mass_matrix.size(),
          mass_matrix[0].rows(),
          mass_matrix[0].cols());
        fmt::print(
          "The size of christoffel_symbols is {}, size of the inner matrices is {}x{}.\n",
          christoffel_symbols.size(),
          christoffel_symbols[0].rows(),
          christoffel_symbols[0].cols());
        fmt::print("The size of potential_energy is {}.\n", potential_energy.size());
        /// [example_getEomParams]
        /// [example_getEomParams_output]
        /*
        The size of mass_matrix is 8, size of the inner matrices is 7x7.
        The size of christoffel_symbols is 7, size of the inner matrices is 7x7.
        The size of potential_energy is 8.
        */
        /// [example_getEomParams_output]
    }

    {
        /// [example_getMotorTorque]
        // srl::model::RobotModel<FP> model { /* ... model parameters for nDoF robot ... */ };
        srl::vector<FP> q(nDoF, 1.1);
        srl::vector<FP> dq(nDoF, 1.1);
        srl::vector<FP> ddq(nDoF, 1.1);
        srl::VectorX<FP> motor_torques { model.getMotorTorque(q, dq, ddq) };
        std::cout << "The motor torques needed for sustaining the given q, dq, and ddq are: \n" << motor_torques.transpose() << "\n";
        /// [example_getMotorTorque]
        /// [example_getMotorTorque_output]
        /*
        The motor torques needed for sustaining the given q, dq, and ddq are:
          2.21675  -28.2294  -9.17486  -6.38909 -0.592774 -0.563513   0.11099
        */
        /// [example_getMotorTorque_output]
    }

    {
        /// [example_getAngularAcceleration]
        // srl::model::RobotModel<FP> model { /* ... model parameters for nDoF robot ... */ };
        srl::vector<FP> q(nDoF, 1.1);
        srl::vector<FP> dq(nDoF, 1.1);
        srl::vector<FP> mot(nDoF, 1.1);
        srl::VectorX<FP> angular_accelerations { model.getAngularAcceleration(q, dq, mot) };
        std::cout << "The resulting angular accelerations when in the state q, dq and applying the given motor torques are: \n"
                  << angular_accelerations.transpose() << "\n";
        /// [example_getAngularAcceleration]
        /// [example_getAngularAcceleration_output]
        /*
        The resulting angular accelerations when in the state q, dq and applying the given motor torques are:
         4.98194  33.1436 -23.6524  17.8698  146.168  93.9708  3350.62
        */
        /// [example_getAngularAcceleration_output]
    }

    {
        /// [example_getJacobian]
        // srl::model::RobotModel<FP> model { /* ... model parameters for nDoF robot ... */ };
        srl::vector<FP> q(nDoF, 1.1);
        auto [jacobian_rot, jacobian_tran] { model.getJacobian(q) };
        fmt::print(
          "The size of jacobian_rot is {}, size of the inner matrices is {}x{}.\n",
          jacobian_rot.size(),
          jacobian_rot[0].rows(),
          jacobian_rot[0].cols());
        fmt::print(
          "The size of jacobian_tran is {}, size of the inner matrices is {}x{}.\n",
          jacobian_tran.size(),
          jacobian_tran[0].rows(),
          jacobian_tran[0].cols());
        /// [example_getJacobian]
        /// [example_getJacobian_output]
        /*
        The size of jacobian_rot is 7, size of the inner matrices is 21x21.
        The size of jacobian_tran is 7, size of the inner matrices is 21x7.
        */
        /// [example_getJacobian_output]
    }

    fmt::print("----- End of the example.\n");

    return 0;
}
