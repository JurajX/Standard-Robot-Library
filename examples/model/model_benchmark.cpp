#include <srl/models/model.hpp>

#include <srl/utils/data_gen.hpp>
#include <srl/utils/timers.hpp>

#include <resources/robot_params.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <tuple>

auto main(int /*argc*/, char ** /*argv*/) -> int    // NOLINT (bugprone-exception-escape)
{
    fmt::print("----- Start of the benchmark\n");
    const size_t SIZE = 500'000;
    fmt::print("----- ----- Creating testing sets of size {}.\n", SIZE);

    using FP = double;
    auto set { srl::utils::getRandVectors<FP>(SIZE, srl::resources::panda::numLinks, -srl::constants::kPI<FP>, srl::constants::kPI<FP>, true) };

    fmt::print("----- ----- Creating the model\n");
    srl::model::RobotModel<FP> model {
        srl::resources::panda::jointRotAxes<FP>, srl::resources::panda::frameCoos<FP>,         srl::resources::panda::masses<FP>,
        srl::resources::panda::linkCoM<FP>,      srl::resources::panda::principalInertias<FP>, srl::resources::panda::rotationOfPrincipalAxes<FP>,
        srl::resources::panda::damping<FP>
    };
    using namespace std::placeholders;

    fmt::print("----- ----- ----- Timing forward kinematics for flange.\n");
    srl::utils::timeFn(
      "wrt. robot base quat",
      [&model](const auto &val) { model.fkQuat(val, true); },
      set);
    srl::utils::timeFn(
      "wrt. world frame quat",
      [&model](const auto &val) { model.fkQuat(val, false); },
      set);
    srl::utils::timeFn(
      "wrt. robot base",
      [&model](const auto &val) { model.fk(val, true); },
      set);
    srl::utils::timeFn(
      "wrt. world frame",
      [&model](const auto &val) { model.fk(val, false); },
      set);

    fmt::print("----- ----- ----- Timing forward kinematics for all links.\n");
    srl::utils::timeFn(
      "wrt. robot base, quat",
      [&model](const auto &val) { model.fkAllQuat(val, true); },
      set);
    srl::utils::timeFn(
      "wrt. world frame, quat",
      [&model](const auto &val) { model.fkAllQuat(val, false); },
      set);
    srl::utils::timeFn(
      "wrt. robot base",
      [&model](const auto &val) { model.fkAll(val, true); },
      set);
    srl::utils::timeFn(
      "wrt. world frame",
      [&model](const auto &val) { model.fkAll(val, false); },
      set);

    fmt::print("----- ----- ----- Timing dynamics.\n");
    srl::utils::timeFn(
      "Mass matrix, potential energy",
      [&model](const auto &val) { model.getMassMatrixAndPotEnergy(val); },
      set);
    srl::utils::timeFn(
      "Lagrangian",
      [&model](const auto &val) { model.getLagrangian(val, val); },
      set);
    srl::utils::timeFn(
      "Params for Equation of Motion",
      [&model](const auto &val) { model.getEomParams(val); },
      set);
    srl::utils::timeFn(
      "Inverse dynamics",
      [&model](const auto &val) { model.getMotorTorque(val, val, val); },
      set);
    srl::utils::timeFn(
      "Forward dynamics",
      [&model](const auto &val) { model.getAngularAcceleration(val, val, val); },
      set);

    fmt::print("----- ----- ----- Timing Jacobian.\n");
    srl::utils::timeFn(
      "Rot. and Transl. Jacobian",
      [&model](const auto &val) { model.getJacobian(val); },
      set);

    fmt::print("----- End of the benchmark.\n");
    return 0;
}
