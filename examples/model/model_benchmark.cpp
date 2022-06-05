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
    auto set { srl::utils::getRandVectors<FP>(SIZE, srl::resources::panda::numLinks, -srl::kPI<FP>, srl::kPI<FP>, true) };

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
      [&model](const auto &q) { model.fkQuat(q, true); },
      set);
    srl::utils::timeFn(
      "wrt. world frame quat",
      [&model](const auto &q) { model.fkQuat(q, false); },
      set);
    srl::utils::timeFn(
      "wrt. robot base",
      [&model](const auto &q) { model.fk(q, true); },
      set);
    srl::utils::timeFn(
      "wrt. world frame",
      [&model](const auto &q) { model.fk(q, false); },
      set);

    fmt::print("----- ----- ----- Timing forward kinematics for all links.\n");
    srl::utils::timeFn(
      "wrt. robot base, quat",
      [&model](const auto &q) { model.fkAllQuat(q, true); },
      set);
    srl::utils::timeFn(
      "wrt. world frame, quat",
      [&model](const auto &q) { model.fkAllQuat(q, false); },
      set);
    srl::utils::timeFn(
      "wrt. robot base",
      [&model](const auto &q) { model.fkAll(q, true); },
      set);
    srl::utils::timeFn(
      "wrt. world frame",
      [&model](const auto &q) { model.fkAll(q, false); },
      set);

    fmt::print("----- ----- ----- Timing dynamics.\n");
    srl::utils::timeFn(
      "Mass matrix, potential energy",
      [&model](const auto &q) { model.getMassMatrixAndPotEnergy(q); },
      set);
    srl::utils::timeFn(
      "Lagrangian",
      [&model](const auto &q) { model.getLagrangian(q, q); },
      set);
    srl::utils::timeFn(
      "Params for Equation of Motion",
      [&model](const auto &q) { model.getEomParams(q); },
      set);
    srl::utils::timeFn(
      "Inverse dynamics",
      [&model](const auto &q) { model.getMotorTorque(q, q, q); },
      set);
    srl::utils::timeFn(
      "Forward dynamics",
      [&model](const auto &q) { model.getAngularAcceleration(q, q, q); },
      set);

    fmt::print("----- ----- ----- Timing Jacobian.\n");
    srl::utils::timeFn(
      "Rot. and Transl. Jacobian",
      [&model](const auto &q) { model.getJacobian(q); },
      set);

    fmt::print("----- End of the benchmark.\n");
    return 0;
}
