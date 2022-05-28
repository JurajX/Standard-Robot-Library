#include <srl/models/model.hpp>

#include <srl/utils/data_gen.hpp>
#include <srl/utils/timers.hpp>

#include <resources/robot_params.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <tuple>

int main(int, char *[])
{
    fmt::print("----- Start of the benchmark\n");
    size_t SIZE = 500'000;
    fmt::print("----- ----- Creating testing sets of size {}.\n", SIZE);

    using FP = double;
    auto set1 { srl::utils::getRandVectors<FP>(SIZE, 7, -srl::kPI<FP>, srl::kPI<FP>, true) };

    using Cont = decltype(set1)::value_type;
    srl::vector<std::tuple<Cont, Cont>> set2;
    set2.reserve(SIZE);
    std::transform(set1.begin(), set1.end(), std::back_inserter(set2), [](Cont vec) -> std::tuple<Cont, Cont> { return { vec, vec }; });

    srl::vector<std::tuple<Cont, Cont, Cont>> set3;
    set3.reserve(SIZE);
    std::transform(set1.begin(), set1.end(), std::back_inserter(set3), [](Cont vec) -> std::tuple<Cont, Cont, Cont> { return { vec, vec, vec }; });

    fmt::print("----- ----- Creating the model\n");
    srl::model::RobotModel<FP> model {
        srl::resources::panda::jointRotAxes<FP>, srl::resources::panda::frameCoos<FP>,         srl::resources::panda::masses<FP>,
        srl::resources::panda::linkCoM<FP>,      srl::resources::panda::principalInertias<FP>, srl::resources::panda::rotationOfPrincipalAxes<FP>,
        srl::resources::panda::damping<FP>
    };
    using namespace std::placeholders;

    fmt::print("----- ----- ----- Timing forward kinematics for flange.\n");
    srl::utils::timeMemberFn("wrt. robot base quat", std::bind(&srl::model::RobotModel<FP>::fkQuat<Cont>, _1, _2, true), model, set1);
    srl::utils::timeMemberFn("wrt. world frame quat", std::bind(&srl::model::RobotModel<FP>::fkQuat<Cont>, _1, _2, false), model, set1);
    srl::utils::timeMemberFn("wrt. robot base", std::bind(&srl::model::RobotModel<FP>::fk<Cont>, _1, _2, true), model, set1);
    srl::utils::timeMemberFn("wrt. world frame", std::bind(&srl::model::RobotModel<FP>::fk<Cont>, _1, _2, false), model, set1);

    fmt::print("----- ----- ----- Timing forward kinematics for all links.\n");
    srl::utils::timeMemberFn("wrt. robot base, quat", std::bind(&srl::model::RobotModel<FP>::fkAllQuat<Cont>, _1, _2, true), model, set1);
    srl::utils::timeMemberFn("wrt. world frame, quat", std::bind(&srl::model::RobotModel<FP>::fkAllQuat<Cont>, _1, _2, false), model, set1);
    srl::utils::timeMemberFn("wrt. robot base", std::bind(&srl::model::RobotModel<FP>::fkAll<Cont>, _1, _2, true), model, set1);
    srl::utils::timeMemberFn("wrt. world frame", std::bind(&srl::model::RobotModel<FP>::fkAll<Cont>, _1, _2, false), model, set1);

    fmt::print("----- ----- ----- Timing dynamics.\n");
    srl::utils::timeMemberFn("Mass matrix, potential energy", &srl::model::RobotModel<FP>::getMassMatrixAndPotEnergy<Cont>, model, set1);
    srl::utils::timeMemberFn("Lagrangian", &srl::model::RobotModel<FP>::getLagrangian<Cont>, model, set2);
    srl::utils::timeMemberFn("Params for Equation of Motion", &srl::model::RobotModel<FP>::getEomParams<Cont>, model, set1);
    srl::utils::timeMemberFn("Inverse dynamics", &srl::model::RobotModel<FP>::getMotorTorque<Cont>, model, set3);
    srl::utils::timeMemberFn("Forward dynamics", &srl::model::RobotModel<FP>::getAngularAcceleration<Cont>, model, set3);

    fmt::print("----- ----- ----- Timing Jacobian.\n");
    srl::utils::timeMemberFn("Rot. and Transl. Jacobian", &srl::model::RobotModel<FP>::getJacobian<Cont>, model, set1);

    fmt::print("----- End of the benchmark.\n");
    return 0;
}
