#include <srl/utils/data_gen.hpp>
#include <srl/utils/timers.hpp>

#include <srl/types.hpp>

#include <fmt/format.h>

#include <iostream>
#include <thread>

using namespace std::chrono_literals;

auto main(int /*argc*/, char ** /*argv*/) -> int
{
    fmt::print("----- Start of the example.\n");

    const size_t set_size { 1000 };
    auto set { srl::utils::getRandVectors(set_size, 3, 0.0, 10.0, true) };    // NOLINT

    fmt::print("\n----- ----- Example for getRandVectors.\n");
    {
        /// [example_getRandVectors]
        const auto res { srl::utils::getRandVectors(3, 5, 0.0, 10.0, true) };
        std::cout << "The randomly generated set of size 3 with vector size 5 and elements in the range [0, 10] is:\n";
        for (const auto &vec : res) {
            std::cout << vec.transpose() << '\n';
        }
        /// [example_getRandVectors]
        /// [example_getRandVectors_output]
        /*
        The randomly generated set of size 3 with vector size 5 and elements in the range [0, 10] is:
        5.92845 8.44266 8.57946 8.47252 6.23564
        3.84382 2.97535 0.56713 2.72656 4.77665
        8.12169 4.79977 3.92785 8.36079 3.37396
        */
        /// [example_getRandVectors_output]
    }

    fmt::print("\n----- ----- Example for timeFn no args.\n");
    {
        /// [example_timeFn0]
        const auto delay { 100us };
        srl::utils::timeFn(
          "timeFn no args",
          [&delay]() { std::this_thread::sleep_for(delay); },
          set_size);
        /// [example_timeFn0]
        /// [example_timeFn0_output]
        /*
                timeFn no args:     135.03 us
        */
        /// [example_timeFn0_output]
    }

    fmt::print("\n----- ----- Example for timeFn with args.\n");
    {
        /// [example_timeFn1]
        const auto delay { 100us };
        srl::utils::timeFn(
          "timeFn with args",
          [&delay](const auto &) { std::this_thread::sleep_for(delay); },
          set);
        /// [example_timeFn1]
        /// [example_timeFn1_output]
        /*
              timeFn with args:     134.61 us
        */
        /// [example_timeFn1_output]
    }

    fmt::print("----- End of the example.\n");

    return 0;
}
