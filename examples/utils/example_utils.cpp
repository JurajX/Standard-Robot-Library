#include <srl/utils/data_gen.hpp>
#include <srl/utils/timers.hpp>

#include <srl/types.hpp>

#include <fmt/format.h>

#include <iostream>

int main(int, char *[])
{
    fmt::print("----- Start of the example.\n");

    {
        /// [example_getRandVectors]
        auto res { srl::utils::getRandVectors(3, 5, 0.0, 10.0, true) };
        std::cout << "The randomly generated set of size 3 with vector size 5 and elements in the range [0, 10] is:\n";
        for (auto &vec : res) {
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

    fmt::print("----- End of the example.\n");

    return 0;
}
