#include <gtest/gtest.h>

#include <srl/utils/timers.hpp>

auto main(int argc, char **argv) -> int
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
