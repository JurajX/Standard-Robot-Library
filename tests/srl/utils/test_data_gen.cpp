#include <gtest/gtest.h>

#include <srl/utils/data_gen.hpp>

auto main(int argc, char **argv) -> int
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
