#include <gtest/gtest.h>

#include <srl/utils/data_gen.hpp>

#include <limits>

class TestDataGens : public ::testing::Test
{
protected:
    const static inline size_t set_size { 100 };
    const static inline size_t vec_size { 10 };
    const static inline double minimum { 0.0 };
    const static inline double maximum { 10.0 };
    const static inline auto set { srl::utils::getRandVectors(set_size, vec_size, minimum, maximum, false) };
};

TEST_F(TestDataGens, TestGetRandVectors)    // NOLINT
{
    EXPECT_EQ(set.size(), set_size);
    EXPECT_EQ(set[0].size(), vec_size);
    double min { std::numeric_limits<double>::max() };
    double max { std::numeric_limits<double>::lowest() };
    for (const auto &vector : set) {
        min = vector.minCoeff() < min ? vector.minCoeff() : min;
        max = vector.maxCoeff() > max ? vector.maxCoeff() : max;
    }
    EXPECT_TRUE(minimum < min);
    EXPECT_TRUE(maximum > max);
}

auto main(int argc, char **argv) -> int
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
