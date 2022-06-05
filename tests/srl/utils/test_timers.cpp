#include <gtest/gtest.h>

#include <srl/utils/timers.hpp>

#include <srl/types.hpp>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class TestTimers : public ::testing::Test
{
protected:
    TestTimers();
    const static inline auto delay { 100us };
    const static inline size_t set_size { 100 };
    srl::vector<int> set { set_size, 1 };                   // NOLINT
    srl::vector<std::tuple<int, int>> set1 { set_size };    // NOLINT
};

TestTimers::TestTimers()
{
    std::transform(set.begin(), set.end(), std::back_inserter(set1), [](int vec) -> std::tuple<int, int> { return { vec, vec }; });
}

TEST_F(TestTimers, TestTimeFnNoArgs)    // NOLINT
{
    // NOLINTNEXTLINE
    EXPECT_NO_THROW(srl::utils::timeFn(
      "timeFn no args",
      []() { std::this_thread::sleep_for(delay); },
      set_size));
}

TEST_F(TestTimers, TestTimeFnWithArgs)    // NOLINT
{
    // NOLINTNEXTLINE
    EXPECT_NO_THROW(srl::utils::timeFn(
      "timeFn with args",
      [](const auto &) { std::this_thread::sleep_for(delay); },
      set));
}

TEST_F(TestTimers, TestTimeFnWithTuples)    // NOLINT
{
    // NOLINTNEXTLINE
    EXPECT_NO_THROW(srl::utils::timeFn(
      "timeFn with args",
      [](const auto &, const auto &) { std::this_thread::sleep_for(delay); },
      set1));
}

auto main(int argc, char **argv) -> int
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
