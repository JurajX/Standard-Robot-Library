#ifndef SRL_TIMERS_IMPL_HPP
#define SRL_TIMERS_IMPL_HPP
#pragma once

#include <srl/utils/timers.hpp>

#include <fmt/format.h>

#include <functional>
#include <iomanip>
#include <iostream>

namespace srl::utils {

auto printDuration(const std::chrono::duration<double, std::nano> &duration, const size_t &n, std::string_view msg) -> void
{
    using namespace std::chrono_literals;

    std::chrono::duration<double, std::nano> period { duration / n };
    double diff {};
    std::string unit {};
    if (period < 1us) {
        diff = period.count();
        unit = "ns";
    } else if (period < 1ms) {
        diff = period.count() / std::milli::den;
        unit = "us";
    } else if (period < 1s) {
        diff = period.count() / std::micro::den;
        unit = "ms";
    } else {
        diff = period.count() / std::nano::den;
        unit = "s";
    }
    fmt::print("{:>30}: {:10.2f} {}\n", msg, diff, unit);
}

template<class Fn, class... Args>
requires srl::concepts::invocable<Fn, Args...>
auto timeFn(std::string_view msg, Fn &&fn, std::vector<std::tuple<Args...>> &args_vec) -> void
{
    auto start { std::chrono::steady_clock::now() };
    for (auto &args : args_vec) {
        std::apply(fn, args);
    }
    auto finish { std::chrono::steady_clock::now() };
    printDuration(finish - start, args_vec.size(), msg);
}

template<class Fn, class Arg>
requires srl::concepts::invocable<Fn, Arg>
auto timeFn(std::string_view msg, Fn &&fn, std::vector<Arg> &arg_vec) -> void
{
    auto start { std::chrono::steady_clock::now() };
    for (auto &arg : arg_vec) {
        fn(arg);
    }
    auto finish { std::chrono::steady_clock::now() };
    printDuration(finish - start, arg_vec.size(), msg);
}

template<class Fn>
requires srl::concepts::invocable<Fn>
auto timeFn(std::string_view msg, Fn &&fn, const size_t &n) -> void
{
    auto start { std::chrono::steady_clock::now() };
    for (size_t idx = 0; idx < n; idx += 1) {
        fn();
    }
    auto finish { std::chrono::steady_clock::now() };
    printDuration(finish - start, n, msg);
}

}    // namespace srl::utils

#endif    // SRL_TIMERS_IMPL_HPP
