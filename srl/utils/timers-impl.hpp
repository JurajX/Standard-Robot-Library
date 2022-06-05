#ifndef SRL_TIMERS_IMPL_HPP
#define SRL_TIMERS_IMPL_HPP
#pragma once

#include <srl/utils/timers.hpp>

#include <fmt/format.h>

#include <chrono>
#include <functional>
#include <iomanip>
#include <iostream>

namespace srl::utils {

template<class Fn, class... Args>
requires srl::concepts::invocable<Fn, Args...>
auto timeFn(std::string_view msg, Fn &&fn, srl::vector<std::tuple<Args...>> &args_vec) -> void
{
    auto start { std::chrono::steady_clock::now() };
    for (auto &args : args_vec) {
        std::apply(fn, args);
    }
    auto finish { std::chrono::steady_clock::now() };
    double diff { std::chrono::duration<double, std::nano>(finish - start).count() / static_cast<double>(args_vec.size()) };
    fmt::print("{:>30}: {:10.1f} ns\n", msg, diff);
}

template<class Fn, class Arg>
requires srl::concepts::invocable<Fn, Arg>
auto timeFn(std::string_view msg, Fn &&fn, srl::vector<Arg> &arg_vec) -> void
{
    auto start { std::chrono::steady_clock::now() };
    for (auto &arg : arg_vec) {
        fn(arg);
    }
    auto finish { std::chrono::steady_clock::now() };
    double diff { std::chrono::duration<double, std::nano>(finish - start).count() / static_cast<double>(arg_vec.size()) };
    fmt::print("{:>30}: {:10.1f} ns\n", msg, diff);
}

template<class Fn>
requires srl::concepts::invocable<Fn>
auto timeFn(std::string_view msg, Fn &&fn, size_t &n) -> void
{
    auto start { std::chrono::steady_clock::now() };
    for (size_t idx = 0; idx < n; idx += 1) {
        fn();
    }
    auto finish { std::chrono::steady_clock::now() };
    double diff { std::chrono::duration<double, std::nano>(finish - start).count() / static_cast<double>(n) };
    fmt::print("{:>30}: {:10.1f} ns\n", msg, diff);
}

}    // namespace srl::utils

#endif    // SRL_TIMERS_IMPL_HPP
