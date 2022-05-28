#ifndef SRL_TIMERS_HPP
#define SRL_TIMERS_HPP
#pragma once

#include <srl/concepts.hpp>
#include <srl/types.hpp>

#include <string_view>
#include <tuple>

namespace srl::utils {

/**
 * @brief Time a member function taking multiple arguments.
 *
 * @tparam Fn The function type.
 * @tparam Obj The object type to which the function belongs to.
 * @tparam Args Variadic template for the function argument types.
 * @param msg The message to be printed.
 * @param m_fn The member function to time.
 * @param obj The object instance having the member function.
 * @param args_vec A vector of argument tuples the function is timed on.
 */
template<class Fn, class Obj, class... Args>
requires srl::concepts::invocable<Fn, Obj, Args...>
auto timeMemberFn(std::string_view msg, Fn &&m_fn, Obj obj, srl::vector<std::tuple<Args...>> &args_vec) -> void;

/**
 * @brief Time a member function taking a single argument.
 *
 * @tparam Fn The function type.
 * @tparam Obj The object type to which the function belongs to.
 * @tparam Arg The function argument type.
 * @param msg The message to be printed.
 * @param m_fn The member function to time.
 * @param obj The object instance having the member function.
 * @param arg_vec A vector of arguments the function is timed on.
 */
template<class Fn, class Obj, class Arg>
requires srl::concepts::invocable<Fn, Obj, Arg>
auto timeMemberFn(std::string_view msg, Fn &&m_fn, Obj obj, srl::vector<Arg> &arg_vec) -> void;

/**
 * @brief Time a member function taking no arguments.
 *
 * @tparam Fn The function type.
 * @tparam Obj The object type to which the function belongs to.
 * @param msg The message to be printed.
 * @param m_fn The member function to time.
 * @param obj The object instance having the member function.
 * @param n The number of iterations the function is timed on.
 */
template<class Fn, class Obj>
requires srl::concepts::invocable<Fn, Obj>
auto timeMemberFn(std::string_view msg, Fn &&m_fn, Obj obj, size_t &n) -> void;

/**
 * @brief Time a function taking multiple arguments.
 *
 * @tparam Fn The function type.
 * @tparam Args Variadic template for the function argument types.
 * @param msg The message to be printed.
 * @param fn The function to time.
 * @param args_vec A vector of argument tuples the function is timed on.
 */
template<class Fn, class... Args>
requires srl::concepts::invocable<Fn, Args...>
auto timeFn(std::string_view msg, Fn &&fn, srl::vector<std::tuple<Args...>> &args_vec) -> void;

/**
 * @brief Time a function taking a single argument.
 *
 * @tparam Fn The function type.
 * @tparam Arg The function argument type.
 * @param msg The message to be printed.
 * @param fn The function to time.
 * @param arg_vec A vector of arguments the function is timed on.
 */
template<class Fn, class Arg>
requires srl::concepts::invocable<Fn, Arg>
auto timeFn(std::string_view msg, Fn &&fn, srl::vector<Arg> &arg_vec) -> void;

/**
 * @brief Time a function taking no arguments.
 *
 * @tparam Fn The function type.
 * @param msg The message to be printed.
 * @param fn The function to time.
 * @param n The number of iterations the function is timed on.
 */
template<class Fn>
requires srl::concepts::invocable<Fn>
auto timeFn(std::string_view msg, Fn &&fn, size_t &n) -> void;

}    // namespace srl::utils

#include <srl/utils/timers-impl.hpp>

#endif    // SRL_TIMERS_HPP
