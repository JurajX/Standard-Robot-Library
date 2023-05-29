#ifndef SRL_COMMON_CONCEPTS_HPP
#define SRL_COMMON_CONCEPTS_HPP
#pragma once

#include <Eigen/Core>

#include <concepts>
#include <type_traits>

namespace srl::concepts {

/// @brief Concept for floating point type.
using std::floating_point;

/// @brief Concept for integral type.
using std::integral;

/// @brief Concept for invocable type.
using std::invocable;

/// @brief Concept for a generic matrix type.
template<typename T>
concept matrix = std::derived_from<T, Eigen::Matrix<typename T::Scalar, T::RowsAtCompileTime, T::ColsAtCompileTime, Eigen::AutoAlign>>;

/// @brief Concept for a static matrix type; i.e. size known at compile time. (See also srl::concepts::matrix.)
template<typename T>
concept static_matrix = matrix<T> && (T::RowsAtCompileTime != Eigen::Dynamic) && (T::ColsAtCompileTime != Eigen::Dynamic);

/// @brief Concept for a dynamic matrix type; i.e. size not known at compile time.
template<typename T>
concept dynamic_matrix = std::derived_from<T, Eigen::Matrix<typename T::Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::AutoAlign>>;

/// @brief Concept for a square static matrix type; i.e. size known at compile time, and rows == cols. (See also srl::concepts::static_matrix.)
template<typename T>
concept square_static_matrix = static_matrix<T> && (T::RowsAtCompileTime == T::ColsAtCompileTime);

/// @brief Concept for a dynamic or square static matrix type. (See also srl::concepts::square_static_matrix and srl::concepts::dynamic_matrix.)
template<typename T>
concept dynamic_or_square_matrix = square_static_matrix<T> || dynamic_matrix<T>;

/// @brief Concept for an indexable container (class).
template<typename T>
concept indexable = requires(T x, size_t idx) {
    typename T::value_type;
    {
        x.operator[](idx)
    } -> std::same_as<typename T::value_type &>;
    {
        x.size()
    } -> std::convertible_to<size_t>;
};

}    // namespace srl::concepts

#endif    // SRL_COMMON_CONCEPTS_HPP
