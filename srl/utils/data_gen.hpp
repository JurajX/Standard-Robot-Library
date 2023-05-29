#ifndef SRL_DATA_GEN_HPP
#define SRL_DATA_GEN_HPP
#pragma once

#include <srl/common/common.hpp>

namespace srl::utils {

/**
 * @brief Generate a set of random vectors.
 *
 * **Example**
 * \snippet example_utils.cpp example_getRandVectors
 * **Example Output**
 * \snippet example_utils.cpp example_getRandVectors_output
 *
 * @tparam FP Floating point type.
 * @param vec_size The size of the generated vectors.
 * @param set_size The size of the generated set.
 * @param low The lower bound on the reandom values.
 * @param high The upper bound on the random values.
 * @param deterministic If true, the seed of the generator is set to 0.
 * @return std::vector<srl::types::VectorX<FP>> The randomly generated set of vectors.
 */
template<srl::concepts::floating_point FP>
auto getRandVectors(const size_t &set_size, const size_t &vec_size, FP low, FP high, bool deterministic = false)
  -> std::vector<srl::types::VectorX<FP>>;

}    // namespace srl::utils

#include <srl/utils/data_gen-impl.hpp>

#endif    // SRL_DATA_GEN_HPP
