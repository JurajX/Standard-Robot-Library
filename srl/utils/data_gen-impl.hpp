#ifndef SRL_DATA_GEN_IMPL_HPP
#define SRL_DATA_GEN_IMPL_HPP
#pragma once

#include <srl/utils/data_gen.hpp>

#include <random>

namespace srl::utils {

// ----- Random vector generator
template<srl::concepts::floating_point FP>
auto getRandVectors(const size_t &set_size, const size_t &vec_size, FP low, FP high, bool deterministic) -> srl::vector<srl::VectorX<FP>>
{
    std::random_device rd;
    std::mt19937 mt { deterministic ? 0 : rd() };
    std::uniform_real_distribution<FP> dist(low, high);
    srl::vector<srl::VectorX<FP>> set;
    set.reserve(set_size);
    for (size_t iElement = 0; iElement < set_size; iElement += 1) {
        srl::VectorX<FP> vec { vec_size };
        for (unsigned int idx = 0; idx < vec_size; idx += 1)
            vec[idx] = dist(mt);
        set.emplace_back(vec);
    }
    return set;
}

}    // namespace srl::utils

#endif    // SRL_DATA_GEN_IMPL_HPP
