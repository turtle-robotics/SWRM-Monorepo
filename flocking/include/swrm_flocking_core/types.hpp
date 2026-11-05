#ifndef SWRM_FLOCKING_TYPES_HPP
#define SWRM_FLOCKING_TYPES_HPP

#include <boost/multiprecision/cpp_bin_float.hpp>
#include <Eigen/Core>

#include <ranges>
#include <type_traits>

namespace swrm::flocking {
    constexpr unsigned int HP_PRECISION = 100;
    using hpscalar = boost::multiprecision::number<boost::multiprecision::cpp_bin_float<HP_PRECISION>>;

    using vector2d = Eigen::Matrix<hpscalar, 2, 1>;
    
    template <typename T, typename C>
    concept collection_of = std::ranges::range<C> && std::is_same_v<std::ranges::range_value_t<C>, T>;
}
#endif // SWRM_FLOCKING_TYPES_HPP
