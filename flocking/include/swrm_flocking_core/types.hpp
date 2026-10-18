#ifndef SWRM_FLOCKING_TYPES_HPP
#define SWRM_FLOCKING_TYPES_HPP

#include <boost/multiprecision/cpp_bin_float.hpp>

namespace swrm::flocking {
    constexpr unsigned int HP_PRECISION = 100;
    using hpscalar = boost::multiprecision::number<boost::multiprecision::cpp_bin_float<HP_PRECISION>>;
}
#endif // SWRM_FLOCKING_TYPES_HPP
