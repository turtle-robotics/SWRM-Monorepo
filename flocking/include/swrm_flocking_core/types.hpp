#ifndef SWRM_FLOCKING_TYPES_HPP
#define SWRM_FLOCKING_TYPES_HPP

#include <boost/multiprecision/mpfr.hpp>

constexpr unsigned int HP_PRECISION = 1000;
using hpscalar = boost::multiprecision::number<boost::multiprecision::mpfr_float_backend<HP_PRECISION>>;

#endif // SWRM_FLOCKING_TYPES_HPP
