#ifndef SWRM_FLOCKING_SIGMA_NORM_HPP
#define SWRM_FLOCKING_SIGMA_NORM_HPP

#include "swrm_flocking_core/types.hpp"

namespace swrm::flocking {
    hpscalar sigma_norm(const hpscalar& z);
    hpscalar sigma_norm(const vector2d& z);
    vector2d sigma_norm_gradient(const vector2d& z);
    vector2d bounded_sigmoidal(const vector2d& z);
    hpscalar bounded_sigmoidal(const hpscalar& z);
}

#endif // SWRM_FLOCKING_SIGMA_NORM_HPP
