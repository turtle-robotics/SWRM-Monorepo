#ifndef SWRM_FLOCKING_SMOOTH_BUMP_FUNCTION_HPP
#define SWRM_FLOCKING_SMOOTH_BUMP_FUNCTION_HPP

#include "swrm_flocking_core/types.hpp"

namespace swrm::flocking {
    /**
     * Returns a smooth cutoff weight for a normalized distance.
     *
     * The weight is 1 from 0 through `BUMP_CUTOFF`, falls smoothly to 0 between
     * `BUMP_CUTOFF` and 1, and is 0 outside that span.
     *
     * @param z Normalized distance.
     * @return Cutoff weight.
     */
    hpscalar smooth_bump_function(const hpscalar& z);
}

#endif // SWRM_FLOCKING_SMOOTH_BUMP_FUNCTION_HPP
