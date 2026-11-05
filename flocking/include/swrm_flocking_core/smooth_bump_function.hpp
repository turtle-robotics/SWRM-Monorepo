#ifndef SWRM_FLOCKING_SMOOTH_BUMP_FUNCTION_HPP
#define SWRM_FLOCKING_SMOOTH_BUMP_FUNCTION_HPP

#include "swrm_flocking_core/types.hpp"

namespace swrm::flocking {
    /**
     * Evaluates the smooth bump function at a normalized distance.
     *
     * The function equals one up to `BUMP_CUTOFF`, falls to zero over the rest
     * of the unit interval with a raised cosine, and stays at zero past one. The
     * falling section is `0.5 * (1 + cos(pi * (z - h) / (1 - h)))`, where `h`
     * is `BUMP_CUTOFF`.
     *
     * @param z Normalized distance, expected in `[0, 1]`.
     * @return Weight between zero and one.
     */
    hpscalar smooth_bump_function(const hpscalar& z);
}

#endif // SWRM_FLOCKING_SMOOTH_BUMP_FUNCTION_HPP
