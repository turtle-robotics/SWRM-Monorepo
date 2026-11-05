#ifndef SWRM_FLOCKING_UNEVEN_SIGMOIDAL_FUNCTION_HPP
#define SWRM_FLOCKING_UNEVEN_SIGMOIDAL_FUNCTION_HPP

#include "swrm_flocking_core/types.hpp"

namespace swrm::flocking {
    /**
     * Evaluates the uneven sigmoidal function used by the action function.
     *
     * The response mixes the bounded sigmoid with the two action gains:
     * `0.5 * (a + b) * sigma(z) + (a - b)`, where `a` is `ACTION_GAIN_A` and
     * `b` is `ACTION_GAIN_B`. Equal gains reduce the function to
     * `a * sigma(z)`.
     *
     * @param z Input value, typically a sigma-norm distance shifted by the
     * lattice scale.
     * @return Sigmoidal response scaled by the action gains.
     */
    hpscalar uneven_sigmoidal_function(const hpscalar& z);
}

#endif // SWRM_FLOCKING_UNEVEN_SIGMOIDAL_FUNCTION_HPP
