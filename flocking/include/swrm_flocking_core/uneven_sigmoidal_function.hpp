#ifndef SWRM_FLOCKING_UNEVEN_SIGMOIDAL_FUNCTION_HPP
#define SWRM_FLOCKING_UNEVEN_SIGMOIDAL_FUNCTION_HPP

#include "swrm_flocking_core/types.hpp"

namespace swrm::flocking {
    /**
     * Returns the sigmoidal response used by the action function.
     *
     * The response is smooth, approaches `ACTION_GAIN_A` for large positive
     * inputs and `-ACTION_GAIN_B` for large negative inputs, and stays
     * continuous when the two gains differ.
     *
     * @param z Input value, usually a sigma-norm distance shifted by the
     * lattice scale.
     * @return Scaled sigmoidal response.
     */
    hpscalar uneven_sigmoidal_function(const hpscalar& z);
}

#endif // SWRM_FLOCKING_UNEVEN_SIGMOIDAL_FUNCTION_HPP
