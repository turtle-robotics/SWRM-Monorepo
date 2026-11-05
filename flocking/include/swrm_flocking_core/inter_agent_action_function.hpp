#ifndef SWRM_FLOCKING_INTER_AGENT_ACTION_FUNCTION_HPP
#define SWRM_FLOCKING_INTER_AGENT_ACTION_FUNCTION_HPP

#include "swrm_flocking_core/types.hpp"
#include "swrm_flocking_core/parameters.hpp"
#include "swrm_flocking_core/sigma_norm.hpp"

namespace swrm::flocking {
    const hpscalar SIGMA_INTERACTION_RANGE = sigma_norm(INTERACTION_RANGE);
    const hpscalar SIGMA_LATTICE_SCALE = sigma_norm(LATTICE_SCALE);
    /**
     * Computes the action function for a pair of agents.
     *
     * The value is the smooth bump weight at `sigma_norm(z) /
     * SIGMA_INTERACTION_RANGE` multiplied by the uneven sigmoidal response at
     * `z - SIGMA_LATTICE_SCALE`. The bump factor reaches zero at the
     * interaction range, which cuts the interaction off.
     *
     * @param z Sigma-norm distance between the two agents.
     * @return Action value for the pair.
     */
    hpscalar inter_agent_action_function(const hpscalar& z);
}

#endif // SWRM_FLOCKING_INTER_AGENT_ACTION_FUNCTION_HPP
