#ifndef SWRM_FLOCKING_INTER_AGENT_ACTION_FUNCTION_HPP
#define SWRM_FLOCKING_INTER_AGENT_ACTION_FUNCTION_HPP

#include "swrm_flocking_core/types.hpp"
#include "swrm_flocking_core/parameters.hpp"
#include "swrm_flocking_core/sigma_norm.hpp"

namespace swrm::flocking {
    /**
     * Sigma-norm value of INTERACTION_RANGE.
     */
    const hpscalar SIGMA_INTERACTION_RANGE = sigma_norm(INTERACTION_RANGE);
    /**
     * Sigma-norm value of LATTICE_SCALE.
     */
    const hpscalar SIGMA_LATTICE_SCALE = sigma_norm(LATTICE_SCALE);
    /**
     * Computes the action between two agents at a given sigma-norm distance.
     *
     * The bump weight fades the action to zero at `INTERACTION_RANGE`, so
     * agents beyond that range do not affect each other.
     *
     * @param z Sigma-norm distance between the agents.
     * @return Action value for the pair.
     */
    hpscalar inter_agent_action_function(const hpscalar& z);
}

#endif // SWRM_FLOCKING_INTER_AGENT_ACTION_FUNCTION_HPP
