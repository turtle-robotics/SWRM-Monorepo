#ifndef SWRM_FLOCKING_INTER_AGENT_ACTION_FUNCTION_HPP
#define SWRM_FLOCKING_INTER_AGENT_ACTION_FUNCTION_HPP

#include "swrm_flocking_core/types.hpp"
#include "swrm_flocking_core/parameters.hpp"
#include "swrm_flocking_core/sigma_norm.hpp"

namespace swrm::flocking {
    const hpscalar SIGMA_INTERACTION_RANGE = sigma_norm(INTERACTION_RANGE);
    const hpscalar SIGMA_LATTICE_SCALE = sigma_norm(LATTICE_SCALE);
    hpscalar inter_agent_action_function(const hpscalar& z);
}

#endif // SWRM_FLOCKING_INTER_AGENT_ACTION_FUNCTION_HPP
