#include "swrm_flocking_core/types.hpp"
#include "swrm_flocking_core/sigma_norm.hpp"
#include "swrm_flocking_core/smooth_bump_function.hpp"
#include "swrm_flocking_core/uneven_sigmoidal_function.hpp"
#include "swrm_flocking_core/inter_agent_action_function.hpp"

namespace swrm::flocking {
    hpscalar inter_agent_action_function(const hpscalar& z) {
        return smooth_bump_function(sigma_norm(z) / SIGMA_INTERACTION_RANGE) 
            * uneven_sigmoidal_function(z - SIGMA_LATTICE_SCALE);
    }
}
