#ifndef SWRM_FLOCKING_PARAMETERS_HPP
#define SWRM_FLOCKING_PARAMETERS_HPP

#include <boost/multiprecision/cpp_bin_float.hpp>

#include "swrm_flocking_core/types.hpp"

namespace swrm::flocking {
    inline const hpscalar LATTICE_SCALE = 0.5;
    inline const hpscalar INTERACTION_RANGE_SCALE = 1.2;
    inline const hpscalar INTERACTION_RANGE = INTERACTION_RANGE_SCALE * LATTICE_SCALE;
    inline const hpscalar SIGMA_NORM_SMOOTHNESS = 0.1;

    inline const hpscalar ACTION_GAIN_A = 5;
    inline const hpscalar ACTION_GAIN_B = 5;
    inline const hpscalar BUMP_CUTOFF = 0.2;
    inline const hpscalar AGENT_POSITION_GAIN = 1;
    inline const hpscalar AGENT_VELOCITY_GAIN = 2 * boost::multiprecision::sqrt(AGENT_POSITION_GAIN);
    inline const hpscalar NAVIGATIONAL_POSITION_GAIN = 1.2;
    inline const hpscalar NAVIGATIONAL_VELOCITY_GAIN = 2 * boost::multiprecision::sqrt(NAVIGATIONAL_POSITION_GAIN);
}

#endif // SWRM_FLOCKING_PARAMETERS_HPP
