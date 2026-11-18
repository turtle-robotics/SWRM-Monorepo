#ifndef SWRM_FLOCKING_PARAMETERS_HPP
#define SWRM_FLOCKING_PARAMETERS_HPP

#include <boost/multiprecision/cpp_bin_float.hpp>

#include "swrm_flocking_core/types.hpp"

namespace swrm::flocking {
    /**
     * Nominal spacing between neighboring alpha agents.
     */
    inline const hpscalar LATTICE_SCALE = 0.5;
    /**
     * Interaction range as a multiple of LATTICE_SCALE.
     */
    inline const hpscalar INTERACTION_RANGE_SCALE = 1.2;
    /**
     * Distance beyond which two alpha agents do not interact.
     */
    inline const hpscalar INTERACTION_RANGE = INTERACTION_RANGE_SCALE * LATTICE_SCALE;
    /**
     * Smoothing factor applied by sigma_norm near zero.
     */
    inline const hpscalar SIGMA_NORM_SMOOTHNESS = 0.1;

    /**
     * Asymptotic action value for large positive sigma-norm differences.
     */
    inline const hpscalar ACTION_GAIN_A = 5;
    /**
     * Magnitude of the asymptotic action value for large negative sigma-norm
     * differences.
     */
    inline const hpscalar ACTION_GAIN_B = 5;
    /**
     * Normalized distance through which smooth_bump_function remains one.
     */
    inline const hpscalar BUMP_CUTOFF = 0.2;
    /**
     * Gain applied to inter-agent position control.
     */
    inline const hpscalar AGENT_POSITION_GAIN = 1;
    /**
     * Gain applied to inter-agent velocity matching.
     */
    inline const hpscalar AGENT_VELOCITY_GAIN = 2 * boost::multiprecision::sqrt(AGENT_POSITION_GAIN);
    /**
     * Gain applied to gamma-agent position control.
     */
    inline const hpscalar NAVIGATIONAL_POSITION_GAIN = 1.2;
    /**
     * Gain applied to gamma-agent velocity control.
     */
    inline const hpscalar NAVIGATIONAL_VELOCITY_GAIN = 2 * boost::multiprecision::sqrt(NAVIGATIONAL_POSITION_GAIN);

    /**
     * Default time step for simulation and control loops, in simulation time.
     */
    static constexpr double DEFAULT_TIME_STEP = 0.01;
}

#endif // SWRM_FLOCKING_PARAMETERS_HPP
