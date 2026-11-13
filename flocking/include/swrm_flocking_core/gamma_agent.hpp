#ifndef SWRM_FLOCKING_GAMMA_AGENT_HPP
#define SWRM_FLOCKING_GAMMA_AGENT_HPP

#include "swrm_flocking_core/types.hpp"

namespace swrm::flocking {
    const hpscalar DEFAULT_TIME_STEP = 0.01;

    class gamma_agent {
    private:
        vector2d p;
        vector2d v;
        hpscalar t;
        const hpscalar time_step;
        const vector_function velocity_parameterization;

    public:
        /**
         * Creates a gamma agent that follows a velocity profile.
         *
         * Simulated time starts at zero.
         *
         * @param position Initial position.
         * @param velocity Initial velocity.
         * @param velocity_parameterization Function that maps simulated time to
         * a commanded velocity.
         * @param time_step Step applied by each call to update().
         */
        gamma_agent(
                const vector2d& position, 
                const vector2d& velocity, 
                const vector_function& velocity_parameterization,
                hpscalar time_step
            );
        /**
         * Returns the current position.
         *
         * @return Position of this agent.
         */
        const vector2d& position() const noexcept;

        /**
         * Returns the current velocity.
         *
         * @return Velocity of this agent.
         */
        const vector2d& velocity() const noexcept;

        /**
         * Returns the elapsed simulated time.
         *
         * @return Elapsed time, starting at zero.
         */
        const hpscalar& time() const noexcept;

        /**
         * Returns the first component of the position.
         *
         * @return x coordinate.
         */
        const hpscalar& x() const noexcept;

        /**
         * Returns the second component of the position.
         *
         * @return y coordinate.
         */
        const hpscalar& y() const noexcept;

        /**
         * Advances the agent by one time step.
         *
         * Each call adds `time_step` to the simulated time, samples the
         * velocity parameterization at the new time, and moves the position by
         * one explicit Euler step.
         */
        void update();
    };
}

#endif // SWRM_FLOCKING_GAMMA_AGENT_HPP
