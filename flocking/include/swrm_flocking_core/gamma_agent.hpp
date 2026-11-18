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
         * Creates a gamma agent that moves along a velocity profile.
         *
         * The simulated time starts at zero. The first call to update()
         * replaces the given velocity with the value sampled from the profile.
         *
         * @param position Initial position.
         * @param velocity Initial velocity.
         * @param velocity_parameterization Maps simulated time to a velocity.
         * @param time_step Time added by each update(). Use
         * `DEFAULT_TIME_STEP` for the usual step.
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
         * @return Position vector.
         */
        const vector2d& position() const noexcept;

        /**
         * Returns the current velocity.
         *
         * @return Velocity vector.
         */
        const vector2d& velocity() const noexcept;

        /**
         * Returns the elapsed simulated time, which starts at zero.
         *
         * @return Elapsed simulated time.
         */
        const hpscalar& time() const noexcept;

        /**
         * Returns the x coordinate of the current position.
         *
         * @return X coordinate.
         */
        const hpscalar& x() const noexcept;

        /**
         * Returns the y coordinate of the current position.
         *
         * @return Y coordinate.
         */
        const hpscalar& y() const noexcept;

        /**
         * Advances the agent by one time step.
         *
         * The function adds `time_step` to the simulated time, samples the
         * velocity profile at the new time, and moves the position by one
         * explicit Euler step using the sampled velocity.
         */
        void update();
    };
}

#endif // SWRM_FLOCKING_GAMMA_AGENT_HPP
