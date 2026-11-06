#ifndef SWRM_FLOCKING_GAMMA_AGENT_HPP
#define SWRM_FLOCKING_GAMMA_AGENT_HPP

#include "swrm_flocking_core/types.hpp"

namespace swrm::flocking {
    const hpscalar DEFAULT_TIME_STEP = 0.01;

    class gamma_agent {
    private:
        vector2d position;
        vector2d velocity;
        hpscalar time;
        const hpscalar time_step;
        const vector_function velocity_parameterization;

    public:
        gamma_agent(
                const vector2d& position, 
                const vector2d& velocity, 
                const vector_function& velocity_parameterization,
                hpscalar time_step
            );
        const vector2d& get_position() const noexcept;

        const vector2d& get_velocity() const noexcept;

        const hpscalar& get_time() const noexcept;

        const hpscalar& x() const noexcept;

        const hpscalar& y() const noexcept;

        void update();
    };
}

#endif // SWRM_FLOCKING_GAMMA_AGENT_HPP
