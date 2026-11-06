#include "swrm_flocking_core/gamma_agent.hpp"
#include "swrm_flocking_core/types.hpp"

namespace swrm::flocking {
    gamma_agent::gamma_agent(
        const swrm::flocking::vector2d& position, 
        const vector2d& velocity, 
        const vector_function& velocity_parameterization,
        hpscalar time_step = DEFAULT_TIME_STEP
        ) : 
        position(position), 
        velocity(velocity), 
        time(0),
        time_step(time_step),
        velocity_parameterization(velocity_parameterization) {}
    
    const vector2d& gamma_agent::get_position() const noexcept {
        return this->position;
    }
    
    const vector2d& gamma_agent::get_velocity() const noexcept {
        return this->velocity;
    }

    const hpscalar& gamma_agent::get_time() const noexcept {
        return this->time;
    }

    const hpscalar& gamma_agent::x() const noexcept {
        return this->position(0);
    }

    const hpscalar& gamma_agent::y() const noexcept {
        return this->position(1);
    }

    void gamma_agent::update() {
        this->time += this->time_step;
        this->velocity = velocity_parameterization(this->time);
        this->position += this->velocity * this->time_step;
    }
}
