#include "swrm_flocking_core/gamma_agent.hpp"
#include "swrm_flocking_core/types.hpp"

namespace swrm::flocking {
    gamma_agent::gamma_agent(
        const swrm::flocking::vector2d& p, 
        const vector2d& v, 
        const vector_function& velocity_parameterization,
        hpscalar time_step = DEFAULT_TIME_STEP
        ) : 
        p(p), 
        v(v), 
        t(0),
        time_step(time_step),
        velocity_parameterization(velocity_parameterization) {}
    
    const vector2d& gamma_agent::position() const noexcept {
        return this->p;
    }
    
    const vector2d& gamma_agent::velocity() const noexcept {
        return this->v;
    }

    const hpscalar& gamma_agent::time() const noexcept {
        return this->t;
    }

    const hpscalar& gamma_agent::x() const noexcept {
        return this->p(0);
    }

    const hpscalar& gamma_agent::y() const noexcept {
        return this->p(1);
    }

    void gamma_agent::update() {
        this->t += this->time_step;
        this->v = velocity_parameterization(this->t);
        this->p += this->v * this->time_step;
    }
}
