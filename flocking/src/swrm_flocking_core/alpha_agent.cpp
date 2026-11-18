#include "swrm_flocking_core/alpha_agent.hpp"
#include "swrm_flocking_core/types.hpp"
#include "swrm_flocking_core/parameters.hpp"
#include "swrm_flocking_core/sigma_norm.hpp"


namespace swrm::flocking {
    vector2d alpha_agent::gamma_agent_control(const gamma_agent& agent) const {

        vector2d position_gain = -NAVIGATIONAL_POSITION_GAIN * bounded_sigmoidal(this->p - agent.position());
        vector2d velocity_gain = -NAVIGATIONAL_VELOCITY_GAIN * (this->v - agent.velocity());

        return position_gain + velocity_gain;
    }

    alpha_agent::alpha_agent() noexcept : 
        id(alpha_agent::next_id++), p(0.0, 0.0), v(0.0, 0.0), a(0.0, 0.0) {}

    alpha_agent::alpha_agent(const vector2d& p, const vector2d& v) noexcept : 
        id(alpha_agent::next_id++), p(p), v(v), a(0.0, 0.0) {}
    

    size_t alpha_agent::ID() const noexcept {
        return this->id;
    }
    
    const vector2d& alpha_agent::position() const noexcept {
        return this->p;
    }

    const vector2d& alpha_agent::velocity() const noexcept {
        return this->v;
    }

    const vector2d& alpha_agent::acceleration() const noexcept {
        return this->a;
    }

    void alpha_agent::update(const vector2d& acceleration) {
        this->a = acceleration;
    }

    void alpha_agent::update(const vector2d& position, const vector2d& velocity) {
        this->p = position;
        this->v = velocity;
    }
}
