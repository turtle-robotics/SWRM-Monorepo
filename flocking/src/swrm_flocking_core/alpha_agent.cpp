#include "swrm_flocking_core/alpha_agent.hpp"
#include "swrm_flocking_core/types.hpp"
#include "swrm_flocking_core/parameters.hpp"
#include "swrm_flocking_core/sigma_norm.hpp"
#include "swrm_flocking_core/inter_agent_action_function.hpp"
#include "swrm_flocking_core/smooth_bump_function.hpp"

#include <list>
#include <ranges>

namespace swrm::flocking {
    vector2d alpha_agent::inter_agent_control(const std::list<alpha_agent>& agents) const {
        vector2d position_control(0.0, 0.0);
        vector2d velocity_control(0.0, 0.0);

        for (const auto& [i, agent] : std::views::zip(std::views::iota(0ul), agents)) {
            // Skip if same agent
            if (agent.ID() == this->id) continue;
            if ((this->p - agent.position()).norm() > INTERACTION_RANGE) continue;

            hpscalar sigma_distance = sigma_norm(this->p - agent.position());
            vector2d sigma_gradient = sigma_norm_gradient(this->p - agent.position());
            position_control += inter_agent_action_function(sigma_distance) * sigma_gradient;

            hpscalar spatial_adjacency = smooth_bump_function(sigma_distance / SIGMA_INTERACTION_RANGE);
            velocity_control += spatial_adjacency * (agent.velocity() - this->v);
        }

        return AGENT_POSITION_GAIN * position_control + AGENT_VELOCITY_GAIN * velocity_control;
    }

    vector2d alpha_agent::gamma_agent_control(const gamma_agent& agent) const {

        vector2d position_gain = -NAVIGATIONAL_POSITION_GAIN * bounded_sigmoidal(this->p - agent.position());
        vector2d velocity_gain = -NAVIGATIONAL_VELOCITY_GAIN * (this->v - agent.velocity());

        return position_gain + velocity_gain;
    }

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
}
