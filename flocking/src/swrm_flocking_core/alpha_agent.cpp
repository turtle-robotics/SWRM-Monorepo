#include "swrm_flocking_core/alpha_agent.hpp"
#include "swrm_flocking_core/types.hpp"
#include "swrm_flocking_core/parameters.hpp"
#include "swrm_flocking_core/sigma_norm.hpp"
#include "swrm_flocking_core/inter_agent_action_function.hpp"
#include "swrm_flocking_core/smooth_bump_function.hpp"

#include <list>
#include <ranges>

namespace swrm::flocking {
    void alpha_agent::update_safety(const acceleration_update_t check) {
        if (this->safe_update == UPDATE_BOTH) this->safe_update = check;
        else if (this->safe_update & check) throw std::runtime_error("Double acceleration update from same source!");
        else this->safe_update |= check;

    }

    void alpha_agent::inter_agent_control(const std::list<alpha_agent>& agents) {
        this->update_safety(alpha_agent::UPDATE_ALPHA);

        vector2d position_control(0.0, 0.0);
        vector2d velocity_control(0.0, 0.0);

        for (const auto& [i, agent] : std::views::zip(std::views::iota(0ul), agents)) {
            // Skip if same agent
            if (agent.ID() == this->id) continue;
            if ((this->p - agent.position()).norm() > INTERACTION_RANGE) continue;

            // TODO: Actually write lmfao
            hpscalar sigma_distance = sigma_norm(this->p - agent.position());
            vector2d sigma_gradient = sigma_norm_gradient(this->p - agent.position());
            position_control += inter_agent_action_function(sigma_distance) * sigma_gradient;

            hpscalar spatial_adjacency = smooth_bump_function(sigma_distance / SIGMA_INTERACTION_RANGE);
            velocity_control += spatial_adjacency * (agent.velocity() - this->v);
        }

        this->a = AGENT_POSITION_GAIN * position_control + AGENT_VELOCITY_GAIN * velocity_control;
    }

    void alpha_agent::gamma_agent_control(const gamma_agent& agent) {
        this->update_safety(alpha_agent::UPDATE_GAMMA);

        vector2d position_gain = -NAVIGATIONAL_POSITION_GAIN * bounded_sigmoidal(this->p - agent.position());
        vector2d velocity_gain = -NAVIGATIONAL_VELOCITY_GAIN * (this->v - agent.velocity());

        this->a = position_gain + velocity_gain;
    }

    alpha_agent::alpha_agent(const vector2d& p, const vector2d& v) noexcept : 
        id(alpha_agent::next_id++), 
        p(p), 
        v(v), 
        a(0.0, 0.0), 
        safe_update(alpha_agent::UPDATE_NONE) {}

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
}
