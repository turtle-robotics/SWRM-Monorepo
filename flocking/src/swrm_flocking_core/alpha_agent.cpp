#include "swrm_flocking_core/alpha_agent.hpp"
#include "swrm_flocking_core/types.hpp"
#include "swrm_flocking_core/parameters.hpp"

#include <list>
#include <ranges>

namespace swrm::flocking {
    hpscalar alpha_agent::inter_agent_control(const std::list<alpha_agent>& agents) const {
        hpscalar position_control = 0.0;
        hpscalar velocity_control = 0.0;

        for (const auto& [i, agent] : std::views::zip(std::views::iota(0ul), agents)) {
            // Skip if same agent
            if (agent.ID() == this->id) continue;
            if ((this->p - agent.position()).norm() > INTERACTION_RANGE) continue;

            // TODO: Actually write lmfao
        }

        return AGENT_POSITION_GAIN * position_control + AGENT_VELOCITY_GAIN * velocity_control;
    }

    alpha_agent::alpha_agent(const vector2d& p, const vector2d& v) noexcept : 
        id(next_id++), p(p), v(v), a(0.0, 0.0) {}
    
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
