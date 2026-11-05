#ifndef SWRM_FLOCKING_ALPHA_AGENT_HPP
#define SWRM_FLOCKING_ALPHA_AGENT_HPP

#include "swrm_flocking_core/types.hpp"
#include "swrm_flocking_core/parameters.hpp"

#include <list>
#include <ranges>

namespace swrm::flocking {
    class alpha_agent {
    private:
        const size_t id;
        vector2d position;
        vector2d velocity;
        vector2d acceleration;

    hpscalar inter_agent_control(const std::list<alpha_agent>& agents) const {
        hpscalar position_control = 0.0;
        hpscalar velocity_control = 0.0;

        for (const auto& [i, agent] : std::views::zip(std::views::iota(0ul), agents)) {
            if (i == this->id) continue;
            if ((this->position - agent.position).norm() > INTERACTION_RANGE) continue;

            // TODO: Implement rest
        }

        return AGENT_POSITION_GAIN * position_control + AGENT_VELOCITY_GAIN * velocity_control;
    }

    public:
        alpha_agent(size_t id, const vector2d& p, const vector2d& v) noexcept : 
            id(id), position(p), velocity(v), acceleration(0.0, 0.0) {}

        size_t ID() const noexcept {
            return this->id;
        }

        const vector2d& get_position() const noexcept {
            return this->position;
        }
        
        const vector2d& get_velocity() const noexcept {
            return this->velocity;
        }

        const vector2d& get_acceleration() const noexcept {
            return this->acceleration;
        }
    };
}

#endif // SWRM_FLOCKING_ALPHA_AGENT_HPP
