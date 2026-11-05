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

    /**
     * Sums the flocking control contributed by agents within interaction range.
     *
     * The loop skips this agent and any agent farther than
     * `INTERACTION_RANGE`. Both accumulators are still empty, so the result
     * stays at zero until the per-neighbour terms are filled in.
     *
     * @param agents Agents to consider, this one included.
     * @return Control input for this agent.
     */
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
        /**
         * Creates an agent with a position, a velocity, and zero acceleration.
         *
         * @param id Identifier, unique within the flock.
         * @param p Initial position.
         * @param v Initial velocity.
         */
        alpha_agent(size_t id, const vector2d& p, const vector2d& v) noexcept : 
            id(id), position(p), velocity(v), acceleration(0.0, 0.0) {}

        /**
         * Returns the agent identifier.
         *
         * @return Identifier given at construction.
         */
        size_t ID() const noexcept {
            return this->id;
        }

        /**
         * Returns the current position.
         *
         * @return Position vector.
         */
        const vector2d& get_position() const noexcept {
            return this->position;
        }
        
        /**
         * Returns the current velocity.
         *
         * @return Velocity vector.
         */
        const vector2d& get_velocity() const noexcept {
            return this->velocity;
        }

        /**
         * Returns the current acceleration.
         *
         * @return Acceleration vector, zero until an update is implemented.
         */
        const vector2d& get_acceleration() const noexcept {
            return this->acceleration;
        }
    };
}

#endif // SWRM_FLOCKING_ALPHA_AGENT_HPP
