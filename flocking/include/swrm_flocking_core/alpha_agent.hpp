#ifndef SWRM_FLOCKING_ALPHA_AGENT_HPP
#define SWRM_FLOCKING_ALPHA_AGENT_HPP

#include "swrm_flocking_core/types.hpp"
#include "swrm_flocking_core/gamma_agent.hpp"

#include <list>

namespace swrm::flocking {
    class alpha_agent {
    private:
        inline static size_t next_id = 0;

        const size_t id;
        vector2d p;
        vector2d v;
        vector2d a;

    public:
        /**
         * Creates an agent at the given position and velocity.
         *
         * The agent receives an identifier that is unique within the process,
         * and its acceleration starts at zero.
         *
         * @param p Initial position.
         * @param v Initial velocity.
         */
        alpha_agent(const vector2d& p, const vector2d& v) noexcept;

        /**
         * Returns the identifier assigned at construction.
         *
         * @return Identifier of this agent.
         */
        size_t ID() const noexcept;

        /**
         * Returns the current position.
         *
         * @return Position of this agent.
         */
        const vector2d& position() const noexcept;
        
        /**
         * Returns the current velocity.
         *
         * @return Velocity of this agent.
         */
        const vector2d& velocity() const noexcept;

        /**
         * Returns the acceleration command set by update().
         *
         * @return Acceleration of this agent.
         */
        const vector2d& acceleration() const noexcept;

        /**
         * Sets the acceleration command.
         *
         * The command is stored only. Velocity and position change through the
         * caller's integration step.
         *
         * @param acceleration Acceleration to store.
         */
        void update(const vector2d& acceleration);

        /**
         * Computes the control command from nearby agents.
         *
         * Position and velocity terms are summed over the agents within
         * `INTERACTION_RANGE`, then scaled by `AGENT_POSITION_GAIN` and
         * `AGENT_VELOCITY_GAIN`. Entries whose identifier matches this agent
         * are skipped, so this agent may appear anywhere in the list.
         *
         * @param agents Agents to consider as neighbors.
         * @return Acceleration command from the inter-agent terms.
         */
        vector2d inter_agent_control(const std::list<alpha_agent>& agents) const;
        /**
         * Computes the control command that tracks a gamma agent.
         *
         * The position term is bounded by the sigmoid, and the velocity term
         * drives this agent's velocity toward the gamma agent's velocity. Both
         * use the navigational gains.
         *
         * @param agent Gamma agent to track.
         * @return Acceleration command toward the gamma agent.
         */
        vector2d gamma_agent_control(const gamma_agent& agent) const;
    };
}

#endif // SWRM_FLOCKING_ALPHA_AGENT_HPP
