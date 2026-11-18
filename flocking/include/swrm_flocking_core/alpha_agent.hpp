#ifndef SWRM_FLOCKING_ALPHA_AGENT_HPP
#define SWRM_FLOCKING_ALPHA_AGENT_HPP

#include <ranges>
#include <concepts>

#include "swrm_flocking_core/types.hpp"
#include "swrm_flocking_core/parameters.hpp"
#include "swrm_flocking_core/inter_agent_action_function.hpp"
#include "swrm_flocking_core/smooth_bump_function.hpp"
#include "swrm_flocking_core/gamma_agent.hpp"


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
        template <typename Container> 
            requires std::ranges::range<Container> 
            && std::same_as<std::ranges::range_value_t<Container>, alpha_agent>
        vector2d inter_agent_control(const Container& agents) const {
            vector2d position_control(0.0, 0.0);
            vector2d velocity_control(0.0, 0.0);

            for (const alpha_agent& agent : agents) {
                // Skip if same agent
                if (agent.ID() == this->id) continue;
                if ((this->p - agent.position()).norm() > INTERACTION_RANGE) continue;

                hpscalar sigma_distance = sigma_norm(agent.position() - this->p);
                vector2d sigma_gradient = sigma_norm_gradient(agent.position() - this->p);
                position_control += inter_agent_action_function(sigma_distance) * sigma_gradient;

                hpscalar spatial_adjacency = smooth_bump_function(sigma_distance / SIGMA_INTERACTION_RANGE);
                velocity_control += spatial_adjacency * (agent.velocity() - this->v);
            }

            return AGENT_POSITION_GAIN * position_control + AGENT_VELOCITY_GAIN * velocity_control;

        }
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
