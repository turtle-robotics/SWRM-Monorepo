#ifndef SWRM_FLOCKING_ALPHA_AGENT_HPP
#define SWRM_FLOCKING_ALPHA_AGENT_HPP

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
         * The agent receives a process-wide unique identifier, and its
         * acceleration command starts at zero.
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
         * @return Position vector.
         */
        const vector2d& position() const noexcept;
        
        /**
         * Returns the current velocity.
         *
         * @return Velocity vector.
         */
        const vector2d& velocity() const noexcept;

        /**
         * Returns the acceleration command set by the last call to update().
         *
         * @return Acceleration vector.
         */
        const vector2d& acceleration() const noexcept;

        /**
         * Stores an acceleration command without changing the position or
         * velocity.
         *
         * @param acceleration Acceleration to store.
         */
        void update(const vector2d& acceleration);

        /**
         * Computes the control command from neighboring agents.
         *
         * Neighbors within `INTERACTION_RANGE` contribute a position term that
         * holds the lattice spacing and a velocity term that matches their
         * velocity. The terms are scaled by `AGENT_POSITION_GAIN` and
         * `AGENT_VELOCITY_GAIN`. If this agent appears in `agents`, its own
         * entry is skipped.
         *
         * @tparam Container Range of alpha_agent values.
         * @param agents Agents that may act as neighbors.
         * @return Acceleration command from the neighboring agents.
         */
        template <typename Container> 
        requires collection_of<alpha_agent, Container>
        vector2d inter_agent_control(const Container& agents) const {
            vector2d position_control(0.0, 0.0);
            vector2d velocity_control(0.0, 0.0);

            for (const alpha_agent& agent : agents) {
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
         * Computes the control command that follows a gamma agent.
         *
         * The position error passes through the bounded sigmoid, which limits
         * the response to large separations, and the velocity error is used
         * directly. The navigational gains scale both terms.
         *
         * @param agent Gamma agent to follow.
         * @return Acceleration command toward the gamma agent.
         */
        vector2d gamma_agent_control(const gamma_agent& agent) const;
    };
}

#endif // SWRM_FLOCKING_ALPHA_AGENT_HPP
