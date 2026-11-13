#ifndef SWRM_FLOCKING_ALPHA_AGENT_HPP
#define SWRM_FLOCKING_ALPHA_AGENT_HPP

#include "swrm_flocking_core/types.hpp"
#include "swrm_flocking_core/gamma_agent.hpp"

#include <list>

namespace swrm::flocking {
    using acceleration_update_t = int;

    class alpha_agent {
    private:
        inline static size_t next_id = 0;

        const size_t id;
        vector2d p;
        vector2d v;
        vector2d a;
        acceleration_update_t safe_update;

        void update_safety(const acceleration_update_t check);

    public:
        constexpr static acceleration_update_t UPDATE_NONE = 0;
        constexpr static acceleration_update_t UPDATE_ALPHA = 1;
        constexpr static acceleration_update_t UPDATE_GAMMA = 2;
        constexpr static acceleration_update_t UPDATE_BOTH = 3;

        alpha_agent(const vector2d& p, const vector2d& v) noexcept;

        size_t ID() const noexcept;

        const vector2d& position() const noexcept;
        
        const vector2d& velocity() const noexcept;

        const vector2d& acceleration() const noexcept;

        void inter_agent_control(const std::list<alpha_agent>& agents);
        void gamma_agent_control(const gamma_agent& agent);
    };
}

#endif // SWRM_FLOCKING_ALPHA_AGENT_HPP
