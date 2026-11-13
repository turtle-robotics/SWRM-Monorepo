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
        alpha_agent(const vector2d& p, const vector2d& v) noexcept;

        size_t ID() const noexcept;

        const vector2d& position() const noexcept;
        
        const vector2d& velocity() const noexcept;

        const vector2d& acceleration() const noexcept;

        void update(const vector2d& acceleration);

        vector2d inter_agent_control(const std::list<alpha_agent>& agents) const;
        vector2d gamma_agent_control(const gamma_agent& agent) const;
    };
}

#endif // SWRM_FLOCKING_ALPHA_AGENT_HPP
