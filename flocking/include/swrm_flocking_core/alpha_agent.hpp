#ifndef SWRM_FLOCKING_ALPHA_AGENT_HPP
#define SWRM_FLOCKING_ALPHA_AGENT_HPP

#include "swrm_flocking_core/types.hpp"

#include <list>

namespace swrm::flocking {
    class alpha_agent {
    private:
        inline static size_t next_id = 0;

        const size_t id;
        vector2d p;
        vector2d v;
        vector2d a;

        hpscalar inter_agent_control(const std::list<alpha_agent>& agents) const;
          
    public:
        alpha_agent(const vector2d& p, const vector2d& v) noexcept;

        size_t ID() const noexcept {
            return this->id;
        }

        const vector2d& position() const noexcept;
        
        const vector2d& velocity() const noexcept;

        const vector2d& acceleration() const noexcept;
    };
}

#endif // SWRM_FLOCKING_ALPHA_AGENT_HPP
