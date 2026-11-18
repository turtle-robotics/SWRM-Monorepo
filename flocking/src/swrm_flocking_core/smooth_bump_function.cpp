#include "swrm_flocking_core/smooth_bump_function.hpp"

#include <boost/math/constants/constants.hpp>

#include "swrm_flocking_core/types.hpp"
#include "swrm_flocking_core/parameters.hpp"

namespace swrm::flocking {
    hpscalar smooth_bump_function(const hpscalar& z) {
        if (z >= 0 && z <= BUMP_CUTOFF) {
            return 1.0;
        } else if (z > BUMP_CUTOFF && z < 1) {
            hpscalar numerator = boost::math::constants::pi<hpscalar>() * (z - BUMP_CUTOFF);
            hpscalar denominator = 1.0 - BUMP_CUTOFF;
            return 0.5 * (1.0 + boost::multiprecision::cos(numerator / denominator));
        } else {
            return 0.0;
        }
    }
}
