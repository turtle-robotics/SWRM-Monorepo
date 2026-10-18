#include "swrm_flocking_core/sigma_norm.hpp"

#include <boost/multiprecision/cpp_bin_float.hpp>

#include "swrm_flocking_core/types.hpp"
#include "swrm_flocking_core/parameters.hpp"

namespace swrm::flocking {
    hpscalar sigma_norm(const vector2d& z) {
        hpscalar z_norm = z.norm();
        hpscalar radical = boost::multiprecision::sqrt(1.0 + SIGMA_NORM_SMOOTHNESS * z_norm * z_norm);
        return (1.0 / SIGMA_NORM_SMOOTHNESS) * (radical - 1.0);
    }
}
