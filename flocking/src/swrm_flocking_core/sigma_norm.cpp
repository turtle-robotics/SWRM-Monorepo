#include "swrm_flocking_core/sigma_norm.hpp"

#include <boost/multiprecision/cpp_bin_float.hpp>
#include <boost/math/special_functions/rsqrt.hpp>

#include "swrm_flocking_core/types.hpp"
#include "swrm_flocking_core/parameters.hpp"

namespace swrm::flocking {
    hpscalar sigma_norm(const vector2d& z) {
        hpscalar z_norm_sq = z.squaredNorm();
        hpscalar radical = boost::multiprecision::sqrt(1.0 + SIGMA_NORM_SMOOTHNESS * z_norm_sq);
        return (1.0 / SIGMA_NORM_SMOOTHNESS) * (radical - 1.0);
    }

    vector2d sigma_norm_gradient(const vector2d& z) {
        hpscalar z_norm_sq = z.squaredNorm();
        hpscalar denominator = boost::math::rsqrt(1.0 + SIGMA_NORM_SMOOTHNESS * z_norm_sq);
        return denominator * z;
    }

    vector2d bounded_sigmoidal(const vector2d& z) {
        hpscalar z_norm_sq = z.squaredNorm();
        hpscalar denominator = boost::math::rsqrt(1.0 + z_norm_sq);
        return denominator * z;
    }
}
