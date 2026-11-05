#include "swrm_flocking_core/uneven_sigmoidal_function.hpp"

#include <boost/multiprecision/cpp_bin_float.hpp>
#include <boost/math/special_functions/rsqrt.hpp>

#include "swrm_flocking_core/types.hpp"
#include "swrm_flocking_core/parameters.hpp"
#include "swrm_flocking_core/sigma_norm.hpp"

namespace swrm::flocking {
    hpscalar uneven_sigmoidal_function(const hpscalar& z) {
        if (ACTION_GAIN_A == ACTION_GAIN_B) return ACTION_GAIN_A * bounded_sigmoidal(z);

        hpscalar c_numerator = boost::multiprecision::abs(ACTION_GAIN_A - ACTION_GAIN_B);
        hpscalar c_denominator = boost::math::rsqrt(4 * ACTION_GAIN_A * ACTION_GAIN_B);
        hpscalar c = c_numerator * c_denominator;
        return 0.5 
            * (ACTION_GAIN_A + ACTION_GAIN_B)
            * bounded_sigmoidal(z)
            + (ACTION_GAIN_A - ACTION_GAIN_B);
    }
}
