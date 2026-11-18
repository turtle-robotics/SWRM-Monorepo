#ifndef SWRM_FLOCKING_SIGMA_NORM_HPP
#define SWRM_FLOCKING_SIGMA_NORM_HPP

#include "swrm_flocking_core/types.hpp"

namespace swrm::flocking {
    /**
     * Returns a smoothed distance.
     *
     * The result is 0 at 0, depends only on the magnitude of `z`, and has a
     * finite derivative at the origin. `SIGMA_NORM_SMOOTHNESS` controls the
     * amount of smoothing.
     *
     * @param z Distance to smooth.
     * @return Smoothed distance.
     */
    hpscalar sigma_norm(const hpscalar& z);
    /**
     * Returns the smoothed length of a vector.
     *
     * The direction of `z` does not affect the result.
     *
     * @param z Vector whose length is measured.
     * @return Smoothed length.
     */
    hpscalar sigma_norm(const vector2d& z);
    /**
     * Returns the gradient of the sigma-norm of a vector.
     *
     * The result points in the direction of `z`. Its length approaches
     * `1 / sqrt(SIGMA_NORM_SMOOTHNESS)` as the length of `z` grows.
     *
     * @param z Point at which the gradient is evaluated.
     * @return Gradient at `z`.
     */
    vector2d sigma_norm_gradient(const vector2d& z);
    /**
     * Maps a vector into the unit disc.
     *
     * The direction of `z` is preserved, and the resulting length is always
     * less than one.
     *
     * @param z Vector to map.
     * @return Vector inside the unit disc.
     */
    vector2d bounded_sigmoidal(const vector2d& z);
    /**
     * Maps a scalar onto the interval (-1, 1).
     *
     * The result is odd, remains strictly between -1 and 1, and approaches
     * those bounds as the magnitude of `z` grows.
     *
     * @param z Value to map.
     * @return Value in (-1, 1).
     */
    hpscalar bounded_sigmoidal(const hpscalar& z);
}

#endif // SWRM_FLOCKING_SIGMA_NORM_HPP
