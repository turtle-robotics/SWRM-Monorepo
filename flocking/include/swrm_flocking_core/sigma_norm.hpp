#ifndef SWRM_FLOCKING_SIGMA_NORM_HPP
#define SWRM_FLOCKING_SIGMA_NORM_HPP

#include "swrm_flocking_core/types.hpp"

namespace swrm::flocking {
    /**
     * Computes the sigma-norm of a scalar.
     *
     * The result is `(sqrt(1 + eps * z * z) - 1) / eps`, with `eps` set by
     * `SIGMA_NORM_SMOOTHNESS`. It is quadratic near zero and grows linearly
     * with slope `1 / sqrt(eps)` for large inputs.
     *
     * @param z Scalar to map.
     * @return Sigma-norm of `z`.
     */
    hpscalar sigma_norm(const hpscalar& z);
    /**
     * Computes the sigma-norm of a 2D vector.
     *
     * Applies the same mapping to the Euclidean norm of `z`, so the result
     * depends only on the length of the vector.
     *
     * @param z Vector to map.
     * @return Sigma-norm of the vector length.
     */
    hpscalar sigma_norm(const vector2d& z);
    /**
     * Computes the gradient of the sigma-norm with respect to the vector.
     *
     * The result is `z / sqrt(1 + eps * ||z||^2)`, which preserves the
     * direction of `z`.
     *
     * @param z Point at which to evaluate the gradient.
     * @return Gradient vector.
     */
    vector2d sigma_norm_gradient(const vector2d& z);
    /**
     * Applies the bounded sigmoid to a 2D vector.
     *
     * The result is `z / sqrt(1 + ||z||^2)`, which keeps the direction of `z`
     * and has length below one.
     *
     * @param z Vector to map.
     * @return Vector inside the unit disc.
     */
    vector2d bounded_sigmoidal(const vector2d& z);
    /**
     * Applies the bounded sigmoid to a scalar.
     *
     * The result is `z / sqrt(1 + z * z)`, which rises smoothly from -1 to 1
     * and stays inside that range.
     *
     * @param z Scalar to map.
     * @return Value between -1 and 1.
     */
    hpscalar bounded_sigmoidal(const hpscalar& z);
}

#endif // SWRM_FLOCKING_SIGMA_NORM_HPP
