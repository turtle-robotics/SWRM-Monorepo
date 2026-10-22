#include <gtest/gtest.h>

#include <swrm_flocking_core/types.hpp>
#include <swrm_flocking_core/sigma_norm.hpp>

#include <boost/multiprecision/cpp_bin_float.hpp>
#include <boost/math/special_functions/rsqrt.hpp>

TEST(TestSigmaNorm, SigmaNormValue) {
    swrm::flocking::vector2d z(6.0, 8.0);
    swrm::flocking::hpscalar expected_value = 10 * boost::multiprecision::sqrt(11) - 10;
    swrm::flocking::hpscalar epsilon = 1e-6;
    swrm::flocking::hpscalar margin = boost::multiprecision::abs(swrm::flocking::sigma_norm(z) - expected_value);
    EXPECT_TRUE(margin < epsilon) << "Sigma norm value is incorrect.";
}

TEST(TestSigmaNorm, SigmaNormNegativeValue) {
    swrm::flocking::vector2d z(-6.0, -8.0);
    swrm::flocking::hpscalar expected_value = 10 * boost::multiprecision::sqrt(11) - 10;
    swrm::flocking::hpscalar epsilon = 1e-6;
    swrm::flocking::hpscalar margin = boost::multiprecision::abs(swrm::flocking::sigma_norm(z) - expected_value);
    EXPECT_TRUE(margin < epsilon) << "Sigma norm value is incorrect for negative vector input.";
}

TEST(TestSigmaNorm, SigmaNormMixedValue) {
    swrm::flocking::vector2d z(-6.0, 8.0);
    swrm::flocking::hpscalar expected_value = 10 * boost::multiprecision::sqrt(11) - 10;
    swrm::flocking::hpscalar epsilon = 1e-6;
    swrm::flocking::hpscalar margin = boost::multiprecision::abs(swrm::flocking::sigma_norm(z) - expected_value);
    EXPECT_TRUE(margin < epsilon) << "Sigma norm value is incorrect for mixed sign vector input.";
}

TEST(TestSigmaNorm, SigmaNormAtZero) {
    swrm::flocking::vector2d z(0.0, 0.0);
    swrm::flocking::hpscalar expected_value = 0.0;
    swrm::flocking::hpscalar epsilon = 1e-6;
    swrm::flocking::hpscalar margin = boost::multiprecision::abs(swrm::flocking::sigma_norm(z) - expected_value);
    EXPECT_TRUE(margin < epsilon) << "Sigma norm value is incorrect at zero vector input.";
}

TEST(TestSigmaNorm, SigmaNormGradient) {
    swrm::flocking::vector2d z(16.0, 12.0);
    swrm::flocking::hpscalar denominator = boost::math::rsqrt(swrm::flocking::hpscalar(21.0));
    swrm::flocking::vector2d expected_gradient = denominator * z;
    swrm::flocking::vector2d computed_gradient = swrm::flocking::sigma_norm_gradient(z);
    swrm::flocking::hpscalar epsilon = 1e-6;
    EXPECT_TRUE(
        computed_gradient.x() - expected_gradient.x() < epsilon &&
        computed_gradient.y() - expected_gradient.y() < epsilon
    ) << "Sigma norm gradient is incorrect.";
}

TEST(TestSigmaNorm, SigmaNormNegativeVectorGradient) {
    swrm::flocking::vector2d z(-16.0, -12.0);
    swrm::flocking::hpscalar denominator = boost::math::rsqrt(swrm::flocking::hpscalar(21.0));
    swrm::flocking::vector2d expected_gradient = denominator * z;
    swrm::flocking::vector2d computed_gradient = swrm::flocking::sigma_norm_gradient(z);
    swrm::flocking::hpscalar epsilon = 1e-6;
    EXPECT_TRUE(
        computed_gradient.x() - expected_gradient.x() < epsilon &&
        computed_gradient.y() - expected_gradient.y() < epsilon
    ) << "Sigma norm gradient is incorrect for negative vector input.";
}

TEST(TestSigmaNorm, SigmaNormMixedVectorGradient) {
    swrm::flocking::vector2d z(-16.0, 12.0);
    swrm::flocking::hpscalar denominator = boost::math::rsqrt(swrm::flocking::hpscalar(21.0));
    swrm::flocking::vector2d expected_gradient = denominator * z;
    swrm::flocking::vector2d computed_gradient = swrm::flocking::sigma_norm_gradient(z);
    swrm::flocking::hpscalar epsilon = 1e-6;
    EXPECT_TRUE(
        computed_gradient.x() - expected_gradient.x() < epsilon &&
        computed_gradient.y() - expected_gradient.y() < epsilon
    ) << "Sigma norm gradient is incorrect for mixed sign vector input.";
}

TEST(TestSigmaNorm, SigmaNormGradientAtZero) {
    swrm::flocking::vector2d z(0.0, 0.0);
    swrm::flocking::vector2d expected_gradient(0.0, 0.0);
    swrm::flocking::vector2d computed_gradient = swrm::flocking::sigma_norm_gradient(z);
    swrm::flocking::hpscalar epsilon = 1e-6;
    EXPECT_TRUE(
        computed_gradient.x() - expected_gradient.x() < epsilon &&
        computed_gradient.y() - expected_gradient.y() < epsilon
    ) << "Sigma norm gradient is incorrect at zero vector input.";
}

TEST(TestSigmaNorm, BoundedSigmoidal) {
    swrm::flocking::vector2d z(
        boost::multiprecision::sqrt(swrm::flocking::hpscalar(3.0)), 
        boost::multiprecision::sqrt(swrm::flocking::hpscalar(5.0))
    );
    swrm::flocking::hpscalar denominator = 1.0 / swrm::flocking::hpscalar(3.0);
    swrm::flocking::vector2d expected_value = denominator * z;
    swrm::flocking::vector2d computed_value = swrm::flocking::bounded_sigmoidal(z);
    swrm::flocking::hpscalar epsilon = 1e-6;
    EXPECT_TRUE(
        computed_value.x() - expected_value.x() < epsilon &&
        computed_value.y() - expected_value.y() < epsilon
    ) << "Bounded sigmoidal function is incorrect.";
}

TEST(TestSigmaNorm, BoundedSigmoidalNegativeVector) {
    swrm::flocking::vector2d z(
        boost::multiprecision::sqrt(swrm::flocking::hpscalar(-3.0)), 
        boost::multiprecision::sqrt(swrm::flocking::hpscalar(-5.0))
    );
    swrm::flocking::hpscalar denominator = 1.0 / swrm::flocking::hpscalar(3.0);
    swrm::flocking::vector2d expected_value = denominator * z;
    swrm::flocking::vector2d computed_value = swrm::flocking::bounded_sigmoidal(z);
    swrm::flocking::hpscalar epsilon = 1e-6;
    EXPECT_TRUE(
        computed_value.x() - expected_value.x() < epsilon &&
        computed_value.y() - expected_value.y() < epsilon
    ) << "Bounded sigmoidal function is incorrect for negative vector input.";
}

TEST(TestSigmaNorm, BoundedSigmoidalMixedVector) {
    swrm::flocking::vector2d z(
        boost::multiprecision::sqrt(swrm::flocking::hpscalar(-3.0)), 
        boost::multiprecision::sqrt(swrm::flocking::hpscalar(5.0))
    );
    swrm::flocking::hpscalar denominator = 1.0 / swrm::flocking::hpscalar(3.0);
    swrm::flocking::vector2d expected_value = denominator * z;
    swrm::flocking::vector2d computed_value = swrm::flocking::bounded_sigmoidal(z);
    swrm::flocking::hpscalar epsilon = 1e-6;
    EXPECT_TRUE(
        computed_value.x() - expected_value.x() < epsilon &&
        computed_value.y() - expected_value.y() < epsilon
    ) << "Bounded sigmoidal function is incorrect for mixed sign vector input.";
}

TEST(TestSigmaNorm, BoundedSigmoidalAtZero) {
    swrm::flocking::vector2d z(0.0, 0.0);
    swrm::flocking::vector2d expected_value(0.0, 0.0);
    swrm::flocking::vector2d computed_value = swrm::flocking::bounded_sigmoidal(z);
    swrm::flocking::hpscalar epsilon = 1e-6;
    EXPECT_TRUE(
        computed_value.x() - expected_value.x() < epsilon &&
        computed_value.y() - expected_value.y() < epsilon
    ) << "Bounded sigmoidal function is incorrect at zero vector input.";
}
