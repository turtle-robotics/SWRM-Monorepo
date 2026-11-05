#include <gtest/gtest.h>

#include <swrm_flocking_core/types.hpp>
#include <swrm_flocking_core/parameters.hpp>
#include <swrm_flocking_core/uneven_sigmoidal_function.hpp>

TEST(TestUnevenSigmoidalFunction, UnevenSigmoidalFunction) {
    swrm::flocking::hpscalar z = 2.0;

    swrm::flocking::hpscalar expected_value = 0.5
        * (swrm::flocking::ACTION_GAIN_A + swrm::flocking::ACTION_GAIN_B)
        * (2.0 / boost::multiprecision::sqrt(swrm::flocking::hpscalar(5.0)))
        + (swrm::flocking::ACTION_GAIN_A - swrm::flocking::ACTION_GAIN_B);
    swrm::flocking::hpscalar computed_value = swrm::flocking::uneven_sigmoidal_function(z);

    swrm::flocking::hpscalar epsilon = 1e-6;
    swrm::flocking::hpscalar margin = boost::multiprecision::abs(computed_value - expected_value);

    EXPECT_TRUE(margin < epsilon)
        << "Uneven sigmoidal function value "
        << computed_value
        << " is incorrect for expected value "
        << expected_value;
}

TEST(TestUnevenSigmoidalFunction, UnevenSigmoidalFunctionAtNegative) {
    swrm::flocking::hpscalar z = -2.0;

    swrm::flocking::hpscalar expected_value = 0.5
        * (swrm::flocking::ACTION_GAIN_A + swrm::flocking::ACTION_GAIN_B)
        * (-2.0 / boost::multiprecision::sqrt(swrm::flocking::hpscalar(5.0)))
        + (swrm::flocking::ACTION_GAIN_A - swrm::flocking::ACTION_GAIN_B);
    swrm::flocking::hpscalar computed_value = swrm::flocking::uneven_sigmoidal_function(z);

    swrm::flocking::hpscalar epsilon = 1e-6;
    swrm::flocking::hpscalar margin = boost::multiprecision::abs(computed_value - expected_value);

    EXPECT_TRUE(margin < epsilon)
        << "Uneven sigmoidal function value "
        << computed_value
        << " is incorrect for expected value "
        << expected_value;
}

TEST(TestUnevenSigmoidalFunction, UnevenSigmoidalFunctionAtZero) {
    swrm::flocking::hpscalar z = 0.0;

    swrm::flocking::hpscalar expected_value = 0.0;
    swrm::flocking::hpscalar computed_value = swrm::flocking::uneven_sigmoidal_function(z);

    swrm::flocking::hpscalar epsilon = 1e-6;
    swrm::flocking::hpscalar margin = boost::multiprecision::abs(computed_value - expected_value);

    EXPECT_TRUE(margin < epsilon)
        << "Uneven sigmoidal function value "
        << computed_value
        << " is incorrect for expected value "
        << expected_value;
}
