#include <gtest/gtest.h>

#include <swrm_flocking_core/types.hpp>
#include <swrm_flocking_core/sigma_norm.hpp>

TEST(TestSigmaNorm, SigmaNormValue) {
    swrm::flocking::vector2d z(90.0, 120.0);
    swrm::flocking::hpscalar expected_value = 150.0;
    swrm::flocking::hpscalar epsilon = 1e-6;
    swrm::flocking::hpscalar margin = boost::multiprecision::abs(swrm::flocking::sigma_norm(z) - expected_value);
    EXPECT_TRUE(margin < epsilon) << "Sigma norm value is incorrect.";
}

TEST(TestSigmaNorm, SigmaNormNegativeValue) {
    swrm::flocking::vector2d z(-90.0, -120.0);
    swrm::flocking::hpscalar expected_value = 150.0;
    swrm::flocking::hpscalar epsilon = 1e-6;
    swrm::flocking::hpscalar margin = boost::multiprecision::abs(swrm::flocking::sigma_norm(z) - expected_value);
    EXPECT_TRUE(margin < epsilon) << "Sigma norm value is incorrect for negative input.";
}
