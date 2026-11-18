#include <gtest/gtest.h>

#include <swrm_flocking_core/types.hpp>
#include <swrm_flocking_core/parameters.hpp>
#include <swrm_flocking_core/smooth_bump_function.hpp>

#include <boost/multiprecision/cpp_bin_float.hpp>

#include <iomanip>

TEST(TestSmoothBumpFunction, SmoothBumpFunctionBelowRange) {
    swrm::flocking::hpscalar z = -0.5;

    swrm::flocking::hpscalar expected_value = 0.0;
    swrm::flocking::hpscalar computed_value = swrm::flocking::smooth_bump_function(z);

    EXPECT_EQ(computed_value, expected_value) 
        << "Smooth bump function value " 
        << computed_value
        << " is incorrect for expected value "
        << expected_value;
}

TEST(TestSmoothBumpFunction, SmoothBumpFunctionAboveRange) {
    swrm::flocking::hpscalar z = 1.5;

    swrm::flocking::hpscalar expected_value = 0.0;
    swrm::flocking::hpscalar computed_value = swrm::flocking::smooth_bump_function(z);

    EXPECT_EQ(computed_value, expected_value) 
        << "Smooth bump function value " 
        << computed_value
        << " is incorrect for expected value "
        << expected_value;
}

TEST(TestSmoothBumpFunction, SmoothBumpFunctionBelowBumpCutoff) {
    swrm::flocking::hpscalar z = 0.1;

    swrm::flocking::hpscalar expected_value = 1.0;
    swrm::flocking::hpscalar computed_value = swrm::flocking::smooth_bump_function(z);

    EXPECT_EQ(computed_value, expected_value) 
        << "Smooth bump function value " 
        << computed_value
        << " is incorrect for expected value "
        << expected_value;
}

TEST(TestSmoothBumpFunction, SmoothBumpFunctionAboveBumpCutoff) {
    swrm::flocking::hpscalar z = 0.8;

    swrm::flocking::hpscalar expected_value = 0.5 - 0.25 * boost::multiprecision::sqrt(swrm::flocking::hpscalar(2.0));
    swrm::flocking::hpscalar computed_value = swrm::flocking::smooth_bump_function(z);

    swrm::flocking::hpscalar epsilon = 1e-6;
    swrm::flocking::hpscalar margin = boost::multiprecision::abs(computed_value - expected_value);

    EXPECT_TRUE(margin < epsilon)
        << "Smooth bump function value "
        << std::setprecision(10)
        << computed_value
        << " is incorrect from expected value "
        << std::setprecision(10)
        << expected_value;
}

TEST(TestSmoothBumpFunction, SmoothBumpFunctionAtZero) {
    swrm::flocking::hpscalar z = 0.0;

    swrm::flocking::hpscalar expected_value = 1.0;
    swrm::flocking::hpscalar computed_value = swrm::flocking::smooth_bump_function(z);

    EXPECT_EQ(computed_value, expected_value) 
        << "Smooth bump function value " 
        << computed_value
        << " is incorrect for expected value "
        << expected_value;
}

TEST(TestSmoothBumpFunction, SmoothBumpFunctionAtOne) {
    swrm::flocking::hpscalar z = 1.0;

    swrm::flocking::hpscalar expected_value = 0.0;
    swrm::flocking::hpscalar computed_value = swrm::flocking::smooth_bump_function(z);

    EXPECT_EQ(computed_value, expected_value) 
        << "Smooth bump function value " 
        << computed_value
        << " is incorrect for expected value "
        << expected_value;
}

TEST(TestSmoothBumpFunction, SmoothBumpFunctionAtBumpCutoff) {
    swrm::flocking::hpscalar z = swrm::flocking::BUMP_CUTOFF;

    swrm::flocking::hpscalar expected_value = 1.0;
    swrm::flocking::hpscalar computed_value = swrm::flocking::smooth_bump_function(z);

    EXPECT_EQ(computed_value, expected_value) 
        << "Smooth bump function value " 
        << computed_value
        << " is incorrect for expected value "
        << expected_value;
}
