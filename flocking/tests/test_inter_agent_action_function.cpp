#include <gtest/gtest.h>

#include <boost/multiprecision/cpp_bin_float.hpp>
#include <boost/math/special_functions/rsqrt.hpp>
#include <boost/math/constants/constants.hpp>

#include <swrm_flocking_core/types.hpp>
#include <swrm_flocking_core/parameters.hpp>
#include <swrm_flocking_core/inter_agent_action_function.hpp>

TEST(TestInterAgentActionFunction, InterAgentActionFunctionBelowBumpRange) {
    swrm::flocking::hpscalar z = -0.5;

    swrm::flocking::hpscalar expected_value = 0.0;
    swrm::flocking::hpscalar computed_value = swrm::flocking::inter_agent_action_function(z);

    EXPECT_EQ(computed_value, expected_value) 
        << "Inter-agent action function value " 
        << computed_value
        << " is incorrect for expected value "
        << expected_value;
}

TEST(TestInterAgentActionFunction, InterAgentActionFunctionAboveBumpRange) {
    swrm::flocking::hpscalar z = 1.5;

    swrm::flocking::hpscalar expected_value = 0.0;
    swrm::flocking::hpscalar computed_value = swrm::flocking::inter_agent_action_function(z);

    EXPECT_EQ(computed_value, expected_value) 
        << "Inter-agent action function value " 
        << computed_value
        << " is incorrect for expected value "
        << expected_value;
}

TEST(TestInterAgentActionFunction, InterAgentActionFunctionBelowBumpCutoff) {
    swrm::flocking::hpscalar z = 0.1 / swrm::flocking::SIGMA_INTERACTION_RANGE;
    swrm::flocking::hpscalar expected_value = swrm::flocking::ACTION_GAIN_A 
        * z * boost::math::rsqrt((1.0 + z * z));
    swrm::flocking::hpscalar computed_value = swrm::flocking::inter_agent_action_function(z);

    swrm::flocking::hpscalar epsilon = 1e-6;
    swrm::flocking::hpscalar margin = boost::multiprecision::abs(computed_value - expected_value);

    EXPECT_TRUE(margin < epsilon)
        << "Inter-agent action function value "
        << computed_value
        << " is incorrect for expected value "
        << expected_value;
}

TEST(TestInterAgentActionFunction, InterAgentActionFunctionAboveBumpCutoff) {
    swrm::flocking::hpscalar z = 0.8;
    swrm::flocking::hpscalar pz = z / swrm::flocking::SIGMA_INTERACTION_RANGE;
    swrm::flocking::hpscalar dz = z - swrm::flocking::SIGMA_LATTICE_SCALE;
    
    swrm::flocking::hpscalar numerator = boost::math::constants::pi<swrm::flocking::hpscalar>() * (pz - swrm::flocking::BUMP_CUTOFF);
    swrm::flocking::hpscalar denominator = 1.0 - swrm::flocking::BUMP_CUTOFF;
    swrm::flocking::hpscalar uneven_sigmoidal = swrm::flocking::ACTION_GAIN_A * dz * boost::math::rsqrt((1.0 + dz * dz));
    swrm::flocking::hpscalar expected_value = uneven_sigmoidal * (numerator / denominator);
    swrm::flocking::hpscalar computed_value = swrm::flocking::inter_agent_action_function(z);

    swrm::flocking::hpscalar epsilon = 1e-6;
    swrm::flocking::hpscalar margin = boost::multiprecision::abs(computed_value - expected_value);

    EXPECT_TRUE(margin < epsilon)
        << "Inter-agent action function value "
        << computed_value
        << " is incorrect for expected value "
        << expected_value;
}
