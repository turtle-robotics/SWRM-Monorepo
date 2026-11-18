#include <gtest/gtest.h>

#include <boost/multiprecision/cpp_bin_float.hpp>

#include <swrm_flocking_core/types.hpp>
#include <swrm_flocking_core/parameters.hpp>
#include <swrm_flocking_core/alpha_agent.hpp>
#include <swrm_flocking_core/inter_agent_action_function.hpp>
#include <swrm_flocking_core/smooth_bump_function.hpp>

#include <vector>

TEST(TestAlphaAgent, TestConstructorAndGetters) {
    swrm::flocking::vector2d position(0.0, 0.0);
    swrm::flocking::vector2d velocity(1.0, 1.0);
    swrm::flocking::alpha_agent agent(position, velocity);

    EXPECT_EQ(agent.position(), position);
    EXPECT_EQ(agent.velocity(), velocity);
    EXPECT_EQ(agent.acceleration(), swrm::flocking::vector2d(0.0, 0.0));
}

TEST(TestAlphaAgent, TestIDIncrementing) {
    swrm::flocking::vector2d position(0.0, 0.0);
    swrm::flocking::vector2d velocity(1.0, 1.0);
    swrm::flocking::alpha_agent agent1(position, velocity);
    swrm::flocking::alpha_agent agent2(position, velocity);

    EXPECT_EQ(agent2.ID(), agent1.ID() + 1);
}

TEST(TestAlphaAgent, TestUpdateAcceleration) {
    swrm::flocking::vector2d position(0.0, 0.0);
    swrm::flocking::vector2d velocity(1.0, 1.0);
    swrm::flocking::alpha_agent agent(position, velocity);

    swrm::flocking::vector2d new_acceleration(0.5, 0.5);
    agent.update(new_acceleration);

    EXPECT_EQ(agent.acceleration(), new_acceleration);
}

TEST(TestAlphaAgent, TestInterAgentControlSelfOnlyCollection) {
    swrm::flocking::vector2d position(0.0, 0.0);
    swrm::flocking::vector2d velocity(0.2, -0.1);
    swrm::flocking::alpha_agent agent(position, velocity);

    std::vector<swrm::flocking::alpha_agent> agents;
    agents.push_back(agent);

    // Olfati-Saber sum excludes the agent itself, so the collection holds no
    // neighbors and the control command is zero.
    swrm::flocking::vector2d expected_control(0.0, 0.0);
    swrm::flocking::vector2d computed_control = agent.inter_agent_control(agents);
    EXPECT_EQ(computed_control, expected_control)
        << "Inter-agent control "
        << computed_control
        << " is incorrect for expected control "
        << expected_control;
}

TEST(TestAlphaAgent, TestInterAgentControlSingleRobotCollection) {
    swrm::flocking::vector2d position(0.0, 0.0);
    swrm::flocking::vector2d velocity(0.2, -0.1);
    swrm::flocking::vector2d neighbor_position(0.3, 0.2);
    swrm::flocking::vector2d neighbor_velocity(-0.1, 0.4);
    swrm::flocking::alpha_agent agent(position, velocity);
    swrm::flocking::alpha_agent neighbor(neighbor_position, neighbor_velocity);

    std::vector<swrm::flocking::alpha_agent> agents;
    agents.push_back(neighbor);

    // Olfati-Saber (2006), equation (5):
    // u_i = sum_j phi_alpha(||q_j - q_i||_sigma) * n_ij
    //     + sum_j a_ij(q) * (p_j - p_i)
    swrm::flocking::vector2d difference = neighbor_position - position;
    swrm::flocking::hpscalar squared_distance = difference.squaredNorm();
    swrm::flocking::hpscalar smoothing = boost::multiprecision::sqrt(
        1.0 + swrm::flocking::SIGMA_NORM_SMOOTHNESS * squared_distance);
    swrm::flocking::hpscalar sigma_distance = (smoothing - 1.0) / swrm::flocking::SIGMA_NORM_SMOOTHNESS;
    swrm::flocking::vector2d sigma_gradient = difference / smoothing;

    swrm::flocking::hpscalar action = swrm::flocking::inter_agent_action_function(sigma_distance);
    swrm::flocking::hpscalar adjacency = swrm::flocking::smooth_bump_function(
        sigma_distance / swrm::flocking::SIGMA_INTERACTION_RANGE);

    swrm::flocking::vector2d expected_control =
        swrm::flocking::AGENT_POSITION_GAIN * action * sigma_gradient
        + swrm::flocking::AGENT_VELOCITY_GAIN * adjacency * (neighbor_velocity - velocity);
    swrm::flocking::vector2d computed_control = agent.inter_agent_control(agents);

    swrm::flocking::hpscalar epsilon = 1e-6;
    swrm::flocking::hpscalar margin_x = boost::multiprecision::abs(computed_control.x() - expected_control.x());
    swrm::flocking::hpscalar margin_y = boost::multiprecision::abs(computed_control.y() - expected_control.y());
    EXPECT_TRUE(margin_x < epsilon && margin_y < epsilon)
        << "Inter-agent control "
        << computed_control
        << " is incorrect for expected control "
        << expected_control;
}

TEST(TestAlphaAgent, TestInterAgentControlThreeRobotCollection) {
    swrm::flocking::vector2d position(0.0, 0.0);
    swrm::flocking::vector2d velocity(0.2, -0.1);

    std::vector<swrm::flocking::vector2d> neighbor_positions = {
        swrm::flocking::vector2d(0.3, 0.2),
        swrm::flocking::vector2d(-0.2, 0.15),
        swrm::flocking::vector2d(0.1, -0.3)
    };
    std::vector<swrm::flocking::vector2d> neighbor_velocities = {
        swrm::flocking::vector2d(-0.1, 0.4),
        swrm::flocking::vector2d(0.3, -0.2),
        swrm::flocking::vector2d(0.0, 0.1)
    };

    swrm::flocking::alpha_agent agent(position, velocity);
    std::vector<swrm::flocking::alpha_agent> agents;
    for (std::size_t i = 0; i < neighbor_positions.size(); ++i) {
        agents.emplace_back(neighbor_positions[i], neighbor_velocities[i]);
    }

    // Each neighbor contributes to both sums of Olfati-Saber equation (5).
    swrm::flocking::vector2d position_control(0.0, 0.0);
    swrm::flocking::vector2d velocity_control(0.0, 0.0);
    for (std::size_t i = 0; i < neighbor_positions.size(); ++i) {
        swrm::flocking::vector2d difference = neighbor_positions[i] - position;
        swrm::flocking::hpscalar squared_distance = difference.squaredNorm();
        swrm::flocking::hpscalar smoothing = boost::multiprecision::sqrt(
            1.0 + swrm::flocking::SIGMA_NORM_SMOOTHNESS * squared_distance);
        swrm::flocking::hpscalar sigma_distance = (smoothing - 1.0) / swrm::flocking::SIGMA_NORM_SMOOTHNESS;
        swrm::flocking::vector2d sigma_gradient = difference / smoothing;

        position_control += swrm::flocking::inter_agent_action_function(sigma_distance) * sigma_gradient;
        velocity_control += swrm::flocking::smooth_bump_function(
            sigma_distance / swrm::flocking::SIGMA_INTERACTION_RANGE)
            * (neighbor_velocities[i] - velocity);
    }

    swrm::flocking::vector2d expected_control =
        swrm::flocking::AGENT_POSITION_GAIN * position_control
        + swrm::flocking::AGENT_VELOCITY_GAIN * velocity_control;
    swrm::flocking::vector2d computed_control = agent.inter_agent_control(agents);

    swrm::flocking::hpscalar epsilon = 1e-6;
    swrm::flocking::hpscalar margin_x = boost::multiprecision::abs(computed_control.x() - expected_control.x());
    swrm::flocking::hpscalar margin_y = boost::multiprecision::abs(computed_control.y() - expected_control.y());
    EXPECT_TRUE(margin_x < epsilon && margin_y < epsilon)
        << "Inter-agent control "
        << computed_control
        << " is incorrect for expected control "
        << expected_control;
}

TEST(TestAlphaAgent, TestInterAgentControlFourRobotCollectionWithSelf) {
    swrm::flocking::vector2d position(0.0, 0.0);
    swrm::flocking::vector2d velocity(0.2, -0.1);

    std::vector<swrm::flocking::vector2d> neighbor_positions = {
        swrm::flocking::vector2d(0.3, 0.2),
        swrm::flocking::vector2d(-0.2, 0.15),
        swrm::flocking::vector2d(0.1, -0.3)
    };
    std::vector<swrm::flocking::vector2d> neighbor_velocities = {
        swrm::flocking::vector2d(-0.1, 0.4),
        swrm::flocking::vector2d(0.3, -0.2),
        swrm::flocking::vector2d(0.0, 0.1)
    };

    swrm::flocking::alpha_agent agent(position, velocity);
    std::vector<swrm::flocking::alpha_agent> agents;
    agents.emplace_back(neighbor_positions[0], neighbor_velocities[0]);
    agents.push_back(agent);
    agents.emplace_back(neighbor_positions[1], neighbor_velocities[1]);
    agents.emplace_back(neighbor_positions[2], neighbor_velocities[2]);

    // The self entry is skipped wherever it appears, so the expected control
    // matches the sum over the three neighbors only.
    swrm::flocking::vector2d position_control(0.0, 0.0);
    swrm::flocking::vector2d velocity_control(0.0, 0.0);
    for (std::size_t i = 0; i < neighbor_positions.size(); ++i) {
        swrm::flocking::vector2d difference = neighbor_positions[i] - position;
        swrm::flocking::hpscalar squared_distance = difference.squaredNorm();
        swrm::flocking::hpscalar smoothing = boost::multiprecision::sqrt(
            1.0 + swrm::flocking::SIGMA_NORM_SMOOTHNESS * squared_distance);
        swrm::flocking::hpscalar sigma_distance = (smoothing - 1.0) / swrm::flocking::SIGMA_NORM_SMOOTHNESS;
        swrm::flocking::vector2d sigma_gradient = difference / smoothing;

        position_control += swrm::flocking::inter_agent_action_function(sigma_distance) * sigma_gradient;
        velocity_control += swrm::flocking::smooth_bump_function(
            sigma_distance / swrm::flocking::SIGMA_INTERACTION_RANGE)
            * (neighbor_velocities[i] - velocity);
    }

    swrm::flocking::vector2d expected_control =
        swrm::flocking::AGENT_POSITION_GAIN * position_control
        + swrm::flocking::AGENT_VELOCITY_GAIN * velocity_control;
    swrm::flocking::vector2d computed_control = agent.inter_agent_control(agents);

    swrm::flocking::hpscalar epsilon = 1e-6;
    swrm::flocking::hpscalar margin_x = boost::multiprecision::abs(computed_control.x() - expected_control.x());
    swrm::flocking::hpscalar margin_y = boost::multiprecision::abs(computed_control.y() - expected_control.y());
    EXPECT_TRUE(margin_x < epsilon && margin_y < epsilon)
        << "Inter-agent control "
        << computed_control
        << " is incorrect for expected control "
        << expected_control;
}
