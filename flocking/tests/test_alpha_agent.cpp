#include <gtest/gtest.h>

#include <swrm_flocking_core/types.hpp>
#include <swrm_flocking_core/alpha_agent.hpp>

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
