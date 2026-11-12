#include <gtest/gtest.h>

#include <swrm_flocking_core/types.hpp>
#include <swrm_flocking_core/gamma_agent.hpp>

class TestGammaAgent : public ::testing::Test {
protected:
    swrm::flocking::vector2d initial_position;
    swrm::flocking::vector2d initial_velocity;
    swrm::flocking::hpscalar time_step;

    TestGammaAgent() {
        initial_position = swrm::flocking::vector2d(0.0, 0.0);
        initial_velocity = swrm::flocking::vector2d(0.0, 0.0);
        time_step = 1;
    }
};

TEST_F(TestGammaAgent, ConstructorAndGetters) {
    swrm::flocking::gamma_agent agent(initial_position, initial_velocity, [](swrm::flocking::hpscalar t) {
            return swrm::flocking::vector2d(0.0, 0.0);
    }, time_step);

    EXPECT_EQ(agent.position(), initial_position);
    EXPECT_EQ(agent.velocity(), initial_velocity);
    EXPECT_EQ(agent.time(), 0.0);
    EXPECT_EQ(agent.x(), initial_position(0));
    EXPECT_EQ(agent.y(), initial_position(1));
}

TEST_F(TestGammaAgent, TimeStepUpdate) {
    swrm::flocking::gamma_agent agent(initial_position, initial_velocity, [](swrm::flocking::hpscalar t) {
            return swrm::flocking::vector2d(1.0, 0.0);
    }, time_step);

    agent.update();

    EXPECT_EQ(agent.time(), time_step);
    EXPECT_EQ(agent.position(), swrm::flocking::vector2d(1.0, 0.0));
    EXPECT_EQ(agent.velocity(), swrm::flocking::vector2d(1.0, 0.0));
    EXPECT_EQ(agent.x(), 1.0);
    EXPECT_EQ(agent.y(), 0.0);
}
