#include <gtest/gtest.h>

#include <iterator>
#include <sstream>
#include <vector>

#include <swrm_flocking_core/types.hpp>
#include <swrm_flocking_core/agent_manager.hpp>

namespace {
    const swrm::flocking::vector2d GAMMA_POSITION(0.0, 0.0);
    const swrm::flocking::vector2d GAMMA_VELOCITY(0.0, 0.0);
    constexpr double MANAGER_TIME_STEP = 0.01;
    constexpr double GAMMA_TIME_STEP = 0.001;

    swrm::flocking::vector_function zero_velocity_profile() {
        return [](swrm::flocking::hpscalar) {
            return swrm::flocking::vector2d(0.0, 0.0);
        };
    }
}

TEST(TestAgentManager, TestReadDataAndIteration) {
    std::istringstream input(
        "0 1.0 2.0 1.0 2.0\n"
        "1 3.0 4.0 3.0 4.0\n"
        "STOP\n"
    );
    swrm::flocking::agent_manager<std::vector> manager(
        input,
        2,
        MANAGER_TIME_STEP,
        GAMMA_POSITION,
        GAMMA_VELOCITY,
        zero_velocity_profile(),
        GAMMA_TIME_STEP
    );

    EXPECT_EQ(std::distance(manager.begin(), manager.end()), 2);
    EXPECT_EQ(manager.begin()->position(), swrm::flocking::vector2d(0.0, 0.0));

    manager.read_data(input);

    auto agent = manager.begin();
    EXPECT_EQ(agent->position(), swrm::flocking::vector2d(1.0, 2.0))
        << "First agent position was not read from the input stream.";
    EXPECT_EQ(agent->velocity(), swrm::flocking::vector2d(1.0, 2.0))
        << "First agent velocity was not read from the input stream.";

    ++agent;
    EXPECT_EQ(agent->position(), swrm::flocking::vector2d(3.0, 4.0))
        << "Second agent position was not read from the input stream.";
    EXPECT_EQ(agent->velocity(), swrm::flocking::vector2d(3.0, 4.0))
        << "Second agent velocity was not read from the input stream.";
    EXPECT_EQ(agent->acceleration(), swrm::flocking::vector2d(0.0, 0.0))
        << "STOP should clear the acceleration commands.";
}

TEST(TestAgentManager, TestInvalidIdsIgnored) {
    std::istringstream input(
        "-2 8.0 8.0 8.0 8.0\n"
        "99 9.0 9.0 9.0 9.0\n"
        "0 1.0 2.0 1.0 2.0\n"
        "1 3.0 4.0 3.0 4.0\n"
        "STOP\n"
    );
    swrm::flocking::agent_manager<std::vector> manager(
        input,
        2,
        MANAGER_TIME_STEP,
        GAMMA_POSITION,
        GAMMA_VELOCITY,
        zero_velocity_profile(),
        GAMMA_TIME_STEP
    );

    manager.read_data(input);

    auto agent = manager.begin();
    EXPECT_EQ(agent->position(), swrm::flocking::vector2d(1.0, 2.0))
        << "An invalid ID changed the first agent state.";
    ++agent;
    EXPECT_EQ(agent->position(), swrm::flocking::vector2d(3.0, 4.0))
        << "An invalid ID changed the second agent state.";
}

TEST(TestAgentManager, TestStopBeforeAllStates) {
    std::istringstream input(
        "0 1.0 2.0 0.0 0.0\n"
        "STOP\n"
    );
    swrm::flocking::agent_manager<std::vector> manager(
        input,
        2,
        MANAGER_TIME_STEP,
        GAMMA_POSITION,
        GAMMA_VELOCITY,
        zero_velocity_profile(),
        GAMMA_TIME_STEP
    );

    manager.read_data(input);

    auto agent = manager.begin();
    EXPECT_EQ(agent->position(), swrm::flocking::vector2d(1.0, 2.0))
        << "The first state should be stored before STOP is read.";
    ++agent;
    EXPECT_EQ(agent->position(), swrm::flocking::vector2d(0.0, 0.0))
        << "The second agent should keep its initial state.";
}

TEST(TestAgentManager, TestEmptyAgentContainer) {
    std::istringstream input("STOP\n");
    swrm::flocking::agent_manager<std::vector> manager(
        input,
        0,
        MANAGER_TIME_STEP,
        GAMMA_POSITION,
        GAMMA_VELOCITY,
        zero_velocity_profile(),
        GAMMA_TIME_STEP
    );

    manager.read_data(input);

    EXPECT_EQ(manager.begin(), manager.end())
        << "An empty agent container should produce an empty range.";
}

TEST(TestAgentManager, TestDestructorStopsGammaThread) {
    std::istringstream input;
    {
        swrm::flocking::agent_manager<std::vector> manager(
            input,
            1,
            0.0,
            GAMMA_POSITION,
            GAMMA_VELOCITY,
            zero_velocity_profile(),
            GAMMA_TIME_STEP
        );
    }

    SUCCEED() << "Gamma thread stopped when the manager was destroyed.";
}
