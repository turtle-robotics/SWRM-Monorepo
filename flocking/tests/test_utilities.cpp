#include <gtest/gtest.h>

#include <sstream>
#include <string>

#include <swrm_flocking_core/utilities.hpp>

TEST(TestUtilities, TestingTest) {
    testing::internal::CaptureStdout();
    print_hello();
    std::string output = testing::internal::GetCapturedStdout();
    std::ostringstream oss;
    oss << "Hello, World!" << std::endl;
    std::string expected_output = oss.str();

    EXPECT_EQ(output, expected_output);
}
