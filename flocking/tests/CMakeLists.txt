include(FetchContent)
include(GoogleTest)

# -- INCLUDE GTEST ------------------------------------------------------------
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.18.0.zip
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# -- TEST EXECUTABLE ----------------------------------------------------------
# Create test executable
add_executable(swrm_flocking_tests
    test_directional_pointer.cpp
)

# Link library to test and GTest
target_link_libraries(swrm_flocking_tests PRIVATE
    swrm_flocking_core
    GTest::gtest_main
)

# Add ASAN flags to the test executable if supported
if (ASAN_SUPPORTED)
    target_compile_options(swrm_flocking_tests PRIVATE ${ASAN_FLAG})
    target_link_options(swrm_flocking_tests PRIVATE ${ASAN_FLAG})
endif()

# Add TSAN flags to the test executable if supported
if (TSAN_SUPPORTED)
    target_compile_options(swrm_flocking_tests PRIVATE ${TSAN_FLAG})
    target_link_options(swrm_flocking_tests PRIVATE ${TSAN_FLAG})
endif()

# Discover and register tests with CTest
gtest_discover_tests(swrm_flocking_tests)
