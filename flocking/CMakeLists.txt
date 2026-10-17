cmake_minimum_required(VERSION 3.22.1)

include(CheckCXXCompilerFlag)

# -- PROJECT CONFIGURATION ----------------------------------------------------
project(
    SWRM_Flocking
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (APPLE)
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew" "/opt/homebrew/opt/qt")
endif()

# Rely on Boost to find itself instead of FindBoost
if (POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

# Forward timestamps for URL fetching
if (POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# -- ADDRESS SANITIZER CONFIGURATION ------------------------------------------
if (MSVC)
    set(ASAN_FLAG "/fsanitize=address")
else()
    set(ASAN_FLAG "-fsanitize=address")
endif()

# Check if address sanitizer is supported by the compiler
set(CMAKE_REQUIRED_LINK_OPTIONS ${ASAN_FLAG})
check_cxx_compiler_flag(${ASAN_FLAG} ASAN_SUPPORTED)
unset(CMAKE_REQUIRED_LINK_OPTIONS)

# -- PROJECT DEPENDENCIES -----------------------------------------------------
# TODO: Link to ROS2 libraries as needed
find_package(Boost REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
pkg_check_modules(GMP REQUIRED IMPORTED_TARGET gmp)
pkg_check_modules(MPFR REQUIRED IMPORTED_TARGET mpfr)

# -- LINK PROJECT DEPENDENCIES ------------------------------------------------
# All non-main files should be linked as an object library for tests
add_library(swrm_flocking_core OBJECT
    src/swrm_flocking_core/utilities.cpp
)

# Main executable 
add_executable(swrm_flocking src/main.cpp)

# Add headers and libraries to the object library
target_include_directories(swrm_flocking_core PUBLIC 
    ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(swrm_flocking_core PUBLIC
    Boost::boost
    PkgConfig::MPFR
    PkgConfig::GMP
    Eigen3::Eigen
)

# Link to main executable
target_link_libraries(swrm_flocking PRIVATE
    swrm_flocking_core
)

# Set ASAN for both the core library and the main executable
if (ASAN_SUPPORTED)
    target_compile_options(swrm_flocking_core PUBLIC ${ASAN_FLAG})
    target_link_options(swrm_flocking_core PUBLIC ${ASAN_FLAG})
endif()

# -- TESTING CONFIGURATION ----------------------------------------------------
include(CTest)

if (BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()

