include(CheckCXXCompilerFlag)

cmake_minimum_required(VERSION 4.2.3)

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
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew" "/usr/local" "/opt/homebrew/opt/qt")
endif()

# Rely on Boost to find itself instead of FindBoost
if (POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

# -- ADDRESS SANITIZER CONFIGURATION ------------------------------------------
if (MSVC)
    set(ASAN_FLAG "/fsanitize=address")
else()
    set(ASAN_FLAG "-fsanitize=address")
endif()

# Check if address sanitizer is supported by the compiler
check_cxx_compiler_flag(${ASAN_FLAG} ASAN_SUPPORTED)

# -- TESTING CONFIGURATION ----------------------------------------------------
include(CTest)

if (BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()

# -- PROJECT DEPENDENCIES -----------------------------------------------------
find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGL)
find_package(CGAL REQUIRED COMPONENTS Qt6)
find_package(Boost REQUIRED)
find_package(MPFR REQUIRED)

# -- LINK PROJECT DEPENDENCIES ------------------------------------------------
# All non-main files should be linked as an object library for tests
add_library(swrm_flocking_core OBJECT
    src/utilities.cpp
)

# Main executable 
add_executable(swrm_flocking src/main.cpp)

# Add headers and libraries to the object library
target_include_directories(swrm_flocking_core PUBLIC 
    ${PROJECT_SOURCE_DIR}include
)
target_link_libraries(swrm_flocking_core PUBLIC
    CGAL::CGAL
    CGAL::CGAL_Qt6
    Boost:boost
    ${MPFR_LIBRARIES}
)

# Link to main executable
target_link_libraries(swrm_flocking PRIVATE
    swrm_flocking_core
)

# CGAL compile-time definitions
target_compile_definitions(swrm_flocking_core PUBLIC
    CGAL_USE_BASIC_VIEWER
    CGAL_CHECK_EXPENSIVE
)

# Set ASAN for both the core library and the main executable
if (ASAN_SUPPORTED)
    target_compile_options(swrm_flocking_core PUBLIC ${ASAN_FLAG})
    target_link_options(swrm_flocking_core PUBLIC ${ASAN_FLAG})
    target_compile_options(swrm_flocking PRIVATE ${ASAN_FLAG})
    target_link_options(swrm_flocking PRIVATE ${ASAN_FLAG})
endif()
