cmake_minimum_required(VERSION 3.16)

project(SwarmEKF LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(Boost REQUIRED)

add_executable(ekf ekf.cpp)

target_include_directories(ekf PRIVATE ${Boost_INCLUDE_DIRS})

if(MSVC)
    target_compile_options(ekf PRIVATE /W4 $<$<CONFIG:Release>:/O2>)
else()
    target_compile_options(ekf PRIVATE -Wall -Wextra -Wpedantic $<$<CONFIG:Release>:-O3>)
endif()
