#pragma once
#include "profiles/robot_a.h"
#include "profiles/robot_b.h"
#ifndef SWRM_ROBOT_ID
#define SWRM_ROBOT_ID 1
#endif
namespace robot {
#if SWRM_ROBOT_ID == 1
constexpr Settings selected = robotA;
#elif SWRM_ROBOT_ID == 2
constexpr Settings selected = robotB;
#else
#error "SWRM_ROBOT_ID must be 1 (A) or 2 (B)"
#endif
}
