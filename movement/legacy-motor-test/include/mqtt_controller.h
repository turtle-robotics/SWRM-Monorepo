#pragma once
#include "robot_commands.h"

#ifdef SWRM_ENABLE_MQTT
void beginRobotNetwork(); // Creates one networking task; never initializes/arms
                          // motors.
void reportRobotCommandResult(robot::Command cmd, robot::Result result);
void reportRobotState(
    const robot::Status &status); // Thread-safe, no network I/O here.
#else
inline void beginRobotNetwork() {}
inline void reportRobotCommandResult(robot::Command, robot::Result) {}
inline void reportRobotState(const robot::Status &) {}
#endif
