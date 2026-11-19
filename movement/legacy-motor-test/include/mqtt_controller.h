#pragma once
#include "robot_commands.h"

#ifdef SWRM_ENABLE_MQTT
void beginRobotNetwork(); // Creates one networking task; never initializes/arms motors.
void reportRobotCommandResult(robot::Command cmd, robot::Result result);
#else
inline void beginRobotNetwork() {}
inline void reportRobotCommandResult(robot::Command, robot::Result) {}
#endif
