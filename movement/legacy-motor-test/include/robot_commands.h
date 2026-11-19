#pragma once
#include <cstddef>
#include <cstring>

// No Arduino, MQTT, Wi-Fi, allocation, or null-terminated payload dependency.
namespace robot {
enum class Command { none, square, stop, spin };
enum class Result { started, stopped, queued, unknown, notReady, busy, invalidSettings, fault };
inline Command parseCommand(const char* bytes, std::size_t length) {
    if (!bytes) return Command::none;
    if (length==6 && !std::memcmp(bytes,"square",6)) return Command::square;
    if (length==4 && !std::memcmp(bytes,"stop",4)) return Command::stop;
    if (length==4 && !std::memcmp(bytes,"spin",4)) return Command::spin;
    return Command::none; // Exact lowercase words, no whitespace/NUL suffix.
}
const char* resultName(Result result);
}

// Call direct handling/initialization/update ONLY from the Arduino loop task.
// Explicit init can recover a fault, but always leaves the robot disarmed.
bool initializeRobotMovement();
robot::Result handleRobotCommand(const char* bytes, std::size_t length);
inline robot::Result handleRobotCommand(const char* word) {
    return handleRobotCommand(word,word ? std::strlen(word) : 0);
}
void stopRobotMovement();
void updateRobotMovement(); // Drains inbox BEFORE advancing motion deadlines.

// Callback-safe fixed inbox: one pending start; stop replaces it and wins until
// serviced. No motor I/O/logging here. Callback must reject retained starts and
// handle payload completeness according to its actual MQTT library contract.
robot::Result queueRobotCommand(const char* bytes, std::size_t length);
void onRobotConnectionLost(); // Queue zero/disarm; reconnect never queues a start.
