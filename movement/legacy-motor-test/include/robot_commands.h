#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

// No Arduino, MQTT, Wi-Fi, allocation, or null-terminated payload dependency.
namespace robot {
enum class Command { none, square, stop, spin, init, status };
enum class Result {
  started,
  stopped,
  queued,
  unknown,
  notReady,
  busy,
  invalidSettings,
  fault,
  ready,
  reported
};
inline Command parseCommand(const char *bytes, std::size_t length) {
  if (!bytes)
    return Command::none;
  if (length == 6 && !std::memcmp(bytes, "square", 6))
    return Command::square;
  if (length == 4 && !std::memcmp(bytes, "stop", 4))
    return Command::stop;
  if (length == 4 && !std::memcmp(bytes, "spin", 4))
    return Command::spin;
  if (length == 4 && !std::memcmp(bytes, "init", 4))
    return Command::init;
  if (length == 6 && !std::memcmp(bytes, "status", 6))
    return Command::status;
  return Command::none; // Exact lowercase words, no whitespace/NUL suffix.
}
const char *resultName(Result result);
inline const char *commandName(Command cmd) {
  switch (cmd) {
  case Command::square:
    return "square";
  case Command::stop:
    return "stop";
  case Command::spin:
    return "spin";
  case Command::init:
    return "init";
  case Command::status:
    return "status";
  default:
    return "";
  }
}
struct Status {
  bool ready = false, configured = false, armed = false, running = false,
       fault = false;
  bool square = false, paused = false, spin = false;
  unsigned retries = 0;
  uint8_t error = 0, reg = 0, firstError = 0, firstReg = 0;
  unsigned faultLeg = 0;
  uint32_t cycles = 0, sequence = 0;
};
} // namespace robot

// Call direct handling/initialization/update ONLY from the Arduino loop task.
// Init requires idle/disarmed, no pending stop retries and an acknowledged
// preflight zero. Successful recovery never arms or resumes a previous run.
bool initializeRobotMovement();
robot::Status getRobotStatus(); // Loop task only; no hardware reads/writes.
robot::Result handleRobotCommand(const char *bytes, std::size_t length);
inline robot::Result handleRobotCommand(const char *word) {
  return handleRobotCommand(word, word ? std::strlen(word) : 0);
}
void stopRobotMovement();
void updateRobotMovement(); // Drains inbox BEFORE advancing motion deadlines.

// Callback-safe fixed inbox: one pending start; stop replaces it and wins until
// serviced. No motor I/O/logging here. Callback must reject retained starts and
// handle payload completeness according to its actual MQTT library contract.
robot::Result queueRobotCommand(const char *bytes, std::size_t length);
void onRobotConnectionLost(); // Queue zero/disarm; reconnect never queues a
                              // start.
