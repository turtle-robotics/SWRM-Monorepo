#pragma once
#include "shared_square.h"
namespace robot {
// B uses A's settings as the starting profile. B has NOT been physically validated.
// Replace sharedSquare here with a B-specific SquareProfile for later overrides.
constexpr Settings robotB = {"B", "robot2", "robots/robot2/cmd", "robots/robot2/status", true, sharedSquare, 10};
}
