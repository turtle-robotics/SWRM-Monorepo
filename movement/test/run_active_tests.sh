#!/bin/sh
set -eu
movement_dir="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
test_dir="$(mktemp -d "${TMPDIR:-/tmp}/swrm-active-tests.XXXXXX")"
trap 'rm -rf "$test_dir"' EXIT HUP INT TERM
for suite in legacy square trim monorepo orientation reliability speed_range square_repeat robot_command; do
    "${CXX:-clang++}" -std=c++17 -Wall -Wextra -Werror -I"$movement_dir/test/host" \
        "$movement_dir/test/host/${suite}_tests.cpp" -o "$test_dir/$suite"
    "$test_dir/$suite"
done
"${CXX:-clang++}" -std=c++17 -Wall -Wextra -Werror -DSWRM_ROBOT_ID=2 -I"$movement_dir/test/host" \
    "$movement_dir/test/host/robot_command_tests.cpp" -o "$test_dir/robot_command_b"
"$test_dir/robot_command_b"

# Keep upstream v2.8 unchanged: suppress its two existing signedness warnings
# only for that library object. Our firmware/test code retains warnings as errors.
"${CXX:-clang++}" -std=c++17 -Wall -Wextra -Werror -Wno-sign-compare \
    -I"$movement_dir/test/mqtt_host" -I"$movement_dir/legacy-motor-test/lib/PubSubClient/src" \
    -c "$movement_dir/legacy-motor-test/lib/PubSubClient/src/PubSubClient.cpp" -o "$test_dir/pubsubclient.o"
for robot_id in 1 2; do
    "${CXX:-clang++}" -std=c++17 -Wall -Wextra -Werror -DSWRM_ROBOT_ID="$robot_id" \
        -I"$movement_dir/test/host" "$movement_dir/test/host/remote_recovery_tests.cpp" -o "$test_dir/recovery_$robot_id"
    "$test_dir/recovery_$robot_id"
    "${CXX:-clang++}" -std=c++17 -Wall -Wextra -Werror -DSWRM_ROBOT_ID="$robot_id" \
        -I"$movement_dir/test/host" "$movement_dir/test/host/battery_boot_tests.cpp" -o "$test_dir/battery_$robot_id"
    "$test_dir/battery_$robot_id"
    "${CXX:-clang++}" -std=c++17 -Wall -Wextra -Werror -DSWRM_ROBOT_ID="$robot_id" \
        -I"$movement_dir/test/mqtt_host" -I"$movement_dir/test/host" \
        -I"$movement_dir/legacy-motor-test/lib/PubSubClient/src" \
        "$movement_dir/test/host/mqtt_controller_tests.cpp" "$test_dir/pubsubclient.o" -o "$test_dir/mqtt_$robot_id"
    "$test_dir/mqtt_$robot_id"
done
