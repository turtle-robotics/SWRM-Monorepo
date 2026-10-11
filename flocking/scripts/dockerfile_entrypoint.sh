#!/bin/sh
set -e

if [ "$BUILD_TESTING_ARG" = "true" ]; then
    ./build/tests/swrm_flocking_tests
fi

if [ "$IGNORE_MAIN_ARG" = "false" ]; then
    ./build/swrm_flocking
fi
