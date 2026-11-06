# Swarm EKF

This C++17 simulation uses Boost.Multiprecision's 100-decimal-digit type
throughout the EKF, simulation math, and reported statistics. Boost is
header-only for this project.

## Requirements

- CMake 3.16 or newer
- A C++17 compiler
- Boost headers

## Build and run

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
./build/ekf
```

For a fast chart preview, run `./build/ekf --quick` (5 bots, 1 run of 20
simulated seconds, with a 3-second settling period). The no-argument run keeps
the full configuration.

On Windows, run `build\Release\ekf.exe` after building with a multi-configuration
generator, or `build\ekf.exe` with a single-configuration generator.

Adjust simulation settings in `Config` in `ekf.cpp`.

Each run also writes `rmse_chart.svg` in the working directory. The SVG charts
the pooled X, Y, and position RMSE over time after the settling period; open the
file in a browser to view it.

The EKF and simulation math stay in 100-digit multiprecision. Standard-library
random distributions generate sensor-noise samples as native doubles, which are
immediately converted to multiprecision; this avoids doing expensive transcendental
math inside the random-number generator. True-bot motion also reuses its shared
per-step heading trigonometry.
