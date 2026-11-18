# SWRM Flocking Library
The flocking algorithm used to coordinate the SWRM robots, packaged as the
`swrm_flocking_core` C++ library.

This is a library, not a complete standalone application. It does not run a
simulation, integrate agent motion, or communicate with robot hardware. A host
application supplies agent states to `swrm::flocking::agent_manager` and is
responsible for applying the acceleration commands the library computes. See
[Using the Library](#using-the-library) for how to link it into your own
program; the Doxygen main page (`mainpage.md`) describes the algorithm and the
full integration contract.

## Dependencies
### Through Docker
The only dependency is Docker. You can install Docker by following the
instructions on the [Docker website](https://docs.docker.com/get-docker/).

Prior to building the library, make sure you have Docker installed and running
on your machine.

### Locally
This project depends on two libraries: `Eigen` and `boost`. Building locally
also needs CMake 3.22.1 or newer and a compiler that supports C++23.

On Debian, Ubuntu, or WSL, install both libraries with:
```
sudo apt-get install libeigen3-dev libboost-all-dev
```

On macOS, install them with Homebrew:
```
brew install eigen boost
```

Windows is supported through WSL 2. Complete the setup in
[Windows (WSL)](#windows-wsl), then follow the Debian/Ubuntu instructions above
inside the WSL environment.

Doxygen is only needed for the optional API documentation; see
[Building Documentation](#building-documentation).

## Building the Library
### Through Docker
The Docker image builds the library; the project does not provide a standalone
application to run.
1. Clone the repository to your local machine.
2. Open a terminal and navigate to the `flocking` directory.
3. Build the Docker image by running the following command:
   ```
   docker build -t swrm-flocking .
   ```
4. To compile the tests into the image as well, pass `BUILD_TESTING`:
   ```
   docker build --build-arg BUILD_TESTING=true -t swrm-flocking .
   ```
   Run them with:
   ```
   docker run --rm --entrypoint /app/build/tests/swrm_flocking_tests swrm-flocking
   ```

#### Build Arguments
There are three build arguments that can be passed to the Docker build command:
- `BUILD_TESTING`: `true` if the test code should be built, `false` otherwise. 
Default is `false`.
- `USE_ASAN`: `true` to build with AddressSanitizer. Default is `false`.
- `USE_TSAN`: `true` to build with ThreadSanitizer. Default is `false`.

The sanitizer options are mutually exclusive.

> When testing the code, run the suite once with `USE_ASAN` and once with
> `USE_TSAN` to ensure that the code is free of both memory and thread errors.

These can be passed to the Docker build command as follows:
```
docker build --build-arg BUILD_TESTING=true --build-arg USE_ASAN=true -t swrm-flocking .
```

### Locally
To build the library locally, follow these steps:
1. Install the dependencies as described in the [Dependencies](#dependencies) section.
2. Clone the repository to your local machine.
3. Open a terminal and navigate to the `flocking` directory.
4. Configure the build by running the following command:
   ```
   cmake -S . -B build
   ```
5. Build the project by running the following command:
   ```
   cmake --build build
   ```

The build produces the `swrm_flocking_core` object library. No application
executable is created.

CMake writes everything into the `build` directory and uses the `Release`
build type unless you pass `-DCMAKE_BUILD_TYPE`.

#### Tests
CTest is enabled by default, so the build above also compiles the tests. The
first configure downloads GoogleTest, which needs an internet connection. Run
the tests with:
```
ctest --test-dir build --output-on-failure
```
To leave the tests out, configure with `-DBUILD_TESTING=FALSE`.

#### Sanitizers
To build with AddressSanitizer or ThreadSanitizer, pass `USE_ASAN` or `USE_TSAN`
when configuring:
```
cmake -S . -B build -DUSE_ASAN=TRUE
cmake --build build
```
The two options are mutually exclusive; configuring with both set to `TRUE`
stops with an error.

## Using the Library
Add the library to your application's build and link `swrm_flocking_core`. The
target carries the public include directory and its Eigen and Boost
dependencies:
```cmake
add_subdirectory(path/to/flocking)
target_link_libraries(my_application PRIVATE swrm_flocking_core)
```

Your application owns the simulation or robot driver. It feeds agent states to
`swrm::flocking::agent_manager`, and the library computes and stores each
alpha agent's acceleration command. The library never applies those commands,
integrates motion, or writes output itself. The Doxygen main page includes a
complete usage example and the input stream format.

## Windows (WSL)
Use WSL 2 to build the project in a Linux environment on Windows.

1. Install WSL 2 and Ubuntu from an elevated PowerShell:
   ```powershell
   wsl --install -d Ubuntu
   ```
   Restart Windows if prompted. If WSL is already installed, run `wsl --update`
   and `wsl --set-default-version 2` before continuing. Then launch Ubuntu and
   create a Linux username and password.
2. Install the compiler, CMake, and libraries inside Ubuntu. Use Ubuntu 22.04
   or newer with a compiler that supports C++23:
   ```bash
   sudo apt update
   sudo apt install build-essential cmake libeigen3-dev libboost-all-dev
   ```
3. Clone the repository into the WSL home directory and enter the `flocking`
   directory:
   ```bash
   git clone <repository-url> ~/SWRM-Monorepo
   cd ~/SWRM-Monorepo/flocking
   ```
   Keeping the checkout under the Linux filesystem, such as `~/`, is much faster
   than building from `/mnt/c`.
4. Configure and build:
   ```bash
   cmake -S . -B build
   cmake --build build
   ```
5. Run the tests:
   ```bash
   ctest --test-dir build --output-on-failure
   ```
   The first configure downloads GoogleTest, so the WSL environment needs an
   internet connection.

These commands work in Windows Terminal's Ubuntu profile or in VS Code
connected to WSL. For API documentation, install Doxygen as described in
[Building Documentation](#building-documentation).

## Building Documentation
The API documentation is generated from the Doxygen comments in `include/` and
`src/`, plus the main-page introduction in `mainpage.md`, using the `Doxyfile`
in the `flocking` directory.

Install Doxygen:
- Debian, Ubuntu, or WSL: `sudo apt-get install doxygen`
- macOS: `brew install doxygen`

From the `flocking` directory, generate the HTML documentation:
```
doxygen Doxyfile
```
The entry point is `docs/html/index.html`. Open it with:
- macOS: `open docs/html/index.html`
- Linux: `xdg-open docs/html/index.html`
- WSL: `explorer.exe docs/html/index.html`

The `docs/` directory is generated output and is ignored by Git. Graph
generation is disabled by default in `Doxyfile` (`HAVE_DOT = NO`), so Graphviz
is not required. To enable class and include diagrams, install Graphviz
(`sudo apt-get install graphviz` on Debian/Ubuntu/WSL or
`brew install graphviz` on macOS) and set `HAVE_DOT = YES` in `Doxyfile`.
