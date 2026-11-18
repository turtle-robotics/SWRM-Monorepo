# SWRM Flocking Code
This is the code for the flocking algorithm used to coordinate the SWRM robots.

## Dependencies
### Through Docker
The only dependency is Docker. You can install Docker by following the
instructions on the [Docker website](https://docs.docker.com/get-docker/).

Prior to running the code, make sure you have Docker installed and running on 
your machine.

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

## Running the Code
### Through Docker
To run the code through Docker, follow these steps:
1. Clone the repository to your local machine.
2. Open a terminal and navigate to the `flocking` directory.
3. Build the Docker image by running the following command:
   ```
   docker build -t swrm-flocking .
   ```
4. Run the Docker container by executing the following command:
   ```
   docker run --rm swrm-flocking
   ```

#### Build Arguments
There are two build arguments that can be passed to the Docker build command:
- `BUILD_TESTING`: `true` if the test code should be built, `false` otherwise. 
Default is `false`.
- `IGNORE_MAIN`: `true` if the main code shouldn't be executed, `false` 
otherwise. Default is `false`.
- `USE_ASAN`: `true` if the code should be built with AddressSanitizer, `false` 
if it should be built with ThreadSanitizer (both of these are mutually 
exclusive) Default is `false`.

> When testing the code, it's recommended to test with `USE_ASAN` as both 
`true` and `false` in separate testing runs to ensure that the code is free of 
both memory and thread errors.

These can be passed to the Docker build command as follows:
```
docker build --build-arg BUILD_TESTING=true --build-arg IGNORE_MAIN=false -t swrm-flocking .
```

### Locally
To run the code locally, follow these steps:
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
6. Run the executable by executing the following command:
   ```
   ./build/swrm_flocking
   ```

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

## Windows (WSL)
Use WSL 2 to build and run the project in a Linux environment on Windows.

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
4. Configure, build, and run:
   ```bash
   cmake -S . -B build
   cmake --build build
   ./build/swrm_flocking
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
`src/` using the `Doxyfile` in the `flocking` directory.

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
