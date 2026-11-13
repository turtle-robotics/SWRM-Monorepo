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

On Debian or Ubuntu, install both libraries with:
```
sudo apt-get install libeigen3-dev libboost-all-dev
```

On macOS, install them with Homebrew:
```
brew install eigen boost
```

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
