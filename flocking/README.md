# SWRM Flocking Code
This is the code for the flocking algorithm used to coordinate the SWRM robots.

## Dependencies
The only dependency is Docker. You can install Docker by following the
instructions on the [Docker website](https://docs.docker.com/get-docker/).

Prior to running the code, make sure you have Docker installed and running on 
your machine.

## Running the Code
To run the code, follow these steps:
1. Clone the repository to your local machine.
2. Open a terminal and navigate to the `flocking` directory.
3. Build the Docker image by running the following command:
   ```
   docker build -t swrm-flocking .
4. Run the Docker container by executing the following command:
   ```
   docker run --rm swrm-flocking
   ```

### Build Arguments
There are two build arguments that can be passed to the Docker build command:
- `BUILD_TESTING`: `true` if the test code should be built, `false` otherwise. 
Default is `false`.
- `IGNORE_MAIN`: `true` if the main code shouldn't be executed, `false` otherwise. 
Default is `false`.

These can be passed to the Docker build command as follows:
```
docker build --build-arg BUILD_TESTING=true --build-arg IGNORE_MAIN=false -t swrm-flocking .
```
