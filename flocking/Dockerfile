FROM ros:jazzy

ARG BUILD_TESTING=false
ARG IGNORE_MAIN=false
ARG USE_ASAN=false

ADD . /app
WORKDIR /app

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    libgmp-dev libmpfr-dev libboost-all-dev libeigen3-dev \
    ros-jazzy-turtlesim && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN cmake -S . -B build -DBUILD_TESTING=${BUILD_TESTING} -DUSE_ASAN=${USE_ASAN} && \
    cmake --build build

RUN chmod +x /app/scripts/dockerfile_entrypoint.sh

ENV BUILD_TESTING_ARG=${BUILD_TESTING}
ENV IGNORE_MAIN_ARG=${IGNORE_MAIN}
ENTRYPOINT ["/app/scripts/dockerfile_entrypoint.sh"]
