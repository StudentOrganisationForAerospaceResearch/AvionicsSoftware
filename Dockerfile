FROM debian:latest

# Environment Setup

ARG BRANCH_OR_TAG=main
ARG CMAKE_OPTIONS=
ENV DEBIAN_FRONTEND=noninteractive
RUN env \
  && apt-get update \
  && apt-get install -q -y git cmake make g++ lcov gettext-base jq curl \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

# Compilation

ENV BUILD_DIR=build

ADD . /repo
WORKDIR /repo
RUN cmake -S . -B $BUILD_DIR
RUN cmake --build $BUILD_DIR

# Run

RUN cd $BUILD_DIR && ctest