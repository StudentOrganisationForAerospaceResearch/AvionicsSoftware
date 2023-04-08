#! /bin/sh

docker build -t gtest .
# docker run --rm -it --mount type=bind,source="$(pwd)",target=/repo --entrypoint bash gtest
docker run --rm -it --entrypoint bash gtest
