#! /bin/sh

#DOCKER_BUILDKIT=1 BUILDKIT_PROGRESS=plain docker build -t gtest . #--build-arg CACHEBUST=$(date +%s) # --no-cache
# DOCKER_BUILDKIT=1 docker build -t gtest . #--build-arg CACHEBUST=$(date +%s) # --no-cache
# docker run --rm -it --mount type=bind,source="$(pwd)",target=/repo --entrypoint bash gtest
# docker run --rm -it --entrypoint bash gtest

LOG_FILE=logfile.txt

DOCKER_BUILDKIT=1 docker build -t gtest .
docker run --rm --mount type=bind,source="$(pwd)",target=/repo -t gtest \
  bash -c "cd /repo && cmake -S . -B build && cmake --build build && unbuffer build/tests | tee $LOG_FILE"

sed -i -e "s/\x1B[^m]*m//g" $LOG_FILE
