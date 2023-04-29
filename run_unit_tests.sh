#! /bin/sh

LOG_DIR=./logs
LOG_FILE=log_$(date +%F_%H:%M:%S).txt

if [ ! -d './logs' ]; then
  mkdir ./logs
fi

DOCKER_BUILDKIT=1 docker build -t gtest .
docker run --rm --mount type=bind,source="$(pwd)",target=/repo -t gtest \
  bash -c "cd /repo && \
           cmake -S . -B build && \
           cmake --build build && \
           unbuffer build/tests | tee $LOG_DIR/$LOG_FILE"

sed -i -e 's/\x1B[^m]*m//g' "$LOG_DIR/$LOG_FILE"
cp "$LOG_DIR/$LOG_FILE" "$LOG_DIR/log_latest.txt"
