#!/bin/bash

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

cd ${SCRIPT_DIR}/../jetson-inference/docker && \
    ./run.sh \
    --ros=humble \
    -v ${SCRIPT_DIR}:/app/src