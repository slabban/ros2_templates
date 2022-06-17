#!/bin/bash

docker run \
    -it --rm \
    --name="ros2_torch_container" \
    --volume="$PWD/../ros2_ws:/home/ros2_ws:rw" \
    --runtime=nvidia \
    ros2_torch:latest
