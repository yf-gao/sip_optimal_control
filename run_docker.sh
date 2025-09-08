#!/usr/bin/env bash

xhost +local:root
docker run \
    --rm \
    -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix  -it sipoc:main
