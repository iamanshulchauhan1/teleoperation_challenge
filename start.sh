#!/bin/bash
xhost +
docker run --rm -it \
    --net=host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --device="/dev/video0:/dev/video0" \
    -v "$(realpath .):/workspaces/challenge_ws" \
    -w /workspaces/challenge_ws \
    telechallenge_image:latest \
    bash -c "/workspaces/challenge_ws/start_teleop.sh"
