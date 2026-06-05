#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Allow local X11 connections for Docker GUI
xhost +local:docker

# Generate XAUTH file
XAUTH=/tmp/.docker.xauth
if [ ! -f "$XAUTH" ]; then
    touch "$XAUTH"
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ -n "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f "$XAUTH" nmerge -
    fi
    chmod a+r "$XAUTH"
fi

# Run Docker container
docker run -it --rm \
  --privileged \
  --runtime=nvidia \
  --net=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="XAUTHORITY=$XAUTH" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$XAUTH:$XAUTH" \
  --volume="$REPO_ROOT:/external:rw" \
  dlio-go2w:latest bash
