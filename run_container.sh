#!/usr/bin/env bash
set -euo pipefail

# Helper to run the ROS2 Humble dev container with X11 support.
# Usage: ./run_container.sh

if [ -z "${DISPLAY:-}" ]; then
  echo "DISPLAY is not set. Are you running in a graphical session?"
  exit 1
fi

echo "Allowing local connections to X server (you can revoke with: xhost -local:root)"
xhost +local:root

docker run --rm -it \
  --network host \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$PWD":/workspace -w /workspace \
  --name ros2-humble-dev-instance \
  --user "$(id -u):$(id -g)" \
  ros2-humble-dev /bin/bash

# Note: to revoke X access after exiting the container, run:
#   xhost -local:root
