# ROS2 Humble Dev Container

This dev container config provides a development environment for ROS 2 Humble using Ubuntu 22.04.

Files created:

- `.devcontainer/Dockerfile` — Dockerfile that installs ROS 2 Humble and common build tools.
- `.devcontainer/devcontainer.json` — VS Code devcontainer configuration.

Quick usage

1) Open this workspace in VS Code and choose "Reopen in Container" (Remote - Containers extension). VS Code will build the image using the Dockerfile and open the workspace inside the container.

2) Build the container image manually (optional):

```bash
# from inside the repo root (/home/ezis/ros2_ws)
docker build -t ros2-humble-dev -f .devcontainer/Dockerfile .

# run an interactive container (map current workspace)
docker run --rm -it \
  -v "$PWD":/workspace -w /workspace \
  --name ros2-humble-dev-instance \
  ros2-humble-dev /bin/bash
```

Gazebo

The container includes Gazebo (the physics engine) and ROS 2 Gazebo integration packages:
- `gazebo` — the Gazebo simulator
- `ros-humble-gazebo-ros-pkgs` — ROS 2 integration
- `ros-humble-gazebo-ros2-control` — control interface
- `ros-humble-gazebo-dev` — development headers

To run Gazebo with X11 support, use:

```bash
./run_container.sh
gazebo  # from inside the container
```

Or with Docker Compose:

```bash
docker compose up --build
docker compose run --rm ros2-humble-dev gazebo
```

Notes and tips

- The container creates a `vscode` user (UID/GID 1000 by default). If your host user has a different UID/GID, set `USER_UID`/`USER_GID` in `devcontainer.json` build args.
- The container sources `/opt/ros/humble/setup.bash` automatically in interactive shells.
- The `postCreateCommand` runs `rosdep update` in the container; if it fails, re-run `rosdep update` manually:

```bash
sudo rosdep update
```

Optional: GUI apps

For GUI tools (rqt, RViz) you can use X11 forwarding. This repository now includes a `docker-compose.yml` and a `run_container.sh` helper to simplify running the container with GUI support.

Run the helper (recommended):

```bash
# allow X connections from local Docker containers (you can revoke with 'xhost -local:root')
./run_container.sh
```

Or with `docker-compose` (passes through your DISPLAY and mounts the X socket):

```bash
docker compose up --build
docker compose run --rm ros2-humble-dev /bin/bash
```

Security note: `run_container.sh` uses `xhost +local:root` to allow the container to connect to your X server. This is commonly used for local development but reduces X server access control — revoke it with `xhost -local:root` after use.

