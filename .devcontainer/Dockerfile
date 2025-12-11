FROM ubuntu:22.04

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

# Basic packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    dialog \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    python3-pip \
    python3-argcomplete \
    git \
    wget \
    cmake \
    ninja-build \
 && rm -rf /var/lib/apt/lists/*

# Locale
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Add ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
 && echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble desktop (use retries to handle transient mirror errors)
RUN apt-get update \
 && apt-get -o Acquire::Retries=3 install -y --no-install-recommends ros-humble-desktop \
 || apt-get -o Acquire::Retries=3 install -y --no-install-recommends --fix-missing ros-humble-desktop \
 && rm -rf /var/lib/apt/lists/*

# Create non-root user to match VS Code Remote-Containers conventions
RUN groupadd --gid $USER_GID $USERNAME || true \
 && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
 && mkdir -p /etc/sudoers.d \
 && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
 && chmod 0440 /etc/sudoers.d/$USERNAME

# Source ROS setup for all new shells
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/skel/.bashrc

# Install and initialize rosdep (init may be skipped if already initialized)
RUN apt-get update && apt-get install -y --no-install-recommends python3-rosdep \
 && rm -rf /var/lib/apt/lists/* \
 && rosdep init || true

# Install Gazebo and ROS 2 Gazebo integration packages (with retries for mirror issues)
RUN apt-get update && \
    apt-get -o Acquire::Retries=3 -o Acquire::http::Timeout=120 install -y --no-install-recommends \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-dev \
 && rm -rf /var/lib/apt/lists/*

# Upgrade pip and install Python tooling used for ROS builds (install as root)
RUN pip3 install -U pip && pip3 install -U vcstool colcon-common-extensions

# Create non-root user to match VS Code Remote-Containers conventions
USER $USERNAME
WORKDIR /home/$USERNAME

# Ensure user's shell sources ROS 2
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/$USERNAME/.bashrc

ENV ROS_DISTRO=humble

CMD ["sleep", "infinity"]
