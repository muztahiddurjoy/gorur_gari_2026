FROM osrf/ros:humble-desktop-full

# Avoid interactive prompts during apt install
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies, Ignition Fortress, and ROS 2 utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    git \
    nano \
    v4l-utils \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-serial \
    python3-yaml \
    python3-opencv \
    ros-humble-cv-bridge \
    ros-humble-tf2-ros \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-usb-cam \
    ros-humble-rplidar-ros \
    ros-humble-joy \
    ros-humble-ros-gz \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ignition-fortress \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages required by controls and autonomy
RUN pip3 install --no-cache-dir \
    pymavlink

# Create non-root user matching host UID/GID (1000:1000)
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Automatically source ROS 2 Humble in user shell
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "export IGN_IP=127.0.0.1" >> /home/$USERNAME/.bashrc

USER $USERNAME
WORKDIR /workspace
