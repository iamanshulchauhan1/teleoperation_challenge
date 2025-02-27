FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

# Install essential packages and dependencies
RUN apt-get update && apt-get install -y \
    git build-essential cmake wget \
    python3-pip python3-vcstool python3-rosdep \
    python3-colcon-common-extensions python3-colcon-mixin \
    libeigen3-dev libassimp-dev libtinyxml2-dev libbullet-dev \
    libopencv-dev x11-apps \
    sudo && \
    rm -rf /var/lib/apt/lists/*

# Install rosdep and initialize it
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
    sudo rosdep init; \
    fi && \
    rosdep update && \
    sudo apt update && \
    sudo apt dist-upgrade -y

# Install colcon mixins
RUN if ! colcon mixin list | grep -q "default"; then \
    colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml; \
    fi && \
    colcon mixin update default

WORKDIR /root
SHELL ["/bin/bash", "-c"]

# Source ROS 2 by default
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Create Colcon workspace and download MoveIt2 tutorials
RUN mkdir -p ws_moveit/src && cd ws_moveit/src && \
    git clone -b main https://github.com/moveit/moveit2_tutorials && \
    vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos && \
    rm -rf ros2_kortex ros2_robotiq_gripper serial

# Remove previously installed MoveIt binaries
RUN sudo apt remove -y ros-$ROS_DISTRO-moveit* && sudo apt update
RUN sudo rosdep fix-permissions && rosdep update

# Install dependencies for MoveIt2
RUN cd /root/ws_moveit && \
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

# Build MoveIt2 with colcon
RUN cd /root/ws_moveit && \
    source /opt/ros/humble/setup.bash && \
    colcon build --parallel-workers 2 && \
    pip install mediapipe opencv-python
# Source MoveIt 2 workspace by default
RUN echo "source /root/ws_moveit/install/setup.bash" >> /root/.bashrc

WORKDIR /root/ws_moveit
