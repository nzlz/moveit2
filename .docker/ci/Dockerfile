# Sets up a base image to use for running Continuous Integration on Travis

FROM ros:crystal-ros-base-bionic
MAINTAINER Dave Coleman dave@picknik.ai

ENV TERM xterm

# Setup ros2 workspace
ENV ROS2_WS=/root/ws_moveit2
WORKDIR $ROS2_WS

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
# https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#minimize-the-number-of-layers
RUN \
    mkdir src && \
    #
    # Update apt package list as previous containers clear the cache
    apt-get -qq update && \
    apt-get -qq dist-upgrade && \
    #
    # Install some base dependencies
    apt-get -qq install -y \
        # Some source builds require a package.xml be downloaded via wget from an external location
        python3-colcon-common-extensions \
        libboost-all-dev \
        pkg-config \
        libfcl-dev \
        libassimp-dev \
        libqhull-dev \
        ros-$ROS_DISTRO-action-msgs \
        ros-$ROS_DISTRO-resource-retriever \
        python3-vcstool \
        wget \
        # Required for rosdep command
        # Preferred build tools
        clang clang-format-3.9 clang-tidy clang-tools \
        ccache && \
    #
    # Download all dependencies of MoveIt2!
    wget https://raw.githubusercontent.com/AcutronicRobotics/moveit2/master/external-repos.repos \
    && vcs import src < external-repos.repos

    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*

# Continous Integration Setting
ENV IN_DOCKER 1
