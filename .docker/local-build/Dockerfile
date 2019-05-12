FROM ubuntu:bionic

LABEL maintainer="Lander Usategui lander@erlerobotics.com"

ENV TERM xterm
ENV ROS_DISTRO crystal
ENV ROS_WS=/opt/ws_moveit
WORKDIR $ROS_WS

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN \
     echo 'Etc/UTC' > /etc/timezone \
      && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
      && apt-get update -qq && apt-get install -qq -y tzdata dirmngr gnupg2 lsb-release curl \
      # setup ros2 keys
      && curl http://repo.ros2.org/repos.key | apt-key add - \
      # setup sources.list
      && echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list \
      && apt-get update -qq && apt-get install -qq -y \
         build-essential \
         cmake \
         git \
         python3-colcon-common-extensions \
         python3-pip \
         python3-vcstool \
         python3-wstool \
         python-rosdep \
         wget \
      	 libboost-all-dev \
         libglew-dev \
         freeglut3-dev \
         pkg-config \
         libfcl-dev \
         libassimp-dev \
         libqhull-dev \
         libopencv-dev \
         clang clang-format-3.9 clang-tidy clang-tools \
         ros-$ROS_DISTRO-ros-base \
         ros-$ROS_DISTRO-action-msgs \
         ros-$ROS_DISTRO-message-filters \
         ros-$ROS_DISTRO-rclcpp-action \
         ros-$ROS_DISTRO-resource-retriever \
         libompl-dev \
      && rm -rf /var/lib/apt/lists/*

# Install Fast-RTPS dependencies
RUN apt-get -qq update && \
        apt-get install -qq --no-install-recommends -y \
            libasio-dev \
            libtinyxml2-dev \
            && \
            # Clear apt-cache to reduce image size
            rm -rf /var/lib/apt/lists/*

ARG MOVEIT2_BRANCH=
# Download moveit source, so that we can get necessary dependencies
RUN mkdir -p $ROS_WS/src \
    && bash -c 'if [ -z "$MOVEIT2_BRANCH" ]; then MOVEIT2_BRANCH=master; else if [ "$(git ls-remote https://github.com/AcutronicRobotics/moveit2 $MOVEIT2_BRANCH | wc -l)" -eq 0 ]; then exit 1234; fi fi' \
    && wget https://raw.githubusercontent.com/AcutronicRobotics/moveit2/master/moveit2.repos \
    && echo "  moveit2:\n    type: git\n    url: https://github.com/AcutronicRobotics/moveit2\n    version: $MOVEIT2_BRANCH" >> moveit2.repos \
    && vcs import src < moveit2.repos
    # wstool init --shallow . https://raw.githubusercontent.com/ros-planning/moveit2/master/moveit.rosinstall

# Download all MoveIt 2 dependencies
RUN \
    apt-get -qq update && \
    rosdep init && \
    rosdep update -q && \
    rosdep install -q -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false || true && \
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*

COPY ./docker-entrypoint.sh /
ENTRYPOINT ["/docker-entrypoint.sh"]

CMD ["bash"]
