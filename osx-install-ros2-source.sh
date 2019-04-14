#!/bin/bash

#set -x #Debug
set -e #exit on failure

# Install ROS 2 from sources in OS X
export dir=$(PWD)
cd /tmp

export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/usr/local/opt/qt
export PATH=$PATH:/usr/local/opt/qt/bin
python3 -m pip install argcomplete catkin_pkg colcon-common-extensions coverage empy flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes git+https://github.com/lark-parser/lark.git@0.7d mock nose pep8 pydocstyle pyparsing setuptools vcstool
# Unable to disable CSR thereby, won't be able to launch examples, just compile
mkdir -p ros2_ws/src
cd ros2_ws
wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
vcs import src < ros2.repos

############
# Fetch the right branches in some conflicting repos
############
# Fix issue with Fast-RTPS by fetching master
cd /tmp/ros2_ws/src/eProsima/Fast-RTPS/
git checkout master
# Fast-CDR
cd /tmp/ros2_ws/src/eProsima/Fast-CDR/
git checkout master
# rclcpp
cd /tmp/ros2_ws/src/ros2/rclcpp
git checkout master
# rcl
cd /tmp/ros2_ws/src/ros2/rcl
git checkout master
# rmw
cd /tmp/ros2_ws/src/ros2/rmw
git checkout master
# rmw_fasrtps
cd /tmp/ros2_ws/src/ros2/rmw_fastrtps
git checkout master
# rmw_implementation
cd /tmp/ros2_ws/src/ros2/rmw_implementation
git checkout master
# rcl_interfaces
cd /tmp/ros2_ws/src/ros2/rcl_interfaces
git checkout master

############
# Fetch packages not present in the ws by default
############
cd /tmp/ros2_ws/src
git clone https://github.com/ros2/rcpputils

cd /tmp/ros2_ws/
# IGNORE rviz cause apparently XCode is required (not only the tools)
touch src/ros2/rviz/COLCON_IGNORE
# Issues with Qt and dependencies
touch src/ros-visualization/rqt/COLCON_IGNORE
touch src/ros-visualization/qt_gui_core/COLCON_IGNORE
colcon build --symlink-install

# # Remove tf2_eigen
# find ros2-osx/ -name tf2_eigen | xargs rm -rf
