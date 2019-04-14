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
# Fix issue with Fast-RTPS
cd src/eProsima/Fast-RTPS/
git checkout 3a1bf2ab6670cf5295eae3af5641522c84716c35
cd /tmp/ros2_ws/
# IGNORE rviz cause apparently XCode is required (not only the tools)
touch src/ros2/rviz/COLCON_IGNORE
# Issues with Qt and dependencies
touch src/ros-visualization/rqt/COLCON_IGNORE
touch src/ros-visualization/qt_gui_core/COLCON_IGNORE
colcon build --symlink-install

# # Remove tf2_eigen
# find ros2-osx/ -name tf2_eigen | xargs rm -rf
