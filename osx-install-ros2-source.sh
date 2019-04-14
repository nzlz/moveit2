#!/bin/bash

#set -x #Debug
set -e #exit on failure

# Install ROS 2 from sources in OS X
export dir=$(PWD)
cd /tmp

# TODO

# Remove tf2_eigen
find ros2-osx/ -name tf2_eigen | xargs rm -rf
