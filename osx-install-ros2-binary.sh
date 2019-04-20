#!/bin/bash

#set -x #Debug
set -e #exit on failure

#Prepare for ros2
export dir=$(PWD)
cd /tmp
wget https://github.com/ros2/ros2/releases/download/release-crystal-20190408/ros2-crystal-20190408-macos-amd64.tar.bz2
tar jxf ros2-crystal-20190408-macos-amd64.tar.bz2
# Remove tf2_eigen
find ros2-osx/ -name tf2_eigen | xargs rm -rf
find ros2-osx/ -name *connext* | xargs rm -rf # Try to remove all warnings of RTI connext
