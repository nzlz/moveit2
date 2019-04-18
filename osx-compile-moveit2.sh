#!/bin/bash

#set -x #Debug
set -e #exit on failure

export dir=$(PWD)
cd /tmp
source /tmp/ros2-osx/setup.bash
mkdir -p /tmp/ros2_ws/src
cp -r $dir /tmp/ros2_ws/src/moveit2
cd /tmp/ros2_ws && wget https://raw.githubusercontent.com/AcutronicRobotics/moveit2/master/moveit2.repos
vcs import src < moveit2.repos
export OPENSSL_ROOT_DIR="/usr/local/opt/openssl"
#Ignore packages
touch src/image_common/camera_calibration_parsers/COLCON_IGNORE
touch src/image_common/camera_info_manager/COLCON_IGNORE
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/usr/local/opt/qt
export PATH=$PATH:/usr/local/opt/qt/bin
export LIBRARY_PATH="/usr/local/opt/tinyxml2/lib/"
colcon build --merge-install --cmake-args -DBUILD_TESTING=False -DOpenMP_C_LIB_NAMES="omp" -DOpenMP_CXX_LIB_NAMES="omp" -DOpenMP_C_FLAGS="-Xpreprocessor -fopenmp -lomp -I/usr/local/opt/libomp/include" -DOpenMP_CXX_FLAGS="-Xpreprocessor -fopenmp -lomp -I/usr/local/opt/libomp/include" -DOpenMP_omp_LIBRARY="/usr/local/opt/libomp/lib/libomp.dylib"
