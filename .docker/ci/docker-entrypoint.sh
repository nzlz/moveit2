#!/bin/bash

set -e

# # setup ros2 environment
source "/opt/ros/crystal/setup.bash"

find /opt/ros/crystal/ -name tf2_eigen | xargs rm -rf && source /opt/ros/crystal/setup.bash \
	       && touch /opt/ws_moveit/src/image_common/camera_calibration_parsers/COLCON_IGNORE \
              && touch /opt/ws_moveit/src/image_common/camera_info_manager/COLCON_IGNORE \
              && colcon build --merge-install

exec "$@"
