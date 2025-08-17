#!/bin/bash

cd /ros_ws
colcon build --symlink-install --packages-select tb4_code
source /ros_ws/install/setup.bash