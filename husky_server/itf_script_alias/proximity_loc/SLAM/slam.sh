#!/bin/bash -i

SCRIPT=$(readlink -f $0)

SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"

echo $SCRIPTPATH

tmuxinator start -p /ros_ws/src/ITF_husky_demo/ITF_ros2_package/itf_script_alias/proximity_loc/SLAM/slam.yaml

shopt -s expand_aliases
source ~/.bashrc
