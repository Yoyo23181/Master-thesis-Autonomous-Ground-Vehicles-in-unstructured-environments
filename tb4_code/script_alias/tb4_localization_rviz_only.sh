#!/bin/bash -i

SCRIPT=$(readlink -f $0)

SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"

echo $SCRIPTPATH

tmuxinator start -p /ros_ws/src/tb4_code/script_alias/simulation_launch/tb4_localization_rviz_only.yaml

shopt -s expand_aliases
source ~/.bashrc