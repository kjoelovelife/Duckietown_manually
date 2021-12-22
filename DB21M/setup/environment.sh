#!/usr/bin/env bash

export workspace=$HOME/duckietown
echo "Setting up PYTHONPATH."
export PYTHONPATH=$workspace/catkin_ws/src:$PYTHONPATH

echo "Source workspace \"duckietown\"."
source ~/duckietown/DB21M/catkin_ws/devel/setup.bash

echo "Setting VEHICLE_NAME..."
if [ $# -gt 0 ]; then
	# provided a hostname, use it as ROS_MASTER_URI
	export VEHICLE_NAME=$1
else
	echo "No hostname provided. Using $HOSTNAME."
	export VEHICLE_NAME=$HOSTNAME
fi
echo "VEHICLE_NAME set to $VEHICLE_NAME"
echo ""
