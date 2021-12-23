#!/usr/bin/env bash

current_ip=$(hostname -I | awk '{print $1}')
export workspace=$HOME/Duckietown_manually/DB21M
echo "Setting up PYTHONPATH."
export PYTHONPATH=$workspace/catkin_ws/src:$PYTHONPATH

echo "Source workspace \"$workspace\"."
source ~/Duckietown_manually/DB21M/catkin_ws/devel/setup.bash

echo "Setting VEHICLE_NAME..."
if [ $# -gt 0 ]; then
	# provided a hostname, use it as ROS_MASTER_URI
	export VEHICLE_NAME=$1
	export ROS_MASTER_URI=http://$1.local:11311
else
	echo "No hostname provided. Using $HOSTNAME."
	export VEHICLE_NAME=$HOSTNAME
	export ROS_MASTER_URI=http://$current_ip:11311
fi
echo "VEHICLE_NAME set to $VEHICLE_NAME"
echo "ROS_MASTER_URI set to $ROS_MASTER_URI"
echo "ROS_HOSTNAME set to $current_ip"
echo ""
