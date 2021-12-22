#!/usr/bin/env bash

# Shell script scripts to install jetfalcon dependencies
# -------------------------------------------------------------------------
#Copyright © 2021 Wei-Chih Lin , kjoelovelife@gmail.com 

#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in all
#copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.
# -------------------------------------------------------------------------
# reference
# https://chtseng.wordpress.com/2019/05/01/nvida-jetson-nano-%E5%88%9D%E9%AB%94%E9%A9%97%EF%BC%9A%E5%AE%89%E8%A3%9D%E8%88%87%E6%B8%AC%E8%A9%A6/
#
# -------------------------------------------------------------------------

if [[ `id -u` -eq 0 ]] ; then
    echo "Do not run this with sudo (do not run random things with sudo!)." ;
    exit 1 ;
fi

# Parameters
sleep_time=3s
ubuntu_distro=$(grep RELEASE /etc/lsb-release | awk -F '=' '{print $2}')
ros_distro=$ROS_DISTRO
hardware_architecture=$(uname -i)
project_name="jetfalcon"


# functions

function text_color(){
<<"###comment"
	Select color of text
	  Args: 
	    $1: color
		$2: ground, default is fore, can select "fore" or "back"
	  Return: code of color
###comment
	color=("black" "red" "green" "yellow" "blue" "purple" "cyan" "white") 
	declare -A color_code
	initial_color_code=30
	for _color in ${color[*]};
	do
		color_code[$_color]=$initial_color_code
		initial_color_code=$(($initial_color_code + 1))
	done

	if [ $# -eq 1 ]; then
		echo "${color_code[$1]}"
	else
		if [ "$2" == "fore" ]; then
			echo "${color_code[$1]}"
		elif [ "$2" == "back" ]; then
			echo $((${color_code[$1]} + 10))
		else
			echo "${color_code[$1]}"
		fi
	fi
}

function apt_install(){
<<'###comment'
    Use apt install ros2 dependencies for this project
    Args:
      $1: project name
###comment
    _ros_distro=$1
    sudo apt install ros-${_ros_distro}-teleop-twist-keyboard

}



# Install
apt_install $ros_distro