#!/bin/bash
# -------------------------------------------------------------------------------------------------------- #
# Copyright (c) 2018, Bioengineering and Robotics Research Center "E. Piaggio"
# Some rights reserved.
#
# ADAPTIVE GRASP CONTROLLER - INSTALLER
#
# This script clones and builds all necessary packages needed for Adaptive Grasp Controller (version 1.0)
# Before running this script make sure that this repository (adaptive_grasp_controller) is in your ROS ws
# -------------------------------------------------------------------------------------------------------- #

echo "***************************************"
echo " ADAPTIVE GRASP CONTROLLER - INSTALLER "
echo "***************************************"

sleep 2

echo "As we are in SOMA, I am assuming that this system has ROS Indigo."

# Ask if to use catkin_make or catkin build
echo "What is your catkin build tool? Type 'm' for caktin_make or 'b' for catkin build."
read -n 1 btl
echo " "

if ["$btl" == "m"]; then
    echo "Using catkin_make for building the packages!"
elif ["$btl" == "b"]; then
    echo "Using catkin build for building the packages!"
else
    echo "ERROR: the typed char should have been either 'm' or 'b'!"
    echo "Please re-run the script and choose a valid option."
    exit
fi

#-----------#
# FUNCTIONS #
#-----------#

# This function checks if the given packages are installed, if some are missing it stops the script
function test_if_package_is_installed
{
 stopscript=0
 until [ -z "$1" ]; do
    if [ $(dpkg-query -W -f='${Status}' $1 2>/dev/null | grep -c "ok installed") -eq 0 ];then
       echo "Please install package $1 and rerun the script"
       stopscript=1
    fi
    shift
 done
 if [ "$stopscript" -eq "1" ];then
	exit
 fi
}

# This function checks if the last exit code was success (0) and exits the script in case it was not success (the error code is preserved)
function check_last_exit_code
{
 local laststatus=$?
 if [ $laststatus -ne 0 ]
 then
    echo "$1 failed. Exit now!"
    exit $laststatus
 fi
}

#---------------------------#
# CHECKING FOR DEPENDENCIES #
#---------------------------#

echo "Assuming that the most basic ROS packages are installed (RViz, Gazebo, MoveIt, TF and Actionlib). Checking for other needed packages!"
test_if_package_is_installed ros-indigo-rviz-* ros-indigo-actionlib-msgs ros-indigo-interactive-markers ros-indigo-visualization-msgs
test_if_package_is_installed libsdformat2-dev sdformat-sdf libeigen3-dev ros-indigo-controller-interface ros-indigo-control-msgs
test_if_package_is_installed ros-indigo-forward-command-controller ros-indigo-control-toolbox ros-indigo-realtime-tools ros-indigo-urdf
test_if_package_is_installed ros-indigo-kdl-parser ros-indigo-kdl-conversions ros-indigo-cmake-modules ros-indigo-tf-conversions
test_if_package_is_installed ros-indigo-controller-manager ros-indigo-hardware-interface ros-indigo-joint-limits-interface
test_if_package_is_installed ros-indigo-pluginlib ros-indigo-transmission-interface 

#--------------------------#
# CLONING AND CHECKING OUT #
#--------------------------#



