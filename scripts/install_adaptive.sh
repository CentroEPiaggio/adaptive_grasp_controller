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

echo " "
echo "***************************************"
echo " ADAPTIVE GRASP CONTROLLER - INSTALLER "
echo "***************************************"

sleep 2

# Ask if the system has Indigo or Kinetic
echo " "
echo "As we are in SOMA, it is recommended that you have ROS Indigo."
echo " "
echo "However, are you using ROS Indigo or ROS Kinetic?"
echo "Type 'i' for ROS Indigo or 'k' for ROS Kinetic."
read -n1 rosdist

if [[ "$rosdist" == "i" ]]; then
    echo " "
    echo "Got It! ROS Indigo."
elif [[ "$rosdist" == "k" ]]; then
    echo " "
    echo "Got It! ROS Kinetic."
else
    echo " "
    echo "ERROR: the typed char is $rosdist, it should have been either 'i' or 'k'!"
    echo "Please re-run the script and choose a valid option."
    exit
fi

# Ask if this package is cloned inside the src folder of catkin workspace
echo " "
echo "Did you clone the adaptive_grasp_controller package inside your catkin workspace src folder?"
echo "Type 'y' to confirm or 'n' otherwise."
read -n1 confirm

if [[ "$confirm" == "y" ]]; then
    echo " "
    echo "Proceeding!"
elif [[ "$confirm" == "n" ]]; then
    echo " "
    echo "Exiting! Please clone the package in your catkin workspace src folder!"
    exit
else
    echo " "
    echo "ERROR: the typed char is $confirm, it should have been either 'y' or 'n'!"
    echo "Please re-run the script and choose a valid option."
    exit
fi

echo " "
echo "---"

# Ask if to use catkin_make or catkin build
echo " "
echo "What is your catkin build tool?"
echo "Type 'm' for caktin_make or 'b' for catkin build."
read -n1 btl

if [[ "$btl" == "m" ]]; then
    echo " "
    echo "Using catkin_make for building the packages!"
elif [[ "$btl" == "b" ]]; then
    echo " "
    echo "Using catkin build for building the packages!"
else
    echo " "
    echo "ERROR: the typed char is $btl, it should have been either 'm' or 'b'!"
    echo "Please re-run the script and choose a valid option."
    exit
fi
echo " "
echo "***"

#-----------#
# FUNCTIONS #
#-----------#

# This function checks if the given packages are installed, if some are missing it stops the script
function test_if_package_is_installed
{
 stopscript=0
 until [ -z "$1" ]; do
    if [ $(dpkg-query -W -f='${Status}' $1 2>/dev/null | grep -c "ok installed") -eq 0 ];then
        echo " "
        echo "Please install package $1 and re-run the script!"
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
    echo " "
    echo "$1 failed. Exiting now!"
    exit $laststatus
 fi
}

#---------------------------#
# CHECKING FOR DEPENDENCIES #
#---------------------------#

echo " "
echo "Assuming that the most basic ROS packages are installed (RViz, Gazebo, MoveIt, TF and Actionlib). Checking for other needed packages!"

if [[ "$rosdist" == "i" ]]; then
    test_if_package_is_installed ros-indigo-rviz-* ros-indigo-actionlib-msgs ros-indigo-interactive-markers ros-indigo-visualization-msgs
    test_if_package_is_installed libsdformat2-dev sdformat-sdf libeigen3-dev ros-indigo-controller-interface ros-indigo-control-msgs
    test_if_package_is_installed ros-indigo-forward-command-controller ros-indigo-control-toolbox ros-indigo-realtime-tools ros-indigo-urdf
    test_if_package_is_installed ros-indigo-kdl-parser ros-indigo-kdl-conversions ros-indigo-cmake-modules ros-indigo-tf-conversions
    test_if_package_is_installed ros-indigo-controller-manager ros-indigo-hardware-interface ros-indigo-joint-limits-interface
    test_if_package_is_installed ros-indigo-pluginlib ros-indigo-transmission-interface
else
    test_if_package_is_installed ros-kinetic-rviz-* ros-kinetic-actionlib-msgs ros-kinetic-interactive-markers ros-kinetic-visualization-msgs
    test_if_package_is_installed libsdformat2-dev sdformat-sdf libeigen3-dev ros-kinetic-controller-interface ros-kinetic-control-msgs
    test_if_package_is_installed ros-kinetic-forward-command-controller ros-kinetic-control-toolbox ros-kinetic-realtime-tools ros-kinetic-urdf
    test_if_package_is_installed ros-kinetic-kdl-parser ros-kinetic-kdl-conversions ros-kinetic-cmake-modules ros-kinetic-tf-conversions
    test_if_package_is_installed ros-kinetic-controller-manager ros-kinetic-hardware-interface ros-kinetic-joint-limits-interface
    test_if_package_is_installed ros-kinetic-pluginlib ros-kinetic-transmission-interface
fi

echo " "
echo "It seems you have the required packages. Proceeding!"
echo " "
echo "***"

sleep 2

#--------------------------#
# CLONING AND CHECKING OUT #
#--------------------------#

# Getting the script directory path and cd to it
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd $SCRIPT_DIR

sleep 2

# Going back to src of catkin_workspace
cd ../..
CWS_DIR=$(pwd)
echo " "
echo "The path to the catkin workspace src is $CWS_DIR. I jumped to this directory."

# Starting to clone the packages and switching to relevant branch
echo " "
echo "Starting to clone the repos for Adaptive Grasping and switching to correct branches!"

echo " "
echo "---"

# Package hrl-kdl
if ! [ -d "hrl-kdl" ]; then
    echo " "
	echo "Cloning KDL utility package: hrl-kdl."
    echo " "
	git clone https://github.com/CentroEPiaggio/hrl-kdl.git hrl-kdl
	check_last_exit_code "Cloning hrl-kdl"
	cd hrl-kdl; git checkout indigo-devel
    cd $CWS_DIR
else
    echo " "
	echo "It seems that you already have KDL utility package: hrl-kdl! Skipping it..."
    echo "Is it the correct one? Later, please make sure it is on the required branch (see README.md)."
fi

echo " "
echo "---"

# Package finger_fk
if ! [ -d "finger_fk" ]; then
    echo " "
	echo "Cloning Finger FK package: finger_fk."
    echo " "
	git clone https://github.com/CentroEPiaggio/finger_fk.git finger_fk
	check_last_exit_code "Cloning finger_fk"
	cd finger_fk; git checkout soma_july_review
    cd $CWS_DIR
else
    echo " "
	echo "It seems that you already have Finger FK package: finger_fk! Skipping it..."
    echo "Is it the correct one? Later, please make sure it is on the required branch (see README.md)."
fi

echo " "
echo "---"

# Package IMU
if ! [ -d "IMU" ]; then
    echo " "
	echo "Cloning IMU Glove package: IMU."
    echo " "
	git clone https://github.com/CentroEPiaggio/IMU.git IMU
	check_last_exit_code "Cloning IMU"
	cd IMU; git checkout master
    echo " "
    echo "Creating executables qbAPI and qbadmin."
    cd Management/qbAPI/src; make
    cd ../../qbadmin/src; make
    cd $CWS_DIR
else
    echo " "
	echo "It seems that you already have IMU Glove package: IMU! Skipping it..."
    echo "Is it the correct one? Later, please make sure it is on the required branch (see README.md)."
fi

echo " "
echo "---"

# Package kuka-lwr
if ! [ -d "kuka-lwr" ]; then
    echo " "
	echo "Cloning KUKA LWR package: kuka-lwr."
    echo " "
	git clone https://github.com/CentroEPiaggio/kuka-lwr.git kuka-lwr
	check_last_exit_code "Cloning kuka-lwr"
	cd kuka-lwr; git checkout soma_devel
    cd $CWS_DIR
else
    echo " "
	echo "It seems that you already have KUKA LWR package: kuka-lwr! Skipping it..."
    echo "Is it the correct one? Later, please make sure it is on the required branch (see README.md)."
fi

echo " "
echo "---"

# Package pisa-iit-soft-hand
if ! [ -d "pisa-iit-soft-hand" ]; then
    echo " "
	echo "Cloning Pisa/IIT SoftHand package recursively: pisa-iit-soft-hand."
    echo " "
	git clone --recursive https://github.com/CentroEPiaggio/pisa-iit-soft-hand.git pisa-iit-soft-hand
	check_last_exit_code "Cloning pisa-iit-soft-hand"
	cd pisa-iit-soft-hand; git checkout indigo_devel
    cd $CWS_DIR
else
    echo " "
	echo "It seems that you already have Pisa/IIT SoftHand package: pisa-iit-soft-hand! Skipping it..."
    echo "Is it the correct one? Later, please make sure it is on the required branch (see README.md)."
fi

echo " "
echo "---"

# Package vito-robot
if ! [ -d "vito-robot" ]; then
    echo " "
	echo "Cloning Vito Robot package: vito-robot."
    echo " "
	git clone https://github.com/CentroEPiaggio/vito-robot.git vito-robot
	check_last_exit_code "Cloning vito-robot"
	cd vito-robot; git checkout soma_july_review
    cd $CWS_DIR
else
    echo " "
	echo "It seems that you already have Vito Robot package: vito-robot! Skipping it..."
    echo "Is it the correct one? Later, please make sure it is on the required branch (see README.md)."
fi

echo " "
echo "***"

sleep 2

#-----------#
# COMPILING #
#-----------#

# Going to catkin workspace folder
cd $CWS_DIR; cd ..

# For compiling faster
MAKEFLAGS="-j8"

# Making packages in the correct order

echo " "
echo "Finished cloning... Now proceeding to make/build the repos for Adaptive Grasping!"

if [[ "$btl" == "m" ]]; then
    echo " "
	echo "Making hrl_kdl package."
    echo " "
    catkin_make --pkg hrl_geom pykdl_utils
    echo " "
	echo "Making finger_fk package."
    echo " "
    catkin_make --pkg finger_fk
    echo " "
	echo "Making IMU (qb_interface) package."
    echo " "
    catkin_make --pkg qb_interface
    echo " "
	echo "Making kuka-lwr package."
    echo " "
    catkin_make --pkg lwr_controllers lwr_description lwr_hw single_lwr_launch single_lwr_moveit single_lwr_robot
    echo " "
	echo "Making pisa-iit-soft-hand package."
    echo " "
    catkin_make --pkg gazebo_ros_soft_hand soft_hand_controllers soft_hand_description soft_hand_qb_ros_control soft_hand_ros_control
    echo " "
	echo "Making vito-robot package."
    echo " "
    catkin_make --pkg vito_description vito_moveit_configuration
    echo " "
	echo "And finally making adaptive_grasp_controller package."
    echo " "
    catkin_make --pkg adaptive_grasp_controller
else
    echo " "
	echo "Building hrl_kdl package."
    echo " "
    catkin build hrl_geom pykdl_utils
    echo " "
	echo "Building finger_fk package."
    echo " "
    catkin build finger_fk
    echo " "
	echo "Building IMU (qb_interface) package."
    echo " "
    catkin build qb_interface
    echo " "
	echo "Building kuka-lwr package."
    echo " "
    catkin build lwr_controllers lwr_description lwr_hw single_lwr_launch single_lwr_moveit single_lwr_robot
    echo " "
	echo "Building pisa-iit-soft-hand package."
    echo " "
    catkin build gazebo_ros_soft_hand soft_hand_controllers soft_hand_description soft_hand_qb_ros_control soft_hand_ros_control
    echo " "
	echo "Building vito-robot package."
    echo " "
    catkin build vito_description vito_moveit_configuration
    echo " "
	echo "And finally building adaptive_grasp_controller package."
    echo " "
    catkin build adaptive_grasp_controller
fi

echo " "
echo "Finished making/building... Exiting!"
echo " "
echo "Please refer to the README.md of adaptive_grasp_controller and of the other packages for execution."

echo " "
echo "***"




