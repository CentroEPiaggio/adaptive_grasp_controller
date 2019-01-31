# ADAPTIVE GRASP CONTROLLER

This package implements the Adaptive Grasp Controller for the Pisa/IIT SoftHand. This controller is to be integrated in the SOMA Planner.
In order to try this on a real robot, the IMU Glove for the Pisa/IIT SoftHand is required.

Many topic names and joint names are still hard coded in the src code, so be careful when using this code with any robot. This needs to be changed later by parsing everything from a proper yaml file.

## Authors

* **George Jose Pollayil** - [gpollayil](https://github.com/gpollayil)
* **Mathew Jose Pollayil** - [mpollayil](https://github.com/mpollayil)

## Getting Started

### Prerequisites

This package depends on ROS Indigo or newer.

To build the Adaptive Grasp Controller, it is necessary to have the following packages by Centro Piaggio:

(UPDATE HERE)
- finger_fk : https://github.com/CentroEPiaggio/finger_fk
- imu_glove_finger_touch_utils : https://github.com/CentroEPiaggio/imu_glove_finger_touch_utils (ONLY FOR REAL ROBOT)

So please install them properly before.

### Installing

To install this package just clone this and the other needed packages into your catkin workspace and catkin_make.

### Some preliminary notes

This controller has been coded in the form of a ROS service. Most of the parameters of the Adaptive Grasp can be changed from the `adaptive_grasp_config.yaml` file which is already loaded by the `launchAdaptiveGraspSim.launch` launch file. A short description of each parameter is given in the yaml file itself.

N.B: A proper and synced functioning of the Adaptive IMU Grasping depends highly on the SoftHand and KUKA hardware interface for ROS.

## Running the Adaptive Grasp

Roslaunch the launch files for the robot and the controller service as follows:

```
roslaunch adaptive_grasp_controller launchLWRSoftHandJointTraj.launch
```
```
roslaunch adaptive_grasp_controller launchAdaptiveGraspSim.launch
```

The first file launches a KUKA LWR 4+ robot with a Pisa/IIT SoftHand mounted on it. Please make sure that the args right_arm_enabled and right_hand_enabled are set correctly: these args should be set to true in case of a real robot launch and should be set to false for simulations in Gazebo.

### Trying the Adaptive Grasp with the IMU Glove

For doing experiments using the IMU Glove, the `finger_fk` service and the `collision_identification` node need to be running.

```
rosrun finger_fk finger_fk_main.py
```
```
roslaunch imu_glove_finger_touch_utils launchCollisionIdentification.launch
```

or include both in one of your launch files. Remember that the latter starts also the also the `qb_interface_imu` node.



### Trying the Adaptive Grasp without the IMU Glove

The adaptive grasp can still be tried on the robot without an acutal touch sensing device (IMU Glove) through publishing touches manually.
It is enough to publish the id of the finger, that we suppose touches the object, to the topic `/finger_touching_topic`.

The ids of the fingers are 1 for thumb, 2 for index, 3 for middle, 4 for ring, and 5 for little.

For example:

Before calling the Adaptive Grasp Controller service:

```
rostopic pub -r 10 /touching_finger_topic std_msgs/Int8 "data: 0"
```

When the Adaptive Grasp Controller service has been called and the SoftHand starts to close, a finger touch on the thumb can be passed manually to the controller by publishing (also on another terminal window) on `/finger_touching_topic` as follows:

```
rostopic pub -r 10 /touching_finger_topic std_msgs/Int8 "data: 1"
```

## Yet to be implemented (Force/Torque sensor on the wrist)
The compensating motion must stop obviously if a big force/torque is sensed on the force/torque sensor on the wrist of the KUKA. This can be done easily by adding an if condition in the function `void IMUGraspController::stopArmWhenCollision` in the file `IMUGraspController.cpp` where the function `cancelGoal()` is called.

## To be changed in later versions
- Replace arrays whith std::vectors
- Compute more precisely the values of the joints of a finger from the value of synergy


