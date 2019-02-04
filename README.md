# ADAPTIVE GRASP CONTROLLER

This package implements the Adaptive Grasp Controller for the *Pisa/IIT SoftHand* and the *KUKA LWR*. This controller can be used as an alternative for the simple grasp in the SOMA Planner.
In order to try this on a real robot, the *IMU Glove* for the *Pisa/IIT SoftHand* is required.

#### Authors

* **George Jose Pollayil** - [gpollayil](https://github.com/gpollayil)
* **Mathew Jose Pollayil** - [mpollayil](https://github.com/mpollayil)

## Getting Started

### Prerequisites

This package is tested to work on Ubuntu 14.04 with ROS Indigo.
To build the Adaptive Grasp Controller, it is necessary to have the following packages from the GitHub page of Centro Piaggio:

(UPDATE HERE)
- **kuka-lwr** (branch: soma_devel) : https://github.com/CentroEPiaggio/kuka-lwr
- **pisa-iit-soft-hand** (branch: qb_interface_devel) : https://github.com/CentroEPiaggio/pisa-iit-soft-hand
- **vito-robot** (branch: soma_july_review) : https://github.com/CentroEPiaggio/vito-robot
- **finger_fk** (branch: soma_july_review) : https://github.com/CentroEPiaggio/finger_fk
- **imu_glove_finger_touch_utils** (branch: master) : https://github.com/CentroEPiaggio/imu_glove_finger_touch_utils 

So please install them properly before.

### Installing

To install this package just clone this and the other needed packages into your catkin workspace and catkin_make.

### Some preliminary notes

This controller has been coded in the form of a ROS service. Most of the parameters of the Adaptive Grasp can be changed from the `adaptive_grasp_config.yaml` file which is already loaded by the `launchAdaptiveGraspSim.launch` launch file. A short description of each parameter is given in the yaml file itself.

**N.B.** A proper and synced functioning of the Adaptive IMU Grasping depends highly on the specifications of the used system (thus on a smooth functioning of the *SoftHand* and *KUKA* hardware interface for ROS).

## Running the Adaptive Grasp

### Main files to be launched

Roslaunch the launch files for the robot and the controller service as follows:

```
roslaunch adaptive_grasp_controller launchLWRSoftHandJointTraj.launch
roslaunch adaptive_grasp_controller launchAdaptiveGraspSim.launch
```

The first file launches a *KUKA LWR 4+* robot with a *Pisa/IIT SoftHand* mounted on it. Please make sure that the args `right_arm_enabled` and `right_hand_enabled` are set correctly: these args should be set to `true` in case of a real robot launch (in this case the arg `use_gazebo` must be set to `false`) and should be set to `false` for simulations in Gazebo (obviously now the arg `use_gazebo` must be set to `true`).

### Trying the Adaptive Grasp with the IMU Glove

Here we suppose that the robot is already in a grasp pose above the object to be grabbed. In order to do experiments using the IMU Glove, the `finger_fk` service node and the `collision_identification` node need to be running. 

```
rosrun finger_fk finger_fk_main.py
roslaunch imu_glove_finger_touch_utils launchCollisionIdentification.launch
```

For running the `finger_fk` service it is also sufficient to set the arg `use_other_utils` is set to true. However the `collision_identification` node need to be launched separately so that it does not crash. Remember that the latter starts also the `qb_interface_imu` node. 

Then call the Adaptive Grasp service by typing in a new terminal window the following:

```
rosservice call /adaptive_grasp_controller "goal: 0.0"
```

This will make the robot start to close the hand and whenever a touch is found by the `collision_identification` node the id of the finger in collision will be published to the topic `/finger_touching_topic`. This will cause a change in the grasping strategy (For more, refer to the paper).

### Trying the Adaptive Grasp without the IMU Glove

The Adaptive Grasp can still be tried on the robot without an acutal touch sensing device (*IMU Glove*) through publishing touches manually.
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

## Yet to be implemented 

### Force/Torque sensor on the wrist

The Adaptive motion must stop obviously if a big force/torque is sensed on the force/torque sensor on the wrist of the *KUKA*. This can be done easily by adding an if condition in the function `void AdaptiveGraspController::stopArmWhenCollision` in the file `AdaptiveGraspController.cpp` where the function `cancelGoal()` is called.

### To be changed in later versions
- Replace arrays whith std::vectors
- Compute more precisely the values of the joints of a finger from the value of synergy


