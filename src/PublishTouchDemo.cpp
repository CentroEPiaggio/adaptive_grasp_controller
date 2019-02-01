/**
    PublishTouchDemo.cpp

    Purpose:    a ROS node for SOMA simulation demo which publishes touches. This is used to show the adaptive motions 
                of different fingers

    @authors Pollayil George Jose, Pollayil Mathew Jose
*/


//BASIC INCLUDES
#include <sstream>
#include <string>
#include <exception>
#include <math.h>

// ROS INCLUDES
#include <ros/ros.h>
#include <ros/subscribe_options.h>

// MSG INCLUDES
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8.h>

// OTHER INCLUDES
#include <cmath>
#include <boost/thread.hpp>

#define DEBUG	        0
#define HAND_NAME	    "right_hand"

using namespace std;

// GLOBAL VARIABLES
sensor_msgs::JointState::ConstPtr full_joint_state;	        // a msg where the subscriber will save the joint states
ros::Publisher pub_fing_id; 							    // publisher for the id of the finger
int finger_id;
double input_synergy_threshold;

/**********************************************************************************************
 GET FINGER JOINT STATES
**********************************************************************************************/
double getFingerJointState(){
	// Creating a JointState to be filled up
	double finger_joint;

	// Saving the state of the hand synergy joint
    ROS_DEBUG_STREAM("STARTING TO FILL UP SYNERGY JOINT!");

	int index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
		string(HAND_NAME) + "_synergy_joint") - full_joint_state->name.begin();
    ROS_DEBUG_STREAM("IN MID TO FILL UP SYNERGY JOINT!");
	finger_joint = full_joint_state->position[index];

    ROS_DEBUG_STREAM("FINISHED TO FILL UP SYNERGY JOINT!");

	return finger_joint;
}

/**********************************************************************************************
 SUBSCRIBER CALLBACK
**********************************************************************************************/
void getJointStates(const sensor_msgs::JointState::ConstPtr &msg){
	// Storing the message into another global message variable
	ROS_DEBUG_STREAM("GOT JOINTSTATE MSG: STARTING TO SAVE!");
	full_joint_state = msg;
	ROS_DEBUG_STREAM("SAVED JOINTSTATE MSG!");
}

/**********************************************************************************************
 PARSING PARAMS
**********************************************************************************************/
bool getParamsOfYaml(){
	bool success = true;

	if(!ros::param::get("/publish_touch_demo/input_finger_id", finger_id)){
		ROS_WARN("input_finger_id param not found in param server! Using default value 1.");
		finger_id = 1;
		success = false;
    }
	
	if(!ros::param::get("/publish_touch_demo/input_synergy_threshold", input_synergy_threshold)){
		ROS_WARN("input_synergy_threshold param not found in param server! Using default value 0.3.");
		input_synergy_threshold = 0.3;
		success = false;
	}

	return success;
}



/* ******************************************************************************************** */

// MAIN
int main(int argc, char** argv){

	// Initializing ROS node
	ros::init(argc, argv, "publish_touch_demo");
	ros::NodeHandle* fjs_nh = new ros::NodeHandle();

    // Getting params from yaml
    if(!getParamsOfYaml()){
        ROS_ERROR_STREAM("Params not parsed correctly!");
    }

    // Converting the finger_id passed as arg: this will determine the adaptive motion
    std_msgs::Int8 finger_id_msg;
    finger_id_msg.data = finger_id;

    // Temporary joint state for saving synergy joint
    double synergy_joint;

	// The subscriber for saving joint states
	ros::SubscribeOptions joint_state_so = ros::SubscribeOptions::create<sensor_msgs::JointState>("joint_states", 
        1, getJointStates, ros::VoidPtr(), fjs_nh->getCallbackQueue());
	ros::Subscriber js_sub = fjs_nh->subscribe(joint_state_so);

    // Creating a ROS publisher for publishing all the joint states of the finger
	pub_fing_id = fjs_nh->advertise<std_msgs::Int8>("output_topic", 1);

	// Success message
	ROS_INFO("The Touch Publisher for demo ready to publish!");
	ROS_DEBUG_STREAM("DEBUG ACTIVATED!");

    // Spinning to process callback
    full_joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", *fjs_nh);
    ros::spinOnce();

	// Spining and looking for synergy value to publish at correct time
	while(ros::ok()){
        // Getting the present synergy
        synergy_joint = getFingerJointState();

        // Publishing the finger_id if synergy is at a certain value
        if(synergy_joint >= input_synergy_threshold){
            ROS_INFO("Publishing the finger_id on the touch topic!");
            pub_fing_id.publish(finger_id_msg);
        }

        // Spinning to process callback
        ros::spinOnce();
    }

    // Exit message
	ROS_INFO("Exiting Touch Publisher!");

}
