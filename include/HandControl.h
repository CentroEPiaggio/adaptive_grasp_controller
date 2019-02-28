/* HAND CONTROL - For closing SoftHand in to a desired position or at a desired velocity
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

// Basic includes
#include <ros/service.h>

// ROS msg includes
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

// Custom msg and srv includes
#include "adaptive_grasp_controller/hand_control.h"

// ROS action includes
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// Defines
#define     DEBUG   1       // Prints out additional stuff

class HandControl {

    /// public variables and functions ------------------------------------------------------------
	public:
		HandControl(ros::NodeHandle& nh_, int n_wp_, std::string synergy_joint_name_,
            boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> hand_client_ptr_);

        ~HandControl();

        // This is the callback function of the hand control service
	  	bool call_hand_control(adaptive_grasp_controller::hand_control::Request &req, adaptive_grasp_controller::hand_control::Response &res);

        // The callback function for the joint states subscriber
	  	void joints_callback(const sensor_msgs::JointState::ConstPtr &jnt_msg);

	  	// Initialize the things for setting up things. It is called by the callback
	  	bool initialize();

		// Performs computation of points towards goal
		void computeTrajectory(double present_syn, double goal_syn, double time);

		// Sends trajectory to the hand joint trajectory controller
		bool sendHandTrajectory(trajectory_msgs::JointTrajectory trajectory);

	/// private variables -------------------------------------------------------------------------
	private:
		ros::NodeHandle nh;

        // Subscriber to the joint states
        ros::Subscriber joints_sub;

        // The hand action client
        boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> hand_client_ptr;

        // The name and the present value of synergy joint
        std::string synergy_joint_name;
        double present_syn;

        // Latest saved joint states message
        sensor_msgs::JointState::ConstPtr saved_jnt_msg;

        // Number of waypoints (needed for 0 to 1 synergy) for trajectory points
        int n_wp;

        // The goal stuff and closing time (in seconds)
        double goal_value;
        double goal_duration;

        // Basic times
        ros::Duration nanosecond = ros::Duration(0, 1);
        ros::Duration millisecond = ros::Duration(0, 1000000);

        // Joint trajectory computed to be sent to robot
        trajectory_msgs::JointTrajectory computed_trajectory;  
	
};