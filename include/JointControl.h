/* JOINT CONTROL - Uses moveit movegroupinterface to plan towards a joint configuration
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

// Basic includes
#include <ros/service.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// ROS msg includes
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

// Custom msg and srv includes
#include "adaptive_grasp_controller/joint_control.h"

// MoveIt! includes
#include <moveit/move_group_interface/move_group_interface.h>

// ROS action includes
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// Defines
#define     DEBUG   1       // Prints out additional stuff
#define     VISUAL          // Publishes visual info on RViz

class JointControl {

    /// public variables and functions ------------------------------------------------------------
	public:
		JointControl(ros::NodeHandle& nh_, std::string group_name_,
            boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> arm_client_ptr_);

        ~JointControl();

        // This is the callback function of the joint control service
	  	bool call_joint_control(adaptive_grasp_controller::joint_control::Request &req, adaptive_grasp_controller::joint_control::Response &res);

	  	// Initialize the things for motion planning. It is called by the callback
	  	bool initialize(adaptive_grasp_controller::joint_control::Request &req);

		// Performs motion planning for the joints towards goal
		bool performMotionPlan();

		// Sends trajectory to the joint_traj controller
		bool sendJointTrajectory(trajectory_msgs::JointTrajectory trajectory);

	/// private variables -------------------------------------------------------------------------
	private:
		ros::NodeHandle nh;

        // Important names and values
        std::string group_name;                                 // Name of the MoveIt group

        // The arm action client
        boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> arm_client_ptr;

        // The present joint config and the goal joint config
		std::vector<double> joint_now;							// The current joint config
	  	std::vector<double> joint_goal;							// The goal joint config given by service call

        // Joint trajectory computed to be sent to robot
        trajectory_msgs::JointTrajectory computed_trajectory;  
	
};