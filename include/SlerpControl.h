/* SLERP CONTROL - Uses SLERP interpolation between presnt ee pose and goal pose to create homogeneous motion
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

// Basic includes
#include <ros/service.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// ROS msg includes
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

// Custom msg and srv includes
#include "adaptive_grasp_controller/slerp_control.h"

// MoveIt! includes
#include <moveit/move_group_interface/move_group_interface.h>

// ROS action includes
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// Defines
#define     DEBUG   1       // Prints out additional stuff

class SlerpControl {

    /// public variables and functions ------------------------------------------------------------
	public:
		SlerpControl(ros::NodeHandle& nh_, std::string group_name_, std::string end_effector_name_, int n_wp_,
            boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> arm_client_ptr_);

        ~SlerpControl();

        // This is the callback function of the slerp control service
	  	bool call_slerp_control(adaptive_grasp_controller::slerp_control::Request &req, adaptive_grasp_controller::slerp_control::Response &res);

	  	// Initialize the things for motion planning. It is called by the callback
	  	bool initialize(geometry_msgs::Pose goal_pose, bool is_goal_relative);

		// Performs motion planning for the end-effector towards goal
		bool performMotionPlan();

		// Computes waypoints using SLERP from two poses
		void computeWaypointsFromPoses(const Eigen::Affine3d& start_pose, const Eigen::Affine3d& goal_pose, std::vector<geometry_msgs::Pose>& waypoints);

		// Sends trajectory to the joint_traj controller
		bool sendJointTrajectory(trajectory_msgs::JointTrajectory trajectory);

	/// private variables -------------------------------------------------------------------------
	private:
		ros::NodeHandle nh;

        // Important names
        std::string end_effector_name;                          // Name of the end-effector link
        std::string group_name;                                 // Name of the MoveIt group

        // Number of waypoints for slerp interpolation
        int n_wp;

        // The arm action client
        boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> arm_client_ptr;

        // Tf listener and transform and the tmp eigen
	    tf::TransformListener tf_listener;
        tf::StampedTransform stamp_ee_transform;
        Eigen::Affine3d end_effector_state;

        // The start and goal poses of the control
	  	Eigen::Affine3d startAff; 								// The starting ee pose 
	  	Eigen::Affine3d goalAff; 								// The goal pose given by service call

        // Joint trajectory computed to be sent to robot
        trajectory_msgs::JointTrajectory computed_trajectory;  
	
};