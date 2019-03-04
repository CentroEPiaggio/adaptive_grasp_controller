/* SLERP CONTROL - Uses SLERP interpolation between ee pose and goal pose to create homogeneous motion
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

#include "ros/ros.h"
#include <cstdlib>
#include <sstream>
#include <string>

#include "SlerpControl.h"

#include <moveit_visual_tools/moveit_visual_tools.h>

SlerpControl::SlerpControl(ros::NodeHandle& nh_, std::string group_name_, std::string end_effector_name_,  int n_wp_,
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> arm_client_ptr_){
        
        ROS_INFO("Starting to create SlerpControl object");

        // Initializing node handle
        this->nh = nh_;

        // Initializing names
        this->end_effector_name = end_effector_name_;
        this->group_name = group_name_;

        // Setting number of waypoints 
        this->n_wp = n_wp_;

        // Initializing the arm client
        this->arm_client_ptr = arm_client_ptr_;

        ROS_INFO("Finished creating SlerpControl object");
}

SlerpControl::~SlerpControl(){
    
    // Nothing to do here yet
}

// This is the callback function of the slerp control service
bool SlerpControl::call_slerp_control(adaptive_grasp_controller::slerp_control::Request &req, adaptive_grasp_controller::slerp_control::Response &res){

    // Setting up things
    if(!this->initialize(req.goal_pose, req.is_goal_relative)){
        ROS_ERROR("Could not initialize SlerpControl object. Returning...");
        res.answer = false;
        return false;
    }

	// Perform motion plan towards the goal pose
    if(!this->performMotionPlan()){
        ROS_ERROR("Could not perform motion planning in SlerpControl object. Returning...");
        res.answer = false;
        return false;
    }

    // Send computed joint motion
    if(!this->sendJointTrajectory(this->computed_trajectory)){
        ROS_ERROR("Could not send computed trajectory from SlerpControl object. Returning...");
        res.answer = false;
        return false;
    }

    // At this point everything is completed, return true
    res.answer = true;
    return true;
}


// Initialize the things for motion planning. Is called by the callback
bool SlerpControl::initialize(geometry_msgs::Pose goal_pose, bool is_goal_relative){

    // Getting the current ee transform
    try {
		this->tf_listener.waitForTransform("/world", this->end_effector_name, ros::Time(0), ros::Duration(10.0) );
		this->tf_listener.lookupTransform("/world", this->end_effector_name, ros::Time(0), this->stamp_ee_transform);
    } catch (tf::TransformException ex){
      	ROS_ERROR("%s", ex.what());
      	ros::Duration(1.0).sleep();
        return false;
    }

    tf::Transform ee_transform(this->stamp_ee_transform.getRotation(), this->stamp_ee_transform.getOrigin());
    tf::transformTFToEigen(ee_transform, this->end_effector_state);

	// Print the current end-effector pose
	if(DEBUG) ROS_INFO_STREAM("Endeffector current Translation: " << this->end_effector_state.translation());
	if(DEBUG) ROS_INFO_STREAM("Endeffector current Rotation: " << this->end_effector_state.rotation());

	// Setting the start pose 
	this->startAff = this->end_effector_state;

	// Setting the goal pose
    tf::poseMsgToEigen(goal_pose, this->goalAff);

	// If the goal is relative, get the global goal pose by multiplying it with ee pose (end_effector_state)
	if(is_goal_relative){
		this->goalAff = this->end_effector_state * this->goalAff;
	}

    // Print the goal end-effector pose
    if(DEBUG) ROS_INFO_STREAM("Endeffector goal Translation: " << this->goalAff.translation());
	if(DEBUG) ROS_INFO_STREAM("Endeffector goal Rotation: " << this->goalAff.linear());

    return true;
}

// Performs motion planning for the end-effector towards goal
bool SlerpControl::performMotionPlan(){

    // Move group interface
    moveit::planning_interface::MoveGroupInterface group(this->group_name);

    /* If VISUAL is enabled */
    #ifdef VISUAL

    ros::spinOnce();        // May not be necessary

    // Getting the robot joint model
    const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup(this->group_name);

    // Visual tools
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();

    // Loading the remote control for visual tools and promting a message
    visual_tools.loadRemoteControl();
    visual_tools.trigger();

    #endif

	// Printing the planning group frame and the group ee frame
	if(DEBUG) ROS_INFO("MoveIt Group Reference frame: %s", group.getPlanningFrame().c_str());
	if(DEBUG) ROS_INFO("MoveIt Group End-effector frame: %s", group.getEndEffectorLink().c_str());

	// Calling the waypoint creator with start and goal poses
	std::vector<geometry_msgs::Pose> cart_waypoints;
	this->computeWaypointsFromPoses(this->startAff, this->goalAff, cart_waypoints);

	// Planning for the waypoints path
	moveit_msgs::RobotTrajectory trajectory;
	double fraction = group.computeCartesianPath(cart_waypoints, 0.01, 0.0, trajectory);
    this->computed_trajectory = trajectory.joint_trajectory;

	ROS_INFO("Plan (cartesian path) (%.2f%% acheived)", fraction * 100.0);

    // If complete path is not achieved return false, true otherwise
	if(fraction != 1.0) return false;

    std::cout << "FOUND COMPLETE PLAN FOR WAYPOINTS!!!" << std::endl;

    /* If VISUAL is enabled */
    #ifdef VISUAL

    ROS_INFO("Visualizing the computed plan as trajectory line.");
    visual_tools.publishAxisLabeled(cart_waypoints.back(), "goal pose");
    visual_tools.publishTrajectoryLine(trajectory, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute the motion on the robot.");

    #endif

    return true;
}

// Computes waypoints using SLERP from two poses
void SlerpControl::computeWaypointsFromPoses(const Eigen::Affine3d& start_pose, const Eigen::Affine3d& goal_pose, std::vector<geometry_msgs::Pose>& waypoints){
    
    // Compute waypoints as linear interpolation (SLERP for rotations) between the two poses
	Eigen::Affine3d wp_eigen;
	geometry_msgs::Pose current_wp;

	Eigen::Vector3d start_vec = start_pose.translation();
	Eigen::Vector3d goal_vec = goal_pose.translation();
	Eigen::Vector3d diff_vec = goal_vec - start_vec;

	Eigen::Quaterniond start_quat(start_pose.linear());
	Eigen::Quaterniond goal_quat(goal_pose.linear());
	Eigen::Quaterniond diff_quat;

    // Setting the number of wp according to diff_vec
    this->real_n_wp = std::floor(diff_vec.norm() * this->n_wp);
    if(DEBUG) ROS_INFO_STREAM("The norm of the diff_vec is " << diff_vec.norm() << 
        ", so the new number of waypoints is " << this->real_n_wp << ".");

	for(int j = 1; j <= this->n_wp; j++){
		wp_eigen.translation() = start_vec + (diff_vec / this->real_n_wp) * j;
		diff_quat = start_quat.slerp(double(j)/double(this->real_n_wp), goal_quat);
		wp_eigen.linear() = diff_quat.toRotationMatrix();
		tf::poseEigenToMsg (wp_eigen, current_wp);
		waypoints.push_back(current_wp);

        if(DEBUG){
            std::cout << "WAYPOINT NUMBER " << j << "." << std::endl;
		    std::cout << "WP R: " << wp_eigen.linear() << std::endl;
		    std::cout << "WP t: " << wp_eigen.translation() << std::endl;
        }
    }
}

// Sends trajectory to the joint_traj controller
bool SlerpControl::sendJointTrajectory(trajectory_msgs::JointTrajectory trajectory){
    
    // Waiting for the arm server to be ready
    if(!this->arm_client_ptr->waitForServer(ros::Duration(1,0))){
        ROS_ERROR("The arm client is taking too much to get ready. Returning...");
        return false;
    }

	// Send the message and wait for the result
	control_msgs::FollowJointTrajectoryGoal goalmsg;
	goalmsg.trajectory = trajectory;

    this->arm_client_ptr->sendGoal(goalmsg);

    if(!this->arm_client_ptr->waitForResult(ros::Duration(30, 0))){
        ROS_WARN("The arm client is taking too to complete goal execution. Is it a really long motion???");
    }

    return true;
}
