/* POSE CONTROL - Uses moveit movegroupinterface to plan towards a pose
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

#include "ros/ros.h"
#include <cstdlib>
#include <sstream>
#include <string>

#include "PoseControl.h"

#include <moveit_visual_tools/moveit_visual_tools.h>

PoseControl::PoseControl(ros::NodeHandle& nh_, std::string group_name_, std::string end_effector_name_,
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> arm_client_ptr_){
        
        ROS_INFO("Starting to create PoseControl object");

        // Initializing node handle
        this->nh = nh_;

        // Initializing names
        this->end_effector_name = end_effector_name_;
        this->group_name = group_name_;

        // Initializing the arm client
        this->arm_client_ptr = arm_client_ptr_;

        ROS_INFO("Finished creating PoseControl object");
}

PoseControl::~PoseControl(){
    
    // Nothing to do here yet
}

// This is the callback function of the pose control service
bool PoseControl::call_pose_control(adaptive_grasp_controller::pose_control::Request &req, adaptive_grasp_controller::pose_control::Response &res){

    // Setting up things
    if(!this->initialize(req.goal_pose, req.is_goal_relative)){
        ROS_ERROR("Could not initialize PoseControl object. Returning...");
        res.answer = false;
        return false;
    }

	// Perform motion plan towards the goal pose
    if(!this->performMotionPlan()){
        ROS_ERROR("Could not perform motion planning in PoseControl object. Returning...");
        res.answer = false;
        return false;
    }

    // Send computed joint motion
    if(!this->sendJointTrajectory(this->computed_trajectory)){
        ROS_ERROR("Could not send computed trajectory from PoseControl object. Returning...");
        res.answer = false;
        return false;
    }

    // At this point everything is completed, return true
    res.answer = true;
    return true;
}


// Initialize the things for motion planning. Is called by the callback
bool PoseControl::initialize(geometry_msgs::Pose goal_pose, bool is_goal_relative){

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
	if(DEBUG) ROS_INFO_STREAM("Endeffector current Translation: \n" << this->end_effector_state.translation());
	if(DEBUG) ROS_INFO_STREAM("Endeffector current Rotation: \n" << this->end_effector_state.rotation());

	// Setting the goal pose
    tf::poseMsgToEigen(goal_pose, this->goalPoseAff);

	// If the goal is relative, get the global goal pose by multiplying it with ee pose (end_effector_state)
	if(is_goal_relative){
		this->goalPoseAff = this->end_effector_state * this->goalPoseAff;
	}

    // Reconvert to geometry_msgs Pose
    tf::poseEigenToMsg(this->goalPoseAff, this->goalPose);

    // Print the goal end-effector pose
    if(DEBUG) ROS_INFO_STREAM("Endeffector goal Translation: \n" << this->goalPoseAff.translation());
	if(DEBUG) ROS_INFO_STREAM("Endeffector goal Rotation: \n" << this->goalPoseAff.linear());

    return true;
}

// Performs motion planning for the end-effector towards goal
bool PoseControl::performMotionPlan(){

    // Move group interface
    moveit::planning_interface::MoveGroupInterface group(this->group_name);

    // Getting the robot joint model
    ros::spinOnce();                    // May not be necessary
    const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup(this->group_name);

    /* If VISUAL is enabled */
    #ifdef VISUAL

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

    // Setting the pose target of the move group
    group.setPoseTarget(this->goalPose);

    if(DEBUG) ROS_INFO("Done setting the target pose in MoveIt Group.");

    // Planning to Pose
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Motion Plan towards goal pose %s.", success ? "SUCCEDED" : "FAILED");

    // If complete path is not achieved return false, true otherwise
	if(!success) return false;

    /* If VISUAL is enabled */
    #ifdef VISUAL

    ROS_INFO("Visualizing the computed plan as trajectory line.");
    visual_tools.publishAxisLabeled(this->goalPose, "goal pose");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute the motion on the robot.");

    #endif

    // Saving the computed trajectory and returning true
    this->computed_trajectory = my_plan.trajectory_.joint_trajectory;
    return true;
}

// Sends trajectory to the joint_traj controller
bool PoseControl::sendJointTrajectory(trajectory_msgs::JointTrajectory trajectory){
    
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
