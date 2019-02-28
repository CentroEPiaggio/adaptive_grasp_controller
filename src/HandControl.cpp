/* HAND CONTROL - For closing SoftHand in to a desired position or at a desired velocity
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

#include "HandControl.h"

HandControl::HandControl(ros::NodeHandle& nh_, int n_wp_, std::string synergy_joint_name_,
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> hand_client_ptr_){
        
        // Initializing the class node
        this->nh = nh_;

        // Saving the synergy joint name
        this->synergy_joint_name = synergy_joint_name_;

        // Initializing the subscriber and waiting for a message
        this->joints_sub = this->nh.subscribe("/joint_states", 1, &HandControl::joints_callback, this);
        this->saved_jnt_msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", this->nh);

        // Setting the number of waypoints, closing time and the action client
        this->n_wp = n_wp_;
        this->hand_client_ptr = hand_client_ptr_;
}

HandControl::~HandControl(){
    // Nothing to do here yet
}

// This is the callback function of the hand control service
bool HandControl::call_hand_control(adaptive_grasp_controller::hand_control::Request &req, adaptive_grasp_controller::hand_control::Response &res){

    // Saving the callback msgs and checking limits (saturating)
    this->goal_value = req.goal_syn;
    if(this->goal_value < 0.0) this->goal_value = 0.0;
    if(this->goal_value > 1.0) this->goal_value = 1.0;

    this->goal_duration = req.goal_duration;
    if(this->goal_duration <= 0.0){
        ROS_ERROR("The given hand closing duration is not positive. Returning...");
        res.answer = false;
        return false;
    }

    // Setting things by saving joint states and synergy value, ecc.
    if(!this->initialize()){
        ROS_ERROR("Could not initialize HandControl object. Returning...");
        res.answer = false;
        return false;
    }

    // Computing the joint trajectory from present_syn to goal_value (or with wanted vel until complete closure)
    this->computeTrajectory(this->present_syn, this->goal_value, this->goal_duration);

    // Sending the trajectory to hand
    if(!this->sendHandTrajectory(this->computed_trajectory)){
        ROS_ERROR("Could not send computed trajectory from HandControl object. Returning...");
        res.answer = false;
        return false;
    }

    // At this point all is fine, return true
    res.answer = true;
    return true;
}

// The callback function for the joint states subscriber
void HandControl::joints_callback(const sensor_msgs::JointState::ConstPtr &jnt_msg){

    // Saving the message
    this->saved_jnt_msg = jnt_msg;
    // if(DEBUG) ROS_WARN_STREAM("Inside the joint state subscriber calback.");
}

// Initialize the things for setting up things. It is called by the callback
bool HandControl::initialize(){

    // Spinning once for message
    ros::spinOnce();

    // Getting the synergy value from the latest saved joint message
    int index = std::find(saved_jnt_msg->name.begin(),saved_jnt_msg->name.end(),
		this->synergy_joint_name) - saved_jnt_msg->name.begin();

    // Getting the synergy value, making sure that it exists
    try {
        this->present_syn = saved_jnt_msg->position[index];
    } catch (const std::exception& e) {
        std::cout << e.what();
        ROS_ERROR_STREAM("Could not find the joint " << this->synergy_joint_name << " in the joint states (saved msg). Returning...");
        return false;
    }

    // Cout and return
    if(DEBUG) ROS_INFO_STREAM("The present synergy value is " << this->present_syn << ".");
    return true;
}

// Performs computation of points towards goal
void HandControl::computeTrajectory(double present_syn, double goal_syn, double time){

    // Computing the real number of waypoints with proportion
    int real_n_wp = std::floor(std::abs(goal_syn - present_syn) * this->n_wp);

    // Objects needed for generating trajectory
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names.push_back(this->synergy_joint_name);
    traj.header.stamp = ros::Time::now();
    trajectory_msgs::JointTrajectoryPoint point;

    // Does hand need to be opened or closed?
    bool hand_closed = (goal_syn > present_syn);

    // Generating waypoints
    for(int i = 1; i <= real_n_wp; i++){

        //Computing the position according to hand closing or opening
        double position;
        if(hand_closed){
            position = present_syn + ( (double (i) / double (real_n_wp)) * (goal_syn - present_syn) );
        } else {
            position = present_syn - ( (double (i) / double (real_n_wp)) * (goal_syn - present_syn) );
        }

        // Computing the time for the waypoint
        double time_wp = (double (i) / double (real_n_wp)) * time;

        // Debug cout
        if(DEBUG) ROS_INFO_STREAM("The " << i << "th computed position is " << position << " at time " << time_wp << ".");

        // Pushing back position and time in point
        point.positions.clear();
        point.positions.push_back(position);
        point.time_from_start = ros::Duration(this->millisecond * 1000 * time_wp);

        // Pushing back point into traj
        traj.points.push_back(point);
    }

    this->computed_trajectory = traj;
}

// Sends trajectory to the hand joint trajectory controller
bool HandControl::sendHandTrajectory(trajectory_msgs::JointTrajectory trajectory){

    // Waiting for the hand server to be ready
    if(!this->hand_client_ptr->waitForServer(ros::Duration(1,0))){
        ROS_ERROR("The hand client is taking too much to get ready. Returning...");
        return false;
    }

	// Send the message and wait for the result
	control_msgs::FollowJointTrajectoryGoal goalmsg;
	goalmsg.trajectory = trajectory;

    this->hand_client_ptr->sendGoal(goalmsg);

    if(!this->hand_client_ptr->waitForResult(ros::Duration(20, 0))){
        ROS_ERROR("The hand client is taking too to complete goal execution. Returning...");
        return false;
    }

    return true;
}