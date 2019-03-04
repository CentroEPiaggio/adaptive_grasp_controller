/* PANDA SOFTHAND CLIENT - Contains all necessary objects and functions to call the services to 
control Panda + SoftHand
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

#include "PandaSoftHandClient.h"

PandaSoftHandClient::PandaSoftHandClient(){

    // Nothing to do here
}

PandaSoftHandClient::PandaSoftHandClient(ros::NodeHandle& nh_){

    // Initializing object
    if(!this->initialize(nh_)){
        ROS_ERROR("The PandaSoftHandClient was not initialized successfully. Some servers are missing...");
        ros::shutdown();
    } 
}

PandaSoftHandClient::~PandaSoftHandClient(){
    
    // Nothing to do here yet
}

// Initializing function
bool PandaSoftHandClient::initialize(ros::NodeHandle& nh_){

    // Initializing node handle
    this->nh = nh_;

    // Initializing the service names (TODO: Change the hard coded names of the services and parse them from same yaml file of main_server)
    this->adaptive_service_name = "adaptive_control_service";
    this->hand_service_name = "hand_control_service";
    this->joint_service_name = "joint_control_service";
    this->pose_service_name = "pose_control_service";
    this->slerp_service_name = "slerp_control_service";

    // Initializing service clients after waiting
    if(!ros::service::waitForService(this->adaptive_service_name, ros::Duration(1.0))) return false;
    this->adaptive_client = this->nh.serviceClient<adaptive_grasp_controller::adaptive_control>(this->adaptive_service_name);

    if(!ros::service::waitForService(this->hand_service_name, ros::Duration(1.0))) return false;
    this->hand_client = this->nh.serviceClient<adaptive_grasp_controller::hand_control>(this->hand_service_name);

    if(!ros::service::waitForService(this->joint_service_name, ros::Duration(1.0))) return false;
    this->joint_client = this->nh.serviceClient<adaptive_grasp_controller::joint_control>(this->joint_service_name);

    if(!ros::service::waitForService(this->pose_service_name, ros::Duration(1.0))) return false;
    this->pose_client = this->nh.serviceClient<adaptive_grasp_controller::pose_control>(this->pose_service_name);

    if(!ros::service::waitForService(this->slerp_service_name, ros::Duration(1.0))) return false;
    this->slerp_client = this->nh.serviceClient<adaptive_grasp_controller::slerp_control>(this->slerp_service_name);

    // At this point initializing completed
    return true;
}

// Service call function for adaptive control
bool PandaSoftHandClient::call_adaptive_service(bool run){

    // Creating and filling up the request
    adaptive_grasp_controller::adaptive_control adaptive_control_srv;
    adaptive_control_srv.request.run = run;

    // Calling the service
    if(!this->adaptive_client.call(adaptive_control_srv)){
        ROS_ERROR("Failed to contact the adaptive control server. Returning...");
        return false;
    }

    return adaptive_control_srv.response.answer;
}

// Service call function for hand control
bool PandaSoftHandClient::call_hand_service(double goal_syn, double goal_duration){

    // Creating and filling up the request
    adaptive_grasp_controller::hand_control hand_control_srv;
    hand_control_srv.request.goal_syn = goal_syn;
    hand_control_srv.request.goal_duration = goal_duration;

    // Calling the service
    if(!this->hand_client.call(hand_control_srv)){
        ROS_ERROR("Failed to contact the hand control server. Returning...");
        return false;
    }

    return hand_control_srv.response.answer;
}

// Service call function for joint control
bool PandaSoftHandClient::call_joint_service(std::vector<double> joint_goal){

    // Creating and filling up the request
    adaptive_grasp_controller::joint_control joint_control_srv;
    joint_control_srv.request.joint_goal = joint_goal;

    // Calling the service
    if(!this->joint_client.call(joint_control_srv)){
        ROS_ERROR("Failed to contact the joint control server. Returning...");
        return false;
    }

    return joint_control_srv.response.answer;
}

// Service call function for pose control
bool PandaSoftHandClient::call_pose_service(geometry_msgs::Pose goal_pose, bool is_goal_relative){

    // Creating and filling up the request
    adaptive_grasp_controller::pose_control pose_control_srv;
    pose_control_srv.request.goal_pose = goal_pose;
    pose_control_srv.request.is_goal_relative = is_goal_relative;

    // Calling the service
    if(!this->pose_client.call(pose_control_srv)){
        ROS_ERROR("Failed to contact the pose control server. Returning...");
        return false;
    }

    return pose_control_srv.response.answer;
}

// Service call function for slerp control
bool PandaSoftHandClient::call_slerp_service(geometry_msgs::Pose goal_pose, bool is_goal_relative){

    // Creating and filling up the request
    adaptive_grasp_controller::slerp_control slerp_control_srv;
    slerp_control_srv.request.goal_pose = goal_pose;
    slerp_control_srv.request.is_goal_relative = is_goal_relative;

    // Calling the service
    if(!this->pose_client.call(slerp_control_srv)){
        ROS_ERROR("Failed to contact the slerp control server. Returning...");
        return false;
    }

    return slerp_control_srv.response.answer;
}