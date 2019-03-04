/* TASK SEQUENCER - Contains all recepies for grasping, handshake and other tasks
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

#include "TaskSequencer.h"

TaskSequencer::TaskSequencer(ros::NodeHandle& nh_){
    
    // Initializing Node Handle
    this->nh = nh_;

    // Initializing Panda SoftHand Client
    this->panda_softhand_client.initialize(this->nh);

    // Setting the task service names
    this->adaptive_task_service_name = "adaptive_task_service";
    this->grasp_task_service_name = "grasp_task_service";
    this->handshake_task_service_name = "handshake_task_service";

    // Parsing the task params
    if(!this->parse_task_params()){
        ROS_ERROR("The parsing of task parameters went wrong. Returning...");
        ros::shutdown();
    }

    // Advertising the services
    this->nh.advertiseService(this->adaptive_task_service_name, &TaskSequencer::call_adaptive_grasp_task, this);

}

TaskSequencer::~TaskSequencer(){
    
    // Nothing to do here yet
}

// Parameters parsing
bool TaskSequencer::parse_task_params(){
    bool success = true;

	if(!ros::param::get("/task_sequencer/home_joints", this->home_joints)){
		ROS_WARN("The param 'home_joints' not found in param server! Using default.");
		this->home_joints.resize(7);
        std::fill(this->home_joints.begin(), this->home_joints.end(), 0.0);
		success = false;
	}

    return success;
}

// Callback for adaptive grasp task service
bool TaskSequencer::call_adaptive_grasp_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    
    // Checking the request for correctness
    if(!req.data){
        ROS_WARN("Did you really want to call the adaptive grasp task service with data = false?");
        res.success = true;
        res.message = "The service call_adaptive_grasp_task done correctly with false request!";
        return true;
    }
    // Going to home configuration
    if(!this->panda_softhand_client.call_joint_service(this->home_joints)){
        ROS_ERROR("Could not go to the specified home joint configuration.");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }

    res.success = true;
    res.message = "The service call_adaptive_grasp_task was correctly performed!";
    return true;
}