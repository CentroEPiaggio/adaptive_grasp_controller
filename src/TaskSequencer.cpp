/* TASK SEQUENCER - Contains all recepies for grasping, handshake and other tasks
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

#include "sensor_msgs/JointState.h"

#include "utils/parsing_utilities.h"
#include "TaskSequencer.h"

TaskSequencer::TaskSequencer(ros::NodeHandle& nh_){
    
    // Initializing Node Handle
    this->nh = nh_;

    // Parsing the task params
    if(!this->parse_task_params()){
        ROS_ERROR("The parsing of task parameters went wrong. Be careful, using default values...");
    }

    // Initializing the object subscriber and waiting (TODO: parse the topic name)
    this->object_topic_name = "object_pose";
    this->object_sub = this->nh.subscribe(this->object_topic_name, 1, &TaskSequencer::get_object_pose, this);
    ros::topic::waitForMessage<geometry_msgs::Pose>(this->object_topic_name, ros::Duration(2.0));

    // Initializing the franka_state_sub subscriber and waiting
    this->franka_state_sub = this->nh.subscribe(this->robot_name + this->franka_state_topic_name, 1, &TaskSequencer::get_franka_state, this);
    ros::topic::waitForMessage<franka_msgs::FrankaState>(this->franka_state_topic_name, ros::Duration(2.0));

    // Initializing the tau_ext norm and franka recovery publishers
    this->pub_franka_recovery = this->nh.advertise<franka_control::ErrorRecoveryActionGoal>(this->robot_name + "/franka_control/error_recovery/goal", 1);
    this->pub_tau_ext_norm = this->nh.advertise<std_msgs::Float64>("tau_ext_norm", 1);

    // Initializing Panda SoftHand Client
    this->panda_softhand_client.initialize(this->nh);

    // Setting the task service names
    this->adaptive_task_service_name = "adaptive_task_service";
    this->grasp_task_service_name = "grasp_task_service";
    this->handshake_task_service_name = "handshake_task_service";
    this->set_object_service_name = "set_object_service";

    // Advertising the services
    this->adaptive_task_server = this->nh.advertiseService(this->adaptive_task_service_name, &TaskSequencer::call_adaptive_grasp_task, this);
    this->grasp_task_server = this->nh.advertiseService(this->grasp_task_service_name, &TaskSequencer::call_simple_grasp_task, this);
    this->handshake_task_server = this->nh.advertiseService(this->handshake_task_service_name, &TaskSequencer::call_handshake_task, this);
    this->end_handshake_server = this->nh.advertiseService(this->handshake_end_srv_name, &TaskSequencer::call_handshake_end, this);
    this->set_object_server = this->nh.advertiseService(this->set_object_service_name, &TaskSequencer::call_set_object, this);

    // Setting other utils
    this->handshake_ended = false;

    // Setting handshake joint config if requested
    if(this->handshake_config_rec) this->set_handshake_config();

    // Spinning once
    ros::spinOnce();

}

TaskSequencer::~TaskSequencer(){
    
    // Nothing to do here yet
}

// Parameters parsing
bool TaskSequencer::parse_task_params(){
    bool success = true;

    if(!ros::param::get("/task_sequencer/robot_name", this->robot_name)){
		ROS_WARN("The param 'robot_name' not found in param server! Using default.");
		this->robot_name = "panda_arm";
		success = false;
	}

    if(!ros::param::get("/task_sequencer/robot_joints_name", this->robot_joints_name)){
		ROS_WARN("The param 'robot_joints_name' not found in param server! Using default.");
		this->robot_joints_name = "panda_joint";
		success = false;
	}

    if(!ros::param::get("/task_sequencer/pos_controller", this->pos_controller)){
		ROS_WARN("The param 'pos_controller' not found in param server! Using default.");
		this->pos_controller = "position_joint_trajectory_controller";
		success = false;
	}

    if(!ros::param::get("/task_sequencer/imp_controller", this->imp_controller)){
		ROS_WARN("The param 'imp_controller' not found in param server! Using default.");
		this->imp_controller = "cartesian_impedance_controller_softbots_stiff_matrix";
		success = false;
	}

    if(!ros::param::get("/task_sequencer/handshake_ekf_srv_name", this->handshake_ekf_srv_name)){
		ROS_WARN("The param 'handshake_ekf_srv_name' not found in param server! Using default.");
		this->handshake_ekf_srv_name = "call_handshake_ekf";
		success = false;
	}

    if(!ros::param::get("/task_sequencer/handshake_cont_srv_name", this->handshake_cont_srv_name)){
		ROS_WARN("The param 'handshake_cont_srv_name' not found in param server! Using default.");
		this->handshake_cont_srv_name = "call_handshake_control";
		success = false;
	}

    if(!ros::param::get("/task_sequencer/handshake_end_srv_name", this->handshake_end_srv_name)){
		ROS_WARN("The param 'handshake_end_srv_name' not found in param server! Using default.");
		this->handshake_end_srv_name = "handshake_ending";
		success = false;
	}

	if(!ros::param::get("/task_sequencer/home_joints", this->home_joints)){
		ROS_WARN("The param 'home_joints' not found in param server! Using default.");
		this->home_joints = {-0.035, -0.109, -0.048, -1.888, 0.075, 1.797, -0.110};
		success = false;
	}

    if(!ros::param::get("/task_sequencer/grasp_transform", this->grasp_transform)){
		ROS_WARN("The param 'grasp_transform' not found in param server! Using default.");
		this->grasp_transform.resize(6);
        std::fill(this->grasp_transform.begin(), this->grasp_transform.end(), 0.0);
		success = false;
	}

    // Converting the grasp_transform vector to geometry_msgs Pose
    this->grasp_T = this->convert_vector_to_pose(this->grasp_transform);

    if(!ros::param::get("/task_sequencer/pre_grasp_transform", this->pre_grasp_transform)){
		ROS_WARN("The param 'pre_grasp_transform' not found in param server! Using default.");
		this->pre_grasp_transform.resize(6);
        std::fill(this->pre_grasp_transform.begin(), this->pre_grasp_transform.end(), 0.0);
		success = false;
	}

    // Converting the pre_grasp_transform vector to geometry_msgs Pose
    this->pre_grasp_T = this->convert_vector_to_pose(this->pre_grasp_transform);

    if(!ros::param::get("/task_sequencer/handover_joints", this->handover_joints)){
		ROS_WARN("The param 'handover_joints' not found in param server! Using default.");
		this->handover_joints = {-0.101, 0.161, 0.159, -1.651, 2.023, 2.419, -0.006};
		success = false;
	}

    if(!ros::param::get("/task_sequencer/handover_thresh", this->handover_thresh)){
		ROS_WARN("The param 'handover_thresh' not found in param server! Using default.");
		this->handover_thresh = 4.5;
		success = false;
	}

    if(!ros::param::get("/task_sequencer/handshake_joints", this->handshake_joints)){
		ROS_WARN("The param 'handshake_joints' not found in param server! Using default.");
		this->handshake_joints = {0.062, 0.420, -0.362, -1.885, 1.489, 1.261, -0.031};
		success = false;
	}

    if(!ros::param::get("/task_sequencer/handshake_config_rec", this->handshake_config_rec)){
		ROS_WARN("The param 'handshake_config_rec' not found in param server! Using default.");
		this->handshake_config_rec = false;
		success = false;
	}

    // Getting the XmlRpc value and parsing
    if(!this->nh.getParam("/task_sequencer", this->task_seq_params)){
        ROS_ERROR("Could not get the XmlRpc value.");
        success = false;
    }

    if(!parseParameter(this->task_seq_params, this->grasp_poses_map, "grasp_poses_map")){
        ROS_ERROR("Could not parse the grasp poses map.");
        success = false;
    }

    if(DEBUG){
        ROS_INFO_STREAM("The grasp poses map is");
        for(auto it : this->grasp_poses_map){
            std::cout << it.first << " : [ ";
            for(auto vec_it : it.second){
                std::cout << vec_it << " ";
            } 
            std::cout << "]" << std::endl;     
        }
    }

    return success;
}

// Convert xyzrpy vector to geometry_msgs Pose
geometry_msgs::Pose TaskSequencer::convert_vector_to_pose(std::vector<double> input_vec){
    
    // Creating temporary variables
    geometry_msgs::Pose output_pose;
    Eigen::Affine3d output_affine;

    // Getting translation and rotation
    Eigen::Vector3d translation(input_vec[0], input_vec[1], input_vec[2]);
    output_affine.translation() = translation;
    Eigen::Matrix3d rotation = Eigen::Matrix3d(Eigen::AngleAxisd(input_vec[5], Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(input_vec[4], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(input_vec[3], Eigen::Vector3d::UnitX()));
    output_affine.linear() = rotation;    
    
    // Converting to geometry_msgs and returning
    tf::poseEigenToMsg(output_affine, output_pose);
    return output_pose;
}

// To switch the controllers
bool TaskSequencer::switch_controllers(std::string robot_name, std::string from_controller, std::string to_controller){

    // Temporary bool to be returned
    bool success = false;

    // Clearing the switch message
    this->switch_controller.request.start_controllers.clear();
    this->switch_controller.request.stop_controllers.clear();

    // Filling up the switch message
    this->switch_controller.request.start_controllers.push_back(to_controller);
    this->switch_controller.request.stop_controllers.push_back(from_controller);
    this->switch_controller.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;

    // Swithching controller by calling the service
    return ros::service::call<controller_manager_msgs::SwitchController>(robot_name + this->switch_service_name, this->switch_controller);
}

// Callback for object pose subscriber
void TaskSequencer::get_object_pose(const geometry_msgs::Pose::ConstPtr &msg){

    // Saving the message
    this->object_pose_T = *msg;
}

// Callback for franka state subscriber
void TaskSequencer::get_franka_state(const franka_msgs::FrankaState::ConstPtr &msg){

    // Saving the message
    this->latest_franka_state = *msg;

    // Checking for libfranka errors
    if(msg->robot_mode != 2 && msg->robot_mode != 5){       // The robot state is not "automatic" or "manual guiding"
        this->franka_ok = false;
        if(DEBUG && false) ROS_ERROR("Something happened to the robot!");
    }else if(msg->robot_mode == 2){
        this->franka_ok = true;
        if(DEBUG && false) ROS_WARN("Now Franka is in a good mood!");
    }

    // Getting the tau ext
    if(DEBUG && false){
        std::cout << "The latest tau ext vector is \n [ ";
        for(auto it : this->latest_franka_state.tau_ext_hat_filtered)  std::cout << it << " ";
        std::cout << "]" << std::endl;
    }

    // Computing the norm
    this->tau_ext_norm = 0.0;
    for(auto it : this->latest_franka_state.tau_ext_hat_filtered){
        this->tau_ext_norm += std::pow(it, 2);
    }
    this->tau_ext_norm = std::sqrt(this->tau_ext_norm);

    // Publishing norm
    std_msgs::Float64 norm_msg; norm_msg.data = this->tau_ext_norm;
    this->pub_tau_ext_norm.publish(norm_msg);
    
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

    // 1) Going to home configuration
    if(!this->panda_softhand_client.call_joint_service(this->home_joints) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified home joint configuration.");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }

    // Computing the grasp and pregrasp pose and converting to geometry_msgs Pose
    Eigen::Affine3d object_pose_aff; tf::poseMsgToEigen(this->object_pose_T, object_pose_aff);
    Eigen::Affine3d grasp_transform_aff; tf::poseMsgToEigen(this->grasp_T, grasp_transform_aff);
    Eigen::Affine3d pre_grasp_transform_aff; tf::poseMsgToEigen(this->pre_grasp_T, pre_grasp_transform_aff);

    geometry_msgs::Pose pre_grasp_pose; geometry_msgs::Pose grasp_pose;
    tf::poseEigenToMsg(object_pose_aff * grasp_transform_aff * pre_grasp_transform_aff, pre_grasp_pose);
    tf::poseEigenToMsg(object_pose_aff * grasp_transform_aff, grasp_pose);

    // 2) Going to pregrasp pose
    if(!this->panda_softhand_client.call_pose_service(pre_grasp_pose, false) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified pre grasp pose.");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }

    // 3) Going to grasp pose
    if(!this->panda_softhand_client.call_slerp_service(grasp_pose, false) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified grasp pose.");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }

    // 4) Performing adaptive grasp
    if(!this->panda_softhand_client.call_adaptive_service(true) || !this->franka_ok){
        ROS_ERROR("Could not perform the adaptive grasp.");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }

    // 5) Lift up to pregrasp pose
    if(!this->panda_softhand_client.call_slerp_service(pre_grasp_pose, false) || !this->franka_ok){
        ROS_ERROR("Could not lift to the specified pose.");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }

    // 6) Going to handover joint config
    if(!this->panda_softhand_client.call_joint_service(this->handover_joints) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified handover joint config.");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }

    // 7) Waiting for threshold or for some time
    sleep(1);       // Sleeping for a second to avoid robot stopping peaks
    bool hand_open = false; ros::Time init_time = ros::Time::now(); ros::Time now_time;
    while(!hand_open){
        now_time = ros::Time::now();
        usleep(500);                         // Don't know why, but the threshold works with this sleeping
        if(this->tau_ext_norm > this->handover_thresh){
            hand_open = true;
            if(DEBUG) ROS_WARN_STREAM("Opening condition reached!" << " SOMEONE PULLED!");
            if(DEBUG) ROS_WARN_STREAM("The tau_ext_norm is " << this->tau_ext_norm << " and the threshold is " << this->handover_thresh << ".");
        }
        if((now_time - init_time) > ros::Duration(10, 0)){
            hand_open = true;
            if(DEBUG) ROS_WARN_STREAM("Opening condition reached!" << " TIMEOUT!");
            if(DEBUG) ROS_WARN_STREAM("The initial time was " << init_time << ", now it is " << now_time 
                << ", the difference is " << (now_time - init_time) << " and the timeout thresh is " << ros::Duration(10, 0));
        }
    }

    // 8) Opening hand 
    if(!this->panda_softhand_client.call_hand_service(0.0, 2.0) || !this->franka_ok){
        ROS_ERROR("Could not open the hand.");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }

    // Now, everything finished well
    res.success = true;
    res.message = "The service call_adaptive_grasp_task was correctly performed!";
    return true;
}

// Callback for simple grasp task service
bool TaskSequencer::call_simple_grasp_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    
    // Checking the request for correctness
    if(!req.data){
        ROS_WARN("Did you really want to call the simple grasp task service with data = false?");
        res.success = true;
        res.message = "The service call_simple_grasp_task done correctly with false request!";
        return true;
    }

    // 1) Going to home configuration
    if(!this->panda_softhand_client.call_joint_service(this->home_joints) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified home joint configuration.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    }

    // Computing the grasp and pregrasp pose and converting to geometry_msgs Pose
    Eigen::Affine3d object_pose_aff; tf::poseMsgToEigen(this->object_pose_T, object_pose_aff);
    Eigen::Affine3d grasp_transform_aff; tf::poseMsgToEigen(this->grasp_T, grasp_transform_aff);
    Eigen::Affine3d pre_grasp_transform_aff; tf::poseMsgToEigen(this->pre_grasp_T, pre_grasp_transform_aff);

    geometry_msgs::Pose pre_grasp_pose; geometry_msgs::Pose grasp_pose;
    tf::poseEigenToMsg(object_pose_aff * grasp_transform_aff * pre_grasp_transform_aff, pre_grasp_pose);
    tf::poseEigenToMsg(object_pose_aff * grasp_transform_aff, grasp_pose);

    // 2) Going to pregrasp pose
    if(!this->panda_softhand_client.call_pose_service(pre_grasp_pose, false) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified pre grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    }

    // 3) Going to grasp pose
    if(!this->panda_softhand_client.call_slerp_service(grasp_pose, false) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    }

    // 4) Performing simple grasp
    if(!this->panda_softhand_client.call_hand_service(1.0, 2.0) || !this->franka_ok){
        ROS_ERROR("Could not perform the simple grasp.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    }

    // 5) Lift up to pregrasp pose
    if(!this->panda_softhand_client.call_slerp_service(pre_grasp_pose, false) || !this->franka_ok){
        ROS_ERROR("Could not lift to the specified pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    }

    // 6) Going to handover joint config
    if(!this->panda_softhand_client.call_joint_service(this->handover_joints) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified handover joint config.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    }

    // 7) Waiting for threshold or for some time
    sleep(1);       // Sleeping for a second to avoid robot stopping peaks
    bool hand_open = false; ros::Time init_time = ros::Time::now(); ros::Time now_time;
    while(!hand_open){
        now_time = ros::Time::now();
        usleep(500);                         // Don't know why, but the threshold works with this sleeping
        if(this->tau_ext_norm > this->handover_thresh){
            hand_open = true;
            if(DEBUG) ROS_WARN_STREAM("Opening condition reached!" << " SOMEONE PULLED!");
            if(DEBUG) ROS_WARN_STREAM("The tau_ext_norm is " << this->tau_ext_norm << " and the threshold is " << this->handover_thresh << ".");
        }
        if((now_time - init_time) > ros::Duration(10, 0)){
            hand_open = true;
            if(DEBUG) ROS_WARN_STREAM("Opening condition reached!" << " TIMEOUT!");
            if(DEBUG) ROS_WARN_STREAM("The initial time was " << init_time << ", now it is " << now_time 
                << ", the difference is " << (now_time - init_time) << " and the timeout thresh is " << ros::Duration(10, 0));
        }
    }

    // 8) Opening hand 
    if(!this->panda_softhand_client.call_hand_service(0.0, 2.0) || !this->franka_ok){
        ROS_ERROR("Could not open the hand.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    }

    // Now, everything finished well
    res.success = true;
    res.message = "The service call_simple_grasp_task was correctly performed!";
    return true;
}

// Callback for handshake task service
bool TaskSequencer::call_handshake_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

    // Checking the request for correctness
    if(!req.data){
        ROS_WARN("Did you really want to call the handshake task service with data = false?");
        res.success = true;
        res.message = "The service call_handshake_task done correctly with false request!";
        return true;
    }

    // 1) Going to handshake configuration
    if(!this->panda_softhand_client.call_joint_service(this->handshake_joints) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified handshake joint configuration.");
        res.success = false;
        res.message = "The service call_handshake_task was NOT performed correctly!";
        return false;
    }

    // 2) Swithching to impedance controller
    if(!this->switch_controllers(this->robot_name, this->pos_controller, this->imp_controller) || !this->franka_ok){
        ROS_ERROR_STREAM("Could not switch to the impedance controller " 
            << this->imp_controller << " from " << this->pos_controller << ". Are these controllers loaded?");
        res.success = false;
        res.message = "The service call_handshake_task was NOT performed correctly!";
        return false;
    }

    ROS_WARN_STREAM("Switched controllers: from " << this->pos_controller << " to " << this->imp_controller << ".");

    // 3) Calling the handshake filter and control services
    std_srvs::SetBool bool_srv; bool_srv.request.data = true;
    if(!ros::service::call<std_srvs::SetBool>(this->handshake_ekf_srv_name, bool_srv)){
        ROS_ERROR("Could not call the handshake ekf service.");
        res.success = false;
        res.message = "The service call_handshake_task was NOT performed correctly!";
        return false;
    }
    if(!ros::service::call<std_srvs::SetBool>(this->handshake_cont_srv_name, bool_srv)){
        ROS_ERROR("Could not call the handshake control service.");
        res.success = false;
        res.message = "The service call_handshake_task was NOT performed correctly!";
        return false;
    }

    ROS_INFO("Called the ekf and the control of handshake!");

    // 4) Wait for an event on a topic
    while(!this->handshake_ended){
        // Sleep for some time
        usleep(50);
    }

    // 3) Calling the handshake filter and control services
    bool_srv.request.data = false;
    if(!ros::service::call<std_srvs::SetBool>(this->handshake_ekf_srv_name, bool_srv)){
        ROS_WARN("Could not stop the handshake ekf service.");
    }
    if(!ros::service::call<std_srvs::SetBool>(this->handshake_cont_srv_name, bool_srv)){
        ROS_WARN("Could not stop the handshake control service.");
    }

    // 4) Swithching to position controller
    if(!this->switch_controllers(this->robot_name, this->imp_controller, this->pos_controller) || !this->franka_ok){
        ROS_ERROR_STREAM("Could not switch to the position controller " 
            << this->pos_controller << " from " << this->imp_controller << ". Are these controllers loaded?");
        res.success = false;
        res.message = "The service call_handshake_task was NOT performed correctly!";
        return false;
    }

    ROS_WARN_STREAM("Switched controllers: from " << this->imp_controller << " to " << this->pos_controller << ".");

    // Resetting handshake ending
    this->handshake_ended = false;

    // At this point everything finished correctly
    res.success = true;
    res.message = "The service call_handshake_task was performed correctly!";
    return true;
}

// Callback for handshake ending service
bool TaskSequencer::call_handshake_end(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

    // Stopping the handshake
    this->handshake_ended = req.data;

    // Setting the response
    res.message = "Set the handshake_ending according to request.";
    res.success = true;
    return true;
}

// Callback for handshake task service
bool TaskSequencer::call_set_object(adaptive_grasp_controller::set_object::Request &req, adaptive_grasp_controller::set_object::Response &res){

    // Checking if the parsed map contains the requested object
    auto search = this->grasp_poses_map.find(req.object_name);
    if(search == this->grasp_poses_map.end()){
        ROS_WARN_STREAM("The object " << req.object_name << " is not present in my memory; using the previously used one or default... Did you spell it correctly? Is it in the yaml?");
        res.result = false;
        return res.result;
    }

    // Setting the grasp pose as requested
    this->grasp_transform = this->grasp_poses_map.at(req.object_name);

    // Converting the grasp_transform vector to geometry_msgs Pose
    this->grasp_T = this->convert_vector_to_pose(this->grasp_transform);

    // Now, everything is ok
    ROS_INFO_STREAM("Grasp pose changed. Object set to " << req.object_name << ".");
    res.result = true;
    return res.result;
}

// Function for setting the present joint config as handshake config
bool TaskSequencer::set_handshake_config(){

    // Waiting for joint states and saving
    sensor_msgs::JointState::ConstPtr joint_state = 
		ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(1.0));

    // Finding the robot joints and filling up the handshake config vector    
    int joint_index;
    int num_joints = this->handshake_joints.size();
    this->handshake_joints.clear();

    for(int i = 0; i < num_joints; i++){
        
        try{
            joint_index = std::find(joint_state->name.begin(), joint_state->name.end(), this->robot_joints_name + std::to_string(i+1)) - joint_state->name.begin();
        } catch (const std::exception &ex) {
            ROS_ERROR_STREAM("Could not find the joint: " << this->robot_joints_name + std::to_string(i+1));
            std::cerr << ex.what();
            return false;
        }
        this->handshake_joints.push_back(double (joint_state->position[joint_index]));
    }

    // Now all is well
    ROS_INFO_STREAM("The handshake joint config has been set to ");
    std::cout << "[ ";
    for(auto vec_it : this->handshake_joints){
        std::cout << vec_it << " ";
    } 
    std::cout << "]" << std::endl;
    
}