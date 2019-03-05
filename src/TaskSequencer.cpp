/* TASK SEQUENCER - Contains all recepies for grasping, handshake and other tasks
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

#include "parsing_utilities.h"
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

    // Initializing Panda SoftHand Client
    this->panda_softhand_client.initialize(this->nh);

    // Setting the task service names
    this->adaptive_task_service_name = "adaptive_task_service";
    this->grasp_task_service_name = "grasp_task_service";
    this->handshake_task_service_name = "handshake_task_service";

    // Advertising the services
    this->adaptive_task_server = this->nh.advertiseService(this->adaptive_task_service_name, &TaskSequencer::call_adaptive_grasp_task, this);

    // Spinning once
    ros::spinOnce();

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

    if(!ros::param::get("/task_sequencer/handover_pose", this->handover_pose)){
		ROS_WARN("The param 'handover_pose' not found in param server! Using default.");
		this->handover_pose.resize(6);
        std::fill(this->handover_pose.begin(), this->handover_pose.end(), 0.0);
		success = false;
	}

    // Converting the handover_pose vector to geometry_msgs Pose
    this->handover_T = this->convert_vector_to_pose(this->handover_pose);

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

// Callback for object pose subscriber
void TaskSequencer::get_object_pose(const geometry_msgs::Pose::ConstPtr &msg){

    // Saving the message
    this->object_pose_T = *msg;
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
    if(!this->panda_softhand_client.call_joint_service(this->home_joints)){
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
    if(!this->panda_softhand_client.call_pose_service(pre_grasp_pose, false)){
        ROS_ERROR("Could not go to the specified pre grasp pose.");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }

    // 3) Going to grasp pose
    if(!this->panda_softhand_client.call_slerp_service(grasp_pose, false)){
        ROS_ERROR("Could not go to the specified grasp pose.");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }

    // 4) Performing adaptive grasp
    if(!this->panda_softhand_client.call_adaptive_service(true)){
        ROS_ERROR("Could not perform the adaptive grasp.");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }

    // 5) Lift up to pregrasp pose
    if(!this->panda_softhand_client.call_slerp_service(pre_grasp_pose, false)){
        ROS_ERROR("Could not lift to the specified pose.");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }

    // 6) Going to handover pose
    if(!this->panda_softhand_client.call_pose_service(handover_T, false)){
        ROS_ERROR("Could not go to the specified handover pose.");
        res.success = false;
        res.message = "The service call_adaptive_grasp_task was NOT performed correctly!";
        return false;
    }

    // 7) TMP: open hand
    if(!this->panda_softhand_client.call_hand_service(0.0, 2.0)){
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