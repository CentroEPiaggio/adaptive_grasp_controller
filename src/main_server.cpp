/* MAIN SERVERS - Creates all necessary service servers to command Panda and SoftHand
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

// Basic Includes
#include "ros/ros.h"

// Object Includes
#include "HandControl.h"
#include "SlerpControl.h"
#include "PoseControl.h"
#include "AdaptiveControl.h"
#include "JointControl.h"

/**********************************************
ROS NODE MAIN SERVICE SERVERS 
**********************************************/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "adaptive_grasp_controller");

    ros::NodeHandle nh_;

    ROS_INFO("Creating the arm client pointer");

    std::string arm_jt_topic = "/panda_arm/position_joint_trajectory_controller/follow_joint_trajectory/";
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> arm_client_ptr_(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(arm_jt_topic, true));

    ROS_INFO("Creating the hand client pointer");

    std::string hand_jt_topic = "/right_hand/joint_trajectory_controller/follow_joint_trajectory/";
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> hand_client_ptr_(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(hand_jt_topic, true));

    ROS_INFO("Creating the slerp control object");

    SlerpControl slerp_control_obj(nh_, "panda_arm", "right_hand_ee_link", 60, arm_client_ptr_);

    ROS_INFO("Creating the hand control object");

    HandControl hand_control_obj(nh_, 20, "right_hand_synergy_joint", hand_client_ptr_);

    ROS_INFO("Creating the pose control object");

    PoseControl pose_control_obj(nh_, "panda_arm", "right_hand_ee_link", arm_client_ptr_);

    ROS_INFO("Creating the adaptive control object");

    AdaptiveControl adaptive_control_obj(nh_, arm_client_ptr_, hand_client_ptr_);

    ROS_INFO("Creating the joint control object");

    JointControl joint_control_obj(nh_, "panda_arm", arm_client_ptr_);
    
    ROS_INFO("Advertising the services");

    ros::ServiceServer slerp_service = nh_.advertiseService("slerp_control_service", &SlerpControl::call_slerp_control, &slerp_control_obj);
    ros::ServiceServer hand_service = nh_.advertiseService("hand_control_service", &HandControl::call_hand_control, &hand_control_obj);
    ros::ServiceServer pose_service = nh_.advertiseService("pose_control_service", &PoseControl::call_pose_control, &pose_control_obj);
    ros::ServiceServer adaptive_service = nh_.advertiseService("adaptive_control_service", &AdaptiveControl::call_adaptive_grasp_controller, &adaptive_control_obj);
    ros::ServiceServer joint_service = nh_.advertiseService("joint_control_service", &JointControl::call_joint_control, &joint_control_obj);

    ROS_INFO("The main service server is running. Running as fast as possible!");

    // ROS Async spinner (necessary for processing callbacks inside the service callbacks)
    ros::AsyncSpinner spinner(4);
    spinner.start();

    while(ros::ok()){
        // Nothing to do here
    }

    spinner.stop();

    return 0;
}