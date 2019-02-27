/* MAIN SERVERS - Creates all necessary service servers to command Panda and SoftHand
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

// Basic Includes
#include "ros/ros.h"

// Object Includes
#include "SlerpControl.h"

/**********************************************
ROS NODE MAIN SERVICE SERVERS 
**********************************************/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "adaptive_grasp_controller");

    ros::NodeHandle nh_;

    std::string arm_jt_topic = "/panda_arm/position_joint_trajectory_controller/follow_joint_trajectory/";
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> arm_client_ptr_(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(arm_jt_topic, true));

    SlerpControl slerp_control_obj(nh_, "panda_arm", "right_hand_ee_link", 60, arm_client_ptr_);

    ros::ServiceServer service = nh_.advertiseService("adaptive_grasp_controller", &SlerpControl::call_slerp_control, &slerp_control_obj);

    ros::Rate loop_rate(50);

    ROS_INFO("The main service server is running");

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}