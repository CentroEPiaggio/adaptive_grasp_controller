/* TASK SEQUENCER - Contains all recepies for grasping, handshake and other tasks
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

// Basic Includes
#include "ros/ros.h"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>

// ROS Service and Message Includes
#include "std_srvs/SetBool.h"
#include "geometry_msgs/Pose.h"

// Custom Includes
#include "PandaSoftHandClient.h"

// Other Includes

// Defines
#define     DEBUG   1       // Prints out additional stuff
#define     VISUAL          // Publishes visual info on RViz

class TaskSequencer {

    /// public variables and functions ------------------------------------------------------------
	public:
		TaskSequencer(ros::NodeHandle& nh_);

        ~TaskSequencer();

        // Parameters parsing
        bool parse_task_params();

        // Convert xyzrpy vector to geometry_msgs Pose
        geometry_msgs::Pose convert_vector_to_pose(std::vector<double> input_vec);

        // Callback for object pose subscriber
        void get_object_pose(const geometry_msgs::Pose::ConstPtr &msg);

        // Callback for adaptive grasp task service
        bool call_adaptive_grasp_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

        // Callback for simple grasp task service
        bool call_simple_grasp_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);


	/// private variables -------------------------------------------------------------------------
	private:
		ros::NodeHandle nh;

        // Subscriber to object pose and the pose
        ros::Subscriber object_sub;
        geometry_msgs::Pose object_pose_T;

        // The Panda SoftHand Client
        PandaSoftHandClient panda_softhand_client;

        // Topic and service names
        std::string object_topic_name;
        std::string adaptive_task_service_name;
        std::string grasp_task_service_name;
        std::string handshake_task_service_name;

        // Service Servers
        ros::ServiceServer adaptive_task_server;
        ros::ServiceServer grasp_task_server;
        ros::ServiceServer handshake_task_server;

        // Parsed task sequence variables
        std::vector<double> home_joints;
        std::vector<double> grasp_transform;
        geometry_msgs::Pose grasp_T;
        std::vector<double> pre_grasp_transform;
        geometry_msgs::Pose pre_grasp_T;
        std::vector<double> handover_pose;
        geometry_msgs::Pose handover_T;
};