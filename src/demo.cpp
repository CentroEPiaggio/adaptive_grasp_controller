/* DEMO MAIN - Calls in sequence grasping, adaptive grasping and handshake (demo march 2019)
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

// Basic Includes
#include "ros/ros.h"

// ROS Service and Message Includes
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "adaptive_grasp_controller/set_object.h"

/**********************************************
ROS NODE MAIN TASK SEQUENCE CLIENT 
**********************************************/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_demo");

    ros::NodeHandle nh_;

    std::cout << "/**********************************************" << std::endl;
    std::cout << "                DEMO MARCH 2019                " << std::endl;
    std::cout << "**********************************************/" << std::endl;

    ROS_INFO("The demo is on! \n");

    // Asking for the object
    std::string object_name;
    std::cout << "Please tell me which object to use... Type and press enter!" << std::endl;
    std::cin >> object_name;

    // Changing the object with service call
    if(!ros::service::waitForService("set_object_service", ros::Duration(3.0))){
        ROS_ERROR("Could not contact the set object service!");
        return 1;
    }
    adaptive_grasp_controller::set_object obj_srv;
    obj_srv.request.object_name = object_name;
    if(!ros::service::call("set_object_service", obj_srv)){
        ROS_ERROR("Could not change the object! Please retry... May be the object is not in the yaml or you spelled it wrong.");
        return 1;
    }

    ROS_INFO_STREAM("The object has been set to " << object_name << ". \n");

    // Asking for the grasp type
    int grasp_type;
    std::cout << "Please tell me which grasping to perform... Type [1] for simple grasp or [2] for adaptive grasp, then press enter!" << std::endl;
    std::cin >> grasp_type;
    if(!(grasp_type == 1 || grasp_type == 2)){
        ROS_ERROR("The number you typed is not valid. Please rerun the node and type a valid number!");
        return 1;
    }

    // Asking if to perform handshake or not
    std::string handshake;
    std::cout << "Would you like to perform a handshake after the grasping? Type [y] if yes or [n] otherwise, then press enter!" << std::endl;
    std::cin >> handshake;
    if(!(handshake == "y" || handshake == "n" || handshake == "Y" || handshake == "N")){
        ROS_ERROR("The character you typed is not valid. Please rerun the node and type a valid character!");
        return 1;
    }

    // Calling the chosen grasp
    std_srvs::SetBool task_srv; task_srv.request.data = true;
    if(grasp_type == 1){        // simple grasp
        if(!ros::service::waitForService("grasp_task_service", ros::Duration(3.0))){
            ROS_ERROR("Could not contact the grasp task service!");
            return 1;
        }
        if(!ros::service::call("grasp_task_service", task_srv)){
            ROS_ERROR("Could not perform the grasp correctly! Please retry...");
            return 1;
        }
    } else {
        if(!ros::service::waitForService("adaptive_task_service", ros::Duration(3.0))){
            ROS_ERROR("Could not contact the adaptive task service!");
            return 1;
        }
        if(!ros::service::call("adaptive_task_service", task_srv)){
            ROS_ERROR("Could not perform the adaptive correctly! Please retry...");
            return 1;
        }
    }

    ROS_INFO("The %s grasp has been performed correctly... \n", (grasp_type == 1) ? "simple" : "adaptive");

    // Going for the handshake
    if(handshake == "y" || handshake == "Y" ){
        if(!ros::service::waitForService("handshake_task_service", ros::Duration(3.0))){
            ROS_ERROR("Could not contact the handshake task service!");
            return 1;
        }
        if(!ros::service::call("handshake_task_service", task_srv)){
            ROS_ERROR("Could not perform the handshake correctly! Please retry...");
            return 1;
        }
        ROS_INFO("The handshake has been performed correctly... \n");
    }

    ROS_INFO_STREAM("Shutting down the demo!");

    return 0;
}