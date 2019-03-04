/* MAIN CLIENT - Creates the task sequence client to perform some tasks
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

// Basic Includes
#include "ros/ros.h"

// Object Includes
#include "TaskSequencer.h"

/**********************************************
ROS NODE MAIN TASK SEQUENCE CLIENT 
**********************************************/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_task_client");

    ros::NodeHandle nh_;

    ROS_INFO("Creating the TaskSequencer object");

    TaskSequencer task_sequencer_obj(nh_);

    ROS_INFO("The main task sequence client is running. Running as fast as possible!");

    // ROS Async spinner (necessary for processing callbacks inside the service callbacks)
    ros::AsyncSpinner spinner(4);
    spinner.start();

    while(ros::ok()){
        // Nothing to do here
    }

    spinner.stop();

    return 0;
}