
// Basic Includes
#include <sstream>
#include <string>
#include <exception>
#include <math.h>
#include <chrono>
#include <thread>

#include "AdaptiveControl.h"
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Float64.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
// #include <moveit/trajectory_processing/iterative_time_parameterization.h> 		// TODO: scaling using proper moveit tools

// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

// RViz
#include <rviz_visual_tools/rviz_visual_tools.h>

// Kinematics
#include <moveit_msgs/GetPositionIK.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <geometry_msgs/Twist.h>

#define DEBUG 				0					// Prints out additional info if 1
#define DEBUG_VISUAL 		0					// Publishes additional stuff to RViz

using namespace std;


AdaptiveControl::AdaptiveControl(ros::NodeHandle& nh, 
			boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> arm_client,
				boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> hand_client):n(nh){
	
	ROS_INFO("Starting to create AdaptiveControl object");
	
	// Setting visual tools
	if(!visual_tools_){
		if(DEBUG) std::cout << "RVIZ: Resetting Visual Tools!!!" << std::endl;
		visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world","/rviz_visual_markers"));
	}

	// Setting the action clients
	joint_client = arm_client;
	move_ = hand_client;

	ROS_INFO("Finished creating AdaptiveControl object");
}

AdaptiveControl::~AdaptiveControl(){
	// Nothing to do here
}

/* ******************************************************************************************** */
void AdaptiveControl::fingerColCallback(const std_msgs::Int8::ConstPtr& msg){
	touching_finger = msg->data;
}

/* ******************************************************************************************** */
int AdaptiveControl::startClosingHand(){
	ROS_INFO("Starting to close hand!\n");

	// Checking is hand action client is connected for starting to close the hand
	if(!move_->waitForServer(ros::Duration(1,0))){
		ROS_ERROR("Cannot connect to the hand action client!\n");
	}

	// Listening to topic for finger collision
	touching_finger = 0;

	// Closing hand slowly
	sendHandTrajectory(double (MAX_SYNERGY));

	// Starting to measure time
	ros::Time before_time = ros::Time::now();
	ros::Duration timeout(HAND_TIMEOUT);

	bool first_print_out = true;
	ROS_INFO("I'm waiting for a finger collision!\n");

	// Entering while and staying in it until collision or time duration
	while(touching_finger == 0 && ros::Time::now() - before_time < timeout){

		ros::spinOnce();

		if (!touching_finger) {
			if(first_print_out){
				ROS_INFO("No finger collision received yet!\n");
				first_print_out = false;
			}
		} else {
			ROS_INFO("A COLLISION DETECTED ON FINGER %d!\n", touching_finger);
			move_->cancelGoal();
		}
	}

	return touching_finger;
}

/* ******************************************************************************************* */
trajectory_msgs::JointTrajectoryPoint AdaptiveControl::gen_point(double position, double velocity,
                                                              ros::Duration t){
    trajectory_msgs::JointTrajectoryPoint p;
    p.positions.push_back(position);
    p.velocities.push_back(velocity);
    p.time_from_start = t;
    return p;
}

/* ******************************************************************************************* */
control_msgs::FollowJointTrajectoryGoal AdaptiveControl::create_trajectory_goal(double position){
    auto nanosecond = ros::Duration(0, 1);
    auto millisecond = ros::Duration(0, 1000000);

    // Plan trajectory
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names.push_back(HAND_JOINT);
    // Delay in ms to avoid "first trajectory before current time" in first hand close
    traj.header.stamp = ros::Time::now() + ros::Duration(0, int (SKIP_TRAJ_DELAY) * 1000000);

    // Pushing back trajectory points to slow down the hand closing
    int time_increment = floor(CLOSE_TIME / N_WP_CLOSE);

    for(int i = 1; i <= N_WP_CLOSE; i++){
    	double hand_frac = (double (i)) / (double (N_WP_CLOSE));
    	// velocity has to be 0!!!!!!!!!!!!
    	traj.points.push_back(gen_point(position * hand_frac, 0.0,
    		millisecond * (CLOSE_TIME - (N_WP_CLOSE - i) * time_increment) + nanosecond));
    	if(DEBUG) cout << "THE HAND REFERENCES GIVEN TO CONTROLLER ARE " << position * hand_frac << endl;
    }

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = traj;
    return goal;
}

/* ******************************************************************************************* */
void AdaptiveControl::send_goal(double position){
    // Check whether the server is available
    if (move_->waitForServer(ros::Duration(1,0))){
    	// Create goal and send it to the controller
      	control_msgs::FollowJointTrajectoryGoal goal = create_trajectory_goal(position);

      	move_->sendGoal(goal);
    } else {
      	ROS_WARN_STREAM("No softhand server available. Are you running in simulation without Gazebo?");
    }
}

/* ******************************************************************************************* */
void AdaptiveControl::sendHandTrajectory(double increment){
  	if (increment > 1.0) increment = 1.0;
  	else if (increment < 0.0) increment = 0.0;

  	send_goal(increment);
}

/* ******************************************************************************************** */
void AdaptiveControl::performMotionPlan() {
	// Create the monitor
	if(DEBUG) cout << "Initializing planning scene monitor" << endl;
	boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener());
	psm = new planning_scene_monitor::PlanningSceneMonitor ("robot_description", tf);
	if (psm->getPlanningScene() == NULL) {
	  ROS_ERROR("Unable to initialize planning scene monitor");
	  return;
	}

	// Start monitoring the scene
	psm->startSceneMonitor();
	psm->startWorldGeometryMonitor();
	psm->startStateMonitor();

	// Starting move group interface
	moveit::planning_interface::MoveGroup group(MOVEIT_GROUP);

	// Printing the planning group frame and the group ee frame
	if(DEBUG) ROS_INFO("MoveIt Group Reference frame: %s", group.getPlanningFrame().c_str());
	if(DEBUG) ROS_INFO("MoveIt Group End-effector frame: %s", group.getEndEffectorLink().c_str());

	// Calling the waypoint creator with start and goal poses
	std::vector<geometry_msgs::Pose> cart_waypoints;
	computeWaypointsFromPoses(finger_name, fingAff, cart_waypoints);

	// Publishing waypoints to RViz
	if(DEBUG_VISUAL) publishStuff(cart_waypoints);

	// Some additional parameters for MoveIt
	group.setPlanningTime(5.0);
	group.setPlannerId("RRTConnectkConfigDefault"); // RRTstarkConfigDefault

	// Planning for the waypoints path
	moveit_msgs::RobotTrajectory trajectory;
	double fraction = group.computeCartesianPath(cart_waypoints, 0.01, 0.0, trajectory);

	ROS_INFO("Plan (cartesian path) (%.2f%% acheived)", fraction * 100.0);

	// Printing the positions and vels and slowing down the trajectory with VELOCITY_SCALING
	int num_traj_points = trajectory.joint_trajectory.points.size();
	for(int k = 0; k < num_traj_points; k++){
		// trajectory.joint_trajectory.points[k].time_from_start *= (double(1/VELOCITY_SCALING));
		ROS_INFO_STREAM("The " << k << "th trajectory position of fifth joint is \n" << trajectory.joint_trajectory.points[k].positions[4]
			<< ", the velocity is " << trajectory.joint_trajectory.points[k].velocities[4]
			<< "and the acceleration is " << trajectory.joint_trajectory.points[k].accelerations[4]);
	}

	if(fraction > 0) {
		// Setting current_fraction for using it later on sending partial trajectory to arm
		current_fraction = fraction;

		if(fraction != 1.0) cout << "FOUND ONLY PARTIAL PLAN FOR ENCLOSING HAND AND ARM MOTION!!!" << endl; // return;
		else cout << "FOUND COMPLETE PLAN FOR ENCLOSING HAND AND ARM MOTION!!!" << endl;

		// Send the trajectory to the robot
		sendJointTrajectory(trajectory.joint_trajectory);
		cout << "Sent trajectory to Hand and Arm Joint Trajectory Controller!" << endl;
	}
	else {
		cout << "Motion Plan not found, just closing the hand!" << endl;
		closeHandFromCurrentPosition();
	}
}

/* ******************************************************************************************** */
void AdaptiveControl::closeHandFromCurrentPosition() {
	auto nanosecond = ros::Duration(0, 1);
    auto millisecond = ros::Duration(0, 1000000);

	// Getting current hand synergy state
	sensor_msgs::JointState::ConstPtr hand_joint_state = 
		ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", n);
	
	if (!hand_joint_state) {
		ROS_ERROR("Hand joint states not received!!! \n");
	}

	double synergy_position = 
		hand_joint_state->position[find (hand_joint_state->name.begin(),hand_joint_state->name.end(), 
			string(HAND_JOINT)) - hand_joint_state->name.begin()];

	// How much snynergy can be incremented form current synergy to close hand completely
	double remaining_synergy = double(MAX_SYNERGY) - synergy_position;

	 // Plan trajectory
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names.push_back(HAND_JOINT);
    // Delay in ms to avoid "first trajectory before current time" in first hand close
    traj.header.stamp = ros::Time::now() + ros::Duration(0, int (SKIP_TRAJ_DELAY) * 1000000);

    // Pushing back trajectory points to slow down the hand closing
    int time_increment = floor(CLOSE_TIME / N_WP_CLOSE);
	
	
	for(int i = 1; i <= N_WP_CLOSE; i++){
    	double hand_frac = (double (i)) / (double (N_WP_CLOSE));
    	// velocity has to be 0!!!!!!!!!!!!
    	traj.points.push_back(gen_point(synergy_position + remaining_synergy * hand_frac, 0.0, 
    		millisecond * (CLOSE_TIME - (N_WP_CLOSE - i) * time_increment) + nanosecond));
    	if(DEBUG) cout << "THE HAND REFERENCES GIVEN TO CONTROLLER ARE " << synergy_position + remaining_synergy * hand_frac << endl;
    }

	control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = traj;
    move_->sendGoal(goal);
}

/* ******************************************************************************************** */
void AdaptiveControl::getFingerJointStates(float finger_positions_a[], float finger_positions_b[]) {
	sensor_msgs::JointState::ConstPtr finger_joint_state =
		ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", n);
		// ros::topic::waitForMessage<sensor_msgs::JointState>("/" + string(HAND_NAME) + "/joint_states", n);
	if (!finger_joint_state) {
		ROS_ERROR("Finger joint states not received!!! \n");
	}

	// Saving the needed joint states of finger in collision
	if (finger_name == "thumb") {
		finger_positions_a[0] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_abd_joint") - finger_joint_state->name.begin()];
		finger_positions_a[1] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_inner_joint") - finger_joint_state->name.begin()];
		finger_positions_a[2] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_inner_joint_mimic") - finger_joint_state->name.begin()];
		finger_positions_a[3] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_outer_joint") - finger_joint_state->name.begin()];
		finger_positions_a[4] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_outer_joint_mimic") - finger_joint_state->name.begin()];
	} else {
		finger_positions_b[0] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_abd_joint") - finger_joint_state->name.begin()];
		finger_positions_b[1] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_inner_joint") - finger_joint_state->name.begin()];
		finger_positions_b[2] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_inner_joint_mimic") - finger_joint_state->name.begin()];
		finger_positions_b[3] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_middle_joint") - finger_joint_state->name.begin()];
		finger_positions_b[4] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_middle_joint_mimic") - finger_joint_state->name.begin()];
		finger_positions_b[5] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_outer_joint") - finger_joint_state->name.begin()];
		finger_positions_b[6] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_outer_joint_mimic") - finger_joint_state->name.begin()];
	}
}

/* ******************************************************************************************** */
void AdaptiveControl::substractArrays(float a[], float b[], float res[], int size) {
	for (int i = 0; i < size; i++) {
		res[i] = a[i] - b[i];
	}
}

/* ******************************************************************************************** */
void AdaptiveControl::addArrays(float a[], float b[], float res[], int size) {
	for (int i = 0; i < size; i++) {
		res[i] = a[i] + b[i];
	}
}

/* ******************************************************************************************** */
void AdaptiveControl::divideArray(float a[], int size) {
	for (int i = 0; i < size; i++) {
		a[i] = a[i] / float (N_WP);
	}
}

/* ******************************************************************************************** */
void AdaptiveControl::computeWaypointsFromPoses(string finger_name,
	const Eigen::Affine3d& fing_pose, vector<geometry_msgs::Pose>& waypoints){
	// Compute waypoints
	// Listening transform from ee to palm
	Eigen::Affine3d palmeeAff;
	tf::TransformListener tf_listener_ee_palm;
	tf::StampedTransform stamp_ee_palm_transform;

	try {
		tf_listener_ee_palm.waitForTransform("/" + string(HAND_NAME) + "_palm_link", "/" + string(HAND_NAME) + "_ee_link", ros::Time(0), ros::Duration(10.0) );
		tf_listener_ee_palm.lookupTransform("/" + string(HAND_NAME) + "_palm_link", "/" + string(HAND_NAME) + "_ee_link", ros::Time(0), stamp_ee_palm_transform);
    } catch (tf::TransformException ex){
      	ROS_ERROR("%s",ex.what());
      	ros::Duration(1.0).sleep();
    }

    tf::Transform ee_palm_transform(stamp_ee_palm_transform.getRotation(), stamp_ee_palm_transform.getOrigin());
    tf::transformTFToEigen(ee_palm_transform, palmeeAff);

    // Getting current finger joint state (MAYBE THIS PART NEEDS TO BE MADE MORE PRECISE)
    // sleep(1); 			// Sleeping for better joint_states reading (not necessary)

    float finger_positions_thumb[5];
	float finger_positions_other[7];

	getFingerJointStates(finger_positions_thumb, finger_positions_other);

	// Length of arrays for cout
	int len_thumb_array = (sizeof(finger_positions_thumb)/sizeof(*finger_positions_thumb));
	int len_other_array = (sizeof(finger_positions_other)/sizeof(*finger_positions_other));
	int len_array = (finger_name == "thumb") ? len_thumb_array : len_other_array;

	// Cout initial finger joint positions
	if(DEBUG) cout << "THE INITIAL FINGER JOINT POSITIONS ARE: " << endl;
	for ( int i = 0; i < len_array; i++ ) {
        if(DEBUG) cout << ((finger_name == "thumb") ? finger_positions_thumb[i] : finger_positions_other[i]) << ' ';
    }
    if(DEBUG) cout << endl;

    // Finding the final joint positions and the increment needed to get there from initial in N_WP steps
    float finger_positions_thumb_final[5] = {abd_closed[0], THUMB_CLOSED, THUMB_CLOSED, JOINTS_CLOSED, JOINTS_CLOSED};
    float finger_positions_other_final[7] =
    	{JOINTS_CLOSED, JOINTS_CLOSED, JOINTS_CLOSED, JOINTS_CLOSED, JOINTS_CLOSED, JOINTS_CLOSED, JOINTS_CLOSED};

    // Assigning the first abd_joint of the soft hand which has different values for each finger
    finger_positions_other_final[0] = abd_closed[finger_id-1];

    // Cout final finger joint positions
	if(DEBUG) cout << "THE FINAL FINGER JOINT POSITIONS ARE: " << endl;
	for ( int i = 0; i < len_array; i++ ) {
        if(DEBUG) cout << ((finger_name == "thumb") ? finger_positions_thumb_final[i] : finger_positions_other_final[i]) << ' ';
    }
    if(DEBUG) cout << endl;

    // Find increment array as difference divided by N_WP (the function divideArray divides by N_WP)
    float finger_positions_thumb_inc[5];
	float finger_positions_other_inc[7];
    if (finger_name == "thumb") {
    	substractArrays(finger_positions_thumb_final, finger_positions_thumb, finger_positions_thumb_inc, 5);
    	divideArray(finger_positions_thumb_inc, 5);
    } else {
    	substractArrays(finger_positions_other_final, finger_positions_other, finger_positions_other_inc, 7);
    	divideArray(finger_positions_other_inc, 7);
    }

    // Cout finger joint increments
	if(DEBUG) cout << "THE FINGER POSITIONS INCREMENTS ARE: " << endl;
	for ( int i = 0; i < len_array; i++ ) {
        if(DEBUG) cout << ((finger_name == "thumb") ? finger_positions_thumb_inc[i] : finger_positions_other_inc[i]) << ' ';
    }
    if(DEBUG) cout << endl;

    // Computing N_WP waypoints using finger_fk service
    Eigen::Affine3d wp_eigen; 								// to store the computed waypoint at each for step
    Eigen::Affine3d p2f_eigen;								// to store the transform from palm to finger
    Eigen::Affine3d f2p_eigen;								// to store the transform from finger to palm (inverse of previous)
	geometry_msgs::Pose current_wp;							// needed to push waypoints to moveit
    std_msgs::Float64 msg64;								// for filling the finger_fk_service request

	for(int j = 1; j <= N_WP; j++){
		// Initializing service message inside for!
		finger_fk::FingerFkService palm_to_finger_fk;

		// Finding the joint state for current waypoint by adding increment to previous joint state
		if (finger_name == "thumb") addArrays(finger_positions_thumb, finger_positions_thumb_inc, finger_positions_thumb, 5);
		else addArrays(finger_positions_other, finger_positions_other_inc, finger_positions_other, 7);

		// Fill finger fk service
		palm_to_finger_fk.request.ee_link_name = string(HAND_NAME) + "_palm_link";

		if(DEBUG) cout << "FILLED FIRST PART OF REQUEST NUMBER " << j << endl;

		if (finger_name == "thumb") palm_to_finger_fk.request.finger_link_name = string(HAND_NAME) + "_thumb_distal_link";
		else if (finger_name == "index") palm_to_finger_fk.request.finger_link_name = string(HAND_NAME) + "_index_distal_link";
		else if (finger_name == "middle") palm_to_finger_fk.request.finger_link_name = string(HAND_NAME) + "_middle_distal_link";
		else if (finger_name == "ring") palm_to_finger_fk.request.finger_link_name = string(HAND_NAME) + "_ring_distal_link";
		else if (finger_name == "little") palm_to_finger_fk.request.finger_link_name = string(HAND_NAME) + "_little_distal_link";
		else palm_to_finger_fk.request.finger_link_name = string(HAND_NAME) + "_thumb_distal_link";

		if(DEBUG) cout << "FILLED SECOND PART OF REQUEST NUMBER " << j << endl;

		if (finger_name == "thumb") {
			msg64.data = static_cast<float>(finger_positions_thumb[0]);
			palm_to_finger_fk.request.joint_state_positions.push_back(msg64.data);
			msg64.data = static_cast<float>(finger_positions_thumb[1]);
			palm_to_finger_fk.request.joint_state_positions.push_back(msg64.data);
			msg64.data = static_cast<float>(finger_positions_thumb[2]);
			palm_to_finger_fk.request.joint_state_positions.push_back(msg64.data);
			msg64.data = static_cast<float>(finger_positions_thumb[3]);
			palm_to_finger_fk.request.joint_state_positions.push_back(msg64.data);
			msg64.data = static_cast<float>(finger_positions_thumb[4]);
			palm_to_finger_fk.request.joint_state_positions.push_back(msg64.data);

		} else {
			msg64.data = static_cast<float>(finger_positions_other[0]);
			palm_to_finger_fk.request.joint_state_positions.push_back(msg64.data);
			msg64.data = static_cast<float>(finger_positions_other[1]);
			palm_to_finger_fk.request.joint_state_positions.push_back(msg64.data);
			msg64.data = static_cast<float>(finger_positions_other[2]);
			palm_to_finger_fk.request.joint_state_positions.push_back(msg64.data);
			msg64.data = static_cast<float>(finger_positions_other[3]);
			palm_to_finger_fk.request.joint_state_positions.push_back(msg64.data);
			msg64.data = static_cast<float>(finger_positions_other[4]);
			palm_to_finger_fk.request.joint_state_positions.push_back(msg64.data);
			msg64.data = static_cast<float>(finger_positions_other[5]);
			palm_to_finger_fk.request.joint_state_positions.push_back(msg64.data);
			msg64.data = static_cast<float>(finger_positions_other[6]);
			palm_to_finger_fk.request.joint_state_positions.push_back(msg64.data);

		}

		if(DEBUG) cout << "THE REQUEST JOINTS ARE: " << endl;
		for ( int i = 0; i < len_array; i++ ) {
        	if(DEBUG) cout << palm_to_finger_fk.request.joint_state_positions[i] << ' ';
    	}
    	if(DEBUG) cout << endl;

		if(DEBUG) cout << "FILLED REQUEST NUMBER " << j << "AND GOING TO WP COMPUTATION." << endl;

		// Calling the finger fk service
		if (fk_client.call(palm_to_finger_fk)) {
			tf::poseMsgToEigen (palm_to_finger_fk.response.ee_frame_in_finger, p2f_eigen);
  		} else {
    		ROS_ERROR("Failed to call service finger_fk");
  		}

  		// Composing transforms: world->finger * finger->palm * palm->ee
  		f2p_eigen = p2f_eigen.inverse();

  		if(DEBUG) cout << "WORLD TO FINGER TRANSFORM NUMBER " << j << "." << endl;
		if(DEBUG) cout << "W2F R: " << fing_pose.linear() << endl;
		if(DEBUG) cout << "W2F t: " << fing_pose.translation() << endl;

  		if(DEBUG) cout << "FINGER TO PALM TRANSFORM NUMBER " << j << "." << endl;
		if(DEBUG) cout << "F2P R: " << f2p_eigen.linear() << endl;
		if(DEBUG) cout << "F2P t: " << f2p_eigen.translation() << endl;

		if(DEBUG) cout << "PALM TO EE TRANSFORM NUMBER " << j << "." << endl;
		if(DEBUG) cout << "P2E R: " << palmeeAff.linear() << endl;
		if(DEBUG) cout << "P2E t: " << palmeeAff.translation() << endl;

		// Defining signature axis here temporarily
		Eigen::Vector3d signature_axis(0, 1, 0);

		// Computing waypoints using traditional or new (signature) method (not done yet)
		if(!USE_SIGNATURE){
			wp_eigen = fing_pose * f2p_eigen * palmeeAff;
		} else {
			wp_eigen = waypoint_signature(fing_pose, p2f_eigen, palmeeAff, signature_axis);
		}

		tf::poseEigenToMsg (wp_eigen, current_wp);
		waypoints.push_back(current_wp);

		if(DEBUG) cout << "WAYPOINT NUMBER " << j << "." << endl;
		if(DEBUG) cout << "WP R: " << wp_eigen.linear() << endl;
		if(DEBUG) cout << "WP t: " << wp_eigen.translation() << endl;

		// sleep(1);											// This sleep might not be necessary
	}

}

/* ******************************************************************************************* */
Eigen::Affine3d AdaptiveControl::waypoint_signature(Eigen::Affine3d fing_pose, Eigen::Affine3d p2f_eigen, Eigen::Affine3d palmeeAff, Eigen::Vector3d signature_axis){
	// Getting axis angle parametrization of finger to palm transform
	// Eigen::Affine3d f2p_eigen = p2f_eigen.inverse();
	// Eigen::AngleAxisd newAngleAxis(f2p_eigen.linear());
	// double angle_tmp = newAngleAxis.angle();
	// Eigen::Vector3d axis_tmp = newAngleAxis.axis();

	// // Computing new rotation angle and the corresponding rotation arround signature axis
	// double new_angle = angle_tmp * axis_tmp.dot(signature_axis);
	// Eigen::Matrix3d new_rot;
	// new_rot = Eigen::AngleAxisd(new_angle, signature_axis);

	// Eigen::Affine3d new_f2p;
	// new_f2p.translation() = f2p_eigen.translation();
	// new_f2p.linear() = new_rot;

	// return fing_pose * new_f2p * palmeeAff;

	// Eigen::Affine3d new_transform;
	// new_transform.linear() = new_rot.inverse();

	// return fing_pose * new_transform * f2p_eigen * palmeeAff;


	Eigen::AngleAxisd newAngleAxis(p2f_eigen.linear());
	double angle_tmp = newAngleAxis.angle();
	Eigen::Vector3d axis_tmp = newAngleAxis.axis();

	// Computing new rotation angle and the corresponding rotation arround signature axis
	double new_angle = angle_tmp * axis_tmp.dot(signature_axis);
	Eigen::Matrix3d new_rot;
	new_rot = Eigen::AngleAxisd(new_angle, signature_axis);

	Eigen::Affine3d new_p2f;
	new_p2f.translation() = p2f_eigen.translation();
	new_p2f.linear() = new_rot;

	Eigen::Affine3d new_f2p;
	new_f2p = new_p2f.inverse();

	return fing_pose * new_f2p * palmeeAff;
}


/* ******************************************************************************************* */
void AdaptiveControl::sendHandTrajectoryWithTiming(double increment, trajectory_msgs::JointTrajectory ext_trajectory){
  	// SoftHand synergy (increment) must be in [0, 1]
  	if (increment > 1.0) increment = 1.0;
  	else if (increment < 0.0) increment = 0.0;

  	// Getting current hand synergy state
	sensor_msgs::JointState::ConstPtr hand_joint_state =
		ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", n);

	if (!hand_joint_state) {
		ROS_ERROR("Hand joint states not received!!! \n");
	}

	double synergy_position =
		hand_joint_state->position[find (hand_joint_state->name.begin(),hand_joint_state->name.end(),
			string(HAND_JOINT)) - hand_joint_state->name.begin()];

	// How much snynergy can be incremented form current synergy to close hand completely
	double remaining_synergy = increment - synergy_position;

  	// Check whether the server is available
    if (move_->waitForServer(ros::Duration(1,0))){
    	// Plan the trajectory to give to the hand (trajectory with only one trajectory point)
    	trajectory_msgs::JointTrajectory hand_traj;
    	hand_traj.joint_names.push_back(HAND_JOINT);

    	hand_traj.header.stamp = ros::Time::now();

    	/* For each trajectory point in ext_trajectory create a trajectory point for the hand
    	as a proportional portion of full synergy and give it the same time as the corresponding
    	point in ext_trajectory and the push each point to the hand trajectory*/
    	int traj_len = ext_trajectory.points.size();
    	double percentage;
    	for(int i = 0; i < traj_len; i++){
    		trajectory_msgs::JointTrajectoryPoint traj_point;
    		percentage = double(i+1)/double(traj_len);
    		// Give the point position a portion of remaining synergy + current synergy (first is current synergy!!!)
    		traj_point.positions.push_back(synergy_position + percentage * remaining_synergy);
    		// (velocity has to be 0!!!!!!!!!!)
    		traj_point.velocities.push_back(0.0);

    		if(DEBUG) cout << "THE HAND REFERENCES GIVEN TO CONTROLLER ARE " << synergy_position + percentage * remaining_synergy << endl;

    		// Give the trajectory point the same time_from_start as the input trajectory (its last point)
    		traj_point.time_from_start = ext_trajectory.points[i].time_from_start;

    		// Push the trajectory point to the trajectory
    		hand_traj.points.push_back(traj_point);
    	}

    	control_msgs::FollowJointTrajectoryGoal goal;
    	goal.trajectory = hand_traj;

    	// Send goal to the controller
      	move_->sendGoal(goal);
    } else {
      	ROS_WARN_STREAM("No softhand server available. Are you running in simulation without Gazebo?");
    }
}

/* ******************************************************************************************** */
void AdaptiveControl::stopArmWhenCollision(boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> arm_joint_client){
	ROS_INFO("I'm listening for further finger collisions or arm collision!\n");

	// Some needed data
	int old_touching_finger = finger_id;

	//ROS_WARN("I am going for a nap");
	// Sleep for sometime for not having immediate other collision
	// sleep(SLEEP_FOR_MOVE);
	//ROS_WARN("AWAKE AWAKE AWAKE AWAKE");

	bool first_print_out = true;
	bool exitStopArmWhenCollisionLoop = false;

	ros::Rate rate(100.0);
	// Listening from finger collision topic and arm wrist force torque topic while executing compensating trajectory
	while(!arm_joint_client->getState().isDone() && !exitStopArmWhenCollisionLoop) {
	// while(!exitStopArmWhenCollisionLoop) {

		ros::spinOnce();
		// ROS_INFO("Difference F/T%f/n", diff_ft);
		/* 
		TODO: Add also a force/torque sensor condition for stopping the arm motion <<---
		e.g.: if(norm_wrench > threshold) arm_joint_client->cancelGoal();
		*/
		if (diff_ft > FT_THRESHOLD) {
			ROS_INFO("F/T THRESHOLD EXCEEDED, VALUE %f! STOPPING THE COMPENSATING ARM MOTION!\n", diff_ft);
			arm_joint_client->cancelGoal();
			//ros::spinOnce();
			exitStopArmWhenCollisionLoop = true;
		}

		// If the old finger is thumb, then stop if any other finger touches
		if (old_touching_finger == 1 && old_touching_finger == touching_finger) {
			if(first_print_out){
				ROS_INFO("No further finger collision received yet!\n");
				first_print_out = false;
			}
		} else if(old_touching_finger > 1 && touching_finger > 1 && touching_finger <= 5) {
			if(first_print_out){
				ROS_INFO("No further relevant finger collision received yet!\n");
				first_print_out = false;
			}
		} else if(old_touching_finger == 0 || touching_finger == 0) {
			if(first_print_out){
				ROS_INFO("No finger collision received yet!\n");
				first_print_out = false;
			}
		} else {
			ROS_INFO("A FURTHER COLLISION DETECTED ON FINGER %d! STOPPING THE COMPENSATING ARM MOTION!\n", touching_finger);
			// arm_joint_client->cancelGoal();				// Comment this out if working with IMU Glove
		}
		rate.sleep();
	}
}

/* ******************************************************************************************** */
void AdaptiveControl::sendJointTrajectory(trajectory_msgs::JointTrajectory trajectory) {
	// Checking if the action client for the arm is connected
	if(!joint_client->waitForServer(ros::Duration(1,0))){
		ROS_ERROR("Cannot connect to the arm action client!\n");
	}

	// Not setting time (oscillating problem)

	// Save the arm trajectory in a goal
	control_msgs::FollowJointTrajectoryGoal goalmsg;
	goalmsg.trajectory = trajectory;

	int traj_len = goalmsg.trajectory.points.size();

	// Compute the hand trajectory in sync with the arm motion and send to hand controller
	sendHandTrajectoryWithTiming(double(MAX_SYNERGY), goalmsg.trajectory);

	// A DELAY MIGHT BE NECESSARY
	usleep(CONTROL_DELAY);

	// Change trajectory of arm motion if COMPENS_PERC is not 100 (Reduced Compensating motion)
	if(COMPENS_PERC < (current_fraction * 100)){
		ROS_WARN("Removing some points from trajectory as COMPENS_PERC is smaller.");
		int full_len = traj_len / current_fraction;
		int remove_num = traj_len - (double (COMPENS_PERC) / 100) * full_len;
		for(int i = 0; i < remove_num; i++){
			goalmsg.trajectory.points.pop_back();
		}
	}

	std::cout << "Trajectory was " << traj_len << " points long and now it's "
		<<  goalmsg.trajectory.points.size() << " points long!" << std::endl;

	// Send the trajectory to the arm controler
	joint_client->sendGoal(goalmsg);

	// For the WAM we are publishing the whole trajectory to a topic and and the send_traj_to_wam node will manage it
	pub_traj_to_topic.publish(goalmsg.trajectory);

	// Listen for arm collision or other finger contact and stop if any
	stopArmWhenCollision(joint_client); // THIS WILL WORK ONLY IF sendHandTrajectoryWithTiming IS USED

	// Sleep to wait the conclusion of execution
	sleep(2);

	if(psm != NULL) {
		delete psm;
		psm = NULL;
	}

	cout << "FINISHED EVERYTHING: TERMINATING CONTROLLER!" << endl;
}

/* ******************************************************************************************** */
/* Gets from the parameter server the values of the needed vars loaded from config_col.yaml */
bool AdaptiveControl::getParamsOfYaml(){
	bool success = true;

	if(!ros::param::get("/adaptive_grasp_controller/ARM_NAME", ARM_NAME)){
		ROS_WARN("ARM_NAME param not found in param server! Using default.");
		ARM_NAME = "panda_arm";
		success = false;
	}
	if(!ros::param::get("/adaptive_grasp_controller/MOVEIT_GROUP", MOVEIT_GROUP)){
		ROS_WARN("MOVEIT_GROUP param not found in param server! Using default.");
		MOVEIT_GROUP = "panda_arm";
		success = false;
	}
	if(!ros::param::get("/adaptive_grasp_controller/HAND_NAME", HAND_NAME)){
		ROS_WARN("HAND_NAME param not found in param server! Using default.");
		HAND_NAME = "right_hand";
		success = false;
	}
	if(!ros::param::get("/adaptive_grasp_controller/HAND_JOINT", HAND_JOINT)){
		ROS_WARN("HAND_JOINT param not found in param server! Using default.");
		HAND_JOINT = "right_hand_synergy_joint";
		success = false;
	}
	if(!ros::param::get("/adaptive_grasp_controller/IMU_TOPIC", IMU_TOPIC)){
		ROS_WARN("IMU_TOPIC param not found in param server! Using default.");
		IMU_TOPIC = "touching_finger_topic";
		success = false;
	}
	if(!ros::param::get("/adaptive_grasp_controller/FT_TOPIC", FT_TOPIC)){
		ROS_WARN("FT_TOPIC param not found in param server! Using default.");
		FT_TOPIC = "/ft_sensor";
		success = false;
	} 
	if(!ros::param::get("/adaptive_grasp_controller/N_WP", N_WP)){
		ROS_WARN("N_WP param not found in param server! Using default.");
		N_WP = 20;
		success = false;
	}
	if(!ros::param::get("/adaptive_grasp_controller/CONTROL_DELAY", CONTROL_DELAY)){
		ROS_WARN("CONTROL_DELAY param not found in param server! Using default.");
		CONTROL_DELAY = 40000;
		success = false;
	}
	if(!ros::param::get("/adaptive_grasp_controller/MAX_SYNERGY", MAX_SYNERGY)){
		ROS_WARN("MAX_SYNERGY param not found in param server! Using default.");
		MAX_SYNERGY = 1.00;
		success = false;
	}
	if(!ros::param::get("/adaptive_grasp_controller/CLOSE_TIME", CLOSE_TIME)){
		ROS_WARN("CLOSE_TIME param not found in param server! Using default.");
		CLOSE_TIME = 7000;
		success = false;
	}
	if(!ros::param::get("/adaptive_grasp_controller/N_WP_CLOSE", N_WP_CLOSE)){
		ROS_WARN("N_WP_CLOSE param not found in param server! Using default.");
		N_WP_CLOSE = 60;
		success = false;
	}
	if(!ros::param::get("/adaptive_grasp_controller/HAND_TIMEOUT", HAND_TIMEOUT)){
		ROS_WARN("HAND_TIMEOUT param not found in param server! Using default.");
		HAND_TIMEOUT = 7.0;
		success = false;
	}
	if(!ros::param::get("/adaptive_grasp_controller/SLEEP_FOR_MOVE", SLEEP_FOR_MOVE)){
		ROS_WARN("SLEEP_FOR_MOVE param not found in param server! Using default.");
		SLEEP_FOR_MOVE = 2.0;
		success = false;
	}
	if(!ros::param::get("/adaptive_grasp_controller/SKIP_TRAJ_DELAY", SKIP_TRAJ_DELAY)){
		ROS_WARN("SKIP_TRAJ_DELAY param not found in param server! Using default.");
		SKIP_TRAJ_DELAY = 50;
		success = false;
	}
	if(!ros::param::get("/adaptive_grasp_controller/COMPENS_PERC", COMPENS_PERC)){
		ROS_WARN("COMPENS_PERC param not found in param server! Using default.");
		COMPENS_PERC = 50;
		success = false;
	}
	if(!ros::param::get("/adaptive_grasp_controller/FT_THRESHOLD", FT_THRESHOLD)){
		ROS_WARN("FT_THRESHOLD param not found in param server! Using default.");
		FT_THRESHOLD = 3;
		success = false;
	}
	if(!ros::param::get("/adaptive_grasp_controller/USE_SIGNATURE", USE_SIGNATURE)){
		ROS_WARN("USE_SIGNATURE param not found in param server! Using default.");
		USE_SIGNATURE = 0;
		success = false;
	}
	if(!ros::param::get("/adaptive_grasp_controller/VELOCITY_SCALING", VELOCITY_SCALING)){
		ROS_WARN("VELOCITY_SCALING param not found in param server! Using default.");
		VELOCITY_SCALING = 0.2;
		success = false;
	}

	return success;
}

/* ******************************************************************************************** */
void AdaptiveControl::getFT(const geometry_msgs::WrenchStamped& curr_ft){	
	if(isFTFirstRead) {
		initial_ft = curr_ft;
		isFTFirstRead = false;
	}
	diff_ft = sqrt(pow(curr_ft.wrench.force.x - initial_ft.wrench.force.x, 2) + pow(curr_ft.wrench.force.y - initial_ft.wrench.force.y, 2) + pow(curr_ft.wrench.force.z - initial_ft.wrench.force.z, 2));
}

/* ******************************************************************************************** */
void AdaptiveControl::publishStuff(const std::vector<geometry_msgs::Pose>& pose_msgs){
	for(auto& pose:pose_msgs){
		visual_tools_->publishAxis(pose);
		ros::Duration(0.1).sleep();
	}
}

/* ******************************************************************************************** */
bool AdaptiveControl::call_adaptive_grasp_controller(adaptive_grasp_controller::adaptive_control::Request &req, adaptive_grasp_controller::adaptive_control::Response &res) {
	// Checking the request
	if(req.run == false) return false;

	// Getting params from parameter server
	if(!getParamsOfYaml()) ROS_WARN("Some params not loaded correctly!");

	// Cast the system to a Kuka one for accessing specifics
	psm = NULL;

	// Subscribing to handle finger collision
	finger_col_sub = n.subscribe("/" + std::string(IMU_TOPIC), 1000, &AdaptiveControl::fingerColCallback, this);

	// Start closing hand and listen for finger collisions
	finger_id = startClosingHand();
	finger_id = int(finger_id);

	ROS_INFO_STREAM("DID THE HAND STOP??? \n");

	// Waiting for the hand to finish moving
	if(!move_->waitForResult(ros::Duration(20, 0))){
		ROS_WARN_STREAM("The hand is taking too much time to close! \n");
	}

	// Choosing the correct frame relative to the finger in collision
	if (finger_id == 1) finger_frame_name = "/" + string(HAND_NAME) + "_thumb_distal_link";
	else if (finger_id == 2) finger_frame_name = "/" + string(HAND_NAME) + "_index_distal_link";
	else if (finger_id == 3) finger_frame_name = "/" + string(HAND_NAME) + "_middle_distal_link";
	else if (finger_id == 4) finger_frame_name = "/" + string(HAND_NAME) + "_ring_distal_link";
	else if (finger_id == 5) finger_frame_name = "/" + string(HAND_NAME) + "_little_distal_link";
	else finger_frame_name = "/" + string(HAND_NAME) + "_thumb_distal_link";

	// Choosing the correct name relative to the finger in collision
	if (finger_id == 1) finger_name = "thumb";
	else if (finger_id == 2) finger_name = "index";
	else if (finger_id == 3) finger_name = "middle";
	else if (finger_id == 4) finger_name = "ring";
	else if (finger_id == 5) finger_name = "little";
	else finger_name = "thumb";

	// Using tf listener to get current ee pose, finger pose and object pose and converting to eigen
	tf::TransformListener tf_listener_ee;
	tf::TransformListener tf_listener_fing;
	tf::TransformListener tf_listener_obj;
	tf::StampedTransform stamp_ee_transform;
	tf::StampedTransform stamp_finger_transform;

	try {
		tf_listener_ee.waitForTransform("/world", "/" + string(HAND_NAME) + "_ee_link", ros::Time(0), ros::Duration(10.0) );
		tf_listener_ee.lookupTransform("/world", "/" + string(HAND_NAME) + "_ee_link", ros::Time(0), stamp_ee_transform);
    } catch (tf::TransformException ex){
      	ROS_ERROR("%s",ex.what());
      	ros::Duration(1.0).sleep();
    }

    try {
		tf_listener_fing.waitForTransform("/world", finger_frame_name, ros::Time(0), ros::Duration(10.0) );
		tf_listener_fing.lookupTransform("/world", finger_frame_name, ros::Time(0), stamp_finger_transform);
    } catch (tf::TransformException ex){
      	ROS_ERROR("%s",ex.what());
      	ros::Duration(1.0).sleep();
    }

    tf::Transform ee_transform(stamp_ee_transform.getRotation(), stamp_ee_transform.getOrigin());
    tf::transformTFToEigen(ee_transform, eeAff);
    tf::Transform fing_transform(stamp_finger_transform.getRotation(), stamp_finger_transform.getOrigin());
    tf::transformTFToEigen(fing_transform, fingAff);

	// Print all poses
	if(DEBUG){
		ROS_INFO_STREAM("Endeffector current Translation: " << eeAff.translation());
		ROS_INFO_STREAM("Endeffector current Rotation: " << eeAff.rotation());
		ROS_INFO_STREAM("Finger current Translation: " << fingAff.translation());
		ROS_INFO_STREAM("Finger current Rotation: " << fingAff.rotation());
	}

	// Perform motion plan and sendJointCommand()
	if(finger_id != 0) performMotionPlan();
	else ROS_INFO_STREAM("THIS IS STRANGE: NO FINGER COLLISION WAS RECEIVED FROM IMUs! SO THIS IS A SIMPLE SURFACE GRASP!");

	res.answer = true;
	return true;
}