// Basic Includes
#include "ros/ros.h"
#include <cstdlib>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/service.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int8.h>

// Service Includes
#include "finger_fk/FingerFkService.h"
#include "adaptive_grasp_controller/adaptive_control.h"

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

// RViz
#include <rviz_visual_tools/rviz_visual_tools.h>

// Action Client
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#define PI 				3.1415

#define JOINTS_CLOSED	0.635										// Closed hand joint value of most of joints
#define THUMB_CLOSED	0.617										// Closed thumb inner joint values

class AdaptiveControl {

	/// private variables -------------------------------------------------------------------------
	private:
		ros::NodeHandle n;

		planning_scene_monitor::PlanningSceneMonitor* psm;		// To read the scene with coll. objs
		robot_model::RobotModelPtr rob_model;					// To read the robot model for the planner
	  
	  	Eigen::Affine3d eeAff; 									// The starting ee pose before rotation
	  	Eigen::Affine3d fingAff; 								// The collision finger pose
	  	Eigen::Affine3d objAff; 								// The object pose

	  	std::string finger_frame_name;							// Name of finger frame in collision
	  	std::string finger_name;								// Name of finger in collision

	  	int finger_id;											// Id of finger in collision

	  	// Action client ptr for hand
	  	std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> move_;
	  	// Action client ptr for arm
	  	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *joint_client;
	  	// Service client for finger_fk_service
	  	ros::ServiceClient fk_client = n.serviceClient<finger_fk::FingerFkService>("finger_fk_service");
	  	// Subscribing to handle finger collision
		ros::Subscriber finger_col_sub;
		// Creating a ROS publisher for sending the trajectory to external node for WAM robot
		ros::Publisher pub_traj_to_topic = n.advertise<trajectory_msgs::JointTrajectory>("from_adaptive_grasp_controller", 1);
		// RViz visual tools
		rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

		const float abd_closed[5] = {0.268, -0.031, 0.007, 0.045, 0.083}; 	// Closed hand abd_joint values

		// Adaptive control variables
		std::string ARM_NAME; 												// Name of the arm (namespace of arm joint_trajectory_controller)
		std::string MOVEIT_GROUP; 											// Name of the MoveIt! MoveGroup of the arm, as in srdf
		std::string HAND_NAME; 												// Name of the hand (namespace of hand joint_trajectory_controller)
		std::string HAND_JOINT; 											// Name of the hand joint
		std::string IMU_TOPIC; 												// Topic from which id of finger in collision is recieved from IMU Glove

		int N_WP; 															// Number of waypoints
		int CONTROL_DELAY; 													// Microseconds of delay between hand and arm controllers in enclosing
		float MAX_SYNERGY; 													// Maximum synergy of SoftHand
		int CLOSE_TIME; 													// Time in which the SoftHand closes totally
		int N_WP_CLOSE; 													// Number of trajectory points of the slow hand closing
		double HAND_TIMEOUT; 												// Hand closing timeout after which, if no collision, go on
		double SLEEP_FOR_MOVE; 												// Sleep to do at least a bit of compensation movement even if other finger touches
		int SKIP_TRAJ_DELAY; 												// Delay in ms to avoid "first trajectory before current time" in first hand close
		int COMPENS_PERC;													// Percentage of the compensation motion to be effectively given to the robot
		int USE_SIGNATURE;                       							// Int for using closure signature axis
		double diff_ft;														// The following are for the force/torque sensor
		geometry_msgs::WrenchStamped initial_ft;
		double FT_THRESHOLD;
		std::string FT_TOPIC;
		bool isFTFirstRead = true;


		int touching_finger = 0;											// For gradual closing and arm stopping
		double current_fraction = 0.0;										// Fraction of compensation motion achieved by MoveIt
		ros::Time arm_motion_start;											// Time at which arm motion starts

	/// public typedefs ---------------------------------------------------------------------------
	public:
	  	typedef boost::shared_ptr<AdaptiveControl> Ptr;
	  	typedef boost::shared_ptr<const AdaptiveControl> ConstPtr;
	  
	/// public functions --------------------------------------------------------------------------
	public:
		AdaptiveControl(ros::NodeHandle& nh);

		~AdaptiveControl();

	  	// Get parameters from parameter server
	  	bool getParamsOfYaml();

	  	// For publishing geometry_msgs pose to RViz
	  	void publishStuff(const std::vector<geometry_msgs::Pose>& pose_msgs);

		// Finger touch subscriber callback
		void fingerColCallback(const std_msgs::Int8::ConstPtr& msg);

	  	// Activate the controller for execution. This is the callback function of the controller service
	  	bool call_adaptive_grasp_controller(adaptive_grasp_controller::adaptive_control::Request &req, adaptive_grasp_controller::adaptive_control::Response &res);

	  	// Starts closing SoftHand and listens for touch events and stops eventually and returns the id of the touching finger
		int startClosingHand();

		// Pushes back position, velocity and duration to a JointTrajectoryPoint
		trajectory_msgs::JointTrajectoryPoint gen_point(double position, double velocity, ros::Duration t);

		// Creates a JointTrajectoryGoal form a position
		control_msgs::FollowJointTrajectoryGoal create_trajectory_goal(double position);

		// Sends trajectory to hand after waiting for server
		void send_goal(double position);

		// Sends a trajectory to the hand controller for execution.
		virtual void sendHandTrajectory(double goal);

		// Performs motion planning and sends trajectory to the joint_trajectory_controller
		void performMotionPlan();

		// To close hand if MoveIt does not find any solution
		void closeHandFromCurrentPosition();

		// Gets finger joint states (first one returns thumb joints and the second one of other fingers)
		void getFingerJointStates(float finger_positions_a[], float finger_positions_b[]);

		/* Callback for FT topic subscriber */
		void getFT(const geometry_msgs::WrenchStamped& curr_ft);

		// Substract arrays
		void substractArrays(float a[], float b[], float res[], int size);

		// Add arrays
		void addArrays(float a[], float b[], float res[], int size);

		// Divide array into N_WP portions
		void divideArray(float a[], int size);

		// Computes waypoints using axis angle rotation of ee pose wrt object pose
		void computeWaypointsFromPoses(std::string finger_name, 
			const Eigen::Affine3d& fing_pose, std::vector<geometry_msgs::Pose>& waypoints);

		// Use closure signature axis to modify rotation of compensatory motion (INTEGRATION WITH CLOSURE SIGNATURE - yet to be finished)
		Eigen::Affine3d waypoint_signature(Eigen::Affine3d fing_pose, Eigen::Affine3d p2f_eigen, 
			Eigen::Affine3d palmeeAff, Eigen::Vector3d signature_axis);

		// Closes hand with timing specified in input
		void sendHandTrajectoryWithTiming(double increment, trajectory_msgs::JointTrajectory ext_trajectory);

		// While performing compensating movement stop if arm in collision or other finger comes to contact
		void stopArmWhenCollision(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *arm_joint_client);

		// Sends trajectory to the joint_trajectory_controller of both arm and hand
		void sendJointTrajectory (trajectory_msgs::JointTrajectory trajectory);
};