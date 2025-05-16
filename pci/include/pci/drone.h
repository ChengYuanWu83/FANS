/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/LinkStates.h>
#include <ros/service.h>

#include <eigen3/Eigen/Dense>

// mavros_msgs::State current_state;
// void state_cb(const mavros_msgs::State::ConstPtr& msg){
//     current_state = *msg;
// }

enum struct SP_mode
{
	kPos = 0,
	kVel = 1
};

class Drone
{
private:
	// cyw add
	std::string id;
	ros::Subscriber camera_pose_sub;
	ros::Publisher  camera_pose_pub;
	ros::Publisher  camera_pose_for_unity_pub;
	ros::Subscriber image_sub;
	ros::Publisher  image_pub_;
	ros::Publisher  cam_ori_pub;
	
  	ros::Subscriber state_sub;
	ros::Subscriber sp_pos_sub;
	ros::Subscriber sp_vel_sub;
	ros::Subscriber odom_sub;
  	ros::Publisher local_pos_pub;
	ros::Publisher local_vel_pub;
	ros::Publisher global_pose_pub_;
  	ros::ServiceClient arming_client;
  	ros::ServiceClient set_mode_client;

	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;

	std::string ns;

	sensor_msgs::Image current_image;

	mavros_msgs::State current_state;

	geometry_msgs::Pose sp_pose, current_pose, camera_pose;
	geometry_msgs::Twist sp_vel;
	SP_mode sp_mode;

	Eigen::Vector3d takeoff_pose;
	Eigen::Vector3d current_pose_vec;
	Eigen::Vector3d offset;
	bool pci_ready;  // Check if the sp are to be followed
public:
	Drone(ros::NodeHandle nh, ros::NodeHandle nh_private);
	void link_states_callback(const gazebo_msgs::LinkStates::ConstPtr& msg);
	void image_cb(const sensor_msgs::ImageConstPtr& msg);
	void state_cb(const mavros_msgs::State::ConstPtr& msg);
	void odom_cb(const geometry_msgs::PoseStamped& odom);
	void sp_vel_cb(const geometry_msgs::Twist& vel_sp);
	void sp_pos_cb(const geometry_msgs::Pose& pos_sp);
	void timer_cb(const ros::TimerEvent&);
	
	bool initialize_drone();
	void initialize_variables();
	void initialize_pub_sub();
  	void start();
	bool set_offboard();
	bool arm_drone();
	void takeoff();
	bool target_reached(double , Eigen::Vector3d);
	void controller();
	
	//cyw
	std::vector<float> lookAtOrigin(float x, float y, float z);


};

