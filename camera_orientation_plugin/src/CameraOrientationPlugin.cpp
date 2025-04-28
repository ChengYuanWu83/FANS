// File: CameraOrientationPlugin.cpp (Updated version)
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Quaternion.hh>
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>

namespace gazebo
{
class CameraOrientationPlugin : public ModelPlugin
{
private:
	ros::NodeHandle* nh_;
	ros::Subscriber quat_sub_;
	physics::ModelPtr model_;
	physics::JointPtr joint_;
	physics::JointControllerPtr joint_controller_;
	physics::LinkPtr cam_link_;
  
public:
	void Load(physics::ModelPtr _model, sdf::ElementPtr) override
	{
		model_ = _model;
	
		if (!ros::isInitialized())
		{
			std::cout << "[CameraOrientationPlugin] ROS node not initialized!" << std::endl;
			return;
		}
	
		nh_ = new ros::NodeHandle("/camera_orientation_plugin");
		quat_sub_ = nh_->subscribe("/camera/orientation", 10, &CameraOrientationPlugin::OnQuatMsg, this);
		
		joint_ = model_->GetJoint("fpv_cam_pitch_joint");
		if (!joint_)
        {
			std::cout  << "[CameraOrientationPlugin] Cannot find fpv_cam_pitch_joint!" << std::endl;
          	return;
        }
		cam_link_ = model_->GetLink("fpv_cam");
		if (!cam_link_)
		{
			std::cout << "[CameraOrientationPlugin] Camera link not found!" << std::endl;
			return;
		}

	
		std::cout << "[CameraOrientationPlugin] Successfully loaded." << std::endl;
	}

	void OnQuatMsg(const geometry_msgs::Quaternion& msg)
	{

		ignition::math::Quaterniond orientation(msg.w, msg.x, msg.y, msg.z);
		std::cout << "[CameraOrientationPlugin] orientation: " << orientation << std::endl;
        // cam_link_->SetWorldPose(
        //     ignition::math::Pose3d(cam_link_->WorldPose().Pos(), orientation)
        // );

		ignition::math::Vector3d rpy = orientation.Euler();
		double pitch = rpy.Y();

		joint_->SetPosition(0, pitch);
	}

	~CameraOrientationPlugin()
	{
		delete nh_;
	}
};

GZ_REGISTER_MODEL_PLUGIN(CameraOrientationPlugin)
} // namespace gazebo
