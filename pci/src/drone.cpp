#include <pci/drone.h>

Drone::Drone(ros::NodeHandle nh, ros::NodeHandle nh_private):nh_(nh), nh_private_(nh_private)
{
    ros::Timer timer = nh_.createTimer(ros::Duration(0.05), &Drone::timer_cb, this);
    initialize_variables();
    initialize_pub_sub();
    initialize_drone();
    ros::spin();
}

void Drone::initialize_pub_sub()
{
    ros::NodeHandle global_nh("~");

    camera_pose_sub = global_nh.subscribe("/gazebo/link_states", 10, &Drone::link_states_callback, this);
    camera_pose_pub = nh_.advertise<geometry_msgs::Pose>("cam_pose", 10);
    camera_pose_for_unity_pub = nh_.advertise<geometry_msgs::Pose>("cam_pose_for_unity", 10);
    // image_sub = global_nh.subscribe("/iris" + id + "/usb_cam/image_raw", 10, &Drone::image_cb, this);
    image_sub = global_nh.subscribe("/uav/camera/left/image_rect_color", 10, &Drone::image_cb, this);
    image_pub_ = nh_.advertise<sensor_msgs::Image>("image", 10);
    cam_ori_pub = global_nh.advertise<geometry_msgs::Pose>("/camera/orientation", 10);
   
    state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &Drone::state_cb, this);
    odom_sub = nh_.subscribe("mavros/local_position/pose", 10, &Drone::odom_cb, this);
    local_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    local_vel_pub = nh_.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    global_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("global_pose", 10);
    arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    sp_pos_sub = nh_.subscribe("sp_pos", 10, &Drone::sp_pos_cb, this);
    sp_vel_sub = nh_.subscribe("sp_vel", 10, &Drone::sp_vel_cb, this);
}

void Drone::initialize_variables()
{
    sp_mode = SP_mode::kPos;
    pci_ready = false;
    takeoff_pose = Eigen::Vector3d(0.0,0.0,5.0);

    sp_vel.linear.x = 0.0;
    sp_vel.linear.y = 0.0;
    sp_vel.linear.z = 0.0;
    sp_vel.angular.x = 0.0;
    sp_vel.angular.y = 0.0;
    sp_vel.angular.z = 0.0;

    sp_pose.position.x = takeoff_pose(0);
    sp_pose.position.y = takeoff_pose(1);
    sp_pose.position.z = takeoff_pose(2);
    sp_pose.orientation.x = 0.0;
    sp_pose.orientation.y = 0.0;
    sp_pose.orientation.z = 0.0;
    sp_pose.orientation.w = 1.0;

    /* Set params */
    std::vector<double> param_val;
    std::string param_name;
    ns = ros::this_node::getNamespace();
    for (char c : ns) {
        if (std::isdigit(c)) {
            id += c;
        }
    }
    // Offset
    param_name = ns + "/offset";
    // std::cout << param_name << std::endl;
    while(!ros::param::get(param_name, param_val)) 
    {
        // std::cout << "Getting offset" << std::endl; 
        ros::spinOnce();
        // std::cout << "Getting offset" << std::endl; 
    }
    offset << param_val[0], param_val[1], param_val[2];
    // Initial position
    param_val.clear();
    param_name = ns + "/initial_pos";
    // std::cout << param_name << std::endl;
    while(!ros::param::get(param_name, param_val))
    {
        // std::cout << "Getting initial position" << std::endl; 
        ros::spinOnce();
    }
    takeoff_pose << param_val[0], param_val[1], param_val[2];
}

void Drone::link_states_callback(const gazebo_msgs::LinkStates::ConstPtr& msg){
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == "iris1::fpv_cam")
        {
            this->camera_pose = msg->pose[i];
            
            geometry_msgs::Pose unity_camera_pose = this->camera_pose;
            //[cyw] q_new is reponsible for the camera orietation in Unity
            tf2::Quaternion q_orig;
            q_orig.setX(this->camera_pose.orientation.x);
            q_orig.setY(this->camera_pose.orientation.y);
            q_orig.setZ(this->camera_pose.orientation.z);
            q_orig.setW(this->camera_pose.orientation.w);
        
            double roll, pitch, yaw;
            tf2::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);
        
            pitch = -pitch;
        
            tf2::Quaternion q_new;
            q_new.setRPY(roll, pitch, yaw);
            q_new.normalize();
            unity_camera_pose.orientation.x = q_new.x();
            unity_camera_pose.orientation.y = q_new.y();
            unity_camera_pose.orientation.z = q_new.z();
            unity_camera_pose.orientation.w = q_new.w();
            
            camera_pose_pub.publish(this->camera_pose);
            camera_pose_for_unity_pub.publish(unity_camera_pose);
            break;
        }
    }
}

void Drone::image_cb(const sensor_msgs::ImageConstPtr& msg){
    image_pub_.publish(*msg);
    
}

void Drone::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void Drone::sp_pos_cb(const geometry_msgs::Pose& pos_sp)
{
    sp_mode = SP_mode::kPos;
    sp_pose = pos_sp;

    geometry_msgs::Pose cam_ori_msg;
    tf2::Quaternion quaternion = smoothLookAt(sp_pose.position);


    cam_ori_msg.position.x = sp_pose.position.x;
    cam_ori_msg.position.y = sp_pose.position.y;
    cam_ori_msg.position.z = sp_pose.position.z;

    cam_ori_msg.orientation.x = quaternion.x();
    cam_ori_msg.orientation.y = quaternion.y();
    cam_ori_msg.orientation.z = quaternion.z();
    cam_ori_msg.orientation.w = quaternion.w();

    // [cyw]: control the yaw in PX4. turn it off if directly controll the camera
    sp_pose.orientation.x = quaternion.x();
    sp_pose.orientation.y = quaternion.y();
    sp_pose.orientation.z = quaternion.z();
    sp_pose.orientation.w = quaternion.w();
    cam_ori_pub.publish(cam_ori_msg);
}

void Drone::sp_vel_cb(const geometry_msgs::Twist& vel_sp)
{
    sp_mode = SP_mode::kVel;
    sp_vel = vel_sp;
}

void Drone::odom_cb(const geometry_msgs::PoseStamped& odom)
{
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    current_pose.position.x = odom.pose.position.x + offset(0);
    current_pose.position.y = odom.pose.position.y + offset(1);
    current_pose.position.z = odom.pose.position.z + offset(2);
    current_pose.orientation = odom.pose.orientation;
    current_pose_vec(0) = odom.pose.position.x;
    current_pose_vec(1) = odom.pose.position.y;
    current_pose_vec(2) = odom.pose.position.z;
    current_pose_vec = current_pose_vec + offset;

    geometry_msgs::PoseStamped global_pose;
    global_pose.header.frame_id = "map";
    global_pose.pose = current_pose;
    global_pose_pub_.publish(global_pose);

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = ns + "/base_link";
    transformStamped.transform.translation.x = global_pose.pose.position.x;
    transformStamped.transform.translation.y = global_pose.pose.position.y;
    transformStamped.transform.translation.z = global_pose.pose.position.z;
    transformStamped.transform.rotation.x = global_pose.pose.orientation.x;
    transformStamped.transform.rotation.y = global_pose.pose.orientation.y;
    transformStamped.transform.rotation.z = global_pose.pose.orientation.z;
    transformStamped.transform.rotation.w = global_pose.pose.orientation.w;
    br.sendTransform(transformStamped);
}

void Drone::timer_cb(const ros::TimerEvent& e)
{
    controller();
}

bool Drone::initialize_drone()
{
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // Dummy setpoint
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    geometry_msgs::Quaternion cam_ori_msg;
    cam_ori_msg.x = 0.0;
    cam_ori_msg.y = 0.0;
    cam_ori_msg.z = 0.0;
    cam_ori_msg.w = 1.0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        cam_ori_pub.publish(cam_ori_msg);
        ros::spinOnce();
        rate.sleep();
    }
    if(set_offboard())
    {
        if(arm_drone())
        {
            takeoff();
            sp_pose = current_pose;
            pci_ready = true;
            return true;
        }
        else
        {
            ROS_ERROR("UNABLE TO ARM");
            return false;
        }
    }
    else
    {
        ROS_ERROR("UNABLE TO SET INTO OFFBOARD");
        return false;
    }
}

bool Drone::set_offboard()
{
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)    
    {
        ROS_INFO("Offboard enabled");
        return true;
    }
    else return false;
}

bool Drone::arm_drone()
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if( arming_client.call(arm_cmd) && arm_cmd.response.success)
    {
        ROS_INFO("Vehicle armed");
        return true;
    }
    else return false;
}

void Drone::takeoff()
{
    ROS_INFO("TAKING OFF");
    ros::Rate loop_rate(20.0);
    Eigen::Vector3d setpoint;
    setpoint = takeoff_pose - offset;
    geometry_msgs::PoseStamped pose_to_publish;
    pose_to_publish.pose.position.x = setpoint(0);
    pose_to_publish.pose.position.y = setpoint(1);
    pose_to_publish.pose.position.z = setpoint(2);
    pose_to_publish.pose.orientation.x = 0.0;
    pose_to_publish.pose.orientation.y = 0.0;
    pose_to_publish.pose.orientation.z = 0.0;
    pose_to_publish.pose.orientation.w = 1.0;
    
    while(!target_reached(0.2, takeoff_pose))
    {
        local_pos_pub.publish(pose_to_publish);
        ros::spinOnce();
        loop_rate.sleep();
        // std::cout << "takeof manuver" << std::endl;
    }
    // pci_ready = true;
    ROS_INFO("TAKEOFF DONE");
}

void Drone::controller()  // Sends commands to the drone
{
    if(pci_ready)
    {
        if(sp_mode == SP_mode::kPos)
        {
            geometry_msgs::PoseStamped pose_to_pub;
            pose_to_pub.pose.position.x = sp_pose.position.x - offset(0);
            pose_to_pub.pose.position.y = sp_pose.position.y - offset(1);
            pose_to_pub.pose.position.z = sp_pose.position.z - offset(2);
            pose_to_pub.pose.orientation.x = sp_pose.orientation.x;
            pose_to_pub.pose.orientation.y = sp_pose.orientation.y;
            pose_to_pub.pose.orientation.z = sp_pose.orientation.z;
            pose_to_pub.pose.orientation.w = sp_pose.orientation.w;
            local_pos_pub.publish(pose_to_pub);
        }
        else if(sp_mode == SP_mode::kVel)
        {
            geometry_msgs::TwistStamped vel_to_pub;
            vel_to_pub.twist = sp_vel;
            local_vel_pub.publish(vel_to_pub);
        }
    }
}

bool Drone::target_reached(double tol, Eigen::Vector3d target)  // Target is in global frame
{
    if((current_pose_vec-target).norm() < tol) return true;
    else return false;
}

std::vector<float> Drone::lookAtOrigin(float x, float y, float z)
{
    tf2::Vector3 current(x, y, z);
    tf2::Vector3 forward = (-current).normalized();  
    tf2::Vector3 world_up(0, 0, 1);
    tf2::Vector3 right = world_up.cross(forward).normalized();
    tf2::Vector3 up = forward.cross(right).normalized();
    tf2::Matrix3x3 rotMatrix(
        forward.x(), right.x(), up.x(),
        forward.y(), right.y(), up.y(),
        forward.z(), right.z(), up.z()
    );
    tf2::Quaternion q, q_lookat, q_correction;
    rotMatrix.getRotation(q);

    // q_correction.setRPY(0, M_PI, 0);  // Roll 180°
    // q_lookat = q_correction * q;
    q_lookat = q;

    return {static_cast<float>(q_lookat.x()), static_cast<float>(q_lookat.y()),
            static_cast<float>(q_lookat.z()), static_cast<float>(q_lookat.w())};
}

tf2::Quaternion Drone::smoothLookAt(
    const geometry_msgs::Point& target_position)
{
    double max_angle_rad = 40.0 * M_PI / 180.0;

    std::vector<float> target_quat_vec = lookAtOrigin(
        target_position.x,
        target_position.y,
        target_position.z);

    tf2::Quaternion q_target(
        target_quat_vec[0],
        target_quat_vec[1],
        target_quat_vec[2],
        target_quat_vec[3]);

    tf2::Quaternion q_current(
        this->current_pose.orientation.x,
        this->current_pose.orientation.y,
        this->current_pose.orientation.z,
        this->current_pose.orientation.w);

    // 計算旋轉角度
    double angle = q_current.angleShortestPath(q_target);

    tf2::Quaternion q_result;
    if (angle > max_angle_rad) {
        double t = max_angle_rad / angle;
        q_result = q_current.slerp(q_target, t);
    } else {
        q_result = q_target;
    }

    q_result.normalize();
    return q_result;
}
