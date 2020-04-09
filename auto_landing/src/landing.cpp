#include<auto_landing/landing.hpp>

namespace ariitk::auto_landing{

void Landing::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, char** argv){
    mav_command_sub_ = nh.subscribe("mav_command", 1, &Landing::mavCommandCallback, this);
    mav_odometry_sub_ = nh.subscribe("mav_odometry", 1, &Landing::mavOdometryCallback, this);
    husky_odometry_sub_ = nh.subscribe("model_state", 1, &Landing::modelStateCallback, this);
    
    mav_final_command_pub_ = nh.advertise<geometry_msgs::PoseStamped>("mav_final_command", 1);
}

void Landing::run(){
    mav_final_command_ = mav_command_;

    if((fabs(husky_odometry_.pose.pose.position.x-mav_odometry_.pose.pose.position.x) < 0.025) 
        && (fabs(husky_odometry_.pose.pose.position.y-mav_odometry_.pose.pose.position.y) < 0.025)) {
        ROS_INFO("Over Husky.");
        mav_final_command_.pose.position.z = 0.45;
    }
    mav_final_command_pub_.publish(mav_final_command_);
}

void Landing::mavCommandCallback (const geometry_msgs::PoseStamped& msg) {
    mav_command_ = msg;
}

void Landing::mavOdometryCallback (const nav_msgs::Odometry& msg) {
    mav_odometry_ = msg;
}

void Landing::modelStateCallback(const gazebo_msgs::ModelStates& msg) {
    
    int index=2;

    husky_odometry_.pose.pose.position.x = msg.pose[index].position.x;
    husky_odometry_.pose.pose.position.y = msg.pose[index].position.y;
    husky_odometry_.pose.pose.position.z = msg.pose[index].position.z;
}

} //namespace ariitk::auto_landing