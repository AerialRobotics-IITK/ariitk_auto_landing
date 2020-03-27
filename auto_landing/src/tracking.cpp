#include<auto_landing/tracking.hpp>

namespace ariitk::auto_landing {

Tracking::Tracking() :
    height_(2) {}

void Tracking::init(ros::NodeHandle& nh, char** argv) {
    char* end_ptr;
    husky_relative_x_ = strtod(argv[2], &end_ptr);
    husky_relative_y_ = strtod(argv[4], &end_ptr);

    ROS_INFO ("Husky spawned at %lf %lf", husky_relative_x_, husky_relative_y_);

    set_firefly_pose_ = nh.advertise<geometry_msgs::PoseStamped>("commad_pose", 1);
    husky_pose_ = nh.subscribe("husky_odometry", 1, &Tracking::huskyPoseCallback, this);
    quad_pose_ = nh.subscribe("quad_odometry", 1, &Tracking::quadPoseCallback, this);
    landing_client_ = nh.serviceClient<std_srvs::Trigger>("to_land");
}

void Tracking::run(){
    landing_service_.response.success=false;
    
    if((abs(husky_odom_.pose.pose.position.x-quad_odom_.pose.pose.position.x) < 0.1) && (abs(husky_odom_.pose.pose.position.x-quad_odom_.pose.pose.position.x) < 0.1)) {
        ROS_INFO("Over Husky.");
        landing_client_.call(landing_service_);
    }

    set_firefly_pose_.publish(setpt_);
}



void Tracking::huskyPoseCallback(const nav_msgs::Odometry& msg) {
    husky_odom_=msg;
    setpt_.pose.position.x=msg.pose.pose.position.x+husky_relative_x_;
    setpt_.pose.position.y=msg.pose.pose.position.y+husky_relative_y_;
    setpt_.pose.position.z=height_;
}

void Tracking::quadPoseCallback(const nav_msgs::Odometry& msg) {
    quad_odom_=msg;
}

} // namespace ariitk::auto_landing