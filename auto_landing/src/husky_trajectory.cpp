#include <auto_landing/husky_trajectory.hpp>

namespace ariitk::auto_landing{
    
void Huskytrajectory::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private){
    
    trajectory_pub_=nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel",10);
    
    nh_private.getParam("linear_vel",linear_velocity_);
    nh_private.getParam("ang_vel",angular_velocity_);
    
    husky_velocity_.angular.z=angular_velocity_;
    husky_velocity_.linear.x=linear_velocity_;
}

void Huskytrajectory::run(){
    trajectory_pub_.publish(husky_velocity_);
}

} //namespace ariitk::auto_landing;