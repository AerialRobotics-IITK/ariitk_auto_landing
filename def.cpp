#include <template/template_ros.hpp>

namespace ariitk::trajectory_gen{
    
void trajectory::init(ros::NodeHandle& nh){
    trajectory_pub=nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel",10);
}

void trajectory::run(){
    geometry_msgs::Twist msg;
    msg.angular.z=0.5;//optimum value
    msg.linear.x=5;
    trajectory_pub.publish(msg);
}
}//namespace ariitk::namespace_name;