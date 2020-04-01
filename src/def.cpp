#include <template/trajectory_headers.hpp>

namespace ariitk::trajectory_gen{
    
void trajectory::init(ros::NodeHandle& nh){
    trajectory_pub=nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel",10);
    nh.getParam("linear_vel",linear_vel);
    nh.getParam("ang_vel",ang_vel);
    msg.angular.z=ang_vel;
    msg.linear.x=linear_vel;
}

void trajectory::run(){
    trajectory_pub.publish(msg);
}
}//namespace ariitk::namespace_name;