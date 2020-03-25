#include <template/template_ros.hpp>

namespace ariitk::template{
    
void TemplateRos::init(ros::NodeHandle& nh){
    traj_pub=nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel",10);
}

void TemplateRos::run(){
    geometry_msgs::Twist msg1;
        msg1.angular.z=0.5;//optimum value
        msg1.linear.x=5;
        pub1.publish(msg1);
}
}