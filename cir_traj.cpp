#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

int main(int argc,char** argv){
    ros::init(argc,argv,"cir_traj");
    ros::NodeHandle nh;
    ros::Publisher pub=nh.advertise<nav_msgs::Odometry>("/husky_velocity_controller/odom",10);
    ros::Rate loopRate(30);
    double t=0;
    while(ros::ok()){
        nav_msgs::Odometry msg;
        msg.pose.pose.position.x=2-2*cos(t);
        msg.pose.pose.position.y=2*sin(t);
        pub.publish(msg);
        loopRate.sleep();
        t+=0.01;
    }
    return 0;
}