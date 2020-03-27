#include<land_uav/landing.hpp>

namespace ariitk::land_uav;

void Landing::init(ros::NodeHandle& nh){
    landing_firefly=nh.advertise<geometry_msgs::PoseStamped>("firefly/command/pose",1);
    firefly_pose = nh.subscribe("/firefly/ground_truth/pose",1,&Landing::position,this)
}

void Landing::run(){
    
    landing_firefly.publish(pnt);
}

void Landing::position(const geometry_msgs::Pose& msg){
    pnt.pose.position.z=0.4;
}