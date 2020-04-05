#include<land_uav/landing.hpp>

namespace ariitk::land_uav{

void Landing::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, char** argv){
    landing_server_=nh_private.advertiseService("to_land",position_land);
    firefly_pose = nh.subscribe("command_pose",1,&Landing::position_sub,this)
}

void Landing::run(){
    
    landing_server.publish(landing_service_);
}

bool Landing::position_land(std_srvs::Trigger::Response &res){

    res.pose.position.x= pnt.pose.position.x;
    res.pose.position.y= pnt.pose.position.y;
    res.pose.position.z= 0.48;
    return true;
 }
void Landing::position_sub(const geometry_msgs::Pose& msg){
    pnt=msg;
}

} //namespace ariitk::land_uav