#include<auto_landing/tracking.hpp>


namespace ariitk::auto_landing {

// geometry_msgs::PoseStamped pose_;
// pose_.pose.position.x=0;
// pose_.pose.position.y=0;
// pose_.pose.position.z=2;

Tracking::Tracking(){};



void Tracking::init(ros::NodeHandle& nh) {

    set_firefly_pose_ = nh.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose", 1);
    husky_pose_ = nh.subscribe("/husky_velocity_controller/odom", 1, &Tracking::pose_clbk,this);
    landing_server_ = nh.advertiseService("to_land", &Tracking::server_clbk,this);
    landing_client_ = nh.serviceClient<std_srvs::Trigger>("to_land");


}

void Tracking::run(){

    srv_.response.success=false;
    
    
    if(((hk_pose_.pose.pose.position.x-0.1<=setpt_.pose.position.x)||
        (setpt_.pose.position.x<hk_pose_.pose.pose.position.x+0.1))&&
        (hk_pose_.pose.pose.position.y-0.1<=setpt_.pose.position.y)||
        (setpt_.pose.position.y<hk_pose_.pose.pose.position.y+0.1))        {} //{ landing_client_.call(srv_);}

    set_firefly_pose_.publish(setpt_);
}



void Tracking::pose_clbk(const nav_msgs::Odometry& msg) {
    hk_pose_=msg;

    setpt_.pose.position.x=msg.pose.pose.position.x;
    setpt_.pose.position.y=msg.pose.pose.position.y;
    setpt_.pose.position.z=2;
    }


bool Tracking::server_clbk(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp){
    resp.success=true;
    resp.message="trigged";
    return true;
}
} // namespace ariitk::auto_landing