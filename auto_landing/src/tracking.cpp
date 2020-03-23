#include<auto_landing/tracking.hpp>


namespace ariitk::auto_landing{

    void tracking::init(ros::NodeHandle& nh) {

        set_firefly_pose_ = nh.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose", 1);
        husky_pose_ = nh.subscribe("/husky_velocity_controller/odom", 1, &tracking::pose_clbk,this);
        landing_server_ = nh.advertiseService("to_land", &tracking::server_clbk,this);
        landing_client_ = nh.serviceClient<std_srvs::Trigger>("to_land");


    }

    void tracking::run(){

        srv_.response.success=false;
        
        
        if(((hk_pose_.pose.pose.position.x-0.1<=setpt_.pose.position.x)||
            (setpt_.pose.position.x<hk_pose_.pose.pose.position.x+0.1))&&
            (hk_pose_.pose.pose.position.y-0.1<=setpt_.pose.position.y)||
            (setpt_.pose.position.y<hk_pose_.pose.pose.position.y+0.1))          landing_client_.call(srv_);

        set_firefly_pose_.publish(setpt_);
    }



    void tracking::pose_clbk(const nav_msgs::Odometry& msg) {
        hk_pose_=msg;

        setpt_.pose.position.x=msg.pose.pose.position.x;
        setpt_.pose.position.y=msg.pose.pose.position.y;
        setpt_.pose.position.z=2;
        }
    
    
    bool tracking::server_clbk(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp){
        resp.success=true;
        resp.message="trigged";
        return true;
    }
}