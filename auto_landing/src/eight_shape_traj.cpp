#include<auto_landing/eight_shape_traj.hpp>


namespace ariitk::auto_landing {

void Eight_Traj::init(ros::NodeHandle& nh) {

    set_husky_vel_ = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1);
    husky_pose_ = nh.subscribe("/husky_velocity_controller/odom", 1, &Eight_Traj::poseClbk,this);

}

void Eight_Traj::run(){
    t2=ros::Time::now().toSec();
    cmd_vel_=Eight_Traj::cmdVelocity(husky_odom_,count_,t1,t2);
    set_husky_vel_.publish(cmd_vel_);

}

void Eight_Traj::poseClbk(const nav_msgs::Odometry& msg){
    husky_odom_= msg;
}

geometry_msgs::Twist Eight_Traj::cmdVelocity(const nav_msgs::Odometry& husky_odom_,int& count_,double& t1,double& t2){
    geometry_msgs::Twist set_vel_;



    if(t2-t1>22.8){
        t1=ros::Time::now().toSec();
        count_++;
}

    if(count_%2==0){
        set_vel_.linear.x=2;
        set_vel_.linear.y=0;
        set_vel_.linear.z=0;

        set_vel_.angular.x=0;
        set_vel_.angular.y=0;
        set_vel_.angular.z=0.2;
}

else {
        set_vel_.linear.x=2;
        set_vel_.linear.y=0;
        set_vel_.linear.z=0;

        set_vel_.angular.x=0;
        set_vel_.angular.y=0;
        set_vel_.angular.z=-0.2;

}
return set_vel_;
}
} //namespace ariitk::auto_landing