#include <auto_landing/husky_trajectory.hpp>

namespace ariitk::auto_landing {
    
void HuskyTrajectory::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, char**argv) {
    
    trajectory_pub_ = nh.advertise<geometry_msgs::Twist>("command_velocity", 10);
    gazebo_model_state_sub_ = nh.subscribe("model_state",1,&HuskyTrajectory::modelStateCallback,this);
    
    nh_private.getParam("linear_vel",linear_velocity_);
    nh_private.getParam("ang_vel",angular_velocity_);
    
    trajectory_name_ = argv[2];
    time_[0] = 0; count_= 0;
    
}

void HuskyTrajectory::run() {
  
  if (trajectory_name_ == "linear") {
        linearTrajectory();
    }
    else if(trajectory_name_ == "eight") {
        eightTrajectory();
    }

    else {
        circularTrajectory();
    }
    trajectory_pub_.publish(husky_velocity_);
}
void HuskyTrajectory::modelStateCallback(const gazebo_msgs::ModelStates& msg) {
    
    int index;
    if( msg.name[1] == "firefly" ) { index = 2; }
    else if( msg.name[2] == "firefly" ) { index = 1; }

    husky_odom_.pose.pose.position.x = msg.pose[index].position.x;
    husky_odom_.pose.pose.position.y = msg.pose[index].position.y;
}
void HuskyTrajectory::linearTrajectory() {

    husky_velocity_.linear.x = linear_velocity_;
}
void HuskyTrajectory::circularTrajectory() {
    
    husky_velocity_.linear.x = linear_velocity_;
    husky_velocity_.angular.z = angular_velocity_;
}
void HuskyTrajectory::eightTrajectory() {
    
    if( time_[0]==0 ) {
        time_[0] = ros::Time::now().toSec();
    }

    time_[1] = ros::Time::now().toSec();

    if( ((time_[1]-time_[0]) > 43) && (count_%2 ==0)) {
        time_[0] = ros::Time::now().toSec();
        count_++;
    }
    else if( ((time_[1]-time_[0]) > 55) && (count_%2 ==1)) {
        time_[0] = ros::Time::now().toSec();
        count_++;
    }

    if( count_ % 2 == 0 ) {
        husky_velocity_.linear.x = linear_velocity_;
        husky_velocity_.angular.z = angular_velocity_;
    }

    else {
        husky_velocity_.linear.x = linear_velocity_;
        husky_velocity_.angular.z = -1*angular_velocity_;
    }
    std::cout<<std::endl<<"count _" <<count_;

}

} //namespace ariitk::auto_landing

