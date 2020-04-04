#include<auto_landing/tracking.hpp>

namespace ariitk::auto_landing {

Tracking::Tracking() 
    : height_(1) {}

void Tracking::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, char** argv) {
    
    set_firefly_pose_pub_  = nh.advertise<geometry_msgs::PoseStamped>("commad_pose", 1);
    quad_pose_sub_  = nh.subscribe("quad_odometry", 1, &Tracking::quadPoseCallback, this);
    gazebo_model_state_sub_ = nh.subscribe("model_state",1,&Tracking::modelStateCallback,this);
    landing_client_  = nh_private.serviceClient<std_srvs::Trigger>("to_land");

    setpt_.pose.position.z = height_;
}

void Tracking::run() {
        
    if((fabs(husky_odom_.pose.pose.position.x-quad_odom_.pose.pose.position.x) < 0.1) && (fabs(husky_odom_.pose.pose.position.x-quad_odom_.pose.pose.position.x) < 0.1)) {
        ROS_INFO("Over Husky.");
        landing_client_.call(landing_service_);
    }
    
    if((fabs(husky_cmd_vel_[0].linear.x) > 0.0001 || fabs(husky_cmd_vel_[0].linear.y) > 0.0001)) {
        updateSetPoint();
    }
    
    set_firefly_pose_pub_.publish(setpt_);
}

void Tracking::modelStateCallback(const gazebo_msgs::ModelStates& msg) {
    
    int index_;
    if( msg.name[1] == "firefly" ) { 
        index_=2; 
    }

    else if( msg.name[2] == "firefly" ) { 
        index_=1; 
    }

    husky_odom_.pose.pose.position.x = msg.pose[index_].position.x;
    husky_odom_.pose.pose.position.y = msg.pose[index_].position.y;
    husky_odom_.pose.pose.position.z = msg.pose[index_].position.z;

    husky_cmd_vel_[0].linear.x = husky_cmd_vel_[1].linear.x;
    husky_cmd_vel_[0].linear.x = husky_cmd_vel_[1].linear.x;
    husky_cmd_vel_[0].linear.x = husky_cmd_vel_[1].linear.x;

    husky_cmd_vel_[1].linear.x = msg.twist[index_].linear.x;
    husky_cmd_vel_[1].linear.y = msg.twist[index_].linear.y;
    husky_cmd_vel_[1].linear.z = msg.twist[index_].linear.z;

    if(fabs(husky_cmd_vel_[0].linear.x) <= 0.0001 && fabs(husky_cmd_vel_[0].linear.y) <= 0.0001) {
            setpt_.pose.position.x = husky_odom_.pose.pose.position.x;
            setpt_.pose.position.y = husky_odom_.pose.pose.position.y;
    }
}

void Tracking::quadPoseCallback(const nav_msgs::Odometry& msg) {
    quad_odom_ = msg;
}
void Tracking::updateSetPoint() {
    
    double distance_x_  = husky_odom_.pose.pose.position.x-quad_odom_.pose.pose.position.x;
    double distance_y_  = husky_odom_.pose.pose.position.y-quad_odom_.pose.pose.position.y;
    
    int approximation_value_[2] = {3,3};
    inv_state_publish_rate_ = 0.1;
    
    double x_approx_  = husky_cmd_vel_[1].linear.x*inv_state_publish_rate_ + 0.5*(husky_cmd_vel_[1].linear.x-husky_cmd_vel_[0].linear.x)*pow(inv_state_publish_rate_,2);
    double y_approx_  = husky_cmd_vel_[1].linear.y*inv_state_publish_rate_ + 0.5*(husky_cmd_vel_[1].linear.y-husky_cmd_vel_[0].linear.y)*pow(inv_state_publish_rate_,2);
    
    double setpt_approximation_x_ = husky_odom_.pose.pose.position.x + x_approx_ - quad_odom_.pose.pose.position.x;
    double setpt_approximation_y_ = husky_odom_.pose.pose.position.y + y_approx_ - quad_odom_.pose.pose.position.y;
   
    setpt_.pose.position.x = husky_odom_.pose.pose.position.x + approximation_value_[0]*x_approx_ + approximation_value_[1]*setpt_approximation_x_;
    setpt_.pose.position.y = husky_odom_.pose.pose.position.y + approximation_value_[0]*y_approx_ + approximation_value_[1]*setpt_approximation_y_;

    double norm_= sqrt(pow(distance_x_,2) + pow(distance_y_,2));
    
    ROS_INFO("distance_x_ : %lf distance_y_ : %lf ",distance_x_,distance_y_);
}

} // namespace ariitk::auto_landing
