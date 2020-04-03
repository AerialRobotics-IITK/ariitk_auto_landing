#include<auto_landing/tracking.hpp>

namespace ariitk::auto_landing {

Tracking::Tracking() :
    height_(1) {}

void Tracking::init(ros::NodeHandle& nh, char** argv) {
    char* end_ptr;
    husky_relative_x_ = strtod(argv[2], &end_ptr);
    husky_relative_y_ = strtod(argv[4], &end_ptr);

    ROS_INFO ("Husky spawned at %lf %lf", husky_relative_x_, husky_relative_y_);

    set_firefly_pose_ = nh.advertise<geometry_msgs::PoseStamped>("commad_pose", 1);
    quad_pose_ = nh.subscribe("quad_odometry", 1, &Tracking::quadPoseCallback, this);
    gazebo_model_state_=nh.subscribe("model_state",1,&Tracking::modelStateCallback,this);
    landing_client_ = nh.serviceClient<std_srvs::Trigger>("to_land");
}

void Tracking::run(){
    landing_service_.response.success=false;
    
    if((abs(husky_odom_.pose.pose.position.x-quad_odom_.pose.pose.position.x) < 0.1) && (abs(husky_odom_.pose.pose.position.x-quad_odom_.pose.pose.position.x) < 0.1)) {
        ROS_INFO("Over Husky.");
        landing_client_.call(landing_service_);
    }
    
    if((abs(husky_cmd_vel_[0].linear.x)>0.0001 || abs(husky_cmd_vel_[0].linear.y)>0.0001)){
        quadSetPointUpdate();
    }
    
    set_firefly_pose_.publish(setpt_);
}

void Tracking::modelStateCallback(const gazebo_msgs::ModelStates& msg){
    
    time_[0]= time_[1];
    time_[1]= ros::Time::now().toSec();
    
    if( msg.name[1] == "firefly" ) { 
        index_=2; 
    }

    else if( msg.name[2] == "firefly" ) { 
        index_=1; 
    }

    husky_odom_.pose.pose.position.x= msg.pose[index_].position.x;
    husky_odom_.pose.pose.position.y= msg.pose[index_].position.y;
    husky_odom_.pose.pose.position.z= msg.pose[index_].position.z;

    husky_cmd_vel_[0].linear.x= husky_cmd_vel_[1].linear.x;
    husky_cmd_vel_[0].linear.x= husky_cmd_vel_[1].linear.x;
    husky_cmd_vel_[0].linear.x= husky_cmd_vel_[1].linear.x;

    husky_cmd_vel_[1].linear.x= msg.twist[index_].linear.x;
    husky_cmd_vel_[1].linear.y= msg.twist[index_].linear.y;
    husky_cmd_vel_[1].linear.z= msg.twist[index_].linear.z;

    if(abs(husky_cmd_vel_[0].linear.x)<=0.0001 && abs(husky_cmd_vel_[0].linear.y)<=0.0001){
            
            setpt_.pose.position.x=husky_odom_.pose.pose.position.x;
            setpt_.pose.position.y=husky_odom_.pose.pose.position.y;
            setpt_.pose.position.z=height_;
    }
}

void Tracking::quadPoseCallback(const nav_msgs::Odometry& msg) {
    quad_odom_=msg;
}
void Tracking::quadSetPointUpdate(){
    
    double distance_x_= husky_odom_.pose.pose.position.x-quad_odom_.pose.pose.position.x;
    double distance_y_= husky_odom_.pose.pose.position.y-quad_odom_.pose.pose.position.y;
    
    time_diff_= time_[1]-time_[0];

    double x_approx_= husky_cmd_vel_[1].linear.x*time_diff_ + 0.5*(husky_cmd_vel_[1].linear.x-husky_cmd_vel_[0].linear.x)*pow(time_diff_,2);
    double y_approx_= husky_cmd_vel_[1].linear.y*time_diff_ + 0.5*(husky_cmd_vel_[1].linear.y-husky_cmd_vel_[0].linear.y)*pow(time_diff_,2);
    
    setpt_approximation_[0]= husky_odom_.pose.pose.position.x + x_approx_ - quad_odom_.pose.pose.position.x;
    setpt_approximation_[1]= husky_odom_.pose.pose.position.y + y_approx_ - quad_odom_.pose.pose.position.y;
   
    setpt_.pose.position.x= husky_odom_.pose.pose.position.x + appx_value_[0]*x_approx_ + appx_value_[1]*setpt_approximation_[0];
    setpt_.pose.position.y= husky_odom_.pose.pose.position.y + appx_value_[0]*y_approx_ + appx_value_[1]*setpt_approximation_[1];
    setpt_.pose.position.z= height_;

    norm_= sqrt(pow(distance_x_,2)+pow(distance_y_,2));
    
    std::cout<<"distance_x_ : "<<distance_x_<<std::endl<<"distance_y_ : "<<distance_y_<<std::endl;
    }
} // namespace ariitk::auto_landing
