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
    // husky_pose_ = nh.subscribe("husky_odometry", 1, &Tracking::huskyPoseCallback, this);
    // husky_velocity_ = nh.subscribe("husky_cmd_velocity", 1, &Tracking::huskyVelocityCallback, this);
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
    
    if((abs(husky_cmd_vel_.linear.x)>0.0001 || abs(husky_cmd_vel_.linear.y)>0.0001))
                {quadSetPointUpdate(husky_odom_,quad_odom_);}
    
    set_firefly_pose_.publish(setpt_);
}

void Tracking::modelStateCallback(const gazebo_msgs::ModelStates& msg){
    
    t1=t2;
    t2=ros::Time::now().toSec();
    
    if(msg.name[1]=="firefly") {n=2;}
    if(msg.name[2]=="firefly") {n=1;}

    
    husky_odom_.pose.pose.position.x= msg.pose[n].position.x;
    husky_odom_.pose.pose.position.y= msg.pose[n].position.y;
    husky_odom_.pose.pose.position.z= msg.pose[n].position.z;

    husky_cmd_vel_.linear.x=husky_odom_.twist.twist.linear.x;
    husky_cmd_vel_.linear.x=husky_odom_.twist.twist.linear.x;
    husky_cmd_vel_.linear.x=husky_odom_.twist.twist.linear.x;

    husky_odom_.twist.twist.linear.x=msg.twist[n].linear.x;
    husky_odom_.twist.twist.linear.y=msg.twist[n].linear.y;
    husky_odom_.twist.twist.linear.z=msg.twist[n].linear.z;

    if(abs(husky_cmd_vel_.linear.x)<=0.0001 && abs(husky_cmd_vel_.linear.y)<=0.0001){
        
            setpt_.pose.position.x=husky_odom_.pose.pose.position.x;
            setpt_.pose.position.y=husky_odom_.pose.pose.position.y;
            setpt_.pose.position.z=height_;
    }
}

void Tracking::quadPoseCallback(const nav_msgs::Odometry& msg) {
    quad_odom_=msg;
}
void Tracking::quadSetPointUpdate(nav_msgs::Odometry& husky_odom_, nav_msgs::Odometry& quad_odom_){
    
    double distance_x_= husky_odom_.pose.pose.position.x-quad_odom_.pose.pose.position.x;
    double distance_y_= husky_odom_.pose.pose.position.y-quad_odom_.pose.pose.position.y;
    
    double x_approx_= husky_odom_.twist.twist.linear.x*(t2-t1)+0.5*(husky_odom_.twist.twist.linear.x-husky_cmd_vel_.linear.x)*pow(t2-t1,2);
    double y_approx_= husky_odom_.twist.twist.linear.y*(t2-t1)+0.5*(husky_odom_.twist.twist.linear.y-husky_cmd_vel_.linear.y)*pow(t2-t1,2);
    
    double dist_x_approx_= husky_odom_.pose.pose.position.x+x_approx_-quad_odom_.pose.pose.position.x;
    double dist_y_approx_= husky_odom_.pose.pose.position.y+y_approx_-quad_odom_.pose.pose.position.y;
   
    setpt_.pose.position.x=husky_odom_.pose.pose.position.x+3*x_approx_+3*dist_x_approx_;
    setpt_.pose.position.y=husky_odom_.pose.pose.position.y+3*y_approx_+3*dist_y_approx_;
    setpt_.pose.position.z=height_;

    double norm= sqrt(pow(distance_x_,2)+pow(distance_y_,2));
    std::cout <<"norm: "<<norm<<std::endl<<"distance_x_"<<distance_x_<<std::endl<<"distance_y_"<<distance_y_<<std::endl<<"x_approx_"<<x_approx_<<std::endl;
    
    // std::cout<<husky_odom_.pose.pose.position.x<<std::endl<<husky_cmd_vel_<<std::endl;
    // ROS_INFO("GIVING SETPT");
    }
} // namespace ariitk::auto_landing
