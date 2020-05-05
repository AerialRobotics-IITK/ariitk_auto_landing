#include<tracking/tracking.hpp>

namespace ariitk::auto_landing {

Tracking::Tracking() 
    : height_(6)
    , inv_state_publish_rate_(0.1) {}

void Tracking::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, char** argv) {
    
    set_firefly_pose_pub_  = nh.advertise<geometry_msgs::PoseStamped>("command_pose", 1);
    quad_pose_sub_  = nh.subscribe("quad_odometry", 1, &Tracking::quadPoseCallback, this);

    if (std::string(argv[2]) == "true") {
        using_detection_ = true; 
        husky_odometry_sub_ = nh.subscribe("husky_odometry", 1, &Tracking::huskyOdometryCallback, this);
    } else {
        using_detection_ = false; 
        gazebo_model_state_sub_ = nh.subscribe("model_state", 1, &Tracking::modelStateCallback, this);
    }

    setpt_.pose.position.z = height_;
    time_[0]=ros::Time::now().toSec();
}

void Tracking::run() {
        
    if (!using_detection_) {
        if((fabs(husky_odom_.pose.pose.position.x-quad_odom_.pose.pose.position.x) < 0.1) 
            && (fabs(husky_odom_.pose.pose.position.y-quad_odom_.pose.pose.position.y) < 0.1)) {
            ROS_INFO("Over Husky.");
        }
        
        if((fabs(husky_cmd_vel_[0].linear.x) > 0.0001 || fabs(husky_cmd_vel_[0].linear.y) > 0.0001)) {
            updateSetPoint();
        }
        
        set_firefly_pose_pub_.publish(setpt_);

    } else {
        setpt_.pose.position.x = 0;
        setpt_.pose.position.y = 0;

        if (platform_detected_) {
            if((fabs(husky_odom_.pose.pose.position.x-quad_odom_.pose.pose.position.x) < 0.1) 
                && (fabs(husky_odom_.pose.pose.position.y-quad_odom_.pose.pose.position.y) < 0.1)) {
                ROS_INFO("Over Husky.");
            }
        
            updateSetPoint();
        
            set_firefly_pose_pub_.publish(setpt_);

        }
    }
}
void Tracking::modelStateCallback(const gazebo_msgs::ModelStates& msg) {
    
    int index = 0;
    std::string name = msg.name[index];
    while (name != "/") {
        index++;
        name = msg.name[index];
    }

    husky_odom_.pose.pose.position.x = msg.pose[index].position.x;
    husky_odom_.pose.pose.position.y = msg.pose[index].position.y;
    husky_odom_.pose.pose.position.z = msg.pose[index].position.z;

    husky_cmd_vel_[0].linear.x = husky_cmd_vel_[1].linear.x;
    husky_cmd_vel_[0].linear.x = husky_cmd_vel_[1].linear.x;
    husky_cmd_vel_[0].linear.x = husky_cmd_vel_[1].linear.x;

    husky_cmd_vel_[1].linear.x = msg.twist[index].linear.x;
    husky_cmd_vel_[1].linear.y = msg.twist[index].linear.y;
    husky_cmd_vel_[1].linear.z = msg.twist[index].linear.z;

}

void Tracking::huskyOdometryCallback(const nav_msgs::Odometry& msg) {
    husky_odom_ = msg;
    platform_detected_ = true;

    husky_cmd_vel_[0].linear.x = husky_cmd_vel_[1].linear.x;
    husky_cmd_vel_[0].linear.x = husky_cmd_vel_[1].linear.x;
    husky_cmd_vel_[0].linear.x = husky_cmd_vel_[1].linear.x;

    husky_cmd_vel_[1].linear.x = msg.twist.twist.linear.x;
    husky_cmd_vel_[1].linear.y = msg.twist.twist.linear.y;
    husky_cmd_vel_[1].linear.z = msg.twist.twist.linear.z;
}

void Tracking::quadPoseCallback(const nav_msgs::Odometry& msg) {
    quad_odom_ = msg;
}
void Tracking::updateSetPoint() {
    
    double distance_x = husky_odom_.pose.pose.position.x - quad_odom_.pose.pose.position.x;
    double distance_y = husky_odom_.pose.pose.position.y - quad_odom_.pose.pose.position.y;
    double norm = sqrt(pow(distance_x, 2) + pow(distance_y, 2));

    if( (fabs(distance_x) > 2) || (fabs(distance_y) > 2) ){
        
        setpt_.pose.position.x = husky_odom_.pose.pose.position.x;
        setpt_.pose.position.y = husky_odom_.pose.pose.position.y;
        time_[0] = ros::Time::now().toSec();
    }

    else if ( (fabs(distance_x) < 2) && (fabs(distance_y) < 2) ){
        
        time_[1] =  ros::Time::now().toSec();
        if((time_[1] - time_[0]) > 4) {
            
            int approximation_value[2] = {5,3};
            if(norm > 0.071){
                approximation_value[1] = 3 - norm;
            }
            double x_approx = husky_cmd_vel_[1].linear.x * inv_state_publish_rate_ + 0.5 * (husky_cmd_vel_[1].linear.x - husky_cmd_vel_[0].linear.x) * pow(inv_state_publish_rate_, 2);
            double y_approx = husky_cmd_vel_[1].linear.y * inv_state_publish_rate_ + 0.5 * (husky_cmd_vel_[1].linear.y - husky_cmd_vel_[0].linear.y) * pow(inv_state_publish_rate_, 2);
            
            double setpt_approximation_x = husky_odom_.pose.pose.position.x + x_approx - quad_odom_.pose.pose.position.x;
            double setpt_approximation_y = husky_odom_.pose.pose.position.y + y_approx - quad_odom_.pose.pose.position.y;

            setpt_.pose.position.x = husky_odom_.pose.pose.position.x + approximation_value[0] * x_approx + approximation_value[1] * setpt_approximation_x;
            setpt_.pose.position.y = husky_odom_.pose.pose.position.y + approximation_value[0] * y_approx + approximation_value[1] * setpt_approximation_y;
        }
        else {
            setpt_.pose.position.x = husky_odom_.pose.pose.position.x;
            setpt_.pose.position.y = husky_odom_.pose.pose.position.y;
        }
    }
    
    ROS_INFO("distance_x_ : %lf distance_y_ : %lf error : %lf", distance_x, distance_y, norm);

}

} // namespace ariitk::auto_landing

