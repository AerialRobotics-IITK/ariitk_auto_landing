#include<auto_landing/landing.hpp>

namespace ariitk::auto_landing {

void Landing::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, char** argv) {    
    if (std::string(argv[2]) == "true") {
        using_trajectory_generation_ = true;
        ROS_WARN ("Using trajectory_generation.");
    } else {
        using_trajectory_generation_ = false;
        ROS_WARN ("Not using trajectory_generation.");
    }

    mav_odometry_sub_ = nh.subscribe("mav_odometry", 1, &Landing::mavOdometryCallback, this);
  
    if (std::string(argv[4]) == "false") { 
        using_detection_ = false;
        model_states_sub_ = nh.subscribe("model_state", 1, &Landing::modelStateCallback, this);
    }
    else {
        using_detection_ = true;
        husky_odometry_sub_ = nh.subscribe("husky_odometry", 1, &Landing::huskyOdometryCallback, this);
        platform_status_sub_ = nh.subscribe("platform_status", 1, &Landing::platformStatusCallback, this);
    }

    if (!using_trajectory_generation_) {
        mav_command_sub_ = nh.subscribe("mav_command", 1, &Landing::mavCommandCallback, this);
        mav_final_command_pub_ = nh.advertise<geometry_msgs::PoseStamped>("mav_final_command", 1);
    } else {
        mav_command_trajectory_sub_ = nh.subscribe("command_trajectory", 1, &Landing::trajectoryCallback, this);
        mav_final_command_trajectory_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command_trajectory_result", 1);
    }

    initial_takeoff_ = false;
    
}

void Landing::run() {

    if (mav_odometry_.pose.pose.position.z > 4) { initial_takeoff_ = true; }
    
    if (using_trajectory_generation_) {
        mav_final_command_trajectory_ = mav_command_trajectory_;
        
        if((fabs(platform_odometry_.pose.pose.position.x-mav_odometry_.pose.pose.position.x) < distance_threshold_) 
            && (fabs(platform_odometry_.pose.pose.position.y-mav_odometry_.pose.pose.position.y) < distance_threshold_)) {
            ROS_INFO("Over platform.");

            for (int i=0; i<mav_final_command_trajectory_.points.size(); i++) {
                mav_final_command_trajectory_.points[i].transforms[0].translation.z = platform_height_;
            }

            }
        }

        mav_final_command_trajectory_pub_.publish(mav_final_command_trajectory_);
        
    } else {
        mav_final_command_ = mav_command_;

        if ((!using_detection_ || (using_detection_ && is_platform_detected_.data)) && initial_takeoff_) {
            if((fabs(husky_odometry_.pose.pose.position.x-mav_odometry_.pose.pose.position.x) < distance_threshold_) 
                && (fabs(husky_odometry_.pose.pose.position.y-mav_odometry_.pose.pose.position.y) < distance_threshold_)) {
                ROS_INFO("Over platform.");
                mav_final_command_.pose.position.z = platform_height_;
            }
        }

        mav_final_command_pub_.publish(mav_final_command_);
    }

}

void Landing::mavCommandCallback (const geometry_msgs::PoseStamped& msg) {
    mav_command_ = msg;
}

void Landing::mavOdometryCallback (const nav_msgs::Odometry& msg) {
    mav_odometry_ = msg;
}

void Landing::modelStateCallback(const gazebo_msgs::ModelStates& msg) {
    
    int index = 0;
    std::string name = msg.name[index];
    while(name != "/") { name = msg.name[++index]; }
    
    platform_odometry_.pose.pose.position.x = msg.pose[index].position.x;
    platform_odometry_.pose.pose.position.y = msg.pose[index].position.y;
    platform_odometry_.pose.pose.position.z = msg.pose[index].position.z;
}

void Landing::trajectoryCallback (const trajectory_msgs::MultiDOFJointTrajectory& msg) { mav_command_trajectory_ = msg; }

void Landing::huskyOdometryCallback(const nav_msgs::Odometry& msg) { husky_odometry_ = msg; }

void Landing::platformStatusCallback(const std_msgs::Bool& msg) { is_platform_detected_ = msg; }

} // namespace ariitk::auto_landing