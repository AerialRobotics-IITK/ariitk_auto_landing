#include<auto_landing/landing.hpp>

namespace ariitk::auto_landing {

void Landing::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, char** argv) {
    // ROS_ERROR ("%s\n\n\n\n\n\n",argv[2]);
    
    if (std::string(argv[2]) == "true") {
        using_trajectory_generation_ = true;
        ROS_WARN ("Using trajectory_generation.");
    } else {
        using_trajectory_generation_ = false;
        ROS_WARN ("Not using trajectory_generation.");
    }
    // ROS_ERROR("Argv is %s\n", argv[2]);
    mav_odometry_sub_ = nh.subscribe("mav_odometry", 1, &Landing::mavOdometryCallback, this);
    husky_odometry_sub_ = nh.subscribe("model_state", 1, &Landing::modelStateCallback, this);

    if (!using_trajectory_generation_) {
        mav_command_sub_ = nh.subscribe("mav_command", 1, &Landing::mavCommandCallback, this);
        mav_final_command_pub_ = nh.advertise<geometry_msgs::PoseStamped>("mav_final_command", 1);
    } else {
        mav_command_trajectory_sub_ = nh.subscribe("command_trajectory", 1, &Landing::trajectoryCallback, this);
        mav_final_command_trajectory_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command_trajectory_result", 1);
    }
    
}

void Landing::run() {
    if (using_trajectory_generation_) {
        mav_final_command_trajectory_ = mav_command_trajectory_;
        
        if((fabs(husky_odometry_.pose.pose.position.x-mav_odometry_.pose.pose.position.x) < 0.25) 
            && (fabs(husky_odometry_.pose.pose.position.y-mav_odometry_.pose.pose.position.y) < 0.25)) {
            ROS_INFO("Over Husky.");

            for (int i=0; i<mav_final_command_trajectory_.points.size(); i++) {
                mav_final_command_trajectory_.points[i].transforms[0].translation.z = 0.45;
            }

        }

        mav_final_command_trajectory_pub_.publish(mav_final_command_trajectory_);
        
    } else {
        mav_final_command_ = mav_command_;

        if((fabs(husky_odometry_.pose.pose.position.x-mav_odometry_.pose.pose.position.x) < 0.025) 
            && (fabs(husky_odometry_.pose.pose.position.y-mav_odometry_.pose.pose.position.y) < 0.025)) {
            ROS_INFO("Over Husky.");
            mav_final_command_.pose.position.z = 0.45;
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
    
    husky_odometry_.pose.pose.position.x = msg.pose[index].position.x;
    husky_odometry_.pose.pose.position.y = msg.pose[index].position.y;
    husky_odometry_.pose.pose.position.z = msg.pose[index].position.z;
}

void Landing::trajectoryCallback (const trajectory_msgs::MultiDOFJointTrajectory& msg) {
    mav_command_trajectory_ = msg;
}

} //namespace ariitk::auto_landing