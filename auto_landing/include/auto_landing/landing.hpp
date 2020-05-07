#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <gazebo_msgs/ModelStates.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

namespace ariitk::auto_landing {

class Landing {
    public:
        void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private,char** argv);
        void run();
        
    private:
        void mavCommandCallback (const geometry_msgs::PoseStamped& msg);
        void mavOdometryCallback (const nav_msgs::Odometry& msg);
        void huskyOdometryCallback (const nav_msgs::Odometry& msg);
        void modelStateCallback (const gazebo_msgs::ModelStates& msg);  
        void trajectoryCallback (const trajectory_msgs::MultiDOFJointTrajectory& msg);
        void platformStatusCallback (const std_msgs::Bool& msg);

        bool using_trajectory_generation_, using_detection_, initial_takeoff_;
        
        geometry_msgs::PoseStamped mav_command_, mav_final_command_;
        trajectory_msgs::MultiDOFJointTrajectory mav_command_trajectory_, mav_final_command_trajectory_;

        nav_msgs::Odometry mav_odometry_, husky_odometry_;
        std_msgs::Bool is_platform_detected_;

        ros::Subscriber mav_command_sub_, mav_odometry_sub_, model_states_sub_, husky_odometry_sub_, mav_command_trajectory_sub_, platform_status_sub_;
        ros::Publisher mav_final_command_pub_, mav_final_command_trajectory_pub_;
    };
} // namespace ariitk::auto_landing