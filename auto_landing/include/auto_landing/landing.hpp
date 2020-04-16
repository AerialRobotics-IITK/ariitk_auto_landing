#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
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
        void modelStateCallback (const gazebo_msgs::ModelStates& msg);  
        void trajectoryCallback (const trajectory_msgs::MultiDOFJointTrajectory& msg);

        geometry_msgs::PoseStamped mav_command_, mav_final_command_;
        trajectory_msgs::MultiDOFJointTrajectory command_trajectory;

        nav_msgs::Odometry mav_odometry_, husky_odometry_;

        ros::Subscriber mav_command_sub_, mav_odometry_sub_, husky_odometry_sub_;
        ros::Publisher mav_final_command_pub_;
    };
} // namespace ariitk::auto_landing