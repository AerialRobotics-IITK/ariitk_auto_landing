#pragma once

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace ariitk::auto_landing {

class PlatformTrajectory {
	public:
            void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, char** argv);
            void run();
            int loop_rate_;

	private:
            void modelStateCallback(const gazebo_msgs::ModelStates& msg);
            void linearTrajectory();
            void circularTrajectory();
            void eightTrajectory();

            ros::Publisher trajectory_pub_;
            ros::Subscriber gazebo_model_state_sub_;

            geometry_msgs::Twist platform_velocity_;
            nav_msgs::Odometry platform_odom_;

            double angular_velocity_, linear_velocity_;
            double time_[2];
            double platform_x_, platform_y_;
            int count_;
            double t_inverse, min_time_;
            std::string trajectory_name_;
};

} // namespace ariitk::auto_landing
