#pragma once

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace ariitk::auto_landing {

class Tracking {
	public:
        Tracking();
        void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, char** argv);
        void run();

	private:
        void modelStateCallback(const gazebo_msgs::ModelStates& msg);
        void quadPoseCallback(const nav_msgs::Odometry& msg);
        void huskyOdometryCallback(const nav_msgs::Odometry& msg);
        void updateSetPoint();

        int height_;
        double inv_state_publish_rate_;
        double time_[2];
        bool using_detection_, platform_detected_;

        geometry_msgs::PoseStamped setpt_;
        nav_msgs::Odometry husky_odom_, quad_odom_;
        geometry_msgs::Twist husky_cmd_vel_[2];

        ros::Publisher set_firefly_pose_pub_;
        ros::Subscriber quad_pose_sub_;
        ros::Subscriber gazebo_model_state_sub_;
        ros::Subscriber husky_odometry_sub_;
};

} // namespace ariitk::auto_landing
