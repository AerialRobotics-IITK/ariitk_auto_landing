#pragma once

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace ariitk::tracking {

class Tracking {
	public:
		void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, char** argv);
		void run();

		double publish_rate;

	private:
		void modelStateCallback(const gazebo_msgs::ModelStates& msg);
		void quadPoseCallback(const nav_msgs::Odometry& msg);
		void platformOdometryCallback(const nav_msgs::Odometry& msg);
		void updateSetPoint();
		bool isPlatformMoving();
		bool isPlatformFar(double distance_x, double distance_y);
		void calculateSetPoint(double norm);

		int height_;
		double inv_state_publish_rate_;
		double time_[2];
		bool using_detection_, platform_detected_;
		int setpt_approximation_value_, distance_approximation_value_;
		double max_norm_, min_time_;
		int max_x_, max_y_;

		geometry_msgs::PoseStamped setpt_;
		nav_msgs::Odometry platform_odom_, quad_odom_;
		geometry_msgs::Twist platform_cmd_vel_[2];

		ros::Publisher set_quad_pose_pub_;
		ros::Subscriber quad_pose_sub_;
		ros::Subscriber gazebo_model_state_sub_;
		ros::Subscriber platform_odometry_sub_;
};

} // namespace ariitk::tracking
