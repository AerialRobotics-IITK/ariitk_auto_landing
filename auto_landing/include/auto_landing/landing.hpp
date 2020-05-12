#pragma once

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

namespace ariitk::auto_landing {

class Landing {
	public:
		void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, char** argv);
		void run();
		void waitForOdometry();

	private:
		void mavCommandCallback(const geometry_msgs::PoseStamped& msg);
		void mavOdometryCallback(const nav_msgs::Odometry& msg);
		void platformOdometryCallback(const nav_msgs::Odometry& msg);
		void modelStateCallback(const gazebo_msgs::ModelStates& msg);
		void trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory& msg);
		void platformStatusCallback(const std_msgs::Bool& msg);

		bool using_trajectory_generation_;
		bool using_detection_;
		bool initial_takeoff_;

		geometry_msgs::PoseStamped mav_command_;
		geometry_msgs::PoseStamped mav_final_command_;
		trajectory_msgs::MultiDOFJointTrajectory mav_command_trajectory_;
		trajectory_msgs::MultiDOFJointTrajectory mav_final_command_trajectory_;

		nav_msgs::Odometry mav_odometry_;
		nav_msgs::Odometry platform_odometry_;
		std_msgs::Bool is_platform_detected_;

		ros::Subscriber mav_command_sub_;
		ros::Subscriber mav_odometry_sub_;
		ros::Subscriber model_states_sub_;
		ros::Subscriber platform_odometry_sub_;
		ros::Subscriber mav_command_trajectory_sub_;
		ros::Subscriber platform_status_sub_;
		ros::Publisher mav_final_command_pub_;
		ros::Publisher mav_final_command_trajectory_pub_;
};

} // namespace ariitk::auto_landing
