#pragma once

#include <gazebo_msgs/ModelStates.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

namespace ariitk::tracking {

class TrajectoryGenerationTracking {
	public:
		TrajectoryGenerationTracking(char** argv);
		void run();

		int publish_rate_;

	private:
		void mavOdometryCallback(const nav_msgs::Odometry& msg);
		void modelStatesCallback(const gazebo_msgs::ModelStates& msg);
		void platformOdometryCallback(const nav_msgs::Odometry& msg);

		mav_trajectory_generation::Vertex::Vector computePoints();
		void generateTrajectory(std::vector<mav_trajectory_generation::Vertex> vertices);

		bool publish_visualization_;

		double v_max_;
		double a_max_;
		double distance_;

		int dimension_;
		int derivative_to_optimize_;

		Eigen::Vector3d platform_acceleration_;

		mav_trajectory_generation::Trajectory result_;

		trajectory_msgs::MultiDOFJointTrajectory generated_trajectory_;

		visualization_msgs::MarkerArray markers_;

		nav_msgs::Odometry mav_odom_, platform_odom_;

		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;

		ros::Publisher trajectory_pub_, marker_pub_;

		ros::Subscriber mav_odometry_sub_, model_states_sub_, platform_odometry_sub_;
};

} // namespace ariitk::tracking
