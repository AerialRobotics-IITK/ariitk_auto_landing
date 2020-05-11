#include <auto_landing/platform_trajectory.hpp>

namespace ariitk::auto_landing {

void PlatformTrajectory::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, char** argv) {
	trajectory_pub_ = nh.advertise<geometry_msgs::Twist>("command_velocity", 10);
	gazebo_model_state_sub_ = nh.subscribe("model_state", 1, &PlatformTrajectory::modelStateCallback, this);

	nh_private.getParam("linear_vel", linear_velocity_);
	nh_private.getParam("ang_vel", angular_velocity_);
	nh_private.getParam("min_time", min_time_);
	nh_private.getParam("platform_y", platform_y_);
	nh_private.getParam("platform_x", platform_x_);
	nh_private.getParam("loop_rate", loop_rate_);

	std::string choices[3] = {"linear", "circular", "eight"};
	if (std::string(argv[2]) == "random") { trajectory_name_ = choices[rand() % 3]; } 
	else { trajectory_name_ = argv[2]; }
	
	time_[0] = 0;
	count_ = 0;
	t_inverse = 1 / loop_rate_;
}

void PlatformTrajectory::run() {
	if (trajectory_name_ == "linear") {	linearTrajectory();	}
	else if (trajectory_name_ == "eight") { eightTrajectory(); }
	else { circularTrajectory(); }
	
	trajectory_pub_.publish(platform_velocity_);
}

void PlatformTrajectory::modelStateCallback(const gazebo_msgs::ModelStates& msg) {
	int index = 0;

	while (msg.name[index++] != "/") {}

	platform_odom_.pose.pose.position.x = msg.pose[index - 1].position.x;
	platform_odom_.pose.pose.position.y = msg.pose[index - 1].position.y;
}
void PlatformTrajectory::linearTrajectory() { platform_velocity_.linear.x = linear_velocity_; }

void PlatformTrajectory::circularTrajectory() {
	platform_velocity_.linear.x = linear_velocity_;
	platform_velocity_.angular.z = angular_velocity_;
}

void PlatformTrajectory::eightTrajectory() {
	double norm = sqrt(pow(platform_odom_.pose.pose.position.x - platform_x_, 2) + pow(platform_odom_.pose.pose.position.y - platform_y_, 2));

	time_[1] = ros::Time::now().toSec();

	if ((norm <= (linear_velocity_ * t_inverse)) && (time_[1] - time_[0]) > min_time_) {
		time_[0] = ros::Time::now().toSec();
		count_++;
	}

	if (count_ % 2 == 0) {
		platform_velocity_.linear.x = linear_velocity_;
		platform_velocity_.angular.z = angular_velocity_;
	} else {
		platform_velocity_.linear.x = linear_velocity_;
		platform_velocity_.angular.z = -1 * angular_velocity_;
	}
}

} // namespace ariitk::auto_landing
