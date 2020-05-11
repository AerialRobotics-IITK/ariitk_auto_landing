#include <tracking/tracking.hpp>

namespace ariitk::tracking {

void Tracking::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private, char** argv) {
	set_quad_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("command_pose", 1);
	quad_pose_sub_ = nh.subscribe("quad_odometry", 1, &Tracking::quadPoseCallback, this);

	nh_private.getParam("setpt_approximation_value", setpt_approximation_value_);
	nh_private.getParam("distance_approximation_value", distance_approximation_value_);
	nh_private.getParam("max_norm", max_norm_);
	nh_private.getParam("max_x", max_x_);
	nh_private.getParam("max_y", max_y_);
	nh_private.getParam("min_time", min_time_);
	nh_private.getParam("height", height_);
	nh_private.getParam("publish_rate", publish_rate_);
    
    inv_state_publish_rate_ = 1.0/publish_rate_;

	if (std::string(argv[2]) == "true") {
		using_detection_ = true;
		platform_odometry_sub_ = nh.subscribe("platform_odometry", 1, &Tracking::platformOdometryCallback, this);
	} else {
		using_detection_ = false;
		gazebo_model_state_sub_ = nh.subscribe("model_state", 1, &Tracking::modelStateCallback, this);
	}

	setpt_.pose.position.z = height_;
	time_[0] = ros::Time::now().toSec();
}

void Tracking::run() {
	if (!using_detection_) {
		if ((fabs(platform_cmd_vel_[0].linear.x) > 0.0001 || fabs(platform_cmd_vel_[0].linear.y) > 0.0001)) { updateSetPoint(); }

		set_quad_pose_pub_.publish(setpt_);
	} else {
		setpt_.pose.position.x = 0;
		setpt_.pose.position.y = 0;

		if (platform_detected_) {
			updateSetPoint();

			set_quad_pose_pub_.publish(setpt_);
		}
	}
}
void Tracking::modelStateCallback(const gazebo_msgs::ModelStates& msg) {
	int index = 0;
	std::string name = msg.name[index];
	while (name != "/") {
		index++;
		name = msg.name[index];
	}

	platform_odom_.pose.pose.position.x = msg.pose[index].position.x;
	platform_odom_.pose.pose.position.y = msg.pose[index].position.y;
	platform_odom_.pose.pose.position.z = msg.pose[index].position.z;

	platform_cmd_vel_[0].linear.x = platform_cmd_vel_[1].linear.x;
	platform_cmd_vel_[0].linear.x = platform_cmd_vel_[1].linear.x;
	platform_cmd_vel_[0].linear.x = platform_cmd_vel_[1].linear.x;

	platform_cmd_vel_[1].linear.x = msg.twist[index].linear.x;
	platform_cmd_vel_[1].linear.y = msg.twist[index].linear.y;
	platform_cmd_vel_[1].linear.z = msg.twist[index].linear.z;
}

void Tracking::platformOdometryCallback(const nav_msgs::Odometry& msg) {
	platform_odom_ = msg;
	platform_detected_ = true;

	platform_cmd_vel_[0].linear.x = platform_cmd_vel_[1].linear.x;
	platform_cmd_vel_[0].linear.x = platform_cmd_vel_[1].linear.x;
	platform_cmd_vel_[0].linear.x = platform_cmd_vel_[1].linear.x;

	platform_cmd_vel_[1].linear.x = msg.twist.twist.linear.x;
	platform_cmd_vel_[1].linear.y = msg.twist.twist.linear.y;
	platform_cmd_vel_[1].linear.z = msg.twist.twist.linear.z;
}

void Tracking::quadPoseCallback(const nav_msgs::Odometry& msg) { quad_odom_ = msg; }
void Tracking::updateSetPoint() {
	double distance_x = platform_odom_.pose.pose.position.x - quad_odom_.pose.pose.position.x;
	double distance_y = platform_odom_.pose.pose.position.y - quad_odom_.pose.pose.position.y;
	double norm = sqrt(pow(distance_x, 2) + pow(distance_y, 2));

	if ((fabs(distance_x) > max_x_) || (fabs(distance_y) > max_y_)) {
		setpt_.pose.position.x = platform_odom_.pose.pose.position.x;
		setpt_.pose.position.y = platform_odom_.pose.pose.position.y;
		time_[0] = ros::Time::now().toSec();
	}

	else if ((fabs(distance_x) < max_x_) && (fabs(distance_y) < max_y_)) {
		time_[1] = ros::Time::now().toSec();
		if ((time_[1] - time_[0]) > min_time_) {
			int approximation_value[2] = {setpt_approximation_value_, distance_approximation_value_};
			if (norm > max_norm_) { approximation_value[1] = distance_approximation_value_ - norm; }
			double x_approx = platform_cmd_vel_[1].linear.x * inv_state_publish_rate_ +
			                  0.5 * (platform_cmd_vel_[1].linear.x - platform_cmd_vel_[0].linear.x) * pow(inv_state_publish_rate_, 2);
			double y_approx = platform_cmd_vel_[1].linear.y * inv_state_publish_rate_ +
			                  0.5 * (platform_cmd_vel_[1].linear.y - platform_cmd_vel_[0].linear.y) * pow(inv_state_publish_rate_, 2);

			double setpt_approximation_x = platform_odom_.pose.pose.position.x + x_approx - quad_odom_.pose.pose.position.x;
			double setpt_approximation_y = platform_odom_.pose.pose.position.y + y_approx - quad_odom_.pose.pose.position.y;

			setpt_.pose.position.x =
			    platform_odom_.pose.pose.position.x + approximation_value[0] * x_approx + approximation_value[1] * setpt_approximation_x;
			setpt_.pose.position.y =
			    platform_odom_.pose.pose.position.y + approximation_value[0] * y_approx + approximation_value[1] * setpt_approximation_y;
		} else {
			setpt_.pose.position.x = platform_odom_.pose.pose.position.x;
			setpt_.pose.position.y = platform_odom_.pose.pose.position.y;
		}
	}

}

} // namespace ariitk::tracking
