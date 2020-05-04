#include <pose_estimation/pose.hpp>

namespace ariitk::auto_landing {

void PoseEstimation::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
	detected_husky_odom_pub_ = nh.advertise<nav_msgs::Odometry>("detected_pose", 1);
	firefly_pose_sub_ = nh.subscribe("quad_odometry", 1, &PoseEstimation::quadPoseCallBack, this);
	firefly_pixel_coordinates_sub_ = nh.subscribe("pixel_coordinates", 1, &PoseEstimation::pixelCoordinatesCallBack, this);

	nh_private.getParam("camera_to_quad_matrix", camera_to_quad_matrix_);
	nh_private.getParam("camera_matrix/data", camera_matrix_);
	nh_private.getParam("distortion_coefficients/data", distortion_matrix_);
	nh_private.getParam("camera_translation", camera_translation_);
	nh_private.getParam("loop_rate", loop_rate);

	scaleUpMatrix = Eigen::Matrix3f::Zero();
	arrayToMatrixConversion();
}

void PoseEstimation::run() {
	huskyOdomUpdate();
	detected_husky_odom_pub_.publish(husky_odom_[1]);
}

void PoseEstimation::arrayToMatrixConversion() {
	for (int i = 0; i < 3; i++) {
		camera_translation_vector_(i) = camera_translation_[i];
		for (int j = 0; j < 3; j++) {
			cameraToQuadMatrix(i, j) = camera_to_quad_matrix_[3 * i + j];
			cameraMatrix(i, j) = camera_matrix_[3 * i + j];
		}
	}
	invCameraMatrix = cameraMatrix.inverse();
}

void PoseEstimation::quadPoseCallBack(const nav_msgs::Odometry& msg) {
	firefly_odom_ = msg;
	tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
	Eigen::Quaternionf quat = Eigen::Quaternionf(q.w(), q.x(), q.y(), q.z());
	quadOrientationMatrix = quat.normalized().toRotationMatrix();
	scaleUpMatrix(0, 0) = scaleUpMatrix(1, 1) = scaleUpMatrix(2, 2) = msg.pose.pose.position.z - 0.45;
	translation_ = Eigen::Vector3f(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
}

void PoseEstimation::pixelCoordinatesCallBack(const geometry_msgs::Point& msg) { pixel_coordinates_ = msg; }

void PoseEstimation::huskyOdomUpdate() {
	Eigen::Vector3f pixel_coordinates(pixel_coordinates_.x, pixel_coordinates_.y, 1);
	Eigen::Vector3f coordinates_quad_frame = cameraToQuadMatrix * scaleUpMatrix * invCameraMatrix * pixel_coordinates + camera_translation_vector_;
	Eigen::Vector3f global_coordinates = quadOrientationMatrix * coordinates_quad_frame + translation_;

	husky_odom_[0] = husky_odom_[1];

	husky_odom_[1].pose.pose.position.x = global_coordinates(0);
	husky_odom_[1].pose.pose.position.y = global_coordinates(1);
	husky_odom_[1].pose.pose.position.z = global_coordinates(2);

	husky_odom_[1].twist.twist.linear.x = (husky_odom_[1].pose.pose.position.x - husky_odom_[2].pose.pose.position.x) * loop_rate;
	husky_odom_[1].twist.twist.linear.y = (husky_odom_[1].pose.pose.position.y - husky_odom_[2].pose.pose.position.y) * loop_rate;
	husky_odom_[1].twist.twist.linear.z = (husky_odom_[1].pose.pose.position.z - husky_odom_[2].pose.pose.position.z) * loop_rate;
}

} // namespace ariitk::auto_landing
