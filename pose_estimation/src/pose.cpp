#include <pose_estimation/pose.hpp>

namespace ariitk::auto_landing {

void PoseEstimation::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
	detected_husky_odom_pub_ = nh.advertise<geometry_msgs::PoseStamped>("detected_pose", 1);
	firefly_pose_sub_ = nh.subscribe("quad_odometry", 1, &PoseEstimation::quadPoseCallBack, this);
	firefly_pixel_coordinates_sub_ =
	    nh.subscribe("pixel_coordinates", 1, &PoseEstimation::pixelCoordinatesCallBack, this);

	nh_private.getParam("camera_to_quad_matrix", camera_to_quad_matrix_);
	nh_private.getParam("camera_matrix", camera_matrix_);
	nh_private.getParam("distortion_matrix", distortion_matrix_);
	nh_private.getParam("camera_translation", camera_translation_);

	scaleUpMatrix = Eigen::Matrix3f::Zero();
	arrayToMatrixConversion();
}

void PoseEstimation::run() {
	huskyOdomUpdate();
	detected_husky_odom_pub_.publish(husky_odom_);
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

void PoseEstimation::quadPoseCallBack(const geometry_msgs::Pose& msg) {
	firefly_odom_ = msg;
	tf::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
	Eigen::Quaternionf quat = Eigen::Quaternionf(q.w(), q.x(), q.y(), q.z());
	quadOrientationMatrix = quat.normalized().toRotationMatrix();
	scaleUpMatrix(0, 0) = scaleUpMatrix(1, 1) = scaleUpMatrix(2, 2) = msg.position.z - 0.45;
	translation_ = Eigen::Vector3f(msg.position.x, msg.position.y, msg.position.z);
}

void PoseEstimation::pixelCoordinatesCallBack(const geometry_msgs::Point& msg) { pixel_coordinates_ = msg; }

void PoseEstimation::huskyOdomUpdate() {
	Eigen::Vector3f pixel_coordinates(pixel_coordinates_.x, pixel_coordinates_.y, 1);
	Eigen::Vector3f coordinates_quad_frame =
	    cameraToQuadMatrix * scaleUpMatrix * invCameraMatrix * pixel_coordinates + camera_translation_vector_;
	Eigen::Vector3f global_coordinates = quadOrientationMatrix * coordinates_quad_frame + translation_;

	husky_odom_.pose.position.x = global_coordinates(0);
	husky_odom_.pose.position.y = global_coordinates(1);
	husky_odom_.pose.position.z = global_coordinates(2);
}

} // namespace ariitk::auto_landing
