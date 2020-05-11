#include <pose_estimation_ros/pose_estimation_ros.hpp>

namespace ariitk::auto_landing {

void PoseEstimationROS::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
	detected_platform_odom_pub_ = nh.advertise<nav_msgs::Odometry>("detected_pose", 1);
	quad_pose_sub_ = nh.subscribe("quad_odometry", 1, &PoseEstimationROS::quadPoseCallBack, this);
	quad_pixel_coordinates_sub_ = nh.subscribe("platform_centre", 1, &PoseEstimationROS::pixelCoordinatesCallBack, this);
	is_platform_detected_sub_ = nh.subscribe("platform_status", 1, &PoseEstimationROS::platformStatusCallback, this);

	nh_private.getParam("camera_to_quad_matrix", camera_to_quad_matrix_);
	nh_private.getParam("camera_matrix/data", camera_matrix_);
	nh_private.getParam("distortion_coefficients/data", distortion_matrix_);
	nh_private.getParam("camera_translation", camera_translation_);
	nh_private.getParam("loop_rate", loop_rate);
	nh_private.getParam("platform_height", platform_height_);

	ROS_ERROR("THIS IS THE LOOP RATE = %ld", loop_rate);

	scaleUpMatrix = Eigen::Matrix3d::Zero();

	platform_odom_[0].pose.pose.position.x = 0;
	platform_odom_[0].pose.pose.position.y = 0;
	platform_odom_[0].pose.pose.position.z = 0;

	platform_odom_[1].pose.pose.position.x = 0;
	platform_odom_[1].pose.pose.position.y = 0;
	platform_odom_[1].pose.pose.position.z = 0;

	platform_odom_[1].twist.twist.linear.x = 0;
	platform_odom_[1].twist.twist.linear.y = 0;
	platform_odom_[1].twist.twist.linear.z = 0;

	arrayToMatrixConversion();
}

void PoseEstimationROS::run() {
	platformOdomUpdate();
	detected_platform_odom_pub_.publish(platform_odom_[1]);
}

void PoseEstimationROS::arrayToMatrixConversion() {
	for (int i = 0; i < 3; i++) {
		camera_translation_vector_(i) = camera_translation_[i];
		for (int j = 0; j < 3; j++) {
			cameraToQuadMatrix(i, j) = camera_to_quad_matrix_[3 * i + j];
			cameraMatrix(i, j) = camera_matrix_[3 * i + j];
		}
	}
	invCameraMatrix = cameraMatrix.inverse();
  
	pose_object_.setCameraParams(cameraMatrix, cameraToQuadMatrix, camera_translation_vector_);

}

void PoseEstimationROS::quadPoseCallBack(const nav_msgs::Odometry& msg) {
	quad_odom_ = msg;
	tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
	Eigen::Quaterniond quat = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
	quadOrientationMatrix = quat.normalized().toRotationMatrix().inverse();
	scaleUpMatrix(0, 0) = scaleUpMatrix(1, 1) = scaleUpMatrix(2, 2) = msg.pose.pose.position.z - 0.45;
	translation_ = Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
}

void PoseEstimationROS::pixelCoordinatesCallBack(const geometry_msgs::Point& msg) { pixel_coordinates_ = msg; }

void PoseEstimationROS::platformStatusCallback(const std_msgs::Bool& msg) { is_platform_detected_ = msg; }

void PoseEstimationROS::platformOdomUpdate() {
	if(is_platform_detected_.data) {
		Eigen::Vector3d pixel_coordinates(pixel_coordinates_.x, pixel_coordinates_.y, 1);
		pose_object_.setVehicleParams(quadOrientationMatrix, translation_);
		pose_object_.setObjectParams(platform_height_, pixel_coordinates);
		Eigen::Vector3d global_coordinates = pose_object_.getObjectPosition();

		platform_odom_[0] = platform_odom_[1];

		platform_odom_[1].pose.pose.position.x = global_coordinates(0);
		platform_odom_[1].pose.pose.position.y = global_coordinates(1);
		platform_odom_[1].pose.pose.position.z = global_coordinates(2);
		platform_odom_[1].header.stamp = ros::Time::now();

		platform_odom_[1].twist.twist.linear.x = (platform_odom_[1].pose.pose.position.x - platform_odom_[0].pose.pose.position.x) * loop_rate;
		platform_odom_[1].twist.twist.linear.y = (platform_odom_[1].pose.pose.position.y - platform_odom_[0].pose.pose.position.y) * loop_rate;
		platform_odom_[1].twist.twist.linear.z = (platform_odom_[1].pose.pose.position.z - platform_odom_[0].pose.pose.position.z) * loop_rate;
	} //else {
	// 	nav_msgs::Odometry temp = platform_odom_[0];

	// 	platform_odom_[0] = platform_odom_[1];

	// 	platform_odom_[1].pose.pose.position.x = temp.pose.pose.position.x + (temp.twist.twist.linear.x / loop_rate);
	// 	platform_odom_[1].pose.pose.position.y = temp.pose.pose.position.y + (temp.twist.twist.linear.y / loop_rate);
	// 	platform_odom_[1].pose.pose.position.z = temp.pose.pose.position.z + (temp.twist.twist.linear.z / loop_rate);
			// platform_odom_[1].twist.twist.linear.x = 0;
			// platform_odom_[1].twist.twist.linear.y = 0;
			// platform_odom_[1].twist.twist.linear.z = 0;
	// }
}

} // namespace ariitk::auto_landing
