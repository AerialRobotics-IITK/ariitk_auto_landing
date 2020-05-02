#pragma once

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <xmlrpcpp/XmlRpc.h>

namespace ariitk::auto_landing {
class PlatformDetect {
	private:
		image_transport::Publisher image_pub;
		image_transport::Publisher image_pub_preprocess;
		ros::Subscriber image_sub;
		ros::Subscriber quad_height_sub;
		ros::Publisher platform_centre_pub;

		std::vector<double> camera_matrix_;
		std::vector<double> distortion_coefficients_;
		bool is_undistort_;
		bool publish_preprocess_;
		bool publish_detected_platform_;
		double quad_height_;
		double error_limit_;
		int contour_perimeter_threshold_, contour_perimeter_scale_;
		int kernel_size_;
		geometry_msgs::Point center_;
		XmlRpc::XmlRpcValue thresholding_parameters_;

	public:
		void imageCallback(const sensor_msgs::ImageConstPtr& msg);
		void heightCallback(const nav_msgs::Odometry& height_msg);
		void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
		void run();
		cv::Mat preprocessImage(cv::Mat& img, std::vector<double>& camera_matrix_, std::vector<double>& distortion_coefficients_, bool is_undistort_);
};
} // namespace ariitk::auto_landing