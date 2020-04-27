#pragma once

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <xmlrpcpp/XmlRpc.h>

namespace ariitk::detect {
class platform_detect {
	private:
		image_transport::Publisher image_pub;
		image_transport::Publisher image_pub_preprocess;
		ros::Subscriber image_sub;
		ros::Subscriber quad_height_sub;
		ros::Publisher platform_centre_pub;

		std::vector<double> cam_mat;
		std::vector<double> dist_coeff;
		bool is_undistort;
		bool publish_preprocess;
		bool publish_detected_platform;
		double quad_height;
		double error_limit;
		int contour_perimeter_thresh, contour_perimeter_scale;
		int kernel_size_;
		geometry_msgs::Point center;
		XmlRpc::XmlRpcValue thresholding_parameters;

	public:
		void imageCb(const sensor_msgs::ImageConstPtr& msg);
		void heightCb(const nav_msgs::Odometry& height_msg);
		void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
		void run();
		cv::Mat preprocess(cv::Mat& img, std::vector<double>& cam_mat, std::vector<double>& dist_coeff, bool is_undistort);
};
} // namespace ariitk::detect