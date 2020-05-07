#include <detection/detect_platform.hpp>

namespace ariitk::auto_landing {
cv::Mat PlatformDetect::preprocessImage(cv::Mat& img, std::vector<double>& camera_matrix_, std::vector<double>& distortion_coefficients_, bool is_undistort_) {
	ROS_ASSERT(img.empty() != true);

	cv::Mat img_, img_hsv, blur, result;

	cv::Mat kernel = cv::Mat::eye(kernel_size_, kernel_size_, CV_8U);

	int tempIdx = 0;
	cv::Mat intrinsic = cv::Mat_<double>(3, 3);
	cv::Mat dist_coeff_ = cv::Mat_<double>(1, 5);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) { intrinsic.at<double>(i, j) = camera_matrix_.at(tempIdx++); }
	}

	for (int i = 0; i < 5; i++) { dist_coeff_.at<double>(i) = distortion_coefficients_[i]; }

	if (!is_undistort_) {
		cv::undistort(img, img_, intrinsic, dist_coeff_);
		img = img_.clone();
	}
	cv::cvtColor(img, img_hsv, CV_BGR2HSV);
	cv::GaussianBlur(img_hsv, blur, cv::Size(3, 3), 0, 0);
	cv::inRange(img,
	    cv::Scalar(int(thresholding_parameters_["h"]["min"]), int(thresholding_parameters_["s"]["min"]), int(thresholding_parameters_["v"]["min"])),
	    cv::Scalar(int(thresholding_parameters_["h"]["max"]), int(thresholding_parameters_["s"]["max"]), int(thresholding_parameters_["v"]["max"])),
	    result);
	cv::morphologyEx(result, result, cv::MORPH_OPEN, kernel);
	return result;
}

void PlatformDetect::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
	nh_private.getParam("camera_matrix/data", camera_matrix_);
	nh_private.getParam("distortion_matrix/data", distortion_coefficients_);
	nh_private.getParam("is_undistort_", is_undistort_);
	nh_private.getParam("publish_preprocess_", publish_preprocess_);
	nh_private.getParam("publish_detected_platform_", publish_detected_platform_);
	nh_private.getParam("error_limit_", error_limit_);
	nh_private.getParam("contour_perimeter_threshold_", contour_perimeter_threshold_);
	nh_private.getParam("contour_perimeter_scale_", contour_perimeter_scale_);
	nh_private.getParam("thresholding_parameters_", thresholding_parameters_);
	nh_private.getParam("kernel_size", kernel_size_);

	image_sub = nh.subscribe("image_raw", 1, &PlatformDetect::imageCallback, this);
	image_transport::ImageTransport it(nh);
	image_pub = it.advertise("detected_platform", 1);
	image_pub_preprocess = it.advertise("preprocessed_image", 1);
	platform_centre_pub = nh.advertise<geometry_msgs::Point>("platform_centre", 1);
	if_detected_pub_ = nh.advertise<std_msgs::Bool>("platform_status", 1);
}


void PlatformDetect::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) { ROS_ERROR("cv_bridge exception: %s", e.what()); }

	cv::Mat frame;
	frame = cv_ptr->image;
	ROS_ASSERT(frame.empty() != true);

	std::vector<std::vector<cv::Point>> list_contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::Mat processed_frame;
	processed_frame = preprocessImage(frame, camera_matrix_, distortion_coefficients_, is_undistort_);
		if (publish_preprocess_) {
			cv_bridge::CvImage preprocessed_img;
			preprocessed_img.encoding = sensor_msgs::image_encodings::MONO8;
			preprocessed_img.header.stamp = ros::Time::now();
			preprocessed_img.image = processed_frame;
			image_pub_preprocess.publish(preprocessed_img.toImageMsg());
		}

		cv::findContours(processed_frame, list_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		std::vector<std::vector<int>> hull;
		std::vector<std::vector<cv::Point>> hull1(list_contours.size());
		hull.resize(list_contours.size());

		cv::Mat drawing = cv::Mat::zeros(frame.size(), CV_8UC3);
		std::vector<cv::Point> corners;
		std::vector<std::vector<cv::Point>> list_corners;

		is_platform_detected.data = false;

		for (int i = 0; i < list_contours.size(); i++) {
			list_corners.clear();
			corners.clear();
			cv::convexHull(cv::Mat(list_contours[i]), hull[i]);
			cv::convexHull(cv::Mat(list_contours[i]), hull1[i]);

			if (std::fabs(cv::arcLength(cv::Mat(hull1[i]), true)) < contour_perimeter_threshold_ - quad_height_ * contour_perimeter_scale_) {
				cv::drawContours(drawing, hull1, i, cv::Scalar(0, 255, 255), 1, 8);
				continue;
			}

			cv::approxPolyDP(cv::Mat(hull1[i]), corners, cv::arcLength(cv::Mat(hull1[i]), true) * error_limit_, true);
			list_corners.push_back(corners);
			if (corners.size() == 4) {
				cv::drawContours(drawing, list_corners, 0, cv::Scalar(255, 0, 0), 1, 8);
				center_.x = (list_corners.at(0).at(0).x + list_corners.at(0).at(1).x + list_corners.at(0).at(2).x + list_corners.at(0).at(3).x) / 4;
				center_.y = (list_corners.at(0).at(0).y + list_corners.at(0).at(1).y + list_corners.at(0).at(2).y + list_corners.at(0).at(3).y) / 4;
				center_.z = 0.0;
				ROS_INFO("Platform Detected");
				is_platform_detected.data = true;
			}
		}

		if_detected_pub_.publish(is_platform_detected);

		if (publish_detected_platform_) {
			cv_bridge::CvImage Detected_H;
			Detected_H.encoding = sensor_msgs::image_encodings::BGR8;
			Detected_H.header.stamp = ros::Time::now();
			Detected_H.image = drawing;
			image_pub.publish(Detected_H.toImageMsg());
		}
}

void PlatformDetect::run() { platform_centre_pub.publish(center_); }

} // namespace ariitk::auto_landing