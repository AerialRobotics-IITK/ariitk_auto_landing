#include <auto_landing/detect_platform.hpp>

namespace ariitk::detect {
cv::Mat platform_detect::preprocess(cv::Mat& img,
    std::vector<double>& cam_mat,
    std::vector<double>& dist_coeff,
    bool is_undistort) {
	ROS_ASSERT(img.empty() != true);

	cv::Mat img_, gray, blur, result;

	int tempIdx = 0;
	cv::Mat intrinsic = cv::Mat_<double>(3, 3);
	cv::Mat dist_coeff_ = cv::Mat_<double>(1, 5);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			intrinsic.at<double>(i, j) = cam_mat.at(tempIdx++);
		}
	}

	for (int i = 0; i < 5; i++) {
		dist_coeff_.at<double>(i) = dist_coeff[i];
	}

	if (!is_undistort) {
		cv::undistort(img, img_, intrinsic, dist_coeff_);
		img = img_.clone();
	}
	cv::cvtColor(img, gray, CV_BGR2GRAY);
	cv::GaussianBlur(gray, blur, cv::Size(3, 3), 0, 0);
	cv::Canny(blur, result, low_threshold_canny, upper_threshold_canny);
	return result;
}

void platform_detect::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
	nh_private.getParam("low_threshold_canny", low_threshold_canny);
	nh_private.getParam("upper_threshold_canny", upper_threshold_canny);
	nh_private.getParam("camera_matrix/data", cam_mat);
	nh_private.getParam("distortion_coefficients/data", dist_coeff);
	nh_private.getParam("is_undistort", is_undistort);
	nh_private.getParam("publish_preprocess", publish_preprocess);
	nh_private.getParam(
	    "publish_detected_platform", publish_detected_platform);
	image_sub =
	    nh.subscribe("image_raw", 1, &platform_detect::imageCb, this);
	image_transport::ImageTransport it(nh);
	image_pub = it.advertise("detected_platform", 1);
	image_pub_preprocess = it.advertise("preprocessed_image", 1);
}

void platform_detect::imageCb(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(
		    msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

	cv::Mat frame;
	frame = cv_ptr->image;
	ROS_ASSERT(frame.empty() != true);

	std::vector<std::vector<cv::Point>> list_contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::Mat processed_frame;
	processed_frame = preprocess(frame, cam_mat, dist_coeff, is_undistort);
	if (publish_preprocess) {
		cv_bridge::CvImage preprocessed_img;
		preprocessed_img.encoding = sensor_msgs::image_encodings::MONO8;
		preprocessed_img.header.stamp = ros::Time::now();
		preprocessed_img.image = processed_frame;
		image_pub_preprocess.publish(preprocessed_img.toImageMsg());
		ROS_INFO("preprocessed image send");
	}

	cv::findContours(processed_frame,
	    list_contours,
	    hierarchy,
	    CV_RETR_EXTERNAL,
	    CV_CHAIN_APPROX_SIMPLE);
	std::vector<std::vector<cv::Point>> hull(list_contours.size());

	for (int i = 0; i < list_contours.size(); i++)
		cv::convexHull(cv::Mat(list_contours[i]), hull[i]);

	cv::Mat drawing = cv::Mat::zeros(frame.size(), CV_8UC3);

	for (int i = 0; i < list_contours.size(); i++) {
		cv::Scalar color_contours = cv::Scalar(0, 255, 0);
		cv::Scalar color = cv::Scalar(255, 0, 0);
		if (std::fabs(cv::contourArea(hull[i])) < 6000)
			continue;
		cv::drawContours(drawing,
		    list_contours,
		    i,
		    color_contours,
		    1,
		    8,
		    hierarchy,
		    0,
		    cv::Point());
		cv::drawContours(
		    drawing, hull, i, color, 1, 8, hierarchy, 0, cv::Point());
	}

	std::vector<cv::Point> approx;
	for (int i = 0; i < hull.size(); i++) {
		if (std::fabs(cv::contourArea(hull[i])) <
		    6000) // excluding the quad shadow
			continue;
		cv::approxPolyDP(cv::Mat(hull[i]),
		    approx,
		    cv::arcLength(cv::Mat(hull[i]), true) * 0.1,
		    true); // Approximate contour with accuracy proportional to
		           // the contour perimeter
		if (approx.size() == 4) {
			double d1, d2, d3, d4;
			d1 = sqrt(pow(double(approx[0].x - approx[1].x), 2) +
			          pow(double(approx[0].y - approx[1].y), 2));
			d2 = sqrt(pow(double(approx[2].x - approx[3].x), 2) +
			          pow(double(approx[2].y - approx[3].y), 2));
			d3 = sqrt(pow(double(approx[1].x - approx[2].x), 2) +
			          pow(double(approx[1].y - approx[2].y), 2));
			d4 = sqrt(pow(double(approx[3].x - approx[0].x), 2) +
			          pow(double(approx[3].y - approx[0].y), 2));

			if (fabs(d1 - d2) < 10 && fabs(d3 - d4) < 10) {
				ROS_INFO("Platform Detected");
				if (publish_detected_platform) {
					cv_bridge::CvImage Detected_H;
					Detected_H.encoding =
					    sensor_msgs::image_encodings::BGR8;
					Detected_H.header.stamp =
					    ros::Time::now();
					Detected_H.image = frame;
					image_pub.publish(
					    Detected_H.toImageMsg());
					ROS_INFO("detected image send");
				}
			}
		}
	}
}
} // namespace ariitk::detect
