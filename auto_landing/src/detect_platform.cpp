#include <auto_landing/detect_platform.hpp>


namespace ariitk::detect {
cv::Mat platform_detect::preprocess(cv::Mat& img,
    std::vector<double>& cam_mat,
    std::vector<double>& dist_coeff,
    bool is_undistort) 
    {
	ROS_ASSERT(img.empty() != true);

	cv::Mat img_, img_hsv, blur, result;

	cv::Mat kernel = cv::Mat::eye(kernel_size_, kernel_size_, CV_8U);

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
	cv::cvtColor(img, img_hsv, CV_BGR2HSV);
	cv::GaussianBlur(img_hsv, blur, cv::Size(3, 3), 0, 0);
	cv::inRange(img,
	    cv::Scalar(int(thresholding_parameters["h"]["min"]),
	        int(thresholding_parameters["s"]["min"]),
	        int(thresholding_parameters["v"]["min"])),
	    cv::Scalar(int(thresholding_parameters["h"]["max"]),
	        int(thresholding_parameters["s"]["max"]),
	        int(thresholding_parameters["v"]["max"])),
	    result);
	cv::morphologyEx(result, result, cv::MORPH_OPEN, kernel);
	// cv::Canny(blur,result,low_threshold_canny,upper_threshold_canny);
	return result;
}

double platform_detect::crossProduct(cv::Point point, cv::Point line_start, cv::Point line_end) //Calculates Cross Product for finding the curvature
{
  return fabs((point.x - line_start.x) * (line_end.y - line_start.y) -
              (point.y - line_start.y) * (line_end.x - line_start.x));
}

double platform_detect::baseLength(cv::Point line_end, cv::Point line_start)
{
  return cv::norm(line_end - line_start);
}

double platform_detect::pointToLineDistance(const std::vector<cv::Point>& contour, int i, int n) //Calculating Curvature using the Cross Product
{
  if (i - n >= 0 && i + n < contour.size())
    return (crossProduct(contour.at(i), contour.at(i - n), contour.at(i + n)) /
            baseLength(contour.at(i + n), contour.at(i - n)));
  else if (i - n < 0 && i + n < contour.size())
    return (crossProduct(contour.at(i), contour.at(i - n + contour.size()), contour.at(i + n)) /
            baseLength(contour.at(i + n), contour.at(i - n + contour.size())));
  else if (i - n >= 0 && i + n >= contour.size())
    return (crossProduct(contour.at(i), contour.at(i - n), contour.at(i + n - contour.size())) /
            baseLength(contour.at(i + n - contour.size()), contour.at(i - n)));
}

void platform_detect::outlier_filter(const std::vector<cv::Point>& contour, const std::vector<int>& hull, //Eliminating multiple detected Hull Points within a given length
                    std::vector<cv::Point>& corners)                                     //leaving a single point with the highest curvature
{
  int i, j;
  int size = hull.size();
  int perimeter = contour.size();
  int delta = (round)(perimeter / 8.0);

  std::vector<double> distances(contour.size(), 0);
  std::vector<double> signature(contour.size(), 0);

  for (i = 0; i < size; i++)
    distances.at(hull.at(i)) = pointToLineDistance(contour, hull.at(i), delta);

  if (distances.at(0) >= distances.at(1) && distances.at(0) >= distances.at(contour.size() - 1) &&
      distances.at(0) >= 0.02 * contour.size())
    signature.at(0) = distances.at(0);

  if (distances.at(contour.size() - 1) >= distances.at(contour.size() - 2) &&
      distances.at(contour.size() - 1) >= distances.at(0) && distances.at(contour.size() - 1) >= 0.02 * contour.size())
    signature.at(contour.size() - 1) = distances.at(contour.size() - 1);

  for (i = 1; i < contour.size() - 1; i++)
  {
    if (distances.at(i) >= distances.at(i + 1) && distances.at(i) >= distances.at(i - 1) &&
        distances.at(i) >= 0.02 * contour.size())
      signature.at(i) = distances.at(i);
    else
      signature.at(i) = 0;
  }
  int x0 = 0, y0 = 0;

  for (i = 0; i < delta; i++)
  {
    if (signature.at(i) == 0)
      continue;
    else
    {
      x0 = 0;
      y0 = 0;
      for (int j = contour.size() - delta + i; j < contour.size(); j++)  // Handling cases at the end of the vector
      {
        if (signature.at(j) == 0)
          continue;
        else
        {
          if (signature.at(j) > y0)
          {
            x0 = j;
            y0 = signature.at(j);
          }
          signature.at(j) = 0;
        }
      }
      for (int j = 0; j <= delta + i; j++)  // Handling cases at the start of the vector
      {
        if (signature.at(j) == 0)
          continue;
        else
        {
          if (signature.at(j) > y0)
          {
            x0 = j;
            y0 = signature.at(j);
          }
          signature.at(j) = 0;
        }
      }
      signature.at(x0) = y0;
    }
  }

  for (i = delta; i < contour.size() - delta; i++)
  {
    if (signature.at(i) == 0)
      continue;
    else
    {
      x0 = 0;
      y0 = 0;
      for (int j = i - delta; j <= i + delta; j++)
      {
        if (signature.at(j) == 0)
          continue;
        else
        {
          if (signature.at(j) > y0)
          {
            x0 = j;
            y0 = signature.at(j);
          }
          signature.at(j) = 0;
        }
      }
      signature.at(x0) = y0;
    }
  }

  for (i = contour.size() - delta; i < contour.size(); i++)
  {
    if (signature.at(i) == 0)
      continue;
    else
    {
      x0 = 0;
      y0 = 0;
      for (int j = i - delta; j < contour.size(); j++)  // Handling cases at the end of the vector
      {
        if (signature.at(j) == 0)
          continue;
        else
        {
          if (signature.at(j) > y0)
          {
            x0 = j;
            y0 = signature.at(j);
          }
          signature.at(j) = 0;
        }
      }
      for (int j = 0; j <= delta + i - contour.size(); j++)  // Handling cases at the start of the vector
      {
        if (signature.at(j) == 0)
          continue;
        else
        {
          if (signature.at(j) > y0)
          {
            x0 = j;
            y0 = signature.at(j);
          }
          signature.at(j) = 0;
        }
      }
      signature.at(x0) = y0;
    }
  }

  for (i = 0; i < signature.size(); i++)
  {
    if (signature.at(i) != 0)
      corners.push_back(contour.at(i));
  }
}

void platform_detect::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
	nh_private.getParam("low_threshold_canny", low_threshold_canny);
	nh_private.getParam("upper_threshold_canny", upper_threshold_canny);
	nh_private.getParam("camera_matrix/data", cam_mat);
	nh_private.getParam("distortion_coefficients/data", dist_coeff);
	nh_private.getParam("is_undistort", is_undistort);
	nh_private.getParam("publish_preprocess", publish_preprocess);
	nh_private.getParam("publish_detected_platform", publish_detected_platform);
	nh_private.getParam("error_limit", error_limit);
	nh_private.getParam("contour_perimeter_thresh", contour_perimeter_thresh);
	nh_private.getParam("contour_perimeter_scale", contour_perimeter_scale);
	nh_private.getParam("thresholding_parameters", thresholding_parameters);
	nh_private.getParam("kernel_size", kernel_size_);
	image_sub = nh.subscribe("image_raw", 1, &platform_detect::imageCb, this);
	quad_height_sub = nh.subscribe("height_quad", 1, &platform_detect::heightCb, this);
	image_transport::ImageTransport it(nh);
	image_pub = it.advertise("detected_platform", 1);
	image_pub_preprocess = it.advertise("preprocessed_image", 1);
	platform_centre_pub = nh.advertise<geometry_msgs::Point>("platform_centre", 1);
}

void platform_detect::heightCb(const nav_msgs::Odometry& height_msg) {
	quad_height = height_msg.pose.pose.position.z;
}

void platform_detect::imageCb(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_ptr;
	try {cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);}
    catch (cv_bridge::Exception& e) { ROS_ERROR("cv_bridge exception: %s", e.what()); }

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
	}

	cv::findContours(processed_frame, list_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	std::vector<std::vector<int>> hull;
    std::vector<std::vector<cv::Point>> hull1;
    hull.resize(list_contours.size());

	// std::cout << list_contours.size() << std::endl;

	cv::Mat drawing = cv::Mat::zeros(frame.size(), CV_8UC3);
	std::vector<cv::Point> corners;
    std::vector<std::vector<cv::Point>> list_corners;

	for (int i = 0; i < list_contours.size(); i++) {
		// ROS_WARN("%dth contour size = %d", i, list_contours[i].size());
        list_corners.clear();
        corners.clear();
        cv::convexHull(cv::Mat(list_contours[i]), hull[i]);
        cv::convexHull(cv::Mat(list_contours[i]), hull1[i]);
         
		if (std::fabs(cv::arcLength(cv::Mat(hull1[i]), true)) <
		    contour_perimeter_thresh - (quad_height) *contour_perimeter_scale) {
			// cv::drawContours(drawing, list_contours, i, cv::Scalar(0,255,255));
			cv::drawContours(drawing,list_contours,i,cv::Scalar(121,25,87),10);
			// ROS_WARN("%dth contour is too sm	all", i);
			continue;
		}
        outlier_filter(list_contours.at(i), hull.at(i), corners);  // extract corners from hull
        list_corners.push_back(corners);
        if (!list_corners.at(0).empty() && list_corners.at(0).size() == 4)  // quadrilateral check
          {
            cv::drawContours(drawing, list_corners, 0, cv::Scalar(255, 0, 0));
            center.x=(list_corners.at(0).at(0).x + list_corners.at(0).at(1).x + list_corners.at(0).at(2).x +
                              list_corners.at(0).at(3).x) /4;
            center.y=(list_corners.at(0).at(0).y + list_corners.at(0).at(1).y + list_corners.at(0).at(2).y +
                              list_corners.at(0).at(3).y) /4;
            center.z = 0.0;
          }
    }  
				// excluding the quad shadow
		/*cv::approxPolyDP(cv::Mat(hull[i]),
		    approx,
		    cv::arcLength(cv::Mat(hull[i]), true) * error_limit,
		    true); // Approximate contour with accuracy proportional to
		           // the contour perimeter
			// std::cout << approx.size() << std::endl;

		if (corner.size() == 4) {
			double d1, d2, d3, d4;
			d1 = sqrt(pow(double(approx[0].x - approx[1].x), 2) +
			          pow(double(approx[0].y - approx[1].y), 2));
			d2 = sqrt(pow(double(approx[2].x - approx[3].x), 2) +
			          pow(double(approx[2].y - approx[3].y), 2));
			d3 = sqrt(pow(double(approx[1].x - approx[2].x), 2) +
			          pow(double(approx[1].y - approx[2].y), 2));
			d4 = sqrt(pow(double(approx[3].x - approx[0].x), 2) +
			          pow(double(approx[3].y - approx[0].y), 2));

			if (fabs(d1 - d2) < 50 && fabs(d3 - d4) < 50) {
				cv::Scalar color = cv::Scalar(255, 0, 0);
				cv::drawContours(drawing, list_contours, i, color, 1, 8, hierarchy, 0, cv::Point());
				// ROS_INFO("Platform Detected");
				cv::Rect bound_rect = cv::boundingRect(hull[i]);
				center.x = bound_rect.x + bound_rect.width / 2;
				center.y = bound_rect.y + bound_rect.height / 2;
				center.z = 0.0;
				// flag = 1;
			} 
            else {
				cv::drawContours(drawing, list_contours, i, cv::Scalar(0,0,255), 1, 8, hierarchy, 0, cv::Point());
			}
		} 
        else {
			cv::drawContours(drawing, list_contours, i, cv::Scalar(0,255,0), 1, 8, hierarchy, 0, cv::Point());
			//cv::putText(drawing, std::to_string(approx.size()), list_contours[0][0],CV_FONT_HERSHEY_PLAIN,1,cv::Scalar(0,255,0));
		}
	}**/
	if (publish_detected_platform) {
		cv_bridge::CvImage Detected_H;
		Detected_H.encoding = sensor_msgs::image_encodings::BGR8;
		Detected_H.header.stamp = ros::Time::now();
		Detected_H.image = drawing;
		image_pub.publish(Detected_H.toImageMsg());
		// ROS_INFO("detected image send");
	}
}

void platform_detect::run() { platform_centre_pub.publish(center); }
} // namespace ariitk::detect