#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

namespace ariitk::detect
{
       class platform_detect
       {
       private:
              image_transport::Publisher image_pub;
              image_transport::Publisher image_pub_preprocess;
              ros::Subscriber image_sub;

              std::vector<double> cam_mat;
              std::vector<double> dist_coeff;
              bool is_undistort;
              bool publish_preprocess;
              bool publish_detected_platform;
              int low_threshold_canny;
              int upper_threshold_canny;

       public:
              void imageCb(const sensor_msgs::ImageConstPtr& msg);
              void init(ros::NodeHandle& nh,ros::NodeHandle& nh_private);
              cv::Mat preprocess(cv::Mat& img, std::vector<double>& cam_mat,std::vector<double>& dist_coeff, bool is_undistort);
       };
}