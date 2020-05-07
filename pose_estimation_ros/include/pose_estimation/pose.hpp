#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <tf/tf.h>

namespace ariitk::auto_landing {

class PoseEstimation {
    public:
        void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
        void run();
        int loop_rate;

    private:
        void pixelCoordinatesCallBack(const geometry_msgs::Point& msg);
        void quadPoseCallBack(const nav_msgs::Odometry& msg);
        void platformStatusCallback(const std_msgs::Bool& msg);
        void huskyOdomUpdate();
        void arrayToMatrixConversion();

        std::vector<float> distortion_matrix_, camera_to_quad_matrix_, camera_matrix_, camera_translation_;
        Eigen::Matrix3f cameraMatrix, invCameraMatrix;
        Eigen::Matrix3f cameraToQuadMatrix;
        Eigen::Matrix3f quadOrientationMatrix, scaleUpMatrix;
        Eigen::Vector3f translation_, camera_translation_vector_;

        nav_msgs::Odometry husky_odom_[2];
        nav_msgs::Odometry firefly_odom_;
        geometry_msgs::Point pixel_coordinates_;
        std_msgs::Bool is_platform_detected_;

        ros::Publisher detected_husky_odom_pub_;
        ros::Subscriber firefly_pose_sub_;
        ros::Subscriber firefly_pixel_coordinates_sub_;
        ros::Subscriber is_platform_detected_sub_;
};

} // namespace ariitk::auto_landing