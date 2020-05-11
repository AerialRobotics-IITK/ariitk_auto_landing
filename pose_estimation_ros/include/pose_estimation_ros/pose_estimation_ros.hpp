#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>

namespace ariitk::auto_landing {

class PoseEstimationROS {
    public:
        void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
        void run();
        int loop_rate;

    private:
        void pixelCoordinatesCallBack(const geometry_msgs::Point& msg);
        void quadPoseCallBack(const nav_msgs::Odometry& msg);
        void platformOdomUpdate();
        void arrayToMatrixConversion();

        std::vector<float> distortion_matrix_, camera_to_quad_matrix_, camera_matrix_, camera_translation_;
        Eigen::Matrix3f cameraMatrix, invCameraMatrix;
        Eigen::Matrix3f cameraToQuadMatrix;
        Eigen::Matrix3f quadOrientationMatrix, scaleUpMatrix;
        Eigen::Vector3f translation_, camera_translation_vector_;

        double platform_height_;

        nav_msgs::Odometry platform_odom_[2];
        nav_msgs::Odometry quad_odom_;
        geometry_msgs::Point pixel_coordinates_;

        ros::Publisher detected_platform_odom_pub_;
        ros::Subscriber quad_pose_sub_;
        ros::Subscriber quad_pixel_coordinates_sub_;
};

} // namespace ariitk::auto_landing