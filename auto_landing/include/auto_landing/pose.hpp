#pragma once

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

namespace ariitk::auto_landing {

class PoseEstimation {
    public:
        PoseEstimation();
        void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
        void run();

    private:
        void pixelCoordinatesCallBack(const geometry_msgs::Point& msg);
        void quadPoseCallBack(const nav_msgs::Odometry& msg);
        void setptUpdate();
        void arrayToMatrixConversion();

        int height_;
        std::vector<float> distortion_matrix_;
        std::vector<float> camera_matrix_;
        std::vector<float> rotation_matrix_;
        std::vector<float> projection_matrix_;

        Eigen::Matrix3f cameraMatrix, invCameraMatrix ;
        Eigen::Matrix3f projectionMatrix , invProjectionMatrix ;
        Eigen::Matrix3f rotationMatrix , quadOrientationMatrix , scaleUpMatrix ;
        Eigen::Vector3f translation_;

        geometry_msgs::PoseStamped setpt_firefly_;
        geometry_msgs::Point pixel_coordinates_;
        nav_msgs::Odometry firefly_odom_;

        ros::Publisher set_firefly_pose_pub_;
        ros::Subscriber firefly_pose_sub_;
        ros::Subscriber firefly_pixel_coordinates_sub_;
};

} //namespace ariitk::auto_landing