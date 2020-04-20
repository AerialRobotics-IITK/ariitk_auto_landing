#pragma once

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
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
        void quadPoseCallBack(const geometry_msgs::Pose& msg);
        void setptUpdate();
        void arrayToMatrixConversion();

        int height_, count_;
        std::vector<float> distortion_matrix_, camera_to_quad_matrix_, rectification_matrix_, camera_matrix_, tcamera_;
        Eigen::Matrix3f cameraMatrix, invCameraMatrix ;
        Eigen::Matrix3f cameraToQuadMatrix ;
        Eigen::Matrix3f rectificationMatrix , quadOrientationMatrix , scaleUpMatrix ;
        Eigen::Vector3f translation_,tcam_;

        geometry_msgs::PoseStamped setpt_firefly_;
        geometry_msgs::Point pixel_coordinates_;
        geometry_msgs::Pose firefly_odom_;

        ros::Publisher set_firefly_pose_pub_;
        ros::Subscriber firefly_pose_sub_;
        ros::Subscriber firefly_pixel_coordinates_sub_;
};

} //namespace ariitk::auto_landing