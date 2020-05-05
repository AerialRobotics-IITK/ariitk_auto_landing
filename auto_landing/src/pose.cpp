#include <auto_landing/pose.hpp>

namespace ariitk::auto_landing {

PoseEstimation::PoseEstimation()
    : height_(1) {}

void PoseEstimation::init(ros::NodeHandle& nh , ros::NodeHandle& nh_private) {

    set_firefly_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("command_pose",1);
    firefly_pose_sub_ = nh.subscribe("quad_odometry",1,&PoseEstimation::quadPoseCallBack,this);
    firefly_pixel_coordinates_sub_ = nh.subscribe("pixel_coordinates",1,&PoseEstimation::pixelCoordinatesCallBack,this);

    nh_private.getParam("K",camera_matrix_);
    nh_private.getParam("P",projection_matrix_);
    nh_private.getParam("R",rotation_matrix_);
    nh_private.getParam("D",distortion_matrix_);

    scaleUpMatrix = Eigen::Matrix3f::Zero();
    setpt_firefly_.pose.position.z = height_;
    arrayToMatrixConversion();
}

void PoseEstimation::run() {
    setptUpdate();
    set_firefly_pose_pub_.publish(setpt_firefly_);
}

void PoseEstimation::arrayToMatrixConversion() {
    for(int i = 0 ; i < 3 ; i++) {
        for( int j = 0 ; j < 3 ; j++) {
            cameraMatrix(i , j) = camera_matrix_[3 * i + j ];
            projectionMatrix(i , j) = projection_matrix_[4 * i + j ]; // optical centre is aligned with physical centre of camera so neglecting fourth column
            rotationMatrix(i , j) = rotation_matrix_[3 * i + j];
        } 
    }
    invCameraMatrix = cameraMatrix.inverse();
    invProjectionMatrix = projectionMatrix.inverse();
}

void PoseEstimation::quadPoseCallBack(const nav_msgs::Odometry& msg) {
    firefly_odom_ = msg; 
    tf::Quaternion q(firefly_odom_.pose.pose.orientation.x , firefly_odom_.pose.pose.orientation.y , firefly_odom_.pose.pose.orientation.z , firefly_odom_.pose.pose.orientation.w );
    Eigen::Quaternionf quat = Eigen::Quaternionf(q.w(), q.x(), q.y(), q.z());
    quadOrientationMatrix = quat.normalized().toRotationMatrix(); 
    scaleUpMatrix(0,0) = scaleUpMatrix(1,1) = scaleUpMatrix(2,2) = msg.pose.pose.position.z;
    Eigen::Vector3f T(firefly_odom_.pose.pose.position.x , firefly_odom_.pose.pose.position.y , firefly_odom_.pose.pose.position.z);
    translation_ = T;
}

void PoseEstimation::pixelCoordinatesCallBack(const geometry_msgs::Point& msg) {
    pixel_coordinates_ = msg;
}

void PoseEstimation::setptUpdate() {
    Eigen::Vector3f pixel_coordinates(pixel_coordinates_.x , pixel_coordinates_.y , 1);
    Eigen::Vector3f coordinates_quad_frame = cameraMatrix * scaleUpMatrix * rotationMatrix * projectionMatrix * pixel_coordinates; //gives coordinates in quad frame
    // translationalCameraMatrix is taken to be zero as distance between camera and quad frame is just negligible
    coordinates_quad_frame = quadOrientationMatrix * coordinates_quad_frame; // aligning axis of quad along world frame
    Eigen::Vector3f global_coordinates = coordinates_quad_frame + translation_; //adding translational vector 

    setpt_firefly_.pose.position.x = global_coordinates(0);
    setpt_firefly_.pose.position.y = global_coordinates(1);
    setpt_firefly_.pose.position.z = global_coordinates(2);
    }

} //namespace ariitk::autolanding 