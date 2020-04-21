#include <auto_landing/pose.hpp>

namespace ariitk::auto_landing {

PoseEstimation::PoseEstimation()
    : height_(3) {}

void PoseEstimation::init(ros::NodeHandle& nh , ros::NodeHandle& nh_private) {

    set_firefly_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("command_pose",1);
    firefly_pose_sub_ = nh.subscribe("quad_odometry",1,&PoseEstimation::quadPoseCallBack,this);
    firefly_pixel_coordinates_sub_ = nh.subscribe("pixel_coordinates",1,&PoseEstimation::pixelCoordinatesCallBack,this);

    nh_private.getParam("C",camera_to_quad_matrix_);
    nh_private.getParam("K",camera_matrix_);
    nh_private.getParam("R",rectification_matrix_);
    nh_private.getParam("D",distortion_matrix_);
    nh_private.getParam("t",tcamera_);

    count_=0;
    scaleUpMatrix = Eigen::Matrix3f::Zero();
    setpt_firefly_.pose.position.z = height_;
    arrayToMatrixConversion();
}

void PoseEstimation::run() {
    if(count_%10==0) setptUpdate();
    set_firefly_pose_pub_.publish(setpt_firefly_);
    count_++;
}

void PoseEstimation::arrayToMatrixConversion() {
    for(int i = 0 ; i < 3 ; i++) {
        tcam_(i) = tcamera_[i]; 
        for( int j = 0 ; j < 3 ; j++) {
            cameraToQuadMatrix(i , j) = camera_to_quad_matrix_[3 * i + j ];
            cameraMatrix(i , j) = camera_matrix_[3 * i + j ]; // optical centre is aligned with physical centre of camera so neglecting fourth column
            rectificationMatrix(i , j) = rectification_matrix_[3 * i + j];
        } 
    }
    invCameraMatrix = cameraMatrix.inverse();
   }

void PoseEstimation::quadPoseCallBack(const geometry_msgs::Pose& msg) {
    firefly_odom_ = msg; 
    tf::Quaternion q(msg.orientation.x , msg.orientation.y , msg.orientation.z , msg.orientation.w );
    Eigen::Quaternionf quat = Eigen::Quaternionf(q.w(), q.x(), q.y(), q.z());
    quadOrientationMatrix = quat.normalized().toRotationMatrix(); 
    scaleUpMatrix(0,0) = scaleUpMatrix(1,1) = scaleUpMatrix(2,2) = msg.position.z -0.45;
    // std::cout<<scaleUpMatrix<<std::endl;
    Eigen::Vector3f T(msg.position.x, msg.position.y , msg.position.z);
    translation_ = T;
}

void PoseEstimation::pixelCoordinatesCallBack(const geometry_msgs::Point& msg) {
    pixel_coordinates_ = msg;
}

void PoseEstimation::setptUpdate() {
    Eigen::Vector3f pixel_coordinates(pixel_coordinates_.x , pixel_coordinates_.y , 1);
    // std::cout<<pixel_coordinates<<std::endl;
    Eigen::Vector3f coordinates_quad_frame = cameraToQuadMatrix * scaleUpMatrix * invCameraMatrix * pixel_coordinates; //gives coordinates in quad frame
    // std::cout<<"1st" <<coordinates_quad_frame<<std::endl;

    coordinates_quad_frame += tcam_;
    coordinates_quad_frame = quadOrientationMatrix * coordinates_quad_frame; // aligning axis of quad along world frame
    Eigen::Vector3f global_coordinates = coordinates_quad_frame + translation_; //adding translational vector 

    setpt_firefly_.pose.position.x = global_coordinates(0);
    setpt_firefly_.pose.position.y = global_coordinates(1);
    // setpt_firefly_.pose.position.z = 3;
    // std::cout<<"SS"<<scaleUpMatrix<<std::endl;
    std::cout <<global_coordinates<<std::endl;
    // std::cout<<invCameraMatrix<<std::endl;

}

} //namespace ariitk::autolanding 