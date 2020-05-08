#include <pose_estimation/pose.hpp>

namespace ariitk::pose_estimation {

PoseEstimation::PoseEstimation(Eigen::MatrixXd projection_matrix, Eigen::Matrix3d camera_to_vehicle_rotation, Eigen::Vector3d vehicle_to_camera_translation) {
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) { camera_matrix_(i,j) = projection_matrix(i,j); }
        if (projection_matrix.cols() == 4) { camera_centre_displacement_(i) = projection_matrix(i,3); }
        else { camera_centre_displacement_(i) = 0; }
    }
    inverse_camera_matrix_ = camera_matrix_.inverse();

    camera_to_vehicle_rotation_ = camera_to_vehicle_rotation;
    vehicle_to_ground_rotation_ = ground_to_vehicle_rotation_.inverse();
    vehicle_to_camera_translation_ = vehicle_to_camera_translation;
    
    is_data_changed_ = true;
}

void PoseEstimation::setCameraParams(Eigen::MatrixXd projection_matrix, Eigen::Matrix3d camera_to_vehicle_rotation, Eigen::Vector3d vehicle_to_camera_translation) {
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) { camera_matrix_(i,j) = projection_matrix(i,j); }
        if (projection_matrix.cols() == 4) { camera_centre_displacement_(i) = projection_matrix(i,3); }
        else { camera_centre_displacement_(i) = 0; }
    }
    inverse_camera_matrix_ = camera_matrix_.inverse();

    camera_to_vehicle_rotation_ = camera_to_vehicle_rotation;
    vehicle_to_camera_rotation_ = camera_to_vehicle_rotation_.inverse();
    vehicle_to_camera_translation_ = vehicle_to_camera_translation;

    is_data_changed_ = true;
}

void PoseEstimation::setVehicleParams(Eigen::Matrix3d ground_to_vehicle_rotation, Eigen::Vector3d vehicle_position) {
    ground_to_vehicle_rotation_ = ground_to_vehicle_rotation;
    vehicle_to_camera_rotation_ = camera_to_vehicle_rotation_.inverse();
    vehicle_position_ = vehicle_position;

    is_data_changed_ = true;
}

void PoseEstimation::setObjectParams(Eigen::Matrix3d scale_up_matrix, Eigen::Vector3d pixel_coordinates) {
    scale_up_matrix_ = scale_up_matrix;
    estimation_type = scaleup_known;
    pixel_coordinates_ = pixel_coordinates;

    is_data_changed_ = true;
}

void PoseEstimation::setObjectParams(double z_coordinate, Eigen::Vector3d pixel_coordinates) {
    object_position_(2) = z_coordinate;
    estimation_type = z_known;
    pixel_coordinates_ = pixel_coordinates;

    is_data_changed_ = true;
}

Eigen::Vector3d PoseEstimation::getObjectPosition() {
    if (!is_data_changed_) return object_position_;
    if (estimation_type == z_known) {
        zKnownCalculation();
        is_data_changed_ = false;
        return object_position_;
    } else {
        scaleupKnownCalculation();
        is_data_changed_ = false;
        return object_position_;
    }
}

void PoseEstimation::scaleupKnownCalculation() {
    object_position_ = (vehicle_to_ground_rotation_ * ((camera_to_vehicle_rotation_ * ((scale_up_matrix_ * inverse_camera_matrix_ * pixel_coordinates_) + camera_centre_displacement_)) + vehicle_to_camera_translation_)) + vehicle_position_;
}

}