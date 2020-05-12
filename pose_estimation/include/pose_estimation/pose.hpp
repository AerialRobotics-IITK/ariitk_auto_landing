#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <tf/tf.h>

namespace ariitk::pose_estimation {

enum class UseCase{ scaleup_known, z_known };
class PoseEstimation {
	public:
        PoseEstimation(){};
        PoseEstimation(Eigen::MatrixXd projection_matrix, Eigen::Matrix3d camera_to_vehicle_rotation, Eigen::Vector3d vehicle_to_camera_translation);
        void setCameraParams(Eigen::MatrixXd projection_matrix, Eigen::Matrix3d camera_to_vehicle_rotation, Eigen::Vector3d vehicle_to_camera_translation);
        void setVehicleParams(Eigen::Matrix3d ground_to_vehicle_rotation, Eigen::Vector3d vehicle_position);
        void setObjectParams(Eigen::Matrix3d scale_up_matrix, Eigen::Vector3d pixel_coordinates);
        void setObjectParams(double z_coordinate, Eigen::Vector3d pixel_coordinates);
        Eigen::Vector3d getObjectPosition();

	private:
        void scaleupKnownCalculation();
        void zKnownCalculation();

        Eigen::Matrix3d scale_up_matrix_;
        Eigen::Matrix3d camera_matrix_;
        Eigen::Matrix3d inverse_camera_matrix_;
        Eigen::Matrix3d camera_to_vehicle_rotation_;
        Eigen::Matrix3d ground_to_vehicle_rotation_;
        Eigen::Matrix3d vehicle_to_camera_rotation_;
        Eigen::Matrix3d vehicle_to_ground_rotation_;

        Eigen::Vector3d object_position_;
        Eigen::Vector3d vehicle_position_;
        Eigen::Vector3d vehicle_to_camera_translation_;
        Eigen::Vector3d camera_centre_displacement_;
        Eigen::Vector3d pixel_coordinates_;

        bool is_data_changed_;
        
        UseCase use_case;
};

} // namespace ariitk::pose_estimation
