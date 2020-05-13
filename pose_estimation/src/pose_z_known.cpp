#include <pose_estimation/pose.hpp>

namespace ariitk::pose_estimation {

void PoseEstimation::zKnownCalculation() {
	object_position_(0) = -(camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 0) * camera_centre_displacement_(2) *
	                              pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) *
	                              camera_centre_displacement_(1) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(1, 1) *
	                              camera_centre_displacement_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 1) *
	                              camera_centre_displacement_(1) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(1, 2) *
	                              camera_centre_displacement_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(2, 2) *
	                              camera_centre_displacement_(1) * pixel_coordinates_(0) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(0, 0) *
	                              camera_centre_displacement_(2) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(0, 1) *
	                              camera_centre_displacement_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) *
	                              camera_centre_displacement_(0) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(0, 2) *
	                              camera_centre_displacement_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 1) *
	                              camera_centre_displacement_(0) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(2, 2) *
	                              camera_centre_displacement_(0) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(0, 0) *
	                              camera_centre_displacement_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(0, 0) *
	                              camera_centre_displacement_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 0) *
	                              camera_centre_displacement_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(0, 1) *
	                              camera_centre_displacement_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(0, 1) *
	                              camera_centre_displacement_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 0) *
	                              camera_centre_displacement_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(1, 1) *
	                              camera_centre_displacement_(0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) *
	                              camera_centre_displacement_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(0, 2) *
	                              camera_centre_displacement_(1) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) *
	                              camera_centre_displacement_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(0, 2) *
	                              camera_centre_displacement_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(1, 1) *
	                              camera_centre_displacement_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(1, 2) *
	                              camera_centre_displacement_(0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 1) *
	                              camera_centre_displacement_(0) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 1) *
	                              camera_centre_displacement_(1) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(1, 2) *
	                              camera_centre_displacement_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(2, 2) *
	                              camera_centre_displacement_(0) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(2, 2) *
	                              camera_centre_displacement_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_translation_(1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_translation_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_translation_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_translation_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_translation_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_translation_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_translation_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_translation_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_translation_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_translation_(0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_translation_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_translation_(1) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(1) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(0) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(0) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(1) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(0) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(0) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(1) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(1) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(1) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(1) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(0) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(0) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(0) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(0) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(1) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(1) * pixel_coordinates_(0) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(1) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(1) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(0) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(0) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 1) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(0) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(0) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(1) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(2, 1) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(1) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 1) * vehicle_position_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 0) * vehicle_position_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 1) * vehicle_position_(0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 0) * vehicle_position_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 1) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 0) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 1) * vehicle_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 0) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 2) * vehicle_position_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 0) * vehicle_position_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 2) * vehicle_position_(0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 0) * vehicle_position_(0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 2) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 0) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 2) * vehicle_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 0) * vehicle_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 2) * vehicle_position_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 1) * vehicle_position_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 2) * vehicle_position_(0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 1) * vehicle_position_(0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(0) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(0) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(0) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(0) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 2) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 1) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 2) * vehicle_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 1) * vehicle_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(0) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(0) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(0) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(0) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(0) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(0) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(0) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(0) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 1) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 0) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 1) * object_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 0) * object_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 2) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 0) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 2) * object_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 0) * object_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * object_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * object_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 2) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 1) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 2) * object_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 1) * object_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * object_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * object_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * object_position_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * object_position_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * object_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * object_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * object_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * object_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * object_position_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * object_position_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * object_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * object_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * object_position_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * object_position_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * object_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * object_position_(2) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(0) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(0) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(0) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(0) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(0) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(0) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(0) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(0) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(0) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(0) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(0) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(0) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) * pixel_coordinates_(0) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(0) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(0) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(0) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(0) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(0) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(0) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(0) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(0) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(0) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(0) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(0) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(0) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) * pixel_coordinates_(1) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(0) * object_position_(2) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(0) * object_position_(2) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(0) * object_position_(2) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(0) * object_position_(2) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(0) * object_position_(2) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(0) * object_position_(2) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(0) * object_position_(2) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(0) * object_position_(2) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(0) * object_position_(2) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(0) * object_position_(2) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(0) * object_position_(2) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(0) * object_position_(2) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(1) * object_position_(2) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(1) * object_position_(2) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(1) * object_position_(2) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(1) * object_position_(2) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(1) * object_position_(2) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(1) * object_position_(2) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(1) * object_position_(2) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(1) * object_position_(2) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(1) * object_position_(2) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(1) * object_position_(2) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(1) * object_position_(2) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(1) * object_position_(2)) /
	                      (camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(0) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(1));

	object_position_(1) = (camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 0) * camera_centre_displacement_(2) *
	                              pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 0) *
	                              camera_centre_displacement_(1) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(1, 1) *
	                              camera_centre_displacement_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) *
	                              camera_centre_displacement_(1) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(1, 2) *
	                              camera_centre_displacement_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(2, 2) *
	                              camera_centre_displacement_(1) * pixel_coordinates_(0) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(0, 0) *
	                              camera_centre_displacement_(2) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(0, 1) *
	                              camera_centre_displacement_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 0) *
	                              camera_centre_displacement_(0) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(0, 2) *
	                              camera_centre_displacement_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) *
	                              camera_centre_displacement_(0) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(2, 2) *
	                              camera_centre_displacement_(0) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(0, 0) *
	                              camera_centre_displacement_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(0, 0) *
	                              camera_centre_displacement_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 0) *
	                              camera_centre_displacement_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(0, 1) *
	                              camera_centre_displacement_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(0, 1) *
	                              camera_centre_displacement_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 0) *
	                              camera_centre_displacement_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(1, 1) *
	                              camera_centre_displacement_(0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 0) *
	                              camera_centre_displacement_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(0, 2) *
	                              camera_centre_displacement_(1) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 0) *
	                              camera_centre_displacement_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(0, 2) *
	                              camera_centre_displacement_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(1, 1) *
	                              camera_centre_displacement_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(1, 2) *
	                              camera_centre_displacement_(0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) *
	                              camera_centre_displacement_(0) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) *
	                              camera_centre_displacement_(1) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(1, 2) *
	                              camera_centre_displacement_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(2, 2) *
	                              camera_centre_displacement_(0) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(2, 2) *
	                              camera_centre_displacement_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_translation_(1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_translation_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_translation_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_translation_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_translation_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_translation_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_translation_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_translation_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_translation_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_translation_(0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_translation_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_translation_(1) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(1) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(0) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(0) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(1) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(0) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(0) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(1) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(1) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(1) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(1) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(0) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(0) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(0) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(0) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(1) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(1) * pixel_coordinates_(0) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(1) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(1) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(0) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(0) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 0) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(0, 0) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(0) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(2, 0) * vehicle_to_camera_translation_(0) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(0, 1) *
	                              vehicle_to_camera_rotation_(2, 2) * vehicle_to_camera_translation_(1) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(2, 0) * vehicle_to_camera_rotation_(0, 2) *
	                              vehicle_to_camera_rotation_(2, 1) * vehicle_to_camera_translation_(1) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 1) * vehicle_position_(1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 0) * vehicle_position_(1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 1) * vehicle_position_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 0) * vehicle_position_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 1) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 0) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 1) * vehicle_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 0) * vehicle_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 2) * vehicle_position_(1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 0) * vehicle_position_(1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 2) * vehicle_position_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 0) * vehicle_position_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 2) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 0) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 2) * vehicle_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 0) * vehicle_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 2) * vehicle_position_(1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 1) * vehicle_position_(1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 2) * vehicle_position_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 1) * vehicle_position_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(1) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(1) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(1) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(1) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 2) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 1) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 2) * vehicle_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 1) * vehicle_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(1) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(1) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(1) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(1) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(1) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(1) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(1) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(1) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 1) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 0) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 1) * object_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 0) * object_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 2) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 0) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 2) * object_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 0) * object_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * object_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * object_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 2) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 1) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 2) * object_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 1) * object_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * object_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * object_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * object_position_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * object_position_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * object_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * object_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * object_position_(2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * object_position_(2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * object_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * object_position_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * object_position_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * object_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * object_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * object_position_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * object_position_(2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * object_position_(2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * object_position_(2) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(1) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(1) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(1) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(1) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(1) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(1) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(1) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(1) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(1) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(1) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(1) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(1) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) * pixel_coordinates_(0) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(1) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(1) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(1) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(1) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(1) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(1) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(1) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(1) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * vehicle_position_(2) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(1) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(1) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(1) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(1) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * vehicle_position_(2) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * vehicle_position_(2) * pixel_coordinates_(1) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(0) * object_position_(2) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(0) * object_position_(2) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(0) * object_position_(2) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(0) * object_position_(2) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(0) * object_position_(2) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(0) * object_position_(2) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(0) * object_position_(2) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(0) * object_position_(2) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(0) * object_position_(2) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(0) * object_position_(2) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(0) * object_position_(2) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(0) * object_position_(2) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(1) * object_position_(2) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(1) * object_position_(2) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(1) * object_position_(2) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(1) * object_position_(2) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(1) * object_position_(2) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(1) * object_position_(2) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(1) * object_position_(2) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(1) * object_position_(2) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(1) * object_position_(2) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 2) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(1) * object_position_(2) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(1) * object_position_(2) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 2) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(1) * object_position_(2)) /
	                      (camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(1, 2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(1, 2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(1, 1) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) -
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) +
	                          camera_matrix_(0, 0) * camera_matrix_(1, 2) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) +
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) -
	                          camera_matrix_(0, 2) * camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 0) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(0) -
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 1) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(0) +
	                          camera_matrix_(1, 1) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(1, 2) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(0) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(1, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(1, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 0) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(0, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 0) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 0) * ground_to_vehicle_rotation_(2, 1) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(1) +
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 1) * vehicle_to_camera_rotation_(2, 2) * pixel_coordinates_(1) -
	                          camera_matrix_(0, 0) * ground_to_vehicle_rotation_(1, 1) * ground_to_vehicle_rotation_(2, 0) *
	                              vehicle_to_camera_rotation_(0, 2) * vehicle_to_camera_rotation_(2, 1) * pixel_coordinates_(1));
}

} // namespace ariitk::pose_estimation
