#include <pose_estimation_ros/pose_estimation_ros.hpp>

using namespace ariitk::auto_landing;

int main(int argc, char** argv) {
	ros::init(argc, argv, "pose_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	PoseEstimationROS pose;

	pose.init(nh, nh_private);

	ros::Rate loop_rate(pose.loop_rate);

	while (ros::ok()) {
		ros::spinOnce();
		pose.run();
		loop_rate.sleep();
	}
	return 0;
}