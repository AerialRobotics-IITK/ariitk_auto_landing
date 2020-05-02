#include <pose_estimation/pose.hpp>

using namespace ariitk::auto_landing;

int main(int argc, char** argv) {
	ros::init(argc, argv, "pose_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	PoseEstimation pose;

	pose.init(nh, nh_private);

	ros::Rate loop_rate(10);

	while (ros::ok()) {
		ros::spinOnce();
		pose.run();
		loop_rate.sleep();
	}
	return 0;
}