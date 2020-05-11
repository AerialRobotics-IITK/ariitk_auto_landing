#include <auto_landing/platform_trajectory.hpp>

using namespace ariitk::auto_landing;

int main(int argc, char** argv) {
	ros::init(argc, argv, "platform_trajectory_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	PlatformTrajectory platform;

	platform.init(nh, nh_private, argv);

	ros::Rate loop_rate(platform.loop_rate_);

	while (ros::ok()) {
		ros::spinOnce();
		platform.run();
		loop_rate.sleep();
	}
	return 0;
}
