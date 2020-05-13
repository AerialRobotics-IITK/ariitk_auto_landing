#include <tracking/tracking.hpp>

using namespace ariitk::tracking;

int main(int argc, char** argv) {
	ros::init(argc, argv, "tracking_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	Tracking mav_platform;

	mav_platform.init(nh, nh_private, argv);

	ros::Rate loop_rate(mav_platform.publish_rate);

	while (ros::ok()) {
		ros::spinOnce();
		mav_platform.run();
		loop_rate.sleep();
	}

	return 0;
}
