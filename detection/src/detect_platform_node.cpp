#include <detection/detect_platform.hpp>

using namespace ariitk::detection;

int main(int argc, char** argv) {
	ros::init(argc, argv, "detect_platform_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	PlatformDetect detect;

	detect.init(nh, nh_private);

	ros::Rate loopRate(10);

	while (ros::ok()) {
		detect.run();
		ros::spinOnce();
		loopRate.sleep();
	}
	return 0;
}