#include <tracking/trajectory_generation_tracker.hpp>

using namespace ariitk::tracking;

int main(int argc, char** argv) {
	ros::init(argc, argv, "trajectory_pub");
	TrajectoryGenerationTracking tracking(argv);

	ros::Rate loop_rate(tracking.publish_rate);

	tracking.waitForOdometry();

	while (ros::ok()) {
		tracking.run();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
