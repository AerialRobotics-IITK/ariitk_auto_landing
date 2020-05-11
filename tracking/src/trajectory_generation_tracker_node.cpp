#include <tracking/trajectory_generation_tracker.hpp>

using namespace ariitk::tracking;

int main(int argc, char** argv) {
	ros::init(argc, argv, "trajectory_pub");
	TrajectoryGenerationTracking tracking(argv);

	ros::Rate loop_rate(tracking.publish_rate_);

	int count = 0;
	while (count < 10) {
		ros::spinOnce();
		ros::Duration(0.5).sleep();
		count++;
	}

	while (ros::ok()) {
		tracking.run();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
