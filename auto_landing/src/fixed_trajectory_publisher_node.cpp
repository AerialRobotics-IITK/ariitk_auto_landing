#include <auto_landing/fixed_trajectory_publisher.hpp>

using namespace ariitk::auto_landing;

int main(int argc, char** argv) {
    ros::init (argc, argv, "trajectory_pub");
    TrajectoryPublisher publisher;

    ros::Rate loop_rate(1);
    // int count = 0;

    while (ros::ok()) {
        publisher.run();
        // count++;
        loop_rate.sleep();
    }

    return 0;
}