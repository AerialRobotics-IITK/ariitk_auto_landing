#include <auto_landing/trajectory_publisher_visualizer.hpp>

using namespace ariitk::auto_landing;

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_pub");
    ExampleTrajectoryGeneration generator;
    
    ros::Rate loop_rate(1);

    int count = 0;

    while (ros::ok()) {
        if (count == 5) return 0;
        generator.run();
        count++;
        loop_rate.sleep();
    }
    
    return 0;
}