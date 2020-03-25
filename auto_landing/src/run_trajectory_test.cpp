#include <auto_landing/trajectory_test.hpp>

using namespace ariitk::auto_landing;

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_pub");
    ExampleTrajectoryGeneration generator;
    
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        generator.run();
        loop_rate.sleep();
    }
    
    return 0;
}