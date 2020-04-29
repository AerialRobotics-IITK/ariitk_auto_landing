#include <auto_landing/husky_trajectory.hpp>

using namespace ariitk::auto_landing;

int main(int argc, char** argv) {
    ros::init(argc,argv,"husky_trajectory_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    HuskyTrajectory husky;

    husky.init(nh, nh_private, argv);
    
    ros::Rate loop_rate(10);

    while(ros::ok()) {
        ros::spinOnce();
        husky.run();
        loop_rate.sleep();
    }
    return 0;
}

