#include <auto_landing/husky_trajectory.hpp>

using namespace ariitk::auto_landing;

int main(int argc, char** argv){
    ros::init(argc,argv,"husky_trajectory_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    Huskytrajectory husky;

    husky.init(nh, nh_private);

    ros::Rate loop_rate(30);

    while(ros::ok()){
        husky.run();
        loop_rate.sleep();
    }
    
    return 0;
}

