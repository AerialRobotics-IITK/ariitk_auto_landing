#include<land_uav/landing.hpp>

using namespace ariitk::land_uav;

int main(int argc,char** argv){
    ros::init(argc,argv,"landing_node");
    ros::NodeHandle nh;

    Landing firefly;
    firefly.init(nh);
    ros::spin();
    firefly.run();
    return 0;
}
