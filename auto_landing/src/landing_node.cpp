#include<land_uav/landing.hpp>

using namespace ariitk::land_uav;

int main(int argc,char** argv){
    ros::init(argc,argv,"landing_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    Landing firefly;
    firefly.init(nh,nh_private,argv);
    ros::spin();
    if(firefly.position())firefly.run();
    //else 
    return 0; 
}
