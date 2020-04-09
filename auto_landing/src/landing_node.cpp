#include<auto_landing/landing.hpp>

using namespace ariitk::auto_landing;

int main(int argc,char** argv) {
    ros::init(argc,argv,"landing_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    Landing mav;
    mav.init(nh, nh_private, argv);

    ros::Rate loop_rate(10);

    int count=0;
    while (count<5) {
        ros::spinOnce();
        count++;
        ros::Duration(1).sleep();
    }

    while (ros::ok()) {
        mav.run();
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0; 
}
