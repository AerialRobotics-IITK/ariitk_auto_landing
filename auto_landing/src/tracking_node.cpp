#include<auto_landing/tracking.hpp>

using namespace ariitk::auto_landing;

int main(int argc, char **argv){
    ros::init(argc,argv,"tracking_node");
    ros::NodeHandle nh;

    tracking mavHusky;

    mavHusky.init(nh);

    ros::Rate loop_rate(10);

    while(ros::ok()) {
        ros::spinOnce();
        mavHusky.run();
        loop_rate.sleep();
    }

    return 0;

}