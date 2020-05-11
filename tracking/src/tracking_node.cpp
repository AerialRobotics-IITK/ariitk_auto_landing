#include <tracking/tracking.hpp>

using namespace ariitk::auto_landing;

int main(int argc, char** argv) {
    ros::init(argc, argv, "tracking_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    Tracking mav_platform;

    mav_platform.init(nh, nh_private, argv);

    ros::Rate loop_rate(10);

    while(ros::ok()) {
        ros::spinOnce();
        mav_platform.run();
        loop_rate.sleep();
    }

    return 0;
}

