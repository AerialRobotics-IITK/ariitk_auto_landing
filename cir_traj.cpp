#include <template/template_ros.hpp>

using namespace ariitk::template;

int main(int argc,char** argv){
    ros::init(argc,argv,"cir_traj");
    ros::NodeHandle nh;

    TemplateRos traj;

    traj.init(nh);

    ros::Rate loopRate(30);

    while(ros::ok()){
        traj.run();
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}