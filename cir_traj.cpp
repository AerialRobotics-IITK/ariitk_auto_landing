#include <template/template_ros.hpp>

using namespace ariitk::trajectory_gen;

int main(int argc,char** argv){
    ros::init(argc,argv,"cir_traj");
    ros::NodeHandle nh;

    Trajectory cir_traj;

    cir_traj.init(nh);

    ros::Rate loopRate(30);

    while(ros::ok()){
        cir_traj.run();
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}