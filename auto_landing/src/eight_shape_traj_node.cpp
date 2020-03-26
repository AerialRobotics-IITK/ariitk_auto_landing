#include<auto_landing/eight_shape_traj.hpp>

using namespace ariitk::auto_landing;

int main(int argc, char **argv){
    ros::init(argc,argv,"eight_shape_traj_node");
    ros::NodeHandle nh;

    Eight_Traj traj;
    
    traj.init(nh);

    ros::Rate loop_rate(10);

    while(ros::ok()) {
        ros::spinOnce();
        traj.run();
        loop_rate.sleep();
    }

    return 0;

}