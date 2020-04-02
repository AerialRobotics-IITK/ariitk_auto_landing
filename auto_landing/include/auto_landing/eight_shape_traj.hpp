#include<ros/ros.h>

#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Twist.h>

#include<nav_msgs/Odometry.h>

namespace ariitk::auto_landing {

class EightTraj {
    
    public:
        
        void init(ros::NodeHandle& nh);
        void run();

    private:

        geometry_msgs::Twist cmdVelocity(const nav_msgs::Odometry& husky_odom_, int& count_, double& t1, double& t2);
        
        void poseCallback(const nav_msgs::Odometry& msg);

        ros::Publisher set_husky_vel_;
        ros::Subscriber husky_pose_;

        int count_=0;

        double t1=ros::Time::now().toSec();
        double t2;

        geometry_msgs::Twist cmd_vel_;
        nav_msgs::Odometry husky_odom_;
};

} //ariitk::auto_landing