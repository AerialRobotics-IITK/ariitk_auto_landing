#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace ariitk::trajectory_gen{

class trajectory{
        public:
           void init(ros::NodeHandle& nh);
           void run();
        
        private:
           ros::Publisher trajectory_pub;
           double ang_vel,linear_vel;
           geometry_msgs::Twist msg;
    };
}//namespace ariitk::namespace_name;