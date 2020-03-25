#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace ariitk::template{
    class TemplateRos {
        public:
           void init(ros::NodeHandle& nh);
           void run();
           ros::Publisher traj_pub;
    };
}