#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Odometry.h>

namespace ariitk::land_uav{

    class Landing {
        public:
            void init(ros::NodeHandle& nh);
            void run();
        private:
            void position(const geometry_msgs::Pose& msg );
           // void serv();

            geometry_msgs::PoseStamped pnt;

            ros::Subscriber firefly_pose ;
          //  ros::ServiceClient landing;
            ros::Publisher landing_firefly;
    };
}
