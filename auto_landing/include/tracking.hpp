#include<ros/ros.h>
#include<geometry_msgs/Twist>
#include<nav_msgs/Odometry>
#include<geometry_msgs/PoseStamped>

namespace ariitk::auto_landing{
    
    class tracking {
        public:
            tracking();
            ~tracking(){};
           
           
            void init(ros::NodeHandle& nh);
            void run();
        
        private:
            
            void pose_clbk(const nav_msgs::Odometry& msg);
            
            geometry_msgs::PoseStamped setpt_;
            ros::Publisher set_firefly_pose_;
            ros::Subscriber husky_pose_;
            ros::Subscriber husky_cmd_vel_;
    };
}