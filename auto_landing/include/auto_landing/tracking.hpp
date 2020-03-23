#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_srvs/Trigger.h>

namespace ariitk::auto_landing{
    
    class tracking {
        public:
            tracking();
            ~tracking(){};
           
           
            void init(ros::NodeHandle& nh);
            void run();
        
        private:
            
            void pose_clbk(const nav_msgs::Odometry& msgs);
            bool serverCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
            
            geometry_msgs::PoseStamped setpt_;
            nav_msgs::Odometry hk_pose_;
            std_srvs::Trigger srv_;

            ros::Publisher set_firefly_pose_;
            ros::Subscriber husky_pose_;
            ros::Subscriber husky_cmd_vel_;

            ros::ServiceServer landing_server_;
            ros::ServiceClient landing_client_;
    };
}