#include<ros/ros.h>

#include<geometry_msgs/Twist.h>
#include<geometry_msgs/PoseStamped.h>

#include<nav_msgs/Odometry.h>

#include<std_srvs/Trigger.h>

namespace ariitk::auto_landing {
    
class Tracking {
    public:
        Tracking();
        void init(ros::NodeHandle& nh, char** argv);
        void run();

    private:
    
        void huskyPoseCallback(const nav_msgs::Odometry& msg);
        void quadPoseCallback(const nav_msgs::Odometry& msg);

        int height_;
        geometry_msgs::PoseStamped setpt_;
        nav_msgs::Odometry husky_odom_, quad_odom_;
        std_srvs::Trigger landing_service_;

        double husky_relative_x_, husky_relative_y_;

        ros::Publisher set_firefly_pose_;
        ros::Subscriber husky_pose_;
        ros::Subscriber quad_pose_;

        ros::ServiceServer landing_server_;
        ros::ServiceClient landing_client_;
    };
} // namespace ariitk::auto_landing