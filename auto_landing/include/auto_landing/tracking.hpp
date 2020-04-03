#include<ros/ros.h>

#include<geometry_msgs/Twist.h>
#include<geometry_msgs/PoseStamped.h>

#include<gazebo_msgs/ModelStates.h>

#include<nav_msgs/Odometry.h>

#include<std_srvs/Trigger.h>

namespace ariitk::auto_landing {
    
class Tracking {
    public:
        Tracking();
        void init(ros::NodeHandle& nh, char** argv);
        void run();

    private:
    
        // void huskyPoseCallback(const nav_msgs::Odometry& msg);
        void modelStateCallback(const gazebo_msgs::ModelStates& msg);
        void quadPoseCallback(const nav_msgs::Odometry& msg);
        void quadSetPointUpdate(nav_msgs::Odometry& husky_odom_, nav_msgs::Odometry& quad_odom_);
        // void huskyVelocityCallback(const geometry_msgs::Twist& msg);
        // void spericalCoordinates(nav_msgs::Odometry& husky_odom_);

        int height_,n;
        geometry_msgs::PoseStamped setpt_;
        nav_msgs::Odometry husky_odom_,quad_odom_;
        std_srvs::Trigger landing_service_;
        geometry_msgs::Twist husky_cmd_vel_;

        double husky_relative_x_, husky_relative_y_;
        double t1=0,t2=0,t_now;
        double rad_,angle_;

        ros::Publisher set_firefly_pose_;
        ros::Subscriber husky_pose_;
        ros::Subscriber quad_pose_;
        // ros::Subscriber husky_velocity_;
        ros::Subscriber gazebo_model_state_;

        ros::ServiceServer landing_server_;
        ros::ServiceClient landing_client_;
    };
} // namespace ariitk::auto_landing