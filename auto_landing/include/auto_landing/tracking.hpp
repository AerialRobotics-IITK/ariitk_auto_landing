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
    
        void modelStateCallback(const gazebo_msgs::ModelStates& msg);
        void quadPoseCallback(const nav_msgs::Odometry& msg);
        void quadSetPointUpdate();

        int height_,index_;
        int appx_value_[2]={3,3};

        geometry_msgs::PoseStamped setpt_;
        nav_msgs::Odometry husky_odom_,quad_odom_;
        std_srvs::Trigger landing_service_;
        geometry_msgs::Twist husky_cmd_vel_[2];

        double husky_relative_x_, husky_relative_y_;
        double time_[2]={0,0},time_diff_;
        double setpt_approximation_[2];
        double norm_;
        
        ros::Publisher set_firefly_pose_;
        ros::Subscriber husky_pose_;
        ros::Subscriber quad_pose_;
        ros::Subscriber gazebo_model_state_;

        ros::ServiceServer landing_server_;
        ros::ServiceClient landing_client_;
    };
} // namespace ariitk::auto_landing