#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Odometry.h>
#include<std_srvs/Trigger.h>


namespace ariitk::land_uav{

class Landing {
    public:
        void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private,char** argv);
        void run();
    private:
        void position_sub(const geometry_msgs::PoseStamped& msg );
        bool position_land();  

        geometry_msgs::PoseStamped pnt;
        std_srvs::Trigger landing_service_;

        ros::Subscriber firefly_pose ;
        ros::ServiceServer landing_server_;
        ros::Publisher landing_firefly;
    };
} // namespace ariitk::land_uav
