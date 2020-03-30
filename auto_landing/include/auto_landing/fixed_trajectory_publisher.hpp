#include <XmlRpcValue.h>
#include <eigen_conversions/eigen_msg.h>

#include <ros/ros.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>

namespace ariitk::auto_landing {

class TrajectoryPublisher {
    public:
        TrajectoryPublisher();
        void run();

    private:
        double getValueAsDouble (XmlRpc::XmlRpcValue& value);
        
        ros::NodeHandle nh_private_, nh_;
        ros::Publisher trajectory_publisher_;

        trajectory_msgs::MultiDOFJointTrajectory trajectory_;
};

}