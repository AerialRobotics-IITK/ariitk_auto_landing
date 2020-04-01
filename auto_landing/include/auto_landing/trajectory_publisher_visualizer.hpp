#include <XmlRpcValue.h>

#include <ros/ros.h>

#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/conversions.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <gazebo_msgs/ModelStates.h>

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>

namespace ariitk::auto_landing{
class ExampleTrajectoryGeneration {
    public:
        ExampleTrajectoryGeneration();
        ~ExampleTrajectoryGeneration(){};
        void run();

    private:
        int dimension_;
        int derivative_to_optimize_;
        double v_max_;
        double a_max_;
        double distance_;

        XmlRpc::XmlRpcValue points_;
        Eigen::Vector3d husky_acceleration_;

        mav_trajectory_generation::Trajectory result_;

        trajectory_msgs::MultiDOFJointTrajectory generated_trajectory_;
        visualization_msgs::MarkerArray markers_;
        nav_msgs::Odometry mav_odom_, husky_odom_;

        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Publisher trajectory_pub_, marker_pub_;

        ros::Subscriber mav_odometry_sub_, husky_odometry_sub_;

        // double current_time_;

        void generateTrajectory (std::vector<mav_trajectory_generation::Vertex> vertices);
        double getValueAsDouble (XmlRpc::XmlRpcValue& value);
        int setConstVariables (std::string argument);
        mav_trajectory_generation::Vertex::Vector generateLinearTrajectory();

        void mavOdometryCallback(const nav_msgs::Odometry& msg);
        void huskyOdometryCallback(const gazebo_msgs::ModelStates& msg);

};

} //namespace ariitk::auto_landing