#include <auto_landing/fixed_trajectory_publisher.hpp>
#include <auto_landing/trajectory_publisher_visualizer.hpp>

namespace ariitk::auto_landing {
TrajectoryPublisher::TrajectoryPublisher()
    : nh_private_("~"), nh_() {
    // ros::Time time = ros::Time::now();
    XmlRpc::XmlRpcValue trajectory_param;
    trajectory_publisher_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("trajectory", 1);
    nh_private_.getParam("trajectory", trajectory_param);

    int number_of_points = trajectory_param["points"].size();
    trajectory_.header.frame_id = std::string(trajectory_param["header"]["frame_id"]);
    trajectory_.header.stamp = ros::Time::now();
    trajectory_.joint_names.push_back(trajectory_param["joint_names"]);
    
    for (int i=0; i<number_of_points; i++) {
        trajectory_msgs::MultiDOFJointTrajectoryPoint point;
        geometry_msgs::Transform transform;
        geometry_msgs::Twist velocity, acceleration;

        transform.translation.x = getValueAsDouble(trajectory_param["points"][i]["transforms"]["translation"]["x"]);
        transform.translation.y = getValueAsDouble(trajectory_param["points"][i]["transforms"]["translation"]["y"]);
        transform.translation.z = getValueAsDouble(trajectory_param["points"][i]["transforms"]["translation"]["z"]);

        transform.rotation.x = getValueAsDouble(trajectory_param["points"][i]["transforms"]["rotation"]["x"]);
        transform.rotation.y = getValueAsDouble(trajectory_param["points"][i]["transforms"]["rotation"]["y"]);
        transform.rotation.z = getValueAsDouble(trajectory_param["points"][i]["transforms"]["rotation"]["z"]);
        transform.rotation.w = getValueAsDouble(trajectory_param["points"][i]["transforms"]["rotation"]["w"]);

        velocity.linear.x = getValueAsDouble(trajectory_param["points"][i]["velocities"]["linear"]["x"]);
        velocity.linear.y = getValueAsDouble(trajectory_param["points"][i]["velocities"]["linear"]["y"]);
        velocity.linear.z = getValueAsDouble(trajectory_param["points"][i]["velocities"]["linear"]["z"]);

        velocity.angular.x = getValueAsDouble(trajectory_param["points"][i]["velocities"]["angular"]["x"]);
        velocity.angular.y = getValueAsDouble(trajectory_param["points"][i]["velocities"]["angular"]["y"]);
        velocity.angular.z = getValueAsDouble(trajectory_param["points"][i]["velocities"]["angular"]["z"]);

        acceleration.linear.x = getValueAsDouble(trajectory_param["points"][i]["accelerations"]["linear"]["x"]);
        acceleration.linear.y = getValueAsDouble(trajectory_param["points"][i]["accelerations"]["linear"]["y"]);
        acceleration.linear.z = getValueAsDouble(trajectory_param["points"][i]["accelerations"]["linear"]["z"]);

        acceleration.angular.x = getValueAsDouble(trajectory_param["points"][i]["accelerations"]["angular"]["x"]);
        acceleration.angular.y = getValueAsDouble(trajectory_param["points"][i]["accelerations"]["angular"]["y"]);
        acceleration.angular.z = getValueAsDouble(trajectory_param["points"][i]["accelerations"]["angular"]["z"]);

        point.transforms.push_back(transform);
        point.velocities.push_back(velocity);
        point.accelerations.push_back(acceleration);

        // ros::Duration delta_time = ros::Time::now() - time;
        ros::Duration duration(getValueAsDouble(trajectory_param["points"][i]["time_from_start"]));
        point.time_from_start = duration;

        trajectory_.points.push_back(point);
        Eigen::Affine3d aff = Eigen::Affine3d::Identity();

        geometry_msgs::Pose pose;
        pose.position.x=1;
        pose.position.y=2;
        pose.position.z=3;
        pose.orientation.x=4;
        pose.orientation.y=5;
        pose.orientation.z=6;
        pose.orientation.w=7;

        tf::poseMsgToEigen(pose, aff);
        
        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++) {
                std::cout << aff(i, j) << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
}

void TrajectoryPublisher::run() {
    trajectory_publisher_.publish(trajectory_);
}

double TrajectoryPublisher::getValueAsDouble(XmlRpc::XmlRpcValue& value) {
    if (value.getType() == 2) {
        return (double(int(value)));
    } else if (value.getType() == 3){
        return (double(value));
    } else {
        ROS_ERROR("Parameter specified is not integer or double. Taking 0.0 as the value.");
        return 0.0;
    }
    
}

}